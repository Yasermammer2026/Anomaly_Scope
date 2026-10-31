"""Lossless, bounded-block experiments on verified native sensor files.

Never changes source files. Native counters/timestamps remain exact. Float data
uses integer bit-pattern prediction, never lossy floating-point subtraction.
Host timings are not MCU measurements. Framing overhead is 48 bytes per block.
"""
import argparse
import datetime as dt
import hashlib
import json
from pathlib import Path
import struct
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
import numpy as np
import lz4
import lz4.block

CODECS = ('raw', 'lz4', 'delta_lz4', 'predictor_lz4',
          'delta_zigzag_bitpack', 'delta_zigzag_bitpack_lz4')
FRAME_BYTES = 48
GROUP = 128


def layout(length, offset, schema):
    width, item = schema['scalar_bytes'], schema['item_size']
    if offset % width or length % width or item % width:
        raise ValueError('Unaligned native sample layout')
    pos = np.arange(length//width, dtype=np.int64)*width + offset
    within = pos % item
    payload = (pos//item)*(item-4) + within-4
    sample_span = schema['samples_per_ts']*schema['dim']*width
    group = sample_span+8 if schema['samples_per_ts'] else 0
    relative = payload % group if group else payload
    mask = (within >= 4) & ((relative < sample_span) if group else True)
    lanes = (relative//width) % schema['dim']
    return mask, lanes


def transform(raw, offset, schema, order=1, inverse=False):
    width = schema['scalar_bytes']; dtype = '<u'+str(width)
    x = np.frombuffer(raw, dtype=dtype).copy()
    mask, lanes = layout(len(raw), offset, schema)
    modulus = (1 << (width*8))-1
    for lane in range(schema['dim']):
        select = mask & (lanes == lane)
        values = x[select].astype(np.uint64)
        for _ in range(order):
            if inverse:
                values = np.cumsum(values, dtype=np.uint64) & modulus
            elif len(values):
                values = np.concatenate((values[:1], (values[1:]-values[:-1]) & modulus))
        x[select] = values.astype(dtype)
    return x.tobytes()


def pack(values, word_bits):
    n = len(values); count = (n+GROUP-1)//GROUP
    padded = np.zeros(count*GROUP, dtype=np.uint64)
    padded[:n] = values
    groups = padded.reshape(count, GROUP)
    widths = np.frexp(groups.max(axis=1).astype(np.float64))[1].astype(np.uint8) if count else np.zeros(0, dtype=np.uint8)
    parts = [struct.pack('<I', n), widths.tobytes()]
    for width in range(1, word_bits+1):
        selected = groups[widths == width]
        if selected.size:
            bits = ((selected[..., None] >> np.arange(width, dtype=np.uint64)) & 1).astype(np.uint8)
            parts.append(np.packbits(bits.reshape(-1), bitorder='little').tobytes())
    return b''.join(parts)


def unpack(encoded, word_bits, expected_n):
    if len(encoded)<4: raise ValueError('Short bitpack header')
    n = struct.unpack_from('<I', encoded)[0]
    if n != expected_n: raise ValueError('Sample count mismatch')
    count = (n+GROUP-1)//GROUP
    if len(encoded)<4+count: raise ValueError('Short bit widths')
    widths = np.frombuffer(encoded, np.uint8, count=count, offset=4)
    if np.any(widths > word_bits): raise ValueError('Invalid bit width')
    result = np.zeros((count, GROUP), dtype=np.uint64); cursor = 4+count
    for width in range(1, word_bits+1):
        indices = np.flatnonzero(widths == width)
        size = len(indices)*GROUP*width//8
        if cursor+size > len(encoded): raise ValueError('Short bitpack payload')
        if size:
            bits = np.unpackbits(np.frombuffer(encoded[cursor:cursor+size], np.uint8), bitorder='little')
            bits = bits.reshape(len(indices), GROUP, width).astype(np.uint64)
            result[indices] = (bits << np.arange(width, dtype=np.uint64)).sum(axis=2)
        cursor += size
    if cursor != len(encoded): raise ValueError('Trailing bitpack data')
    return result.reshape(-1)[:n]


def zigzag_pack(raw, offset, schema):
    width = schema['scalar_bytes']; bits = width*8
    mask, _ = layout(len(raw), offset, schema)
    words = np.frombuffer(raw, dtype='<u'+str(width))
    signed = words[mask].view('<i'+str(width)).astype(np.int64)
    zigzag = ((signed << 1) ^ (signed >> (bits-1))).astype(np.uint64)
    return words[~mask].tobytes() + pack(zigzag, bits)


def zigzag_unpack(encoded, length, offset, schema):
    width = schema['scalar_bytes']; dtype = '<u'+str(width)
    mask, _ = layout(length, offset, schema)
    literal_size = int((~mask).sum())*width
    if len(encoded) < literal_size+4: raise ValueError('Short literal metadata')
    words = np.empty(length//width, dtype=dtype)
    words[~mask] = np.frombuffer(encoded[:literal_size], dtype=dtype)
    z = unpack(encoded[literal_size:], width*8, int(mask.sum()))
    signed = (z >> 1).astype(np.int64) ^ -(z & 1).astype(np.int64)
    words[mask] = signed.astype(dtype)
    return words.tobytes()


def encode(raw, offset, schema, codec):
    if codec == 'raw': return 'raw', raw
    value = raw
    if codec.startswith('delta'): value = transform(value, offset, schema)
    elif codec.startswith('predictor'): value = transform(value, offset, schema, order=2)
    if 'zigzag_bitpack' in codec: value = zigzag_pack(value, offset, schema)
    if codec.endswith('lz4') or codec == 'lz4':
        # Store transformed size for bitpack+LZ4; size is bounded and validated by decode.
        value = lz4.block.compress(value, store_size=True)
    return ('raw', raw) if len(value) >= len(raw) else (codec, value)


def decode(encoded, length, offset, schema, codec):
    if codec == 'raw':
        if len(encoded) != length: raise ValueError('Raw length mismatch')
        return encoded
    value = encoded
    if codec.endswith('lz4') or codec == 'lz4':
        if len(value)<4: raise ValueError('Short LZ4 size')
        declared = struct.unpack_from('<I', value)[0]
        # Bitpack worst case: literal bytes + up to one width byte per 128 words + header/padding.
        bound = length + (length//schema['scalar_bytes']+GROUP-1)//GROUP + GROUP*schema['scalar_bytes']+4
        if declared > bound: raise ValueError('Oversized LZ4 output')
        value = lz4.block.decompress(value)
    if 'zigzag_bitpack' in codec: value = zigzag_unpack(value, length, offset, schema)
    if len(value) != length: raise ValueError('Decoded length mismatch')
    if codec.startswith('delta'): value = transform(value, offset, schema, inverse=True)
    elif codec.startswith('predictor'): value = transform(value, offset, schema, order=2, inverse=True)
    return value



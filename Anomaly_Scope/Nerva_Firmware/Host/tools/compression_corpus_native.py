"""Faster host corpus measurement with C transforms and bit packing.

Same six algorithms and exact wire bytes as the NumPy reference. Original
reference results are preserved. MCU firmware uses its own recorded source
identity; these host timings must not be described as MCU performance.
"""
import argparse
import ctypes as C
import datetime as dt
import hashlib
import json
from pathlib import Path
import subprocess
import time
import uuid

import compression_corpus as reference
ROOT=Path(__file__).resolve().parents[1]

class Schema(C.Structure):
    _fields_=[(k,C.c_uint32) for k in ('item_size','dim','samples_per_ts','scalar_bytes')]


class Native:
    def __init__(self):
        dest=ROOT/'artifacts/native-codec';dest.mkdir(parents=True,exist_ok=True)
        source_hash=hashlib.sha256(b''.join((ROOT/'backend'/f).read_bytes() for f in
                                  ['sensor_lossless.c','sensor_lossless.h','sensor_lossless_decode.c'])).hexdigest()[:16]
        # Windows locks a loaded DLL. Every owner gets a unique output, so a
        # second decoder or concurrent process cannot overwrite a loaded image.
        library=dest/('sensor_lossless_full_'+source_hash+'_'+uuid.uuid4().hex+'.dll')
        subprocess.run(['gcc','-O2','-shared','-std=c11','-Wall','-Wextra','-Werror',
                        str(ROOT/'backend/sensor_lossless.c'),str(ROOT/'backend/sensor_lossless_decode.c'),
                        '-o',str(library)],check=True)
        self.lib=C.CDLL(str(library))
        self.lib.sl_encode_packed.argtypes=[C.c_void_p,C.c_size_t,C.c_uint64,C.POINTER(Schema),C.c_void_p,C.c_size_t,
                                           C.c_void_p,C.c_size_t,C.POINTER(C.c_size_t),C.POINTER(C.c_uint32)]
        self.lib.sl_decode.argtypes=[C.c_void_p,C.c_size_t,C.c_size_t,C.c_uint64,C.POINTER(Schema),C.c_uint32,
                                    C.c_void_p,C.c_size_t,C.c_void_p,C.c_size_t]
        self.lib.sl_transform.argtypes=[C.c_void_p,C.c_size_t,C.c_uint64,C.POINTER(Schema),C.c_uint32,C.c_uint32,C.c_void_p,C.c_size_t]
        for f in ('sl_encode_packed','sl_decode','sl_transform'):getattr(self.lib,f).restype=C.c_int
        self.capacity=262144+262144//256+1024
        self.work=C.create_string_buffer(self.capacity);self.output=C.create_string_buffer(self.capacity)

    @staticmethod
    def model(schema):return Schema(**{k:schema[k] for k,_ in Schema._fields_})

    def transform(self,raw,offset,schema,order,inverse=False):
        if len(raw)>self.capacity:raise ValueError('Host transform block too large')
        model=self.model(schema);source=C.create_string_buffer(raw)
        status=self.lib.sl_transform(source,len(raw),offset,C.byref(model),order,int(inverse),self.output,self.capacity)
        if status:raise ValueError('Native transform error '+str(status))
        return C.string_at(self.output,len(raw))

    def encode(self,raw,offset,schema,codec):
        if codec=='raw':return 'raw',raw
        value=raw
        if 'zigzag_bitpack' in codec:
            model=self.model(schema);source=C.create_string_buffer(raw);size,used=C.c_size_t(),C.c_uint32()
            status=self.lib.sl_encode_packed(source,len(raw),offset,C.byref(model),self.work,self.capacity,
                                             self.output,self.capacity,C.byref(size),C.byref(used))
            if status:raise ValueError('Native encode error '+str(status))
            value=C.string_at(self.output,size.value)
        elif codec.startswith('delta'):value=self.transform(raw,offset,schema,1)
        elif codec.startswith('predictor'):value=self.transform(raw,offset,schema,2)
        if codec.endswith('lz4') or codec=='lz4':value=reference.lz4.block.compress(value,store_size=True)
        return ('raw',raw) if len(value)>=len(raw) else (codec,value)

    def decode(self,encoded,length,offset,schema,codec):
        if codec=='raw':
            if len(encoded)!=length:raise ValueError('Raw length mismatch')
            return encoded
        value=encoded
        if codec.endswith('lz4') or codec=='lz4':
            if len(value)<4:raise ValueError('Short LZ4 size')
            declared=int.from_bytes(value[:4],'little')
            bound=length+(length//schema['scalar_bytes']+127)//128+128*schema['scalar_bytes']+4
            if declared>bound:raise ValueError('Oversized LZ4 output')
            value=reference.lz4.block.decompress(value)
        if 'zigzag_bitpack' in codec:
            model=self.model(schema);source=C.create_string_buffer(value)
            status=self.lib.sl_decode(source,len(value),length,offset,C.byref(model),3,self.work,self.capacity,self.output,self.capacity)
            if status:raise ValueError('Native decode error '+str(status))
            return C.string_at(self.output,length)
        if len(value)!=length:raise ValueError('Decoded length mismatch')
        if codec.startswith('delta'):return self.transform(value,offset,schema,1,True)
        if codec.startswith('predictor'):return self.transform(value,offset,schema,2,True)
        return value



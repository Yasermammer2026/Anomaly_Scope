"""Durable, bounded storage of lossless wire frames; no expanded sensor copy.

Uses the existing protocol validator/decoder. A receipt follows an independent
readback, decode and SHA pass over the persisted archive. SQLite checkpoints
cover fsynced archive prefixes; reconnects discard only our uncommitted tail.
"""
import hashlib
import datetime as dt
import json
import os
from pathlib import Path
import sqlite3
import struct
import time
import zlib

from backend.lossless_transfer import (
    ACK, BLOCK, FRAME, ChunkStore, Native, checksummed, manifest_digest, validate_manifest,
)
from backend.native_audit import NativeAudit

RECORD = struct.Struct('<II')


class Sink:
    def __init__(self, model):
        schema = model.get('schema')
        self.audit = NativeAudit(schema['item_size'], schema['scalar_bytes'] * schema['dim'],
                                 schema['samples_per_ts']) if schema else None
        self.metadata = (bytearray() if model['name'] in
                         ('device_config.json','acquisition_info.json') and model['size']<=65536 else None)

    def write(self, raw):
        if self.audit:
            self.audit.feed(raw)
        if self.metadata is not None:
            if len(self.metadata)+len(raw)>65536:raise ValueError('Unbounded acquisition metadata')
            self.metadata.extend(raw)
        return len(raw)


class Validator(ChunkStore):
    def __init__(self, manifest, decoder):
        self.manifest = manifest
        self.identity = manifest_digest(manifest)
        self.acquisition = manifest['acquisition_id']
        self.streams = manifest['files']
        self.decoder = decoder
        self.files = [Sink(m) for m in self.streams]
        self.hashes = [hashlib.sha256() for _ in self.streams]
        self.offsets = [0] * len(self.streams)
        self.complete = [False] * len(self.streams)
        self.bytes_since_checkpoint = 0
        self.pending_checks = []

    def checkpoint(self):
        self.pending_checks.clear()
        self.bytes_since_checkpoint = 0


class ArchiveStore(Validator):
    def __init__(self, directory, manifest, decoder=None, max_bytes=620_000_000):
        validate_manifest(manifest)
        super().__init__(manifest, decoder or Native())
        self.directory = Path(directory).resolve()
        self.directory.mkdir(parents=True, exist_ok=True)
        self.max_bytes = max_bytes
        self.db = sqlite3.connect(self.directory / 'receipt.sqlite3', isolation_level=None)
        self.db.execute('PRAGMA journal_mode=WAL')
        self.db.execute('PRAGMA synchronous=FULL')
        self.db.executescript('''
          CREATE TABLE IF NOT EXISTS identity(hash BLOB PRIMARY KEY, manifest TEXT NOT NULL);
          CREATE TABLE IF NOT EXISTS streams(id INTEGER PRIMARY KEY, offset INTEGER NOT NULL,
            prefix TEXT NOT NULL, complete INTEGER NOT NULL);
          CREATE TABLE IF NOT EXISTS receipt(acquisition TEXT PRIMARY KEY, raw_bytes INTEGER NOT NULL,
            manifest_sha256 BLOB NOT NULL);
          CREATE TABLE IF NOT EXISTS archive_checkpoint(id INTEGER PRIMARY KEY, end_pos INTEGER NOT NULL);
        ''')
        prior = self.db.execute('SELECT hash FROM identity').fetchall()
        if prior and prior != [(self.identity,)]:
            self.db.close()
            raise ValueError('Archive manifest identity differs')
        if not prior:
            self.db.execute('INSERT INTO identity VALUES (?,?)',
                            (self.identity, json.dumps(manifest, sort_keys=True)))
            self.db.execute('INSERT INTO archive_checkpoint VALUES (1,0)')
        self.archive = (self.directory / 'frames.z').open('a+b')
        self.in_accept = False
        self.checkpoint_requested = False
        try:
            end = self.db.execute('SELECT end_pos FROM archive_checkpoint WHERE id=1').fetchone()[0]
            if os.fstat(self.archive.fileno()).st_size < end:
                raise ValueError('Durable archive prefix is truncated')
            restored = Validator(self.manifest, self.decoder)
            self._replay(restored, end)
            self.files, self.hashes = restored.files, restored.hashes
            self.offsets, self.complete = restored.offsets, restored.complete
            states = self.db.execute('SELECT id,offset,prefix,complete FROM streams ORDER BY id').fetchall()
            expected = [(i, self.offsets[i], self.hashes[i].hexdigest(), int(self.complete[i]))
                        for i in range(len(self.streams))]
            if states and states != expected:
                raise ValueError('Archive checkpoint state/hash differs')
            # This is our own container's uncommitted suffix, never a device file.
            self.archive.truncate(end)
            self.archive.seek(end)
        except BaseException:
            self.close()
            raise

    def _replay(self, validator, end):
        self.archive.seek(0)
        while self.archive.tell() < end:
            prefix = self.archive.read(RECORD.size)
            if len(prefix) != RECORD.size:
                raise ValueError('Incomplete archived record')
            size, checksum = RECORD.unpack(prefix)
            if not 0 < size <= BLOCK + FRAME.size + 1024 or self.archive.tell() + size > end:
                raise ValueError('Unbounded/incomplete archived payload')
            compressed = self.archive.read(size)
            if zlib.crc32(compressed) != checksum:
                raise ValueError('Archived record CRC differs')
            decoder = zlib.decompressobj()
            record = decoder.decompress(compressed, BLOCK + FRAME.size + 1)
            if (len(record) > BLOCK + FRAME.size or not decoder.eof or
                    decoder.unused_data or decoder.unconsumed_tail or len(record) < FRAME.size):
                raise ValueError('Invalid bounded archive compression')
            # Explicit base dispatch avoids re-appending records during replay.
            ChunkStore.accept(validator, record[:FRAME.size], record[FRAME.size:])
        if self.archive.tell() != end:
            raise ValueError('Archive prefix boundary differs')

    def accept(self, header, payload):
        self.in_accept = True
        self.checkpoint_requested = False
        try:
            ChunkStore.accept(self, header, payload)
        finally:
            self.in_accept = False
        encoded = zlib.compress(header + payload, level=1)
        if self.archive.tell() + RECORD.size + len(encoded) > self.max_bytes:
            raise OSError('Bounded PC archive allowance exhausted; SD sources remain retained')
        self.archive.write(RECORD.pack(len(encoded), zlib.crc32(encoded)))
        self.archive.write(encoded)
        if self.checkpoint_requested:
            self.checkpoint()

    def checkpoint(self):
        if not hasattr(self, 'archive'):
            return
        if self.in_accept:
            self.checkpoint_requested = True
            return
        self.archive.flush()
        os.fsync(self.archive.fileno())
        self.db.execute('BEGIN IMMEDIATE')
        try:
            self.db.executemany('INSERT OR REPLACE INTO streams VALUES (?,?,?,?)',
                [(i, self.offsets[i], self.hashes[i].hexdigest(), int(self.complete[i]))
                 for i in range(len(self.streams))])
            self.db.execute('UPDATE archive_checkpoint SET end_pos=? WHERE id=1',
                            (self.archive.tell(),))
            self.db.execute('COMMIT')
        except BaseException:
            self.db.execute('ROLLBACK')
            raise
        self.bytes_since_checkpoint = 0
        self.pending_checks.clear()
        now=time.monotonic()
        if now-getattr(self,'last_progress_time',0)>=10:
            self.last_progress_time=now
            total=sum(m['size'] for m in self.streams)
            (self.directory/'progress.json').write_text(json.dumps(dict(
                utc=dt.datetime.now(dt.timezone.utc).isoformat(),stage='receiving',
                verified_prefix_bytes=sum(self.offsets),source_bytes=total,
                percent=100*sum(self.offsets)/total,archive_bytes=self.archive.tell(),
                completed_files=sum(self.complete),total_files=len(self.streams)))+'\n')

    def commit(self):
        if not all(self.complete):
            raise ValueError('Acquisition has incomplete streams')
        self.checkpoint()
        end = self.archive.tell()
        independent = Validator(self.manifest, self.decoder)
        self._replay(independent, end)
        if (independent.offsets != self.offsets or independent.complete != self.complete or
                [x.digest() for x in independent.hashes] != [x.digest() for x in self.hashes]):
            raise ValueError('Persisted archive reconstruction/SHA differs')
        self.archive.seek(end)
        native = {}
        for model, sink in zip(self.streams, independent.files):
            if sink.audit:
                native[model['name']] = sink.audit.finish()
        total = sum(self.offsets)
        proof = dict(acquisition_id=self.acquisition, manifest=self.manifest,
                     manifest_sha256=self.identity.hex(), raw_bytes=total, archive_bytes=end,
                     persisted_readback_SHA_pass=True, SD_sources_deleted=False,
                     native_accounting=native,
                     source_SHA256={m['name']:h.hexdigest() for m,h in zip(self.streams,self.hashes)})
        def sanitized(value):
            if isinstance(value,dict):
                return {k:sanitized(v) for k,v in value.items()
                        if not any(secret in k.lower() for secret in ('wifi','ssid','password','passphrase'))}
            if isinstance(value,list):return [sanitized(v) for v in value]
            return value
        proof['metadata']={m['name']:sanitized(json.loads(sink.metadata.decode('utf-8-sig')))
                           for m,sink in zip(self.streams,independent.files) if sink.metadata is not None}
        proof_path = self.directory / 'verified.json'
        with proof_path.open('w') as stream:
            json.dump(proof, stream, indent=2)
            stream.flush()
            os.fsync(stream.fileno())
        self.db.execute('INSERT OR REPLACE INTO receipt VALUES (?,?,?)',
                        (str(self.acquisition), total, self.identity))
        self.commit_metrics = dict(archive_bytes=end, persisted_readback_SHA_pass=True)
        (self.directory/'progress.json').write_text(json.dumps(dict(
            utc=dt.datetime.now(dt.timezone.utc).isoformat(),stage='committed',
            verified_prefix_bytes=total,source_bytes=total,percent=100,
            archive_bytes=end,completed_files=len(self.streams),total_files=len(self.streams)))+'\n')
        return checksummed(ACK.pack(b'ACK1', 1, self.acquisition, total,
                                     self.identity, len(self.streams), 0))

    def close(self):
        if hasattr(self, 'archive'):
            self.archive.close()
        self.db.close()


def export_archive(directory, destination):
    """Explicit export; caller must reserve space for expanded sensor files."""
    directory = Path(directory).resolve(strict=True)
    destination = Path(destination).resolve()
    if destination == directory or directory in destination.parents:
        raise ValueError('Export to a separate directory')
    proof = json.loads((directory / 'verified.json').read_text())
    manifest = proof['manifest']
    store = ArchiveStore(directory, manifest)
    handles = []
    try:
        destination.mkdir(parents=True, exist_ok=False)
        handles = [(destination / model['name']).open('xb') for model in manifest['files']]
        validator = Validator(manifest, store.decoder)
        class Output:
            def __init__(self, stream): self.stream = stream
            def write(self, raw): return self.stream.write(raw)
        validator.files = [Output(stream) for stream in handles]
        end = store.db.execute('SELECT end_pos FROM archive_checkpoint WHERE id=1').fetchone()[0]
        store._replay(validator, end)
        if not all(validator.complete):
            raise ValueError('Incomplete export')
        for i, stream in enumerate(handles):
            stream.flush();os.fsync(stream.fileno())
            if validator.hashes[i].hexdigest() != proof['source_SHA256'][manifest['files'][i]['name']]:
                raise ValueError('Export hash differs')
    finally:
        for stream in handles: stream.close()
        store.close()

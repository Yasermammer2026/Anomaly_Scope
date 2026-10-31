"""Chunked lossless acquisition staging, durable checkpoints and final receipt.

The caller supplies a trusted native-file manifest. Never touches device SD.
Files stay in owned host storage; SQLite commits identities, offsets and hashes.
Device deletion is permitted only after accepting and persisting COMMITTED.
"""
import hashlib
from contextlib import closing
import json
import os
from pathlib import Path
import re
import socket
import sqlite3
import struct
import sys
import time
import zlib

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from compression_corpus_native import Native

BLOCK=65536
FRAME=struct.Struct('<4sHHQIIQIIII') # 48 bytes; last field CRC of first 44.
HELLO=struct.Struct('<4sQ32sI')
ACK=struct.Struct('<4sIQQ32sII')    # 64 bytes, COMMITTED=1, CRC of first 60.
RESUME=struct.Struct('<4sQIII')    # acquisition, count, completed-stream bitmap, receipt-present flag.
END=65535
CODECS={0:'raw',1:'lz4',2:'delta_lz4',3:'delta_zigzag_bitpack',4:'predictor_lz4',5:'delta_zigzag_bitpack_lz4'}

def checksummed(value):return value[:-4]+struct.pack('<I',zlib.crc32(value[:-4]))

def manifest_digest(manifest):
    return hashlib.sha256(json.dumps(manifest,sort_keys=True,separators=(',',':')).encode()).digest()

def validate_manifest(manifest):
    if set(manifest)!= {'acquisition_id','block_bytes','files'} or manifest['block_bytes']!=BLOCK:
        raise ValueError('Unsupported acquisition manifest')
    if type(manifest['acquisition_id']) is not int or not 0<manifest['acquisition_id']<2**64:
        raise ValueError('Invalid acquisition identity')
    files=manifest['files']
    if not 1<=len(files)<=16:raise ValueError('Invalid stream count')
    names=set()
    for f in files:
        if not re.fullmatch(r'[A-Za-z0-9_.-]{1,96}',f['name']) or f['name'] in ('.','..') or f['name'] in names:
            raise ValueError('Unsafe or duplicated stream name')
        names.add(f['name'])
        if type(f['size']) is not int or not 0<f['size']<2**32:raise ValueError('Invalid stream length')
        if f.get('sha256') is not None and not re.fullmatch('[0-9a-f]{64}',f['sha256']):raise ValueError('Invalid source hash')
        schema=f.get('schema')
        if schema:
            w=schema['scalar_bytes'];dim=schema['dim'];item=schema['item_size'];samples=schema['samples_per_ts']
            if w not in (2,4) or not 1<=dim<=16 or item<8 or item%w or samples<0 or samples>(2**32-5)//dim or f['size']%w:
                raise ValueError('Invalid native sensor schema')

def encode_frame(acquisition,stream,index,offset,raw,encoded,codec):
    return checksummed(FRAME.pack(b'CHN1',1,codec,acquisition,stream,index,offset,len(raw),len(encoded),zlib.crc32(raw),0))+encoded

def end_frame(acquisition,stream,size,digest):
    return checksummed(FRAME.pack(b'CHN1',1,END,acquisition,stream,(size+BLOCK-1)//BLOCK,size,0,32,0,0))+digest


class CompareExistingFile:
    """Accept identical received bytes without overwriting verified backing.

    A read/write handle permits Windows durability flushes. Public mutation
    operations compare or reject; they never write, resize or replace backing.
    """
    def __init__(self,file): self._file=file
    def __getattr__(self,name): return getattr(self._file,name)
    def write(self,data):
        if self._file.read(len(data))!=data:
            raise ValueError('Received bytes differ from verified immutable backing')
        return len(data)
    def truncate(self,*args): raise ValueError('Immutable backing cannot be truncated')
    def writelines(self,*args): raise ValueError('Immutable backing cannot be overwritten')


def verify_immutable_backing(directory,manifest):
    directory=Path(directory).resolve(strict=True)
    dbpath=directory/'receipt.sqlite3'
    with closing(sqlite3.connect(dbpath.as_uri()+'?mode=ro',uri=True)) as db:
        identities=db.execute('SELECT hash,manifest FROM identity').fetchall()
        if len(identities)!=1: raise ValueError('Immutable backing needs one committed identity')
        identity,body=identities[0];old=json.loads(body)
        if old['files']!=manifest['files'] or manifest_digest(old)!=identity:
            raise ValueError('Immutable backing source manifest differs')
        receipt=db.execute('SELECT raw_bytes,manifest_sha256 FROM receipt WHERE acquisition=?',
            (str(old['acquisition_id']),)).fetchone()
        total=sum(f['size'] for f in manifest['files'])
        if receipt!=(total,identity): raise ValueError('Immutable backing has no complete durable receipt')
        states=db.execute('SELECT id,offset,prefix,complete FROM streams ORDER BY id').fetchall()
    if len(states)!=len(manifest['files']): raise ValueError('Immutable backing stream set differs')
    for i,(state,model) in enumerate(zip(states,manifest['files'])):
        if state[0]!=i or state[1]!=model['size'] or state[3]!=1:
            raise ValueError('Immutable backing stream is incomplete')
        path=directory/(model['name']+'.raw')
        if path.stat().st_size!=model['size']: raise ValueError('Immutable backing file size differs')
        with path.open('rb') as file: digest=hashlib.file_digest(file,'sha256').hexdigest()
        if digest!=state[2]: raise ValueError('Immutable backing stored SHA differs')
    return directory


class ChunkStore:
    def __init__(self,directory,manifest,decoder=None,immutable_backing=None):
        validate_manifest(manifest)
        backing=verify_immutable_backing(immutable_backing,manifest) if immutable_backing is not None else None
        self.directory=Path(directory);self.directory.mkdir(parents=True,exist_ok=True)
        if backing==self.directory.resolve(): raise ValueError('Backing and new receipt directory must differ')
        self.immutable_backing=backing
        self.manifest=manifest;self.identity=manifest_digest(manifest);self.acquisition=manifest['acquisition_id']
        self.decoder=decoder or Native();self.streams=manifest['files'];self.files=[];self.hashes=[]
        self.offsets=[];self.complete=[];self.bytes_since_checkpoint=0;self.pending_checks=[]
        self.db=sqlite3.connect(self.directory/'receipt.sqlite3',isolation_level=None)
        self.db.execute('PRAGMA journal_mode=WAL');self.db.execute('PRAGMA synchronous=FULL')
        self.db.executescript('''CREATE TABLE IF NOT EXISTS identity(hash BLOB PRIMARY KEY,manifest TEXT NOT NULL);
          CREATE TABLE IF NOT EXISTS streams(id INTEGER PRIMARY KEY,offset INTEGER NOT NULL,prefix TEXT NOT NULL,complete INTEGER NOT NULL);
          CREATE TABLE IF NOT EXISTS receipt(acquisition TEXT PRIMARY KEY,raw_bytes INTEGER NOT NULL,manifest_sha256 BLOB NOT NULL);
          CREATE TABLE IF NOT EXISTS chunk_checks(stream INTEGER NOT NULL,chunk INTEGER NOT NULL,raw_crc INTEGER NOT NULL,PRIMARY KEY(stream,chunk));''')
        prior=self.db.execute('SELECT hash FROM identity').fetchall()
        if prior and prior != [(self.identity,)]:self.db.close();raise ValueError('Resume manifest differs from durable identity')
        if not prior:
            self.db.execute('INSERT INTO identity VALUES (?,?)',(self.identity,json.dumps(manifest,sort_keys=True)))
        try:
            for i,stream in enumerate(self.streams):
                state=self.db.execute('SELECT offset,prefix,complete FROM streams WHERE id=?',(i,)).fetchone()
                path=(backing or self.directory)/(stream['name']+'.raw')
                if state is None:
                    # A crash may leave our newly created, still empty file
                    # before its initial stream row. Never overwrite data.
                    if backing:
                        file=CompareExistingFile(path.open('r+b'))
                    else:
                        if path.exists() and path.stat().st_size:raise ValueError('Untracked nonempty staging file')
                        file=path.open('r+b' if path.exists() else 'x+b')
                    count=0;digest=hashlib.sha256();finished=False
                    self.db.execute('INSERT INTO streams VALUES (?,0,?,0)',(i,digest.hexdigest()))
                else:
                    count,prefix,finished=state
                    if count<0 or count>stream['size'] or (finished and count!=stream['size']) or not path.exists() or path.stat().st_size<count:
                        raise ValueError('Durable staging prefix missing')
                    file=path.open('r+b')
                    if backing: file=CompareExistingFile(file)
                    digest=hashlib.sha256();remaining=count
                    while remaining:
                        data=file.read(min(BLOCK,remaining));digest.update(data);remaining-=len(data)
                    if digest.hexdigest()!=prefix: file.close();raise ValueError('Durable staging prefix corrupted')
                    if not backing: file.truncate(count) # Only uncommitted bytes of our own host file.
                    file.seek(count)
                self.files.append(file);self.offsets.append(count);self.hashes.append(digest);self.complete.append(bool(finished))
        except BaseException:self.close();raise

    def checkpoint(self):
        for file in self.files:file.flush();os.fsync(file.fileno())
        self.db.execute('BEGIN IMMEDIATE')
        try:
            for i in range(len(self.files)):
                self.db.execute('UPDATE streams SET offset=?,prefix=?,complete=? WHERE id=?',
                                (self.offsets[i],self.hashes[i].hexdigest(),int(self.complete[i]),i))
            self.db.executemany('INSERT OR REPLACE INTO chunk_checks VALUES (?,?,?)',self.pending_checks)
            self.db.execute('COMMIT')
        except BaseException:self.db.execute('ROLLBACK');raise
        self.bytes_since_checkpoint=0;self.pending_checks=[]

    def accept(self,header,payload):
        if len(header)!=FRAME.size:raise ValueError('Wrong chunk header size')
        magic,version,codec,acquisition,stream,index,offset,raw_length,encoded_length,crc,header_crc=FRAME.unpack(header)
        if magic!=b'CHN1' or version!=1 or acquisition!=self.acquisition or header_crc!=zlib.crc32(header[:-4]):
            raise ValueError('Chunk header identity/CRC differs')
        if stream>=len(self.files) or encoded_length!=len(payload):raise ValueError('Invalid chunk stream/payload length')
        model=self.streams[stream];size=model['size']
        if codec==END:
            if (offset,raw_length,encoded_length,index,crc)!=(size,0,32,(size+BLOCK-1)//BLOCK,0) or self.offsets[stream]!=size:
                raise ValueError('Premature/malformed stream digest')
            digest=self.hashes[stream].digest()
            if payload!=digest or (model.get('sha256') is not None and payload.hex()!=model['sha256']):
                raise ValueError('Stream SHA differs from device/source')
            self.complete[stream]=True;self.checkpoint();return
        if codec not in CODECS or self.complete[stream] or offset!=self.offsets[stream] or offset%BLOCK or index!=offset//BLOCK:
            raise ValueError('Chunk order or codec differs')
        if raw_length!=min(BLOCK,size-offset) or not 0<raw_length<=BLOCK or encoded_length>raw_length or not encoded_length:
            raise ValueError('Invalid bounded chunk length')
        if codec and not model.get('schema'):raise ValueError('Metadata must be RAW')
        raw=self.decoder.decode(payload,raw_length,offset,model.get('schema'),CODECS[codec])
        if zlib.crc32(raw)!=crc:raise ValueError('Decoded chunk CRC differs')
        self.files[stream].write(raw);self.hashes[stream].update(raw);self.offsets[stream]+=raw_length
        self.pending_checks.append((stream,index,crc))
        self.bytes_since_checkpoint+=raw_length
        if self.bytes_since_checkpoint>=1024*1024:self.checkpoint()

    def commit(self):
        if not all(self.complete):raise ValueError('Acquisition has incomplete streams')
        checkpoint_started=time.perf_counter()
        self.checkpoint()
        checkpoint_seconds=time.perf_counter()-checkpoint_started
        verify_seconds=flush_seconds=0.0
        # Verify stored bytes again, independently of incremental receive state.
        for i,model in enumerate(self.streams):
            started=time.perf_counter()
            file=self.files[i];file.seek(0);digest=hashlib.file_digest(file,'sha256')
            if os.fstat(file.fileno()).st_size!=model['size'] or digest.digest()!=self.hashes[i].digest():raise ValueError('Final stored file size/hash differs')
            verify_seconds+=time.perf_counter()-started
            started=time.perf_counter()
            file.flush();os.fsync(file.fileno())
            flush_seconds+=time.perf_counter()-started
        total=sum(f['size'] for f in self.streams)
        started=time.perf_counter()
        self.db.execute('INSERT OR REPLACE INTO receipt VALUES (?,?,?)',(str(self.acquisition),total,self.identity))
        self.commit_metrics=dict(final_checkpoint_seconds=checkpoint_seconds,stored_SHA_verify_seconds=verify_seconds,
            final_file_flush_seconds=flush_seconds,receipt_SQLite_commit_seconds=time.perf_counter()-started)
        return checksummed(ACK.pack(b'ACK1',1,self.acquisition,total,self.identity,len(self.streams),0))

    def close(self):
        for file in self.files:file.close()
        self.files=[]
        self.db.close()


def receive_exact(connection,count):
    data=bytearray(count);view=memoryview(data);offset=0
    while offset<count:
        n=connection.recv_into(view[offset:])
        if not n:raise EOFError('Incomplete acquisition frame')
        offset+=n
    return bytes(data)

def serve_connection(connection,store,interrupt_after=None):
    start=time.perf_counter();connection.settimeout(90)
    hello=receive_exact(connection,HELLO.size)
    magic,acquisition,identity,crc=HELLO.unpack(hello)
    if magic!=b'LST1' or acquisition!=store.acquisition or identity!=store.identity or crc!=zlib.crc32(hello[:-4]):
        raise ValueError('Acquisition hello differs from trusted manifest')
    completed=sum(int(flag)<<i for i,flag in enumerate(store.complete))
    receipt=store.db.execute('SELECT 1 FROM receipt WHERE acquisition=?',(str(store.acquisition),)).fetchone()
    prefix=RESUME.pack(b'RSM1',store.acquisition,len(store.offsets),completed,int(bool(receipt)))+struct.pack('<'+'Q'*len(store.offsets),*store.offsets)
    connection.sendall(prefix+struct.pack('<I',zlib.crc32(prefix)))
    wire=0
    while not all(store.complete):
        header=receive_exact(connection,FRAME.size);fields=FRAME.unpack(header);encoded=fields[8]
        if encoded>BLOCK:raise ValueError('Unbounded network payload')
        payload=receive_exact(connection,encoded);store.accept(header,payload);wire+=len(header)+len(payload)
        if interrupt_after is not None and sum(store.offsets)>=interrupt_after:
            store.checkpoint();raise ConnectionAbortedError('Intentional transfer interruption; no COMMITTED')
    received=time.perf_counter();ack=store.commit();committed=time.perf_counter()
    connection.sendall(ack)
    if receive_exact(connection,4)!=b'DCON':raise ValueError('Device did not confirm persisted receipt')
    return dict(total_seconds=time.perf_counter()-start,receive_seconds=received-start,
                verify_commit_seconds=committed-received,wire_bytes=wire,raw_bytes=sum(store.offsets),
                commit_metrics=getattr(store,'commit_metrics',{}),
                server_committed=True,device_confirmed=True,device_SD_deleted=False)

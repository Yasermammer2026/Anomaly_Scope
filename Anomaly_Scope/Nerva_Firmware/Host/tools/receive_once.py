"""Receive one approved one-hour test connection into a verified archive."""
import argparse
import datetime as dt
import json
from pathlib import Path
import socket
import struct
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from backend.cycle_archive import ArchiveStore
from backend.lossless_transfer import Native, receive_exact, serve_connection, validate_manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--listen', default='192.168.8.4')
    parser.add_argument('--port', type=int, default=19195)
    parser.add_argument('--storage', type=Path, required=True)
    parser.add_argument('--max-storage-bytes', type=int, default=700_000_000)
    parser.add_argument('--storage-approval', type=Path)
    args = parser.parse_args()
    if args.max_storage_bytes <= 0:
        parser.error('Positive storage cap required')
    if args.max_storage_bytes > 700_000_000:
        if not args.storage_approval:
            parser.error('Explicit approved storage record required above existing cap')
        approved = json.loads(args.storage_approval.read_text())
        if not approved.get('approved_for_onehour_storage') or args.max_storage_bytes > approved['receiver_cap_bytes']:
            parser.error('Requested cap exceeds the recorded approval')
    storage = args.storage.resolve()
    if not storage.is_relative_to(ROOT/'artifacts'):
        parser.error('Store only inside owned project artifacts')
    if storage.exists():
        parser.error('Use a new storage directory; preserve all existing archives')
    storage.mkdir(parents=True, exist_ok=False)
    decoder = Native()
    event_path = storage.parent/'receiver-events.jsonl'
    def event(**value):
        value['utc'] = dt.datetime.now(dt.timezone.utc).isoformat()
        line = json.dumps(value)
        print(line, flush=True)
        with event_path.open('a') as stream: stream.write(line+'\n')
    with socket.socket() as listener:
        listener.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        listener.bind((args.listen,args.port));listener.listen(1)
        listener.settimeout(4200)  # Bounded wait for this one scheduled recording.
        event(stage='listening',address=args.listen,port=args.port,storage=str(storage))
        for attempt in range(1):  # Exactly one connection; never accept a retry.
            connection,peer = listener.accept()
            store = None
            started = time.monotonic()
            try:
                with connection:
                    connection.settimeout(90)
                    magic,length = struct.unpack('<4sI',receive_exact(connection,8))
                    if magic!=b'MAN1' or not 0<length<=8192:
                        raise ValueError('Invalid bounded manifest preface')
                    manifest=json.loads(receive_exact(connection,length))
                    validate_manifest(manifest)
                    identity=manifest['acquisition_id']
                    directory=storage/str(identity)
                    other_bytes=sum(p.stat().st_size for p in storage.rglob('*')
                                    if p.is_file() and not p.is_relative_to(directory))
                    allowance=args.max_storage_bytes-other_bytes-4*1024**2
                    if allowance<=0:raise OSError('PC archive allowance exhausted')
                    store=ArchiveStore(directory,manifest,decoder,max_bytes=allowance)
                    event(stage='connected',peer=peer[0],acquisition=identity,
                          files=len(manifest['files']),raw_bytes=sum(m['size'] for m in manifest['files']),
                          resumed_bytes=sum(store.offsets))
                    result=serve_connection(connection,store)
                    if not result.get('device_confirmed'):
                        raise RuntimeError('Durable receipt not confirmed by device')
                    (directory/'delivery.json').write_text(json.dumps(dict(result,
                        utc=dt.datetime.now(dt.timezone.utc).isoformat()),indent=2)+'\n')
                    event(stage='delivered',acquisition=identity,elapsed_seconds=time.monotonic()-started,**result)
            except (OSError,ValueError,EOFError,RuntimeError) as error:
                event(stage='connection_failed',error=type(error).__name__,message=str(error),
                      elapsed_seconds=time.monotonic()-started)
                raise
            finally:
                if store:store.close()


if __name__=='__main__': main()

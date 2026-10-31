#!/usr/bin/env python3
"""Summarize ST HSD 2.0 USB recordings using stdlib only."""
import json, struct, sys
from pathlib import Path
from datetime import datetime
root=Path(sys.argv[1] if len(sys.argv)>1 else 'Samples_of_data')
config=json.loads((root/'device_config.json').read_text())
info=json.loads((root/'acquisition_info.json').read_text())
duration=(datetime.fromisoformat(info['end_time'].replace('Z','+00:00'))-datetime.fromisoformat(info['start_time'].replace('Z','+00:00'))).total_seconds()
assert info['interface']==1, 'Only USB recordings supported'
rows=[]
for comp in config['devices'][0]['components']:
 for name,c in comp.items():
  if c.get('c_type')!=0: continue
  p=root/(name+'.dat')
  if not p.exists():
   rows.append((name,c['dim'],0,0,None,'disabled' if not c['enable'] else 'missing file'))
   continue
  dps=c['usb_dps']; packet=dps+4
  width={'int16':2,'float_t':4,'int8':1}[c['data_type']]*c['dim']
  spt=c['samples_per_ts']; frame=spt*width+8
  previous=None; gaps=0; frames=0; payload_bytes=0
  pending=bytearray(); first_ts=None; last_ts=None
  with p.open('rb') as stream:
   while True:
    block=stream.read(packet*4096)
    if not block: break
    payload=bytearray()
    for start in range(0,len(block),packet):
     chunk=block[start:start+packet]
     if len(chunk)<4: break
     counter=struct.unpack_from('<I',chunk)[0]
     if previous is not None and (counter-previous)%(2**32)!=dps: gaps+=1
     previous=counter; payload.extend(chunk[4:])
    payload_bytes+=len(payload); pending.extend(payload)
    complete=len(pending)//frame
    for i in range(complete):
     timestamp=struct.unpack_from('<d',pending,i*frame+spt*width)[0]
     if first_ts is None: first_ts=timestamp
     last_ts=timestamp
    frames+=complete
    del pending[:complete*frame]
  samples=frames*spt+min(spt,len(pending)//width)
  rate=((frames-1)*spt/(last_ts-first_ts)) if frames>1 and last_ts>first_ts else None
  rows.append((name,c['dim'],samples,samples/duration,rate,f'{gaps} packet counter gaps'))
report=f'Recording duration: {duration:.3f} seconds.\n\n'
report+='| Sensor stream | Channels | Stored samples | Samples/sec over recording | Samples/sec between frame timestamps | Integrity |\n|---|---:|---:|---:|---:|---|\n'
for name,dim,n,avg,rate,status in rows:
 report+=f'| {name} | {dim} | {n:,} | {avg:,.2f} | {rate:,.2f} | {status} |\n' if rate is not None else f'| {name} | {dim} | {n:,} | {avg:,.2f} | — | {status} |\n'
report+='\nA sample is one simultaneous set of channel values. USB four-byte counters and eight-byte frame timestamps are excluded from sample counts. Timestamp rates exclude partial final frames. Whole-recording averages include startup/shutdown time.\n'
(root/'sampling_report.md').write_text(report)
print(report)

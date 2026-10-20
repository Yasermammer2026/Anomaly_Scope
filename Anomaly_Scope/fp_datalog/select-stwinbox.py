from pathlib import Path
import xml.etree.ElementTree as ET,re,subprocess,posixpath,json
root=Path(__file__).resolve().parent; project='Projects/STM32U585AI-STWIN.box/Applications/DATALOG2'; ide=root/project/'STM32CubeIDE'
# Fix an upstream CubeIDE link: the shared header lives in eLooM.
metadata=ide/'.project'
text=metadata.read_text()
text=text.replace('PARENT-4-PROJECT_LOC/eLooM_Components/SensorManager/Inc/drivers/HWDriverMap.h','PARENT-5-PROJECT_LOC/Middlewares/ST/eLooM/Inc/drivers/HWDriverMap.h')
metadata.write_text(text)
tracked=subprocess.check_output(['git','ls-tree','-r','--name-only','HEAD'],text=True).splitlines(); trackedset=set(tracked); casepaths={x.lower():x for x in tracked}
selected={x for x in tracked if '/' not in x or (x.startswith(project+'/') and not x.startswith((project+'/EWARM/',project+'/MDK-ARM/')))}
links=[]; stale=[]
for n in ET.parse(ide/'.project').findall('.//locationURI'):
 m=re.fullmatch(r'PARENT-(\d+)-PROJECT_LOC/(.*)',n.text)
 if not m: raise RuntimeError(n.text)
 base=ide
 for _ in range(int(m[1])): base=base.parent
 rel=(base/m[2]).resolve().relative_to(root).as_posix()
 rel=casepaths.get(rel.lower(),rel)
 if rel not in trackedset:
  stale.append(rel); continue
 links.append(rel); selected.add(rel)
# CubeIDE runs the compiler from Debug/Release, one level below its metadata.
for n in ET.parse(ide/'.cproject').findall('.//listOptionValue'):
 v=n.get('value','')
 if not v.startswith('../'): continue
 rel=posixpath.normpath(project+'/STM32CubeIDE/Debug/'+v)
 rel=casepaths.get(rel.lower(),rel)
 if rel in trackedset: selected.add(rel)
 else: selected.update(x for x in tracked if x.lower().startswith(rel.lower()+'/'))
# Keep license/copyright notices for every included dependency.
parents={str(p) for f in selected for p in Path(f).parents}
for f in tracked:
 name=Path(f).name.lower()
 if str(Path(f).parent) in parents and any(k in name for k in ('license','copying','copyright','notice')): selected.add(f)
patterns=''.join('/'+x+'\n' for x in sorted(selected))
subprocess.run(['git','sparse-checkout','set','--no-cone','--stdin'],input=patterns,text=True,check=True)
missing=[x for x in links if not (root/x).is_file()]
if missing: raise RuntimeError(missing)
info={'upstream':subprocess.check_output(['git','remote','get-url','origin'],text=True).strip(),'commit':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),'board':'STEVAL-STWINBX1 (STWIN.box), STM32U585AI','application':project,'upstream_missing_links':stale,'linked_resources_checked':len(links),'selected_files':len(selected),'full_repository_files':len(tracked),'selected_bytes':sum((root/x).stat().st_size for x in selected)}
(root/'STWINBOX-MANIFEST.json').write_text(json.dumps(info,indent=2)+'\n')
print(json.dumps(info,indent=2))

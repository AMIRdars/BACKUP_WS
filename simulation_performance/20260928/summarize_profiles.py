#!/usr/bin/python3
from pathlib import Path
import json,struct,collections,subprocess,pstats,csv
BASE=Path(__file__).resolve().parent; native=BASE/'diagnostic/native'; mappings=[]
for line in (native/'maps.txt').read_text().splitlines():
 fields=line.split(None,5)
 if len(fields)<6:continue
 lo,hi=[int(s,16) for s in fields[0].split('-')];off=int(fields[2],16);mappings.append((lo,hi,off,fields[5]))
data=(native/'main_cpu_pc.bin').read_bytes();ips=struct.unpack('<'+'Q'*(len(data)//8),data);counts=collections.Counter(ips);libs=collections.Counter();names=collections.Counter();resolved=[]
grouped=collections.defaultdict(list)
for ip,n in counts.items():
 m=next((m for m in mappings if m[0]<=ip<m[1]),None)
 if m:
  lo,hi,off,path=m;libs[path]+=n;grouped[path].append((ip-lo+off,n))
 else:libs['[unmapped]']+=n
for path,items in grouped.items():
 if not path.startswith('/') or not Path(path).exists():continue
 symbols=[]
 for line in subprocess.check_output(['nm','-D','-S','-C','--defined-only',path],text=True,stderr=subprocess.DEVNULL).splitlines():
  fields=line.split(None,3)
  if len(fields)==4 and fields[2] in ('T','t','W','w','i','I'):
   try: symbols.append((int(fields[0],16),int(fields[1],16),fields[3]))
   except ValueError:pass
 symbols.sort()
 for addr,n in items:
  matches=[sym for sym in symbols if sym[0]<=addr<sym[0]+sym[1]]
  function=matches[0][2] if matches else '[not covered by exported function symbol]'
  names[(Path(path).name,function)]+=n
result={'n_samples':len(ips),'sampling_cpu_period_ms':10,'libraries':[{'library':k,'samples':v,'percent':100*v/len(ips)} for k,v in libs.most_common()],'functions':[{'library':k[0],'function':k[1],'samples':v,'percent':100*v/len(ips)} for k,v in names.most_common()]}
(native/'pc_summary.json').write_text(json.dumps(result,indent=2))
print('Native CPU samples',len(ips))
for d in result['libraries'][:8]:print(d['samples'],round(d['percent'],1),d['library'])
for d in result['functions'][:15]:print(d['samples'],round(d['percent'],1),d['function'])
python={}
for file in (BASE/'diagnostic/python').glob('*.pstats'):
 s=pstats.Stats(str(file));total=s.total_tt;entries=[]
 for (path,line,name),(cc,nc,tt,ct,callers) in s.stats.items():
  entries.append({'file':path,'line':line,'function':name,'calls':nc,'self_cpu_s':tt,'inclusive_cpu_s':ct,'inclusive_percent':100*ct/total if total else 0})
 python[file.stem]={'main_cpu_s':total,'top_self':sorted(entries,key=lambda x:x['self_cpu_s'],reverse=True)[:20], 'selected':[e for e in entries if e['function'] in ['_take_subscription','_wait_for_ready_callbacks','_control_tick','_control_loop','_tick','_monitor_tick','_on_payload_pose','_on_contact','_on_finger_wrench','_on_clock_message'] or 'convert_to_py' in e['function']]}
(BASE/'diagnostic/python/summary.json').write_text(json.dumps(python,indent=2))
print('Python key costs')
for k,v in python.items():
 if k.startswith(('coordinator-','slip_monitor-','friction_grasp_manager-','rotation_controller-')):
  print(k,v['main_cpu_s'])
  for e in v['selected']:print(e['function'],round(e['inclusive_percent'],1),e['calls'])

#!/usr/bin/python3
from pathlib import Path
import json,csv,collections,statistics,sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'20260928'))
import importlib.util
# Import the label helper without rewriting the historical baseline summaries.
source=(Path(__file__).resolve().parents[1]/'20260928/analyze.py').read_text()
exec(source[source.index('def label('):source.index('def summarize(')],globals())
base=Path(__file__).resolve().parent
result=[]
for path in sorted(base.iterdir()):
 if not path.is_dir() or not (path/'samples.jsonl').exists():continue
 rows=[json.loads(s) for s in (path/'samples.jsonl').read_text().splitlines()];valid=[r for r in rows if r['sim'] is not None];last=valid[-1]
 quality=json.loads((path/'quality.json').read_text()) if (path/'quality.json').exists() else last.get('quality',{})
 for phase,selected in [('sim_65_80',[r for r in valid if 65<=r['sim']<80]),('supported_grasp',[r for r in valid if r.get('quality',{}).get('ever_grasp') and not r.get('quality',{}).get('support_removed')]),('transport',[r for r in valid if r.get('quality',{}).get('support_removed')])]:
  if len(selected)<2:continue
  a,b=selected[0],selected[-1];cpus=collections.defaultdict(list)
  for r in selected:
   for p in r['processes']:
    if p['cpu'] is not None:cpus[label(p)].append(p['cpu'])
  update=None
  if (path/'update_pass.csv').exists():
   u=[x for x in csv.DictReader(open(path/'update_pass.csv')) if a['sim']<=float(x['sim'])<=b['sim']]
   if u:update=1000*sum(float(x['cpu']) for x in u)/sum(int(x['count']) for x in u)
  result.append({'condition':path.name,'phase':phase,'rtf':(b['sim']-a['sim'])/(b['wall']-a['wall']),'update_ms':update,'cpu':{k:statistics.mean(v) for k,v in cpus.items()},'quality':quality,'last_wall':last['wall'],'last_sim':last['sim']})
(base/'results.json').write_text(json.dumps(result,indent=2))
with (base/'comparison.csv').open('w') as f:
 columns=['condition','phase','rtf','update_ms','gazebo_server_cpu','gazebo_gui_cpu','coordinator_cpu','slip_monitor_cpu','friction_grasp_manager_cpu','grasp_success','support_removed','rotation_complete','max_slip','vertical_slip','min_payload_z','notes'];w=csv.DictWriter(f,fieldnames=columns);w.writeheader()
 for r in result:
  q=r['quality'];cpu=r['cpu'];w.writerow({'condition':r['condition'],'phase':r['phase'],'rtf':r['rtf'],'update_ms':r['update_ms'],'gazebo_server_cpu':cpu.get('ign gazebo server'),'gazebo_gui_cpu':cpu.get('ign gazebo gui'),'coordinator_cpu':cpu.get('coordinator'),'slip_monitor_cpu':cpu.get('slip_monitor'),'friction_grasp_manager_cpu':cpu.get('friction_grasp_manager'),'grasp_success':q.get('ever_grasp'),'support_removed':q.get('support_removed'),'rotation_complete':q.get('rotation_complete'),'max_slip':q.get('max_slip_horizontal'),'vertical_slip':q.get('max_slip_vertical'),'min_payload_z':q.get('min_payload_z_after_support'),'notes':str(q.get('statuses'))})
for r in result:print(r['condition'],r['phase'],'RTF',round(r['rtf'],3),'update_ms',r['update_ms'],'grasp',r['quality'].get('ever_grasp'),'support',r['quality'].get('support_removed'),'slip',r['quality'].get('max_slip_horizontal'))

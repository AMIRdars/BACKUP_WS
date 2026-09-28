"""Compare the same behavioral phases; short and full averages are separate."""
from pathlib import Path
import collections, csv, json, statistics
base=Path(__file__).resolve().parent
rows=[]
for path in sorted(base.iterdir()):
 if not path.is_dir() or not (path/'quality.json').exists():continue
 quality=json.loads((path/'quality.json').read_text())
 if not quality.get('pivot_complete'):continue
 samples=[json.loads(s) for s in (path/'samples.jsonl').read_text().splitlines()]
 phases=collections.defaultdict(list)
 for s in samples:
  q=s.get('quality',{})
  if not q.get('support_removed'):continue
  status=q.get('statuses',{}).get('rotation','').split(':')[0]
  phase='pivot' if q.get('pivot_started') else status
  phases[phase].append(s)
 for phase,s in phases.items():
  if len(s)<2:continue
  a,b=s[0],s[-1];rtf=(b['sim']-a['sim'])/(b['wall']-a['wall'])
  costs=collections.defaultdict(list)
  total=[]
  for sample in s:
   total.append(sum(p['cpu'] for p in sample['processes'] if p['cpu'] is not None))
   for p in sample['processes']:
    args=' '.join(p.get('args',[]))
    if p['cpu'] is None:continue
    if 'finger_contact_bridge' in args:costs['bridge'].append(p['cpu'])
    if 'finger_contact_aggregator' in args:costs['agg'].append(p['cpu'])
  rows.append({'condition':path.name,'phase':phase,'wall_seconds':b['wall']-a['wall'],'sim_seconds':b['sim']-a['sim'],'rtf':rtf,'total_cpu_percent':statistics.mean(total),'cpu_seconds_per_sim_second':statistics.mean(total)/100/rtf if rtf else None})
with (base/'phase_comparison.csv').open('w') as f:
 w=csv.DictWriter(f,fieldnames=['condition','phase','wall_seconds','sim_seconds','rtf','total_cpu_percent','cpu_seconds_per_sim_second'],lineterminator='\n');w.writeheader();w.writerows(rows)
for r in rows:print(r['condition'],r['phase'],round(r['rtf'],3),round(r['cpu_seconds_per_sim_second'],3))

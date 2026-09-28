#!/usr/bin/python3
import json,statistics,collections
from pathlib import Path
BASE=Path(__file__).resolve().parent

def label(p):
    args=p['args']
    if not args: return p['name']
    if args[0].startswith('ign gazebo'):return args[0].strip()
    if len(args)>1 and ('python' in Path(args[0]).name):name=Path(args[1]).name
    else:name=Path(args[0]).name
    for i,a in enumerate(args):
        if a.startswith('__node:='):name=a.split(':=')[1]
        if a.startswith('__ns:='):name=a.split(':=')[1]+'/'+name
    if name=='parameter_bridge':name='world_clock_pose_bridge'
    return name

def summarize(rows):
    if len(rows)<2:return {}
    d=collections.defaultdict(list); threads=collections.defaultdict(list); totals=[]
    for r in rows:
        totals.append(sum(p['cpu'] or 0 for p in r['processes']))
        for p in r['processes']:
            name=label(p)
            if p['cpu'] is not None:d[name].append(p['cpu'])
            for th in p['threads']:threads[(name,th['tid'],th['name'])].append(th['cpu'])
    a,b=rows[0],rows[-1]
    return {'n':len(rows),'wall_start':a['wall'],'wall_end':b['wall'],'sim_start':a['sim'],'sim_end':b['sim'],'rtf':(b['sim']-a['sim'])/(b['wall']-a['wall']), 'total_cpu':statistics.mean(totals),'system_cpu':statistics.mean(r['system_cpu'] for r in rows),'available_gib_min':min(r['memory_available'] for r in rows)/2**30, 'process_cpu':{k:statistics.mean(v) for k,v in sorted(d.items(),key=lambda kv:statistics.mean(kv[1]),reverse=True)},'threads_top':[(str(k),statistics.mean(v)) for k,v in sorted(threads.items(),key=lambda kv:sum(kv[1])/len(rows),reverse=True)[:12]]}
result={}
for run in ['gui','headless','diagnostic']:
    file=BASE/run/'samples.jsonl'
    if not file.exists():continue
    rows=[json.loads(s) for s in file.read_text().splitlines()]; valid=[r for r in rows if r['sim'] is not None]
    result[run]={}
    for name,l,h in [('wall_30_60',30,60),('wall_90_150',90,150),('wall_150_180',150,181)]:result[run][name]=summarize([r for r in valid if l<=r['wall']<h])
    for name,l,h in [('sim_20_40',20,40),('sim_40_55',40,55),('sim_65_80',65,80),('sim_80_90',80,90)]:result[run][name]=summarize([r for r in valid if l<=r['sim']<h])
    result[run]['last']={k:valid[-1][k] for k in ['wall','sim']}
(BASE/'summary.json').write_text(json.dumps(result,indent=2))
for run,regions in result.items():
 print(run)
 for name,summary in regions.items():
  if 'rtf' in summary:print(name,'RTF',round(summary['rtf'],3),'CPU',round(summary['total_cpu'],1),'system',round(summary['system_cpu'],1))
 print('latest',regions['last'])

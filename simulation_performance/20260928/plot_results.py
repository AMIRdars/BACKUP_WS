#!/usr/bin/python3
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
BASE=Path(__file__).resolve().parent
runs={k:[json.loads(s) for s in (BASE/k/'samples.jsonl').read_text().splitlines()] for k in ['gui','diagnostic']}
fig,axes=plt.subplots(3,1,figsize=(10,11),constrained_layout=True)
for name,rows in runs.items():
 rows=[r for r in rows if r['sim'] is not None]
 axes[0].plot([r['wall'] for r in rows],[r['sim'] for r in rows],label=name)
 times=[];rtfs=[]
 for i in range(10,len(rows)):
  a,b=rows[i-10],rows[i];times.append(b['wall']);rtfs.append((b['sim']-a['sim'])/(b['wall']-a['wall']))
 axes[1].plot(times,rtfs,label=name)
axes[0].plot([0,210],[0,210],'--',color='gray',label='real time (RTF = 1)')
axes[0].set(xlabel='Wall time since launch [s]',ylabel='Simulation time [s]',title='Requested GUI launch: simulation falls behind wall time')
axes[1].axhline(1,ls='--',color='gray');axes[1].axvspan(160,190,alpha=.14,color='green',label='GUI suspended (diagnostic only)')
axes[1].set(xlabel='Wall time since launch [s]',ylabel='Real-time factor (10 s window)',ylim=(0,1.12),title='GUI suspension helps, but the simulation remains below real time')
s=json.loads((BASE/'summary.json').read_text())['gui']['sim_65_80']['process_cpu']; pairs=list(s.items())[:10]
axes[2].barh([k for k,v in reversed(pairs)],[v/100 for k,v in reversed(pairs)],color='#34699a')
axes[2].set(xlabel='CPU cores used (100% process CPU = 1 core)',title='Baseline GUI run: mean process CPU at simulation time 65-80 s')
for ax in axes[:2]:ax.legend();ax.grid(alpha=.2)
axes[2].grid(axis='x',alpha=.2)
fig.savefig(BASE/'performance.png',dpi=160)

"""Create a standalone chart of the matched 180 s contact experiment."""
from pathlib import Path
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
base=Path(__file__).resolve().parent
r=json.loads((base/'results.json').read_text())
rows=[next(x for x in r if x['condition']==n and x['phase']=='transport') for n in ['M_ros_contact_baseline','N_native_contact_candidate']]
fig,axes=plt.subplots(1,2,figsize=(9,3.5),layout='constrained')
labels=['ROS Contacts bridge','Native Gazebo input']
axes[0].bar(labels,[x['rtf'] for x in rows],color=['#75899d','#218875'])
axes[0].set_ylabel('Simulation seconds / wall second');axes[0].set_title('Transport real-time factor')
axes[0].set_ylim(0,.65)
agg=[x['cpu'].get('finger_contact_aggregator',0) for x in rows]
bridge=[sum(v for k,v in x['cpu'].items() if k.endswith('/finger_contact_bridge')) for x in rows]
axes[1].bar(labels,bridge,label='ROS contact bridges',color='#75899d')
axes[1].bar(labels,agg,bottom=bridge,label='C++ aggregator',color='#218875')
axes[1].set_ylabel('CPU % (100% = one logical core)');axes[1].set_title('Contact notification CPU');axes[1].legend()
for ax in axes:ax.tick_params(axis='x',labelsize=9);ax.grid(axis='y',alpha=.2)
fig.savefig(base/'native_contact_comparison.png',dpi=180)
fig.savefig(base/'native_contact_comparison.svg')

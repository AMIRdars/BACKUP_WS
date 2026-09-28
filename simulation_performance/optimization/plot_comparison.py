"""Export the successful 180-second experiments; full runs are separate."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

base = Path(__file__).resolve().parent
rows = json.loads((base / 'results.json').read_text())
transport = {r['condition']: r for r in rows if r['phase'] == 'transport'}
selection = [('A_baseline','Original 2 ms'), ('D_physics_3ms','3 ms'),
             ('E_physics_4ms','4 ms'), ('F_pose_filter','4 ms + pose'),
             ('G3_contact_aggregation_retry','+ contact (GUI)'),
             ('G4_contact_aggregation_headless','+ contact (headless)')]
selection = [(key,label) for key,label in selection if key in transport]
fig,axes = plt.subplots(1,2,figsize=(12,4.6))
colors = ['#527ba1' if 'headless' not in key else '#52916e' for key,label in selection]
axes[0].barh([label for key,label in selection],[transport[key]['rtf'] for key,label in selection],color=colors)
axes[0].invert_yaxis();axes[0].set_xlabel('RTF (sim seconds / wall seconds)')
axes[0].axvline(.5,color='gray',linestyle='--',linewidth=1);axes[0].set_xlim(0,.8)
axes[0].set_title('Transport phase: successful 180-second tests')
for i,(key,label) in enumerate(selection):axes[0].text(transport[key]['rtf']+.012,i,f"{transport[key]['rtf']:.3f}",va='center')
labels=['Coordinator','Slip monitor','Grasp manager']
sets=[('E_physics_4ms','Before pose/contact'),('F_pose_filter','After pose'),('G3_contact_aggregation_retry','After contact')]
for offset,(key,name) in enumerate(sets):
 if key not in transport:continue
 axes[1].bar([i+.25*(offset-1) for i in range(3)],
             [transport[key]['cpu'].get(n,0) for n in ['coordinator','slip_monitor','friction_grasp_manager']],width=.25,label=name)
axes[1].set_xticks(range(3),labels);axes[1].set_ylabel('Mean process CPU (%)');axes[1].set_title('GUI tests: 100% equals one logical CPU')
axes[1].legend(fontsize=8)
fig.text(.5,.01,'Different runs; GUI CPU and contact state vary. Raw data and quality checks accompany the report.',ha='center',fontsize=8)
fig.tight_layout(rect=[0,.03,1,1]);fig.savefig(base/'optimization_comparison.png',dpi=160)
fig.savefig(base/'optimization_comparison.svg');plt.close(fig)

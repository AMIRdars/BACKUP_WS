#!/usr/bin/env python3
"""Offline same-physics comparison; ground truth never enters control nodes."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def write(root,name,items):
    (root/(name+'.json')).write_text(json.dumps(items,indent=2)+'\n')
    with (root/(name+'.csv')).open('w') as file:
        writer=csv.DictWriter(file,fieldnames=sorted({k for item in items for k in item}))
        writer.writeheader();writer.writerows(items)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--audit-directory',type=Path,required=True)
    a=p.parse_args();root=a.audit_directory
    read=lambda f:json.loads(f.read_text())
    comparisons=[];faults=[];plot_data={};evidence=[]
    for directory in sorted(root.iterdir()):
        if not directory.is_dir() or not (directory/'result.json').exists():continue
        result=read(directory/'result.json');graph=read(directory/'graph.json')
        position=result.get('condition')=='comparison_position_baseline'
        legacy=result.get('condition')=='ARM_ONLY_ungrasped'
        required=['/follower/safety_gate','/follower/actuator_interlock','/follower/controller_audit']
        if not position:required+=['/follower/follower_arm_base','/follower/wrench_preprocessor']
        if legacy:required=['/follower/safety_gate','/follower/follower_arm_admittance','/follower/wrench_preprocessor']
        missing=[node for node in required if node not in graph]
        if missing and (result['passed'] or result.get('error_phase')!='startup'):
            raise ValueError('missing control node after startup: '+directory.name+' '+str(missing))
        for node in required:
            if node not in graph:continue
            for topic,_ in graph[node]['subscriptions']:
                if topic.startswith(('/leader/','/world/','/experiment/')) or 'ground_truth' in topic or topic.endswith(('/tf','/tf_static','/odom')) or 'transport_goal' in topic:
                    raise ValueError('forbidden follower input: '+topic)
        rows=[json.loads(line) for line in (directory/'samples.jsonl').open()]
        motion=[r for r in rows if r['phase'] in ('motion','settle','fault_injected') and 'payload_x' in r]
        evidence.append({'trial':directory.name,'passed':result['passed'],'error':result.get('error'),
            'error_phase':result.get('error_phase'),'boundary_passed':True,
            'required_nodes_complete':not missing,'missing_nodes':','.join(missing),'rows':len(rows)})
        if legacy or directory.name=='final_transport':continue
        if result['fault_test']!='none':
            if result['passed']:
                if not result['no_automatic_restart'] or not all(abs(result['twists_at_injection'][r][0])>5e-6 for r in ('leader','follower')):
                    raise ValueError('fault not tested while both robots were moving: '+directory.name)
                faults.append({'trial':directory.name,'fault':result['fault_test'],
                    'stop_latency_wall_s':result['dual_zero_latency_wall_s'],
                    'observation':result['stop_observation'],'max_post_fault_wheel_speed_rad_s':max(result['post_fault_max_wheel_speed_rad_s'].values()),
                    'max_post_fault_joint_span_rad':max(result['post_fault_joint_span_rad']),
                    'no_automatic_restart':result['no_automatic_restart']})
            continue
        if directory.name=='smoke':continue  # separate development smoke, not a fixed-environment comparison
        if not motion:continue
        # Startup failures are retained in evidence, not classified as delay results.
        if not result['passed'] and result.get('error_phase') not in ('motion','settle'):continue
        mode='POSITION' if position else result['control_mode'];delay=result.get('delay_s',0.)
        baseline=[r for r in rows if r['phase']==('hold' if position else 'baseline')]
        if not baseline:raise ValueError('missing pre-motion baseline: '+directory.name)
        t0=motion[0]['sim_time'];base=baseline[-1]
        late=[r for r in baseline if r['sim_time']>=base['sim_time']-1.]
        forces=[]
        for robot in ('leader','follower'):
            bias=np.mean([[r[robot+'_'+k] for k in ('fx','fy','fz')] for r in late],axis=0)
            forces.append(np.linalg.norm(np.array([[r[robot+'_'+k] for k in ('fx','fy','fz')] for r in motion])-bias,axis=1))
        times=np.array([r['sim_time']-t0 for r in motion]);force=np.maximum(*forces)
        final=motion[-1];travel={robot:final[robot+'_gt_x']-base[robot+'_gt_x'] for robot in ('leader','follower')}
        travel['payload']=final['payload_x']-base['payload_x']
        outcome='TRACKED' if result['passed'] else 'SAFE_STOP' if result.get('safe_stop_observed') else 'NOT_TRACKED'
        metrics={'trial':directory.name,'mode':mode,'delay_ms':delay*1000,'outcome':outcome,
            'reason':result.get('error',''),'duration_observed_s':float(times[-1]),
            'peak_delta_force_norm_N':float(np.max(force)),'rms_delta_force_norm_N':float(np.sqrt(np.mean(force**2))),
            'peak_URDF_effort_fraction':max(result['peak_joint_reaction_torque_Nm'][robot][i]/effort
                for robot in ('leader','follower') for i,effort in enumerate((33,22,26,13,4))),
            **{k+'_travel_m':v for k,v in travel.items()},
            'target_distance_m':result['distance_m'],'speed_m_s':result['speed_m_s'],'cfm':result['weld_cfm']}
        for robot in ('leader','follower'):
            profile=read(directory/'profiles_used'/(robot+'_safety.json'))
            metrics[robot+'_urdf_sha256']=profile['urdf_sha256'];metrics['world_sha256']=profile['world_sha256']
        comparisons.append(metrics);plot_data[mode,delay]=(times,motion,base,force,outcome)
        if not position:
            requests=[json.loads(line) for line in (directory/'command_outputs.jsonl').open()]
            ages=[r['receiver_sim']-r['stamp'] for r in requests if r.get('kind')=='arm_request' and r['robot']=='follower'
                  and motion[0]['wall_time']<=r['wall_time']<=motion[-1]['wall_time']]
            metrics['observed_arm_request_age_min_s']=min(ages) if ages else None
            metrics['observed_arm_request_age_max_s']=max(ages) if ages else None
        if not position and result['endpoint_states']['follower']['accepted_goals']!=0:
            raise ValueError('follower accepted goal: '+directory.name)
        if not position and any(r['applied_fx_N']!=0 or r['applied_fy_N']!=0 for r in rows):
            raise ValueError('external excitation during transport: '+directory.name)
    if comparisons:
        for key in ('world_sha256','leader_urdf_sha256','follower_urdf_sha256','target_distance_m','speed_m_s','cfm'):
            if len({row[key] for row in comparisons})!=1:raise ValueError('comparison conditions differ: '+key)
        # Use the standard twelve cases in the grid; optional extra delays remain CSV rows.
        for kind in ('travel','force'):
            fig,axes=plt.subplots(3,4,figsize=(17,10),sharex=True,layout='constrained')
            for i,mode in enumerate(('POSITION','ARM_ONLY','ARM_BASE')):
                for j,delay in enumerate((0.,.05,.1,.2)):
                    ax=axes[i,j];item=plot_data.get((mode,delay))
                    if item:
                        times,motion,base,force,outcome=item
                        if kind=='travel':
                            for robot in ('leader','follower'):
                                ax.plot(times,[(r[robot+'_gt_x']-base[robot+'_gt_x'])*1e3 for r in motion],label=robot)
                            ax.plot(times,[(r['payload_x']-base['payload_x'])*1e3 for r in motion],label='payload')
                        else:ax.plot(times,force,label='max of two wrist force changes')
                        ax.set_title(f'{mode}, {delay*1000:g} ms | {outcome}')
                        if kind=='travel' and outcome=='SAFE_STOP':
                            ax.scatter([times[-1]],[(motion[-1]['payload_x']-base['payload_x'])*1e3],marker='x',c='red',zorder=5)
                    ax.grid(True);ax.set_xlim(0,28)
                    if kind=='travel':ax.set_ylim(-.05,1.05)
                    if j==0:ax.set_ylabel('Travel [mm]' if kind=='travel' else 'Delta force norm [N]')
                    if i==2:ax.set_xlabel('Time since motion [sim s]')
            axes[0,0].legend(loc='best');fig.suptitle('Same .52 kg / CFM .1 / 1 mm / .05 mm/s; original command timestamps retained')
            fig.savefig(root/('comparison_'+kind+'.png'),dpi=160);plt.close(fig)
    write(root,'comparison',comparisons);write(root,'faults',faults);write(root,'experiment_inventory',evidence)
    print(json.dumps({'comparison_trials':len(comparisons),'successful_fault_trials':len(faults),
        'outcomes':{value:sum(r['outcome']==value for r in comparisons) for value in ('TRACKED','SAFE_STOP','NOT_TRACKED')}},indent=2))


if __name__=='__main__':main()

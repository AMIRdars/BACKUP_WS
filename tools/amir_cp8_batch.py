#!/usr/bin/env python3
"""Serial CP-8 same-physics comparison; failures retained, no auto re-arm."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--suite',choices=['comparison','faults'],required=True)
    p.add_argument('--prefix',default='');p.add_argument('--audit-directory',type=Path,required=True);p.add_argument('--yaw-profile',choices=['initial','stiff','filtered'],default='initial');p.add_argument('--control-mode',choices=['ALL','ARM_BASE','WHOLE_BODY'],default='ALL');p.add_argument('--settle-seconds',type=float,choices=[6.,8.,12.],default=6.);a=p.parse_args()
    root=Path('audits/whole_body_compliance_20261019/cp6').resolve();records=[]
    if a.suite=='comparison':
        cases=[(mode.lower()+'_'+axis,mode,axis,dist,'none') for mode in ('ARM_BASE','WHOLE_BODY')
            for axis,dist in [('forward',.001),('backward',.0005),('lateral',.0001),('yaw',.0001),('pivot',.0001)]]
    else:
        cases=[('fault_'+fault,'WHOLE_BODY','forward',.0005,fault) for fault in
            ('gate','interlock','supervisor','relay','audit','controller','ownership','action','dropout','preprocessor','clock')]
    if a.control_mode!='ALL':cases=[c for c in cases if c[1]==a.control_mode]
    for i,(name,mode,axis,distance,fault) in enumerate(cases):
        name=a.prefix+name;output=a.audit_directory/name
        if output.exists():raise FileExistsError(output)
        command=['/usr/bin/python3','tools/amir_cp8_trial.py','--run-directory',str(root/'generated'),
            '--calibration-file',str(root/'calibration/calibration.json'),'--output',str(output),
            '--control-mode',mode,'--yaw-profile',a.yaw_profile,'--settle-seconds',str(a.settle_seconds),'--axis',axis,'--distance',str(distance),'--fault',fault]
        env=dict(os.environ,ROS_DOMAIN_ID=str(100+i),ROS_LOCALHOST_ONLY='1',IGN_PARTITION='amir_cp8_'+name,
            RMW_FASTRTPS_PUBLICATION_MODE='ASYNCHRONOUS',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
        files=[*Path('src/humbleble_ws/amir_compliance_control').rglob('*.py'),Path('tools/amir_cp8_trial.py')]
        snapshot={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in files}
        with (a.audit_directory/(name+'_console.log')).open('w') as log:result=subprocess.run(command,env=env,stdout=log,stderr=subprocess.STDOUT)
        report=json.loads((output/'result.json').read_text()) if (output/'result.json').exists() else {}
        if output.exists():(output/'batch_snapshot.json').write_text(json.dumps({'command':command,'source_hashes':snapshot,
            'env':{k:env[k] for k in ('ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','IGN_PARTITION','OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS')}},indent=2)+'\n')
        record={'trial':name,'mode':mode,'axis':axis,'distance_m':distance,'fault':fault,'exit_code':result.returncode,
            **{k:report.get(k) for k in ('passed','safe_stop_observed','error','error_phase','tracking_ratios','dual_zero_latency_wall_s')}}
        records.append(record);(a.audit_directory/(a.prefix+a.suite+'_batch.json')).write_text(json.dumps(records,indent=2)+'\n');print(json.dumps(record),flush=True)
        bounded=(report.get('error_phase')=='settle' and report.get('error')=='local follower transport or final settling check failed')
        if not report or (not report.get('passed') and not (a.suite=='comparison' and report.get('safe_stop_observed')) and not bounded):
            raise RuntimeError('unexpected experiment failure; inspect before continuing: '+name)


if __name__=='__main__':main()

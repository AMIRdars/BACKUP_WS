#!/usr/bin/env python3
"""Serial isolated CP-7 experiments; preserve failures and bounded startup retry."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--suite',choices=['faults','comparison'],required=True)
    p.add_argument('--audit-directory',type=Path,required=True)
    p.add_argument('--run-directory',type=Path,required=True)
    p.add_argument('--calibration-file',type=Path,required=True)
    p.add_argument('--prefix',default='',help='distinct output names for a later verification batch')
    a=p.parse_args();records=[]
    if a.suite=='faults':
        cases=[('guarded_fault_'+f,'ARM_BASE',0.,f) for f in
               ('gate','interlock','supervisor','relay','audit','controller','ownership','action','dropout','preprocessor','clock')]
    else:
        cases=[(m.lower()+'_delay_'+str(round(d*1000))+'ms',m,d,'none')
               for m in ('POSITION','ARM_ONLY','ARM_BASE') for d in (0.,.05,.1,.2)]
    for index,(case,mode,delay,fault) in enumerate(cases):
        for attempt in range(2):
            name=a.prefix+case+('_retry' if attempt else '')
            output=a.audit_directory/name
            if output.exists():raise FileExistsError(output)
            tool='tools/amir_cp7_position_trial.py' if mode=='POSITION' else 'tools/amir_cp7_trial.py'
            command=['python3',tool,'--run-directory',str(a.run_directory),'--output',str(output),'--distance','.001' if fault=='none' else '.0005',
                     '--delay',str(delay)]
            if mode!='POSITION':command+=['--calibration-file',str(a.calibration_file),'--control-mode',mode,'--fault',fault]
            env=dict(os.environ,ROS_DOMAIN_ID=str(75+index),ROS_LOCALHOST_ONLY='1',IGN_PARTITION='amir_cp7_'+name,
                     RMW_FASTRTPS_PUBLICATION_MODE='ASYNCHRONOUS',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
            files=[*Path('src/humbleble_ws/amir_compliance_control').rglob('*.py'),
                   Path('src/humbleble_ws/amir_compliance_control/package.xml'),Path(tool)]
            snapshot={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in files}
            with (a.audit_directory/(name+'_console.log')).open('w') as log:
                result=subprocess.run(command,env=env,stdout=log,stderr=subprocess.STDOUT)
            report=json.loads((output/'result.json').read_text()) if (output/'result.json').exists() else {}
            if output.exists():
                (output/'batch_snapshot.json').write_text(json.dumps({'command':command,'source_hashes':snapshot,
                    'env':{k:env[k] for k in ('ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','IGN_PARTITION','OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS')}},indent=2)+'\n')
            record={'case':case,'trial':name,'mode':mode,'delay_s':delay,'fault':fault,'exit_code':result.returncode,
                **{k:report.get(k) for k in ('passed','safe_stop_observed','error','error_phase','dual_zero_latency_wall_s')}}
            records.append(record)
            (a.audit_directory/(a.prefix+a.suite+'_batch.json')).write_text(json.dumps(records,indent=2)+'\n')
            print(json.dumps(record),flush=True)
            if not report:raise RuntimeError('no complete report: '+name)
            bounded_nontracking=(mode=='ARM_ONLY' and report.get('error_phase')=='settle'
                and report.get('error')=='local follower transport or final settling check failed'
                and report.get('endpoint_states',{}).get('leader',{}).get('completed_goals')==1)
            if report.get('passed') or report.get('safe_stop_observed') or bounded_nontracking:break
            if report.get('error_phase') in ('motion','settle','fault_injected') or attempt:
                raise RuntimeError('unexpected experiment failure: '+name)
            # One fresh-scene retry only before motion. The original failure is
            # retained and is never counted as a successful injected fault.


if __name__=='__main__':main()

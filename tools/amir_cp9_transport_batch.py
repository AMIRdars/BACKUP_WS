#!/usr/bin/env python3
"""Sequential native contact trials, isolated domain/partition, no hardware."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys


CASES = ['static', 'forward100', 'forward1000', 'backward500', 'lateral100'] + [
    'fault_'+v for v in ('contacts','nonfinite','mimic','overload','dropout','controller',
                        'ownership','interlock','grasp_monitor')]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--audit-directory',type=Path,required=True)
    p.add_argument('--metrology-directory',type=Path,required=True)
    p.add_argument('--run-directory',type=Path,required=True)
    p.add_argument('--prefix',required=True)
    p.add_argument('--cases',nargs='+',choices=CASES,required=True)
    a=p.parse_args();a.audit_directory=a.audit_directory.resolve()
    results=[]
    for index,case in enumerate(a.cases):
        name=a.prefix+case;output=a.audit_directory/name
        if output.exists():raise ValueError('evidence already exists: '+str(output))
        env=dict(os.environ,ROS_DOMAIN_ID=str(90+index),ROS_LOCALHOST_ONLY='1',
            IGN_PARTITION='amir_cp9_'+name,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
        args=[sys.executable,'tools/amir_cp9_transport_trial.py','--run-directory',str(a.run_directory.resolve()),
            '--metrology-directory',str(a.metrology_directory.resolve()),'--output',str(output)]
        if case=='static':args += ['--mode','static']
        elif case.startswith('fault_'):args += ['--fault',case[6:],'--distance','.001']
        else:
            axis='backward' if case.startswith('backward') else 'lateral' if case.startswith('lateral') else 'forward'
            distance={'forward100':'.0001','forward1000':'.001','backward500':'.0005','lateral100':'.0001'}[case]
            args += ['--axis',axis,'--distance',distance]
        print('START '+name,flush=True)
        with (a.audit_directory/(name+'.log')).open('w') as log:
            completed=subprocess.run(args,env=env,stdout=log,stderr=subprocess.STDOUT)
        report=json.loads((output/'result.json').read_text()) if (output/'result.json').exists() else {}
        results.append({'trial':name,'returncode':completed.returncode,'passed':report.get('passed',False),
            'error':report.get('error'),'phase':report.get('error_phase')})
        (a.audit_directory/(a.prefix+'batch.json')).write_text(json.dumps(results,indent=2))
        print(json.dumps(results[-1]),flush=True)
        if completed.returncode or not report.get('passed'):
            raise SystemExit(1)


if __name__=='__main__':main()

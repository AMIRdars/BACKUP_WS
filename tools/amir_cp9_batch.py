#!/usr/bin/env python3
"""Serial isolated native contact qualification; no hardware actions."""
import argparse
import json
import os
from pathlib import Path
import subprocess


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--audit-directory',type=Path,required=True)
    p.add_argument('--prefix',default='validated_');a=p.parse_args();results=[]
    for index,role in enumerate(('hold','negative','contacts','nonfinite','mimic','overload')):
        trial=a.prefix+'contact_'+role;output=a.audit_directory/trial
        command=['/usr/bin/python3','tools/amir_cp9_contact_probe.py','--output',str(output)]
        if role!='negative':command+=['--close-grippers']
        if role not in ('negative','hold'):command+=['--fault',role]
        env=dict(os.environ,ROS_DOMAIN_ID=str(93+index),ROS_LOCALHOST_ONLY='1',IGN_PARTITION='amir_cp9_'+trial,
            OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
        with (a.audit_directory/(trial+'_console.log')).open('w') as log:
            code=subprocess.call(command,env=env,stdout=log,stderr=subprocess.STDOUT)
        report=json.loads((output/'result.json').read_text()) if (output/'result.json').exists() else {}
        item={'trial':trial,'role':role,'return_code':code,'passed':report.get('passed',False),'fault_injected':report.get('fault_injected',False)}
        results.append(item);(a.audit_directory/(a.prefix+'batch.json')).write_text(json.dumps(results,indent=2)+'\n')
        print(json.dumps(item),flush=True)
        if role=='hold' and not item['passed']:raise SystemExit('static hold failed; fault cases not run')
    if not all(r['passed'] for r in results):raise SystemExit(1)


if __name__=='__main__':main()

#!/usr/bin/env python3
"""Run CP-8 ungrasped sign probes with measured CP-6 test profiles, restore bytes."""
import argparse
import json
import os
from pathlib import Path
import subprocess


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--mode',choices=['signs_xy','signs_yaw'],required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--yaw-profile',choices=['initial','stiff','filtered'],default='initial')
    a=p.parse_args()
    root=Path('audits/whole_body_compliance_20261019/cp6').resolve()
    backups={r:(root/'generated'/(r+'_safety.json')).read_bytes() for r in ('leader','follower')}
    try:
        for r in backups:
            (root/'generated'/(r+'_safety.json')).write_bytes((root/'sign_profiles'/(r+'_safety.json')).read_bytes())
        env=dict(os.environ,ROS_DOMAIN_ID='92',ROS_LOCALHOST_ONLY='1',IGN_PARTITION='amir_cp8_'+a.output.name,
            RMW_FASTRTPS_PUBLICATION_MODE='ASYNCHRONOUS',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
        result=subprocess.run(['/usr/bin/python3','tools/amir_cp8_trial.py','--run-directory',str(root/'generated'),
            '--calibration-file',str(root/'calibration/calibration.json'),'--output',str(a.output),'--mode',a.mode,'--yaw-profile',a.yaw_profile],env=env)
    finally:
        for r,data in backups.items():(root/'generated'/(r+'_safety.json')).write_bytes(data)
        a.output.parent.joinpath(a.output.name+'_profiles_restored.json').write_text(json.dumps({r:
            (root/'generated'/(r+'_safety.json')).read_bytes()==data for r,data in backups.items()},indent=2))
    raise SystemExit(result.returncode)


if __name__=='__main__':main()

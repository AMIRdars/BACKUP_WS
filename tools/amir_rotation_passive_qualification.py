#!/usr/bin/env python3
"""Offline issuance only after real local static / positive / negative data.

Each JSONL row: phase (static/positive/negative), timestamp, q_rad[5],
qdot_rad_s[5], wrench_tcp[6] (preprocessed, at grasp in local base axes),
base_twist[3], grasp_attached. No leader/GT value is used for acceptance.
This tool never moves a robot and never supplies a missing trial.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from amir_compliance_control.rotation_observer import ProjectedYaw,qualify_passive


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('urdf','evidence','sensor-certificate','output'):p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--orientation-envelope-rad',type=float,required=True)
    p.add_argument('--joint-margin-rad',type=float,required=True)
    p.add_argument('--yaw-heading-frame',choices=('tcp_link','grasp_heading'),default='tcp_link')
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    report=dict(passed=False,certificate_issued=False)
    try:
        xml=a.urdf.read_text();model_hash=hashlib.sha256(xml.encode()).hexdigest()
        sensor=json.loads(a.sensor_certificate.read_text())
        if (not isinstance(sensor,dict) or sensor.get('sensor_calibration_passed') is not True
                or sensor.get('urdf_sha256')!=model_hash or set(sensor.get('verified_axes',[]))!={'fx','fy','fz','tx','ty','tz'}):
            raise ValueError('matching measured sensor certificate required')
        commands=np.array([r['command'] for r in sensor['tests']])
        if commands.ndim!=2 or commands.shape[1]!=6 or not np.isfinite(commands).all():
            raise ValueError('measured six-axis excitation records required')
        forces=np.linalg.norm(commands[:,:3],axis=1);moments=np.linalg.norm(commands[:,3:],axis=1)
        g=ProjectedYaw(xml,orientation_envelope_rad=a.orientation_envelope_rad,joint_margin_rad=a.joint_margin_rad,
                       heading_frame=a.yaw_heading_frame)
        result=qualify_passive([json.loads(line) for line in a.evidence.open()],g,
            force_limit_N=float(min(forces[forces>0.])),torque_limit_Nm=float(min(moments[moments>0.])),
            evidence_sha256={str(a.evidence):hashlib.sha256(a.evidence.read_bytes()).hexdigest()})
        result.update(robot='follower',urdf_sha256=model_hash)
        (a.output/'qualification.json').write_text(json.dumps(result,indent=2)+'\n')
        report.update(passed=True,certificate_issued=True)
    except (ValueError,KeyError,TypeError) as exc:report['reason']=str(exc)
    (a.output/'result.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2));return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())

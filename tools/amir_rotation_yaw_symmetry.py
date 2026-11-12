#!/usr/bin/env python3
"""Opt-in exact URDF symmetry extension of an already measured O1 certificate.

No new physical rotation is claimed. Empirical bounds and noise thresholds
remain recorded unchanged. The first-joint extension is valid only for a
proved vertical first axis, within the existing local pose envelope; all
other non-invariant joints retain their measured region. Runtime F/T, FK
continuity/noise, joint limits, pose and time watchdogs remain mandatory.
"""
import argparse,hashlib,json
from pathlib import Path
from amir_compliance_control.rotation_observer import ProjectedYaw,read_qualification


def extend(file,urdf,*,joint_margin_rad):
    digest=hashlib.sha256(urdf.read_bytes()).hexdigest()
    old=json.loads(file.read_text())
    g=ProjectedYaw(urdf.read_text(),orientation_envelope_rad=old['invariant_joint_pose_envelope_rad'],
        joint_margin_rad=joint_margin_rad,heading_frame=old['yaw_heading_frame'])
    cert=read_qualification(file,digest,g)
    for source,expected in cert['evidence_sha256'].items():
        if hashlib.sha256(Path(source).read_bytes()).hexdigest()!=expected:
            raise ValueError('measured O1 evidence changed')
    if g.yaw_equivariance_proof is None:raise ValueError('exact vertical first-axis proof unavailable')
    cert.update(yaw_equivariance_proof=g.yaw_equivariance_proof,
        equivariant_joint_pose_envelope_rad=g.envelope,
        analytic_extension_source_sha256=hashlib.sha256(file.read_bytes()).hexdigest(),
        analytic_extension='URDF first-axis yaw equivariance; no additional passive object angle measured')
    return cert


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('qualification','urdf','output'):p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--joint-margin-rad',type=float,required=True)
    a=p.parse_args();value=extend(a.qualification.resolve(),a.urdf.resolve(),joint_margin_rad=a.joint_margin_rad)
    with a.output.open('x') as f:f.write(json.dumps(value,indent=2)+'\n')
    read_qualification(a.output,value['urdf_sha256'],ProjectedYaw(a.urdf.read_text(),
        orientation_envelope_rad=value['equivariant_joint_pose_envelope_rad'],joint_margin_rad=a.joint_margin_rad,
        heading_frame=value['yaw_heading_frame']))
    print(json.dumps({'certificate':str(a.output),'measured_passive_angles_unchanged':True,
        'proof':value['yaw_equivariance_proof']},indent=2))
    return 0

if __name__=='__main__':raise SystemExit(main())

#!/usr/bin/env python3
"""Fit offline leader distance scale from completed native simulation trials."""
import argparse
import hashlib
import json
from pathlib import Path


def calibrate(cases,root,method='min_minus_observed_range'):
    samples=[];artifacts=None;cap=None
    for case in cases:
        case=case.resolve();e=json.loads((case/'p0_evidence.json').read_text())
        h=json.loads((case/'held_result.json').read_text())
        # A target-distance miss is precisely a valid distance-calibration
        # observation. Every other acceptance check must still pass.
        other_checks={k:v for k,v in e['checks'].items()
                      if k!='calibrated_leader_actual_world_x_reaches_requested_distance'}
        if not (all(other_checks.values()) and h['passed'] and h['safe_stop_verified'] and h['grasp_held_after_stop']
                and h['native_arm_stroke'] and h['follower_base_fixture']):
            raise ValueError('completed native friction + base fixture transport evidence required')
        values={name:hashlib.sha256((case/'generated'/name).read_bytes()).hexdigest()
                for name in ('leader.urdf','leader_controllers.yaml','leader_base_controller.yaml','world.sdf','base_fixture_proof.json')}
        if artifacts is not None and values!=artifacts:raise ValueError('distance samples use different artifacts')
        if cap is not None and cap!=h['leader_velocity_cap_m_s']:raise ValueError('distance samples use different speed caps')
        artifacts=values;cap=h['leader_velocity_cap_m_s']
        samples.append({'case':str(case.relative_to(root)),
            'raw_odom_distance_m':e['leader_distance_m'],
            'measured_body_gt_x_m':e['leader_world_base_displacement_xyz_m'][0],
            'transport_checks_passed':True,
            'target_distance_reached':e['leader_world_base_displacement_xyz_m'][0]>=h['requested_distance_m'],
            'p0_evidence_sha256':hashlib.sha256((case/'p0_evidence.json').read_bytes()).hexdigest(),
            'scope':'completed transport and verified stop; live GUI endurance is separate'})
    if len(samples)<2 or len({s['case'] for s in samples})!=len(samples):raise ValueError('two independent trials required')
    ratios=[s['measured_body_gt_x_m']/s['raw_odom_distance_m'] for s in samples]
    span=max(ratios)-min(ratios)
    scale=sum(ratios)/len(ratios) if method=='mean' else min(ratios)-span
    return {'schema':1,'scope':'simulation_native_forward_progressive_stroke_distance_only',
        'calibration_source':'completed native friction + follower base fixture trials; evaluation GT is offline metrology only',
        'artifact_sha256':artifacts,'samples':samples,'metres_per_raw_odom_metre':scale,
        'fit_method':method,'empirical_scale_reserve':span if method=='min_minus_observed_range' else 0.,
        'sample_scale_range':[min(ratios),max(ratios)],'raw_command_cap_m_s':cap,'passed':True}


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--case',action='append',type=Path,required=True)
    p.add_argument('--method',choices=['mean','min_minus_observed_range'],default='min_minus_observed_range')
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    cert=calibrate(a.case,Path(__file__).resolve().parents[1],a.method)
    a.output.write_text(json.dumps(cert,indent=2)+'\n');print(cert['metres_per_raw_odom_metre'])

#!/usr/bin/env python3
"""Separate experiment safety evidence from actual speed qualification."""
import argparse
import csv
import hashlib
import json
import subprocess
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def verify_case(folder):
    result = json.loads((folder/'result.json').read_text())
    if not (folder/'graph.json').exists():
        return {'startup_completed':False,'no_unready_motion':result.get('leader_odom_peak_m_s',0)<1e-6},False
    graph = json.loads((folder/'graph.json').read_text())
    policy = json.loads((folder/'experiment_policy.json').read_text())
    rows = list(csv.DictReader((folder/'timeseries.csv').open()))
    active = [r for r in rows if r.get('admittance_enabled') == 'True']
    events = [json.loads(s) for s in (folder/'events.jsonl').read_text().splitlines()]
    checks = {}
    checks['actual_stop_and_cleanup'] = result.get('safe_stop_verified') and result.get('remaining_owned_pids') == []
    checks['both_grippers_active'] = all(result['post_stop_controllers'][r].get('gripper_controller') == 'active'
                                         for r in ('leader', 'follower'))
    checks['grasp_retained'] = result['stage'] == 'AF-1' or result.get('grasp_held_after_stop')
    evidence = result['base_stop_evidence']
    checks['fresh_stop_evidence'] = set(evidence) == {'leader', 'follower'} and all(
        e['stopped'] and e['wheel_velocity_peak_rad_s'] < 1e-4
        and e['ground_truth_position_span_m'] < 1e-6 for e in evidence.values())
    command = [e for e in events if e['robot'] == 'follower' and e['kind'] in ('base_cmd', 'rover_twist')]
    checks['every_base_request_and_output_zero'] = bool(command) and all(v == 0. for e in command for v in e['value'])
    for topic, name in [('base_cmd', 'follower_arm_only'), ('rover_twist', 'actuator_interlock'),
                        ('arm_controller/joint_trajectory', 'actuator_interlock')]:
        publishers = graph['/follower/'+topic]
        checks['single_writer:'+topic] = len(publishers) == 1 and publishers[0]['node'] == '/follower/'+name
    for name in ('follower_arm_only', 'wrench_preprocessor'):
        topics = [t[0] for t in graph['/follower/'+name+'/subscriptions']]
        checks['local_inputs:'+name] = bool(topics) and not any(t.startswith(('/leader/', '/world/'))
            or 'ground_truth' in t or 'goal' in t for t in topics)
    parameters = policy['controller_parameters']
    checks['original_incremental_wrench_boundaries'] = (parameters['force_increment_stop_N'] == .5
        and parameters['torque_increment_stop_Nm'] == .05)
    checks['same_measured_fixture'] = all(e['only_controller_path_changed'] and e['world_unchanged']
        for e in json.loads((folder/'generated/fixture_relocation.json').read_text()).values())
    if 'controller_states' in graph:
        checks['both_grippers_ready_before_arm'] = all(graph['controller_states'][r].get('gripper_controller')=='active'
                                                       for r in ('leader','follower'))

    def vector(row, prefix, size):
        return np.array([float(row[prefix+'_'+str(i)]) for i in range(size)])

    measured = [r for r in active if r.get('adm_p_actual_m_0')]
    residual = []
    for row in measured:
        home, actual = vector(row, 'adm_p_home_m', 3), vector(row, 'adm_p_actual_m', 3)
        ref = vector(row, 'adm_p_ref_xy_m', 2)
        residual.append(np.max(np.abs(np.r_[vector(row, 'adm_e_arm_m', 3)-(actual-home),
            vector(row, 'adm_e_track_xy_m', 2)-(ref-actual[:2]),
            vector(row, 'adm_delta_p_adm_m', 2)-(ref-home[:2])])) )
    checks['distinct_admittance_measured_and_tracking_states'] = bool(residual) and max(residual) < 1e-12
    checks['home_stays_latched'] = bool(measured) and np.max(np.ptp(
        [vector(r, 'adm_p_home_m', 3) for r in measured], axis=0)) < 1e-12
    checks['arm_only_rotation_disabled'] = bool(active) and all(r.get('adm_mode') == 'ARM_ONLY'
        and r.get('adm_rotation_tracking_enabled') == 'False' for r in active)
    checks['joint_request_rate_bound'] = bool(measured) and all(
        max(np.abs(vector(r, 'adm_qdot_cmd_rad_s', 5))) <= .030000001 for r in measured)
    checks['base_drift_below_1mm'] = bool(measured) and all(
        np.linalg.norm(vector(r, 'adm_base_drift_xy_yaw', 3)[:2]) < .001 for r in measured)
    if result['stage'] == 'AF-2' and not result['passed']:
        reason = result['final_endpoints']['follower']['reason']
        checks['failed_trial_has_latched_local_stop'] = reason.startswith('ARM_ONLY ') and all(
            e['state'] == 'SAFE_STOP' for e in result['final_endpoints'].values())
    if result['stage']=='AF-3':
        checks['fault_test_passed']=result['passed']
        checks['bounded_stop_latency']=0<=result.get('stop_latency_wall_s',float('inf'))<=.4
        fraction=result.get('fault_min_speed_fraction',0.)
        checks['actual_motion_before_injection']=result.get('leader_odom_velocity_at_injection',0)>0
        if fraction:
            required=result['leader_velocity_cap_m_s']*fraction
            checks['injection_at_required_speed']=(result.get('leader_odom_velocity_at_injection',0)>=required
                and result.get('leader_command_at_injection',[0])[0]>=required)
    if (folder/'servo_response_used.json').exists():
        from amir_compliance_control.arm_servo_response import validate_response
        cert = json.loads((folder/'servo_response_used.json').read_text())
        generated = folder/'generated'
        validate_response(cert, (generated/'follower.urdf').read_text(), generated)
        params = graph['follower_servo_parameters']
        checks['measured_servo_parameters_match'] = (
            params['gz_ros2_control']['position_proportional_gain'] == cert['position_proportional_gain']
            and params['controller_manager']['update_rate'] == cert['controller_update_rate_hz']
            and params['arm_controller']['open_loop_control'] == cert['open_loop_control'])
    speed_qualified = bool(result['stage'] == 'AF-2' and result['passed']
        and result.get('required_speed_fraction', 0) >= .98
        and result.get('leader_odom_peak_m_s', 0) >= .98*.005
        and result.get('leader_command_peak_m_s', 0) >= .98*.005)
    return {k: bool(v) for k, v in checks.items()}, speed_qualified


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--directory', type=Path, default=ROOT/'audits/arm_only_speed_20261109')
    p.add_argument('--case', action='append', help='explicit evidence subset; historical failures remain in reports')
    p.add_argument('--output-name', default='verification')
    a = p.parse_args(); root = a.directory.resolve()
    checks = {'preexisting_tracked_diff_preserved': subprocess.check_output(
        ['git', 'diff', '--binary'], cwd=ROOT) == (root/'preexisting.patch').read_bytes()}
    baseline = json.loads((root/'baseline_manifest.json').read_text())
    # These existing actuator/safety interfaces have no changes in this phase.
    for item in baseline:
        if Path(item['path']).name in ('setup.py','safety.py','transport.py','test_cp7_safety.py','arm_only_dls.py'):
            checks['preserved:'+item['path']] = hashlib.sha256((ROOT/item['path']).read_bytes()).hexdigest() == item['sha256']
    qualified = []
    files=[root/c/'result.json' for c in a.case] if a.case else sorted(root.glob('*/result.json'))
    for f in files:
        case_checks, passed = verify_case(f.parent)
        checks.update({f.parent.name+':'+k: v for k, v in case_checks.items()})
        if passed: qualified.append(f.parent.name)
    report = {'safety_invariants_passed': all(checks.values()), 'checks': checks,
              'cases':[f.parent.name for f in files],
              'speed_qualified_cases': qualified,
              'required_speed_demonstrated': bool(qualified),
              'limitations': ['Simulation fixed constraint only; no hardware load rating or friction grasp qualification.']}
    (root/(a.output_name+'.json')).write_text(json.dumps(report, indent=2))
    print(json.dumps({'checks': len(checks), 'safety_invariants_passed': all(checks.values()),
        'required_speed_demonstrated': bool(qualified), 'failed': [k for k, v in checks.items() if not v]}))
    return 0 if all(checks.values()) else 1


if __name__ == '__main__':
    raise SystemExit(main())

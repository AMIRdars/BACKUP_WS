"""CP-13 evaluation and release checks, outside both robot controllers.

Ground truth only evaluates completed legs. It never supplies a pose goal,
velocity command or a reference to either robot.
"""
import hashlib
import json
import math
from pathlib import Path


STAGE = Path('audits/whole_body_compliance_20261019/cp13_round_trip')
SOURCES = (
    'tools/amir_round_trip_trial.py',
    'tools/round_trip_evidence.py',
    'tools/round_trip_safety_gate.py',
    'tools/test_round_trip.py',
    'tools/test_round_trip_gate.py',
    'src/humbleble_ws/cooperative_transport_bringup/launch/compliance_round_trip.launch.py',
    'src/humbleble_ws/cooperative_transport_bringup/launch/compliance_round_trip_safety.launch.py',
    'src/humbleble_ws/cooperative_transport_bringup/launch/compliance_round_trip_scene.launch.py',
)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def leg_metrics(rows, origins, end, signed_distance):
    """Signed world/encoder displacement, with no control side effects."""
    result = {'signed_distance_m': signed_distance, 'bodies': {}}
    direction = math.copysign(1., signed_distance)
    for body, prefix in (('leader', 'leader_gt_'), ('follower', 'follower_gt_'),
                         ('cooperative_payload', 'payload_')):
        start = origins[body]
        finish = end[body]
        # Compare in the leader's initial local x axis, also for the return.
        yaw = origins['leader'][2]
        dx, dy = finish[0]-start[0], finish[1]-start[1]
        along = math.cos(yaw)*dx+math.sin(yaw)*dy
        cross = -math.sin(yaw)*dx+math.cos(yaw)*dy
        directed_speeds = []
        first = 0
        for row in rows:
            while first+1 < len(rows) and rows[first+1]['sim_time'] <= row['sim_time']-1.:
                first += 1
            previous = rows[first]
            dt = row['sim_time']-previous['sim_time']
            if dt >= 1.:
                x = row[prefix+'x']-previous[prefix+'x']
                y = row[prefix+'y']-previous[prefix+'y']
                directed_speeds.append(direction*(math.cos(yaw)*x+math.sin(yaw)*y)/dt)
        result['bodies'][body] = {
            'signed_displacement_m': along, 'cross_displacement_m': cross,
            'tracking_ratio': along/signed_distance,
            'peak_one_second_directed_speed_m_s': max(directed_speeds, default=0.),
        }
    return result


def normal_passed(result):
    legs = result.get('legs', [])
    return (result.get('passed') is True and result.get('round_trip_verified') is True
        and result.get('distance_m') == 1. and result.get('speed_m_s') == .1
        and len(legs) == 2 and [v.get('signed_distance_m') for v in legs] == [1., -1.]
        and all(set(v.get('bodies', {})) == {'leader','follower','cooperative_payload'}
            and all(.7 <= b.get('tracking_ratio', 0.) <= 1.3
                and b.get('peak_one_second_directed_speed_m_s', 0.) > .05
                for b in v['bodies'].values())
            and 0 <= v.get('final_arm_offset_xy_m', math.inf) <= .0001
            and 0 <= v.get('final_follower_speed_m_s', math.inf) <= .00001 for v in legs)
        and result.get('leader_completed_goals_before_teardown') == 2
        and result.get('follower_accepted_goals_before_teardown') == 0)


def fault_passed(result):
    return (result.get('passed') is True and result.get('fault_injected') is True
        and result.get('fault_leg') == 'return' and result.get('fault_test') == 'nonfinite'
        and result.get('distance_m') == 1. and result.get('speed_m_s') == .1
        and result.get('no_automatic_restart') is True
        and 0 <= result.get('dual_zero_latency_wall_s', math.inf) <= .4
        and result.get('goals_after_fault_recovery') == result.get('goals_at_injection') == 2
        and set(result.get('measured_encoder_vx_at_injection', {})) == {'leader','follower'}
        and result['measured_encoder_vx_at_injection']['leader'] < -.01
        and all(math.isfinite(v) and abs(v)>.01
                and v*result.get('twists_at_injection',{}).get(r,[0.])[0]>0
                for r,v in result['measured_encoder_vx_at_injection'].items())
        and all(result.get('post_fault_controller_states', {}).get(r, {}).get(c) == 'inactive'
                for r in ('leader','follower') for c in ('arm_controller','mecanum_drive_controller')))


def qualification(root, parent):
    root = Path(root)
    path = root/STAGE/'qualified_round_trip.json'
    if not path.is_file():
        raise ValueError('CP-13 round-trip qualification is incomplete: '+str(path))
    m = json.loads(path.read_text())
    if (m.get('schema_version') != 1 or m.get('passed') is not True
            or m.get('simulation_only') is not True or m.get('distance_m') != 1.
            or m.get('speed_m_s') != .1):
        raise ValueError('passing simulation-only 1 m out / 1 m back qualification required')
    if not set(SOURCES) <= m.get('source_sha256', {}).keys():
        raise ValueError('round-trip sources are not covered by evidence')
    parent_path = 'audits/whole_body_compliance_20261019/cp12_speed_release/qualified_speed_010.json'
    if m.get('parent_qualification_sha256') != sha(root/parent_path):
        raise ValueError('parent qualification changed')
    for mapping in ('source_sha256','evidence_sha256'):
        for relative, digest in m.get(mapping, {}).items():
            if sha(root/relative) != digest:
                raise ValueError('round-trip source/evidence changed: '+relative)
    for key, valid in (('normal_trial',normal_passed),('fault_trial',fault_passed)):
        folder = root/m[key]
        required = {str(Path(m[key])/name) for name in
            ('result.json','control_source_hashes.json','trial_source_hashes.json',
             'control_policy.json','leader_policy.json','original_profiles_restored.json','command.json')}
        if not required <= m.get('evidence_sha256', {}).keys():
            raise ValueError('round-trip runtime evidence is incomplete')
        result = json.loads((folder/'result.json').read_text())
        if (not valid(result) or not result.get('both_arm_base_inactive')
                or not result.get('follower_input_boundary_passed')):
            raise ValueError('round-trip normal and return-fault acceptance required')
        if not all(json.loads((folder/'original_profiles_restored.json').read_text()).values()):
            raise ValueError('ordinary profiles were not restored')
        expected_control = json.loads((root/parent['normal_trial']/'control_source_hashes.json').read_text())
        if json.loads((folder/'control_source_hashes.json').read_text()) != expected_control:
            raise ValueError('round-trip controller sources differ from qualified controllers')
        for filename,digest in json.loads((folder/'trial_source_hashes.json').read_text()).items():
            if sha(filename) != digest:
                raise ValueError('round-trip runtime trial source changed: '+filename)
    return m

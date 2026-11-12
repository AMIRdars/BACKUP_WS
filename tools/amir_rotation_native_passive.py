"""Bounded native-contact Gate-1/2 evidence, using own measurements only.

The follower base stays still. This code never enables autonomous base yaw
and cannot turn GT, a leader reference, or virtual admittance into evidence.
"""
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import yaml

from amir_compliance_control.rotation_observer import qualify_passive
from amir_compliance_control.wrench import finite
from amir_rotation_integration_audit import canonical_model
from amir_rotation_native_static import NativeStaticYaw


def require_native_gate0(result_file, fixture):
    result_file, fixture = Path(result_file).resolve(), Path(fixture).resolve()
    result = json.loads(result_file.read_text())
    if (result.get('condition') != 'native_static_yaw_gate0'
            or any(result.get(k) is not True for k in ('passed', 'both_outputs_zero_after_stop',
                'both_grasps_retained_after_stop', 'leader_input_boundary_passed', 'follower_input_boundary_passed'))
            or result.get('grasp_loss_events') != []
            or result.get('autonomous_yaw_qualification_issued') is not False
            or result.get('native_gate0', {}).get('gate0_static_passed') is not True):
        raise ValueError('completed native static Gate-0 and stop/hold evidence required')
    static = result['native_gate0']
    if (not math.isfinite(static['yaw_std_rad']) or static['yaw_std_rad'] < 0.
            or static['samples'] < 8 or not static['measured_duration_s'] > 0.):
        raise ValueError('native Gate-0 measured noise/sample evidence invalid')
    effective_file = result_file.parent/'static_effective_parameters.json'
    effective = json.loads(effective_file.read_text())
    if (effective != result.get('effective_audit_parameters') or effective.get('yaw_heading_frame') != 'grasp_heading'
            or effective.get('wrench_force_limit_N') != static.get('force_limit_N')
            or effective.get('wrench_torque_limit_Nm') != static.get('torque_limit_Nm')):
        raise ValueError('native Gate-0 effective pose/load parameters missing or inconsistent')
    numeric = finite([effective[k] for k in ('rotation_error_limit_rad', 'joint_margin_rad',
        'wrench_force_limit_N', 'wrench_torque_limit_Nm')], (4,))
    if np.min(numeric) <= 0.:
        raise ValueError('native Gate-0 actual pose/load parameters must be positive')
    for robot in ('leader', 'follower'):
        state = result.get('states', {}).get(robot, {})
        if state.get('state') != 'SAFE_STOP' or state.get('accepted_goals') != 0 or state.get('completed_goals') != 0:
            raise ValueError('native Gate-0 must have zero goals and both SAFE_STOP')
    argv = result.get('argv', [])
    if '--static-yaw-audit' not in argv or '--run-directory' not in argv:
        raise ValueError('native static source fixture is missing')
    source = Path(argv[argv.index('--run-directory')+1]).resolve()
    required = {'world.sdf', *[r+s for r in ('leader', 'follower')
        for s in ('.urdf', '_controllers.yaml', '_base_controller.yaml')]}
    if result.get('fixture_unchanged') is not True or set(result.get('fixture_sha256', {})) != required:
        raise ValueError('native Gate-0 recorded fixture hashes required')
    for name, digest in result['fixture_sha256'].items():
        saved = result_file.parent/'fixture_snapshot'/name
        if (not saved.exists() or hashlib.sha256(saved.read_bytes()).hexdigest() != digest
                or hashlib.sha256((source/name).read_bytes()).hexdigest() != digest):
            raise ValueError('native Gate-0 recorded fixture changed after measurement')
    hashes = {}
    for robot in ('leader', 'follower'):
        original, current = source/(robot+'.urdf'), fixture/(robot+'.urdf')
        if canonical_model(original) != canonical_model(current):
            raise ValueError('native Gate-0 model differs beyond controller path')
        for suffix in ('_controllers.yaml', '_base_controller.yaml'):
            if (source/(robot+suffix)).read_bytes() != (fixture/(robot+suffix)).read_bytes():
                raise ValueError('native Gate-0 controller configuration changed')
        hashes[robot] = hashlib.sha256(original.read_bytes()).hexdigest()
    import xml.etree.ElementTree as ET
    if ET.tostring(ET.parse(source/'world.sdf').getroot()) != ET.tostring(ET.parse(fixture/'world.sdf').getroot()):
        raise ValueError('native Gate-0 world/contact policy changed')
    manifest_file = result_file.parent/'source_sha256.json'
    manifest = json.loads(manifest_file.read_text())
    if not manifest:
        raise ValueError('native Gate-0 nonempty source snapshot required')
    snapshot = result_file.parent/'source_snapshot'
    workspace = Path(__file__).resolve().parents[1]
    for name, digest in manifest.items():
        try:
            saved = snapshot/Path(name).relative_to(workspace)
        except ValueError as exc:
            raise ValueError('Gate-0 source path outside recorded workspace') from exc
        if not saved.exists() or hashlib.sha256(saved.read_bytes()).hexdigest() != digest:
            raise ValueError('native Gate-0 source snapshot missing or changed')
    return dict(result_file=str(result_file), result_sha256=hashlib.sha256(result_file.read_bytes()).hexdigest(),
                source_fixture=str(source), source_models_sha256=hashes,
                model_world_controllers_identical=True, static_yaw_std_rad=result['native_gate0']['yaw_std_rad'],
                effective_parameters=effective)


def configure_passive(leader_file, follower_file, *, delta_deg, duration_s, leader_limits_file,
                      xy_allocation=False, allocation_policy_file=None):
    if not np.isfinite([delta_deg, duration_s]).all() or not 0. < delta_deg <= .005 or duration_s < 75.:
        raise ValueError('initial native passive envelope is at most 0.005 degrees and at least 75 s per goal')
    follower = yaml.safe_load(Path(follower_file).read_text())
    if set(follower) != {'/follower/follower_yaw_whole_body'}:
        raise ValueError('one existing follower endpoint required')
    params = follower['/follower/follower_yaw_whole_body']['ros__parameters']
    params.update(base_gains=[0., 0., 0.], base_force_acceleration_gains=[0., 0., 0.],
        base_force_velocity_gains=[0., 0., 0.], passive_yaw_calibration=True,
        rotation_observation_required=False, arm_yaw_admittance_enabled=True,
        rotational_relief_enabled=False)
    params['passive_xy_allocation_enabled']=bool(xy_allocation)
    if xy_allocation:
        if allocation_policy_file is None:raise ValueError('explicit existing allocation policy required for passive XY')
        allocation=yaml.safe_load(Path(allocation_policy_file).read_text())['/follower/follower_yaw_whole_body']['ros__parameters']
        if (set(allocation)!={'allocation_base_regularization','allocation_task_compliance_budget'}
                or allocation['allocation_task_compliance_budget'] is not True
                or not math.isfinite(allocation['allocation_base_regularization']) or allocation['allocation_base_regularization']<=0.):
            raise ValueError('bounded existing allocation policy required')
        params.update(allocation)
        limits=finite(params['base_velocity_limits'],(3,)).tolist()
        limits[:2]=[min(v,.0001) for v in limits[:2]]  # Existing passive measured-stationarity envelope.
        params['base_velocity_limits']=limits
    # All integrated modes retain the existing whole_body endpoint name;
    # XY_ONLY selects the arm task internally, without another controller.
    Path(follower_file).write_text(yaml.safe_dump(follower))
    data = yaml.safe_load(Path(leader_file).read_text())
    leader = data['/leader/leader_object_rotation']['ros__parameters']
    limits = next(iter(yaml.safe_load(Path(leader_limits_file).read_text()).values()))['ros__parameters']
    keys = ('rotation_max_angular_speed', 'rotation_max_angular_acceleration',
            'rotation_max_base_speed', 'rotation_max_base_acceleration', 'rotation_max_grasp_speed')
    for key in keys:
        values = finite([leader[key], limits[key]], (2,))
        if np.min(values) <= 0.:
            raise ValueError('existing explicit positive leader limits required')
        leader[key] = float(min(values))  # Planning reduction; no native load certificate transfer.
    leader.update(rotation_duration_s=duration_s, rotation_max_yaw_rad=math.radians(2.*delta_deg),
                  actuator_ack_required=True)
    Path(leader_file).write_text(yaml.safe_dump(data))
    return dict(rotation_follow_mode='XY_ONLY', passive_yaw_calibration=True,
        passive_xy_allocation_enabled=bool(xy_allocation),base_yaw_command_locked=True,
        rotational_relief_enabled=False, arm_policy_revision='native_xy_yaw_orientation_hold_v1',
        autonomous_follower_yaw_enabled=False, base_gains=params['base_gains'],
        leader_limits_sha256=hashlib.sha256(Path(leader_limits_file).read_bytes()).hexdigest(),
        leader_planning_caps={key: leader[key] for key in keys},
        delta_deg=delta_deg, duration_s=duration_s,
        scope=('new native yaw-locked XY-allocation passive experiment' if xy_allocation else
            'new native stationary-base passive experiment; fixed-grasp qualification is not transferred'))


def read_passive_endpoint(node, wait):
    """Verify the live node, not only the file we intended to load."""
    from rcl_interfaces.srv import GetParameters
    from rcl_interfaces.msg import ParameterType
    client = node.create_client(GetParameters, '/follower/follower_yaw_whole_body/get_parameters')
    try:
        wait(client.service_is_ready, 10.)
        names = ['rotation_follow_mode', 'passive_yaw_calibration', 'arm_yaw_admittance_enabled',
                 'rotation_observation_required', 'base_gains', 'base_force_acceleration_gains', 'base_force_velocity_gains',
                 'passive_xy_allocation_enabled','base_velocity_limits','allocation_task_compliance_budget']
        request = GetParameters.Request(names=names)
        future = client.call_async(request); wait(future.done, 10.)
        values = future.result().values
        expected = [ParameterType.PARAMETER_STRING]+[ParameterType.PARAMETER_BOOL]*3+[ParameterType.PARAMETER_DOUBLE_ARRAY]*3+[
            ParameterType.PARAMETER_BOOL,ParameterType.PARAMETER_DOUBLE_ARRAY,ParameterType.PARAMETER_BOOL]
        if len(values) != len(names) or any(v.type != typ for v, typ in zip(values, expected)):
            raise ValueError('live passive endpoint mode/gain parameters unavailable')
        result = dict(zip(names, [values[0].string_value, *[v.bool_value for v in values[1:4]],
                                  *[list(v.double_array_value) for v in values[4:7]],values[7].bool_value,
                                  list(values[8].double_array_value),values[9].bool_value]))
        if (result['rotation_follow_mode'] != 'XY_ONLY' or result['passive_yaw_calibration'] is not True
                or result['arm_yaw_admittance_enabled'] is not True or result['rotation_observation_required'] is not False
                or any(result[k] != [0., 0., 0.] for k in names[4:7])):
            raise ValueError('live passive follower must have XY_ONLY, arm compliance and zero base gains')
        if result['passive_xy_allocation_enabled'] and (max(result['base_velocity_limits'][:2])>.0001
                or result['allocation_task_compliance_budget'] is not True):
            raise ValueError('live passive XY allocation exceeds original stationary-speed envelope or lacks budget')
        return result
    finally:
        node.destroy_client(client)


class NativePassiveYaw:
    def __init__(self, geometry, *, force_limit_N, torque_limit_Nm, allow_base_xy=False):
        self.geometry = geometry
        self.static = NativeStaticYaw(geometry, force_limit_N=force_limit_N, torque_limit_Nm=torque_limit_Nm)
        self.force_limit, self.torque_limit = float(force_limit_N), float(torque_limit_Nm)
        self.records, self.previous, self.last_phase = [], None, 'static'
        self.static_report = None
        self.maximum_force = self.maximum_moment = 0.
        self.allow_base_xy=bool(allow_base_xy)

    def add(self, row):
        if any(k in row for k in ('leader_pose', 'leader_velocity', 'goal_angle_rad', 'object_center', 'object_pose_GT')):
            raise ValueError('passive qualification accepts own measurements only')
        phase = row['phase']
        phases = ('static', 'positive', 'negative')
        if phase not in phases or phases.index(phase) < phases.index(self.last_phase):
            raise ValueError('ordered static/positive/negative native phases required')
        q = finite(row['q_rad'], (5,)); qd = finite(row['qdot_rad_s'], (5,))
        w = finite(row['wrench_tcp'], (6,)); v = finite(row['base_twist'], (3,))
        command = finite(row['base_command'], (3,))
        t, now = float(row['joint_stamp']), float(row['timestamp'])
        stamps = finite([t, row['wrench_stamp'], row['odom_stamp']], (3,))
        if not math.isfinite(now) or np.any(now-stamps > .05+1e-12) or np.any(now-stamps < -.02-1e-12) or np.ptp(stamps) > .02+1e-12:
            raise ValueError('native passive own sources stale/future/skewed')
        if row['grasp_attached'] is not True:
            raise ValueError('native own grasp not retained')
        allowed_command=(abs(command[2])<=1e-12 and np.max(abs(command[:2]))<=.0001) if self.allow_base_xy else np.max(abs(command))<=1e-12
        if np.max(abs(v)) >= .0001 or not allowed_command:
            raise ValueError('passive follower base must be stationary with zero command')
        if np.any(abs(qd) > self.geometry.model.velocity):
            raise ValueError('native joint velocity exceeds actual URDF')
        measured = self.geometry.measure(q, compute_jacobian=False)
        if not measured['geometry_valid']:
            raise ValueError(measured['valid_reason'])
        if phase != 'static' and self.static_report is None:
            self.static_report = self.static.finish()
        if phase == 'static' and self.static.baseline is None:
            self.static.add(row)
        if self.static.baseline is None:
            raise ValueError('measured loaded static baseline required before passive rotation')
        force = float(np.linalg.norm((w-self.static.baseline)[:3]))
        moment = float(np.linalg.norm((w-self.static.baseline)[3:]))
        self.maximum_force, self.maximum_moment = max(self.maximum_force, force), max(self.maximum_moment, moment)
        # EVERY raw FT event is guarded, even if its FK sample was already seen.
        if force >= self.force_limit or moment >= self.torque_limit:
            raise ValueError(f'native passive incremental wrench exceeded: {force:.9g} N, {moment:.9g} Nm')
        if phase == 'static':
            self.static.add(row)
        if self.previous is not None:
            old_t, old_q = self.previous
            if t == old_t:
                if not np.array_equal(q, old_q):
                    raise ValueError('native passive duplicate-stamp FK discontinuity')
                return
            if not 0. < t-old_t <= .05+1e-12:
                raise ValueError('native passive source gap/discontinuity')
            if np.any(abs(q-old_q) > self.geometry.model.velocity*(t-old_t)+1e-12):
                raise ValueError('native passive joint displacement exceeds URDF velocity bound')
        self.previous, self.last_phase = (t, q.copy()), phase
        self.records.append(dict(phase=phase, timestamp=t, receive_timestamp=now,
            q_rad=q.tolist(), qdot_rad_s=qd.tolist(), wrench_tcp=w.tolist(), base_twist=v.tolist(),
            grasp_attached=True, psi_ee_B=measured['psi_ee_B'], projection_norm=measured['projection_norm'],
            yaw_heading_frame=self.geometry.heading_frame, geometry_valid=True))

    def finish(self, evidence_sha256):
        certificate = qualify_passive(self.records, self.geometry, force_limit_N=self.force_limit,
            torque_limit_Nm=self.torque_limit, evidence_sha256=evidence_sha256)
        certificate.update(native_contact_only=True,
            calibration_scope='native yaw-locked XY-allocation passive yaw' if self.allow_base_xy else 'native stationary follower base passive yaw',
            calibration_base_policy=dict(xy_allocation=self.allow_base_xy,yaw_command_rad_s=0.,
                xy_command_limit_m_s=.0001 if self.allow_base_xy else 0.,actual_stationarity_envelope=.0001))
        return certificate


def execute_passive(args, *, node, service, settle, wait, phase, goal, states,
                    auditor, rows_stream, ready, source_hashes):
    """Only the leader receives two bounded relative calibration goals."""
    from std_msgs.msg import Float64
    # Include the enable transient in measured static evidence and fault reporting.
    phase[0] = 'native_passive_static'
    service('/follower/admittance/enable')
    settle(2.)
    wait(lambda: ready.get('seen') is True, 10., True)
    if ready.get('ever_true'):
        raise ValueError('passive calibration cannot publish autonomous rotation readiness')
    for robot, owner in (('leader', 'leader_object_rotation'), ('follower', 'follower_yaw_whole_body')):
        for suffix in ('base_cmd', 'arm_joint_cmd'):
            writers = node.get_publishers_info_by_topic('/'+robot+'/'+suffix)
            if len(writers) != 1 or writers[0].node_name != owner or writers[0].node_namespace != '/'+robot:
                raise ValueError('single passive arm/base command writer not verified')
    phase[0] = 'native_passive_static'
    settle(args.static_observe_s)
    auditor.static.finish()  # Refuse rotation without fresh loaded static evidence.
    if any(hashlib.sha256(Path(file).read_bytes()).hexdigest() != digest for file, digest in source_hashes.items()):
        raise ValueError('native passive source changed before first goal')
    for name, delta, completed in (('positive', args.delta_deg, 1), ('negative', -2.*args.delta_deg, 2)):
        if ready.get('ever_true'):
            raise ValueError('passive follower incorrectly authorized autonomous rotation')
        phase[0] = 'native_passive_'+name
        goal.publish(Float64(data=math.radians(delta)))
        wait(lambda: states.get('leader', {}).get('accepted_goals') == completed, 10., True)
        wait(lambda: states.get('leader', {}).get('completed_goals') == completed,
             max(1000., args.duration_s*5.), True)
        settle(3.)
    if ready.get('ever_true'):
        raise ValueError('passive follower readiness changed during calibration')
    rows_stream.flush()
    evidence = args.output/'native_passive_local.jsonl'
    return auditor.finish({str(evidence): hashlib.sha256(evidence.read_bytes()).hexdigest()})


def export_passive_csv(source, destination, geometry=None):
    """Keep all raw events, phase labels, base commands, and terminal faults."""
    import csv
    keys = ['phase', 'timestamp', 'joint_stamp', 'wrench_stamp', 'odom_stamp', 'grasp_attached',
            'psi_ee_B', 'projection_norm', 'geometry_valid', 'fault']
    vectors = {'q_rad': [f'q{i}_rad' for i in range(1, 6)],
        'qdot_rad_s': [f'qdot{i}_rad_s' for i in range(1, 6)],
        'wrench_tcp': ['fx_N', 'fy_N', 'fz_N', 'tx_Nm', 'ty_Nm', 'tz_Nm'],
        'base_twist': ['vx_m_s', 'vy_m_s', 'wz_rad_s'],
        'base_command': ['cmd_vx_m_s', 'cmd_vy_m_s', 'cmd_wz_rad_s']}
    count = 0
    with source.open() as raw, destination.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=keys+[k for names in vectors.values() for k in names])
        writer.writeheader()
        for line in raw:
            row = json.loads(line); value = {k: row.get(k) for k in keys}
            for key, names in vectors.items():
                value.update(zip(names, row.get(key, [])))
            if geometry is not None and 'q_rad' in row:
                try:
                    measured = geometry.measure(row['q_rad'], compute_jacobian=False)
                    value.update({k: measured[k] for k in ('psi_ee_B', 'projection_norm', 'geometry_valid')})
                except (ValueError, KeyError, TypeError) as exc:
                    value['fault'] = value.get('fault') or str(exc)
            writer.writerow(value); count += 1
    return count

"""Native O2: bounded own-base compensation, no leader/object rotation goal."""
from collections import OrderedDict
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import time

import numpy as np
import yaml

from amir_compliance_control.rotation_observer import ProjectedYaw, read_qualification
from amir_compliance_control.wrench import finite
from amir_rotation_integration_audit import canonical_model


def configure_o2(result_file, fixture, output, follower_file):
    result_file, fixture, output = map(Path, (result_file, fixture, output))
    result = json.loads(result_file.read_text())
    xy_calibration=result.get('condition')=='native_yaw_locked_xy_allocation_passive_yaw'
    if (result.get('condition') not in ('native_stationary_follower_base_passive_yaw','native_yaw_locked_xy_allocation_passive_yaw')
            or any(result.get(k) is not True for k in ('passed', 'passive_yaw_qualification_issued',
                'positive_negative_programs_completed', 'both_outputs_zero_after_stop', 'both_grasps_retained_after_stop',
                'leader_input_boundary_passed', 'follower_input_boundary_passed', 'fixture_unchanged'))
            or result.get('autonomous_follower_yaw_enabled') is not False
            or result.get('grasp_loss_events') != []):
        raise ValueError('completed native passive yaw qualification and stop/hold evidence required')
    if xy_calibration and (result.get('passive_policy',{}).get('passive_xy_allocation_enabled') is not True
            or result.get('passive_live_parameters',{}).get('passive_xy_allocation_enabled') is not True):
        raise ValueError('yaw-locked XY calibration method must match live measured policy')
    for robot in ('leader', 'follower'):
        state = result.get('states', {}).get(robot, {})
        if state.get('state') != 'SAFE_STOP' or state.get('completed_goals') != (2 if robot == 'leader' else 0):
            raise ValueError('native passive predecessor must be stopped after exactly two leader programs')
    argv = result['argv']; source = Path(argv[argv.index('--run-directory')+1])
    required = {'world.sdf', *[r+s for r in ('leader', 'follower') for s in ('.urdf', '_controllers.yaml', '_base_controller.yaml')]}
    if set(result.get('fixture_sha256', {})) != required:
        raise ValueError('native passive measured fixture hashes required')
    for name, digest in result['fixture_sha256'].items():
        if hashlib.sha256((source/name).read_bytes()).hexdigest() != digest:
            raise ValueError('native passive source model changed after measurement')
        snapshot = result_file.parent/'fixture_snapshot'/name
        if not snapshot.exists() or hashlib.sha256(snapshot.read_bytes()).hexdigest() != digest:
            raise ValueError('native passive measured fixture snapshot missing or changed')
    for robot in ('leader', 'follower'):
        if canonical_model(source/(robot+'.urdf')) != canonical_model(fixture/(robot+'.urdf')):
            raise ValueError('native O2 model changed beyond generated controller path')
        for suffix in ('_controllers.yaml', '_base_controller.yaml'):
            if (source/(robot+suffix)).read_bytes() != (fixture/(robot+suffix)).read_bytes():
                raise ValueError('native O2 controller changed')
    import xml.etree.ElementTree as ET
    if ET.tostring(ET.parse(source/'world.sdf').getroot()) != ET.tostring(ET.parse(fixture/'world.sdf').getroot()):
        raise ValueError('native O2 world/contact model changed')
    effective = result['effective_audit_parameters']
    geometry = ProjectedYaw((source/'follower.urdf').read_text(),
        orientation_envelope_rad=effective['rotation_error_limit_rad'], joint_margin_rad=effective['joint_margin_rad'],
        heading_frame=effective['yaw_heading_frame'])
    qfile = result_file.parent/'qualification.json'
    certificate = read_qualification(qfile, hashlib.sha256((source/'follower.urdf').read_bytes()).hexdigest(), geometry)
    if certificate.get('native_contact_only') is not True:
        raise ValueError('fixed-grasp qualification cannot authorize native O2')
    base_policy=certificate.get('calibration_base_policy',{})
    if bool(base_policy.get('xy_allocation',False))!=xy_calibration:
        raise ValueError('native passive result and certificate calibration-base condition differ')
    if xy_calibration and (base_policy.get('yaw_command_rad_s')!=0.
            or not 0.<base_policy.get('xy_command_limit_m_s',0.)<=.0001):
        raise ValueError('native XY calibration must retain zero yaw and the original measured-speed envelope')
    from amir_rotation_native_arm_policy import validate
    arm_policy=validate(result.get('native_arm_policy'))
    if certificate.get('native_arm_policy')!=arm_policy:
        raise ValueError('native certificate and measured arm mechanics differ')
    for file, digest in certificate['evidence_sha256'].items():
        if hashlib.sha256(Path(file).read_bytes()).hexdigest() != digest:
            raise ValueError('native measured passive evidence changed')
    certificate.update(urdf_sha256=hashlib.sha256((fixture/'follower.urdf').read_bytes()).hexdigest(),
        native_transfer_proof='identical native canonical URDF, world, controllers; only generated controller path differs',
        native_passive_result_sha256=hashlib.sha256(result_file.read_bytes()).hexdigest())
    target = output/'qualification.json'; target.write_text(json.dumps(certificate, indent=2)+'\n')
    angle = .5*min(certificate['measured_signal_rad']['positive'], -certificate['measured_signal_rad']['negative'])
    duration = float(result['passive_policy']['duration_s'])
    if not np.isfinite([angle, duration]).all() or angle <= 0. or duration <= 0.:
        raise ValueError('measured positive/negative native yaw signal and actual duration required')
    policy = yaml.safe_load(Path(follower_file).read_text())
    params = policy['/follower/follower_yaw_whole_body']['ros__parameters']
    from amir_rotation_native_arm_policy import transfer
    transfer(result,certificate,params)
    velocity = list(params['base_velocity_limits']); velocity[2] = min(velocity[2], .5*1.875*angle/duration)
    params.update(base_gains=[0., 0., 0.], base_force_acceleration_gains=[0., 0., 0.],
        base_force_velocity_gains=[0., 0., 0.], passive_yaw_calibration=False,
        rotation_observation_required=True, arm_yaw_admittance_enabled=True,
        probe_angle_rad=angle, probe_duration_s=duration, probe_hold_s=2.,
        probe_qualification_file=str(target.resolve()), base_velocity_limits=velocity)
    Path(follower_file).write_text(yaml.safe_dump(policy))
    observer = output/'observer_policy.yaml'
    observer.write_text(yaml.safe_dump({'/follower/rotation_observer': {'ros__parameters': dict(
        orientation_envelope_rad=effective['rotation_error_limit_rad'], joint_margin_rad=effective['joint_margin_rad'],
        qualification_file=str(target.resolve()), allow_loaded_reference_latch=True,
        align_own_source_history=True, publish_on_raw_measurement=True)}}))
    return dict(native_passive_result=str(result_file.resolve()), native_passive_result_sha256=hashlib.sha256(result_file.read_bytes()).hexdigest(),
        native_contact_only=True, effective_parameters=effective, probe_angle_rad=angle,
        native_arm_policy=arm_policy,
        probe_duration_s=duration, probe_hold_s=2., reduced_yaw_speed_cap_rad_s=velocity[2],
        observer_policy_file=str(observer.resolve()), qualification_file=str(target.resolve()),
        autonomous_follower_yaw_enabled=False)


class NativeO2Recorder:
    def __init__(self, output, phase, *, force_limit_N, torque_limit_Nm, fault_injection=False,
                 filename_prefix='native_o2',csv_fields=None):
        self.phase, self.fault_injection = phase, fault_injection
        self.filename_prefix=filename_prefix
        self.injected = False; self.fault = ''; self.latest = {}; self.debug = {}
        self.observations = OrderedDict(); self.count = self.moving = self.limited = self.missing = 0
        self.maximum_residual = 0.; self.maximum_age_ns = 0
        self.force_limit, self.torque_limit = map(float, (force_limit_N, torque_limit_Nm))
        if not np.isfinite([self.force_limit,self.torque_limit]).all() or min(self.force_limit,self.torque_limit)<=0.:
            raise ValueError('actual positive native load caps required')
        self.raw_baseline=None;self.maximum_force=self.maximum_moment=0.;self.raw_events=0
        self.raw_ft=(output/(filename_prefix+'_raw_ft.jsonl')).open('w',buffering=1)
        self.raw = (output/(filename_prefix+'_local.jsonl')).open('w', buffering=1)
        self.csv_stream = (output/(filename_prefix+'_local.csv')).open('w', newline='')
        self.writer = csv.DictWriter(self.csv_stream, fieldnames=csv_fields or ['sim_time_s', 'phase', 'probe_phase',
            'requested_wz_rad_s', 'vx_m_s', 'vy_m_s', 'wz_rad_s', 'point_residual_m_s',
            'consumed_sequence', 'consumed_snapshot_matched', 'source_age_ns', 'yaw_valid', 'fault'])
        self.writer.writeheader()

    def active(self):
        return self.phase[0].startswith(self.filename_prefix+'_')

    def wrench(self, row):
        if not self.active():return
        self.raw_events+=1
        try:
            w=finite(row['wrench_tcp'],(6,))
            times=finite([row['joint_stamp'],row['wrench_stamp'],row['odom_stamp']],(3,))
            now=float(row['timestamp'])
            if not math.isfinite(now) or np.any(now-times>.05+1e-12) or np.any(now-times<-.02-1e-12) or np.ptp(times)>.02+1e-12:
                raise ValueError('native O2 raw own sources stale/future/skewed')
            if row['grasp_attached'] is not True:raise ValueError('native O2 own grasp lost')
            if self.raw_baseline is None:self.raw_baseline=w.copy()
            force=float(np.linalg.norm((w-self.raw_baseline)[:3]));moment=float(np.linalg.norm((w-self.raw_baseline)[3:]))
            self.maximum_force=max(self.maximum_force,force);self.maximum_moment=max(self.maximum_moment,moment)
            if force>=self.force_limit or moment>=self.torque_limit:
                raise ValueError(f'native O2 raw incremental wrench exceeded: {force:.9g} N, {moment:.9g} Nm')
        except (ValueError,KeyError,TypeError) as exc:
            self.fault=self.fault or str(exc);row={**row,'fault':str(exc)}
        self.raw_ft.write(json.dumps(row)+'\n')

    def observe(self, value):
        self.latest = value
        sequence = value.get('publication_sequence')
        if sequence is not None:
            self.observations[sequence] = value
            while len(self.observations) > 256:
                self.observations.popitem(last=False)
        if self.active() and value.get('yaw_valid') is not True and not (self.fault_injection and self.injected):
            self.fault = self.fault or 'native O2 qualified yaw invalid: '+str(value.get('valid_reason'))

    def record(self, value):
        self.debug = value
        if not self.active():
            return
        consumed = value.get('consumed_observation', {})
        sequence = consumed.get('publication_sequence'); observation = self.observations.get(sequence)
        self.count += 1
        if observation is None:
            self.missing += 1
        residual = None
        command = value.get('probe_limited_twist_footprint')
        try:
            if command is not None:
                u = finite(command, (3,)); p = finite(value['probe_position_footprint_m'], (3,))
                vector = np.r_[u[:2], 0.]+np.cross([0., 0., u[2]], p)
                residual = float(np.linalg.norm(vector)); self.maximum_residual = max(self.maximum_residual, residual)
                scale = max(np.linalg.norm(u[:2])+abs(u[2])*np.linalg.norm(p), np.finfo(float).tiny)
                if residual > 64*np.finfo(float).eps*scale:
                    raise ValueError('native O2 compensation does not use the limited yaw command')
                if value.get('state')!='SAFE_STOP':
                    actual=finite(value['base_twist'],(3,))
                    if np.linalg.norm(actual-u)>64*np.finfo(float).eps*max(np.linalg.norm(u),np.finfo(float).tiny):
                        raise ValueError('native O2 actual allocated base command differs from the compensated command')
                if abs(u[2]) > 0.:
                    self.moving += 1
                    if abs(value['probe_requested_wz_rad_s']) > abs(u[2])+np.finfo(float).eps*abs(u[2]):
                        self.limited += 1
            age = consumed.get('source_age_ns')
            if age is not None:
                self.maximum_age_ns = max(self.maximum_age_ns, age)
            if value.get('state') == 'SAFE_STOP' and not (self.fault_injection and self.injected):
                raise ValueError('native O2 controller stopped: '+str(value.get('reason')))
        except (ValueError, KeyError, TypeError) as exc:
            self.fault = self.fault or str(exc)
        self.raw.write(json.dumps(dict(phase=self.phase[0], debug=value, consumed_observation=observation,
            exact_consumed_snapshot_match=observation is not None, fault=self.fault))+'\n')
        u = command if isinstance(command,(list,tuple)) and len(command)==3 else [None]*3
        self.writer.writerow(dict(sim_time_s=value.get('sim_time'), phase=self.phase[0],
            probe_phase=value.get('probe_phase'), requested_wz_rad_s=value.get('probe_requested_wz_rad_s'),
            vx_m_s=u[0], vy_m_s=u[1], wz_rad_s=u[2], point_residual_m_s=residual,
            consumed_sequence=sequence, consumed_snapshot_matched=observation is not None,
            source_age_ns=consumed.get('source_age_ns'), yaw_valid=None if observation is None else observation.get('yaw_valid'),
            fault=self.fault))

    def close(self):
        self.raw.close(); self.csv_stream.close();self.raw_ft.close()

    def summary(self):
        return dict(local_samples=self.count, moving_samples=self.moving, limited_samples=self.limited,
            missing_consumed_snapshot_rows=self.missing, maximum_compensated_point_velocity_residual_m_s=self.maximum_residual,
            maximum_consumed_source_age_ns=self.maximum_age_ns, local_fault=self.fault,raw_ft_events=self.raw_events,
            maximum_incremental_force_N=self.maximum_force,maximum_incremental_moment_Nm=self.maximum_moment)


def owned_observer_pids(partition, domain):
    if not partition.startswith('amir_cp9_cp8r3_native_o2_') or not domain:
        raise ValueError('exact isolated native O2 owner required')
    matches = []
    for folder in Path('/proc').iterdir():
        if not folder.name.isdigit():
            continue
        try:
            env = dict(v.split(b'=', 1) for v in (folder/'environ').read_bytes().split(b'\0') if b'=' in v)
            argv = (folder/'cmdline').read_bytes().split(b'\0')
            if (env.get(b'IGN_PARTITION') == partition.encode() and env.get(b'ROS_DOMAIN_ID') == domain.encode()
                    and any(v.endswith(b'/rotation_observer') for v in argv)):
                matches.append(int(folder.name))
        except OSError:
            pass
    return matches


def execute_o2(args, *, service, settle, wait, phase, states, outputs, recorder, ready, source_hashes):
    paused = []; report = dict(leader_object_goal_count=0, autonomous_follower_yaw_enabled=False)
    try:
        previous = recorder.latest.get('reference_sequence', 0)
        service('/follower/rotation/latch_reference')
        wait(lambda: recorder.latest.get('reference_sequence', 0) > previous
             and recorder.latest.get('yaw_valid') is True, 10., True)
        service('/follower/admittance/enable')
        wait(lambda: ready.get('seen') is True, 10., True)
        if ready.get('ever_true'):
            raise ValueError('native O2 cannot authorize leader rotation')
        phase[0] = 'native_o2_baseline'; settle(3.)
        if any(hashlib.sha256(Path(file).read_bytes()).hexdigest() != digest for file, digest in source_hashes.items()):
            raise ValueError('native O2 source changed before probe')
        service('/follower/rotation/start_compensation_probe'); phase[0] = 'native_o2_probe'
        if args.o2_observer_stop:
            wait(lambda: recorder.debug.get('probe_phase') == 'positive' and recorder.limited > 0
                 and abs(outputs.get('follower', [0.,0.,0.])[2]) > 0., 60., True)
            paused = owned_observer_pids(os.environ['IGN_PARTITION'], os.environ['ROS_DOMAIN_ID'])
            if len(paused) != 1:
                raise ValueError('exactly one owned native observer required for source-stop injection')
            recorder.injected = True
            began = time.monotonic(); os.kill(paused[0], signal.SIGSTOP)
            wait(lambda: all(states.get(r, {}).get('state') == 'SAFE_STOP' for r in ('leader', 'follower')), 10., False)
            reason = str(states.get('follower', {}).get('reason', '')).lower()
            if not ('yaw' in reason or 'observer' in reason) or not any(s in reason for s in ('age', 'watchdog', 'stale', 'time')):
                raise ValueError('native source-stop ended without the expected yaw freshness fault: '+reason)
            report.update(observer_stop_detected=True, observer_stop_pid=paused[0],
                fault_detection_wall_s=time.monotonic()-began, detected_reason=reason)
        else:
            wait(lambda: recorder.debug.get('probe_phase') == 'complete', 1000., True)
            phase[0] = 'native_o2_hold'; settle(3.)
            report['positive_negative_pulse_completed'] = True
        if ready.get('ever_true') or states.get('leader', {}).get('accepted_goals') != 0:
            raise ValueError('native O2 unexpectedly authorized/received a leader rotation goal')
        if recorder.fault or not recorder.moving or not recorder.limited or recorder.raw_events<8:
            raise ValueError(recorder.fault or 'native O2 lacks moving and yaw-limited samples')
        report.update(recorder.summary()); return report
    finally:
        for pid in paused:
            try:
                os.kill(pid, signal.SIGCONT)
            except ProcessLookupError:
                pass

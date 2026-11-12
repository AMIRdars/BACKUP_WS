#!/usr/bin/env python3
"""CP-8R-3 separate native-friction grasp-priority rotation trial: leader-only goal, no shared centre.

Ground truth belongs to this evaluator only. Control nodes are separate
processes. Only the leader receives its constant, verified fixture calibration.
"""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from geometry_msgs.msg import Twist, WrenchStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import DeleteEntity
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty, Float64, String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
import yaml

from amir_compliance_control.object_center_rotation_trajectory import inverse_transform
from amir_compliance_control.admittance import rotation_error
from amir_compliance_control.wrench import LocalModel, rotation
from amir_cp8r2_trial import prepare_control_policies as cp8r2_policies
from amir_cp8r3_clearance import BaseClearance


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1.-2*(q.y*q.y+q.z*q.z))


def pose_matrix(position, quaternion):
    q = np.array([quaternion.x, quaternion.y, quaternion.z, quaternion.w], dtype=float)
    if not np.isfinite(q).all() or abs(np.linalg.norm(q)-1.) > 1e-5:
        raise ValueError('invalid evaluator quaternion')
    x, y, z, w = q/np.linalg.norm(q)
    t = np.eye(4)
    t[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                  [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                  [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    t[:3, 3] = [position.x, position.y, position.z]
    return t


def prepare_control_policies(folder, output, duration, force_gain, force_velocity_gain, yaw_gain, xy_gain, yaw_metric, sensor_certificate):
    root = Path(__file__).resolve().parents[1]/'audits/object_center_rotation_20261103'
    follower = yaml.safe_load((root/'cp8r2/plus5_01/follower_policy.yaml').read_text())
    f = follower['/follower/follower_yaw_arm_base']['ros__parameters']
    f.update(base_gains=[xy_gain, xy_gain, yaw_gain], arm_task_metric=[1.,1.,1.,.1,.1,yaw_metric],
        base_force_acceleration_gains=[force_gain,force_gain,0.],base_force_velocity_gains=[force_velocity_gain,force_velocity_gain,0.],
        virtual_yaw_inertia=20.*yaw_metric**2, virtual_yaw_damping=1200.*yaw_metric**2, virtual_yaw_stiffness=400.*yaw_metric**2)
    cert = json.loads(Path(sensor_certificate).read_text())
    if not cert['sensor_calibration_passed'] or cert['urdf_sha256'] != hashlib.sha256((folder/'follower.urdf').read_bytes()).hexdigest():
        raise ValueError('fresh measured native sensor certificate required')
    probes=np.array([t['command'] for t in cert['tests']]);fn=np.linalg.norm(probes[:,:3],axis=1);tn=np.linalg.norm(probes[:,3:],axis=1)
    f.update(wrench_force_limit_N=float(min(fn[fn>0.])),wrench_torque_limit_Nm=float(min(tn[tn>0.])),native_grasp_monitor_required=True,rotational_relief_enabled=True,
        relief_inertia=[.2,.2],relief_damping=[12.,12.],relief_stiffness=[4.,4.],
        relief_displacement_rad=[.001,.001],relief_velocity_rad_s=[.002,.002],relief_acceleration_rad_s2=[.02,.02])
    follower_file = output/'follower_policy.yaml';follower_file.write_text(yaml.safe_dump(follower))
    leader = yaml.safe_load((root/'cp8r1/leader_minus5_01/rotation_policy.yaml').read_text())
    leader['/leader/leader_object_rotation']['ros__parameters'].update(rotation_duration_s=duration,
        rotation_ungrasped_test=False,grasp_calibration_file=str(output/'leader_native_grasp_calibration.json'))
    leader_file = output/'rotation_policy.yaml';leader_file.write_text(yaml.safe_dump(leader))
    return leader_file, follower_file


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run-directory', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--native-metrology-directory', type=Path, required=True)
    p.add_argument('--controller', choices=['whole_body', 'arm_base'], default='whole_body')
    p.add_argument('--grasp-slow-rate', type=float, default=.5)
    p.add_argument('--warning-fraction', type=float, default=.6)
    p.add_argument('--allocation-filter-hz', type=float, default=0.)
    p.add_argument('--allocation-base-regularization', type=float, default=1000.)
    p.add_argument('--relief-limit-rad',type=float,default=.001,
        help='per-axis virtual relief; capped at half the existing 0.01 rad pose-monitor envelope')
    p.add_argument('--delta-deg', type=float, required=True)
    p.add_argument('--duration-s', type=float, required=True)
    p.add_argument('--force-memory-gain', type=float, required=True)
    p.add_argument('--force-velocity-gain', type=float, required=True)
    p.add_argument('--yaw-gain', type=float, default=2.)
    p.add_argument('--xy-gain', type=float, default=.5)
    p.add_argument('--arm-yaw-metric', type=float, required=True)
    p.add_argument('--settle-s', type=float, default=30.)
    p.add_argument('--stop-observe-s', type=float, default=2.)
    p.add_argument('--wrench-calibration-file', type=Path, required=True)
    audit = p.add_mutually_exclusive_group()
    audit.add_argument('--static-yaw-audit',action='store_true',
        help='native Gate-0 only: no admittance enable or rotation goal; no autonomous yaw certificate')
    audit.add_argument('--passive-yaw-audit',action='store_true',
        help='native Gate-1/2 only: XY_ONLY arm compliance, zero follower base, bounded leader positive/negative goals')
    audit.add_argument('--native-o2-audit',action='store_true',
        help='native O2 only: qualified own-base compensation probe; zero leader goals')
    audit.add_argument('--native-follow-audit',action='store_true',
        help='first qualified native WHOLE_BODY transport, leader-only positive/negative goals')
    p.add_argument('--native-passive-result',type=Path)
    p.add_argument('--native-o2-result',type=Path)
    p.add_argument('--native-o2-fault-result',type=Path)
    p.add_argument('--native-allocation-policy',type=Path)
    p.add_argument('--o2-observer-stop',action='store_true')
    p.add_argument('--native-gate0-result',type=Path)
    p.add_argument('--passive-leader-limits-file',type=Path)
    p.add_argument('--static-observe-s',type=float,default=10.)
    p.add_argument('--native-step-divisor',type=int,choices=(1,2,4),default=1)
    p.add_argument('--passive-xy-allocation',action='store_true')
    a = p.parse_args()
    if a.passive_xy_allocation and (not a.passive_yaw_audit or a.native_allocation_policy is None):
        raise ValueError('yaw-locked XY allocation is restricted to explicit native passive calibration')
    measured_step_proof=None
    if a.native_step_divisor!=1 and not a.static_yaw_audit:
        if a.native_step_divisor!=2 or not (a.passive_yaw_audit or a.native_o2_audit or a.native_follow_audit):
            raise ValueError('nondefault operational step requires explicit remeasured 2 ms native audit')
        from amir_rotation_native_measured_step import require_certificate
        measured_step_proof=require_certificate(a.native_metrology_directory,a.run_directory,a.wrench_calibration_file)
    if a.static_yaw_audit and (not np.isfinite(a.static_observe_s) or a.static_observe_s<=0.
            or a.force_memory_gain!=0. or a.force_velocity_gain!=0. or a.controller!='whole_body'):
        raise ValueError('static native audit requires the WHOLE_BODY endpoint, positive observation time and zero force-to-base gains')
    if a.passive_yaw_audit and (a.native_gate0_result is None or a.passive_leader_limits_file is None
            or not np.isfinite([a.static_observe_s,a.delta_deg,a.duration_s]).all()
            or not 0.<a.delta_deg<=.005 or a.duration_s<75. or a.static_observe_s<=0.
            or a.force_memory_gain!=0. or a.force_velocity_gain!=0. or a.xy_gain!=0. or a.yaw_gain!=0.
            or a.controller!='whole_body'):
        raise ValueError('native passive audit requires Gate-0, explicit leader limits, 0<delta<=0.005 deg, duration>=75 s and zero base gains')
    if a.native_o2_audit and (a.native_gate0_result is None or a.native_passive_result is None
            or a.force_memory_gain!=0. or a.force_velocity_gain!=0. or a.xy_gain!=0. or a.yaw_gain!=0.
            or a.controller!='whole_body'):
        raise ValueError('native O2 requires native Gate-0/passive results, one endpoint and zero recentering/force gains')
    if a.o2_observer_stop and not a.native_o2_audit:
        raise ValueError('observer-stop injection is confined to the isolated native O2 audit')
    if a.native_follow_audit and (any(v is None for v in (a.native_gate0_result,a.native_passive_result,
            a.native_o2_result,a.native_o2_fault_result,a.native_allocation_policy,a.passive_leader_limits_file))
            or not np.isfinite([a.delta_deg,a.duration_s,a.xy_gain,a.yaw_gain]).all()
            or not 0.<a.delta_deg<=.005 or a.duration_s<75. or min(a.xy_gain,a.yaw_gain)<=0.
            or a.force_memory_gain!=0. or a.force_velocity_gain!=0. or a.controller!='whole_body'):
        raise ValueError('native follow requires actual Gate-0/passive/O2/fault evidence, explicit allocation/leader limits and bounded initial goals')
    if not os.getenv('IGN_PARTITION','').startswith('amir_cp9_cp8r3_'):
        raise ValueError('existing native preparation requires an amir_cp9_cp8r3_ isolated partition')
    if not np.isfinite(a.relief_limit_rad) or not 0.<a.relief_limit_rad<=.005:
        raise ValueError('virtual relief must stay inside the existing measured-pose envelope')
    a.output, a.run_directory = a.output.resolve(), a.run_directory.resolve()
    a.output.mkdir(parents=True, exist_ok=False)
    if (not np.isfinite([a.delta_deg, a.duration_s, a.settle_s, a.stop_observe_s, a.force_memory_gain,
                         a.force_velocity_gain, a.yaw_gain, a.xy_gain, a.arm_yaw_metric]).all()
            or not 0. < abs(a.delta_deg) <= 5. or a.duration_s <= 0. or a.settle_s < 0.
            or a.stop_observe_s < .5 or min(a.force_memory_gain, a.force_velocity_gain, a.yaw_gain, a.xy_gain) < 0.
            or a.arm_yaw_metric <= 0.):
        raise ValueError('small-angle positive-duration simulation envelope required')
    grasped = True
    report = {'passed': False, 'condition': 'native_grasp_priority_'+a.controller, 'delta_deg': a.delta_deg,
              'argv': __import__('sys').argv, 'ROS_DISTRO': os.getenv('ROS_DISTRO'),
              'ROS_DOMAIN_ID': os.getenv('ROS_DOMAIN_ID'), 'IGN_PARTITION': os.getenv('IGN_PARTITION')}
    if measured_step_proof is not None:report['measured_step_sensor_proof']=measured_step_proof
    if report['ROS_DISTRO'] != 'humble' or not report['IGN_PARTITION'] or not report['ROS_DOMAIN_ID']:
        raise ValueError('explicit Humble, isolated ROS_DOMAIN_ID and IGN_PARTITION required')
    policy_file, follower_file = prepare_control_policies(a.run_directory, a.output, a.duration_s,
        a.force_memory_gain, a.force_velocity_gain, a.yaw_gain, a.xy_gain, a.arm_yaw_metric, a.wrench_calibration_file)
    if a.controller == 'whole_body':
        policy = yaml.safe_load(follower_file.read_text()).pop('/follower/follower_yaw_arm_base')
        policy['ros__parameters'].update(allocation_base_filter_hz=a.allocation_filter_hz,
            allocation_base_regularization=a.allocation_base_regularization, grasp_warning_fraction=a.warning_fraction,
            relief_displacement_rad=[a.relief_limit_rad,a.relief_limit_rad])
        follower_file.write_text(yaml.safe_dump({'/follower/follower_yaw_whole_body': policy}))
        leader_policy = yaml.safe_load(policy_file.read_text())
        leader_policy['/leader/leader_object_rotation']['ros__parameters'].update(grasp_slow_rate=a.grasp_slow_rate,
            grasp_progress_deceleration_s_inv=.5)
        policy_file.write_text(yaml.safe_dump(leader_policy))
    report['completion_policy'] = 'grasp retained, rotation program finished, settled; GT angle/centre are diagnostic only'
    fixture_hashes={}
    if a.static_yaw_audit or a.passive_yaw_audit or a.native_o2_audit or a.native_follow_audit:
        fixture_names=['world.sdf',*[robot+suffix for robot in ('leader','follower')
            for suffix in ('.urdf','_controllers.yaml','_base_controller.yaml')]]
        for name in fixture_names:
            source=a.run_directory/name;destination=a.output/'fixture_snapshot'/name
            destination.parent.mkdir(parents=True,exist_ok=True);destination.write_bytes(source.read_bytes())
            fixture_hashes[name]=hashlib.sha256(source.read_bytes()).hexdigest()
        report['fixture_sha256']=fixture_hashes
    if a.static_yaw_audit:
        report.update(condition='native_static_yaw_gate0',delta_deg=None,
            completion_policy='native static observation, zero goals, stopped outputs and retained physical grasp',
            autonomous_yaw_qualification_issued=False)
    if a.passive_yaw_audit:
        from amir_rotation_native_passive import require_native_gate0,configure_passive
        report['native_gate0_reference']=require_native_gate0(a.native_gate0_result,a.run_directory)
        report['passive_policy']=configure_passive(policy_file,follower_file,delta_deg=a.delta_deg,
            duration_s=a.duration_s,leader_limits_file=a.passive_leader_limits_file,
            xy_allocation=a.passive_xy_allocation,allocation_policy_file=a.native_allocation_policy)
        report.update(condition='native_yaw_locked_xy_allocation_passive_yaw' if a.passive_xy_allocation else 'native_stationary_follower_base_passive_yaw',
            autonomous_follower_yaw_enabled=False,passive_yaw_qualification_issued=False,
            completion_policy='two bounded leader programs, detectable own FK yaw, zero follower base and retained physical grasp')
    if a.native_o2_audit:
        from amir_rotation_native_passive import require_native_gate0
        from amir_rotation_native_o2 import configure_o2
        report['native_gate0_reference']=require_native_gate0(a.native_gate0_result,a.run_directory)
        report['native_o2_reference']=configure_o2(a.native_passive_result,a.run_directory,a.output,follower_file)
        report.update(condition='native_O2_local_compensation',delta_deg=None,observer_stop_injected=a.o2_observer_stop,
            autonomous_follower_yaw_enabled=False,native_o2_verified=False,
            completion_policy='no leader goals; bounded own-base pulse, qualified observation, compensated command and physical stop/hold')
    if a.native_follow_audit:
        from amir_rotation_native_passive import require_native_gate0
        from amir_rotation_native_follow import configure_follow
        report['native_gate0_reference']=require_native_gate0(a.native_gate0_result,a.run_directory)
        report['native_follow_reference']=configure_follow(a,a.run_directory,a.output,policy_file,follower_file)
        report.update(condition='native_qualified_WHOLE_BODY_transport',native_follow_verified=False,
            angle_accuracy_is_diagnostic=True,physical_center_fix_achieved=False,autonomous_follower_yaw_enabled=False)

    profiles = {r: (a.run_directory/(r+'_safety.json')).read_bytes() for r in ('leader', 'follower')}
    for robot, data in profiles.items():
        profile = json.loads(data)
        profile['require_grasp_for_base_motion'] = True
        profile['ungrasped_base_test_only'] = False
        (a.run_directory/(robot+'_safety.json')).write_text(json.dumps(profile, indent=2)+'\n')
        (a.output/(robot+'_safety_used.json')).write_text(json.dumps(profile, indent=2)+'\n')
    code = Path(__file__).resolve().parents[1]/'src/humbleble_ws/amir_compliance_control/amir_compliance_control'
    dependencies=[Path(__file__),*[Path(__file__).with_name(n) for n in (
        'amir_cp8r2_trial.py','amir_cp8r1_base_clearance.py','amir_cp8r3_clearance.py','amir_cp8r3_process.py',
        'cp9_transport_preparation.py','amir_cp9_grasp_preparation.py')]]
    if a.static_yaw_audit or a.passive_yaw_audit or a.native_o2_audit or a.native_follow_audit:
        dependencies.append(Path(__file__).with_name('amir_rotation_native_startup_trace.py'))
        dependencies.append(Path(__file__).with_name('amir_rotation_native_arm_policy.py'))
        dependencies.append(Path(__file__).with_name('amir_rotation_native_measured_step.py'))
        from ament_index_python.packages import get_package_share_directory
        share=Path(get_package_share_directory('cooperative_transport_bringup'))
        dependencies.extend((share/'launch'/name).resolve() for name in (
            'compliance_cp9_guarded_scene.launch.py','compliance_cp9_contact.launch.py','compliance_cp1.launch.py'))
        dependencies.append((Path(get_package_share_directory('cooperative_transport_gazebo'))/'worlds/cooperative_transport_contact.sdf').resolve())
    if a.static_yaw_audit:dependencies.append(Path(__file__).with_name('amir_rotation_native_static.py'))
    if a.passive_yaw_audit:dependencies.extend(Path(__file__).with_name(n) for n in (
        'amir_rotation_native_passive.py','amir_rotation_native_static.py','amir_rotation_integration_audit.py'))
    if a.native_o2_audit:dependencies.extend(Path(__file__).with_name(n) for n in (
        'amir_rotation_native_o2.py','amir_rotation_native_passive.py','amir_rotation_native_static.py','amir_rotation_integration_audit.py'))
    if a.native_follow_audit:dependencies.extend(Path(__file__).with_name(n) for n in (
        'amir_rotation_native_follow.py','amir_rotation_native_o2.py','amir_rotation_native_passive.py',
        'amir_rotation_native_static.py','amir_rotation_integration_audit.py'))
    source_files=[*code.glob('*.py'),*dependencies]
    source_hashes={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in source_files}
    (a.output/'source_sha256.json').write_text(json.dumps(source_hashes,indent=2)+'\n')
    workspace=Path(__file__).resolve().parents[1]
    for file in source_files:
        destination=a.output/'source_snapshot'/file.resolve().relative_to(workspace)
        destination.parent.mkdir(parents=True,exist_ok=True)
        destination.write_bytes(file.read_bytes())
    processes, logs = [], []

    def start(name, command):
        log = (a.output/(name+'.log')).open('w')
        logs.append(log)
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(process)
        return process

    rclpy.init(args=[])
    node = rclpy.create_node('cp8r3_native_evaluator')
    latest, states, gates, guards, poses, outputs = {}, {}, {}, {}, {}, {}
    now, safety, ref, rows, phase = [0.], {}, {}, [], ['startup']
    grasp_states, follow_debug, filter_state = {}, {}, ['']
    progress, grasp_loss, readiness_loss = {}, [], []
    active_grasp_phases=('baseline','rotation','hold','native_static_yaw_audit',
        'native_passive_static','native_passive_positive','native_passive_negative',
        'native_o2_baseline','native_o2_probe','native_o2_hold',
        'native_follow_baseline','native_follow_positive','native_follow_negative')
    qualification_contact_loss = []
    prep, monitor, maintenance, native_poses = {}, {}, {}, {}
    def grasp_update(robot, msg):
        grasp_states[robot] = msg.data
        if phase[0] in active_grasp_phases and msg.data != 'attached':
            readiness_loss.append({'robot': robot, 'sim_time': now[0], 'state': msg.data})
    models = {r: LocalModel((a.run_directory/(r+'.urdf')).read_text()) for r in ('leader', 'follower')}
    initial_object_grasps = {}
    clearance = BaseClearance(a.run_directory)
    (a.output/'base_geometry.json').write_text(json.dumps({'bounds':clearance.bounds,'reserve_m':clearance.reserve},indent=2)+'\n')
    def monitor_update(robot,msg):
        sample=json.loads(msg.data);monitor[robot]=sample
        if not all(sample.get('own_contacts',{}).get(side,False) for side in ('left','right')):
            event={'robot':robot,'sim_time':now[0],'contacts':sample.get('own_contacts',{})}
            if phase[0] in active_grasp_phases:
                grasp_loss.append(event)
            elif phase[0] in ('explicit_monitor_handoff','home','remove_native_support','native_static_hold') and sample.get('state') in ('HOLDING','FAULT'):
                qualification_contact_loss.append(dict(event,phase=phase[0]))
    subs = []
    def output_update(robot,msg):
        values=[msg.linear.x,msg.linear.y,msg.angular.z]
        outputs[robot]=values
        if a.passive_yaw_audit and robot=='follower' and phase[0].startswith('native_passive_'):
            permitted=(max(map(abs,values[:2]))<=.0001 and abs(values[2])<=1e-12) if a.passive_xy_allocation else max(map(abs,values))<=1e-12
            if not np.isfinite(values).all() or not permitted:static_error[0]='passive follower base output violates calibration envelope'
        if a.native_o2_audit and robot=='leader' and phase[0].startswith('native_o2_') and (
                not np.isfinite(values).all() or max(map(abs,values))>1e-12):
            static_error[0]='native O2 leader received a nonzero base output'
    for robot in ('leader', 'follower'):
        for key, suffix, typ in [('joints', 'joint_states', JointState), ('encoder', 'mecanum_drive_controller/odometry', Odometry),
                                 ('wrench', 'ft_sensor', WrenchStamped)]:
            subs.append(node.create_subscription(typ, f'/{robot}/{suffix}',
                lambda msg, r=robot, k=key: latest.__setitem__((r, k), msg), qos_profile_sensor_data))
        for suffix, target in [('transport/state', states), ('safety_debug', gates), ('interlock/debug', guards)]:
            subs.append(node.create_subscription(String, f'/{robot}/{suffix}',
                lambda msg, r=robot, d=target: d.__setitem__(r, json.loads(msg.data)), 10))
        subs.append(node.create_subscription(Twist, f'/{robot}/rover_twist',
            lambda msg, r=robot: output_update(r,msg), 10))
        subs.append(node.create_subscription(String, f'/{robot}/grasp/state',
            lambda msg, r=robot: grasp_update(r, msg), 10))
    for robot in ('leader', 'follower'):
        for suffix, target in [('grasp/debug', prep), ('grasp/monitor_debug', monitor), ('grasp/maintenance_debug', maintenance)]:
            subs.append(node.create_subscription(String, '/'+robot+'/'+suffix, lambda msg,r=robot,d=target:monitor_update(r,msg) if d is monitor else d.__setitem__(r,json.loads(msg.data)), 10))
    subs.append(node.create_subscription(Clock, '/clock', lambda msg: now.__setitem__(0,
        msg.clock.sec+msg.clock.nanosec*1e-9), qos_profile_sensor_data))
    subs.append(node.create_subscription(String, '/transport/safety_status', lambda msg: safety.update(json.loads(msg.data)), 10))
    subs.append(node.create_subscription(String, '/leader/grasp_priority/debug', lambda msg: progress.update(json.loads(msg.data)), 10))
    subs.append(node.create_subscription(String, '/leader/rotation/debug', lambda msg: ref.update(json.loads(msg.data)), 10))
    subs.append(node.create_subscription(String, '/follower/rotation_follow/debug', lambda msg: follow_debug.update(json.loads(msg.data)), 10))
    subs.append(node.create_subscription(String, '/follower/wrench_status', lambda msg: filter_state.__setitem__(0, msg.data), 10))


    def gt(msg):
        for t in msg.transforms:
            if t.child_frame_id in ('leader', 'follower', 'cooperative_payload'):
                native_poses[t.child_frame_id] = t.transform
                poses[t.child_frame_id] = pose_matrix(t.transform.translation, t.transform.rotation)

    subs.append(node.create_subscription(TFMessage, '/world/cooperative_transport/pose/info', gt, qos_profile_sensor_data))
    remover = node.create_client(DeleteEntity, '/world/cooperative_transport/remove')
    goal = node.create_publisher(Float64, '/leader/object_rotation_delta', 1)
    attach = {r: node.create_publisher(Empty, '/'+r+'/grasp/attach', 1) for r in ('leader', 'follower')}
    raw = (a.output/'samples.jsonl').open('w', buffering=1)
    last_record = [0.]
    static_error=[''];static_rows=None;static_auditor=None;pending_certificate=None;geometry=None
    startup_trace=None
    if a.static_yaw_audit or a.passive_yaw_audit or a.native_o2_audit or a.native_follow_audit:
        from amir_rotation_native_startup_trace import NativeStartupTrace
        from amir_compliance_control.rotation_observer import ProjectedYaw
        from trajectory_msgs.msg import JointTrajectory
        startup_trace=NativeStartupTrace(a.output,phase)
        def trace_wrench(msg,kind):
            w=msg.wrench
            try:
                startup_trace.wrench(kind,msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                    msg.header.frame_id,[w.force.x,w.force.y,w.force.z,w.torque.x,w.torque.y,w.torque.z],now[0])
            except (ValueError,TypeError) as exc:
                static_error[0]=str(exc);startup_trace.mark('fault',fault=str(exc))
        for kind,suffix in [('raw','wrench_at_grasp_raw'),('filtered','wrench_filtered')]:
            subs.append(node.create_subscription(WrenchStamped,'/follower/'+suffix,
                lambda msg,k=kind:trace_wrench(msg,k),qos_profile_sensor_data))
        def trace_command(msg):
            if not startup_trace.active():return
            try:
                j=latest['follower','joints']
                if len(msg.points)!=1:raise ValueError('native trace requires one own arm trajectory point')
                startup_trace.command(msg.joint_names,msg.points[0].positions,
                    msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,now[0],j.name,j.position,
                    j.header.stamp.sec+j.header.stamp.nanosec*1e-9)
            except (ValueError,KeyError,TypeError) as exc:
                static_error[0]=str(exc);startup_trace.mark('fault',fault=str(exc))
        subs.append(node.create_subscription(JointTrajectory,'/follower/arm_joint_cmd',trace_command,10))
    passive_ready={'seen':False,'ever_true':False}
    if a.passive_yaw_audit or a.native_o2_audit:
        def passive_ready_update(msg):
            passive_ready['seen']=True
            if msg.data:
                passive_ready['ever_true']=True
                static_error[0]='calibration follower published autonomous rotation readiness'
        subs.append(node.create_subscription(Bool,'/transport/rotation_follower_ready',passive_ready_update,10))
    if a.native_follow_audit:
        def active_ready_update(msg):
            passive_ready.update(seen=True,value=msg.data)
            if msg.data:passive_ready['ever_true']=True
        subs.append(node.create_subscription(Bool,'/transport/rotation_follower_ready',active_ready_update,10))
    o2_recorder=None
    def own_raw_row(msg):
        if msg.header.frame_id!='follower/grasp_aligned':raise ValueError('native raw TCP wrench frame mismatch')
        j=latest['follower','joints'];o=latest['follower','encoder']
        q=dict(zip(j.name,j.position));qd=dict(zip(j.name,j.velocity));w=msg.wrench;t=o.twist.twist
        seconds=lambda m:m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        return dict(timestamp=now[0],joint_stamp=seconds(j),wrench_stamp=seconds(msg),odom_stamp=seconds(o),
            q_rad=[q[n] for n in models['follower'].names],qdot_rad_s=[qd[n] for n in models['follower'].names],
            wrench_tcp=[w.force.x,w.force.y,w.force.z,w.torque.x,w.torque.y,w.torque.z],
            base_twist=[t.linear.x,t.linear.y,t.angular.z],grasp_attached=grasp_states.get('follower')=='attached')
    if a.native_o2_audit or a.native_follow_audit:
        from amir_rotation_native_o2 import NativeO2Recorder
        limits=report['native_follow_reference' if a.native_follow_audit else 'native_o2_reference']['effective_parameters']
        if a.native_follow_audit:
            from amir_rotation_native_follow import NativeFollowRecorder
            o2_recorder=NativeFollowRecorder(a.output,phase,force_limit_N=limits['wrench_force_limit_N'],
                torque_limit_Nm=limits['wrench_torque_limit_Nm'])
        else:
            o2_recorder=NativeO2Recorder(a.output,phase,force_limit_N=limits['wrench_force_limit_N'],
                torque_limit_Nm=limits['wrench_torque_limit_Nm'],fault_injection=a.o2_observer_stop)
        def o2_observe(msg):
            try:
                o2_recorder.observe(json.loads(msg.data))
                if o2_recorder.fault:static_error[0]=o2_recorder.fault
            except (ValueError,KeyError,TypeError) as exc:static_error[0]=str(exc)
        def o2_debug(msg):
            try:
                o2_recorder.record(json.loads(msg.data))
                if o2_recorder.fault:static_error[0]=o2_recorder.fault
            except (ValueError,KeyError,TypeError) as exc:static_error[0]=str(exc)
        subs.append(node.create_subscription(String,'/follower/rotation/observation',o2_observe,1))
        subs.append(node.create_subscription(String,'/follower/rotation_follow/debug',o2_debug,10))
        def o2_raw(msg):
            if not o2_recorder.active():return
            try:
                row=own_raw_row(msg);row.update(phase=phase[0],base_command=outputs['follower'])
                o2_recorder.wrench(row)
                if o2_recorder.fault:static_error[0]=o2_recorder.fault
            except (ValueError,KeyError,TypeError) as exc:
                static_error[0]=str(exc)
                o2_recorder.raw_ft.write(json.dumps(dict(timestamp=now[0],phase=phase[0],fault=str(exc)))+'\n')
        subs.append(node.create_subscription(WrenchStamped,'/follower/wrench_at_grasp_raw',o2_raw,10))
    if a.static_yaw_audit or a.passive_yaw_audit:
        from amir_rotation_native_static import NativeStaticYaw,export_csv
        geometry=None  # Resolve the running endpoint's actual policy below.
        static_rows=(a.output/('native_passive_local.jsonl' if a.passive_yaw_audit else 'native_static_local.jsonl')).open('w',buffering=1)
        def static_sample(msg):
            if phase[0] not in (('native_passive_static','native_passive_positive','native_passive_negative')
                    if a.passive_yaw_audit else ('native_static_yaw_audit',)) or static_error[0]:return
            try:
                row=own_raw_row(msg)
                if a.passive_yaw_audit:
                    row.update(phase=phase[0].removeprefix('native_passive_'),base_command=outputs['follower'])
                static_rows.write(json.dumps(row,allow_nan=False)+'\n');static_auditor.add(row)
            except (ValueError,KeyError,TypeError) as exc:
                static_error[0]=str(exc)
                static_rows.write(json.dumps(dict(timestamp=now[0],fault=str(exc)))+'\n')
        subs.append(node.create_subscription(WrenchStamped,'/follower/wrench_at_grasp_raw',static_sample,10))

    def step():
        rclpy.spin_once(node, timeout_sec=.002)
        if time.monotonic()-last_record[0] < .05:
            return
        last_record[0] = time.monotonic()
        row = {'t_s': now[0], 'wall_s': time.monotonic(), 'phase': phase[0], 'safety_fault': safety.get('fault', ''),
               'leader_state': states.get('leader', {}).get('state', ''),
               'follower_state': states.get('follower', {}).get('state', ''),
               'progress_rate': progress.get('rate', 1.), 'grasp_warning_latched': progress.get('warning_latched', False),
               'grasp_utilization': follow_debug.get('grasp_utilization'),
               'phi_cmd_rad': ref.get('phi_rad'), 'omega_cmd_rad_s': ref.get('omega_rad_s'),
               'yaw_tracking_error_rad': ref.get('yaw_error_rad'),
               'arm_position_error_m': ref.get('arm_position_error_m'), 'arm_angle_error_rad': ref.get('arm_angle_error_rad')}
        for robot in ('leader', 'follower'):
            if (robot, 'encoder') in latest:
                e = latest[robot, 'encoder'].pose.pose
                row.update({robot+'_encoder_x': e.position.x, robot+'_encoder_y': e.position.y, robot+'_encoder_yaw': yaw(e.orientation)})
            if robot in poses:
                for i, k in enumerate('xyz'):
                    row[robot+'_GT_'+k] = float(poses[robot][i, 3])
                row[robot+'_GT_yaw'] = math.atan2(poses[robot][1, 0], poses[robot][0, 0])
            for value, component in zip(outputs.get(robot, [None]*3), ('vx', 'vy', 'wz')):
                row[robot+'_output_'+component] = value
            row[robot+'_grasp_state'] = grasp_states.get(robot, '')
            row[robot+'_native_contacts'] = all(monitor.get(robot,{}).get('own_contacts',{}).get(k,False) for k in ('left','right'))
            for side in ('left','right'):
                row[robot+'_finger_'+side+'_N'] = monitor.get(robot,{}).get('own_forces_N',{}).get(side)
            if (robot,'wrench') in latest and (robot,'joints') in latest and robot in poses:
                row[robot+'_world_vertical_force_N'] = measured_world_vertical_force(robot)
            if (robot, 'wrench') in latest:
                w = latest[robot, 'wrench'].wrench
                for key, value in zip(('fx', 'fy', 'fz', 'tx', 'ty', 'tz'),
                                     (w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z)):
                    row[robot+'_wrist_raw_'+key] = value
            if (robot, 'joints') in latest:
                j = latest[robot, 'joints']
                mapping = dict(zip(j.name, j.position))
                model = models[robot]
                q = np.array([mapping[n] for n in model.names])
                tcp, sensor = model.frames(q)
                row[robot+'_joint_margin_rad'] = float(np.min(np.r_[q-model.lower, model.upper-q]))
                for i, k in enumerate('xyz'):
                    row[robot+'_ee_in_base_'+k] = float(tcp[i])
                for i, value in enumerate(q):
                    row[robot+'_q'+str(i+1)+'_rad'] = float(value)
                for i, value in enumerate(gates.get(robot, {}).get('reaction_torque_Nm') or []):
                    row[robot+'_reaction_torque_'+str(i+1)+'_Nm'] = value
                if grasped and robot in poses and 'cooperative_payload' in poses:
                    local_grasp = sensor.copy()
                    local_grasp[:3, 3] = tcp
                    world_grasp = poses[robot]@local_grasp
                    if a.native_follow_audit and phase[0]=='native_follow_baseline' and robot not in initial_object_grasps:
                        # Evaluation reference only. Never passed to the follower or leader.
                        initial_object_grasps[robot]=inverse_transform(poses['cooperative_payload'])@world_grasp
                    if robot in initial_object_grasps:
                        desired = poses['cooperative_payload']@initial_object_grasps[robot]
                        row[robot+'_grasp_constraint_position_error_m'] = float(np.linalg.norm(world_grasp[:3, 3]-desired[:3, 3]))
                        if a.native_follow_audit:
                            row[robot+'_object_grasp_relation_rotation_change_rad']=float(np.linalg.norm(
                                rotation_error(world_grasp[:3,:3],desired[:3,:3])))
        if 'center_odom_xyz' in ref:
            for i, k in enumerate('xyz'):
                row['center_odom_'+k] = ref['center_odom_xyz'][i]
        if 'cooperative_payload' in poses and grasped:
            for i, k in enumerate('xyz'):
                row['object_center_GT_'+k] = float(poses['cooperative_payload'][i, 3])
            row['object_yaw_GT_rad'] = math.atan2(poses['cooperative_payload'][1, 0], poses['cooperative_payload'][0, 0])
            row['object_vertical_axis_dot_z'] = float(poses['cooperative_payload'][2, 1])
        for index, key in enumerate(('x', 'y', 'yaw')):
            row['follower_local_error_'+key] = (follow_debug.get('measured_error_xy_yaw') or [0.]*3)[index]
            row['follower_force_memory_'+key] = (follow_debug.get('force_memory_velocity_xy_yaw') or [0.]*3)[index]
        row['follower_tau_z_at_tcp_Nm'] = follow_debug.get('tau_z_at_tcp_Nm')
        row['follower_yaw_admittance_rad'] = follow_debug.get('admittance_yaw_rad')
        row['follower_wrench_status'] = filter_state[0]
        for kind in ('raw_tared_tcp_wrench', 'filtered_tcp_wrench'):
            for i, value in enumerate(follow_debug.get(kind) or []):
                row['follower_'+kind+'_'+str(i)] = value
        for i, value in enumerate(follow_debug.get('residual_twist') or []):
            row['follower_arm_task_residual_'+str(i)] = value
        row['follower_arm_min_metric_singular'] = follow_debug.get('arm_min_metric_singular', follow_debug.get('arm_min_singular'))
        for i,value in enumerate(follow_debug.get('relief_roll_pitch_rad') or []):
            row['follower_relief_'+('roll' if i==0 else 'pitch')+'_rad'] = value
        for key in ('whole_min_singular','base_regularization','grasp_warning'):
            row['follower_'+key] = follow_debug.get(key)
        for i,value in enumerate(follow_debug.get('velocity') or []):
            row['follower_allocated_velocity_'+str(i)] = value
        if all(r in poses for r in ('leader','follower')):
            row['base_surface_gap_m'] = clearance.gap(poses['leader'],poses['follower'])
            if phase[0] in ('rotation','hold','native_static_yaw_audit',
                    'native_passive_static','native_passive_positive','native_passive_negative',
                    'native_o2_baseline','native_o2_probe','native_o2_hold',
                    'native_follow_baseline','native_follow_positive','native_follow_negative') and row['base_surface_gap_m'] <= clearance.reserve:
                raise RuntimeError('evaluated base separation below geometry reserve')
        rows.append(row)
        raw.write(json.dumps(row)+'\n')

    def wait(predicate, timeout=60., check_fault=False):
        deadline = time.monotonic()+timeout
        while not predicate():
            if check_fault and static_error[0]:raise RuntimeError(static_error[0])
            if phase[0]=='explicit_monitor_handoff' and any(v.get('state')=='FAULT' for v in monitor.values()):
                raise RuntimeError('native qualification monitor fault before transport: '+str({r:v.get('reason') for r,v in monitor.items()}))
            if check_fault and grasp_loss:
                raise RuntimeError('grasp loss observed: '+str(grasp_loss[0]))
            if check_fault and readiness_loss:
                raise RuntimeError('native grasp readiness withdrawn: '+str(readiness_loss[0]))
            if check_fault and safety.get('fault'):
                raise RuntimeError('shared stop: '+safety['fault'])
            if time.monotonic() > deadline:
                raise RuntimeError(f'timeout phase={phase[0]} states={states} safety={safety}')
            for process in processes:
                retired_preparation = ('tools/amir_cp9_grasp_preparation.py' in process.args and any(
                    '__ns:=/'+robot in process.args and prep.get(robot,{}).get('state')=='RELEASED'
                    for robot in ('leader','follower')))
                if process.poll() is not None and not retired_preparation:
                    raise RuntimeError('trial-owned process exited: '+str(process.args))
            step()

    def settle(seconds, check_fault=True):
        begin = now[0]
        wait(lambda: now[0]-begin >= seconds, timeout=max(30., seconds*5.), check_fault=check_fault)

    def service(name):
        client = node.create_client(Trigger, name)
        wait(client.service_is_ready, 15.)
        deadline = time.monotonic()+5.
        while True:
            f = client.call_async(Trigger.Request())
            wait(f.done, 10.)
            result = f.result()
            if result.success or time.monotonic() > deadline:
                break
            until = time.monotonic()+.2
            wait(lambda: time.monotonic() >= until, 2.)
        node.destroy_client(client)
        with (a.output/'services.jsonl').open('a') as stream:
            stream.write(json.dumps({'name': name, 'success': result.success, 'message': result.message, 't_s': now[0]})+'\n')
        if not result.success:
            raise RuntimeError(name+': '+result.message)

    def remove(name):
        req = DeleteEntity.Request()
        req.entity.name, req.entity.type = name, Entity.MODEL
        f = remover.call_async(req)
        wait(f.done, 10.)
        if not f.result().success:
            raise RuntimeError('cannot remove '+name)

    def measured_world_vertical_force(robot):
        j = latest[robot, 'joints']
        mapping = dict(zip(j.name, j.position))
        model = models[robot]
        _, sensor = model.frames([mapping[n] for n in model.names])
        f = latest[robot, 'wrench'].wrench.force
        return float((poses[robot][:3, :3]@sensor[:3, :3]@np.array([f.x, f.y, f.z]))[2])

    try:
        start('scene', ['ros2', 'launch', 'cooperative_transport_bringup', 'compliance_cp9_guarded_scene.launch.py',
                        'run_directory:='+str(a.run_directory),'physics_step_divisor:='+str(a.native_step_divisor)])
        start('evaluation_bridge', ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/world/cooperative_transport/remove@ros_gz_interfaces/srv/DeleteEntity',
            '/world/cooperative_transport/set_pose@ros_gz_interfaces/srv/SetEntityPose',
            '/world/cooperative_transport/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'])
        wait(lambda: all((r, k) in latest for r in ('leader', 'follower') for k in ('joints', 'encoder'))
             and all(r in poses for r in ('leader', 'follower', 'cooperative_payload')) and remover.service_is_ready(), 100.)
        phase[0] = 'settled_fixture_measurement'
        settle(2., False)
        if fixture_hashes and any(hashlib.sha256((a.run_directory/name).read_bytes()).hexdigest()!=digest
                                  for name,digest in fixture_hashes.items()):
            raise RuntimeError('native fixture changed during scene launch; no grasp or rotation allowed')
        measured = {}
        for robot in ('leader', 'follower'):
            xml = (a.run_directory/(robot+'.urdf')).read_text()
            model = LocalModel(xml)
            j = latest[robot, 'joints']
            mapping = dict(zip(j.name, j.position))
            q = [mapping[n] for n in model.names]
            tcp, sensor = model.frames(q)
            world_sensor = poses[robot]@sensor
            measured[robot] = {'q_rad': q, 'world_T_base_GT': poses[robot].tolist(),
                'sensor_xyz_in_base': sensor[:3, 3].tolist(), 'sensor_xyz_world_evaluation': world_sensor[:3, 3].tolist(),
                'object_xyz_world_evaluation': poses['cooperative_payload'][:3, 3].tolist()}
        (a.output/'settled_fixture_measurement.json').write_text(json.dumps(measured, indent=2)+'\n')
        if not grasped:
            remove('cooperative_payload')
            remove('grasp_support')
        # The native world must contain no fixture constraint, synthetic
        # heartbeat or calibration actuator. All contact support is physical.
        world = ET.parse(a.run_directory/'world.sdf')
        allowed = {'Physics','UserCommands','SceneBroadcaster','Contact','Sensors','ForceTorque','DartContactParameters'}
        plugins = [e.get('name','').rsplit('::',1)[-1] for e in world.findall('.//plugin')]
        if not all(name in allowed for name in plugins):
            raise RuntimeError('non-native support/calibration plugin refused')
        report['native_world_plugins'] = plugins
        check = subprocess.run(['python3','tools/amir_make_safety_profiles.py','--probe',
            str(a.native_metrology_directory.resolve()/'static_proof/runtime.json'),'--run-directory',str(a.run_directory),
            '--motion-evidence','audits/whole_body_compliance_20261019/cp1/motion_run3/motion.json'],capture_output=True,text=True)
        (a.output/'native_profile_derivation.log').write_text(check.stdout+check.stderr)
        if check.returncode: raise RuntimeError('native measured safety profile generation failed')
        for robot in ('leader','follower'):
            (a.output/(robot+'_safety_used.json')).write_bytes((a.run_directory/(robot+'_safety.json')).read_bytes())
        # Match the existing native bench: joints/odom can arrive before
        # gripper_controller has activated. Do not start a closing watchdog
        # while the original spawners are still bringing up its action server.
        phase[0]='startup_controllers'
        if a.static_yaw_audit or a.passive_yaw_audit or a.native_o2_audit or a.native_follow_audit:
            # Match the current fixed-grasp bench: allow all original
            # spawners to acknowledge activation before a read-only RPC.
            names=('joint_state_broadcaster','arm_controller','mecanum_drive_controller','gripper_controller')
            def spawners_finished():
                lines=(a.output/'scene.log').read_text().splitlines()
                return all(any('['+r+'.spawner_'+name+']' in line and 'Configured and activated' in line
                    for line in lines) for r in ('leader','follower') for name in names)
            wait(spawners_finished,180.)
        from controller_manager_msgs.srv import ListControllers
        startup={}
        for robot in ('leader','follower'):
            client=node.create_client(ListControllers,'/'+robot+'/controller_manager/list_controllers')
            wait(client.service_is_ready,30.)
            deadline=time.monotonic()+120.
            while time.monotonic()<deadline:
                future=client.call_async(ListControllers.Request())
                reply_deadline=time.monotonic()+2.
                while not future.done() and time.monotonic()<reply_deadline:step()
                if future.done() and future.exception() is None:
                    states_now={c.name:c.state for c in future.result().controller}
                    startup[robot]=states_now
                    if all(states_now.get(c)=='active' for c in ('joint_state_broadcaster','arm_controller','mecanum_drive_controller','gripper_controller')):break
                else:
                    client.remove_pending_request(future);future.cancel()
                until=time.monotonic()+.2
                wait(lambda:time.monotonic()>=until,2.)
            else:raise RuntimeError('native initial controller readiness timeout: '+robot)
            node.destroy_client(client)
        (a.output/'startup_controllers.json').write_text(json.dumps(startup,indent=2)+'\n')
        def wall_wait(seconds):
            end = time.monotonic()+seconds
            wait(lambda:time.monotonic()>=end,max(5.,seconds+2.))
        from cp9_transport_preparation import prepare
        prepare(node=node,robots=('leader','follower'),generated=a.run_directory,output=a.output,
            start=start,wait=wait,sim_wait=settle,wall_wait=wall_wait,service=service,
            poses=native_poses,now=now,phase=phase,prep=prep,monitor=monitor,report=report,grip_maintenance=True)
        # A ONE-TIME supported, stationary fixture calibration before the
        # operational leader starts. No running controller ever reads GT.
        if not all(monitor.get(r,{}).get('state')=='HOLDING' and monitor[r].get('healthy')
                   and all(monitor[r]['own_contacts'].values()) for r in ('leader','follower')):
            raise RuntimeError('supported native grasp not verified; no operational leader started')
        world_object = poses['cooperative_payload'].copy()
        p_to_o = np.eye(4);p_to_o[:3,:3]=rotation([1.,0.,0.],-math.pi/2.)
        world_object = world_object@p_to_o
        model=models['leader'];mapping=dict(zip(latest['leader','joints'].name,latest['leader','joints'].position))
        tcp,sensor=model.frames([mapping[n] for n in model.names]);sensor[:3,3]=tcp
        certificate={'robot':'leader','grasp_frame':'tcp_link','object_frame':'geometric_center_z_up','base_frame':'base_footprint',
            'urdf_sha256':hashlib.sha256((a.run_directory/'leader.urdf').read_bytes()).hexdigest(),
            'scope':'native_contact_verified','native_contact_only':True,'both_native_monitors_holding':True,
            'calibration_verified':True,'calibration_phase':'stationary supported fixture before control startup',
            'object_T_grasp':(inverse_transform(world_object)@poses['leader']@sensor).tolist()}
        (a.output/'leader_native_grasp_calibration.json').write_text(json.dumps(certificate,indent=2)+'\n')
        leader_policy=yaml.safe_load(policy_file.read_text())
        leader_policy['/leader/leader_object_rotation']['ros__parameters']['native_grasp_monitor_required']=True
        policy_file.write_text(yaml.safe_dump(leader_policy))
        start('safety', ['ros2', 'launch', 'amir_compliance_control', 'guarded_safety.launch.py',
                         'run_directory:='+str(a.run_directory)])
        if a.passive_yaw_audit or a.native_o2_audit:
            # Isolated calibration exception only. Production launch gates remain unchanged.
            start('leader',['ros2','run','amir_compliance_control','leader_object_rotation','--ros-args',
                '-r','__ns:=/leader','--params-file',str(policy_file),'-p','use_sim_time:=true',
                '-p','urdf_file:='+str(a.run_directory/'leader.urdf')])
            if a.native_o2_audit:
                start('follower',['ros2','launch','amir_compliance_control','rotation_compensation_probe.launch.py',
                    'run_directory:='+str(a.run_directory),'follower_policy_file:='+str(follower_file),
                    'wrench_calibration_file:='+str(a.wrench_calibration_file.resolve()),
                    'observer_policy_file:='+report['native_o2_reference']['observer_policy_file']])
            else:
                start('follower',['ros2','launch','amir_compliance_control','rotation_follow_integrated.launch.py',
                    'run_directory:='+str(a.run_directory),'follower_policy_file:='+str(follower_file),
                    'wrench_calibration_file:='+str(a.wrench_calibration_file.resolve()),
                    'rotation_follow_mode:=XY_ONLY','yaw_heading_frame:=grasp_heading',
                    'process_wrench_on_measurement:=true'])
        else:
            start('leader', ['ros2', 'launch', 'amir_compliance_control', ('grasp_priority_leader.launch.py' if a.controller == 'whole_body' else 'object_rotation_leader.launch.py'),
                             'run_directory:='+str(a.run_directory), 'rotation_policy_file:='+str(policy_file)])
            if a.native_follow_audit:
                start('follower',['ros2','launch','amir_compliance_control','rotation_follow_integrated.launch.py',
                    'run_directory:='+str(a.run_directory),'follower_policy_file:='+str(follower_file),
                    'wrench_calibration_file:='+str(a.wrench_calibration_file.resolve()),
                    'rotation_follow_mode:=WHOLE_BODY','yaw_heading_frame:=grasp_heading',
                    'observer_policy_file:='+report['native_follow_reference']['observer_policy_file'],
                    'process_wrench_on_measurement:=true'])
            else:
                start('follower', ['ros2', 'launch', 'amir_compliance_control', ('yaw_whole_body.launch.py' if a.controller == 'whole_body' else 'yaw_arm_base.launch.py'),
                    'run_directory:='+str(a.run_directory), 'follower_policy_file:='+str(follower_file),
                    'wrench_calibration_file:='+str(a.wrench_calibration_file.resolve()),
                    *(['yaw_heading_frame:=grasp_heading'] if a.static_yaw_audit else [])])
        wait(lambda: safety.get('ready') and all(states.get(r, {}).get('local_ready') for r in ('leader', 'follower')), 45.)
        wall_wait(2.)
        # All DDS participants must finish startup before enabling the strict
        # maintenance watchdog. A startup FAULT is never reset or bypassed.
        for robot in ('leader','follower'):
            service('/'+robot+'/grasp/enable_maintenance')
        wait(lambda:all(maintenance.get(r,{}).get('enabled') for r in ('leader','follower')),10.)
        wall_wait(.5)
        if not all(maintenance.get(r,{}).get('state') in ('READY','ACTIVE')
                   and monitor.get(r,{}).get('state')=='HOLDING'
                   and monitor[r].get('healthy') for r in ('leader','follower')):
            raise RuntimeError('native maintenance/monitor qualification failed before arming')
        service('/leader/transport/prepare')
        service('/follower/transport/prepare')
        service('/transport/safety/arm')
        phase[0] = 'home'
        wait(lambda: all(states.get(r, {}).get('state') == 'READY' for r in ('leader', 'follower')), 20., True)
        phase[0] = 'remove_native_support'
        before_grasp = [r for r in rows if r['phase']=='pre_grasp' and 'leader_world_vertical_force_N' in r]
        if not before_grasp: raise RuntimeError('no native pre-grasp weight baseline')
        baseline_fz = sum(before_grasp[-1][r+'_world_vertical_force_N'] for r in ('leader','follower'))
        remove('grasp_support')
        phase[0] = 'native_static_hold'
        settle(10.)
        late = [r for r in rows if r['phase']=='native_static_hold' and r['t_s']>now[0]-2.]
        drift=[float(np.ptp([r['object_center_GT_'+k] for r in late])) for k in 'xyz']
        mass=sum(float(e.text) for e in world.findall("world/model[@name='cooperative_payload']/link/inertial/mass"))
        weight=baseline_fz-sum(measured_world_vertical_force(r) for r in ('leader','follower'))
        expected=mass*abs(float(world.findtext('world/gravity').split()[2]))
        report['static_grasp_check']={'mass_kg':mass,'expected_weight_N':expected,'measured_added_weight_N':weight,'late_center_span_xyz_m':drift}
        if max(drift)>.0001 or abs(weight-expected)>.1 or not all(monitor.get(r,{}).get('state')=='HOLDING' for r in ('leader','follower')):
            raise RuntimeError('native static support qualification failed; no rotation goal sent')
        phase[0] = 'tare'
        service('/follower/tare_wrench')
        tare_completed_s=now[0]
        if startup_trace is not None:startup_trace.mark('tare_completed',tare_completed_s=tare_completed_s)
        wait(lambda: filter_state[0] == 'READY' and (startup_trace is None
            or startup_trace.pair_after_tare(tare_completed_s,now[0])), 10., True)
        if a.static_yaw_audit or a.passive_yaw_audit or a.native_o2_audit or a.native_follow_audit:
            if any(hashlib.sha256(Path(file).read_bytes()).hexdigest()!=digest for file,digest in source_hashes.items()):
                raise RuntimeError('native audit source changed before observation')
            from rcl_interfaces.srv import GetParameters
            from rcl_interfaces.msg import ParameterType
            controller='follower_yaw_whole_body' if a.controller=='whole_body' else 'follower_yaw_arm_base'
            client=node.create_client(GetParameters,'/follower/'+controller+'/get_parameters')
            wait(client.service_is_ready,10.)
            request=GetParameters.Request();request.names=['rotation_error_limit_rad','joint_margin_rad',
                'wrench_force_limit_N','wrench_torque_limit_Nm','yaw_heading_frame']
            future=client.call_async(request);wait(future.done,10.)
            values=future.result().values
            if (len(values)!=len(request.names) or any(v.type!=ParameterType.PARAMETER_DOUBLE for v in values[:4])
                    or values[4].type!=ParameterType.PARAMETER_STRING or values[4].string_value!='grasp_heading'):
                raise RuntimeError('actual native endpoint pose/load parameters unavailable')
            effective=dict(zip(request.names[:4],[v.double_value for v in values[:4]]))
            effective['yaw_heading_frame']=values[4].string_value;node.destroy_client(client)
            report['effective_audit_parameters']=effective
            if (a.passive_yaw_audit or a.native_o2_audit or a.native_follow_audit) and effective!=report['native_gate0_reference']['effective_parameters']:
                raise RuntimeError('native Gate-0 and passive endpoint pose/load policies differ')
            (a.output/('passive_effective_parameters.json' if a.passive_yaw_audit else 'static_effective_parameters.json')).write_text(json.dumps(effective,indent=2)+'\n')
            geometry=ProjectedYaw((a.run_directory/'follower.urdf').read_text(),
                orientation_envelope_rad=effective['rotation_error_limit_rad'],joint_margin_rad=effective['joint_margin_rad'],
                heading_frame='grasp_heading')
            if a.passive_yaw_audit or a.native_o2_audit or a.native_follow_audit:
                from amir_rotation_native_arm_policy import read_live
                arm_policy=read_live(node,wait)
                report['native_arm_policy']=arm_policy
                (a.output/'native_arm_policy.json').write_text(json.dumps(arm_policy,indent=2)+'\n')
                if a.passive_yaw_audit:
                    if arm_policy['rotational_relief_enabled']:
                        raise RuntimeError('native passive orientation-hold policy requires no roll/pitch relief')
                elif arm_policy!=report['native_follow_reference' if a.native_follow_audit else 'native_o2_reference']['native_arm_policy']:
                    raise RuntimeError('live native arm mechanics differ from qualified passive evidence')
            if a.native_follow_audit:
                from amir_rotation_native_follow import execute_follow
                report['native_follow_measurement']=execute_follow(a,report['native_follow_reference'],service=service,
                    settle=settle,wait=wait,phase=phase,goal=goal,states=states,recorder=o2_recorder,
                    ready=passive_ready,source_hashes=source_hashes)
                report.update(passed=True,positive_negative_programs_completed=True)
            elif a.native_o2_audit:
                from amir_rotation_native_o2 import execute_o2
                report['native_o2_measurement']=execute_o2(a,service=service,settle=settle,wait=wait,phase=phase,
                    states=states,outputs=outputs,recorder=o2_recorder,ready=passive_ready,source_hashes=source_hashes)
                encoders=[r['follower_encoder_yaw'] for r in rows if r['phase'].startswith('native_o2_') and 'follower_encoder_yaw' in r]
                report['native_o2_encoder_yaw_span_rad']=float(np.ptp(encoders)) if encoders else 0.
                report['passed']=bool(report['native_o2_encoder_yaw_span_rad']>0.)
            elif a.passive_yaw_audit:
                from amir_rotation_native_passive import NativePassiveYaw,execute_passive,read_passive_endpoint
                report['passive_live_parameters']=read_passive_endpoint(node,wait)
                if report['passive_live_parameters']['passive_xy_allocation_enabled']!=a.passive_xy_allocation:
                    raise RuntimeError('live passive XY allocation selection differs from requested calibration condition')
                static_auditor=NativePassiveYaw(geometry,force_limit_N=effective['wrench_force_limit_N'],
                    torque_limit_Nm=effective['wrench_torque_limit_Nm'],allow_base_xy=a.passive_xy_allocation)
                pending_certificate=execute_passive(a,node=node,service=service,settle=settle,wait=wait,
                    phase=phase,goal=goal,states=states,auditor=static_auditor,rows_stream=static_rows,
                    ready=passive_ready,source_hashes=source_hashes)
                report.update(passed=True,positive_negative_programs_completed=True,
                    measured_signal_rad=pending_certificate['measured_signal_rad'],
                    minimum_signal_to_noise=pending_certificate['minimum_signal_to_noise'])
            else:
                static_auditor=NativeStaticYaw(geometry,force_limit_N=effective['wrench_force_limit_N'],
                    torque_limit_Nm=effective['wrench_torque_limit_Nm'])
                phase[0]='native_static_yaw_audit'
                settle(a.static_observe_s)
                if static_error[0]:raise RuntimeError(static_error[0])
                report['native_gate0']=static_auditor.finish()
                report['motion_completed_in_own_odom']=False
                report['passed']=True
        else:
            service('/follower/admittance/enable')
            settle(.3)
            phase[0] = 'baseline'
            settle(1.)
            origin = {r: poses[r].copy() for r in ('leader', 'follower')}
            object_start = poses.get('cooperative_payload', np.eye(4)).copy()
            for robot in ('leader', 'follower'):
                mapping = dict(zip(latest[robot, 'joints'].name, latest[robot, 'joints'].position))
                tcp, sensor = models[robot].frames([mapping[n] for n in models[robot].names])
                sensor[:3, 3] = tcp
                initial_object_grasps[robot] = inverse_transform(object_start)@origin[robot]@sensor
            if any(hashlib.sha256(Path(file).read_bytes()).hexdigest()!=digest for file,digest in source_hashes.items()):
                raise RuntimeError('experiment source changed before goal; motion refused')
            goal.publish(Float64(data=math.radians(a.delta_deg)))
            phase[0] = 'rotation'
            wait(lambda: states.get('leader', {}).get('accepted_goals') == 1, 5., True)
            wait(lambda: states.get('leader', {}).get('completed_goals') == 1, max(1000., a.duration_s*5.), True)
            phase[0] = 'hold'
            hold_begin, quiet_since = now[0], [None]
            def settled_with_grasp():
                error = follow_debug.get('measured_error_xy_yaw')
                command = follow_debug.get('base_twist')
                quiet = (error is not None and command is not None and
                    np.linalg.norm([command[0],command[1],.1*command[2]]) < .00001 and
                    np.linalg.norm(error[:2]) < .0001 and abs(error[2]) < .0001 and
                    all(grasp_states.get(robot)=='attached' for robot in ('leader','follower')))
                quiet_since[0] = (now[0] if quiet_since[0] is None else quiet_since[0]) if quiet else None
                if now[0]-hold_begin > a.settle_s:
                    raise RuntimeError('local grip-preserving settling deadline exceeded')
                return quiet_since[0] is not None and now[0]-quiet_since[0] >= 2.
            wait(settled_with_grasp,max(30.,a.settle_s*5.),True)
            report['settling_time_s'] = now[0]-hold_begin
            movement = {r: inverse_transform(origin[r])@poses[r] for r in origin}
            gt_angle = math.atan2(movement['leader'][1, 0], movement['leader'][0, 0])
            report['leader_GT_delta_yaw_rad'] = gt_angle
            report['leader_GT_delta_xyz_m'] = movement['leader'][:3, 3].tolist()
            report['follower_GT_delta_xyz_m'] = movement['follower'][:3, 3].tolist()
            report['leader_encoder_angle_error_rad'] = ref.get('yaw_error_rad')
            report['gt_angle_ratio'] = gt_angle/math.radians(a.delta_deg)
            report['motion_completed_in_own_odom'] = True
            report['passed'] = True  # Angle error is a recorded diagnostic, per user priority.
            late = [r for r in rows if r['phase'] == 'hold' and r['t_s'] > now[0]-1.]
            report['final_follower_speed_m_s'] = float(max(np.linalg.norm([
                r.get('follower_output_vx', 0.), r.get('follower_output_vy', 0.), .1*r.get('follower_output_wz', 0.)]) for r in late))
            report['final_follower_local_error_xy_yaw'] = follow_debug.get('measured_error_xy_yaw')
            report['passed'] = bool(report['passed'] and report['final_follower_speed_m_s'] < .00001
                and np.linalg.norm(report['final_follower_local_error_xy_yaw'][:2]) < .0001
                and abs(report['final_follower_local_error_xy_yaw'][2]) < .0001)
            report['local_following_passed'] = bool(report['passed'] and states.get('follower', {}).get('accepted_goals') == 0)
            if grasped:
                object_delta = inverse_transform(object_start)@poses['cooperative_payload']
                # Object P has rolled axes: use world long-axis heading directly.
                object_yaw = math.atan2(poses['cooperative_payload'][1, 0], poses['cooperative_payload'][0, 0])
                object_yaw0 = math.atan2(object_start[1, 0], object_start[0, 0])
                object_angle = math.atan2(math.sin(object_yaw-object_yaw0), math.cos(object_yaw-object_yaw0))
                report['object_GT_delta_yaw_rad'] = object_angle
                report['object_motion_observed'] = abs(object_angle)>1e-6
                report['object_center_drift_m'] = float(np.linalg.norm(object_delta[:3, 3]))
                moving_centers = np.array([[r['object_center_GT_'+k] for k in 'xyz'] for r in rows
                                          if r['phase'] in ('rotation', 'hold') and 'object_center_GT_z' in r])
                report['object_center_max_drift_m'] = float(np.max(np.linalg.norm(moving_centers-object_start[:3, 3], axis=1)))
                report['evaluation_only_shared_reference'] = False
                report['center_drift_limit_m'] = .001
                report['center_drift_criterion_passed'] = max(report['object_center_drift_m'], report['object_center_max_drift_m']) < .001
                report['passed'] = bool(report['passed'] and states.get('follower', {}).get('accepted_goals') == 0
                    and not grasp_loss and report['object_motion_observed'])
            if not report['passed']:
                report['error'] = 'grasp preservation, program completion or local settling criterion failed'
    except Exception as exc:
        report['error'], report['error_phase'] = str(exc), phase[0]
    finally:
        try:
            phase[0] = 'safe_stop'
            service('/transport/safety/stop')
            settle(a.stop_observe_s, False)
            stopped = [r for r in rows if r['phase'] == 'safe_stop']
            # Allow the established stop propagation interval, then require
            # persistent zero output and grasp hold, without any reset/rearm.
            stable = [r for r in stopped if r['t_s'] >= stopped[0]['t_s']+.3]
            report['both_outputs_zero_after_stop'] = bool(stable) and all(
                max(abs(r.get(robot+'_output_'+k, float('inf'))) for k in ('vx', 'vy', 'wz')) < 1e-12
                for r in stable for robot in ('leader', 'follower'))
            report['both_grasps_retained_after_stop'] = bool(stable) and all(
                r.get(robot+'_native_contacts') is True and all(8. <= r.get(robot+'_finger_'+side+'_N',-1.) <= 26. for side in ('left','right')) for r in stable for robot in ('leader', 'follower'))
            report['post_stop_observation_s'] = a.stop_observe_s
        except Exception as exc:
            report['cleanup_stop_error'] = str(exc)
        report['states'], report['final_safety'], report['gate_states'], report['interlocks'] = states, safety, gates, guards
        graph = {}
        for name, namespace in node.get_node_names_and_namespaces():
            try:
                graph[namespace.rstrip('/')+'/'+name] = {
                    'subscriptions': node.get_subscriber_names_and_types_by_node(name, namespace),
                    'publishers': node.get_publisher_names_and_types_by_node(name, namespace)}
            except Exception:
                pass
        (a.output/'graph.json').write_text(json.dumps(graph, indent=2)+'\n')
        report['command_writer_counts'] = {r: len(node.get_publishers_info_by_topic('/'+r+'/base_cmd'))
                                           for r in ('leader', 'follower')}
        report['arm_command_writer_counts'] = {r: len(node.get_publishers_info_by_topic('/'+r+'/arm_joint_cmd'))
                                               for r in ('leader', 'follower')}
        leader_inputs = graph.get('/leader/leader_object_rotation', {}).get('subscriptions', [])
        report['leader_input_boundary_passed'] = bool(leader_inputs) and not any(
            t.startswith(('/follower/', '/experiment/', '/world/')) or 'ground_truth' in t or '/tf' in t for t, _ in leader_inputs)
        follower_names = (('/follower/follower_yaw_whole_body' if a.controller == 'whole_body' else '/follower/follower_yaw_arm_base'), '/follower/wrench_preprocessor',
                          '/follower/safety_gate', '/follower/actuator_interlock', '/follower/controller_audit', '/follower/native_grasp_monitor', '/follower/local_grip_maintenance')
        if a.static_yaw_audit or a.passive_yaw_audit or a.native_o2_audit or a.native_follow_audit:follower_names+=('/follower/rotation_observer',)
        report['follower_input_boundary_passed'] = all(n in graph for n in follower_names) and not any(
            t.startswith(('/leader/', '/experiment/', '/world/')) or 'goal' in t or 'ground_truth' in t or '/tf' in t
            for name in follower_names for t, _ in graph.get(name, {}).get('subscriptions', []))
        report['grasp_loss_events'] = grasp_loss
        report['grasp_readiness_loss_events'] = readiness_loss
        report['qualification_contact_loss_events'] = qualification_contact_loss
        report['last_progress'] = progress
        report['native_grasp_monitors'] = monitor
        report['native_grasp_preparation'] = prep
        report['grip_maintenance'] = maintenance
        report['last_follower_debug'], report['last_wrench_status'] = follow_debug, filter_state[0]
        report['passed'] = bool(report['passed'] and report['leader_input_boundary_passed'] and report['follower_input_boundary_passed']
                                and report.get('both_outputs_zero_after_stop') and report.get('both_grasps_retained_after_stop')
                                and not grasp_loss and states.get('leader', {}).get('completed_goals') == (2 if a.passive_yaw_audit or a.native_follow_audit else (0 if a.static_yaw_audit or a.native_o2_audit else 1))
                                and all(n == 1 for n in report['command_writer_counts'].values())
                                and all(n == 1 for n in report['arm_command_writer_counts'].values()))
        if a.static_yaw_audit or a.passive_yaw_audit or a.native_o2_audit or a.native_follow_audit:
            report['fixture_unchanged']=all(hashlib.sha256((a.run_directory/name).read_bytes()).hexdigest()==digest
                for name,digest in fixture_hashes.items())
            report['passed']=bool(report['passed'] and report['fixture_unchanged'])
        if a.static_yaw_audit:
            report['passed']=bool(report['passed']
                and all(states.get(r,{}).get('state')=='SAFE_STOP' for r in ('leader','follower'))
                and all(states.get(r,{}).get('accepted_goals')==0 for r in ('leader','follower')))
            report['static_audit_fault']=static_error[0]
            report['autonomous_yaw_qualification_issued']=False
            # Even a successful static audit cannot establish a yaw signal.
            report['next_gate']='native passive positive/negative local yaw evidence; no autonomous yaw allowed'
        if a.passive_yaw_audit:
            report['passed']=bool(report['passed'] and not static_error[0] and not passive_ready['ever_true']
                and all(states.get(r,{}).get('state')=='SAFE_STOP' for r in ('leader','follower'))
                and states.get('follower',{}).get('accepted_goals')==0)
            report.update(passive_audit_fault=static_error[0],autonomous_follower_yaw_enabled=False,
                passive_samples=len(static_auditor.records) if static_auditor is not None else 0)
            if static_auditor is not None:
                report.update(maximum_passive_incremental_force_N=static_auditor.maximum_force,
                    maximum_passive_incremental_moment_Nm=static_auditor.maximum_moment)
            if report['passed'] and pending_certificate is not None:
                pending_certificate.update(robot='follower',urdf_sha256=hashlib.sha256((a.run_directory/'follower.urdf').read_bytes()).hexdigest(),
                    native_gate0_reference=report['native_gate0_reference'],native_arm_policy=report['native_arm_policy'])
                (a.output/'qualification.json').write_text(json.dumps(pending_certificate,indent=2)+'\n')
                report['passive_yaw_qualification_issued']=True
        if a.native_o2_audit:
            report['passed']=bool(report['passed'] and not static_error[0] and not passive_ready['ever_true']
                and all(states.get(r,{}).get('state')=='SAFE_STOP' for r in ('leader','follower'))
                and all(states.get(r,{}).get('accepted_goals')==0 for r in ('leader','follower')))
            report.update(native_o2_verified=report['passed'],native_o2_audit_fault=static_error[0],
                autonomous_follower_yaw_enabled=False)
            if o2_recorder is not None:
                report['native_o2_statistics']=o2_recorder.summary();o2_recorder.close()
                from amir_rotation_native_passive import export_passive_csv
                report['native_o2_raw_csv_rows']=export_passive_csv(a.output/'native_o2_raw_ft.jsonl',
                    a.output/'native_o2_raw_ft.csv',geometry)
        if a.native_follow_audit:
            report['passed']=bool(report['passed'] and not static_error[0]
                and all(states.get(r,{}).get('state')=='SAFE_STOP' for r in ('leader','follower'))
                and states.get('follower',{}).get('accepted_goals')==0)
            report.update(native_follow_verified=report['passed'],native_follow_audit_fault=static_error[0])
            report['autonomous_follower_yaw_enabled']=passive_ready['ever_true']
            if o2_recorder is not None:
                report['native_follow_statistics']=o2_recorder.summary();o2_recorder.close()
                from amir_rotation_native_passive import export_passive_csv
                report['native_follow_raw_csv_rows']=export_passive_csv(a.output/'native_follow_raw_ft.jsonl',
                    a.output/'native_follow_raw_ft.csv',geometry)
            from amir_rotation_native_follow import evaluation_diagnostics
            report['native_follow_evaluation_only']=evaluation_diagnostics(rows)
        keys = sorted({key for row in rows for key in row})
        with (a.output/'timeseries.csv').open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=keys)
            writer.writeheader()
            writer.writerows(rows)
        if a.static_yaw_audit:
            static_rows.flush()
            report['static_csv_rows']=export_csv(a.output/'native_static_local.jsonl',
                a.output/'native_static_local.csv',geometry)
        if a.passive_yaw_audit:
            static_rows.flush()
            from amir_rotation_native_passive import export_passive_csv
            report['passive_csv_rows']=export_passive_csv(a.output/'native_passive_local.jsonl',
                a.output/'native_passive_local.csv',geometry)
        if startup_trace is not None:report['startup_trace_events']=startup_trace.close()
        (a.output/'result.json').write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps({k: v for k, v in report.items() if k not in ('gate_states', 'interlocks')}, indent=2), flush=True)
        raw.close()
        if static_rows is not None:static_rows.close()
        node.destroy_node()
        rclpy.shutdown()
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
        from amir_cp8r3_process import reap_partition
        (a.output/'process_cleanup.json').write_text(json.dumps(reap_partition(report['IGN_PARTITION'], report['ROS_DOMAIN_ID']), indent=2)+'\n')
        for log in logs:
            log.close()
        for robot, data in profiles.items():
            (a.run_directory/(robot+'_safety.json')).write_bytes(data)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())

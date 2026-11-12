#!/usr/bin/env python3
"""Isolated chapter-24 evaluator. Peer/GT data never enter follower controller."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET
import numpy as np
import rclpy
from geometry_msgs.msg import Twist, TwistStamped, Vector3Stamped, Wrench, WrenchStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger
from ros_gz_interfaces.srv import DeleteEntity
from ros_gz_interfaces.msg import Entity
from controller_manager_msgs.srv import ListControllers, ListHardwareInterfaces
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.node import NodeNameNonExistentError
from rclpy.signals import SignalHandlerOptions
from rclpy.parameter import parameter_value_to_python
from rcl_interfaces.srv import GetParameters
from control_msgs.msg import JointTrajectoryControllerState
from amir_compliance_control.wrench import LocalModel, origin
from amir_compliance_control.wrench import at_grasp
from amir_compliance_control.arm_reach import translation_progress
from tf2_msgs.msg import TFMessage
from amir_af_local_trace import LocalControlTrace
from af_gui_hold import hold_until_stopped, pump_feedback, coherent_source_ages,pause_world_for_review

ROOT = Path(__file__).resolve().parents[1]
SEED = ROOT/'audits/whole_body_compliance_20261019/cp6'


def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()


def relocate_fixture(folder, ungrasped):
    """Relocate certified artifacts ONLY when exact contents except paths match."""
    evidence = {}
    for r in ('leader', 'follower'):
        old = SEED/'generated'
        for name in (r+'.urdf', r+'_controllers.yaml', r+'_base_controller.yaml', 'world.sdf'):
            expected = (old/name).read_text().replace(str(old), str(folder))
            if (folder/name).read_text() != expected:
                raise ValueError('AF fixture differs from measured CP6 evidence: '+name)
        profile = json.loads((old/(r+'_safety.json')).read_text())
        profile.update(urdf_sha256=sha(folder/(r+'.urdf')), world_sha256=sha(folder/'world.sdf'),
                       controller_yaml_file=str(folder/(r+'_controllers.yaml')),
                       base_controller_yaml_file=str(folder/(r+'_base_controller.yaml')),
                       require_grasp_for_base_motion=not ungrasped, ungrasped_base_test_only=ungrasped)
        (folder/(r+'_safety.json')).write_text(json.dumps(profile, indent=2))
        evidence[r] = {'old_urdf': sha(old/(r+'.urdf')), 'new_urdf': profile['urdf_sha256'],
                       'only_controller_path_changed': True, 'world_unchanged': True}
    cert = json.loads((SEED/'calibration/calibration.json').read_text())
    cert['artifact_relocation'] = evidence['follower']
    cert['urdf_sha256'] = sha(folder/'follower.urdf')
    (folder/'calibration.json').write_text(json.dumps(cert, indent=2))
    (folder/'fixture_relocation.json').write_text(json.dumps(evidence, indent=2))


def pose_matrix(t):
    q = t.rotation; x,y,z,w = q.x,q.y,q.z,q.w
    a = np.eye(4)
    a[:3,:3] = [[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                  [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                  [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]
    a[:3,3] = [t.translation.x,t.translation.y,t.translation.z]
    return a


def wait_for_grasps(grasp, publishers, wait, clock=time.monotonic):
    """Bounded preparation handshake; never used to regrasp during transport."""
    last_send=[float('-inf')]
    def ready():
        pending=[r for r in publishers if grasp.get(r)!='attached']
        if not pending:return True
        now=clock()
        if now-last_send[0]>=.05:
            for robot in pending:publishers[robot].publish(Empty())
            last_send[0]=now
        return False
    # The fixed-grasp plugin treats attach as idempotent desired=true. Keep
    # the support in place until real ECM attachment state confirms both ends.
    wait(ready,10.,True)


def owned_children():
    """Only this explicit experiment domain/partition; ignore reaped zombies."""
    found=[]
    for path in Path('/proc').iterdir():
        if not path.name.isdigit() or int(path.name)==os.getpid():continue
        try:
            env=(path/'environ').read_bytes().split(b'\0')
            stat=(path/'stat').read_text().rsplit(')',1)[1].split()
            if (stat[0]!='Z' and ('IGN_PARTITION='+os.environ['IGN_PARTITION']).encode() in env
                    and ('ROS_DOMAIN_ID='+os.environ['ROS_DOMAIN_ID']).encode() in env):
                found.append(int(path.name))
        except (OSError,ValueError):pass
    return found


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--stage',choices=['AF-0','AF-1','AF-2','AF-3'],required=True)
    p.add_argument('--gui',action='store_true',help='show the existing Gazebo scene GUI')
    p.add_argument('--keep-gui-open',action='store_true',help='after verified stop/hold, monitor stationary grasp until Ctrl+C')
    p.add_argument('--pause-gui-after-stop',action='store_true',help='native GUI: pause physics after verified stop for final-state review; not live hold endurance')
    p.add_argument('--start-delay-s',type=float,default=0.,help='stationary wall-time observation before AF-2 goal (0..60 s)')
    p.add_argument('--native-arm-stroke',action='store_true',help='CP13 native grasp then fixed follower base; leader arm compensates half base travel')
    p.add_argument('--dual-arm-only',action='store_true',help='physical base approach then both bases zero; leader arm translation only')
    p.add_argument('--leader-base-distance-calibration',type=Path,help='native only: offline measured leader odometry distance certificate')
    p.add_argument('--prefer-smt-cores',action='store_true',help='native only: runner/children use measured SMT CPU subset; watchdog limits unchanged')
    p.add_argument('--follower-base-fixture',action='store_true',help='Gazebo test only: lock follower base to ground AFTER native grasp')
    p.add_argument('--progressive-stroke',action='store_true',help='native only: stop/hold at 0.1, 0.3 and final base distance; home stays latched')
    p.add_argument('--fault',choices=['none','dropout','force','saturation','constraint',
                                    'joint_margin','singularity','base_drift','dt'],default='none')
    p.add_argument('--distance',type=float,default=.005)
    p.add_argument('--direction',choices=['forward','backward'],default='forward',
                   help='AF-2 only: signed leader local x; follower receives no goal')
    p.add_argument('--speed',type=float,default=.005)
    p.add_argument('--profile',choices=['chapter24','damped','pose_hold','pose_hold_micro'],default='chapter24',
                   help='explicit simulation gain/acceleration exploration; force guards unchanged')
    p.add_argument('--arm-nullspace-pose-hold',action=argparse.BooleanOptionalAction,default=None,
                   help='opt-in measured z/orientation hold in the XY nullspace; no base motion')
    p.add_argument('--leader-acceleration',type=float,
                   help='explicit simulation-only acceleration for a matched comparison (m/s^2)')
    p.add_argument('--leader-reference-acceleration',type=float)
    p.add_argument('--fault-during-motion',action='store_true',
                   help='AF-3: verify the leader is actually moving before fault injection')
    p.add_argument('--fault-min-speed-fraction',type=float,default=0.,
                   help='AF-3 moving fault: require both actual odom and final output at this fraction of cap')
    p.add_argument('--require-speed-fraction',type=float,default=0.,
                   help='AF-2: require measured odom and command peak >= this fraction of speed cap')
    p.add_argument('--virtual-mass-kg',type=float)
    p.add_argument('--virtual-damping-ns-m',type=float)
    p.add_argument('--virtual-stiffness-n-m',type=float)
    p.add_argument('--arm-command-reference',choices=['integrated','servo_trajectory'])
    p.add_argument('--arm-horizon',type=float)
    p.add_argument('--control-on-wrench',action=argparse.BooleanOptionalAction,default=None)
    p.add_argument('--wrench-cutoff-hz',type=float)
    p.add_argument('--wrench-on-measurement',action=argparse.BooleanOptionalAction,default=None)
    p.add_argument('--servo-response',type=Path,
                   help='measured ungrasped position-servo response; never changes plugin/controller gain')
    p.add_argument('--af1-force-axis',type=int,choices=[0,1],
                   help='AF-1 only: one isolated pulse from rest, paired with --af1-force-sign')
    p.add_argument('--af1-force-sign',type=int,choices=[-1,1])
    a=p.parse_args()
    if a.dual_arm_only and (not a.native_arm_stroke or a.follower_base_fixture or a.leader_base_distance_calibration
            or a.progressive_stroke):
        raise ValueError('dual arm experiment: native, no base fixture/calibration/progressive base stroke')
    if a.dual_arm_only:
        from dual_arm_workspace import common_reach
        if not 0<a.distance<=common_reach(ROOT)['maximum_common_verified_m']:
            raise ValueError('dual arm distance exceeds actual common URDF horizontal reach')
    if a.follower_base_fixture and not a.native_arm_stroke:raise ValueError('post-grasp base fixture requires native arm stroke')
    if a.progressive_stroke and not a.native_arm_stroke:raise ValueError('progressive stroke requires native arm stroke')
    if a.keep_gui_open and (not a.gui or a.stage!='AF-2'):
        raise ValueError('keep-gui-open requires GUI AF-2')
    if a.pause_gui_after_stop and not (a.keep_gui_open and a.native_arm_stroke):
        raise ValueError('paused review requires native ARM_ONLY GUI keep-open')
    if a.leader_base_distance_calibration and not a.native_arm_stroke:
        raise ValueError('base distance calibration is restricted to native strokes')
    if a.prefer_smt_cores and not a.native_arm_stroke:raise ValueError('CPU experiment is native only')
    if not np.isfinite(a.start_delay_s) or not 0<=a.start_delay_s<=60:
        raise ValueError('start-delay-s must be finite in 0..60 seconds')
    if a.direction!='forward' and a.stage!='AF-2':
        raise ValueError('backward direction is currently an AF-2 transport test only')
    signed_distance=a.distance*(1. if a.direction=='forward' else -1.)
    if ((a.af1_force_axis is None)!=(a.af1_force_sign is None)
            or (a.af1_force_axis is not None and a.stage!='AF-1')):
        raise ValueError('isolated force axis/sign must be paired and require AF-1')
    if a.native_arm_stroke:
        if a.stage!='AF-2' or a.direction!='forward' or not (0<a.distance<=.5 and 0<a.speed<=.005):
            raise ValueError('native compensated stroke: AF-2 forward, <=0.5m base and <=0.005m/s only')
        if not os.getenv('IGN_PARTITION','').startswith('amir_cp9_'):raise ValueError('native preparation requires isolated amir_cp9_ partition')
        from amir_native_arm_support import controller_isolation_evidence,verify_loaded_controller
        controller_isolation=controller_isolation_evidence()
    elif not (0<a.distance<=.01 and 0<a.speed<=.01): raise ValueError('micro-test envelope exceeded')
    if a.leader_acceleration is not None and (not np.isfinite(a.leader_acceleration)
            or not 0<a.leader_acceleration<=.01 or a.stage=='AF-0'):
        raise ValueError('AF acceleration must be positive and <=0.01 m/s^2; not a legacy AF-0 option')
    if a.leader_reference_acceleration is not None and (not np.isfinite(a.leader_reference_acceleration)
            or not 0<a.leader_reference_acceleration<=.01 or a.stage=='AF-0'):
        raise ValueError('positive experimental reference acceleration <=0.01 required')
    if a.fault_during_motion and a.stage!='AF-3':raise ValueError('fault-during-motion requires AF-3')
    if (not np.isfinite(a.fault_min_speed_fraction) or not 0<=a.fault_min_speed_fraction<=1
            or (a.fault_min_speed_fraction and (not a.fault_during_motion or a.fault=='saturation'))):
        raise ValueError('speed-fraction injection requires moving AF-3; saturation boundary is tested separately')
    if not np.isfinite(a.require_speed_fraction) or not 0<=a.require_speed_fraction<=1 or (a.require_speed_fraction and a.stage!='AF-2'):
        raise ValueError('speed fraction must be in [0,1] and is an AF-2 check')
    for v in (a.virtual_mass_kg,a.virtual_damping_ns_m):
        if v is not None and (not np.isfinite(v) or v<=0 or a.stage=='AF-0'):
            raise ValueError('virtual mechanics overrides must be finite positive AF experiment values')
    if a.virtual_stiffness_n_m is not None and (not np.isfinite(a.virtual_stiffness_n_m)
            or a.virtual_stiffness_n_m<0 or a.stage=='AF-0'):
        raise ValueError('nonnegative finite virtual stiffness required')
    if a.arm_horizon is not None and (not np.isfinite(a.arm_horizon) or not .02<=a.arm_horizon<=.1):
        raise ValueError('AF arm horizon must remain in [0.02,0.1] seconds')
    if a.wrench_cutoff_hz is not None and (not np.isfinite(a.wrench_cutoff_hz) or a.wrench_cutoff_hz<=0):
        raise ValueError('positive finite experimental wrench cutoff required')
    if os.getenv('ROS_LOCALHOST_ONLY')!='1' or not os.getenv('IGN_PARTITION') or not os.getenv('ROS_DOMAIN_ID'):
        raise ValueError('explicit isolated localhost ROS domain and Gazebo partition required')
    out=a.output.resolve();out.mkdir(parents=True,exist_ok=False);folder=out/'generated';folder.mkdir()
    cpu_policy=None
    if a.prefer_smt_cores:
        from af_cpu_affinity import prefer_smt_cores
        cpu_policy=prefer_smt_cores()
        (out/'cpu_affinity.json').write_text(json.dumps(cpu_policy,indent=2))
    if a.native_arm_stroke:
        (out/'controller_isolation.json').write_text(json.dumps(controller_isolation,indent=2))
    (out/'command.json').write_text(json.dumps({'argv':__import__('sys').argv,
        'env':{k:os.getenv(k) for k in ('ROS_DOMAIN_ID','IGN_PARTITION','ROS_LOCALHOST_ONLY')},
        'stage':a.stage,'fault':a.fault,'base_fixture':'post-grasp Gazebo kinematic base velocity clamp' if a.follower_base_fixture else 'none; original wheel/floor physics',
        'grasp_fixture':'native friction contact (CP13 preparation)' if a.native_arm_stroke else 'fixed constraint; weld_cfm=0.1 (existing CP6)'},indent=2))
    code=ROOT/'src/humbleble_ws/amir_compliance_control'
    sources=[Path(__file__),Path(__file__).with_name('amir_af_local_trace.py'),
             Path(__file__).with_name('af_gui_hold.py'),Path(__file__).with_name('arm_only_micro_runtime.py'),
             Path(__file__).with_name('af_cpu_affinity.py'),
             ROOT/'src/humbleble_ws/cooperative_transport_bringup/launch/compliance_round_trip.launch.py',
             ROOT/'src/humbleble_ws/cooperative_transport_bringup/launch/compliance_arm_first.launch.py',
             *code.glob('amir_compliance_control/*.py'),*code.glob('launch/*.py')]
    if a.leader_base_distance_calibration:sources.append(a.leader_base_distance_calibration.resolve())
    if a.native_arm_stroke:
        sources += [Path(__file__).with_name(name) for name in ('amir_native_arm_support.py',
            'native_arm_stroke_runtime.py','cp9_transport_preparation.py','amir_cp9_grasp_preparation.py')]
        if a.dual_arm_only:
            sources += [ROOT/'tools/dual_arm_only_runtime.py',ROOT/'tools/dual_arm_preparation.py',ROOT/'tools/dual_arm_single_probe.py',ROOT/'tools/dual_arm_workspace.py']
            sources += [ROOT/'src/humbleble_ws/cooperative_transport_bringup/config/compliance_dual_arm_gui.config']
        sources += list((ROOT/'src/vendor/joint_trajectory_controller').rglob('*.cpp'))
        sources += [ROOT/'src/vendor/joint_trajectory_controller/AMIR_PATCH.json']
        if a.follower_base_fixture:
            sources += [ROOT/'src/humbleble_ws/amir_follower_gazebo/src/base_hold_fixture.cpp',
                ROOT/'src/humbleble_ws/amir_follower_gazebo/CMakeLists.txt',
                ROOT/'src/humbleble_ws/cooperative_transport_bringup/launch/compliance_native_arm_scene.launch.py']
    (out/'source_hashes.json').write_text(json.dumps({str(f.relative_to(ROOT)):sha(f) for f in sources},indent=2))
    for source in sources:
        copy=out/'source_snapshot'/source.relative_to(ROOT)
        copy.parent.mkdir(parents=True,exist_ok=True);copy.write_bytes(source.read_bytes())
    processes=[];logs=[];paused=[];rows=[];events=[];latest={};state={};gates={};interlocks={};poses={};adm={};status={}
    grasp={};grasp_wall={};sim=[0.];phase=['startup'];filtered=[None];filter_state=[''];base={};output={};final_output={};qcmd={};models={};pose_receipt_sim={};pose_source_stamp={};odom_peak={}
    gui_holding=[False];gui_process=[None]
    native_prep={};native_monitor={};native_maintenance={};native_jtc_samples={r:[] for r in ('leader','follower')}
    native_monitor_wall={};native_wrench_limits={}
    native_base_home={};native_base_radii={};native_leader_debug={};native_grasp_offsets={}
    native_wrench_violation_since={}
    base_fixture_state=['unknown']
    if a.native_arm_stroke:
        import yaml
        native_wrench_limits=yaml.safe_load((ROOT/'src/humbleble_ws/cooperative_transport_control/config/friction_transport.yaml').read_text())['/**']['ros__parameters']
    constraint_home={};last_record=[0.];applied=[0.]*6;last_force=[0.]
    injected_constraint_error=[0.];constraint_stop=[None]
    trace=(out/'events.jsonl').open('w',buffering=1)
    jtc_trace=(out/'jtc_state_samples.jsonl').open('w',buffering=1);jtc_state={}
    local_trace=LocalControlTrace(out)
    def start(name,argv):
        if name=='gui' and a.dual_arm_only:
            from af_cpu_affinity import gui_command
            argv,proof=gui_command(argv,cpu_policy)
            (out/'gui_cpu_affinity.json').write_text(json.dumps(proof,indent=2))
        if a.native_arm_stroke and name=='grasp_monitor':argv=[*argv,'hold_safe_force_imbalance:=true','simulation_source_watchdog:=true']
        f=(out/(name+'.log')).open('w');logs.append(f)
        proc=subprocess.Popen(argv,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);processes.append(proc)
        return proc
    # Keep the ROS context alive on launch Ctrl+C so finally can request the
    # shared stop, verify grip/hold and reap the separately owned process groups.
    cancel_requested=[None];cleaning_up=[False]
    rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO)
    old_signals={s:signal.getsignal(s) for s in (signal.SIGINT,signal.SIGTERM)}
    def cancel(signum,frame):cancel_requested[0]=signal.Signals(signum).name
    for s in old_signals:signal.signal(s,cancel)
    n=rclpy.create_node('af_experiment_recorder')
    def event(kind,r,value):
        if gui_holding[0]:return
        row={'sim_time':sim[0],'wall_time':time.monotonic(),'phase':phase[0],'kind':kind,'robot':r,'value':value}
        events.append(row);trace.write(json.dumps(row)+'\n')
    subs=[]
    def receive_arm_actuator(msg):
        # Evaluation only: measure request-to-final-JTC interpolation latency.
        event('arm_actuator_cmd','follower',{
            'stamp_s':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
            'positions':list(msg.points[-1].positions) if msg.points else [],
            'points':[{'positions':list(p.positions),
                       'time_from_start_s':p.time_from_start.sec+p.time_from_start.nanosec*1e-9}
                      for p in msg.points],
            'horizon_s':msg.points[-1].time_from_start.sec+msg.points[-1].time_from_start.nanosec*1e-9 if msg.points else None})
    from trajectory_msgs.msg import JointTrajectory
    subs.append(n.create_subscription(JointTrajectory,'/follower/arm_controller/joint_trajectory',receive_arm_actuator,10))
    def receive_jtc(msg,robot='follower'):
        value={'robot':robot,'sim_time':sim[0],'wall_time':time.monotonic(),'phase':phase[0],
            'stamp_s':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,'joint_names':list(msg.joint_names)}
        for name in ('reference','feedback','output','error'):
            point=getattr(msg,name)
            value[name+'_positions']=list(point.positions);value[name+'_velocities']=list(point.velocities)
        if robot=='follower':jtc_state.clear();jtc_state.update(value)
        if a.native_arm_stroke and phase[0] in ('servo_probe_readiness','servo_probe'):
            native_jtc_samples[robot].append(value)
        if not gui_holding[0]:jtc_trace.write(json.dumps(value)+'\n')
    subs.append(n.create_subscription(JointTrajectoryControllerState,'/follower/arm_controller/controller_state',receive_jtc,qos_profile_sensor_data))
    if a.native_arm_stroke:
        subs.append(n.create_subscription(String,'/leader/arm_compensation/debug',lambda m:native_leader_debug.update(json.loads(m.data)),10))
        subs.append(n.create_subscription(JointTrajectoryControllerState,'/leader/arm_controller/controller_state',
            lambda m:receive_jtc(m,'leader'),qos_profile_sensor_data))
        for robot in ('leader','follower'):
            for suffix,dest in (('grasp/debug',native_prep),('grasp/monitor_debug',native_monitor),('grasp/maintenance_debug',native_maintenance)):
                def receive_native(m,r=robot,d=dest,component=suffix):
                    value=json.loads(m.data)
                    if value.get('first_fault_context') and not d.get(r,{}).get('first_fault_context'):
                        event('native_grasp_component_fault',r,{'component':component,'context':value['first_fault_context']})
                    d[r]=value
                    if d is native_monitor:native_monitor_wall[r]=time.monotonic()
                subs.append(n.create_subscription(String,'/'+robot+'/'+suffix,receive_native,10))
        if a.follower_base_fixture:
            subs.append(n.create_subscription(String,'/experiment/follower/base_fixture/state',lambda m:base_fixture_state.__setitem__(0,m.data),10))
    def receive_local(m,robot,key):
        latest[robot,key]=m
        if key=='odom':odom_peak[robot]=max(odom_peak.get(robot,0.),abs(m.twist.twist.linear.x))
        if key=='ft':trace_wrench(m,robot,'raw')
        if key=='ft' and a.native_arm_stroke and status.get('armed') and robot in models and (robot,'q') in latest:
            mapping=dict(zip(latest[robot,'q'].name,latest[robot,'q'].position))
            grasp_point,sensor=models[robot].frames([mapping[name] for name in models[robot].names])
            w=m.wrench;absolute=at_grasp([w.force.x,w.force.y,w.force.z,w.torque.x,w.torque.y,w.torque.z],sensor,grasp_point,1.)
            if (np.linalg.norm(absolute[:2])>=native_wrench_limits['planar_force_stop']
                    or abs(absolute[2])>=native_wrench_limits['vertical_force_stop']
                    or np.linalg.norm(absolute[3:])>=native_wrench_limits['torque_stop']):
                wall=time.monotonic()
                if robot not in native_wrench_violation_since:native_wrench_violation_since[robot]=wall
                if wall-native_wrench_violation_since[robot]>=native_wrench_limits['fault_duration'] and constraint_stop[0] is None:
                    event('inherited_native_absolute_wrench_stop',robot,absolute.tolist())
                    constraint_stop[0]=constraint_stop_client.call_async(Trigger.Request())
            else:native_wrench_violation_since.pop(robot,None)
    def trace_wrench(msg,robot,kind):
        if gui_holding[0]:return
        w=msg.wrench
        local_trace.wrench(kind=kind,robot=robot,phase=phase[0],
            source_stamp_s=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
            receipt_sim_time_s=sim[0],receipt_wall_time_s=time.monotonic(),
            frame_id=msg.header.frame_id,admittance_enabled=bool(adm.get('enabled')),
            values=[w.force.x,w.force.y,w.force.z,w.torque.x,w.torque.y,w.torque.z])
    def receive_filtered(msg):
        filtered[0]=msg
        trace_wrench(msg,'follower','filtered')
    def receive_admittance(msg):
        value=json.loads(msg.data)
        adm.update(value)
        if not gui_holding[0]:
            local_trace.control(value,phase=phase[0],receipt_sim_time_s=sim[0],
                                receipt_wall_time_s=time.monotonic())
    for r in ('leader','follower'):
        for key,suffix,typ in [('q','joint_states',JointState),('odom','mecanum_drive_controller/odometry',Odometry),('ft','ft_sensor',WrenchStamped)]:
            subs.append(n.create_subscription(typ,f'/{r}/{suffix}',lambda m,r=r,k=key:receive_local(m,r,k),qos_profile_sensor_data))
        for key,topic,dest in [('endpoint',f'/{r}/transport/state',state),('gate',f'/{r}/safety_debug',gates),('interlock',f'/{r}/interlock/debug',interlocks)]:
            subs.append(n.create_subscription(String,topic,lambda m,r=r,d=dest:d.__setitem__(r,json.loads(m.data)),10))
        def receive_grasp(m,r=r):
            grasp[r]=m.data;grasp_wall[r]=time.monotonic()
        subs.append(n.create_subscription(String,f'/{r}/grasp/state',receive_grasp,10))
        def base_cb(m,r=r):
            base[r]=[m.twist.linear.x,m.twist.linear.y,m.twist.angular.z];event('base_cmd',r,base[r])
        def output_cb(m,r=r):
            output[r]=[m.linear.x,m.linear.y,m.angular.z];event('rover_twist',r,output[r])
        def final_output_cb(m,r=r):
            final_output[r]=[m.linear.x,m.linear.y,m.angular.z];event('base_final_reference',r,final_output[r])
        def arm_cb(m,r=r):
            qcmd[r]=list(m.points[-1].positions) if m.points else [];event('arm_cmd',r,qcmd[r])
        from trajectory_msgs.msg import JointTrajectory
        subs += [n.create_subscription(TwistStamped,f'/{r}/base_cmd',base_cb,10),
                 n.create_subscription(Twist,f'/{r}/rover_twist',output_cb,10),
                 n.create_subscription(Twist,f'/{r}/mecanum_drive_controller/reference_unstamped',final_output_cb,10),
                 n.create_subscription(JointTrajectory,f'/{r}/arm_joint_cmd',arm_cb,10)]
    def receive_poses(msg):
        for t in msg.transforms:
            poses[t.child_frame_id]=t.transform;pose_receipt_sim[t.child_frame_id]=sim[0]
            pose_source_stamp[t.child_frame_id]=t.header.stamp.sec+t.header.stamp.nanosec*1e-9
    subs += [n.create_subscription(Clock,'/clock',lambda m:sim.__setitem__(0,m.clock.sec+m.clock.nanosec*1e-9),qos_profile_sensor_data),
             n.create_subscription(String,'/transport/safety_status',lambda m:status.update(json.loads(m.data)),10),
             n.create_subscription(String,'/follower/admittance/debug',receive_admittance,10),
             n.create_subscription(String,'/follower/wrench_status',lambda m:filter_state.__setitem__(0,m.data),10),
             n.create_subscription(WrenchStamped,'/follower/wrench_filtered',receive_filtered,qos_profile_sensor_data),
             n.create_subscription(TFMessage,'/world/cooperative_transport/pose/info',receive_poses,qos_profile_sensor_data)]
    goal=n.create_publisher(Vector3Stamped,'/leader/transport_goal',1)
    excitation=n.create_publisher(Wrench,'/experiment/follower/calibration_wrench',1)
    base_fixture_lock=n.create_publisher(Empty,'/experiment/follower/base_fixture/lock',1) if a.follower_base_fixture else None
    attach={r:n.create_publisher(Empty,f'/{r}/grasp/attach',1) for r in ('leader','follower')}
    report={'passed':False,'stage':a.stage,'fault':a.fault,'native_arm_stroke':a.native_arm_stroke,
            'dual_arm_only':a.dual_arm_only,
            'grasp_fixture':'native friction contact' if a.native_arm_stroke else 'fixed constraint',
            'follower_base_fixture':a.follower_base_fixture}
    constraint_stop_client=n.create_client(Trigger,'/transport/safety/stop')
    def native_grip_held(robot=None):
        return all(time.monotonic()-native_monitor_wall.get(r,0.)<.25
            and all(native_monitor.get(r,{}).get('own_contacts',{}).get(side) is True
                    and 8.<=native_monitor[r].get('own_forces_N',{}).get(side,float('inf'))<=26.
                    for side in ('left','right')) for r in ((robot,) if robot else ('leader','follower')))
    def vector(w):return [w.force.x,w.force.y,w.force.z,w.torque.x,w.torque.y,w.torque.z]
    def record():
        row={'wall_time':time.monotonic(),'sim_time':sim[0],'phase':phase[0],'mode':adm.get('mode','ARM_BASE' if a.stage=='AF-0' else 'ARM_ONLY'),
             'stop_reason':status.get('fault',''),'follower_reason':adm.get('reason',''), 'admittance_enabled':adm.get('enabled',False)}
        if a.follower_base_fixture:row['follower_base_fixture_state']=base_fixture_state[0]
        def fields(prefix,v):
            if v is not None:
                if isinstance(v,(list,tuple,np.ndarray)):
                    for i,x in enumerate(v):row[prefix+'_'+str(i)]=float(x)
                elif isinstance(v,(int,float,bool,str)):row[prefix]=v
        for k,v in adm.items(): fields('adm_'+k,v)
        if a.dual_arm_only:
            stages={'startup':'INIT','servo_probe':'INIT','servo_probe_readiness':'INIT',
                    'BASE_APPROACH':'BASE_APPROACH','BASE_STOP':'BASE_STOP','pre_grasp':'BASE_STOP',
                    'DUAL_GRASP':'DUAL_GRASP','explicit_monitor_handoff':'DUAL_GRASP',
                    'home':'SETTLE_AND_BIAS','SETTLE_AND_BIAS':'SETTLE_AND_BIAS','tare':'SETTLE_AND_BIAS',
                    'baseline':'ARM_ONLY','stationary_before_goal':'ARM_ONLY','leader_translation':'ARM_ONLY','HOLD':'HOLD'}
            record.experiment_state=stages.get(phase[0],getattr(record,'experiment_state','INIT'))
            row['experiment_state']='ABORT' if report.get('error') else record.experiment_state
        if a.native_arm_stroke:
            for k,v in native_leader_debug.items():fields('leader_comp_'+k,v)
        for k in ('reference_positions','feedback_positions','feedback_velocities','output_positions','error_positions'):
            fields('follower_jtc_'+k,jtc_state.get(k))
        row['follower_jtc_stamp_s']=jtc_state.get('stamp_s')
        for r in ('leader','follower'):
            fields(r+'_base_cmd',base.get(r));fields(r+'_rover_twist',output.get(r));fields(r+'_q_cmd',qcmd.get(r))
            fields(r+'_base_final_reference',final_output.get(r))
            row[r+'_state']=state.get(r,{}).get('state');row[r+'_grasp']=grasp.get(r)
            if a.native_arm_stroke:
                row[r+'_native_monitor_state']=native_monitor.get(r,{}).get('state')
                row[r+'_maintenance_state']=native_maintenance.get(r,{}).get('state')
                row[r+'_maintenance_reason']=native_maintenance.get(r,{}).get('reason')
                row[r+'_maintenance_heartbeat_age_wall_s']=native_monitor.get(r,{}).get('maintenance_heartbeat_age_wall_s')
                row[r+'_keeper_tick_gap_wall_s']=native_maintenance.get(r,{}).get('tick_gap_wall_s')
                for side in ('left','right'):
                    fields(r+'_native_'+side+'_force_N',native_monitor.get(r,{}).get('own_forces_N',{}).get(side))
                    fields(r+'_native_'+side+'_contact',native_monitor.get(r,{}).get('own_contacts',{}).get(side))
            if (r,'q') in latest:
                m=latest[r,'q'];q=dict(zip(m.name,m.position));v=dict(zip(m.name,m.velocity))
                row[r+'_joint_stamp_s']=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
                if r in models:
                    arm=[q[k] for k in models[r].names];fields(r+'_q_actual',arm);fields(r+'_qdot_actual',[v[k] for k in models[r].names])
                    fields(r+'_wheel_qdot_actual_rad_s',[v[k] for k in ('front_left_joint','front_right_joint','back_left_joint','back_right_joint')])
                    try:
                        tcp,sensor=models[r].frames(arm);fields(r+'_p_actual',tcp)
                        row[r+'_fk_valid']=True
                    except ValueError as exc:
                        # During native controller discovery an initial zero
                        # state can lie just outside a URDF bound. Preserve raw
                        # q and invalid-FK evidence, without treating startup as
                        # an armed transport sample. No q is clamped/fabricated.
                        if not a.native_arm_stroke or status.get('armed'):raise
                        row[r+'_fk_valid']=False;row[r+'_fk_error']=str(exc)
                    if not a.native_arm_stroke and r in poses and 'cooperative_payload' in poses and constraint_home:
                        anchor=ET.parse(folder/'world.sdf').find(f"world/model[@name='cooperative_payload']/link[@name='grasp_r{1 if r=='leader' else 2}_link']/pose")
                        xyzrpy=anchor.text.split();el=ET.Element('origin',xyz=' '.join(xyzrpy[:3]),rpy=' '.join(xyzrpy[3:]))
                        error=(pose_matrix(poses['cooperative_payload'])@origin(el))[:3,3]-(pose_matrix(poses[r])@sensor)[:3,3]
                        if r not in constraint_home:constraint_home[r]=error
                        row[r+'_constraint_error_m']=float(np.linalg.norm(error-constraint_home[r]))
            if (r,'ft') in latest:
                m=latest[r,'ft'];fields(r+'_wrench_raw',vector(m.wrench))
                row[r+'_wrench_stamp_s']=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
                row[r+'_wrench_frame']=m.header.frame_id
            if (r,'odom') in latest:
                m=latest[r,'odom'];fields(r+'_odom_pose',[m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.orientation.z,m.pose.pose.orientation.w])
                fields(r+'_odom_twist',[m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.angular.z])
                row[r+'_odom_stamp_s']=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
            for k in ('reaction_torque_Nm','peak_reaction_torque_Nm'):fields(r+'_'+k,gates.get(r,{}).get(k))
            if r in poses:
                fields(r+'_gt_position',[poses[r].translation.x,poses[r].translation.y,poses[r].translation.z])
                row[r+'_gt_receipt_sim_time']=pose_receipt_sim[r]
                row[r+'_gt_source_stamp_s']=pose_source_stamp[r]
                if a.dual_arm_only:
                    row[r+'_gt_yaw_rad']=2*np.arctan2(poses[r].rotation.z,poses[r].rotation.w)
                    if row.get(r+'_fk_valid'):
                        fields(r+'_tcp_world_evaluation',(pose_matrix(poses[r])@np.r_[row[r+'_p_actual_0'],row[r+'_p_actual_1'],row[r+'_p_actual_2'],1.])[:3])
        if 'cooperative_payload' in poses:
            payload=poses['cooperative_payload']
            fields('payload_model_gt_position',[payload.translation.x,payload.translation.y,payload.translation.z])
            fields('payload_model_gt_orientation_xyzw',[
                payload.rotation.x,payload.rotation.y,payload.rotation.z,payload.rotation.w])
            row['payload_model_gt_receipt_sim_time']=pose_receipt_sim['cooperative_payload']
            row['payload_model_gt_source_stamp_s']=pose_source_stamp['cooperative_payload']
        if filtered[0] is not None:
            fields('wrench_filtered',vector(filtered[0].wrench))
            row['filtered_wrench_stamp_s']=filtered[0].header.stamp.sec+filtered[0].header.stamp.nanosec*1e-9
        row['injected_constraint_error_m']=injected_constraint_error[0]
        error=max(row.get('leader_constraint_error_m',0.),row.get('follower_constraint_error_m',0.),injected_constraint_error[0])
        if a.native_arm_stroke and adm.get('enabled') and all(r in poses for r in ('leader','follower')):
            xy={r:np.array([poses[r].translation.x,poses[r].translation.y]) for r in ('leader','follower')}
            if not native_base_home:native_base_home.update({r:v.copy() for r,v in xy.items()})
            drift=float(max(np.linalg.norm(xy[r]-native_base_home[r]) for r in ('leader','follower')) if a.dual_arm_only else np.linalg.norm(xy['follower']-native_base_home['follower']))
            separation=float(np.linalg.norm(xy['leader']-xy['follower']))
            clearance=separation-sum(native_base_radii.values())
            row['follower_gt_drift_since_grasp_m']=drift
            row['base_collision_circle_clearance_m']=clearance
            if a.dual_arm_only and 'cooperative_payload' in poses:
                payload=np.array([poses['cooperative_payload'].translation.x,
                    poses['cooperative_payload'].translation.y,poses['cooperative_payload'].translation.z])
                for robot in ('leader','follower'):
                    if robot+'_tcp_world_evaluation_0' not in row:continue
                    tcp=np.array([row[robot+'_tcp_world_evaluation_'+str(i)] for i in range(3)])
                    offset=tcp-payload
                    if robot not in native_grasp_offsets:native_grasp_offsets[robot]=offset.copy()
                    residual=float(np.linalg.norm(offset-native_grasp_offsets[robot]))
                    row[robot+'_native_grasp_translation_residual_m']=residual
                    if status.get('armed') and residual>.002 and constraint_stop[0] is None:
                        event('native_grasp_translation_guard_stop','recorder',{'robot':robot,'residual_m':residual})
                        constraint_stop[0]=constraint_stop_client.call_async(Trigger.Request())
            if status.get('armed') and constraint_stop[0] is None and (drift>=.001 or clearance<=a.speed*.15):
                event('native_base_drift_or_collision_stop','recorder',{'drift_m':drift,'circle_clearance_m':clearance})
                constraint_stop[0]=constraint_stop_client.call_async(Trigger.Request())
        if status.get('armed') and error>.002 and constraint_stop[0] is None:
            event('constraint_guard_stop','recorder',error)
            constraint_stop[0]=constraint_stop_client.call_async(Trigger.Request())
        fields('applied_wrench',applied);rows.append(row)
    def step():
        if gui_process[0] is not None and gui_process[0].poll() is not None and not cleaning_up[0]:
            cancel_requested[0]='GUI_CLIENT_EXIT'
        if cancel_requested[0] and not cleaning_up[0]:
            raise RuntimeError('requested stop signal: '+cancel_requested[0])
        rclpy.spin_once(n,timeout_sec=.001);now=time.monotonic()
        if now-last_force[0]>.025:
            w=Wrench();w.force.x,w.force.y,w.force.z,w.torque.x,w.torque.y,w.torque.z=applied;excitation.publish(w);last_force[0]=now
        if now-last_record[0]>.02:
            last_record[0]=now;record()
    def wait(predicate,timeout=40.,healthy=False):
        end=time.monotonic()+timeout
        while not predicate():
            if healthy and status.get('fault'):raise RuntimeError('shared stop: '+status['fault'])
            if time.monotonic()>end:raise RuntimeError('timeout at '+phase[0]+' status='+str(status)+' states='+str(state))
            step()
    def simwait(seconds,healthy=True):
        begin=sim[0];wait(lambda:sim[0]-begin>=seconds,max(40.,seconds*30),healthy)
    def wallwait(seconds):
        end=time.monotonic()+seconds;wait(lambda:time.monotonic()>=end,seconds+5)
    def call(name,retry=False):
        c=n.create_client(Trigger,name);wait(c.service_is_ready,20.)
        end=time.monotonic()+10.
        while True:
            f=c.call_async(Trigger.Request());wait(f.done,10.);res=f.result()
            event('service',name,{'success':res.success,'message':res.message})
            if res.success or not retry or time.monotonic()>end:break
            wallwait(.2)
        n.destroy_client(c)
        if not res.success:raise RuntimeError(name+': '+res.message)
        return res
    def ensure_grippers():
        """Recover a stuck disarmed spawner, preserving its exact type/config.

        Some installed spawner parameter calls have no bounded reply wait.
        Recreate ONLY that experiment-owned client process; never unload an
        active controller or proceed with an unready gripper.
        """
        for robot in ('leader','follower'):
            c=n.create_client(ListControllers,f'/{robot}/controller_manager/list_controllers')
            wait(c.service_is_ready,15.)
            def controllers():
                f=c.call_async(ListControllers.Request())
                try:
                    wait(f.done,2.)
                    return {v.name:v.state for v in f.result().controller}
                except RuntimeError:
                    if not f.done():c.remove_pending_request(f);f.cancel()
                    return {}
            try:
                deadline=time.monotonic()+10.;value={}
                while time.monotonic()<deadline:
                    value=controllers()
                    if value.get('gripper_controller')=='active':break
                    wallwait(.2)
                if value.get('gripper_controller')=='active':continue
                if 'gripper_controller' in value:
                    raise RuntimeError('gripper loaded but not active; preserve state for review: '+robot)
                candidates=[]
                for pid in owned_children():
                    try:argv=[s.decode() for s in Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\0') if s]
                    except OSError:continue
                    manager=f'/{robot}/controller_manager'
                    if ('gripper_controller' in argv and manager in argv
                            and any(v.endswith('/controller_manager/spawner') for v in argv)):
                        candidates.append((pid,argv))
                if len(candidates)!=1:raise RuntimeError('exact owned gripper spawner unavailable: '+robot)
                pid,argv=candidates[0]
                event('disarmed_gripper_spawner_recovery',robot,{'pid':pid,'original_argv':argv})
                os.kill(pid,signal.SIGINT)
                def ended():
                    try:return Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split()[0]=='Z'
                    except OSError:return True
                wait(ended,3.)
                argv[argv.index('--service-call-timeout')+1]='3.0'
                for attempt in range(3):
                    proc=start(f'gripper_{robot}_retry_{attempt+1}',argv)
                    deadline=time.monotonic()+10.
                    while time.monotonic()<deadline:
                        value=controllers()
                        if value.get('gripper_controller')=='active':break
                        wallwait(.2)
                    if value.get('gripper_controller')=='active':break
                    if proc.poll() is None:
                        os.killpg(proc.pid,signal.SIGINT)
                        try:proc.wait(timeout=2.)
                        except subprocess.TimeoutExpired:
                            proc.kill();proc.wait(timeout=2.)
                    if 'gripper_controller' in value:break
                if value.get('gripper_controller')!='active':raise RuntimeError('bounded gripper startup recovery failed: '+robot)
                event('disarmed_gripper_spawner_recovered',robot,value)
            finally:n.destroy_client(c)
    def graph():
        phase[0]='disarmed_graph_audit'
        expected_node='follower_arm_base' if a.stage=='AF-0' else 'follower_arm_only'
        observed_subscriptions={}
        for name in (expected_node,'wrench_preprocessor'):
            def discovered(name=name):
                try:value=n.get_subscriber_names_and_types_by_node(name,'/follower')
                except NodeNameNonExistentError:return False
                if value:observed_subscriptions[name]=value
                return bool(value)
            wait(discovered,10.)
        result={'nodes':n.get_node_names_and_namespaces(),'topics':n.get_topic_names_and_types(), 'controller_states':{}}
        topics=('/follower/base_cmd','/follower/rover_twist','/follower/mecanum_drive_controller/reference_unstamped','/follower/arm_controller/joint_trajectory')
        if a.dual_arm_only:topics+=tuple(t.replace('/follower/','/leader/') for t in topics)
        for topic in topics:
            result[topic]=[{'node':x.node_namespace+'/'+x.node_name,'types':x.topic_type} for x in n.get_publishers_info_by_topic(topic)]
        for name in (expected_node,'wrench_preprocessor'):
            result['/follower/'+name+'/subscriptions']=observed_subscriptions[name]
        for r in ('leader','follower'):
            for typ,suffix in ((ListControllers,'list_controllers'),(ListHardwareInterfaces,'list_hardware_interfaces')):
                c=n.create_client(typ,f'/{r}/controller_manager/{suffix}');wait(c.service_is_ready,10.)
                for attempt in range(3):
                    f=c.call_async(typ.Request())
                    try:
                        wait(f.done,5.)
                        result[r+'/'+suffix]=str(f.result())
                        if suffix=='list_controllers':
                            result['controller_states'][r]={v.name:v.state for v in f.result().controller}
                        break
                    except RuntimeError:
                        if f.done():raise
                        c.remove_pending_request(f);f.cancel()
                        event('disarmed_read_retry',r+'/'+suffix,attempt+1)
                        if attempt==2:raise
                n.destroy_client(c)
        result['follower_servo_parameters']={}
        for node,names in [('gz_ros2_control',['position_proportional_gain']),('controller_manager',['update_rate']),
                           ('arm_controller',['open_loop_control','state_publish_rate','command_interfaces','state_interfaces'])]:
            c=n.create_client(GetParameters,f'/follower/{node}/get_parameters');wait(c.service_is_ready,10.)
            for attempt in range(3):
                future=c.call_async(GetParameters.Request(names=names))
                try:
                    wait(future.done,5.)
                    result['follower_servo_parameters'][node]={name:parameter_value_to_python(v) for name,v in zip(names,future.result().values)}
                    break
                except RuntimeError:
                    if future.done():raise
                    c.remove_pending_request(future);future.cancel()
                    if attempt==2:raise
            n.destroy_client(c)
        (out/'graph.json').write_text(json.dumps(result,indent=2))
        if a.servo_response:
            cert=json.loads(a.servo_response.read_text());params=result['follower_servo_parameters']
            if (cert['position_proportional_gain']!=params['gz_ros2_control']['position_proportional_gain']
                    or cert['controller_update_rate_hz']!=params['controller_manager']['update_rate']
                    or cert['open_loop_control']!=params['arm_controller']['open_loop_control']):
                raise ValueError('live actuator parameters differ from measured servo response')
        return result
    try:
        # Separate GUI client uses the existing side-view config. Server/world
        # generation stays byte-identical; GUI settings never touch URDF/SDF.
        if a.native_arm_stroke:
            from amir_native_arm_support import copy_native,measure_servo,stroke_plan
            copy_native(ROOT,folder,base_fixture=a.follower_base_fixture)
            if a.leader_base_distance_calibration:
                from amir_compliance_control.base_odometry_calibration import validate_distance_calibration
                a.leader_base_distance_calibration=a.leader_base_distance_calibration.resolve()
                measured=json.loads(a.leader_base_distance_calibration.read_text())
                distance_scale=validate_distance_calibration(measured,folder)
                if a.speed!=measured['raw_command_cap_m_s']:
                    raise ValueError('distance calibration is measured only for raw speed cap 0.005 m/s')
                (out/'leader_base_distance_calibration.json').write_text(json.dumps(measured,indent=2))
            native_base_radii.update({r:p['xy_radius_bound_m'] for r,p in
                json.loads((folder/'base_collision_envelopes.json').read_text()).items()})
            if a.dual_arm_only:
                from dual_arm_only_runtime import arm_plan
                plan=arm_plan(ROOT,a.distance)
            else:plan=stroke_plan((folder/'follower.urdf').read_text(),a.distance)
            (out/'stroke_plan.json').write_text(json.dumps(plan,indent=2))
        start('scene',['ros2','launch','cooperative_transport_bringup',
            ('compliance_native_arm_scene.launch.py' if a.follower_base_fixture else 'compliance_round_trip_scene.launch.py') if a.native_arm_stroke else 'compliance_cp6_fixture.launch.py',
            'run_directory:='+str(folder)]+([] if a.native_arm_stroke else ['headless:=true']))
        if a.gui:
            from ament_index_python.packages import get_package_share_directory
            gui_config=Path(get_package_share_directory('cooperative_transport_bringup'))/'config'/('compliance_dual_arm_gui.config' if a.dual_arm_only else 'compliance_contact_gui.config')
            if not gui_config.is_file():raise ValueError('existing side-view GUI config missing: '+str(gui_config))
            gui_process[0]=start('gui',['ign','gazebo','-g','--gui-config',str(gui_config),'--force-version','6'])
        wait(lambda:sim[0]>0 and all((r,k) in latest for r in ('leader','follower') for k in ('q','ft','odom')),110.)
        if a.native_arm_stroke:
            controller_isolation=verify_loaded_controller(controller_isolation,os.environ['IGN_PARTITION'])
            (out/'controller_isolation.json').write_text(json.dumps(controller_isolation,indent=2))
        ungrasped=a.stage in ('AF-0','AF-1')
        if not a.native_arm_stroke:relocate_fixture(folder,ungrasped)
        for r in ('leader','follower'):models[r]=LocalModel((folder/(r+'.urdf')).read_text())
        start('evaluation',['ros2','run','ros_gz_bridge','parameter_bridge',
            '/world/cooperative_transport/remove@ros_gz_interfaces/srv/DeleteEntity',
            *(['/world/cooperative_transport/set_pose@ros_gz_interfaces/srv/SetEntityPose'] if a.native_arm_stroke and not a.dual_arm_only else []),
            *(['/experiment/follower/base_fixture/lock@std_msgs/msg/Empty]gz.msgs.Empty',
                '/experiment/follower/base_fixture/state@std_msgs/msg/String[gz.msgs.StringMsg'] if a.follower_base_fixture else []),
            '/world/cooperative_transport/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            *([] if a.native_arm_stroke else [s for r in ('leader','follower') for s in (f'/{r}/grasp/attach@std_msgs/msg/Empty]gz.msgs.Empty',f'/{r}/grasp/state@std_msgs/msg/String[gz.msgs.StringMsg')])])
        if a.native_arm_stroke:
            ensure_grippers()
            measure_servo(node=n,folder=folder,out=out,latest=latest,jtc_samples=native_jtc_samples,sim=sim,
                wait=wait,simwait=simwait,phase=phase,event=event)
            if a.dual_arm_only:
                from dual_arm_single_probe import ungrasped_arm_probe
                ungrasped_arm_probe(node=n,folder=folder,out=out,latest=latest,sim=sim,wait=wait,phase=phase,poses=poses)
            from cp9_transport_preparation import prepare
            def native_wait(predicate,timeout=40.,check_fault=False):return wait(predicate,timeout,check_fault)
            def native_simwait(seconds,check_fault=False):return simwait(seconds,check_fault)
            def native_service(name,**_):return call(name,True)
            prepare(node=n,robots=('leader','follower'),generated=folder,output=out,start=start,wait=native_wait,
                sim_wait=native_simwait,wall_wait=wallwait,service=native_service,poses=poses,now=sim,
                phase=phase,prep=native_prep,monitor=native_monitor,report=report,grip_maintenance=True,
                physical_base_approach=a.dual_arm_only)
            if a.follower_base_fixture:
                phase[0]='lock_follower_base_after_grasp';last_lock=[float('-inf')]
                def locked():
                    if base_fixture_state[0]=='locked':return True
                    if time.monotonic()-last_lock[0]>.05:
                        base_fixture_lock.publish(Empty());last_lock[0]=time.monotonic()
                    return False
                wait(locked,10.)
                report['base_fixture_locked_after_native_grasp']=True
        start('safety',['ros2','launch','amir_compliance_control','guarded_safety.launch.py','run_directory:='+str(folder),
                        'follower_arm_only:='+str(a.stage!='AF-0').lower(),
                        'both_bases_zero:='+str(a.dual_arm_only).lower(),
                        'coherent_sensor_history:='+str(a.native_arm_stroke).lower(),
                        'simulation_source_watchdog:='+str(a.native_arm_stroke).lower()])
        wait(lambda:len(grasp)==2,45.)
        if a.stage!='AF-0':ensure_grippers()
        policy={'scope':'simulation_micro_test_only_not_hardware_ratings','force_source':str(SEED/'calibration/calibration.json'),
                'leader_goal_tolerance_m':.00001,
                'controller_parameters':{'force_increment_stop_N':.5,'torque_increment_stop_Nm':.05,
                    'enable_base_assist':False,'wrench_source_mode':'sensor'}}
        if a.profile in ('damped','pose_hold'):
            policy['leader_acceleration_m_s2']=.00005
            policy['controller_parameters'].update(virtual_mass=[20.,20.],virtual_damping=[1200.,1200.],virtual_stiffness=[50.,50.])
        if a.profile=='pose_hold':
            policy['leader_reference_acceleration_m_s2']=.00001
        if a.profile=='pose_hold_micro':
            policy['leader_acceleration_m_s2']=.01
            policy['leader_reference_acceleration_m_s2']=.005
            policy['controller_parameters'].update(virtual_mass=[2.,2.],virtual_damping=[34.6,34.6],virtual_stiffness=[50.,50.])
        if a.profile in ('pose_hold','pose_hold_micro'):policy['controller_parameters']['arm_nullspace_pose_hold']=True
        for key,v in [('virtual_mass',a.virtual_mass_kg),('virtual_damping',a.virtual_damping_ns_m),('virtual_stiffness',a.virtual_stiffness_n_m)]:
            if v is not None:policy['controller_parameters'][key]=[v,v]
        if a.arm_command_reference is not None:policy['controller_parameters']['arm_command_reference']=a.arm_command_reference
        if a.arm_horizon is not None:policy['controller_parameters']['arm_command_horizon_s']=a.arm_horizon
        if a.control_on_wrench is not None:policy['controller_parameters']['control_on_wrench']=a.control_on_wrench
        if a.wrench_cutoff_hz is not None:
            rate=float(ET.parse(folder/'follower.urdf').findtext("gazebo[@reference='Joint_5']/sensor[@type='force_torque']/update_rate"))
            if a.wrench_cutoff_hz>=rate/2:raise ValueError('wrench cutoff must be below audited F/T Nyquist frequency')
            policy['wrench_cutoff_hz']=a.wrench_cutoff_hz
            policy['audited_ft_rate_hz']=rate
        if a.wrench_on_measurement is not None:policy['wrench_process_on_measurement']=a.wrench_on_measurement
        if a.servo_response:
            a.servo_response=a.servo_response.resolve()
            policy['controller_parameters']['arm_servo_response_file']=str(a.servo_response)
            (out/'servo_response_used.json').write_bytes(a.servo_response.read_bytes())
        if a.native_arm_stroke:
            policy['scope']='simulation_native_stroke_only_not_hardware_ratings'
            policy['grip_safe_force_imbalance_hold']=True
            policy['force_source']=str(ROOT/'audits/whole_body_compliance_20261019/cp9_transport/metrology_02/calibration/calibration.json')
            policy['leader_arm_compensation']=True
            policy['controller_parameters']['simulation_source_watchdog']=True
            if a.leader_base_distance_calibration:
                policy['leader_base_distance_calibration_file']=str(out/'leader_base_distance_calibration.json')
                policy['leader_distance_metres_per_raw_odom_metre']=distance_scale
                report['leader_base_distance_calibrated']=True
                report['leader_distance_metres_per_raw_odom_metre']=distance_scale
            policy['leader_servo_response_file']=str(out/'leader_servo_response.json')
            policy['native_arm_stroke']=True
            policy['controller_parameters'].update(arm_servo_response_file=str(out/'follower_servo_response.json'),
                arm_command_reference='servo_trajectory',arm_displacement_stop_m=.26,
                displacement_limit_m=[.26,.012],torque_increment_stop_Nm=float(native_wrench_limits['torque_stop']),
                force_increment_stop_N=float(native_wrench_limits['planar_force_stop']))
            policy['native_wrench_limits_source']='cooperative_transport_control/config/friction_transport.yaml'
            policy['native_absolute_wrench_limits']={k:native_wrench_limits[k] for k in ('planar_force_stop','vertical_force_stop','torque_stop','fault_duration')}
            (out/'servo_response_used.json').write_bytes((out/'follower_servo_response.json').read_bytes())
            if a.dual_arm_only:
                policy['dual_arm_only']=True
                policy['controller_parameters'].update(virtual_stiffness=[0.,0.],force_deadband_N=.003,
                    joint_request_acceleration_rad_s2=.15,
                    arm_displacement_stop_m=.26,displacement_limit_m=[.26,.012])
        if a.arm_nullspace_pose_hold is not None:
            policy['controller_parameters']['arm_nullspace_pose_hold']=a.arm_nullspace_pose_hold
        if a.leader_acceleration is not None:policy['leader_acceleration_m_s2']=a.leader_acceleration
        if a.leader_reference_acceleration is not None:policy['leader_reference_acceleration_m_s2']=a.leader_reference_acceleration
        if a.fault=='saturation':policy['controller_parameters']['displacement_limit_m']=[.000015 if a.fault_during_motion else .00005]*2
        if a.stage=='AF-3' and a.fault in ('joint_margin','singularity','base_drift','dt'):
            policy['controller_parameters']['simulation_fault_injection_enabled']=True
        (out/'experiment_policy.json').write_text(json.dumps(policy,indent=2))
        if a.stage=='AF-0':
            control=['ros2','launch','amir_compliance_control','arm_base.launch.py','ungrasped_test:=true']
        else:
            control=['ros2','launch','amir_compliance_control','arm_only_af.launch.py','experiment_policy:='+str(out/'experiment_policy.json'),
                     'ungrasped_test:='+str(ungrasped).lower(),'leader_speed:='+str(a.speed),'leader_distance_limit:='+str(a.distance)]
        start('transport',control+['run_directory:='+str(folder),'calibration_file:='+str(folder/'calibration.json')])
        wait(lambda:status.get('ready') and all(state.get(r,{}).get('local_ready') for r in ('leader','follower')),120.)
        for r in ('leader','follower'):call(f'/{r}/transport/prepare',True)
        graph_info=graph()
        if a.stage!='AF-0' and not all(graph_info['controller_states'][r].get('gripper_controller')=='active'
                                      for r in ('leader','follower')):
            raise RuntimeError('both gripper controllers must be active before arming')
        if any(len(graph_info[t])!=1 for t in ('/follower/base_cmd','/follower/rover_twist',
                '/follower/mecanum_drive_controller/reference_unstamped','/follower/arm_controller/joint_trajectory')):
            raise RuntimeError('single-writer violation')
        if a.native_arm_stroke:
            wallwait(2.)
            for r in ('leader','follower'):call('/'+r+'/grasp/enable_maintenance',True)
            from amir_native_arm_support import wait_native_keep_ready
            wait_native_keep_ready(wait,native_monitor,native_maintenance,status,state,native_monitor_wall)
            (out/'native_prearm_readiness.json').write_text(json.dumps({'monitor':native_monitor,
                'maintenance':native_maintenance,'stable_window_wall_s':1.},indent=2))
        call('/transport/safety/arm',True)
        phase[0]='home';wait(lambda:all(state.get(r,{}).get('state')=='READY' for r in ('leader','follower')),35.,True)
        if not ungrasped:
            phase[0]='attach'
            if not a.native_arm_stroke:
                for _ in range(12):
                    for pub in attach.values():pub.publish(Empty())
                    wallwait(.05)
                wait_for_grasps(grasp,attach,wait)
            else:
                wait(lambda:all(grasp.get(r)=='attached' for r in ('leader','follower')),10.,True)
            c=n.create_client(DeleteEntity,'/world/cooperative_transport/remove');wait(c.service_is_ready)
            req=DeleteEntity.Request();req.entity.name='grasp_support';req.entity.type=Entity.MODEL
            f=c.call_async(req);wait(f.done);assert f.result().success;n.destroy_client(c)
            phase[0]='SETTLE_AND_BIAS' if a.dual_arm_only else 'static_grasp';simwait(10. if a.native_arm_stroke else 5.)
            constraint_home['_active']=True
        phase[0]='tare';simwait(1.);call('/follower/tare_wrench',True);wait(lambda:filter_state[0]=='READY',10.,True);simwait(.2)
        call('/follower/admittance/enable');wait(lambda:adm.get('enabled'),5.,True)
        phase[0]='baseline';simwait(.5)
        if a.stage in ('AF-0','AF-1'):
            signs=[]
            pulses=([(a.af1_force_axis,float(a.af1_force_sign))] if a.af1_force_axis is not None
                    else [(axis,sign) for axis in (0,1) for sign in (1.,-1.)])
            report['force_test_protocol']=('single isolated pulse from rest' if a.af1_force_axis is not None
                                           else 'four sequential signed pulses (legacy AF-1)')
            for axis,sign in pulses:
                phase[0]=f'force_{axis}_{sign}';applied[axis]=sign*.1;simwait(1.5)
                selected=[r for r in rows if r['phase']==phase[0]];last=selected[-1]
                actual=last.get(f'adm_e_arm_m_{axis}',last.get(f'follower_p_actual_{axis}',0.))
                signs.append({'axis':axis,'sign':sign,'measured_arm_component_m':actual})
                applied[axis]=0.;phase[0]='release';simwait(2.)
            report['signs']=signs
            if a.stage=='AF-1' and not all(s['sign']*s['measured_arm_component_m']>.00005 for s in signs):raise RuntimeError('measured FK force-direction test failed')
        elif a.stage=='AF-2':
            phase[0]='stationary_before_goal';wallwait(a.start_delay_s)
            phase[0]='leader_translation'
            waypoints=([d for d in (.1,.3) if d<a.distance-1e-12]+[a.distance]) if a.progressive_stroke else [a.distance]
            previous=0.;report['completed_waypoints']=[]
            for index,d in enumerate(waypoints,1):
                g=Vector3Stamped();g.header.stamp=Time(seconds=sim[0]).to_msg();g.header.frame_id='leader/base_footprint'
                g.vector.x=(d-previous)*(1 if a.direction=='forward' else -1)
                goal.publish(g);wait(lambda:state['leader'].get('accepted_goals')==index,5.,True)
                if a.dual_arm_only:
                    from dual_arm_workspace import wait_for_arm_completion
                    wait_for_arm_completion(wait,sim,lambda:state['leader'].get('completed_goals')==index,d-previous,a.speed)
                else:
                    wait(lambda:state['leader'].get('completed_goals')==index,max(60.,(d-previous)/a.speed*4+30),True)
                simwait(2. if a.progressive_stroke else 0.)
                checkpoint={'leader_target_total_m':d,'leader_odom_x_m':latest['leader','odom'].pose.pose.position.x,
                    'follower_arm_actual_m':adm.get('e_arm_m'),'grasp_held':native_grip_held() if a.native_arm_stroke else all(grasp.get(r)=='attached' for r in ('leader','follower')),
                    'sim_time_s':sim[0]}
                report['completed_waypoints'].append(checkpoint)
                (out/'waypoints.json').write_text(json.dumps(report['completed_waypoints'],indent=2))
                print('STROKE_WAYPOINT '+json.dumps(checkpoint),flush=True)
                previous=d
            phase[0]='HOLD' if a.dual_arm_only else 'hold';simwait(2. if a.dual_arm_only else 1.)
            report['leader_completed']=True
            report['leader_odom_peak_m_s']=odom_peak.get('leader',0.)
            report['leader_command_peak_m_s']=max(abs(e['value'][0]) for e in events if e['robot']=='leader' and e['kind']=='base_cmd')
            report['leader_velocity_cap_m_s']=a.speed
            report['required_speed_fraction']=a.require_speed_fraction
            movement=[r for r in rows if r['phase']=='leader_translation']
            report['leader_odom_distance_m']=movement[-1]['leader_odom_pose_0']-movement[0]['leader_odom_pose_0']
            payload_rows=[r for r in movement if 'payload_model_gt_position_0' in r]
            if payload_rows:
                report['payload_model_gt_displacement_xyz_m']=[payload_rows[-1]['payload_model_gt_position_'+str(i)]
                    -payload_rows[0]['payload_model_gt_position_'+str(i)] for i in range(3)]
                report['payload_model_gt_last_age_sim_s']=(payload_rows[-1]['sim_time']
                    -payload_rows[-1]['payload_model_gt_receipt_sim_time'])
                report['payload_model_gt_last_receipt_age_sim_s']=report['payload_model_gt_last_age_sim_s']
                report['payload_model_gt_source_time_available']=payload_rows[-1]['payload_model_gt_source_stamp_s']>0
                report['payload_model_gt_age_basis']='receipt in evaluator clock; zero TF source stamp is not reconstructed'
            payload_distance=a.distance*(.5 if a.native_arm_stroke and not a.dual_arm_only else 1.)
            leader_motion=report['leader_odom_distance_m']
            if a.dual_arm_only:
                leader_motion=movement[-1]['leader_p_actual_0']-movement[0]['leader_p_actual_0']
                report['leader_actual_arm_displacement_m']=leader_motion
                report['both_bases_max_abs_command']={r:max((abs(v) for e in events if e['robot']==r and e['kind']=='base_cmd' for v in e['value']),default=float('inf')) for r in ('leader','follower')}
                if any(report['both_bases_max_abs_command'].values()):raise RuntimeError('dual arm nonzero base command')
            if (not payload_rows or not 0<=report['payload_model_gt_last_age_sim_s']<=.1
                    or not .9*a.distance<=leader_motion*(1 if a.direction=='forward' else -1)<=1.1*a.distance
                    or not translation_progress(payload_distance,a.direction,payload_distance*(1 if a.direction=='forward' else -1),
                                                report['payload_model_gt_displacement_xyz_m'])):
                raise RuntimeError('signed leader/payload straight translation not demonstrated with fresh evaluation data')
            report['follower_measured_arm_xy_max_m']=max(np.hypot(r.get('adm_e_arm_m_0',0.),r.get('adm_e_arm_m_1',0.)) for r in movement)
            if report['follower_measured_arm_xy_max_m']<payload_distance*.5:raise RuntimeError('insufficient measured arm following')
            if min(report['leader_odom_peak_m_s'],report['leader_command_peak_m_s'])<a.speed*a.require_speed_fraction:
                raise RuntimeError('requested measured peak speed not reached')
        else:
            if a.fault_during_motion:
                phase[0]='leader_translation_before_fault'
                g=Vector3Stamped();g.header.stamp=Time(seconds=sim[0]).to_msg()
                g.header.frame_id='leader/base_footprint';g.vector.x=a.distance
                goal.publish(g);wait(lambda:state['leader'].get('accepted_goals')==1,5.,True)
                threshold=min(.000005 if a.fault=='saturation' else .00005,a.speed/4)
                actual_threshold=threshold/2
                if a.fault_min_speed_fraction:
                    threshold=max(threshold,a.speed*a.fault_min_speed_fraction)
                    actual_threshold=threshold
                wait(lambda:output.get('leader',[0.])[0]>=threshold
                     and latest['leader','odom'].twist.twist.linear.x>=actual_threshold,30.,True)
                report['leader_command_at_injection']=output['leader'].copy()
                report['leader_odom_velocity_at_injection']=latest['leader','odom'].twist.twist.linear.x
                report['fault_min_speed_fraction']=a.fault_min_speed_fraction
            phase[0]='fault_injected';injection=time.monotonic()
            if a.fault=='dropout':
                found=[]
                for x in Path('/proc').iterdir():
                    if not x.name.isdigit():continue
                    try:
                        args=(x/'cmdline').read_bytes();env=(x/'environ').read_bytes()
                        if b'/follower/ft_sensor@geometry_msgs/msg/WrenchStamped[' in args and ('IGN_PARTITION='+os.environ['IGN_PARTITION']).encode()+b'\0' in env:found.append(int(x.name))
                    except OSError:pass
                if len(found)!=1:raise RuntimeError('cannot identify owned sensor bridge '+str(found))
                os.kill(found[0],signal.SIGSTOP);paused.extend(found)
            elif a.fault in ('force','saturation'):
                applied[0]=(-1. if a.fault_during_motion else 1.)*(.6 if a.fault=='force' else .1)
            elif a.fault=='constraint':
                report['constraint_injection']='recorder constraint-error guard synthetic 3 mm; physical weld retained'
                injected_constraint_error[0]=.003
            elif a.fault in ('joint_margin','singularity','base_drift','dt'):
                report['fault_injection_scope']='synthetic local guard input; physical q/odom/FK and Gazebo geometry unchanged'
                call('/follower/arm_only/test/'+a.fault)
            else:raise ValueError('AF-3 requires fault')
            wait(lambda:all(state.get(r,{}).get('state')=='SAFE_STOP' and max(abs(x) for x in output.get(r,[1.]))<1e-12 for r in ('leader','follower')),3.)
            stopped=time.monotonic()
            report['injection_to_stop_wall_s']=stopped-injection
            report['stop_latency_wall_s']=stopped-injection
            report['stop_latency_basis']='fault injection to both endpoints SAFE_STOP and both final commands zero'
            if a.fault=='saturation':
                wait(lambda:adm.get('fault_observed_reason')=='ARM_ONLY admittance displacement saturation',1.)
                onset=float(adm['fault_observed_wall_time_s'])
                report['condition_to_stop_wall_s']=max(0.,stopped-onset)
                report['injection_to_condition_wall_s']=onset-injection
                report['stop_latency_wall_s']=report['condition_to_stop_wall_s']
                report['stop_latency_basis']='admittance boundary detection to both endpoints SAFE_STOP and both final commands zero'
            if report['stop_latency_wall_s']>.4:raise RuntimeError('stop latency >0.4s')
            applied[:]=[0.]*6
            for pid in paused:os.kill(pid,signal.SIGCONT)
            paused.clear();simwait(.5,False)
            if not all(grasp.get(r)=='attached' for r in ('leader','follower')):raise RuntimeError('grasp lost on stop')
            report['grasp_held']=True
        zeros=[e for e in events if e['robot']=='follower' and e['kind'] in ('base_cmd','rover_twist','base_final_reference')]
        if a.dual_arm_only:
            zeros=[e for e in zeros if e['sim_time']>=report['physical_base_approach']['sim_stopped_s']]
        report['follower_zero_command_samples']=len(zeros)
        report['follower_max_abs_base_cmd']=max([abs(x) for e in zeros for x in e['value']],default=0.)
        if a.stage=='AF-0':
            report['simultaneous_base_arm_reproduced']=report['follower_max_abs_base_cmd']>0 and any(e['kind']=='arm_cmd' and e['robot']=='follower' for e in events)
            if not report['simultaneous_base_arm_reproduced']:raise RuntimeError('baseline simultaneous motion not reproduced')
        elif not zeros or report['follower_max_abs_base_cmd']!=0.:raise RuntimeError('ARM_ONLY nonzero base command')
        if a.stage not in ('AF-0','AF-1') and any(r.get('leader_constraint_error_m',0.)>.002 or r.get('follower_constraint_error_m',0.)>.002 for r in rows):raise RuntimeError('grasp constraint error >2mm')
        report['passed']=True
    except Exception as exc:
        report['error']=str(exc)
    finally:
        cleaning_up[0]=True
        report['cancelled']=cancel_requested[0] is not None
        if cancel_requested[0]:report['requested_stop_signal']=cancel_requested[0]
        applied[:]=[0.]*6
        for pid in paused:
            try:os.kill(pid,signal.SIGCONT)
            except ProcessLookupError:pass
        if any(state.get(r,{}).get('prepared') for r in ('leader','follower')):
            try:
                if status.get('armed'):call('/transport/safety/stop')
                simwait(.7,False)
            except Exception as exc:report['cleanup_stop_error']=str(exc)
        report['post_stop_controllers']={}
        for r in ('leader','follower'):
            c=n.create_client(ListControllers,f'/{r}/controller_manager/list_controllers')
            try:
                if not c.service_is_ready():continue
                for attempt in range(2):
                    f=c.call_async(ListControllers.Request())
                    try:
                        wait(f.done,3.);report['post_stop_controllers'][r]={v.name:v.state for v in f.result().controller};break
                    except RuntimeError:
                        if not f.done():c.remove_pending_request(f);f.cancel()
            finally:n.destroy_client(c)
        late=[r for r in rows if r['sim_time']>=sim[0]-.2]
        span={r:[float(np.ptp([x.get(f'{r}_q_actual_{i}',float('nan')) for x in late])) for i in range(5)]
              for r in ('leader','follower')} if late else {}
        report['post_stop_joint_span_rad']=span
        report['post_stop_odom_twist']={r:[latest[r,'odom'].twist.twist.linear.x,
            latest[r,'odom'].twist.twist.linear.y,latest[r,'odom'].twist.twist.angular.z]
            for r in ('leader','follower') if (r,'odom') in latest}
        report['safe_stop_verified']=(all(state.get(r,{}).get('state')=='SAFE_STOP'
            and max(abs(x) for x in output.get(r,[1.]))<1e-12 for r in ('leader','follower'))
            and bool(span) and all(np.isfinite(v) and v<1e-5 for s in span.values() for v in s))
        report['base_stop_evidence']={}
        for r in ('leader','follower'):
            if (r,'q') not in latest or (r,'odom') not in latest:continue
            qmsg=latest[r,'q'];vel=dict(zip(qmsg.name,qmsg.velocity))
            wheel=max(abs(vel.get(k,float('inf'))) for k in ('front_left_joint','front_right_joint','back_left_joint','back_right_joint'))
            qage=sim[0]-qmsg.header.stamp.sec-qmsg.header.stamp.nanosec*1e-9
            omsg=latest[r,'odom'];oage=sim[0]-omsg.header.stamp.sec-omsg.header.stamp.nanosec*1e-9
            gtspan=float(np.linalg.norm([np.ptp([float(x.get(r+'_gt_position_'+str(i),float('nan'))) for x in late]) for i in range(3)])) if late else float('inf')
            gt_age=sim[0]-pose_receipt_sim.get(r,float('-inf'))
            inactive=report['post_stop_controllers'].get(r,{}).get('mecanum_drive_controller')=='inactive'
            fresh_odom=-.02<=oage<=.05
            evidence={'odom_fresh':fresh_odom,'odom_age_sim_s':oage,'wheel_velocity_peak_rad_s':wheel,
                'wheel_sample_age_sim_s':qage,'ground_truth_position_span_m':gtspan,
                'ground_truth_receipt_age_sim_s':gt_age,'base_controller_inactive':inactive}
            evidence['ground_truth_span_bound_m']=.001 if a.native_arm_stroke else 1e-6
            evidence['stopped']=(wheel<1e-4 and -.02<=qage<=.05 and gtspan<evidence['ground_truth_span_bound_m'] and 0<=gt_age<=.1
                and (all(abs(v)<1e-6 for v in report['post_stop_odom_twist'][r]) if fresh_odom else inactive))
            report['base_stop_evidence'][r]=evidence
        report['safe_stop_verified']=report['safe_stop_verified'] and len(report['base_stop_evidence'])==2 and all(
            e['stopped'] for e in report['base_stop_evidence'].values())
        report['grasp_held_after_stop']=native_grip_held() if a.native_arm_stroke else all(grasp.get(r)=='attached' for r in ('leader','follower'))
        if a.native_arm_stroke:
            report['last_native_monitor']=native_monitor.copy()
            report['last_native_maintenance']=native_maintenance.copy()
            report['native_stop_grasp_basis']='fresh bilateral finger contact and 8..26N inherited grip band; latched FAULT/detached does not mean physical release'
        report['gripper_hold_verified']=all(report['post_stop_controllers'].get(r,{}).get('gripper_controller')=='active'
                                             for r in ('leader','follower'))
        if report['passed'] and (not report['safe_stop_verified'] or
                (a.stage in ('AF-2','AF-3') and (not report['grasp_held_after_stop'] or not report['gripper_hold_verified']))):
            report['passed']=False;report['error']='post-stop arm/base hold verification failed'
        report.update(final_safety=status.copy(),final_endpoints=state.copy(),final_grasp=grasp.copy(),rows=len(rows),
                      final_gate_debug=gates.copy(),final_interlock_debug=interlocks.copy())
        report.update(leader_odom_peak_m_s=odom_peak.get('leader',0.),
            leader_command_peak_m_s=max((abs(e['value'][0]) for e in events if e['robot']=='leader' and e['kind']=='base_cmd'),default=0.),
            leader_velocity_cap_m_s=a.speed,required_speed_fraction=a.require_speed_fraction,
            requested_distance_m=a.distance,direction=a.direction,signed_leader_goal_m=signed_distance)
        if rows:
            keys=sorted(set().union(*(r.keys() for r in rows)))
            with (out/'timeseries.csv').open('w') as f:
                w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows(rows)
        trace.close();jtc_trace.close();local_trace.close()
        if a.keep_gui_open and report['passed'] and not cancel_requested[0]:
            # Save evidence before waiting; observation must not grow the high-
            # rate traces indefinitely. No controller is rearmed or restarted.
            gui_holding[0]=True
            pause_error=None
            if a.pause_gui_after_stop:
                try:
                    proof=pause_world_for_review()
                    # Drain feedback queued before the pause, then verify the
                    # simulator clock is stationary before saving its value.
                    stable_since=time.monotonic();previous_clock=sim[0];deadline=stable_since+3.
                    while time.monotonic()-stable_since<.3:
                        if time.monotonic()>deadline:raise RuntimeError('review pause clock did not settle')
                        pump_feedback(lambda timeout:rclpy.spin_once(n,timeout_sec=timeout))
                        if sim[0]!=previous_clock:previous_clock=sim[0];stable_since=time.monotonic()
                    proof['frozen_sim_time_s']=sim[0]
                    (out/'paused_review_proof.json').write_text(json.dumps(proof,indent=2))
                    report['gui_review_physics_paused']=True
                except Exception as exc:
                    pause_error=exc;report['passed']=False
                    report['gui_hold_error']='review pause failed: '+str(exc)
            snapshot={**report,'cleanup_pending':True,'gui_hold':True,'runner_pid':os.getpid()}
            (out/'held_result.json').write_text(json.dumps(snapshot,indent=2))
            print(('GUI_PAUSED_REVIEW_READY: measured stop/grip verified, physics paused; ' if a.pause_gui_after_stop
                   else 'GUI_HOLD_READY: robots stopped, grasp held; ')+'Ctrl+C closes the scene. '+str(out),flush=True)
            held_q={r:dict(zip(latest[r,'q'].name,latest[r,'q'].position)) for r in ('leader','follower')}
            held_base={r:np.array([poses[r].translation.x,poses[r].translation.y]) for r in ('leader','follower')}
            last_clock=[sim[0],time.monotonic()]
            def hold_health():
                now=time.monotonic()
                if sim[0]>last_clock[0]:last_clock[:]=[sim[0],now]
                if now-last_clock[1]>1.:raise RuntimeError('GUI hold clock stopped')
                ages=coherent_source_ages(sim[0],{r:latest[r,'q'].header.stamp.sec+
                    latest[r,'q'].header.stamp.nanosec*1e-9 for r in ('leader','follower')},
                    max_skew=.25 if a.native_arm_stroke else .1)
                for r in ('leader','follower'):
                    if (not native_grip_held(r) if a.native_arm_stroke else grasp.get(r)!='attached' or now-grasp_wall.get(r,0)>1.):
                        raise RuntimeError('GUI hold grasp missing: '+r)
                    msg=latest[r,'q'];q=dict(zip(msg.name,msg.position));vel=dict(zip(msg.name,msg.velocity))
                    if ages[r]>(.25 if a.native_arm_stroke else .1):raise RuntimeError(f'GUI hold joint feedback stale: {r}')
                    if any(abs(q[name]-held_q[r][name])>=1e-5 for name in models[r].names):
                        raise RuntimeError('GUI hold arm moved: '+r)
                    if max(abs(vel.get(k,float('inf'))) for k in ('front_left_joint','front_right_joint','back_left_joint','back_right_joint'))>=1e-4:
                        raise RuntimeError('GUI hold wheels moving: '+r)
                    xy=np.array([poses[r].translation.x,poses[r].translation.y])
                    if np.linalg.norm(xy-held_base[r])>=.001 or not 0<=sim[0]-pose_receipt_sim[r]<=(.25 if a.native_arm_stroke else .1):
                        raise RuntimeError('GUI hold physical base drift/stale pose: '+r)
                    if max(abs(x) for x in output.get(r,[1.]))!=0. or state.get(r,{}).get('state')!='SAFE_STOP':
                        raise RuntimeError('GUI hold command/state invalid: '+r)
            frozen_sim=sim[0]
            def paused_health():
                if pause_error is not None:raise RuntimeError(str(pause_error))
                if sim[0]!=frozen_sim:raise RuntimeError('paused review was unpaused')
                for process in controller_isolation['loaded_processes']:
                    if not Path('/proc/'+str(process['pid'])).exists():raise RuntimeError('paused review Gazebo server exited')
            try:
                hold_until_stopped(pump=lambda:pump_feedback(lambda timeout:rclpy.spin_once(n,timeout_sec=timeout)),
                    health=paused_health if a.pause_gui_after_stop else hold_health,
                    cancelled=lambda:cancel_requested[0] is not None or gui_process[0].poll() is not None,
                    status_file=out/'gui_hold_status.json',
                    metadata={'runner_pid':os.getpid(),'ROS_DOMAIN_ID':os.getenv('ROS_DOMAIN_ID'),
                              'IGN_PARTITION':os.getenv('IGN_PARTITION'),
                              'physics_paused':a.pause_gui_after_stop,'live_grasp_monitor':not a.pause_gui_after_stop},
                    holding_state='SAFE_STOP_GUI_PAUSED_REVIEW' if a.pause_gui_after_stop else 'SAFE_STOP_GUI_HOLD')
                report['gui_hold_closed_by_signal']=cancel_requested[0] or 'GUI_CLIENT_EXIT'
            except Exception as exc:
                report['passed']=False;report['gui_hold_error']=str(exc)
                report['gui_hold_fault_context']={'sim_time_s':sim[0],
                    'native_monitor':native_monitor.copy(),
                    'native_monitor_receipt_age_wall_s':{r:time.monotonic()-native_monitor_wall.get(r,0.) for r in ('leader','follower')}}
                try:call('/transport/safety/stop')
                except Exception:pass
            report['gui_hold_monitor_completed']=True
        for proc in reversed(processes):
            if proc.poll() is None:
                try:os.killpg(proc.pid,signal.SIGINT);proc.wait(timeout=12.)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid,signal.SIGTERM)
                    try:proc.wait(timeout=3.)
                    except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait(timeout=3.)
                except ProcessLookupError:pass
        cleanup=[]
        for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGKILL):
            children=owned_children()
            if not children:break
            cleanup.append({'signal':sig.name,'pids':children})
            for pid in children:
                try:os.kill(pid,sig)
                except ProcessLookupError:pass
            deadline=time.monotonic()+1.
            while owned_children() and time.monotonic()<deadline:time.sleep(.05)
        report['cleanup_escalations']=cleanup
        report['remaining_owned_pids']=owned_children()
        if report['remaining_owned_pids']:
            report['passed']=False;report['cleanup_error']='owned child process survived teardown'
        (out/'result.json').write_text(json.dumps(report,indent=2))
        for f in logs:f.close()
        n.destroy_node();rclpy.try_shutdown()
        for s,handler in old_signals.items():signal.signal(s,handler)
    print(json.dumps(report,indent=2));return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())

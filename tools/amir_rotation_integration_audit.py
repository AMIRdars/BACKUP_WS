#!/usr/bin/env python3
"""Chapter 23 O0 preflight with explicit O1, O2 and qualified A/B/C/D benches.

Default live mode sends no motion goal. O1 requires a bounded passive-angle
option; O2 requires measured qualification; A/B/C/D additionally requires an
explicit comparison mode and existing base policy, and sends goals only to
the leader. All benches use a NEW fixed-grasp fixture and enable-refusal
preflight. Missing gates are never replaced with fabricated qualification.
"""
import argparse
from collections import deque
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
import yaml

from amir_compliance_control.rotation_observer import ProjectedYaw,wrap_pi
from amir_compliance_control.rotation_recentering import MODES


def canonical_model(file):
    root=ET.parse(file).getroot()
    for e in root.findall('gazebo/plugin/parameters'):e.text='CONTROLLER_FILE_PATH'
    return ET.tostring(root,encoding='unicode')


def require_completed_passive_range(delta_deg,passive):
    """A requested expansion needs an actual completed, held O1 bench.

    This evaluator bound is never sent to the follower as a target angle.
    Its own observer still enforces the measured joint/noise region.
    """
    if any(passive.get(k) is not True for k in ('passed','certificate_issued',
            'positive_negative_programs_completed','stop_and_hold_confirmed','both_fixed_grasps_retained')):
        raise ValueError('completed positive/negative O1, certificate and stop/hold evidence required')
    measured=float(passive.get('delta_deg',float('nan')))
    if not np.isfinite([delta_deg,measured]).all() or not 0.<delta_deg<=measured<=.1:
        raise ValueError('comparison angle exceeds completed O1 passive trial')


def passive_profile_duration(requested,previous):
    duration=previous if requested is None else requested
    if not np.isfinite([duration,previous]).all() or not 0.<previous<=duration:
        raise ValueError('passive duration must retain or slow the existing leader profile')
    return float(duration)


def require_completed_active_step(delta_deg,previous,mode,qualification):
    """A bounded next step needs completed transport and exact local symmetry.

    This extends the experiment command range, not any physical or watchdog
    limit. Threefold steps match the requested 5 -> 15 -> ... progression.
    """
    if (any(previous.get(k) is not True for k in ('passed','positive_negative_programs_completed',
            'stop_and_hold_confirmed','both_fixed_grasps_retained')) or previous.get('completed_goals')!=2
            or previous.get('mode')!=mode or previous.get('grasp_losses')):
        raise ValueError('completed same-mode active transport and stop/hold evidence required')
    old=float(previous.get('delta_deg',float('nan')))
    if not np.isfinite([delta_deg,old]).all() or not 0.<old<=delta_deg<=min(90.,3*old):
        raise ValueError('active expansion exceeds the completed threefold step or 90 degree endpoint')
    if not qualification.get('yaw_equivariance_proof'):
        raise ValueError('exact first-axis yaw equivariance certificate required for active expansion')


def conservative_leader_limits(params,overrides):
    allowed={'rotation_max_angular_speed','rotation_max_base_speed','rotation_max_grasp_speed',
             'rotation_max_angular_acceleration','rotation_max_base_acceleration'}
    if not overrides or set(overrides)-allowed:
        raise ValueError('leader planning overrides may only reduce existing speed/acceleration limits')
    for key,value in overrides.items():
        if not math.isfinite(value) or not 0.<value<=params[key]:
            raise ValueError('leader planning speed/acceleration limit increase refused')
    return dict(params,**overrides)


def require_qualified_ablation(delta_deg,reference,mode,qualification):
    """Compare A/B/C at an endpoint already transported by qualified WB.

    An ablation may stop on load; it never disables that load guard or
    establishes autonomous rotation merely by issuing the same command.
    """
    if mode not in ('XY_ONLY','XY_YAW','XY_YAW_COMP') or reference.get('mode')!='WHOLE_BODY':
        raise ValueError('ablation needs A/B/C and a completed WHOLE_BODY reference')
    if (any(reference.get(k) is not True for k in ('passed','positive_negative_programs_completed',
            'stop_and_hold_confirmed','both_fixed_grasps_retained')) or reference.get('completed_goals')!=2
            or reference.get('grasp_losses')):
        raise ValueError('completed reference transport and stop/hold evidence required')
    bound=float(reference.get('delta_deg',float('nan')))
    if not np.isfinite([delta_deg,bound]).all() or not 0.<delta_deg<=bound<=90.:
        raise ValueError('ablation endpoint exceeds completed reference transport')
    if not qualification.get('yaw_equivariance_proof'):
        raise ValueError('exact first-axis yaw equivariance certificate required')


def offline(a):
    xml=(a.source_run/'follower.urdf').read_text()
    g=ProjectedYaw(xml,orientation_envelope_rad=.01,joint_margin_rad=.01,heading_frame=a.yaw_heading_frame)
    root=ET.fromstring(xml)
    home=np.array([float(root.find("ros2_control/joint[@name='"+n+"']/state_interface[@name='position']/param").text) for n in g.model.names])
    measured=g.measure(home)
    sensitivity=[]
    for sign in (-1.,1.):
        q=home.copy();q[4]+=sign*1e-6
        sensitivity.append(dict(joint5_test_perturbation_rad=sign*1e-6,**g.measure(q)))
    samples=[]
    for line in a.source_samples.open():
        row=json.loads(line)
        if row['phase']=='baseline':
            q=[row['follower_q'+str(i)+'_rad'] for i in range(1,6)]
            samples.append(dict(timestamp=row['t_s'],**g.measure(q)))
    if len(samples)<2:raise RuntimeError('measured stationary FK history required')
    unwrapped=np.unwrap([s['psi_ee_B'] for s in samples])
    report=dict(home_q_rad=home.tolist(),home=measured,sensitivity=sensitivity,
        sensitivity_is_numerical_not_sensor_resolution=True,
        frame_transform_footprint_T_base_link=g.footprint_T_base.tolist(),
        orientation_envelope_rad=.01,joint_margin_rad=.01,
        policy_origin='existing CP-8R-3 declared rotation_error_limit_rad/joint_margin_rad',
        measured_static_sample_count=len(samples),measured_static_yaw_variance_rad2=float(np.var(unwrapped,ddof=1)),
        source_urdf_sha256=hashlib.sha256(xml.encode()).hexdigest(),
        source_samples=str(a.source_samples),source_samples_sha256=hashlib.sha256(a.source_samples.read_bytes()).hexdigest(),
        Gate_0_passed=all(s['geometry_valid'] for s in samples),
        yaw_heading_frame=g.heading_frame,geometry_signature=g.geometry_signature,
        O1='NOT RUN: separate passive trial required',
        R_A_to_R_E='NOT RUN: passive positive/negative qualification unavailable',
        qualification_certificate_issued=False)
    (a.output/'offline_geometry.json').write_text(json.dumps(report,indent=2)+'\n')
    with (a.output/'offline_observer.csv').open('w',newline='') as f:
        keys=('timestamp','psi_ee_B','projection_norm','vertical_clearance_rad','joint_margin_rad','geometry_valid','valid_reason')
        w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows({k:s[k] for k in keys} for s in samples)
    return report


def live(a):
    import rclpy
    from std_msgs.msg import Bool,Empty,String
    from std_srvs.srv import Trigger
    from sensor_msgs.msg import JointState
    from rclpy.qos import qos_profile_sensor_data
    from amir_cp8r3_process import reap_partition
    partition,domain=os.getenv('IGN_PARTITION',''),os.getenv('ROS_DOMAIN_ID','')
    if os.getenv('ROS_DISTRO')!='humble' or not partition.startswith('amir_cp8r3_ch23_') or not domain:
        raise ValueError('isolated Humble experiment domain/partition required')
    generated=a.output/'generated';generated.mkdir()
    processes,logs=[],[]
    def start(name,cmd):
        log=(a.output/(name+'.log')).open('w');logs.append(log)
        p=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);processes.append(p)
    rclpy.init(args=[]);node=rclpy.create_node('rotation_integration_evaluator')
    # All observations are streamed to disk. Long transport does not need
    # unbounded Python copies of every snapshot or output command.
    observations=deque(maxlen=2048) if a.comparison_mode else []
    grasp,commands,ready={},deque(maxlen=2048),deque(maxlen=2048)
    rows=(a.output/'observer.jsonl').open('w',buffering=1)
    grasp_events=(a.output/'grasp_events.jsonl').open('w',buffering=1)
    def observed(msg):
        data=json.loads(msg.data);observations.append(data);rows.write(json.dumps(data)+'\n')
    node.create_subscription(String,'/follower/rotation/observation',observed,10)
    node.create_subscription(Bool,'/transport/rotation_follower_ready',lambda m:ready.append(m.data),10)
    for robot in ('leader','follower'):
        def grasped(msg,r=robot):
            grasp[r]=msg.data
            grasp_events.write(json.dumps(dict(robot=r,state=msg.data,wall_s=time.monotonic(),
                latest_observer_sim_s=observations[-1]['timestamp'] if observations else None))+'\n')
        node.create_subscription(String,'/'+robot+'/grasp/state',grasped,10)
    latest_joints={}
    node.create_subscription(JointState,'/follower/joint_states',lambda m:latest_joints.__setitem__('joints',m),qos_profile_sensor_data)
    from geometry_msgs.msg import TwistStamped
    node.create_subscription(TwistStamped,'/follower/base_cmd',lambda m:commands.append([m.twist.linear.x,m.twist.linear.y,m.twist.angular.z]),10)
    def wait(predicate,timeout=45.):
        end=time.monotonic()+timeout
        while not predicate():
            if time.monotonic()>end:raise RuntimeError('static observer audit timeout')
            if any(p.poll() is not None for p in processes):raise RuntimeError('owned audit process exited')
            rclpy.spin_once(node,timeout_sec=.01)
    def service(name):
        client=node.create_client(Trigger,name);wait(client.service_is_ready)
        future=client.call_async(Trigger.Request());wait(future.done)
        response=future.result();node.destroy_client(client)
        return dict(success=response.success,message=response.message)
    result=dict(passed=False,experiment='supported stationary fixed grasp; NO rotation goal',
                ROS_DISTRO=os.getenv('ROS_DISTRO'),ROS_DOMAIN_ID=domain,IGN_PARTITION=partition)
    result['grasp_state_publish_period_sim_s']=a.grasp_state_publish_period_s
    from ament_index_python.packages import get_package_prefix
    grasp_lib=Path(get_package_prefix('amir_follower_gazebo'))/'lib/libamir_grasp_heartbeat-system.so'
    result['grasp_plugin_sha256']=hashlib.sha256(grasp_lib.read_bytes()).hexdigest()
    try:
        start('scene',['ros2','launch','cooperative_transport_bringup','compliance_cp6_fixture.launch.py',
            'run_directory:='+str(generated),'grasp_state_publish_period_s:='+str(a.grasp_state_publish_period_s)])
        start('grasp_bridge',['ros2','run','ros_gz_bridge','parameter_bridge',
            *['/'+r+'/grasp/state@std_msgs/msg/String[gz.msgs.StringMsg' for r in ('leader','follower')],
            *['/'+r+'/grasp/attach@std_msgs/msg/Empty]gz.msgs.Empty' for r in ('leader','follower')]])
        wait(lambda:(generated/'follower.urdf').exists() and 'joints' in latest_joints,120.)
        controllers=('joint_state_broadcaster','arm_controller','mecanum_drive_controller','gripper_controller')
        last_log_check=[0.]
        def spawners_finished():
            if time.monotonic()-last_log_check[0]<.2:return False
            last_log_check[0]=time.monotonic()
            lines=(a.output/'scene.log').read_text().splitlines()
            return all(any('['+r+'.spawner_'+name+']' in line and 'Configured and activated' in line for line in lines)
                       for r in ('leader','follower') for name in controllers)
        # Do not compete with startup spawners or cancel in-flight manager
        # replies. First observe their activation acknowledgements, then make
        # one read-only confirmation request per manager.
        wait(spawners_finished,180.)
        from controller_manager_msgs.srv import ListControllers
        startup={}
        for robot in ('leader','follower'):
            client=node.create_client(ListControllers,'/'+robot+'/controller_manager/list_controllers')
            wait(client.service_is_ready)
            future=client.call_async(ListControllers.Request());wait(future.done,45.)
            active={c.name:c.state for c in future.result().controller};startup[robot]=active
            if not all(active.get(n)=='active' for n in controllers):
                raise RuntimeError('original controller not active: '+robot)
            node.destroy_client(client)
        result['startup_controllers']=startup
        if canonical_model(generated/'follower.urdf')!=canonical_model(a.source_run/'follower.urdf'):
            raise RuntimeError('fixture model changed beyond controller path; calibration transfer refused')
        cert=json.loads(a.sensor_certificate.read_text())
        if cert['urdf_sha256']!=hashlib.sha256((a.source_run/'follower.urdf').read_bytes()).hexdigest():
            raise RuntimeError('source sensor certificate/model mismatch')
        cert['urdf_sha256']=hashlib.sha256((generated/'follower.urdf').read_bytes()).hexdigest()
        cert['transfer_proof']='canonical XML equality except controller parameter path'
        calibration=a.output/'sensor_calibration.json';calibration.write_text(json.dumps(cert,indent=2)+'\n')
        old=yaml.safe_load(a.follower_policy.read_text())
        params=next(iter(old.values()))['ros__parameters']
        params.update(base_force_acceleration_gains=[0.,0.,0.],base_force_velocity_gains=[0.,0.,0.],
                      rotation_error_limit_rad=.01,joint_margin_rad=.01,arm_yaw_admittance_enabled=False)
        if a.passive_delta_deg:
            params['base_gains']=[0.,0.,0.]
            params['passive_yaw_calibration']=a.passive_arm_yaw
            params['arm_yaw_admittance_enabled']=a.passive_arm_yaw
        qualification=None
        if a.o2_qualification_run:
            from amir_compliance_control.rotation_observer import read_qualification
            source=a.o2_qualification_run
            if canonical_model(source/'generated/follower.urdf')!=canonical_model(generated/'follower.urdf'):
                raise ValueError('O2 qualification model changed beyond controller path')
            geometry=ProjectedYaw((source/'generated/follower.urdf').read_text(),orientation_envelope_rad=.01,
                joint_margin_rad=.01,heading_frame='grasp_heading')
            source_qualification=a.o2_qualification_file or source/'qualification.json'
            qualification=read_qualification(source_qualification,hashlib.sha256((source/'generated/follower.urdf').read_bytes()).hexdigest(),geometry)
            for file,digest in qualification['evidence_sha256'].items():
                if hashlib.sha256(Path(file).read_bytes()).hexdigest()!=digest:raise ValueError('O1 evidence hash mismatch')
            qualification['transferred_from']=str(source_qualification)
            qualification['transfer_proof']='canonical URDF identical except controller file path; measured region/noise unchanged'
            qualification['urdf_sha256']=hashlib.sha256((generated/'follower.urdf').read_bytes()).hexdigest()
            qfile=a.output/'qualification.json';qfile.write_text(json.dumps(qualification,indent=2)+'\n')
            params.update(passive_yaw_calibration=False,arm_yaw_admittance_enabled=True)
            if a.comparison_mode:
                # Explicit existing policy supplies the common base gains and
                # limits; O2's reduced calibration pulse is not a transport gain.
                base=next(iter(yaml.safe_load(a.comparison_base_policy.read_text()).values()))['ros__parameters']
                for key in ('base_gains','base_velocity_limits','base_acceleration_limits'):
                    params[key]=base[key]
                if a.comparison_allocation_policy:
                    allocator=next(iter(yaml.safe_load(a.comparison_allocation_policy.read_text()).values()))['ros__parameters']
                    if (a.comparison_mode!='WHOLE_BODY' or not allocator
                            or set(allocator)-{'allocation_base_regularization','allocation_task_compliance_budget'}):
                        raise ValueError('explicit WHOLE_BODY allocation-only policy required')
                    params.update(allocator)
                params['rotation_observation_required']=True
                passive=json.loads((source/'passive_result.json').read_text())
                if a.comparison_previous_run or a.comparison_ablation_reference_run:
                    prior=a.comparison_previous_run or a.comparison_ablation_reference_run
                    previous=json.loads((prior/'comparison_result.json').read_text())
                    if a.comparison_previous_run:require_completed_active_step(a.comparison_delta_deg,previous,a.comparison_mode,qualification)
                    else:require_qualified_ablation(a.comparison_delta_deg,previous,a.comparison_mode,qualification)
                    if canonical_model(prior/'generated/follower.urdf')!=canonical_model(generated/'follower.urdf'):
                        raise ValueError('active expansion follower model changed')
                    if canonical_model(prior/'generated/leader.urdf')!=canonical_model(generated/'leader.urdf'):
                        raise ValueError('active expansion leader model changed')
                    if (prior/'generated/world.sdf').read_text().replace(str(prior),'RUN')!=(generated/'world.sdf').read_text().replace(str(a.output),'RUN'):
                        raise ValueError('active expansion physical scene changed')
                    old_params=next(iter(yaml.safe_load((prior/'follower_policy.yaml').read_text()).values()))['ros__parameters']
                    checked_params=dict(params)
                    if a.comparison_ablation_reference_run:
                        for key in ('allocation_base_regularization','allocation_task_compliance_budget'):
                            old_params.pop(key,None);checked_params.pop(key,None)
                    if old_params!=checked_params:raise ValueError('active expansion follower mechanics/gains/limits changed')
                    old_cert=json.loads((prior/'qualification.json').read_text())
                    for key in ('joint_noise_std_rad','yaw_noise_std_rad','yaw_innovation_limit_rad',
                            'yaw_residual_std_limit_rad','noise_window_samples','stationary_joint_speed_rad_s',
                            'qualified_q_lower_rad','qualified_q_upper_rad','geometry_signature','evidence_sha256'):
                        if old_cert.get(key)!=qualification.get(key):raise ValueError('active expansion measured qualification changed: '+key)
                else:require_completed_passive_range(a.comparison_delta_deg,passive)
            else:
                angle=.5*min(qualification['measured_signal_rad']['positive'],-qualification['measured_signal_rad']['negative'])
                duration=yaml.safe_load((source/'passive_leader_policy.yaml').read_text())['/leader/leader_object_rotation']['ros__parameters']['rotation_duration_s']
                vmax=list(params['base_velocity_limits']);vmax[2]=min(vmax[2],.5*1.875*angle/duration)
                params.update(base_gains=[0.,0.,0.],base_velocity_limits=vmax,
                    probe_angle_rad=angle,probe_duration_s=duration,probe_hold_s=2.,probe_qualification_file=str(qfile))
        policy=a.output/'follower_policy.yaml';policy.write_text(yaml.safe_dump({'/follower/follower_yaw_whole_body':{'ros__parameters':params}}))
        observer_policy=a.output/'observer_policy.yaml'
        observer_policy.write_text(yaml.safe_dump({'/follower/rotation_observer':{'ros__parameters':dict(
            orientation_envelope_rad=.01,joint_margin_rad=.01,timeout_s=.15,source_timeout_s=.05,
            allow_loaded_reference_latch=bool(a.o2_qualification_run),
            align_own_source_history=a.align_observer_sources,
            publish_on_raw_measurement=a.align_observer_sources,
            qualification_file=str(a.output/'qualification.json') if qualification else '')}}))
        start('integrated',['ros2','launch','amir_compliance_control',
            'rotation_compensation_probe.launch.py' if qualification and not a.comparison_mode else 'rotation_follow_integrated.launch.py',
            'run_directory:='+str(generated),'follower_policy_file:='+str(policy),
            'observer_policy_file:='+str(observer_policy),'wrench_calibration_file:='+str(calibration),
            *([] if qualification and not a.comparison_mode else [
                'rotation_follow_mode:='+(a.comparison_mode or ('XY_ONLY' if a.passive_delta_deg else 'WHOLE_BODY')),
                'yaw_heading_frame:='+a.yaw_heading_frame,
                'process_wrench_on_measurement:='+('true' if a.comparison_mode or a.process_wrench_on_measurement else 'false')])])
        attach={r:node.create_publisher(Empty,'/'+r+'/grasp/attach',10) for r in ('leader','follower')}
        wait(lambda:bool(observations))
        for _ in range(20):
            for pub in attach.values():pub.publish(Empty())
            until=time.monotonic()+.05;wait(lambda:time.monotonic()>=until,2.)
        wait(lambda:all(grasp.get(r)=='attached' for r in ('leader','follower')))
        wait(lambda:sum('projection_norm' in o for o in observations)>=20,60.)
        start_time=observations[-1]['timestamp']
        wait(lambda:observations[-1]['timestamp']-start_time>=5.,60.)
        result['latch_reference']=service('/follower/rotation/latch_reference')
        if result['latch_reference']['success']:
            wait(lambda:'psi_home_B' in observations[-1])
        result['enable_rotation']=service('/follower/admittance/enable')
        names=node.get_node_names_and_namespaces()
        result['observer_inputs']=node.get_subscriber_names_and_types_by_node('rotation_observer','/follower')
        result['controller_inputs']=node.get_subscriber_names_and_types_by_node('follower_yaw_whole_body','/follower')
        topics=[name for name,types in result['observer_inputs']+result['controller_inputs']]
        boundary=not any(t.startswith(('/leader/','/experiment/','/world/')) or '/tf' in t for t in topics)
        result['input_boundary_passed']=boundary
        result['base_writer_count']=len(node.get_publishers_info_by_topic('/follower/base_cmd'))
        result['arm_writer_count']=len(node.get_publishers_info_by_topic('/follower/arm_joint_cmd'))
        result['leader_goal_publishers']=len(node.get_publishers_info_by_topic('/leader/object_rotation_delta'))
        measured=[o for o in observations if 'projection_norm' in o and o['timestamp']>=start_time]
        result['measured_samples']=len(measured)
        result['maximum_projection_norm']=max(o['projection_norm'] for o in measured) if measured else None
        result['geometry_gate_passed']=bool(measured) and all(o.get('geometry_valid') for o in measured)
        result['yaw_heading_frame']=a.yaw_heading_frame
        expected_geometry=a.yaw_heading_frame=='grasp_heading'
        result['yaw_valid_ever_true']=any(o['yaw_valid'] for o in observations)
        result['ready_ever_true']=any(ready)
        result['both_fixed_grasps_retained']=all(grasp.get(r)=='attached' for r in ('leader','follower'))
        result['nonzero_base_commands']=sum(any(v!=0. for v in c) for c in commands)
        result['passed']=bool(measured and result['geometry_gate_passed']==expected_geometry and not result['yaw_valid_ever_true']
            and ready and not any(ready) and not result['enable_rotation']['success']
            and result['latch_reference']['success']==expected_geometry
            and boundary and result['base_writer_count']==result['arm_writer_count']==1
            and result['leader_goal_publishers']==0 and result['both_fixed_grasps_retained']
            and result['nonzero_base_commands']==0)
        result['O1']='NOT RUN: no passive qualification; no leader goal sent'
        if a.passive_delta_deg:
            if not result['passed']:raise RuntimeError('static calibration precondition failed')
            from amir_rotation_passive_trial import passive_trial
            result['passive_trial']=passive_trial(a,node,start,wait,service,generated,calibration,grasp,observations)
            result['passed']=result['passive_trial']['passed']
            result['O1']='See passive_trial; this is a bounded calibration bench, not autonomous yaw following'
        if qualification and a.comparison_mode:
            if not result['passed']:raise RuntimeError('R-A/B/C static startup precondition failed')
            from amir_rotation_abc_trial import abc_trial
            result['comparison_trial']=abc_trial(a,node,start,wait,service,generated,grasp,observations)
            result['passed']=result['comparison_trial']['passed']
        elif qualification:
            if not result['passed']:raise RuntimeError('O2 static startup precondition failed')
            from amir_rotation_o2_trial import o2_trial
            result['o2_trial']=o2_trial(a,node,start,wait,service,generated,grasp,observations)
            result['passed']=result['o2_trial']['passed']
        with (a.output/'timeseries.csv').open('w',newline='') as f:
            keys=('timestamp','joint_stamp','psi_ee_B','projection_norm','vertical_clearance_rad','yaw_valid','valid_reason')
            rows.flush()
            w=csv.DictWriter(f,fieldnames=keys);w.writeheader()
            with (a.output/'observer.jsonl').open() as full_log:
                for line in full_log:
                    o=json.loads(line);w.writerow({k:o.get(k) for k in keys})
            result['observer_csv_uses_complete_stream']=True
    except Exception as exc:result['error']=str(exc)
    finally:
        rows.close();grasp_events.close();node.destroy_node();rclpy.try_shutdown()
        for process in reversed(processes):
            if process.poll() is None:
                try:os.killpg(process.pid,signal.SIGINT)
                except ProcessLookupError:pass
        for process in reversed(processes):
            try:process.wait(timeout=3.)
            except subprocess.TimeoutExpired:pass
        result['process_cleanup']=reap_partition(partition,domain)
        for log in logs:log.close()
        (a.output/'live_result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2));return result['passed']


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--source-run',type=Path,required=True)
    p.add_argument('--source-samples',type=Path,required=True)
    p.add_argument('--sensor-certificate',type=Path,required=True)
    p.add_argument('--follower-policy',type=Path,required=True)
    p.add_argument('--live',action='store_true')
    p.add_argument('--align-observer-sources',action='store_true',help='align own actual source histories within existing freshness/skew limits')
    p.add_argument('--grasp-state-publish-period-s',type=float,default=.1,
                   help='actual constraint status period in simulation seconds; legacy default 0.1')
    p.add_argument('--process-wrench-on-measurement',action='store_true',
                   help='opt-in existing arrival-driven FT preprocessing, including passive calibration')
    p.add_argument('--yaw-heading-frame',choices=('tcp_link','grasp_heading'),default='tcp_link')
    p.add_argument('--passive-delta-deg',type=float,default=0.,
                   help='explicit fixed-grasp calibration bench; stationary follower base, at most 0.1 degree')
    p.add_argument('--passive-arm-yaw',action='store_true',
                   help='passive arm-only torque admittance; base yaw remains zero and unqualified')
    p.add_argument('--passive-duration-s',type=float,
                   help='explicit slower calibration duration; default retains the input leader profile')
    p.add_argument('--o2-qualification-run','--qualification-run',type=Path)
    p.add_argument('--o2-qualification-file','--qualification-file',type=Path)
    p.add_argument('--o2-observer-stop',action='store_true')
    p.add_argument('--comparison-mode',choices=MODES)
    p.add_argument('--comparison-previous-run',type=Path,help='completed same-mode transport authorizing at most a threefold next endpoint; exact local symmetry required')
    p.add_argument('--comparison-allocation-policy',type=Path,help='explicit WHOLE_BODY-only allocator calibration; no arm mechanics or safety limit overrides')
    p.add_argument('--comparison-leader-limits',type=Path,help='explicit calibrated planning speed/acceleration reductions only')
    p.add_argument('--comparison-ablation-reference-run',type=Path,help='completed qualified WHOLE_BODY endpoint for an A/B/C ablation under identical arm mechanics and safety limits')
    p.add_argument('--comparison-delta-deg',type=float,default=.001)
    p.add_argument('--comparison-base-policy',type=Path)
    a=p.parse_args()
    if not math.isfinite(a.grasp_state_publish_period_s) or a.grasp_state_publish_period_s<=0.:
        p.error('positive finite actual grasp-state publication period required')
    if not math.isfinite(a.passive_delta_deg) or not 0.<=a.passive_delta_deg<=.1:
        p.error('passive angle must be finite and within [0,0.1] degree')
    if a.passive_delta_deg and (not a.live or a.yaw_heading_frame!='grasp_heading'):
        p.error('passive bench requires live mode and explicit grasp_heading condition')
    if a.passive_arm_yaw and not a.passive_delta_deg:
        p.error('passive arm admittance requires an explicit bounded passive trial')
    if a.passive_duration_s is not None and (not a.passive_delta_deg or not math.isfinite(a.passive_duration_s) or a.passive_duration_s<=0.):
        p.error('finite positive duration requires an explicit passive trial')
    if a.o2_qualification_run:
        a.o2_qualification_run=a.o2_qualification_run.resolve()
        if not a.live or a.yaw_heading_frame!='grasp_heading' or a.passive_delta_deg:
            p.error('O2 requires live grasp_heading and no O1 calibration motion')
    if a.o2_observer_stop and not a.o2_qualification_run:p.error('observer stop is confined to O2')
    if a.comparison_mode:
        if not a.o2_qualification_run or not a.comparison_base_policy or a.o2_observer_stop:
            p.error('comparison requires O1 qualification and explicit base policy, without O2 fault injection')
        if a.comparison_previous_run and a.comparison_ablation_reference_run:p.error('choose active expansion or ablation reference')
        if not math.isfinite(a.comparison_delta_deg) or not 0.<a.comparison_delta_deg<=(90. if a.comparison_previous_run or a.comparison_ablation_reference_run else .1):
            p.error('comparison requires a finite angle within the bounded O1 bench; completed range is checked before launch')
        a.comparison_base_policy=a.comparison_base_policy.resolve()
        if a.comparison_previous_run:a.comparison_previous_run=a.comparison_previous_run.resolve()
        if a.comparison_allocation_policy:
            if a.comparison_mode!='WHOLE_BODY':p.error('allocation policy requires WHOLE_BODY')
            a.comparison_allocation_policy=a.comparison_allocation_policy.resolve()
        if a.comparison_leader_limits:a.comparison_leader_limits=a.comparison_leader_limits.resolve()
        if a.comparison_ablation_reference_run:a.comparison_ablation_reference_run=a.comparison_ablation_reference_run.resolve()
    elif a.comparison_previous_run:p.error('previous active run requires an explicit comparison mode')
    elif a.comparison_allocation_policy:p.error('allocation policy requires an explicit WHOLE_BODY comparison')
    elif a.comparison_leader_limits:p.error('leader planning limits require an explicit comparison')
    elif a.comparison_ablation_reference_run:p.error('ablation reference requires an explicit comparison')
    if a.o2_qualification_file:
        a.o2_qualification_file=a.o2_qualification_file.resolve()
        if not a.o2_qualification_run:p.error('explicit O1 model and evidence are required for O2 certificate selection')
    for name in ('output','source_run','source_samples','sensor_certificate','follower_policy'):
        setattr(a,name,getattr(a,name).resolve())
    a.output.mkdir(parents=True,exist_ok=False)
    workspace=Path(__file__).resolve().parents[1]
    sources=[*list((workspace/'src/humbleble_ws/amir_compliance_control').rglob('*.py')),
             *[Path(__file__).with_name(n) for n in ('amir_rotation_integration_audit.py',
                'amir_rotation_passive_trial.py','amir_rotation_passive_qualification.py','amir_rotation_o2_trial.py',
                'amir_rotation_abc_trial.py','amir_rotation_abc_report.py','amir_rotation_stream_report.py','amir_rotation_yaw_symmetry.py','test_rotation_bench_policy.py')],
             *[workspace/n for n in ('src/humbleble_ws/amir_follower_gazebo/src/grasp_heartbeat.cpp',
                'src/humbleble_ws/cooperative_transport_bringup/launch/compliance_cp1.launch.py',
                'src/humbleble_ws/cooperative_transport_bringup/launch/compliance_cp6_fixture.launch.py')]]
    manifest={}
    for source in sources:
        if '__pycache__' in source.parts:continue
        relative=source.relative_to(workspace)
        target=a.output/'source_snapshot'/relative;target.parent.mkdir(parents=True,exist_ok=True)
        target.write_bytes(source.read_bytes());manifest[str(relative)]=hashlib.sha256(source.read_bytes()).hexdigest()
    (a.output/'source_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    report=offline(a)
    if a.live:return 0 if live(a) else 1
    print(json.dumps(report,indent=2));return 0


if __name__=='__main__':raise SystemExit(main())

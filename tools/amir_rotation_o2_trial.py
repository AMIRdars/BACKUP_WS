"""O2: local follower base pulse with fixed grasps and no leader/object goal."""
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import time
import numpy as np


def o2_trial(a,node,start,wait,service,generated,grasp,observations):
    from geometry_msgs.msg import Twist,TwistStamped
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    from ros_gz_interfaces.msg import Entity
    from ros_gz_interfaces.srv import DeleteEntity
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
    from amir_cp8r3_clearance import BaseClearance
    from amir_cp8r3_trial import pose_matrix
    from amir_rotation_integration_audit import canonical_model

    states,safety,outputs,poses,debug,encoder={},{},{},{},{},{}
    records,truth,losses=[],[],[];active=[False];phase=['startup'];fault=[''];paused=[]
    local_file=(a.output/'o2_local.jsonl').open('w',buffering=1)
    truth_file=(a.output/'evaluation_only.jsonl').open('w',buffering=1)
    report=dict(passed=False,condition='O2_single_base_post_saturation_compensation',
                observer_stop_injected=a.o2_observer_stop,leader_object_goal_count=0)
    bounds=BaseClearance(generated);report['base_collision_reserve_m']=bounds.reserve

    def observe(msg):
        o=json.loads(msg.data)
        if not active[0]:return
        if not a.o2_observer_stop and o.get('yaw_valid') is not True:fault[0]='qualified observer lost: '+o['valid_reason']

    def record(msg):
        data=json.loads(msg.data);debug.clear();debug.update(data)
        if not active[0]:return
        if not a.o2_observer_stop and data.get('state')=='SAFE_STOP' and not fault[0]:
            fault[0]='local controller stop: '+data.get('reason','unknown')
        o=observations[-1]
        row=dict(timestamp=data['sim_time'],wall_s=time.monotonic(),phase=phase[0],debug=data,observation=o,
                 outputs={r:list(v) for r,v in outputs.items()},grasp=dict(grasp),
                 states={r:dict(s) for r,s in states.items()})
        if encoder:
            m=encoder['odom'];row['follower_odom_pose']=[m.pose.pose.position.x,m.pose.pose.position.y,
                math.atan2(2*(m.pose.pose.orientation.w*m.pose.pose.orientation.z+m.pose.pose.orientation.x*m.pose.pose.orientation.y),
                           1-2*(m.pose.pose.orientation.y**2+m.pose.pose.orientation.z**2))]
        records.append(row);local_file.write(json.dumps(row)+'\n')
        if any(grasp.get(r)!='attached' for r in ('leader','follower')):
            losses.append(dict(timestamp=row['timestamp'],grasp=dict(grasp)));fault[0]='fixed grasp confirmation lost'

    def gt(msg):
        for t in msg.transforms:
            if t.child_frame_id in ('leader','follower','cooperative_payload'):
                poses[t.child_frame_id]=pose_matrix(t.transform.translation,t.transform.rotation)
        if not active[0] or not all(r in poses for r in ('leader','follower','cooperative_payload')):return
        gap=bounds.gap(poses['leader'],poses['follower'])
        row=dict(timestamp=observations[-1]['timestamp'],phase=phase[0],base_surface_gap_m=gap,
                 object_pose_GT=poses['cooperative_payload'].tolist(),
                 follower_base_pose_GT=poses['follower'].tolist(),leader_base_pose_GT=poses['leader'].tolist())
        truth.append(row);truth_file.write(json.dumps(row)+'\n')
        if gap<=bounds.reserve:fault[0]='base collision reserve violated'

    subs=[node.create_subscription(String,'/follower/rotation/observation',observe,10),
          node.create_subscription(String,'/follower/rotation_follow/debug',record,10),
          node.create_subscription(String,'/transport/safety_status',lambda m:safety.update(json.loads(m.data)),10),
          node.create_subscription(Odometry,'/follower/mecanum_drive_controller/odometry',lambda m:encoder.__setitem__('odom',m),qos_profile_sensor_data),
          node.create_subscription(TFMessage,'/world/cooperative_transport/pose/info',gt,qos_profile_sensor_data)]
    for r in ('leader','follower'):
        subs += [node.create_subscription(String,'/'+r+'/transport/state',lambda m,r=r:states.__setitem__(r,json.loads(m.data)),10),
                 node.create_subscription(Twist,'/'+r+'/rover_twist',lambda m,r=r:outputs.__setitem__(r,[m.linear.x,m.linear.y,m.angular.z]),10)]

    def checked(predicate,timeout=60.):
        def check():
            if fault[0] or safety.get('fault'):raise RuntimeError(fault[0] or safety['fault'])
            return predicate()
        wait(check,timeout)

    def require(name):
        response=service(name)
        if not response['success']:raise RuntimeError(name+': '+response['message'])
        return response

    def settle(seconds):
        t=observations[-1]['timestamp'];checked(lambda:observations[-1]['timestamp']-t>=seconds,max(60.,seconds*6.))

    try:
        for robot in ('leader','follower'):
            if canonical_model(generated/(robot+'.urdf'))!=canonical_model(a.source_run/(robot+'.urdf')):
                raise ValueError('O2 safety profile model transfer refused')
            profile=json.loads((a.source_run/(robot+'_safety.json')).read_text())
            if profile['urdf_sha256']!=hashlib.sha256((a.source_run/(robot+'.urdf')).read_bytes()).hexdigest():
                raise ValueError('O2 source safety model hash mismatch')
            for key,suffix in [('controller_yaml','_controllers.yaml'),('base_controller_yaml','_base_controller.yaml')]:
                file=generated/(robot+suffix)
                if hashlib.sha256(file.read_bytes()).hexdigest()!=profile[key+'_sha256']:
                    raise ValueError('O2 original controller configuration changed')
                profile[key+'_file']=str(file)
            profile.update(urdf_sha256=hashlib.sha256((generated/(robot+'.urdf')).read_bytes()).hexdigest(),
                world_sha256=hashlib.sha256((generated/'world.sdf').read_bytes()).hexdigest(),
                require_grasp_for_base_motion=True,ungrasped_base_test_only=False)
            (generated/(robot+'_safety.json')).write_text(json.dumps(profile,indent=2)+'\n')
        start('evaluation_bridge',['ros2','run','ros_gz_bridge','parameter_bridge',
            '/world/cooperative_transport/remove@ros_gz_interfaces/srv/DeleteEntity',
            '/world/cooperative_transport/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'])
        start('safety',['ros2','launch','amir_compliance_control','guarded_safety.launch.py','run_directory:='+str(generated)])
        start('leader_hold',['ros2','run','amir_compliance_control','leader_trajectory_node','--ros-args','-r','__ns:=/leader',
            '-p','use_sim_time:=true','-p','actuator_ack_required:=true','-p','urdf_file:='+str(generated/'leader.urdf')])
        wait(lambda:safety.get('ready') and all(states.get(r,{}).get('local_ready') for r in ('leader','follower')),60.)
        expected=[('/'+r+'/'+t,n,r) for r in ('leader','follower') for t,n in (
            ('rover_twist','actuator_interlock'),('arm_controller/joint_trajectory','actuator_interlock'),
            ('mecanum_drive_controller/reference_unstamped','rover_twist_relay_ign'))]
        def discovered():
            for topic,name,r in expected:
                writers=node.get_publishers_info_by_topic(topic)
                if len(writers)!=1 or writers[0].node_name!=name or writers[0].node_namespace!='/'+r:return False
            return True
        wait(discovered,30.);settle(1.)
        for r in ('leader','follower'):require('/'+r+'/transport/prepare')
        require('/transport/safety/arm')
        checked(lambda:all(states.get(r,{}).get('state')=='READY' for r in ('leader','follower')),30.)
        remover=node.create_client(DeleteEntity,'/world/cooperative_transport/remove');wait(remover.service_is_ready)
        request=DeleteEntity.Request();request.entity.name='grasp_support';request.entity.type=Entity.MODEL
        f=remover.call_async(request);wait(f.done)
        if not f.result().success:raise RuntimeError('O2 support removal failed')
        node.destroy_client(remover);settle(10.)
        require('/follower/tare_wrench');settle(1.)
        prior_sequence=observations[-1].get('reference_sequence',0)
        report['loaded_reference_latch']=require('/follower/rotation/latch_reference')
        checked(lambda:observations[-1].get('reference_sequence',0)>prior_sequence
                and observations[-1].get('yaw_valid') is True,10.)
        report['loaded_reference_sequence']=observations[-1]['reference_sequence']
        report['enable']=require('/follower/admittance/enable')
        active[0]=True;phase[0]='baseline';settle(3.)
        report['probe_start']=require('/follower/rotation/start_compensation_probe')
        phase[0]='probe'
        if a.o2_observer_stop:
            checked(lambda:debug.get('probe_phase')=='positive'
                and abs(debug.get('probe_requested_wz_rad_s',0.))>abs(debug.get('probe_limited_twist_footprint',[0.,0.,0.])[2])+1e-15
                and abs(outputs.get('follower',[0.,0.,0.])[2])>0.,60.)
            # Only the observer process owned by this exact partition/domain.
            for proc in Path('/proc').iterdir():
                if not proc.name.isdigit():continue
                try:
                    env=dict(s.split(b'=',1) for s in (proc/'environ').read_bytes().split(b'\0') if b'=' in s)
                    args=(proc/'cmdline').read_bytes().split(b'\0')
                    if (env.get(b'IGN_PARTITION',b'').decode()==os.getenv('IGN_PARTITION')
                            and env.get(b'ROS_DOMAIN_ID',b'').decode()==os.getenv('ROS_DOMAIN_ID')
                            and any(x.endswith(b'/rotation_observer') for x in args)):paused.append(int(proc.name))
                except OSError:pass
            if len(paused)!=1:raise RuntimeError('one owned observer PID required for fault injection')
            report['fault_injection']=dict(pid=paused[0],wall_s=time.monotonic(),sim_s=debug['sim_time'],
                                          follower_twist_before=list(outputs['follower']))
            os.kill(paused[0],signal.SIGSTOP)
            wait(lambda:all(states.get(r,{}).get('state')=='SAFE_STOP' for r in ('leader','follower')),10.)
            report['observer_stop_detected']=True
            report['fault_detection_wall_s']=time.monotonic()-report['fault_injection']['wall_s']
        else:
            checked(lambda:debug.get('probe_phase')=='complete',240.);phase[0]='hold';settle(3.)
            report['positive_negative_pulse_completed']=True
        report['leader_object_goal_count']=states.get('leader',{}).get('accepted_goals',-1)
        report['passed']=bool(report['leader_object_goal_count']==0 and not losses)
    except Exception as exc:report['reason']=str(exc)
    finally:
        phase[0]='safe_stop'
        try:report['stop_response']=service('/transport/safety/stop')
        except Exception as exc:report['stop_error']=str(exc)
        initial=time.monotonic()
        try:wait(lambda:time.monotonic()-initial>=2.,5.)
        except Exception:pass
        for pid in paused:
            try:os.kill(pid,signal.SIGCONT)
            except ProcessLookupError:pass
        report.update(grasp_losses=losses,local_samples=len(records),final_states=states,final_safety=safety,
            final_outputs=outputs,both_fixed_grasps_retained=all(grasp.get(r)=='attached' for r in ('leader','follower')),
            minimum_base_surface_gap_m=min((r['base_surface_gap_m'] for r in truth),default=None))
        report['stop_and_hold_confirmed']=bool(report['both_fixed_grasps_retained']
            and all(states.get(r,{}).get('state')=='SAFE_STOP' for r in ('leader','follower'))
            and all(r in outputs and np.max(abs(np.array(outputs[r])))<1e-12 for r in ('leader','follower')))
        samples=[r for r in records if 'probe_limited_twist_footprint' in r['debug']]
        moving=[r for r in samples if abs(r['debug']['probe_limited_twist_footprint'][2])>0.]
        report['moving_samples']=len(moving)
        report['limited_yaw_observed']=any(abs(r['debug']['probe_requested_wz_rad_s'])>abs(r['debug']['probe_limited_twist_footprint'][2])+1e-15 for r in moving)
        report['maximum_compensated_point_velocity_residual_m_s']=max((float(np.linalg.norm(r['debug']['probe_rigid_point_velocity_footprint_m_s'])) for r in samples),default=None)
        report['passed']=bool(report['passed'] and report['stop_and_hold_confirmed'] and moving)
        local_file.close();truth_file.close()
        with (a.output/'timeseries_o2.csv').open('w',newline='') as f:
            keys=['t_s','phase','yaw_valid','reason','probe_phase','requested_wz','vx','vy','wz','comp_x','comp_y',
                  'p_x','p_y','p_z','point_residual_m_s','grasp_leader','grasp_follower']
            writer=csv.DictWriter(f,fieldnames=keys);writer.writeheader()
            for row in records:
                d,o=row['debug'],row['observation'];u=d.get('probe_limited_twist_footprint',[None]*3)
                comp=d.get('probe_compensation_footprint_m_s',[None]*2);p=d.get('probe_position_footprint_m',[None]*3)
                writer.writerow(dict(t_s=row['timestamp'],phase=row['phase'],yaw_valid=o['yaw_valid'],reason=o['valid_reason'],
                    probe_phase=d.get('probe_phase'),requested_wz=d.get('probe_requested_wz_rad_s'),
                    vx=u[0],vy=u[1],wz=u[2],comp_x=comp[0],comp_y=comp[1],p_x=p[0],p_y=p[1],p_z=p[2],
                    point_residual_m_s=None if 'probe_rigid_point_velocity_footprint_m_s' not in d else float(np.linalg.norm(d['probe_rigid_point_velocity_footprint_m_s'])),
                    grasp_leader=row['grasp'].get('leader'),grasp_follower=row['grasp'].get('follower')))
        for sub in subs:node.destroy_subscription(sub)
        (a.output/'o2_result.json').write_text(json.dumps(report,indent=2)+'\n')
    return report

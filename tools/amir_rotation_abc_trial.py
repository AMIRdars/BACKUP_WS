"""Qualified fixed-grasp A/B/C bench; only the leader receives rotation goals.

Ground truth is confined to this evaluator, for clearance supervision and
post-run metrics. All variants use identical arm and safety parameters.
"""
import hashlib
import json
import math
from pathlib import Path
import time
import numpy as np
import yaml


def abc_trial(a,node,start,wait,service,generated,grasp,observations):
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    from ros_gz_interfaces.msg import Entity
    from ros_gz_interfaces.srv import DeleteEntity
    from std_msgs.msg import Bool,Float64,String
    from tf2_msgs.msg import TFMessage
    from amir_cp8r3_clearance import BaseClearance
    from amir_cp8r3_trial import pose_matrix
    from amir_rotation_integration_audit import canonical_model,conservative_leader_limits

    states,safety,outputs,poses,debug,encoder={},{},{},{},{},{}
    sample_count=[0];minimum_gap=[float('inf')];losses=[];leader_status={}
    active=[False];phase=['startup'];fault=[''];ready=[False]
    local_file=(a.output/'comparison_local.jsonl').open('w',buffering=1)
    truth_file=(a.output/'evaluation_only.jsonl').open('w',buffering=1)
    leader_file=(a.output/'leader_debug.jsonl').open('w',buffering=1)
    report=dict(passed=False,condition='fixed_grasp_qualified_'+('D' if a.comparison_mode=='WHOLE_BODY' else 'ABC'),mode=a.comparison_mode,
                delta_deg=a.comparison_delta_deg,leader_goal_values_rad=[],angle_accuracy_is_diagnostic=True)
    bounds=BaseClearance(generated);report['base_collision_reserve_m']=bounds.reserve

    def observe(msg):
        o=json.loads(msg.data)
        if active[0] and phase[0]!='safe_stop' and o.get('yaw_valid') is not True and not fault[0]:
            fault[0]='qualified observer lost: '+o['valid_reason']

    def record(msg):
        data=json.loads(msg.data);debug.clear();debug.update(data)
        if not active[0]:return
        if phase[0]!='safe_stop' and data.get('state')=='SAFE_STOP' and not fault[0]:
            fault[0]='local controller stop: '+data.get('reason','unknown')
        row=dict(timestamp=data['sim_time'],wall_s=time.monotonic(),phase=phase[0],debug=data,
                 observation=observations[-1],outputs={r:list(v) for r,v in outputs.items()},
                 grasp=dict(grasp),states={r:dict(s) for r,s in states.items()})
        if encoder:
            m=encoder['odom'];q=m.pose.pose.orientation
            row['follower_odom_pose']=[m.pose.pose.position.x,m.pose.pose.position.y,
                math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
        sample_count[0]+=1;local_file.write(json.dumps(row)+'\n')
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
        minimum_gap[0]=min(minimum_gap[0],gap);truth_file.write(json.dumps(row)+'\n')
        if gap<=bounds.reserve:fault[0]='base collision reserve violated'

    def leader_debug(msg):
        leader_status.clear();leader_status.update(json.loads(msg.data));leader_file.write(msg.data+'\n')

    subs=[node.create_subscription(String,'/follower/rotation/observation',observe,10),
          node.create_subscription(String,'/leader/rotation/debug',leader_debug,10),
          node.create_subscription(String,'/follower/rotation_follow/debug',record,10),
          node.create_subscription(String,'/transport/safety_status',lambda m:safety.update(json.loads(m.data)),10),
          node.create_subscription(Bool,'/transport/rotation_follower_ready',lambda m:ready.__setitem__(0,m.data),10),
          node.create_subscription(Odometry,'/follower/mecanum_drive_controller/odometry',lambda m:encoder.__setitem__('odom',m),qos_profile_sensor_data),
          node.create_subscription(TFMessage,'/world/cooperative_transport/pose/info',gt,qos_profile_sensor_data)]
    for r in ('leader','follower'):
        subs += [node.create_subscription(String,'/'+r+'/transport/state',lambda m,r=r:states.__setitem__(r,json.loads(m.data)),10),
                 node.create_subscription(Twist,'/'+r+'/rover_twist',lambda m,r=r:outputs.__setitem__(r,[m.linear.x,m.linear.y,m.angular.z]),10)]
    goal=node.create_publisher(Float64,'/leader/object_rotation_delta',1)

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
                raise ValueError('comparison safety profile model transfer refused')
            profile=json.loads((a.source_run/(robot+'_safety.json')).read_text())
            if profile['urdf_sha256']!=hashlib.sha256((a.source_run/(robot+'.urdf')).read_bytes()).hexdigest():
                raise ValueError('source safety model hash mismatch')
            for key,suffix in [('controller_yaml','_controllers.yaml'),('base_controller_yaml','_base_controller.yaml')]:
                file=generated/(robot+suffix)
                if hashlib.sha256(file.read_bytes()).hexdigest()!=profile[key+'_sha256']:
                    raise ValueError('original controller configuration changed')
                profile[key+'_file']=str(file)
            profile.update(urdf_sha256=hashlib.sha256((generated/(robot+'.urdf')).read_bytes()).hexdigest(),
                world_sha256=hashlib.sha256((generated/'world.sdf').read_bytes()).hexdigest(),
                require_grasp_for_base_motion=True,ungrasped_base_test_only=False)
            (generated/(robot+'_safety.json')).write_text(json.dumps(profile,indent=2)+'\n')
        source=a.o2_qualification_run
        certificate=json.loads((source/'leader_fixed_grasp_calibration.json').read_text())
        if (canonical_model(source/'generated/leader.urdf')!=canonical_model(generated/'leader.urdf')
                or certificate['urdf_sha256']!=hashlib.sha256((source/'generated/leader.urdf').read_bytes()).hexdigest()):
            raise ValueError('leader grasp calibration source/model mismatch')
        certificate['urdf_sha256']=hashlib.sha256((generated/'leader.urdf').read_bytes()).hexdigest()
        certificate['transfer_proof']='canonical URDF equality except controller path'
        grasp_file=a.output/'leader_fixed_grasp_calibration.json';grasp_file.write_text(json.dumps(certificate,indent=2)+'\n')
        params=yaml.safe_load((source/'passive_leader_policy.yaml').read_text())['/leader/leader_object_rotation']['ros__parameters']
        if a.comparison_leader_limits:
            overrides=next(iter(yaml.safe_load(a.comparison_leader_limits.read_text()).values()))['ros__parameters']
            params=conservative_leader_limits(params,overrides)
        if a.comparison_previous_run or a.comparison_ablation_reference_run:
            # Only the experiment's requested angle envelope changes. Actual
            # speed/acceleration and the leader's minimum-duration calculation
            # remain at the existing calibrated values.
            params['rotation_max_yaw_rad']=max(params['rotation_max_yaw_rad'],math.radians(2*a.comparison_delta_deg))
        if math.radians(2*a.comparison_delta_deg)>params['rotation_max_yaw_rad']:
            raise ValueError('relative return goal exceeds previous leader envelope')
        params.update(grasp_calibration_file=str(grasp_file),actuator_ack_required=True)
        policy=a.output/'leader_policy.yaml';policy.write_text(yaml.safe_dump({'/leader/leader_object_rotation':{'ros__parameters':params}}))
        start('evaluation_bridge',['ros2','run','ros_gz_bridge','parameter_bridge',
            '/world/cooperative_transport/remove@ros_gz_interfaces/srv/DeleteEntity',
            '/world/cooperative_transport/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'])
        start('safety',['ros2','launch','amir_compliance_control','guarded_safety.launch.py','run_directory:='+str(generated)])
        start('leader',['ros2','launch','amir_compliance_control','object_rotation_leader.launch.py',
            'run_directory:='+str(generated),'rotation_policy_file:='+str(policy)])
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
        if not f.result().success:raise RuntimeError('support removal failed')
        node.destroy_client(remover);settle(10.)
        require('/follower/tare_wrench');settle(1.)
        prior_sequence=observations[-1].get('reference_sequence',0)
        report['loaded_reference_latch']=require('/follower/rotation/latch_reference')
        checked(lambda:observations[-1].get('reference_sequence',0)>prior_sequence
                and observations[-1].get('yaw_valid') is True,10.)
        report['enable']=require('/follower/admittance/enable')
        active[0]=True;phase[0]='baseline';settle(3.);checked(lambda:ready[0],10.)
        for name,delta,index in [('positive',a.comparison_delta_deg,1),('negative',-2*a.comparison_delta_deg,2)]:
            phase[0]=name;value=math.radians(delta);goal.publish(Float64(data=value));report['leader_goal_values_rad'].append(value)
            checked(lambda:states.get('leader',{}).get('accepted_goals',0)>=index,10.)
            accepted_time=states['leader']['sim_time']
            checked(lambda:leader_status.get('sim_time',-1.)>=accepted_time and leader_status.get('duration_s',0.)>0.,10.)
            timeout=max(240.,6*(leader_status['duration_s']+params['rotation_terminal_timeout_s']+params['rotation_hold_s']))
            report.setdefault('leader_profile_durations_s',[]).append(leader_status['duration_s'])
            checked(lambda:states.get('leader',{}).get('completed_goals',0)>=index,timeout)
            settle(3.)
        phase[0]='hold';settle(3.)
        report.update(positive_negative_programs_completed=True,accepted_goals=states['leader'].get('accepted_goals'),
                      completed_goals=states['leader'].get('completed_goals'))
        report['passed']=bool(report['completed_goals']==2 and not losses)
    except Exception as exc:report['reason']=str(exc)
    finally:
        phase[0]='safe_stop'
        try:report['stop_response']=service('/transport/safety/stop')
        except Exception as exc:report['stop_error']=str(exc)
        initial=time.monotonic()
        try:wait(lambda:time.monotonic()-initial>=2.,5.)
        except Exception:pass
        report.update(grasp_losses=losses,local_samples=sample_count[0],final_states=states,final_safety=safety,
            final_outputs=outputs,both_fixed_grasps_retained=all(grasp.get(r)=='attached' for r in ('leader','follower')),
            minimum_base_surface_gap_m=None if not math.isfinite(minimum_gap[0]) else minimum_gap[0])
        report['stop_and_hold_confirmed']=bool(report['both_fixed_grasps_retained']
            and all(states.get(r,{}).get('state')=='SAFE_STOP' for r in ('leader','follower'))
            and all(r in outputs and np.max(abs(np.array(outputs[r])))<1e-12 for r in ('leader','follower')))
        report['passed']=bool(report['passed'] and report['stop_and_hold_confirmed'])
        local_file.close();truth_file.close();leader_file.close()
        for sub in subs:node.destroy_subscription(sub)
        node.destroy_publisher(goal)
        (a.output/'comparison_result.json').write_text(json.dumps(report,indent=2)+'\n')
    return report

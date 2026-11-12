"""Bounded Gate-1 bench, invoked only by the isolated integration auditor.

This is calibration with a stationary follower base and its existing arm
compliance. It never publishes qualified-ready or enables autonomous yaw.
The normal leader launch retains the chapter 23 readiness gate.
"""
import csv
import hashlib
import json
import math
from pathlib import Path
import time

import numpy as np
import yaml


def passive_trial(a,node,start,wait,service,generated,calibration,grasp,observations):
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import JointState
    from rclpy.qos import qos_profile_sensor_data
    from ros_gz_interfaces.msg import Entity
    from ros_gz_interfaces.srv import DeleteEntity
    from std_msgs.msg import Float64,String
    from tf2_msgs.msg import TFMessage
    from amir_compliance_control.rotation_observer import ProjectedYaw,qualify_passive,wrap_pi
    from amir_cp8r3_clearance import BaseClearance
    from amir_cp8r3_trial import pose_matrix
    from amir_rotation_integration_audit import canonical_model,passive_profile_duration

    states,safety,outputs,poses={},{},{},{}
    records,evaluation,losses=[],[],[]
    phase=['preparation'];last_stamp=[None];active=[False]
    evidence=(a.output/'passive_local.jsonl').open('w',buffering=1)
    truth=(a.output/'evaluation_only.jsonl').open('w',buffering=1)
    control=(a.output/'passive_control.jsonl').open('w',buffering=1)
    raw_joints=(a.output/'passive_joint_raw.jsonl').open('w',buffering=1)
    control_rows=[]
    report=dict(passed=False,certificate_issued=False,delta_deg=a.passive_delta_deg,
                condition='fixed_grasp_stationary_follower_base_calibration',
                passive_arm_yaw_admittance=a.passive_arm_yaw,
                autonomous_follower_yaw_enabled=False,qualification_signal='own FK, joints and F/T only')
    geometry=ProjectedYaw((generated/'follower.urdf').read_text(),orientation_envelope_rad=.01,
                          joint_margin_rad=.01,heading_frame=a.yaw_heading_frame)
    bounds=BaseClearance(generated)
    report['base_collision_reserve_m']=bounds.reserve
    fault=[''];pending_certificate=None

    def controller(msg):
        if not active[0] and phase[0]!='safe_stop':return
        debug=json.loads(msg.data)
        row=dict(timestamp=debug['sim_time'],wall_s=time.monotonic(),phase=phase[0],debug=debug,
                 states={r:dict(s) for r,s in states.items()},outputs={r:list(v) for r,v in outputs.items()},grasp=dict(grasp))
        control_rows.append(row);control.write(json.dumps(row)+'\n')

    def joint_sample(msg):
        if not active[0]:return
        q=dict(zip(msg.name,msg.position));qd=dict(zip(msg.name,msg.velocity))
        raw_joints.write(json.dumps(dict(timestamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
            phase=phase[0],q_rad=[q[n] for n in geometry.model.names],
            qdot_rad_s=[qd[n] for n in geometry.model.names]))+'\n')

    def local(msg):
        o=json.loads(msg.data)
        if not active[0] or phase[0] not in ('static','positive','negative'):return
        if not all(k in o for k in ('joint_stamp','q_rad','qdot_rad_s','wrench_tcp','base_twist')):return
        stamp=o['joint_stamp']
        if stamp==last_stamp[0]:return
        last_stamp[0]=stamp
        row=dict(phase=phase[0],timestamp=stamp,q_rad=o['q_rad'],qdot_rad_s=o['qdot_rad_s'],
            wrench_tcp=o['wrench_tcp'],base_twist=o['base_twist'],grasp_attached=o.get('grasp_attached',False),
            psi_ee_B=o['psi_ee_B'],e_yaw_rad=o.get('e_yaw_rad'),geometry_valid=o['geometry_valid'],
            yaw_heading_frame=geometry.heading_frame,yaw_valid=o['yaw_valid'])
        records.append(row);evidence.write(json.dumps(row)+'\n')
        if not row['grasp_attached'] or any(grasp.get(r)!='attached' for r in ('leader','follower')):
            losses.append(dict(timestamp=stamp,grasp=dict(grasp)));fault[0]='fixed grasp confirmation lost'
        if not row['geometry_valid']:fault[0]='projected FK geometry invalid'
        if len(records)>1 and not 0.<stamp-records[-2]['timestamp']<=.05:
            fault[0]='local calibration source gap exceeds 50 ms'
        if np.max(abs(np.array(row['base_twist'])))>=.0001:
            fault[0]='follower base moved outside stationary calibration envelope'
        if phase[0]!='static':
            w0=np.mean([r['wrench_tcp'] for r in records if r['phase']=='static'],axis=0)
            w=np.array(row['wrench_tcp'])-w0
            if np.linalg.norm(w[:3])>=.5 or np.linalg.norm(w[3:])>=.05:
                fault[0]='incremental follower wrench exceeds measured 0.5 N / 0.05 Nm envelope'

    def gt(msg):
        for t in msg.transforms:
            if t.child_frame_id in ('leader','follower','cooperative_payload'):
                poses[t.child_frame_id]=pose_matrix(t.transform.translation,t.transform.rotation)
        if not active[0] or not all(r in poses for r in ('leader','follower','cooperative_payload')):return
        gap=bounds.gap(poses['leader'],poses['follower'])
        row=dict(timestamp=observations[-1]['timestamp'],phase=phase[0],
                 object_pose_GT=poses['cooperative_payload'].tolist(),base_surface_gap_m=gap,
                 leader_base_pose_GT=poses['leader'].tolist(),follower_base_pose_GT=poses['follower'].tolist())
        evaluation.append(row);truth.write(json.dumps(row)+'\n')
        if gap<=bounds.reserve:fault[0]='base separation reserve violated'

    subscriptions=[node.create_subscription(String,'/follower/rotation/observation',local,10),
        node.create_subscription(String,'/follower/rotation_follow/debug',controller,10),
        node.create_subscription(JointState,'/follower/joint_states',joint_sample,qos_profile_sensor_data),
        node.create_subscription(String,'/transport/safety_status',lambda m:safety.update(json.loads(m.data)),10),
        node.create_subscription(TFMessage,'/world/cooperative_transport/pose/info',gt,qos_profile_sensor_data)]
    for robot in ('leader','follower'):
        subscriptions.append(node.create_subscription(String,'/'+robot+'/transport/state',
            lambda m,r=robot:states.__setitem__(r,json.loads(m.data)),10))
        subscriptions.append(node.create_subscription(Twist,'/'+robot+'/rover_twist',
            lambda m,r=robot:outputs.__setitem__(r,[m.linear.x,m.linear.y,m.angular.z]),10))
    goal=node.create_publisher(Float64,'/leader/object_rotation_delta',1)

    def checked_wait(predicate,timeout=60.):
        def checked():
            if safety.get('fault') or fault[0]:raise RuntimeError(safety.get('fault') or fault[0])
            return predicate()
        wait(checked,timeout)

    def require(name):
        response=service(name)
        if not response['success']:raise RuntimeError(name+': '+response['message'])
        return response

    def settle(seconds):
        initial=observations[-1]['timestamp']
        checked_wait(lambda:observations[-1]['timestamp']-initial>=seconds,max(60.,seconds*6.))

    try:
        # Transfer only previously measured identical robot/controller models.
        transfer={}
        for robot in ('leader','follower'):
            if canonical_model(generated/(robot+'.urdf'))!=canonical_model(a.source_run/(robot+'.urdf')):
                raise ValueError('safety calibration transfer: robot model mismatch')
            profile=json.loads((a.source_run/(robot+'_safety.json')).read_text())
            if profile['urdf_sha256']!=hashlib.sha256((a.source_run/(robot+'.urdf')).read_bytes()).hexdigest():
                raise ValueError('safety calibration source hash mismatch')
            for key,suffix in [('controller_yaml','_controllers.yaml'),('base_controller_yaml','_base_controller.yaml')]:
                new=generated/(robot+suffix)
                if hashlib.sha256(new.read_bytes()).hexdigest()!=profile[key+'_sha256']:
                    raise ValueError('safety controller configuration changed')
                profile[key+'_file']=str(new)
            profile.update(urdf_sha256=hashlib.sha256((generated/(robot+'.urdf')).read_bytes()).hexdigest(),
                world_sha256=hashlib.sha256((generated/'world.sdf').read_bytes()).hexdigest(),
                require_grasp_for_base_motion=True,ungrasped_base_test_only=False)
            (generated/(robot+'_safety.json')).write_text(json.dumps(profile,indent=2)+'\n')
            transfer[robot]=dict(canonical_URDF_identical=True,controller_files_identical=True)
        report['safety_transfer']=transfer
        prior=a.follower_policy.parent
        certificate=json.loads((prior/'leader_fixed_grasp_calibration.json').read_text())
        if certificate['urdf_sha256']!=hashlib.sha256((a.source_run/'leader.urdf').read_bytes()).hexdigest():
            raise ValueError('leader grasp calibration source mismatch')
        certificate['urdf_sha256']=hashlib.sha256((generated/'leader.urdf').read_bytes()).hexdigest()
        certificate['transfer_proof']='canonical URDF equality except controller path'
        grasp_file=a.output/'leader_fixed_grasp_calibration.json'
        grasp_file.write_text(json.dumps(certificate,indent=2)+'\n')
        params=yaml.safe_load((prior/'rotation_policy.yaml').read_text())['/leader/leader_object_rotation']['ros__parameters']
        duration=passive_profile_duration(a.passive_duration_s,params['rotation_duration_s'])
        report['rotation_duration_s']=duration
        params.update(grasp_calibration_file=str(grasp_file),rotation_duration_s=duration,
                      rotation_max_yaw_rad=math.radians(2*a.passive_delta_deg),actuator_ack_required=True)
        policy=a.output/'passive_leader_policy.yaml'
        policy.write_text(yaml.safe_dump({'/leader/leader_object_rotation':{'ros__parameters':params}}))
        start('evaluation_bridge',['ros2','run','ros_gz_bridge','parameter_bridge',
            '/world/cooperative_transport/remove@ros_gz_interfaces/srv/DeleteEntity',
            '/world/cooperative_transport/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'])
        start('safety',['ros2','launch','amir_compliance_control','guarded_safety.launch.py','run_directory:='+str(generated)])
        # Historical leader executable is confined to this calibration bench.
        # The production leader launch is unchanged and still requires ready.
        start('passive_leader',['ros2','run','amir_compliance_control','leader_object_rotation','--ros-args',
            '-r','__ns:=/leader','--params-file',str(policy),'-p','use_sim_time:=true',
            '-p','urdf_file:='+str(generated/'leader.urdf')])
        wait(lambda:safety.get('ready') and all(states.get(r,{}).get('local_ready') for r in ('leader','follower')),60.)
        expected=[('/'+r+'/'+topic,owner,r) for r in ('leader','follower') for topic,owner in (
            ('rover_twist','actuator_interlock'),('arm_controller/joint_trajectory','actuator_interlock'),
            ('mecanum_drive_controller/reference_unstamped','rover_twist_relay_ign'))]
        def writers_discovered():
            for topic,owner,robot in expected:
                writers=node.get_publishers_info_by_topic(topic)
                if len(writers)!=1 or writers[0].node_name!=owner or writers[0].node_namespace!='/'+robot:return False
            return True
        wait(writers_discovered,30.)
        settle(1.)
        for r in ('leader','follower'):require('/'+r+'/transport/prepare')
        require('/transport/safety/arm')
        checked_wait(lambda:all(states.get(r,{}).get('state')=='READY' for r in ('leader','follower')),30.)
        remover=node.create_client(DeleteEntity,'/world/cooperative_transport/remove')
        wait(remover.service_is_ready)
        request=DeleteEntity.Request();request.entity.name='grasp_support';request.entity.type=Entity.MODEL
        future=remover.call_async(request);wait(future.done)
        if not future.result().success:raise RuntimeError('support removal failed')
        node.destroy_client(remover)
        settle(10.)
        require('/follower/tare_wrench');settle(1.)
        # The observer reference was latched while disarmed in Gate-0.
        # Re-latching while the safety permit is active is intentionally
        # forbidden. This passive certificate derives W0/psi0 from the
        # following measured loaded static interval, without mutating it.
        require('/follower/admittance/enable')
        # XY_ONLY + zero recentering gains: arm compliance, stationary base.
        settle(2.)
        active[0]=True;phase[0]='static';settle(5.)
        if len(records)<32:raise RuntimeError('insufficient stationary local samples')
        for name,delta,index in [('positive',a.passive_delta_deg,1),('negative',-2*a.passive_delta_deg,2)]:
            phase[0]=name
            goal.publish(Float64(data=math.radians(delta)))
            checked_wait(lambda:states.get('leader',{}).get('accepted_goals',0)>=index,10.)
            checked_wait(lambda:states.get('leader',{}).get('completed_goals',0)>=index,240.)
            settle(3.)
        report['positive_negative_programs_completed']=True
        if any(abs(v)>1e-12 for v in outputs.get('follower',[1.])):
            raise RuntimeError('passive follower base received a nonzero command')
        evidence.flush()
        cert=qualify_passive(records,geometry,force_limit_N=.5,torque_limit_Nm=.05,
            evidence_sha256={str(a.output/'passive_local.jsonl'):hashlib.sha256((a.output/'passive_local.jsonl').read_bytes()).hexdigest()})
        cert.update(robot='follower',urdf_sha256=hashlib.sha256((generated/'follower.urdf').read_bytes()).hexdigest())
        pending_certificate=cert
        report.update(passed=True,minimum_signal_to_noise=cert['minimum_signal_to_noise'],
                      measured_signal_rad=cert['measured_signal_rad'])
    except Exception as exc:report['reason']=str(exc)
    finally:
        active[0]=False
        phase[0]='safe_stop'
        try:report['stop_response']=service('/transport/safety/stop')
        except Exception as exc:report['stop_response_error']=str(exc)
        initial=time.monotonic()
        try:wait(lambda:time.monotonic()-initial>=2.,5.)
        except Exception:pass
        report.update(local_samples=len(records),grasp_losses=losses,
            both_fixed_grasps_retained=all(grasp.get(r)=='attached' for r in ('leader','follower')),
            final_outputs=outputs,final_safety=safety,final_states=states,
            minimum_base_surface_gap_m=min((r['base_surface_gap_m'] for r in evaluation),default=None))
        report['stop_and_hold_confirmed']=bool(report['both_fixed_grasps_retained']
            and all(states.get(r,{}).get('state')=='SAFE_STOP' for r in ('leader','follower'))
            and all(r in outputs and np.max(abs(np.array(outputs[r])))<1e-12 for r in ('leader','follower')))
        if not report['stop_and_hold_confirmed']:report['passed']=False
        if report['passed'] and pending_certificate is not None:
            (a.output/'qualification.json').write_text(json.dumps(pending_certificate,indent=2)+'\n')
            report['certificate_issued']=True
        evidence.close();truth.close();control.close();raw_joints.close()
        state_rows=[]
        for row in control_rows:
            value=dict(t_s=row['timestamp'],wall_s=row['wall_s'],phase=row['phase'],
                follower_debug_state=row['debug']['state'],follower_debug_reason=row['debug']['reason'])
            for robot in ('leader','follower'):
                value[robot+'_state']=row['states'].get(robot,{}).get('state')
                value[robot+'_grasp']=row['grasp'].get(robot)
                for axis,u in zip(('vx_m_s','vy_m_s','wz_rad_s'),row['outputs'].get(robot,[None]*3)):
                    value[robot+'_received_output_'+axis]=u
            state_rows.append(value)
        if state_rows:
            with (a.output/'state_timeseries.csv').open('w',newline='') as f:
                writer=csv.DictWriter(f,fieldnames=list(state_rows[0]));writer.writeheader();writer.writerows(state_rows)
        with (a.output/'passive_timeseries.csv').open('w',newline='') as f:
            keys=['timestamp','phase','psi_ee_B','e_yaw_rad','geometry_valid','yaw_valid','grasp_attached']
            keys += ['q'+str(i+1)+'_rad' for i in range(5)]+['qd'+str(i+1)+'_rad_s' for i in range(5)]
            keys += ['fx_N','fy_N','fz_N','tx_Nm','ty_Nm','tz_Nm','vx_m_s','vy_m_s','wz_rad_s']
            writer=csv.DictWriter(f,fieldnames=keys);writer.writeheader()
            for row in records:
                value={k:row[k] for k in keys if k in row}
                value.update(zip(keys[7:],row['q_rad']+row['qdot_rad_s']+row['wrench_tcp']+row['base_twist']))
                writer.writerow(value)
        for sub in subscriptions:node.destroy_subscription(sub)
        node.destroy_publisher(goal)
        (a.output/'passive_result.json').write_text(json.dumps(report,indent=2)+'\n')
    return report

#!/usr/bin/env python3
"""Isolated contact fixture: passive negative control or local gripper setup.

An open-gripper payload must fall after removing its support. This detects
hidden support/attachment assistance; it is NOT a successful grasp/carry test.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import math
import xml.etree.ElementTree as ET


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--close-grippers',action='store_true')
    p.add_argument('--fault',choices=['none','contacts','nonfinite','mimic','overload'],default='none');a=p.parse_args()
    if a.fault!='none' and not a.close_grippers:raise ValueError('fault requires an established local contact grasp')
    if os.getenv('ROS_LOCALHOST_ONLY')!='1' or not os.getenv('IGN_PARTITION','').startswith('amir_cp9_') or not os.getenv('ROS_DOMAIN_ID'):
        raise ValueError('explicit isolated localhost domain and amir_cp9_ partition required')
    a.output=a.output.resolve();a.output.mkdir(parents=True,exist_ok=False)
    generated=a.output/'generated';processes=[];logs=[];suspended=[]
    report={'passed':False,'kind':'local_contact_static_hold' if a.close_grippers else 'passive_open_gripper_negative_control','hardware_motion_gate_open':False,
        'positive_contact_grasp_verified':False,'cooperative_contact_transport_verified':False,'actuator_goals_sent':0,'fault':a.fault}
    def start(name,argv):
        log=(a.output/(name+'.log')).open('w');logs.append(log)
        process=subprocess.Popen(argv,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(process);return process
    command=['ros2','launch','cooperative_transport_bringup','compliance_cp9_contact.launch.py','headless:=true',
        'force_torque_system:=true','joint_load_sensors:=true','constraint_fixture:=false','calibration_fixture:=false',
        'run_directory:='+str(generated)]
    (a.output/'command.json').write_text(json.dumps({'argv':__import__('sys').argv,'scene':command,
        **{k:os.getenv(k) for k in ('ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','IGN_PARTITION')}},indent=2)+'\n')
    sources=[Path(__file__),Path(__file__).with_name('amir_cp9_local_grasp.py'),Path(__file__).with_name('amir_cp9_observe.py'),
        Path('src/humbleble_ws/cooperative_transport_bringup/launch/compliance_cp9_contact.launch.py')]
    (a.output/'source_hashes.json').write_text(json.dumps({str(f.resolve()):hashlib.sha256(f.read_bytes()).hexdigest() for f in sources},indent=2)+'\n')
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from ros_gz_interfaces.srv import DeleteEntity,SetEntityPose
    from ros_gz_interfaces.msg import Entity
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import JointState
    from tf2_msgs.msg import TFMessage
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from geometry_msgs.msg import WrenchStamped
    from controller_manager_msgs.srv import ListControllers
    rclpy.init(args=[]);node=rclpy.create_node('cp9_passive_evaluator')
    latest={};poses={};clock=[0.];rows=[];phase=['startup'];last=[0.];grasp={}
    raw=(a.output/'samples.jsonl').open('w',buffering=1)
    def pose(msg):
        for t in msg.transforms:
            if t.child_frame_id in ('leader','follower','cooperative_payload'):
                poses[t.child_frame_id]=t.transform
    subscriptions=[node.create_subscription(Clock,'/clock',lambda m:clock.__setitem__(0,m.clock.sec+m.clock.nanosec*1e-9),qos_profile_sensor_data),
        node.create_subscription(TFMessage,'/world/cooperative_transport/pose/info',pose,qos_profile_sensor_data)]
    for robot in ('leader','follower'):
        subscriptions.append(node.create_subscription(JointState,'/'+robot+'/joint_states',lambda m,r=robot:latest.__setitem__(r,m),qos_profile_sensor_data))
        subscriptions.append(node.create_subscription(String,'/'+robot+'/grasp/debug',lambda m,r=robot:grasp.__setitem__(r,json.loads(m.data)),10))
    remover=node.create_client(DeleteEntity,'/world/cooperative_transport/remove')
    placer=node.create_client(SetEntityPose,'/world/cooperative_transport/set_pose')
    managers={r:node.create_client(ListControllers,'/'+r+'/controller_manager/list_controllers') for r in ('leader','follower')}
    def step():
        rclpy.spin_once(node,timeout_sec=.01)
        if 'cooperative_payload' in poses and time.monotonic()-last[0]>.05:
            last[0]=time.monotonic();t=poses['cooperative_payload'].translation
            rotation=poses['cooperative_payload'].rotation
            row={'wall':last[0],'sim':clock[0],'phase':phase[0],'payload_xyz':[t.x,t.y,t.z],
                'payload_quaternion_xyzw':[rotation.x,rotation.y,rotation.z,rotation.w],'grasp':dict(grasp)}
            for r,m in latest.items():
                row[r+'_q']=dict(zip(m.name,m.position))
                if r in poses:
                    tr=poses[r].translation;qt=poses[r].rotation
                    row[r+'_gt_xyz']=[tr.x,tr.y,tr.z];row[r+'_gt_quaternion_xyzw']=[qt.x,qt.y,qt.z,qt.w]
            rows.append(row);raw.write(json.dumps(row)+'\n')
    def wait(predicate,timeout):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            step()
            if predicate():return
        raise RuntimeError('timeout in '+phase[0])
    def sim_wait(seconds):
        begin=clock[0];wait(lambda:clock[0]-begin>=seconds,max(30,seconds*10))
    try:
        scene=start('scene',command)
        start('bridge',['ros2','run','ros_gz_bridge','parameter_bridge',
            '/world/cooperative_transport/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/world/cooperative_transport/remove@ros_gz_interfaces/srv/DeleteEntity',
            '/world/cooperative_transport/set_pose@ros_gz_interfaces/srv/SetEntityPose'])
        wait(lambda:len(latest)==2 and len(poses)==3 and remover.service_is_ready(),100)
        world=ET.parse(generated/'world.sdf').getroot()
        plugins=[v.get('name','') for v in world.findall('.//plugin')]
        allowed={'Physics','UserCommands','SceneBroadcaster','Contact','Sensors','ForceTorque','DartContactParameters'}
        if any(v.rsplit('::',1)[-1] not in allowed for v in plugins):raise ValueError('unapproved world force/attachment plugin')
        payload=world.find("world/model[@name='cooperative_payload']")
        if payload is None or payload.findall('joint') or payload.findall('plugin'):raise ValueError('payload assistance found')
        report['world_plugins']=plugins;report['payload_mass_kg']=float(payload.findtext('link/inertial/mass'))
        report['generated_hashes']={f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in generated.iterdir() if f.is_file()}
        phase[0]='supported';sim_wait(5.)
        supported=[v for v in rows if v['phase']=='supported']
        report['supported_z_range_m']=[min(v['payload_xyz'][2] for v in supported),max(v['payload_xyz'][2] for v in supported)]
        if not .64<report['supported_z_range_m'][0]<.68:raise ValueError('payload not initially supported')
        if a.close_grippers:
            phase[0]='local_grasp_preparation'
            contacts_process=start('contacts',['ros2','run','cooperative_transport_gazebo','finger_contact_aggregator','--ros-args',
                '-p','use_sim_time:=true','-p','robot_names:=[leader,follower]','-p','input_transport:=gazebo',
                '-p','world_name:=cooperative_transport'])
            for robot in ('leader','follower'):
                start('grasp_'+robot,['/usr/bin/python3','tools/amir_cp9_local_grasp.py','--ros-args',
                    '-r','__ns:=/'+robot,'-p','use_sim_time:=true','-p','urdf_file:='+str(generated/(robot+'.urdf'))])
            clients={r:node.create_client(Trigger,'/'+r+'/grasp/close') for r in ('leader','follower')}
            prepares={r:node.create_client(Trigger,'/'+r+'/grasp/prepare') for r in ('leader','follower')}
            wait(lambda:len(grasp)==2 and all(c.service_is_ready() for c in clients.values()),30)
            sim_wait(1.)
            for robot,c in prepares.items():
                wait(c.service_is_ready,10);future=c.call_async(Trigger.Request());wait(future.done,10)
                if not future.result().success:raise ValueError(robot+' local home not ready: '+future.result().message)
            wait(lambda:any(v['state']=='FAULT' for v in grasp.values()) or all(v['state']=='READY_TO_CLOSE' for v in grasp.values()),20)
            if any(v['state']=='FAULT' for v in grasp.values()):raise ValueError('home failed; keep supported')
            phase[0]='fixture_placement_after_home'
            wait(placer.service_is_ready,10)
            staging=json.loads((generated/'scene_staging.json').read_text())
            for robot in ('leader','follower'):
                tr=poses[robot];request=SetEntityPose.Request();request.entity.name=robot;request.entity.type=Entity.MODEL
                request.pose.position.x=staging['robots'][robot]['staged_x_m'];request.pose.position.y=tr.translation.y;request.pose.position.z=tr.translation.z
                request.pose.orientation=tr.rotation
                future=placer.call_async(request);wait(future.done,10)
                if not future.result().success:raise ValueError('simulated fixture placement failed')
                with (a.output/'fixture_placement.jsonl').open('a') as log:
                    log.write(json.dumps({'robot':robot,'sim':clock[0],'phase':phase[0],'requested_xyz':
                        [request.pose.position.x,request.pose.position.y,request.pose.position.z],
                        'grasp_states':{r:v['state'] for r,v in grasp.items()},'success':True})+'\n')
            sim_wait(1.)
            report['fixture_placement']='Gazebo model initialization after local home; never used as following feedback'
            phase[0]='local_grasp_preparation'
            for robot,c in clients.items():
                f=c.call_async(Trigger.Request());wait(f.done,10)
                if not f.result().success:raise ValueError(robot+' local gripper not ready: '+f.result().message)
                report['actuator_goals_sent']='local model-derived arm home and gripper preparation only; no base/transport goals'
            def closure_done():return any(v['state']=='FAULT' for v in grasp.values()) or (len(grasp)==2 and all(v['state']=='HOLDING' for v in grasp.values()))
            wait(closure_done,180)
            if any(v['state']=='FAULT' for v in grasp.values()):
                wait(lambda:len(grasp)==2 and all(v['state']=='FAULT' for v in grasp.values()),5)
                sim_wait(.5)
            report['grasp_before_support_removal']=dict(grasp)
            if any(v['state']!='HOLDING' for v in grasp.values()):raise ValueError('local contact grasp failed; support retained')
            report['positive_contact_grasp_verified']=True
        topics={'topics':[{'topic':'/'+r+'/'+suffix,'type':typ} for r in ('leader','follower') for suffix,typ in
            [('joint_states','sensor_msgs/msg/JointState'),('ft_sensor','geometry_msgs/msg/WrenchStamped'),
             ('mecanum_drive_controller/odometry','nav_msgs/msg/Odometry')]]}
        topics_file=a.output/'observed_topics.json';topics_file.write_text(json.dumps(topics,indent=2)+'\n')
        observer=start('observer',['/usr/bin/python3','tools/amir_cp9_observe.py','--topics-file',str(topics_file),
            '--output',str(a.output/'observation'),'--duration','5'])
        wait(lambda:observer.poll() is not None,20)
        if observer.returncode:raise RuntimeError('observer failed')
        graph={}
        for name,namespace in node.get_node_names_and_namespaces():
            graph[namespace.rstrip('/')+'/'+name]={'publishers':node.get_publisher_names_and_types_by_node(name,namespace),
                'subscriptions':node.get_subscriber_names_and_types_by_node(name,namespace)}
        (a.output/'graph.json').write_text(json.dumps(graph,indent=2)+'\n')
        phase[0]='remove_support';request=DeleteEntity.Request();request.entity.name='grasp_support';request.entity.type=Entity.MODEL
        release_pose=poses['cooperative_payload'];position=release_pose.translation;rotation=release_pose.rotation
        release_xyz=[position.x,position.y,position.z];release_q=[rotation.x,rotation.y,rotation.z,rotation.w]
        future=remover.call_async(request);wait(future.done,10)
        if not future.result() or not future.result().success:raise RuntimeError('simulation support removal failed')
        phase[0]='unsupported_closed_gripper' if a.close_grippers else 'unsupported_open_gripper';sim_wait(10. if a.close_grippers else 5.)
        after=[v for v in rows if v['phase']==phase[0]]
        report['unsupported_min_z_m']=min(v['payload_xyz'][2] for v in after)
        report['vertical_drop_m']=supported[-1]['payload_xyz'][2]-report['unsupported_min_z_m']
        report['final_gripper_positions_rad']={r:dict(zip(latest[r].name,latest[r].position)).get('Gripper') for r in latest}
        report['negative_control_detected_loss_of_support']=report['vertical_drop_m']>.01
        report['grasp_after_support_removal']=dict(grasp)
        report['unsupported_max_position_drift_m']=max(math.dist(v['payload_xyz'],release_xyz) for v in after)
        report['unsupported_max_angle_drift_rad']=max(2*math.acos(min(1.,abs(sum(x*y for x,y in zip(v['payload_quaternion_xyzw'],release_q))))) for v in after)
        report['passed']=(report['unsupported_max_position_drift_m']<.001 and report['unsupported_max_angle_drift_rad']<.01 and all(v['state']=='HOLDING' for v in grasp.values())) if a.close_grippers else report['negative_control_detected_loss_of_support']
        if a.close_grippers:
            report['local_grasp_input_boundary_passed']=all(all(t=='/clock' or t=='/transport/emergency_stop' or t.startswith('/'+robot+'/')
                for t,types in graph['/'+robot+'/cp9_local_grasp']['subscriptions']) for robot in ('leader','follower'))
            report['passed']=report['passed'] and report['local_grasp_input_boundary_passed']
            report['actuator_goals_sent']='local model-derived arm home and gripper preparation only; no base/carry goals'
            report['unsupported_static_hold_verified']=report['passed']
        if a.fault!='none' and report['passed']:
            phase[0]='injected_fault';injection=time.monotonic();report['fault_injected']=True
            if a.fault=='contacts':
                os.killpg(contacts_process.pid,signal.SIGSTOP);suspended.append(contacts_process.pid)
            elif a.fault in ('nonfinite','overload'):
                bad_pub=node.create_publisher(WrenchStamped,'/follower/finger_left_wrench',10)
                end=time.monotonic()+.4
                while time.monotonic()<end:
                    bad=WrenchStamped();bad.header.stamp.sec=int(clock[0]);bad.header.stamp.nanosec=int((clock[0]%1)*1e9);bad.wrench.force.x=float('nan') if a.fault=='nonfinite' else 27.
                    bad_pub.publish(bad);step()
                    if len(grasp)==2 and all(v['state']=='FAULT' for v in grasp.values()):break
            else:
                from copy import deepcopy
                bad_pub=node.create_publisher(JointState,'/follower/joint_states',10)
                model=ET.parse(generated/'follower.urdf').getroot()
                lower=float(model.find("joint[@name='left_inner_back_joint']/limit").get('lower'))
                end=time.monotonic()+.4
                while time.monotonic()<end:
                    bad=deepcopy(latest['follower']);bad.header.stamp.sec=int(clock[0]);bad.header.stamp.nanosec=int((clock[0]%1)*1e9)
                    bad.position[bad.name.index('left_inner_back_joint_mimic')]=lower-.001
                    bad_pub.publish(bad);step()
                    if len(grasp)==2 and all(v['state']=='FAULT' for v in grasp.values()):break
            wait(lambda:len(grasp)==2 and all(v['state']=='FAULT' for v in grasp.values()),5)
            report['both_local_grippers_fault_latency_wall_s']=time.monotonic()-injection
            counts={r:v['sent_gripper_goals'] for r,v in grasp.items()}
            for pid in suspended:os.killpg(pid,signal.SIGCONT)
            suspended.clear();phase[0]='fault_recovery';sim_wait(1.)
            report['no_automatic_gripper_restart']=all(v['state']=='FAULT' and v['sent_gripper_goals']==counts[r] for r,v in grasp.items())
            report['passed']=report['passed'] and report['no_automatic_gripper_restart'] and report['both_local_grippers_fault_latency_wall_s']<=1.
        if not report['passed']:raise RuntimeError('no fall detected: investigate contact/assistance; do not claim qualification')
    except Exception as error:
        report['error']=repr(error);report['error_phase']=phase[0]
    finally:
        report['last_grasp_debug']=dict(grasp)
        try:
            states={}
            for robot,client in managers.items():
                if not client.service_is_ready():continue
                future=client.call_async(ListControllers.Request());wait(future.done,3)
                states[robot]={v.name:v.state for v in future.result().controller}
            report['final_controller_states']=states
            report['base_arm_inactive_after_fault']=len(states)==2 and all(states[r].get(c)=='inactive' for r in states for c in ('arm_controller','mecanum_drive_controller'))
            if a.fault!='none' and report.get('fault_injected'):report['passed']=report['passed'] and report['base_arm_inactive_after_fault']
        except Exception as error:report['controller_snapshot_error']=repr(error);report['passed']=False
        report['rows']=len(rows)
        for pid in suspended:
            try:os.killpg(pid,signal.SIGCONT)
            except ProcessLookupError:pass
        for process in reversed(processes):
            try:os.killpg(process.pid,signal.SIGINT)
            except ProcessLookupError:pass
        end=time.monotonic()+8
        for process in reversed(processes):
            try:process.wait(timeout=max(.1,end-time.monotonic()))
            except subprocess.TimeoutExpired:
                try:os.killpg(process.pid,signal.SIGKILL)
                except ProcessLookupError:pass
                process.wait()
        for log in logs:log.close()
        raw.close();node.destroy_node();rclpy.shutdown()
        (a.output/'result.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
    if not report['passed']:raise SystemExit(1)


if __name__=='__main__':main()

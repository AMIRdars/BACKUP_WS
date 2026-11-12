"""External CP-9 native fixture preparation, never imported by control nodes."""
import hashlib
import json
from pathlib import Path
import signal
import os
import xml.etree.ElementTree as ET
from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.msg import Entity
from std_srvs.srv import Trigger


def prepare(*, node, robots, generated, output, start, wait, sim_wait, wall_wait,
            service, poses, now, phase, prep, monitor, report, grip_maintenance=False, physical_base_approach=False):
    """Only initialize the fixture. No pose RPC is made after this function."""
    contacts = start('contacts', ['ros2', 'run', 'cooperative_transport_gazebo', 'finger_contact_aggregator',
        '--ros-args', '-p', 'use_sim_time:=true', '-p', 'robot_names:=[leader,follower]',
        '-p', 'input_transport:=gazebo', '-p', 'world_name:=cooperative_transport'])
    # Warm up all local sensor readers before any preparation actuator moves.
    # Creating DDS participants while a grip watchdog is active can interrupt
    # delivery during discovery; retain the original freshness windows.
    start('grasp_monitor', ['ros2', 'launch', 'amir_compliance_control', 'native_grasp_monitor.launch.py',
        'run_directory:='+str(generated), 'grip_maintenance:='+str(grip_maintenance).lower()])
    wait(lambda: len(monitor) == 2 and all(v['healthy'] for v in monitor.values()), 30.)
    preparations = {}
    for robot in robots:
        preparations[robot] = start('grasp_'+robot, ['/usr/bin/python3', 'tools/amir_cp9_grasp_preparation.py',
            '--ros-args', '-r', '__ns:=/'+robot, '-p', 'use_sim_time:=true',
            '-p', 'urdf_file:='+str(generated/(robot+'.urdf'))])
    wait(lambda: len(prep) == 2 and all(v.get('local_ready') for v in prep.values()), 30.)
    sim_wait(1., check_fault=False)
    for robot in robots:
        service('/'+robot+'/grasp/prepare')
    wait(lambda: any(v['state'] == 'FAULT' for v in prep.values()) or
        all(v['state'] == 'READY_TO_CLOSE' for v in prep.values()), 25.)
    if any(v['state'] == 'FAULT' for v in prep.values()):
        raise RuntimeError('preparation home failed; support retained')
    if physical_base_approach:
        from dual_arm_preparation import physical_approach
        physical_approach(node=node,robots=robots,generated=generated,output=output,poses=poses,
                         now=now,phase=phase,wait=wait,sim_wait=sim_wait,report=report)
    else:
        phase[0] = 'fixture_initialization'
        placer = node.create_client(SetEntityPose, '/world/cooperative_transport/set_pose')
        wait(placer.service_is_ready, 10.)
        staging = json.loads((generated/'scene_staging.json').read_text())
        for robot in robots:
            request = SetEntityPose.Request()
            request.entity.name, request.entity.type = robot, Entity.MODEL
            request.pose.position.x = staging['robots'][robot]['staged_x_m']
            request.pose.position.y, request.pose.position.z = poses[robot].translation.y, poses[robot].translation.z
            request.pose.orientation = poses[robot].rotation
            future = placer.call_async(request); wait(future.done, 10.)
            if not future.result().success:
                raise RuntimeError('initial fixture placement rejected')
            with (output/'fixture_placement.jsonl').open('a') as log:
                log.write(json.dumps({'robot': robot, 'sim_time': now[0], 'phase': phase[0],
                    'states': {r: v['state'] for r, v in prep.items()},
                    'x': request.pose.position.x, 'success': True})+'\n')
        node.destroy_client(placer)
    phase[0] = 'pre_grasp'
    sim_wait(1., check_fault=False)
    phase[0] = 'DUAL_GRASP' if physical_base_approach else 'closing'
    for robot in robots:
        service('/'+robot+'/grasp/close')
    wait(lambda: any(v['state'] == 'FAULT' for v in prep.values()) or
        all(v['state'] == 'HOLDING' for v in prep.values()), 180.)
    report['preparation_final'] = dict(prep)
    if any(v['state'] != 'HOLDING' for v in prep.values()):
        failed={r:dict(state=v.get('state'),reason=v.get('reason'),first_fault_context=v.get('first_fault_context'))
                for r,v in prep.items() if v.get('state')!='HOLDING'}
        raise RuntimeError('native grasp refused; support retained: '+json.dumps(failed))
    phase[0] = 'explicit_monitor_handoff'
    wait(lambda: len(monitor) == 2 and all(v['healthy'] for v in monitor.values()), 25.)
    rejected = {}
    for robot in robots:
        client = node.create_client(Trigger, '/'+robot+'/grasp/takeover')
        wait(client.service_is_ready, 10.)
        future = client.call_async(Trigger.Request()); wait(future.done, 10.)
        rejected[robot] = not future.result().success
        node.destroy_client(client)
    if not all(rejected.values()):
        raise RuntimeError('monitor accepted handoff while preparation arm writer remained')
    report['premature_takeover_rejected'] = rejected
    # Release the arm writer while retaining the preparation grip watchdog.
    # Only retire it after the replacement is actively monitoring HOLDING.
    for robot in robots:
        service('/'+robot+'/grasp/release_arm_writer')
    wait(lambda: all(not node.get_publishers_info_by_topic('/'+r+'/arm_controller/joint_trajectory')
        for r in robots), 10.)
    for robot in robots:
        service('/'+robot+'/grasp/takeover')
    wait(lambda: all(v['state'] == 'HOLDING' for v in monitor.values()), 5.)
    for robot in robots:
        service('/'+robot+'/grasp/retire_preparation')
    wait(lambda:all(v['state']=='RELEASED' for v in prep.values()),5.)
    for process in preparations.values():
        os.killpg(process.pid, signal.SIGINT)
    wait(lambda: all(p.poll() is not None for p in preparations.values()), 10.)
    report['handoff'] = {'preparation_processes_exited': all(p.poll() is not None for p in preparations.values()),
        'preparation_arm_writers_absent': True, 'monitor_healthy': True,
        'replacement_monitor_holding_before_preparation_retirement': True,
        'no_gripper_open_or_reclose': True, 'sim_time': now[0]}
    (output/'handoff.json').write_text(json.dumps(report['handoff'], indent=2))
    return contacts

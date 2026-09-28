"""ROS integration: controllers and measured physical readiness gate launch."""
import os
import signal
import subprocess
import time
from pathlib import Path

import rclpy
from control_msgs.action import GripperCommand
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers
from rclpy.action import ActionServer
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import String

rclpy.init(args=[])
node = rclpy.create_node('startup_gate_test')
executor=SingleThreadedExecutor();executor.add_node(node)
rsp=[rclpy.create_node('robot_state_publisher',namespace='/' + name) for name in ['amir1','amir2']]
for publisher in rsp:publisher.declare_parameter('robot_description','');executor.add_node(publisher)
active = False
required = ['joint_state_broadcaster', 'arm_controller', 'mecanum_drive_controller', 'gripper_controller']
def listed(req, response):
    response.controller = [ControllerState(name=name, state='active' if active else 'inactive') for name in required]
    return response
services = [node.create_service(ListControllers, f'/{name}/controller_manager/list_controllers', listed) for name in ['amir1', 'amir2']]
def execute(goal):
    goal.succeed()
    return GripperCommand.Result()
actions = [ActionServer(node, GripperCommand, f'/{name}/gripper_controller/gripper_cmd', execute) for name in ['amir1', 'amir2']]
status = node.create_publisher(String, '/cooperative_transport/friction_grasp_status', 10)
pubs = [node.create_publisher(JointState, f'/{name}/joint_states', 10) for name in ['amir1', 'amir2']]
exe = ['/usr/bin/python3', os.environ['GATE_CANDIDATE']] if 'GATE_CANDIDATE' in os.environ else [str(Path('install/cooperative_transport_control/lib/cooperative_transport_control/simulation_startup_gate').resolve())]
processes=[]
def start(mode, timeout=10.):
    p=subprocess.Popen(exe+['--ros-args','-p',f'mode:={mode}','-p',f'timeout:={timeout}'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    processes.append(p)
    return p
def spin(duration, physical=None, velocity=0.0):
    end=time.monotonic()+duration
    while time.monotonic()<end:
        if physical is not None:
            msg=JointState(name=['Joint_1','Joint_2','Joint_3','Joint_4','Joint_5','Gripper'],position=[0.,1.4,-1.6,.2,1.570796,physical], velocity=[velocity]*6)
            for pub in pubs:pub.publish(msg)
            status.publish(String(data='READY_TO_GRASP; held=False'))
        executor.spin_once(timeout_sec=.02)
try:
    p=start('description');spin(2.5);assert p.poll() is None, 'empty description accepted'
    for publisher in rsp:publisher.set_parameters([Parameter('robot_description',value='<robot name="test"/>')])
    spin(3);assert p.poll()==0, 'valid description response not detected'
    p=start('controllers');spin(2.5);assert p.poll() is None, 'inactive controllers accepted'
    active=True;spin(3);assert p.poll()==0, 'active controllers and action readiness not detected'
    p=start('open_home');spin(2,physical=-.5);assert p.poll() is None, 'closed grippers accepted'
    spin(1.5,physical=-1.,velocity=.2);assert p.poll() is None, 'moving joints accepted'
    spin(2,physical=-1.);assert p.poll()==0, 'fresh Q_HOME and safe opening not detected'
    p=start('world',timeout=1.);spin(2.5);assert p.poll()==1, 'absent clock must fail without progressing'
    print('PASS: empty description blocks, valid description succeeds, inactive blocks, active succeeds, closed/moving blocks, measured stationary opening succeeds, absent clock timeout fails')
finally:
    for p in processes:
        if p.poll() is None:p.send_signal(signal.SIGINT);p.wait(timeout=5)
    executor.shutdown()
    for publisher in rsp:publisher.destroy_node()
    node.destroy_node();rclpy.try_shutdown()

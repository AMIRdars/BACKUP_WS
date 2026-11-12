"""Restore the measured start of a small failed trial while follower is disabled."""
import argparse
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import WrenchStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('control_log')
    parser.add_argument('--output',required=True)
    args=parser.parse_args()
    target=next(json.loads(line)['q'] for line in Path(args.control_log).open() if '"q":' in line)
    rclpy.init()
    node=Node('restore_small_trial_fixture',parameter_overrides=[Parameter('use_sim_time',value=True)])
    samples={}
    def joints(msg):
        mapping=dict(zip(msg.name,msg.position))
        samples['q']=[mapping[f'Joint_{i}'] for i in range(1,6)]
    def wrench(msg):
        w=msg.wrench
        samples['raw']=[w.force.x,w.force.y,w.force.z,w.torque.x,w.torque.y,w.torque.z]
    node.create_subscription(JointState,'/amir2/joint_states',joints,qos_profile_sensor_data)
    node.create_subscription(WrenchStamped,'/amir2/ft_sensor',wrench,qos_profile_sensor_data)
    node.create_subscription(String,'/amir2/follower/status',lambda m:samples.update(state=m.data),10)
    client=ActionClient(node,FollowJointTrajectory,'/amir2/arm_controller/follow_joint_trajectory')
    def wait(predicate):
        deadline=time.monotonic()+30
        while not predicate():
            if time.monotonic()>deadline:
                raise RuntimeError('fixture restore timeout')
            rclpy.spin_once(node,timeout_sec=.02)
    try:
        wait(lambda:len(samples)==3 and client.server_is_ready())
        if samples['state'] not in ('FAULT','DISABLED'):
            raise ValueError('disable follower before fixture restore')
        if np.max(np.abs(np.array(target)-samples['q']))>.02:
            raise ValueError('fixture restore limited to 0.02 rad from measured position')
        before=dict(samples)
        goal=FollowJointTrajectory.Goal()
        goal.trajectory.header.stamp=node.get_clock().now().to_msg()
        goal.trajectory.joint_names=[f'Joint_{i}' for i in range(1,6)]
        point=JointTrajectoryPoint(positions=target,velocities=[0.]*5)
        point.time_from_start.sec=2
        goal.trajectory.points=[point]
        sent=client.send_goal_async(goal)
        wait(sent.done)
        if not sent.result().accepted:
            raise RuntimeError('fixture restore rejected')
        done=sent.result().get_result_async()
        wait(done.done)
        result={'before':before,'target':target,'after':samples,
                'action_status':done.result().status,'controller_error_code':done.result().result.error_code}
        Path(args.output).write_text(json.dumps(result,indent=2))
        print(json.dumps(result,indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()

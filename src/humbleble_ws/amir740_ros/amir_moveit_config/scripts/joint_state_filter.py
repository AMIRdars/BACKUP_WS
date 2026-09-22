#!/usr/bin/env python3
"""
gazebo_ros2_control/GazeboSystem がミミックジョイントの ODE 拘束実装のために
生成する "<joint>_mimic" ジョイントを /joint_states から除外し、
/joint_states_filtered として再配信する。
MoveIt2 の move_group はこのトピックを購読することでエラーを回避できる。
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateFilter(Node):
    def __init__(self):
        super().__init__('joint_state_filter')
        self.pub = self.create_publisher(JointState, 'joint_states_filtered', 10)
        self.sub = self.create_subscription(
            JointState, 'joint_states', self._cb, 10)

    def _cb(self, msg: JointState):
        indices = [
            i for i, name in enumerate(msg.name)
            if not name.endswith('_mimic')
        ]
        out = JointState()
        out.header = msg.header
        out.name = [msg.name[i] for i in indices]
        out.position = [msg.position[i] for i in indices] if msg.position else []
        out.velocity = [msg.velocity[i] for i in indices] if msg.velocity else []
        out.effort = [msg.effort[i] for i in indices] if msg.effort else []
        self.pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(JointStateFilter())
    rclpy.shutdown()


if __name__ == '__main__':
    main()

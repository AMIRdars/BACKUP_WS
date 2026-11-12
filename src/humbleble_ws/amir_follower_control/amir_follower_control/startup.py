"""Reuse existing motion/grasp startup with shutdown outside ROS callbacks."""
import rclpy
import time
from control_msgs.action import FollowJointTrajectory
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from coop_transport_controller.arm_home_positioner import ArmHomePositioner
from cooperative_transport_control.dual_base_approach import DualBaseApproach


class HomeOnce(ArmHomePositioner):
    def __init__(self):
        super().__init__()
        self.declare_parameter('home_positions', [0.,1.4,-1.6,.2,1.570796]*len(self._names))
        positions = self.get_parameter('home_positions').value
        if len(positions) != 5*len(self._names):
            raise ValueError('one five-joint home posture per robot required')
        self.home_positions = {r: positions[5*i:5*i+5] for i,r in enumerate(self._names)}
        self.finished = None

    def _send_home_goals(self):
        for robot,client in self._arm_clients.items():
            trajectory = self._home_trajectory()
            trajectory.points[0].positions = self.home_positions[robot]
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = trajectory
            future = client.send_goal_async(goal)
            future.add_done_callback(lambda response,r=robot: self._on_goal_response(r,response))
        self.get_logger().info(f'Configured home positions: {self.home_positions}')

    def _finish(self, success, message):
        self._publish_zero()
        if not success:
            self._cancel_home_goals()
            self._hold_interrupted_arm()
        self.get_logger().info(message) if success else self.get_logger().error(message)
        self._timer.cancel()
        self.finished = bool(success)


class ApproachOnce(DualBaseApproach):
    def __init__(self):
        super().__init__()
        self.finished = None

    def _on_transport_active(self, msg):
        super()._on_transport_active(msg)
        if self._phase == 'HANDED_OFF':
            self.finished = True


def run(kind, args):
    rclpy.init(args=args)
    node = kind()
    result = False
    try:
        while rclpy.ok() and node.finished is None:
            rclpy.spin_once(node, timeout_sec=.1)
        result = node.finished is True
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if result else 1)


def home_main(args=None):
    run(HomeOnce, args)


def approach_main(args=None):
    run(ApproachOnce, args)


def wait_grasp_start_main(args=None):
    """Optional experiment gate: keep the no-payload calibration fixture open."""
    rclpy.init(args=args)
    node=Node('wait_calibration_before_grasp')
    accepted=[None]
    def trigger(request,response):
        accepted[0]=time.monotonic()
        response.success=True
        response.message='starting grasp after experiment handoff'
        return response
    node.create_service(Trigger,'/experiment/start_grasp',trigger)
    deadline=time.monotonic()+900.
    success=False
    try:
        while rclpy.ok() and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
            # Leave time for the asynchronous DDS service response to leave
            # the process before OnProcessExit starts the next phase.
            if accepted[0] is not None and time.monotonic()-accepted[0]>1.:
                success=True
                break
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    raise SystemExit(0 if success else 1)


def wait_grasp_main(args=None):
    rclpy.init(args=args)
    node=Node('wait_contact_verified_before_support_removal')
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    since=None
    latest=[False,0.]
    def receive(msg):
        latest[:]=[msg.data,node.get_clock().now().nanoseconds*1e-9]
    node.create_subscription(Bool,'/cooperative_transport/contact_closure_verified',receive,qos)
    deadline=time.monotonic()+600.
    success=False
    try:
        while rclpy.ok() and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
            now=node.get_clock().now().nanoseconds*1e-9
            if latest[0] and now-latest[1]<.3:
                since=now if since is None else since
                if now-since>.3:
                    success=True
                    break
            else:
                since=None
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if success else 1)

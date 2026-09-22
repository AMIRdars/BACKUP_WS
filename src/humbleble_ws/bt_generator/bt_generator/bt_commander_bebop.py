import sys
from lxml.builder import E
from lxml import etree
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from behavior_tree_msgs.action import SendGoal
from behavior_tree_msgs.srv import SetBlackBoard
from behavior_tree_msgs.msg import BBPose
from behavior_tree_msgs.msg import BBMessage
from .node_template import *

from geometry_msgs.msg import PoseStamped

# condition = Constraint('IsObjectAt', is_obj_at_xml,{'object': 0, 'at': 0})
# goalCondition = etree.fromstring(condition.xml)
# goalCondition.set("obj_id", "test_obj")
# goalCondition.set("obj_pose", "{test_obj_pose}")

# goalCondition.set("location_id", "test_location")
# goalCondition.set("location_pose", "{test_location_pose}")
# goalCondition.tag = "IsObjectAt"  #応急処置

#わたしの
condition = Constraint('LowTask',is_low_task_xml,{'ltheight':0})
goalCondition = etree.fromstring(condition.xml)
goalCondition.tag = "LowTask"


# goal_conditions = [etree.tostring(goalCondition, encoding="unicode", pretty_print=True)]

#######################################
# condition = constraints['IsRobotCloseTo']
# goalCondition = etree.fromstring(condition.xml)
# goalCondition.set("location_id", "area1")
# goalCondition.set("location_pose", "{area1_pose}")

# # goalCondition.set("location_id", "test_location")
# # goalCondition.set("location_pose", "{test_location_pose}")
# goalCondition.tag = "IsRobotCloseTo"  #応急処置
goal_conditions = [etree.tostring(goalCondition, encoding="unicode", pretty_print=True)]


# bb_req = SetBlackBoard.Request()
bb_message = BBMessage()

bb_pose = BBPose()
bb_pose.key = "test_obj_pose"
bb_pose.value.header.frame_id = "map"
bb_pose.value.pose.position.x= 0.0
bb_pose.value.pose.position.y= -1.0
bb_pose.value.pose.orientation.w = 1.0
# pose.

bb_message.poses.append(bb_pose)
##################################################
bb_location_pose = BBPose()
bb_location_pose.key = "test_location_pose"
bb_location_pose.value.header.frame_id = "map"
bb_location_pose.value.pose.position.x=-0.5
bb_location_pose.value.pose.position.y=-2.5
bb_location_pose.value.pose.position.z= 0.01 #   １段
bb_location_pose.value.pose.orientation.w = 1.0
bb_message.poses.append(bb_location_pose)



class BT_Client(Node):

    def __init__(self):
        super().__init__('bt_commander')
        self._action_client = ActionClient(self, SendGoal, 'send_goal')
        self.bb_client = self.create_client(SetBlackBoard, 'set_blackboard_bt')
        while not self.bb_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

    def send_goal(self):
        # # BB書き込み
        # self.future = self.bb_client.call_async(bb_req)
        # if self.future.done():
        #     try:
        #         response = self.future.result()
        #     except Exception as e:
        #         self.get_logger().info(
        #             'Service call failed %r' % (e,))

        # 指令
        goal_msg = SendGoal.Goal()
        goal_msg.goal_states = goal_conditions
        goal_msg.bb_message = bb_message
        
        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)

    action_client = BT_Client()
    future = action_client.send_goal()

    rclpy.spin_until_future_complete(action_client, future)


if __name__ == '__main__':
    main()
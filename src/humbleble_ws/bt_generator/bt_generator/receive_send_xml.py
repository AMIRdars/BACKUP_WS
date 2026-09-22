import sys
from lxml.builder import E
from lxml import etree
import rclpy
from rclpy.node import Node
# from behavior_tree_msgs.srv import GetBT
from rclpy.action import ActionClient
from behavior_tree_msgs.action import ExecuteTree
from behavior_tree_msgs.action import SendGoal
# from behavior_tree_msgs.srv import SetBlackBoard
# from behavior_tree_msgs.msg import BBPose
# from behavior_tree_msgs.msg import BBMessage
# from behavior_tree_msgs.msg import BBCollisionObject
# from moveit_msgs.msg import CollisionObject
from .node_template import *
from std_msgs.msg import Float64
import os




def get_xml():
    dirname = os.path.dirname(__file__)
    parent_dir = '/'.join(dirname.split('/')[:-1]) 
    xml_dir = parent_dir + '/bt_xml/navigation.xml'

    with open(xml_dir, mode='rt', encoding='utf-8') as f:
        xml_string = f.read()

    root = ET.fromstring(xml_string)
    ret = ET.tostring(root, encoding='unicode')
    return ret



class BTGeneratorService(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self._action_client = ActionClient(self, SendGoal, 'send_goal')
        # self._execute_tree_client = ActionClient(self, ExecuteTree, 'execute_bt') #   ツリーをexecuterに送信するアクションクライアント
 
        self.create_subscription(Float64,"/action_start",self.callback,10) #メカナムからBTの開始の合図を送る

    # def send_goal(self):
    #     goal_msg = ExecuteTree.Goal()
    #     goal_msg.bt = get_xml()
    #     self._execute_tree_client.wait_for_server()

    #     return self._execute_tree_client.send_goal_async(goal_msg)

    def callback(self, msg):
        self._execute_tree_client = ActionClient(self, ExecuteTree, 'execute_bt') #   ツリーをexecuterに送信するアクションクライアント
        goal_msg = ExecuteTree.Goal()
        goal_msg.bt = get_xml()
        self._execute_tree_client.wait_for_server()
        future = self._execute_tree_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        # return self._execute_tree_client.send_goal_async(goal_msg)        





def main():
    rclpy.init()
    # cwd = os.getcwd()
    # xml_string = get_xml()
    # print(xml_string)
    # root = root.getroot()
    # print(root)
    # tree = etree.tostring(root, encoding="unicode", pretty_print=True)
    # print(tree)
    bt_generator = BTGeneratorService()
    rclpy.spin_once(bt_generator)
    # future = bt_generator.send_goal()

    #future = bt_generator.send_goal(10)

    # rclpy.spin_until_future_complete(bt_generator, future)
    # rclpy.spin_until_future_complete(bt_generator)



if __name__ == '__main__':
    main()
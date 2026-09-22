import sys
from lxml.builder import E
from lxml import etree
import rclpy
from rclpy.node import Node
# from behavior_tree_msgs.srv import GetBT
from rclpy.action import ActionClient
from behavior_tree_msgs.action import ExecuteTree
from .node_template import *

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

        self._execute_tree_client = ActionClient(self, ExecuteTree, 'execute_bt') #   ツリーをexecuterに送信するアクションクライアント
 


    def send_goal(self):
        goal_msg = ExecuteTree.Goal()
        goal_msg.bt = get_xml()
        self._execute_tree_client.wait_for_server()

        return self._execute_tree_client.send_goal_async(goal_msg)

        




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
    future = bt_generator.send_goal()

    #future = bt_generator.send_goal(10)

    rclpy.spin_until_future_complete(bt_generator, future)



if __name__ == '__main__':
    main()
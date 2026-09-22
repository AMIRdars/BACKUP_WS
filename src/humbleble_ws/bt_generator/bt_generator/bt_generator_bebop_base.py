import sys
import os
from typing import no_type_check_decorator
from lxml.builder import E
from lxml import etree
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action import ActionClient
from rclpy.action import CancelResponse
from rclpy.executors import MultiThreadedExecutor
from behavior_tree_msgs.action import ExecuteTree
from threading import Event
import numpy as np
from collections import deque
from behavior_tree_msgs.srv import GetBT
from behavior_tree_msgs.action import SendGoal
from behavior_tree_msgs.msg import BTStatus
from behavior_tree_msgs.msg import NodeStatus
from behavior_tree_msgs.srv import SetBlackBoard
from behavior_tree_msgs.srv import CreateBT
from behavior_tree_msgs.msg import BBPath
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter
from .node_template import *

def get_xml():
        dirname = os.path.dirname(__file__)
        parent_dir = '/'.join(dirname.split('/')[:-1]) 
        xml_dir = parent_dir + '/bt_xml/navigation.xml'

        parser = etree.XMLParser(remove_blank_text=True, recover=True, remove_comments=True)
        with open(xml_dir, mode='rt', encoding='utf-8') as f:
            tree = etree.parse(f, parser=parser)
        # print(etree.tostring(tree).decode())
        root = tree.getroot()
        return root





    




#   BTアクションに対応するBTを生成するためのクラス
class BTGeneratorBase(Node):
    solutions = []
    def __init__(self, node_name):
        super().__init__(node_name)
        self.bt_name = node_name    #   btのアクション識別名
        self.bt_status = NodeStatus.IDLE
        self.bb_req = SetBlackBoard.Request()
        self.bb_dict = {}   #   BBのキー：　値
        self.splution_bb_poses = []
        self.root_status = NodeStatus.IDLE
        self.node_status_dict = {}  #   BTノードの状態の辞書
        self.solition_dict = {}
        self.execute_tree_done_event = Event()
        self.bb_client = self.create_client(SetBlackBoard, 'set_blackboard_' + self.bt_name)

        #   bt_executorにツリーに準備してもらうためのサービスクライアント（サーバーが複数のBTを実行するためには複数のアクション通信を用意する必要があるため）
        self.create_bt_client = self.create_client(CreateBT, 'create_bt') 
 
        
        # while not self.bb_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('service not available, waiting again...')
       
    #   BBを一括で辞書に保存
    def set_bb_dict(self, bb_message):
        for bb_pose in bb_message.poses:
            self.bb_dict[bb_pose.key] = bb_pose.value
        for bb_path in bb_message.paths:
            self.bb_dict[bb_path.key] = bb_path.value
        for float_array in bb_message.float_array_set:
            self.bb_dict[float_array.key] = float_array.value

    #   BB追加
    def set_bb_req(self, bb_message):
        self.bb_req.bb_message.poses.extend(bb_message.poses)
        self.bb_req.bb_message.paths.extend(bb_message.paths)
        self.bb_req.bb_message.float_array_set.extend(bb_message.float_array_set)

       

    def cancel_callback(self, cancel_request):
        future = self.execute_tree_goal_handle.cancel_goal_async()
        future.add_done_callback(self.goal_canceled_callback)

        return CancelResponse.ACCEPT

    def goal_canceled_callback(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Cancelling of goal complete')
        else:
            self.get_logger().warning('Goal failed to cancel')
            
            
    def set_black_board(self):
        self.service_request(self.bb_client, self.bb_req)

    def create_bt_request(self):
        req = CreateBT.Request()
        req.bt_name = self.bt_name
        res = self.service_request(self.create_bt_client, req)
        print(res)


    #   BTの実行
    def execute_tree(self, bt):
        self.node_status_dict = {}  #   ノードの状態を初期化
        self.solition_dict = {} #   解決策を初期化
        # print(bt)
        self.bt_status = NodeStatus.RUNNING
        self.set_black_board()  #   behavior treeに変数を登録
        goal_msg = ExecuteTree.Goal()
        goal_msg.bt = bt
        execute_tree_client = ActionClient(self, ExecuteTree, 'execute_'+ self.bt_name) #   ツリーをexecuterに送信するアクションクライアント
        execute_tree_client.wait_for_server()
        self.execute_tree_done_event.clear()
        execute_tree_future = execute_tree_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        execute_tree_future.add_done_callback(self.execute_tree_response_callback)

        # Wait for action to be done
        self.execute_tree_done_event.wait()

        self.get_logger().info('Execute BT Finished')
        if self.root_status == NodeStatus.FAILURE:
            self.get_logger().warning('ROOT FAILURE')
            return False
        else:
            return True

         

    def execute_tree_response_callback(self, future):
        self.execute_tree_goal_handle = future.result()
        if not self.execute_tree_goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._execute_tree_result_future = self.execute_tree_goal_handle.get_result_async()
        self._execute_tree_result_future.add_done_callback(self.execute_tree_result_callback)


    #   BTのリザルト
    def execute_tree_result_callback(self, future):
        print("execute_tree_result_callback")
        result = future.result().result
        for node_status in result.bt_status.bt_status:
            if node_status.node_name.startswith('MainTree') is False:
                continue
            print(node_status)
            self.node_status_dict[node_status.node_name] = node_status


        self.bt_status = result.bt_status.root_status
        self.execute_tree_done_event.set()

    #   BTのフィードバック(ノードの状態を保存)
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.root_status = feedback.bt_status.root_status.status

        for node_status in feedback.bt_status.bt_status:
            if node_status.node_name.startswith('MainTree') is False:   #   subtreeは取り敢えず考えない
                continue
            # print(node_status)
            if node_status.status != NodeStatus.IDLE:   #   IDLEになると結果がわからなくなるので除外
                self.node_status_dict[node_status.node_name] = node_status



    #   実行中のBTをキャンセルする
    def cancel_tree(self):
        self.get_logger().info('Canceling goal')
        future = self.execute_tree_goal_handle.cancel_goal_async()
        future.add_done_callback(self.tree_canceled_callback)

    def tree_canceled_callback(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Cancelling of goal complete')
        else:
            self.get_logger().warning('Goal failed to cancel')

    
    # def get_bt_node_type(self, node_name):
    #     if self.node_status_dict[node_name].node_type == NodeStatus.CONDITION:
    #         return 
        




    ###################################################################################################

    #   汎用サービスリクエスト関数
    def service_request(self, client, req):
        future = client.call_async(req)
        while rclpy.ok():
            if future.done():
                try:
                    response = future.result()
                except Exception as e:
                    self.get_logger().info(
                        'Service call failed %r' % (e,))
                else:
                    return response
                break

    #   パラメータを設定
    def set_param(self, node_name, param_msgs):
        param_client = self.create_client(SetParameters, node_name)
        req = SetParameters.Request()
        req.parameters = param_msgs
        self.service_request(param_client, req)


    #   パラメータメッセージ生成（値はstringでもboolでも大丈夫）
    def get_param_msg(self, param_name, param_value):
        param = Parameter(name=param_name, value=param_value).to_parameter_msg()
        return param





    #   汎用アクションクライアント関数
    def action_send(self, action_client, goal_msg):
        action_done_event = Event()
        action_done_event.clear()
        result = None


        def get_result_callback(future):
            nonlocal result
            result = future.result().result
            action_done_event.set()

        def goal_response_callback(future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().info('Goal rejected :(')
                return

            self.get_logger().info('Goal accepted :)')

            self._get_result_future = goal_handle.get_result_async()
            self._get_result_future.add_done_callback(get_result_callback)


        send_goal_future = action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(goal_response_callback)

        # Wait for action to be done
        action_done_event.wait()


        return result


######################################################################################

    def get_node_name(self, node):
        if node.tag == "SubTreePlus":
            return node.attrib["ID"]    
        else:
            return node.tag









# class MinimalService(Node):

#     def __init__(self):
#         super().__init__('minimal_service')
#         self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

#     def add_two_ints_callback(self, request, response):
#         response.sum = request.a + request.b
#         self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

#         return response

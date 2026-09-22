import sys
import os
from typing import no_type_check_decorator
from lxml.builder import E
from lxml import etree
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action import ActionClient
from behavior_tree_msgs.action import ExecuteTree
import numpy as np
from behavior_tree_msgs.srv import GetBT
from behavior_tree_msgs.action import SendGoal
from behavior_tree_msgs.msg import BTStatus
from behavior_tree_msgs.msg import NodeStatus
from .bt_generator_base import *
from .node_template import *
import copy
import re
# from deepdiff import DeepDiff



dict = {}
parallel = []
def getXmlFromConditionString(goal_condition):
    goalCondition = etree.fromstring(goal_condition)
    # for c in constraints.values() :
    #     element = etree.fromstring(c.xml)
    #     if goalCondition.tag == element.tag :
    #         # 例：print(element.tag) 結果：IsObjectAt
    #         condition = c
    condition = None
    for c in constraints.keys() :
        if goalCondition.tag == c :
            # 例：print(element.tag) 結果：IsObjectAt
            condition = constraints[c]
            goalCondition.tag = etree.fromstring(condition.xml).tag
    
    if condition is None:
        print("invalid condition")
        sys.exit()

    
    condition.xml = etree.tostring(goalCondition, encoding="unicode", pretty_print=True)
    return goalCondition, condition

def getConditionFromXml(xml):
    condition = None
    # print("xml = ", xml)
    for c in constraints.values() :
        element = etree.fromstring(c.xml)
        if xml.tag == element.tag :
            condition = c
            condition.xml = etree.tostring(xml, encoding="unicode", pretty_print=True)
            # print("condition found type = ", condition.type)
    return condition


class BTGenerator(BTGeneratorBase):
    def __init__(self):
        super().__init__('bt')
        # self.nodeID = 0#どのノードか判別する用
        self.bt_xml = None    #   実行中のbt
        self.recovery_fallback = None
        self.pose_dict = {}

    
    def createInitialTree(self, goal_conditions):
        root = etree.Element("root")
        root.attrib['main_tree_to_execute']="MainTree"
        main_tree = etree.Element("BehaviorTree")
        main_tree.attrib['ID']="MainTree"

        sequenceStar = etree.Element("Sequence") #最初をシーケンスにすることで複数の成約を満たせるようにする
        main_tree.append(sequenceStar)

        for goal_condition in goal_conditions:
            fallback = etree.Element("ReactiveFallback")
            fallback.attrib["ID"] = "resolve_fallback"    #   サブツリーに置き換えたFallbackということを記録  

            goalCondition, condition = getXmlFromConditionString(goal_condition)

            fallback.append(goalCondition)
   
            sub_bt = self.get_subtree(condition)  # condition(xml)を満たすサブツリー生成
            fallback.append(sub_bt)
            sequenceStar.append(fallback)

        root.append(main_tree)   

        ##############アクションのサブツリーの追加　TODO:executor側でファイルを開いたほうが効率いいはず####################
        nav_bt_root = self.get_xml()    #   サブツリーの取得
        for sub_tree in nav_bt_root:
            if sub_tree.attrib["ID"] != "MainTree":
                print(sub_tree.attrib["ID"])
                print(type(sub_tree))
                root.append(sub_tree)   

        ####################################
        for child in root:
            self.set_id(child)   #   ノードにidをセット
        self.bt_xml = root  #   最終的なツリーを保存
        tree = etree.tostring(root, encoding="unicode", pretty_print=True)
        
        return tree

    # def predict_error(self, failed_action):	
    #     rospy.wait_for_service('/error_detection')
    #     try:
    #         error_detection_client = rospy.ServiceProxy('/error_detection', ErrorDetection)
            
    #         # req1.visualSupport = [w1]
    #         # resp1 = bt(req1.type, req1.bt, req1.goal_condition )
    #         req = ErrorDetection()
    #         req.error_name = failed_action
    #         req.related_action_name = ""
    #         resp = error_detection_client(req.error_name, req.related_action_name)
    #         # print(resp1.behavior_tree)
    #         print(resp.result)

    #     except rospy.ServiceException as e:
    #         print("Service call failed: %s"%e)

    #     return resp.result

    #元のツリー、失敗したノード
    def updateTree(self):
        tree = None
        # print("!!Failed_condition = ",failed_node_name)
        # if (str(goal_condition) == "PickProjected" or str(goal_condition) == "PlaceProjected" or str(goal_condition) == "ApproachProjected"):
        #     self.predict_error(goal_condition)

        root = self.bt_xml

        #   前回のリカバリーノードを削除
        if self.recovery_fallback != None:
            fallback = self.recovery_fallback
            fallback.getparent().remove(fallback)
            self.recovery_fallback = None


        foundElement = []

        main_tree = None
        #   アップデートするノードを取得
        failed_node = None
        for subtree in root:
            if subtree.attrib["ID"] == "MainTree":
                main_tree = subtree
                failed_node = self.get_failed_node(subtree)
                break

        foundElement.append(failed_node)      

        

        #   conditionノードの場合サブツリーに置換
        if self.node_status_dict[failed_node.attrib["name"]].node_type == NodeStatus.CONDITION:
            #   謎
            found = None
            # print(foundElement)
            for i in foundElement :
                if i.tag == "IsRobotCloseTo"  :
                    print("foundElement.attrib",i.attrib)
                    # dict[i] = [i, i.tag, i.attrib['location_pose']]
                    found = i
                    break
                elif i.tag == "IsPathValid" :
                    dict[i] = [i, i.tag]
                    found = i
                    print ("found IsPathValid")
                else :
                    if 'obj_id' in i.attrib :
                        dict[i.tag] = i.attrib['obj_id']
                    else :
                        dict[i] = [i, i.tag]
                    found = i
                    break
            # if found.tag == "IsRobotCloseTo"  :
            #     for key in dict :
            #         if dict[key][1] == "IsRobotCloseTo" :
            #             found = dict[key][0]
            # print("found= ", found)
            if found is None :
                print("not found ...")
                return tree

            
            index = found.getparent().index(found)
            # print('index= ', index)
            fallback = etree.Element("ReactiveFallback")
            fallback.attrib["ID"] = "resolve_fallback"    #   サブツリーに置き換えたFallbackということを記録  
            parent = found.getparent()
            #満たしていない成約をサブツリーに置き換える
            found.getparent().remove(found)
            fallback.append(found)

            #障害物だったらどける
            if found.tag == "IsPathCollFree"  :
                print("Path is not Free!!!")
            # print("[]: ", found)
            # print("found=",found)   #found= <Element IsRobotCloseTo at 0x7f0de0533d48>

            condition = getConditionFromXml(found)
            if not condition :
                return tree
            sub_bt = self.get_subtree(condition)

            fallback.append(sub_bt)
            parent.insert(index, fallback)

            

        #   actionノードの場合、失敗原因を推定して解決策を生成する
        elif self.node_status_dict[failed_node.attrib["name"]].node_type in [NodeStatus.ACTION, NodeStatus.SUBTREE]:
            
            failed_action_node = foundElement[0]
            print("resolve", failed_action_node)
            
            # parent = failed_action_node.getparent() #   失敗ノードの親を取得
            # index = parent.index(failed_action_node)    #   失敗ノードが親の何番目か取得
            
            # foundElement[0].

           

            temp_tree_list = []
            if failed_node.attrib["name"] in self.solition_dict.keys():
            
                # for state in temp_tree.iter('state'):

                # once_node = etree.Element("Once")   #   応急処置は一度だけ（あくまでも前回の失敗に対するリカバリで再度実行しない）
                edited_index = len(list(main_tree.iter()))  #   最も実行順の早い（深さ優先）編集したノードのインデックス
                #   各解決策
                for recovery_node_xml in self.solition_dict[failed_node.attrib["name"]]:

                    #   プランを複製してあとで共通部分を一つのツリーにする？
                    temp_tree = copy.copy(main_tree)   
                    #   複製したツリー中の失敗したアクションノードを取得
                    temp_failed_node = temp_tree.xpath("//" + failed_action_node.tag + "[@name='" + failed_action_node.attrib["name"] + "']")[0]

                    #   失敗ノードの親を取得
                    reactive_sequence = temp_failed_node.getparent()
                    print("reactive_sequence???", reactive_sequence)

                    #   edited_indexの更新
                    for i, node in enumerate(temp_tree.iter()):
                        if node == reactive_sequence:
                            if i < edited_index:
                                edited_index = i

                    #   失敗したアクションの事前条件を取り出す
                    pre_conditions = self.get_pre_condition_nodes(temp_failed_node)
                    print(pre_conditions)

                    recovery_node = etree.fromstring(recovery_node_xml)

                    #   リカバリ方法が事前条件追加の場合
                    if recovery_node.tag in constraints.keys():
                        print("aaaaaaaaaaaaaaa")
                        for i, pre_condition in enumerate(pre_conditions):
                            print("BBBBBBBBBBBBBBBB")
                            #   元々の事前条件と同じタイプの場合、合体させる。そのノードを満たすためのサブツリーを生成する必要
                            if pre_condition.tag == recovery_node.tag:
                                new_condition = self.merge_conditions(pre_condition, recovery_node)
                                print("CCCCCCCCCCCCCCCCCCCCc")
                                # マージした条件ノードを置換  
                                # index = reactive_sequence.index(pre_condition)
                                print("!!remove=",list(reactive_sequence)[i])   #   i番目の事前条件を削除する
                                reactive_sequence.remove(list(reactive_sequence)[i])    #   対応する事前条件（置換したReactiveFallbackも含む）を削除。

                                #   新しい条件を満たすためのサブツリーを生成する
                                reactive_fallback = etree.Element("ReactiveFallback")
                                sub_bt = self.get_subtree(getConditionFromXml(new_condition))
                                reactive_fallback.append(new_condition)
                                reactive_fallback.append(sub_bt)

                                #   一度だけ満たせばいいものとする（仮）
                                once_node = etree.Element("Once")
                                once_node.append(reactive_fallback)

                                #   上記サブツリーを追加
                                reactive_sequence.insert(i, once_node)
                                # reactive_sequence.insert(i, reactive_fallback)
                                
                                # pre_condition = 
                                print("aaa")
                    elif recovery_node.tag in actions.keys():
                        print("bbb")
                        temp_failed_node.addprevious(recovery_node)
                    temp_tree_list.append(temp_tree)    #   各解決策の場合の全体のツリーを追加する

                print("edited_index", edited_index)

                fallback = list(main_tree.iter())[edited_index -1] #   fallbackのはず

                #   リカバリの選択肢選択ようにツリーを生成（TODO：今はFallbackだけどPriorityにする予定）
                recovery_fallback = etree.Element("Fallback")   
                for tree in temp_tree_list:
                    subtree = list(tree.iter())[edited_index]
                    print(subtree)
                    recovery_fallback.append(subtree)
                # fallback.append(recovery_fallback)
                fallback.insert(0, recovery_fallback)   #   リカバリツリーを最優先（一番左）にしておく（最後に元々のツリーを実行するけど保存しておいてリカバリ終了後に戻したほうがいい）
                self.recovery_fallback = recovery_fallback  #   追加したリカバリツリーを保存

            else:
                print("NOT FOUND SOLUTION!!")
                # rclpy.shutdown()
                # sys.exit("NOT FOUND SOLUTION!!")
        
        else:
            print("??????")
            # tree = etree.tostring(root, encoding="unicode", pretty_print=True)  #   再実行

        conditions = {'hand': None, 'robot_location_id': None}

        # print(etree.tostring(root, encoding="unicode", pretty_print=True))
        feasible = self.is_tree_feasible(root[0], conditions)
        print(conditions)
        print("!!!!!feasible",feasible)

        #   stringに変換
        for child in root:
            self.set_id(child)   #   ノードにidをセット
            # self.set_id(root)   #   ノードにidをセット
            

        self.bt_xml = root  #   最終的なツリーを保存
        tree = etree.tostring(root, encoding="unicode", pretty_print=True)
        return tree
    

    #   条件ノードの結合
    def merge_conditions(self, node_a, node_b):
        if node_a.tag == "IsRobotCloseTo":

            pose_a_id = re.sub("[{}]","", node_a.attrib["location_pose"])   #   {}を除去
            print("pose_a_id=",pose_a_id)
            pose_a = self.bb_dict[pose_a_id]

            pose_a_radius = float(node_a.attrib["outer_radius"])

            pose_b_id = re.sub("[{}]","", node_b.attrib["location_pose"])
            print("pose_b_id=",pose_b_id)
            pose_b = self.bb_dict[pose_b_id] #   BBキーに対応する値の取得

            d = np.sqrt(pow(pose_a.pose.position.x - pose_b.pose.position.x, 2) + pow(pose_a.pose.position.y - pose_b.pose.position.y, 2))
            #   pose_bがpose_a内にあればそのままpose_bを返す
            if d < pose_a_radius:
                return copy.copy(node_b)
            else:
                print("conflict!!")
                return None


    ############################################################################################################
    def get_subtree(self, constraint):
            print("constraint", constraint)

            bt = None
            fallback = etree.Element("Fallback")
             
            # print(constraint.type)
            temp = []
            for action in actions.values():
                # print('Trying with action', action.name, 'Effects', action.effects, 'Condition fluents', fluent.parameters_dict.keys())
                #アクションの効果に成約を満たすものがある場合(同じアクションでも目標が違えば別のものとみなすべきでは？)
                if set(action.effects).issubset(set(constraint.params.keys())):
                    # print('The action ', action.name, ' can hold ', fluent.name)
                    
                    
                    action_xml_set = self.set_action_values_from_condition(action, constraint)
                    print("action_xml_set", action_xml_set)
                    for action_xml in action_xml_set:
                        #アクションの事前条件追加
                        bt = etree.Element("ReactiveSequence")
                        # bt.attrib["name"] = "Sequence:" + action_xml.tag  #識別用
                        for c in action.precond:
                            c_xml = self.set_condition_values_from_action(c, action_xml)
                            bt.append(c_xml)
                        #最後にアクションを追加
                        bt.append(action_xml)
                        fallback.append(bt)
                    return fallback
                        # print("action_xml=", action_xml)  #action_xml= <Element PlaceProjected at 0x7f786d52db08>
                        # temp.append(action_xml)
            #成約を満たし得るアクションが複数ある場合
            if len(temp) > 1 and not parallel:
                if constraint.type == "IsPathValid" :
                    tree =  etree.Element("Parallel")
                    print("setting parallel")
                    for t in temp :
                        tree.append(t)
                    tree.set("success_threshold","1")
                    tree.set("failure_threshold","1")
                    parallel.append(tree)
                    return tree
                #通常はfallbackで順に生成（最適化の余地あり）
                tree =  etree.Element("Fallback")
                for t in temp :
                    tree.append(t)
                return tree
            if bt is None:
                raise Exception('Cannot find action with effects', constraint.params.keys())
            return bt

   
    # def get_poses_by_group(self, group_id):	
    #     rospy.wait_for_service('/get_obj_by_type')
    #     try:
    #         error_detection_client = rospy.ServiceProxy('/get_obj_by_type', get_objects)
    #         resp = error_detection_client(group_id)
    #         print(resp.target_poses)

    #     except rospy.ServiceException as e:
    #         print("Service call failed: %s"%e)

    #     return resp.target_poses

    # def get_pose_by_id(self, search_id):	
    #     rospy.wait_for_service('/get_obj')
    #     try:
    #         get_pose_by_id_client = rospy.ServiceProxy('/get_obj', get_obj)
    #         resp = get_pose_by_id_client(str(search_id))

    #     except rospy.ServiceException as e:
    #         print("Service call failed: %s"%e)

    #     return resp.target_pose

    def set_action_values_from_condition(self, action, condition):
        action_xml_set =[]
        
        c = etree.fromstring(condition.xml)
        
        
        # #グループの場合該当するidを取得する
        # c_pose_set=[]
        # if 'id' in c.attrib :
        #     if c.attrib['id'].startswith('GROUP_'):
        #         group_id = c.attrib['id'].replace('GROUP_', '')
        #         c_pose_set = self.get_poses_by_group(group_id)
        #     else:
                
        #         search_id = c.attrib['id']
        #         print("input id=",search_id)
        #         pose = self.get_pose_by_id(search_id)
        #         c_pose_set.append(pose)
        #         print("pose=",pose)

        # for object in c_pose_set:

        action_xml = etree.fromstring(action.xml)
        print("action_xml", action_xml)

        if action.cos == "pose2d" :
            action_xml.attrib['goal'] = c.attrib['location_pose']   #
            action_xml.attrib['position_tolerance'] = c.attrib['outer_radius'] 
            
        elif action.cos == "obj" :
            if 'obj_id' in c.attrib :
                action_xml.attrib['obj_id'] = c.attrib['obj_id']
                action_xml.attrib['obj_pose'] = "{" + c.attrib['obj_id'] + "_pose}"

        elif action.cos == "at" :
            action_xml.attrib['location_pose'] = c.attrib['location_pose']
            # print(c.attrib['target_pose'])
            if 'obj_id'in action_xml.attrib:
                action_xml.attrib['obj_id'] = c.attrib['obj_id']
            if 'obj_pose'in action_xml.attrib:
                action_xml.attrib['obj_pose'] = "{" + c.attrib['obj_id'] + "_pose}"
            
            if 'location_id'in action_xml.attrib:
                action_xml.attrib['location_id'] = c.attrib['location_id']
            if 'location_pose'in action_xml.attrib: 
                action_xml.attrib['location_pose'] = "{" + c.attrib['location_id'] + "_pose}"
    
            # action_xml.attrib['at'] = globalObjs[action_xml.attrib['location']]["pose"]
        
        action_xml_set.append(action_xml)
        
        return action_xml_set

    def set_condition_values_from_action(self, condition, action_xml):
        c = etree.fromstring(condition.xml)
        #   サブツリー用
        action = None
        for a in actions.values() :
            if action_xml.tag == "SubTreePlus":
                if a.action == action_xml.attrib['ID'] :
                    action = a

            elif a.action == action_xml.tag :
                action = a
            
        print("action found = ", action.action)

        if action.cos == "at" :
            if 'obj_id' in c.attrib and 'obj_id' in action_xml.attrib:
                c.attrib['obj_id'] = action_xml.attrib['obj_id']
            if 'obj_pose' in c.attrib:
                c.attrib['obj_pose'] = "{" + c.attrib['obj_id'] + "_pose}"
            if 'location_id' in action_xml.attrib and 'location_id' in c.attrib:
                c.attrib['location_id'] = action_xml.attrib['location_id']
            if 'location_pose' in c.attrib:
                c.attrib['location_pose'] = "{" + c.attrib['location_id'] + "_pose}"

        if action.cos == "obj" :
            if action_xml.tag == "SubTreePlus" :
                if action_xml.attrib['ID'] == "Pick" and c.tag == "IsRobotCloseTo":
                    c.attrib['location_id'] = action_xml.attrib['obj_id']
                    c.attrib['location_pose'] = action_xml.attrib['obj_pose']
        else :
            print("inside else") 
            if 'obj_id' in c.attrib:
                c.attrib['obj_id'] = action_xml.attrib['obj_id']
                print("set obj_id")   

        return c

    
    #  幅優先探索で失敗したノードを取得（fallbackの子ノードは無視）
    def get_failed_node(self, main_tree):
        print("get_failed_node")
        d = deque([main_tree])
        while d:
            el = d.popleft() 

            if el.getparent().tag != "ReactiveFallback":    #   親がReactiveFallbackの場合は満たしていない条件を満たすためのサブツリーを実行するので除外
                if el.attrib["name"] in self.node_status_dict:  #   ノードの状態辞書にあるか確認（BehaviorTreeのtagなどは含まれない）
                    node_status = self.node_status_dict[el.attrib["name"]]
                    # print(el.attrib["name"], node_status)

                    if node_status.node_type in [NodeStatus.ACTION, NodeStatus.CONDITION, NodeStatus.SUBTREE]:
                        if node_status.status == NodeStatus.FAILURE:
                            print("failed_node = ", el.attrib["name"])
                            return el
            d.extend(el)
        return None

    # アクションの事前条件ノードを生成したツリーから取得
    def get_pre_condition_nodes(self, action_node):
        parent = action_node.getparent() #   アクションノードの親を取得
        conditions = []
        if parent.tag == "ReactiveSequence":
            for child in parent:
                if child == action_node:    continue
                #   事前条件をサブツリーに置き換えられている場合
                if child.tag == "ReactiveFallback":
                    conditions.append(child[0]) #   TODO 条件ノードか確認すること
                
                elif child.tag in constraints.keys():   #   条件ノードなら追加
                    conditions.append(child)
        return conditions
           

    def set_current_conditions(self, tree, current_conditions, force_true = False):
        node_name = self.get_node_name(tree)
        if node_name == 'IsHandFree':
            if current_conditions['hand'] is None or current_conditions['hand'] == -1 or force_true:
                current_conditions.update({'hand': -1})
                return True
        elif node_name == 'IsGraspedObj':
            # input('------current_conditions: ' + str(current_conditions['hand']) + ' tree.fluent.parameters_dict: ' + str(tree.fluent.parameters_dict['object']) )

            if current_conditions['hand'] is None or current_conditions['hand'] == tree.attrib['obj_id'] or force_true:
                current_conditions.update({'hand': tree.attrib['obj_id']})
                return True
        elif node_name == 'IsRobotCloseTo':
            if current_conditions['robot_location_id'] is None or current_conditions['robot_location_id'] == tree.attrib['location_id'] or force_true:
                current_conditions.update({'robot_location_id': tree.attrib['location_id']})
                return True
        elif node_name == 'IsObjectAt':
            if tree.attrib['obj_pose'] not in current_conditions:    current_conditions[tree.attrib['obj_pose']]=None
            if current_conditions[tree.attrib['obj_pose']] is None or current_conditions[tree.attrib['obj_pose']] == tree.attrib['location_pose'] or force_true:
                current_conditions.update({tree.attrib['obj_pose']: tree.attrib['location_pose']})
                return True   
        else:
            return True
        return False

    

    #   treeを実行可能か確認
    def is_tree_feasible(self, tree, current_conditions):
        node_name = self.get_node_name(tree)
        print(current_conditions)
        print(node_name)
        #   Conditionノードの場合
        if node_name in constraints.keys():
            return self.set_current_conditions(tree, current_conditions)
    
        #   Actionノードの場合
        elif node_name in actions.keys():
            if node_name == 'Place':
                current_conditions.update({'hand': None})
            if node_name == 'Pick':
                current_conditions.update({'hand': tree.attrib['obj_id']})
            return True

        #   ReactiveFallbackノードの場合
        elif node_name in ['Fallback', 'ReactiveFallback']:

            #   条件を満たすサブツリーの実行可能性を確認した後、満たしたい条件だけ状態を保存する
            #   （サブツリー内のアクションの事前条件を保持しないため）
            if  "ID" in tree.attrib and tree.attrib["ID"] == "resolve_fallback":
                temp_conditions = current_conditions.copy()
                #   先に条件を満たすサブツリーから確認
                is_child_feasible = self.is_tree_feasible(tree[1], temp_conditions)
                if not is_child_feasible:
                    return False
                #   次に条件を保存
                return self.set_current_conditions(tree[0], current_conditions, True)
            else:
                return self.is_tree_feasible(tree[0], current_conditions)

        #   Sequenceノードの場合、最後の子ノードの効果だけ保存する?
        elif node_name in ['Sequence', 'ReactiveSequence']:
            for child in tree:
                is_child_feasible = self.is_tree_feasible(child, current_conditions)
                if not is_child_feasible:
                    return False
            return True
        #   その他のControlノードの場合は全ての子に対して確認
        else: 
            for child in tree:
                is_child_feasible = self.is_tree_feasible(child, current_conditions)
                if not is_child_feasible:
                    return False
            return True

    #   事前条件と事後条件の干渉が発生する場合は優先度を上げる



    # def breadth_first_order(xml):
    #     d = deque([xml])
    #     while d:
    #         el = d.popleft() 
    #         d.extend(el)
    #         print(el.attrib["name"])


    # def set_id(self, tree_xml):
    #     for cnt, node in enumerate(tree_xml.iter()):
    #         node.attrib["name"] = node.tag + str(cnt)

    #   ノードに識別用のidをセット
    def set_id(self, tree_xml):
        for cnt, node in enumerate(tree_xml.iter()):
            node.attrib["name"] = tree_xml.attrib["ID"] + "_" + node.tag + "_" + str(cnt)
            if node.tag == "SubTreePlus":
                node.attrib["subtree_name"] = node.attrib["name"]
                

    def get_xml(self):
        dirname = os.path.dirname(__file__)
        parent_dir = '/'.join(dirname.split('/')[:-1]) 
        xml_dir = parent_dir + '/bt_xml/subtree.xml'

        parser = etree.XMLParser(remove_blank_text=True, recover=True, remove_comments=True)
        with open(xml_dir, mode='rt', encoding='utf-8') as f:
            tree = etree.parse(f, parser=parser)
        # print(etree.tostring(tree).decode())
        root = tree.getroot()
        return root



class BTGeneratorService(BTGenerator):

    def __init__(self):
        super().__init__()
        # self.generator = BTGenerator()
        self.ignore_update = False  #　ツリー生成中は新規に受け付けない

        #   目標を受信するアクションサーバー
        self._action_server = ActionServer(
            self,
            SendGoal,
            'send_goal',
            self.execute_callback)


    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        print(goal_handle.request.goal_states)

        #   BB書き込み
        self.bb_req.bb_message = goal_handle.request.bb_message
        self.set_bb_dict(goal_handle.request.bb_message)
        print(self.bb_dict)

        #   初期BT生成
        bt = self.createInitialTree(goal_handle.request.goal_states)
        # self.send_request(bt)
        while self.execute_tree(bt)==False and rclpy.ok():
            bt = self.updateTree()
            print("Failure")


        goal_handle.succeed()
        result = SendGoal.Result()
        return result


    


def main():
    rclpy.init()
    bt_generator = BTGeneratorService()

    executor = MultiThreadedExecutor()  #   これじゃないと止まってしまう

    rclpy.spin(bt_generator, executor)

    bt_generator.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()


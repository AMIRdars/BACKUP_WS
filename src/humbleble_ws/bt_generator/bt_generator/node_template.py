# from winreg import DisableReflectionKey
from .base_template import *
from lxml.builder import E
from lxml import etree


# priority_fallback_xml = '<PriorityFallback priority="{priority}"/>'


path_check_xml = 'IsPathValid'
print_xml = '<PrintValue message="some_message"/>'
# some_xml_data = '<BehaviorTree ID="MoveRobot"><Fallback><SequenceStar><MoveBase       goal="{target}"/><SetBlackboard output_key="output" value="mission accomplished" /></SequenceStar></Fallback></BehaviorTree>'
# element = etree.fromstring(some_xml_data)

move_base_xml = '<MoveBase server_name="mecanum/move_base" x="0" y ="0" frame="mecanum/map"/>'

# pick_xml = '<PickProjected server_name="bt_pick" id="else" pose_x="0.0" pose_y="-1.0" pose_z="0.1" /> '
# place_xml = '<PlaceProjected server_name="bt_pick" id="else" pose_x="0.0" pose_y="-1.0" pose_z="0.1" />'
# approach_xml = '<ApproachProjected server_name="bt_approach" id="else" pose_x="0.91" pose_y="-1.0" pose_z="0.1" />'
# detect_xml = '<DetectIPP server_name="youbot_search" id="else" pose_x="0.91" pose_y="-1.0" pose_z="0.1" />'
# request_assist_xml = '<AssistIPP server_name="youbot_search" id="else" pose_x="0.91" pose_y="-1.0" pose_z="0.1" />'

#   name=" "でノードの名前を指定できる

pick_xml = '<SubTreePlus ID="Pick" obj_pose="{goal}" subtree_name="Pick" ros2_node="{node}" server_timeout="{server_timeout}" bt_loop_duration="{bt_loop_duration}" pickResult="{grasped_object}" block_grasp_check="{block_grasp_check}"/>'
# place_xml = '<Place action_name="bt_place" server_timeout="{server_timeout}" grasped_object="{grasped_object}" obj_id="none" location_id="none" location_pose="none" />'
place_xml = '<SubTreePlus ID="Place" subtree_name="Place" ros2_node="{node}" server_timeout="{server_timeout}" bt_loop_duration="{bt_loop_duration}" grasped_object="{grasped_object}" obj_id="none" location_id="none" location_pose="none" block_grasp_check="{block_grasp_check}"/>'
# approach_xml = '<Approach server_name="bt_approach" obj_id="{dummy_obj}" pose="0.0; 0.0; 0.0" type="none" output="{approachedResult}" result="{approachedCode}"/>'
follow_path_xml = '<FollowPath path="{path}" controller_id="FollowPath"/>'
search_obj_xml = '<SearchObj action_name="bt_search" server_timeout="{server_timeout}" label="{dummy_obj}" pose="{dummy_pose}"/>'
search_ipp_xml = '<ExploreIPP service_name="/youbot/self_explore" trajectory="{traj}"  />'
# approach_xml = '<NavigateToPose goal="{goal}"/>'

approach_xml = '<SubTreePlus ID="Approach" node="{node}" server_timeout="{server_timeout}" bt_loop_duration="{bt_loop_duration}" goal="{goal}" position_only="true"/> '
# approach_xml = '<SubTreePlus ID="Approach" bt_name="{bt_name}"  node="{node}" server_timeout="{server_timeout}" bt_loop_duration="{bt_loop_duration}" goal="{goal}"/>'


# camera_search_xml = '<CameraSearch action_name="bt_camera_search" server_timeout="{server_timeout}" obj_id="bottle" obj_pose="{pose}" />'       


detect_xml = '<DetectIPP server_name="youbot_search" obj_id="else" pose_x="0.91" pose_y="-1.0" pose_z="0.1" />'
request_assist_xml = '<VisualAssist server_name="visual_assist" type="path" path="{traj}" at="{location}" />'


IsPathValid_xml ='<IsPathValid service_name="gridmap/trajEntropy" pose="0.0;0.0;0.0" segment="{traj}" />'
# is_path_coll_free_xml ='<IsPathCollFree service_name="coll_check_srv" obj_id="else" pose="0.0;0.0;0.0"/>'
is_path_coll_free_xml ='<ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>'

IsGraspedObj_xml ='<IsGraspedObj grasp_state_topic="gripper_state" obj_id="{dummy_obj}" grasped_object="{grasped_object}" block_grasp_check="{block_grasp_check}"/>'
is_hand_xml ='<IsHandFree grasp_state_topic="gripper_state" block_grasp_check="{block_grasp_check}" grasped_object="{grasped_object}"/>'
is_robot_close_xml ='<IsRobotCloseTo location_id="none" location_pose="{location_pose}" outer_radius="0.8" inner_radius="0.0" position_only="true"/>'

is_obj_at_xml ='<IsObjectAt obj_pose="{source_pose}" location_pose="{location_pose}" outer_radius="0.5"/>'
# IsObjInFOV = '<IsObjInFOV obj_id="{obj_id}" candidate_pose="{obj_pose}" obj_pose="{obj_pose}"/>'
# spin_xml = '<Spin spin_dist="0.523"/>' 
spin_xml = '<Spin spin_dist="0.785"/>' 

#kobori#
#drone_xml
# drone_path_xml = '<DronePath/>'
# is_high_xml = '<High/>'
# is_low_xml = '<Low/>'
# is_low_task_xml = '<LowTask/>'
# land_position_is_free_xml = '<LandPositionIsFree/>'
# task_free_xml = '<TaskFree/>'

# Drone
# anafi_takeoff_xml='<AnafiTakeoff server_name="anafi/takeoff" />'
#anafi
# takeoff_xml = '<SubTreePlus ID="Takeoff"subtree_name="Takeoff" ros2_node="{node}" server_timeout="{server_timeout}"/>'
# land_xml = '<SubTreePlus ID="Land"subtree_name="Land" ros2_node="{node}" server_timeout="{server_timeout}"/>'

# bebop
# takeoff_bebop_xml = '<SubTreePlus ID="TakeoffBebop" subtree_name="TakeoffBebop" ros2_node="{node}" server_timeout="{server_timeout}"/>'
# land_bebop_xml = '<SubTreePlus ID="LandBebop" subtree_name="LandBebop" ros2_node="{node}" server_timeout="{server_timeout}"/>'
# pid_bebop_xml = '<SubTreePlus ID="PIDBebop" subtree_name="PIDBebop" ros2_node="{node}" server_timeout="{server_timeout}"/>'
# #path_move_bebop_xml = '<SubTreePlus ID="PathMoveBebop" subtree_name="PathMoveBebop" ros2_node="{node}" server_timeout="{server_timeout}"/>'
# round_bebop_xml = '<SubTreePlus ID="RoundBebop" subtree_name="RoundBebop" ros2_node="{node}" server_timeout="{server_timeout}"/>'

# #kobori end#

#   pddl的なやつ
constraints = {
    # 'is_obj_detected':Constraint('is_obj_detected', is_obj_at_xml,{'object': 0, 'detected': 0}), #for detect or so called get observation action 
    'IsObjectAt':Constraint('IsObjectAt', is_obj_at_xml,{'object': 0, 'at': 0}), 
    'IsRobotCloseTo':Constraint('IsRobotCloseTo',is_robot_close_xml, {'robot': 0, 'to': 0}),
    'IsGraspedObj':Constraint('IsGraspedObj',IsGraspedObj_xml, {'object': 0, 'hand':0}),
    'IsHandFree':Constraint('IsHandFree',is_hand_xml, {'empty': 0, 'hand':0}),
    'IsPathValid':Constraint('IsPathValid',IsPathValid_xml, {'path': 0, 'object': 0}),
    'is_path_coll_free':Constraint('is_path_coll_free',is_path_coll_free_xml, {'path': 0, 'object': 0}),


    # #kobori #
    # 'DronePath':Constraint('DronePath',drone_path_xml,{'plan':0}),
    # 'High':Constraint('High',is_high_xml,{'hheight':0}),
    # 'Low':Constraint('Low',is_low_xml,{'lheight':0}),
    # 'LowTask':Constraint('LowTask',is_low_task_xml,{'ltheight':0}),
    # 'LandPositionIsFree':Constraint('LandPositionIsFree',land_position_is_free_xml,{'position':0}),
    # 'TaskFree':Constraint('TaskFree',task_free_xml,{'task':0}),
    # #kobori end#
}


actions = {
    # 'FollowPath':ActionTemp(follow_path_xml,[constraints['is_path_coll_free']],['robot','to'],'FollowPath', 'pose2d'),
    'Approach': ActionTemp(approach_xml,[],['robot','to'],'Approach', 'pose2d'),
    # 'Approach':ActionTemp(approach_xml,[constraints['is_path_coll_free']],['robot','to'],'Approach', 'pose2d'),
    'DetectIPP': ActionTemp(detect_xml,[],['object','found'],'DetectIPP','o'),
    # 'VisualAssist':ActionTemp(request_assist_xml,[],['path'],'VisualAssist','e' ),
    'Pick': ActionTemp(pick_xml,[constraints['IsHandFree'],constraints['IsRobotCloseTo']],['object','hand'],'Pick', 'obj'),
    'Place': ActionTemp(place_xml,[constraints['IsGraspedObj'],constraints['IsRobotCloseTo']],['object','at'],'Place', 'at'),
    # 'ExploreIPP':ActionTemp(search_ipp_xml,[],['path'],'ExploreIPP','e'),


    # 'CameraSearch': ActionTemp(camera_search_xml,[],[],'CameraSearch',),
    
    #   リカバリ用
    'Spin': ActionTemp(spin_xml,[],['rot'],'Spin', 'rot'),

    # #kobori#
    # # anafi
    # # 'Takeoff':ActionTemp(takeoff_xml,[constraints['Low']],['hheight'],'Takeoff','ta'),
    # # 'Land':ActionTemp(land_xml,[constraints['LandPositionIsFree'],constraints['High']],['lheight'],'Land','la'),
    # #bebop
    # 'TakeoffBebop':ActionTemp(takeoff_bebop_xml,[constraints['Low']],['hheight'],'TakeoffBebop','tab'),
    # 'LandBebop':ActionTemp(land_bebop_xml,[constraints['LandPositionIsFree']],['ltheight'],'LandBebop','lab'),
    # 'PIDBebop':ActionTemp(pid_bebop_xml,[constraints['TaskFree']],['position'],'PIDBebop','pid'),
    # # 'PathMoveBebop':ActionTemp(path_move_bebop_xml,[constraints['DronePath']],['task'],'PathMoveBebop','pathmove'),
    # # 'RoundBebop':ActionTemp(round_bebop_xml,[constraints['High']],['plan'],'RoundBebop','round'),
    # 'RoundBebop':ActionTemp(round_bebop_xml,[constraints['High']],['task'],'RoundBebop','round'),
    
    # #kobori end#
}


# recoveries = {
#     "movement_of_viewpoint"
# }



# mapping_rule = {

# }

# approach_to = ActionTemp(approach_xml,[is_path_coll_free],['robot','to'],'ApproachProjected', 'pose2d')
# detect_obj = ActionTemp(detect_xml,[],['object','found'],'DetectIPP','o')
# assist_detect_obj = ActionTemp(request_assist_xml,[],['path'],'VisualAssist','e' )
# pick_obj = ActionTemp(pick_xml,[IsHandFree,IsRobotCloseTo],['object','hand'],'PickProjected', 'o')
# place_obj = ActionTemp(place_xml,[IsGraspedObj,IsRobotCloseTo],['object','at'],'PlaceProjected', 'at')

# # drop_obj = ActionTemp(place_xml,[],['hand','empty'],'drop_obj', {'at': 0})
# explore_path = ActionTemp(search_ipp_xml,[],['path'],'ExploreIPP','e')


# actions.append(approach_to)
# # actions.append(detect_obj)
# # actions.append(assist_detect_obj)
# actions.append(pick_obj)
# actions.append(place_obj)

# # actions.append(drop_obj)
# actions.append(explore_path)






# is_obj_detected = Constraint('IsObjectAt', is_obj_at_xml,{'object': 0, 'detected': 0}) #for detect or so called get observation action 
# IsObjectAt = Constraint('IsObjectAt', is_obj_at_xml,{'object': 0, 'at': 0}) 
# IsRobotCloseTo = Constraint('IsRobotCloseTo',is_robot_close_xml, {'robot': 0, 'to': 0})
# IsGraspedObj = Constraint('IsGraspedObj',IsGraspedObj_xml, {'object': 0, 'hand':0})
# IsHandFree = Constraint('IsHandFree',is_hand_xml, {'empty': 0, 'hand':0})
# IsPathValid = Constraint('IsPathValid',IsPathValid_xml, {'path': 0, 'object': 0})
# is_path_coll_free = Constraint('is_path_coll_free',is_path_coll_free_xml, {'path': 0, 'object': 0})

# constraints= []
# constraints.append(is_obj_detected)
# constraints.append(IsObjectAt)
# constraints.append(IsRobotCloseTo)
# constraints.append(IsHandFree)
# constraints.append(IsGraspedObj)

# constraints.append(IsPathValid)


if __name__ == '__main__':
    print("test")
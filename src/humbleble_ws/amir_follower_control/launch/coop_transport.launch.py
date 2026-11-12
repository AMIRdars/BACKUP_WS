"""Opt-in fixed-grasp scene with F/T system and event-based startup.

Existing launch files and models remain intact. Startup ownership is released
explicitly by the experiment supervisor only after both grasps are attached.
"""
import os
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET
import json
import shlex
import numpy as np
import xacro

from ament_index_python.packages import get_package_share_directory as share, get_package_prefix
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from amir_follower_control.core import ArmURDF
from amir_follower_control.posture import HOME, paired_postures, with_initial_positions


def setup(context):
    headless = LaunchConfiguration('headless').perform(context) == 'true'
    output = Path(LaunchConfiguration('evidence_directory').perform(context)).expanduser()
    output.mkdir(parents=True, exist_ok=True)
    pitch = float(LaunchConfiguration('posture_pitch_rad').perform(context))
    approach_distance = float(LaunchConfiguration('approach_distance').perform(context))
    descriptions = {}
    homes = [HOME.tolist(),HOME.tolist()]
    spawn_distance = 1.174987
    grasp_mode=LaunchConfiguration('grasp_mode').perform(context)
    if grasp_mode not in ('fixed','friction'):
        raise ValueError('grasp_mode must be fixed or friction')
    friction=grasp_mode=='friction'
    pinch=LaunchConfiguration('pinch_orientation').perform(context)
    if pinch not in ('vertical','lateral'):
        raise ValueError('pinch_orientation must be vertical or lateral')
    if pinch=='lateral' and (not friction or not pitch):
        raise ValueError('lateral pinch requires the opt-in tilted friction scene')
    wrist_angle=0. if pinch=='lateral' else float(HOME[4])
    payload_roll=0. if pinch=='lateral' else float(np.pi/2)
    reach=float(LaunchConfiguration('arm_reach_m').perform(context))
    if not np.isfinite(reach) or (reach!=0 and not .35<=reach<=.475):
        raise ValueError('arm_reach_m must be zero (original) or 0.35..0.475 m')
    if reach and (pinch!='lateral' or not pitch):
        raise ValueError('shorter reach is limited to the staged lateral posture')
    payload_model_name='cooperative_payload_friction' if friction else 'cooperative_payload'
    payload_file = Path(share('cooperative_transport_description'))/f'models/{payload_model_name}/model.sdf'
    controlled = LaunchConfiguration('controlled_grasp').perform(context) == 'true'
    if friction and (controlled or LaunchConfiguration('gentle_grasp').perform(context)!='true'):
        raise ValueError('friction requires gentle_grasp=true and controlled_grasp=false')
    requested_mass = float(LaunchConfiguration('payload_mass_kg').perform(context))
    if not np.isfinite(requested_mass) or requested_mass < 0 or requested_mass > 8:
        raise ValueError('staged payload mass must be between 0 and the audited 8 kg')
    if controlled or requested_mass:
        payload_tree = ET.parse(payload_file)
        payload_model = payload_tree.getroot().find('model')
        if controlled:
            for plugin in list(payload_model.findall('plugin')):
                if plugin.get('name') == 'ignition::gazebo::systems::DetachableJoint':
                    payload_model.remove(plugin)
        if requested_mass:
            inertial=payload_model.find("link[@name='payload_link']/inertial")
            original_mass=float(inertial.find('mass').text)
            inertial.find('mass').text=str(requested_mass)
            for component in inertial.find('inertia'):
                component.text=str(float(component.text)*requested_mass/original_mass)
        payload_file = output/'controlled_payload.sdf'
        payload_tree.write(payload_file,encoding='unicode',xml_declaration=True)
    if pitch:
        xacro_file = Path(share('amir_description'))/'urdf/amir_mecanum3_sim.xacro'
        xml = xacro.process_file(str(xacro_file),mappings={'namespace':'amir2',
            'enable_d435':'false','enable_lidar':'false','finger_collision_boxes':'9'}).toxml()
        size = [float(v) for v in ET.parse(payload_file).find('.//collision/geometry/box/size').text.split()]
        half_span = size[0]/2-float(LaunchConfiguration('grasp_inset_m').perform(context))
        homes = [q.tolist() for q in paired_postures(xml,pitch,half_span,wrist_angle=wrist_angle,
                                                   reach=reach if reach else None)]
        arm = ArmURDF(xml)
        spawn_distance = half_span*np.cos(pitch)+arm.evaluate(homes[0])[0][0,3]+approach_distance
        metadata = []
        for ns,q in zip(('amir1','amir2'),homes):
            source = xacro.process_file(str(xacro_file),mappings={'namespace':ns,
                'enable_d435':'false','enable_lidar':'false','finger_collision_boxes':'9'}).toxml()
            descriptions[ns] = str(output/f'{ns}.urdf')
            Path(descriptions[ns]).write_text(with_initial_positions(source,q))
            t,_,j = arm.evaluate(q)
            metadata.append({'robot':ns,'q':q,'grasp_B':t[:3,3].tolist(),
                             'pinch_orientation':pinch,'payload_roll_rad':payload_roll,
                             'arm_reach_m':float(t[0,3]),
                             'sigma':np.linalg.svd(j,compute_uv=False).tolist()})
        (output/'posture.json').write_text(json.dumps(metadata,indent=2))
    path = Path(share('cooperative_transport_gazebo'))/'worlds/cooperative_transport.sdf'
    tree = ET.parse(path)
    world = tree.getroot().find('world')
    if LaunchConfiguration('contact_compliance').perform(context) == 'true':
        cfm=float(LaunchConfiguration('contact_cfm').perform(context))
        if not np.isfinite(cfm) or not 1e-4<=cfm<=.1:
            raise ValueError('staged contact_cfm must be between 0.0001 and 0.1')
        compliance = ET.SubElement(world,'plugin',filename=str(Path(
            get_package_prefix('cooperative_transport_gazebo'))/'lib/libcooperative_dart_contact_parameters-system.so'),
            name='cooperative_transport::DartContactParameters')
        for key,value in {'constraint_force_mixing':str(cfm),'error_reduction_parameter':'0.2',
                          'max_error_velocity':'0.2'}.items():
            ET.SubElement(compliance,key).text=value
    if pitch:
        world.remove(world.find("model[@name='grasp_support']"))
    ET.SubElement(world, 'plugin', filename='ignition-gazebo-forcetorque-system',
                  name='ignition::gazebo::systems::ForceTorque')
    heartbeat = ET.SubElement(world,'plugin',filename=str(Path(
        get_package_prefix('amir_follower_gazebo'))/'lib/libamir_grasp_heartbeat-system.so'),
        name='amir_follower::GraspHeartbeat')
    for key,value in {'payload':'cooperative_payload','robot1':'amir1','robot2':'amir2',
                      'parent_link1':'grasp_r1_link','parent_link2':'grasp_r2_link',
                      'child_link':'gripper_base_1'}.items():
        ET.SubElement(heartbeat,key).text=value
    ET.SubElement(heartbeat,'manage_joints').text=str(controlled).lower()
    if friction:
        world.remove(heartbeat)
    if LaunchConfiguration('external_wrench_test').perform(context) == 'true':
        plugin = ET.SubElement(world, 'plugin', filename=str(Path(
            get_package_prefix('amir_follower_gazebo'))/'lib/libamir_calibration_wrench-system.so'),
            name='amir_follower::CalibrationWrench')
        for key,value in {'model':'amir2','link':'gripper_base_1','base':'base_footprint',
                          'offset':'0 0.0795 0','topic':'/experiment/amir2/wrench_base'}.items():
            ET.SubElement(plugin,key).text = value
    # The on-disk generated world is retained as part of the evidence.
    world_path = output/'world.sdf'
    tree.write(world_path, encoding='unicode', xml_declaration=True)
    gui_config=LaunchConfiguration('gui_config').perform(context)
    if gui_config and not Path(gui_config).is_file():
        raise ValueError('gui_config must name an existing GUI configuration')
    gazebo_args=('-s ' if headless else '')+f'-r -v 2 {shlex.quote(str(world_path))}'
    if gui_config and not headless:
        gazebo_args+=' --gui-config '+shlex.quote(gui_config)
    sim = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(share('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gazebo_args,
                          'gz_version':'6', 'on_exit_shutdown':'true'}.items())
    robot_file = os.path.join(share('amir_gazebo'), 'launch', 'robot_bringup.launch.py')
    def robot(ns,x,yaw):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(robot_file),
            launch_arguments={'namespace':ns,'x':x,'y':'0.0','z':'0.03','yaw':yaw,
                              'world_name':'cooperative_transport','launch_sim':'false',
                              'pose_bridge':'false','headless':str(headless).lower(),
                              'event_startup':'true', 'enable_d435':'false',
                              'robot_description_file':descriptions.get(ns,''),
                              'enable_lidar':'false','enable_wrist_ft':'true'}.items())
    bridge = Node(package='ros_gz_bridge',executable='parameter_bridge',
        name='follower_experiment_bridge', arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/cooperative_transport/remove@ros_gz_interfaces/srv/DeleteEntity',
            '/world/cooperative_transport/create@ros_gz_interfaces/srv/SpawnEntity',
            *[f'/{ns}/grasp/{action}@std_msgs/msg/Empty]gz.msgs.Empty'
              for ns in ('amir1','amir2') for action in ('attach','detach')],
            *([] if friction else [f'/{ns}/grasp/state@std_msgs/msg/String[gz.msgs.StringMsg'
              for ns in ('amir1','amir2')])])
    gate = Node(package='cooperative_transport_control',executable='simulation_startup_gate',
                name='follower_scene_controllers_ready',parameters=[{'use_sim_time':True,
                'mode':'controllers','timeout':180.}])
    first_gate = Node(package='cooperative_transport_control',executable='simulation_startup_gate',
        name='first_robot_controllers_ready',parameters=[{'use_sim_time':True,
            'mode':'controllers','robot_namespaces':['amir1'],'timeout':120.}])
    def after_first_robot(event,_):
        if event.returncode:
            from launch.actions import EmitEvent
            from launch.events import Shutdown
            return [EmitEvent(event=Shutdown(reason='first robot controller startup failed'))]
        return [robot('amir2',str(spawn_distance),'3.141592653589793')]
    home = Node(package='amir_follower_control',executable='home_once',
                parameters=[{'use_sim_time':True,'robot_namespaces':['amir1','amir2'],
                             'home_positions':homes[0]+homes[1],
                             'motion_duration':8.,'compensate_base':False}])
    payload = Node(package='ros_gz_sim',executable='create',name='spawn_follower_payload',
        arguments=['-world','cooperative_transport','-name','cooperative_payload','-file',
                   str(payload_file),
                   '-x','0','-y','0','-z','.660','-R','1.5707963267948966','-allow_renaming','false'])
    if pitch:
        payload = Node(package='amir_follower_control',executable='prepare_payload',
            parameters=[{'use_sim_time':True,'evidence_directory':str(output),
                         'payload_file':str(payload_file),'approach_distance':approach_distance,
                         'payload_roll_rad':payload_roll,
                         'expected_axis_pitch_rad':pitch}])
    gentle = LaunchConfiguration('gentle_grasp').perform(context) == 'true'
    grasp = Node(package='amir_follower_control',
        executable='gentle_friction_grasp' if friction else ('gentle_attach' if gentle else 'managed_attach'),
        parameters=[{'use_sim_time':True,'auto_attach':False,
                     'close_grippers_before_attach':True,
                     'contact_force_n':float(LaunchConfiguration('contact_force_n').perform(context)),
                     'holding_force_n':float(LaunchConfiguration('holding_force_n').perform(context)),
                     'holding_position_gain':float(LaunchConfiguration('holding_position_gain').perform(context)),
                     'holding_max_step_rad':float(LaunchConfiguration('holding_max_step_rad').perform(context)),
                     'holding_interval_s':float(LaunchConfiguration('holding_interval_s').perform(context)),
                     'gripper_close_position':float(LaunchConfiguration('gripper_close_position').perform(context)),
                     'gripper_maximum_effort':.8,'require_gripper_contacts_before_attach':True}])
    approach = Node(package='amir_follower_control',executable='approach_once',
        parameters=[{'use_sim_time':True,'robot_namespaces':['amir1','amir2'],
                     'open_grippers_before_approach':True,'gripper_open_position':-1.,
                     'gripper_maximum_effort':.8,
                     'approach_distance':float(LaunchConfiguration('approach_distance').perform(context)),
                     'maximum_speed':.035,'position_tolerance':.001}])
    support = Node(package='cooperative_transport_control',executable='grasp_support_remover',
        parameters=[{'use_sim_time':True,'world_name':'cooperative_transport',
                     'require_physical_contact':False,'require_friction_holding':False,
                     'removal_delay':1.}])
    grasp_gate = Node(package='amir_follower_control',executable='wait_grasp_ready',
                      parameters=[{'use_sim_time':True}])
    calibration_gate=Node(package='amir_follower_control',executable='wait_grasp_start',
                          parameters=[{'use_sim_time':True}])
    def begin_grasp():
        if friction:
            # The contact adapter reports detached before the payload exists.
            # Keep the bases stationary until predicted placement is complete.
            return [grasp,grasp_gate,payload]
        return [grasp,approach,grasp_gate,TimerAction(period=1.,actions=[payload])]
    def after_payload(event,_):
        if event.returncode:
            from launch.actions import EmitEvent
            from launch.events import Shutdown
            return [EmitEvent(event=Shutdown(reason='payload/support initialization failed'))]
        return [approach] if friction else []
    def after_calibration(event,_):
        if event.returncode:
            from launch.actions import EmitEvent
            from launch.events import Shutdown
            return [EmitEvent(event=Shutdown(reason='calibration handoff timed out'))]
        return begin_grasp()
    def after_verified(event,_):
        if event.returncode:
            from launch.actions import EmitEvent
            from launch.events import Shutdown
            return [EmitEvent(event=Shutdown(reason='contact closure verification failed'))]
        return [support]
    def after_gate(event, _):
        if event.returncode:
            from launch.actions import EmitEvent
            from launch.events import Shutdown
            return [EmitEvent(event=Shutdown(reason='controller readiness failed'))]
        return [home]
    def after_home(event, _):
        if event.returncode:
            from launch.actions import EmitEvent
            from launch.events import Shutdown
            return [EmitEvent(event=Shutdown(reason='arm home failed'))]
        if LaunchConfiguration('with_payload').perform(context) != 'true':
            return []
        if LaunchConfiguration('defer_grasp').perform(context)=='true':
            return [calibration_gate]
        return begin_grasp()
    follower = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        str(Path(share('amir_follower_control'))/'launch/follower.launch.py')),
        launch_arguments={'config':LaunchConfiguration('config'),
                          'robot_description_file':descriptions.get('amir2',''),
                          'log_file':str(output/'follower.jsonl')}.items())
    extra = []
    if LaunchConfiguration('evaluation_logging').perform(context) == 'true':
        extra.extend([
            Node(package='ros_gz_bridge',executable='parameter_bridge',name='evaluation_pose_bridge',
                 arguments=['/world/cooperative_transport/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V']),
            Node(package='cooperative_transport_gazebo',executable='transport_pose_filter',
                 parameters=[{'input_topic':'/world/cooperative_transport/pose/info',
                              'output_topic':'/experiment/model_poses'}])])
    if LaunchConfiguration('external_wrench_test').perform(context) == 'true':
        extra.append(Node(package='ros_gz_bridge',executable='parameter_bridge',
            name='calibration_wrench_bridge', arguments=[
                '/experiment/amir2/wrench_base@geometry_msgs/msg/Wrench]gz.msgs.Wrench']))
    return [*extra,sim,bridge,TimerAction(period=2.,actions=[robot('amir1',str(-spawn_distance),'0'),first_gate]),
            RegisterEventHandler(OnProcessExit(target_action=first_gate,on_exit=after_first_robot)),
            RegisterEventHandler(OnProcessExit(target_action=gate,on_exit=after_gate)),
            RegisterEventHandler(OnProcessExit(target_action=grasp_gate,on_exit=after_verified)),
            RegisterEventHandler(OnProcessExit(target_action=calibration_gate,on_exit=after_calibration)),
            RegisterEventHandler(OnProcessExit(target_action=payload,on_exit=after_payload)),
            RegisterEventHandler(OnProcessExit(target_action=home,on_exit=after_home)),gate,follower]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('headless',default_value='true'),
        DeclareLaunchArgument('gui_config',default_value=''),
        DeclareLaunchArgument('with_payload',default_value='true'),
        DeclareLaunchArgument('defer_grasp',default_value='false'),
        DeclareLaunchArgument('grasp_mode',default_value='fixed'),
        DeclareLaunchArgument('pinch_orientation',default_value='vertical'),
        DeclareLaunchArgument('arm_reach_m',default_value='0.0'),
        DeclareLaunchArgument('external_wrench_test',default_value='false'),
        DeclareLaunchArgument('gripper_close_position',default_value='0.20'),
        DeclareLaunchArgument('approach_distance',default_value='0.110'),
        DeclareLaunchArgument('gentle_grasp',default_value='false'),
        DeclareLaunchArgument('contact_force_n',default_value='3.0'),
        DeclareLaunchArgument('holding_force_n',default_value='5.0'),
        DeclareLaunchArgument('holding_position_gain',default_value='0.00015'),
        DeclareLaunchArgument('holding_max_step_rad',default_value='0.0003'),
        DeclareLaunchArgument('holding_interval_s',default_value='0.1'),
        DeclareLaunchArgument('posture_pitch_rad',default_value='0.0'),
        DeclareLaunchArgument('grasp_inset_m',default_value='0.01'),
        DeclareLaunchArgument('contact_compliance',default_value='false'),
        DeclareLaunchArgument('contact_cfm',default_value='0.05'),
        DeclareLaunchArgument('controlled_grasp',default_value='false'),
        DeclareLaunchArgument('payload_mass_kg',default_value='0.0'),
        DeclareLaunchArgument('evaluation_logging',default_value='false'),
        DeclareLaunchArgument('config',default_value=str(
            Path(share('amir_follower_control'))/'config/follower.yaml')),
        DeclareLaunchArgument('evidence_directory',default_value=str(
            Path(tempfile.gettempdir())/'amir_follower_experiment')),
        OpaqueFunction(function=setup)])

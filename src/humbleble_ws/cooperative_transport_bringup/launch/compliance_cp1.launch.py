"""CP-1: independent leader/follower, supported payload, encoder-only TF.

Derived files are retained with the run. No grasp, trajectory generator, or
ground-truth bridge runs here. Original bringup defaults remain unchanged.
"""
from pathlib import Path
from copy import deepcopy
import shlex
import tempfile
import math
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory as share, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
import yaml
from amir_compliance_control.wrench import LocalModel


def setup(context):
    output = Path(LaunchConfiguration('run_directory').perform(context)).expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    constraint = LaunchConfiguration('constraint_fixture').perform(context).lower() == 'true'
    grasp_period = float(LaunchConfiguration('grasp_state_publish_period_s',default='.1').perform(context))
    if not math.isfinite(grasp_period) or grasp_period<=0.:
        raise ValueError('positive finite grasp_state_publish_period_s required')
    world = ET.parse(Path(share('cooperative_transport_gazebo'))/'worlds/cooperative_transport.sdf')
    weld_cfm = float(LaunchConfiguration('weld_cfm').perform(context))
    if not math.isfinite(weld_cfm) or weld_cfm < 0 or weld_cfm > 1:
        raise ValueError('weld_cfm must be zero (unchanged) or in DART range')
    if weld_cfm:
        if not constraint or weld_cfm < 1e-9:
            raise ValueError('positive weld_cfm requires constraint fixture and DART range')
        plugin = ET.SubElement(world.getroot().find('world'), 'plugin',
            filename=str(Path(get_package_prefix('cooperative_transport_gazebo'))/'lib/libcooperative_dart_weld_parameters-system.so'),
            name='cooperative_transport::DartWeldParameters')
        ET.SubElement(plugin, 'constraint_force_mixing').text = str(weld_cfm)
    if LaunchConfiguration('force_torque_system').perform(context).lower() == 'true':
        ET.SubElement(world.getroot().find('world'), 'plugin',
            filename='ignition-gazebo-forcetorque-system',
            name='ignition::gazebo::systems::ForceTorque')
    if LaunchConfiguration('calibration_fixture').perform(context).lower() == 'true':
        if LaunchConfiguration('force_torque_system').perform(context).lower() != 'true':
            raise ValueError('calibration_fixture requires force_torque_system')
        plugin = ET.SubElement(world.getroot().find('world'), 'plugin',
            filename=str(Path(get_package_prefix('amir_follower_gazebo'))/
                         'lib/libamir_calibration_wrench-system.so'),
            name='amir_follower::CalibrationWrench')
        for key, value in {'model': 'follower', 'link': 'gripper_base_1',
                'base': 'base_footprint', 'offset': '0 0.0795 0',
                'topic': '/experiment/follower/calibration_wrench'}.items():
            ET.SubElement(plugin, key).text = value
    payload = ET.parse(Path(share('cooperative_transport_description'))/
                       'models/cooperative_payload_friction/model.sdf').getroot().find('model')
    if constraint:
        measured_mass = float(payload.findtext('link/inertial/mass'))
        payload = ET.parse(Path(share('cooperative_transport_description'))/
                          'models/cooperative_payload/model.sdf').getroot().find('model')
        for plugin in list(payload.findall('plugin')):
            payload.remove(plugin)
        inertial = payload.find("link[@name='payload_link']/inertial")
        ratio = measured_mass/float(inertial.findtext('mass'))
        inertial.find('mass').text = str(measured_mass)
        for value in inertial.find('inertia'):
            value.text = str(float(value.text)*ratio)
        manager = ET.SubElement(world.getroot().find('world'), 'plugin',
            filename=str(Path(get_package_prefix('amir_follower_gazebo'))/'lib/libamir_grasp_heartbeat-system.so'),
            name='amir_follower::GraspHeartbeat')
        for key, value in {'payload': 'cooperative_payload', 'robot1': 'leader', 'robot2': 'follower',
                'parent_link1': 'grasp_r1_link', 'parent_link2': 'grasp_r2_link',
                'child_link': 'gripper_base_1', 'manage_joints': 'true'}.items():
            ET.SubElement(manager, key).text = value
        # Keep the legacy generated world byte-for-byte at its default.
        # Faster status still reads actual ECM constraints in PreUpdate.
        if grasp_period!=.1:
            ET.SubElement(manager,'state_publish_period_s').text=str(grasp_period)
    # Source model is 0.5 kg, with no DetachableJoint or grasp assistance.
    ET.SubElement(payload, 'pose').text = '0 0 0.660 1.5707963267948966 0 0'
    world.getroot().find('world').append(payload)
    world_path = output/'world.sdf'
    simulation = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        str(Path(share('ros_gz_sim'))/'launch/gz_sim.launch.py')),
        launch_arguments={'gz_version': '6', 'on_exit_shutdown': 'true',
                          'gz_args': ('-s ' if headless else '')+'-r -v 2 '+shlex.quote(str(world_path))}.items())
    actions = [simulation, Node(package='ros_gz_bridge', executable='parameter_bridge',
        name='compliance_clock_bridge', parameters=[{'use_sim_time': True}],
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'])]
    for i, (robot, xpos, yaw) in enumerate([
            ('leader', '-1.174987', '0'), ('follower', '1.174987', '3.141592653589793')]):
        config = yaml.safe_load((Path(share('mecanumrover_description'))/
                                 'params/controllers.yaml').read_text())
        params = config['/**/mecanum_drive_controller']['ros__parameters']
        params.update(base_frame_id=robot+'/base_footprint', odom_frame_id=robot+'/odom',
                      enable_odom_tf=True)
        # Supply controller parameters through that manager's spawner private
        # NodeOptions, rather than the shared Gazebo process's global arguments.
        base_controllers = output/(robot+'_base_controller.yaml')
        base_controllers.write_text(yaml.safe_dump({'/**/mecanum_drive_controller': config.pop('/**/mecanum_drive_controller')}))
        # Both embedded managers share a Gazebo process. Wildcard parameter
        # overrides from either file can otherwise reach the other controller.
        config = {'/'+robot+'/'+key.rsplit('/', 1)[-1]: value for key, value in config.items()}
        controllers = output/(robot+'_controllers.yaml')
        controllers.write_text(yaml.safe_dump(config))
        xml = xacro.process_file(str(Path(share('amir_description'))/
            'urdf/amir_mecanum3_sim.xacro'), mappings={'namespace': robot,
            'enable_d435': 'false', 'enable_lidar': 'false'}).toxml()
        root = ET.fromstring(xml)
        if LaunchConfiguration('joint_load_sensors').perform(context).lower() == 'true':
            if LaunchConfiguration('force_torque_system').perform(context).lower() != 'true':
                raise ValueError('joint_load_sensors requires force_torque_system')
            template = root.find("gazebo[@reference='Joint_5']/sensor")
            for name in LocalModel(xml).names[:-1]:
                sensor = deepcopy(template)
                sensor.set('name', 'joint_load_'+name)
                sensor.find('topic').text = '/'+robot+'/joint_load/'+name
                ET.SubElement(root, 'gazebo', reference=name).append(sensor)
        if constraint:
            model = LocalModel(xml)
            q = [float(root.find(f"ros2_control/joint[@name='{name}']/state_interface[@name='position']/param").text)
                 for name in model.names]
            grasp, sensor = model.frames(q)
            anchor = payload.find(f"link[@name='grasp_r{i+1}_link']/pose")
            anchor_x = float(anchor.text.split()[0])
            xpos = str(anchor_x-sensor[0, 3] if i == 0 else anchor_x+sensor[0, 3])
            payload.find('pose').text = f'0 0 {sensor[2, 3]} 1.5707963267948966 0 0'
            # Derived support matches the existing payload's rotated thickness.
            thickness = float(payload.findtext("link[@name='payload_link']/collision/geometry/box/size").split()[1])
            top = float(sensor[2, 3])-thickness/2
            support = world.getroot().find("world/model[@name='grasp_support']")
            support.find('pose').text = f'0 0 {top/2} 0 0 0'
            for geometry in support.findall('.//geometry/box/size'):
                size = geometry.text.split()
                geometry.text = ' '.join(size[:2]+[str(top)])
        for plugin in root.findall('gazebo/plugin'):
            if plugin.get('name') == 'gz_ros2_control::GazeboSimROS2ControlPlugin':
                plugin.find('parameters').text = str(controllers)
                # Humble's installed controller actually publishes ~/tf_odometry.
                ET.SubElement(plugin.find('ros'), 'remapping').text = (
                    '/'+robot+'/mecanum_drive_controller/tf_odometry:=/'+robot+'/tf')
            elif plugin.get('name') == 'ignition::gazebo::systems::OdometryPublisher':
                plugin.find('odom_topic').text = '/experiment/'+robot+'/ground_truth/odom'
                plugin.find('tf_topic').text = '/experiment/'+robot+'/ground_truth/tf'
                plugin.find('odom_frame').text = 'world'
                plugin.find('robot_base_frame').text = robot+'/base_footprint'
        urdf = output/(robot+'.urdf')
        urdf.write_text(ET.tostring(root, encoding='unicode'))
        actions.append(TimerAction(period=2.+4.*i, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(Path(share('amir_gazebo'))/'launch/robot_bringup.launch.py')),
            launch_arguments={'namespace': robot, 'x': xpos, 'yaw': yaw, 'z': '.03',
                'launch_sim': 'false', 'headless': str(headless).lower(),
                'world_name': 'cooperative_transport', 'event_startup': 'false',
                'enable_d435': 'false', 'enable_lidar': 'false',
                'bridge_ground_truth': 'false', 'tf_frame_prefix': robot+'/',
                'base_controllers_file': str(base_controllers),
                'robot_description_file': str(urdf)}.items())]))
    world.write(world_path, encoding='unicode', xml_declaration=True)
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('force_torque_system', default_value='false'),
        DeclareLaunchArgument('calibration_fixture', default_value='false'),
        DeclareLaunchArgument('constraint_fixture', default_value='false'),
        DeclareLaunchArgument('grasp_state_publish_period_s', default_value='.1'),
        DeclareLaunchArgument('joint_load_sensors', default_value='false'),
        DeclareLaunchArgument('weld_cfm', default_value='0'),
        DeclareLaunchArgument('run_directory', default_value=str(
            Path(tempfile.gettempdir())/'amir_compliance_cp1')),
        OpaqueFunction(function=setup)])

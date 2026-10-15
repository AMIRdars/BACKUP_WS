"""Spawn a static AMIR model with every joint at zero for photographs."""

import os
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction,
    RegisterEventHandler, SetEnvironmentVariable,
)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def make_zero_posture_world(directory, mappings, pose):
    """Keep URDF zero transforms, without control or sensor plugins."""
    description_dir = get_package_share_directory('amir_description')
    urdf = ET.fromstring(xacro.process_file(
        os.path.join(description_dir, 'urdf', 'amir_mecanum3_sim.xacro'),
        mappings=mappings,
    ).toxml())
    for control in list(urdf.findall('ros2_control')):
        urdf.remove(control)
    for parent in urdf.iter():
        for child in list(parent):
            if child.tag in ('plugin', 'sensor'):
                parent.remove(child)
    # Absolute mesh paths also work without ROS / Gazebo resource-path hooks.
    for mesh in urdf.iter('mesh'):
        filename = mesh.get('filename', '')
        if filename.startswith('package://'):
            package, relative = filename[len('package://'):].split('/', 1)
            mesh.set('filename', Path(
                get_package_share_directory(package), relative).as_uri())
    urdf_path = Path(directory, 'zero_posture.urdf')
    ET.ElementTree(urdf).write(urdf_path, encoding='unicode')
    result = subprocess.run(
        ['ign', 'sdf', '-p', str(urdf_path)],
        check=True, capture_output=True, text=True,
    )
    model = ET.fromstring(result.stdout).find('model')
    if model is None:
        raise RuntimeError('URDF conversion did not produce a model')
    model.set('name', 'amir_zero_posture')
    static = model.find('static')
    if static is None:
        static = ET.SubElement(model, 'static')
    static.text = 'true'
    model_pose = model.find('pose')
    if model_pose is None:
        model_pose = ET.SubElement(model, 'pose')
    model_pose.text = pose
    # Static links retain the converted URDF transforms at q=0; no controller
    # can apply Q_HOME and gravity cannot move the joints during photography.
    gazebo_dir = get_package_share_directory('amir_gazebo')
    world = ET.parse(os.path.join(gazebo_dir, 'worlds', 'zero_posture.sdf'))
    world.getroot().find('world').append(model)
    world_path = Path(directory, 'zero_posture.sdf')
    world.write(world_path, encoding='unicode')
    return str(world_path)


def launch_setup(context):
    mappings = {
        name: LaunchConfiguration(name).perform(context)
        for name in ('enable_d435', 'enable_lidar')
    }
    mappings['enable_wrist_ft'] = 'false'
    coordinates = [
        str(float(LaunchConfiguration(name).perform(context)))
        for name in ('x', 'y', 'z', 'yaw')
    ]
    pose = ' '.join(coordinates[:3] + ['0', '0', coordinates[3]])
    temporary = tempfile.TemporaryDirectory(prefix='amir_zero_posture_')
    try:
        world_path = make_zero_posture_world(temporary.name, mappings, pose)
    except Exception:
        temporary.cleanup()
        raise
    headless = LaunchConfiguration('headless').perform(context) == 'true'
    gz_args = ('-s ' if headless else '') + '-r -v 1 ' + world_path
    return [
        RegisterEventHandler(OnShutdown(on_shutdown=[
            OpaqueFunction(function=lambda context: temporary.cleanup()),
        ])),
        SetEnvironmentVariable('__EGL_VENDOR_LIBRARY_FILENAMES',
                               '/usr/share/glvnd/egl_vendor.d/10_nvidia.json'),
        SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1'),
        SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia'),
        LogInfo(msg='Photography model: all arm, gripper and wheel joints '
                    'are fixed at 0 rad.'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py')),
            launch_arguments={
                'gz_args': gz_args, 'gz_version': '6',
                'on_exit_shutdown': 'true',
            }.items(),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('enable_d435', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('enable_lidar', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        OpaqueFunction(function=launch_setup),
    ])

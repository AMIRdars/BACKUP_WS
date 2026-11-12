"""Start certified CP-9 files without regenerating URDF paths or world settings."""
import json
from pathlib import Path
import shlex
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    folder=Path(LaunchConfiguration('run_directory').perform(context)).resolve()
    staging=json.loads((folder/'scene_staging.json').read_text())['robots']
    proof=json.loads((folder/'display_only_equivalence.json').read_text())
    if proof.get('passed') is not True or not proof.get('all_other_world_xml_equal'):
        raise ValueError('certified display-only scene copy required')
    actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(
        str(Path(share('ros_gz_sim'))/'launch/gz_sim.launch.py')),
        launch_arguments={'gz_version':'6','on_exit_shutdown':'true',
            'gz_args':'-s -r -v 2 '+shlex.quote(str(folder/'world.sdf'))}.items()),
        Node(package='ros_gz_bridge',executable='parameter_bridge',name='compliance_clock_bridge',
            parameters=[{'use_sim_time':True}],arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']),
        Node(package='ros_gz_bridge',executable='parameter_bridge',name='cp9_native_joint_load_bridge',
            parameters=[{'use_sim_time':True}],arguments=[
                f'/{r}/joint_load/Joint_{i}@geometry_msgs/msg/WrenchStamped[gz.msgs.Wrench'
                for r in ('leader','follower') for i in range(1,5)])]
    # Exact spawn policy from compliance_cp1.launch.py and CP-9's recorded
    # original_x_m. Preparation, not this scene, performs the measured approach.
    for i,(robot,yaw) in enumerate((('leader','0'),('follower','3.141592653589793'))):
        actions.append(TimerAction(period=2.+4.*i,actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(Path(share('amir_gazebo'))/'launch/robot_bringup.launch.py')),
            launch_arguments={'namespace':robot,'x':str(staging[robot]['original_x_m']),
                'yaw':yaw,'z':'.03','launch_sim':'false','headless':'true',
                'world_name':'cooperative_transport','event_startup':'false',
                'enable_d435':'false','enable_lidar':'false','bridge_ground_truth':'false',
                'tf_frame_prefix':robot+'/',
                'base_controllers_file':str(folder/(robot+'_base_controller.yaml')),
                'robot_description_file':str(folder/(robot+'.urdf'))}.items())]))
    return actions


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('run_directory'),OpaqueFunction(function=setup)])

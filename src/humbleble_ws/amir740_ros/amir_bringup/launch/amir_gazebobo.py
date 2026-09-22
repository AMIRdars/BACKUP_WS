# import os
# from ament_index_python.packages import get_package_share_path, get_package_share_directory

# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, ExecuteProcess
# from launch.substitutions import Command, LaunchConfiguration

# from launch_ros.actions import Node, SetParameter
# from launch_ros.parameter_descriptions import ParameterValue

# def generate_launch_description():
#     # パッケージパスの取得
#     description_package_path = get_package_share_path('amir_description')
#     default_model_path = description_package_path / 'urdf/amir_mecanum3.xacro'

#     # 引数の宣言
#     model_arg = DeclareLaunchArgument(
#         name='model', 
#         default_value=str(default_model_path),
#         description='Absolute path to robot urdf file'
#     )

#     # Xacroコマンドを実行してURDF文字列を生成
#     robot_description = ParameterValue(
#         Command(['xacro ', LaunchConfiguration('model')]),
#         value_type=str
#     )

#     # 1. robot_state_publisherの起動
#     robot_state_publisher_node = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         parameters=[{'robot_description': robot_description}]
#     )

#     # 2. Ignition Gazeboの起動 (空のワールド -r オプションで自動再生)
#     ign_gazebo = ExecuteProcess(
#         cmd=['ign gazebo -r empty.sdf'],
#         output='screen',
#         shell=True
#     )

#     # 3. Ignition Gazeboへのエンティティ（ロボット）のスポーン
#     ignition_spawn_entity = Node(
#         package='ros_gz_sim',
#         executable='create',
#         output='screen',
#         arguments=[
#             '-topic', 'robot_description',
#             '-name', 'amir_robot',
#             '-allow_renaming', 'true'
#         ],
#     )

#     # 4. ROS 2とGazeboの時刻を同期するためのブリッジ
#     bridge = Node(
#         package='ros_gz_bridge',
#         executable='parameter_bridge',
#         arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
#         output='screen'
#     )

#     return LaunchDescription([
#         # 全ノードでシミュレーション時間を使用するように設定
#         SetParameter(name='use_sim_time', value=True),
#         model_arg,
#         robot_state_publisher_node,
#         ign_gazebo,
#         ignition_spawn_entity,
#         bridge
#     ])

import os
from ament_index_python.packages import get_package_share_path, get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # パッケージパスの取得
    description_package_path = get_package_share_path('amir_description')
    default_model_path = description_package_path / 'urdf/amir_mecanum3.xacro'

    # --- [追加部分] Ignition GazeboにSTLファイルの場所を教えるための環境変数設定 ---
    # amir_description と mecanumrover_description の共有ディレクトリの「親」ディレクトリを取得
    amir_model_path = os.path.dirname(get_package_share_directory('amir_description'))
    mecanum_model_path = os.path.dirname(get_package_share_directory('mecanumrover_description'))
    
    # 2つのパスをコロン(:)で繋いで環境変数として定義
    gazebo_env = {
        'IGN_GAZEBO_RESOURCE_PATH': f"{amir_model_path}:{mecanum_model_path}"
    }
    # ---------------------------------------------------------------------

    # 引数の宣言
    model_arg = DeclareLaunchArgument(
        name='model', 
        default_value=str(default_model_path),
        description='Absolute path to robot urdf file'
    )

    # Xacroコマンドを実行してURDF文字列を生成
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )

    # 1. robot_state_publisherの起動
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # 2. Ignition Gazeboの起動 (空のワールド -r オプションで自動再生)
    ign_gazebo = ExecuteProcess(
        cmd=['ign gazebo -r empty.sdf'],
        output='screen',
        additional_env=gazebo_env,  # ← [変更部分] 作成した環境変数をここで渡す
        shell=True
    )

    # 3. Ignition Gazeboへのエンティティ（ロボット）のスポーン
    ignition_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'amir_robot',
            '-allow_renaming', 'true'
        ],
    )

    # 4. ROS 2とGazeboの時刻を同期するためのブリッジ
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen'
    )

    return LaunchDescription([
        # 全ノードでシミュレーション時間を使用するように設定
        SetParameter(name='use_sim_time', value=True),
        model_arg,
        robot_state_publisher_node,
        ign_gazebo,
        ignition_spawn_entity,
        bridge
    ])
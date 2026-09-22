from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    # 1. パッケージ名とファイル名の設定（※ご自身の環境に合わせて変更してください）
    robot_description_pkg = "amir_description" # URDFがあるパッケージ
    # urdf_file = "amir.urdf.xacro"         # URDFファイル名
    urdf_file = "amir_mecanum3.xacro"
    controllers_file = "amir_ros2_controllers.yaml" # コントローラー設定ファイル名

    # 2. URDFファイルの読み込み (xacroを展開)
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(robot_description_pkg), "urdf", urdf_file]
            ),
        ]
    )
    # robot_description = {"robot_description": robot_description_content}
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # 3. コントローラーのパラメータファイルへのパス
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("amir_driver"),
            "config",
            controllers_file,
        ]
    )

    # --- 以下、起動するノードの定義 ---

    # A. Controller Manager (ros2_control本体の起動)
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )

    # B. Robot State Publisher (モデル情報の配信)
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # # C. Joint State Broadcasterの起動 (状態取得)
    # joint_state_broadcaster_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    # )

    # # D. Joint Trajectory Controllerの起動 (軌道制御)
    # robot_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    # )

    # # --- 起動順序の制御 (Broadcasterが立ち上がってからControllerを立ち上げる) ---
    # delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=joint_state_broadcaster_spawner,
    #         on_exit=[robot_controller_spawner],
    #     )
    # )

    # return LaunchDescription(
    #     [
    #         control_node,
    #         robot_state_pub_node,
    #         joint_state_broadcaster_spawner,
    #         delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    #     ]
    # )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # D. アーム用コントローラーの起動 (※YAMLの名前に合わせて arm_controller に変更)
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "-c", "/controller_manager"],
    )

    # E. グリッパー用コントローラーの起動 (※新規追加)
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "-c", "/controller_manager"],
    )

    # --- 起動順序の制御 ---
    delay_arm_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner, gripper_controller_spawner], # アームとグリッパーを同時に起動
        )
    )

    return LaunchDescription(
        [
            control_node,
            robot_state_pub_node,
            joint_state_broadcaster_spawner,
            delay_arm_controller_spawner_after_joint_state_broadcaster_spawner,
        ]
    )
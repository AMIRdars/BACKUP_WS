import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def _rewrite_costmap_topics(params_file, ns):
    """コストマップの sensor/map トピックを絶対名 /<ns>/... に書き換えた一時 params
    を生成する。

    nav2 の costmap は相対トピック (scan / map) をコストマップ・サブ namespace
    (/<ns>/local_costmap/scan, /<ns>/global_costmap/map) 配下に解決してしまい、
    実際の /<ns>/scan・/<ns>/map と一致しない。その結果コストマップが地図/スキャン
    を受け取れず "current" にならず、プランナ/コントローラがハングして動かない。
    絶対名にすると costmap はそのまま購読する。
    """
    # Empty namespace must resolve to /scan and /map, not //scan and //map.
    prefix = ('/' + ns) if ns else ''
    with open(params_file) as f:
        cfg = yaml.safe_load(f)

    # local_costmap: obstacle_layer の観測源 (scan) を /<ns>/scan へ
    try:
        obl = cfg['local_costmap']['local_costmap']['ros__parameters']['obstacle_layer']
        for src in str(obl.get('observation_sources', '')).split():
            if isinstance(obl.get(src), dict) and 'topic' in obl[src]:
                obl[src]['topic'] = prefix + '/' + str(obl[src]['topic']).lstrip('/')
    except (KeyError, TypeError):
        pass

    # global_costmap: static_layer の地図 (map) を /<ns>/map へ
    try:
        stl = cfg['global_costmap']['global_costmap']['ros__parameters']['static_layer']
        if 'map_topic' in stl:
            stl['map_topic'] = prefix + '/' + str(stl['map_topic']).lstrip('/')
    except (KeyError, TypeError):
        pass

    fd, path = tempfile.mkstemp(prefix='nav2_%s_' % ns, suffix='.yaml')
    with os.fdopen(fd, 'w') as f:
        yaml.safe_dump(cfg, f)
    return path


def _bringup_group(context, *args, **kwargs):
    """navigation_launch を包む。namespace 付きのときは costmap トピックを
    書き換えた params を渡す (二重ネスト回避)。

    lifecycle manager の autostart は、全ノード生成とDDS discoveryが安定する前に
    change_stateを送ると2台目だけ応答を失う場合がある。そのため内蔵autostartを
    無効化し、3秒後に専用クライアントからSTARTUPを送る。
    """
    nav2_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch')
    ns = LaunchConfiguration('namespace').perform(context)
    params_file = _rewrite_costmap_topics(
        LaunchConfiguration('params_file').perform(context), ns)
    mode = LaunchConfiguration('mode').perform(context)
    map_file = LaunchConfiguration('map').perform(context)
    requested_autostart = (
        LaunchConfiguration('autostart').perform(context).lower()
        in ('true', '1', 'yes')
    )

    actions = []
    manager_names = []
    # A non-empty namespace must always contain the complete Nav2 stack.  This avoids
    # a mixed graph where relays are namespaced but planner/controller nodes are not.
    if ns:
        actions.append(PushRosNamespace(namespace=ns))

    if mode == 'localization':
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_launch_dir, 'localization_launch.py')
            ),
            launch_arguments={
                'namespace': ns,
                'map': map_file,
                'use_sim_time': LaunchConfiguration('use_sim_time').perform(context),
                'autostart': 'false',
                'params_file': params_file,
                'use_composition': 'False',
            }.items(),
        ))
        manager_names.append(
            f"/{ns}/lifecycle_manager_localization"
            if ns else '/lifecycle_manager_localization')

    actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_launch_dir, 'navigation_launch.py')
            ),
            launch_arguments={
                'namespace': ns,
                'use_sim_time': LaunchConfiguration('use_sim_time').perform(context),
                'autostart': 'false',
                'params_file': params_file,
                'use_composition': 'False',
            }.items(),
        ))
    manager_names.append(
        f"/{ns}/lifecycle_manager_navigation"
        if ns else '/lifecycle_manager_navigation')

    if requested_autostart:
        actions.append(TimerAction(
            period=3.0,
            actions=[Node(
                package='mecanum_navigation2',
                executable='nav2_lifecycle_startup.py',
                name='nav2_lifecycle_startup',
                namespace=ns,
                output='screen',
                parameters=[{'manager_names': manager_names}],
            )],
        ))
    return [GroupAction(actions)]


def _nav_rviz_config(base_path, ns):
    """nav2 RViz設定のtopicを /<ns>/... の絶対名にした一時設定を生成する。

    nav2_default_view.rviz は costmap/scan/plan 等を絶対ルート名 (/global_costmap/
    costmap 等) で持ち <robot_namespace> 置換もされないため、namespace 起動でも
    RViz は root を購読し実データ (/<ns>/*) と繋がらず costmap 等が表示されない
    (map/tf/goal_pose だけはremapされるので地図は出る)。
    相対名はRVizプラグインの初期化順序により解決先が不明瞭になる場合があるため、
    表示用topicはすべて /<ns>/... の絶対名へ固定する。
    """
    import re
    with open(base_path) as f:
        text = f.read()
    prefix = '/' + ns.strip('/')
    text = text.replace('<robot_namespace>', prefix)
    text = re.sub(
        r'(?m)^(\s*Value: )/(?!' + re.escape(ns.strip('/')) + r'/)',
        r'\1' + prefix + '/',
        text,
    )
    config = yaml.safe_load(text)

    def disable_unused_downsampled_costmap(value):
        if isinstance(value, dict):
            if value.get('Name') == 'Downsampled Costmap':
                value['Enabled'] = False
                value['Value'] = False
            for child in value.values():
                disable_unused_downsampled_costmap(child)
        elif isinstance(value, list):
            for child in value:
                disable_unused_downsampled_costmap(child)

    # NavFnはdownsampled_costmapを配信しない。Global Costmapの異常と誤認しない
    # よう、存在しない表示だけを既定で無効化する。
    disable_unused_downsampled_costmap(config)
    fd, path = tempfile.mkstemp(prefix='nav2_rviz_%s_' % ns, suffix='.rviz')
    with os.fdopen(fd, 'w') as f:
        yaml.safe_dump(config, f, sort_keys=False)
    return path


def _rviz_cmd(context, *args, **kwargs):
    """RViz を起動。namespace 付きのときは topic を相対名化した設定を渡し、
    costmap 等も /<ns>/ 配下を購読させる。"""
    if LaunchConfiguration('use_rviz').perform(context).lower() not in ('true', '1', 'yes'):
        return []
    ns = LaunchConfiguration('namespace').perform(context)
    cfg = LaunchConfiguration('rviz_config_file').perform(context)
    if ns:
        cfg = _nav_rviz_config(cfg, ns)
    remappings = []
    if ns:
        remappings = [
            ('/map', 'map'), ('/tf', 'tf'), ('/tf_static', 'tf_static'),
            ('/goal_pose', 'goal_pose'), ('/clicked_point', 'clicked_point'),
            ('/initialpose', 'initialpose'),
        ]
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() in (
        'true', '1', 'yes')
    return [Node(
        package='rviz2', executable='rviz2', namespace=ns,
        arguments=['-d', cfg], parameters=[{'use_sim_time': use_sim_time}],
        remappings=remappings, output='screen')]


def generate_launch_description():
    this_dir = get_package_share_directory('mecanum_navigation2')
    this_launch_dir = os.path.join(this_dir, 'launch')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='',
        description='Top-level namespace'
    )
    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace', default_value='false',
        description='Compatibility argument; a non-empty namespace is always applied'
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(this_dir, 'params', 'amir.yaml'),
        description='Full path to the ROS2 parameters file'
    )
    declare_mode_cmd = DeclareLaunchArgument(
        'mode', default_value='slam', choices=['slam', 'localization'],
        description='slam: independent online mapping; localization: saved map + AMCL'
    )
    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(this_dir, 'map', 'amir_world.yaml'),
        description='Saved map YAML used in localization mode'
    )
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack'
    )
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        choices=['true', 'false'],
        description='Whether to start RViz'
    )
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(
            get_package_share_directory('nav2_bringup'),
            'rviz', 'nav2_default_view.rviz'),
        description='Full path to the RViz config file'
    )

    # slam_toolbox: online async mapping (builds map while driving)
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(this_launch_dir, 'slam_toolbox_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'namespace': namespace,
        }.items(),
        condition=LaunchConfigurationEquals('mode', 'slam'),
    )

    # nav2: planner + controller + behavior (no localization — slam_toolbox handles TF)
    # costmap の sensor/map トピック二重ネストを避けるため OpaqueFunction 内で
    # namespace に応じた params を生成して渡す (_bringup_group / _rewrite_costmap_topics)。
    bringup_cmd_group = OpaqueFunction(function=_bringup_group)

    # Relay Nav2 /cmd_vel output to /rover_twist (→ mecanum drive controller)
    cmd_vel_relay = Node(
        package='mecanum_navigation2',
        executable='cmd_vel_relay.py',
        name='cmd_vel_relay',
        namespace=namespace,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # RViz は OpaqueFunction 内で生成 (namespace 付きは topic を相対名化した設定を渡す)。
    # false だと RViz がルートの /map /scan /global_costmap/costmap 等を掴み、実データ
    # (/<ns>/*) と繋がらず costmap 等が表示されない。
    rviz_cmd = OpaqueFunction(function=_rviz_cmd)

    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_mode_cmd)
    ld.add_action(declare_map_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_rviz_config_file_cmd)

    ld.add_action(slam_toolbox_launch)
    ld.add_action(bringup_cmd_group)
    ld.add_action(cmd_vel_relay)
    ld.add_action(rviz_cmd)

    return ld

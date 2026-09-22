import os
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 【追加】パッケージのパスを取得するためのモジュールをインポート
from ament_index_python.packages import get_package_share_directory

def _gen_sdf_and_spawn(context, *args, **kwargs):
    entity   = LaunchConfiguration('entity').perform(context)
    stl_file = LaunchConfiguration('stl_file').perform(context)
    scale    = float(LaunchConfiguration('scale').perform(context))

    x     = float(LaunchConfiguration('x').perform(context))
    y     = float(LaunchConfiguration('y').perform(context))
    z     = float(LaunchConfiguration('z').perform(context))
    roll  = float(LaunchConfiguration('roll').perform(context))
    pitch = float(LaunchConfiguration('pitch').perform(context))
    yaw   = float(LaunchConfiguration('yaw').perform(context))

    # 【変更】mecanumrover3_gazebo パッケージの絶対パスを取得
    pkg_share_path = get_package_share_directory('mecanumrover3_gazebo')

    # 【変更】パッケージパス + 'models' + ファイル名 で確実な絶対パスを生成
    abs_stl_path = os.path.join(pkg_share_path, 'models', stl_file)
    stl_uri = f"file://{abs_stl_path}"

    # --- (これ以降の sdf 生成や Node 定義は前回と全く同じです) ---
    sdf = f'''<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="{entity}">
    <static>true</static>
    <link name="wall_link">
      <collision name="collision">
        <geometry>
          <mesh>
            <uri>{stl_uri}</uri>
            <scale>{scale} {scale} {scale}</scale>
          </mesh>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <mesh>
            <uri>{stl_uri}</uri>
            <scale>{scale} {scale} {scale}</scale>
          </mesh>
        </geometry>
        <material>
          <ambient>0.6 0.6 0.6 1.0</ambient>
          <diffuse>0.6 0.6 0.6 1.0</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
'''

    tmpdir = tempfile.mkdtemp(prefix='spawn_wall_')
    sdf_path = os.path.join(tmpdir, f'{entity}.sdf')
    with open(sdf_path, 'w') as f:
        f.write(sdf)

    delete_cmd = ExecuteProcess(
        cmd=[
            'ign', 'service',
            '-s', '/world/default/remove',
            '--reqtype', 'ignition.msgs.Entity',
            '--reptype', 'ignition.msgs.Boolean',
            '--timeout', '1000',
            '--req', f'name: "{entity}" type: MODEL',
        ],
        output='log',
    )

    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', sdf_path,
            '-name', entity,
            '-x', str(x), '-y', str(y), '-z', str(z),
            '-R', str(roll), '-P', str(pitch), '-Y', str(yaw),
            '-allow_renaming', 'false',
        ],
        output='screen',
    )

    return [
        delete_cmd,
        TimerAction(period=0.5, actions=[spawn_node]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('entity',   default_value='custom_wall'),
        # ファイル名だけを指定（パスはLaunch内で自動補完される）
        DeclareLaunchArgument('stl_file', default_value='Wall.stl'),
        DeclareLaunchArgument('scale',    default_value='1.0'),

        DeclareLaunchArgument('x',     default_value='1.0'),
        DeclareLaunchArgument('y',     default_value='0.0'),
        DeclareLaunchArgument('z',     default_value='0.0'),
        DeclareLaunchArgument('roll',  default_value='0.0'),
        DeclareLaunchArgument('pitch', default_value='0.0'),
        DeclareLaunchArgument('yaw',   default_value='0.0'),

        OpaqueFunction(function=_gen_sdf_and_spawn),
    ])
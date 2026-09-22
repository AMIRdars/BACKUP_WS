import math
import os
import random  # ランダム生成用に追加
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _gen_sdf_and_spawn(context, *args, **kwargs):
    base_entity = LaunchConfiguration('entity').perform(context)
    shape  = LaunchConfiguration('shape').perform(context)

    # 固定のZ, 姿勢, 質量を取得
    z      = float(LaunchConfiguration('z').perform(context))
    roll   = float(LaunchConfiguration('roll').perform(context))
    pitch  = float(LaunchConfiguration('pitch').perform(context))
    yaw    = float(LaunchConfiguration('yaw').perform(context))
    mass   = float(LaunchConfiguration('mass').perform(context))
    
    # 複数生成用のパラメータ
    count  = int(LaunchConfiguration('count').perform(context))
    
    # ランダム配置の範囲を取得
    x_min  = float(LaunchConfiguration('x_min').perform(context))
    x_max  = float(LaunchConfiguration('x_max').perform(context))
    y_min  = float(LaunchConfiguration('y_min').perform(context))
    y_max  = float(LaunchConfiguration('y_max').perform(context))

    # Ignition Gazebo の摩擦・反発係数
    kp = 1e6
    kd = 1e3
    mu = 3.0

    surface_xml = f"""
        <surface>
          <friction>
            <ode>
              <mu>{mu}</mu>
              <mu2>{mu}</mu2>
            </ode>
          </friction>
          <contact>
            <ode>
              <kp>{kp}</kp>
              <kd>{kd}</kd>
              <max_vel>0.01</max_vel>
              <min_depth>0.001</min_depth>
            </ode>
          </contact>
        </surface>"""

    # 形状ごとの慣性モーメント・ジオメトリ計算
    if shape == 'cylinder':
        radius = float(LaunchConfiguration('radius').perform(context))
        height = float(LaunchConfiguration('size_z').perform(context))
        ixx = iyy = (1.0 / 12.0) * mass * (3.0 * radius**2 + height**2)
        izz = 0.5 * mass * radius**2
        geom_xml = f"""<cylinder>
              <radius>{radius}</radius>
              <length>{height}</length>
            </cylinder>"""
        link_name = 'cylinder_link'
    else:
        size_x = float(LaunchConfiguration('size_x').perform(context))
        size_y = float(LaunchConfiguration('size_y').perform(context))
        size_z = float(LaunchConfiguration('size_z').perform(context))
        ixx = (1.0 / 12.0) * mass * (size_y**2 + size_z**2)
        iyy = (1.0 / 12.0) * mass * (size_x**2 + size_z**2)
        izz = (1.0 / 12.0) * mass * (size_x**2 + size_y**2)
        geom_xml = f"""<box>
              <size>{size_x} {size_y} {size_z}</size>
            </box>"""
        link_name = 'box_link'

    actions = []
    tmpdir = tempfile.mkdtemp(prefix='spawn_obj_')

    # 指定された数だけループしてエンティティを生成
    for i in range(count):
        # 名前を個別に設定
        entity_name = f"{base_entity}_{i}"
        
        # XとYの座標を範囲内でランダムに生成
        current_x = random.uniform(x_min, x_max)
        current_y = random.uniform(y_min, y_max)

        # 個別のSDFを生成
        sdf = f'''<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="{entity_name}">
    <static>false</static>
    <link name="{link_name}">
      <inertial>
        <mass>{mass}</mass>
        <inertia>
          <ixx>{ixx}</ixx>
          <iyy>{iyy}</iyy>
          <izz>{izz}</izz>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyz>0.0</iyz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          {geom_xml}
        </geometry>
        {surface_xml}
      </collision>
      <visual name="visual">
        <geometry>
          {geom_xml}
        </geometry>
        <material>
          <ambient>0.9 0.5 0.1 1.0</ambient>
          <diffuse>0.9 0.5 0.1 1.0</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
'''
        sdf_path = os.path.join(tmpdir, f'{entity_name}.sdf')
        with open(sdf_path, 'w') as f:
            f.write(sdf)

        # 既存モデルの削除コマンド
        delete_cmd = ExecuteProcess(
            cmd=[
                'ign', 'service',
                '-s', '/world/default/remove',
                '--reqtype', 'ignition.msgs.Entity',
                '--reptype', 'ignition.msgs.Boolean',
                '--timeout', '1000',
                '--req', f'name: "{entity_name}" type: MODEL',
            ],
            output='log',
        )

        # 新規モデルのスポーンノード
        spawn_node = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-file', sdf_path,
                '-name', entity_name,
                '-x', str(current_x), '-y', str(current_y), '-z', str(z),
                '-R', str(roll), '-P', str(pitch), '-Y', str(yaw),
                '-allow_renaming', 'false',
            ],
            output='screen',
        )

        # アクションリストに追加（各オブジェクトの生成タイミングを少しずらして衝突を緩和）
        actions.extend([
            delete_cmd,
            TimerAction(period=0.5 + (i * 0.1), actions=[spawn_node]),
        ])

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('entity', default_value='target_obj'),
        # DeclareLaunchArgument('shape',  default_value='box', description="'cylinder' or 'box'"),
        DeclareLaunchArgument('shape',  default_value='cylinder', description="'cylinder' or 'box'"),

        # 個数指定（デフォルト5個）
        DeclareLaunchArgument('count',  default_value='5', description='Number of models to spawn'),
        
        # ランダム配置の範囲指定（x:1〜3, y:-2〜2）
        DeclareLaunchArgument('x_min',  default_value='1.0'),
        DeclareLaunchArgument('x_max',  default_value='3.0'),
        DeclareLaunchArgument('y_min',  default_value='-2.0'),
        DeclareLaunchArgument('y_max',  default_value='2.0'),
        
        DeclareLaunchArgument('z',     default_value='0.05'),
        DeclareLaunchArgument('roll',  default_value='0.0'),
        DeclareLaunchArgument('pitch', default_value='0.0'),
        DeclareLaunchArgument('yaw',   default_value='0.0'),
        DeclareLaunchArgument('radius',  default_value='0.025'),
        DeclareLaunchArgument('size_z',  default_value='0.10'),
        DeclareLaunchArgument('size_x',  default_value='0.05'), 
        DeclareLaunchArgument('size_y',  default_value='0.05'),
        DeclareLaunchArgument('mass',    default_value='0.1'),

        OpaqueFunction(function=_gen_sdf_and_spawn),
    ])
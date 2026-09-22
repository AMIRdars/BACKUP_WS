import math
import os
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _gen_sdf_and_spawn(context, *args, **kwargs):
    base_entity = LaunchConfiguration('entity').perform(context)
    shape  = LaunchConfiguration('shape').perform(context)

    x_base = float(LaunchConfiguration('x').perform(context))
    y_base = float(LaunchConfiguration('y').perform(context))
    z      = float(LaunchConfiguration('z').perform(context))
    roll   = float(LaunchConfiguration('roll').perform(context))
    pitch  = float(LaunchConfiguration('pitch').perform(context))
    yaw    = float(LaunchConfiguration('yaw').perform(context))
    mass   = float(LaunchConfiguration('mass').perform(context))

    count  = int(LaunchConfiguration('count').perform(context))
    cols   = int(LaunchConfiguration('cols').perform(context))
    x_step = float(LaunchConfiguration('x_step').perform(context))
    y_step = float(LaunchConfiguration('y_step').perform(context))

    rows_per_col = math.ceil(count / cols)

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

    for i in range(count):
        entity_name = f"{base_entity}_{i}"
        col_idx   = i // rows_per_col
        row_idx   = i %  rows_per_col
        current_x = x_base + col_idx * x_step
        current_y = y_base + row_idx * y_step

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

        actions.extend([
            delete_cmd,
            TimerAction(period=0.5, actions=[spawn_node]),
        ])

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('entity', default_value='target_obj'),
        DeclareLaunchArgument('shape',  default_value='cylinder', description="'cylinder' or 'box'"),

        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        # 物体数・グリッド設定 — ここを変更するだけで調整できる
        #
        #   count  : スポーンする総個数
        #   cols   : 列数 (count ÷ cols = 1列あたりの行数)
        #   x      : 1列目の X 座標 [m]
        #   x_step : 列間の X 方向間隔 [m]  (2列目は x + x_step)
        #   y      : 各列の先頭 Y 座標 [m]
        #   y_step : 行間の Y 方向間隔 [m]
        #   z      : スポーン Z 座標 (物体中心高さ) [m]
        #
        #   デフォルト: 2列×5行
        #     列0: x=0.50, y=-0.30,-0.15,0.00,0.15,0.30
        #     列1: x=0.65, y=-0.30,-0.15,0.00,0.15,0.30
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        DeclareLaunchArgument('count',  default_value='10'),
        DeclareLaunchArgument('cols',   default_value='2'),
        DeclareLaunchArgument('x',      default_value='0.3'),
        DeclareLaunchArgument('x_step', default_value='0.15'),
        DeclareLaunchArgument('y',      default_value='-0.3'),
        DeclareLaunchArgument('y_step', default_value='0.15'),
        DeclareLaunchArgument('z',      default_value='0.05'),

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

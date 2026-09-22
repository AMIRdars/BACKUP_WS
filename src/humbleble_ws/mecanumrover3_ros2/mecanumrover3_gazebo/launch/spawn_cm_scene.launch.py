"""
spawn_cm_scene.launch.py

jointfix_cm_pick_place.cpp の衝突オブジェクト定義に対応した
Ignition Gazebo モデルを一括スポーンする。

スポーン順序（重なり防止）:
  Phase1 (0.5s〜) : 静的オブジェクト（table_pick, table_place, place_divider）
  Phase2 (2.5s〜) : 動的オブジェクト（円柱 obj_*）をテーブル上面直上に配置

座標は jointfix_cm_pick_place.cpp の base_footprint 基準値と一致。
world→base_footprint TF がゼロオフセットであることを前提とする。

使い方:
  ros2 launch mecanumrover3_gazebo spawn_cm_scene.launch.py
"""

import os
import tempfile

from launch import LaunchDescription
from launch.actions import ExecuteProcess, OpaqueFunction, TimerAction


# ══════════════════════════════════════════════════════════════════════════════
# jointfix_cm_pick_place.cpp の定数と同じ値
# ══════════════════════════════════════════════════════════════════════════════

OBJ_RADIUS = 0.025   # [m]
OBJ_HEIGHT = 0.10    # [m]

OBJ_POSITIONS = [
    (0.5, -0.20, 0.18),
    (0.5, -0.05, 0.18),
]

PLACE_POSITIONS = [
    (-0.20, 0.40, 0.15),
    (-0.05, 0.40, 0.15),
]

# table_pick
PICK_TOP_Z   = 0.10
TABLE_THICK  = 0.10
PICK_CX      = 0.50
PICK_CY      = 0.025
PICK_SIZE_X  = 0.25
PICK_SIZE_Y  = 0.65

# table_place
PLACE_TOP_Z      = 0.10
PLACE_SLAB_THICK = 0.10
PLACE_CX         = 0.025
PLACE_CY         = 0.40
PLACE_SIZE_X     = 0.65
PLACE_SIZE_Y     = 0.25

# place_divider サイズ
DIV_THICK  = 0.02   # x方向の厚み [m]
DIV_HEIGHT = 0.14   # z方向の高さ [m]
DIV_DEPTH  = 0.25   # y方向の奥行き [m]

# 仕切りの中心座標リスト（base_footprint 基準）
# (cx, cy, cz) を直接指定。要素を増減するだけで枚数を変えられる。
DIVIDER_POSITIONS = [
    (-0.125, 0.40, 0.17),  # 仕切り0
]
DIVIDER_POSITIONS = []

# 円柱をテーブル上面から何メートル上に落とすか（初期重なり防止）
OBJ_SPAWN_CLEARANCE = 0.03   # [m]

# 静的オブジェクトが落ち着くのを待つ時間 [s]
STATIC_SETTLE_TIME = 2.0


# ══════════════════════════════════════════════════════════════════════════════
# SDF 生成ヘルパー
# ══════════════════════════════════════════════════════════════════════════════

def _surface_xml_static():
    """テーブル・仕切り用: 高剛性・適度な摩擦"""
    return """
        <surface>
          <friction>
            <ode><mu>0.8</mu><mu2>0.8</mu2></ode>
          </friction>
          <contact>
            <ode>
              <kp>1e10</kp>
              <kd>1e4</kd>
              <max_vel>0.001</max_vel>
              <min_depth>0.0005</min_depth>
            </ode>
          </contact>
        </surface>"""


def _surface_xml_dynamic():
    """把持対象円柱用: 高摩擦・高剛性（グリッパーで掴みやすく、台に沈まない）"""
    return """
        <surface>
          <friction>
            <ode><mu>3.0</mu><mu2>3.0</mu2></ode>
          </friction>
          <contact>
            <ode>
              <kp>1e8</kp>
              <kd>1e3</kd>
              <max_vel>0.01</max_vel>
              <min_depth>0.001</min_depth>
            </ode>
          </contact>
        </surface>"""


def _make_cylinder_sdf(name, radius, height, mass, color_rgba):
    ixx = iyy = (1.0 / 12.0) * mass * (3 * radius**2 + height**2)
    izz = 0.5 * mass * radius**2
    r, g, b, a = color_rgba
    geom = f"<cylinder><radius>{radius}</radius><length>{height}</length></cylinder>"
    return f"""<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="{name}">
    <static>false</static>
    <link name="link">
      <inertial>
        <mass>{mass}</mass>
        <inertia>
          <ixx>{ixx:.6f}</ixx><iyy>{iyy:.6f}</iyy><izz>{izz:.6f}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>{geom}</geometry>
        {_surface_xml_dynamic()}
      </collision>
      <visual name="visual">
        <geometry>{geom}</geometry>
        <material>
          <ambient>{r} {g} {b} {a}</ambient>
          <diffuse>{r} {g} {b} {a}</diffuse>
          <specular>0.3 0.3 0.3 1.0</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


def _make_box_sdf(name, sx, sy, sz, mass, color_rgba):
    ixx = (1.0 / 12.0) * mass * (sy**2 + sz**2)
    iyy = (1.0 / 12.0) * mass * (sx**2 + sz**2)
    izz = (1.0 / 12.0) * mass * (sx**2 + sy**2)
    r, g, b, a = color_rgba
    geom = f"<box><size>{sx} {sy} {sz}</size></box>"
    return f"""<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <inertial>
        <mass>{mass}</mass>
        <inertia>
          <ixx>{ixx:.6f}</ixx><iyy>{iyy:.6f}</iyy><izz>{izz:.6f}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>{geom}</geometry>
        {_surface_xml_static()}
      </collision>
      <visual name="visual">
        <geometry>{geom}</geometry>
        <material>
          <ambient>{r} {g} {b} {a}</ambient>
          <diffuse>{r} {g} {b} {a}</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


# ══════════════════════════════════════════════════════════════════════════════
# スポーンアクション生成
# ══════════════════════════════════════════════════════════════════════════════

def _make_spawn_actions(name, sdf_str, cx, cy, cz, tmpdir, delay):
    path = os.path.join(tmpdir, f'{name}.sdf')
    with open(path, 'w') as f:
        f.write(sdf_str)

    delete = ExecuteProcess(
        cmd=[
            'ign', 'service',
            '-s', '/world/default/remove',
            '--reqtype', 'ignition.msgs.Entity',
            '--reptype', 'ignition.msgs.Boolean',
            '--timeout', '1000',
            '--req', f'name: "{name}" type: MODEL',
        ],
        output='log',
    )
    spawn = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-file', path,
            '-name', name,
            '-x', str(round(cx, 6)),
            '-y', str(round(cy, 6)),
            '-z', str(round(cz, 6)),
            '-allow_renaming', 'false',
        ],
        output='screen',
    )
    return [delete, TimerAction(period=delay, actions=[spawn])]


def _spawn_actions(context, *args, **kwargs):
    tmpdir = tempfile.mkdtemp(prefix='spawn_cm_scene_')
    actions = []

    # ─────────────────────────────────────────────────────────────────────────
    # Phase 1: 静的オブジェクト（テーブル・仕切り）を先に出現させる
    # ─────────────────────────────────────────────────────────────────────────
    static_delay = 0.5   # 最初の静的オブジェクトのスポーン時刻 [s]
    STATIC_INTERVAL = 0.4

    # ── ピック台 table_pick（ベージュ）
    sdf = _make_box_sdf(
        'table_pick',
        sx=PICK_SIZE_X, sy=PICK_SIZE_Y, sz=TABLE_THICK,
        mass=10.0,
        color_rgba=(0.76, 0.60, 0.42, 1.0),
    )
    cz = PICK_TOP_Z - TABLE_THICK / 2.0   # 上面が PICK_TOP_Z になる中心 z
    actions += _make_spawn_actions('table_pick', sdf, PICK_CX, PICK_CY, cz, tmpdir, static_delay)
    static_delay += STATIC_INTERVAL

    # ── プレース台 table_place（青灰）
    sdf = _make_box_sdf(
        'table_place',
        sx=PLACE_SIZE_X, sy=PLACE_SIZE_Y, sz=PLACE_SLAB_THICK,
        mass=10.0,
        color_rgba=(0.45, 0.60, 0.75, 1.0),
    )
    cz = PLACE_TOP_Z - PLACE_SLAB_THICK / 2.0
    actions += _make_spawn_actions('table_place', sdf, PLACE_CX, PLACE_CY, cz, tmpdir, static_delay)
    static_delay += STATIC_INTERVAL

    # ── プレース仕切り壁（薄灰）
    for k, (div_cx, div_cy, div_cz) in enumerate(DIVIDER_POSITIONS):
        name = f'place_divider_{k}'
        sdf = _make_box_sdf(
            name,
            sx=DIV_THICK, sy=DIV_DEPTH, sz=DIV_HEIGHT,
            mass=1.0,
            color_rgba=(0.80, 0.80, 0.80, 1.0),
        )
        actions += _make_spawn_actions(name, sdf, div_cx, div_cy, div_cz, tmpdir, static_delay)
        static_delay += STATIC_INTERVAL

    # ─────────────────────────────────────────────────────────────────────────
    # Phase 2: 動的オブジェクト（円柱）を静的オブジェクト出現後に落とす
    #   spawn z = テーブル上面 + 半径高 + クリアランス
    #   → 重力で落下してテーブル上に着地する
    # ─────────────────────────────────────────────────────────────────────────
    obj_delay = static_delay + STATIC_SETTLE_TIME
    OBJ_INTERVAL = 0.4

    obj_spawn_z = PICK_TOP_Z + OBJ_HEIGHT / 2.0 + OBJ_SPAWN_CLEARANCE

    for i, (ox, oy, _) in enumerate(OBJ_POSITIONS):
        sdf = _make_cylinder_sdf(
            f'obj_{i}',
            radius=OBJ_RADIUS,
            height=OBJ_HEIGHT,
            mass=0.1,
            color_rgba=(0.9, 0.5, 0.1, 1.0),
        )
        # x, y は OBJ_POSITIONS から、z はテーブル上面直上を計算値で使う
        actions += _make_spawn_actions(f'obj_{i}', sdf, ox, oy, obj_spawn_z, tmpdir, obj_delay)
        obj_delay += OBJ_INTERVAL

    return actions


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=_spawn_actions),
    ])

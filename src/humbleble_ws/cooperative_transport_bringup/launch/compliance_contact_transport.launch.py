"""One-command CP-9 native grasp, guarded low-speed carry, evaluation and stop.

This is an experiment launcher. Ground truth stays inside the external trial;
the existing follower and its safety/actuator boundary remain unchanged.
"""
from datetime import datetime
import json
import math
from pathlib import Path
import re
import uuid

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo, OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration


STAGE = Path('audits/whole_body_compliance_20261019/cp9_transport')


def default_workspace():
    # Support both a symlink install and an installed copy of the launch file.
    share = Path(get_package_share_directory('cooperative_transport_bringup'))
    for candidate in (*Path(__file__).resolve().parents, *share.parents):
        if (candidate / 'tools/amir_cp9_transport_trial.py').is_file():
            return str(candidate)
    return ''


def trial_finished(event, context, *, output):
    result_file = output / 'result.json'
    try:
        result = json.loads(result_file.read_text())
    except (OSError, ValueError) as error:
        if context.is_shutdown:
            return []
        raise RuntimeError(f'CP-9 produced no readable result: {result_file}: {error}')
    if context.is_shutdown:
        return [LogInfo(msg=f'CP-9 interrupted. Result: {result_file}')]
    if event.returncode != 0 or result.get('passed') is not True:
        raise RuntimeError(
            f'CP-9 FAILED: {result.get("error", "see result.json")}. Result: {result_file}')
    return [LogInfo(msg=f'CP-9 PASSED. Result: {result_file}'),
            EmitEvent(event=Shutdown(reason='CP-9 trial completed and stopped'))]


def setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    for unsupported in ('translation_acceleration_m_s2', 'angular_velocity_deg_s',
                        'angular_acceleration_deg_s2'):
        if unsupported in context.launch_configurations:
            raise ValueError(f'{unsupported} is not configurable in the validated CP-9 trial')
    distance = float(value('forward_distance_m'))
    speed = float(value('translation_velocity_m_s'))
    if not math.isfinite(distance) or not 0 < distance <= 1.:
        raise ValueError('forward_distance_m must be >0 and <=1.0 m')
    if not math.isfinite(speed) or not 0 < speed <= .005:
        raise ValueError('translation_velocity_m_s must be >0 and <=0.005 (5 mm/s trial envelope)')
    for name in ('rotation_angle_deg', 'world_y_distance_m'):
        if float(value(name)) != 0.:
            raise ValueError(f'{name} must be zero: native-contact turning/lateral carry is unvalidated')
    domain = int(value('ros_domain_id'))
    if not 0 <= domain <= 232:
        raise ValueError('ros_domain_id must be between 0 and 232')
    partition = value('ign_partition') or 'amir_cp9_launch_' + uuid.uuid4().hex[:12]
    if not re.fullmatch(r'amir_cp9_[A-Za-z0-9_]+', partition):
        raise ValueError('ign_partition must start with amir_cp9_ and contain letters, digits or _')
    root_arg = value('workspace_directory')
    if not root_arg:
        raise ValueError('workspace_directory is required: the CP-9 tools and evidence are workspace-local')
    root = Path(root_arg).expanduser().resolve()
    for relative in ('tools/amir_cp9_transport_trial.py', 'tools/cp9_transport_preparation.py',
                     'tools/amir_cp9_grasp_preparation.py', 'tools/amir_make_safety_profiles.py',
                     'src/humbleble_ws/amir_compliance_control/amir_compliance_control',
                     'audits/whole_body_compliance_20261019/cp1/motion_run3/motion.json'):
        if not (root / relative).exists():
            raise ValueError(f'Missing CP-9 workspace input: {root / relative}')
    generated = Path(value('run_directory')).expanduser().resolve() if value('run_directory') else root / STAGE / 'generated'
    metrology = Path(value('metrology_directory')).expanduser().resolve() if value('metrology_directory') else root / STAGE / 'metrology_02'
    if not generated.is_dir() or not (metrology / 'calibration/calibration.json').is_file():
        raise ValueError('Existing run_directory and metrology_directory/calibration/calibration.json required')
    output = Path(value('output_directory')).expanduser().resolve() if value('output_directory') else root / STAGE / ('launch_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    if output.exists():
        raise ValueError(f'output_directory already exists; use a new path: {output}')
    env = {'ROS_DOMAIN_ID': str(domain), 'ROS_LOCALHOST_ONLY': '1',
           'IGN_PARTITION': partition, 'OPENBLAS_NUM_THREADS': '1',
           'OMP_NUM_THREADS': '1', 'MKL_NUM_THREADS': '1', 'PYTHONUNBUFFERED': '1'}
    trial_cmd = ['/usr/bin/python3', str(root / 'tools/amir_cp9_transport_trial.py'),
        '--run-directory', str(generated), '--metrology-directory', str(metrology),
        '--output', str(output), '--distance', str(distance), '--speed', str(speed)]
    if context.launch_configurations.get('grip_maintenance', 'false') == 'true':
        trial_cmd.append('--grip-maintenance')
    trial = ExecuteProcess(
        name='cp9_contact_transport', cwd=str(root), additional_env=env, output='screen',
        # The trial stops its own independently grouped children in finally.
        sigterm_timeout='120', sigkill_timeout='10',
        cmd=trial_cmd)
    actions = [LogInfo(msg=f'CP-9 native-contact forward carry: {distance * 1000:g} mm, maximum {speed * 1000:g} mm/s'),
               LogInfo(msg=f'Minimum cruise time: {distance / speed:g} simulated seconds; preparation and settling are additional.'),
               LogInfo(msg='把持 → 安全確認 → 10秒静止保持 → 搬送 → 整定を自動実行します。準備・搬送の進捗を約5秒ごとに表示します。'),
               LogInfo(msg=f'CP-9 output_directory: {output}'),
               LogInfo(msg=f'ROS_DOMAIN_ID={domain}, ROS_LOCALHOST_ONLY=1, IGN_PARTITION={partition}'),
               RegisterEventHandler(OnProcessExit(target_action=trial,
                   on_exit=lambda event, ctx: trial_finished(event, ctx, output=output)))]
    if value('headless') == 'false':
        gui_config = Path(get_package_share_directory('cooperative_transport_bringup')) / 'config/compliance_contact_gui.config'
        if not gui_config.is_file():
            raise ValueError('Missing contact GUI config; rebuild cooperative_transport_bringup')
        actions.append(ExecuteProcess(
            name='cp9_gazebo_gui', cwd=str(root), additional_env=env, output='screen',
            # Own the GUI PID directly: the nested ros_gz_sim shell can exit
            # on SIGTERM without terminating its still-visible GUI child.
            sigterm_timeout='5', sigkill_timeout='5',
            cmd=['ign', 'gazebo', '-g', '--gui-config', str(gui_config),
                 '--force-version', '6']))
    actions.append(trial)
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('forward_distance_m', default_value='1.0',
                              description='Positive forward goal in metres, maximum 1.0.'),
        DeclareLaunchArgument('translation_velocity_m_s', default_value='0.005',
                              description='Positive maximum speed in m/s, maximum 0.005.'),
        DeclareLaunchArgument('rotation_angle_deg', default_value='0.0',
                              description='Reserved; only zero is supported in this CP-9 entry.'),
        DeclareLaunchArgument('world_y_distance_m', default_value='0.0',
                              description='Reserved; only zero is supported in this CP-9 entry.'),
        DeclareLaunchArgument('record_data', default_value='true', choices=['true'],
                              description='Always retain safety, graph and evaluation evidence.'),
        DeclareLaunchArgument('grip_maintenance', default_value='false', choices=['true', 'false'],
                              description='Opt-in CP-11 local bounded grip maintenance; existing stop limits remain active.'),
        DeclareLaunchArgument('workspace_directory', default_value=default_workspace(),
                              description='Workspace containing CP-9 tools and calibration evidence.'),
        DeclareLaunchArgument('run_directory', default_value='',
                              description='Existing generated path matching the calibration; empty uses CP-9 default.'),
        DeclareLaunchArgument('metrology_directory', default_value='',
                              description='Existing metrology evidence path; empty uses metrology_02.'),
        DeclareLaunchArgument('output_directory', default_value='',
                              description='New result directory; empty generates a timestamped directory.'),
        DeclareLaunchArgument('ros_domain_id', default_value='93'),
        DeclareLaunchArgument('ign_partition', default_value='',
                              description='Empty generates a unique amir_cp9_ partition.'),
        OpaqueFunction(function=setup),
    ])

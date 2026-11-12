"""Qualified CP-12 straight native-contact carry with local grip maintenance.

The 1 m / 0.005 m/s public entry remains unchanged. This separate entry
requires passing motion and cruise-fault evidence from the current sources.
"""
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import uuid

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration


STAGE = Path('audits/whole_body_compliance_20261019/cp12_speed_release')
PRIOR = Path('audits/whole_body_compliance_20261019/cp9_transport')
DISPLAY_SCENE = Path('audits/whole_body_compliance_20261019/cp12_speed_release/generated_gui20')
SCENE_FILES = ('leader.urdf','follower.urdf','world.sdf','leader_base_controller.yaml',
    'follower_base_controller.yaml','leader_controllers.yaml','follower_controllers.yaml',
    'scene_staging.json','mimic_initialization.json','display_only_equivalence.json')


def workspace():
    share = Path(get_package_share_directory('cooperative_transport_bringup'))
    for candidate in (*Path(__file__).resolve().parents,*share.parents):
        if (candidate/'tools/amir_cp9_transport_trial.py').is_file():
            return str(candidate)
    return ''


def qualification(root):
    manifest_path = root/STAGE/'qualified_speed_010.json'
    if not manifest_path.is_file():
        raise ValueError('CP-12 high-speed qualification is incomplete: '+str(manifest_path))
    manifest = json.loads(manifest_path.read_text())
    if manifest.get('passed') is not True or manifest.get('simulation_only') is not True:
        raise ValueError('passing simulation-only qualification required')
    if (manifest.get('schema_version') != 3 or manifest.get('grip_maintenance') is not True
            or manifest.get('publication_mode') != 'ASYNCHRONOUS'
            or manifest.get('speed_m_s') != .1 or manifest.get('distance_m') != 3.):
        raise ValueError('CP-12 maintained-grip 3 m / 0.1 m/s qualification required')
    control = root/'src/humbleble_ws/amir_compliance_control/amir_compliance_control'
    required_sources = {str(p.relative_to(root)) for p in control.glob('*.py')}
    required_sources.update(str(p.relative_to(root)) for p in (
        Path(__file__).resolve(), root/'tools/amir_cp9_transport_trial.py',
        control/'grip_maintenance.py', control/'native_grasp.py',
        root/'src/humbleble_ws/amir_compliance_control/launch/native_grasp_monitor.launch.py',
        root/'src/humbleble_ws/amir_compliance_control/launch/whole_body_transport.launch.py',
        root/'src/humbleble_ws/cooperative_transport_bringup/config/compliance_speed_gui.config'))
    required_sources.add('src/humbleble_ws/cooperative_transport_bringup/launch/compliance_cp12_display_scene.launch.py')
    required_sources.add('tools/cp12_display_scene.py')
    required_sources.update(str(DISPLAY_SCENE/name) for name in SCENE_FILES)
    if not required_sources <= manifest.get('source_sha256', {}).keys():
        raise ValueError('qualification does not cover current control/launch sources')
    for relative, expected in manifest['source_sha256'].items():
        if hashlib.sha256((root/relative).read_bytes()).hexdigest() != expected:
            raise ValueError('source differs from qualified experiment: '+relative)
    for relative, expected in manifest['evidence_sha256'].items():
        if hashlib.sha256((root/relative).read_bytes()).hexdigest() != expected:
            raise ValueError('qualification evidence changed: '+relative)
    normal = json.loads((root/manifest['normal_trial']/'result.json').read_text())
    fault = json.loads((root/manifest['fault_trial']/'result.json').read_text())
    required_evidence = {str(Path(manifest[key])/name)
        for key in ('normal_trial', 'fault_trial')
        for name in ('result.json', 'control_source_hashes.json', 'trial_source_hashes.json',
                     'control_policy.json', 'leader_policy.json', 'original_profiles_restored.json', 'command.json')}
    if not required_evidence <= manifest.get('evidence_sha256', {}).keys():
        raise ValueError('qualification does not cover normal/fault evidence')
    current_control = {p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in control.glob('*.py')}
    for key in ('normal_trial', 'fault_trial'):
        folder = root/manifest[key]
        policy = json.loads((folder/'control_policy.json').read_text())
        if (policy.get('base_force_acceleration_gains') != [.05, 0., 0.] or policy.get('base_force_filter_hz') != .1
                or policy.get('base_integral_gains') != [0., 0., 0.] or policy.get('base_stationary_hold') is not True
                or policy.get('arm_command_horizon_s') != .02
                or policy.get('allocation_on_wrench') is not True
                or policy.get('allocation_use_clock') is not True):
            raise ValueError('own-force sustained velocity memory qualification required')
        if json.loads((folder/'command.json').read_text()).get('RMW_FASTRTPS_PUBLICATION_MODE') != 'ASYNCHRONOUS':
            raise ValueError('runtime communication mode differs from qualified mode')
        if json.loads((folder/'control_source_hashes.json').read_text()) != current_control:
            raise ValueError('runtime control sources differ from current sources')
        for name, expected in json.loads((folder/'trial_source_hashes.json').read_text()).items():
            if hashlib.sha256(Path(name).read_bytes()).hexdigest() != expected:
                raise ValueError('runtime trial source changed: '+name)
        if not all(json.loads((folder/'original_profiles_restored.json').read_text()).values()):
            raise ValueError('ordinary profiles were not restored')
    if not all(r.get('passed') is True and r.get('both_arm_base_inactive') is True
               and r.get('follower_input_boundary_passed') is True and r.get('speed_m_s') == .1
               for r in (normal,fault)):
        raise ValueError('normal and cruise-fault transport at 0.1 m/s must both pass')
    if not normal.get('cooperative_contact_transport_verified') or not fault.get('fault_injected'):
        raise ValueError('native carry and moving fault-stop proof required')
    if normal.get('distance_m') != 3. or fault.get('fault_test') != 'nonfinite':
        raise ValueError('3 m normal completion and nonfinite cruise fault required')
    if not all(r.get('grip_maintenance_enabled') is True for r in (normal, fault)):
        raise ValueError('both normal and fault evidence must enable local grip maintenance')
    latency = fault.get('dual_zero_latency_wall_s', math.inf)
    if fault.get('no_automatic_restart') is not True or not math.isfinite(latency) or not 0 <= latency <= .4:
        raise ValueError('bounded fault stop and no automatic restart required')
    if not all(abs(fault['twists_at_injection'][r][0]) >= .0999 for r in ('leader','follower')):
        raise ValueError('fault was not injected at 0.1 m/s cruise')
    if not all(abs(fault.get('measured_encoder_vx_at_injection', {}).get(r, 0.)) >= .0999
               for r in ('leader', 'follower')):
        raise ValueError('both own encoders must confirm 0.1 m/s cruise')
    if not all(fault.get('post_fault_controller_states', {}).get(r, {}).get(c) == 'inactive'
               for r in ('leader', 'follower') for c in ('arm_controller', 'mecanum_drive_controller')):
        raise ValueError('both robots must deactivate arm/base before cleanup')
    for robot in ('leader', 'follower'):
        stopped = fault.get('maintenance_at_dual_stop', {}).get(robot, {})
        recovered = fault.get('maintenance_after_fault_recovery', {}).get(robot, {})
        if ('sent_goals' not in stopped or recovered.get('state') != 'FAULT'
                or recovered.get('sent_goals') != stopped['sent_goals']):
            raise ValueError('grip maintenance must not resume after fault')
    for key, limit in (('final_arm_offset_xy_m', .0001), ('final_follower_speed_m_s', .00001)):
        measured = normal.get(key, math.inf)
        if not math.isfinite(measured) or not 0 <= measured <= limit:
            raise ValueError('normal trial did not satisfy original settling limits')
    tracks = normal.get('independent_speed_evaluation', {})
    if set(tracks) != {'leader', 'follower', 'cooperative_payload'} or not all(
            .098 <= v.get('steady_gt_mean_m_s', 0.) <= .102 for v in tracks.values()):
        raise ValueError('independent GT must confirm all three bodies at 0.1 m/s')
    return manifest


def completed(event, context, *, output):
    if context.is_shutdown:
        return []
    try:
        result = json.loads((output/'result.json').read_text())
    except (OSError,ValueError) as exc:
        raise RuntimeError('CP-12 result unavailable: '+str(output)) from exc
    if event.returncode != 0 or result.get('passed') is not True:
        raise RuntimeError('CP-12 FAILED; inspect '+str(output/'result.json'))
    return [LogInfo(msg='CP-12 PASSED; stopped. Result: '+str(output/'result.json')),
            EmitEvent(event=Shutdown(reason='CP-12 complete'))]


def setup(context):
    value = lambda key: LaunchConfiguration(key).perform(context)
    root = Path(value('workspace_directory')).expanduser().resolve()
    speed, distance = float(value('translation_velocity_m_s')),float(value('forward_distance_m'))
    if not math.isfinite(speed) or not .01 <= speed <= .1:
        raise ValueError('experimental speed must be between 0.01 and 0.1 m/s')
    minimum = speed*speed/.005+2*speed
    if not math.isfinite(distance) or not minimum <= distance <= 3.:
        raise ValueError(f'forward_distance_m must be {minimum:g}..3 m for this speed and measured acceleration')
    if float(value('rotation_angle_deg')) != 0. or float(value('world_y_distance_m')) != 0.:
        raise ValueError('CP-12 supports forward straight motion only')
    for unsupported in ('translation_acceleration_m_s2','angular_velocity_deg_s','angular_acceleration_deg_s2'):
        if unsupported in context.launch_configurations:
            raise ValueError('CP-12 qualified acceleration is fixed: '+unsupported)
    domain = int(value('ros_domain_id'))
    if not 0 <= domain <= 232:
        raise ValueError('invalid ROS domain')
    manifest = qualification(root)
    if speed != manifest['speed_m_s'] or distance != manifest['distance_m']:
        raise ValueError('current qualification covers exactly 3 m at 0.1 m/s')
    output = (Path(value('output_directory')).expanduser().resolve() if value('output_directory')
              else root/STAGE/('launch_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')))
    if output.exists():
        raise ValueError('output directory already exists: '+str(output))
    partition = 'amir_cp9_cp12_launch_'+uuid.uuid4().hex[:12]
    env = {'ROS_DOMAIN_ID':str(domain),'ROS_LOCALHOST_ONLY':'1','IGN_PARTITION':partition,
           'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1','MKL_NUM_THREADS':'1','PYTHONUNBUFFERED':'1',
           'RMW_FASTRTPS_PUBLICATION_MODE':'ASYNCHRONOUS'}
    trial = ExecuteProcess(name='cp12_contact_speed_trial',cwd=str(root),additional_env=env,output='screen',
        sigterm_timeout='120',sigkill_timeout='10',cmd=['/usr/bin/python3',str(root/'tools/amir_cp9_transport_trial.py'),
            '--run-directory',str(root/DISPLAY_SCENE),'--metrology-directory',str(root/PRIOR/'metrology_02'),
            '--output',str(output),'--distance',str(distance),'--speed',str(speed),
            '--experimental-speed-profile','--grip-maintenance','--display-scene',
            '--base-speed-evidence',str(root/manifest['base_speed_evidence'])])
    actions = [LogInfo(msg=f'CP-12 simulation-only: {distance:g} m, maximum {speed:g} m/s; '
                           f'planned {distance/speed+speed/.005:g} simulated seconds plus preparation/settling.'),
               LogInfo(msg='CP-12 output_directory: '+str(output)),
               LogInfo(msg=f'ROS_DOMAIN_ID={domain}, IGN_PARTITION={partition}'),
               RegisterEventHandler(OnProcessExit(target_action=trial,
                   on_exit=lambda event,ctx:completed(event,ctx,output=output)))]
    if value('headless') == 'false':
        gui_config = Path(get_package_share_directory('cooperative_transport_bringup'))/'config/compliance_speed_gui.config'
        if not gui_config.is_file():
            raise ValueError('rebuild cooperative_transport_bringup: speed GUI config missing')
        gui_env = dict(env)
        if value('gui_nvidia_offload') == 'true':
            gui_env.update(__NV_PRIME_RENDER_OFFLOAD='1', __GLX_VENDOR_LIBRARY_NAME='nvidia')
        actions.append(ExecuteProcess(name='cp12_gazebo_gui',cwd=str(root),additional_env=gui_env,output='screen',
            sigterm_timeout='5',sigkill_timeout='5',cmd=['ign','gazebo','-g','--gui-config',str(gui_config),
                                                       '--force-version','6']))
    actions.append(trial)
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('workspace_directory',default_value=workspace()),
        DeclareLaunchArgument('headless',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('gui_nvidia_offload',default_value='true',choices=['true','false'],
            description='Use the verified local NVIDIA GPU for GUI rendering only.'),
        DeclareLaunchArgument('forward_distance_m',default_value='3.0'),
        DeclareLaunchArgument('translation_velocity_m_s',default_value='0.1'),
        DeclareLaunchArgument('rotation_angle_deg',default_value='0.0'),
        DeclareLaunchArgument('world_y_distance_m',default_value='0.0'),
        DeclareLaunchArgument('record_data',default_value='true',choices=['true']),
        DeclareLaunchArgument('output_directory',default_value=''),
        DeclareLaunchArgument('ros_domain_id',default_value='93'),
        OpaqueFunction(function=setup)])

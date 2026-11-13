"""Native grasp + ARM_ONLY stroke; optional micro and sealed legacy paths."""
import importlib.util
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction


def workspace():
    for root in Path(__file__).resolve().parents:
        if (root/'tools/round_trip_runtime.py').is_file():
            return str(root)
    raise ValueError('Workspace containing tools/round_trip_runtime.py not found')


def setup(context):
    root = Path(context.launch_configurations['workspace_directory']).expanduser().resolve()
    mode = context.launch_configurations['control_mode']
    if context.launch_configurations.get('enable_base_assist', 'false') != 'false':
        raise ValueError('Phase 1 requires enable_base_assist=false; Phase 2 is not qualified')
    if context.launch_configurations.get('wrench_source_mode', 'sensor') != 'sensor':
        raise ValueError('Phase 1 requires the audited Gazebo F/T sensor; proxy/injection modes are not enabled')
    if mode not in ('ARM_ONLY', 'WHOLE_BODY'):
        raise ValueError('control_mode must be ARM_ONLY or WHOLE_BODY')
    if mode == 'WHOLE_BODY':
        # Reject unsupported motion before loading the archived evidence.
        if (float(context.launch_configurations['forward_distance_m']) != 1.
                or float(context.launch_configurations['translation_velocity_m_s']) != .1):
            raise ValueError('legacy WHOLE_BODY supports exactly 1 m out / 1 m back at 0.1 m/s')
        for key in ('rotation_angle_deg', 'world_y_distance_m', 'translation_acceleration_m_s2',
                    'angular_velocity_deg_s', 'angular_acceleration_deg_s2', 'return_distance_m'):
            if key in context.launch_configurations:
                raise ValueError('legacy round-trip route and acceleration are fixed: '+key)
    trial=context.launch_configurations.get('arm_only_trial','micro')
    if trial not in ('micro','native_stroke','dual_arm'):raise ValueError('unsupported ARM_ONLY trial')
    if trial=='dual_arm' and context.launch_configurations.get('arm_only_experiment.enabled','true')=='false':
        trial='native_stroke'
    path = root/'tools'/('dual_arm_only_runtime.py' if mode=='ARM_ONLY' and trial=='dual_arm'
                        else 'native_arm_stroke_runtime.py' if mode=='ARM_ONLY' and trial=='native_stroke'
                        else 'arm_only_micro_runtime.py' if mode == 'ARM_ONLY' else 'round_trip_runtime.py')
    # Experiment tools stay in the workspace, not the controller import graph.
    import sys
    if str(root/'tools') not in sys.path:sys.path.insert(0,str(root/'tools'))
    spec = importlib.util.spec_from_file_location('cp13_runtime', path)
    runtime = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runtime)
    return runtime.setup(context)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('workspace_directory', default_value=workspace()),
        DeclareLaunchArgument('headless', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('gui_nvidia_offload', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('control_mode', default_value='ARM_ONLY', choices=['ARM_ONLY', 'WHOLE_BODY']),
        DeclareLaunchArgument('enable_base_assist', default_value='false', choices=['false', 'true'],
            description='Phase 1: must remain false; Phase 2 base assistance is not qualified'),
        DeclareLaunchArgument('wrench_source_mode', default_value='sensor',
            description='Audited Gazebo F/T sensor only; no target proxy or synthetic force'),
        DeclareLaunchArgument('arm_only_trial', default_value='dual_arm',choices=['micro','native_stroke','dual_arm']),
        DeclareLaunchArgument('arm_only_experiment.enabled',default_value='true',choices=['true','false']),
        DeclareLaunchArgument('dual_arm_distance_m',default_value='0.213'),
        DeclareLaunchArgument('dual_arm_speed_m_s',default_value='0.003'),
        DeclareLaunchArgument('dual_arm_admittance_scale',default_value='0.25',
            description='Simulation impedance comparison: scales virtual M and D together, not actuator or safety limits'),
        DeclareLaunchArgument('leader_base_distance_m',default_value='0.5'),
        DeclareLaunchArgument('calibrate_leader_distance',default_value='true',choices=['true','false'],
            description='Native 0.005m/s only: measured offline leader raw-odom distance correction'),
        DeclareLaunchArgument('native_cpu_policy',default_value='smt',choices=['smt','inherit'],
            description='Native runner/children only: observed SMT cores, or inherited CPU affinity'),
        DeclareLaunchArgument('arm_only_speed_m_s',default_value='0.005'),
        DeclareLaunchArgument('arm_only_direction', default_value='forward', choices=['forward', 'backward']),
        DeclareLaunchArgument('arm_only_distance_m', default_value='0.0093',
            description='ARM_ONLY micro distance; maximum 0.01 m'),
        DeclareLaunchArgument('backward_distance_m', default_value='',
            description='Previous backward test distance; requires arm_only_direction:=backward'),
        DeclareLaunchArgument('keep_gui_open', default_value='true', choices=['true', 'false'],
            description='After verified stop/hold, keep GUI visible until Ctrl+C'),
        DeclareLaunchArgument('gui_review_mode',default_value='paused',choices=['paused','live'],
            description='Native only: pause after verified stop for review, or keep live physics/hold monitoring'),
        DeclareLaunchArgument('start_delay_s', default_value='10.0',
            description='Wall seconds of stationary observation before the one leader goal'),
        DeclareLaunchArgument('forward_distance_m', default_value='1.0'),
        DeclareLaunchArgument('translation_velocity_m_s', default_value='0.005'),
        DeclareLaunchArgument('record_data', default_value='true', choices=['true']),
        DeclareLaunchArgument('output_directory', default_value=''),
        DeclareLaunchArgument('ros_domain_id', default_value='93'),
        OpaqueFunction(function=setup)])

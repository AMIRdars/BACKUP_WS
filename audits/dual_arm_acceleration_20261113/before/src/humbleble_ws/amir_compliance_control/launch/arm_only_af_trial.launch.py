"""Run the requested fixed-grasp AF-2 trial: 3 mm/s, 9 mm, M/D/K=40/80/5."""
from datetime import datetime
from pathlib import Path
import sys
from uuid import uuid4

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration


def default_workspace():
    # Resolve the installed symlink; do not depend on the caller's directory.
    for folder in Path(__file__).resolve().parents:
        if (folder/'tools/amir_af_trial.py').is_file():
            return str(folder)
    return str(Path.cwd())


def trial_exit(event, context):
    if context.is_shutdown:
        return []
    if event.returncode != 0:
        raise RuntimeError(f'AF-2 trial failed (exit {event.returncode}); inspect output_directory/result.json')
    return [EmitEvent(event=Shutdown(reason='AF-2 completed; robots stopped and experiment cleaned up'))]


def setup(context):
    value=lambda key: LaunchConfiguration(key).perform(context)
    workspace=Path(value('workspace')).expanduser().resolve()
    runner=workspace/'tools/amir_af_trial.py'
    response=workspace/'audits/arm_only_speed_20261109/servo_response.json'
    for required in (runner, workspace/'tools/amir_af_local_trace.py', response):
        if not required.is_file():
            raise ValueError('AF trial dependency missing: '+str(required))
    domain=value('ros_domain_id')
    if not domain.isdecimal() or not 0<=int(domain)<=232:
        raise ValueError('ros_domain_id must be an integer from 0 to 232')
    gui=value('gui')
    if gui not in ('true','false'):
        raise ValueError('gui must be true or false')
    tag=datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid4().hex[:8]
    output=Path(value('output_directory')).expanduser() if value('output_directory') else Path('audits/arm_only_rerun_'+tag)
    output=(output if output.is_absolute() else workspace/output).resolve()
    if output.exists():
        raise ValueError('output_directory already exists; choose a new directory: '+str(output))
    # Override only the trial child's environment. Giving the parent launch
    # this same partition would let the runner's owned-process cleanup match it.
    environment={'ROS_DOMAIN_ID':str(int(domain)), 'ROS_LOCALHOST_ONLY':'1',
        'IGN_PARTITION':'amir_af_launch_'+tag,
        'OPENBLAS_NUM_THREADS':'1', 'OMP_NUM_THREADS':'1'}
    process=ExecuteProcess(name='arm_only_af_trial', cwd=str(workspace),
        cmd=[sys.executable, '-u', str(runner),
            '--stage','AF-2','--profile','pose_hold_micro',
            '--virtual-mass-kg','40','--virtual-damping-ns-m','80','--virtual-stiffness-n-m','5',
            '--arm-command-reference','servo_trajectory','--arm-horizon','.08',
            '--control-on-wrench','--wrench-on-measurement','--wrench-cutoff-hz','8',
            '--servo-response',str(response),'--leader-reference-acceleration','.0012',
            '--distance','.009','--speed','.003','--require-speed-fraction','.98',
            '--output',str(output)]+(['--gui'] if gui=='true' else []),
        additional_env=environment, output='screen',
        # The runner owns several process groups and verifies stop/hold before
        # tearing them down; allow that cleanup on Ctrl+C rather than killing it.
        sigterm_timeout='120', sigkill_timeout='20')
    return [LogInfo(msg='ARM_ONLY AF-2: fixed grasp, 3 mm/s, 9 mm; GUI='+gui+'; output: '+str(output)),
        RegisterEventHandler(OnProcessExit(target_action=process, on_exit=trial_exit)), process]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('workspace', default_value=default_workspace(),
            description='Existing workspace containing tools and measured audit artifacts'),
        DeclareLaunchArgument('ros_domain_id', default_value='70',
            description='Local ROS domain for this isolated trial'),
        DeclareLaunchArgument('gui', default_value='false', choices=['true','false'],
            description='Show the Gazebo GUI; the server, control and recording stay the same'),
        DeclareLaunchArgument('output_directory', default_value='',
            description='New output directory; default audits/arm_only_rerun_<time>_<unique id>'),
        OpaqueFunction(function=setup)])

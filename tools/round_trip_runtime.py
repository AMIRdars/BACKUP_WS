"""Select the byte-identical CP-13 overlay, keeping newer rotation work intact."""
import hashlib
import importlib.util
import json
import os
from datetime import datetime
from pathlib import Path

from launch.actions import ExecuteProcess, LogInfo, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration


RUNTIME = Path('.simulation_runtimes/cp13_round_trip')
AUDIT = Path('audits/round_trip_command_restore_20261105')
ENTRY = Path('src/humbleble_ws/cooperative_transport_bringup/launch/compliance_round_trip.launch.py')


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load(path):
    spec = importlib.util.spec_from_file_location('sealed_cp13_entry', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def validate(root):
    root = Path(root).resolve()
    runtime = root/RUNTIME
    record = json.loads((root/AUDIT/'recovery.json').read_text())
    for relative, expected in record['runtime_sha256'].items():
        path = runtime/relative
        if not path.is_file() or sha(path) != expected:
            raise ValueError('CP-13 isolated runtime changed: '+str(path))
    # The historical release checks run unchanged, against their original bytes.
    entry = load(runtime/ENTRY)
    release, parent = entry.qualification(runtime)
    for manifest in (release, parent):
        # URDF plugin parameters and shared simulator assets retain historical
        # absolute paths. Check those live dependencies as well as the archive.
        for relative, expected in manifest['source_sha256'].items():
            if relative.startswith('src/humbleble_ws/amir_compliance_control/') or relative == str(ENTRY):
                continue
            path = root/relative
            if not path.is_file() or sha(path) != expected:
                raise ValueError('CP-13 shared dependency changed: '+str(path))
        for trial in ('normal_trial', 'fault_trial'):
            sources = json.loads((runtime/manifest[trial]/'trial_source_hashes.json').read_text())
            for absolute, expected in sources.items():
                path = runtime/Path(absolute).relative_to(root)
                if not path.is_file() or sha(path) != expected:
                    raise ValueError('CP-13 runtime preparation source changed: '+str(path))
    installed = json.loads((root/AUDIT/'installed_runtime.json').read_text())
    for relative, expected in installed.items():
        path = runtime/relative
        if not path.is_file() or sha(path) != expected:
            raise ValueError('CP-13 installed overlay changed: '+str(path))
    control = runtime/'install/amir_compliance_control/lib/python3.10/site-packages/amir_compliance_control'
    source = runtime/'src/humbleble_ws/amir_compliance_control/amir_compliance_control'
    if {p.name: sha(p) for p in control.glob('*.py')} != {p.name: sha(p) for p in source.glob('*.py')}:
        raise ValueError('CP-13 installed controller differs from qualified source')
    for name in ('compliance_round_trip_scene.launch.py', 'compliance_round_trip_safety.launch.py'):
        path = runtime/'install/cooperative_transport_bringup/share/cooperative_transport_bringup/launch'/name
        if path.resolve() != runtime/'src/humbleble_ws/cooperative_transport_bringup/launch'/name:
            raise ValueError('CP-13 launch requires symlink installation: '+str(path))
    return entry, runtime


def environment(runtime, existing):
    prefix = runtime/'install'
    control = prefix/'amir_compliance_control'
    bringup = prefix/'cooperative_transport_bringup'
    first = {'AMENT_PREFIX_PATH': [bringup, control],
             'PYTHONPATH': [control/'lib/python3.10/site-packages'],
             'PATH': [control/'bin', bringup/'bin']}
    return {key: os.pathsep.join([*(str(p) for p in paths), existing.get(key, '')]).rstrip(os.pathsep)
            for key, paths in first.items()}


def cpu_groups():
    """Use measured CPU maximum frequencies, respecting the caller's cpuset."""
    frequencies = {}
    for cpu in os.sched_getaffinity(0):
        path = Path(f'/sys/devices/system/cpu/cpu{cpu}/cpufreq/cpuinfo_max_freq')
        if path.is_file():
            frequencies[cpu] = int(path.read_text())
    if not frequencies or min(frequencies.values()) == max(frequencies.values()):
        return None
    cutoff = (min(frequencies.values())+max(frequencies.values()))/2
    fast = {cpu for cpu, frequency in frequencies.items() if frequency > cutoff}
    background = set(frequencies)-fast
    return (fast, background) if fast and background else None


def assign_cpu(pid, cpus):
    # Set all threads already created; subsequent threads/children inherit this.
    for task in Path(f'/proc/{pid}/task').iterdir():
        try:
            os.sched_setaffinity(int(task.name), cpus)
        except ProcessLookupError:
            pass


def cpu_handler(action, cpus, label):
    def started(event, context):
        assign_cpu(event.pid, cpus)
        return [LogInfo(msg=f'CP-13 {label} CPU affinity: {sorted(cpus)}')]
    return RegisterEventHandler(OnProcessStart(target_action=action, on_start=started))


def setup(context):
    value = lambda key: LaunchConfiguration(key).perform(context)
    root = Path(value('workspace_directory')).expanduser().resolve()
    entry, runtime = validate(root)
    # Delegate argument, route, sensor, safety and result checks to the sealed entry.
    original_workspace = context.launch_configurations['workspace_directory']
    original_output = context.launch_configurations.get('output_directory', '')
    context.launch_configurations['workspace_directory'] = str(runtime)
    if not value('output_directory'):
        context.launch_configurations['output_directory'] = str(
            root/entry.STAGE/('launch_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')))
    try:
        actions = entry.setup(context)
    finally:
        context.launch_configurations['workspace_directory'] = original_workspace
        context.launch_configurations['output_directory'] = original_output
    env = environment(runtime, context.environment)
    cpu_actions = []
    groups = cpu_groups()
    if groups:
        fast, background = groups
        for action in actions:
            if isinstance(action, ExecuteProcess):
                argv = [sub.perform(context) for sub in action.cmd[0]]
                is_gui = ''.join(argv) == 'ign'
                cpu_actions.append(cpu_handler(action, background if is_gui else fast,
                                               'GUI' if is_gui else 'simulation/control'))
    return [LogInfo(msg='Using qualified CP-13 isolated runtime: '+str(runtime)),
            *(SetEnvironmentVariable(key, val) for key, val in env.items()), *cpu_actions, *actions]

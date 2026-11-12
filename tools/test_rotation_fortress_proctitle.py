import importlib.util
from pathlib import Path

PATH=Path(__file__).resolve().parents[1]/'audits/rotation_native_enable_20261106/watch_physics_affinity.py'
spec=importlib.util.spec_from_file_location('fortress_affinity',PATH)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def test_actual_fortress_proctitle_and_non_simulator_commands():
    assert module.physics(['ign gazebo -s -r -v 2 /ws/case/world.sdf   '],'ruby')
    assert module.physics(['ruby','/usr/bin/ign','gazebo','-s'],'ruby')
    assert not module.physics(['ign gazebo --version'],'ruby')
    assert not module.physics(['/bin/sh','-c','ruby /usr/bin/ign gazebo world.sdf'],'sh')
    assert not module.physics(['ros2','launch','a.launch.py'],'ros2')

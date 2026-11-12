"""Prove the requested step survives the same generator used at launch."""
import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.utilities import perform_substitutions
import pytest

from amir_rotation_native_fixture import prepare
from amir_rotation_native_step_fixture import refine

ROOT=Path(__file__).resolve().parents[1]
SOURCE=ROOT/'audits/object_center_rotation_20261103/cp8r3'


def module(name):
    path=Path(get_package_share_directory('cooperative_transport_bringup'))/'launch'/name
    spec=importlib.util.spec_from_file_location(name,path)
    value=importlib.util.module_from_spec(spec);spec.loader.exec_module(value)
    return value


@pytest.mark.parametrize('divisor',[1,2,4])
def test_generated_fixture_remains_byte_identical_after_scene_regeneration(tmp_path,divisor):
    output=tmp_path/'fixture';prepare(output,SOURCE/'native_generated',SOURCE/'native_metrology')
    if divisor!=1:refine(output,divisor)
    names=['world.sdf',*[r+s for r in ('leader','follower') for s in ('.urdf','_controllers.yaml','_base_controller.yaml')]]
    before={name:(output/name).read_bytes() for name in names}
    native=module('compliance_cp9_contact.launch.py');context=LaunchContext()
    context.launch_configurations.update(run_directory=str(output),physics_step_divisor=str(divisor))
    for entity in native.generate_launch_description().entities:
        if isinstance(entity,DeclareLaunchArgument) and entity.name not in context.launch_configurations:
            context.launch_configurations[entity.name]=perform_substitutions(context,entity.default_value)
    native.setup(context)
    assert before=={name:(output/name).read_bytes() for name in names}


def test_guarded_scene_forwards_only_explicit_optional_step_with_default_one():
    actions=module('compliance_cp9_guarded_scene.launch.py').generate_launch_description().entities
    arg=next(a for a in actions if isinstance(a,DeclareLaunchArgument) and a.name=='physics_step_divisor')
    assert perform_substitutions(LaunchContext(),arg.default_value)=='1'
    context=LaunchContext();context.launch_configurations['physics_step_divisor']='2'
    include=next(a for a in actions if isinstance(a,IncludeLaunchDescription))
    value=dict(include.launch_arguments)['physics_step_divisor']
    assert value.perform(context)=='2'

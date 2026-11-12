"""Opt-in native contact fixture; no weld, grasp assistance or autonomy.

Reuse CP-1 generation and the existing contact world's DART solver policy.
Only derived run files are written. Original model/controller defaults remain.
"""
from copy import deepcopy
import importlib.util
from pathlib import Path
import json
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.utilities import perform_substitutions, normalize_to_list_of_substitutions


def setup(context):
    path=Path(share('cooperative_transport_bringup'))/'launch/compliance_cp1.launch.py'
    spec=importlib.util.spec_from_file_location('cp1_contact_generator',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    actions=module.setup(context)
    output=Path(LaunchConfiguration('run_directory').perform(context)).resolve()
    initialization={}
    for robot in ('leader','follower'):
        path=output/(robot+'.urdf');urdf=ET.parse(path);model=urdf.getroot();initialization[robot]={}
        for joint in model.findall('ros2_control/joint'):
            mimic=joint.find("param[@name='mimic']")
            if mimic is None:continue
            relation=model.find("joint[@name='"+joint.get('name')+"']/mimic")
            if relation is None or relation.get('joint')!=mimic.text:raise ValueError('inconsistent mimic model')
            master=model.find("ros2_control/joint[@name='"+mimic.text+"']/state_interface[@name='position']/param[@name='initial_value']")
            if master is None:raise ValueError('missing measured-model master initial position')
            value=float(master.text)*float(relation.get('multiplier','1'))+float(relation.get('offset','0'))
            state=joint.find("state_interface[@name='position']")
            param=state.find("param[@name='initial_value']")
            if param is None:param=ET.SubElement(state,'param',name='initial_value')
            param.text=str(value);initialization[robot][joint.get('name')]={'master':mimic.text,'initial_position_rad':value}
        urdf.write(path,encoding='unicode',xml_declaration=True)
    (output/'mimic_initialization.json').write_text(json.dumps(initialization,indent=2)+'\n')
    # The original friction launch approaches by 0.110 m before closing.
    # Record the target fixture placement. The evaluator moves the simulated
    # models there only AFTER controllers/home are ready; inserting fingers
    # into the payload before controller activation can jam a passive mimic.
    staging={}
    for timer in actions:
        if not isinstance(timer,TimerAction):continue
        for index,include in enumerate(timer.actions):
            if not isinstance(include,IncludeLaunchDescription):continue
            args=dict(include.launch_arguments)
            robot=perform_substitutions(context,normalize_to_list_of_substitutions(args['namespace']))
            old_x=float(perform_substitutions(context,normalize_to_list_of_substitutions(args['x'])))
            new_x=old_x+(.110 if robot=='leader' else -.110)
            staging[robot]={'original_x_m':old_x,'initial_approach_m':.110,'staged_x_m':new_x}
    if set(staging)!={'leader','follower'}:raise ValueError('expected two original robot spawn actions')
    (output/'scene_staging.json').write_text(json.dumps({'robots':staging,
        'policy_source':'cooperative_transport_bringup/launch/friction_simulation.launch.py: approach_distance'},indent=2)+'\n')
    world=ET.parse(output/'world.sdf')
    source=ET.parse(Path(share('cooperative_transport_gazebo'))/'worlds/cooperative_transport_contact.sdf')
    policy=source.find("world/plugin[@name='cooperative_transport::DartContactParameters']")
    if policy is None:raise ValueError('existing native contact solver policy missing')
    world.getroot().find('world').append(deepcopy(policy))
    # Match the existing supported contact grasp, rather than CP-1's
    # deliberately separated/support-only payload staging at z=.660 m.
    root=world.getroot().find('world')
    physics=root.find('physics');root.remove(physics)
    root.append(deepcopy(source.find('world/physics')))
    # Numerical-resolution diagnostics are explicit; the default is byte-
    # identical to the existing native contact fixture.
    divisor=int(LaunchConfiguration('physics_step_divisor',default='1').perform(context))
    if divisor not in (1,2,4):raise ValueError('native physics step divisor must be 1, 2 or 4')
    if divisor!=1:
        step=root.find('physics/max_step_size')
        value=float(step.text)
        if not 0.<value<float('inf'):raise ValueError('finite positive native physics step required')
        step.text=repr(value/divisor)
    support=root.find("model[@name='grasp_support']");root.remove(support)
    root.append(deepcopy(source.find("world/model[@name='grasp_support']")))
    payload=root.find("model[@name='cooperative_payload']/pose")
    pose=payload.text.split();pose[2]='0.650';payload.text=' '.join(pose)
    staging_info=json.loads((output/'scene_staging.json').read_text())
    staging_info.update(payload_z_m=.650,support_source='cooperative_transport_contact.sdf',
        payload_pose_source='friction_simulation.launch.py: spawn_payload')
    (output/'scene_staging.json').write_text(json.dumps(staging_info,indent=2)+'\n')
    world.write(output/'world.sdf',encoding='unicode',xml_declaration=True)
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('run_directory'),
        DeclareLaunchArgument('physics_step_divisor',default_value='1',choices=['1','2','4']),
        DeclareLaunchArgument('headless',default_value='true',choices=['true']),
        DeclareLaunchArgument('constraint_fixture',default_value='false',choices=['false']),
        DeclareLaunchArgument('calibration_fixture',default_value='false',choices=['false']),
        DeclareLaunchArgument('force_torque_system',default_value='true',choices=['true']),
        DeclareLaunchArgument('joint_load_sensors',default_value='true',choices=['true']),
        DeclareLaunchArgument('weld_cfm',default_value='0',choices=['0']),
        OpaqueFunction(function=setup)])

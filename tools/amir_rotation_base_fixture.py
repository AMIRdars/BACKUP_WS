"""Isolated ungrasped metrology fixture; default launch files stay unchanged."""
import argparse
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions
import yaml

from amir_rotation_integration_audit import canonical_model


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def prepare(output, reference, candidate=None):
    output, reference = output.resolve(), reference.resolve()
    output.mkdir(parents=True, exist_ok=False)
    path = Path(get_package_share_directory('cooperative_transport_bringup'))/'launch/compliance_cp1.launch.py'
    spec = importlib.util.spec_from_file_location('rotation_base_fixture', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            context.launch_configurations[entity.name] = perform_substitutions(context, entity.default_value)
    context.launch_configurations.update(run_directory=str(output), headless='true',
        force_torque_system='true', joint_load_sensors='true', calibration_fixture='false',
        constraint_fixture='false', weld_cfm='0')
    actions = module.setup(context)  # File generation only until caller launches.
    proof = dict(ungrasped_only=True, transport_qualified=False, candidate_applied=candidate is not None,
                 generator_sha256=sha(path), robots={})
    proposal = json.loads((candidate/'proof.json').read_text()) if candidate else None
    for robot in ('leader', 'follower'):
        urdf, baseline = output/(robot+'.urdf'), reference/(robot+'.urdf')
        if canonical_model(urdf) != canonical_model(baseline):
            raise ValueError('reference robot differs beyond generated controller path')
        root = ET.parse(urdf)
        plugins = [e.text for e in root.findall('ros2_control/hardware/plugin')]
        if not plugins or set(plugins) != {'gz_ros2_control/GazeboSimSystem'}:
            raise ValueError('verified Fortress ros2_control plugin required')
        config = output/(robot+'_base_controller.yaml')
        if config.read_bytes() != (reference/config.name).read_bytes():
            raise ValueError('reference base controller differs from default generated controller')
        if (output/(robot+'_controllers.yaml')).read_bytes() != (reference/(robot+'_controllers.yaml')).read_bytes():
            raise ValueError('reference arm/controller interfaces changed')
        before = sha(config)
        if candidate:
            evidence = proposal['robots'][robot]
            target = candidate/config.name
            if (evidence['source_urdf_sha256'] != sha(baseline)
                    or evidence['source_controller_sha256'] != before
                    or evidence['candidate_controller_sha256'] != sha(target)):
                raise ValueError('candidate/reference hashes mismatch')
            old, new = yaml.safe_load(config.read_text()), yaml.safe_load(target.read_text())
            p = next(iter(new.values()))['ros__parameters']['kinematics']
            lever = p['sum_of_robot_center_projection_on_X_Y_axis']
            if not isinstance(lever, (int, float)) or not math.isfinite(lever) or lever <= 0.:
                raise ValueError('invalid measured-model wheel geometry')
            p['sum_of_robot_center_projection_on_X_Y_axis'] = next(iter(old.values()))['ros__parameters']['kinematics']['sum_of_robot_center_projection_on_X_Y_axis']
            if new != old:
                raise ValueError('candidate changes more than the audited wheel projection sum')
            config.write_bytes(target.read_bytes())
        proof['robots'][robot] = dict(urdf_sha256=sha(urdf), reference_urdf_sha256=sha(baseline),
            original_controller_sha256=before, controller_sha256=sha(config), canonical_model_identical=True)
    allowed = {'Physics', 'UserCommands', 'SceneBroadcaster', 'Contact', 'Sensors', 'ForceTorque'}
    world = ET.parse(output/'world.sdf')
    plugins = [e.get('name', '').rsplit('::', 1)[-1] for e in world.findall('.//plugin')]
    if not set(plugins) <= allowed:
        raise ValueError('ungrasped fixture contains an unreviewed actuator/grasp plugin')
    proof.update(world_sha256=sha(output/'world.sdf'), plugins=plugins)
    (output/'base_fixture_proof.json').write_text(json.dumps(proof, indent=2)+'\n')
    return actions, proof


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--reference', type=Path, required=True)
    parser.add_argument('--candidate', type=Path)
    parser.add_argument('--prepare-only', action='store_true')
    args = parser.parse_args()
    if not args.prepare_only and (os.getenv('ROS_DISTRO') != 'humble'
            or os.getenv('ROS_LOCALHOST_ONLY') != '1' or not os.getenv('ROS_DOMAIN_ID')
            or not os.getenv('IGN_PARTITION', '').startswith('amir_cp8r3_base_metrology_')):
        parser.error('isolated Humble metrology environment required before launching physics')
    actions, proof = prepare(args.output, args.reference, args.candidate)
    print(json.dumps(proof, indent=2), flush=True)
    if args.prepare_only:
        return 0
    service = LaunchService(argv=[])
    service.include_launch_description(LaunchDescription(actions))
    return service.run()


if __name__ == '__main__':
    raise SystemExit(main())

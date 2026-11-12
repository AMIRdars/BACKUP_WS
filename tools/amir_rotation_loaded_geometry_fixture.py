"""Generate, but never launch, a candidate fixed-grasp loaded fixture.

Match the already measured fixed world/URDF/interfaces and change only the
audited wheel projection sum in two local controller YAML files. No old
passive-yaw or grasp-calibration certificate is copied to the new condition.
"""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def canonical_robot(path):
    root = ET.parse(path).getroot()
    for element in root.findall('gazebo/plugin/parameters'):
        element.text = 'CONTROLLER_FILE_PATH'
    return ET.tostring(root)


def controller_candidate(original, candidate, proof, robot, reference):
    """Allow only the already derived lever-arm scalar, without new dimensions."""
    evidence = proof['robots'][robot]
    if (evidence['source_urdf_sha256'] != sha(reference/(robot+'.urdf'))
            or evidence['source_controller_sha256'] != sha(original)
            or evidence['candidate_controller_sha256'] != sha(candidate)):
        raise ValueError('audited geometry candidate/reference hash mismatch')
    before, after = yaml.safe_load(original.read_text()), yaml.safe_load(candidate.read_text())
    key = 'sum_of_robot_center_projection_on_X_Y_axis'
    group = '/**/mecanum_drive_controller'
    old = before[group]['ros__parameters']['kinematics'][key]
    new = after[group]['ros__parameters']['kinematics'][key]
    if old != evidence['old_projection_sum_m'] or new != evidence['candidate_projection_sum_m']:
        raise ValueError('controller value is not the audited URDF-derived dimension')
    after[group]['ros__parameters']['kinematics'][key] = old
    if after != before:
        raise ValueError('geometry candidate changes another controller parameter')
    return dict(old_projection_sum_m=old, candidate_projection_sum_m=new,
                original_controller_sha256=sha(original), candidate_controller_sha256=sha(candidate))


def prepare(output, reference, candidate):
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from launch.utilities import perform_substitutions

    output, reference, candidate = [Path(p).resolve() for p in (output, reference, candidate)]
    output.mkdir(parents=True, exist_ok=False)
    generator = Path(get_package_share_directory('cooperative_transport_bringup'))/'launch/compliance_cp1.launch.py'
    spec = importlib.util.spec_from_file_location('loaded_geometry_fixture', generator)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            context.launch_configurations[action.name] = perform_substitutions(context, action.default_value)
    # Same values as the measured fixed CP6 scene; equality is checked below.
    context.launch_configurations.update(run_directory=str(output), headless='true',
        force_torque_system='true', calibration_fixture='true', constraint_fixture='true',
        weld_cfm='.1', joint_load_sensors='true', grasp_state_publish_period_s='.02')
    module.setup(context)  # Return launch actions, but never execute any of them.
    proof = json.loads((candidate/'proof.json').read_text())
    if proof.get('auto_activated') is not False or proof.get('requires_new_motion_evidence') is not True:
        raise ValueError('unactivated audited candidate proof required')
    if ET.tostring(ET.parse(output/'world.sdf').getroot()) != ET.tostring(ET.parse(reference/'world.sdf').getroot()):
        raise ValueError('loaded fixture world differs from measured fixed-grasp reference')
    report = dict(physics_started=False, loaded_transport_executor_implemented=False,
        ready_to_launch=False, transport_qualified=False, precision_qualified=False,
        yaw_qualification_issued=False, grasp_calibration_transferred=False,
        required_next_evidence=['actual independent baseline/candidate comparison',
            'fresh loaded static grasp transform and initial centre verification',
            'new-condition passive positive/negative and O2 observation-stop qualification'],
        reference=str(reference), candidate=str(candidate), generator_sha256=sha(generator),
        candidate_proof_sha256=sha(candidate/'proof.json'), robots={})
    for robot in ('leader', 'follower'):
        urdf = output/(robot+'.urdf')
        original = output/(robot+'_base_controller.yaml')
        proposed = candidate/original.name
        if canonical_robot(urdf) != canonical_robot(reference/urdf.name):
            raise ValueError('loaded robot model differs beyond controller parameter-file path')
        if sha(original) != sha(reference/original.name):
            raise ValueError('generated baseline controller differs from measured reference')
        interfaces = output/(robot+'_controllers.yaml')
        if interfaces.read_bytes() != (reference/interfaces.name).read_bytes():
            raise ValueError('arm/gripper/ros2_control interfaces changed')
        value = controller_candidate(original, proposed, proof, robot, reference)
        original.write_bytes(proposed.read_bytes())
        value.update(urdf_sha256=sha(urdf), canonical_model_identical=True,
                     arm_gripper_interfaces_identical=True)
        report['robots'][robot] = value
    report['generated_sha256'] = {p.name: sha(p) for p in output.iterdir() if p.is_file()}
    (output/'loaded_geometry_fixture_proof.json').write_text(json.dumps(report, indent=2)+'\n')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--reference', type=Path, required=True)
    parser.add_argument('--candidate', type=Path, required=True)
    parser.add_argument('--prepare-only', action='store_true', required=True)
    args = parser.parse_args()
    print(json.dumps(prepare(args.output, args.reference, args.candidate), indent=2))


if __name__ == '__main__':
    main()

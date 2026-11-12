#!/usr/bin/env python3
"""Offline CP-8R-0: expand the existing fixture without executing launch actions.

Both ideal robot paths are EVALUATION ONLY. Nothing is sent to either robot.
Fixture-derived transforms are nominal design data, not measured calibration.
"""
import argparse
import ast
import csv
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
from launch import LaunchContext
from launch.actions import TimerAction

from amir_compliance_control.object_center_rotation_trajectory import (
    ObjectCenterRotation, initialize_object_pose, inverse_transform,
    rigid_transform, world_velocity_to_body)
from amir_compliance_control.wrench import LocalModel, origin, rotation


ROOT = Path(__file__).resolve().parents[1]
BRINGUP = ROOT/'src/humbleble_ws/cooperative_transport_bringup/launch'


def sdf_pose(element):
    if element is None:
        return np.eye(4)
    if element.attrib:
        raise ValueError('audit requires explicit default-parent SDF poses')
    xyzrpy = np.array(element.text.split(), dtype=float)
    if xyzrpy.shape != (6,) or not np.isfinite(xyzrpy).all():
        raise ValueError('invalid SDF pose')
    return origin(ET.Element('origin', xyz=' '.join(map(str, xyzrpy[:3])),
                             rpy=' '.join(map(str, xyzrpy[3:]))))


def generate_fixture(output):
    """Call the CURRENT source setup, retaining its exact spawn arguments.

    No LaunchService is created, and returned Node/Include actions are never
    executed. Values match compliance_cp6_fixture.launch.py, without its
    calibration-force input or any physics process.
    """
    source = BRINGUP/'compliance_cp1.launch.py'
    spec = importlib.util.spec_from_file_location('cp8r0_existing_fixture', source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update(
        run_directory=str(output.resolve()), headless='true',
        force_torque_system='true', calibration_fixture='false',
        constraint_fixture='true', joint_load_sensors='true', weld_cfm='.1')
    actions = module.setup(context)
    # Resolve the actual bringup y default rather than guessing a spawn axis.
    from ament_index_python.packages import get_package_share_directory as share
    bringup = Path(share('amir_gazebo'))/'launch/robot_bringup.launch.py'
    defaults = []
    for call in ast.walk(ast.parse(bringup.read_text())):
        if (isinstance(call, ast.Call) and isinstance(call.func, ast.Name)
                and call.func.id == 'DeclareLaunchArgument' and call.args
                and isinstance(call.args[0], ast.Constant) and call.args[0].value == 'y'):
            defaults.extend(ast.literal_eval(k.value) for k in call.keywords if k.arg == 'default_value')
    if len(defaults) != 1:
        raise ValueError('cannot resolve existing spawn y default; inspect bringup')
    spawn = {}
    for timer in actions:
        if isinstance(timer, TimerAction):
            for include in timer.actions:
                arguments = dict(include.launch_arguments)
                arguments.setdefault('y', defaults[0])
                spawn[arguments['namespace']] = arguments
    if set(spawn) != {'leader', 'follower'}:
        raise ValueError('cannot resolve both existing spawn configurations')
    return spawn


def fixture_geometry(folder, spawn):
    world = ET.parse(folder/'world.sdf').getroot().find('world')
    payload = world.find("model[@name='cooperative_payload']")
    link = payload.find("link[@name='payload_link']")
    collision = link.find('collision')
    size = np.array(collision.findtext('geometry/box/size').split(), float)
    if size.shape != (3,) or np.any(size <= 0) or np.argmax(size) != 0:
        raise ValueError('audit supports the inspected long-x box only')
    w_p = sdf_pose(payload.find('pose'))
    p_geometry = sdf_pose(link.find('pose')) @ sdf_pose(collision.find('pose'))
    w_geometry = w_p @ p_geometry
    # O +x = long box axis, O +z = up, O +y = z cross x. Preserve P.
    x = w_geometry[:3, 0]
    z = np.array([0., 0., 1.])
    if abs(x @ z) > 1e-9:
        raise ValueError('payload long axis is not horizontal')
    o = np.eye(4)
    o[:3, :3] = np.column_stack((x, np.cross(z, x), z))
    o[:3, 3] = w_geometry[:3, 3]
    o = rigid_transform(o)
    robots = {}
    for robot, args in spawn.items():
        xml = (folder/(robot+'.urdf')).read_text()
        root = ET.fromstring(xml)
        model = LocalModel(xml)
        q = np.array([float(root.find(
            f"ros2_control/joint[@name='{name}']/state_interface[@name='position']"
            "/param[@name='initial_value']").text) for name in model.names])
        tcp_position, b_sensor = model.frames(q)
        tcp_fixed = root.find("joint[@name='tcp_fixed']")
        if (tcp_fixed.find('parent').get('link') != 'gripper_base_1'
                or tcp_fixed.find('child').get('link') != 'tcp_link'):
            raise ValueError('TCP chain changed; inspect before deriving calibration')
        b_tcp = b_sensor @ origin(tcp_fixed.find('origin'))
        np.testing.assert_allclose(b_tcp[:3, 3], tcp_position, atol=1e-12)
        w_b = np.eye(4)
        w_b[:3, 3] = [float(args['x']), float(args['y']), float(args['z'])]
        w_b[:3, :3] = rotation([0, 0, 1], float(args['yaw']))
        w_grasp = w_b @ b_tcp
        w_sensor = w_b @ b_sensor
        anchor_name = 'grasp_r1_link' if robot == 'leader' else 'grasp_r2_link'
        anchor = payload.find(f"link[@name='{anchor_name}']")
        w_anchor = w_p @ sdf_pose(anchor.find('pose'))
        # Calibrations are NOMINAL fixture geometry, including full rotations.
        o_tcp = inverse_transform(o) @ w_grasp
        o_sensor = inverse_transform(o) @ w_sensor
        o_anchor = inverse_transform(o) @ w_anchor
        reconstructed = initialize_object_pose(w_b, b_tcp, o_tcp)
        wrong = initialize_object_pose(w_b, b_sensor, o_anchor)
        robots[robot] = {
            'joint_names': list(model.names), 'initial_q_rad': q.tolist(),
            'joint_lower_rad': model.lower.tolist(), 'joint_upper_rad': model.upper.tolist(),
            'joint_velocity_urdf_rad_s': model.velocity.tolist(),
            'initial_min_joint_margin_rad': float(np.min(np.r_[q-model.lower, model.upper-q])),
            'world_T_base_footprint': w_b.tolist(),
            'base_footprint_T_base_link': origin(root.find("joint[@name='base_joint']/origin")).tolist(),
            'base_footprint_T_tcp': b_tcp.tolist(),
            'base_footprint_T_gripper_base': b_sensor.tolist(),
            'object_T_tcp_nominal': o_tcp.tolist(),
            'object_T_gripper_base_nominal': o_sensor.tolist(),
            'object_T_sdf_anchor': o_anchor.tolist(),
            'world_T_tcp': w_grasp.tolist(),
            'sensor_minus_anchor_xyz_m': (w_sensor[:3, 3]-w_anchor[:3, 3]).tolist(),
            'sensor_anchor_rotation_frobenius': float(np.linalg.norm(w_sensor[:3, :3]-w_anchor[:3, :3])),
            'tcp_sensor_distance_m': float(np.linalg.norm(b_tcp[:3, 3]-b_sensor[:3, 3])),
            'reconstruction_max_error': float(np.max(abs(reconstructed-o))),
            'anchor_as_calibration_center_error_m': float(np.linalg.norm(wrong[:3, 3]-o[:3, 3])),
            'anchor_as_calibration_object_z': wrong[:3, 2].tolist(),
            'wrist_sensor': ET.tostring(root.find("gazebo[@reference='Joint_5']/sensor"), encoding='unicode'),
            'ros2_control_hardware_plugins': [e.text for e in root.findall('ros2_control/hardware/plugin')],
            'gazebo_control_plugins': [dict(e.attrib) for e in root.findall('gazebo/plugin')
                                       if 'ros2_control' in e.get('name', '')],
        }
    masses, weighted = [], []
    for part in payload.findall('link'):
        mass = float(part.findtext('inertial/mass'))
        pose = w_p @ sdf_pose(part.find('pose')) @ sdf_pose(part.find('inertial/pose'))
        masses.append(mass)
        weighted.append(mass*pose[:3, 3])
    return {
        'world_T_object': o.tolist(), 'payload_model_T_object': (inverse_transform(w_p)@o).tolist(),
        'world_T_payload_model': w_p.tolist(), 'box_size_in_payload_axes_m': size.tolist(),
        'object_geometric_center_world_m': o[:3, 3].tolist(),
        'assembly_com_world_m': (np.sum(weighted, axis=0)/sum(masses)).tolist(),
        'assembly_mass_kg': sum(masses), 'robots': robots,
    }


def write_timeseries(path, geometry, angles_deg, duration, count):
    """Analytic ideal references, including the follower as an offline oracle."""
    o = np.array(geometry['world_T_object'])
    leader_b = np.array(geometry['robots']['leader']['world_T_base_footprint'])
    leader_w = inverse_transform(leader_b)
    # W_L is the initial encoder frame. No need to know its world placement
    # in an eventual controller; the world conversion here is evaluation only.
    local_o = leader_w @ o
    maxima = dict(radius_error_m=0., grasp_closure_error=0.,
                  center_drift_m=0., numeric_velocity_error_m_s=0.,
                  numeric_acceleration_error_m_s2=0.)
    header = ['reference_kind', 'delta_yaw_deg', 'robot', 't_s', 'phi_rad', 'omega_rad_s', 'alpha_rad_s2']
    for group in ('center_WL', 'base_WL', 'grasp_WL', 'base_v_WL', 'base_a_WL',
                  'grasp_v_WL', 'grasp_a_WL', 'base_v_body'):
        header += [group+'_'+axis for axis in 'xyz']
    header += ['base_yaw_unwrapped_rad', 'radius_error_m', 'grasp_closure_error']
    header += ['grasp_R_'+str(i)+str(j) for i in range(3) for j in range(3)]
    with path.open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        for degrees in angles_deg:
            for robot, values in geometry['robots'].items():
                b, g = (leader_w @ np.array(values[k]) for k in
                        ('world_T_base_footprint', 'world_T_tcp'))
                trajectory = ObjectCenterRotation(local_o, b, g, duration=duration,
                                                   delta_yaw_rad=np.deg2rad(degrees))
                bg = inverse_transform(b) @ g
                radius0 = np.linalg.norm(b[:2, 3]-local_o[:2, 3])
                for t in np.linspace(0., duration, count):
                    ref = trajectory.sample(t)
                    radius_error = abs(np.linalg.norm(ref.base_pose[:2, 3]-local_o[:2, 3])-radius0)
                    closure = np.max(abs(ref.base_pose @ bg-ref.grasp_pose))
                    maxima['radius_error_m'] = max(maxima['radius_error_m'], float(radius_error))
                    maxima['grasp_closure_error'] = max(maxima['grasp_closure_error'], float(closure))
                    maxima['center_drift_m'] = max(maxima['center_drift_m'], float(np.linalg.norm(
                        ref.object_pose[:3, 3]-local_o[:3, 3])))
                    h = duration*1e-5
                    if h < t < duration-h:
                        before, after = trajectory.sample(t-h), trajectory.sample(t+h)
                        for key, numeric, analytic in (
                            ('numeric_velocity_error_m_s', (after.base_pose[:3, 3]-before.base_pose[:3, 3])/(2*h), ref.base_velocity_world),
                            ('numeric_acceleration_error_m_s2', (after.base_velocity_world-before.base_velocity_world)/(2*h), ref.base_acceleration_world)):
                            maxima[key] = max(maxima[key], float(np.max(abs(numeric-analytic))))
                    row = ['offline_ideal_reference', degrees, robot, t, ref.phi, ref.omega, ref.alpha]
                    for value in (local_o[:3, 3], ref.base_pose[:3, 3], ref.grasp_pose[:3, 3],
                                  ref.base_velocity_world, ref.base_acceleration_world,
                                  ref.grasp_velocity_world, ref.grasp_acceleration_world,
                                  world_velocity_to_body(ref.base_velocity_world, ref.base_pose[:3, :3])):
                        row.extend(value)
                    row += [ref.base_yaw_unwrapped, radius_error, closure]
                    row.extend(ref.grasp_pose[:3, :3].ravel())
                    writer.writerow(row)
    return maxima


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration-s', type=float, required=True,
                        help='calculation duration only; not an approved robot speed')
    parser.add_argument('--angles-deg', nargs='+', type=float, default=[-5., 5., -15., 15., -30., 30., -90., 90.])
    parser.add_argument('--samples', type=int, default=201)
    args = parser.parse_args()
    from amir_compliance_control.object_center_rotation_trajectory import quintic
    for angle in args.angles_deg:
        quintic(0., args.duration_s, np.deg2rad(angle))
    if args.samples < 3:
        parser.error('--samples must be >= 3')
    args.output.mkdir(parents=True, exist_ok=True)
    generated = args.output/'generated'
    spawn = generate_fixture(generated)
    geometry = fixture_geometry(generated, spawn)
    maxima = write_timeseries(args.output/'reference_timeseries.csv', geometry,
                             args.angles_deg, args.duration_s, args.samples)
    # All source inputs used by xacro, plus launch, payload and controller files.
    import xacro
    from ament_index_python.packages import get_package_share_directory as share
    source_paths = {Path(p).resolve() for p in xacro.all_includes}
    source_paths.update([BRINGUP/'compliance_cp1.launch.py', BRINGUP/'compliance_cp6_fixture.launch.py',
        Path(share('amir_description'))/'urdf/amir_mecanum3_sim.xacro',
        Path(share('amir_gazebo'))/'launch/robot_bringup.launch.py',
        Path(share('cooperative_transport_gazebo'))/'worlds/cooperative_transport.sdf',
        Path(share('mecanumrover_description'))/'params/controllers.yaml',
        *(Path(share('cooperative_transport_description'))/'models'/p/'model.sdf'
          for p in ('cooperative_payload', 'cooperative_payload_friction')),
        Path(__file__), ROOT/'src/humbleble_ws/amir_compliance_control/amir_compliance_control/object_center_rotation_trajectory.py'])
    hashes = {str(p.resolve()): hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(source_paths)}
    passed = (all(v < 1e-8 for v in maxima.values()) and all(
        r['reconstruction_max_error'] < 1e-10 for r in geometry['robots'].values()))
    report = {
        'checkpoint': 'CP-8R-0', 'geometry_passed': passed,
        'scope': 'offline nominal geometry; no ROS graph, physics, GT measurements or control commands',
        'csv_coordinates': 'W_L = leader initial base_footprint, +z up; WL suffix denotes this local frame',
        'fixture_launch_overrides': {'constraint_fixture': True, 'weld_cfm': .1, 'calibration_fixture': False},
        'duration_s_calculation_only': args.duration_s, 'angles_deg_calculation_only': args.angles_deg,
        'samples_per_robot_per_angle': args.samples, 'spawn_arguments': spawn,
        'geometry': geometry, 'numerical_maxima': maxima,
        'ready_for_grasped_cp8r1': False,
        'remaining': ['Verify settled odom/FK, actual attachment offset and full grasp orientation before latching C.',
                      'Source spawn is 0.03 m above the payload anchor design height; measure settled placement.',
                      'SDF anchor and gripper rotations differ; anchor pose cannot substitute for object_T_grasp.',
                      'Test leader alone, command exclusion, dual stop/hold and local follower yaw before grasped rotation.',
                      'Calibrate collision sweep, FT, joint/pose/contact thresholds and velocity/acceleration limits.'],
        'source_sha256': hashes, 'argv': sys.argv,
    }
    (args.output/'geometry_audit.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps({'geometry_passed': passed, 'numerical_maxima': maxima,
                      'ready_for_grasped_cp8r1': False, 'output': str(args.output)}, indent=2))
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())

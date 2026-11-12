"""Current source model checks; no running Gazebo or ROS graph required."""
import csv
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from amir_cp8r0_geometry_audit import generate_fixture, fixture_geometry, write_timeseries
from amir_compliance_control.object_center_rotation_trajectory import initialize_object_pose


@pytest.fixture
def audited(tmp_path):
    spawn = generate_fixture(tmp_path/'generated')
    return tmp_path, fixture_geometry(tmp_path/'generated', spawn)


def test_actual_model_exposes_frame_and_initial_placement_differences(audited):
    _, geometry = audited
    assert geometry['box_size_in_payload_axes_m'] == [1.2, .03, .08]
    assert geometry['assembly_mass_kg'] == pytest.approx(.52)
    o = np.array(geometry['world_T_object'])
    np.testing.assert_allclose(o[:3, :3], np.eye(3), atol=1e-15)
    np.testing.assert_allclose(geometry['assembly_com_world_m'], o[:3, 3], atol=1e-15)
    for robot in geometry['robots'].values():
        assert robot['tcp_sensor_distance_m'] == pytest.approx(.0795)
        assert robot['initial_min_joint_margin_rad'] > 0
        b_link = np.array(robot['base_footprint_T_base_link'])
        assert np.arctan2(b_link[1, 0], b_link[0, 0]) == pytest.approx(-1.5708)
        np.testing.assert_allclose(robot['sensor_minus_anchor_xyz_m'][2], .03, atol=1e-15)
        assert robot['sensor_anchor_rotation_frobenius'] > 1.
        assert robot['anchor_as_calibration_center_error_m'] > .5
        assert robot['reconstruction_max_error'] < 1e-14
        assert all(p == 'gz_ros2_control/GazeboSimSystem' for p in robot['ros2_control_hardware_plugins'])


def test_tcp_and_sensor_calibrations_are_consistent_but_not_interchangeable(audited):
    _, geometry = audited
    robot = geometry['robots']['leader']
    wb = robot['world_T_base_footprint']
    for fk_key, calibration_key in (
            ('base_footprint_T_tcp', 'object_T_tcp_nominal'),
            ('base_footprint_T_gripper_base', 'object_T_gripper_base_nominal')):
        np.testing.assert_allclose(initialize_object_pose(wb, robot[fk_key], robot[calibration_key]),
                                   geometry['world_T_object'], atol=1e-14)
    mixed = initialize_object_pose(wb, robot['base_footprint_T_tcp'], robot['object_T_gripper_base_nominal'])
    assert np.linalg.norm(mixed[:3, 3]-np.array(geometry['world_T_object'])[:3, 3]) == pytest.approx(.0795)


def test_evaluation_csv_both_signed_paths_have_constant_center_and_closure(audited):
    folder, geometry = audited
    maxima = write_timeseries(folder/'reference.csv', geometry, [-90., -5., 5., 90.], 40., 21)
    assert all(error < 1e-8 for error in maxima.values())
    with (folder/'reference.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 4*2*21
    assert {row['reference_kind'] for row in rows} == {'offline_ideal_reference'}
    assert {row['robot'] for row in rows} == {'leader', 'follower'}
    assert len({row['center_WL_z'] for row in rows}) == 1


def test_audit_rejects_uninspected_tilted_long_axis(audited):
    folder, _ = audited
    world_file = folder/'generated/world.sdf'
    root = ET.parse(world_file)
    payload = root.find("world/model[@name='cooperative_payload']")
    payload.find('pose').text = '0 0 .7 0 .1 0'
    root.write(world_file)
    with pytest.raises(ValueError, match='horizontal'):
        fixture_geometry(folder/'generated', generate_spawn_for_saved_fixture(folder))


def generate_spawn_for_saved_fixture(folder):
    # Save original spawn values in a separate folder without replacing the
    # intentionally invalid world used by the negative test.
    return generate_fixture(folder/'other')

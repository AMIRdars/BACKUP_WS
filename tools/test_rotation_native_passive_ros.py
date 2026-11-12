"""Use the actual passive ROS endpoint; no simulator or actuation is started."""
import json
import time
from pathlib import Path

import rclpy
from rclpy.parameter import Parameter
import yaml

from amir_compliance_control.rotation_follow import RotationFollower
from amir_compliance_control.local_yaw import ArmSoftTask
from amir_cp8r3_native_trial import prepare_control_policies
from amir_rotation_native_passive import configure_passive, read_passive_endpoint
from amir_rotation_native_arm_policy import read_live, KEYS


ROOT = Path(__file__).resolve().parents[1]
NATIVE = ROOT/'audits/object_center_rotation_20261103/cp8r3'


def test_live_passive_endpoint_cannot_authorize_rotation_and_has_one_arm_task(tmp_path):
    leader, follower = prepare_control_policies(NATIVE/'native_generated', tmp_path, 75., 0., 0., 0., 0., .4,
        NATIVE/'native_metrology/calibration/calibration.json')
    p = yaml.safe_load(follower.read_text()).pop('/follower/follower_yaw_arm_base')
    follower.write_text(yaml.safe_dump({'/follower/follower_yaw_whole_body': p}))
    configure_passive(leader, follower, delta_deg=.005, duration_s=75.,
        leader_limits_file=ROOT/'audits/rotation_equivariant_expansion_20261105/wb_15_03/leader_policy.yaml')
    params = next(iter(yaml.safe_load(follower.read_text()).values()))['ros__parameters']
    common = dict(use_sim_time=True, urdf_file=str(NATIVE/'native_generated/follower.urdf'),
        rotation_follow_mode='XY_ONLY', yaw_heading_frame='grasp_heading', actuator_ack_required=True)
    rclpy.init(args=[]); node = None
    try:
        node = RotationFollower(namespace='/follower', parameter_overrides=[Parameter(k, value=v) for k, v in {**params, **common}.items()])
        assert node.get_fully_qualified_name() == '/follower/follower_yaw_whole_body'
        assert node.mode == 'XY_ONLY' and node.passive_calibration
        assert not node.requires_rotation_observation() and not node.may_authorize_rotation()
        assert isinstance(node.new_arm(), ArmSoftTask)
        assert node.core is None and not node.permit
        def wait(predicate, timeout):
            deadline = time.monotonic()+timeout
            while not predicate():
                if time.monotonic() > deadline:
                    raise RuntimeError('actual parameter RPC timed out')
                rclpy.spin_once(node, timeout_sec=.01)
        actual = read_passive_endpoint(node, wait)
        mechanics = read_live(node,wait)
        assert mechanics=={name:node.p(name) for name in KEYS}
        assert mechanics['rotational_relief_enabled'] is False
        assert mechanics['arm_yaw_admittance_enabled'] is True
        assert actual['base_gains'] == [0.]*3 and actual['arm_yaw_admittance_enabled']
        assert not any(s.topic_name.startswith(('/leader/', '/world/', '/experiment/')) for s in node.subscriptions)
        assert sum(p.topic_name == '/follower/base_cmd' for p in node.publishers) == 1
        assert sum(p.topic_name == '/follower/arm_joint_cmd' for p in node.publishers) == 1
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()

"""Instantiate the real native O2 endpoint without any simulator or permit."""
import rclpy
from rclpy.parameter import Parameter
import yaml

from amir_compliance_control.rotation_compensation_probe import CompensationProbe, ProbeRecentering
from test_rotation_native_o2 import configure, SOURCE
from amir_rotation_native_o2 import configure_o2


def test_actual_native_o2_class_preserves_exclusive_outputs_and_cannot_authorize_leader(tmp_path):
    path, report, certificate, output, follower = configure(tmp_path)
    plan = configure_o2(path,SOURCE,output,follower)
    params = next(iter(yaml.safe_load(follower.read_text()).values()))['ros__parameters']
    common = dict(use_sim_time=True,urdf_file=str(SOURCE/'follower.urdf'),yaw_heading_frame='grasp_heading',
        rotation_follow_mode='XY_YAW_COMP',actuator_ack_required=True)
    rclpy.init(args=[]);node=None
    try:
        node=CompensationProbe(namespace='/follower',parameter_overrides=[Parameter(k,value=v) for k,v in {**params,**common}.items()])
        assert node.get_fully_qualified_name()=='/follower/follower_yaw_whole_body'
        assert node.mode=='XY_YAW_COMP' and node.requires_rotation_observation()
        assert not node.may_authorize_rotation() and not node.passive_calibration
        assert isinstance(node.new_base(),ProbeRecentering)
        assert node.core is None and not node.permit and node.probe_start is None
        assert node.get_parameter('probe_angle_rad').value==plan['probe_angle_rad']
        assert sum(p.topic_name=='/follower/base_cmd' for p in node.publishers)==1
        assert sum(p.topic_name=='/follower/arm_joint_cmd' for p in node.publishers)==1
        assert not any(s.topic_name.startswith(('/leader/','/world/','/experiment/')) for s in node.subscriptions)
    finally:
        if node is not None:node.destroy_node()
        rclpy.try_shutdown()

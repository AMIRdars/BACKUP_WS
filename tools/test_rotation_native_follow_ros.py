"""Actual qualified WHOLE_BODY endpoint; absence of live yaw still blocks enable."""
import rclpy
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger
import yaml

from amir_compliance_control.rotation_follow import RotationFollower
from amir_compliance_control.whole_body import WholeBodyAllocator
from amir_rotation_native_follow import configure_follow
from test_rotation_native_follow import follow_case,SOURCE


def test_actual_native_follow_node_preserves_one_allocator_and_mandatory_live_gate(tmp_path):
    args,output,follower,_=follow_case(tmp_path)
    configure_follow(args,SOURCE,output,output/'rotation_policy.yaml',follower)
    params=next(iter(yaml.safe_load(follower.read_text()).values()))['ros__parameters']
    common=dict(use_sim_time=True,urdf_file=str(SOURCE/'follower.urdf'),rotation_follow_mode='WHOLE_BODY',
        yaw_heading_frame='grasp_heading',actuator_ack_required=True)
    rclpy.init(args=[]);node=None
    try:
        node=RotationFollower(namespace='/follower',parameter_overrides=[Parameter(k,value=v) for k,v in {**params,**common}.items()])
        assert node.mode=='WHOLE_BODY' and not node.passive_calibration and node.requires_rotation_observation()
        assert isinstance(node.new_arm(),WholeBodyAllocator) and node.task_compliance_budget is not None
        response=node.enable(Trigger.Request(),Trigger.Response())
        assert not response.success and node.core is None and not node.permit
        assert node.get_parameter('allocation_base_regularization').value==3e-5
        assert sum(p.topic_name=='/follower/base_cmd' for p in node.publishers)==1
        assert sum(p.topic_name=='/follower/arm_joint_cmd' for p in node.publishers)==1
        assert not any(s.topic_name.startswith(('/leader/','/world/','/experiment/')) for s in node.subscriptions)
    finally:
        if node is not None:node.destroy_node()
        rclpy.try_shutdown()

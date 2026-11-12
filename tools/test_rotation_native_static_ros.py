"""Actual native endpoint parameter RPC, isolated from every simulator."""
import math,time
from pathlib import Path
import rclpy,yaml
from rclpy.parameter import Parameter
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterType
from amir_compliance_control.rotation_follow import RotationFollower
from amir_cp8r3_native_trial import prepare_control_policies

ROOT=Path(__file__).resolve().parents[1]
NATIVE=ROOT/'audits/object_center_rotation_20261103/cp8r3'


def test_unqualified_native_endpoint_exposes_real_policy_without_enabling_motion(tmp_path):
 _,policy=prepare_control_policies(NATIVE/'native_generated',tmp_path,60.,0.,0.,2.,.5,.4,
  NATIVE/'native_metrology/calibration/calibration.json')
 params=next(iter(yaml.safe_load(policy.read_text()).values()))['ros__parameters']
 # The old YAML omits pose defaults. Resolve the running class through its
 # actual ROS service rather than filling those keys in the evaluator.
 assert 'rotation_error_limit_rad' not in params and 'joint_margin_rad' not in params
 rclpy.init(args=[]);node=None
 try:
  common=dict(use_sim_time=True,urdf_file=str(NATIVE/'native_generated/follower.urdf'),
   rotation_follow_mode='WHOLE_BODY',yaw_heading_frame='grasp_heading',actuator_ack_required=True)
  node=RotationFollower(namespace='/follower',parameter_overrides=[Parameter(k,value=v) for k,v in {**params,**common}.items()])
  assert node.get_fully_qualified_name()=='/follower/follower_yaw_whole_body'
  client=node.create_client(GetParameters,node.get_fully_qualified_name()+'/get_parameters')
  request=GetParameters.Request();request.names=['rotation_error_limit_rad','joint_margin_rad',
   'wrench_force_limit_N','wrench_torque_limit_Nm','yaw_heading_frame']
  assert client.wait_for_service(timeout_sec=5.)
  future=client.call_async(request);deadline=time.monotonic()+5.
  while not future.done() and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.02)
  assert future.done() and future.exception() is None
  values=future.result().values
  assert len(values)==5 and all(v.type==ParameterType.PARAMETER_DOUBLE for v in values[:4])
  assert all(math.isfinite(v.double_value) and v.double_value>0. for v in values[:4])
  assert values[4].type==ParameterType.PARAMETER_STRING and values[4].string_value=='grasp_heading'
  assert node.core is None and not node.permit
  assert not any(s.topic_name.startswith(('/leader/','/world/','/experiment/')) for s in node.subscriptions)
  node.destroy_client(client)
 finally:
  if node is not None:node.destroy_node()
  rclpy.try_shutdown()

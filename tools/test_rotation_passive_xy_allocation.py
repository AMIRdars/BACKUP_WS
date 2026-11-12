"""Own XY translation can restore rank without granting autonomous base yaw."""
from pathlib import Path

import numpy as np
import pytest
import rclpy
from rclpy.parameter import Parameter
import yaml

from amir_compliance_control.wrench import LocalModel
from amir_compliance_control.whole_body import WholeBodyAllocator,whole_jacobian
from amir_compliance_control.local_yaw import ArmSoftTask
from amir_compliance_control.rotation_follow import RotationFollower
from amir_cp8r3_native_trial import prepare_control_policies
from amir_rotation_native_passive import configure_passive

ROOT=Path(__file__).resolve().parents[1]
NATIVE=ROOT/'audits/object_center_rotation_20261103/cp8r3'
HOME=np.array([0.,1.4,-1.6,.2,1.570796])


def test_model_rank_and_locked_yaw_allocation_relieve_the_incompatible_arm_task():
    model=LocalModel((NATIVE/'native_generated/follower.urdf').read_text());q=HOME.copy()
    j=whole_jacobian(model,q)
    assert np.linalg.matrix_rank(j[:,:5],tol=1e-5)==5
    assert np.linalg.matrix_rank(j[:,:7],tol=1e-5)==6
    desired=np.array([0.,-1e-5,0.,0.,0.,1e-5]);metric=[1.,1.,1.,.1,.1,.4]
    arm=ArmSoftTask(model,metric=metric,speed=.03,acceleration=.15,margin=.01)
    rigid=arm.step(q,q,desired,.01)
    allocator=WholeBodyAllocator(model,task_metric=metric,base_speed=(.0001,.0001,.002),lock_base_yaw=True)
    for _ in range(5):
        u=allocator.step(q,q,desired,[0.,0.,0.],1.,.01)
        assert u[7]==0. and abs(u[5])<=.0001 and abs(u[6])<=.0001
    assert np.linalg.norm((j@u-desired)*metric)<np.linalg.norm((j[:,:5]@rigid-desired)*metric)/100.
    assert allocator.last['base_yaw_locked'] and len(allocator.last['velocity'])==8


def params(tmp_path,flag=True):
    leader,follower=prepare_control_policies(NATIVE/'native_generated',tmp_path,75.,0.,0.,0.,0.,.4,
        NATIVE/'native_metrology/calibration/calibration.json')
    p=yaml.safe_load(follower.read_text()).pop('/follower/follower_yaw_arm_base')
    follower.write_text(yaml.safe_dump({'/follower/follower_yaw_whole_body':p}))
    configure_passive(leader,follower,delta_deg=.005,duration_s=75.,
        leader_limits_file=ROOT/'audits/rotation_equivariant_expansion_20261105/wb_15_03/leader_policy.yaml',
        xy_allocation=flag,allocation_policy_file=ROOT/'audits/rotation_equivariant_expansion_20261105/whole_body_allocation_policy.yaml')
    return next(iter(yaml.safe_load(follower.read_text()).values()))['ros__parameters']


def test_live_calibration_keeps_one_writer_and_never_authorizes_base_yaw(tmp_path):
    p=params(tmp_path);p.update(use_sim_time=True,urdf_file=str(NATIVE/'native_generated/follower.urdf'),
        rotation_follow_mode='XY_ONLY',yaw_heading_frame='grasp_heading')
    rclpy.init(args=[]);node=None
    try:
        node=RotationFollower(namespace='/follower',parameter_overrides=[Parameter(k,value=v) for k,v in p.items()])
        a=node.new_arm()
        assert isinstance(a,WholeBodyAllocator) and a.lock_base_yaw
        assert not node.may_authorize_rotation() and not node.requires_rotation_observation()
        assert node.task_compliance_budget is not None
        assert sum(pub.topic_name=='/follower/base_cmd' for pub in node.publishers)==1
        assert sum(pub.topic_name=='/follower/arm_joint_cmd' for pub in node.publishers)==1
        assert not any(s.topic_name.startswith(('/leader/','/world/','/experiment/')) for s in node.subscriptions)
    finally:
        if node is not None:node.destroy_node()
        rclpy.try_shutdown()


@pytest.mark.parametrize('change',['not_calibration','wrong_mode','no_budget','too_fast','no_arm_compliance'])
def test_unqualified_operational_mode_cannot_enable_the_calibration_exception(tmp_path,change):
    p=params(tmp_path)
    p.update(use_sim_time=True,urdf_file=str(NATIVE/'native_generated/follower.urdf'),rotation_follow_mode='XY_ONLY')
    if change=='not_calibration':p['passive_yaw_calibration']=False
    elif change=='wrong_mode':p['rotation_follow_mode']='WHOLE_BODY'
    elif change=='no_budget':p['allocation_task_compliance_budget']=False
    elif change=='too_fast':p['base_velocity_limits'][0]=.0002
    else:p['arm_yaw_admittance_enabled']=False
    rclpy.init(args=[])
    try:
        with pytest.raises(ValueError):RotationFollower(namespace='/follower',parameter_overrides=[Parameter(k,value=v) for k,v in p.items()])
    finally:rclpy.try_shutdown()

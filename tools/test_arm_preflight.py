"""Long IK preflight must not block receive callbacks or accept after a stop."""
from concurrent.futures import Future,ThreadPoolExecutor
from pathlib import Path
from threading import Event
from types import SimpleNamespace
import time
import numpy as np
import pytest
from geometry_msgs.msg import Vector3Stamped
from amir_compliance_control import arm_only_leader as leader
from amir_compliance_control.wrench import LocalModel
from amir_compliance_control.stroke_kinematics import verify_stroke_branch
from amir_native_arm_support import SCENE

ROOT=Path(__file__).resolve().parents[1]


def local_endpoint():
    q=np.array([0.,1.4,-1.6,.2,1.570796])
    model=LocalModel((ROOT/SCENE/'leader.urdf').read_text())
    node=SimpleNamespace(state='READY',motion=None,pending_motion=None,homed=True,
        local_health=lambda:(True,''),q_actual=lambda:q.copy(),model=model,
        base_pose=lambda:np.zeros(3),sim=1.,last_goal_stamp=0.,accepted=0,rejected=0,
        last_rejection='',latched=False,emergency=False,permit=True,
        p=lambda key:{'max_distance_m':.213,'timeout_s':.15}[key])
    node.finish_preflight=lambda:leader.ArmOnlyLeader.finish_preflight(node)
    return node,q


def test_pending_ik_keeps_zero_output_and_can_process_local_state(monkeypatch):
    entered,release=Event(),Event()
    def blocked(*args):
        entered.set();assert release.wait(2.)
    monkeypatch.setattr(leader,'verify_stroke_branch',blocked)
    node,q=local_endpoint()
    with ThreadPoolExecutor(max_workers=1) as executor:
        node.preflight_executor=executor
        try:
            msg=Vector3Stamped();msg.header.frame_id='leader/base_footprint'
            msg.header.stamp.sec=1;msg.vector.x=.213
            leader.ArmOnlyLeader.goal(node,msg)
            assert entered.wait(1.)
            assert not node.pending_motion[0].done()
            # These main-thread receive/timer operations remain available while
            # the branch solver is blocked; no receipt timeout is increased.
            for _ in range(5):
                assert node.local_health()[0]
                np.testing.assert_array_equal(leader.ArmOnlyLeader.base_request(node,0.),np.zeros(3))
            assert node.accepted==0 and node.state=='READY'
            release.set();node.pending_motion[0].result(timeout=1.)
            node.finish_preflight()
            assert node.accepted==1 and node.state=='TRANSPORT'
        finally:release.set()


@pytest.mark.parametrize('changed',['stop','health','arm_position','IK_failure'])
def test_completed_preflight_cannot_bypass_current_local_safety(changed):
    node,q=local_endpoint();future=Future()
    if changed=='IK_failure':future.set_exception(ValueError('unreachable branch'))
    else:future.set_result(None)
    node.pending_motion=(future,time.monotonic(),.213,1.,q.copy())
    if changed=='stop':node.emergency=True
    if changed=='health':node.local_health=lambda:(False,'stale input')
    if changed=='arm_position':node.q_actual=lambda:q+np.array([0.,.002,0.,0.,0.])
    node.finish_preflight()
    assert node.accepted==0 and node.rejected==1 and node.motion is None


def test_full_preflight_uses_real_branch_without_mutating_latched_seed():
    node,q=local_endpoint();p,s=node.model.frames(q);original=q.copy()
    verify_stroke_branch(node.model,q,p,s[:3,:3],.213)
    np.testing.assert_array_equal(q,original)

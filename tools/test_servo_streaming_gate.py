"""Audited servo targets retain independent model velocity/position guards."""
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from builtin_interfaces.msg import Duration
from rclpy.time import Time
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from amir_compliance_control.servo_streaming_gate import servo_reference_ratio
from amir_compliance_control.safety import RobotSafetyGate
from amir_compliance_control.wrench import LocalModel
from amir_native_arm_support import SCENE

ROOT=Path(__file__).resolve().parents[1]


def request(servo=True,rate=.12):
    q=np.array([0.,1.4,-1.6,.2,1.570796]);sent=[];fault=[]
    rig=SimpleNamespace(robot='leader',sim=2.,armed_sim=1.,enabled=lambda:True,
        p=lambda k:{'max_skew_s':.02,'command_timeout_s':.15}[k],q=q,
        model=LocalModel((ROOT/SCENE/'leader.urdf').read_text()),margin=.01,
        healthy=lambda:(True,''),arm_pub=SimpleNamespace(publish=sent.append),trip=fault.append,
        servo_tau=np.full(5,.04) if servo else None,servo_period=.004,servo_reference_ratio_peak=0.)
    v=np.array([0.,-.7*rate,rate,-.3*rate,0.])
    msg=JointTrajectory(joint_names=list(rig.model.names));msg.header.stamp=Time(seconds=1.996).to_msg()
    msg.header.frame_id='leader/base_footprint'
    for value,t in [(q+.04*v,.004),(q+.12*v,.084)]:
        p=JointTrajectoryPoint(positions=value.tolist());p.time_from_start=Duration(nanosec=round(t*1e9));msg.points.append(p)
    return rig,msg,sent,fault


def test_same_audited_reference_passes_model_but_not_four_ms_jump_assumption():
    for enabled in (False,True):
        rig,msg,sent,fault=request(enabled);RobotSafetyGate.arm_command(rig,msg)
        assert bool(sent)==enabled
        if enabled:assert rig.servo_reference_ratio_peak==pytest.approx(.12)
        else:assert fault==['arm command violates model limits']


def test_servo_target_that_would_drive_over_urdf_velocity_is_rejected():
    rig,msg,sent,fault=request(True,1.01);RobotSafetyGate.arm_command(rig,msg)
    assert not sent and 'unchanged URDF velocity' in fault[0]


def test_unchanged_joint_position_margin_still_rejects_a_small_step():
    rig,msg,sent,fault=request()
    rig.q[0]=rig.model.upper[0]-.02
    for p in msg.points:p.positions[0]=rig.model.upper[0]-.005
    RobotSafetyGate.arm_command(rig,msg)
    assert not sent and fault==['arm command violates model limits']


def test_fresh_feedback_is_required_for_servo_model():
    rig,msg,sent,fault=request();rig.healthy=lambda:(False,'stale joints')
    RobotSafetyGate.arm_command(rig,msg)
    assert not sent and 'fresh local state' in fault[0]


def test_future_segment_cannot_exceed_urdf_even_when_now_error_is_zero():
    with pytest.raises(ValueError,match='URDF velocity'):
        servo_reference_ratio(np.zeros(5),np.zeros(5),np.ones(5)*.09,0.,.08,0.,np.ones(5)*.04,np.ones(5))


def test_old_first_knot_checks_the_interpolated_reference_at_receipt():
    # A stale first point cannot hide a later already-over-speed servo target.
    with pytest.raises(ValueError,match='URDF velocity'):
        servo_reference_ratio(np.zeros(5),np.zeros(5),np.ones(5)*.08,0.,.08,.05,np.ones(5)*.04,np.ones(5))


@pytest.mark.parametrize('tau',[np.zeros(5),np.ones(5)*float('nan')])
def test_unmeasured_or_invalid_servo_response_is_rejected(tau):
    with pytest.raises(ValueError):servo_reference_ratio(np.zeros(5),np.zeros(5),np.zeros(5),0.,.08,0.,tau,np.ones(5))

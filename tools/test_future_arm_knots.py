"""Future timestamps must preserve the original URDF safety gate checks."""
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from builtin_interfaces.msg import Duration
from rclpy.time import Time
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from amir_compliance_control.arm_trajectory_bounds import trajectory_times
from amir_compliance_control.safety import RobotSafetyGate
from amir_compliance_control.wrench import LocalModel
from amir_native_arm_support import SCENE

ROOT=Path(__file__).resolve().parents[1]


def gate_request(delay,rate=.12):
    q=np.array([0.,1.4,-1.6,.2,1.570796]);sent=[];fault=[]
    rig=SimpleNamespace(robot='leader',sim=2.,armed_sim=1.,enabled=lambda:True,
        p=lambda k:{'max_skew_s':.02,'command_timeout_s':.15}[k],
        q=q,model=LocalModel((ROOT/SCENE/'leader.urdf').read_text()),margin=.01,
        healthy=lambda:(True,''),arm_pub=SimpleNamespace(publish=sent.append),trip=fault.append)
    header,first,end=trajectory_times(2.,.004,.08,delay)
    msg=JointTrajectory(joint_names=list(rig.model.names))
    msg.header.stamp=Time(seconds=header).to_msg();msg.header.frame_id='leader/base_footprint'
    velocity=np.array([0.,-.7*rate,rate,-.3*rate,0.])
    for position,t in [(q+.04*velocity,first),(q+.12*velocity,end)]:
        point=JointTrajectoryPoint(positions=position.tolist())
        point.time_from_start=Duration(sec=0,nanosec=round(t*1e9));msg.points.append(point)
    RobotSafetyGate.arm_command(rig,msg)
    return sent,fault,msg


def test_legacy_first_point_remains_at_now():
    header,first,end=trajectory_times(2.,.004,.08)
    assert header+first==2.
    assert header+end==pytest.approx(2.08)


def test_compensated_fast_reference_is_rejected_when_scheduled_in_four_ms():
    sent,fault,_=gate_request(0.)
    assert not sent and fault==['arm command violates model limits']


def test_future_point_is_reachable_and_moves_actual_target_time_forward():
    sent,fault,msg=gate_request(.04)
    assert len(sent)==1 and not fault
    assert msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9==2.
    assert msg.points[0].time_from_start.nanosec==40000000


def test_future_timing_cannot_bypass_unchanged_urdf_velocity_check():
    sent,fault,_=gate_request(.04,rate=1.2)
    assert not sent and fault==['arm command violates model limits']


@pytest.mark.parametrize('delay',[-.001,float('nan'),.101])
def test_bad_future_delay_is_refused(delay):
    with pytest.raises(ValueError):trajectory_times(2.,.004,.08,delay)

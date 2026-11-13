"""Physical model feasibility, velocity admittance and final output guards."""
from pathlib import Path
import runpy
import numpy as np
import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument,ExecuteProcess
from launch.utilities import perform_substitutions
from amir_compliance_control.admittance import PlanarAdmittance
from amir_compliance_control.arm_translation import reference,duration
from dual_arm_only_runtime import arm_plan

ROOT=Path(__file__).resolve().parents[1]


def test_quintic_arm_path_respects_speed_acceleration_and_stops_smoothly():
    for distance in (.005,.03,-.03):
        total=duration(abs(distance),.005,.0012)
        times=np.linspace(0,total,20001)
        data=np.array([reference(t,distance,.005,.0012)[:2] for t in times])
        assert np.max(abs(data[:,1]))<=.005+1e-12
        assert np.max(abs(np.diff(data[:,1])/np.diff(times)))<=.0012+1e-9
        np.testing.assert_allclose(data[[0,-1],0],[0,distance],atol=1e-14)
        np.testing.assert_allclose(data[[0,-1],1],0,atol=1e-14)


def test_zero_stiffness_stops_velocity_and_preserves_offset():
    core=PlanarAdmittance([2.,2.],[34.6,34.6],[0.,0.],[.05,.05],[.01,.01],[.02,.02])
    for _ in range(100):core.step([.1,0.],.01)
    assert core.x[0]>.001 and core.v[0]>0
    for _ in range(100):core.step([0.,0.],.01)
    assert abs(core.v[0])<1e-9 and core.x[0]>.001
    previous=core.x.copy()
    for _ in range(50):core.step([0.,0.],.01)
    np.testing.assert_allclose(core.x,previous,atol=1e-9)
    with pytest.raises(ValueError):PlanarAdmittance([2.,2.],[34.6,34.6],[-1.,0.],[.05,.05],[.01,.01],[.02,.02])


def test_both_arm_branches_reachable_for_actual_opposed_model():
    plan=arm_plan(ROOT,.03)
    assert plan['leader']['local_direction_x']==1.
    assert plan['follower']['local_direction_x']==-1.
    for proof in plan.values():
        assert min(s['joint_margin_rad'] for s in proof['samples'])>.8
        assert min(s['sigma_xy_m'] for s in proof['samples'])>.1


def public_context():
    module=runpy.run_path(str(ROOT/'src/humbleble_ws/cooperative_transport_bringup/launch/compliance_round_trip.launch.py'))
    ctx=LaunchContext()
    for action in module['generate_launch_description']().entities:
        if isinstance(action,DeclareLaunchArgument):action.execute(ctx)
    return module,ctx


def test_original_command_selects_both_bases_stopped_not_one_metre_base_trip(tmp_path):
    module,ctx=public_context()
    ctx.launch_configurations.update(workspace_directory=str(ROOT),output_directory=str(tmp_path/'out'),
        forward_distance_m='1.0',translation_velocity_m_s='0.1')
    process=next(a for a in module['setup'](ctx) if isinstance(a,ExecuteProcess))
    argv=[perform_substitutions(ctx,v) for v in process.cmd]
    assert '--dual-arm-only' in argv and '--follower-base-fixture' not in argv
    assert argv[argv.index('--distance')+1]=='0.213'
    assert float(argv[argv.index('--speed')+1])==0.003
    assert argv[argv.index('--virtual-stiffness-n-m')+1]=='0'


@pytest.mark.parametrize('robot',['leader','follower'])
def test_nonzero_base_command_rejected_for_each_robot(tmp_path,robot):
    from test_cp7_safety import GuardRig
    from geometry_msgs.msg import Twist
    rig=GuardRig(tmp_path)
    try:
        rig.start();guard=rig.guards[robot];guard.base_zero_only=True
        msg=Twist();msg.linear.x=.000001;guard.base(msg)
        assert guard.fault=='ARM_ONLY rejected nonzero final base request'
        rig.pump(.3)
        assert rig.output=={'leader':0.,'follower':0.}
    finally:rig.close()

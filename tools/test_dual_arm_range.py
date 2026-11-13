"""The visible excursion stays inside actual common reach and joint rate bounds."""
from pathlib import Path
import numpy as np
import pytest
from dual_arm_workspace import common_reach,trajectory_rate_proof,wait_for_arm_completion
from amir_compliance_control.arm_translation import duration
from dual_arm_only_runtime import arm_plan
from test_dual_arm_only import public_context
from launch.actions import ExecuteProcess
from launch.utilities import perform_substitutions

ROOT=Path(__file__).resolve().parents[1]


def test_common_reach_is_limited_by_extended_leader_not_follower():
    proof=common_reach(ROOT)
    assert .21<proof['maximum_common_verified_m']<.22
    assert proof['arms']['follower']['verified_horizontal_reach_m']>.26
    plan=arm_plan(ROOT,proof['maximum_common_verified_m'])
    assert abs(plan['leader']['samples'][-1]['q_rad'][2]-plan['leader']['samples'][0]['q_rad'][2])>1.4


@pytest.mark.parametrize('distance',[.214,.5,float('nan')])
def test_unreachable_request_is_refused_before_simulation(tmp_path,distance):
    module,ctx=public_context()
    ctx.launch_configurations.update(workspace_directory=str(ROOT),dual_arm_distance_m=str(distance),output_directory=str(tmp_path/'out'))
    with pytest.raises(ValueError,match='URDF reach'):module['setup'](ctx)
    assert not (tmp_path/'out').exists()


def test_endpoint_profile_slows_instead_of_increasing_joint_speed_limit():
    initial=trajectory_rate_proof(ROOT,.213,.005)
    assert initial['peak_joint_speed_rad_s']>.03
    selected=.005*(.03*.95)/initial['peak_joint_speed_rad_s']
    slowed=trajectory_rate_proof(ROOT,.213,selected)
    assert slowed['peak_joint_speed_rad_s']<.03
    assert slowed['duration_s']>initial['duration_s']


def test_slow_endpoint_velocity_avoids_ik_tolerance_acceleration_spikes():
    from amir_compliance_control.wrench import LocalModel
    from amir_compliance_control.stroke_kinematics import solve_stroke_pose,stroke_pose_velocity
    from amir_compliance_control.arm_translation import reference
    from amir_native_arm_support import SCENE
    model=LocalModel((ROOT/SCENE/'leader.urdf').read_text())
    q=np.array([0.,1.4,-1.6,.2,1.570796]);home,sensor=model.frames(q)
    rotation=sensor[:3,:3];total=duration(.213,.003,.0012)
    for x in np.linspace(0,reference(total-12,.213,.003,.0012)[0],100):
        q=solve_stroke_pose(model,q,home+[x,0,0],rotation)
    old=[];smooth=[]
    for t in np.arange(total-12,total+1,.008):
        x,v,_=reference(t,.213,.003,.0012)
        q=solve_stroke_pose(model,q,home+[x,0,0],rotation)
        future=solve_stroke_pose(model,q,home+[reference(t+.08,.213,.003,.0012)[0],0,0],rotation)
        old.append((future-q)/.08)
        smooth.append(stroke_pose_velocity(model,q,v))
    # The previous implementation violates the unchanged 0.15 rad/s2 guard
    # even on this exact, noise-free URDF path. The differential solve does not.
    assert np.max(abs(np.diff(old,axis=0)/.008))>.15
    assert np.max(abs(np.diff(smooth,axis=0)/.008))<.01
    assert np.max(np.abs(smooth))<.03
    np.testing.assert_allclose(smooth[-1],0.,atol=1e-14)


def test_lower_virtual_impedance_preserves_pole_and_existing_actuator_caps(tmp_path):
    module,ctx=public_context()
    ctx.launch_configurations.update(workspace_directory=str(ROOT),dual_arm_distance_m='.1',
        dual_arm_admittance_scale='.25',dual_arm_speed_m_s='.003',
        dual_arm_joint_speed_rad_s='.03',dual_arm_joint_acceleration_rad_s2='.15',
        dual_arm_acceleration_m_s2='.0012',headless='true',output_directory=str(tmp_path/'out'))
    process=next(a for a in module['setup'](ctx) if isinstance(a,ExecuteProcess))
    argv=[perform_substitutions(ctx,v) for v in process.cmd]
    mass=float(argv[argv.index('--virtual-mass-kg')+1]);damping=float(argv[argv.index('--virtual-damping-ns-m')+1])
    assert mass==.5 and damping==8.65
    assert mass/damping==pytest.approx(2/34.6)
    assert argv[argv.index('--speed')+1]=='0.003'
    assert '--follower-base-fixture' not in argv


def test_slow_simulation_can_complete_full_profile_and_hold_without_wall_deadline_false_failure():
    speed=.004374273787108695;total=duration(.213,speed,.0012);sim=[0.]
    def wait(predicate,wall_timeout,healthy):
        assert healthy is True
        # RTF=0.35 still keeps the existing sensor/clock heartbeats healthy.
        assert wall_timeout>(total+2)/.35
        for _ in range(2000):
            sim[0]+=.1
            if predicate():return
        pytest.fail('profile and hold could not complete')
    wait_for_arm_completion(wait,sim,lambda:sim[0]>=total+2,.213,speed)


def test_uncompleted_arm_mission_still_has_bounded_simulation_deadline():
    sim=[0.]
    def wait(predicate,wall_timeout,healthy):
        sim[0]=10000.
        predicate()
    with pytest.raises(RuntimeError,match='simulation-time mission budget'):
        wait_for_arm_completion(wait,sim,lambda:False,.213,.004374273787108695)


def test_gui_affinity_only_uses_spare_cpus_within_original_permission():
    from af_cpu_affinity import gui_command
    command,proof=gui_command(['ign','gazebo','-g'],{'before':[0,1,2,3],'after':[0,1]})
    assert command[-3:]==['ign','gazebo','-g']
    assert proof['gui_cpus']==[2,3] and proof['control_cpus']==[0,1]
    assert proof['watchdogs_changed'] is False
    command,proof=gui_command(['ign','gazebo','-g'],{'before':[0,1],'after':[0,1]})
    assert command==['ign','gazebo','-g']

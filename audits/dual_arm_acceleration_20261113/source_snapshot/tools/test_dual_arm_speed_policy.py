"""Two-arm limits must be checked before ROS/Gazebo can start a faster trial."""
from pathlib import Path
import numpy as np
import pytest
from launch.actions import ExecuteProcess
from launch.utilities import perform_substitutions
from dual_arm_speed_policy import validate_speed_policy,select_speed
from test_dual_arm_only import public_context

ROOT=Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('values',[(0.,.15,.0012),(float('nan'),.15,.0012),
    (1.001,.15,.0012),(.1,.501,.0012),(.1,.15,.051)])
def test_bad_policy_cannot_change_or_exceed_urdf(values):
    with pytest.raises(ValueError):validate_speed_policy(ROOT,*values)


def test_faster_profile_is_bounded_in_both_arms():
    policy,proof=select_speed(ROOT,.213,.02,.15,.15,.005)
    assert policy['selected_speed_m_s']==.02
    assert policy['actual_profile_peak_speed_m_s']==pytest.approx(.02)
    assert proof['peak_joint_speed_rad_s']<.15*.95
    assert proof['peak_joint_acceleration_rad_s2']<.15*.95
    assert all(min(v)==1. for v in policy['URDF_velocity_rad_s'].values())


def test_overspeed_is_slowed_without_exceeding_either_joint_policy():
    policy,proof=select_speed(ROOT,.213,.1,.1,.15,.01)
    assert policy['selected_speed_m_s']<.02
    assert proof['peak_joint_speed_rad_s']<=.1*.95
    assert proof['peak_joint_acceleration_rad_s2']<=.15*.95


def test_public_launch_passes_one_consistent_policy_to_existing_runner(tmp_path):
    module,ctx=public_context()
    ctx.launch_configurations.update(workspace_directory=str(ROOT),output_directory=str(tmp_path/'out'),
        dual_arm_speed_m_s='.02',dual_arm_joint_speed_rad_s='.15',dual_arm_acceleration_m_s2='.005',
        headless='true')
    process=next(a for a in module['setup'](ctx) if isinstance(a,ExecuteProcess))
    argv=[perform_substitutions(ctx,v) for v in process.cmd]
    assert float(argv[argv.index('--dual-arm-joint-speed')+1])==.15
    assert float(argv[argv.index('--dual-arm-acceleration')+1])==.005
    assert float(argv[argv.index('--speed')+1])==.02
    assert '--dual-arm-only' in argv and '--follower-base-fixture' not in argv

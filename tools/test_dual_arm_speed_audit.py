"""Speed promises must respect actual URDF and distinguish request from result."""
from pathlib import Path
import numpy as np
import pytest
from amir_dual_arm_speed_audit import assess
from test_dual_arm_only import public_context

ROOT=Path(__file__).resolve().parents[1]


@pytest.fixture(scope='module')
def metre_per_second():
    return assess(ROOT,.213,1.)


def test_one_metre_per_second_is_not_advertised_as_supported(metre_per_second):
    result=metre_per_second
    assert result['launch_accepts_requested_speed'] is False
    assert result['target_respects_URDF_velocity'] is False
    assert result['arms']['leader']['requested_peak_joint_speed_rad_s'][2]>6.
    assert result['arms']['leader']['URDF_velocity_rad_s'][2]==1.
    assert result['target_required_TCP_peak_acceleration_m_s2']>7.


def test_acceleration_and_joint_caps_are_not_silently_presented_as_one_metre_per_second(metre_per_second):
    result=metre_per_second
    assert result['current_acceleration_limited_TCP_peak_speed_m_s']<.013
    assert .0043<result['existing_guard_bounded_TCP_peak_speed_m_s']<.0045
    assert result['URDF_velocity_only_TCP_peak_ceiling_m_s']<.16
    assert result['URDF_ceiling_is_not_a_qualified_command'] is True


def test_four_millimetres_per_second_fits_preserved_path_limits():
    result=assess(ROOT,.213,.004)
    assert result['launch_accepts_requested_speed'] and result['target_respects_URDF_velocity']
    assert result['existing_guard_bounded_TCP_peak_speed_m_s']==pytest.approx(.004)
    for arm in result['arms'].values():
        assert max(arm['requested_peak_joint_speed_rad_s'])<.03
        assert max(arm['requested_peak_joint_acceleration_rad_s2'])<.15


@pytest.mark.parametrize('distance,speed',[(.214,1.),(.213,np.nan),(.213,0.)])
def test_invalid_or_unreachable_requests_are_refused(distance,speed):
    with pytest.raises(ValueError):assess(ROOT,distance,speed)


def test_public_launch_still_refuses_unqualified_one_metre_per_second(tmp_path):
    module,context=public_context()
    context.launch_configurations.update(workspace_directory=str(ROOT),dual_arm_speed_m_s='1.0',
        output_directory=str(tmp_path/'out'))
    with pytest.raises(ValueError,match='0.2'):module['setup'](context)
    assert not (tmp_path/'out').exists()

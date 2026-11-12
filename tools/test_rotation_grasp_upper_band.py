"""The close search must agree with the existing per-finger HOLDING band."""
import pytest

from amir_cp9_grasp_preparation import position_step


@pytest.mark.parametrize('forces',[[21.4406697,24.3424523],[24.,18.],[18.,24.],[22.5,23.1]])
def test_finger_above_23_cannot_stall_when_mean_is_below_23(forces):
    position=-.661569
    target=position_step(position,forces,True,True)
    assert position-.002<=target<position
    assert target-position==pytest.approx(.0001*(20-sum(forces)/2))


@pytest.mark.parametrize('forces',[[17.,23.],[23.,17.],[21.,22.],[20.,20.]])
def test_existing_valid_bilateral_holding_band_keeps_position(forces):
    assert position_step(-.66,forces,True,True)==-.66

"""Run the existing real ROS gate fault contracts on the CP-13 subclass."""
import importlib.util
from pathlib import Path

import pytest
from round_trip_safety_gate import PreciseSafetyGate


ROOT=Path(__file__).resolve().parent.parent
SPEC=importlib.util.spec_from_file_location('cp13_shared_contracts',ROOT/
    'src/humbleble_ws/amir_compliance_control/test/test_shared_safety.py')
shared=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(shared)


@pytest.fixture
def rig(tmp_path,monkeypatch):
    monkeypatch.setattr(shared,'RobotSafetyGate',PreciseSafetyGate)
    instance=shared.Rig(tmp_path)
    yield instance
    instance.close()


@pytest.mark.parametrize('fault',['missing_ft','missing_joints','nonfinite','overload',
    'joint_limit','clock_freeze','clock_reverse','supervisor_dead','planner_dead',
    'grasp_missing','grasp_detached','bad_encoder'])
def test_isolated_gate_stops_both_and_latches(rig,fault):
    shared.test_either_robot_fault_stops_both_and_latches(rig,fault)


def test_isolated_gate_manual_rearm_does_not_replay_commands(rig):
    shared.test_manual_rearm_does_not_replay_previous_commands(rig)


def test_isolated_gate_local_inputs(rig):
    shared.test_follower_safety_reads_only_local_state_and_stop_bus(rig)


@pytest.mark.parametrize('duplicate',[False,True])
def test_isolated_gate_exclusive_command_writers(rig,duplicate):
    shared.test_arming_requires_exclusive_real_command_writers(rig,duplicate)

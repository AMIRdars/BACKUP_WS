"""Opt-in predicates exercise real guards without moving physical joints/base."""
import numpy as np
import pytest
from std_srvs.srv import Trigger
from test_strict_arm_only import AFRig


@pytest.mark.parametrize(('fault','reason'),[
    ('joint_margin','joint software margin'),('singularity','XY task singularity'),
    ('base_drift','measured base drift/speed'),('dt','timestep outside watchdog')])
def test_validation_fault_retains_measurements_and_stops_both(tmp_path,fault,reason):
    rig=AFRig(tmp_path)
    try:
        rig.start();end=rig.ends['follower']
        assert end.enable(Trigger.Request(),Trigger.Response()).success
        rig.pump(.08)
        q=end.measured_q().copy();pose=end.base_pose().copy()
        assert end.inject_test_fault(Trigger.Request(),Trigger.Response(),fault=fault).success
        rig.pump(.3)
        assert reason in end.reason
        np.testing.assert_array_equal(end.measured_q(),q)
        np.testing.assert_array_equal(end.base_pose(),pose)
        assert all(e.state=='SAFE_STOP' for e in rig.ends.values())
        assert rig.output=={'leader':0.,'follower':0.}
        assert end.fault_debug['simulation_fault_injection']==fault
        assert not end.enable(Trigger.Request(),Trigger.Response()).success
        assert not end.test_fault_services  # production default exposes no hooks
    finally:rig.close()

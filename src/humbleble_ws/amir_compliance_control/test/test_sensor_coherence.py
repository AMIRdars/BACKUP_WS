import pytest
from amir_compliance_control.sensor_coherence import coherent_stamps


def select(h,sim=1.,wall=10.):
    return coherent_stamps(h,('q','load'),sim,wall,.05,.02,.15)


def test_aligns_different_rate_inputs_within_original_age_and_skew():
    h={'q':[(.96,9.91),(.98,9.95),(1.,10.)],'load':[(.968,9.99)]}
    assert select(h)=={'q':.98,'load':.968}


def test_latest_state_qos_does_not_mutate_the_legacy_sensor_profile():
    from amir_compliance_control.sensor_coherence import local_sensor_qos
    from rclpy.qos import qos_profile_sensor_data,ReliabilityPolicy
    depth=qos_profile_sensor_data.depth
    latest=local_sensor_qos(True)
    assert latest.depth==1 and latest.reliability==ReliabilityPolicy.BEST_EFFORT
    assert local_sensor_qos(False) is qos_profile_sensor_data and qos_profile_sensor_data.depth==depth


@pytest.mark.parametrize('history',[
    {'q':[(1.,10.)],'load':[(.94,9.99)]},  # source older than 50 ms
    {'q':[(1.,10.)],'load':[(.97,9.8)]},   # receipt older than heartbeat
    {'q':[(1.03,10.)],'load':[(.97,9.99)]}, # future source
    {'q':[(1.,10.)],'load':[(.97,9.99)]},  # no coherent pair in stored history
    {'q':[(1.,10.)],'load':[]},
])
def test_no_valid_snapshot_is_rejected(history):
    assert select(history) is None

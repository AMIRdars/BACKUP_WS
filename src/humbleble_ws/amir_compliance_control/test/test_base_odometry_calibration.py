import copy
import hashlib
import json
from pathlib import Path
import pytest
from nav_msgs.msg import Odometry
from amir_compliance_control.base_odometry_calibration import validate_distance_calibration,scaled_odometry

ROOT=Path(__file__).resolve().parents[4]


def measured():
    return json.loads((ROOT/'src/humbleble_ws/amir_compliance_control/config/native_leader_odometry.json').read_text())


def test_measured_scale_matches_the_actual_qualified_scene():
    cert=measured();folder=ROOT/'audits/arm_follow_resume_20261111/paused_050_gui/trial/generated'
    scale=validate_distance_calibration(cert,folder)
    assert scale==pytest.approx(0.9917571679608245)


@pytest.mark.parametrize('change',['model','ratio','unverified','one_trial','repeat','speed'])
def test_invalid_or_mismatched_calibration_cannot_authorize_correction(tmp_path,change):
    cert=measured()
    for name in cert['artifact_sha256']:
        (tmp_path/name).write_text(name);cert['artifact_sha256'][name]=hashlib.sha256((tmp_path/name).read_bytes()).hexdigest()
    if change=='model':(tmp_path/'world.sdf').write_text('different physics')
    if change=='ratio':cert['metres_per_raw_odom_metre']=1.
    if change=='unverified':cert['samples'][0]['transport_checks_passed']=False
    if change=='one_trial':cert['samples']=cert['samples'][:1]
    if change=='repeat':cert['samples'][1]['case']=cert['samples'][0]['case']
    if change=='speed':cert['raw_command_cap_m_s']=.003
    with pytest.raises(ValueError):validate_distance_calibration(cert,tmp_path)


def test_private_distance_estimate_preserves_raw_message_frame_stamp_and_zero_motion():
    raw=Odometry();raw.header.frame_id='leader/odom';raw.header.stamp.sec=10
    raw.pose.pose.position.x=2.;raw.twist.twist.linear.x=.005;raw.pose.covariance[0]=4.
    untouched=copy.deepcopy(raw);mapped=scaled_odometry(raw,.99)
    assert raw==untouched
    assert mapped.header==raw.header and mapped.pose.pose.orientation==raw.pose.pose.orientation
    assert mapped.pose.pose.position.x==pytest.approx(1.98)
    assert mapped.twist.twist.linear.x==pytest.approx(.00495)
    assert mapped.pose.covariance[0]==pytest.approx(4*.99**2)

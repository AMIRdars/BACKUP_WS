import json
import pytest
from amir_native_distance_calibration import calibrate


def cases(root):
    result=[]
    for i,gt in enumerate((.49598,.499994)):
        case=root/str(i);folder=case/'generated';folder.mkdir(parents=True)
        for name in ('leader.urdf','leader_controllers.yaml','leader_base_controller.yaml','world.sdf','base_fixture_proof.json'):
            (folder/name).write_text(name)
        (case/'p0_evidence.json').write_text(json.dumps({'passed':False,
            'checks':{'grasp_held_after_stop':True,'calibrated_leader_actual_world_x_reaches_requested_distance':False},
            'leader_distance_m':.504,'leader_world_base_displacement_xyz_m':[gt,0.,0.]}))
        (case/'held_result.json').write_text(json.dumps({'passed':True,'safe_stop_verified':True,
            'grasp_held_after_stop':True,'native_arm_stroke':True,'follower_base_fixture':True,
            'leader_velocity_cap_m_s':.005,'requested_distance_m':.5}))
        result.append(case)
    return result


def test_distance_miss_is_a_measurement_and_reserve_is_observed_not_guessed(tmp_path):
    proof=calibrate(cases(tmp_path),tmp_path)
    assert all(not s['target_distance_reached'] for s in proof['samples'])
    assert proof['metres_per_raw_odom_metre']<min(proof['sample_scale_range'])
    assert proof['empirical_scale_reserve']==pytest.approx(.004014/.504)


def test_failed_grasp_is_never_accepted_as_distance_calibration(tmp_path):
    samples=cases(tmp_path);p=samples[1]/'p0_evidence.json';value=json.loads(p.read_text())
    value['checks']['grasp_held_after_stop']=False;p.write_text(json.dumps(value))
    with pytest.raises(ValueError,match='completed native'):calibrate(samples,tmp_path)

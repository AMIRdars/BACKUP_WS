"""Native Gate-0 never authorizes autonomous yaw or hides a raw F/T event."""
import copy,json
from pathlib import Path
import numpy as np
import pytest
from amir_rotation_native_static import NativeStaticYaw
from amir_compliance_control.rotation_observer import ProjectedYaw

ROOT=Path(__file__).resolve().parents[1]
MODEL=ROOT/'audits/object_center_rotation_20261103/cp8r3/native_generated/follower.urdf'
PROOF=ROOT/'audits/object_center_rotation_20261103/cp8r3/native_smoke005_08/result.json'
# Do not depend on JSON mapping insertion order for the actual arm chain.
MAPPING=json.loads(PROOF.read_text())['preparation_final']['follower']['home_from_actual_urdf']
HOME=[MAPPING[f'Joint_{i}'] for i in range(1,6)]


def auditor():
 g=ProjectedYaw(MODEL.read_text(),orientation_envelope_rad=.01,joint_margin_rad=.01,heading_frame='grasp_heading')
 return NativeStaticYaw(g,force_limit_N=.5,torque_limit_Nm=.05)


def row(i=0):
 t=1.+i*.01
 return dict(timestamp=t,joint_stamp=t,wrench_stamp=t,odom_stamp=t,q_rad=HOME.copy(),qdot_rad_s=[0.]*5,
  wrench_tcp=[1.,-2.,-3.,.1,.2,-.3],base_twist=[0.]*3,grasp_attached=True)


def test_static_noise_is_measured_without_issuing_a_yaw_certificate():
 a=auditor()
 for i in range(32):
  x=row(i);x['q_rad'][0]+=np.sin(i)*1e-7;a.add(x)
 result=a.finish()
 assert result['gate0_static_passed'] is True
 assert result['autonomous_yaw_qualification_issued'] is False
 assert result['yaw_std_rad']>1e-8 and result['samples']==32
 assert 'positive/negative' in result['gate1_gate2']
 assert a.geometry.model.names==tuple(f'Joint_{i}' for i in range(1,6))


def test_even_duplicate_joint_snapshot_cannot_hide_a_raw_ft_excursion():
 a=auditor();a.add(row());x=row();x['timestamp']+=.002;x['wrench_stamp']+=.002;x['wrench_tcp'][5]+=.051
 with pytest.raises(ValueError,match='wrench envelope exceeded'):a.add(x)
 assert a.maximum_moment>.05


@pytest.mark.parametrize('change,reason',[
 ({'wrench_stamp':.94},'stale/future/skewed'),({'odom_stamp':1.03},'stale/future/skewed'),
 ({'base_twist':[0.,.0001,0.]},'stationary'),({'qdot_rad_s':[.001,0.,0.,0.,0.]},'stationary'),
 ({'grasp_attached':False},'grasp not retained'),({'wrench_tcp':[float('nan')]*6},'nonfinite'),
])
def test_static_audit_refuses_unhealthy_own_measurements(change,reason):
 a=auditor();x=row();x.update(change)
 with pytest.raises(ValueError,match=reason):a.add(x)


def test_duplicate_joint_stamp_with_changed_position_is_a_discontinuity():
 a=auditor();a.add(row());x=row();x['q_rad'][0]+=1e-8
 with pytest.raises(ValueError,match='discontinuity'):a.add(x)


def test_insufficient_static_samples_cannot_pass():
 a=auditor();a.add(row())
 with pytest.raises(ValueError,match='eight'):a.finish()


def test_reported_zero_velocity_cannot_hide_a_joint_position_jump():
 a=auditor();a.add(row());x=row(1);x['q_rad'][0]+=.001
 with pytest.raises(ValueError,match='contradicts stationary'):a.add(x)


@pytest.mark.parametrize('override',[['--force-memory-gain','.1'],['--static-observe-s','-1']])
def test_static_cli_refuses_unsafe_options_before_creating_any_output(tmp_path,override):
 import subprocess,sys
 output=tmp_path/'must_not_exist'
 args=[sys.executable,str(ROOT/'tools/amir_cp8r3_native_trial.py'),'--run-directory',str(tmp_path/'fixture'),
  '--output',str(output),'--native-metrology-directory',str(tmp_path/'source'),'--wrench-calibration-file',str(tmp_path/'cert.json'),
  '--delta-deg','.005','--duration-s','75','--force-memory-gain','0','--force-velocity-gain','0',
  '--arm-yaw-metric','.4','--static-yaw-audit',*override]
 result=subprocess.run(args,capture_output=True,text=True)
 assert result.returncode!=0 and 'static native audit requires' in result.stderr
 assert not output.exists()


def test_csv_retains_duplicate_raw_events_and_the_terminal_fault(tmp_path):
 import csv
 from amir_rotation_native_static import export_csv
 first=row();second=row();second['wrench_tcp'][5]+=.051
 source=tmp_path/'raw.jsonl';source.write_text('\n'.join(json.dumps(x) for x in [first,second,{'timestamp':1.,'fault':'load excursion'}])+'\n')
 destination=tmp_path/'raw.csv'
 assert export_csv(source,destination,auditor().geometry)==3
 result=list(csv.DictReader(destination.open()))
 assert result[0]['joint_stamp']==result[1]['joint_stamp']
 assert float(result[1]['tz_Nm'])-float(result[0]['tz_Nm'])==pytest.approx(.051)
 assert result[2]['fault']=='load excursion'

#!/usr/bin/env python3
"""Verify simulation evidence, unchanged original sources and explicit scope."""
import argparse
import ast
import hashlib
import json
import math
from pathlib import Path
import re
import subprocess
import xml.etree.ElementTree as ET


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--audit-directory',type=Path,required=True)
    p.add_argument('--negative',required=True);p.add_argument('--hold',required=True)
    p.add_argument('--faults',nargs='*',default=[]);a=p.parse_args();root=a.audit_directory
    read=lambda p:json.loads(p.read_text());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest();checks={}
    before=read(root/'before/source_hashes.json')
    checks['all_preexisting_source_files_unchanged']=len(before)==564 and all(Path(f).exists() and sha(Path(f))==h for f,h in before.items())
    checks['preexisting_tracked_diff_preserved']=subprocess.check_output(['git','diff','--binary'])==(root/'before/tracked.patch').read_bytes()
    original=[r for r in read(root.parent/'cp0/source_files.json') if Path(r['path']).suffix in ('.urdf','.xacro','.sdf','.yaml')]
    checks['original_105_model_and_yaml_hashes_preserved']=len(original)==105 and all(sha(Path(v['path']))==v['sha256'] for v in original)
    cp8=read(root.parent/'cp8/final_checks.json')
    checks['cp8_yaw_pivot_not_promoted']=not cp8['all_five_cooperative_motions_passed'] and set(cp8['remaining_unvalidated_cooperative_motions'])=={'yaw','pivot'}
    checks['hardware_explicitly_skipped_by_user']=read(root/'scope.json')['hardware_scope']=='SKIPPED_BY_USER'
    code=ast.parse(Path('tools/amir_cp9_local_grasp.py').read_text())
    imports={n.module.split('.')[0] for n in ast.walk(code) if isinstance(n,ast.ImportFrom) and n.module}
    checks['local_grasp_has_no_gazebo_tf_or_evaluation_import']=not imports.intersection({'ros_gz_interfaces','tf2_msgs','tools','gazebo_msgs'})
    selected=[a.negative,a.hold,*a.faults];results={}
    for name in selected:
        folder=root/name;r=read(folder/'result.json');results[name]=r
        checks[name+'_passed']=r['passed']
        checks[name+'_current_source']=all(sha(Path(f))==h for f,h in read(folder/'source_hashes.json').items())
        world=ET.parse(folder/'generated/world.sdf').getroot();plugins=[v.get('name','') for v in world.findall('.//plugin')]
        checks[name+'_native_contact_only']=set(v.rsplit('::',1)[-1] for v in plugins)=={'Physics','UserCommands','SceneBroadcaster','Contact','Sensors','ForceTorque','DartContactParameters'} and not world.findall('.//joint')
        robot_models=[ET.parse(folder/'generated'/(robot+'.urdf')).getroot() for robot in ('leader','follower')]
        checks[name+'_robot_plugins_no_grasp_assistance']=all({v.get('name') for v in model.findall('gazebo/plugin')}=={'gz_ros2_control::GazeboSimROS2ControlPlugin','ignition::gazebo::systems::OdometryPublisher'} and all(v.text=='gz_ros2_control/GazeboSimSystem' for v in model.findall('ros2_control/hardware/plugin')) for model in robot_models)
        samples=[json.loads(l) for l in (folder/'samples.jsonl').read_text().splitlines()]
        checks[name+'_saved_gt_finite_and_unit_quaternions']=bool(samples) and all(all(math.isfinite(x) for x in v['payload_xyz']+v['payload_quaternion_xyzw']) and abs(sum(x*x for x in v['payload_quaternion_xyzw'])-1)<1e-6 for v in samples)
        checks[name+'_payload_mass_and_friction_from_existing_model']=float(world.findtext("world/model[@name='cooperative_payload']/link/inertial/mass"))==.5 and float(world.findtext("world/model[@name='cooperative_payload']/link/collision/surface/friction/ode/mu"))==10
        checks[name+'_existing_contact_solver_policy']=float(world.findtext("world/plugin[@name='cooperative_transport::DartContactParameters']/constraint_force_mixing"))==.05 and float(world.findtext('world/physics/max_step_size'))==.004
        initialization=read(folder/'generated/mimic_initialization.json')
        checks[name+'_model_consistent_mimic_initial_values']=all(len(initialization[r])==5 and all(v['master']=='Gripper' and v['initial_position_rad']==-1. for v in initialization[r].values()) for r in ('leader','follower'))
        observation=read(folder/'observation/summary.json')
        checks[name+'_read_only_observer_no_hardware_claim']=len(observation['topics'])==6 and all(v['samples']>0 and not v['hardware_qualified'] and not v['end_to_end_actuator_latency_measured'] for v in observation['topics'].values()) and all(v[0]=='/parameter_events' for v in observation['own_publishers'])
        if name==a.negative:
            checks[name+'_loss_of_support_detected']=r['negative_control_detected_loss_of_support'] and r['actuator_goals_sent']==0
        else:
            checks[name+'_unsupported_ten_second_hold']=r.get('unsupported_static_hold_verified',False) and r['unsupported_max_position_drift_m']<.001 and r['unsupported_max_angle_drift_rad']<.01
            checks[name+'_local_input_boundary']=r['local_grasp_input_boundary_passed']
            checks[name+'_running_controller_matches_current_source']=all(v['source_sha256']==sha(Path('tools/amir_cp9_local_grasp.py')) for v in r['grasp_before_support_removal'].values())
            checks[name+'_no_transport_qualification_claim']=not r['cooperative_contact_transport_verified']
            placement=[json.loads(l) for l in (folder/'fixture_placement.jsonl').read_text().splitlines()]
            checks[name+'_only_initial_fixture_placement']=len(placement)==2 and {v['robot'] for v in placement}=={'leader','follower'} and all(v['phase']=='fixture_placement_after_home' and set(v['grasp_states'].values())=={'READY_TO_CLOSE'} for v in placement)
        if name in a.faults:
            checks[name+'_fault_injected_after_hold']=r.get('fault_injected',False) and r['fault'] in ('contacts','nonfinite','mimic','overload')
            checks[name+'_both_base_arm_inactive_no_gripper_restart']=r['base_arm_inactive_after_fault'] and r['no_automatic_gripper_restart'] and r['both_local_grippers_fault_latency_wall_s']<=1.
    checks['four_distinct_stationary_faults_verified']=len(a.faults)==4 and {results[n]['fault'] for n in a.faults}=={'contacts','nonfinite','mimic','overload'}
    tests=(root/'unit_tests.log').read_text();match=re.findall(r'(\d+) passed',tests)
    checks['new_measurement_and_preparation_tests_pass']=bool(match) and int(match[-1])>=45 and ' failed' not in tests
    checks['launch_package_build_passed']='1 package finished' in (root/'build.log').read_text()
    checks['offline_plot_and_inventory_saved']=all((root/f).exists() for f in ('contact_grasp.png','experiments.csv','experiment_inventory.json'))
    remaining=[]
    for proc in Path('/proc').iterdir():
        try:
            if proc.name.isdigit() and any(v.startswith(b'IGN_PARTITION=amir_cp9_') for v in (proc/'environ').read_bytes().split(b'\0')):remaining.append(int(proc.name))
        except OSError:pass
    checks['all_owned_simulation_processes_stopped']=not remaining
    report={'checkpoint':'CP-9 simulation contact assessment','date_jst':'2026-10-20','checks':checks,'all_checks_passed':all(checks.values()),
        'hardware_scope':'SKIPPED_BY_USER','hardware_motion_gate_open':False,
        'cooperative_contact_transport_verified':False,'native_static_hold_verified':results[a.hold].get('unsupported_static_hold_verified',False),
        'selected_trials':selected,'tests':int(match[-1]) if match else 0,'remaining_owned_pids':remaining,
        'remaining':['integrate native local grasp into guarded whole-body transport with matching calibration/profiles',
            'cooperative contact transport and moving fault tests','CP-8 cooperative yaw/pivot convergence']}
    (root/'final_checks.json').write_text(json.dumps(report,indent=2)+'\n')
    paths=[*Path('tools').glob('*cp9*.py'),Path('src/humbleble_ws/cooperative_transport_bringup/launch/compliance_cp9_contact.launch.py'),root/'README.md',root/'scope.json',root.parent/'README.md']
    (root/'source_manifest.json').write_text(json.dumps({str(p):sha(p) for p in paths if p.exists()},indent=2)+'\n')
    print(json.dumps(report,indent=2))
    if not all(checks.values()):raise SystemExit(1)


if __name__=='__main__':main()

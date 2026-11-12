#!/usr/bin/env python3
"""Offline CP-6 completion gates, source hashes and preservation checks."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--audit-directory', type=Path, required=True)
    p = parser.parse_args().audit_directory
    sha = lambda data: hashlib.sha256(data).hexdigest()
    read = lambda path: json.loads(path.read_text())
    before, checks = read(p/'before.json'), {}
    file = Path('src/humbleble_ws/amir_compliance_control/amir_compliance_control/safety.py')
    normalized = file.read_text().replace(
        "raise ValueError('stale or invalid arm request header: '\n"
        "                    f'frame={msg.header.frame_id!r}, stamp={stamp(msg):.9f}, '\n"
        "                    f'armed={self.armed_sim:.9f}, now={self.sim:.9f}')",
        "raise ValueError('stale or invalid arm request header')")
    checks['safety_acceptance_unchanged_only_diagnostics_added'] = sha(normalized.encode()) == before[str(file)]
    file = Path('src/humbleble_ws/amir_compliance_control/amir_compliance_control/wrench_preprocessor.py')
    checks['wrench_preprocessor_unchanged'] = sha(file.read_bytes()) == before[str(file)]
    checks['preexisting_tracked_diff_unchanged'] = subprocess.check_output(['git','diff']) == (p/'preexisting.patch').read_bytes()
    originals = [r for r in read(p.parent/'cp0/source_files.json')
                 if Path(r['path']).suffix in ('.urdf','.xacro','.sdf','.yaml')]
    checks['original_105_models_and_yaml_unchanged'] = len(originals)==105 and all(
        sha(Path(r['path']).read_bytes())==r['sha256'] for r in originals)
    cal = read(p/'calibration/calibration.json')
    checks['calibration_all_6_axes_12_directions'] = (cal['sensor_calibration_passed']
        and len(cal['verified_axes'])==6 and len(cal['tests'])==12 and all(t['passed'] for t in cal['tests']))
    checks['calibration_matches_urdf'] = cal['urdf_sha256']==sha((p/'generated/follower.urdf').read_bytes())
    checks['static_controllers_ready'] = read(p/'static_proof/runtime.json')['controllers_ready']
    for robot in ('leader','follower'):
        profile = read(p/'generated'/(robot+'_safety.json'))
        checks[robot+'_profile_hashes_match'] = (
            profile['urdf_sha256']==sha((p/'generated'/(robot+'.urdf')).read_bytes())
            and profile['world_sha256']==sha((p/'generated/world.sdf').read_bytes())
            and all(profile[k+'_sha256']==sha(Path(profile[k+'_file']).read_bytes())
                    for k in ('controller_yaml','base_controller_yaml')))
        checks[robot+'_normal_profile_requires_grasp'] = profile['require_grasp_for_base_motion'] and not profile['ungrasped_base_test_only']
    checks['all_126_tests_passed'] = '126 passed' in (p/'all_tests_final.log').read_text()
    checks['safety_16_tests_after_diagnostics'] = '16 passed' in (p/'safety_diagnostics_tests.log').read_text()
    checks['final_build_2_packages'] = '2 packages finished' in (p/'build_final.log').read_text()
    for item in read(p/'comparison.json'):
        name = item['trial']
        result = read(p/name/'result.json')
        checks[name+'_passed_and_boundary'] = item['passed'] and item['follower_boundary_passed']
        policy = read(p/name/'control_policy.json') if name!='grasp_static' else None
        if policy:
            checks[name+'_final_virtual_tuning'] = policy['virtual_mass']==[20,20] and policy['virtual_damping']==[1200,1200]
        if result['condition']=='transport':
            checks[name+'_leader_only_and_no_excitation'] = (result['endpoint_states']['leader']['accepted_goals']==1
                and result['endpoint_states']['follower']['accepted_goals']==0 and item['zero_external_excitation'])
            checks[name+'_grasp_required_and_orientation_locked'] = (all(read(p/name/'profiles_used'/(r+'_safety.json'))[
                'require_grasp_for_base_motion'] for r in ('leader','follower')) and policy['preserve_orientation']
                and policy['straight_only'] and not policy['allow_ungrasped_test'])
        if result['fault_test']!='none':
            checks[name+'_moving_at_injection'] = all(abs(result['twists_at_injection'][r][0])>5e-6 for r in ('leader','follower'))
            checks[name+'_no_restart_hold_and_stopped'] = (result['no_automatic_restart']
                and max(result['post_fault_joint_span_rad'])<1e-5
                and item['post_fault_max_encoder_translation_m_s']<1e-6 and item['post_fault_max_encoder_wz_rad_s']<1e-5)
    signs, bands = read(p/'signs.json'), read(p/'bands.json')
    checks['six_signed_gt_directions'] = len(signs)==6 and all(r['ground_truth_passed'] for r in signs)
    checks['band_separation'] = bands[0]['measured_lowpass_gain']>.9 and bands[1]['measured_lowpass_gain']<.11
    reg = read(p/'cp5_regression/result.json')
    checks['cp5_runtime_regression_passed'] = (reg['passed'] and reg['saturation_observed']
        and len(reg['direction_tests'])==4 and all(t['passed'] for t in reg['direction_tests']))
    outs = [json.loads(line) for line in (p/'cp5_regression/command_outputs.jsonl').open()]
    checks['cp5_runtime_all_base_axes_zero'] = bool(outs) and all(
        max(abs(r[k]) for k in ('vx','vy','wz'))<1e-12 for r in outs)
    remaining = []
    for process in Path('/proc').iterdir():
        if not process.name.isdigit(): continue
        try:
            if any(e.startswith(b'IGN_PARTITION=amir_cp6_') for e in (process/'environ').read_bytes().split(b'\0')):
                remaining.append(int(process.name))
        except OSError: pass
    checks['all_cp6_owned_processes_stopped'] = not remaining
    source_files = [*Path('src/humbleble_ws/amir_compliance_control').rglob('*.py'),
        Path('src/humbleble_ws/amir_compliance_control/package.xml'),
        Path('src/humbleble_ws/amir_compliance_control/README.md'),
        Path('src/humbleble_ws/cooperative_transport_bringup/launch/compliance_cp6_fixture.launch.py'),
        *[Path('tools')/name for name in ('amir_cp6_trial.py','amir_cp6_analyze.py','amir_cp6_verify.py','amir_make_safety_profiles.py')],
        p/'README.md', p.parent/'README.md']
    (p/'source_manifest.json').write_text(json.dumps({str(f):sha(f.read_bytes()) for f in sorted(set(source_files))},indent=2)+'\n')
    report = {'checkpoint':'CP-6','continued_date_jst':'2026-10-20',
        'original_model_and_yaml_files_checked':len(originals),'checks':checks,
        'all_passed':all(checks.values()),'remaining_owned_pids':remaining,
        'unresolved':['One initial-home timestamp rejection before injection: root cause unconfirmed; fail-closed behavior retained, diagnostics added.',
            'CP-7 delay/comparison/full-fault coverage, CP-8 full-body/lateral/yaw, CP-9 physical ratings/grip safety remain open.']}
    (p/'final_checks.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({'checks':len(checks),'all_passed':report['all_passed'],
                      'failed':[k for k,v in checks.items() if not v]},indent=2))
    if not report['all_passed']: raise SystemExit(1)


if __name__=='__main__':
    main()

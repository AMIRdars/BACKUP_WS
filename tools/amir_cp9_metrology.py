#!/usr/bin/env python3
"""Run the separate six-axis excitation and static reaction proof, then exit.

Generated robot files are reused at exactly the same paths for native trials.
Only the metrology world has CalibrationWrench; it is archived before shutdown.
"""
import argparse
import hashlib
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import time


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--run-directory', type=Path, required=True)
    a = p.parse_args()
    if os.getenv('ROS_LOCALHOST_ONLY') != '1' or not os.getenv('IGN_PARTITION', '').startswith('amir_cp9_'):
        raise ValueError('isolated CP-9 simulation required')
    a.output = a.output.resolve(); a.output.mkdir(parents=True, exist_ok=False)
    a.run_directory = a.run_directory.resolve()
    a.run_directory.mkdir(parents=True, exist_ok=True)
    scene_lock = (a.run_directory/'.cp9_scene.lock').open('a')
    fcntl.flock(scene_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    finish = a.output/'finished'
    argv = ['/usr/bin/python3', 'tools/amir_cp_audit.py', '--output', str(a.output/'static_proof'),
        '--robots', 'leader', 'follower', '--joint-loads', '--timeout', '240', '--finish-file', str(finish),
        '--', 'ros2', 'launch', 'cooperative_transport_bringup', 'compliance_cp9_metrology.launch.py',
        'run_directory:='+str(a.run_directory)]
    (a.output/'command.json').write_text(json.dumps({'audit_argv': argv,
        **{k: os.getenv(k) for k in ('ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY', 'IGN_PARTITION')}}, indent=2))
    log = (a.output/'audit.log').open('w')
    process = subprocess.Popen(argv, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    passed = False
    try:
        end = time.monotonic()+80
        while not (a.run_directory/'follower.urdf').exists():
            if process.poll() is not None or time.monotonic() > end:
                raise RuntimeError('metrology generation failed')
            time.sleep(.1)
        with (a.output/'calibration.log').open('w') as out:
            result = subprocess.run(['/usr/bin/python3', 'tools/amir_cp2_probe.py', '--urdf',
                str(a.run_directory/'follower.urdf'), '--output', str(a.output/'calibration'),
                '--finish-file', str(finish)], stdout=out, stderr=subprocess.STDOUT, timeout=180)
        if result.returncode:
            raise RuntimeError('six-axis identification failed')
        process.wait(timeout=35)
        if process.returncode:
            raise RuntimeError('static reaction audit failed')
        archive = a.output/'generated'; archive.mkdir()
        hashes = {}
        for f in a.run_directory.iterdir():
            if f.is_file():
                (archive/f.name).write_bytes(f.read_bytes())
                hashes[f.name] = hashlib.sha256(f.read_bytes()).hexdigest()
        (a.output/'generated_hashes.json').write_text(json.dumps(hashes, indent=2))
        runtime = json.loads((a.output/'static_proof/runtime.json').read_text())
        passed = (runtime['controllers_ready'] and json.loads(
            (a.output/'calibration/calibration.json').read_text())['sensor_calibration_passed']
            and all(runtime['counts'].get('/'+r+'/joint_load/Joint_'+str(i), 0) >= 100
                for r in ('leader', 'follower') for i in range(1, 5)))
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try: process.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL); process.wait()
        log.close()
        (a.output/'result.json').write_text(json.dumps({'passed': passed, 'hardware_motion_gate_open': False}, indent=2))
    if not passed:
        raise SystemExit(1)


if __name__ == '__main__':
    main()

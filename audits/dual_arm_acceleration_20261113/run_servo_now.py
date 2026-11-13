"""Qualify the current-knot servo-model validator before the upper trial."""
import json
from pathlib import Path
import subprocess
import time

ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).resolve().parent
first=OUT/'servo_now_050/trial/result.json'
deadline=time.monotonic()+900
while not first.exists():
    if time.monotonic()>deadline:raise RuntimeError('current-knot 50mm/s trial missing')
    time.sleep(1.)
def qualify(name):
    folder=OUT/name/'trial'
    with (OUT/name/'evidence.log').open('w') as stream:
        subprocess.run(['/usr/bin/python3','tools/amir_dual_arm_evidence.py','--case',str(folder)],cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT)
    e=json.loads((folder/'dual_arm_evidence.json').read_text())
    passed=all(v for k,v in e['checks'].items() if not k.startswith('native_force_band:'))
    print(json.dumps({'case':name,'transport_integrity_passed':passed,'strict_passed':e['passed'],'failed':[k for k,v in e['checks'].items() if not v]}),flush=True)
    if not passed:raise RuntimeError('current-knot transport qualification failed: '+name)
qualify('servo_now_050')
cmd=['ros2','launch','cooperative_transport_bringup','compliance_round_trip.launch.py','headless:=true',
     'keep_gui_open:=false','start_delay_s:=0','ros_domain_id:=216','dual_arm_speed_m_s:=0.0536',
     'dual_arm_joint_speed_rad_s:=0.75','dual_arm_joint_acceleration_rad_s2:=0.5','dual_arm_acceleration_m_s2:=0.05',
     'output_directory:='+str(OUT/'servo_now_0536')]
with (OUT/'servo_now_0536_console.log').open('w') as stream:subprocess.run(cmd,cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT)
qualify('servo_now_0536')

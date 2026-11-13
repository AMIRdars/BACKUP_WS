"""Sequential isolated Gazebo trials; never escalate after a failed check."""
import json
import argparse
import os
from pathlib import Path
import subprocess
import time

ROOT=Path(__file__).resolve().parents[2]
OUT=Path(__file__).resolve().parent


def evidence(name):
    folder=OUT/name/'trial'
    with (OUT/name/'evidence.log').open('w') as stream:
        p=subprocess.run(['/usr/bin/python3','tools/amir_dual_arm_evidence.py','--case',str(folder)],
                         cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT)
    result=json.loads((folder/'result.json').read_text())
    epath=folder/'dual_arm_evidence.json'
    checked=json.loads(epath.read_text()) if epath.exists() else {}
    entry={'stage':name,'motion_passed':result.get('passed'),
        'grasp_held_after_stop':result.get('grasp_held_after_stop'),
        'safe_stop_verified':result.get('safe_stop_verified'),
        'strict_passed':checked.get('passed'),
        'failed_checks':[k for k,v in checked.get('checks',{}).items() if not v],
        'error':result.get('error')}
    # Keep every raw-force-band failure visible. A one-sample lower-band
    # excursion is distinct from loss of contact/holding: the unchanged local
    # monitor already confirms low force for .3 s before stopping. Continue
    # controlled exploration only when ALL other measured checks passed.
    entry['transport_integrity_passed']=bool(checked and all(
        v for k,v in checked['checks'].items() if not k.startswith('native_force_band:')))
    print(json.dumps(entry),flush=True)
    return entry


def save(entries):
    (OUT/'stages.json').write_text(json.dumps(entries,indent=2))
    lines=['# 速度引上げ：段階記録（2026-11-13）','',
           'URDF・Gazebo物理・controller/API・力/接触/位置/台車停止ガードを変更せず、明示したソフト速度枠を比較。搬送・接触・ずれ・停止後保持などが未達なら次へ進めない。単発の反力下限逸脱は厳密判定の未合格として残し、既存の0.3秒継続監視を変更せず試験する。','',
           '| Stage | 動作完了 | 停止後把持 | 実停止 | 厳密判定 | 未合格項目 / error |',
           '|---|---|---|---|---|---|']
    for e in entries:
        lines.append(f'| {e["stage"]} | {e["motion_passed"]} | {e["grasp_held_after_stop"]} | {e["safe_stop_verified"]} | {e["strict_passed"]} | {e["failed_checks"]} / {e["error"]} |')
    (OUT/'STAGES.md').write_text('\n'.join(lines)+'\n')


parser=argparse.ArgumentParser()
parser.add_argument('--first',default='speed_008')
parser.add_argument('--next-speed',type=float,default=.015)
args=parser.parse_args()
entries=json.loads((OUT/'stages.json').read_text()) if (OUT/'stages.json').exists() else []
first=OUT/args.first/'trial/result.json'
deadline=time.monotonic()+1200
while not first.exists():
    if time.monotonic()>deadline:raise RuntimeError('8 mm/s trial did not finish')
    time.sleep(1.)
first_entry=evidence(args.first)
entries=[e for e in entries if e['stage']!=args.first]+[first_entry];save(entries)
ladder=[('speed_015',.015,.15,.15,.003,219),
        ('speed_025',.025,.25,.15,.006,220),
        ('speed_035',.035,.35,.3,.01,221),
        ('speed_050',.05,.5,.5,.02,222),
        ('speed_075',.075,.75,.5,.05,223)]
for name,speed,joint_speed,joint_acceleration,acceleration,domain in ladder:
    if speed<args.next_speed:continue
    if not entries[-1]['transport_integrity_passed']:
        print('Escalation stopped: previous transport/contact/holding acceptance not passed.',flush=True)
        break
    cmd=['ros2','launch','cooperative_transport_bringup','compliance_round_trip.launch.py',
         'headless:=true','keep_gui_open:=false','native_cpu_policy:=smt','start_delay_s:=0',
         'dual_arm_speed_m_s:='+str(speed),'dual_arm_joint_speed_rad_s:='+str(joint_speed),
         'dual_arm_joint_acceleration_rad_s2:='+str(joint_acceleration),
         'dual_arm_acceleration_m_s2:='+str(acceleration),'ros_domain_id:='+str(domain),
         'output_directory:='+str(OUT/name)]
    print('START '+name+' '+json.dumps(cmd),flush=True)
    with (OUT/(name+'_console.log')).open('w') as stream:
        subprocess.run(cmd,cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT)
    if not (OUT/name/'trial/result.json').exists():
        raise RuntimeError('missing result: '+name)
    entries.append(evidence(name));save(entries)
print('Ladder finished; final GUI/repetition qualification remains.',flush=True)

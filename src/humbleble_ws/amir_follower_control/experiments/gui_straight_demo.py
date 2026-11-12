"""Interactive GUI for the verified 0.5 kg, 1 mm/s straight trial.

Run in a terminal after sourcing this workspace. Enter starts one bounded
trial; q or closing Gazebo stops this session. No automatic FAULT reset.
"""
import argparse
from datetime import datetime
import fcntl
import json
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import yaml


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain',type=int,default=90)
    parser.add_argument('--output-directory')
    args=parser.parse_args()
    if not 0<=args.domain<=232:
        parser.error('ROS domain must be between 0 and 232')
    session_lock=open(f'/tmp/amir_gui_straight_domain_{args.domain}.lock','w')
    try:
        fcntl.flock(session_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:
        parser.error('A GUI demo already owns this domain; close it or select --domain')
    workspace=Path(__file__).resolve().parents[4]
    scripts=Path(__file__).resolve().parent
    stamp=datetime.now().strftime('%Y%m%d_%H%M%S')
    out=Path(args.output_directory or workspace/f'gazebo_evidence/{stamp[:8]}/gui_straight_{stamp[9:]}').resolve()
    out.mkdir(parents=True,exist_ok=False)
    baseline=workspace/'gazebo_evidence/20261017/lateral/low_load'
    certificate=baseline/'calibration_tz_passed.json'
    if not certificate.is_file():
        raise RuntimeError('Verified local baseline certificate is missing: '+str(certificate))
    config=yaml.safe_load((baseline/'c.yaml').read_text())
    config['/**/follower_controller']['ros__parameters']['calibration_file']=str(certificate)
    (out/'follower.yaml').write_text(yaml.safe_dump(config))
    # Reuse plugins confirmed in the installed Fortress GUI configuration.
    template=Path('/usr/share/ignition/ignition-gazebo6/gui/gui.config').read_text()
    template=template[template.index('?>')+2:]
    gui=ET.fromstring('<configuration>'+template+'</configuration>')
    gui.find("dialog[@name='quick_start']").set('show_again','false')
    gui.find('window/width').text='1400'
    gui.find('window/height').text='900'
    gui.find("plugin[@filename='MinimalScene']/camera_pose").text='-2.5 -2.5 2.2 0 0.42 0.7853981634'
    (out/'gui.config').write_text('<?xml version="1.0"?>\n'+''.join(ET.tostring(c,encoding='unicode') for c in gui))
    env=os.environ.copy()
    env.update(ROS_DOMAIN_ID=str(args.domain),IGN_PARTITION='amir_gui_straight_'+stamp,
               RMW_FASTRTPS_PUBLICATION_MODE='ASYNCHRONOUS')
    launch=['ros2','launch','amir_follower_control','coop_transport.launch.py',
        'headless:=false',f'gui_config:={out / "gui.config"}',
        'grasp_mode:=friction','pinch_orientation:=lateral','gentle_grasp:=true',
        'contact_force_n:=6.0','holding_force_n:=5.0','holding_position_gain:=0.00015',
        'holding_max_step_rad:=0.0003','holding_interval_s:=0.1','grasp_inset_m:=0.025',
        'posture_pitch_rad:=0.2','contact_compliance:=true','contact_cfm:=0.05',
        'payload_mass_kg:=0.5','evaluation_logging:=true',
        f'config:={out / "follower.yaml"}',f'evidence_directory:={out}']
    session={'evidence_directory':str(out),'ROS_DOMAIN_ID':args.domain,
             'IGN_PARTITION':env['IGN_PARTITION'],'launch_command':launch,
             'protocol':'0.5 kg lateral; 1 mm/s max, 0.001 m/s^2, 6 s + 10 s settle, leader lead .3 s'}
    (out/'session.json').write_text(json.dumps(session,indent=2))
    (workspace/'gazebo_evidence/gui_straight_latest.txt').write_text(str(out)+'\n')
    log=(out/'launch.log').open('w')
    root=subprocess.Popen(launch,env=env,cwd=workspace,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    session['launch_pid']=root.pid
    (out/'session.json').write_text(json.dumps(session,indent=2))

    def pause(value):
        subprocess.run(['ign','service','-s','/world/cooperative_transport/control',
            '--reqtype','ignition.msgs.WorldControl','--reptype','ignition.msgs.Boolean',
            '--timeout','3000','--req','pause: '+str(value).lower()],env=env,
            stdout=subprocess.DEVNULL,check=True,timeout=5)

    def trial(name,extra,startup=30):
        path=out/name
        with (out/(name+'.log')).open('w') as target:
            process=subprocess.Popen(['python3',str(scripts/'run_local_trial.py'),
                '--output-directory',str(path),'--wait-startup-s',str(startup),*extra],
                env=env,cwd=workspace,stdout=target,stderr=subprocess.STDOUT,start_new_session=True)
            deadline=time.monotonic()+max(180,startup+90)
            try:
                while process.poll() is None:
                    if root.poll() is not None:raise RuntimeError('Gazebo session closed; see launch.log')
                    if time.monotonic()>deadline:raise RuntimeError('Bounded trial wall timeout')
                    time.sleep(.2)
            finally:
                if process.poll() is None:
                    os.killpg(process.pid,signal.SIGINT)
                    try:process.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid,signal.SIGTERM)
                        process.wait(timeout=5)
        result=json.loads((path/'summary.json').read_text())
        return path,result

    try:
        print('Gazebo GUIを起動しています。初期姿勢・接近・把持確認を待っています。',flush=True)
        print('証跡: '+str(out),flush=True)
        _,ready=trial('startup_hold',['--stimulus','hold','--duration','1','--settle-seconds','1'],360)
        pause(True)
        (out/'ready.json').write_text(json.dumps(ready,indent=2))
        print('把持確認済み・pause中。' if ready['success'] else '初期確認で停止: '+ready.get('failure',''),flush=True)
        allowed=ready['success']
        number=0
        while root.poll() is None:
            print('\nEnter: 直進6秒＋姿勢回復10秒 / q: 終了' if allowed else '\n試験は停止しています。q: 終了（自動resetなし）',flush=True)
            while root.poll() is None:
                if select.select([sys.stdin],[],[],.5)[0]:
                    command=sys.stdin.readline()
                    break
            else:break
            if not command or command.strip().lower()=='q':break
            if not allowed:continue
            number+=1
            print('直進開始（最大1 mm/s、Leader先行.3秒）。終了後にpauseします。',flush=True)
            pause(False)
            try:
                path,result=trial(f'straight_{number:03d}',['--stimulus','leader','--axis','fx',
                    '--delay','.3','--speed','.001','--acceleration','.001','--duration','6','--settle-seconds','10'])
            finally:pause(True)
            allowed=result['success']
            print('完走・pause中。' if allowed else '停止: '+result.get('failure',''),flush=True)
            if allowed:
                analysis_env=env.copy();analysis_env['PYTHONNOUSERSITE']='1'
                commands=[['analyze_trial.py',str(path)],
                    ['analyze_control.py',str(out/'follower.jsonl'),'--urdf',str(out/'amir2.urdf'),
                     '--trial',str(path/'summary.json'),'--output-directory',str(path/'analysis')],
                    ['analyze_recenter.py',str(out/'follower.jsonl'),'--urdf',str(out/'amir2.urdf'),
                     '--trial',str(path/'summary.json'),'--output',str(path/'recenter_metrics.json')]]
                for argv in commands:
                    with (path/(argv[0]+'.log')).open('w') as analysis_log:
                        subprocess.run(['python3',str(scripts/argv[0]),*argv[1:]],env=analysis_env,
                            stdout=analysis_log,stderr=subprocess.STDOUT,check=True,timeout=30)
                measured=json.loads((path/'evaluation_metrics.json').read_text())
                delta=measured['GT_translation_delta_m']
                print('実測x移動: Leader %.3f mm / Follower台車 %.3f mm / 物体 %.3f mm'%
                      tuple(1000*delta[n][0] for n in ('amir1','amir2','cooperative_payload')),flush=True)
            print('ログ: '+str(path),flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        if root.poll() is None:
            try:pause(True)
            except (subprocess.SubprocessError,OSError):pass
            os.killpg(root.pid,signal.SIGINT)
            try:root.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(root.pid,signal.SIGTERM)
                root.wait(timeout=5)
        log.close()


if __name__=='__main__':
    main()

"""Read-only observer cancels its OWN launch at measured near-peak arm speed."""
import csv
from collections import deque
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from amir_compliance_control.wrench import LocalModel

ROOT=Path(__file__).resolve().parents[2]
OUT=Path(__file__).resolve().parent
CASE=OUT/'servo_now_stop'
if CASE.exists():raise RuntimeError('probe output already exists')
env={**os.environ,'ROS_DOMAIN_ID':'232','ROS_LOCALHOST_ONLY':'1'}
log=(OUT/'servo_now_stop_console.log').open('w')
cmd=['ros2','launch','cooperative_transport_bringup','compliance_round_trip.launch.py',
     'headless:=true','keep_gui_open:=false','start_delay_s:=0','ros_domain_id:=232',
     'output_directory:='+str(CASE)]
proc=subprocess.Popen(cmd,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT)
os.environ['ROS_DOMAIN_ID']='232';os.environ['ROS_LOCALHOST_ONLY']='1'
# Join as a read-only observer only AFTER the experiment has captured its own
# startup writer audit. No experiment/controller readiness check is weakened.
discovery_deadline=time.monotonic()+600
while proc.poll() is None:
    graph_path=CASE/'trial/graph.json'
    if graph_path.exists():
        try:
            graph=json.loads(graph_path.read_text())
            topics=['/follower/'+t for t in ('base_cmd','rover_twist',
                'mecanum_drive_controller/reference_unstamped','arm_controller/joint_trajectory')]
            if all(len(graph.get(t,[]))==1 for t in topics):break
        except ValueError:pass
    if time.monotonic()>discovery_deadline:
        proc.send_signal(signal.SIGINT);proc.wait(timeout=180)
        raise RuntimeError('probe startup audit deadline')
    time.sleep(.1)
rclpy.init(args=[])
node=rclpy.create_node('fast_stop_observer')
history=deque(maxlen=2);model=[None];debug=[None];trigger=None


def joints(msg):
    urdf=CASE/'trial/generated/leader.urdf'
    if model[0] is None:
        if not urdf.exists():return
        model[0]=LocalModel(urdf.read_text())
    values=dict(zip(msg.name,msg.position))
    if not all(n in values for n in model[0].names):return
    stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
    if history and stamp<=history[-1][0]:return
    try:p=model[0].frames([values[n] for n in model[0].names])[0]
    except ValueError:return
    history.append((stamp,p))


node.create_subscription(JointState,'/leader/joint_states',joints,qos_profile_sensor_data)
node.create_subscription(String,'/leader/arm_compensation/debug',
    lambda msg:debug.__setitem__(0,json.loads(msg.data)),10)
deadline=time.monotonic()+900
try:
    while proc.poll() is None:
        rclpy.spin_once(node,timeout_sec=.05)
        if trigger is None and debug[0] and len(history)==2:
            t0,p0=history[0];t1,p1=history[1]
            speed=float((p1[0]-p0[0])/(t1-t0))
            reference=debug[0]['arm_reference_velocity_m_s']
            if reference>=.0536*.95 and speed>=.0536*.9:
                trigger={'leader_debug':debug[0],'joint_source_stamp_s':t1,
                    'actual_FK_interval_speed_m_s':speed,'launch_pid':proc.pid,
                    'signal':'SIGINT to this probe own launch only'}
                (OUT/'moving_stop_trigger.json').write_text(json.dumps(trigger,indent=2))
                proc.send_signal(signal.SIGINT)
                print('STOP_TRIGGER '+json.dumps(trigger),flush=True)
        if time.monotonic()>deadline:
            proc.send_signal(signal.SIGINT)
            raise RuntimeError('moving-stop probe deadline; own launch requested to stop')
    result=json.loads((CASE/'trial/result.json').read_text())
    passed=bool(trigger and result.get('safe_stop_verified') and result.get('grasp_held_after_stop')
        and result.get('gripper_hold_verified'))
    contacts=True
    with (CASE/'trial/timeseries.csv').open() as stream:
        for row in csv.DictReader(stream):
            if row.get('admittance_enabled')=='True':
                contacts &= all(row.get(f'{r}_native_{s}_contact')=='True'
                    for r in ('leader','follower') for s in ('left','right'))
    report={'passed':bool(passed and contacts),'scope':'intentional cancellation during measured fast arm motion',
        'full_transport_completion_expected':False,'trigger':trigger,'contacts_continuous':contacts,
        'safe_stop_verified':result.get('safe_stop_verified'),
        'grasp_held_after_stop':result.get('grasp_held_after_stop'),
        'gripper_hold_verified':result.get('gripper_hold_verified'),
        'original_trial_passed':result.get('passed'),'original_trial_error':result.get('error')}
    (OUT/'moving_stop_evidence.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report),flush=True)
    if not report['passed']:raise RuntimeError('moving-stop/grip proof failed')
finally:
    if proc.poll() is None:
        proc.send_signal(signal.SIGINT)
        proc.wait(timeout=180)
    node.destroy_node();rclpy.try_shutdown();log.close()

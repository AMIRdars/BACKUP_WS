#!/usr/bin/env python3
"""Replay measured OWN data through the real observer; publishes no commands.

This is offline software validation, not another physical rotation trial.
"""
import argparse
import json
from pathlib import Path
import time

import rclpy
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from amir_compliance_control.rotation_observer import observer_type


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('folder',type=Path);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--qualification-file',type=Path)
    p.add_argument('--align-observer-sources',action='store_true')
    a=p.parse_args();a.folder=a.folder.resolve();a.output.mkdir(parents=True,exist_ok=False)
    rows=[json.loads(line) for line in (a.folder/'passive_local.jsonl').read_text().splitlines()]
    rclpy.init();node=None;outputs=[];faults={};report=dict(passed=False,condition='offline measured local observer replay')
    try:
        node=observer_type()(namespace='/follower',parameter_overrides=[Parameter('use_sim_time',value=True),
            Parameter('urdf_file',value=str(a.folder/'generated/follower.urdf')),
            Parameter('qualification_file',value=str((a.qualification_file or a.folder/'qualification.json').resolve())),
            Parameter('orientation_envelope_rad',value=.01),Parameter('joint_margin_rad',value=.01),
            Parameter('yaw_heading_frame',value='grasp_heading'),
            Parameter('align_own_source_history',value=a.align_observer_sources)])
        for index,row in enumerate(rows):
            node.get_clock().set_ros_time_override(Time(seconds=row['timestamp']))
            msg=JointState(name=list(node.geometry.model.names),position=row['q_rad'],velocity=row['qdot_rad_s'])
            odom=Odometry();wrench=WrenchStamped()
            for m in (msg,odom,wrench):m.header.stamp=Time(seconds=row['timestamp']).to_msg()
            odom.twist.twist.linear.x,odom.twist.twist.linear.y,odom.twist.twist.angular.z=row['base_twist']
            for key,value in zip(('force.x','force.y','force.z','torque.x','torque.y','torque.z'),row['wrench_tcp']):
                group,component=key.split('.');setattr(getattr(wrench.wrench,group),component,value)
            for key,m in [('joint_states',msg),('mecanum_drive_controller/odometry',odom),
                          ('wrench_at_grasp_raw',wrench),('grasp/state',String(data='attached'))]:node.receive(key,m)
            if index==0:
                latch=node.latch(Trigger.Request(),Trigger.Response())
                if not latch.success:raise ValueError(latch.message)
                node.armed=True
            out=node.observation();out['replay_phase']=row['phase'];outputs.append(out)
        report['sample_count']=len(outputs)
        report['invalid_reasons']={reason:sum(o['valid_reason']==reason for o in outputs)
            for reason in set(o['valid_reason'] for o in outputs if not o['yaw_valid'])}
        final=node.observation()
        for key in node.receipt:node.receipt[key]-=.2
        faults['wall_receipt_stale']=not node.observation()['yaw_valid']
        for key in node.receipt:node.receipt[key]=time.monotonic()
        t=rows[-1]['timestamp'];node.get_clock().set_ros_time_override(Time(seconds=t+.1))
        faults['source_stale']=not node.observation()['yaw_valid']
        node.get_clock().set_ros_time_override(Time(seconds=t+.02))
        for key in ('joint_states','mecanum_drive_controller/odometry','wrench_at_grasp_raw'):
            node.latest[key].header.stamp=Time(seconds=t+.02).to_msg()
        node.latest['joint_states'].position=[rows[-1]['q_rad'][0]+.2,*rows[-1]['q_rad'][1:]]
        jump=node.observation();faults['joint_jump']=not jump['yaw_valid'] and any(s in jump['valid_reason'] for s in ('discontinuity','innovation'))
        report['joint_jump_reason']=jump['valid_reason']
        report.update(last_valid=final['yaw_valid'],fault_rejections=faults,
                      passed=all(o['yaw_valid'] for o in outputs) and all(faults.values()))
    except Exception as exc:report['error']=str(exc)
    finally:
        if node is not None:node.destroy_node()
        rclpy.try_shutdown()
        (a.output/'observations.jsonl').write_text(''.join(json.dumps(o)+'\n' for o in outputs))
        (a.output/'result.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2));return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())

"""Startup/evaluation-only contact geometry inspection from actual receipts."""
import argparse
import itertools
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage

from amir_follower_control.core import origin,rotation
from amir_follower_control.prepare_payload import quaternion_matrix


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('directory')
    args=parser.parse_args()
    directory=Path(args.directory)
    rclpy.init()
    node=rclpy.create_node('inspect_grasp_geometry')
    joints={}
    poses={}
    for robot in ('amir1','amir2'):
        node.create_subscription(JointState,f'/{robot}/joint_states',
            lambda m,r=robot:joints.update({r:m}),qos_profile_sensor_data)
    def receive(msg):
        for t in msg.transforms:
            matrix=np.eye(4)
            matrix[:3,:3]=quaternion_matrix(t.transform.rotation)
            p=t.transform.translation
            matrix[:3,3]=[p.x,p.y,p.z]
            poses[t.child_frame_id]=matrix
    node.create_subscription(TFMessage,'/experiment/model_poses',receive,qos_profile_sensor_data)
    deadline=time.monotonic()+10
    try:
        while len(joints)<2 or not {'amir1','amir2','cooperative_payload'}.issubset(poses):
            if time.monotonic()>deadline:
                raise RuntimeError('geometry inspection needs evaluation GT and both joint states')
            rclpy.spin_once(node,timeout_sec=.02)
        result={'scope':'startup/evaluation only; no observations forwarded to follower','robots':{}}
        inverse=np.linalg.inv(poses['cooperative_payload'])
        for robot in ('amir1','amir2'):
            root=ET.parse(directory/f'{robot}.urdf').getroot()
            parents={j.find('child').get('link'):j for j in root.findall('joint')}
            mapping=dict(zip(joints[robot].name,joints[robot].position))
            def frame(link):
                if link=='base_footprint':
                    return poses[robot]
                joint=parents[link]
                t=frame(joint.find('parent').get('link'))@origin(joint.find('origin'))
                if joint.get('type')!='fixed':
                    name=joint.get('name')
                    mimic=joint.find('mimic')
                    if name in mapping:q=mapping[name]
                    elif name+'_mimic' in mapping:q=mapping[name+'_mimic']
                    elif mimic is not None:
                        q=float(mimic.get('multiplier','1'))*mapping[mimic.get('joint')]+float(mimic.get('offset','0'))
                    else:raise ValueError('unobserved joint '+name)
                    r=np.eye(4)
                    r[:3,:3]=rotation([float(v) for v in joint.find('axis').get('xyz').split()],q)
                    t=t@r
                return t
            pads={}
            for side in ('left','right'):
                link=f'finger_{side}_1'
                boxes=[]
                for collision in root.find(f'link[@name="{link}"]').findall('collision'):
                    size=np.array([float(v) for v in collision.find('geometry/box').get('size').split()])
                    t=inverse@frame(link)@origin(collision.find('origin'))
                    corners=np.array([t[:3,3]+t[:3,:3]@(size*np.array(sign)/2)
                                      for sign in itertools.product((-1,1),repeat=3)])
                    boxes.append({'min_payload_xyz':corners.min(axis=0).tolist(),
                                  'max_payload_xyz':corners.max(axis=0).tolist()})
                pads[side]=boxes
            result['robots'][robot]={'observed_joint_positions':mapping,'finger_boxes_in_payload':pads,
                                    'tcp_in_payload':(inverse@frame('tcp_link'))[:3,3].tolist()}
        (directory/'grasp_geometry.json').write_text(json.dumps(result,indent=2,allow_nan=False))
        for robot,row in result['robots'].items():
            print(robot,'tcp',row['tcp_in_payload'])
            for side,boxes in row['finger_boxes_in_payload'].items():
                relevant=[b for b in boxes if b['min_payload_xyz'][0]<.6 and b['max_payload_xyz'][0]>-.6
                          and b['min_payload_xyz'][2]<.04 and b['max_payload_xyz'][2]>-.04]
                print(side,[(np.round(b['min_payload_xyz'],5).tolist(),np.round(b['max_payload_xyz'],5).tolist())
                            for b in relevant])
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()

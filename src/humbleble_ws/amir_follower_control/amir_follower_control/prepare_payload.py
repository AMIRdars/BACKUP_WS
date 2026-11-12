"""Evaluation/startup-only GT use to place a supported, tilted payload.

This process exits before compliant control starts. Its observations are never
sent to the follower; only its own resulting physical wrist wrench is sensed.
"""
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.srv import SpawnEntity
from sensor_msgs.msg import JointState

from .core import ArmURDF, rotation


def quaternion_matrix(q):
    a=np.array([q.x,q.y,q.z,q.w],dtype=float)
    if not np.all(np.isfinite(a)) or np.linalg.norm(a)<.9:
        raise ValueError('invalid observed base orientation')
    x,y,z,w=a/np.linalg.norm(a)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def set_orientation(q,roll,pitch,yaw):
    sr,cr=np.sin(roll/2),np.cos(roll/2)
    sp,cp=np.sin(pitch/2),np.cos(pitch/2)
    sy,cy=np.sin(yaw/2),np.cos(yaw/2)
    q.x=float(sr*cp*cy-cr*sp*sy)
    q.y=float(cr*sp*cy+sr*cp*sy)
    q.z=float(cr*cp*sy-sr*sp*cy)
    q.w=float(cr*cp*cy+sr*sp*sy)


class PreparePayload(Node):
    def __init__(self):
        super().__init__('prepare_tilted_payload')
        for key,value in {'evidence_directory':'','payload_file':'','approach_distance':.11,
                          'expected_axis_pitch_rad':0.,'payload_roll_rad':float(np.pi/2)}.items():
            self.declare_parameter(key,value)
        self.directory=Path(self.get_parameter('evidence_directory').value)
        self.arms={r:ArmURDF((self.directory/f'{r}.urdf').read_text()) for r in ('amir1','amir2')}
        self.odom,self.joints={},{}
        self.done=None
        self.client=self.create_client(SpawnEntity,'/world/cooperative_transport/create')
        for robot in self.arms:
            self.create_subscription(Odometry,f'/{robot}/odom',
                lambda m,r=robot:self.odom.update({r:m}),qos_profile_sensor_data)
            self.create_subscription(JointState,f'/{robot}/joint_states',
                lambda m,r=robot:self.joints.update({r:m}),qos_profile_sensor_data)
        self.pending=None
        self.request_queue=[]
        self.started=False
        self.create_timer(.1,self.tick)

    def request(self,name,xml,position,angles):
        req=SpawnEntity.Request()
        req.entity_factory.name=name
        req.entity_factory.sdf=xml
        req.entity_factory.pose.position.x=float(position[0])
        req.entity_factory.pose.position.y=float(position[1])
        req.entity_factory.pose.position.z=float(position[2])
        set_orientation(req.entity_factory.pose.orientation,*angles)
        return req

    def build(self):
        points=[]
        for robot,arm in self.arms.items():
            m=self.joints[robot]
            mapping=dict(zip(m.name,m.position))
            q=[mapping[n] for n in arm.names]
            g,_,_=arm.evaluate(q)
            pose=self.odom[robot].pose.pose
            rb=quaternion_matrix(pose.orientation)
            pb=np.array([pose.position.x,pose.position.y,pose.position.z])
            # The existing approach will move each base along its own +x.
            pb+=rb@np.array([self.get_parameter('approach_distance').value,0.,0.])
            points.append(pb+rb@g[:3,3])
        center=.5*(points[0]+points[1])
        axis=(points[1]-points[0])/np.linalg.norm(points[1]-points[0])
        pitch=np.arctan2(axis[2],np.linalg.norm(axis[:2]))
        expected=self.get_parameter('expected_axis_pitch_rad').value
        if expected and abs(pitch-expected)>.001:
            raise ValueError('predicted payload axis disagrees with paired arm posture; keep bases stationary during placement')
        yaw=np.arctan2(axis[1],axis[0])
        payload=ET.parse(self.get_parameter('payload_file').value)
        sizes=[float(x) for x in payload.find('.//link[@name="payload_link"]/collision/geometry/box/size').text.split()]
        roll=float(self.get_parameter('payload_roll_rad').value)
        if not np.isfinite(roll) or min(abs(roll),abs(roll-np.pi/2))>1e-8:
            raise ValueError('payload roll must use the staged lateral or vertical pinch orientation')
        normal=rotation([0,0,1],yaw)@rotation([0,1,0],-pitch)@np.array([0.,0.,1.])
        half_height=(abs(np.sin(roll))*sizes[1]+abs(np.cos(roll))*sizes[2])/2
        support_center=center-normal*(half_height+.02)
        support='''<sdf version="1.8"><model name="grasp_support"><static>true</static>
          <link name="link"><collision name="collision"><geometry><box><size>.22 .16 .04</size></box></geometry>
          <surface><friction><ode><mu>.8</mu><mu2>.8</mu2></ode></friction></surface></collision>
          <visual name="visual"><geometry><box><size>.22 .16 .04</size></box></geometry></visual></link></model></sdf>'''
        (self.directory/'payload_initialization.json').write_text(json.dumps({
            'predicted_grasp_points_world':[p.tolist() for p in points],
            'payload_center_world':center.tolist(),'pitch_rad':float(pitch),
            'expected_axis_pitch_rad':expected,
            'payload_roll_rad':roll,'support_center_world':support_center.tolist(),
            'support_normal_world':normal.tolist(),
            'yaw_rad':float(yaw),'payload_size_m':sizes,
            'mass_kg':float(payload.find('.//link[@name="payload_link"]/inertial/mass').text),
            'mass_field_scope':'payload_link; auxiliary link masses remain as modeled',
            'total_model_mass_kg':sum(float(link.find('inertial/mass').text)
                                     for link in payload.findall('.//model/link')
                                     if link.find('inertial/mass') is not None),
            'GT_use':'startup-only separate process'},indent=2))
        self.request_queue=[self.request('grasp_support',support,support_center,[0.,-pitch,yaw]),
                            self.request('cooperative_payload',ET.tostring(payload.getroot(),encoding='unicode'),
                                         center,[roll,-pitch,yaw])]

    def tick(self):
        if self.done is not None:
            return
        if not self.started:
            if len(self.odom)!=2 or len(self.joints)!=2 or not self.client.service_is_ready():
                return
            self.build()
            self.started=True
        if self.pending is not None:
            if not self.pending.done():
                return
            if not self.pending.result().success:
                self.done=False
                return
            self.pending=None
        if self.request_queue:
            self.pending=self.client.call_async(self.request_queue.pop(0))
        else:
            self.done=True


def main(args=None):
    rclpy.init(args=args)
    node=PreparePayload()
    deadline=time.monotonic()+30.
    try:
        while rclpy.ok() and node.done is None and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1)
        success=node.done is True
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if success else 1)

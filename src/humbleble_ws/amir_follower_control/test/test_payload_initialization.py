"""Regression for placement using bases that already started their approach."""
from pathlib import Path
from unittest.mock import patch

import numpy as np
import pytest
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory
import xacro
import itertools

from amir_follower_control.core import ArmURDF
from amir_follower_control.posture import HOME,paired_postures,with_initial_positions
from amir_follower_control.prepare_payload import PreparePayload,quaternion_matrix


@pytest.mark.parametrize('advance',[0.,.025])
@pytest.mark.parametrize('roll',[0.,float(np.pi/2)])
def test_payload_placement_requires_stationary_approach_start(tmp_path,advance,roll):
    source=Path(get_package_share_directory('amir_description'))/'urdf/amir_mecanum3_sim.xacro'
    xml=xacro.process_file(str(source),mappings={'namespace':'amir2'}).toxml()
    arm=ArmURDF(xml)
    pitch,span=.2,.575
    poses=paired_postures(xml,pitch,span,wrist_angle=roll)
    for robot,q in zip(('amir1','amir2'),poses):
        (tmp_path/f'{robot}.urdf').write_text(with_initial_positions(xml,q))
    payload=Path(get_package_share_directory('cooperative_transport_description'))/'models/cooperative_payload_friction/model.sdf'
    base_init=Node.__init__
    def init(self,*args,**kwargs):
        kwargs['parameter_overrides']=[Parameter('evidence_directory',value=str(tmp_path)),
            Parameter('payload_file',value=str(payload)),Parameter('expected_axis_pitch_rad',value=pitch),
            Parameter('payload_roll_rad',value=roll)]
        base_init(self,*args,**kwargs)
    rclpy.init()
    with patch.object(Node,'__init__',init):
        node=PreparePayload()
    try:
        for robot,side,q in zip(('amir1','amir2'),(-1,1),poses):
            msg=Odometry()
            msg.pose.pose.position.x=float(side*(span*np.cos(pitch)+arm.evaluate(HOME)[0][0,3]+.11-advance))
            yaw=0. if side<0 else np.pi
            msg.pose.pose.orientation.z=float(np.sin(yaw/2))
            msg.pose.pose.orientation.w=float(np.cos(yaw/2))
            node.odom[robot]=msg
            node.joints[robot]=JointState(name=list(arm.names),position=q.tolist())
        if advance:
            with pytest.raises(ValueError,match='keep bases stationary'):
                node.build()
            assert not node.request_queue
        else:
            node.build()
            import json
            metadata=json.loads((tmp_path/'payload_initialization.json').read_text())
            assert metadata['pitch_rad']==pytest.approx(pitch,abs=1e-8)
            assert metadata['total_model_mass_kg']==pytest.approx(.5)
            assert len(node.request_queue)==2
            stand,bar=[req.entity_factory.pose for req in node.request_queue]
            n=quaternion_matrix(stand.orientation)[:,2]
            stand_top=np.array([stand.position.x,stand.position.y,stand.position.z])+n*.02
            center=np.array([bar.position.x,bar.position.y,bar.position.z])
            orientation=quaternion_matrix(bar.orientation)
            half_size=np.array(metadata['payload_size_m'])/2
            corners=[center+orientation@(half_size*np.array(signs))
                     for signs in itertools.product((-1,1),repeat=3)]
            assert min(n@(point-stand_top) for point in corners)==pytest.approx(0.,abs=1e-9)
    finally:
        node.destroy_node()
        rclpy.shutdown()

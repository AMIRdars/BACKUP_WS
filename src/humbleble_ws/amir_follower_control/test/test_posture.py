from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory

from amir_follower_control.core import ArmURDF
from amir_follower_control.posture import HOME, paired_postures, tip_pitch, with_initial_positions


@pytest.mark.parametrize('pitch',[.1,.15,.2])
@pytest.mark.parametrize('wrist_angle',[HOME[4],0.])
def test_pair_preserves_payload_axis_and_rank_with_real_urdf(pitch,wrist_angle):
    source=Path(get_package_share_directory('amir_description'))/'urdf/amir_mecanum3_sim.xacro'
    xml=subprocess.check_output(['xacro',str(source),'namespace:=amir2'],text=True)
    arm=ArmURDF(xml)
    home=arm.evaluate(HOME)[0]
    q1,q2=paired_postures(xml,pitch,.59,wrist_angle=wrist_angle)
    poses=[]
    for side,q in zip((-1,1),(q1,q2)):
        t,_,jac=arm.evaluate(q)
        np.testing.assert_allclose(t[0,3],home[0,3],atol=1e-8)
        np.testing.assert_allclose(t[2,3],home[2,3]+side*.59*np.sin(pitch),atol=1e-8)
        assert abs(tip_pitch(t)+side*pitch)<1e-8
        assert np.linalg.matrix_rank(jac)==4
        assert q[4]==wrist_angle
        if wrist_angle==0.:
            assert abs(t[2,0])<1e-5  # Pinch monitoring axis is horizontal in B.
        modified=with_initial_positions(xml,q)
        np.testing.assert_allclose(ArmURDF(modified).evaluate(q)[2],jac)
        root=ET.fromstring(modified)
        for name,value in zip(arm.names,q):
            initial=root.find(f".//ros2_control/joint[@name='{name}']/state_interface[@name='position']/param")
            assert initial is not None and float(initial.text)==pytest.approx(value)


@pytest.mark.parametrize('reach',[.4,.42])
def test_short_lateral_reach_keeps_height_alignment_and_four_axes(reach):
    source=Path(get_package_share_directory('amir_description'))/'urdf/amir_mecanum3_sim.xacro'
    xml=subprocess.check_output(['xacro',str(source),'namespace:=amir2'],text=True)
    arm=ArmURDF(xml)
    home=arm.evaluate(HOME)[0]
    for side,q in zip((-1,1),paired_postures(xml,.2,.575,wrist_angle=0.,reach=reach)):
        tip,_,jac=arm.evaluate(q)
        assert tip[0,3]==pytest.approx(reach,abs=1e-8)
        assert tip[2,3]==pytest.approx(home[2,3]+side*.575*np.sin(.2),abs=1e-8)
        assert np.linalg.matrix_rank(jac)==4
        assert np.min(np.minimum(q-arm.lower,arm.upper-q))>.08

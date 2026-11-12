import numpy as np
import pytest
from amir_compliance_control.arm_trajectory_bounds import streaming_knots


def test_reversing_velocity_preserves_both_knot_slew_and_segment_speed():
    h,dt=.08,.008
    limits=np.full(5,.03)
    start=np.zeros(5);end=np.full(5,.002)
    measured=np.full(5,.0005)
    sl=np.maximum(start-limits*dt,measured-limits*h)
    su=np.minimum(start+limits*dt,measured+limits*h)
    el=np.maximum(end-limits*dt,measured-limits*h)
    eu=np.minimum(end+limits*dt,measured+limits*h)
    # A direct reversal's negative knot offset cannot fit these position boxes.
    desired=np.full(5,-.03)
    a,b,v=streaming_knots(end-.001,desired,h,limits,sl,su,el,eu)
    assert np.all(a>=sl-1e-14) and np.all(a<=su+1e-14)
    assert np.all(b>=el-1e-14) and np.all(b<=eu+1e-14)
    assert np.max(abs(v))<=.03+1e-14
    assert np.allclose(b-a,v*h)
    assert not np.allclose(v,desired)


def test_feasible_request_is_unchanged():
    a,b,v=streaming_knots(np.full(5,.002),np.full(5,.02),.08,np.full(5,.03),
        np.full(5,-.001),np.full(5,.001),np.full(5,.001),np.full(5,.003))
    assert np.allclose(b,.002) and np.allclose(a,.0004) and np.allclose(v,.02)


def test_empty_measured_position_box_still_faults():
    with pytest.raises(ValueError,match='position bounds infeasible'):
        streaming_knots(np.zeros(5),np.zeros(5),.08,np.full(5,.03),
            np.ones(5),np.zeros(5),np.zeros(5),np.ones(5))


def test_disjoint_knots_require_excessive_speed_and_still_fault():
    with pytest.raises(ValueError,match='rate infeasible'):
        streaming_knots(np.zeros(5),np.zeros(5),.08,np.full(5,.03),
            np.zeros(5),np.full(5,.0001),np.full(5,.01),np.full(5,.0101))

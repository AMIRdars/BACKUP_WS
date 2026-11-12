"""Task feasibility and authority separation for the rank-five arm."""
import inspect
import numpy as np
import pytest
from amir_compliance_control.allocation_budget import TaskComplianceBudget
from amir_compliance_control.whole_body import WholeBodyAllocator,whole_jacobian
from amir_compliance_control.rotation_observer import ProjectedYaw
from amir_compliance_control.rotation_recentering import RotationRecentering
from test_local_yaw import URDF,HOME


def test_virtual_authority_is_slow_resettable_and_does_not_change_measured_yaw_preference():
    b=TaskComplianceBudget([.0005,.0005,.001],.2)
    r=RotationRecentering(mode='WHOLE_BODY',gains=[.5,.5,2.],velocity=[.002]*3,acceleration=[.002]*3,cutoff=.2)
    first=b.step([0.,0.,.0005],.02);assert 0.<first<.05
    for _ in range(200):
        credit=b.step([0.,0.,.0005],.02)
        assert np.array_equal(r.step([0.,0.,0.],[.475,0.,.65],.02,yaw_valid=True),np.zeros(3))
    assert .49<credit<.5
    b.reset();assert np.array_equal(b.low,np.zeros(3))


def test_virtual_task_can_use_one_allocator_to_fill_the_missing_arm_direction():
    g=ProjectedYaw(URDF.read_text(),orientation_envelope_rad=.01,joint_margin_rad=.01,heading_frame='grasp_heading')
    q=HOME.copy();target=np.array([0.,-5e-5,0.,0.,0.,1e-4]);j=whole_jacobian(g.model,q)
    kwargs=dict(base_speed=[.002]*3,base_acceleration=[.002]*3,task_metric=[1.,1.,1.,.1,.1,.4],
        base_regularization=inspect.signature(WholeBodyAllocator).parameters['base_regularization'].default)
    stopped=WholeBodyAllocator(g.model,**kwargs);moving=WholeBodyAllocator(g.model,**kwargs)
    budget=TaskComplianceBudget([.0005,.0005,.001],.2);old=np.zeros(8)
    for _ in range(250):
        credit=budget.step([0.,-2.5e-5,5e-5],.02)
        a=stopped.step(q,q,target,np.zeros(3),0.,.02)
        b=moving.step(q,q,target,np.zeros(3),credit,.02)
        assert np.all(abs(b-old)<=moving.acceleration*.02+1e-12)
        assert np.all(abs(b)<=moving.scale+1e-12);old=b
    assert np.array_equal(a[5:],np.zeros(3)) and np.linalg.norm(b[5:])>0.
    assert np.linalg.norm(j@b-target)<.01*np.linalg.norm(j@a-target)


@pytest.mark.parametrize('dt',[0.,-.01,.050001,float('nan')])
def test_authority_does_not_bypass_update_timing(dt):
    with pytest.raises(ValueError):TaskComplianceBudget([1.,1.,1.],.2).step([0.,0.,1.],dt)

"""Slow own-force memory, bounded commands and unchanged legacy modes."""
import numpy as np
import pytest
from amir_compliance_control.recentering import BaseRecentering


@pytest.mark.parametrize('new_sample',[True,False,'clock'])
@pytest.mark.parametrize('on_wrench',[False,True])
def test_allocation_updates_before_base_output_or_latches_on_real_staleness(tmp_path,new_sample,on_wrench):
    import time
    from rclpy.parameter import Parameter
    from std_srvs.srv import Trigger
    from rclpy.time import Time
    from geometry_msgs.msg import WrenchStamped
    from test_admittance import ArmRig
    from test_recentering import JointOverride
    from amir_compliance_control.whole_body_node import WholeBodyFollower
    rig=ArmRig(tmp_path)
    try:
        old=rig.ends['follower'];rig.executor.remove_node(old);rig.nodes.remove(old);old.destroy_node()
        node=WholeBodyFollower(namespace='/follower',parameter_overrides=[
            Parameter('use_sim_time',value=True),Parameter('urdf_file',value=str(rig.file)),
            Parameter('allow_ungrasped_test',value=True),Parameter('allocation_on_wrench',value=on_wrench),Parameter('allocation_use_clock',value=on_wrench)])
        rig.ends['follower']=node;rig.nodes.append(node);rig.executor.add_node(node)
        override=JointOverride(rig.pubs['follower','joints']);rig.pubs['follower','joints']=override
        rig.start();assert node.enable(Trigger.Request(),Trigger.Response()).success
        override.positions=np.array([.0002,0.,0.,0.,0.]);rig.pump(.2)
        assert node.base_twist.any(),node.reason
        node.timer.cancel()
        # Allocation is old but a fresh input can already be waiting in this
        # tick. It must be consumed before base publication, without a zero pulse.
        node.base_wall=time.monotonic()-.06
        if new_sample is True:
            previous=node.last_step_stamp
            node.sim=node.force_stamp+.004
            msg=WrenchStamped();msg.header.stamp=Time(seconds=node.sim).to_msg()
            msg.header.frame_id='follower/grasp_aligned';msg.wrench.force.x=.1
            node.filtered(msg)
            assert (node.last_step_stamp>previous) == on_wrench
        elif new_sample=='clock':
            if not on_wrench:node.force_stamp=node.last_step_stamp
            node.sim+=.004
            node.force_wall=time.monotonic()-.065
            force_stamp,force_wall=node.force_stamp,node.force_wall
        else:
            node.force_stamp=node.last_step_stamp;node.sim=node.last_step_stamp
        output=node.base_request(0.)
        if new_sample is True or (new_sample=='clock' and on_wrench):
            assert not node.latched and output.any(),node.reason
            assert node.last_step_stamp==(node.sim if on_wrench else node.force_stamp)
            if new_sample=='clock':
                # A real new integration step may use a held valid sample.
                # Never relabel that measurement or renew its receipt time.
                assert node.force_stamp==force_stamp and node.force_wall==force_wall
                node.force_wall-=.2;node.sim+=.004
                assert not node.base_request(0.).any() and node.latched
                assert 'filtered wrench/clock watchdog' in node.reason
        else:
            assert node.latched and not output.any()
            assert node.reason=='whole body allocation watchdog'
            assert node.first_control_fault['allocation_age_wall_s'] >= .05
            rig.pump(.1)
            assert node.latched and node.core is None
    finally:rig.close()


def core(*,hold=False):
    return BaseRecentering(8.,[12.,12.,1.],[1.05,.01,.001],[.05,.02,.002],
        force_acceleration_gains=[.05,0.,0.],force_filter_hz=.1,stationary_hold=hold)


def test_sustained_force_accumulates_velocity_and_zero_force_retains_memory():
    c=core()
    for _ in range(5000):c.step([0.,0.,0.],[.5,0.,.6],.004,measured_force=[1.,0.,0.])
    for _ in range(5000):v=c.step([0.,0.,0.],[.5,0.,.6],.004,measured_force=[0.,0.,0.])
    assert .98 < v[0] < 1.02
    assert c.authority([.0005,.0005,.001]) >= abs(v[0])/1.05
    assert not c.integral.any()


def test_short_force_changes_go_to_arm_before_slow_base_memory():
    c=core();v=c.step([0.,0.,0.],[.5,0.,.6],.02,measured_force=[1.,0.,0.])
    assert 0 < c.force_low[0] < .1
    assert v[0] < .0002
    assert not v[1:].any()


@pytest.mark.parametrize('force',[None,[float('nan'),0.,0.],[1.,0.]])
def test_invalid_own_force_is_rejected(force):
    with pytest.raises(ValueError):core().step([0.,0.,0.],[.5,0.,.6],.004,measured_force=force)


def test_same_axis_force_and_deflection_integrators_cannot_be_combined():
    with pytest.raises(ValueError):
        BaseRecentering(8.,[12.,12.,1.],[1.05,.01,.001],[.05,.02,.002],
            integral_gains=[36.,0.,0.],force_acceleration_gains=[.1,0.,0.])


def test_saturation_does_not_wind_up_and_restart_has_zero_memory():
    c=core();previous=np.zeros(3)
    for _ in range(2000):
        v=c.step([.0001,0.,0.],[.5,0.,.6],.02,measured_force=[100.,0.,0.])
        assert np.all(abs(v-previous) <= c.amax*.02+1e-12)
        assert abs(c.force_memory[0]) <= 1.05
        assert np.all(abs(v) <= c.vmax+1e-12)
        previous=v
    reset=core();assert not reset.force_memory.any() and not reset.command.any()


def test_local_hold_requires_actual_prior_motion_and_a_continuous_quiet_window():
    c=core(hold=True)
    for _ in range(100):c.step([0.,0.,0.],[.5,0.,.6],.02,measured_force=[0.,0.,0.],measured_base_velocity=[0.,0.,0.])
    assert not c.hold.any()
    c.step([0.,0.,0.],[.5,0.,.6],.02,measured_force=[0.,0.,0.],measured_base_velocity=[.01,0.,0.])
    for _ in range(10):c.step([0.,0.,0.],[.5,0.,.6],.02,measured_force=[0.,0.,0.],measured_base_velocity=[0.,0.,0.])
    assert not c.hold.any()
    for _ in range(10):v=c.step([0.,0.,0.],[.5,0.,.6],.02,measured_force=[0.,0.,0.],measured_base_velocity=[0.,0.,0.])
    assert c.hold[0] and not v.any()
    assert c.authority([.0005,.0005,.001]) == 0.
    for _ in range(20):v=c.step([.000005,0.,0.],[.5,0.,.6],.02,measured_force=[.01,0.,0.],measured_base_velocity=[0.,0.,0.])
    assert c.hold[0] and not v.any() and c.force_memory[0]==0.
    with pytest.raises(ValueError):
        c.step([0.,0.,0.],[.5,0.,.6],.02,measured_force=[float('nan'),0.,0.],measured_base_velocity=[0.,0.,0.])
    v=c.step([.00011,0.,0.],[.5,0.,.6],.02,measured_force=[.1,0.,0.],measured_base_velocity=[0.,0.,0.])
    assert not c.hold.any() and v[0]>0.


def test_stationary_hold_locks_allocator_base_and_keeps_arm_response():
    from pathlib import Path
    from amir_compliance_control.wrench import LocalModel
    from amir_compliance_control.whole_body import WholeBodyAllocator
    root=Path(__file__).resolve().parents[4]
    model=LocalModel((root/'audits/whole_body_compliance_20261019/cp9_transport/generated/follower.urdf').read_text())
    q=np.array([0.,1.4,-1.6,.2,1.570796])
    c=core(hold=True);c.seen_motion[0]=True
    for _ in range(30):c.step([0.,0.,0.],[.5,0.,.6],.02,measured_force=[0.,0.,0.],measured_base_velocity=[0.,0.,0.])
    allocator=WholeBodyAllocator(model,base_speed=[1.05,.01,.001],base_acceleration=[.05,.02,.002],
        base_cost_velocity=[.01,.01,.001])
    velocity=allocator.step(q,q,[.0005,0.,0.,0.,0.,0.],np.zeros(3),c.authority([.0005,.0005,.001]),.004)
    assert not velocity[5:].any() and velocity[:5].any()
    # A persistent displacement must restore ordinary base allocation.
    c.step([0.,.00011,0.],[.5,0.,.6],.02,measured_force=[0.,0.,0.],measured_base_velocity=[0.,0.,0.])
    assert not c.hold.any() and c.authority([.0005,.0005,.001]) > 0.


def test_local_hold_never_replaces_measured_motion_with_a_stop():
    c=core(hold=True)
    for _ in range(100):
        c.step([0.,0.,0.],[.5,0.,.6],.02,measured_force=[.1,0.,0.],measured_base_velocity=[1.,0.,0.])
    assert not c.hold.any() and c.command[0]>0.
    with pytest.raises(ValueError):
        c.step([0.,0.,0.],[.5,0.,.6],.02,measured_force=[0.,0.,0.])


def test_local_virtual_demand_can_release_hold_before_measured_arm_moves():
    c=core(hold=True);c.seen_motion[0]=True
    for _ in range(30):c.step([0.,0.,0.],[.5,0.,.6],.02,measured_force=[0.,0.,0.],measured_base_velocity=[0.,0.,0.])
    assert c.hold[0]
    v=c.step([0.,0.,0.],[.5,0.,.6],.02,measured_force=[.1,0.,0.],measured_base_velocity=[0.,0.,0.],allow_stationary_hold=False)
    assert not c.hold.any() and v[0]>0.


@pytest.mark.parametrize('delay_ticks',[1,5,10])
def test_long_ramp_in_limited_contact_model_from_own_force_and_arm_only(delay_ticks):
    # This synthetic sensor model uses the existing virtual damping/stiffness;
    # it is a component check, not a physical-contact acceptance substitute.
    c=core();dt=.004;error=0.;base=0.;queue=[0.]*delay_ticks;peak=0.
    for i in range(110000):
        t=i*dt;unknown_load_speed=max(0.,min(1.,.005*t,.005*(420-t)))
        relative_speed=unknown_load_speed-base
        own_force=400.*error+1200.*relative_speed
        cmd=c.step([error,0.,0.],[.5,0.,.6],dt,measured_force=[own_force,0.,0.])[0]
        queue.append(cmd);base=queue.pop(0)
        error+=(unknown_load_speed-base)*dt;peak=max(peak,abs(error))
    assert peak < .0005
    assert abs(error) < .0001 and abs(cmd) < .00001

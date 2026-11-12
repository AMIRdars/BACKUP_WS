"""Local sustained-deflection memory, legacy equivalence, and evidence rejection."""
import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest
from amir_compliance_control.recentering import BaseRecentering
from amir_compliance_control.whole_body import WholeBodyAllocator, whole_jacobian, base_jacobian
from amir_compliance_control.wrench import LocalModel

ROOT = Path(__file__).resolve().parent.parent


def test_zero_integral_is_bitwise_legacy_recentring():
    core = BaseRecentering(8., [12.,12.,1.], [.01,.01,.001], [.02,.02,.002])
    low, command = np.zeros(3), np.zeros(3)
    rng = np.random.default_rng(710)
    for _ in range(1000):
        error = rng.normal(size=3)*.0002
        grasp = np.array([.5,.1,.6])
        dt = .004
        low += (1-np.exp(-2*np.pi*8*dt))*(error-low)
        task = np.array([12.,12.,1.])*low
        desired = [task[0]+grasp[1]*task[2],task[1]-grasp[0]*task[2],task[2]]
        desired = np.clip(desired, [-.01,-.01,-.001],[.01,.01,.001])
        command += np.clip(desired-command, -np.array([.02,.02,.002])*dt,
                           np.array([.02,.02,.002])*dt)
        assert np.array_equal(core.step(error,grasp,dt),command)
        assert not core.integral.any()


def test_local_pi_tracks_one_metre_per_second_without_large_arm_deflection():
    # Synthetic base response with a deliberately delayed 20 ms actuator. This
    # is control stability evidence only; Gazebo contact must be tested too.
    core = BaseRecentering(8.,[12.,12.,1.],[1.05,.01,.001],[.05,.02,.002],integral_gains=[36.,0.,0.])
    error, dt, max_error, max_command = 0., .004, 0., 0.
    pending = [0.]*5
    for i in range(110000):
        t = i*dt
        payload_velocity = max(0.,min(1.,.005*t,.005*(420-t)))
        command = core.step([error,0.,0.],[.5,0.,.6],dt)[0]
        pending.append(command)
        error += (payload_velocity-pending.pop(0))*dt
        max_error = max(max_error,abs(error))
        max_command = max(max_command,abs(command))
    assert max_error < .0005
    assert max_command >= .999 and max_command <= 1.05
    assert abs(error) < 1e-10 and abs(core.command[0]) < 1e-9


def test_local_pi_initial_error_goes_to_arm_before_sustained_memory():
    core = BaseRecentering(8.,[12.,12.,1.],[1.05,.01,.001],[.05,.02,.002],integral_gains=[36.,0.,0.])
    assert not core.integral.any() and not core.command.any()
    core.step([.0001,0.,0.],[.5,0.,.6],.004)
    assert 0 < core.low[0] < .0001
    assert core.integral[0] < 36*.0001*.004
    assert core.command[0] <= .05*.004
    for _ in range(1000):
        core.step([1.,0.,0.],[.5,0.,.6],.004)
    assert abs(core.integral[0]) <= 1.05
    assert abs(core.command[0]) <= 1.05
    # Re-enabling creates a new core, so there is no accumulated restart speed.
    reset = BaseRecentering(8.,[12.,12.,1.],[1.05,.01,.001],[.05,.02,.002],integral_gains=[36.,0.,0.])
    assert not reset.integral.any() and not reset.command.any()


@pytest.mark.parametrize('gains',[[-1.,0.,0.],[float('nan'),0.,0.],[float('inf'),0.,0.]])
def test_invalid_integral_gains_rejected(gains):
    with pytest.raises(ValueError):
        BaseRecentering(8.,[12.,12.,1.],[1.05,.01,.001],[.05,.02,.002],integral_gains=gains)


def test_failed_base_evidence_cannot_expand_loaded_gate(tmp_path):
    spec = importlib.util.spec_from_file_location('cp10_speed_evidence',ROOT/'tools/cp10_speed_evidence.py')
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    (tmp_path/'result.json').write_text(json.dumps({'passed':False}))
    (tmp_path/'motion.json').write_text('[]')
    with pytest.raises(ValueError,match='passing CP-10'):
        module.combine_evidence(tmp_path,tmp_path,tmp_path/'unused')


def test_wider_speed_bound_preserves_arm_base_cost_in_physical_units():
    xml = (ROOT/'audits/whole_body_compliance_20261019/cp9_transport/generated/follower.urdf').read_text()
    model = LocalModel(xml)
    q = np.array([0.,1.4,-1.6,.2,1.570796])
    old = WholeBodyAllocator(model,base_speed=[.01,.01,.001],base_acceleration=[.05,.02,.002])
    widened = WholeBodyAllocator(model,base_speed=[1.05,.01,.001],base_acceleration=[.05,.02,.002],
                                base_cost_velocity=[.01,.01,.001])
    for _ in range(100):
        a = old.step(q,q,[-.001,0.,0.,0.,0.,0.],np.zeros(3),1.,.004)
        b = widened.step(q,q,[-.001,0.,0.,0.,0.,0.],np.zeros(3),1.,.004)
        np.testing.assert_allclose(a,b,atol=1e-8,rtol=1e-5)
    arm = abs((whole_jacobian(model,q)[:,:5]@b[:5])[0])
    assert arm > abs(b[5])


def test_high_speed_authority_preserves_proportional_damping_above_integral():
    # A state reachable after accumulating a sustained local deflection.
    # No peer state enters either the slow controller or the real allocator.
    core = BaseRecentering(8.,[12.,12.,1.],[1.05,.01,.001],[.05,.02,.002],integral_gains=[36.,0.,0.])
    core.integral[0] = core.command[0] = -.5
    core.low[0] = -.000139
    xml = (ROOT/'audits/whole_body_compliance_20261019/cp9_transport/generated/follower.urdf').read_text()
    model = LocalModel(xml)
    q = np.array([0.,1.4,-1.6,.2,1.570796])
    position,_ = model.frames(q)
    preferred = core.step([-.000139,0.,0.],position,.004)
    credit = core.authority([.0005,.0005,.001])
    assert credit*1.05 >= abs(preferred[0])-1e-12
    assert abs(preferred[0]) > abs(core.integral[0])
    allocator = WholeBodyAllocator(model,base_speed=[1.05,.01,.001],base_acceleration=[.05,.02,.002],
                                  base_cost_velocity=[.01,.01,.001])
    allocator.previous[5] = -.5
    result = allocator.step(q,q,base_jacobian(position)@preferred,preferred,credit,.004)
    assert abs(result[5]-preferred[0]) < 1e-7
    assert abs(result[5]) > abs(core.integral[0])+1e-4


@pytest.mark.parametrize('sign',[-1.,1.])
def test_high_speed_deceleration_with_shrinking_authority_keeps_bounds_and_task(sign):
    model = LocalModel((ROOT/'audits/whole_body_compliance_20261019/cp9_transport/generated/follower.urdf').read_text())
    rng = np.random.default_rng(4323)
    for speed in np.linspace(.3,1.,60):
        for dt in (.004,.012,.02):
            q = np.array([0.,1.4,-1.6,.2,1.570796])+rng.normal(size=5)*1e-5
            allocator = WholeBodyAllocator(model,base_speed=[1.05,.01,.001],base_acceleration=[.05,.02,.002],
                base_cost_velocity=[.01,.01,.001],base_filter_hz=8.)
            allocator.previous[5] = sign*speed
            prior = allocator.previous.copy()
            preferred = np.array([sign*(speed-.005*dt),0.,0.])
            desired = base_jacobian(model.frames(q)[0])@preferred+rng.normal(size=6)*1e-7
            velocity = allocator.step(q,q,desired,preferred,abs(preferred[0])/1.05,dt)
            assert np.all(abs(velocity) <= allocator.scale+1e-12)
            assert np.all(abs(velocity-prior) <= allocator.acceleration*dt+1e-12)
            assert np.linalg.norm(whole_jacobian(model,q)@velocity-desired) < 1e-6

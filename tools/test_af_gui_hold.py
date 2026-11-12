"""Observation is cancellable and cannot conceal a grip/motion health failure."""
import json
import pytest
from af_gui_hold import hold_until_stopped, pump_feedback, coherent_source_ages,pause_world_for_review


def test_long_hold_has_bounded_status_and_clean_cancel(tmp_path):
    now=[0.];calls=[0]
    def pump():
        now[0]+=.01;calls[0]+=1
    path=tmp_path/'status.json'
    hold_until_stopped(pump=pump,health=lambda:None,cancelled=lambda:calls[0]>=1000,
        status_file=path,metadata={'runner_pid':123},clock=lambda:now[0])
    status=json.loads(path.read_text())
    assert status['state']=='GUI_HOLD_CLOSED' and status['monitor_healthy']
    assert calls[0]==1000 and path.stat().st_size<400


@pytest.mark.parametrize('paused',[False,True])
def test_review_pause_requires_real_gazebo_ack_and_paused_stats(paused):
    from types import SimpleNamespace
    def run(args,**kwargs):
        assert kwargs['check'] and kwargs['timeout']<=5.
        return SimpleNamespace(stdout='data: true' if args[1]=='service' else json.dumps({'paused':paused}))
    if paused:assert pause_world_for_review(run)['world_stats']['paused']
    else:
        with pytest.raises(RuntimeError,match='did not confirm'):pause_world_for_review(run)


@pytest.mark.parametrize('reason',['grasp missing','wheels moving','joint feedback stale','clock stopped'])
def test_observation_failure_propagates_for_immediate_cleanup(tmp_path,reason):
    def health():raise RuntimeError(reason)
    path=tmp_path/'status.json'
    with pytest.raises(RuntimeError,match=reason):
        hold_until_stopped(pump=lambda:None,health=health,cancelled=lambda:False,status_file=path,metadata={})
    status=json.loads(path.read_text())
    assert not status['monitor_healthy'] and status['state']=='GUI_HOLD_FAILED'


def test_queued_clock_and_joint_samples_are_dispatched_before_health_check():
    # CSV export temporarily stops dispatch. A single /clock callback would
    # pair t=1.0 with the previous q(t=.5), incorrectly declaring dropout.
    state={'clock':.5,'q':.5}
    queue=[('clock',1.),('q',.91),('clock',1.01),('q',1.)]
    def spin(timeout):
        if queue:
            key,value=queue.pop(0);state[key]=value
    pump_feedback(spin)
    assert not queue and abs(state['clock']-state['q'])<.1


def test_joint_callback_before_clock_does_not_create_false_negative_age():
    ages=coherent_source_ages(10.,{'leader':10.022,'follower':10.02})
    assert ages['leader']==0. and ages['follower']==pytest.approx(.002)


@pytest.mark.parametrize('clock,joints',[(10.,{'leader':9.8,'follower':10.}),
    (10.,{'leader':10.2,'follower':10.}), (10.,{'leader':float('nan')}),
    (0.,{'leader':1.}), (float('inf'),{'leader':1.})])
def test_real_stale_future_discontinuity_and_invalid_stamps_still_fail(clock,joints):
    with pytest.raises(ValueError):coherent_source_ages(clock,joints)

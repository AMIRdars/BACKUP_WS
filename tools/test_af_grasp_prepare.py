"""Lost startup attach notifications must not start unsupported transport."""
from types import SimpleNamespace
import pytest
from amir_af_trial import wait_for_grasps


def test_lost_requests_retry_only_unconfirmed_end():
    grasp={'leader':'detached','follower':'attached'};sent={'leader':0,'follower':0};now=[0.]
    def publish(robot):
        sent[robot]+=1
        # Simulate two lost startup notifications, then a real state ack.
        if robot=='leader' and sent[robot]==3:grasp[robot]='attached'
    pubs={r:SimpleNamespace(publish=lambda _,r=r:publish(r)) for r in grasp}
    def wait(predicate,timeout,healthy):
        assert timeout==10. and healthy is True
        for _ in range(100):
            if predicate():return
            now[0]+=.02
        pytest.fail('startup grasp acknowledgement never completed')
    wait_for_grasps(grasp,pubs,wait,clock=lambda:now[0])
    assert sent=={'leader':3,'follower':0}
    assert all(s=='attached' for s in grasp.values())


def test_missing_ack_does_not_manufacture_attachment():
    grasp={'leader':'detached','follower':'detached'}
    pubs={r:SimpleNamespace(publish=lambda _:None) for r in grasp}
    def wait(predicate,timeout,healthy):
        assert predicate() is False
        raise RuntimeError('timeout at attach')
    with pytest.raises(RuntimeError,match='timeout at attach'):
        wait_for_grasps(grasp,pubs,wait)
    assert all(s=='detached' for s in grasp.values())


def test_already_confirmed_grasp_sends_no_new_command():
    grasp={'leader':'attached','follower':'attached'}
    def unexpected(_):pytest.fail('confirmed grasp must not be commanded again')
    pubs={r:SimpleNamespace(publish=unexpected) for r in grasp}
    wait_for_grasps(grasp,pubs,lambda predicate,*_:predicate())

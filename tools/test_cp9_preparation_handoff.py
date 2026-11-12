"""Regression for the observed shutdown/watchdog race, without actuators.

Execute the preparation class's actual service methods with inert interfaces.
No main(), ROS context, subprocess, simulator or hardware is started.
"""
import ast
import json
from pathlib import Path
import time
from types import SimpleNamespace
import pytest


SOURCE=Path(__file__).with_name('amir_cp9_grasp_preparation.py')
tree=ast.parse(SOURCE.read_text())
class_tree=next(n for n in ast.walk(tree) if isinstance(n,ast.ClassDef) and n.name=='LocalGrasp')
environment={'Node':object,'time':time,'json':json,'String':lambda **kw:SimpleNamespace(**kw)}
exec(compile(ast.Module(body=[class_tree],type_ignores=[]),str(SOURCE),'exec'),environment)
Preparation=environment['LocalGrasp']


def fixture():
    node=Preparation.__new__(Preparation)
    node.state='HOLDING';node.reason='';node.arm_pub=None
    node.native_monitor={'state':'HOLDING','healthy':True};node.native_monitor_wall=time.monotonic()
    node.ready=lambda:True;node.arm_healthy=lambda:True;node.mimic_healthy=lambda:True
    node.get_namespace=lambda:'/follower'
    node.get_publishers_info_by_topic=lambda topic:[SimpleNamespace(node_name='native_grasp_monitor',node_namespace='/follower')]
    node.cancelled=0
    def cancel():node.cancelled+=1
    node.timer=SimpleNamespace(cancel=cancel)
    node.source_hash='test-source';node.now_s=lambda:10.
    node.debug=[];node.debug_pub=SimpleNamespace(publish=node.debug.append)
    node.destroyed=[];node.destroy_publisher=node.destroyed.append
    return node


def test_retirement_before_live_replacement_is_refused_and_watchdog_keeps_running():
    node=fixture();node.native_monitor['state']='WAITING'
    reply=node.retire(None,SimpleNamespace())
    assert not reply.success and node.state=='HOLDING' and node.cancelled==0


@pytest.mark.parametrize('bad',['writer_remains','stale_replacement','unhealthy_replacement','wrong_namespace',
    'missing_replacement','no_sensors','bad_mimic','fault'])
def test_unsafe_retirement_does_not_silence_the_active_watchdog(bad):
    node=fixture()
    if bad=='writer_remains':node.arm_pub=object()
    if bad=='stale_replacement':node.native_monitor_wall-=1.
    if bad=='unhealthy_replacement':node.native_monitor['healthy']=False
    if bad=='wrong_namespace':node.get_publishers_info_by_topic=lambda topic:[SimpleNamespace(node_name='native_grasp_monitor',node_namespace='/leader')]
    if bad=='missing_replacement':node.get_publishers_info_by_topic=lambda topic:[]
    if bad=='no_sensors':node.ready=lambda:False
    if bad=='bad_mimic':node.mimic_healthy=lambda:False
    if bad=='fault':node.state='FAULT'
    assert not node.retire(None,SimpleNamespace()).success
    assert node.cancelled==0


def test_released_writer_keeps_grip_watchdog_until_monitor_takeover():
    node=fixture();node.arm_pub=object();writer=node.arm_pub
    assert node.release_writer(None,SimpleNamespace()).success
    assert node.arm_pub is None and node.destroyed==[writer]
    assert node.state=='HOLDING' and node.cancelled==0


def test_explicit_retirement_prevents_shutdown_from_generating_a_false_stop():
    node=fixture()
    assert node.retire(None,SimpleNamespace()).success
    assert node.state=='RELEASED' and node.cancelled==1
    # This used to latch FAULT and issue SwitchController during SIGINT.
    node.fail('shutdown: sensor server no longer ready')
    assert node.state=='RELEASED' and node.cancelled==1
    assert json.loads(node.debug[-1].data)['replacement_monitor_holding']



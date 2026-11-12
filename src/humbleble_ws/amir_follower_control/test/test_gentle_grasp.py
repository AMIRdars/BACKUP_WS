import pytest
import rclpy
from geometry_msgs.msg import WrenchStamped
from amir_follower_control.gentle_grasp import GentleAttach, GentleFrictionGrasp


@pytest.fixture
def grasp():
    rclpy.init(args=['--ros-args','-p','use_sim_time:=true','-p','auto_attach:=false'])
    node=GentleAttach()
    node.searching=True
    yield node
    node.destroy_node()
    rclpy.shutdown()


def test_discrete_contact_filter_retains_the_raw_peak(grasp,monkeypatch):
    stamp=[1.]
    monkeypatch.setattr(grasp,'_now',lambda:stamp[0])
    for value in [0.,6.]*30:
        stamp[0]+=.01
        msg=WrenchStamped()
        msg.wrench.force.x=value
        grasp._force('amir2','left',msg)
    assert 1.<grasp.forces[('amir2','left')][0]<5.
    msg.wrench.force.x=20.
    stamp[0]+=.01
    grasp._force('amir2','left',msg)
    assert max(value for _,value in grasp.force_peaks)==20.
    assert grasp.raw_forces[('amir2','left')][0]==20.


def test_initial_attachment_cannot_bypass_contact_closure_gate(grasp,monkeypatch):
    sent=[]
    monkeypatch.setattr(grasp.verified_pub,'publish',lambda msg:sent.append(msg.data))
    grasp._states={'amir1':True,'amir2':True}
    grasp._gripper_close_complete=False
    grasp._publish_verified()
    assert sent==[False]
    assert not any('arm_controller' in p.topic_name for p in grasp.publishers)


def test_friction_heartbeat_is_local_and_expires(monkeypatch):
    rclpy.init(args=['--ros-args','-p','use_sim_time:=true','-p','auto_attach:=false'])
    node=GentleFrictionGrasp()
    try:
        clock=[1.]
        monkeypatch.setattr(node,'_now',lambda:clock[0])
        sent=[]
        monkeypatch.setattr(node.local_states['amir2'],'publish',lambda msg:sent.append(msg.data))
        node.armed['amir2']=True
        for side in ('left','right'):
            key=('amir2',side)
            node.contact_current[key]=(True,1.)
            node.forces[key]=(3.,1.)
            node.raw_forces[key]=(3.,1.)
        node._local_heartbeat()
        assert sent==['attached'] and node._states['amir1'] is False
        clock[0]=1.2
        node._local_heartbeat()
        assert sent[-1]=='detached'
        # The adapter never publishes an arm or base command.
        assert not any('arm_controller' in p.topic_name or 'rover_twist' in p.topic_name
                       for p in node.publishers)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_friction_hold_relieves_overforce_using_only_own_fingers(monkeypatch):
    rclpy.init(args=['--ros-args','-p','use_sim_time:=true','-p','auto_attach:=false'])
    node=GentleFrictionGrasp()
    try:
        clock=[1.]
        monkeypatch.setattr(node,'_now',lambda:clock[0])
        node._gripper_close_complete=True
        node.positions['amir2']=(-.65,1.)
        for side,load in [('left',4.),('right',8.2)]:
            key=('amir2',side)
            node.contact_current[key]=(True,1.)
            node.forces[key]=(load,1.)
            node.raw_forces[key]=(load,1.)
        monkeypatch.setattr(node._gripper_clients['amir2'],'server_is_ready',lambda:True)
        sent=[]
        monkeypatch.setattr(node,'_send_position',lambda robot,q:sent.append((robot,q)))
        node._regulate()
        assert sent==[('amir2',pytest.approx(-.6503))]
        clock[0]=1.2
        node._regulate()
        assert len(sent)==1  # stale or missing own contact never drives recovery
        from rclpy.parameter import Parameter
        result=node.set_parameters([Parameter('holding_max_step_rad',value=.01)])[0]
        assert not result.successful
        result=node.set_parameters([Parameter('holding_position_gain',value=float('nan'))])[0]
        assert not result.successful
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_friction_latches_own_raw_peak_between_heartbeat_ticks(monkeypatch):
    rclpy.init(args=['--ros-args','-p','use_sim_time:=true','-p','auto_attach:=false'])
    node=GentleFrictionGrasp()
    try:
        clock=[1.]
        monkeypatch.setattr(node,'_now',lambda:clock[0])
        node.searching=True;node.armed['amir2']=True
        def fill(force):
            for side in ('left','right'):
                node.contact_current[('amir2',side)]=(True,clock[0])
                msg=WrenchStamped();msg.wrench.force.x=force
                node._force('amir2',side,msg)
        fill(8.5)
        clock[0]=1.1;fill(3.)
        node._local_heartbeat()
        assert node._states['amir2'] is False
        clock[0]=1.41;fill(3.)
        # An overload of the other robot cannot invalidate this own heartbeat.
        msg=WrenchStamped();msg.wrench.force.x=20.
        node._force('amir1','left',msg)
        node._local_heartbeat()
        assert node._states['amir2'] is True
    finally:
        node.destroy_node();rclpy.shutdown()


def test_fast_holding_updates_at_20ms_without_duplicate_startup_tick(monkeypatch):
    from rclpy.parameter import Parameter
    rclpy.init(args=['--ros-args','-p','use_sim_time:=true','-p','auto_attach:=false'])
    node=GentleFrictionGrasp()
    try:
        result=node.set_parameters_atomically([Parameter('holding_interval_s',value=.02),
            Parameter('holding_position_gain',value=.000012),Parameter('holding_max_step_rad',value=.00004)])
        assert result.successful, result.reason
        clock=[1.];monkeypatch.setattr(node,'_now',lambda:clock[0])
        node._gripper_close_complete=True;node.positions['amir2']=(-.65,1.)
        for side in ('left','right'):
            key=('amir2',side);node.contact_current[key]=(True,1.)
            node.forces[key]=(4.,1.);node.raw_forces[key]=(4.,1.)
        monkeypatch.setattr(node._gripper_clients['amir2'],'server_is_ready',lambda:True)
        sent=[];monkeypatch.setattr(node,'_send_position',lambda r,q:sent.append((r,q)))
        node._holding_tick();node._regulate()
        clock[0]=1.01;node._holding_tick()
        assert len(sent)==1
        clock[0]=1.02;node._holding_tick()
        assert len(sent)==2 and sent[-1][1]==pytest.approx(-.65+.000012)
    finally:
        node.destroy_node();rclpy.shutdown()

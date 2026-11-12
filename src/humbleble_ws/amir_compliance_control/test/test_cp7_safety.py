"""CP-7 regression: arm-epoch handshake, delayed requests and final watchdog."""
import json
import time
import numpy as np
import pytest
import rclpy
from rclpy.task import Future
from rclpy.parameter import Parameter
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from controller_manager_msgs.srv import SwitchController
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from amir_compliance_control.safety import RobotSafetyGate, SafetySupervisor
from amir_compliance_control.interlock import ActuatorInterlock, ControllerAudit
from controller_manager_msgs.srv import ListControllers
from test_shared_safety import Rig as SafetyRig
from test_leader_only import Rig as EndpointRig
from test_recentering import make_rig
from test_recentering import JointOverride


class GuardRig(SafetyRig):
    def __init__(self,path):
        super().__init__(path)
        self.guards={};self.nodes=[];self.audit_enabled=True;self.relay_enabled=True
        for robot,old in list(self.gates.items()):
            self.executor.remove_node(old);old.destroy_node()
            gate=RobotSafetyGate(namespace='/'+robot,parameter_overrides=[Parameter('use_sim_time',value=True),
                Parameter('urdf_file',value=str(path/'test.urdf')),Parameter('profile_file',value=str(path/(robot+'.json')))],
                cli_args=['--ros-args','-r','rover_twist:=interlock/base_request',
                          '-r','arm_controller/joint_trajectory:=interlock/arm_request'])
            self.gates[robot]=gate;self.executor.add_node(gate)
            guard=ActuatorInterlock(namespace='/'+robot,parameter_overrides=[Parameter('use_sim_time',value=True),
                Parameter('urdf_file',value=str(path/'test.urdf')),Parameter('profile_file',value=str(path/(robot+'.json')))])
            self.guards[robot]=guard;self.executor.add_node(guard)
            self.pubs[robot,'audit']=self.driver.create_publisher(String,'/'+robot+'/interlock/audit',10)
            relay=self.driver.create_publisher(Twist,'/'+robot+'/mecanum_drive_controller/reference_unstamped',10)
            self.driver.create_subscription(Twist,'/'+robot+'/rover_twist',
                lambda msg,r=robot,p=relay: p.publish(msg) if self.relay_enabled or r=='leader' else None,10)
        self.executor.remove_node(self.supervisor);self.supervisor.destroy_node()
        self.supervisor=SafetySupervisor(parameter_overrides=[Parameter('use_sim_time',value=True),
            Parameter('check_writers_on_arm',value=False),Parameter('require_interlocks',value=True)])
        self.executor.add_node(self.supervisor)
        self.driver.create_timer(.02,self.audit)
        self.switch_calls=[]
        for robot in ('leader','follower'):
            self.driver.create_service(SwitchController,'/'+robot+'/controller_manager/switch_controller',
                lambda req,res,r=robot:self.switch(r,req,res))

    def switch(self,robot,request,response):
        self.switch_calls.append((robot,list(request.deactivate_controllers)))
        response.ok=True
        return response

    def audit(self):
        for robot in self.guards:
            if self.audit_enabled or robot=='leader':
                self.pubs[robot,'audit'].publish(String(data=json.dumps({'robot':robot,'ready':True,'reason':''})))

    def close(self):
        for node in self.guards.values():self.executor.remove_node(node);node.destroy_node()
        super().close()


@pytest.fixture
def guard_rig(tmp_path):
    rig=GuardRig(tmp_path)
    yield rig
    rig.close()


@pytest.mark.parametrize('fault',['missing_ft','nonfinite','overload','joint_limit','bad_encoder',
                                  'gate_process','supervisor_process','audit_process','relay_process'])
def test_guarded_faults_stop_both_and_do_not_replay(guard_rig,fault):
    rig=guard_rig;rig.start()
    if fault=='gate_process':rig.gates['follower'].timer.cancel()
    elif fault=='supervisor_process':rig.supervisor.timer.cancel()
    elif fault=='audit_process':rig.audit_enabled=False
    elif fault=='relay_process':rig.relay_enabled=False
    else:rig.inject=fault
    rig.pump(.5)
    assert rig.output=={'leader':0.,'follower':0.},(rig.output,[g.fault for g in rig.guards.values()])
    assert any(g.fault for g in rig.guards.values())
    rig.inject=None;rig.audit_enabled=True;rig.relay_enabled=True;rig.pump(.1)
    assert rig.output=={'leader':0.,'follower':0.}
    assert not any(s.topic_name.startswith('/leader/') or 'goal' in s.topic_name or '/tf' in s.topic_name
                   for s in rig.guards['follower'].subscriptions)


def test_final_interlock_death_requests_controller_deactivation(guard_rig):
    rig=guard_rig;rig.start();rig.guards['follower'].timer.cancel();rig.pump(.3)
    assert 'interlock heartbeat missing' in rig.supervisor.fault
    assert not rig.supervisor.armed
    assert rig.supervisor.controller_stops.keys()=={'leader','follower'}


def test_home_waits_for_own_gate_arm_epoch_instead_of_elapsed_time(tmp_path):
    rig=EndpointRig(tmp_path)
    try:
        rig.pump(.3)
        for end in rig.ends.values():assert end.prepare(Trigger.Request(),Trigger.Response()).success
        rig.permit=True
        # Hold back the arm acknowledgement while fresh local sensors continue.
        class WithoutAck:
            def __init__(self,publisher):self.publisher=publisher
            def publish(self,msg):
                value=json.loads(msg.data);value['armed']=False
                self.publisher.publish(String(data=json.dumps(value)))
        for robot in rig.ends:rig.pubs[robot,'gate']=WithoutAck(rig.pubs[robot,'gate'])
        rig.pump(.3)
        assert all(e.enabled_since is None and not e.home_sent and not e.latched for e in rig.ends.values())
        for robot in rig.ends:rig.pubs[robot,'gate']=rig.pubs[robot,'gate'].publisher
        rig.pump(1.9)
        assert all(e.homed and not e.latched for e in rig.ends.values())
    finally:rig.close()


def test_delayed_commands_keep_original_stamps_and_stop_discards_queue(tmp_path):
    rig=EndpointRig(tmp_path)
    try:
        rig.start();end=rig.ends['follower']
        end.set_parameters([Parameter('command_delay_s',value=.1)])
        assert end.enable_test_delay(Trigger.Request(),Trigger.Response()).success
        rig.pump(.025)
        assert end.pending_commands
        message=end.pending_commands[0][2];stamp=message.header.stamp
        end.trip('injected local fault')
        assert not end.pending_commands
        assert message.header.stamp==stamp
        rig.pump(.1);assert rig.output['follower']==0.
    finally:rig.close()


def test_joint_fault_retains_last_valid_hold_pose(guard_rig):
    rig=guard_rig;rig.start();saved=rig.guards['follower'].q.copy()
    rig.inject='joint_limit';rig.pump(.25)
    np.testing.assert_array_equal(rig.guards['follower'].q,saved)
    np.testing.assert_array_equal(rig.guards['follower'].hold,saved)
    assert rig.output=={'leader':0.,'follower':0.}


def test_trajectory_whose_horizon_expired_is_rejected_before_JTC(guard_rig):
    rig=guard_rig;rig.start();gate=rig.gates['follower']
    msg=JointTrajectory(joint_names=list(gate.model.names))
    msg.header.frame_id='follower/base_footprint'
    msg.header.stamp=Time(seconds=rig.sim-.08).to_msg()
    point=JointTrajectoryPoint(positions=[0.]*5);point.time_from_start.nanosec=40000000
    msg.points=[point];gate.arm_command(msg)
    assert 'trajectory ended before receipt' in gate.fault
    rig.pump(.2);assert rig.output=={'leader':0.,'follower':0.}


def test_explicit_guard_reset_rearm_has_no_previous_motion(guard_rig):
    rig=guard_rig;rig.start();rig.inject='overload';rig.pump(.2)
    rig.inject=None;rig.send_commands=False;rig.pump(.2)
    for robot in rig.gates:assert rig.service('/'+robot+'/safety/reset').success
    rig.pump(.08)
    for robot in rig.guards:assert rig.service('/'+robot+'/interlock/reset').success
    rig.pump(.08)
    assert rig.service('/transport/safety/reset').success
    assert rig.service('/transport/safety/arm').success
    rig.pump(.1);assert rig.output=={'leader':0.,'follower':0.}
    rig.send_commands=True;rig.pump(.1)
    assert all(v>0 for v in rig.output.values())
    rig.inject='overload';rig.pump(.25)
    for robot in ('leader','follower'):
        assert sum(r==robot and set(names)=={'arm_controller','mecanum_drive_controller'} for r,names in rig.switch_calls)>=2
    assert rig.output=={'leader':0.,'follower':0.}


def test_one_shot_home_survives_gate_status_arriving_after_trajectory(guard_rig):
    rig=guard_rig;rig.start();guard=rig.guards['follower'];received=[]
    rig.driver.create_subscription(JointTrajectory,'/follower/arm_controller/joint_trajectory',
                                   lambda m:received.append(m),10)
    rig.pump(.05)
    guard.gate=dict(guard.gate,active=False)
    msg=JointTrajectory(joint_names=list(guard.model.names))
    msg.header.stamp=Time(seconds=rig.sim).to_msg();msg.header.frame_id='follower/base_footprint'
    point=JointTrajectoryPoint(positions=[.0001,0.,0.,0.,0.]);point.time_from_start.sec=3
    msg.points=[point];guard.arm(msg)
    rig.pump(.08)
    assert any(m.points and m.points[0].positions[0]==.0001 for m in received)


def test_lost_initial_home_is_retried_with_same_absolute_deadline(tmp_path):
    rig=EndpointRig(tmp_path);received=[]
    try:
        override=JointOverride(rig.pubs['follower','joints']);override.positions=[.01,0.,0.,0.,0.]
        rig.pubs['follower','joints']=override
        def accept_after_first(msg):
            received.append(msg)
            if len(received)>1:override.positions=msg.points[-1].positions
        rig.driver.create_subscription(JointTrajectory,'/follower/arm_joint_cmd',accept_after_first,10)
        rig.start()
        assert len(received)>=2 and rig.ends['follower'].homed
        endings=[m.header.stamp.sec+m.header.stamp.nanosec*1e-9+m.points[-1].time_from_start.sec+
                 m.points[-1].time_from_start.nanosec*1e-9 for m in received]
        assert max(endings)-min(endings)<1e-7
    finally:rig.close()


def test_lost_controller_service_reply_does_not_block_audit_forever():
    rclpy.init();node=ControllerAudit(namespace='/follower')
    try:
        node.timer.cancel();node.destroy_client(node.client)
        class LostThenReady:
            def __init__(self):self.removed=[]
            def remove_pending_request(self,future):self.removed.append(future)
            def service_is_ready(self):return True
            def call_async(self,request):
                future=Future();future.set_result(ListControllers.Response());return future
        node.client=LostThenReady();lost=Future();node.future=lost;node.request_wall=time.monotonic()-1.
        node.tick()
        assert lost.cancelled() and node.client.removed==[lost]
        node.tick()
        assert time.monotonic()-node.controller_wall<.1
    finally:node.destroy_node();rclpy.shutdown()


def test_measured_local_pose_break_stops_without_ground_truth(tmp_path):
    rig=make_rig(tmp_path,[.5,.5,1.])
    try:
        rig.start();end=rig.ends['follower']
        end.set_parameters([Parameter('monitor_pose',value=True)])
        assert end.enable(Trigger.Request(),Trigger.Response()).success
        rig.override.positions=np.array([.02,0.,0.,0.,0.]);rig.pump(.15)
        assert end.latched and end.core is None and end.base is None
        assert 'validated envelope' in end.reason
    finally:rig.close()

"""Signed tracking, release refusal and launch isolation for CP-13."""
import importlib.util
import json
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.time import Time
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from amir_compliance_control.planar_leader import PlanarLeader
from amir_compliance_control.transport import straight_reference
from round_trip_evidence import leg_metrics, normal_passed, fault_passed


ROOT = Path(__file__).resolve().parent.parent
PATH = ROOT/'src/humbleble_ws/cooperative_transport_bringup/launch/compliance_round_trip.launch.py'
spec = importlib.util.spec_from_file_location('cp13_entry', PATH)
entry = importlib.util.module_from_spec(spec)
spec.loader.exec_module(entry)


def context(tmp_path, **overrides):
    ctx = LaunchContext()
    for action in entry.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(ctx)
    ctx.launch_configurations.update(workspace_directory=str(ROOT), headless='true',
        output_directory=str(tmp_path/'new_output'))
    ctx.launch_configurations.update(overrides)
    return ctx


@pytest.mark.parametrize(('key','value'), [
    ('forward_distance_m','0'),('forward_distance_m','2'),('forward_distance_m','nan'),
    ('translation_velocity_m_s','1'),('translation_velocity_m_s','nan'),
    ('return_distance_m','.5'),('translation_acceleration_m_s2','.025'),
    ('rotation_angle_deg','90'),('ros_domain_id','233')])
def test_unverified_route_refused_before_process_creation(tmp_path,key,value):
    with pytest.raises(ValueError):
        entry.setup(context(tmp_path,**{key:value}))


def test_missing_round_trip_proof_cannot_use_forward_only_proof(tmp_path):
    original = json.loads((ROOT/entry.parent.STAGE/'qualified_speed_010.json').read_text())
    from round_trip_evidence import qualification
    with pytest.raises(ValueError,match='round-trip qualification is incomplete'):
        qualification(tmp_path,original)


@pytest.mark.parametrize('gui',[True,False])
def test_separate_command_keeps_gui_free_and_goal_caller_isolated(tmp_path,monkeypatch,gui):
    monkeypatch.setattr(entry,'qualification',lambda root:({},
        {'base_speed_evidence':'unused-test-evidence'}))
    ctx = context(tmp_path,headless='false' if gui else 'true')
    processes = [a for a in entry.setup(ctx) if isinstance(a,ExecuteProcess)]
    assert len(processes) == (2 if gui else 1)
    for p in processes:
        p.prepare(ctx)
    detail = processes[-1].process_details
    cmd = detail['cmd']
    assert cmd[1].endswith('/tools/amir_round_trip_trial.py')
    assert cmd[cmd.index('--distance')+1] == '1.0'
    assert cmd[cmd.index('--speed')+1] == '0.1'
    assert detail['env']['ROS_LOCALHOST_ONLY']=='1'
    assert detail['env']['RMW_FASTRTPS_PUBLICATION_MODE']=='ASYNCHRONOUS'
    assert detail['env']['IGN_PARTITION'].startswith('amir_cp9_round_trip_')
    if gui:
        gui_detail = processes[0].process_details
        assert gui_detail['env']['IGN_PARTITION']==detail['env']['IGN_PARTITION']
        config = Path(gui_detail['cmd'][gui_detail['cmd'].index('--gui-config')+1])
        text = config.read_text()
        assert 'InteractiveViewControl' in text
        assert 'CameraTracking' not in text and '<camera_pose>' not in text
    assert not (tmp_path/'new_output').exists()


def test_production_leader_stops_then_accepts_negative_goal_with_delayed_encoder():
    # Numerical plant only: exercise the production goal acceptance and tracker,
    # with three delayed 20 ms encoder ticks on both signed legs.
    policy = {'max_distance_m':1.,'speed_m_s':.1,'acceleration_m_s2':.01,
        'reference_acceleration_m_s2':.005,'max_yaw_rad':.001,'yaw_metric_m':.1,
        'terminal_timeout_s':10.,'goal_tolerance_m':.000002,
        'position_gain_s_inv':1.,'timeout_s':.15}
    import time
    odom = Odometry(); odom.pose.pose.orientation.w=1.
    faults=[]
    node=SimpleNamespace(state='READY',latched=False,permit=True,emergency=False,
        permit_wall=time.monotonic(),grasp=True,grasp_wall=time.monotonic(),
        encoder=odom,sim=10.,accepted=0,rejected=0,completed=0,
        last_goal_stamp=0.,last_goal_rejection='',local_health=lambda:(True,''),
        p=policy.__getitem__,trip=faults.append,last_tick_sim=10.,
        planar_velocity=np.zeros(3),path_goal=None)
    pending=[0.]*3
    for signed in (1.,-1.):
        goal=PoseStamped()
        goal.header.frame_id='leader/base_footprint'
        goal.header.stamp=Time(seconds=node.sim).to_msg()
        goal.pose.position.x=signed; goal.pose.orientation.w=1.
        node.permit_wall=node.grasp_wall=time.monotonic()
        PlanarLeader.goal(node,goal)
        target=node.accepted
        assert target==(1 if signed>0 else 2)
        if signed>0:
            # Reversing before the first goal completes must be refused.
            busy=PoseStamped();busy.header=goal.header
            busy.pose.position.x=-1.;busy.pose.orientation.w=1.
            PlanarLeader.goal(node,busy)
            assert node.accepted==1 and node.rejected==1
        commands=[]
        for _ in range(2500):
            node.sim+=.02
            v=PlanarLeader.base_request(node,None)[0]
            node.last_tick_sim=node.sim
            pending.append(v);odom.pose.pose.position.x+=pending.pop(0)*.02
            commands.append(v)
            if node.completed==target or faults:
                break
        assert not faults and node.completed==target and node.state=='READY'
        assert max(abs(np.array(commands)))<=.1+1e-12
        assert max(abs(np.diff(commands)))<=.01*.02+1e-12
        assert (max(commands)>.07) if signed>0 else (min(commands)<-.07)
        assert abs(odom.pose.pose.position.x-(1. if signed>0 else 0.))<.00001
        node.sim+=12.
    assert node.accepted==2 and node.completed==2
    assert straight_reference(0.,1.,.1,.005)[2]==pytest.approx(28.2842712475)


def test_signed_ground_truth_cannot_accept_same_direction_for_return():
    origin={r:[0.,0.,0.] for r in ('leader','follower','cooperative_payload')}
    wrong={r:[1.,0.,0.] for r in origin}
    metric=leg_metrics([],origin,wrong,-1.)
    assert all(b['tracking_ratio']==-1. for b in metric['bodies'].values())
    # No absolute-value displacement can turn a second forward leg into return.
    right=leg_metrics([],origin,{r:[-1.,0.,0.] for r in origin},-1.)
    assert all(b['tracking_ratio']==1. for b in right['bodies'].values())


@pytest.mark.parametrize('mutate',[
    lambda r:r.update(goals_after_fault_recovery=3),
    lambda r:r.update(dual_zero_latency_wall_s=.401),
    lambda r:r.update(measured_encoder_vx_at_injection={'leader':-.02,'follower':-.02}),
    lambda r:r.update(no_automatic_restart=False),
])
def test_fault_release_rejects_wrong_direction_restart_and_late_stop(mutate):
    proof={'passed':True,'fault_injected':True,'fault_leg':'return','fault_test':'nonfinite',
        'distance_m':1.,'speed_m_s':.1,'no_automatic_restart':True,'dual_zero_latency_wall_s':.1,
        'goals_at_injection':2,'goals_after_fault_recovery':2,
        'measured_encoder_vx_at_injection':{'leader':-.02,'follower':.02},
        'twists_at_injection':{'leader':[-.02,0.,0.],'follower':[.02,0.,0.]},
        'post_fault_controller_states':{r:{c:'inactive' for c in
            ('arm_controller','mecanum_drive_controller')} for r in ('leader','follower')}}
    assert fault_passed(proof)
    mutate(proof)
    assert not fault_passed(proof)


def test_forward_only_results_cannot_release_round_trip():
    prior=json.loads((ROOT/'audits/whole_body_compliance_20261019/cp12_speed_release/carry_100_18/result.json').read_text())
    assert not normal_passed(prior)


@pytest.mark.parametrize('boundary',['skew','future','age'])
@pytest.mark.parametrize('extra_ns',[0,1])
def test_isolated_gate_exact_timestamp_boundaries(tmp_path,monkeypatch,boundary,extra_ns):
    # Reuse real ROS sensor fixtures with the new gate class. The common gate
    # source and its existing acceptance remain entirely unchanged.
    from round_trip_safety_gate import PreciseSafetyGate
    path=ROOT/'src/humbleble_ws/amir_compliance_control/test/test_shared_safety.py'
    spec=importlib.util.spec_from_file_location('cp13_sensor_fixture',path)
    fixture=importlib.util.module_from_spec(spec);spec.loader.exec_module(fixture)
    monkeypatch.setattr(fixture,'RobotSafetyGate',PreciseSafetyGate)
    rig=fixture.Rig(tmp_path)
    try:
        import time
        from rosgraph_msgs.msg import Clock
        rig.pump(.3)
        gate=rig.gates['follower'];now_ns=60_816_000_000
        gate.clock(Clock(clock=Time(nanoseconds=now_ns).to_msg()))
        gate.grasp_sim=gate.sim;gate.grasp_wall=time.monotonic()
        for key,msg in gate.samples.items():
            if boundary=='skew':
                value=60_832_000_000+extra_ns if key in ('encoder','joints') else 60_812_000_000
            elif boundary=='future':value=now_ns+20_000_000+extra_ns
            else:value=now_ns-50_000_000-extra_ns
            msg.header.stamp=Time(nanoseconds=value).to_msg()
            gate.received[key]=time.monotonic()
        good,reason=gate.healthy()
        assert good is (extra_ns==0),reason
        if extra_ns:assert reason=='local sensors stale, future or skewed'
    finally:rig.close()

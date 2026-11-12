"""AF mathematical, measured feedback, ownership and fail-closed tests."""
import numpy as np
import pytest
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger
from geometry_msgs.msg import WrenchStamped
from amir_compliance_control.strict_arm_only import StrictArmOnly
from amir_compliance_control.arm_only_dls import xy_velocity_dls
from amir_compliance_control.wrench import LocalModel
from test_admittance import ArmRig, model_xml


def test_xy_dls_matches_task_and_preserves_direction():
    m = LocalModel(model_xml()); q = np.zeros(5)
    for v in ([.001, 0.], [-.001, 0.], [0., .001], [0., -.001]):
        d, s, margin, lam = xy_velocity_dls(m, q, v, damping=.0001, singular_stop=.001, margin=.01, speed=.03)
        np.testing.assert_allclose(m.jacobian(q)[:2]@d, v, atol=1e-8)
        assert s > .001 and margin > .01


class AFRig(ArmRig):
    def __init__(self, path, pose_hold=False, command_reference='integrated', control_on_wrench=False, velocity_mode=False):
        super().__init__(path)
        response_file=''
        if command_reference=='servo_trajectory':
            # Synthetic response proof for the mock plant only. Actual Gazebo
            # trials require a separately measured, passing ungrasped probe.
            import json,hashlib
            from pathlib import Path
            from amir_compliance_control.arm_servo_response import model_digest
            response_file=str(path/'mock_servo_response.json')
            names=[f'Joint_{i}' for i in range(1,6)]
            cert={'scope':'simulation_position_servo_response_only','passed':True,'ungrasped':True,
                'joint_names':names,'normalized_urdf_sha256':model_digest(self.file.read_text(),self.file.parent),
                'position_proportional_gain':.1,'controller_update_rate_hz':250,
                'joint_compensation_s':[.04,.04,.04,.04,0.],
                'joint_response':{n:{'samples':100 if i<4 else 0,'velocity_gain_s_inv':25. if i<4 else None}
                                  for i,n in enumerate(names)},
                'plugin_library_sha256':hashlib.sha256(Path('/opt/ros/humble/lib/libgz_hardware_plugins.so').read_bytes()).hexdigest()}
            Path(response_file).write_text(json.dumps(cert))
        old = self.ends['follower']; self.executor.remove_node(old); self.nodes.remove(old); old.destroy_node()
        end = StrictArmOnly(namespace='/follower', parameter_overrides=[
            Parameter('use_sim_time', value=True), Parameter('urdf_file', value=str(self.file)),
            Parameter('simulation_micro_test', value=True), Parameter('force_increment_stop_N', value=.5),
            Parameter('arm_nullspace_pose_hold', value=pose_hold),
            Parameter('arm_command_reference', value=command_reference),
            Parameter('arm_servo_response_file', value=response_file),
            Parameter('control_on_wrench', value=control_on_wrench),
            Parameter('virtual_stiffness',value=[0.,0.] if velocity_mode else [150.,150.]),
            Parameter('force_deadband_N',value=.003 if velocity_mode else 0.),
            Parameter('joint_request_acceleration_rad_s2',value=.15 if velocity_mode else 0.),
            Parameter('torque_increment_stop_Nm', value=.05)])
        self.ends['follower'] = end; self.nodes.append(end); self.executor.add_node(end)


@pytest.fixture(params=[(False,'integrated'),(True,'integrated'),(True,'servo_trajectory')])
def rig(tmp_path, request):
    r = AFRig(tmp_path, pose_hold=request.param[0],command_reference=request.param[1])
    yield r
    r.close()


def test_measured_feedback_and_three_different_errors(rig):
    rig.start(); end = rig.ends['follower']
    assert end.enable(Trigger.Request(), Trigger.Response()).success
    home = end.reference.copy(); rig.pump(.2)
    d = end.measurements
    assert end.p('enable_base_assist') is False
    assert end.p('wrench_source_mode') == 'sensor'
    assert d['r_arm_xy_m'] == pytest.approx(np.linalg.norm(d['e_arm_m'][:2]))
    np.testing.assert_allclose(d['ik_residual_xy_m_s'],
        end.model.jacobian(end.measured_q())[:2]@d['qdot_dls_rad_s']-d['v_ee_cmd_xy_m_s'])
    np.testing.assert_allclose(d['e_arm_m'], [0., 0., 0.])  # mock plant deliberately held
    np.testing.assert_allclose(d['e_track_xy_m'], d['delta_p_adm_m'])
    assert np.linalg.norm(d['qdot_cmd_rad_s']) > 0 and np.linalg.norm(d['delta_p_adm_m']) > 0
    if end.p('arm_command_reference')=='servo_trajectory':
        np.testing.assert_allclose(np.array(d['q_cmd_rad'])-d['q_start_cmd_rad'],
            np.array(d['qdot_dls_rad_s'])*end.p('arm_command_horizon_s'),atol=1e-10)
        assert d['trajectory_points']==2
    np.testing.assert_array_equal(end.reference, home)
    np.testing.assert_array_equal(end.base_request(1.), [0., 0., 0.])
    assert rig.output['follower'] == 0.
    assert not any('leader' in s.topic_name or 'ground_truth' in s.topic_name or 'goal' in s.topic_name for s in end.subscriptions)
    assert not end.set_parameters([Parameter('control_mode', value='ARM_FIRST_BASE')])[0].successful


@pytest.mark.parametrize('fault', ['dropout', 'force', 'drift', 'singularity', 'saturation', 'joint_limit', 'grasp'])
def test_local_fault_stops_both_without_restart(rig, fault):
    rig.start(); end = rig.ends['follower']; assert end.enable(Trigger.Request(), Trigger.Response()).success
    rig.pump(.08)
    if fault == 'dropout': rig.dropout = True
    if fault == 'force': rig.force = [.6, 0.]
    if fault == 'drift': rig.pose['follower'] += .002
    if fault == 'singularity': end.model.jacobian = lambda q: np.zeros((6, 5))
    if fault == 'saturation': end.core.x[0] = end.core.limit[0]; rig.force = [.4, 0.]
    if fault == 'joint_limit': end.model.lower[:] = -.001
    if fault == 'grasp': end.grasp_wall = 0.; end.tick()
    rig.pump(.3)
    assert end.state == 'SAFE_STOP' and end.core is None and rig.stop_calls > 0
    assert all(e.state == 'SAFE_STOP' for e in rig.ends.values())
    assert all(v == 0. for v in rig.output.values())
    assert not end.enable(Trigger.Request(), Trigger.Response()).success


def test_second_command_writer_blocks_prepare(rig):
    from geometry_msgs.msg import TwistStamped
    rig.driver.create_publisher(TwistStamped, '/follower/base_cmd', 1)
    rig.pump(.3)
    result = rig.ends['follower'].prepare(Trigger.Request(), Trigger.Response())
    assert not result.success and 'publisher' in result.message


def test_base_lock_rejects_nonzero_at_final_writer(tmp_path):
    from test_cp7_safety import GuardRig
    from geometry_msgs.msg import Twist
    r = GuardRig(tmp_path)
    try:
        r.start(); guard = r.guards['follower']; guard.base_zero_only = True
        cmd = Twist(); cmd.linear.x = .001; guard.base(cmd)
        assert guard.fault == 'ARM_ONLY rejected nonzero final base request'
        r.pump(.3)
        assert r.output == {'leader': 0., 'follower': 0.}
    finally:r.close()


def test_prearm_zero_is_discarded_but_active_stale_zero_and_motion_trip(tmp_path):
    from test_cp7_safety import GuardRig
    from geometry_msgs.msg import TwistStamped
    from rclpy.time import Time
    r = GuardRig(tmp_path)
    try:
        r.start(); gate = r.gates['follower']; gate.base_zero_only = True
        msg = TwistStamped(); msg.header.frame_id = 'follower/base_footprint'
        msg.header.stamp = Time(seconds=gate.armed_sim).to_msg()
        gate.base_command(msg); assert not gate.fault
        msg.twist.linear.y = .000001; gate.base_command(msg)
        assert 'ARM_ONLY rejected nonzero' in gate.fault
    finally:r.close()


@pytest.mark.parametrize('component', ['linear.z', 'angular.x', 'angular.y'])
def test_nonfinite_unused_axis_cannot_bypass_prearm_zero_lock(tmp_path, component):
    from test_cp7_safety import GuardRig
    from geometry_msgs.msg import TwistStamped
    from rclpy.time import Time
    r = GuardRig(tmp_path)
    try:
        r.start(); gate = r.gates['follower']; gate.base_zero_only = True
        msg = TwistStamped(); msg.header.frame_id = 'follower/base_footprint'
        msg.header.stamp = Time(seconds=gate.armed_sim).to_msg()
        group,axis=component.split('.');setattr(getattr(msg.twist,group),axis,float('nan'))
        gate.base_command(msg)
        assert 'nonfinite' in gate.fault
        r.pump(.2);assert r.output == {'leader':0., 'follower':0.}
    finally:r.close()


def test_postarm_stale_zero_still_trips(tmp_path):
    from test_cp7_safety import GuardRig
    from geometry_msgs.msg import TwistStamped
    from rclpy.time import Time
    r = GuardRig(tmp_path)
    try:
        r.start(); gate = r.gates['follower']; gate.base_zero_only = True
        msg = TwistStamped(); msg.header.frame_id = 'follower/base_footprint'
        msg.header.stamp = Time(seconds=gate.armed_sim+.001).to_msg()
        assert gate.sim-gate.armed_sim > gate.p('command_timeout_s')+.001
        gate.base_command(msg);assert 'stale' in gate.fault
    finally:r.close()


@pytest.mark.parametrize('mode,rotation', [('ARM_FIRST_BASE','false'),('WHOLE_BODY','false'),('ARM_ONLY','true')])
def test_af_launch_rejects_future_base_or_rotation_modes(mode,rotation):
    from pathlib import Path
    from launch import LaunchContext
    import runpy
    setup=runpy.run_path(str(Path(__file__).resolve().parents[1]/'launch/arm_only_af.launch.py'))['setup']
    context=LaunchContext();context.launch_configurations.update(control_mode=mode,rotation_tracking_enabled=rotation)
    with pytest.raises(ValueError,match='ARM_ONLY'):setup(context)


@pytest.mark.parametrize('name,value', [('enable_base_assist',True),('wrench_source_mode','target_proxy'),
                                     ('wrench_source_mode','test_injection')])
def test_controller_rejects_unimplemented_assist_and_non_sensor_source(tmp_path,name,value):
    import rclpy
    path=tmp_path/'model.urdf';path.write_text(model_xml())
    rclpy.init()
    try:
        with pytest.raises(ValueError,match='Phase 1 requires'):
            StrictArmOnly(namespace='/follower',parameter_overrides=[
                Parameter('use_sim_time',value=True),Parameter('urdf_file',value=str(path)),
                Parameter(name,value=value)])
    finally:rclpy.shutdown()


def test_small_straight_reference_keeps_feedback_acceleration_reserve(tmp_path):
    from test_leader_only import Rig
    r = Rig(tmp_path)
    try:
        r.start();leader=r.ends['leader']
        result=leader.set_parameters([Parameter('straight_reference_acceleration_m_s2',value=.00001)])
        assert result[0].successful
        r.goal(.0001);r.pump(3.7)
        assert leader.completed==1 and leader.state=='READY'
        assert abs(r.pose['leader']-.0001)<=leader.p('goal_tolerance_m')
        history=np.asarray(r.command_history);dt=np.diff(history[:,0]);dv=np.diff(history[:,1])
        assert np.max(np.abs(dv[dt>0]/dt[dt>0]))<=leader.p('acceleration_m_s2')+1e-9
    finally:r.close()


def test_wrench_driven_steps_are_unique_and_wall_watchdog_still_stops(tmp_path):
    r=AFRig(tmp_path,pose_hold=True,command_reference='servo_trajectory',control_on_wrench=True)
    try:
        r.start();end=r.ends['follower'];assert end.enable(Trigger.Request(),Trigger.Response()).success
        observed=[];step=end.core.step
        def record(force,dt):
            observed.append(end.force_stamp)
            return step(force,dt)
        end.core.step=record
        r.pump(.2)
        assert len(observed)>3 and len(observed)==len(set(observed))
        assert end.measurements['control_on_wrench'] is True
        r.dropout=True;r.pump(.3)
        assert all(e.state=='SAFE_STOP' for e in r.ends.values())
        assert end.core is None and r.stop_calls>0 and r.output=={'leader':0.,'follower':0.}
    finally:r.close()


def test_velocity_admittance_local_force_deadband_and_dropout_stop_both(tmp_path):
    r=AFRig(tmp_path,pose_hold=True,command_reference='servo_trajectory',control_on_wrench=True,velocity_mode=True)
    try:
        r.start();end=r.ends['follower'];assert end.enable(Trigger.Request(),Trigger.Response()).success
        r.pump(.2)
        np.testing.assert_array_equal(end.core.stiffness,[0.,0.])
        np.testing.assert_allclose(end.control_force(),[.097,0.])
        assert end.core.x[0]>0 and end.core.v[0]>0
        assert r.output=={'leader':0.,'follower':0.}
        r.dropout=True;r.pump(.3)
        assert all(e.state=='SAFE_STOP' for e in r.ends.values())
        assert end.core is None and r.stop_calls>0
    finally:r.close()


@pytest.mark.parametrize('required,state,ready', [
    (False,None,True), (True,None,False), (True,'inactive',False), (True,'active',True)])
def test_gripper_readiness_is_required_only_for_explicit_af_policy(required,state,ready):
    import json
    import time
    import rclpy
    from types import SimpleNamespace
    from amir_compliance_control.interlock import ControllerAudit
    rclpy.init();node=ControllerAudit(namespace='/follower',parameter_overrides=[
        Parameter('require_gripper_active',value=required)])
    try:
        node.timer.cancel();node.controller_wall=time.monotonic()
        node.controllers={k:'active' for k in ('joint_state_broadcaster','mecanum_drive_controller','arm_controller')}
        if state is not None:node.controllers['gripper_controller']=state
        names={'rover_twist':'actuator_interlock','arm_controller/joint_trajectory':'actuator_interlock',
            'interlock/base_request':'safety_gate','interlock/arm_request':'safety_gate',
            'mecanum_drive_controller/reference_unstamped':'rover_twist_relay_ign'}
        node.get_publishers_info_by_topic=lambda topic: ([SimpleNamespace(node_name=names[topic.removeprefix('/follower/')],
            node_namespace='/follower')] if topic.removeprefix('/follower/') in names else [])
        result=[];node.pub=SimpleNamespace(publish=lambda msg:result.append(json.loads(msg.data)))
        node.tick();assert result[-1]['ready'] is ready
        assert ('gripper_controller' in result[-1]['required_controllers']) is required
    finally:node.destroy_node();rclpy.shutdown()


def test_streaming_knots_pass_unchanged_gate_and_preserve_bounds(tmp_path):
    from amir_compliance_control.safety import stamp
    r=AFRig(tmp_path,pose_hold=True,command_reference='servo_trajectory',control_on_wrench=True)
    received=[]
    try:
        r.driver.create_subscription(__import__('trajectory_msgs.msg',fromlist=['JointTrajectory']).JointTrajectory,
            '/follower/arm_joint_cmd',lambda msg:received.append(msg),10)
        r.start();end=r.ends['follower'];assert end.enable(Trigger.Request(),Trigger.Response()).success
        r.pump(.25)
        active=[m for m in received if len(m.points)==2]
        assert len(active)>5 and not end.latched,(end.reason,end.fault_debug)
        for msg in active:
            first,last=msg.points[0],msg.points[-1]
            t=lambda p:p.time_from_start.sec+p.time_from_start.nanosec*1e-9
            assert t(first)==pytest.approx(end.servo_period)
            assert t(last)-t(first)==pytest.approx(end.p('arm_command_horizon_s'))
            assert not first.velocities and not last.velocities
            assert np.max(np.abs(np.array(last.positions)-first.positions)/(t(last)-t(first)))<=.030000001
            for a,b in zip(msg.points,msg.points[1:]):
                assert np.max(np.abs(np.array(b.positions)-a.positions)/(t(b)-t(a)))<=.030000001
        for before,after in zip(active,active[1:]):
            dt=stamp(after)-stamp(before)
            if dt>0:
                for i in range(len(after.points)):
                    assert np.max(np.abs(np.array(after.points[i].positions)-before.points[i].positions)/dt)<=.030000001
        r.dropout=True;r.pump(.3)
        assert all(e.state=='SAFE_STOP' for e in r.ends.values()) and r.output=={'leader':0.,'follower':0.}
    finally:r.close()



"""First qualified native whole-body transport, with leader-only goals.

No angle expansion: actual native O1/O2 evidence must exist before this
experiment. Its measured O2 base rate bounds the leader planning rate.
"""
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import yaml

from amir_compliance_control.wrench import finite
from amir_rotation_native_o2 import configure_o2, NativeO2Recorder


def require_o2_pair(normal_file, fault_file, passive_file):
    passive_hash=hashlib.sha256(Path(passive_file).read_bytes()).hexdigest()
    references={}
    for kind,path,inject in (('normal',Path(normal_file),False),('observer_stop',Path(fault_file),True)):
        report=json.loads(path.read_text())
        if (report.get('condition')!='native_O2_local_compensation'
                or any(report.get(k) is not True for k in ('passed','native_o2_verified','both_outputs_zero_after_stop',
                    'both_grasps_retained_after_stop','fixture_unchanged','leader_input_boundary_passed','follower_input_boundary_passed'))
                or report.get('observer_stop_injected') is not inject or report.get('grasp_loss_events')!=[]
                or report.get('autonomous_follower_yaw_enabled') is not False
                or report.get('native_o2_reference',{}).get('native_passive_result_sha256')!=passive_hash):
            raise ValueError('matching completed native O2 and expected observer-stop evidence required')
        for robot in ('leader','follower'):
            state=report.get('states',{}).get(robot,{})
            if state.get('state')!='SAFE_STOP' or state.get('accepted_goals')!=0 or state.get('completed_goals')!=0:
                raise ValueError('native O2 pair must have zero goals and both stopped')
        measure=report.get('native_o2_measurement',{})
        if measure.get('observer_stop_detected' if inject else 'positive_negative_pulse_completed') is not True:
            raise ValueError('native O2 completion/source-stop proof missing')
        stats=report.get('native_o2_statistics',{})
        if stats.get('local_fault') or stats.get('raw_ft_events',0)<8 or stats.get('moving_samples',0)<1 or stats.get('limited_samples',0)<1:
            raise ValueError('native O2 own measurement/FT evidence incomplete')
        references[kind]=dict(result_file=str(path.resolve()),result_sha256=hashlib.sha256(path.read_bytes()).hexdigest())
        if not inject:normal=report
    # This is the RECEIVED actuator command, separately from the controller's
    # intended limit and from GT. No single-arc 3x3 motion identification.
    csv_file=Path(normal_file).parent/'timeseries.csv';positive=negative=0.
    with csv_file.open() as stream:
        for row in csv.DictReader(stream):
            if not row['phase'].startswith('native_o2_'):continue
            for key in ('leader_output_vx','leader_output_vy','leader_output_wz'):
                if not math.isfinite(float(row[key])) or abs(float(row[key]))>1e-12:
                    raise ValueError('native O2 leader was not stationary')
            speed=float(row['follower_output_wz'])
            if not math.isfinite(speed):raise ValueError('native O2 received command nonfinite')
            positive=max(positive,speed);negative=max(negative,-speed)
    cap=min(positive,negative,float(normal['native_o2_reference']['reduced_yaw_speed_cap_rad_s']))
    if not math.isfinite(cap) or cap<=0.:raise ValueError('measured positive AND negative native O2 base commands required')
    references.update(received_command_csv=str(csv_file.resolve()),
        received_command_csv_sha256=hashlib.sha256(csv_file.read_bytes()).hexdigest(),
        measured_native_yaw_command_cap_rad_s=cap)
    return references


def configure_follow(args,fixture,output,leader_file,follower_file):
    if not np.isfinite([args.delta_deg,args.duration_s]).all() or not 0.<args.delta_deg<=.005 or args.duration_s<75.:
        raise ValueError('initial native follow is limited to positive 0.005 degrees or less, at least 75 s')
    prior=json.loads(Path(args.native_passive_result).read_text())
    if args.delta_deg>prior.get('delta_deg',0.):raise ValueError('native follow cannot expand the completed passive experiment range')
    o2=require_o2_pair(args.native_o2_result,args.native_o2_fault_result,args.native_passive_result)
    reference=configure_o2(args.native_passive_result,fixture,output,follower_file)
    reference.pop('autonomous_follower_yaw_enabled',None)
    follower=yaml.safe_load(Path(follower_file).read_text())
    p=follower['/follower/follower_yaw_whole_body']['ros__parameters']
    for key in ('probe_angle_rad','probe_duration_s','probe_hold_s','probe_qualification_file'):p.pop(key,None)
    gains=[args.xy_gain,args.xy_gain,args.yaw_gain]
    if not np.isfinite(gains).all() or min(gains)<=0.:raise ValueError('explicit existing positive recentering gains required')
    p.update(base_gains=gains,passive_yaw_calibration=False,rotation_observation_required=True,
        arm_yaw_admittance_enabled=True,base_force_acceleration_gains=[0.,0.,0.],base_force_velocity_gains=[0.,0.,0.])
    policy=yaml.safe_load(Path(args.native_allocation_policy).read_text())
    allocation=policy['/follower/follower_yaw_whole_body']['ros__parameters']
    if (set(allocation)!={'allocation_base_regularization','allocation_task_compliance_budget'}
            or allocation['allocation_task_compliance_budget'] is not True
            or not math.isfinite(allocation['allocation_base_regularization']) or allocation['allocation_base_regularization']<=0.):
        raise ValueError('explicit bounded whole-body allocation policy required')
    p.update(allocation)
    # Never restore the earlier larger yaw cap after an O2-only rate reduction.
    cap=min(o2['measured_native_yaw_command_cap_rad_s'],p['base_velocity_limits'][2])
    p['base_velocity_limits'][2]=cap
    Path(follower_file).write_text(yaml.safe_dump(follower))
    data=yaml.safe_load(Path(leader_file).read_text());leader=data['/leader/leader_object_rotation']['ros__parameters']
    limits=next(iter(yaml.safe_load(Path(args.passive_leader_limits_file).read_text()).values()))['ros__parameters']
    for key in ('rotation_max_angular_speed','rotation_max_angular_acceleration','rotation_max_base_speed',
                'rotation_max_base_acceleration','rotation_max_grasp_speed'):
        values=finite([leader[key],limits[key]],(2,))
        if min(values)<=0.:raise ValueError('explicit existing positive leader planning caps required')
        leader[key]=float(min(values))
    leader['rotation_max_angular_speed']=min(leader['rotation_max_angular_speed'],cap)
    # Plan for the larger relative negative goal. The leader additionally
    # enforces its exact latched radii/acceleration minimum at runtime.
    duration=max(args.duration_s,float(prior['passive_policy']['duration_s']),
        1.875*math.radians(2.*args.delta_deg)/leader['rotation_max_angular_speed'])
    leader.update(rotation_duration_s=duration,rotation_max_yaw_rad=math.radians(2.*args.delta_deg),actuator_ack_required=True)
    Path(leader_file).write_text(yaml.safe_dump(data))
    reference.update(o2_pair=o2,mode='WHOLE_BODY',goal_duration_s=duration,delta_deg=args.delta_deg,
        leader_angular_speed_cap_rad_s=leader['rotation_max_angular_speed'],base_gains=gains,
        allocation_policy_sha256=hashlib.sha256(Path(args.native_allocation_policy).read_bytes()).hexdigest(),
        angle_accuracy_is_diagnostic=True,requires_live_qualified_readiness=True,
        scope='new small-angle native transport experiment; no angle expansion')
    return reference


class NativeFollowRecorder(NativeO2Recorder):
    def __init__(self,output,phase,**kwargs):
        fields=['sim_time_s','phase','state','e_x_m','e_y_m','e_yaw_rad','vx_m_s','vy_m_s','wz_rad_s',
            'tau_z_Nm','grasp_utilization','allocation_variables','consumed_sequence','snapshot_matched','source_age_ns','fault']
        super().__init__(output,phase,filename_prefix='native_follow',csv_fields=fields,**kwargs)
        self.maximum_allocation_output_difference=0.;self.minimum_variables=None

    def record(self,value):
        self.debug=value
        if not self.active():return
        self.count+=1;consumed=value.get('consumed_observation',{});sequence=consumed.get('publication_sequence')
        observation=self.observations.get(sequence)
        if observation is None:self.missing+=1
        command=value.get('base_twist');velocity=value.get('velocity');error=value.get('measured_error_xy_yaw')
        try:
            if value.get('state')=='SAFE_STOP':raise ValueError('native follower stopped: '+str(value.get('reason')))
            if value.get('enabled'):
                u=finite(command,(3,));allocated=finite(velocity,(8,));finite(error,(3,))
                difference=float(np.max(abs(u-allocated[5:])))
                self.maximum_allocation_output_difference=max(self.maximum_allocation_output_difference,difference)
                self.minimum_variables=8
                if difference>64*np.finfo(float).eps*max(float(np.max(abs(u))),np.finfo(float).tiny):
                    raise ValueError('native WHOLE_BODY emitted a second base command outside its allocator')
            age=consumed.get('source_age_ns')
            if age is not None:self.maximum_age_ns=max(self.maximum_age_ns,age)
        except (ValueError,KeyError,TypeError) as exc:self.fault=self.fault or str(exc)
        self.raw.write(json.dumps(dict(phase=self.phase[0],debug=value,consumed_observation=observation,
            exact_consumed_snapshot_match=observation is not None,fault=self.fault))+'\n')
        u=command if isinstance(command,list) and len(command)==3 else [None]*3
        e=error if isinstance(error,list) and len(error)==3 else [None]*3
        self.writer.writerow(dict(sim_time_s=value.get('sim_time'),phase=self.phase[0],state=value.get('state'),
            e_x_m=e[0],e_y_m=e[1],e_yaw_rad=e[2],vx_m_s=u[0],vy_m_s=u[1],wz_rad_s=u[2],
            tau_z_Nm=value.get('tau_z_at_tcp_Nm'),grasp_utilization=value.get('grasp_utilization'),
            allocation_variables=None if velocity is None else len(velocity),consumed_sequence=sequence,
            snapshot_matched=observation is not None,source_age_ns=consumed.get('source_age_ns'),fault=self.fault))

    def summary(self):
        return dict(local_samples=self.count,missing_consumed_snapshot_rows=self.missing,
            maximum_consumed_source_age_ns=self.maximum_age_ns,raw_ft_events=self.raw_events,
            maximum_incremental_force_N=self.maximum_force,maximum_incremental_moment_Nm=self.maximum_moment,
            minimum_allocation_variables=self.minimum_variables,
            maximum_allocation_base_output_difference=self.maximum_allocation_output_difference,local_fault=self.fault)


def execute_follow(args,plan,*,service,settle,wait,phase,goal,states,recorder,ready,source_hashes):
    from std_msgs.msg import Float64
    previous=recorder.latest.get('reference_sequence',0)
    service('/follower/rotation/latch_reference')
    wait(lambda:recorder.latest.get('reference_sequence',0)>previous and recorder.latest.get('yaw_valid') is True,10.,True)
    service('/follower/admittance/enable')
    wait(lambda:ready.get('value') is True and recorder.debug.get('enabled') is True
         and isinstance(recorder.debug.get('velocity'),list) and len(recorder.debug['velocity'])==8,10.,True)
    phase[0]='native_follow_baseline';settle(3.)
    if any(hashlib.sha256(Path(file).read_bytes()).hexdigest()!=digest for file,digest in source_hashes.items()):
        raise ValueError('native follow source changed before first goal')
    for name,delta,count in (('positive',args.delta_deg,1),('negative',-2.*args.delta_deg,2)):
        if ready.get('value') is not True:raise ValueError('native qualified follower readiness withdrawn')
        phase[0]='native_follow_'+name;goal.publish(Float64(data=math.radians(delta)))
        wait(lambda:states.get('leader',{}).get('accepted_goals')==count,10.,True)
        wait(lambda:states.get('leader',{}).get('completed_goals')==count,max(1000.,plan['goal_duration_s']*5.),True)
        settle(3.)
    if recorder.fault or recorder.raw_events<8 or recorder.minimum_variables!=8:
        raise ValueError(recorder.fault or 'native single-allocator own measurement evidence missing')
    if states.get('follower',{}).get('accepted_goals')!=0:raise ValueError('native follower received a prohibited task goal')
    return dict(positive_negative_programs_completed=True,angle_accuracy_is_diagnostic=True,
        leader_goal_values_rad=[math.radians(args.delta_deg),math.radians(-2.*args.delta_deg)],
        completion_policy='two leader programs with qualified local following and grasp retained; final stop/hold is mandatory')


def evaluation_diagnostics(rows):
    """After motion only; GT never enters a controller or qualifier."""
    base=[r for r in rows if r['phase']=='native_follow_baseline' and 'object_center_GT_x' in r]
    moving=[r for r in rows if r['phase'] in ('native_follow_positive','native_follow_negative') and 'object_center_GT_x' in r]
    if not base or not moving:return dict(evaluation_only=True,physical_center_fix_achieved=False)
    initial=np.array([base[-1]['object_center_GT_'+a] for a in 'xyz'])
    points=np.array([[r['object_center_GT_'+a] for a in 'xyz'] for r in moving])
    result=dict(evaluation_only=True,physical_center_fix_achieved=False,
        maximum_object_center_drift_m=float(np.max(np.linalg.norm(points-initial,axis=1))),
        maximum_object_height_change_m=float(np.max(abs(points[:,2]-initial[2]))))
    tilt=[math.acos(float(np.clip(r['object_vertical_axis_dot_z'],-1.,1.))) for r in moving
        if r.get('object_vertical_axis_dot_z') is not None]
    result['maximum_object_vertical_tilt_rad']=max(tilt,default=None)
    for robot in ('leader','follower'):
        for suffix,label in (('_grasp_constraint_position_error_m','position_change_m'),
                ('_object_grasp_relation_rotation_change_rad','rotation_change_rad')):
            values=[r[robot+suffix] for r in moving if robot+suffix in r]
            result['maximum_'+robot+'_object_grasp_relation_'+label]=max(values,default=None)
    for name in ('positive','negative'):
        data=[r for r in moving if r['phase']=='native_follow_'+name]
        if data:
            angle=data[-1]['object_yaw_GT_rad']-base[-1]['object_yaw_GT_rad']
            result[name+'_object_yaw_from_initial_rad']=math.atan2(math.sin(angle),math.cos(angle))
    return result

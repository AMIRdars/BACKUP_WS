"""O2 calibration only: one follower base, local omega pulse, shared C limiter.

No object yaw goal, peer state or truth input. The production leader is never
authorized by this probe. All inherited FT, pose, observer and shared stops
remain active. Normal transport never instantiates this executable.
"""
import math
import numpy as np
import rclpy
from std_srvs.srv import Trigger

from .rotation_follow import RotationFollower
from .rotation_observer import read_qualification
from .rotation_recentering import RotationRecentering


def pulse_rate(elapsed,angle,duration,hold):
    if not np.isfinite([elapsed,angle,duration,hold]).all() or duration<=0. or hold<0.:
        raise ValueError('finite explicit probe profile required')
    if elapsed<0.:return 0.,'waiting'
    if elapsed<duration:delta,u,phase=angle,elapsed/duration,'positive'
    elif elapsed<duration+hold:return 0.,'hold'
    elif elapsed<2*duration+hold:
        delta,u,phase=-angle,(elapsed-duration-hold)/duration,'negative'
    else:return 0.,'complete'
    return delta/duration*30*u*u*(1-u)**2,phase


class ProbeRecentering(RotationRecentering):
    def __init__(self,rate,**kwargs):
        super().__init__(**kwargs);self.rate=rate;self.last={}

    def step(self,error,position,dt,*,yaw_valid):
        if not yaw_valid:raise ValueError('O2 probe requires qualified own yaw')
        requested,phase=self.rate()
        command=self.limit_command([0.,0.],requested,position,dt)
        residual=np.r_[command[:2],0.]+np.cross([0.,0.,command[2]],position)
        self.last=dict(probe_phase=phase,probe_requested_wz_rad_s=requested,
            probe_limited_twist_footprint=command.tolist(),probe_position_footprint_m=list(position),
            probe_compensation_footprint_m_s=self.compensation.tolist(),
            probe_rigid_point_velocity_footprint_m_s=residual.tolist())
        return command


class CompensationProbe(RotationFollower):
    def __init__(self,**kwargs):
        self.probe_start=None
        super().__init__(**kwargs)
        if self.mode!='XY_YAW_COMP' or self.passive_calibration or np.any(np.array(self.p('base_gains'))!=0.):
            raise ValueError('O2 requires exclusive XY_YAW_COMP and zero recentering gains')
        self.declare_parameter('probe_qualification_file','')
        cert=read_qualification(self.p('probe_qualification_file'),self.urdf_hash,self.yaw_geometry)
        if cert is None:raise ValueError('measured O1 qualification required for O2')
        self.probe_profile=tuple(float(self.p(k)) for k in ('probe_angle_rad','probe_duration_s','probe_hold_s'))
        bound=.5*min(cert['measured_signal_rad']['positive'],-cert['measured_signal_rad']['negative'])
        if not 0.<abs(self.p('probe_angle_rad'))<=bound:
            raise ValueError('O2 pulse outside half the measured passive yaw range')
        self.role='O2_LOCAL_COMPENSATION_PROBE'
        self.create_service(Trigger,'rotation/start_compensation_probe',self.start_probe)

    def new_base(self):
        for key,value in [('probe_angle_rad',0.),('probe_duration_s',0.),('probe_hold_s',0.)]:
            if not self.has_parameter(key):self.declare_parameter(key,value)
        if not np.isfinite([self.p('probe_angle_rad'),self.p('probe_duration_s'),self.p('probe_hold_s')]).all() or self.p('probe_duration_s')<=0. or self.p('probe_hold_s')<0.:
            raise ValueError('explicit bounded probe profile required')
        return ProbeRecentering(self.probe_rate,mode='XY_YAW_COMP',gains=self.p('base_gains'),
            velocity=self.p('base_velocity_limits'),acceleration=self.p('base_acceleration_limits'),
            cutoff=self.p('base_cutoff_hz'))

    def probe_rate(self):
        return (0.,'inactive') if self.probe_start is None else pulse_rate(self.sim-self.probe_start,
            *self.probe_profile)

    def start_probe(self,request,response):
        try:
            self.require_yaw()
            if self.probe_start is not None or self.core is None or self.latched or not self.permit or self.emergency:
                raise ValueError('fresh enabled stationary local follower and one probe per run required')
            twist=self.encoder.twist.twist
            if max(abs(twist.linear.x),abs(twist.linear.y),abs(twist.angular.z))>=.0001:
                raise ValueError('stationary local base required before probe')
            self.probe_start=self.sim+.05
            response.success,response.message=True,'O2 local base pulse started; no object goal or leader authorization'
        except (ValueError,KeyError,TypeError) as exc:response.success,response.message=False,str(exc)
        return response

    def update_local(self):
        super().update_local()
        if self.arm_task is not None and self.base is not None:
            self.arm_task.last.update(self.base.last)

    def may_authorize_rotation(self):
        return False


def main(args=None):
    rclpy.init(args=args);node=CompensationProbe()
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.try_shutdown()

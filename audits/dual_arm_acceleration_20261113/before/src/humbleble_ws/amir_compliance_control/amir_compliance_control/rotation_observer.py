"""Chapter 23 projected FK yaw. This measures a local angle, never intent.

The TCP x axis must remain separated from vertical throughout the existing
orientation envelope. A successful matrix logarithm is not a substitute for
the specified atan2(R[1,0], R[0,0]) measurement.
"""
import hashlib
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET
from collections import deque

import numpy as np

from .wrench import LocalModel, finite, origin


def wrap_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_noise_std(window):
    """Remove motion predicted by measured qdot, not a commanded angle.

    A slow real rotation must not count as increased stationary sensor noise.
    Per-sample FK/velocity innovation remains independently monitored.
    """
    data=np.asarray(window,dtype=float)
    if data.ndim!=2 or data.shape[1]!=3 or len(data)<2 or not np.isfinite(data).all():
        raise ValueError('finite measured time/yaw/rate window required')
    dt=np.diff(data[:,0])
    if np.any(dt<=0.):raise ValueError('monotonic measured yaw noise window required')
    predicted=np.r_[0.,np.cumsum(.5*(data[1:,2]+data[:-1,2])*dt)]
    return float(np.std(np.unwrap(data[:,1])-predicted,ddof=1))


class ProjectedYaw:
    def __init__(self, xml, *, orientation_envelope_rad, joint_margin_rad,
                 heading_frame='tcp_link'):
        self.model = LocalModel(xml)
        self.envelope = float(orientation_envelope_rad)
        self.margin = float(joint_margin_rad)
        if not 0. < self.envelope < math.pi/2 or not 0. < self.margin:
            raise ValueError('explicit existing pose/joint envelopes required')
        root = ET.fromstring(xml)
        tcp=root.find("joint[@name='tcp_fixed']")
        if tcp is None or tcp.get('type')!='fixed' or tcp.find('parent').get('link')!='gripper_base_1' or tcp.find('child').get('link')!='tcp_link':
            raise ValueError('verified fixed wrist-to-TCP transform required')
        self.tcp_offset = origin(tcp.find('origin'))
        self.heading_frame = heading_frame
        self.tcp_T_heading = np.eye(4)
        if heading_frame == 'grasp_heading':
            # Explicit alternative measurement condition, never an automatic
            # axis switch. The existing wrist-to-TCP displacement defines
            # approach +Y. Relabel x=+Y, y=-Z, z=-X without moving the TCP.
            if (not np.allclose(self.tcp_offset[:3,:3],np.eye(3),atol=1e-12)
                    or not np.allclose(self.tcp_offset[[0,2],3],0.,atol=1e-12)
                    or self.tcp_offset[1,3]<=0.):
                raise ValueError('grasp_heading requires verified positive-Y TCP approach')
            self.tcp_T_heading[:3,:3]=[[0.,0.,-1.],[1.,0.,0.],[0.,-1.,0.]]
        elif heading_frame != 'tcp_link':
            raise ValueError('explicit tcp_link or grasp_heading frame required')
        self.measurement = ('projected_tcp_x_axis_base_link' if heading_frame=='tcp_link'
                            else 'projected_grasp_heading_x_axis_base_link')
        self.geometry_signature=hashlib.sha256(json.dumps(dict(measurement=self.measurement,
            heading_frame=heading_frame,tcp_T_heading=self.tcp_T_heading.tolist(),
            orientation_envelope_rad=self.envelope,joint_margin_rad=self.margin),sort_keys=True).encode()).hexdigest()
        # Exact serial-chain invariance: the LAST revolute axis is parallel
        # to the heading axis, and TCP translation is along the same axis.
        # Its wrist roll cannot change either this heading or TCP position.
        last_axis=self.model.axes[-1]/np.linalg.norm(self.model.axes[-1])
        heading=self.tcp_offset[:3,:3]@self.tcp_T_heading[:3,0]
        self.yaw_invariant_joint_indices=([4] if heading_frame=='grasp_heading'
            and np.linalg.norm(np.cross(last_axis,heading))<1e-12
            and np.linalg.norm(np.cross(last_axis,self.tcp_offset[:3,3]))<1e-12 else [])
        parents = {j.find('child').get('link'):j for j in root.findall('joint')}
        link, chain,visited = 'base_link', [],set()
        while link != 'base_footprint':
            if link in visited or link not in parents:raise ValueError('disconnected/cyclic local base frame')
            visited.add(link)
            joint = parents[link]
            if joint.get('type') != 'fixed':
                raise ValueError('base_link must have a fixed local transform')
            chain.insert(0, joint)
            link = joint.find('parent').get('link')
        self.footprint_T_base = np.eye(4)
        for joint in chain:
            self.footprint_T_base = self.footprint_T_base@origin(joint.find('origin'))
        if not np.allclose(self.footprint_T_base[2,:3], [0.,0.,1.], atol=1e-9):
            raise ValueError('planar base frame required')
        # An exact vertical FIRST revolute axis rotates the entire remaining
        # chain about local z. Its azimuth changes psi by the same angle,
        # without changing projection clearance or any yaw sensitivity.
        # Merely observing a nonzero numerical Jacobian is not this proof.
        prefix=np.eye(4);first=None
        for joint in self.model.chain:
            prefix=prefix@origin(joint.find('origin'))
            if joint.get('type')!='fixed':first=joint;break
        axis=prefix[:3,:3]@self.model.axes[0]
        self.yaw_equivariance_proof=(dict(joint_name='Joint_1',joint_index=0,
            axis_footprint=axis.tolist(),origin_footprint_m=prefix[:3,3].tolist(),
            projected_yaw_gain=1.,projection_and_yaw_jacobian_invariant=True)
            if first.get('name')=='Joint_1' and first.get('type')=='revolute'
            and np.allclose(axis,[0.,0.,1.],rtol=0.,atol=1e-12) else None)

    def measure(self, q, *, compute_jacobian=True):
        q = finite(q, (5,))
        position, sensor = self.model.frames(q)
        rotation = self.footprint_T_base[:3,:3].T@(
            sensor[:3,:3]@self.tcp_offset[:3,:3]@self.tcp_T_heading[:3,:3])
        axis = rotation[:,0]
        projection = float(np.linalg.norm(axis[:2]))
        clearance = math.atan2(projection, abs(float(axis[2])))
        margin = float(np.min(np.r_[q-self.model.lower,self.model.upper-q]))
        geometry_valid = clearance > self.envelope and margin > self.margin
        reason = ('projected '+self.heading_frame+' x axis can cross vertical inside the pose envelope'
                  if clearance <= self.envelope else 'joint software margin reached'
                  if margin <= self.margin else 'geometry valid; passive qualification still required')
        p_base = self.footprint_T_base[:3,:3].T@(
            position-self.footprint_T_base[:3,3])
        psi = math.atan2(float(axis[1]),float(axis[0])) if projection > 0. else None
        yaw_jacobian=None
        if geometry_valid and compute_jacobian:
            angular=self.footprint_T_base[:3,:3].T@self.model.jacobian(q)[3:]
            derivative=np.cross(angular.T,axis).T
            yaw_jacobian=(axis[0]*derivative[1]-axis[1]*derivative[0])/projection**2
        return dict(p_ee_B=p_base.tolist(),p_ee_footprint=position.tolist(),
            yaw_heading_frame=self.heading_frame,measurement=self.measurement,
            geometry_signature=self.geometry_signature,tcp_T_heading=self.tcp_T_heading.tolist(),
            psi_ee_B=psi,projection_norm=projection,
            yaw_jacobian=None if yaw_jacobian is None else yaw_jacobian.tolist(),
            vertical_clearance_rad=clearance,joint_margin_rad=margin,
            geometry_valid=geometry_valid,valid_reason=reason)


def read_qualification(file, urdf_sha256, geometry=None):
    """A calibration constant may certify detectability; it contains no goal."""
    if not file:
        return None
    cert = json.loads(Path(file).read_text())
    if (cert.get('robot') != 'follower' or cert.get('urdf_sha256') != urdf_sha256
            or cert.get('measurement') != (geometry.measurement if geometry else 'projected_tcp_x_axis_base_link')
            or cert.get('passive_positive_negative_passed') is not True
            or not cert.get('evidence_sha256')):
        raise ValueError('matching measured positive/negative passive qualification required')
    if geometry is not None and (cert.get('geometry_signature')!=geometry.geometry_signature
            or cert.get('yaw_heading_frame')!=geometry.heading_frame):
        raise ValueError('passive qualification measurement frame/envelope mismatch')
    if geometry is not None and 'yaw_invariant_joint_indices' in cert and (
            cert['yaw_invariant_joint_indices']!=geometry.yaw_invariant_joint_indices
            or cert.get('invariant_joint_pose_envelope_rad')!=geometry.envelope):
        raise ValueError('URDF-proved heading invariance/preserved pose envelope mismatch')
    if 'yaw_equivariance_proof' in cert and (geometry is None or geometry.yaw_equivariance_proof is None
            or cert['yaw_equivariance_proof']!=geometry.yaw_equivariance_proof
            or cert.get('equivariant_joint_pose_envelope_rad')!=geometry.envelope):
        raise ValueError('exact first-axis yaw equivariance/preserved pose envelope mismatch')
    if np.any(finite(cert['joint_noise_std_rad'], (5,))<0.):raise ValueError('negative joint noise')
    for key in ('yaw_noise_std_rad','yaw_innovation_limit_rad','stationary_joint_speed_rad_s'):
        if not math.isfinite(cert[key]) or cert[key]<=0.:raise ValueError('measured positive noise/innovation policies required')
    if 'yaw_residual_std_limit_rad' in cert and (not math.isfinite(cert['yaw_residual_std_limit_rad']) or cert['yaw_residual_std_limit_rad']<=0.):
        raise ValueError('positive measured dynamic residual noise limit required')
    if not isinstance(cert['noise_window_samples'],int) or cert['noise_window_samples']<8:
        raise ValueError('measured noise window required')
    if np.any(finite(cert['qualified_q_lower_rad'],(5,))>finite(cert['qualified_q_upper_rad'],(5,))):
        raise ValueError('invalid measured qualification region')
    return cert


def qualify_passive(samples,geometry,*,force_limit_N,torque_limit_Nm,evidence_sha256):
    """Offline Gate-0/1/2. No certificate on absent, flat or wrong-sign yaw.

    The follower base must be held during this calibration. Its measured FK
    signal, NOT the requested/GT angle, establishes local detectability.
    Six-sigma separation and three-sigma online variance are software
    confidence policies; neither is a physical load rating.
    """
    if not evidence_sha256 or not np.isfinite([force_limit_N,torque_limit_Nm]).all() or min(force_limit_N,torque_limit_Nm)<=0.:
        raise ValueError('measured force envelope and evidence hashes required')
    groups={k:[] for k in ('static','positive','negative')}
    previous=None
    for row in samples:
        if row['phase'] not in groups:raise ValueError('explicit passive calibration phases required')
        q=finite(row['q_rad'],(5,));qd=finite(row['qdot_rad_s'],(5,))
        m=geometry.measure(q)
        if not m['geometry_valid']:raise ValueError(m['valid_reason'])
        if row.get('grasp_attached') is not True or np.max(abs(finite(row['base_twist'],(3,))))>=.0001:
            raise ValueError('held grasp and stationary follower base required for passive qualification')
        if np.any(abs(qd)>geometry.model.velocity):raise ValueError('measured joint velocity outside URDF')
        t=float(row['timestamp'])
        if not math.isfinite(t) or (previous is not None and not 0.<t-previous<=.05):
            raise ValueError('passive evidence timestamp discontinuity')
        previous=t
        groups[row['phase']].append(dict(q=q,qd=qd,w=finite(row['wrench_tcp'],(6,)),t=t,
            psi=m['psi_ee_B'],rate=float(np.array(m['yaw_jacobian'])@qd)))
    if any(len(v)<8 for v in groups.values()):raise ValueError('at least eight measured samples per phase required')
    static=groups['static'];sigma=max(float(np.std(np.unwrap([r['psi'] for r in static]),ddof=1)),np.finfo(float).eps)
    psi0=float(np.mean(np.unwrap([r['psi'] for r in static])))
    signal={}
    for name in ('positive','negative'):
        tail=groups[name][-max(4,len(groups[name])//4):]
        signal[name]=float(np.median([wrap_pi(r['psi']-psi0) for r in tail]))
    if signal['positive']<=6*sigma or signal['negative']>=-6*sigma:
        raise ValueError('positive/negative measured yaw not detectable above six-sigma static noise')
    baseline=np.mean([r['w'] for r in static],axis=0)
    records=sorted([r for name in ('static','positive','negative') for r in groups[name]],key=lambda r:r['t'])
    if any(np.linalg.norm((r['w']-baseline)[:3])>=force_limit_N or np.linalg.norm((r['w']-baseline)[3:])>=torque_limit_Nm for r in records):
        raise ValueError('passive evidence exceeds measured wrench envelope')
    noise=np.maximum(np.std([r['q'] for r in static],axis=0,ddof=1),np.finfo(float).eps)
    innovations=[abs(wrap_pi(b['psi']-a['psi'])-.5*(a['rate']+b['rate'])*(b['t']-a['t'])) for a,b in zip(records,records[1:])]
    variation=[.5*abs(b['rate']-a['rate'])*(b['t']-a['t'])+6*sigma for a,b in zip(records,records[1:])]
    if any(error>bound for error,bound in zip(innovations,variation)):
        raise ValueError('passive FK yaw and measured joint velocity are inconsistent')
    window=min(32,len(static))
    residual_limit=max(yaw_noise_std([(r['t'],r['psi'],r['rate']) for r in records[i-window:i]])
                       for i in range(window,len(records)+1))+3*sigma
    return dict(measurement=geometry.measurement,yaw_heading_frame=geometry.heading_frame,
        yaw_invariant_joint_indices=geometry.yaw_invariant_joint_indices,
        invariant_joint_pose_envelope_rad=geometry.envelope,
        geometry_signature=geometry.geometry_signature,tcp_T_heading=geometry.tcp_T_heading.tolist(),
        passive_positive_negative_passed=True,
        evidence_sha256=evidence_sha256,joint_noise_std_rad=noise.tolist(),yaw_noise_std_rad=sigma,
        yaw_innovation_limit_rad=max(innovations,default=0.)+6*sigma,
        stationary_joint_speed_rad_s=max(float(np.max(abs(r['qd']))) for r in static)+6*float(max(noise))/float(np.median(np.diff([r['t'] for r in static]))),
        noise_window_samples=window,yaw_residual_std_limit_rad=residual_limit,W_0=baseline.tolist(),
        qualified_q_lower_rad=(np.min([r['q'] for r in records],axis=0)-6*noise).tolist(),
        qualified_q_upper_rad=(np.max([r['q'] for r in records],axis=0)+6*noise).tolist(),
        measured_signal_rad=signal,minimum_signal_to_noise=min(signal['positive'],-signal['negative'])/sigma)


def observer_type():
    """Create the ROS type lazily; offline geometry stays ROS independent."""
    import rclpy
    from geometry_msgs.msg import WrenchStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, String
    from std_srvs.srv import Trigger
    from rclpy.clock import Clock, ClockType
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data,QoSProfile,ReliabilityPolicy
    from .safety import stamp

    class RotationObserver(Node):
        def __init__(self,**kwargs):
            super().__init__('rotation_observer',**kwargs)
            for k,v in {'urdf_file':'','qualification_file':'','yaw_heading_frame':'tcp_link',
                        'allow_loaded_reference_latch':False,
                        'orientation_envelope_rad':0.,'joint_margin_rad':0.,
                        'timeout_s':.15,'source_timeout_s':.05}.items():
                self.declare_parameter(k,v)
            self.p = lambda k:self.get_parameter(k).value
            if self.get_namespace()!='/follower' or not self.get_parameter('use_sim_time').value:
                raise ValueError('follower simulation namespace required')
            xml = Path(self.p('urdf_file')).read_text()
            self.geometry = ProjectedYaw(xml,orientation_envelope_rad=self.p('orientation_envelope_rad'),
                                         joint_margin_rad=self.p('joint_margin_rad'),heading_frame=self.p('yaw_heading_frame'))
            self.urdf_hash = hashlib.sha256(xml.encode()).hexdigest()
            self.cert = read_qualification(self.p('qualification_file'), self.urdf_hash,self.geometry)
            self.latest, self.receipt = {}, {}
            self.declare_parameter('align_own_source_history',False)
            self.declare_parameter('publish_on_raw_measurement',False)
            self.publish_on_raw=bool(self.p('publish_on_raw_measurement'))
            self.align_sources=bool(self.p('align_own_source_history'))
            self.source_history={k:deque(maxlen=64) for k in
                ('joint_states','mecanum_drive_controller/odometry','wrench_at_grasp_raw')}
            self.reference, self.armed = None, False
            self.reference_sequence=0
            self.valid_reference_sequence=None
            self.online_fault=''
            self.previous, self.continuity_fault = None, ''
            self.noise_window=deque(maxlen=self.cert['noise_window_samples'] if self.cert else 32)
            # This is a state snapshot, not the raw F/T event stream. Keep
            # only the newest snapshot; online invalidity is sticky below.
            self.pub = self.create_publisher(String,'rotation/observation',1)
            for topic,typ in [('joint_states',JointState),('mecanum_drive_controller/odometry',Odometry),
                              ('wrench_at_grasp_raw',WrenchStamped)]:
                self.create_subscription(typ,topic,lambda m,k=topic:self.receive(k,m),
                    10 if topic=='wrench_at_grasp_raw' else
                    QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT) if self.align_sources else qos_profile_sensor_data)
            self.create_subscription(String,'grasp/state',lambda m:self.receive('grasp/state',m),10)
            if self.p('allow_loaded_reference_latch'):
                self.create_subscription(String,'rotation_follow/debug',lambda m:self.receive('rotation_follow/debug',m),10)
            self.create_subscription(Bool,'/transport/safety_permit',lambda m:setattr(self,'armed',m.data),1)
            allowed={'/clock','/transport/safety_permit',*['/follower/'+k for k in
                ('joint_states','mecanum_drive_controller/odometry','wrench_at_grasp_raw','grasp/state')]}
            if self.p('allow_loaded_reference_latch'):allowed.add('/follower/rotation_follow/debug')
            if {s.topic_name for s in self.subscriptions}!=allowed or self.pub.topic_name!='/follower/rotation/observation':
                raise ValueError('observer local input/output boundary remapped')
            self.create_service(Trigger,'rotation/latch_reference',self.latch)
            # FK continuity is measured in source/simulation time. A wall
            # timer can skip >50 ms of source samples when Gazebo runs faster
            # than real time. Retain an independent wall receipt watchdog.
            self.create_timer(.02,self.tick)
            self.create_timer(.02,self.watchdog,clock=Clock(clock_type=ClockType.STEADY_TIME))

        def receive(self,key,msg):
            self.latest[key],self.receipt[key]=msg,time.monotonic()
            if key in self.source_history:self.source_history[key].append(msg)
            # A real new F/T sample can remove the periodic publication wait.
            # Both periodic and independent wall watchdogs remain active.
            if key=='wrench_at_grasp_raw' and self.publish_on_raw and self.reference is not None:
                self.tick()

        def require_region(self,q):
            if self.cert is None or self.reference is None:return
            outside=(q<np.array(self.cert['qualified_q_lower_rad'])) | (q>np.array(self.cert['qualified_q_upper_rad']))
            for i in self.cert.get('yaw_invariant_joint_indices',[]):
                if abs(q[i]-self.reference['q_rad'][i])>self.geometry.envelope:
                    raise ValueError('heading-invariant wrist exceeds existing pose envelope')
                outside[i]=False
            if 'yaw_equivariance_proof' in self.cert:
                i=self.cert['yaw_equivariance_proof']['joint_index']
                if abs(q[i]-self.reference['q_rad'][i])>self.geometry.envelope:
                    raise ValueError('yaw-equivariant shoulder exceeds existing pose envelope')
                outside[i]=False
            if np.any(outside):raise ValueError('outside measured passive qualification joint region')

        def observation(self):
            now=self.get_clock().now().nanoseconds*1e-9
            out=dict(timestamp=now,frame_id='follower/base_link',urdf_sha256=self.urdf_hash,
                     reference_sequence=self.reference_sequence,
                     yaw_valid=False,valid_reason='waiting for own measurements')
            try:
                keys=('joint_states','mecanum_drive_controller/odometry','wrench_at_grasp_raw','grasp/state')
                out['receipt_ages_wall_s']={k:None if k not in self.receipt else time.monotonic()-self.receipt[k] for k in keys}
                missing=[k for k in keys if k not in self.latest or out['receipt_ages_wall_s'][k]>self.p('timeout_s')]
                if missing:raise ValueError('own measurement receipt watchdog: '+', '.join(missing))
                samples={k:self.latest[k] for k in keys[:3]}
                latest_stamps=[stamp(samples[k]) for k in keys[:3]]
                # Old/future LATEST inputs cannot be rescued by older history.
                if any(not -20_000_000<=round((now-s)*1e9)<=round(self.p('source_timeout_s')*1e9) for s in latest_stamps):
                    raise ValueError('own measurements stale/future/skewed')
                latest_measure=None
                if self.align_sources:
                    current=samples['joint_states'];mapping=dict(zip(current.name,current.position))
                    latest_q=finite([mapping[n] for n in self.geometry.model.names],(5,))
                    mapping=dict(zip(current.name,current.velocity))
                    latest_qdot=finite([mapping[n] for n in self.geometry.model.names],(5,))
                    latest_measure=self.geometry.measure(latest_q)
                    if not latest_measure['geometry_valid']:raise ValueError(latest_measure['valid_reason'])
                    w=samples['wrench_at_grasp_raw'].wrench;t=samples['mecanum_drive_controller/odometry'].twist.twist
                    finite([w.force.x,w.force.y,w.force.z,w.torque.x,w.torque.y,w.torque.z],(6,))
                    finite([t.linear.x,t.linear.y,t.angular.z],(3,))
                    anchor=min(latest_stamps)
                    samples={k:min([*self.source_history[k],self.latest[k]],key=lambda m:abs(stamp(m)-anchor)) for k in keys[:3]}
                stamps=[stamp(samples[k]) for k in keys[:3]]
                out.update(source_stamps=dict(zip(keys[:3],stamps)),latest_source_stamps=dict(zip(keys[:3],latest_stamps)),
                    source_skew_s=float(np.ptp(stamps)))
                ages_ns=[round((now-s)*1e9) for s in stamps]
                if any(not -20_000_000<=age<=round(self.p('source_timeout_s')*1e9) for age in ages_ns) or round(float(np.ptp(stamps))*1e9)>20_000_000:
                    raise ValueError('own measurements stale/future/skewed')
                # Projected yaw is FK(q) alone, not a fusion with odom/F/T.
                # Keep its newest source timestamp. The synchronized health
                # tuple retains its own timestamps and the original skew cap.
                joints=self.latest['joint_states'] if self.align_sources else samples['joint_states']
                mapping=dict(zip(joints.name,joints.position))
                q=finite([mapping[n] for n in self.geometry.model.names],(5,))
                out.update(latest_measure if self.align_sources else self.geometry.measure(q),joint_stamp=stamp(joints))
                out['health_source_stamps']=dict(zip(keys[:3],stamps))
                out['yaw_source_stamp']=stamp(joints)
                velocities=dict(zip(joints.name,joints.velocity))
                qdot=finite([velocities[n] for n in self.geometry.model.names],(5,))
                odom=samples['mecanum_drive_controller/odometry'].twist.twist
                wrench=samples['wrench_at_grasp_raw'].wrench
                out.update(q_rad=q.tolist(),qdot_rad_s=qdot.tolist(),
                    base_twist=finite([odom.linear.x,odom.linear.y,odom.angular.z],(3,)).tolist(),
                    wrench_tcp=finite([wrench.force.x,wrench.force.y,wrench.force.z,wrench.torque.x,wrench.torque.y,wrench.torque.z],(6,)).tolist(),
                    grasp_attached=self.latest['grasp/state'].data=='attached')
                yaw_rate=None if out['yaw_jacobian'] is None else float(np.array(out['yaw_jacobian'])@qdot)
                out['psi_dot_ee_B_rad_s']=yaw_rate
                # Continuity/noise still monitor the NEWEST joint sample,
                # independently of the aligned output. Never hide a new jump.
                cq,cqd,ct,cm=(latest_q,latest_qdot,latest_stamps[0],latest_measure) if self.align_sources else (q,qdot,stamps[0],out)
                cr=None if cm['yaw_jacobian'] is None else float(np.array(cm['yaw_jacobian'])@cqd)
                out['continuity_joint_stamp']=ct
                if self.align_sources and self.previous is not None and ct==self.previous[0] and not np.array_equal(cq,self.previous[1]):
                    self.continuity_fault=self.continuity_fault or 'joint/FK discontinuity'
                if self.previous is not None and ct!=self.previous[0]:
                    dt=ct-self.previous[0]
                    noise=np.zeros(5) if self.cert is None else 6*np.array(self.cert['joint_noise_std_rad'])
                    if not 0.<dt<=self.p('source_timeout_s') or np.any(abs(cq-self.previous[1])>self.geometry.model.velocity*dt+noise+1e-12):
                        self.continuity_fault=self.continuity_fault or 'joint/FK discontinuity'
                    if self.cert and cr is not None and self.previous[3] is not None and dt>0.:
                        innovation=abs(wrap_pi(cm['psi_ee_B']-self.previous[2])-.5*(cr+self.previous[3])*dt)
                        out['yaw_innovation_rad']=innovation
                        if innovation>self.cert['yaw_innovation_limit_rad']:
                            self.continuity_fault=self.continuity_fault or 'FK yaw/velocity innovation outside measured qualification'
                        if max(abs(cqd))<=self.cert['stationary_joint_speed_rad_s']:
                            self.noise_window.append((ct,cm['psi_ee_B'],cr))
                            if len(self.noise_window)==self.noise_window.maxlen:
                                sigma=yaw_noise_std(self.noise_window);out['stationary_yaw_std_rad']=sigma
                                if sigma>self.cert.get('yaw_residual_std_limit_rad',3*self.cert['yaw_noise_std_rad']):
                                    self.continuity_fault=self.continuity_fault or 'FK yaw residual noise increased'
                        else:self.noise_window.clear()
                self.previous=(ct,cq.copy(),cm['psi_ee_B'],cr)
                if not out['geometry_valid']:raise ValueError(out['valid_reason'])
                if self.latest['grasp/state'].data!='attached':raise ValueError('own grasp not held')
                if self.continuity_fault:raise ValueError(self.continuity_fault)
                if self.reference is None:raise ValueError('stationary reference not latched')
                out.update(p_home_B=self.reference['p_ee_B'],p_home_footprint=self.reference['p_ee_footprint'],
                    psi_home_B=self.reference['psi_ee_B'],W_0=self.reference['W_0'])
                out['e_xy']=(np.array(out['p_ee_B'])-out['p_home_B'])[:2].tolist()
                out['e_yaw_rad']=wrap_pi(out['psi_ee_B']-out['psi_home_B'])
                if self.cert is None:raise ValueError('Gate-1 positive/negative passive qualification missing')
                if self.align_sources:self.require_region(latest_q)
                self.require_region(q)
                out['yaw_valid'],out['valid_reason']=True,'qualified local projected FK yaw'
            except (ValueError,KeyError,TypeError) as exc:
                out['valid_reason']=str(exc)
            return out

        def latch(self,request,response):
            out=self.observation()
            twist=self.latest.get('mecanum_drive_controller/odometry')
            stationary=twist is not None and max(abs(twist.twist.twist.linear.x),abs(twist.twist.twist.linear.y),abs(twist.twist.twist.angular.z))<.0001
            stationary=stationary and np.max(abs(np.array(out.get('qdot_rad_s',[np.inf]))))<.001
            if self.align_sources:
                try:
                    current=self.latest['joint_states'];v=dict(zip(current.name,current.velocity))
                    stationary=stationary and np.max(abs(finite([v[n] for n in self.geometry.model.names],(5,))))<.001
                except (ValueError,KeyError,TypeError):stationary=False
            loaded_allowed=False
            if self.armed and self.p('allow_loaded_reference_latch'):
                try:
                    debug=json.loads(self.latest['rotation_follow/debug'].data)
                    sources=self.get_publishers_info_by_topic('/follower/rotation_follow/debug')
                    loaded_allowed=bool(len(sources)==1 and sources[0].node_name=='follower_yaw_whole_body'
                        and sources[0].node_namespace=='/follower'
                        and time.monotonic()-self.receipt['rotation_follow/debug']<=self.p('timeout_s')
                        and -.02<=out['timestamp']-debug['sim_time']<=self.p('source_timeout_s')
                        and debug.get('enabled') is False and debug.get('state')=='READY'
                        and np.max(abs(finite(debug['base_twist'],(3,))))<1e-12)
                except (ValueError,KeyError,TypeError):pass
            response.success=bool((not self.armed or loaded_allowed) and stationary and out.get('geometry_valid')
                and self.latest.get('grasp/state') is not None and self.latest['grasp/state'].data=='attached'
                and 'p_ee_B' in out and 'stale' not in out['valid_reason'] and 'watchdog' not in out['valid_reason'])
            if response.success:
                w=self.latest['wrench_at_grasp_raw'].wrench
                self.reference=dict(out,W_0=[w.force.x,w.force.y,w.force.z,w.torque.x,w.torque.y,w.torque.z])
                self.reference_sequence+=1
                self.valid_reference_sequence=None
                self.online_fault=''
                self.continuity_fault=''
                self.previous=None;self.noise_window.clear()
            response.message=('stationary reference latched; qualification unchanged' if response.success
                else 'disarm or verify an inactive stationary local controller before latching' if self.armed
                else 'stationary base and arm required before latching' if not stationary
                else out['valid_reason'])
            return response

        def tick(self):
            started=time.monotonic()
            out=self.observation()
            if self.armed and self.valid_reference_sequence==self.reference_sequence and not out['yaw_valid']:
                self.online_fault=self.online_fault or out['valid_reason']
            if out['yaw_valid'] and self.armed:
                self.valid_reference_sequence=self.reference_sequence
            if self.online_fault:
                out.update(yaw_valid=False,valid_reason='latched online yaw fault: '+self.online_fault)
            self.publication_sequence=getattr(self,'publication_sequence',0)+1
            out.update(publication_sequence=self.publication_sequence,
                observation_duration_wall_s=time.monotonic()-started,
                publication_wall_s=time.monotonic())
            self.pub.publish(String(data=json.dumps(out,allow_nan=False)))

        def watchdog(self):
            keys=('joint_states','mecanum_drive_controller/odometry','wrench_at_grasp_raw','grasp/state')
            if any(k not in self.receipt or time.monotonic()-self.receipt[k]>self.p('timeout_s') for k in keys):
                self.tick()

    return RotationObserver


def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node=observer_type()()
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.try_shutdown()

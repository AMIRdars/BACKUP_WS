"""ROS-free control mathematics. All spatial quantities use the local base axes.

The 4th task component is spatial angular velocity about local z, not an
Euler-angle derivative. No leader state or world pose enters this module.
"""
from dataclasses import dataclass
import xml.etree.ElementTree as ET

import numpy as np


def finite(value, shape=None):
    a = np.asarray(value, dtype=float)
    if (shape is not None and a.shape != shape) or not np.all(np.isfinite(a)):
        raise ValueError('invalid shape or nonfinite state')
    return a


def skew(v):
    x, y, z = v
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def rotation(axis, angle):
    axis = finite(axis, (3,))
    n = np.linalg.norm(axis)
    if n < 1e-12:
        raise ValueError('zero joint axis')
    k = skew(axis / n)
    return np.eye(3) + np.sin(angle) * k + (1 - np.cos(angle)) * k @ k


def origin(element):
    t = np.eye(4)
    if element is not None:
        t[:3, 3] = finite([float(x) for x in element.get('xyz', '0 0 0').split()], (3,))
        r, p, y = [float(x) for x in element.get('rpy', '0 0 0').split()]
        t[:3, :3] = rotation([0, 0, 1], y) @ rotation([0, 1, 0], p) @ rotation([1, 0, 0], r)
    return t


class ArmURDF:
    """Serial URDF chain, including fixed joints, with geometric Jacobian."""

    def __init__(self, xml, base='base_footprint', grasp='tcp_link',
                 sensor='gripper_base_1', names=tuple(f'Joint_{i}' for i in range(1, 6))):
        root = ET.fromstring(xml)
        parents = {j.find('child').get('link'): j for j in root.findall('joint')}
        self.chain = []
        current = grasp
        while current != base:
            if current not in parents or len(self.chain) > len(parents):
                raise ValueError('missing or cyclic URDF chain')
            j = parents[current]
            self.chain.insert(0, j)
            current = j.find('parent').get('link')
        self.names, self.sensor, self.base, self.grasp = tuple(names), sensor, base, grasp
        movable = [j.get('name') for j in self.chain if j.get('type') != 'fixed']
        if movable != list(names) or len(names) != 5:
            raise ValueError('expected audited five arm joints in chain order')
        if sensor not in [j.find('child').get('link') for j in self.chain]:
            raise ValueError('sensor frame is outside local chain')
        limits = [j.find('limit') for j in self.chain if j.get('type') != 'fixed']
        self.lower = np.array([float(x.get('lower')) for x in limits])
        self.upper = np.array([float(x.get('upper')) for x in limits])
        self.velocity = np.array([float(x.get('velocity')) for x in limits])
        self.effort = np.array([float(x.get('effort')) for x in limits])
        sensor_joint=parents[sensor].get('name')
        if sensor_joint not in self.names:
            raise ValueError('audited sensor must be on a movable joint child')
        self.sensor_joint_index=self.names.index(sensor_joint)
        # Mass upstream of the sensor is not included in its wrench. Include
        # fixed side attachments as well as the serial links, but never count
        # the gripper/payload side twice.
        depths={base:0}
        depth=0
        for joint in self.chain:
            depth+=int(joint.get('type')!='fixed')
            depths[joint.find('child').get('link')]=depth
        self.upstream_masses=[]
        for link in root.findall('link'):
            inertial=link.find('inertial')
            if inertial is None:continue
            current=link.get('name')
            path=[]
            while current not in depths:
                if current not in parents:raise ValueError('disconnected inertial link')
                joint=parents[current]
                path.insert(0,joint)
                current=joint.find('parent').get('link')
                if len(path)>len(parents):raise ValueError('cyclic inertial branch')
            count=depths[current]
            if count==0 or count>self.sensor_joint_index:continue
            offset=np.eye(4)
            for joint in path:
                if joint.get('type')!='fixed':
                    raise ValueError('unobserved movable mass upstream of wrist sensor')
                offset=offset@origin(joint.find('origin'))
            mass=float(inertial.find('mass').get('value'))
            if not np.isfinite(mass) or mass<0:raise ValueError('invalid URDF mass')
            com=(offset@origin(inertial.find('origin')))[:3,3]
            self.upstream_masses.append((current,count,mass,com))

    def _kinematics(self,q):
        q = finite(q, (5,))
        t = np.eye(4)
        axes, points = [], []
        frames = {self.base: t.copy()}
        index = 0
        for j in self.chain:
            t = t @ origin(j.find('origin'))
            if j.get('type') != 'fixed':
                if j.get('type') != 'revolute':
                    raise ValueError('only audited revolute arm joints supported')
                axis = np.array([float(x) for x in j.find('axis').get('xyz').split()])
                axes.append(t[:3, :3] @ (axis / np.linalg.norm(axis)))
                points.append(t[:3, 3].copy())
                r = np.eye(4)
                r[:3, :3] = rotation(axis, q[index])
                t = t @ r
                index += 1
            frames[j.find('child').get('link')] = t.copy()
        return frames,np.asarray(axes),np.asarray(points)

    def evaluate(self, q, full_jacobian=False):
        frames,axes,points=self._kinematics(q)
        t=frames[self.grasp]
        jac = np.vstack((np.array([np.cross(a, t[:3, 3] - p)
                                  for a, p in zip(axes, points)]).T, np.array(axes).T))
        return t, frames[self.sensor], jac if full_jacobian else jac[[0, 1, 2, 5], :]

    def quasistatic_joint_load(self,q,raw,sign,gravity=(0.,0.,-9.81)):
        """Model estimate, not measured actuator torque.

        Uses the measured downstream wrist wrench plus URDF gravity upstream
        of the sensor. Excludes upstream inertia/friction and assumes a planar
        base with the audited gravity vector expressed in local B.
        """
        if sign not in (-1.,1.) or self.sensor_joint_index!=4:
            raise ValueError('terminal audited wrist and calibrated sign required')
        raw=finite(raw,(6,))
        gravity=finite(gravity,(3,))
        frames,axes,points=self._kinematics(q)
        sensor=frames[self.sensor]
        force=sign*sensor[:3,:3]@raw[:3]
        moment=sign*sensor[:3,:3]@raw[3:]
        result=np.array([axis@(moment+np.cross(sensor[:3,3]-point,force))
                         for axis,point in zip(axes,points)])
        for anchor,count,mass,offset in self.upstream_masses:
            com=frames[anchor][:3,3]+frames[anchor][:3,:3]@offset
            for i in range(count):
                result[i]+=axes[i]@np.cross(com-points[i],mass*gravity)
        return finite(result,(5,))


def transform_wrench(raw, t_bs, p_bg, sign):
    """Shift S-origin to G then express in B, including r_GS cross force."""
    raw, t_bs, p_bg = finite(raw, (6,)), finite(t_bs, (4, 4)), finite(p_bg, (3,))
    if sign not in (-1., 1.):
        raise ValueError('sensor sign requires measured +1 or -1')
    f = sign * t_bs[:3, :3] @ raw[:3]
    tau = sign * t_bs[:3, :3] @ raw[3:] + np.cross(t_bs[:3, 3] - p_bg, f)
    return np.r_[f, tau]


def check_timing(now, stamps, max_age=.05, max_skew=.02):
    stamps = finite(stamps)
    ages = now - stamps
    if not np.isfinite(now) or np.any(stamps <= 0) or np.any(ages < -1e-6):
        raise ValueError('invalid or future sensor stamp')
    if np.max(ages) > max_age + 1e-9 or np.ptp(stamps) > max_skew + 1e-9:
        raise ValueError('stale or skewed local state')
    return ages


def interpolate_joints(samples, stamp, max_gap=.02):
    """Interpolate local FK input at sensor time; never extrapolate to latest.

    samples contains (stamp_s, q) pairs for the five bounded revolute joints.
    """
    if not np.isfinite(stamp):
        raise ValueError('nonfinite interpolation stamp')
    previous = None
    for time_s,q in samples:
        q = finite(q,(5,))
        if abs(time_s-stamp) < 1e-9:
            return q.copy()
        if previous is not None and previous[0] < stamp < time_s:
            t0,q0 = previous
            if time_s-t0 > max_gap+1e-9:
                raise ValueError('joint interpolation gap')
            return q0+(q-q0)*(stamp-t0)/(time_s-t0)
        previous = (time_s,q)
    raise ValueError('no joint state bracket at wrench timestamp')


class WrenchObserver:
    def __init__(self, samples=100, cutoff_hz=8.):
        if samples < 2 or not np.isfinite(cutoff_hz) or cutoff_hz <= 0:
            raise ValueError('invalid bias sample count or filter cutoff')
        self.samples, self.cutoff = samples, cutoff_hz
        self.count, self.mean, self.filtered = 0, np.zeros(6), None

    def update(self, wrench, dt, calibrating=False):
        w = finite(wrench, (6,))
        if not 0 < dt <= .05:
            raise ValueError('invalid sensor interval')
        if calibrating:
            self.count += 1
            self.mean += (w - self.mean) / self.count
        a = 1 - np.exp(-2 * np.pi * self.cutoff * dt)
        self.filtered = w.copy() if self.filtered is None else a * w + (1-a) * self.filtered
        return (self.filtered - self.mean)[[0, 1, 2, 5]]


class Admittance4D:
    def __init__(self, axes=(True, False, False, False), mass=(3.,3.,3.,.4),
                 damping=(35.,35.,35.,4.), stiffness=(20.,20.,20.,2.),
                 vmax=(.03,.03,.02,.08), xmax=(.05,.05,.03,.1)):
        self.axes = np.asarray(axes, dtype=bool)
        if self.axes.shape != (4,):
            raise ValueError('four axes required')
        self.mass, self.damping, self.stiffness, self.vmax, self.xmax = [
            finite(v, (4,)) for v in (mass,damping,stiffness,vmax,xmax)]
        if (np.any(self.mass <= 0) or np.any(self.damping <= 0) or
                np.any(self.stiffness < 0) or np.any(self.vmax <= 0) or np.any(self.xmax <= 0)):
            raise ValueError('nonpositive admittance parameters')
        self.x, self.v = np.zeros(4), np.zeros(4)

    def step(self, wrench, dt):
        if not np.isfinite(dt) or not 0 < dt <= .05:
            raise ValueError('clock discontinuity')
        w = finite(wrench, (4,))
        a = (w - self.damping * self.v - self.stiffness * self.x) / self.mass
        self.v = np.clip(self.v + dt*a, -self.vmax, self.vmax) * self.axes
        next_x = self.x + dt*self.v
        hit = (np.abs(next_x) >= self.xmax) & (self.v * next_x > 0)
        self.v[hit] = 0  # anti-windup: no velocity into saturated offsets
        self.x = np.clip(next_x, -self.xmax, self.xmax) * self.axes
        return self.v.copy()


def base_jacobian(p_bg):
    x, y, _ = finite(p_bg, (3,))
    return np.array([[1., 0., -y], [0., 1., x], [0., 0., 0.], [0., 0., 1.]])


class Recenter:
    def __init__(self,start_distance=.02,stop_distance=.008):
        if not np.isfinite(start_distance) or not np.isfinite(stop_distance) or not 0<stop_distance<start_distance:
            raise ValueError('recenter requires 0 < stop distance < start distance')
        self.start_distance,self.stop_distance=start_distance,stop_distance
        self.active = False
        self.command = np.zeros(3)

    def step(self, position_error, yaw_error, dt, enabled=False):
        if not 0 < dt <= .05:
            raise ValueError('invalid recenter interval')
        e = finite(position_error, (3,))
        if not np.isfinite(yaw_error):
            raise ValueError('invalid yaw')
        metric = max(np.linalg.norm(e[:2]), abs(yaw_error) * .2)
        self.active = enabled and metric > (self.stop_distance if self.active else self.start_distance)
        target = np.clip(np.r_[.3*e[:2], .3*yaw_error], [-.02, -.02, -.04], [.02, .02, .04])
        if not self.active:
            target[:] = 0
        self.command += np.clip(target-self.command, -dt*np.array([.02,.02,.04]),
                                dt*np.array([.02,.02,.04]))
        return self.command.copy()


@dataclass
class Allocation:
    q_target: np.ndarray
    qdot: np.ndarray
    residual: np.ndarray
    singular_values: np.ndarray


def allocate(arm, q, previous_qdot, j4, velocity4, base_velocity, dt,
             p_bg, damping=.02, margin=.05, max_acceleration=.5):
    """Bounded DLS allocation, subtracting the base contribution at G."""
    if not 0 < dt <= .05 or damping <= 0:
        raise ValueError('invalid allocation period or damping')
    q, j4 = finite(q, (5,)), finite(j4, (4, 5))
    if np.any(q < arm.lower+margin) or np.any(q > arm.upper-margin):
        raise ValueError('arm joint limit margin')
    desired = finite(velocity4, (4,)) - base_jacobian(p_bg) @ finite(base_velocity, (3,))
    # A rank-deficient pose may still execute Fx. Refuse requests outside the
    # attainable task subspace instead of silently coupling lateral/yaw axes.
    projected = j4 @ np.linalg.lstsq(j4, desired, rcond=1e-5)[0]
    unreachable = desired-projected
    if np.max(np.abs(unreachable[:3])) > .002 or abs(unreachable[3]) > .01:
        raise ValueError('requested task outside current arm Jacobian range')
    pinv = np.linalg.solve(j4 @ j4.T + damping*damping*np.eye(4), j4).T
    qdot = pinv @ projected
    # Preserve the available uncommanded orientation in the true null space.
    # Moving toward URDF midlimits at zero wrench changes the clamped object's
    # pitch and creates an internal moment. No unsolicited neutral motion.
    _,singular,vh = np.linalg.svd(j4,full_matrices=True)
    rank=int(np.sum(singular>1e-5))
    null=vh[rank:].T
    if null.shape[1]:
        angular_xy=arm.evaluate(q,full_jacobian=True)[2][3:5]
        correction=np.linalg.lstsq(angular_xy@null,-angular_xy@qdot,rcond=1e-5)[0]
        qdot += null@correction
    bound = np.minimum(arm.velocity, .2)
    lo = np.maximum(-bound, (arm.lower+margin-q)/dt)
    hi = np.minimum(bound, (arm.upper-margin-q)/dt)
    prev = finite(previous_qdot, (5,))
    lo = np.maximum(lo, prev-max_acceleration*dt)
    hi = np.minimum(hi, prev+max_acceleration*dt)
    if np.any(lo > hi):
        raise ValueError('infeasible joint acceleration/position bounds')
    # One interpolation factor preserves the coordinated joint direction.
    # Independent clipping can reintroduce a pitch command during acceleration.
    change=qdot-prev
    scale=1.
    for i in range(5):
        if change[i]>1e-12:
            scale=min(scale,(hi[i]-prev[i])/change[i])
        elif change[i]<-1e-12:
            scale=min(scale,(lo[i]-prev[i])/change[i])
    qdot=prev+np.clip(scale,0.,1.)*change
    if np.any(qdot<lo-1e-10) or np.any(qdot>hi+1e-10):
        raise ValueError('infeasible coordinated joint bounds')
    return Allocation(q+qdot*dt, qdot, desired-j4@qdot, np.linalg.svd(j4, compute_uv=False))


def allowed_subscriptions(robot='amir2'):
    p = '/' + robot.strip('/')
    return {p+'/ft_sensor', p+'/joint_states', p+'/mecanum_drive_controller/odometry',
            p+'/grasp/state', '/clock'}

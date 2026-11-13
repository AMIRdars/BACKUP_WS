"""Local URDF FK and spatial wrench processing, with no ROS dependencies."""
import xml.etree.ElementTree as ET
import numpy as np


def finite(value, shape):
    result = np.asarray(value, dtype=float)
    if result.shape != shape or not np.all(np.isfinite(result)):
        raise ValueError('nonfinite or malformed local state')
    return result


def rotation(axis, angle):
    a = finite(axis, (3,))
    if np.linalg.norm(a) < 1e-12:
        raise ValueError('zero joint axis')
    x, y, z = a/np.linalg.norm(a)
    k = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3)+np.sin(angle)*k+(1-np.cos(angle))*k@k


def origin(element):
    t = np.eye(4)
    if element is not None:
        t[:3, 3] = finite(element.get('xyz', '0 0 0').split(), (3,))
        r, p, y = finite(element.get('rpy', '0 0 0').split(), (3,))
        t[:3, :3] = rotation([0, 0, 1], y)@rotation([0, 1, 0], p)@rotation([1, 0, 0], r)
    return t


class LocalModel:
    def __init__(self, xml):
        root = ET.fromstring(xml)
        parents = {j.find('child').get('link'): j for j in root.findall('joint')}
        current, self.chain = 'tcp_link', []
        while current != 'base_footprint':
            if current not in parents or len(self.chain) > len(parents):
                raise ValueError('disconnected or cyclic local URDF chain')
            joint = parents[current]
            self.chain.insert(0, joint)
            current = joint.find('parent').get('link')
        self.names = tuple(f'Joint_{i}' for i in range(1, 6))
        moving = [j for j in self.chain if j.get('type') != 'fixed']
        if tuple(j.get('name') for j in moving) != self.names:
            raise ValueError('expected audited five-joint local arm')
        self.lower = np.array([float(j.find('limit').get('lower')) for j in moving])
        self.upper = np.array([float(j.find('limit').get('upper')) for j in moving])
        self.effort = np.array([float(j.find('limit').get('effort')) for j in moving])
        self.velocity = np.array([float(j.find('limit').get('velocity')) for j in moving])
        self.axes = np.array([finite(j.find('axis').get('xyz').split(), (3,)) for j in moving])
        lengths = np.linalg.norm(self.axes, axis=1)
        if (np.any(lengths < 1e-12) or not np.all(np.isfinite(np.r_[self.lower, self.upper, self.effort, self.velocity, self.axes.ravel()]))
                or np.any(self.lower >= self.upper) or np.any(self.effort <= 0) or np.any(self.velocity <= 0)):
            raise ValueError('invalid model joint axes/limits')
        self.axes /= lengths[:, None]
        # URDF origins are constant for this immutable model. Parsing their
        # RPY rotations in every FK/Jacobian call adds avoidable executor load.
        self._origins=tuple(origin(j.find('origin')) for j in self.chain)
        self._raw_axes=tuple(finite(j.find('axis').get('xyz').split(),(3,)) for j in moving)

    def frames(self, q):
        q = finite(q, (5,))
        if np.any(q < self.lower) or np.any(q > self.upper):
            raise ValueError('joint state outside URDF limits')
        t, sensor, index = np.eye(4), None, 0
        for joint,fixed_origin in zip(self.chain,self._origins):
            t = t@fixed_origin
            if joint.get('type') != 'fixed':
                if joint.get('type') != 'revolute':
                    raise ValueError('unsupported joint type')
                r = np.eye(4)
                r[:3, :3] = rotation(self._raw_axes[index], q[index])
                t, index = t@r, index+1
            if joint.find('child').get('link') == 'gripper_base_1':
                sensor = t.copy()
        if sensor is None:
            raise ValueError('sensor frame missing from local chain')
        return t[:3, 3].copy(), sensor

    def jacobian(self, q):
        """TCP geometric Jacobian [linear; angular], in local base axes."""
        q = finite(q, (5,))
        self.frames(q)  # validate the same audited chain and limits
        t, axes, points, index = np.eye(4), [], [], 0
        for joint,fixed_origin in zip(self.chain,self._origins):
            t = t@fixed_origin
            if joint.get('type') != 'fixed':
                points.append(t[:3, 3].copy())
                axes.append(t[:3, :3]@self.axes[index])
                r = np.eye(4)
                r[:3, :3] = rotation(self.axes[index], q[index])
                t, index = t@r, index+1
        return np.vstack((np.array([np.cross(a, t[:3, 3]-p) for a, p in zip(axes, points)]).T,
                          np.array(axes).T))


def interpolate(history, stamp, max_gap):
    for (t0, q0), (t1, q1) in zip(history, list(history)[1:]):
        if t0-1e-9 <= stamp <= t1+1e-9 and t1-t0 <= max_gap+1e-9 and t1 > t0:
            return q0+(q1-q0)*(stamp-t0)/(t1-t0)
    raise ValueError('missing joint bracket at sensor timestamp')


def at_grasp(raw, sensor, grasp, sign):
    raw, sensor, grasp = finite(raw, (6,)), finite(sensor, (4, 4)), finite(grasp, (3,))
    if sign not in (-1., 1.):
        raise ValueError('measured sensor sign required')
    force = sign*sensor[:3, :3]@raw[:3]
    moment = sign*sensor[:3, :3]@raw[3:]+np.cross(sensor[:3, 3]-grasp, force)
    return np.r_[force, moment]

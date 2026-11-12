"""Local readiness checks for experiment supervisors; no control commands."""
import numpy as np

class ExperimentGrasps:
    """Experiment supervisor only: both local safety heartbeats must be fresh."""
    def __init__(self):
        self.states={}

    def update(self,robot,state,now):
        self.states[robot]=(state,now)

    def ready(self,now):
        return all(robot in self.states and self.states[robot][0]=='attached'
                   and 0<=now-self.states[robot][1]<=.3 for robot in ('amir1','amir2'))


class StationaryWindow:
    """Require fresh local measurements continuously stationary before biasing."""
    def __init__(self, duration_s=.3):
        self.duration_s = duration_s
        self.since = None

    def ready(self, now, joints, odom):
        good = joints is not None and odom is not None
        if good:
            try:
                velocities = dict(zip(joints.name, joints.velocity))
                qdot = np.array([velocities[f'Joint_{i}'] for i in range(1, 6)])
                twist = odom.twist.twist
                base = np.array([twist.linear.x, twist.linear.y, twist.angular.z])
                ages = [now-(message.header.stamp.sec + message.header.stamp.nanosec*1e-9)
                        for message in (joints, odom)]
                good = (np.all(np.isfinite(qdot)) and np.all(np.isfinite(base))
                        and np.max(np.abs(qdot)) < .005 and np.linalg.norm(base) < .002
                        and all(-.002 <= age <= .05 for age in ages))
            except (KeyError, ValueError):
                good = False
        if not good or (self.since is not None and now < self.since):
            self.since = None
            return False
        if self.since is None:
            self.since = now
        return now-self.since >= self.duration_s

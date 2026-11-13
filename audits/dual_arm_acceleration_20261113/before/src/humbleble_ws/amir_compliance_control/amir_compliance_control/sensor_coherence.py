"""Select a recent coherent timing snapshot without extending watchdog bounds."""
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy,qos_profile_sensor_data


def local_sensor_qos(latest=False):
    # State snapshots are replaceable; do not consume queued old robot states.
    return (QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,
                       durability=DurabilityPolicy.VOLATILE) if latest else qos_profile_sensor_data)


def coherent_stamps(history, keys, sim, wall, age, skew, heartbeat):
    eligible={key:[(s,w) for s,w in history.get(key,())
                   if -skew<=sim-s<=age and 0<=wall-w<=heartbeat] for key in keys}
    if any(not values for values in eligible.values()):return None
    # The oldest latest sample limits the newest possible coherent window.
    end=min(max(s for s,_ in values) for values in eligible.values())+skew
    selected={key:max((s for s,_ in values if s<=end+1e-12),default=float('-inf'))
              for key,values in eligible.items()}
    if max(selected.values())-min(selected.values())>skew+1e-12:return None
    return selected

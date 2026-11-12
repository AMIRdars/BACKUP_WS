"""Bounded-storage GUI observation after verified stop; never rearm or move."""
import json
import time
import math
import subprocess
import re


def pause_world_for_review(run=subprocess.run):
    """Evaluation-only pause after verified stop; inspect actual Gazebo stats."""
    response=run(['ign','service','-s','/world/cooperative_transport/control',
        '--reqtype','ignition.msgs.WorldControl','--reptype','ignition.msgs.Boolean',
        '--timeout','2000','--req','pause: true'],capture_output=True,text=True,check=True,timeout=5.)
    if not re.search(r'\bdata:\s*true\b',response.stdout):
        raise RuntimeError('Gazebo did not acknowledge review pause: '+response.stdout)
    result=run(['ign','topic','-e','-t','/world/cooperative_transport/stats','-n','1',
        '--json-output'],capture_output=True,text=True,check=True,timeout=5.)
    stats=json.loads(result.stdout)
    if stats.get('paused') is not True:raise RuntimeError('Gazebo stats did not confirm paused review')
    return {'pause_response':response.stdout.strip(),'world_stats':stats,
            'scope':'paused final-state review after measured stop; not live grasp endurance'}


def coherent_source_ages(clock_stamp, joint_stamps, max_skew=.1):
    """Independent DDS streams can deliver q slightly ahead of /clock.

    Compare every source with the newest received source, bounding the whole
    spread. This keeps the 100 ms freshness bound without a false negative age.
    """
    values=[clock_stamp,*joint_stamps.values()]
    if not values or any(not math.isfinite(v) or v<=0 for v in values):
        raise ValueError('GUI hold invalid source timestamp')
    newest=max(values)
    if newest-min(values)>max_skew:
        raise ValueError('GUI hold source timestamp spread exceeds freshness bound')
    return {robot:newest-stamp for robot,stamp in joint_stamps.items()}


def pump_feedback(spin_once):
    """Dispatch all local streams before comparing stamps after CSV export."""
    spin_once(.02)
    for _ in range(128):spin_once(0.)


def hold_until_stopped(*, pump, health, cancelled, status_file, metadata, clock=time.monotonic,
                       holding_state='SAFE_STOP_GUI_HOLD'):
    start, last_status = clock(), float('-inf')
    try:
        while not cancelled():
            pump()
            health()  # raises on stale feedback, grip loss or actual movement
            now = clock()
            if now-last_status >= 1.:
                status_file.write_text(json.dumps({**metadata, 'state': holding_state,
                    'elapsed_wall_s': now-start, 'monitor_healthy': True}, indent=2))
                last_status = now
    except Exception as exc:
        status_file.write_text(json.dumps({**metadata, 'state': 'GUI_HOLD_FAILED',
            'elapsed_wall_s':clock()-start,'monitor_healthy': False, 'error': str(exc)}, indent=2))
        raise
    status_file.write_text(json.dumps({**metadata, 'state': 'GUI_HOLD_CLOSED',
        'elapsed_wall_s': clock()-start, 'monitor_healthy': True}, indent=2))

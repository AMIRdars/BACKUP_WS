"""Optional 20-second main-thread CPU profile starting 90 s after node startup.
Baseline runs do not set PROFILE_ROS_DIR and do not import this module.
"""
import os, sys
if os.environ.get('PROFILE_ROS_DIR') and any(name in sys.argv[0] for name in ('friction_grasp_manager','coordinator','rotation_controller','dual_base_approach','slip_monitor','safety_monitor','arm_home_positioner','grasp_support_remover','rover_twist_relay_ign')):
    import atexit, cProfile, time, signal, json
    from pathlib import Path
    destination = Path(os.environ['PROFILE_ROS_DIR']); destination.mkdir(parents=True, exist_ok=True)
    stem = Path(sys.argv[0]).name + '-' + str(os.getpid())
    profiler = cProfile.Profile(timer=time.thread_time)
    state = {'started':False, 'done':False}
    def save():
        if not state['started'] or state['done']:return
        profiler.disable(); state['done']=True; state['end_monotonic']=time.monotonic()
        profiler.dump_stats(str(destination/(stem+'.pstats')))
        (destination/(stem+'.json')).write_text(json.dumps(state))
    def alarm(signum, frame):
        if state['started']: save()
        else:
            state.update(started=True,start_monotonic=time.monotonic()); profiler.enable()
            signal.setitimer(signal.ITIMER_REAL,20)
    signal.signal(signal.SIGALRM,alarm)
    signal.setitimer(signal.ITIMER_REAL,90)
    atexit.register(save)

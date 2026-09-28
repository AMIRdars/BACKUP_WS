#!/usr/bin/python3
"""Measure the requested launch; CPU 100% = one logical core, elapsed = monotonic."""
import os, sys, time, json, signal, subprocess, threading
from pathlib import Path
import psutil
import rclpy
from rosgraph_msgs.msg import Clock
from rclpy.qos import qos_profile_sensor_data
out = Path(sys.argv[1]); out.mkdir(parents=True, exist_ok=True)
duration = float(sys.argv[2]); headless = sys.argv[3]
rclpy.init(); node = rclpy.create_node('performance_observer')
clock = {'sim': None, 'received': None}
def receive(msg):
    clock.update(sim=msg.clock.sec + msg.clock.nanosec * 1e-9, received=time.monotonic())
sub = node.create_subscription(Clock, '/clock', receive, qos_profile_sensor_data)
thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True); thread.start()
log = (out/'launch.log').open('w')
cmd = ['ros2','launch','cooperative_transport_bringup','integrated_transport_simulation.launch.py',f'headless:={headless}']
launch = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
t0 = time.monotonic(); previous = {}; previous_threads = {}; known = {}; records = []; stop = False; suspended_pid = None
stats_log = (out/'world_stats.jsonl').open('w')
stats = subprocess.Popen(['ign','topic','-e','-t','/world/cooperative_transport_friction/stats','--json-output'], stdout=stats_log, stderr=subprocess.DEVNULL, start_new_session=True)
gpu_log = (out/'gpu.csv').open('w')
gpu = subprocess.Popen(['nvidia-smi','--query-gpu=timestamp,utilization.gpu,utilization.memory,memory.used,power.draw','--format=csv','-l','1'], stdout=gpu_log, stderr=subprocess.STDOUT, start_new_session=True)
try:
    with (out/'samples.jsonl').open('w') as output:
        while time.monotonic()-t0 < duration and launch.poll() is None:
            time.sleep(1); now = time.monotonic(); processes = []
            try: children = psutil.Process(launch.pid).children(recursive=True)
            except psutil.Error: children = []
            for p in children:
                try:
                    with p.oneshot():
                        cpu = p.cpu_times(); ticks = cpu.user+cpu.system; args = p.cmdline(); rss = p.memory_info().rss
                        old = previous.get(p.pid); usage = 100*(ticks-old[1])/(now-old[0]) if old else None
                        previous[p.pid] = (now,ticks)
                        threads = []
                        for th in p.threads():
                            total = th.user_time+th.system_time; key=(p.pid,th.id); old_th=previous_threads.get(key)
                            th_cpu=100*(total-old_th[1])/(now-old_th[0]) if old_th else None
                            previous_threads[key]=(now,total)
                            try: name=Path(f'/proc/{p.pid}/task/{th.id}/comm').read_text().strip()
                            except OSError: name=''
                            if th_cpu and th_cpu > 0.5: threads.append({'tid':th.id,'name':name,'cpu':th_cpu})
                        item = {'pid':p.pid,'name':p.name(),'args':args,'cpu':usage,'rss':rss,'threads':threads}
                        processes.append(item); known[p.pid]=args
                except (psutil.Error,OSError): pass
            pause_start=float(os.environ.get('GUI_SUSPEND_START','inf')); pause_end=float(os.environ.get('GUI_SUSPEND_END','inf'))
            elapsed=now-t0
            if pause_start <= elapsed < pause_end and suspended_pid is None:
                for item in processes:
                    if item['args'] and item['args'][0].strip() == 'ign gazebo gui':
                        os.kill(item['pid'],signal.SIGSTOP);suspended_pid=item['pid'];print('GUI suspended at',elapsed,flush=True)
            elif elapsed >= pause_end and suspended_pid:
                os.kill(suspended_pid,signal.SIGCONT);suspended_pid=None;print('GUI resumed at',elapsed,flush=True)
            record={'gui_suspended':suspended_pid is not None,'wall':now-t0,'sim':clock['sim'],'clock_age':None if clock['received'] is None else now-clock['received'],'system_cpu':psutil.cpu_percent(),'memory_available':psutil.virtual_memory().available,'processes':processes}
            output.write(json.dumps(record)+'\n'); output.flush()
            if int(now-t0)%15==0: print(f"{out.name}: wall={now-t0:.1f}s sim={clock['sim']} cpu_sum={sum(p['cpu'] or 0 for p in processes):.1f}%",flush=True)
finally:
    if suspended_pid:
        try: os.kill(suspended_pid,signal.SIGCONT)
        except ProcessLookupError: pass
    children = []
    try: children = psutil.Process(launch.pid).children(recursive=True)
    except psutil.Error: pass
    for process in (launch,stats,gpu):
        try: os.killpg(process.pid,signal.SIGINT)
        except ProcessLookupError: pass
    try: launch.wait(timeout=12)
    except subprocess.TimeoutExpired:
        for p in children:
            try: p.terminate()
            except psutil.Error: pass
        try: os.killpg(launch.pid,signal.SIGTERM)
        except ProcessLookupError: pass
        try: launch.wait(timeout=5)
        except subprocess.TimeoutExpired: launch.kill()
    for p in children:
        try:
            if p.is_running(): p.kill()
        except psutil.Error: pass
    stats.wait(timeout=5); gpu.wait(timeout=5)
    log.close(); stats_log.close(); gpu_log.close(); rclpy.try_shutdown()
    (out/'metadata.json').write_text(json.dumps({'command':cmd,'start_monotonic':t0,'duration':time.monotonic()-t0,'exitcode':launch.returncode,'processes':known},indent=2))

#!/usr/bin/python3
"""Measure the requested launch; CPU 100% = one logical core, elapsed = monotonic."""
import os, sys, time, json, signal, subprocess, threading
from pathlib import Path
import psutil
import rclpy
from rosgraph_msgs.msg import Clock
from rclpy.qos import qos_profile_sensor_data
out = Path(sys.argv[1]); out.mkdir(parents=True, exist_ok=True)
import xml.etree.ElementTree as ET
world_path=Path('src/humbleble_ws/cooperative_transport_gazebo/worlds/cooperative_transport_contact.sdf')
if world_path.exists():
    world_xml=world_path.read_text();(out/'world.sdf').write_text(world_xml)
    (out/'condition.json').write_text(json.dumps({'max_step_size':float(ET.fromstring(world_xml).findtext('world/physics/max_step_size')),'arguments':sys.argv[4:]},indent=2))
duration = float(sys.argv[2]); headless = sys.argv[3]
rclpy.init(args=[]); node = rclpy.create_node('performance_observer')
clock = {'sim': None, 'received': None}
def receive(msg):
    clock.update(sim=msg.clock.sec + msg.clock.nanosec * 1e-9, received=time.monotonic())
sub = node.create_subscription(Clock, '/clock', receive, qos_profile_sensor_data)
from geometry_msgs.msg import PoseStamped, WrenchStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String, Float64MultiArray
from rclpy.qos import QoSProfile, DurabilityPolicy
import math, copy
quality = {'grasp':{}, 'ever_grasp':False, 'support_removed':False, 'rotation_complete':False,
           'max_slip_horizontal':0.0, 'max_slip_vertical':0.0, 'max_slip_yaw':0.0,
           'min_payload_z_after_support':None, 'poses':{}, 'force':{}, 'force_range':{}, 'statuses':{}, 'faults':[]}
subscriptions=[]
def subscribe(topic, cls, callback, durable=False):
    qos=QoSProfile(depth=10)
    if durable:qos.durability=DurabilityPolicy.TRANSIENT_LOCAL
    subscriptions.append(node.create_subscription(cls,topic,callback,qos))
def grasp(name,msg):
    quality['grasp'][name]=msg.data
    if all(quality['grasp'].get(n,False) for n in ['amir1','amir2']):quality['ever_grasp']=True
    elif quality['ever_grasp'] and not msg.data and 'grasp_lost' not in quality['faults']:quality['faults'].append('grasp_lost')
def pose(msg):
    p=msg.pose.position;q=msg.pose.orientation
    quality['poses']['payload']=[p.x,p.y,p.z,q.x,q.y,q.z,q.w]
    if 'target' in quality['poses']:
        desired=quality['poses']['target'];actual=quality['poses']['payload']
        quality['position_error']=math.sqrt(sum((actual[i]-desired[i])**2 for i in range(3)))
        dot=abs(sum(actual[i]*desired[i] for i in range(3,7)))
        quality['orientation_error']=2*math.acos(min(1.0,max(0.0,dot)))
    if quality['support_removed']:
        old=quality['min_payload_z_after_support'];quality['min_payload_z_after_support']=p.z if old is None else min(old,p.z)
def odom(name,msg):
    p=msg.pose.pose.position;q=msg.pose.pose.orientation
    quality['poses'][name]=[p.x,p.y,p.z,q.x,q.y,q.z,q.w]
def slip(msg):
    if len(msg.data)<4:return
    for k,v in zip(['max_slip_horizontal','max_slip_vertical','max_slip_yaw'],[math.hypot(msg.data[0],msg.data[1]),abs(msg.data[2]),abs(msg.data[3])]):quality[k]=max(quality[k],v)
def force(key,msg):
    v=abs(msg.wrench.force.x);quality['force'][key]=v
    if quality['ever_grasp']:
        lo,hi=quality['force_range'].get(key,[v,v]);quality['force_range'][key]=[min(lo,v),max(hi,v)]
def status(key,msg):quality['statuses'][key]=msg.data
for name in ['amir1','amir2']:
    subscribe(f'/cooperative_transport/{name}/grasp_state',Bool,lambda m,n=name:grasp(n,m))
    subscribe(f'/{name}/odom',Odometry,lambda m,n=name:odom(n,m))
    for side in ['left','right']:subscribe(f'/{name}/finger_{side}_wrench',WrenchStamped,lambda m,k=f'{name}/{side}':force(k,m))
subscribe('/cooperative_transport/support_removed',Bool,lambda m:quality.update(support_removed=m.data),True)
subscribe('/cooperative_rotation/complete',Bool,lambda m:quality.update(rotation_complete=m.data),True)
subscribe('/cooperative_transport/measured_payload_pose',PoseStamped,pose)
def target(msg):
    p=msg.pose.position;q=msg.pose.orientation;quality['poses']['target']=[p.x,p.y,p.z,q.x,q.y,q.z,q.w]
subscribe('/cooperative_rotation/object_target',PoseStamped,target)
subscribe('/cooperative_transport/slip_error',Float64MultiArray,slip)
for topic in ['contact_status','friction_grasp_status','slip_status','state']:
    subscribe('/cooperative_transport/'+topic,String,lambda m,k=topic:status(k,m))
subscribe('/cooperative_rotation/status',String,lambda m:status('rotation',m))
def spin_observer():
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException,rclpy._rclpy_pybind11.RCLError):pass
thread = threading.Thread(target=spin_observer, daemon=True); thread.start()
log = (out/'launch.log').open('w')
cmd = ['ros2','launch','cooperative_transport_bringup','integrated_transport_simulation.launch.py',f'headless:={headless}'] + sys.argv[4:]
launch_env=os.environ.copy();launch_env['NATIVE_PROFILE_DIR']=str(out.resolve());launch = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True,env=launch_env)
t0 = time.monotonic(); previous = {}; previous_threads = {}; known = {}; records = []; stop = False; suspended_pid = None
stats_log = (out/'world_stats.jsonl').open('w')
stats = subprocess.Popen(['ign','topic','-e','-t','/world/cooperative_transport_friction/stats','--json-output'], stdout=stats_log, stderr=subprocess.DEVNULL, start_new_session=True)
gpu_log = (out/'gpu.csv').open('w')
gpu = subprocess.Popen(['nvidia-smi','--query-gpu=timestamp,utilization.gpu,utilization.memory,memory.used,power.draw','--format=csv','-l','1'], stdout=gpu_log, stderr=subprocess.STDOUT, start_new_session=True)
native_added=False
try:
    with (out/'samples.jsonl').open('w') as output:
        while time.monotonic()-t0 < duration and launch.poll() is None:
            time.sleep(1); now = time.monotonic(); processes = []
            if not native_added and now-t0 > 40:
                plugin=Path('simulation_performance/20260928/profiling_support/build/libphase_meter.so').resolve()
                if plugin.exists():
                    request='entity {name: "cooperative_transport_friction" type: WORLD} plugins {name: "performance::PhaseMeter" filename: "'+str(plugin)+'"}'
                    added=subprocess.run(['ign','service','-s','/world/cooperative_transport_friction/entity/system/add','--reqtype','ignition.msgs.EntityPlugin_V','--reptype','ignition.msgs.Boolean','--timeout','1000','--req',request],capture_output=True,text=True,timeout=3)
                    (out/'plugin_add.log').write_text(added.stdout+added.stderr); native_added='true' in added.stdout
                else:native_added=True
            if int(now-t0) in [40,80] and not (out/'ros_nodes.txt').exists():
                for noun in ['node','topic']:
                    listed=subprocess.run(['ros2',noun,'list'],capture_output=True,text=True,timeout=10)
                    (out/('ros_'+noun+'s.txt')).write_text(listed.stdout)

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
            record={'quality':copy.deepcopy(quality),'gui_suspended':suspended_pid is not None,'wall':now-t0,'sim':clock['sim'],'clock_age':None if clock['received'] is None else now-clock['received'],'system_cpu':psutil.cpu_percent(),'memory_available':psutil.virtual_memory().available,'processes':processes}
            output.write(json.dumps(record)+'\n'); output.flush()
            if int(now-t0)%15==0: print(f"{out.name}: wall={now-t0:.1f}s sim={clock['sim']} cpu_sum={sum(p['cpu'] or 0 for p in processes):.1f}%",flush=True)
except KeyboardInterrupt:
    pass
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
    log.close(); stats_log.close(); gpu_log.close(); (out/'quality.json').write_text(json.dumps(quality,indent=2)); rclpy.try_shutdown()
    (out/'metadata.json').write_text(json.dumps({'command':cmd,'start_monotonic':t0,'duration':time.monotonic()-t0,'exitcode':launch.returncode,'processes':known},indent=2))

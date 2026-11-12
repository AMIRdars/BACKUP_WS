"""Public approach -> stop both bases -> native grasp -> local arm following."""
from datetime import datetime
import json
import math
from pathlib import Path
from uuid import uuid4
from launch.actions import ExecuteProcess, LogInfo, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from amir_native_arm_support import SCENE
from dual_arm_workspace import common_reach,trajectory_rate_proof
from amir_compliance_control.wrench import LocalModel
from amir_compliance_control.stroke_kinematics import solve_stroke_pose
import xml.etree.ElementTree as ET
import numpy as np


def arm_plan(root,distance):
    proof={}
    for robot,sign in (('leader',1.),('follower',-1.)):
        path=root/SCENE/(robot+'.urdf');tree=ET.parse(path);model=LocalModel(path.read_text())
        q=np.array([float(tree.find(f"ros2_control/joint[@name='{n}']/state_interface[@name='position']/param").text) for n in model.names])
        home,sensor=model.frames(q);samples=[]
        for value in np.linspace(0,distance,61):
            q=solve_stroke_pose(model,q,home+[sign*value,0.,0.],sensor[:3,:3])
            samples.append({'distance_m':float(value),'q_rad':q.tolist(),
                'joint_margin_rad':float(np.min(np.minimum(q-model.lower,model.upper-q))),
                'sigma_xy_m':float(np.linalg.svd(model.jacobian(q)[:2],compute_uv=False)[-1])})
        proof[robot]={'local_direction_x':sign,'home_TCP_m':home.tolist(),'samples':samples}
    return proof


def setup(context):
    config=context.launch_configurations
    root=Path(config['workspace_directory']).expanduser().resolve()
    distance=float(config['dual_arm_distance_m']);speed=float(config['dual_arm_speed_m_s'])
    reach=common_reach(root)
    if not math.isfinite(distance) or not 0<distance<=reach['maximum_common_verified_m']:
        raise ValueError('dual-arm distance exceeds common verified horizontal URDF reach: '+str(reach['maximum_common_verified_m']))
    if not math.isfinite(speed) or not 0<speed<=.005:
        raise ValueError('initial dual-arm experiment requires 0 < arm speed <=0.005 m/s')
    requested_speed=speed
    impedance=float(config.get('dual_arm_admittance_scale','1.0'))
    if not math.isfinite(impedance) or not .25<=impedance<=1.:
        raise ValueError('dual_arm_admittance_scale must be in [0.25,1] for bounded simulation comparison')
    rate=trajectory_rate_proof(root,distance,speed)
    # Slow the Cartesian profile instead of increasing the 0.03 rad/s bound.
    if rate['peak_joint_speed_rad_s']>.03*.95:
        speed*=.03*.95/rate['peak_joint_speed_rad_s']
        rate=trajectory_rate_proof(root,distance,speed)
    if config.get('enable_base_assist','false')!='false':raise ValueError('dual arm requires enable_base_assist=false')
    domain=int(config['ros_domain_id'])
    if not 0<=domain<=232:raise ValueError('invalid ROS domain')
    delay=float(config['start_delay_s'])
    if not math.isfinite(delay) or not 0<=delay<=60:raise ValueError('start delay must be 0..60 s')
    proof=arm_plan(root,distance)
    tag=datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid4().hex[:8]
    out=Path(config['output_directory']).expanduser() if config['output_directory'] else root/('audits/dual_arm_only_'+tag)
    if not out.is_absolute():out=root/out
    if out.exists():raise ValueError('output already exists: '+str(out))
    out.mkdir(parents=True);(out/'arm_plan.json').write_text(json.dumps(proof,indent=2))
    (out/'workspace_reach.json').write_text(json.dumps(reach,indent=2))
    (out/'trajectory_rate_proof.json').write_text(json.dumps(rate,indent=2))
    (out/'launch_policy.json').write_text(json.dumps({'mode':'DUAL_ARM_ONLY','leader_arm_distance_m':distance,
        'leader_arm_speed_m_s':speed,'requested_arm_speed_m_s':requested_speed,
        'base_fixture':False,'physical_base_approach':True,
        'both_base_commands_after_approach':[0.,0.,0.],'follower_force_source':'own Gazebo wrist F/T',
        'follower_stiffness_N_m':0.,'force_deadband_N':.003,
        'virtual_mass_kg':2*impedance,'virtual_damping_Ns_m':34.6*impedance,
        'admittance_scale':impedance,
        'deadband_basis':'previous native stationary measured absolute XY max 0.00287245N; experimental only',
        'legacy_forward_distance_m':config['forward_distance_m'],
        'legacy_translation_velocity_m_s':config['translation_velocity_m_s'],
        'legacy_route_used':False,'gui_review_mode':config['gui_review_mode']},indent=2))
    env={'ROS_DOMAIN_ID':str(domain),'ROS_LOCALHOST_ONLY':'1','IGN_PARTITION':'amir_cp9_dual_arm_'+tag,
         'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1','PYTHONNOUSERSITE':'1'}
    if config['headless']=='false' and config['gui_nvidia_offload']=='true':
        env.update(__NV_PRIME_RENDER_OFFLOAD='1',__GLX_VENDOR_LIBRARY_NAME='nvidia')
    cmd=['/usr/bin/python3','-u',str(root/'tools/amir_af_trial.py'),'--stage','AF-2',
         '--native-arm-stroke','--dual-arm-only','--profile','pose_hold_micro',
         '--virtual-mass-kg',str(2*impedance),'--virtual-damping-ns-m',str(34.6*impedance),'--virtual-stiffness-n-m','0',
         '--arm-horizon','.08','--control-on-wrench','--wrench-on-measurement','--wrench-cutoff-hz','8',
         '--distance',str(distance),'--speed',str(speed),'--start-delay-s',str(delay),
         '--output',str(out/'trial')]
    if config.get('native_cpu_policy','smt')=='smt':cmd+=['--prefer-smt-cores']
    if config['headless']=='false':
        cmd+=['--gui']
        if config['keep_gui_open']=='true':
            cmd+=['--keep-gui-open']
            if config['gui_review_mode']=='paused':cmd+=['--pause-gui-after-stop']
    process=ExecuteProcess(name='dual_arm_only',cwd=str(root),cmd=cmd,additional_env=env,
                           output='screen',sigterm_timeout='120',sigkill_timeout='20')
    def completed(event,ctx):
        path=out/'trial/result.json'
        result=json.loads(path.read_text()) if path.is_file() else {'passed':False,'error':'missing trial result'}
        (out/'result.json').write_text(json.dumps(result,indent=2))
        if ctx.is_shutdown:return []
        if event.returncode or not result.get('passed'):raise RuntimeError('dual arm experiment failed; '+str(path))
        return [EmitEvent(event=Shutdown(reason='dual arm experiment stopped; grip retained'))]
    return [LogInfo(msg='Physical base approach -> BOTH BASES ZERO -> native grasp -> leader ARM translation.'),
        LogInfo(msg=f'Legacy 1m/0.1m/s arguments do not move bases after grasp; arm {distance}m at <= {speed}m/s.'),
        LogInfo(msg='Dual arm output: '+str(out)),RegisterEventHandler(OnProcessExit(target_action=process,on_exit=completed)),process]

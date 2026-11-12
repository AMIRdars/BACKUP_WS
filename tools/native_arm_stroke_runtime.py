"""Public native grasp -> follower base lock -> leader compensated base stroke."""
from datetime import datetime
from pathlib import Path
from uuid import uuid4
import json
import math
from launch.actions import ExecuteProcess,LogInfo,RegisterEventHandler,EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from amir_native_arm_support import SCENE,stroke_plan


def setup(context):
    value=lambda key:LaunchConfiguration(key).perform(context)
    if context.launch_configurations.get('enable_base_assist','false')!='false':
        raise ValueError('Phase 1 requires enable_base_assist=false')
    root=Path(value('workspace_directory')).expanduser().resolve()
    distance=float(value('leader_base_distance_m'));speed=float(value('arm_only_speed_m_s'))
    if value('arm_only_direction')!='forward' or value('backward_distance_m'):
        raise ValueError('native compensated stroke is forward only; use arm_only_trial:=micro for backward micro tests')
    if not math.isfinite(speed) or not 0<speed<=.005:raise ValueError('native ARM_ONLY base speed must be <=0.005m/s')
    calibrated=value('calibrate_leader_distance')=='true'
    if calibrated and speed!=.005:raise ValueError('distance calibration measured at 0.005m/s; use calibrate_leader_distance:=false for other speed experiments')
    plan=stroke_plan((root/SCENE/'follower.urdf').read_text(),distance)
    domain=int(value('ros_domain_id'))
    if not 0<=domain<=232:raise ValueError('invalid ROS domain')
    for key in ('rotation_angle_deg','world_y_distance_m','return_distance_m'):
        if key in context.launch_configurations:raise ValueError('native ARM_ONLY stroke does not support '+key)
    delay=float(value('start_delay_s'))
    if not math.isfinite(delay) or not 0<=delay<=60:raise ValueError('start_delay_s must be in 0..60')
    tag=datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid4().hex[:8]
    out=Path(value('output_directory')).expanduser() if value('output_directory') else root/('audits/native_arm_stroke_'+tag)
    out=(out if out.is_absolute() else root/out).resolve()
    if out.exists():raise ValueError('output_directory already exists: '+str(out))
    out.mkdir(parents=True);(out/'stroke_plan.json').write_text(json.dumps(plan,indent=2))
    (out/'launch_policy.json').write_text(json.dumps({'mode':'ARM_ONLY','grasp':'native CP13 preparation',
        'enable_base_assist':False,'wrench_source_mode':'sensor',
        'leader_base_distance_m':distance,'nominal_payload_distance_m':distance*.5,
        'follower_base_after_grasp':'single writer zero; real odom monitored',
        'follower_base_fixture':'Gazebo-only kinematic base velocity clamp; initially free, engaged only after native grasp',
        'base_speed_m_s':speed,'legacy_forward_distance_m':value('forward_distance_m'),
        'gui_review_mode':value('gui_review_mode'),
        'leader_base_distance_calibrated':calibrated,
        'legacy_translation_velocity_m_s':value('translation_velocity_m_s'),
        'legacy_motion_arguments_used_for_transport':False,'scope':'explicit simulation experiment only'},indent=2))
    env={'ROS_DOMAIN_ID':str(domain),'ROS_LOCALHOST_ONLY':'1','IGN_PARTITION':'amir_cp9_arm_stroke_'+tag,
         'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1','PYTHONNOUSERSITE':'1'}
    if value('headless')=='false' and value('gui_nvidia_offload')=='true':
        env.update(__NV_PRIME_RENDER_OFFLOAD='1',__GLX_VENDOR_LIBRARY_NAME='nvidia')
    cmd=['/usr/bin/python3','-u',str(root/'tools/amir_af_trial.py'),'--stage','AF-2','--native-arm-stroke','--follower-base-fixture','--progressive-stroke',
        '--profile','pose_hold_micro','--virtual-mass-kg','2','--virtual-damping-ns-m','34.6','--virtual-stiffness-n-m','1',
        '--arm-horizon','.08','--control-on-wrench','--wrench-on-measurement','--wrench-cutoff-hz','8',
        '--leader-reference-acceleration','.0012','--distance',str(distance),'--speed',str(speed),
        '--start-delay-s',str(delay),'--output',str(out/'trial')]
    if calibrated:cmd+=['--leader-base-distance-calibration',str(root/'src/humbleble_ws/amir_compliance_control/config/native_leader_odometry.json')]
    if value('native_cpu_policy')=='smt':cmd+=['--prefer-smt-cores']
    if value('headless')=='false':
        cmd+=['--gui']
        if value('keep_gui_open')=='true':cmd+=['--keep-gui-open']
        if value('keep_gui_open')=='true' and value('gui_review_mode')=='paused':cmd+=['--pause-gui-after-stop']
    process=ExecuteProcess(name='native_arm_stroke',cwd=str(root),cmd=cmd,additional_env=env,
        output='screen',sigterm_timeout='120',sigkill_timeout='20')
    def completed(event,ctx):
        path=out/'trial/result.json'
        result=json.loads(path.read_text()) if path.is_file() else {'passed':False,'error':'missing trial result'}
        (out/'result.json').write_text(json.dumps(result,indent=2))
        if ctx.is_shutdown:return []
        if event.returncode!=0 or not result.get('passed'):raise RuntimeError('native arm stroke failed; '+str(path))
        return [EmitEvent(event=Shutdown(reason='native arm stroke finished; verified stop and grip hold'))]
    return [LogInfo(msg=f'Native grasp -> ARM_ONLY; leader base {distance}m, expected object/arm travel {distance*.5}m.'),
        LogInfo(msg='Gazebo test base fixture engages AFTER grasp; this is not unfixtured floor-holding performance.'),
        LogInfo(msg=f'After grasp, legacy 1m/0.1m/s route is replaced by this explicit {speed}m/s base stroke; no follower base recentering.'),
        LogInfo(msg='Final GUI review mode: '+value('gui_review_mode')+' (applied only after measured stop and grasp verification).'),
        LogInfo(msg='Native arm stroke output: '+str(out)),
        RegisterEventHandler(OnProcessExit(target_action=process,on_exit=completed)),process]

"""CP-0..4 ARM_ONLY micro trial; preserve the sealed CP-13 separately."""
from datetime import datetime
import json
import math
from pathlib import Path
from uuid import uuid4

from launch.actions import EmitEvent, ExecuteProcess, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration

from amir_compliance_control.arm_reach import backward_reach_bound


SEED = Path('audits/whole_body_compliance_20261019/cp6/generated/follower.urdf')


def write_report(output, plan, result=None):
    reach = plan['reach']
    text = ['# ARM_ONLY 微小直進試験', '',
        f"要求: リーダー {plan['direction']} {plan['distance_m']} m、速度 {plan['speed_m_s']} m/s。",
        'フォロワは自己F/T→xyアドミッタンス→FK追従誤差→2×5 DLS。',
        'ベースは StrictArmOnly と既存 interlock の単一writer経路からゼロ。',
        'WHOLE_BODY、base_recentering、回転制御は起動しない。', '',
        f"初期TCP（base_footprint）: {reach['home_tcp_m']} m。",
        f"z・姿勢自由でも後退距離の必要上限: {reach['backward_distance_upper_bound_m']:.9f} m。",
        'この上限は関節限界・衝突・把持荷重を考慮しない楽観的上限で、実行許容距離ではない。', '',
        '検証済みの固定拘束把持CP6を使用。ベース固定治具なし。',
        '旧CP13の摩擦接触把持とは別評価であり、摩擦把持の合格を意味しない。',
        '現在の実行枠は10 mm以下、0.01 m/s以下。力・トルク・変位の既存停止値を維持。',
        '高さ変更はこの初期試験では使用しない。CP5以降は無効。',
        '0.5N/0.05Nmは過去の微動シミュレーション保護境界であり許容荷重の校正値ではない。', '',
        '計画: preflight.json。実行した場合のCSV・実topic・停止理由: trial/ 内。']
    if result is not None:
        text += ['', '結果:', '```json', json.dumps(result, ensure_ascii=False, indent=2), '```']
    (output/'REPORT.md').write_text('\n'.join(text)+'\n')


def completed(event, context, *, output, plan):
    path = output/'trial/result.json'
    if path.is_file():
        trial = json.loads(path.read_text())
    else:
        trial = {'passed': False, 'error': 'trial exited without result.json'}
    passed = (event.returncode == 0 and trial.get('passed') is True
              and trial.get('safe_stop_verified') is True
              and trial.get('grasp_held_after_stop') is True
              and trial.get('follower_max_abs_base_cmd') == 0.)
    result = {'passed': passed, 'executed': True, 'returncode': event.returncode,
              'trial_result': str(path), 'trial': trial}
    (output/'result.json').write_text(json.dumps(result, indent=2))
    write_report(output, plan, result)
    if context.is_shutdown:
        return []
    if not passed:
        raise RuntimeError('ARM_ONLY micro trial failed; inspect '+str(output/'result.json'))
    return [LogInfo(msg='ARM_ONLY micro trial passed; '+str(output/'REPORT.md')),
            EmitEvent(event=Shutdown(reason='ARM_ONLY stopped, grasp held, logs saved'))]


def setup(context):
    value = lambda key: LaunchConfiguration(key).perform(context)
    root = Path(value('workspace_directory')).expanduser().resolve()
    runner = root/'tools/amir_af_trial.py'
    seed = root/SEED
    response = root/'audits/arm_only_speed_20261109/servo_response.json'
    for required in (runner, seed, response):
        if not required.is_file():
            raise ValueError('ARM_ONLY dependency missing: '+str(required))
    direction = value('arm_only_direction')
    if direction not in ('forward', 'backward'):
        raise ValueError('ARM_ONLY requires forward or backward')
    distance = float(value('arm_only_distance_m'))
    # Compatibility for the explicit backward argument from the preceding trial.
    if value('backward_distance_m'):
        if direction != 'backward':
            raise ValueError('backward_distance_m requires arm_only_direction:=backward')
        distance = float(value('backward_distance_m'))
    speed = float(value('translation_velocity_m_s'))
    delay = float(value('start_delay_s'))
    if not math.isfinite(delay) or not 0 <= delay <= 60:
        raise ValueError('start_delay_s must be finite in 0..60')
    reach = backward_reach_bound(seed.read_text(), distance)
    domain = int(value('ros_domain_id'))
    if not 0 <= domain <= 232:
        raise ValueError('ros_domain_id must be in 0..232')
    for unsupported in ('rotation_angle_deg', 'world_y_distance_m', 'return_distance_m',
                        'translation_acceleration_m_s2', 'angular_velocity_deg_s', 'angular_acceleration_deg_s2'):
        if unsupported in context.launch_configurations:
            raise ValueError('ARM_ONLY straight backward route does not support '+unsupported)
    tag = datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid4().hex[:8]
    output = Path(value('output_directory')).expanduser() if value('output_directory') else Path('audits/arm_only_p0_'+tag)
    output = (output if output.is_absolute() else root/output).resolve()
    if output.exists():
        raise ValueError('output_directory already exists: '+str(output))
    output.mkdir(parents=True)
    plan = {'mode': 'ARM_ONLY', 'direction': direction, 'distance_m': distance, 'speed_m_s': speed,
            'reach': reach, 'urdf_file': str(seed), 'rotation_enabled': False,
            'legacy_forward_distance_m_unused': value('forward_distance_m'),
            'fixture': 'CP6 fixed grasp; original wheel/floor; no base weld',
            'execution_envelope_m': .01, 'execution_speed_cap_m_s': .01,
            'controller_profile_evidence': str(root/'audits/arm_only_speed_continuation_20261109/approved_profile.json'),
            'load_ratings_calibrated': False, 'scope': 'supervised_simulation_micro_test_only',
            'start_delay_wall_s': delay}
    (output/'preflight.json').write_text(json.dumps(plan, indent=2))
    reason = None
    if direction == 'backward' and reach['geometrically_impossible']:
        reason = (f'Requested backward {distance} m is geometrically impossible with a fixed follower base; '
                  f'optimistic upper bound {reach["backward_distance_upper_bound_m"]:.6f} m even with free z/orientation')
    elif distance > .01:
        reason = 'Reach not ruled out, but only the <=0.01 m ARM_ONLY experiment is verified; do not bypass load/hold guards'
    elif not 0 < speed <= .01:
        reason = 'ARM_ONLY experiment requires finite speed 0 < translation_velocity_m_s <= 0.01'
    if reason:
        result = {'passed': False, 'executed': False, 'phase': 'preflight', 'error': reason}
        (output/'result.json').write_text(json.dumps(result, indent=2))
        write_report(output, plan, result)
        raise ValueError(reason+'; report: '+str(output/'REPORT.md'))
    write_report(output, plan)
    # Unique child-only partition: runner cleanup must not match this launch.
    env = {'ROS_DOMAIN_ID': str(domain), 'ROS_LOCALHOST_ONLY': '1',
           'IGN_PARTITION': 'amir_af_p0_'+tag, 'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1'}
    if value('headless') == 'false' and value('gui_nvidia_offload') == 'true':
        env.update(__NV_PRIME_RENDER_OFFLOAD='1', __GLX_VENDOR_LIBRARY_NAME='nvidia')
    process = ExecuteProcess(name='arm_only_micro', cwd=str(root), additional_env=env, output='screen',
        sigterm_timeout='120', sigkill_timeout='20',
        cmd=['/usr/bin/python3', '-u', str(runner), '--stage', 'AF-2', '--direction', direction,
             '--profile', 'pose_hold_micro', '--virtual-mass-kg', '80',
             '--virtual-damping-ns-m', '5', '--virtual-stiffness-n-m', '1',
             '--arm-command-reference', 'servo_trajectory', '--arm-horizon', '.08',
             '--control-on-wrench', '--wrench-on-measurement', '--wrench-cutoff-hz', '8',
             '--servo-response', str(response), '--leader-reference-acceleration', '.0028',
             '--distance', str(distance), '--speed', str(speed), '--require-speed-fraction', '.98',
             '--start-delay-s', value('start_delay_s'), '--output', str(output/'trial')]
             + (['--gui'] if value('headless') == 'false' else [])
             + (['--keep-gui-open'] if value('headless') == 'false' and value('keep_gui_open') == 'true' else []))
    return [LogInfo(msg=f'ARM_ONLY: leader {direction} {distance} m at <= {speed} m/s; follower base zero; CP6 fixed grasp.'),
            LogInfo(msg='forward_distance_m applies only to control_mode:=WHOLE_BODY; ARM_ONLY uses arm_only_distance_m.'),
            LogInfo(msg='ARM_ONLY output: '+str(output)),
            RegisterEventHandler(OnProcessExit(target_action=process,
                on_exit=lambda event, ctx: completed(event, ctx, output=output, plan=plan))), process]

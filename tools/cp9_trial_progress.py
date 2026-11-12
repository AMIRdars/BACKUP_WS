"""Read-only display of external trial progress; never used by controllers."""
import json
import time


PHASE_LABELS = {
    'startup': 'Gazebo・センサ・コントローラ起動待ち',
    'fixture_initialization': '把持前の試験配置',
    'pre_grasp': '把持前の静止確認',
    'closing': 'グリッパ閉動作・把持力確認（まだ搬送しません）',
    'explicit_monitor_handoff': '把持監視の引き渡し',
    'guarded_initialization': '安全監視と局所制御の準備',
    'unarmed_readiness': '停止状態で通信・安全監視の連続安定を確認',
    'local_home': '両アームの初期姿勢確認',
    'remove_native_support': '支持台の撤去',
    'static_hold': '接触把持で10秒間の静止保持を確認',
    'tare_settle': 'F/Tゼロ点補償・追従制御の準備',
    'baseline': '搬送直前の基準計測',
    'motion': '搬送中',
    'settle': '搬送終了・整定確認',
    'fault_injected': '異常注入・両台停止確認',
}


class TrialProgress:
    def __init__(self, output, *, distance_m, interval_s=5., clock=time.monotonic):
        self.output = output
        self.distance_m = distance_m
        self.interval_s = interval_s
        self.clock = clock
        self.last_wall = None
        self.phase = None
        self.motion_origins = {}

    def update(self, *, phase, sim_time, preparation, encoders):
        wall = self.clock()
        changed = phase != self.phase
        if not changed and self.last_wall is not None and wall - self.last_wall < self.interval_s:
            return
        if phase == 'motion' and changed:
            self.motion_origins = dict(encoders)
        self.phase, self.last_wall = phase, wall
        event = {'phase': phase, 'label': PHASE_LABELS.get(phase, phase),
                 'sim_time_s': sim_time, 'wall_time': wall}
        detail = []
        if phase == 'closing':
            for robot, state in preparation.items():
                sensors = state.get('own_sensors', {})
                event[robot] = {'state': state.get('state'),
                    'left_force_N': sensors.get('left_force'),
                    'right_force_N': sensors.get('right_force')}
                forces = [sensors.get(k) for k in ('left_force', 'right_force')]
                if all(isinstance(f, (int, float)) for f in forces):
                    detail.append(f'{robot}: {state.get("state")} 指力={forces[0]:.2f}/{forces[1]:.2f} N')
        if phase in ('motion', 'settle'):
            displacements = {r: abs(x-self.motion_origins[r]) for r, x in encoders.items()
                             if r in self.motion_origins}
            event['encoder_displacement_m'] = displacements
            event['goal_distance_m'] = self.distance_m
            detail.extend(f'{r}: {x:.3f}/{self.distance_m:g} m' for r, x in displacements.items())
        text = f'[CP-9 progress] sim={sim_time:.1f}s {event["label"]}'
        if detail:
            text += ' | ' + ' | '.join(detail)
        print(text, flush=True)
        with (self.output / 'progress.jsonl').open('a') as log:
            log.write(json.dumps(event, ensure_ascii=False) + '\n')


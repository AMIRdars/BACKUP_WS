"""Generate a reviewable Markdown comparison from measured artifacts."""
from pathlib import Path
import json
import math
import re
import statistics
import xml.etree.ElementTree as ET

base=Path(__file__).resolve().parent
root=base.parents[1]
results=json.loads((base/'results.json').read_text())
indexed={(r['condition'],r['phase']):r for r in results}
launch=(root/'src/humbleble_ws/cooperative_transport_bringup/launch/integrated_transport_simulation.launch.py').read_text()
count=int(re.search(r"DeclareLaunchArgument\('finger_collision_boxes', default_value='(\d+)'",launch).group(1))
world=ET.parse(root/'src/humbleble_ws/cooperative_transport_gazebo/worlds/cooperative_transport_contact.sdf')
step=float(world.findtext('world/physics/max_step_size'))
factor=float(world.findtext('world/physics/real_time_factor'))
def number(value,scale=1,precision=3):
    return '—' if value is None else f'{value*scale:.{precision}f}'
def boolstr(value):return '成功' if value else '未成立'
def cpu(row,key):return number(row['cpu'].get(key),precision=1)
lines=[
'# 協調搬送シミュレーション負荷改善：最終比較',
'',
'指定手順書に沿って一項目ずつ変更・ビルド・測定し、品質が成立する構成を選択した。失敗した条件の速い RTF は改善値として採用していない。変更前データと失敗ログも保存した。',
'',
'## 採用設定',
'',
'| 項目 | 最終設定・判断 |',
'|---|---|',
f'| 指 collision | {count} box / 指。visual・joint・inertial は維持。経緯は実装記録を参照 |',
f'| physics | max_step_size={step:.3f} s、real_time_factor={factor:.1f} |',
'| 姿勢入力 | C++ で amir1 / amir2 / cooperative_payload の 3 モデルへ抽出、位置・姿勢・frame・stamp を保持 |',
'| 接触通知 | C++ が各セグメントを OR・期限付きで集約し、Python は四つの指 Bool を購読 |',
'| 把持力 | 四つの F/T 100 Hz と既存の 20 N 目標・制御式を維持 |',
'| 制御周期 | slip は 50 Hz を維持。30 Hz 試行で CPU 低下を確認できず不採用 |',
'| 起動 | robot_description の応答、controller active、安全開放/Q_HOME/静止、生成成功を確認して進行 |',
'| 不要衝突除外 | 危険な pair を除く根拠を確認できず、mask / collision 削除は未適用 |',
'| GUI | headless=false を既定のまま維持。長時間計測には headless=true を使用できる |',
'',
'## 180 秒試験：正常な把持後の性能',
'',
'同じ既定搬送シナリオのうち、支持台除去後から測定終了までを比較する。100% CPU は論理 1 コア相当。GUI CPU・接触状態・測定区間は試行間で変化するため、単発の差をすべて変更の因果効果とは断定しない。',
'',
'| 条件 | GUI | RTF | Update ms/step | server CPU% | GUI CPU% | coordinator CPU% | slip CPU% | grasp manager CPU% | 最大水平滑り mm |',
'|---|---|---:|---:|---:|---:|---:|---:|---:|---:|',
]
for name,label,gui in [
 ('A_baseline','A: 元の 2 ms','ON'),('D_physics_3ms','D: 3 ms','ON'),
 ('E_physics_4ms','E: 4 ms','ON'),('F_pose_filter','F: +姿勢抽出','ON'),
 ('G3_contact_aggregation_retry','G3: +接触集約','ON'),
 ('H2_headless_approach_timeout','H2: 接触集約なし','OFF'),
 ('G4_contact_aggregation_headless','G4: 接触集約あり','OFF'),
 ('I_slip_30hz','I: slip 30 Hz（不採用）','OFF')]:
 row=indexed.get((name,'transport'))
 if not row:continue
 q=row['quality']
 lines.append(f"| {label} | {gui} | {number(row['rtf'])} | {number(row['update_ms'])} | {cpu(row,'ign gazebo server')} | {cpu(row,'ign gazebo gui')} | {cpu(row,'coordinator')} | {cpu(row,'slip_monitor')} | {cpu(row,'friction_grasp_manager')} | {number(q.get('max_slip_horizontal'),1000)} |")
lines += [
 '',
'接触センサと Gazebo→ROS の 36 raw 接続は維持し、C++ 集約ノードに約 6% CPU が追加される。削減は Python の大型 Contact 変換と callback 数であり、DART 接触計算や ROS raw topic 全体を削減したものではない。',
'',
'接触集約の headless 最終平均は H2 0.554→G4 0.531 と約 4% 低下した。途中値を終了までの改善値とは扱わない。manager CPU 54.4→26.0% と把持・接触判定の品質を採用根拠とし、単発差の RTF 改善は断定しない。',
'',
'Update は最終 PreUpdate→最終 Update の主スレッド CPU 時間で、Physics・Sensors・ROS control 等を含む区間。DART 単体時間ではない。4 ms を超える区間が残るため、RTF=1.0 はまだ達成したとはいえない。headless の server は同一 Ruby CLI プロセス内で動くため、GUI 時の子プロセスと識別方法を分けて集計した。',
'',
'![正常把持後の比較](simulation_performance/optimization/optimization_comparison.png)',
'',
'## 既定の全工程と反復検証',
'',
'0.5 m 前進 → 30°中央軸回転 → 0.3 m 右移動 → 30° A 支点回転。完了後さらに 5 sim 秒保持して停止。初期化を含む RTF と支持台除去後の RTF を分ける。180 秒試験と全工程平均は測定区間が異なる。',
'',
'| 条件 | 計測 wall 秒 | 最終 sim 秒 | 全区間 RTF | 搬送 RTF | main工程 / A支点回転 | 最大水平 / 垂直滑り mm | 最低 payload 高さ m | fault |',
'|---|---:|---:|---:|---:|---|---|---:|---|',
]
full_names=['J_event_headless_full','K_full_gui','K2_full_headless_repeat','L_retry_3box_full','M_1box_full']
for name in full_names:
 row=indexed.get((name,'whole_run'))
 if not row:continue
 q=row['quality'];transport=indexed.get((name,'transport'))
 whole_rtf=number(row['rtf']) if q.get('ever_grasp') else '比較対象外（未把持）'
 slip_text=number(q.get('max_slip_horizontal'),1000)+' / '+number(q.get('max_slip_vertical'),1000) if q.get('ever_grasp') else '未把持'
 failure=q.get('statuses',{}).get('friction_grasp_status','').split(';',1)[0]
 faults=', '.join(q.get('faults',[])) or (failure if failure.startswith('ERROR_') else '観測なし')
 lines.append(f"| {name} | {number(row['last_wall'],precision=1)} | {number(row['last_sim'],precision=1)} | {whole_rtf} | {number(transport['rtf'] if transport else None)} | {boolstr(q.get('rotation_complete'))} / {boolstr(q.get('pivot_complete'))} | {slip_text} | {number(q.get('min_payload_z_after_support'),precision=4)} | {faults} |")
lines += ['', '### 位置・姿勢・接触・力の記録', '',
'| 条件 | main 完了時の位置誤差 mm | main 完了時の姿勢誤差 deg | 中央軸の実測角 deg | 接触 false の 1 Hz サンプル数 |',
'|---|---:|---:|---:|---:|']
for name in full_names:
 row=indexed.get((name,'whole_run'))
 if not row:continue
 q=row['quality'];done=q.get('rotation_completion',{})
 samples=[json.loads(l) for l in (base/name/'samples.jsonl').read_text().splitlines()]
 contacts=[s['quality'].get('contact_states',{}) for s in samples if s.get('quality',{}).get('support_removed')]
 known=[s for s in contacts if len(s)==4]
 bad=sum(not all(s.values()) for s in known)
 contact=f'{bad}/{len(known)}' if known else '未計測'
 lines.append(f"| {name} | {number(done.get('position_error'),1000)} | {number(done.get('orientation_error'),180/math.pi)} | {number((done.get('angles') or {}).get('angle_deg'),precision=2)} | {contact} |")
lines += ['',
'位置・姿勢誤差は main 軌道の完了時を示す。A 支点回転は別ノードへ引き継ぐため、古い main 目標との差を支点回転の追従誤差とは扱わない。J の完了値は 1 Hz サンプルから抽出、以後は完了イベントで記録した。支点回転の成立は既存の目標角 ±1° の停止条件と完了ログで確認する。各台車の位置と quaternion は quality.json / samples.jsonl に保存した。',
'',
'接触は四指の Bool と 1 Hz の観測結果。力は abs(Fx) の瞬時入力範囲と、支持台除去後の 1 Hz 値の中央値を分ける。短い接触変化を 1 Hz の表だけで否定しない。',
'',
'| 条件 | 左指 median N (amir1 / amir2) | 右指 median N (amir1 / amir2) | 把持後の瞬時ピーク N |',
'|---|---:|---:|---:|']
for name in ['A_baseline']+full_names:
 row=indexed.get((name,'whole_run'))
 if not row:continue
 q=row['quality']
 rows=[json.loads(l) for l in (base/name/'samples.jsonl').read_text().splitlines()]
 def median(key):
  vals=[r['quality']['force'][key] for r in rows if r.get('quality',{}).get('support_removed') and key in r['quality'].get('force',{})]
  return statistics.median(vals) if vals else None
 ranges=q.get('force_range',{})
 peak=max((v[1] for v in ranges.values()),default=None)
 left=' / '.join(number(median(f'{n}/left'),precision=2) for n in ['amir1','amir2'])
 right=' / '.join(number(median(f'{n}/right'),precision=2) for n in ['amir1','amir2'])
 lines.append(f'| {name} | {left} | {right} | {number(peak,precision=2)} |')
lines += ['',
'## 不採用・保留と検証限界',
'',
'![指 collision の断面比較](simulation_performance/optimization/finger_collision_comparison.png)',
'',
'外接寸法が同じでも統合 box は段差の隙間を埋める。接触内面の変化が片側接触や力の分布へ影響する可能性があるため、性能だけでは採用しない。これは形状比較からの推定であり、失敗原因をこの図だけで確定したものではない。',
'',
'- 初期 3box B/B2 は起動・接触確認に失敗。L 再評価では接触が成立したが、四指 20±3 N の安定が 45 sim 秒内に成立せず不採用。把持前 RTF を採用根拠にしない。修正済み起動・接触経路での再試行は全工程表と実装記録に含める。1box は 3box の品質が成立した場合だけ実行する。',
'- collision filtering は必須衝突を維持できる除外を証明できず未適用。[分類と方式調査](simulation_performance/optimization/collision_classification.md) に根拠を保存した。',
'- RTF=0.3 の J2 は robot_description 応答喪失で sim=0.408 秒停止。RSP の実際の応答を確認する gate を追加した J3 は把持・支持台除去・保持に成功。データの目標 RTF=0.3 はストレス試験限定で、運用 world は 1.0 に復元した。',
'- Fast DDS サービス応答の一時喪失が残る。準備確認で安全な順序を保つが、すべての通信失敗や起動時間の差を解消したとはいえない。[起動条件・原因・上流ソース](simulation_performance/optimization/startup_events.md) を参照。',
'- ROS と生 Gazebo Contacts は contact position のみを持ち、depth / normal / wrench が欠測だった。深さは未知として扱い、0 の貫通を証明したとは記載しない。力は F/T を使用する。',
'- デスクトップがロックされた状態を確認した。GUI 起動・CPU と機能はログで評価したが、可視画面を目視した検証と描画状態の統一はできていない。shadow / visual 等を未評価のまま変更しない。',
'- 180 秒の headless 搬送 RTF は 0.531、全工程搬送は約 0.495。最低目標 0.5 は全工程平均でわずかに下回り、目標 0.8 / 1.0 は未達。',
'- 把持後に LCP/DART の物理負荷が残る。研究品質を犠牲にして RTF=1 を達成したとは報告しない。',
'',
'## 実行とバックアップ',
'',
'```bash',
'cd /home/dars5070/ros2_humble_ws',
'source /opt/ros/humble/setup.bash',
'source install/setup.bash',
'ros2 launch cooperative_transport_bringup integrated_transport_simulation.launch.py headless:=false',
'```',
'',
'長時間の計測は headless:=true。比較用に filtered_poses:=false / aggregate_contacts:=false / event_startup:=false を指定できる。finger_collision_boxes:=1/3/9 は実験用で、採用済みの既定値以外の成功を保証する引数ではない。',
'',
'変更前を e502dd6 と baseline-before-performance-20260928 タグに保存。[GitHub バックアップ](https://github.com/AMIRdars/BACKUP_WS) に変更前と各段階のブランチ、採用構成の main を保存する。build/install/log は .gitignore で除外し、ソース・MD・生測定データを保存する。',
'',
'測定は simulation_performance/optimization/measure.py、集計は summarize.py、CSV は comparison.csv、全値は results.json と各 quality.json。条件ごとの command、world.sdf、CPU/threads、sim/wall、GPU、Update、ROS node/topic、失敗ログを残した。測定停止時の SIGINT 終了と実行中の機能失敗は区別した。',
'',
'検証：変更パッケージのビルド、既存 77 テスト、姿勢の値・stamp/frame 一致テスト、接触 OR/解除/失効/欠測深さテスト、safe-open の新しい位置判定テスト、起動 gate の空 description/inactive/閉指/関節移動/clock 欠落の拒否と準備完了テスト。',
'',
'[実装・判断の時系列記録](負荷改善_実装結果.md) / [変更前の負荷解析](シミュレーション負荷解析結果.md) / [初回定義調査](負荷改善_初回調査.md)',
]
(root/'負荷改善_最終比較.md').write_text('\n'.join(lines)+'\n')
print('Wrote',root/'負荷改善_最終比較.md')

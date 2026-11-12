"""Record measured maximum-range outcome and preserve all intermediate cases."""
from pathlib import Path
import json,hashlib,csv,difflib,subprocess
import numpy as np
root=Path(__file__).resolve().parents[2];out=Path(__file__).resolve().parent;case=out/'max_213_smooth/trial'
held=json.loads((case/'held_result.json').read_text());evidence=json.loads((case/'dual_arm_evidence.json').read_text())
assert held['passed'] and held['safe_stop_verified'] and held['grasp_held_after_stop']
passed_checks=sum(evidence['checks'].values());failed_checks=[k for k,v in evidence['checks'].items() if not v]
assert set(failed_checks)=={f'native_force_band:{robot}/{side}' for robot in ('leader','follower') for side in ('left','right')}
expected=json.loads((case/'source_hashes.json').read_text());checks={p:hashlib.sha256((root/p).read_bytes()).hexdigest()==v for p,v in expected.items()}
assert all(checks.values())
(out/'tested_source_consistency.json').write_text(json.dumps({'passed':True,'count':len(checks),'checks':checks},indent=2))
before=json.loads((out/'before_hashes.json').read_text());paths=list(before)+['tools/dual_arm_workspace.py','tools/test_dual_arm_range.py','tools/test_dual_arm_only.py','tools/af_cpu_affinity.py','src/humbleble_ws/amir_compliance_control/amir_compliance_control/arm_only_leader.py','src/humbleble_ws/amir_compliance_control/amir_compliance_control/stroke_kinematics.py']
old_test=root/'audits/dual_arm_only_20261112/source_snapshot/tools/test_dual_arm_only.py';copy=out/'before/tools/test_dual_arm_only.py'
if not copy.exists():copy.parent.mkdir(parents=True,exist_ok=True);copy.write_bytes(old_test.read_bytes())
manifest={};patch=''
for name in paths:
    p=root/name;b=out/'before'/name;s=out/'source_snapshot'/name;s.parent.mkdir(parents=True,exist_ok=True);s.write_bytes(p.read_bytes())
    manifest[name]={'sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'baseline_captured':b.exists()}
    patch+=''.join(difflib.unified_diff(b.read_text().splitlines(True) if b.exists() else [],p.read_text().splitlines(True),fromfile='a/'+name if b.exists() else '/dev/null',tofile='b/'+name))
(out/'changed_files.json').write_text(json.dumps(manifest,indent=2));(out/'changes.patch').write_text(patch)
tracked=subprocess.check_output(['git','diff','--binary'],cwd=root)
assert tracked==(out/'tracked_before.patch').read_bytes()
(out/'tracked_preservation.json').write_text(json.dumps({'passed':True,'sha256':hashlib.sha256(tracked).hexdigest()},indent=2))
rows=[r for r in csv.DictReader((case/'timeseries.csv').open()) if r.get('admittance_enabled')=='True' and r.get('adm_p_actual_m_0')]
angles={};speeds={}
for robot in ('leader','follower'):
    q=np.array([[float(r[robot+'_q_actual_'+str(i)]) for i in range(5)] for r in rows])
    v=np.array([[float(r[robot+'_qdot_actual_'+str(i)]) for i in range(5)] for r in rows])
    angles[robot]=(q[-1]-q[0]).tolist();speeds[robot]=float(np.max(abs(v)))
metrics=evidence['dual_arm_metrics'];reach=json.loads((out/'max_213_smooth/workspace_reach.json').read_text());rate=json.loads((out/'max_213_smooth/trajectory_rate_proof.json').read_text());policy=json.loads((out/'max_213_smooth/launch_policy.json').read_text())
summary={'passed':evidence['passed'],'motion_completed':held['passed'],'safe_stop_verified':held['safe_stop_verified'],'grasp_held_after_stop':held['grasp_held_after_stop'],'passed_checks':passed_checks,'failed_checks':failed_checks,'requested_arm_distance_m':held['requested_distance_m'],
 'leader_actual_arm_distance_m':held['leader_actual_arm_displacement_m'],
 'payload_displacement_xyz_m':held['payload_model_gt_displacement_xyz_m'],
 'follower_actual_arm_distance_m':abs(metrics['follower']['local_arm_displacement_m'][0]),
 'kinematic_reach':reach,'trajectory_rate_proof':rate,'launch_policy':policy,
 'measured_joint_change_rad':angles,'measured_joint_speed_peak_rad_s':speeds,
 'dual_arm_metrics':metrics,'evidence_checks':len(evidence['checks'])}
(out/'range_result.json').write_text(json.dumps(summary,indent=2))
case_rows=[]
for name in ['range_100','impedance_030','impedance_100','impedance_180']:
    c=out/name/'trial';r=json.loads((c/'result.json').read_text());e=json.loads((c/'dual_arm_evidence.json').read_text())
    case_rows.append(f"| {name} | {r['requested_distance_m']*1000:.0f} | {r.get('leader_actual_arm_displacement_m',0)*1000:.3f} | {r.get('payload_model_gt_displacement_xyz_m',[0])[0]*1000:.3f} | {e['passed']} |")
failed213=out/'max_213_final/trial';failed_result=json.loads((failed213/'result.json').read_text());failed_evidence=json.loads((failed213/'dual_arm_evidence.json').read_text())
case_rows.append(f"| max_213_final | 213 | {failed_result['leader_actual_arm_displacement_m']*1000:.3f} | {failed_result['payload_model_gt_displacement_xyz_m'][0]*1000:.3f} | {failed_evidence['passed']} |")
case_rows.append(f"| max_213_smooth | 213 | {held['leader_actual_arm_displacement_m']*1000:.3f} | {held['payload_model_gt_displacement_xyz_m'][0]*1000:.3f} | {evidence['passed']} |")
files='\n'.join('- `'+name+'`' for name in manifest)
report=f'''# 両台車停止を維持した最大アーム範囲への拡張

2026-11-12。前回の30mm条件から、現在の把持姿勢・水平並進に対する共通可動域上限 **213mm** へ拡張した。既定の起動コマンドで新しい範囲が選ばれる。

## 最終実測

最終ケース `max_213_smooth/trial/` は動作完了・安全停止・停止後把持保持を確認した。独立評価は **{passed_checks}/{len(evidence['checks'])}項目合格、全項目合格は未達**。リーダー手先 {held['leader_actual_arm_displacement_m']*1000:.6f}mm、物体 {held['payload_model_gt_displacement_xyz_m'][0]*1000:.6f}mm、フォロワー手先 {abs(metrics['follower']['local_arm_displacement_m'][0])*1000:.6f}mm移動。前回30mmの約7.1倍の範囲で、両把持を維持して停止した。

両台車へのbase_cmd/interlock/最終controller入力は、接近停止後すべてゼロ。世界固定治具なし。ARM_ONLY中の物理GT最大ドリフトは、L={metrics['leader']['body_drift_during_ARM_ONLY_peak_m']*1000:.6f}mm、F={metrics['follower']['body_drift_during_ARM_ONLY_peak_m']*1000:.6f}mm。BASE_STOPから把持のなじみを含む変化はL={metrics['leader']['body_drift_from_stop_peak_m']*1000:.6f}mm、F={metrics['follower']['body_drift_from_stop_peak_m']*1000:.6f}mm。指令ゼロと物理的完全固定を混同しない。

手先―物体の相対変位最大はL={metrics['leader']['TCP_object_translation_residual_peak_m']*1000:.6f}mm、F={metrics['follower']['TCP_object_translation_residual_peak_m']*1000:.6f}mmで、既存2mm判定以内。F/T増分最大{evidence['force_increment_peak_N']:.6f}N。実測関節速度最大L={speeds['leader']:.6f}rad/s、F={speeds['follower']:.6f}rad/s。

リーダー関節変化 [rad]: {angles['leader']}。
フォロワー関節変化 [rad]: {angles['follower']}。

移動中16,262サンプルのうちsim=120.520秒の1サンプルで両機左右の指反力が8N未満となり、4つのnative_force_band判定が不合格となった。下限・集計方法は変えていない。全接触フラグは継続し、2mm以内の手先―物体相対変位、実測アーム移動、停止後把持保持は合格した。低反力を無視した全項目合格とは扱わない。瞬間的な反力低下の原因と再現性は未解決であり、数値速度修正だけでは解消していない。

終了後は停止・把持確認を済ませて物理を一時停止し、GUIを残している。最終姿勢とアームの大きな変化を確認できる。これは長時間動的把持耐久試験ではない。

## 上限の求め方と変更理由

実URDF・元の初期把持姿勢・TCP姿勢固定の水平+X並進を1mm刻みで連続IK検査した。L伸展213mm、F引き込み264mmまで解が得られ、次の214mmはLの姿勢IKが未達。共通の検査済み上限を213mmとした。任意の3D姿勢・初期配置を含めた全作業空間の絶対最大値ではない。既存0.01radソフト関節余裕、速度0.03rad/s、力・接触・dt・台車ドリフト停止値は維持する。

213mmをCartesian 5mm/sの五次補間で動かすと、L関節速度の計算ピークは約0.03258rad/sとなる。関節速度上限を増やす代わりに、計算ピークを上限の95%へ抑えるよう時間軌道を減速した。最終のCartesian速度上限は **{policy['leader_arm_speed_m_s']*1000:.6f}mm/s**、予定移動時間は{rate['duration_s']:.3f}sim秒。

最初の100mm試験では接触・把持保持と移動を確認したが、L手先―物体間の相対変位が2.2496mmで独立判定に不合格となった。そこでフォロワーの仮想MとDを1/4（M=0.5kg、D=8.65Ns/m、K=0）にし、M/D=2/34.6秒を維持して必要な反力を減らした。30→100→180→213mmで段階的に検証した。実機の校正値ではなく、このシミュレーション条件で比較・実測した外側アドミッタンス設定である。アームcontrollerの位置ゲインや力停止上限は変えていない。

フォロワーの初期50mm変位枠は、従来nativeストロークの260mm枠に戻した。これは今回拡張した作業変位枠であり、URDFの関節限界やF/T停止条件の解除ではない。

独立評価だけに存在した2mmの手先―物体相対変位監視を、実行中の評価専用監督へも接続した。超過時は共有停止サービスで両機を停止する。GTはこの安全評価にのみ使い、フォロワーの目標生成・IKには渡さない。フォロワーは自身のF/T・FK/関節・odomと既存安全信号だけを使う。

## 段階試験

| ケース | 要求[mm] | L手先実測[mm] | 物体実測[mm] | 独立評価合格 |
|---|---:|---:|---:|---|
{chr(10).join(case_rows)}

最初のrange_100不合格を保存し、合格へ書き換えていない。`max_213_final` は速度4.374mm/sで213mm移動、接触継続・2mm以内の把持ずれ・停止後把持を確認したが、指反力が移動中の数サンプルで8N下限未満となり、独立評価は67/71で不合格。下限は変更せず、既定要求速度を3mm/sへ下げて再試験した。`max_213_slow` では終点近くで既存0.15rad/s²の関節加速度監視により停止し、把持保持を確認した。原因は1µm許容の独立IK解の差分から関節速度を求めた際の数値的な跳ねであり、同じ実URDFの無雑音オフライン再現でも旧方式0.225928rad/s²、新方式0.001987rad/s²となった（velocity_numerics.json）。固定姿勢経路の関節速度を重み付きヤコビアンの最小二乗から求め、元の位置IKと0.03rad/s・0.15rad/s²の監視を維持した。フォロワーのDLSや制御入力境界は変更していない。最初の213mmケース `max_213_gui` は終点直前でバッチ処理のwall時間待ち期限に達して未完了となった。制御・把持の安全異常はなく、安全停止と把持保持は確認した。軌道のsim時間+10秒で完了を待つよう修正し、clock・permit・F/T等の既存wall監視は変えていない。RTF=0.35の単体試験と未完了時のsim期限超過試験を追加した。低インピーダンス条件は30mmの試験から順に通過している。再試験 `max_213_verified` ではrelayのheartbeat欠測により安全停止した（把持は保持）。監視期限は維持し、所有GUI子プロセスのみを元の許可CPU集合内の未使用CPU16〜23へ分離し、制御側0〜15の処理と競合させないようにした。要求割当はgui_cpu_affinity.json、実際の全38スレッドの割当はactual_gui_cpu_affinity.jsonに保存。native_cpu_policy=inheritも選択でき、他アプリ・システム設定は変更しない。各ケースのresult.json、dual_arm_evidence.json、timeseries.csvに合否と停止理由を残した。

`final_smooth_tests.log`:32件合格。旧速度5mm/sを固定期待していたテスト1件を新既定3mm/sへ更新した後に全件再実行した（先行の1失敗・30合格もfinal_slow_tests.logへ保存）。CPU分離、実URDF上限、超過要求の起動前拒否、関節速度に基づく減速、M/D維持、両機非ゼロ台車指令拒否、既存nativeストローク回帰を含む。動画収録の初回は起動メタデータ生成前で失敗し、`max_213_final.avi` はXWD一時エラーで途中終了した（収録済みフレームとエラーを保存）。最終試験は所有ウィンドウの取得を短時間再試行する方法で収録した。

## 起動コマンド

```bash
cd ~/ros2_humble_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch cooperative_transport_bringup compliance_round_trip.launch.py \\
  headless:=false gui_nvidia_offload:=true \\
  forward_distance_m:=1.0 \\
  translation_velocity_m_s:=0.1 record_data:=true
```

新しい既定は `dual_arm_distance_m:=0.213 dual_arm_admittance_scale:=0.25`。Cartesian要求上限は `dual_arm_speed_m_s:=0.003` で、関節速度に応じて自動減速する。小試験は `dual_arm_distance_m:=0.03`。従来M=2,D=34.6の比較条件は `dual_arm_admittance_scale:=1.0`。新旧モードの切替は維持している。旧1m/0.1m/s引数は把持後の台車走行へ使用しない。

ビルドが必要な場合は `colcon build --packages-select amir_compliance_control cooperative_transport_bringup` 後にinstall/setup.bashを再sourceする。制御packageのアーム・台車API、Humble、Fortress、URDF、センサ、JTC2.53.1修正版は継承した。

## 実測トピックと指令元

最終graph.jsonで確認した主要トピックは次のとおり。leader/followerの関節名は既存Joint_1〜Joint_5、位置JointTrajectory APIを維持した。

| 対象 | 実測トピック |
|---|---|
| F/T入力 | `/follower/ft_sensor` |
| 局所フィルタF/T | `/follower/wrench_filtered` |
| 関節実測 | `/leader/joint_states`, `/follower/joint_states` |
| 車輪odom | `/leader/mecanum_drive_controller/odometry`, `/follower/mecanum_drive_controller/odometry` |
| 最終台車入力 | `/leader/mecanum_drive_controller/reference_unstamped`, `/follower/mecanum_drive_controller/reference_unstamped` |
| 最終アーム入力 | `/leader/arm_controller/joint_trajectory`, `/follower/arm_controller/joint_trajectory` |

両機それぞれbase_cmdは自身のendpoint、rover_twistはactuator_interlock、最終台車入力はrover_twist_relay_ign、最終アーム入力はactuator_interlockの単一writer。フォロワー制御とF/T前処理の購読境界にleader/world/experiment目標が無いことを確認した。

## ファイルと証拠

{files}

- `changes.patch`、`source_snapshot/`、`changed_files.json`:今回の差分とソース。
- `tested_source_consistency.json`:試験時ソース{len(checks)}ファイルが現在とSHA一致。
- `tracked_preservation.json`:既存のtracked変更は作業前のまま。
- `max_213_smooth/workspace_reach.json`、`arm_plan.json`、`trajectory_rate_proof.json`:上限・関節経路・速度検査。
- `max_213_smooth/trial/timeseries.csv`、`dual_arm_timeseries.csv`:FT、両腕q/速度/FK/目標、両台車cmd/odom/GT、物体、相対変位監視と停止理由。
- `max_213_smooth/trial/dual_arm_measured.png`/`.pdf`、`dual_arm_fsm.png`:単独で開ける時系列・状態遷移グラフ。
- `grip_force_transient.png`/`.pdf`:不合格となった反力1サンプルの全体・拡大グラフ。
- `gui_max_final.png`:所有GUIの最終姿勢の実画面。
- `video_verification.json`:動画886フレームと時刻メタデータの整合を確認。
- `max_213_smooth.avi`/`.json`:所有Gazeboウィンドウの動画、wall時刻と最後に受信した制御サンプルのsim時刻。映像との完全同期時刻ではない。GUI操作を注入せず撮影。
- `max_213_smooth/trial/graph.json`:両機単一writerとフォロワー購読境界。
- `max_213_smooth/trial/held_result.json`:GUI保持時の停止・把持確認結果。GUI終了後にresult.json確定。

## 残課題

213mmは動作完了の検証範囲であり、指反力の全サンプル8〜26N条件は未達。30/100/180mmの合格結果と分ける。瞬間的な反力低下が制御、接触ソルバ、センサ時刻のいずれに由来するかを次の検証課題とする。

この初期配置・水平移動の検証範囲であり、全方向・全荷重・任意姿勢の限界保証ではない。把持反力・センサの実機許容荷重は未校正。繰返し成功率、往復・長時間保持、異なる床摩擦・物体条件は別試験。213mmを超える水平同姿勢の要求は、現在のURDF検査では拒否する。台車再中心化や物体回転で距離を増やしたものではない。
'''
(out/'REPORT.md').write_text(report);print(out/'REPORT.md')

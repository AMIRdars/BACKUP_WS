#!/usr/bin/env python3
"""Generate chapter-24 checkpoint reports from preserved trial evidence."""
import argparse,csv,json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
ROOT=Path(__file__).resolve().parents[1]


def main():
 p=argparse.ArgumentParser();p.add_argument('--directory',type=Path,default=ROOT/'audits/arm_only_20261107');a=p.parse_args();root=a.directory.resolve()
 excluded=json.loads((root/'evidence_exclusions.json').read_text()).get('exclude_from_final_verdict',[]) if (root/'evidence_exclusions.json').exists() else []
 summary=[]
 for path in sorted(root.glob('*/result.json')):
  j=json.loads(path.read_text());f=path.parent/'timeseries.csv';rows=list(csv.DictReader(f.open())) if f.exists() else [];active=[r for r in rows if r.get('admittance_enabled')=='True']
  def peak(keys,selected=active):return max((float(np.linalg.norm([float(r.get(k) or 0) for k in keys])) for r in selected),default=0.)
  origin=active[0] if active else {}
  gt=max((float(np.linalg.norm([float(r.get('follower_gt_position_'+str(i)) or 0)-float(origin.get('follower_gt_position_'+str(i)) or 0) for i in range(2)])) for r in active),default=0.)
  summary.append({'case':path.parent.name,'stage':j['stage'],'passed':j['passed'],'excluded':path.parent.name in excluded,
   'error':j.get('error',''),'leader_completed':j.get('leader_completed',False),'leader_odom_distance_m':j.get('leader_odom_distance_m'),
   'follower_zero_command_samples':j.get('follower_zero_command_samples'),'follower_max_abs_base_cmd':j.get('follower_max_abs_base_cmd'),
   'base_odom_drift_m':peak(['adm_base_drift_xy_yaw_0','adm_base_drift_xy_yaw_1']),'base_gt_drift_m':gt,
   'force_increment_peak_N':peak(['wrench_filtered_'+str(i) for i in range(3)]),'torque_increment_peak_Nm':peak(['wrench_filtered_'+str(i) for i in range(3,6)]),
   'measured_arm_xy_peak_m':peak(['adm_e_arm_m_0','adm_e_arm_m_1']),'height_error_peak_m':peak(['adm_e_arm_m_2']),
   'orientation_error_peak_rad':peak(['adm_orientation_error_rad_'+str(i) for i in range(3)]),
   'e_track_peak_m':peak(['adm_e_track_xy_m_0','adm_e_track_xy_m_1']),
   'leader_base_cmd_peak_m_s':peak(['leader_base_cmd_0'],rows),
   'constraint_error_peak_m':max((float(r.get(k) or 0) for r in rows for k in ('leader_constraint_error_m','follower_constraint_error_m')),default=0.),
   'stop_latency_wall_s':j.get('stop_latency_wall_s'),'safe_stop_verified':j.get('safe_stop_verified'),
   'injection_to_stop_wall_s':j.get('injection_to_stop_wall_s'),'condition_to_stop_wall_s':j.get('condition_to_stop_wall_s'),
   'stop_latency_basis':j.get('stop_latency_basis'),'leader_moving_at_injection':j.get('leader_odom_velocity_at_injection',0)>0,
   'grasp_held_after_stop':j.get('grasp_held_after_stop'),'remaining_owned_pids':j.get('remaining_owned_pids'),
   'rows':len(rows),'csv':str(f.relative_to(ROOT))})
 (root/'comparison.json').write_text(json.dumps(summary,indent=2))
 if summary:
  with (root/'comparison.csv').open('w') as f:
   w=csv.DictWriter(f,fieldnames=summary[0]);w.writeheader();w.writerows(summary)
 by={s['case']:s for s in summary}
 def select(*names):return next((n for n in names if n in by and by[n]['passed'] and not by[n]['excluded']),None)
 carry=select('af2_5mm_proof','af2_5mm_pose_hold_ref','af2_1mm_final')
 faults=[select('af3_motion_'+f+'_proof_retry','af3_motion_'+f+'_proof','af3_pose_hold_'+f+'_clean','af3_'+f)
         for f in ('dropout','force','saturation','constraint')]
 canonical=['af0',select('af1_pose_hold','af1_final'),carry,*faults]
 lines=['# 第24章 ARM_ONLY 実装・チェックポイント報告（2026-11-07 JST）','',
  '旧回転・摩擦搬送はユーザーの優先変更により一時中断。AF-0〜AF-3の報告を保存し、同章内の原因切り分けと追加検証を継続。ARM_FIRST_BASE・回転・WHOLE_BODYをAF実験から起動していない。','',
  '## 監査と同時動作の原因','',
  'ROS 2 Humble、Gazebo Fortress 6.16.0、既存gz_ros2_controlを使用。AF-0では修正前ARM_BASEの同時アーム・台車動作を再現した。各指令経路のpublisherは1つで、原因はARM_BASEの再中心化／WHOLE_BODY配分を選ぶ起動経路。guarded_transportの旧ARM_ONLYもfollower_arm_baseをゲイン0で使っていた。新AF launchは専用follower_arm_onlyを起動する。','',
  '実URDFのbase_footprint→base_linkはyaw=-1.5708rad。同一座標と推測せず、既存F/T、FK、J、台車APIに整合するfollower/base_footprintをBに採用。実J_xyの有限差分最大誤差3.51e-11、home特異値0.476510/0.251557m。寸法・関節名・センサ符号を変更していない。kinematics_audit.json参照。','',
  '## 実コードの変更','',
  '- 静止把持・tare後の実測FKからp_homeを一度ラッチ。Δp_adm、e_arm=p_actual−p_home、e_track=p_ref−p_actualを別々に記録。Δp_dot_adm+K_ee*e_track→2×5 J_xyのDLS→Joint_1〜5の位置軌道（40ms）へ変換。',
  '- 台車要求は起動前・搬送中・停止後とも毎周期ゼロ。safety_gateと最終actuator_interlockにもAF専用ゼロロックを設置。要求writerの継続監査とロックackがなければprepare不可。追加フラグは既存モードでfalse／空文字。NaNや非使用軸の非ゼロも拒否。',
  '- 関節限界・予測次角・特異値・手先変位／高さ／姿勢・増分F/T・odomドリフト・把持状態／拘束誤差を監視。共通停止後は実測関節holdを維持し、把持を解放しない。',
  '- XYのみDLSは実URDFでvx=1mm/sに対しvz≈−1.424mm/sを生む。固定拘束と競合するため、実験オプションarm_nullspace_pose_holdを追加。XYの3次元ヌル空間内でz／姿勢の副作用を最小化し、XY速度を保持する。5自由度で任意6次元姿勢を達成できるとは仮定せず、残差を記録し既存の姿勢停止を残す。',
  '- リーダー直進に参照加速度と追従補正加速度の分離を追加。straight_reference_acceleration_m_s2の既定0は従来式を使う。AFの低加速度軌道で終端補正用の加速度余裕を確保し、位置許容10µmを広げずに完了させる。',
  '- Gazebo子プロセスがlauncher終了後に残る事例を修正。実験domain＋partitionで所有確認した子のみ終了を確認し、残存PIDを結果へ記録。残存と重なった旧2試験はevidence_exclusions.jsonで最終判定から除外し、別domainで再実施。','',
  '## 試験判定','', '| ケース | 判定 | 台車指令最大 | 停止・把持 | CSV |','|---|---|---:|---|---|']
 for name in canonical:
  if not name:continue
  s=by[name];hold='停止保持・固定把持を確認' if s['safe_stop_verified'] and s['grasp_held_after_stop'] else '把持前／修正前監査'
  label='合格' if s['passed'] else '不合格'
  lines.append(f"| {name} | {label} | {s['follower_max_abs_base_cmd']} | {hold} | [{name}/timeseries.csv]({name}/timeseries.csv) |")
 lines+=['','### 走行中の異常注入と実停止の証拠','',
  '新しい走行中AF-3では、注入直前のleader cmdと実odom速度がともに非ゼロであることを保存する。停止後にベースcontrollerがinactiveになるとodom配信も止まり、最後の非ゼロ値が残り得るため、その古い値を実速度として採用しない。新鮮な車輪joint state、controllerのinactive状態、評価専用GTの静止区間を照合する。GTはフォロワ制御に入力しない。','',
  '| ケース | 走行中注入 | 停止応答ms | 実停止・把持 |','|---|---|---:|---|']
 for name in faults:
  if not name:continue
  s=by[name];lat=s['stop_latency_wall_s']
  lines.append(f"| {name} | {s['leader_moving_at_injection']} | {lat*1000:.3f} | {s['safe_stop_verified'] and s['grasp_held_after_stop']} |")
 lines+=['', '欠測・過大力・拘束誤差の応答は注入から双方SAFE_STOP／最終ゼロ出力まで。飽和は上限到達までの力学応答と、境界検出後の停止応答を分離し、result.json／comparison.csvへ双方保存。境界未到達の時間を検出遅延に含めない。']
 if carry:
  s=by[carry];lines+=['',f"搬送実績：リーダー移動{s['leader_odom_distance_m']*1000:.6f}mm、フォロワ実測アームXY最大{s['measured_arm_xy_peak_m']*1000:.6f}mm。台車cmd {s['follower_zero_command_samples']}件は全て0、odomドリフト最大{s['base_odom_drift_m']:.3g}m、評価用GTでの台車移動最大{s['base_gt_drift_m']:.3g}m。外部ベース固定治具なし。増分F/T最大{s['force_increment_peak_N']:.6f}N／{s['torque_increment_peak_Nm']:.6f}Nm。高さ偏差{s['height_error_peak_m']:.3g}m、姿勢偏差{s['orientation_error_peak_rad']:.3g}rad。",
   '',f"速度上限0.005m/sに対し、低加速度軌道の実ピークcmdは{s['leader_base_cmd_peak_m_s']:.9f}m/s。0.005m/s定速の成立を主張しない。"]
 if 'af2_5mm_xy_ablation' in by:
  lines+=['','同じ仮想力学・参照／追従加速度で姿勢補助だけを外したaf2_5mm_xy_ablationは増分F/T境界で停止し、両機停止・固定把持を維持した。pose_feasibility.jsonと比較CSVを合わせ、XY副作用と拘束反力を切り分けた。失敗例は削除していない。']
 lines+=['', '## 条件と適用範囲','',
  '固定拘束把持：既存CP6のDART weld_cfm=0.1、既存2ms world（旧記載4msを2026-11-09訂正。controller更新周期は4ms）。ベースの位置固定治具は追加していない。全試験は既存モデルと校正証拠のbyte一致（controllerファイルの絶対パス移設だけを除外）を検証し、URDF、world、controller API、車輪・床摩擦を恒久変更していない。',
  '', '初期のM=2,D=34.6,K=150候補はアーム単体の符号試験で検証。結合試験の振動を受け、搬送の実験条件はM=20,D=1200,K=50、XY速度上限0.01m/s、関節要求上限0.03rad/s、p_ref各軸±12mm、実測XY停止10mmとした。姿勢補助gain=4/s、角速度の重み0.1m/radはタスク重みでありロボット寸法ではない。',
  '', '5mm合格条件：leader速度上限0.005m/s、参照加速度0.00001m/s²、追従補正加速度0.00005m/s²、終端許容10µm。力0.5N／トルク0.05Nmは既存6軸校正の刺激範囲によるシミュレーション実験境界であり、hardware定格・把持許容荷重の校正済み値ではない。実機や運用搬送を許可する設定ではない。',
  '', 'AF-3の欠測は実sensor bridgeを停止、過大力・飽和はGazebo物理励振、拘束誤差は評価器の同じ監視経路へ3mm誤差を注入して物理weldを維持。特異点・関節限界等はROS制御器の合成入力試験。全ケースが実Gazeboの危険姿勢への移動試験ではない。','',
  '## 実測API・入力境界','', '| 用途 | 実トピック | 型／API |','|---|---|---|',
  '| 台車要求 | /follower/base_cmd | geometry_msgs/msg/TwistStamped |',
  '| 最終台車writer | /follower/rover_twist | geometry_msgs/msg/Twist |',
  '| ベースcontroller入力 | /follower/mecanum_drive_controller/reference_unstamped | geometry_msgs/msg/Twist |',
  '| 自己odom | /follower/mecanum_drive_controller/odometry | nav_msgs/msg/Odometry |',
  '| 関節実測 | /follower/joint_states | sensor_msgs/msg/JointState |',
  '| アーム要求／JTC | /follower/arm_joint_cmd → /follower/arm_controller/joint_trajectory | trajectory_msgs/msg/JointTrajectory、Joint_1〜5、position |',
  '| F/T | /follower/ft_sensor → /follower/wrench_filtered | geometry_msgs/msg/WrenchStamped |',
  '| 制御状態 | /follower/admittance/debug | std_msgs/msg/String JSON |','',
  '各ケースgraph.jsonは実node、topic、publisher、購読、controllerとhardware interfaceを保存。フォロワ制御器／前処理器にリーダー状態・目標・物体GTの購読はない。GTはaf_experiment_recorderだけで記録・拘束評価に使う。timeseries.csvに元のF/T・joint・odom時刻、Δp_adm、p_ref、p_actual、e_arm、e_track、q/qdot、反力、停止理由を保存。','',
  '## 変更ファイル','', '[changes.json](changes.json)、[implementation.patch](implementation.patch)、source_snapshot/に一覧・差分・実装を保存。主な変更は次の通り。','',
  '- amir_compliance_control/strict_arm_only.py（追加）：第24章専用制御器。','- arm_only_dls.py（追加）：XY速度DLSと任意の姿勢ヌル空間補助。','- launch/arm_only_af.launch.py、setup.py：専用起動経路とentry point。','- safety.py、interlock.py、launch/guarded_safety.launch.py：AF専用ゼロロック、writer継続監査、ack。','- transport.py：既定動作を保つ任意の直進参照加速度。','- test/test_strict_arm_only.py、tools/test_af_pose_hold.py：制御・故障・実URDF幾何の試験。','- tools/amir_af_trial.py／report.py／verify.py：隔離実験、CSV／報告／保存検査。','',
  '## 再現コマンド','', '```bash','cd /home/dars5070/ros2_humble_ws','source /opt/ros/humble/setup.bash','source install/setup.bash',
  'ROS_DOMAIN_ID=221 ROS_LOCALHOST_ONLY=1 IGN_PARTITION=amir_af_user_run01 \\\nOPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \\\npython3 tools/amir_af_trial.py --stage AF-2 --profile pose_hold --distance .005 \\\n  --output /tmp/amir_af2_5mm_run01','```','',
  '出力先は未作成の別名を指定する。scene→証拠一致検査→安全ロック→静止把持→tare→leaderだけへの目標→CSV→停止保持→終了確認を自動実施。AF-1は--stage AF-1 --arm-nullspace-pose-hold。走行中AF-3は--stage AF-3 --profile pose_hold --fault-during-motion --fault dropout（force/saturation/constraintも可）。手動分離起動時はguarded_safety.launch.pyのfollower_arm_only:=trueを使用し、arm_only_af.launch.pyへrun_directory、calibration_file、experiment_policyを渡す。','',
  '## 単体・回帰試験','']
 for name in ('regression','unit_release','pose_hold_geometry_tests','pose_hold_control_tests','regression_release','leader_reference_tests','control_final'):
  f=root/(name+'.xml')
  if not f.exists():continue
  suites=ET.parse(f).getroot().findall('testsuite')
  tests=sum(int(s.get('tests',0)) for s in suites);fail=sum(int(s.get('failures',0))+int(s.get('errors',0)) for s in suites)
  lines.append(f'- {name}.log/xml：{tests}件、失敗{fail}件。各実行段階の履歴として保存。')
 lines+=['','[時系列グラフ](arm_only_5mm.png)は合格した5mm試験のCSVから生成。', '', '## 残るTODO','',
  '1. 0.005〜0.01m/sの定速、小距離の反復・逆方向、荷重変動は別条件。今回の低加速度5mm結果から一般化しない。','2. 力・把持反力・姿勢・停止遅延の許容域を校正。固定拘束の結果から摩擦把持・実機の成立を主張しない。','3. AF-3の実特異姿勢・可動域限界への移動試験は未実施。停止ロジックは合成ROS入力で確認。','4. ARM_FIRST_BASE_DESIGN.mdは設計のみ。r_on15mm、r_off8mm、継続0.15sは未校正で、現行10mm停止域とは両立しない。有効化前に再校正しr_off<r_on<r_stopを満たす。','5. 中心固定・摩擦搬送の旧課題は中断したまま。第21・23章をこの直進試験へ混在させない。']
 (root/'REPORT.md').write_text('\n'.join(lines)+'\n')
 print(json.dumps({'cases':len(summary),'carry_case':carry,'report':str(root/'REPORT.md')},ensure_ascii=False))
if __name__=='__main__':main()

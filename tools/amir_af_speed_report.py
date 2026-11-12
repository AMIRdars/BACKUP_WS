#!/usr/bin/env python3
"""Report measured speed qualification separately from configured caps."""
import argparse,csv,json,hashlib,difflib,shlex,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]

def main():
 p=argparse.ArgumentParser();p.add_argument('--directory',type=Path,default=ROOT/'audits/arm_only_speed_20261109');a=p.parse_args();r=a.directory.resolve();summary=[]
 for f in sorted(r.glob('*/result.json')):
  j=json.loads(f.read_text());rows=list(csv.DictReader((f.parent/'timeseries.csv').open()));motion=[v for v in rows if v['phase'].startswith('leader_translation')];active=[v for v in rows if v.get('admittance_enabled')=='True'];pf=f.parent/'experiment_policy.json';policy=json.loads(pf.read_text()) if pf.exists() else {'controller_parameters':{}}
  def peak(prefix,count,selected=active):return max((float(np.linalg.norm([float(v.get(prefix+'_'+str(i)) or 0) for i in range(count)])) for v in selected),default=0.)
  speed=j.get('leader_odom_peak_m_s')
  if speed is None:speed=max((abs(float(v.get('leader_odom_twist_0') or 0)) for v in rows),default=0.)
  progress=max((float(v.get('leader_odom_pose_0') or 0)-float(motion[0].get('leader_odom_pose_0') or 0) for v in motion),default=0.)
  grippers=all(j.get('post_stop_controllers',{}).get(robot,{}).get('gripper_controller')=='active' for robot in ('leader','follower'))
  qualified=bool(j['stage']=='AF-2' and j['passed'] and j.get('required_speed_fraction',0)>=.98 and speed>=.0049)
  summary.append({'case':f.parent.name,'stage':j['stage'],'passed':j['passed'],'speed_qualified':qualified,
   'startup_completed':(f.parent/'graph.json').exists(),
   'both_grippers_active_after_stop':grippers,'reason':j.get('error',''),
   'follower_reason':j.get('final_endpoints',{}).get('follower',{}).get('reason',''),
   'leader_odom_peak_m_s':speed,'leader_cmd_peak_m_s':j.get('leader_command_peak_m_s'),
   'leader_progress_m':progress,'leader_final_distance_m':j.get('leader_odom_distance_m'),
   'force_peak_N':peak('wrench_filtered',3),'arm_displacement_xy_m':peak('adm_e_arm_m',2),
   'arm_tracking_error_peak_m':peak('adm_e_track_xy_m',2),
   'base_drift_m':peak('adm_base_drift_xy_yaw',2),'grasp_held':j.get('grasp_held_after_stop'),
   'actual_stop_verified':j.get('safe_stop_verified'),'remaining_owned_pids':j.get('remaining_owned_pids'),
   'fault':j.get('fault'),'injection_velocity_m_s':j.get('leader_odom_velocity_at_injection'),
   'stop_latency_wall_s':j.get('stop_latency_wall_s'),'stop_latency_basis':j.get('stop_latency_basis'),
   'controller_parameters':policy['controller_parameters']})
 (r/'comparison.json').write_text(json.dumps(summary,indent=2))
 if summary:
  with (r/'comparison.csv').open('w') as f:
   w=csv.DictWriter(f,fieldnames=summary[0]);w.writeheader();w.writerows(summary)
 carries=[s for s in summary if s['stage']=='AF-2' and s['passed']]
 best=max(carries,key=lambda s:(round(s['leader_odom_peak_m_s']*1000,3),s['case'].startswith('retained_'))) if carries else None
 lines=['# ARM_ONLY 速度条件の継続報告（2026-11-09 JST）','',
  '基準は[前段AF-0〜AF-3報告](../arm_only_20261107/REPORT.md)。5mm搬送と走行中停止は合格したが、実ピーク約0.224mm/sであったため、指定5mm/sへの到達を分離して検証している。ARM_FIRST_BASE・回転は無効。','']
 if best:
  lines += [f"今回の完了搬送の最高実速度は{best['leader_odom_peak_m_s']*1000:.3f}mm/s、移動{best['leader_final_distance_m']*1000:.3f}mm（{best['case']}）。手先追従誤差最大{best['arm_tracking_error_peak_m']*1e6:.3f}µm、増分力最大{best['force_peak_N']:.3f}N。両機実停止・固定拘束保持・グリッパactiveは各result.jsonで検証。5mm/s条件合格={best['speed_qualified']}。",'']
 test_file=r/'retained_release_tests.xml'
 if test_file.exists():
  suite=ET.parse(test_file).getroot().find('testsuite')
  lines += [f"採用コードの単体・回帰試験：{suite.get('tests')}件、failure={suite.get('failures')}、error={suite.get('errors')}。[実行ログ](retained_release_tests.log)／[JUnit](retained_release_tests.xml)。保存ログの採用条件検査は[release_verification.json](release_verification.json)で、速度条件と安全条件を別判定する。",'']
 lines += ['AF-2の指定速度0.005〜0.01m/sは未合格。下表のpassedは各ケースに明示した条件での判定であり、3mm/sの成功を5mm/s合格へ読み替えない。','',
   '座標監査は前報を継承。実URDFのbase_footprint→base_linkはyaw=-1.5708radであるため、既存F/T・FK・J・台車APIと整合するbase_footprintをBとして明記し、両者を同一視しない。グリッパ・把持変換・Joint_1〜5・既存controllerとnamespaceは変更していない。','']
 lines += [
  '| ケース | 段階／合否 | 5mm/s合格 | 実odomピークmm/s | leader進行mm | 両機実停止／固定拘束保持 | CSV |','|---|---|---|---:|---:|---|---|']
 for s in summary:
  hold='未把持(AF-1)' if s['stage']=='AF-1' else (str(s['grasp_held']) if s['startup_completed'] else '把持前中止')
  lines.append(f"| {s['case']} | {s['stage']} / {s['passed']} | {s['speed_qualified']} | {s['leader_odom_peak_m_s']*1000:.6f} | {s['leader_progress_m']*1000:.6f} | {s['actual_stop_verified']} / {hold} | [{s['case']}/timeseries.csv]({s['case']}/timeseries.csv) |")
 faults=[s for s in summary if s['stage']=='AF-3']
 if faults:
  lines+=['','| AF-3 | 実速度（注入時）mm/s | 両機停止までms | 合否 |','|---|---:|---:|---|']
  for s in faults:
   v=s['injection_velocity_m_s'];latency=s['stop_latency_wall_s']
   lines.append(f"| {s['case']} / {s['fault']} | {v*1000 if v is not None else '未注入'} | {latency*1000 if latency is not None else '未判定'} | {s['passed']} |")
  lines+=['','飽和の遅延は変位境界を検出してから両機SAFE_STOP・最終ゼロ出力まで。飽和以外は注入からの遅延。把持拘束誤差は評価guardへ3mmを注入し、物理weldは保持する。特異点・関節限界はROS模擬試験であり、Gazeboで実機構を特異姿勢へ動かした評価ではない。']
 lines+=['','各ケースのexperiment_policy.jsonとsource_snapshot/に条件・実装を固定保存。増分F/T停止境界0.5N／0.05Nm、実測偏差・姿勢・関節限界・ベースゼロロックは緩めていない。固定拘束の既存world・URDF・controllerも同一性を検証。','',
  '速度capだけで合格にせず、--require-speed-fraction .98で実cmd・odomピークの両方にcapの98%以上を要求。これは速度到達の実験判定で、許容荷重の校正ではない。','',
  '採用するarm_command_referenceは既存integrated（既定）と、実測servo応答を使うservo_trajectory（二点）のみ。control_on_wrenchは新しいF/T時刻でアームを更新し、既存20msの定周期タイマはベースゼロ指令・欠測監視を継続する。同じF/T時刻の二重積分は行わない。','',
  '実機構の監査：runtime position_proportional_gain=0.1、controller_manager update_rate=250Hz、position-only JTC / open_loop_control=true。既存gz_ros2_control 0.7.20は位置差を速度へ変換する。[同版の公式実装](https://github.com/ros-controls/gz_ros2_control/blob/0.7.20/gz_ros2_control/src/gz_system.cpp)。把持前servo_probe_01の実測係数はJoint_1〜4で24.7〜25.0/s。servo_response.jsonはURDF・plugin SHAとruntime値を照合し、この4関節だけに40msの予測補償を許可。Joint_5は未励振で補償0。荷重許容値の校正ではない。','',
  '固定拘束の保持とグリッパの準備完了は別に監査する。filter20_01（follower）とpredictive40_low_01（leader）は試験終了時にgripper_controllerが未ロードであり、グリッパ保持の合格証拠として除外。後続ではrequire_gripper_active=trueをAF launchから両機へ設定し、起動完了前のarmを拒否、運転中のinactiveも停止要因とする。旧launchは既定falseで維持。result.jsonのgripper_hold_verifiedで別記録。','',
  'servo_trajectoryは現在点＋将来点のposition-only軌道を同じJTCへ渡す。実測済み関節だけ現在点にtau*qdotを加え、単一点の再補間開始による遅れを避ける。両点の関節余裕、更新速度、実測qからの距離を制限する。[JTC 2.53.1の初回補間処理](https://github.com/ros-controls/ros2_controllers/blob/2.53.1/joint_trajectory_controller/src/joint_trajectory_controller.cpp)。把持前AF-1のXY追従誤差95%値は、単一点予測72.6µmから二点6.94µmへ低減。','',
  '不採用の探索：measured_velocity、単一点integrated_predictive、10ms更新、五次直進参照、加速度予測付き三点軌道は5mm/s完了を改善できず、現行コードから除いた。対応する過去command.jsonは各ケースのsource_snapshotと組み合わせた履歴であり、現行コードの再実行オプションではない。追加探索の全コードはexperimental_snapshot/manifest.jsonと同配下に保存。現行の再実行はRUN_COMMANDS.mdを使う。既存safety.py・transport.py・test_cp7_safety.pyは継続開始時とbyte一致へ戻し、不要な実験分岐を残していない。','',
  '実world max_step_sizeは2msであり、前報の4ms表記はcontroller更新周期との混同として訂正した。','',
  'グリッパ起動サービスの応答待ちでservo_knots_5mm_m20_01は走行前終了。後続の試験器はdisarmed中だけ、同じdomain/partitionに属する当該グリッパspawnerの引数を記録して停止し、同じcontroller名・type・設定ファイルを保つ別プロセスで最大3回再試行する。active確認まで走行しない。アクティブなcontrollerのunloadは行わない。','',
  'ARM_FIRST_BASEは[設計資料](../arm_only_20261107/ARM_FIRST_BASE_DESIGN.md)のみ。r_on=15mm、r_off=8mm、継続0.15sと速度ランプは未校正であり、現行10mm停止域と両立しないため有効化しない。','',
  '現時点の成功条件と未達条件はcomparison.jsonのpassedとspeed_qualifiedで区別する。AF-1合格を速度条件の合格に数えない。速度を達成していないケースを搬送成功としない。失敗理由・停止時の車輪／controller／評価GT・把持保持は各result.jsonを参照。','',
  '変更ファイルと各SHAは[changes.json](changes.json)、差分は[implementation.patch](implementation.patch)。実topic／publisher／controller APIは各ケースgraph.json、全実行引数・隔離環境変数は各ケースcommand.jsonに保存。再実行時は--outputを未使用ディレクトリへ変更する。','',
  f"残課題：実速度5mm/sかつ5〜10mmの反復合格と、その速度での移動中AF-3。5mm/s設定での最高到達は約4.276mm/sで、反力による停止。3mm/sは{sum(.0029<s['leader_odom_peak_m_s']<.0031 for s in carries)}回の完了搬送と移動中AF-3を確認。0.5N／0.05Nmは実験境界であり許容荷重の校正値ではないため、合格目的の緩和は行わない。摩擦把持・実機荷重校正・ARM_FIRST_BASE・回転は未実施。"]
 (r/'REPORT.md').write_text('\n'.join(lines)+'\n')
 status={'scope':'simulation_fixed_grasp_ARM_ONLY',
   'requested_speed_condition_passed':any(s['speed_qualified'] for s in summary),
   'completed_lower_speed_carries':[s['case'] for s in carries],
   'af0_evidence':'../arm_only_20261107/af0',
   'af1_evidence':'servo_knots_probe_01',
   'af3_retained_cases':[s['case'] for s in faults if s['case'].startswith('retained_')],
   'hardware_load_limits_verified':False,'friction_grasp_qualified':False,
   'ARM_FIRST_BASE_executed':False,'rotation_executed':False}
 (r/'status.json').write_text(json.dumps(status,indent=2))
 if best:
  argv=json.loads((r/best['case']/'command.json').read_text())['argv']
  index=argv.index('--output');argv=argv[:index]+argv[index+2:]
  commands=['# 再実行コマンド','',f"完了搬送を再現する条件: {best['case']}。実速度{best['leader_odom_peak_m_s']*1000:.3f}mm/s。5mm/s未達の場合はその条件の合格とはみなさない。",'',
   '```bash','cd /home/dars5070/ros2_humble_ws','source /opt/ros/humble/setup.bash','source install/setup.bash',
   'af_tag=$(date +%Y%m%d_%H%M%S)','export ROS_DOMAIN_ID=70 ROS_LOCALHOST_ONLY=1',
   'export IGN_PARTITION="amir_af_rerun_${af_tag}"','export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1',
   'python3 '+shlex.join(argv)+' --output "audits/arm_only_rerun_${af_tag}"','```','',
   '同時に別の試験を起動する場合は未使用domainとpartitionを使う。上記の既存servo_response.jsonは実測URDF／pluginの照合を行う。','',
   '各過去ケースの完全な引数と環境変数はそのcommand.json、実topicとwriterはgraph.json。','',
   '```bash','python3 tools/amir_af_speed_report.py',
   'python3 tools/amir_af_speed_verify.py --case '+best['case']+' --output-name rerun_evidence_check','```']
  (r/'RUN_COMMANDS.md').write_text('\n'.join(commands)+'\n')
 files=[Path(v['path']) for v in json.loads((r/'baseline_manifest.json').read_text())]
 files.extend([Path(__file__).resolve().relative_to(ROOT),Path('src/humbleble_ws/amir_compliance_control/amir_compliance_control/arm_servo_response.py'),
   Path('tools/amir_af_servo_calibrate.py'),Path('tools/test_af_servo_response.py'),Path('tools/amir_af_speed_verify.py'),
   Path('tools/amir_af_speed_plot.py')]);manifest=[];patch=[]
 for f in files:
  source=ROOT/f;old=r/'before'/f;before=old.read_bytes() if old.exists() else b'';after=source.read_bytes()
  dest=r/'source_snapshot'/f;dest.parent.mkdir(parents=True,exist_ok=True);dest.write_bytes(after)
  if before==after:continue
  manifest.append({'path':str(f),'sha256':hashlib.sha256(after).hexdigest(),'before_sha256':hashlib.sha256(before).hexdigest() if before else None})
  patch.extend(difflib.unified_diff(before.decode().splitlines(True),after.decode().splitlines(True),fromfile='a/'+str(f) if before else '/dev/null',tofile='b/'+str(f)))
 (r/'changes.json').write_text(json.dumps(manifest,indent=2));(r/'implementation.patch').write_text(''.join(patch))
 (r/'source_snapshot/manifest.json').write_text(json.dumps({str(f):hashlib.sha256((ROOT/f).read_bytes()).hexdigest() for f in files},indent=2))
 print(json.dumps({'cases':len(summary),'passes':sum(v['passed'] for v in summary),'report':str(r/'REPORT.md')}))
if __name__=='__main__':main()

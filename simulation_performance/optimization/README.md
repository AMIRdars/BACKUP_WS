# 性能・品質測定データ

[最終比較](../../負荷改善_最終比較.md) と [実装記録](../../負荷改善_実装結果.md) を参照。

## 再測定

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
# Update 区間計測用プラグインを事前にビルド
cmake -S simulation_performance/20260928/profiling_support -B simulation_performance/20260928/profiling_support/build
cmake --build simulation_performance/20260928/profiling_support/build --target phase_meter -j2
/usr/bin/python3 simulation_performance/optimization/measure.py \
  simulation_performance/optimization/new_condition 180 false

# 既定シナリオを完了後 5 sim 秒保持して停止（最大 900 wall 秒）
STOP_ON_COMPLETE=1 /usr/bin/python3 simulation_performance/optimization/measure.py \
  simulation_performance/optimization/new_full_run 900 true

/usr/bin/python3 simulation_performance/optimization/summarize.py
PYTHONNOUSERSITE=1 /usr/bin/python3 simulation_performance/optimization/plot_comparison.py
/usr/bin/python3 simulation_performance/optimization/make_final_report.py
```

measure.py の末尾に launch 引数を追加できる。同時に複数のシミュレーションを起動せず、1 条件ずつ実行する。

## 記録と定義

- samples.jsonl: monotonic wall 秒、/clock の sim 秒、/proc のプロセス・スレッド CPU、品質観測。100% CPU = 論理 1 コア。
- condition.json / world.sdf: 起動時の物理条件・引数・変更前 checkpoint commit と world SHA。初期条件 A～D は旧計測スクリプトのため snapshot 欠落部分をブランチ差分・引数から補う。
- metadata.json: 正確な起動コマンド、起動 PID / 引数、終了を含む duration。検証コードは対応する段階の次 commit に保存したものと整合する。計測 wall 秒は samples の末尾を使用し、終了処理時間と区別する。
- update_pass.csv: PhaseMeter の最終 PreUpdate → 最終 Update の **主スレッド CPU 秒**を count で割って ms/step を集計。Physics / Sensors / ROS control 等を含み、DART 単独計測ではない。40 wall 秒以降に追加する。
- quality.json: grasp / support / main と pivot 完了、最大水平・垂直・yaw 滑り、指 F/T abs(Fx) 範囲、接触、位置・quaternion、fault。
- position_error / orientation_error: main 軌道の完了時。pivot は別目標の制御なので古い main 目標との差は追従誤差として使わない。J は完了直後の 1 Hz サンプル、以後は完了イベントで記録。
- max_contact_depth: depth が実際に配信された場合のみ集計。この環境の GZ / ROS は欠測で null。G3 の古い診断値 0 は貫通なしの証明に使用しない。
- whole_run は初期化を含む。transport は support_removed=true 以降。sim_65_80 は試行により接触成立の段階が異なるため、品質状態を必ず併記する。
- GUI CPU は可視・非可視、画面ロック、接触状態、温度等に依存する。画面ロックを確認した試行を可視描画の厳密比較とはしない。

把持していない条件の高い RTF や滑りゼロは、正常把持の性能改善とは扱わない。SIGINT による計測終了のログエラーと、実行中の ERROR / timeout を区別する。測定プロセスは自分が起動した launch の子プロセスを終了し、観測スレッドを join する。

## 接続テスト

```bash
/usr/bin/python3 simulation_performance/optimization/test_safe_open.py
ROS_DOMAIN_ID=113 /usr/bin/python3 simulation_performance/optimization/test_pose_filter.py
ROS_DOMAIN_ID=114 /usr/bin/python3 simulation_performance/optimization/test_contact_aggregator.py
ROS_DOMAIN_ID=115 /usr/bin/python3 simulation_performance/optimization/test_startup_gate.py
```

追加テストは独立 domain で模擬メッセージ・サービスを使う。通常シミュレーションへの注入は行わない。

# 固定追従：姿勢ログ

対象: `/home/dars5070/ros2_humble_ws/audits/dual_arm_acceleration_20261113/servo_now_gui/trial`

解析区間: leader_translation, sim 99.124–108.672 s。図の0秒は sim 94.000 s。

関節角はJointState実測値。TCPの位置・クォータニオン・相対回転ベクトルは、保存されたgenerated URDFのbase_footprint→tcp_link全チェーンで再計算した値です。独立した姿勢センサの測定ではありません。

TCPはEuler角の特異姿勢付近を取るため、姿勢変化は相対回転ベクトル[deg]とそのノルムで表示します。左右はそれぞれ自身の台車座標です。

同じJointState時刻の繰り返し行は除去し、各CSVに元の関節時刻と最初の観測時刻を残しました。観測時刻はロガーのsim時刻であり、非同期のclock更新により関節スタンプより数ms遅れる行もそのまま保持しています（時刻差範囲はsummary.json）。台車は元ログの平面yawのみを出力し、未記録のroll/pitchを補っていません。台車・物体の相対回転は解析区間開始を基準にしています。GTは評価専用で、GT受信時刻と元スタンプをCSVに保持しています。

| Robot | Joint | 開始 [deg] | 終了 [deg] | 変化 [deg] |
|---|---|---:|---:|---:|
| leader | Joint_1 | 0.00000 | 0.00006 | 0.00006 |
| leader | Joint_2 | 80.21409 | 25.38985 | -54.82424 |
| leader | Joint_3 | -91.67325 | -4.41212 | 87.26113 |
| leader | Joint_4 | 11.45916 | -20.97772 | -32.43688 |
| leader | Joint_5 | 89.99998 | 89.99998 | -0.00000 |

leader: TCP変位 [213.00013105914638, -1.611952268230963e-05, -6.092788928935278e-05] mm。初回active姿勢からの回転角最大 0.0038030 deg。最小URDF関節余裕 4.41212 deg (Joint_3)。

| follower | Joint_1 | -0.00000 | -0.00251 | -0.00251 |
| follower | Joint_2 | 80.21409 | 121.89125 | 41.67715 |
| follower | Joint_3 | -91.67325 | -125.46148 | -33.78823 |
| follower | Joint_4 | 11.45916 | 3.57023 | -7.88892 |
| follower | Joint_5 | 89.99998 | 89.99998 | -0.00000 |

follower: TCP変位 [-210.8877652363727, -0.010851052573433925, -3.775478596423909e-06] mm。初回active姿勢からの回転角最大 0.0044313 deg。最小URDF関節余裕 13.05003 deg (Joint_2)。

元手順判定: True。リーダー完了: True。error: None。

厳密受入判定: True。未合格項目: []。姿勢図の作成によって元の判定は変更していません。

再生成:
```bash
cd /home/dars5070/ros2_humble_ws
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 /usr/bin/python3 tools/amir_posture_logs.py --case /home/dars5070/ros2_humble_ws/audits/dual_arm_acceleration_20261113/servo_now_gui/trial --output /tmp/amir_posture_new
```

閲覧: `index.html`をブラウザで開き、スライダで実測姿勢を確認できます。PNG/PDFは時系列・静止姿勢図、CSVは全保存区間を収録しています。

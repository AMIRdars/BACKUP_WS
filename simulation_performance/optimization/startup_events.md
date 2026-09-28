# 起動を進める条件

実時間の固定待ちを準備判定として使わず、以下の状態を確認する。deadline は wall time、制御と物理は sim time。ready gate 自体は use_sim_time=false にして /clock 停止時にも期限を検出する。

```mermaid
flowchart TD
  A[/clock が進む] --> B[amir1 を生成]
  B --> C[controller manager 応答 / 全コントローラ active]
  C --> D[amir2 を生成]
  D --> E[両台の controllers / gripper action server ready]
  E --> F[指開放と Q_HOME 指令]
  F --> G[新しい JointState の位置と速度 / manager READY_TO_GRASP]
  G --> H[payload を生成]
  H --> I[生成成功 / 接近開始]
  I --> J[両台の把持状態 true]
  J --> K[支持台除去 true]
  K --> L[前進 / 中央軸回転 / 横移動]
  L --> M[rotation complete true / A 支点回転]
```

| 旧 TimerAction | 現在の条件 |
|---|---|
| robot1: 2 wall 秒 | /clock の進行を確認 |
| robot2: 9 wall 秒 | amir1 の JSB/arm/mecanum/gripper が active、action server ready |
| manager: 15 秒 / Q_HOME: 15.5 秒 | amir2 も同じ準備条件に達した |
| payload: 28 秒 | manager READY_TO_GRASP、両台の指 -1.0±0.02 rad、Q_HOME±0.02 rad、各関節速度≤0.02 rad/s、新しい状態、0.5 wall 秒安定 |
| monitors/coordinator/approach: 29 秒 | create プロセス成功 |
| rotation: 21.5 秒 | 早期起動、既存の controller/grasp/support ready 状態機械が開始を制御 |
| pivot: 30 秒 | 早期起動、既存の rotation_complete / grasp / support 条件が開始を制御。sim time を明示 |
| robot spawn 後 JSB: 2 秒 | ListControllers サービスへ応答するまで待つ |

準備に失敗した gate / entity create の非ゼロ終了では launch を停止し、次工程へ進めない。ListControllers の応答待ちは 5 wall 秒で取り消し・再確認し、各 gate に 180 wall 秒の期限を設ける。controller spawner の一時失敗に対する 2/3 秒の再試行 backoff は残すが、次工程は active 状態で判定する。旧 TimerAction 経路は `event_startup:=false` の比較用として残す。

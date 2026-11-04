# AMIR 2台：1 m前進・1 m後退のバックアップ

2026-11-05 JST時点の `/home/dars5070/ros2_humble_ws` から保存した。現在の作業中のソースも含む。GitHub上の既存workspaceを変更せず、このフォルダに独立したスナップショットとして追加している。

## 保存内容

- `workspace.tar.gz`：現在の `src/`、`tools/`、検証済み往復用 `.simulation_runtimes/cp13_round_trip/`。専用runtimeのsource・build・install・資格証跡を含む。
- `evidence.tar.gz`：通常往復・後退中F/T異常停止の資格証跡、2026-11-05のGUI正常往復の生ログ、実行環境復旧の修正記録、起動前照合に必要な監査ファイル。
- `manifest.json`：収録ファイルのSHA256またはsymlink参照先、アーカイブのサイズ・SHA256、最新の受入結果。
- `environment.txt`：元のOS・ROS/Gazebo等のパッケージ情報。
- `SHA256SUMS`：ダウンロード後の整合性確認用。

Gitの認証情報・`.git/`、通常workspaceの `build/ install/ log/`、Pythonキャッシュ、無関係な大量の過去試験は含めていない。通常workspaceのbuild/installは復元後にビルドする。往復専用runtimeのbuild/installは資格ハッシュの維持のため保存した。

## 確認した動作

接触把持 → 1 m前進 → 停止・整定 → 1 m後退 → 停止。GUIとlaunchの正常終了、両台の実arm/base controller inactiveを確認した。

- 公開コマンド終了コード：0、所要285.14秒。
- payloadの独立GT：前進 +1.001531750 m、後退 −0.999831412 m。
- leader目標完了2件、followerへの目標0件。Followerは自分のセンサ状態を用いる。
- 既存65件＋追加10件、計75試験合格。読取照合14項目合格。
- 安全停止条件を維持。異常停止した開発試行の記録も含む。GUI実行の一般的な安定性を保証するものではなく、間欠的な更新遅延の調査は継続している。
- カメラは自由操作。GUIと搬送処理のCPU群を分ける設定を含む。

速度設定0.1 m/sは上限。加速度0.005 m/s²のため、各1 m区間の計画ピークは約0.071 m/s。実機・1.0 m/sの認定には使用しない。

## 復元先と手順

環境はUbuntu 22.04 / ROS 2 Humble / Gazebo Sim Fortress。Jazzy/Harmonicへの移行は行っていない。

**検証済みの復元先は `/home/dars5070/ros2_humble_ws`。** URDF、build/install、過去の資格証跡にこの絶対パスが含まれる。別ユーザー・別パスに移設する場合は、再ビルドと資格・実走の再確認が必要。

まず、このバックアップフォルダで整合性を確認する。

```bash
sha256sum -c SHA256SUMS
```

元のworkspaceを別途退避し、復元先へ両アーカイブを展開する。以下の `BACKUP_DIR` は、このREADMEがあるフォルダの絶対パスに設定する。

```bash
BACKUP_DIR=/path/to/BACKUP_WS/backups/round_trip_20261105
mkdir -p /home/dars5070/ros2_humble_ws
tar -xzf "$BACKUP_DIR/workspace.tar.gz" -C /home/dars5070/ros2_humble_ws
tar -xzf "$BACKUP_DIR/evidence.tar.gz" -C /home/dars5070/ros2_humble_ws

cd /home/dars5070/ros2_humble_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to cooperative_transport_bringup
source install/setup.bash
```

ROS/Gazeboや依存パッケージは元のPCのインストールを前提とし、OSやシステム全体のイメージは含めていない。元の環境がないPCでは、`environment.txt` と各 `package.xml` に従って依存を揃える。

## 実行コマンド

```bash
cd ~/ros2_humble_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch cooperative_transport_bringup compliance_round_trip.launch.py \
  headless:=false gui_nvidia_offload:=true \
  forward_distance_m:=1.0 \
  translation_velocity_m_s:=0.1 record_data:=true
```

記録先は起動ログの `CP-13 output_directory`。今回合格した実行のデータは次のフォルダに含む。

```text
audits/whole_body_compliance_20261019/cp13_round_trip/launch_20261105_023531_518398/
```

詳しい変更・試験・残課題は、展開後の `audits/round_trip_command_restore_20261105/README.md` を参照する。

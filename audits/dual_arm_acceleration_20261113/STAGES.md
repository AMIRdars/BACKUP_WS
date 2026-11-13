# 速度引上げ：段階記録（2026-11-13）

URDF・Gazebo物理・controller/API・力/接触/位置/台車停止ガードを変更せず、明示したソフト速度枠を比較。搬送・接触・ずれ・停止後保持などが未達なら次へ進めない。単発の反力下限逸脱は厳密判定の未合格として残し、既存の0.3秒継続監視を変更せず試験する。

| Stage | 動作完了 | 停止後把持 | 実停止 | 厳密判定 | 未合格項目 / error |
|---|---|---|---|---|---|
| speed_008 | True | True | True | True | [] / None |
| speed_015 | False | True | True | False | ['result_passed', 'base_cmd_all_zero', 'leader_actual_arm_translation_matches_requested', 'follower_actual_arm_translation_matches_requested', 'leader_goal_reached'] / shared stop: leader: arm command violates model limits |
| speed_015_retry | True | True | True | False | ['native_force_band:leader/left', 'native_force_band:leader/right', 'native_force_band:follower/left', 'native_force_band:follower/right'] / None |
| speed_015_repeat | True | True | True | False | ['native_force_band:leader/left', 'native_force_band:leader/right', 'native_force_band:follower/left', 'native_force_band:follower/right'] / None |
| speed_025 | True | True | True | True | [] / None |
| speed_035 | True | True | True | True | [] / None |
| speed_050 | False | True | True | False | ['result_passed', 'base_cmd_all_zero', 'leader_grasp_translation_residual_below_existing_2mm', 'leader_actual_arm_translation_matches_requested', 'follower_actual_arm_translation_matches_requested', 'leader_goal_reached'] / shared stop: requested stop |
| speed_050_retry | True | True | True | True | [] / None |
| speed_075 | False | True | True | False | ['result_passed', 'base_cmd_all_zero', 'leader_actual_arm_translation_matches_requested', 'follower_actual_arm_translation_matches_requested', 'leader_goal_reached'] / shared stop: leader: arm command violates model limits |

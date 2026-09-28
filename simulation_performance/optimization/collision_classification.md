# 衝突対象の分類（変更前）

全 collision の用途を分類。複数用途を持つ形状は安全維持を優先する。

| link | collision | geometry | 分類 | 理由 |
|---|---|---|---|---|
| base_footprint | base_footprint_fixed_joint_lump__base_link_collision | mesh | B/C | 台車・固定支持部の床/障害物/他ロボット衝突 |
| base_footprint | base_footprint_fixed_joint_lump__back_left_arm_collision_1 | mesh | B/C | 台車・固定支持部の床/障害物/他ロボット衝突 |
| base_footprint | base_footprint_fixed_joint_lump__back_right_arm_collision_2 | mesh | B/C | 台車・固定支持部の床/障害物/他ロボット衝突 |
| base_footprint | base_footprint_fixed_joint_lump__link0_1_collision_3 | box | B/C | 台車・固定支持部の床/障害物/他ロボット衝突 |
| base_footprint | base_footprint_fixed_joint_lump__front_left_arm_collision_4 | mesh | B/C | 台車・固定支持部の床/障害物/他ロボット衝突 |
| base_footprint | base_footprint_fixed_joint_lump__front_lrf_link_collision_5 | mesh | D/E 候補 | LiDAR 無効。ただしセンサ筐体の外部衝突は維持が必要 |
| base_footprint | base_footprint_fixed_joint_lump__front_right_arm_collision_6 | mesh | B/C | 台車・固定支持部の床/障害物/他ロボット衝突 |
| back_left_wheel_1 | back_left_wheel_1_collision | cylinder | B | 床との摩擦・メカナム移動 |
| back_right_wheel_1 | back_right_wheel_1_collision | cylinder | B | 床との摩擦・メカナム移動 |
| link1_1 | link1_1_collision | cylinder | C | アーム・グリッパの荷物/台車/他ロボットとの干渉検出 |
| link2_1 | link2_1_collision | cylinder | C | アーム・グリッパの荷物/台車/他ロボットとの干渉検出 |
| link3_1 | link3_1_collision | cylinder | C | アーム・グリッパの荷物/台車/他ロボットとの干渉検出 |
| link4_1 | link4_1_collision | box | C | アーム・グリッパの荷物/台車/他ロボットとの干渉検出 |
| gripper_base_1 | gripper_base_1_collision | mesh | C | アーム・グリッパの荷物/台車/他ロボットとの干渉検出 |
| inner_link_right_1 | inner_link_right_1_collision | mesh | C | アーム・グリッパの荷物/台車/他ロボットとの干渉検出 |
| finger_right_1 | finger_right_1_collision | box | A | 荷物の把持・接触を再現 |
| finger_right_1 | finger_right_1_collision_1 | box | A | 荷物の把持・接触を再現 |
| finger_right_1 | finger_right_1_collision_2 | box | A | 荷物の把持・接触を再現 |
| finger_right_1 | finger_right_1_collision_3 | box | A | 荷物の把持・接触を再現 |
| finger_right_1 | finger_right_1_collision_4 | box | A | 荷物の把持・接触を再現 |
| finger_right_1 | finger_right_1_collision_5 | box | A | 荷物の把持・接触を再現 |
| finger_right_1 | finger_right_1_collision_6 | box | A | 荷物の把持・接触を再現 |
| finger_right_1 | finger_right_1_collision_7 | box | A | 荷物の把持・接触を再現 |
| finger_right_1 | finger_right_1_collision_8 | box | A | 荷物の把持・接触を再現 |
| inner_link_left_1 | inner_link_left_1_collision | mesh | C | アーム・グリッパの荷物/台車/他ロボットとの干渉検出 |
| finger_left_1 | finger_left_1_collision | box | A | 荷物の把持・接触を再現 |
| finger_left_1 | finger_left_1_collision_1 | box | A | 荷物の把持・接触を再現 |
| finger_left_1 | finger_left_1_collision_2 | box | A | 荷物の把持・接触を再現 |
| finger_left_1 | finger_left_1_collision_3 | box | A | 荷物の把持・接触を再現 |
| finger_left_1 | finger_left_1_collision_4 | box | A | 荷物の把持・接触を再現 |
| finger_left_1 | finger_left_1_collision_5 | box | A | 荷物の把持・接触を再現 |
| finger_left_1 | finger_left_1_collision_6 | box | A | 荷物の把持・接触を再現 |
| finger_left_1 | finger_left_1_collision_7 | box | A | 荷物の把持・接触を再現 |
| finger_left_1 | finger_left_1_collision_8 | box | A | 荷物の把持・接触を再現 |
| outer_link_left_1 | outer_link_left_1_collision | mesh | C | アーム・グリッパの荷物/台車/他ロボットとの干渉検出 |
| outer_link_right_1 | outer_link_right_1_collision | mesh | C | アーム・グリッパの荷物/台車/他ロボットとの干渉検出 |
| front_left_wheel_1 | front_left_wheel_1_collision | cylinder | B | 床との摩擦・メカナム移動 |
| front_right_wheel_1 | front_right_wheel_1_collision | cylinder | B | 床との摩擦・メカナム移動 |

注意：base_footprint に self_collide=true、wheel は false、アーム・指は指定なし。固定リンクは URDF→SDF で base_footprint へ統合される。同一固定リンク内の形状は独立したジョイント拘束ではない。

使用可能方式：Fortress の公式比較表は collide bitmask をサポートすると記載する。[公式ドキュメント](https://gazebosim.org/docs/fortress/comparison/)。実際の採用には既存構成の有効な衝突対を確認し、finger↔payload、wheel↔ground、base↔obstacle、arm↔payload、robot↔robot の保護を維持する。

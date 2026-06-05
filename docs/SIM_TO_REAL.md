# sim-to-real ギャップと v0 の境界

RobotDance の `sim_certificate` は **"physically-informed feasibility"** — *参照運動そのものが
ロボットの静的包絡（ROM・速度・重力トルク）と準静的バランスの内側に収まるか* の判別 — を返します。
**実機での安全実行の保証ではありません**。本ドキュメントは v0 の近似と、実機との境界を明示します。

## 何を検証し、何を検証しないか

| ✅ certificate が言えること | ❌ certificate が言えないこと |
| --- | --- |
| 参照姿勢が実機の関節 ROM 内か（実 URDF limit） | 閉ループ制御下で実機が転ばず追従できるか |
| 関節速度が実 actuator 速度上限内か | 接触・スリップ・地形・外乱への頑健性 |
| 関節トルク（重力＋並進＋回転慣性）が実 effort 上限内か | 衝撃・接触トルク・関節摩擦/ゲアが上限内か |
| 準静的 ZMP が支持多角形内か（平地） | 実機の balance 制御が成立するか |

> **受動ヒューマノイドはバランス制御なしでは何でも倒れる**ため、forward physics sim は判別力を持ちません。
> certificate は forward sim ではなく、**参照運動の実現可能性**を運動学＋逆動力学＋ZMP で評価します。

## v0 の近似（パイプライン段階別）

### Retarget（kinematic）
- 人間 → ロボットを **bone 方向マッチング**で写像（bone 長は実機値を保存）。実 actuator 空間 IK は
  別経路（`retarget-ik`）。ball-joint 近似で、bone 軸まわりの **twist は keypoints から定まらない**
  → 時系列コヒーレントな規約で再構成（v0.43→v0.47、位置不変）。

### 質量・慣性
- 質量分布・慣性テンソルは **実 URDF `<inertial>` 由来**（v0.34 / v0.52, 既定で実慣性）。ただし
  link → canonical bone は**世界 COM 最近傍**で集約する近似で、bone は capsule/点質量プロキシ。
  割当 link の無い bone は floor 質量。
- 接地フットプリント（`FOOT_BOX`）は**全機種共通の固定寸法**で、機種ごとの実足形状ではない。

### バランス（ZMP）
- COM 加速度から **準静的 ZMP**（平地・総質量点近似, ground z=0）を計算し、接地足の**実フットプリント
  矩形の凸包**を支持多角形とする。**傾斜・不整地・段差・摩擦円錐・足裏の柔軟性は未モデル**。
- balance 制御器は無い（参照運動の ZMP が支持内かのみ判定）。`march`（単脚支持）が REJECT するのは
  この準静的・受動モデルゆえで、実機は balance 制御で単脚支持を実現しうる。
  - **歩調を落とすと（`march_gentle`: 低速・低い持ち上げ）慣性トルクが下がり、狭股機種（G1/T1）は
    重心軌道が支持多角形内に収まり PASS** する（適切な歩調なら単脚支持は feasible の実証）。一方
    **広股機種（H1/Apollo）は受動準静的モデルではなお balance 違反**で、これは支持足が外側にあり
    重心を支持脚上へ載せるには**足首戦略（接地足を軸に上体を傾ける能動バランス）**が要るため。
    v0 はこの能動バランスを未モデル（剛体並進では支持足も動き COM-足の相対が変わらず無効）。
    つまり march の feasibility は **歩調（慣性）＋形態（股幅）＋能動バランスの有無**で決まる。

### トルク
- `torque_ratio` は Newton-Euler の関節トルク **`τ = dL_com/dt + r × m·(a_com − g)`**／実 per-joint effort
  上限（v0.62 で重力＋並進慣性, v0.63 で **subtree 角運動量変化 dL_com/dt（回転慣性反作用）**を追加）。
  a_com・dL_com/dt は ZMP と同じ中心差分（subtree_angmom は `mj_subtreeVel`）。mj_inverse は本 ball-joint
  浮遊モデルで特異性により非物理値（数千 N·m）を出すため使わず、剛体 subtree 近似の解析法で算出。
- ⚠️ **衝撃・接触トルク**、関節摩擦/ゲア比/バックラッシュは未モデル。**重力保持**（準静的）成分は
  `gravity_torque_nm`、重力＋並進＋回転慣性の合計は `dynamic_torque_nm` として別途報告。負荷率が最大の
  **律速関節**は `torque_limiting_joint`（PASS でも最も上限に近い関節）として出し、REJECT 理由には
  「{関節名} {動的tq}>{上限} N·m」を併記する（どの関節が effort 上限を律速するかを明示）。

### 速度
- 関節速度を **実 per-joint velocity 上限**（v0.38/v0.50）と比較。実値の無い機種（Apollo）は generic
  fallback（[provenance](EMBODIMENTS.md) 参照）。

### 接触
- `contact_schedule` は **keypoints の足首高さ閾値**から導く運動学的な接地で、物理接触解決ではない。
  スリップ・接触力・離地衝撃は未モデル。

### Tracking（RL/PD baseline）
- `TrackingEnv` は PD baseline ＋ PPO 残差。base（骨盤 free joint 6-DOF）は**非駆動**。
  feasibility 検証とは別物で、SOTA tracking（DeepMimic/AMP 等）でも実機転移済みでもない。

## まったくモデル化していないもの

actuator 動力学（ゲア比・摩擦・バックラッシュ・電流/熱制限）・通信遅延・センサノイズ・リンク弾性/
コンプライアンス・自己衝突（一部のみ）・地形/路面材質・把持/外部接触。

## 実機へ渡す前に

certificate が **PASS** でも、実機では必ず (1) balance/姿勢制御、(2) 関節空間 safety guard
（位置/速度/加速度/トルククランプ, `demo-joint-safety`）、(3) 漸進的な実機検証 を通すこと。
certificate は「明らかに不可能な参照を早期に弾く」フィルタであり、最終的な安全性は実機側の制御・
ガード・検証が担う。

> v0 は意図的に近似を多く含む pre-alpha です。各近似は「実機の事実を装った嘘」を避けるため、
> 測れないものは placeholder/None を返し（例: actuator の無い toe の limit, Apollo の velocity）、
> 近似であることを本ドキュメント・各 README・コード comment で明示しています。

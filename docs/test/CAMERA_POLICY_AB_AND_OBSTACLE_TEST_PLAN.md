# 相机→策略差异与避障性能测试方案（下一步）

> **2026-11-19 更新**：本文 §0.4 里"`lookahead_min` 首选 0.65–0.75"的建议已被实机推翻（0.7 更难过窄门）。当前部署默认固定为 `lookahead_min=0.4` + `switch_mode=corner_dist`，前瞻相关实验以 [LOOKAHEAD_SWITCHING_TEST_20261118.md](LOOKAHEAD_SWITCHING_TEST_20261118.md) 和 [PARAMETER_REFERENCE_20261119.md](../deployment/PARAMETER_REFERENCE_20261119.md) 为准。

> 前置阅读：
> - `docs/deployment/REALSENSE_DEPLOYMENT_EVAL_AND_PARAM_TUNING_PLAN.md`（总体方案）
> - `docs/test/DOOR_STAIRS_CAMERA_AB_REALSENSE_VS_ZED_20261110.md`（两组全流程 A/B 结论）
> - `docs/test/PREPROCESSING_ABLATION_V0_V5_20261110.md`（V0–V5 离线消融）
>
> 本文档回答三件事：
> 1. 目前已经确认了什么、改了什么；
> 2. 同一场景/同一状态下，相机差异让策略输出差多少（L1，离线）；
> 3. 避障性能（低障碍、门口侧向障碍、密集障碍）怎么测（L2，实机）。

---

## 0. 当前状态（2026-11-17）

### 0.1 已完成

| 工作 | 结论 | 文档/产物 |
|---|---|---|
| 两组全流程 A/B（2026-11-10） | 真实深度两台基本一致；差距在视场、填充、延迟；平地上楼误触发是路径末点 z 偏移，不是相机 | `DOOR_STAIRS_CAMERA_AB_REALSENSE_VS_ZED_20261110.md` |
| 234 对匹配位姿对比 | ZED 深度年龄中位 540 ms vs RS 183 ms；FOV 101.4°×69.0° vs 89.5°×58.6°；近地盲距 0.92 vs 1.10 m | 同上 |
| V0–V5 预处理消融 | **V2（最近有效值填充）最优**：假近单元 1.3%/0.9% → 0.0%/0.0%，近障碍比例回到真实值，跨相机 Δcenter/Δright 0.42/0.49 → 0.29/0.16 | `PREPROCESSING_ABLATION_V0_V5_20261110.md` |
| V2 上机 A/B（2026-11-10） | **实机变差**：平地 `cmd_wz` 绝对值 0.277→0.440、饱和 2.8%→25.6%、航向误差 13.0°→21.7°；判定为策略在旧填充下训练，V2 造成输入分布外 | `DEPTH_FILL_MODE_STUTTER_ANALYSIS_20261117.md`；分支 `exp/depth-fill-v2`，已回退 |
| 部署代码改动 | 控制器（`rl_nav_controller.py` + `rl_nav_controller_adaptive.py`）的 V2 改动已保存到 `exp/depth-fill-v2` 分支并回退；当前代码恢复为旧的 `min_depth` 填充 | `sru-robot-deployment` |
| **Stage 0 前瞻路径回放（2026-11-17）** | 两段新数据（`session_20261110_144216/145109_realsense`）。① `lookahead_max` 在 1.5–2.5 几乎无影响（曲率分支 57% 的帧压在 `lookahead_min`）；② `lookahead_min` 是主杠杆，平台掉头需求在 0.65–0.75 取极小（35°→17°）；③ 有效前瞻比标称短 **0.35–0.5 m**（等价 τ≈1.2 s），楼梯 0.4 m 被完全抵消；④ `window_dist>2.0` 有害；⑤ `slew_rate_*`/`turn_angle_low` 测不出差别；⑥ `adaptive_mode=both` 不可用（95% 帧压到最小） | `STAGE0_LOOKAHEAD_REPLAY_20261117.md`；`scripts/stage0_lookahead_replay.py`；`door_diagnostics/analysis_stage0/` |

### 0.2 已定位、与相机无关的问题

1. **全局路径末点 z 偏移**：两组路径的末点 z 都精确等于 `goal z + 0.50 m`。ZED 的 goal z 在地面，末点被抬到 −3.62，最后一段是 +0.52 m / 0.04 m 水平（slope +12.5）的假上坡，导致平地误触发“上楼”9 帧；RS 的 goal z 恰好低 0.5 m，抵消了偏移。**先修路径/下点链路，或让楼梯门控忽略末段 slope > 1 的线段。**
2. **TF 记录不完整**：ZED 会话的 `tf_base_to_camera.json` 是 `camera_link` 回退值，与 RS 相同。按用户提供的实际几何：RS 在机器狗自带相机上方 4.5 cm、中轴、前面超出 0.5 cm；ZED 在自带相机上方 7 cm、前面与 RS 后面共面（ZED 厚 3.8 cm）。即 **ZED 比 RS 高 ~2.5 cm、靠后 ~2.5 cm，两台均水平（0°）**。方案 §1.3 的 ZED TF `(0.257, 0, 0.28)` 与本次描述不一致（x 差 8.3 cm），需确认是旧支架还是文档未更新。
3. **logger 的 note 标签写错**：RS 会话写 `RS run1`，ZED 会话写 `RS run2`，容易误用。

### 0.3 相机结论与仍需验证的因素

**相机 A/B 结论（2026-11-17）**：结合两组全流程数据与 V2 上机后的表现，**ZED 与 RealSense 在闭环表现上没有显著差距**——深度精度、近失位置、任务时间都在同一量级，没有一台相机表现出稳定的优势或劣势。**相机选型冻结**：后续前瞻/避障参数测试固定用同一台相机（建议用当前部署的相机，保持基线一致），不再做相机 A/B（除非后续出现与相机强相关的失败）。

| 因素 | 现状 | 是否阻塞当前测试 |
|---|---|---|
| ZED 深度延迟 ~0.35–0.5 s | 记录器侧测量；ZED 特有 | 否（若固定用 RealSense 则无关） |
| FOV/角映射不一致 | 几何已量化；V5 对齐后分歧最小但覆盖率 46.7%/70.5% | 否（留作后续 P1） |
| V2 填充 | 实机变差，已回退到 `min` | 否（当前基线用 `min`） |
| 避障性能 | 未做受控测试 | 否（前瞻参数之后） |

---

### 0.4 下一阶段：前瞻参数测试（Stage 0 已完成，进入 Stage 1）

目的：在**冻结相机与预处理（`min` 填充）**的前提下，系统调前瞻/门/楼梯参数，优先解决平地摆动、转弯内切、门内时序，而不是继续换相机。

#### Stage 0 结论对计划的修改（2026-11-17）

完整报告见 [`STAGE0_LOOKAHEAD_REPLAY_20261117.md`](STAGE0_LOOKAHEAD_REPLAY_20261117.md)。三条改变原计划的结论：

1. **`lookahead_max` 不是杠杆，不要第一个测**：部署默认 `curvature_only`，这条路线 57% 的帧 `L` 就压在 `lookahead_min` 上（`L<1 m` 占 65%），`lookahead_max` 1.5→2.5 只把全局 `dΘ` p95 从 41° 推到 44°，而且方向是"调大更差"。
2. **`lookahead_min` 才是主杠杆**，且是 U 形：平台掉头（蹭栏杆处）的目标夹角需求在 **0.65–0.75 m** 取极小（0.35→48°、0.5→35°、0.7→17°、0.9→27°）。
3. **有效前瞻比标称短 0.35–0.5 m**（等价 1.2 s 位姿/进度延迟；楼梯段标称 0.40 m 实测只有 0.04–0.06 m）。**这是当前最该查的一条**，优先级高于任何参数扫描：它同时影响楼梯、平地、门口三处。

#### 参数分组（按 Stage 0 灵敏度重排）

| 优先级 | 参数 | 当前值 | 建议初值/搜索范围 | 依据 |
|---|---|---:|---|---|
| P0（先查） | 路径进度/胡萝卜延迟 | — | 见 §0.5 | 有效前瞻短 0.35–0.5 m |
| P1 | `lookahead_min` | 0.50 | **0.65–0.75**（测 0.5/0.6/0.7/0.8） | 平台掉头 35°→17° |
| P1 | `stair_lookahead` | 0.40 | **0.55**（与 0.40 A/B） | 底部窄门 15°→13°；0.25 明显更差 |
| P2 | `turn_angle_high` | 50° | 60（2 值即可） | L p50 +0.1 m，拐角无退化 |
| P2 | `window_dist` | 1.60 | 保持（只确认不要 >2.0） | 2.0 时 73% 帧压到最小 |
| P3 | `lookahead_max` | 2.00 | 不动；要测只测 2.0 / 1.5 | 见上 |
| — | `turn_angle_low`、`slew_rate_*` | 20° / 2.5·0.8 | **不测** | 差异 ≤2°，在噪声内 |
| — | 窄门组 `clearance_*`/`passage_latch_*`/`door_speed_boost` | — | **推迟** | `adaptive_mode=both` 时 95% 帧被压到最小，分支要先修才能用 |
| — | 楼梯门控 `stair_slope_threshold` 等 | 见方案 §4.1 | 暂不动 | 楼梯段没有误触发，只有平台 9–12 帧掉线 |

#### Stage 1：实机粗调（修订顺序，3–5 天）

- 固定相机、`min` 填充、同一路线/目标；每组 3–5 个值，每值 5 次；一次只动一个参数；
- 顺序：**`lookahead_min` → `stair_lookahead` → `turn_angle_high` → `window_dist`（只确认）**；
- 指标：平台掉头段的 `|cmd_wz|` 峰值与实际 `odom_wz`、该段航向扫过角、是否蹭栏杆；楼梯段速度与 `depth_center_min` 的关系、底部窄门是否撞墙；TTG；平地速度纹波/`cmd_wz` 饱和（`scripts/analyze_motion_stutter.py`）；接管/碰撞；
- 每组回到该组最优再进下一组。

#### Stage 2：交互与确认（1 周）

- 对 `lookahead_min × turn_angle_high` 做 2 水平全因子（Stage 0 已给出这两个才是交互项）；`stair_lookahead` 单独因子；
- 关键场景 10–15 次、跨天；配对统计（McNemar/Wilcoxon）。

#### 当前可直接做的三件事

1. **查 P0 延迟**：按 `STAGE0_LOOKAHEAD_REPLAY_20261117.md` §7 补三处遥测（控制器每 tick 的 `progress_s/L/位姿`、logger 记录 marker 的 `header.stamp`、控制器启动回读参数），再录一段 3–5 分钟直线+门+楼梯做对账；
2. **暴露参数**：把 `lookahead_*`/`stair_lookahead` 从启动脚本/launch 注入，并记录 profile hash（现在 `session_meta.json` 里 `controller_*` 字段是 `null`）；
3. **建立平滑度基线**：用 `scripts/analyze_motion_stutter.py` 跑 `min` 基线的两段数据，后续每组参数用同一张表对比。

---

### 0.5 P0：路径进度 / 胡萝卜延迟（先查这个，再调参数）

Stage 0 的核心发现：**策略拿到的 carrot 比标称前瞻短 0.35–0.5 m**，等价于 1.2 s 的位姿/进度延迟；楼梯段标称 0.40 m 实测只剩 0.04–0.06 m，等于下楼时没有目标点。证据与复现见 [`STAGE0_LOOKAHEAD_REPLAY_20261117.md`](STAGE0_LOOKAHEAD_REPLAY_20261117.md) §4。

**为什么先查**：它同时影响楼梯、平台掉头、平地三处；不修的话，所有前瞻参数都是在"标称值 ≠ 生效值"的前提下调的，`lookahead_min` 从 0.5 调到 0.7 实际只是把生效值从 0.1 抬到 0.3。

**三种可能，需要一次录制区分**：

| 可能 | 判别方法 |
|---|---|
| 控制器路径进度用了旧位姿（控制环/executor 落后） | 控制器每 tick 发布 `progress_s` 与**本 tick 实际使用的 map 位姿**，与 logger 记录的位姿对时 |
| `/vis/lookahead_goal` marker 被记录器晚收/晚处理 | logger 记录 marker 的 `header.stamp`，与行时间戳比较 |
| 部署端 `adaptive_path_tracker.py`/启动参数与仓库不一致 | 控制器启动回读参数写入 `session_meta.json`；用 `stage0_lookahead_replay.py --tracker <路径>` 换版本对比 |

**最低成本的第一次实验**（不改代码也能做一半）：

1. `ros2 topic hz /loc/robo/odom`、`ros2 topic delay /loc/robo/odom` 记录 odom 的发布频率与延迟；
2. 控制器日志里已经每秒打印 `[AdaptiveLookahead][mode] L=...`，录一段同时抓 `ros2 topic echo /vis/lookahead_goal`，比较同一 `header.stamp`；
3. 若确认是进度延迟，最小改动是 `update_progress()` 前把位姿按 `v·τ` 前推，或改用与命令同源的最新位姿。

---

## 1. V2 上机验证（已回退，存档）

> 2026-11-17：V2 上机后平地偏航摆动明显变差（见 `DEPTH_FILL_MODE_STUTTER_ANALYSIS_20261117.md`），已保存到 `exp/depth-fill-v2` 分支并回退到 `min` 填充。以下内容保留作记录，不再作为当前默认。

### 1.1 改了什么

- `rl_nav_controller.py` 与 `rl_nav_controller_adaptive.py`（部署的 `rl_nav_controller_stairs.py` 继承后者）新增参数 `depth_fill_mode`，默认 `nearest`：
  - `far_mask`（> `max_depth` / +inf）仍置 0；
  - `fill_mask`（0 / NaN / < `min_depth`）用 **最近有效深度** 填充（`cv2.distanceTransformWithLabels`，一次 O(N)，Jetson 上约 1–2 ms）；
  - 没有有效像素时回退到 `min_depth`；
  - 设 `depth_fill_mode:=min` 可一键回到旧行为，便于同一天 A/B。
- `start_nav.sh` / `start_nav_custom.sh` 会传 `depth_fill_mode:=${DEPTH_FILL_MODE:-nearest}`；同一天 A/B 时只需 `export DEPTH_FILL_MODE=min`（或 `nearest`）后重启，不用改代码。
- `door_diagnostic_logger.py` 新增 `--depth-fill-mode`（默认 `nearest`），并写入 `session_args.json` / `session_meta.json`，保证记录的 64×40 与策略输入一致。

### 1.2 上机怎么测

1. **先跑通**：同一条门/楼梯路线，`depth_fill_mode=nearest` 跑 1 次，确认无异常、无新增急停/接管；看 logger 的 64×40 与旧 session 的差异（中心/右侧最小值应回到 1 m 量级）。
2. **同日 A/B**：同一路线、同一 goal/路径，`nearest` 与 `min` 交错各 3–5 次（A B B A）。相机固定，只改填充。
3. **记录**：视频 + logger（`--depth-fill-mode` 会自动记录）+ LiDAR + 接触/接管。
4. **对比指标**：
   - 假近单元（`false_near`）、近障碍比例（`near08`）与真实值；
   - 中心/右侧最小距离、`floor_near`；
   - 窄门/门口通过时间、停滞/恢复次数、人工接管；
   - 楼梯门控进入/退出位置、误触发；
   - 同一位置的横向净空（机体足迹，见 §3.2 / §4.2）。
5. **验收**：`false_near` 明显下降；无新增碰撞/接管；窄门通过时间不劣化；楼梯门控时序不劣化。若出现“过于大胆”（策略不再把空洞当近墙），优先看门口/窄门的 `d_body` 与接管率。

> 预期：V2 会让策略输入更“干净”，减少假近障碍；但**不会改变 ZED 的 0.5 s 延迟和 FOV**，所以门口侧向障碍的风险仍在。V2 的收益应主要体现在窄门/走廊的平顺性，而不是避障距离。

### 1.3 部署步骤（机器狗）

改动文件（开发机 `sru-robot-deployment` → 机器狗 `~/go2w_nav_ros2_ws`）：

- `src/rl_nav_controller/rl_nav_controller/rl_nav_controller.py`
- `src/rl_nav_controller/rl_nav_controller/rl_nav_controller_adaptive.py`（**部署的 `rl_nav_controller_stairs.py` 继承它，必须一起更新**）
- `src/rl_nav_controller/launch/rl_nav_controller.launch.py`
- `door_diagnostic_logger.py`
- `start_nav.sh` / `start_nav_custom.sh`

步骤：

1. 同步上述文件到机器狗 `~/go2w_nav_ros2_ws`；
2. `start_nav.sh` 直接执行 Python 源码，**改完重启即可，不需要 colcon build**；`start_nav_custom.sh` 走 launch 文件，需 `colcon build --packages-select rl_nav_controller`（或使用 `--symlink-install`）；
3. 启动后确认控制器日志有 `Depth fill mode: nearest`；logger 的 `session_meta.json` 里 `depth_fill_mode` 应为 `nearest`；
4. 同一天 A/B：`DEPTH_FILL_MODE=nearest ./start_nav.sh` 与 `DEPTH_FILL_MODE=min ./start_nav.sh`，logger 不需要额外参数（默认 `nearest`）；若控制器用 `min`，logger 也要加 `--depth-fill-mode min`，保证记录的 64×40 与策略输入一致。

---

## 2. L1 离线动作级回放（第二优先，需要补字段）

### 2.1 目的

把“相机差异”与“轨迹分叉”解耦：同一机器人状态、同一目标下，只换深度输入，看策略动作差多少。

### 2.2 现有日志的字段缺口

观测组装（`rl_nav_controller.py: predict()`）：

```
obs = [linear_vel(3), angular_vel(3), gravity(3), last_action(3), target_pos_log(4), (heading(2))]
      + depth_embedding(2560)
```

| 字段 | 控制器来源 | 现有 CSV | 处理 |
|---|---|---|---|
| `linear_vel` / `angular_vel` | odom twist 转 body 系 | `odom_vx/vy/vz/wz`（frame 待确认） | 先确认 frame；必要时补 |
| `gravity_vector` | 由姿态投影 | `imu_ax/ay/az`（原始加速度） | 由 roll/pitch 重建 |
| `last_action` | 上一帧 `raw_action` | 无（`raw_cmd_*` 是 `nav_vel`，不是网络原始输出） | **需补日志**，或用回放重建 |
| `target_yaw_w` | carrot yaw | `goal_yaw_deg` | **需补 carrot yaw** |
| `raw_action` | 网络原始输出 | 无 | **需补日志**，用于一致性校验 |

**严格回放需要先补 `last_action`、`target_yaw_w`、`raw_action` 三个字段**（在控制器里发布/记录即可），并重录一段 5 分钟的直线+门+楼梯做校验。不补只能做近似回放（用 `cmd` 近似 `last_action`、用 goal/carrot yaw 近似 `target_yaw_w`），误差会偏大，结论需标注。

### 2.3 回放与反事实

1. **一致性校验**：从任务段起点 reset LSTM，逐帧回放 ONNX，与 `raw_action` 误差 <1e-3；不通过不进入下一步。
2. **V0 vs V2**：同一状态、同一 LSTM，分别喂 V0/V2 的 64×40，比较 `Δvx/Δvy/Δwz`、动作符号一致率、安全关键分歧率。这是验证“假近减少 → 动作更合理”的最直接证据。
3. **跨相机反事实**：在 234 对匹配位姿上，固定状态换深度（A 状态 + B 深度、B 状态 + A 深度），统计 Δaction 与真实净空的相关性。
4. **预处理变体**：V2（推荐）与 V5（交集 FOV）各跑一遍，比较动作分歧与覆盖率。

### 2.4 产出

`counterfactual_pairs.csv`、Δaction 分布图、按场景分段（直道/门/楼梯/平台）汇总。

---

## 3. 避障 L2 闭环测试（第三优先）

### 3.1 场景与夹具（标准化、可重复）

| ID | 场景 | 夹具 | 关键考核 |
|---|---|---|---|
| O1 | 低障碍 | 10/15/20 cm 高纸箱（30×30 cm），放在直道中央 | 探测距离、碰撞、绕行侧 |
| O2 | 门口侧向障碍 | 灭火器/圆柱（直径 ~15 cm），放在门框边缘，横向侵入 0.10/0.20/0.30 m | 侧面净空、是否接触、通过时间 |
| O3 | 窄门 | 0.80/0.85/0.90 m 门洞（软包门框） | 通过率、门框接触、latch 时序 |
| O4 | 密集障碍 | 3–5 个箱子，通道 1.0/1.2 m | 最小净空、恢复/停滞次数 |
| O5 | 反光/低纹理（可选） | 玻璃、黑绒布、金属板 | 无效像素、假障碍 |
| O6 | 楼梯平台栏杆 | 现有场地，先修全局路径 | 最小净空、是否接触 |

每个场景固定起点胶带、goal、全局路径；障碍物位置用胶带标记，每次复位。

### 3.2 避障专用指标

| 指标 | 定义 | 数据来源 |
|---|---|---|
| 探测距离 `d_det` | 障碍物首次在策略输入（真实有效深度）中 <0.8 m 时，机器人到障碍的距离 | 深度 + LiDAR |
| 避让起点 `d_avoid` | 首次出现与障碍相关的 `wz`/`vy` 变化时的距离 | cmd + 轨迹 |
| 最小机体净空 `d_body` | 机体矩形（含轮距）到障碍的最小距离 | LiDAR + 机体尺寸 |
| 碰撞 `C` | 接触（足端力/视频复核） | 接触/视频 |
| 近失 `NM` | `d_body < 0.10 m` | LiDAR |
| 通过侧 | 从左/右绕行，是否符合安全侧 | 轨迹 |
| 回归距离 | 绕过后回到路径的距离 | 轨迹 + 路径 |
| 停滞/恢复 | `blocked`/重规划次数 | 状态机 |
| 通过时间 | 进入障碍区到离开 | 时间戳 |

### 3.3 协议

- **初筛**：每个（相机 × 场景 × 障碍参数）5 次；先做 O1 10 cm、O2 0.20 m、O3 0.85 m、O4 1.2 m；
- **确认**：关键对比 10–15 次；交错 A/B（A B B A），同一天完成；
- **固定**：同一起点/goal/路径、同一速度上限（0.4–0.6 m/s）、同一光照；
- **记录**：视频 + LiDAR + 深度 + cmd + 接触；每次填写障碍物实际位置；
- **统计**：成功率/碰撞率用 McNemar；连续指标用 Wilcoxon + bootstrap 95% CI；分层报告，不把不同难度平均。

### 3.4 可检验假设

| # | 假设 | 预期结果 | 如果成立说明什么 |
|---|---|---|---|
| H1 | RS 对低障碍探测更晚 | `d_det(RS) < d_det(ZED)`，因近地盲距 1.10 vs 0.92 m | 需要下俯/虚拟相机/ LiDAR 守门 |
| H2 | RS 对黑/低纹理障碍更稳 | RS 碰撞率更低（主动 IR） | ZED 需要补光/换深度模式 |
| H3 | ZED 对门口侧向障碍反应更晚 | O2 的 `d_body` 更小、接触更多（延迟 0.5 s + 斑块更小） | 优先降 ZED 延迟 |
| H4 | V2 让窄门更平顺 | O3 通过时间更短/停滞更少（假近障碍减少） | 保留 V2 为默认 |
| H5 | V5（交集 FOV）降低跨相机分歧且保持覆盖 | L1 Δaction 下降、O1–O4 差异缩小 | 作为 P1 默认预处理 |

### 3.5 安全

- 低障碍/窄门：软包、限速 0.4 m/s、守护人员在位、急停可用；
- 楼梯平台：安全带/软牵引，先单级验证；
- 禁止玻璃门；反光地面单独评估；
- 任何定位跳变/深度丢失立即接管并标记 invalid。

---

## 4. 工具与代码改动清单

### 4.1 已完成

| 位置 | 改动 |
|---|---|
| `sru-robot-deployment/.../rl_nav_controller.py`、`rl_nav_controller_adaptive.py` | `depth_fill_mode` 参数（默认 `nearest`=V2）；`_fill_invalid_depth()`（stairs 节点继承 adaptive） |
| `sru-robot-deployment/.../door_diagnostic_logger.py` | `--depth-fill-mode`（默认 `nearest`）；记录到 args/meta；64×40 与控制器一致 |
| `sru-navigation-learning/scripts/analyze_camera_sessions.py` | 裁剪、匹配位姿、轨迹/门控/近失、路径末点 z 检查 |
| `sru-navigation-learning/scripts/depth_benchmark/reprocess_depth.py` | V0–V5 消融、指标、案例图 |
| `sru-navigation-learning/scripts/stage0_lookahead_replay.py` | Stage 0：用部署端 `AdaptivePathSegmentTracker` 原码回放前瞻/carrot/目标夹角，支持延迟拟合与一维参数扫描 |

### 4.2 待做

| 位置 | 改动 |
|---|---|
| `rl_nav_controller.py` / `_adaptive` / `_stairs` | 每 tick 发布 `progress_s`/`current_lookahead`/`l_curv`/`l_clear`/`turn_deg`/`in_narrow` + **本 tick 使用的位姿**（logger 的 `adaptive_*` 七列已预留但全空）；启动回读前瞻参数 |
| `door_diagnostic_logger.py` | marker 的 `header.stamp`；机体矩形净空、接触/足端力、`stair_gate_slope/dz`；修 `lidar_min_dist_any`（恒 0）与 `lidar_nearest_x/y`（不像世界坐标）；TF 候选帧 |
| 全局路径/下点链路 | 末点 z 不再固定 +0.50 m；用局部地面高度钳位 |
| `scripts/depth_benchmark/replay_policy.py`（新） | L1 回放：ONNX + LSTM + 观测组装 |
| `scripts/depth_benchmark/counterfactual.py`（新） | 匹配状态 + 深度替换 + Δaction |
| `scripts/benchmark/` | 避障场景定义、`d_det`/`d_body`/接触字段 |

---

## 5. 建议的优先级与时间表

| 时间 | 工作 | 产出 |
|---|---|---|
| 已完成 | 冻结相机（不再做相机 A/B）；V2 回退到 `min` 基线；Stage 0 前瞻回放 | 可复现的输入、参数可行域 |
| 第 1 天 | 修路径末点 z（见 §0.2）；查 P0 延迟（§0.5） | 生效前瞻 = 标称前瞻 |
| 第 2–3 天 | Stage 1-1：`lookahead_min` 0.5/0.6/0.7/0.8 | 主杠杆最优 |
| 第 4–5 天 | Stage 1-2：`stair_lookahead` 0.40/0.55（楼梯需守护） | 楼梯/底部窄门 |
| 第 2 周 | Stage 1-3：`turn_angle_high` 50/60、`window_dist` 1.6/1.8 确认 | 前瞻组收敛 |
| 第 3 周 | Stage 2 交互 DOE + 跨天确认 | 稳健参数、profile v1 |
| 第 4 周 | L1 动作级回放（字段补齐后）与避障 O1–O4（可选） | 动作级归因、避障基线 |

---

## 6. 验收标准

- **V2 上机**：`false_near` 明显下降；无新增碰撞/接管；窄门通过时间不劣化；楼梯门控时序不劣化。
- **L1**：回放误差 <1e-3；给出同状态换相机、V0 vs V2 的 Δaction 分布。
- **L2**：关键场景 ≥5 次初筛、10–15 次确认；碰撞率/近失率/`d_body` 有置信区间；给出相机推荐。
- **可复现**：每次 run 记录相机、`depth_fill_mode`、TF、profile hash、电池、视频。

---

## 7. 现在就能做的三件事

1. **查 P0 延迟**（§0.5）：先不改代码，用 `ros2 topic hz/delay /loc/robo/odom` + 控制器每秒打印的 `[AdaptiveLookahead] L=...` 与 `/vis/lookahead_goal` 的 `header.stamp` 对齐，判断是进度延迟还是 marker 记录延迟；
2. **暴露参数并回读**：把 `lookahead_*`/`stair_lookahead` 从启动脚本注入，控制器启动时把实际值写进 `session_meta.json`（现在 `controller_*` 字段是 `null`），并记录 profile hash；
3. **建立平滑度基线**：用 `scripts/analyze_motion_stutter.py` 跑 `min` 基线的两段数据，后续每组参数用同一张表对比。

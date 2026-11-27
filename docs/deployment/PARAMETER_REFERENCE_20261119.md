# 部署参数总表（`go2w_nav_ros2_ws`，2026-11-19，2026-11-26 同步 sru-local）

对应代码：`rl_nav_controller_stairs.py`（继承 `rl_nav_controller_adaptive.py`）+ `adaptive_path_tracker.py`
入口脚本：旧部署仓库 `start_nav.sh`；sru-local 起为 `sru_ws/start_global_sru.sh` / `sru_global.launch.py`

> **2026-11-26 同步 sru-local 启动参数**：以 `~/sru-local` 的 `sru_global.launch.py`、`config/global.yaml`、`rl_nav_controller_stairs.py` 与 [dev-local-sru](https://github.com/agile-nav/agile-nav.github.io/blob/main/draft/dev-local-sru.md) 为准，本次调整的默认值为：
>
> - `lookahead_distance` 1.0 → **2.0 m**，`lookahead_max` 1.0 → **2.0 m**（`constants.py`、adaptive tracker、各 launch 同步）；
> - `segment_arrive_threshold` 0.20 → **0.45 m**；
> - `terminal_align_enter_dist` 0.20 → **0.50 m**，`terminal_align_abort_dist` 0.40 → **0.60 m**。
>
> 下表“当前生效”列已按新默认值更新；来源标注中的“脚本”在 sru-local 下对应 launch/`config/global.yaml` 参数。

---

## 0. 怎么读这张表

所有参数都是 ROS 2 参数。优先级：

```
命令行/launch 传入的 -p        ← start_nav.sh 传了 32 个
        ↓ 没传的
代码 declare_parameter 默认值  ← 有可能再被 constants.py 覆盖
```

所以**"当前生效"这一列 = `./start_nav.sh` 直接启动时的实际值**。表格里"来源"标注：

- `脚本` = start_nav.sh 的默认值，并且会显式 `-p` 传进去
- `代码` = start_nav.sh 不传，用 `declare_parameter` 的默认
- `constants` = 代码默认值来自 `constants.py`

⚠️ **2026-11-19 起默认值已固定为现场验证过的组合**：`LOOKAHEAD_MIN=0.4`、`SWITCH_MODE=corner_dist`。即直接 `./start_nav.sh` 就是当前实机效果最好的一套，不需要再手工加 `--min 0.4 --switch-mode corner_dist`。代码里 `declare_parameter` 的默认值也已同步改成这两个，所以绕过 `start_nav.sh` 直接 `ros2 run` 行为一致。

---

## A. 前瞻 / 切换（核心，本轮唯一要动的族）

| 参数 | 当前生效 | 来源 | 作用 | 可测值 |
|---|---:|---|---|---|
| `enable_adaptive_lookahead` | `true` | 脚本 | 总开关。关掉就走固定 `lookahead_distance` | — |
| `adaptive_mode` | `curvature_only` | 脚本 | 选哪条自适应分支：`curvature_only`（按路径转弯收缩）/ `clearance_only`（按深度看窄门）/ `both`（取两者最小）/ `none` | 保持 |
| `lookahead_min` | **0.40** | 脚本 | 前瞻下界。转弯/楼梯/窄门时收在这里 | **0.40 / 0.50**（0.7 已证明毁窄门） |
| `lookahead_max` | 2.0 | 脚本 | 平地上界，决定避障视野 | 2.0 / 2.5 |
| `adaptive_window_dist` | 1.6 | 脚本 | 曲率规则往前看多远。**只对 `curvature`/`hybrid` 有效** | 1.2 / 1.6 / 2.0 |
| `turn_angle_low_deg` | 20.0 | 脚本 | 曲率规则下阈值：窗口内累计转角 < 它 → 判直道 → 用 max。**`corner_dist` 下无效** | 20 / 30 / 40 |
| `turn_angle_high_deg` | 50.0 | 脚本 | 上阈值：> 它 → 直接收到 min。**`corner_dist` 下无效** | 保持 50 |
| `slew_rate_increase` | 0.8 | 脚本 | **前瞻变长的速率**（m/s）。出弯后多快恢复到长前瞻 | **0.8 / 2.0** |
| `slew_rate_decrease` | 2.5 | 脚本 | 前瞻变短的速率。2.0→0.4 只需 0.64 s，不是瓶颈 | 保持 2.5 |
| `switch_mode` | **`corner_dist`** | 脚本 | 切换规则（见 A.1）：`corner_dist` / `curvature` / `hybrid` | `corner_dist` / `curvature` / `hybrid` |
| `corner_switch_thresh_deg` | 25.0 | 脚本 | `corner_dist` 下：路径航向偏离多少度算"拐角" | 25 / 35 |
| `corner_switch_margin` | 0.25 | 脚本 | `corner_dist` 下：carrot 停在拐角前多少米 | 0.25 / 0.45 |
| `passage_latch_dist` | 1.2 | 代码 | 窄门锁存距离。`clearance` 模式专用，curvature_only 下无效 | 不动 |
| `door_speed_boost` | 0.0 | 脚本 | 过门保底速度。0 = 完全交给策略 | 不动 |
| `lookahead_distance` | 2.0 | 代码 | 固定前瞻。只在 `enable_adaptive_lookahead=false` 或单点模式生效 | 不动 |

### A.1 默认切换规则 `corner_dist`

```
d_corner(s) = 沿路径从 s 往前，航向第一次偏离 s 处航向 ≥ corner_switch_thresh_deg 的弧长（上限 6 m）
L(s)        = clamp(d_corner(s) - corner_switch_margin, lookahead_min, lookahead_max)
```

判定方式：路径一变就把折线按 **0.05 m** 重采样，用 ±0.30 m 窗口平滑出每点航向，再对每个采样点向前搜"第一次偏离 ≥ 阈值的距离"。它评的是**净偏离**，所以直线上的折线抖动不会误触发。

几何含义：**carrot 永远停在拐角前 `corner_switch_margin` 米处**。以 margin 0.25、Lmax 2.0、Lmin 0.4 为例，一个 90° 拐角：

| 离拐角 | 2.25 m | 2.0 m | 1.5 m | 1.0 m | 0.65 m | ≤0.65 m | 过拐角后 |
|---|---:|---:|---:|---:|---:|---:|---:|
| L | 2.00 | 1.75 | 1.25 | 0.75 | 0.40 | 0.40 | 回到 2.00 |

对比旧的曲率规则：拐角进入 1.6 m 窗口（约剩 1.5 m）目标值才瞬间跳到 0.4，实际靠 `slew_rate_decrease` 用约 0.26 m 行程塌下去，等于在拐角前 1.2 m 处一步到位。新规则提前 2.25 m 开始线性收缩，没有突变。

行为差异汇总：

| 场景 | `corner_dist`（默认） | 旧 `curvature` |
|---|---|---|
| 平地直道 | 稳定在 `lookahead_max` | 会被折线抖动/缓弯压短 |
| 缓弯（< 25°） | 仍用长前瞻 | 累计转角超 20° 就收缩 |
| 进弯 | 线性下降，carrot 不切内角 | 到窗口边缘突然塌到 min |
| 出弯恢复 | 目标立刻回 max，实际速率仍由 `slew_rate_increase` 决定 | 同左 |
| 楼梯 | **不受影响**（stairs 分支直接用 `stair_lookahead_dist`） | 不受影响 |
| 到点 | 不受影响（`target_arc ≥ 总长` 时钳到终点） | 不受影响 |

风险：路径在门口/缓弯处若没有 ≥25° 的折角，会一直保持长前瞻，可能贴门框或切 30° 弯的内角。此时降到 `--corner-thresh 20`，或整体切回 `--switch-mode curvature` / `hybrid`（取两规则较小值，最保守）。

回退方式（无需改代码）：

```bash
./start_nav.sh --switch-mode curvature --slew-inc 0.8 --turn-low 20
```

---

## B. 楼梯

| 参数 | 当前生效 | 来源 | 作用 | 可测值 |
|---|---:|---|---|---|
| `enable_stairs_mode` | `true` | 脚本 | 楼梯感知总开关（坡度门控 + 前瞻锁存） | 保持 |
| `enable_stair_remapping` | `false` | 脚本 | 深度重映射（把"深渊"改成渐进坡面、钳制扶手列） | 保持 false |
| `stair_lookahead_dist` | 0.40 | 脚本 | **下楼**时锁定的前瞻距离 | 已定 |
| `stair_up_lookahead_dist` | 0.45 | 脚本 | **上楼**时锁定的前瞻距离 | 已定 |
| `stair_min_forward_vel` | 0.0 | 脚本 | 下楼保底前进速度（0 = 全交策略） | 0 / 0.15 |
| `stair_slope_threshold` | -0.20 | 脚本 | 触发下楼判定的坡度 | 保持 |
| `stair_up_slope_threshold` | 0.20 | 脚本 | 触发上楼判定的坡度 | 保持 |
| `stair_side_min_clamp` | 0.50 | 脚本 | 扶手侧栏杆的最小钳位距离（防贴栏杆） | **0.50 / 0.60**（蹭栏杆时可试） |
| `stair_min_elevation_drop` | 0.15 | 脚本 | 触发下楼的最小落差 | 保持 |
| `stair_lookahead_eval_dist` | 1.2 | 代码 | 评估楼梯坡度的前视距离 | 保持 |
| `stair_exit_slope_threshold` | -0.10 | 代码 | 退出下楼判定的坡度 | 保持 |
| `stair_exit_dist_threshold` | 0.6 | 代码 | 退出下楼判定的距离 | 保持 |
| `stair_up_min_elevation_gain` | 0.15 | 代码 | 触发上楼的最小爬升 | 保持 |
| `stair_up_exit_slope_threshold` | 0.10 | 代码 | 退出上楼判定的坡度 | 保持 |
| `stair_ramp_base` / `stair_ramp_step` / `stair_abyss_threshold` | 1.0 / 0.046 / 2.2 | 代码 | 只在 `enable_stair_remapping=true` 时生效 | 不动 |
| `stair_left_col` / `stair_right_col` | 16 / 48 | 代码 | 重映射时的扶手列范围（40×64 深度） | 不动 |

---

## C. 到点判定与终点转身

| 参数 | 当前生效 | 来源 | 作用 | 可测值 |
|---|---:|---|---|---|
| `segment_arrive_threshold` | **0.45** | 脚本 | **到点判定**：到终点的距离阈值。同时要求剩余弧长 ≤ max(0.40, 2×该值) | **必须 ≤ `terminal_align_enter_dist`（0.50）** |
| `enable_terminal_heading_align` | `true` | 脚本 | 到点后是否原地转身对齐目标朝向 | true |
| `terminal_heading_tol` | 0.26 rad ≈ 14.9° | 脚本 | 航向对齐容差 | 保持 |
| `terminal_heading_kp` | 1.2 | 代码 | 对齐的 P 增益 | 保持 |
| `terminal_heading_deadband` | 0.12 rad/s | 代码 | 死区补偿（克服地面静摩擦） | 保持 |
| `terminal_heading_min_vel` | 0.15 rad/s | 代码 | 最小角速度 | 保持 |
| `terminal_heading_max_vel` | 0.45 rad/s | 代码 | 最大角速度 | 0.45 |
| `terminal_heading_timeout` | 5.0 s | 代码 | 转身超时看门狗 | 保持 |
| `terminal_heading_stable_frames` | 2 | 代码 | 连续几帧进入容差才算收敛 | 保持 |
| `terminal_align_enter_dist` | **0.50** | 脚本 | 距离 ≤ 它才允许进入转身 | 0.50 |
| `terminal_align_abort_dist` | **0.60** | 脚本 | 转身中距离 > 它（扣除旋转漂移后）则放弃 | 0.60 |
| `terminal_align_lever_arm` | 0.55 | 脚本 | **新增**：里程计原点相对旋转中心的杠杆臂，用于扣除原地旋转造成的假位移 | 实测标定 |
| ~~`terminal_heading_drift_tol`~~ | 0.40 | 代码 | **已失效**：新守卫改用 `terminal_align_abort_dist`，这个参数只被读取、不再参与判断 | 可删 |

**杠杆臂标定方法**：遥控让机器人原地转 180°（不前后移动），记录起止位姿，`lever_arm ≈ 位移 / 2`。上一轮实测中位 0.55 m（n=21）。

---

## D. 安全与状态机

| 参数 | 当前生效 | 来源 | 作用 | 可测值 |
|---|---:|---|---|---|
| `report_failure` | `true` | 脚本 | 停滞时置 `failed` 并停车（关掉则只打印） | true |
| `blocked_timeout` | 4.0 s | 代码 | 进度 < 0.1 m 且速度 < 0.1 m/s 持续多久判停滞 | 4.0 |
| `blocked_dist_threshold` | 0.1 m | 代码 | 停滞判定的进度增量阈值 | 保持 |
| `blocked_vel_threshold` | 0.1 m/s | 代码 | 停滞判定的速度阈值 | 保持 |
| `path_start_max_dist` | 2.5 m | 代码 | 路径起点离机器人超过它就告警 | 保持 |
| `camera_timeout` | 2.0 s | 代码 | 深度中断多久判相机丢失 | 保持 |
| `goal_reached_timeout` | 1.0 s | 代码 | `goal_reached` 保持多久后回到 `idle` | 保持 |

---

## E. 感知与话题

| 参数 | 当前生效 | 来源 | 作用 |
|---|---|---|---|
| `depth_topic` | `/camera/camera/depth/image_rect_raw`（realsense）/ `/zed/zed_node/depth/depth_registered`（zed） | 脚本 | 策略输入深度图 |
| `min_depth` / `max_depth` | 0.15 / 10.0（realsense）、0.25 / 10.0（zed） | 脚本 | 深度裁剪范围 |
| `odom_topic` | `/lio/robo/odom` | 代码 | 里程计。**但 `map` 帧跟踪优先用 `/loc/robo/odom`** |
| `cmd_vel_topic` | `/cmd_vel` | 代码 | 下发速度 |
| `path_topic` | `/global_path` | 代码 | 全局路径 |
| `path_frame_id` | `''` | 代码 | 强制指定路径坐标系，空 = 用消息自带的 |
| `status_topic` | `/local_planner/status` | 代码 | 状态上报 |
| `base_frame_ids` | `base,base_link,base_footprint,body,imu` | 代码 | TF 里机器人坐标系的候选名 |

---

## F. 参数之间的耦合（改之前必须知道）

1. **`lookahead_min` ≥ 控制器落后量**
   离线比对显示控制器取点用的参考位置比记录位姿落后约 0.3–0.9 m（来源尚未用 `scripts/analyze_ctrl_latency.py` 定死）。`lookahead_min` 一旦小于这个亏损，carrot 会落到车脚下，策略就减速。所以 `lookahead_min` 不是"越小越稳"。

2. **`corner_switch_margin` ≥ 收缩所需的行程**
   `corner_dist` 下 L 的下降速率 ≈ 车速（因为 `d_corner` 以车速减少），`slew_rate_decrease=2.5` 完全够用，所以 margin 就是纯粹的几何提前量。margin 越大，carrot 停得离拐角越远，越安全但越早减速。

3. **`slew_rate_increase` 与 `lookahead_max` 的乘积**
   从 `lookahead_min` 恢复满程需要 `(max-min)/slew_inc` 秒。0.8 m/s → 2 s（约 0.8 m 行程）；2.0 → 0.8 s。把 `lookahead_max` 提到 2.5 同时不提高 `slew_inc`，恢复时间会跟着变长。

4. **`segment_arrive_threshold` ≤ `terminal_align_enter_dist`**
   否则一到点就直接完成、永远不转身。启动时会打印告警。

5. **`turn_angle_*` / `adaptive_window_dist` 只在 `switch_mode=curvature|hybrid` 下有效**
   切到 `corner_dist` 后这三个参数不起作用（`d_corner` 用的是 `corner_switch_*`）。

6. **`clearance` 相关参数（`passage_latch_dist` / `door_speed_boost`）在 `curvature_only` 下全部无效**

---

## G. 建议测试数值（按优先级）

> 2026-11-19 起默认值已固定为 `--min 0.4 --switch-mode corner_dist`，所以下面的命令里**默认项可以省略**，要测别的规则才需要显式覆盖。

### G1 无代码风险（只改启动参数）

| 轮次 | 命令片段 | 验证什么 |
|---|---|---|
| R1 | 直接 `./start_nav.sh --curv-only` | 当前默认组合的基线 + 拿到遥测 |
| R2 | `--switch-mode curvature --slew-inc 2.0 --turn-low 40` | 旧规则下的恢复速率+缓弯误判，作为对照 |
| R6 | `--max 2.5`（其余默认） | 平地最长前瞻是否够避障 |

### G2 新规则参数微调

| 轮次 | 命令片段 | 验证什么 |
|---|---|---|
| R3 | 默认（`corner_dist` + thresh 25 + margin 0.25） | 主推配置，同 R1 |
| R4 | `--corner-thresh 35` | 25–35° 缓弯算不算拐角 |
| R5 | `--corner-margin 0.45` | carrot 离拐角更远，治楼梯转角蹭栏杆 |
| R7 | `--switch-mode hybrid` | 兜底：取新旧规则较小值，最保守 |

### G3 楼梯侧（如果蹭栏杆仍在）

| 参数 | 试值 | 说明 |
|---|---|---|
| `stair_side_min_clamp` | 0.50 → 0.60 | 扶手侧钳位更保守 |
| `stair_min_forward_vel` | 0.0 → 0.15 | 下楼保底推进，防止策略在台阶边缘犹疑 |

### G4 到点转身（已改完，待现场确认）

| 参数 | 试值 | 说明 |
|---|---|---|
| `terminal_align_lever_arm` | 0.55 → 实测值 | 原地转 180° 标定 |
| `terminal_align_enter_dist` | 0.50 | 要与 `--arrive-thresh` 配合，`segment_arrive_threshold`（0.45）需 ≤ 它 |

---

## H. 不要动的参数

| 参数 | 原因 |
|---|---|
| `lookahead_min = 0.7` | 现场实测：窄门通过率明显变差、更容易蹭墙 |
| `adaptive_mode = both/clearance_only` | 窄走廊双侧墙壁会被误判成窄门，前瞻被锁死在 min |
| `enable_stair_remapping = true` | 会造成策略输入分布漂移，偏航摆动显著增大（见 `DEPTH_FILL_MODE_STUTTER_ANALYSIS_20261117.md`） |
| `turn_angle_high_deg` | 只影响"多久收到最短"，不是当前瓶颈；且 `corner_dist` 下已失效 |
| `slew_rate_decrease` | 2.5 m/s 已经足够快，0.64 s 走完满程 |
| `segment_arrive_threshold > 0.50` | 超过 `terminal_align_enter_dist` 会让终点转身逻辑直接跳过 |

> `switch_mode` 本身不是"不能动"，但**改回 `curvature` 时要把 `--min`、`--turn-low`、`--slew-inc` 一起看**：现场效果最好的组合是 `0.4 + corner_dist`，单独改一个往往会让另一处退化。

# 前瞻切换（长前瞻 ⇄ 短前瞻）参数测试方案

日期：2026-11-18
关联：[STAGE0_LOOKAHEAD_REPLAY_20261117.md](STAGE0_LOOKAHEAD_REPLAY_20261117.md)

---

## 0. 要解决的失效模式

短前瞻（0.40 m）在转弯、上下楼梯、窄门已经被现场验证有效，这部分不再动。
现在要解决的是**另一侧**：平地上前瞻恢复不到长距离，导致避障视野不足。

失效模式拆成四个，每个对应一组参数：

| # | 失效模式 | 现象 | 对应参数 |
|---|---|---|---|
| F1 | **恢复太慢** | 出弯/出窄门后，前瞻要在 1.5–2.5 m 行程后才回到长距离，这段路一直"近视" | `slew_rate_increase` |
| F2 | **误判为拐角** | 缓弯或直线折线抖动被当成转弯，平地上前瞻被压短 | `turn_angle_low_deg`（旧规则）/ `corner_switch_thresh_deg`（新规则） |
| F3 | **收缩太晚** | 拐角前前瞻才收，carrot 切内角；楼梯转角蹭栏杆就是这一类 | `corner_switch_margin` / `adaptive_window_dist` |
| F4 | **平地最长前瞻不够** | 开阔段 carrot 只有 2.0 m，旁侧障碍进入视野太晚 | `lookahead_max` |

---

## 1. 参数全景：哪些测、哪些不测

### 1.1 主试参数（本次要测）

| 参数 | 默认 | 取值 | 解决 | 代码依赖 |
|---|---:|---|---|---|
| `switch_mode` | `corner_dist` | `corner_dist` / `curvature` / `hybrid` | F2 F3 | **新增** |
| `slew_rate_increase` | 0.8 | 0.8 / 2.0 | F1 | 已有 |
| `turn_angle_low_deg` | 20 | 20 / 40 | F2（旧规则） | 已有 |
| `corner_switch_thresh_deg` | 25 | 25 / 35 | F2（新规则） | **新增** |
| `corner_switch_margin` | 0.25 | 0.25 / 0.45 | F3 | **新增** |
| `lookahead_max` | 2.0 | 2.0 / 2.5 | F4 | 已有 |

### 1.2 不动参数（已定论，改了只会引入混淆）

| 参数 | 值 | 理由 |
|---|---:|---|
| `lookahead_min` | **0.40** | 现场已确认 0.4 转弯/楼梯/窄门都够；0.7 会让窄门更差 |
| `stair_lookahead` / `stair_up_lookahead` | 0.40 / 0.45 | 楼梯段独立锁存，与切换逻辑无关 |
| `slew_rate_decrease` | 2.5 | 收缩已经够快（2.0→0.4 只需 0.64 s），不是瓶颈 |
| `turn_angle_high_deg` | 50 | 只影响"多久收到最短"，不是本次问题 |
| `adaptive_window_dist` | 1.6 | 只在 `curvature` 模式下有效；两小时内的对比不要动它 |
| `segment_arrive_threshold` | 0.20 | 与到点转身逻辑绑定，见 Stage 0 §12 |

### 1.3 新规则 `corner_dist` 是什么

```
d_corner(s) = 沿路径从 s 往前走，路径航向第一次偏离 s 处航向 >= corner_switch_thresh_deg 的弧长
L(s)        = clamp(d_corner(s) - corner_switch_margin, lookahead_min, lookahead_max)
```

即"carrot 始终停在拐角前 `margin` 米处"。直线段上 `d_corner` 达到上限 6 m，L 直接顶到 `lookahead_max`；进弯时 L 随距离线性下降，不会像曲率规则那样"到窗口边缘突然塌到最短"。

---

## 2. 离线证据（为什么是这几个参数）

用三段已有数据回放（位姿滞后 τ=1.2 s，与 Stage 0 一致），只统计**任务段且机器人在动**的帧：

`flat_L_p10` = 判定为直道（`d_corner ≥ 2 m`）的帧里，前瞻距离的 10 分位。它是"最坏情况下机器人还能看多远"，避障最关心的就是这个数。

| 规则 | flat_L_p10（3 段） | 直道仍 <1 m 的帧占比 |
|---|---|---|
| `curvature`（改造前的旧规则） | 0.81 / **0.40** / 1.02 | 20% / 16% / 9% |
| `curvature` + slew 4.0 | 1.25 / **0.40** / 1.50 | 5% / 16% / 0% |
| `curvature` + turn_low 40 + slew 2.0 | 1.48 / **0.40** / 1.96 | 5% / 16% / 5% |
| `corner_dist` + slew 2.0（现默认族） | 1.48 / **1.66** / 1.44 | **2% / 2% / 2%** |

结论：

1. **恢复速率（F1）是真问题**：改造前的曲率规则在直道上最坏只看到 0.40 m，等于全程近视。
2. **只调 slew 不够**：第二段数据无论 slew 提到多少，仍有 16% 的直道帧前瞻 <1 m —— 那是规则本身把它们判成拐角了（F2）。
3. **`corner_dist` 在最坏情况下显著更稳**：三段数据的 `flat_L_p10` 都 ≥1.44 m，直道短前瞻只剩 2%。
4. 但**离线最优不等于现场有效**（`lookahead_min=0.7` 就是这样翻车的），所以下面仍然把"旧规则+调参"和"新规则"并行测。

复现命令：

```bash
python3 scripts/analyze_lookahead_switching.py \
  --session door_diagnostics/session_20261110_144216_realsense \
  --session door_diagnostics/session_20261110_145109_realsense \
  --session door_diagnostics/session_20261118_102759_realsense \
  --pose-lag-sec 1.2 --out door_diagnostics/analysis_switching
```

---

## 3. 实验矩阵

> **2026-11-19 更新**：现场实测效果最好的组合已固定为 `start_nav.sh` 的默认值 —— `lookahead_min=0.4`、`switch_mode=corner_dist`、`corner_thresh=25`、`corner_margin=0.25`、`slew_inc=0.8`。所以直接 `./start_nav.sh --curv-only` 就是基线，下表里凡是"默认"的行都不用再加参数。

路线、起点、位姿、终点与之前完全一致（同一 `/goal_3d`，同一楼梯+窄门路线），每次跑完自动停录。
每轮只改**一个**参数族，便于归因。

| 轮次 | `switch_mode` | `slew_inc` | `turn_low` | `corner_thresh` | `corner_margin` | `max` | 目的 |
|---|---|---:|---:|---:|---:|---:|---|
| **R1** | corner_dist（默认） | 0.8 | — | 25 | 0.25 | 2.0 | **基线**：当前默认组合，顺带拿到第一份有前瞻遥测的数据 |
| **R2** | curvature | 2.0 | 40 | — | — | 2.0 | 旧规则对照：同样想解决"恢复慢 + 缓弯误判"，用来确认新规则是否真的更好 |
| **R3** | corner_dist | **2.0** | — | 25 | 0.25 | 2.0 | F1：只提恢复速率，规则不变 |
| **R4** | corner_dist | 0.8 | — | **35** | 0.25 | 2.0 | F2：25–35° 的缓弯到底算不算拐角 |
| **R5** | corner_dist | 0.8 | — | 25 | **0.45** | 2.0 | F3：把 carrot 停得离拐角更远，专治楼梯转角蹭栏杆 |
| **R6** | R1–R5 胜者 | — | — | — | — | **2.5** | F4：平地最长前瞻再放 0.5 m |
| **R7** | R1–R6 胜者 | 复测 | | | | | 重复性确认 |
| **R8** | R1–R6 胜者 | 复测 | | | | | 重复性确认 |

估计 8 轮 ×（路线约 4–5 min + 复位 2 min）≈ **50 min**。

若 R1 与历史数据相比出现明显回归（转角蹭栏杆变多、或到点不转身），**立即停测**，先排查代码，不要往下走。

### 3.1 对应启动命令

```bash
./start_nav.sh --curv-only --stair-lookahead 0.4 --arrive-thresh 0.20 --report-failure
```

R1 就用上面这条——2026-11-19 起 `--min 0.4` 和 `--switch-mode corner_dist` 已经是默认值，不用再写。

```bash
./start_nav.sh --curv-only --stair-lookahead 0.4 --arrive-thresh 0.20 --report-failure --switch-mode curvature --slew-inc 2.0 --turn-low 40
```

```bash
./start_nav.sh --curv-only --stair-lookahead 0.4 --arrive-thresh 0.20 --report-failure --slew-inc 2.0
```

```bash
./start_nav.sh --curv-only --stair-lookahead 0.4 --arrive-thresh 0.20 --report-failure --corner-thresh 35
```

```bash
./start_nav.sh --curv-only --stair-lookahead 0.4 --arrive-thresh 0.20 --report-failure --corner-margin 0.45
```

```bash
./start_nav.sh --curv-only --stair-lookahead 0.4 --arrive-thresh 0.20 --report-failure --max 2.5
```

---

## 4. 每轮的判定标准

### 4.1 硬性（一票否决）

| 检查项 | 通过条件 |
|---|---|
| 楼梯转角 | 前/后轮不蹭栏杆（同一位置，人工观察 + 录像） |
| 窄门 | 不撞门框、不需要按 A 脱困 |
| 到点 | 正常 `goal_reached`，到点转身按新逻辑执行 |
| 全程 | 不触发 `blocked`/`failed` |

### 4.2 定量（离线回放，τ=1.2 s）

| 指标 | 目标 | 含义 |
|---|---:|---|
| `flat_L_p10` | **≥ 1.40 m** | 直道最坏情况下的前瞻距离（避障核心） |
| `flat<1m` | **≤ 5%** | 直道上仍处于近视状态的帧占比 |
| `lead_m` | **≥ 0.50 m** | 拐角前多远开始收缩（不能比现网更晚） |
| `recover_m` | **≤ 1.00 m** | 出弯后多少行程回到 1.5 m 以上 |

四项目标同时满足才算通过；`lead_m` 是防退化的护栏——为了拉长前瞻而牺牲收缩提前量（切内角、蹭栏杆）不可接受。

### 4.3 遥测一致性（新，必查）

每轮跑完，把记录里的 `adaptive_lookahead` 与离线用**同一规则**回放出来的 L 做对比：

| 检查项 | 目标 |
|---|---|
| MAE（记录 vs 回放） | < 0.10 m |
| 相关系数 | > 0.95 |

对不上说明部署的参数没生效（例如 `-p` 没传进去、或走了别的分支），该轮数据作废重录。

---

## 5. 数据采集说明

**此前三段数据不能用来评价切换**：`trajectory.csv` 里的 `adaptive_lookahead` / `l_curv` / `l_target` 全是空值——logger 一直在订阅 `/nav/adaptive_debug`，但控制器从来没有发布过这个话题。

本次已在 `rl_nav_controller_adaptive.py` 中补上发布（每帧 5 Hz，JSON 内容即 tracker 的 `debug_info`，新增 `switch_mode` / `d_corner` 两个字段）。从 R1 起每次录制都会自动带上：

| 列 | 含义 |
|---|---|
| `adaptive_lookahead` | slew 之后的实际前瞻（carrot 距离） |
| `adaptive_l_target` | 本帧目标值（切换规则的原始输出） |
| `adaptive_l_curv` | 曲率规则输出；`corner_dist` 模式下即 `clamp(d_corner - margin)` |
| `adaptive_d_corner` | 到下一个拐角的弧长（本次新增列） |
| `adaptive_switch_mode` | 实际生效的切换规则（本次新增列，用来确认 `-p` 真的传进去了） |
| `adaptive_turn_deg` | 曲率规则的有效转角（仅 `curvature`/`hybrid` 模式有意义） |

开场先做一次 10 s 直行，确认 `adaptive_lookahead ≈ 2.0`、`adaptive_d_corner ≥ 6.0`、`adaptive_switch_mode` 等于当轮设定的规则；若一上来就是 0.4 或规则名不对，说明参数没传进去。

---

## 6. 需要同步到机器狗的文件

| 文件 | 改动 |
|---|---|
| `go2w_nav_ros2_ws/src/rl_nav_controller/rl_nav_controller/adaptive_path_tracker.py` | 新增 `switch_mode` / `corner_switch_*`，`d_corner` 表与切换实现 |
| `go2w_nav_ros2_ws/src/rl_nav_controller/rl_nav_controller/rl_nav_controller_adaptive.py` | 声明/读取新参数，发布 `/nav/adaptive_debug`，相机丢失文案改为通用表述 |
| `go2w_nav_ros2_ws/src/rl_nav_controller/rl_nav_controller/rl_nav_controller_stairs.py` | 调用共享的 `_publish_adaptive_debug` |
| `go2w_nav_ros2_ws/start_nav.sh` | `--switch-mode` `--corner-thresh` `--corner-margin` `--slew-inc` `--slew-dec`；默认值改为 `min 0.4` + `corner_dist` |
| `go2w_nav_ros2_ws/door_diagnostic_logger.py` | 新增 `adaptive_switch_mode` / `adaptive_d_corner` 两列 |

```bash
cd ~/sru-robot-deployment
rsync -avz go2w_nav_ros2_ws/src/rl_nav_controller/rl_nav_controller/adaptive_path_tracker.py \
          go2w_nav_ros2_ws/src/rl_nav_controller/rl_nav_controller/rl_nav_controller_adaptive.py \
          go2w_nav_ros2_ws/src/rl_nav_controller/rl_nav_controller/rl_nav_controller_stairs.py \
          go2w_nav_ros2_ws/start_nav.sh \
          go2w_nav_ros2_ws/door_diagnostic_logger.py \
          agile@<机器人IP>:~/go2w_nav_ros2_ws/
```

纯 Python 源码，**不用 colcon build**，重启 `start_nav.sh` 即生效。

---

## 7. 风险与回退

| 风险 | 表现 | 处理 |
|---|---|---|
| `corner_dist` 把缓弯当直道 | 在 25–35° 弯上切内角、蹭障碍 | R4 就是测这个；失败则把 `corner_thresh` 调回 25 或改用 `hybrid` |
| `slew_inc` 过大 | 出弯瞬间 L 从 0.4 跳到 2.0，carrot 甩到远侧，机器人回头 | 上限取 2.0；R2/R3 已经在验 |
| `lookahead_max=2.5` | 终点附近 carrot 被钳到终点、或跳出台阶 | 只在 R6 试，失败回 2.0 |
| 新代码引入回归 | R1 表现不如历史基线 | 立即回退到上一版 `adaptive_path_tracker.py`，先把 `switch_mode` 之外的部分单独验证 |

回退方式：`switch_mode=curvature` + `slew_inc=0.8` + `turn_low=20` 就是完整旧行为，参数层面即可回退，无需改代码。

# 导航训练设计详解（Navigation Training Deep Dive）

> 姊妹文档：`SRU_NAVIGATION_EXPLAINED.md`（架构总览）。本文聚焦**导航任务本身的设计与目的**：
> 任务定义、目标采样、观测/动作空间、奖励结构、终止判定、域随机化、课程学习、算法超参。
> 所有参数均已对照 `sru-navigation-sim` / `sru-navigation-learning` 当前代码核实（2026-10）。

---

## 1. 训练目的：任务的形式化定义

### 1.1 在学什么

训练对象只有一个：**高层导航策略** π(a_t | o_t, h_{t−1})。

- 输入 o_t：第一视角深度图（VAE 特征）+ 本体感知 + 目标点的**机体系方向/对数距离**；
- 输出 a_t = [v_x, v_y, ω]：基座速度指令，5 Hz，高斯采样 N(μ, σ²)；
- **无地图、无全局定位、无重规划**：目标在 episode 内固定（`resampling_time_range=(1e9, 1e9)`），
  策略必须靠 SRU 隐状态记住"走过的路/去过的地方"，才能完成长程导航——这正是论文核心。

### 1.2 训练要塑造的四种能力

| 能力 | 由什么设计驱动 |
|---|---|
| 长程空间记忆（60 s / 300 步不迷路） | 固定目标 + 60 s episode + SRU 单元 + γ=0.999 长视界 |
| 未知环境避障与路径选择 | 迷宫/楼梯/坑洞地形混合 + 稀疏化奖励（见 §5.2） |
| Sim-to-Real 鲁棒性（zero-shot 上真机） | 全量域随机化（§6）+ 分层架构（locomotion 冻结） |
| 平滑、低能耗运动 | lateral/action_rate/joint_acc 惩罚项（§5.1） |

### 1.3 MDP 要素速览

| 要素 | 内容 |
|---|---|
| 观测 o_t | actor：2576 维（§3.1）；critic：5713 维特权观测（§3.2） |
| 动作 a_t | [v_x, v_y, ω] 高斯采样，tanh 后 ×[1.5, 1, 1] 再低通滤波 |
| 步长 | 高层 1 步 = 0.2 s（decimation 40 × dt 0.005） |
| episode | 60 s = 300 高层步，固定目标 |
| 成功 | 距目标 < 0.5 m（提前到达或超时判定） |
| 失败 | 非法接触 >1 N / 俯仰或侧倾 >40° / 坠落 z<−2 m / 60 s 超时未达 |

---

## 2. 环境与 Episode 设计

### 2.1 地形：180 块程序化迷宫（`terrains/maze_config.py`）

`MAZE_TERRAIN_CFG`：6 行 × 30 列 = **180 个 30 m × 30 m 地形**，高度场分辨率 0.1 m
（`horizontal_scale=0.1`, `vertical_scale=0.005`），外圈 30 m 边界墙。

| 子地形 | 比例 | 特征 |
|---|---|---|
| `maze` | 30% | 15×15 网格、2 m/格，开放概率 0.9，随机墙比 0.5，**迷宫内加楼梯** |
| `non_maze`（随机障碍） | 20% | 无规则墙体，随机障碍散布 |
| `stairs` | 30% | 平地 + 楼梯（考察爬升/下降能力） |
| `pits` | 20% | 带坑槽（trough）地形，考察边缘感知与防坠落 |

B2W 配置（`config/b2w/navigation_env_cfg.py`）：

- `curriculum = False`：**训练中不切换难度**；
- `max_init_terrain_level = 10`、`difficulty_range = [0.5, 1.0]`：所有环境从第 10 级开始，
  整个训练过程地形分布固定——鲁棒性完全靠"混合地形 + 域随机化"，而非课程递进。

### 2.2 出生点与目标采样（`mdp/navigation/goal_commands.py`）

地形生成时预计算三张掩码，采样器 `PositionSampler` 直接查表：

- **目标**：从 `valid_mask`（带安全边距的有效格）均匀采样；
  **平台位置在目标表中重复 ×10**（`platform_repeat_count=10`）→ 目标以约 10 倍概率落在
  高处平台上，隐式鼓励"爬上楼梯/平台顶部"的行为（静态采样课程）；
- **出生点**：从 `spawn_mask`（更大边距，考虑机器人本体尺寸与随机朝向）均匀采样，
  离地 +0.05 m 防穿插；
- **目标高度** = 该格地形高度 + U(0.2, 0.8) m → 目标是悬浮在空中的 3D 点（影响 §5.2 的 soft 奖励）；
- **出生点与目标之间无最小距离约束**：初始距离从几米到 ~40 m 不等，
  策略必须通过 `log(1+d)` 压缩的距离观测自适应远近；
- **episode 内不重采样**（resampling_time_range=(1e9,1e9)）：目标一旦确定直到 episode 结束。

### 2.3 目标观测的表示（为什么是 4 维）

```
target_position = [dir_x, dir_y, dir_z, log(1 + d)]   （机体系，d = ||goal − base||）
```

- **单位方向向量**：给出"往哪走"，与距离解耦；
- **log(1+d)**：把 0.5 m ~ 40 m 的动态范围压到 [~0.4, ~3.7]，远近都可分辨、不爆尺度；
- **噪声**（`DeltaTransformationNoiseCfg`）：每步以 **p=0.1** 概率对目标做随机旋转 0.1 rad /
  平移 0.5 m → 策略不能把目标当"完美 GPS"，必须容忍目标漂移（sim-to-real：真机定位/目标指令有误差）；
- **延迟**：0–2 步随机（§3.3）。

---

## 3. 观测空间（Actor vs Critic）

### 3.1 Actor（policy 组，共 2576 维）——部署时唯一使用的输入

| 项 | 维度 | 噪声 | 延迟 | 说明 |
|---|---|---|---|---|
| `base_lin_vel` | 3 | U(−0.2, 0.2) | 0–2 步 | 基座线速度（机体系） |
| `base_ang_vel` | 3 | U(−0.1, 0.1) | 0–2 步 | 基座角速度 |
| `projected_gravity` | 3 | U(−0.1, 0.1) | 0–2 步 | 投影重力（姿态） |
| `last_action` | 3 | — | — | 上一步高层动作（执行链处理前） |
| `target_position` | 4 | Δ噪声 p=0.1 | 0–2 步 | 见 §2.3 |
| `depth_image_noisy_delayed` | 64×5×8 = 2560 | VAE + 立体噪声仿真 | 0–2 步 | ZedX 射线相机 192×120→**64×40**，最大距离 11 m；冻结 VAE 编码为 64×5×8 特征 |

### 3.2 Critic（特权观测，共 5713 维）——只用于训练价值函数

| 项 | 维度 | 与 actor 的差异 |
|---|---|---|
| `base_lin_vel` / `base_ang_vel` / `projected_gravity` | 9 | **无噪声** |
| `last_action` | 3 | — |
| `target_position` | 4 | **无 Δ噪声**（完美目标） |
| `time_normalized` | 1 | t/T，让 critic 感知"deadline 还剩多少" |
| `height_scan_critic` | 64×7×7 = 3136 | **actor 没有**：10 m×10 m 高度图（0.2 m 分辨率）编码特征 |
| `depth_image_prefect` | 2560 | **完美深度**（无 VAE 噪声、无延迟） |

这是典型的**非对称 actor-critic**：critic 用特权信息把价值估计做准，actor 只依赖真机可得的传感器；
部署时整个 critic 被丢弃。

### 3.3 观测延迟机制（`mdp/delay_manager.py`）

每个环境在 reset 时为每类通道独立采样随机 lag ∈ {0, 1, 2} 步（`max_delay_* = 2`，共 0.4 s），
episode 内保持不变。作用：模拟真实传感器/通信延迟，**防止策略依赖"完美同步"的观测组合**
（否则 sim-to-real 会因几十毫秒延迟而崩溃）。

---

## 4. 动作空间与执行链

```
高斯采样 a ~ N(μ, σ²)            init_noise_std = 1.0，σ 可学习（log_std 参数）
   → tanh → [-1, 1]
   → × policy_scaling [1.5, 1.0, 1.0]     （B2W：前进上限放大到 1.5 m/s）
   → 低通滤波  v_f = α·v_prev + (1−α)·a    α ~ U(0.1, 0.6)，每环境每维度独立（B2W；基类为 0.4–0.9）
   → 50 Hz 冻结 locomotion 策略 → 关节动作
```

额外的**动作尺度随机化**（reset 时，`randomize_action_scale`）：

| 维度 | 增益范围 | 偏置 | 含义 |
|---|---|---|---|
| v_x | 0.8 – 1.2 | ±0.1 | 执行器增益失配 |
| v_y | **0.6 – 1.0** | ±0.2 | 非对称：B2W 横向能力弱，真实中侧移更不可靠 |
| ω | 0.8 – 1.2 | ±0.1 | 转向增益失配 |

低通滤波 α 的随机化同时模拟"执行器响应快慢不一"。两者都是 zero-shot 迁移的关键。

---

## 5. 奖励函数设计（训练目的的核心）

### 5.1 全部奖励项（`navigation_env_cfg.py: RewardsCfg`，B2W/AoW-D 共用）

| 项 | 权重 | 函数 | 作用 |
|---|---|---|---|
| `reach_goal_xy_tight` | **+1.5** | 1/(1+(d_xy/0.25)²)/T_r(0.1) → d→0 时 **最高 +15** | 目标附近强吸引，精确到位 |
| `reach_goal_xy_soft` | **+0.25** | 1/(1+(d_3d/2.5)²)/T_r(1.0)，**flat=False（3D 误差）** | 长程平滑 shaping；因目标是悬浮 3D 点，爬上平台也能得分 |
| `lateral_movement` | −0.1 | \|v_y,机体\| | 抑制侧滑/横移（B2W 横向能力弱） |
| `action_rate_l1` | −0.1 | Σ\|a_t − a_{t−1}\| | 速度指令平滑，避免抖动 |
| `rot_movement` | −1e-5 | ‖ω_root‖ | 轻微抑制原地乱转 |
| `joint_acc_l2_joint` | −1e-7 | Σ‖关节加速度‖²（腿+轮） | 低层执行平滑，保护电机 |
| `episode_termination` | **−50** | `is_terminated`：**仅真终止**（接触/大角度/坠落），超时与到达不计 | 摔倒 = 灾难性失败，量级压倒所有 shaping |
| `backward_movement_penalty` | −0.0 | clamp(−v_x, 0, 1) | **当前配置关闭**（保留课程钩子，§8） |

### 5.2 目标奖励的门控机制（关键细节，容易误读）

两个 reach_goal 项都乘了一个掩码：

```
mask = (t > T − 20)          # episode 最后 4 秒（required_steps_at_goal = 4 s / 0.2 s = 20 步）
       OR (time_at_goal > 0) # 已到达过目标
       OR (rand < 0.01)      # 每步 1% 随机打开 → 每个 episode 期望 ~3 次
```

即：**距离型奖励只在"最后 4 秒 + 稀疏随机时刻"生效**。设计意图：

1. **deadline 语义**：任务本质是"60 秒内到达"。主成功信号不是全程稠密 shaping，而是
   - 提前到达 <0.5 m → `early_termination`（time_out=True）→ **价值函数 bootstrap**，无 −50 惩罚；
   - 超时前在最后阶段靠近目标 → 拿到 dense 奖励；
   这样策略学到的是"持续朝目标推进直到截止"，而不是"中途接近目标就停着吃奖励"。
2. **稀疏随机注入（1%/步）**：在全 horizon 上散布少量距离信号，经 critic bootstrap + RNN 记忆
   向后传播信用——配合 SRU 的长程建模，让早期步骤也能获得梯度。
3. **tight 项在 d<1 m 内急剧上升**（d=0 时 +15），负责"最后一步精确停靠"；
   soft 项 ≤0.25、尺度 2.5 m，负责"大方向别跑偏"。

### 5.3 终止与成功/失败判定（`mdp/terminations.py`）

| 终止项 | 条件 | 记为 | bootstrap | 奖励后果 |
|---|---|---|---|---|
| `time_out` | 60 s 到达 | 超时时刻 <0.5 m → 成功，否则失败 | ✔（timeout） | 无 −50 |
| `early_termination`（at_goal） | 任意时刻 <0.5 m | **成功** | ✔（time_out=True） | 无 −50 |
| `base_contact` | base/hip/thigh 接触力 >1 N | 失败（绊倒/倒地） | ✘ | **−50** |
| `large_pitch_angle` | \|pitch\| 或 \|roll\| >40° | 失败（翻倒） | ✘ | **−50** |
| `terrain_fall` | z < −2 m | 失败（坠坑/坠落） | ✘ | **−50** |

成功统计：每次终止写入 `goal_reached_buffer`，`SuccessRateTracker` 维护每环境滚动 10 episode
的滑动成功率 → tensorboard 中的 `success_rate`。

### 5.4 奖励结构的设计意图（为什么这样配）

- **"站着不动"是早期安全下界**：超时不罚、到达才奖，所以乱动有 −50 风险而不动只有轻微 shaping
  惩罚 → 策略先学会"别摔"，再被稀疏目标奖励拉向"去走"。这是长程导航任务常见的保守起步设计。
- **没有速度跟踪项、没有路径长度惩罚**：导航是"到达"任务而非"跟踪"任务；只罚浪费性的
  横移（lateral）与抖动（action_rate），给策略充分的机动自由度（允许绕路）。
- **−50 的量级选择**：约为 tight 奖励峰值（+15）的 3 倍多、shaping 项的数百倍，
  保证"摔倒一次"在 GAE 里主导该 episode 的信用分配。
- **entropy_coef=0.00375 + init_std=1.0**：前期保持探索；MDPO 的双策略互蒸馏进一步防止过早塌缩。

---

## 6. 域随机化全表（zero-shot sim-to-real 的基础）

| 事件 | 时机 | 参数 | 模拟什么 |
|---|---|---|---|
| `physics_material` | startup | 静摩擦 (0.8,1.2)、动摩擦 (0.7,1.0)、恢复系数 (0,0.1) | 地面材质差异 |
| `reset_base` | reset | yaw ~ U(−π, π) | 随机初始朝向 |
| `reset_robot_joints` | reset | 关节位置 ×U(0.5,1.5) | 初始姿态不确定性 |
| `randomize_camera_height` | reset | 高度 ±0.1 m，roll/pitch/yaw ±5° | 相机安装误差（B2W 挂载点 (0.387, 0, 0.28)，俯仰 20°） |
| `randomize_action_scale` | reset | 见 §4 | 执行器增益失配 |
| `reset_delay_buffer` | reset | 每通道 lag ~ U{0,1,2} 步 | 传感器延迟（§3.3） |
| `randomize_low_pass_filter_alpha` | reset | α ~ U(0.1, 0.6) 每维度（B2W） | 执行器响应快慢不一 |
| `push_robot` | 间隔 0.2–5 s | 水平速度 ±0.5 m/s | 外部扰动/碰撞恢复 |

叠加在观测层的随机化：本体感知均匀噪声（§3.1）、深度 VAE 立体噪声仿真、目标 Δ噪声（§2.3）。
**play 模式（`-Play-v0`）关闭全部 corruption 与 push**，用于评估"干净环境下的真实水平"。

---

## 7. 算法设计与超参

### 7.1 PPO vs MDPO（`config/b2w/agents/rsl_rl_cfg.py`）

| 超参 | PPO（基线） | MDPO（论文主算法） |
|---|---|---|
| 网络 | ActorCriticSRU ×1 | ActorCriticSRU ×**2**（参数独立） |
| γ | 0.995（有效视界 ≈200 步 ≈ **40 s**） | 0.999（≈1000 步 ≈ **200 s**，覆盖整个 60 s episode 且有余量） |
| λ / clip / value_clip | 0.95 / 0.2 / 0.2 | 同左 |
| entropy_coef / value_loss_coef | 0.00375 / 0.02 | 同左 |
| lr schedule | adaptive（desired_kl=0.01） | exponential，1e-3 → 1e-7 |
| num_steps_per_env | 16（=3.2 s rollout） | 同左 |
| epochs × minibatches | 5 × 4（轨迹级切分，回放时恢复 RNN 隐状态） | 同左 |
| max_iterations / save | 15000 / 每 500 | 同左；seed=60 |
| reward_shifting_value | 0.05（runner 层奖励平移，稳定价值目标） | 同左 |
| Muon 优化器 | — | 隐藏层权重用 Muon（Newton-Schulz 正交化动量），bias/其余 AdamW |

### 7.2 MDPO 代码级机制（`rsl_rl/algorithms/mdpo.py`）

1. **环境对半切分**：`indices_1 / indices_2` 把 num_envs 均分给两份策略；
   `act()` 中每个网络只对自己那半环境出动作，再按原索引拼回全量动作张量——
   两个策略在**同一物理世界**里各管一半机器人；
2. **互蒸馏**：每次 update，策略 1 的数据先过策略 2（teacher），算
   `KL(π₂‖π₁)`（对角高斯闭式解 `_kl_gaussian`），反向同理；
   **梯度同时穿过两个网络** → 互相拉近、互为正则，等价于双倍参数的隐式集成，
   提高探索多样性与样本效率；
3. **总损失**：`0.5 × [ (PPO₁ + 0.02·KL₂→₁) + (PPO₂ + 0.02·KL₁→₂) ]`（`distill_coef=0.02`）；
4. **TC-Dropout 0.2**：时序卷积 dropout，掩码每次 update 更新、mini-batch 回放时保持一致，
   防止 RNN 对单条轨迹过拟合。

### 7.3 网络结构（摘要，详见主文档 §6）

```
深度特征(64×5×8) + 本体感知/目标 → CrossAttentionFuseModule(4头, 3D位置编码)
→ LSTM_SRU(hidden 512, 空间变换门 tx=W·x, g←tanh(tx⊙g), 多项式精炼遗忘门)
→ TC-Dropout(0.2) → MLP[512,256,128](ELU) → μ；log_std 可学习
critic：独立 MLP 吃特权观测（§3.2）
```

---

## 8. 课程学习（Curriculum）现状

- **地形课程关闭**（`curriculum=False`，B2W 固定 level 10）：难度不随训练递进；
- `disable_backward_penalty_after_steps(500)`：钩子存在但反向惩罚权重本就是 −0.0 → **当前配置下为 no-op**；
- 实际存在的"静态课程"只有两处：**平台目标 ×10 采样偏置**（§2.2）与**混合地形分布**（§2.1）。

---

## 9. 训练时该看什么（指标与典型动态）

| 指标 | 含义 / 期望趋势 |
|---|---|
| `success_rate` | 滚动 10 episode 成功率；应从 ~0 缓慢爬升，后期 >50%（论文级结果） |
| `Episode_Reward/Time_out` vs `/Termination` | 成功(超时到达)与摔倒的比例变化——先学会"不摔"再学会"到达" |
| `velocity_toward_goal` / `closest_distance_to_goal` | 朝目标推进速度、单 episode 最近距离（即使失败也在缩小 = 在学） |
| MDPO `KL`（蒸馏项） | 先降后平稳；若持续为 0 → 两策略塌缩成同一份，MDPO 失效 |
| entropy | 缓慢下降即可，前期骤降 = 探索过早消失 |
| 视频（`--video --video_interval 2000`） | 早期：原地打转/后退；中期：朝目标走但绕路、撞墙恢复差；后期：稳定绕障直达 |

典型失败模式排查：长期成功率不涨且 `closest_distance` 不缩小 → 检查深度特征是否接入（VAE 权重路径）、
γ 是否被改小、目标观测的 Δ噪声是否过大。

---

## 10. 附录：本文引用的文件

| 内容 | 路径 |
|---|---|
| 基类环境配置（奖励/终止/事件/相机/观测组） | `sru-navigation-sim/isaaclab_nav_task/navigation/navigation_env_cfg.py` |
| B2W / AoW-D 覆盖项 | `.../navigation/config/b2w/navigation_env_cfg.py`、`.../aow_d/navigation_env_cfg.py` |
| RL 超参（PPO/MDPO） | `.../config/b2w/agents/rsl_rl_cfg.py` |
| 目标采样与成功统计 | `.../mdp/navigation/goal_commands.py` |
| 奖励 / 终止函数 | `.../mdp/rewards.py`、`.../mdp/terminations.py` |
| 观测延迟管理 | `.../mdp/delay_manager.py`、`.../mdp/observations.py` |
| 地形配置（180 地形） | `sru-navigation-sim/isaaclab_nav_task/terrains/maze_config.py` |
| MDPO / PPO 算法 | `sru-navigation-learning/rsl_rl/algorithms/mdpo.py`、`ppo.py` |

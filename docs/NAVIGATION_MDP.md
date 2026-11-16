# SRU 导航：系统架构与导航任务设计（Navigation MDP）

> 对应论文：*Spatially-enhanced recurrent memory for long-range mapless navigation via end-to-end reinforcement learning*（IJRR 2025）
> 本文档合并自原 `SRU_NAVIGATION_EXPLAINED.md`（架构总览）与 `NAVIGATION_TRAINING_DESIGN.md`（任务设计），基于 `sru-navigation-learning` / `sru-navigation-sim` 的实际代码整理。
> 训练实操（5-Phase 课程、超参矩阵、导出命令）见同目录 `go2w_nav_refactor_plan.md`；指标解读见 `TRAINING_MONITORING.md`。

---

## 1. 一句话总结

训练一个**高层导航策略**：它只看第一视角深度图 + 本体感知 + 目标点方向，输出速度指令 [vx, vy, ω]；
底层由一个**预训练、冻结的 locomotion 策略**把速度指令翻译成关节动作。
核心创新 SRU（空间增强循环单元）让 RNN 隐式记住“我在哪、我见过什么”，从而实现长距离无地图导航。

---

## 2. 总体架构

```
┌─────────────────────────────────────────────────────────────────────┐
│  Isaac Sim 物理仿真 (200 Hz)                                          │
│                                                                     │
│  迷宫地形(180个,6级难度) + B2W/AoW-D 机器人 + ZedX 深度相机            │
└───────────────────────────────┬─────────────────────────────────────┘
                                │ 每 40 步 (5 Hz)
                    ┌───────────▼────────────────┐
                    │  高层导航策略 (训练中)        │  ← 本仓库唯一在训练的东西
                    │  ActorCriticSRU             │
                    │  输入: 深度特征+本体感知+目标  │
                    │  输出: [vx, vy, ω] ~ N(μ,σ) │
                    └───────────┬────────────────┘
                                │ tanh→[-1,1] → ×policy_scaling → 低通滤波
                                │ 每 4 步 (50 Hz)
                    ┌───────────▼────────────────┐
                    │  低层 locomotion 策略        │  ← 预训练、冻结、JIT 加载
                    │  输入: 本体感知+速度指令      │     (policy_b2w_new_2.pt 等)
                    │  输出: 腿部关节位置目标       │
                    │        + 轮子角速度          │
                    └───────────┬────────────────┘
                                │ 每个物理步 (200 Hz)
                        关节位置/速度控制 → 机器人运动
```

**三个关键频率**（`isaaclab_nav_task/navigation/navigation_env_cfg.py`）：

| 层级 | 频率 | 实现 |
|---|---|---|
| 物理仿真 | 200 Hz | `sim.dt = 0.005` |
| 低层 locomotion | 50 Hz | `low_level_decimation = 4`（每 4 个物理步调用一次） |
| 高层导航策略 | 5 Hz | `decimation = (1/0.005)/PLANNING_FREQ = 40`，`PLANNING_FREQ = 5.0` |

每个 episode 最长 **60 秒**（300 个高层步）。

---

## 3. 什么在训练、什么是冻结的

| 组件 | 状态 | 说明 |
|---|---|---|
| 高层 ActorCriticSRU（actor + critic） | ✅ 训练中 | MDPO 模式下是**两份**相同结构、各自独立参数 |
| 低层 locomotion 策略 | ❄️ 冻结 | `torch.jit.load` 加载，只前向推理，不参与梯度 |
| VAE 深度编码器 | ❄️ 冻结 | 自监督预训练（见 `sru-depth-pretraining` 仓库），这里只做特征提取 + 噪声仿真 |

所以“复现训练”= 只训练高层导航策略；locomotion 和视觉编码器是随 sim 仓库一起发布的资产：
- locomotion：`isaaclab_nav_task/navigation/assets/data/Policies/locomotion/b2w/policy_b2w_new_2.pt`、`.../aow_d/policy_blind_3_1.pt`
- 深度编码器：`.../Policies/depth_encoder/vae_pretrain_new.pth`

---

## 4. Locomotion（机器狗行走）是怎么做的

### 4.1 分层控制原理

高层策略**不直接控制关节**。它输出的速度指令会进入低层策略的观测里，低层策略负责“怎么走”：

```
高层输出 [vx, vy, ω]          （"往哪走、多快"）
        │
        ├─ tanh 压缩到 [-1,1]（gaussian 分布模式）
        ├─ × policy_scaling   （B2W: [1.5, 1.0, 1.0]，即前进速度上限放大 1.5 m/s）
        └─ 低通滤波            filtered = α·prev + (1-α)·new
                               α 每个环境、每个维度独立随机（B2W: 0.1~0.6）
                               → 模拟执行器延迟/平滑，同时是域随机化
        │
        ▼
低层策略观测组 "low_level_policy"（7 项拼接，均带噪声）：
   base_lin_vel, base_ang_vel, projected_gravity,
   velocity_commands (↑上面的滤波后速度指令),
   joint_pos_rel, joint_vel_rel, last_low_level_action
        │
        ▼
低层策略输出 → 拆成两路执行：
   前 N 维: 腿部关节位置目标 = 默认姿态 + action × 0.5   （位置控制）
   后 M 维: 轮子角速度指令     = action × 5.0 (rad/s)    （速度控制）
```

代码位置：`isaaclab_nav_task/navigation/mdp/navigation/actions/navigation_se2_actions.py`（`PerceptiveNavigationSE2Action`）。

### 4.2 两种机器人平台

| | B2W（双足轮式） | AoW-D（Anymal on Wheels，轮式四足） |
|---|---|---|
| 腿部关节 | `.*hip_joint / .*thigh_joint / .*calf_joint`（2 腿 × 3 = 6 个位置控制） | `.*HAA / .*HFE / .*KFE`（4 腿 × 3 = 12 个位置控制） |
| 轮子 | `.*foot_joint` 速度控制 | `.*WHEEL` 速度控制 |
| locomotion 权重 | `policy_b2w_new_2.pt` | `policy_blind_3_1.pt` |
| 相机偏移 | base_link + (0.387, 0, 0.28) | base + (0.4761, 0.0035, 0.1055)，下俯 20° |

文件名里的 **blind** 表示低层策略是“盲走”的——只用本体感知、不看视觉。这是刻意为之：
纯本体感知的 locomotion 更鲁棒、与具体场景无关，可以冻结后跨地形复用；所有“看路”的智能都压在高一层。

### 4.3 Locomotion 策略从哪来

它不是本仓库训练的，而是预先用 RL 单独训练好的速度跟踪（velocity tracking）locomotion 策略
（输入速度指令 + 本体感知 → 输出关节动作），以 TorchScript 形式随仓库发布。
复现导航训练时**不需要也无法**重新训练它；换机器人平台时只需换一个对应的 `.pt` 文件。

---

## 5. 感知管线：深度图是怎么进网络的

1. **相机**：`RayCasterCamera`（射线检测，非渲染），ZedX 内参（fx=fy≈72.7, cx≈94.4, cy≈62.5），
   原始 192×120 **下采样 3 倍 → 64×40**，最大距离 11 m（超出置 0）。
2. **VAE 编码 + 噪声仿真**：`depth_image_noisy_delayed()` 把 `[B,1,64,40]` 原始深度送入
   JIT 编译的 `DepthNoiseEncoder`（RegNet-X-400MF 骨干 + FPN + VAE 采样头，
   自监督预训练权重 `vae_pretrain_new.pth`），一次前向同时得到：
   - **编码特征 `[B,64,5,8]`**（2560 维）→ 这就是策略看到的“深度图”；
   - 仿真了立体相机噪声的深度图（仅用于可视化/研究）。
3. **观测延迟**：每个环境在 episode 重置时为每路信号随机抽一个 0~2 步的延迟
   （`ObservationDelayManager`），模拟真实传感器链路。actor 看到的是“带噪 + 延迟”的特征，
   critic 额外能看到“完美无延迟”的深度（特权信息）。

---

## 6. 高层导航策略：观测与网络结构

### 6.1 Actor 观测（"policy" 组，共 2576 维）——部署时唯一使用的输入

| 项 | 维度 | 噪声 | 延迟 | 说明 |
|---|---|---|---|---|
| `base_lin_vel` | 3 | U(−0.2, 0.2) | 0–2 步 | 基座线速度（机体系） |
| `base_ang_vel` | 3 | U(−0.1, 0.1) | 0–2 步 | 基座角速度 |
| `projected_gravity` | 3 | U(−0.1, 0.1) | 0–2 步 | 投影重力（姿态） |
| `last_action` | 3 | — | — | 上一步高层动作（执行链处理前） |
| `target_position` | 4 | Δ噪声 p=0.1 | 0–2 步 | 见 §7.3 |
| `depth_image_noisy_delayed` | 64×5×8 = 2560 | VAE + 立体噪声仿真 | 0–2 步 | ZedX 射线相机 192×120→**64×40**，最大距离 11 m；冻结 VAE 编码为 64×5×8 特征 |

注意：**没有关节位置/速度**——高层策略只关心机体级状态，这正是它能在不同平台间泛化的原因之一。

### 6.2 Critic 观测（非对称 actor-critic，共 5713 维）——只用于训练价值函数

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

### 6.3 观测延迟机制（`mdp/delay_manager.py`）

每个环境在 reset 时为每类通道独立采样随机 lag ∈ {0, 1, 2} 步（`max_delay_* = 2`，共 0.4 s），
episode 内保持不变。作用：模拟真实传感器/通信延迟，**防止策略依赖"完美同步"的观测组合**
（否则 sim-to-real 会因几十毫秒延迟而崩溃）。

### 6.4 网络数据流（`rsl_rl/modules/actor_critic_sru.py`）

```
深度特征 (B,64,5,8) ─┬─ Self-Attention(3D位置编码, 掩码padding)
本体感知向量 ────────┴─ Cross-Attention(4头: 用本体感知去"查询"视觉特征)
                          │ 池化 → 64 维
                          ▼
            [本体感知 ⊕ 64维视觉融合] → LSTM_SRU (hidden=512, 1层)   ← SRU 记忆在这里
                          │ h_t (512)
                          ▼
            TC-Dropout(0.2) → MLP [512,256,128] → μ (3维速度指令)
                                                        log_std 为可学习参数
```

- actor 与 critic 各有一条独立通道（`memory_a` / `memory_c`），critic 额外融合高度图特征 + 时间嵌入(8维)。
- 注意力在**整条轨迹批量计算**（不在 RNN 循环内），RNN 只处理融合后的向量序列，效率更高。

### 6.5 SRU 单元：核心创新（`rsl_rl/networks/sru_memory/lstm_sru.py`）

标准 LSTM 的候选门 `g = tanh(Wx·x + Wh·h)`；SRU 版改为：

```
tx   = W_tx · x                     # "空间变换项"，只由当前输入生成
g_t  = tanh( tx ⊙ g )              # 逐元素相乘 → 空间变换门
f    = i·(1-(1-f)²) + (1-i)·f²     # 多项式精炼遗忘门 (Polynomial RNN, Gu et al. 2020)
c_t  = f·c_{t-1} + (1-f)·g_t
h_t  = o·tanh(c_t)
```

**直觉**：每步输入 x 里同时含有"我现在在哪（本体感知）"和"我看到了什么（深度特征）"。
`tx ⊙ g` 让网络学会用当前位姿去**变换/对齐**新观察到的内容再写入记忆——相当于隐式地做
"把 landmark A 从视角1的坐标变换到视角2的坐标"，而不需要显式的地图或齐次变换矩阵。
这就是论文说的 spatial memory：长距离走过后还能记得"那个拐角在我左后方 3 米"。

### 6.6 TC-Dropout（时间一致 dropout）

普通 dropout 每步独立采样掩码，会在时序上"随机擦掉记忆流"。
TC-Dropout 在**一条轨迹的所有时间步共享同一组掩码**（存储里保存 `dropout_masks`），
保证训练时的正则化不破坏时间一致性。

---

## 7. 任务设计

### 7.1 训练目的：任务的形式化定义

训练对象只有一个：**高层导航策略** π(a_t | o_t, h_{t−1})。

- 输入 o_t：第一视角深度图（VAE 特征）+ 本体感知 + 目标点的**机体系方向/对数距离**；
- 输出 a_t = [v_x, v_y, ω]：基座速度指令，5 Hz，高斯采样 N(μ, σ²)；
- **无地图、无全局定位、无重规划**：目标在 episode 内固定（`resampling_time_range=(1e9, 1e9)`），
  策略必须靠 SRU 隐状态记住"走过的路/去过的地方"，才能完成长程导航——这正是论文核心。

训练要塑造的四种能力：

| 能力 | 由什么设计驱动 |
|---|---|
| 长程空间记忆（60 s / 300 步不迷路） | 固定目标 + 60 s episode + SRU 单元 + γ=0.999 长视界 |
| 未知环境避障与路径选择 | 迷宫/楼梯/坑洞地形混合 + 稀疏化奖励（见 §8.2） |
| Sim-to-Real 鲁棒性（zero-shot 上真机） | 全量域随机化（§9）+ 分层架构（locomotion 冻结） |
| 平滑、低能耗运动 | lateral/action_rate/joint_acc 惩罚项（§8.1） |

MDP 要素速览：

| 要素 | 内容 |
|---|---|
| 观测 o_t | actor：2576 维（§6.1）；critic：5713 维特权观测（§6.2） |
| 动作 a_t | [v_x, v_y, ω] 高斯采样，tanh 后 ×[1.5, 1, 1] 再低通滤波 |
| 步长 | 高层 1 步 = 0.2 s（decimation 40 × dt 0.005） |
| episode | 60 s = 300 高层步，固定目标 |
| 成功 | 距目标 < 0.5 m（提前到达或超时判定） |
| 失败 | 非法接触 >1 N / 俯仰或侧倾 >40° / 坠落 z<−2 m / 60 s 超时未达 |

### 7.2 环境与 Episode 设计

#### 地形：180 块程序化迷宫（`terrains/maze_config.py`）

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

地形与目标采样的实现细节见 `sru-navigation-sim/docs/TERRAIN_AND_GOALS.md`。

#### 出生点与目标采样（`mdp/navigation/goal_commands.py`）

地形生成时预计算三张掩码，采样器 `PositionSampler` 直接查表：

- **目标**：从 `valid_mask`（带安全边距的有效格）均匀采样；
  **平台位置在目标表中重复 ×10**（`platform_repeat_count=10`）→ 目标以约 10 倍概率落在
  高处平台上，隐式鼓励"爬上楼梯/平台顶部"的行为（静态采样课程）；
- **出生点**：从 `spawn_mask`（更大边距，考虑机器人本体尺寸与随机朝向）均匀采样，
  离地 +0.05 m 防穿插；
- **目标高度** = 该格地形高度 + U(0.2, 0.8) m → 目标是悬浮在空中的 3D 点（影响 §8.2 的 soft 奖励）；
- **出生点与目标之间无最小距离约束**：初始距离从几米到 ~40 m 不等，
  策略必须通过 `log(1+d)` 压缩的距离观测自适应远近；
- **episode 内不重采样**（resampling_time_range=(1e9,1e9)）：目标一旦确定直到 episode 结束。

#### 目标观测的表示（为什么是 4 维）

```
target_position = [dir_x, dir_y, dir_z, log(1 + d)]   （机体系，d = ||goal − base||）
```

- **单位方向向量**：给出"往哪走"，与距离解耦；
- **log(1+d)**：把 0.5 m ~ 40 m 的动态范围压到 [~0.4, ~3.7]，远近都可分辨、不爆尺度；
- **噪声**（`DeltaTransformationNoiseCfg`）：每步以 **p=0.1** 概率对目标做随机旋转 0.1 rad /
  平移 0.5 m → 策略不能把目标当"完美 GPS"，必须容忍目标漂移（sim-to-real：真机定位/目标指令有误差）；
- **延迟**：0–2 步随机（§6.3）。

### 7.3 动作空间与执行链

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

## 8. 奖励函数设计

### 8.1 全部奖励项（`navigation_env_cfg.py: RewardsCfg`）

B2W / AoW-D 共用基类；Go2W 在其上覆盖（见 `go2w_nav_refactor_plan.md`）。

| 项 | 权重 | 函数 | 作用 |
|---|---|---|---|
| `reach_goal_xy_tight` | **+1.5** | 1/(1+(d_xy/0.25)²)/T_r(0.1) → d→0 时 **最高 +15** | 目标附近强吸引，精确到位 |
| `reach_goal_xy_soft` | **+0.25** | 1/(1+(d_3d/2.5)²)/T_r(1.0)，**flat=False（3D 误差）** | 长程平滑 shaping；因目标是悬浮 3D 点，爬上平台也能得分 |
| `lateral_movement` | −0.1 | \|v_y,机体\| | 抑制侧滑/横移（B2W 横向能力弱） |
| `action_rate_l1` | −0.1 | Σ\|a_t − a_{t−1}\| | 速度指令平滑，避免抖动 |
| `rot_movement` | −1e-5 | ‖ω_root‖ | 轻微抑制原地乱转 |
| `joint_acc_l2_joint` | −1e-7 | Σ‖关节加速度‖²（腿+轮） | 低层执行平滑，保护电机 |
| `episode_termination` | **−50** | `is_terminated`：**仅真终止**（接触/大角度/坠落），超时与到达不计 | 摔倒 = 灾难性失败，量级压倒所有 shaping |
| `backward_movement_penalty` | −0.0 | clamp(−v_x, 0, 1) | **B2W 当前配置关闭**；Go2W 配置为 **−1.0** |

另有奖励平移 `reward_shifting_value = 0.05`。

### 8.2 目标奖励的门控机制（关键细节，容易误读）

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

### 8.3 终止与成功/失败判定（`mdp/terminations.py`）

| 终止项 | 条件 | 记为 | bootstrap | 奖励后果 |
|---|---|---|---|---|
| `time_out` | 60 s 到达 | 超时时刻 <0.5 m → 成功，否则失败 | ✔（timeout） | 无 −50 |
| `early_termination`（at_goal） | 任意时刻 <0.5 m | **成功** | ✔（time_out=True） | 无 −50 |
| `base_contact` | base/hip/thigh 接触力 >1 N | 失败（绊倒/倒地） | ✘ | **−50** |
| `large_pitch_angle` | \|pitch\| 或 \|roll\| >40° | 失败（翻倒） | ✘ | **−50** |
| `terrain_fall` | z < −2 m | 失败（坠坑/坠落） | ✘ | **−50** |

成功统计：每次终止写入 `goal_reached_buffer`，`SuccessRateTracker` 维护每环境滚动 10 episode
的滑动成功率 → tensorboard 中的 `success_rate`。

### 8.4 奖励结构的设计意图

- **"站着不动"是早期安全下界**：超时不罚、到达才奖，所以乱动有 −50 风险而不动只有轻微 shaping
  惩罚 → 策略先学会"别摔"，再被稀疏目标奖励拉向"去走"。这是长程导航任务常见的保守起步设计。
- **没有速度跟踪项、没有路径长度惩罚**：导航是"到达"任务而非"跟踪"任务；只罚浪费性的
  横移（lateral）与抖动（action_rate），给策略充分的机动自由度（允许绕路）。
- **−50 的量级选择**：约为 tight 奖励峰值（+15）的 3 倍多、shaping 项的数百倍，
  保证"摔倒一次"在 GAE 里主导该 episode 的信用分配。
- **entropy_coef=0.00375 + init_std=1.0**：前期保持探索；MDPO 的双策略互蒸馏进一步防止过早塌缩。

---

## 9. 域随机化全表（zero-shot sim-to-real 的基础）

| 事件 | 时机 | 参数 | 模拟什么 |
|---|---|---|---|
| `physics_material` | startup | 静摩擦 (0.8,1.2)、动摩擦 (0.7,1.0)、恢复系数 (0,0.1) | 地面材质差异 |
| `reset_base` | reset | yaw ~ U(−π, π) | 随机初始朝向 |
| `reset_robot_joints` | reset | 关节位置 ×U(0.5,1.5) | 初始姿态不确定性 |
| `randomize_camera_height` | reset | 高度 ±0.1 m，roll/pitch/yaw ±5° | 相机安装误差（B2W 挂载点 (0.387, 0, 0.28)，俯仰 20°） |
| `randomize_action_scale` | reset | 见 §7.3 | 执行器增益失配 |
| `reset_delay_buffer` | reset | 每通道 lag ~ U{0,1,2} 步 | 传感器延迟（§6.3） |
| `randomize_low_pass_filter_alpha` | reset | α ~ U(0.1, 0.6) 每维度（B2W） | 执行器响应快慢不一 |
| `push_robot` | 间隔 0.2–5 s | 水平速度 ±0.5 m/s | 外部扰动/碰撞恢复 |

叠加在观测层的随机化：本体感知均匀噪声（§6.1）、深度 VAE 立体噪声仿真、目标 Δ噪声（§7.2）。
**play 模式（`-Play-v0`）关闭全部 corruption 与 push**，用于评估"干净环境下的真实水平"。

---

## 10. 算法设计与超参

### 10.1 PPO vs MDPO（`config/b2w/agents/rsl_rl_cfg.py`）

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

### 10.2 MDPO 代码级机制（`rsl_rl/algorithms/mdpo.py`）

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

### 10.3 MUON 优化器（可选增强）

对 RNN/MLP 的**隐藏层权重**用 Muon（Newton-Schulz 正交化动量，bfloat16），
bias/gain 等仍用 AdamW。降低显存、训练更稳。（README 注明此项为论文之外的新增。）

### 10.4 一次迭代到底发生了什么

```
repeat num_envs × 16 步:
    环境并行前向 (Isaac Sim):
        相机射线 → VAE编码+噪声 → 观测组
        高层策略(5Hz) → 速度指令 → 低通滤波
        低层locomotion(50Hz) → 关节动作 → 物理积分(200Hz)
        累计奖励 / 判定终止
    存储 transition（含 RNN 隐状态、TC-dropout 掩码）
计算 GAE 优势与回报 (γ, λ=0.95)
PPO/MDPO 更新: 5 epochs × 4 mini-batches (轨迹级切分, 重放时恢复隐状态)
每 500 次迭代保存 model_XXXX.pt + 配置快照
```

---

## 11. 课程学习现状

- **地形课程关闭**（`curriculum=False`，B2W 固定 level 10）：难度不随训练递进；
- `disable_backward_penalty_after_steps(500)`：钩子存在但反向惩罚权重本就是 −0.0 → **当前配置下为 no-op**；
- 实际存在的"静态课程"只有两处：**平台目标 ×10 采样偏置**（§7.2）与**混合地形分布**（§7.2）。

---

## 12. 训练时该看什么（指标与典型动态）

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

指标的详细物理含义、公式与健康区间见 `TRAINING_MONITORING.md`。

---

## 13. Sim-to-Real 与部署

- 论文主张 **zero-shot**：仿真训好的策略直接上真机，不微调。靠的是第 9 节的域随机化 +
  分层架构（locomotion 本身也是 sim 预训练后零样本迁移的）。
- 导出：`scripts/export_onnx.py`（或 `scripts/play.py --export_onnx / --export_jit`）。
- **循环网络的 ONNX 约定**：隐状态显式暴露为输入/输出 `h_in, c_in → h_out, c_out`；
  episode 开始时置零，之后每步把上一步的 `h_out/c_out` 回传。
- 真机部署代码在 `sru-robot-deployment`（ROS 2），用导出的 ONNX 跑高层策略，底层同样调用冻结的 locomotion 策略。
  部署文档见该仓库 `docs/GO2W_DEPLOYMENT_GUIDE.md` 与 `docs/GO2W_NAVIGATION_SYSTEM_GUIDE.md`。

---

## 14. 附录：关键文件索引

| 内容 | 路径 |
|---|---|
| 环境基类（频率/场景/奖励/终止/事件） | `sru-navigation-sim/isaaclab_nav_task/navigation/navigation_env_cfg.py` |
| B2W / AoW-D / Go2W 具体配置 | `.../navigation/config/{b2w,aow_d,go2w}/navigation_env_cfg.py` |
| 分层动作（SE2 + 低层策略调用） | `.../mdp/navigation/actions/navigation_se2_actions.py` |
| 观测项（深度编码/延迟等） | `.../mdp/observations.py`、`.../mdp/delay_manager.py` |
| 深度编码器（VAE+噪声） | `.../mdp/depth_noise_encoder.py`、`.../mdp/depth_utils/camera_config.py` |
| 奖励 / 终止函数 | `.../mdp/rewards.py`、`.../mdp/terminations.py` |
| 目标采样与成功统计 | `.../mdp/navigation/goal_commands.py` |
| 地形配置（180 地形） | `sru-navigation-sim/isaaclab_nav_task/terrains/maze_config.py` |
| SRU actor-critic | `sru-navigation-learning/rsl_rl/modules/actor_critic_sru.py` |
| SRU 单元（空间变换门） | `sru-navigation-learning/rsl_rl/networks/sru_memory/lstm_sru.py` |
| 注意力融合（3D 位置编码） | `sru-navigation-learning/rsl_rl/networks/sru_memory/attention.py` |
| MDPO / PPO 算法 | `sru-navigation-learning/rsl_rl/algorithms/mdpo.py`、`ppo.py` |
| 训练/回放脚本 | `sru-navigation-sim/scripts/train.py`、`scripts/play.py` |
| 附录 A 空间记忆实验 | `sru-pytorch-spatial-learning/run_pointcloud.py` |

# SRU 导航项目：训练原理与系统架构说明

> 对应论文：*Spatially-enhanced recurrent memory for long-range mapless navigation via end-to-end reinforcement learning*（IJRR 2025）
> 本文档基于 `sru-navigation-learning` / `sru-navigation-sim` 两个仓库的实际代码整理，所有参数均取自当前配置。
> 导航训练设计的深度剖析（目标采样、奖励门控机制、域随机化全表、MDPO 互蒸馏）见同目录 `NAVIGATION_TRAINING_DESIGN.md`。

---

## 1. 一句话总结

训练一个**高层导航策略**：它只看第一视角深度图 + 本体感知 + 目标点方向，输出速度指令 [vx, vy, ω]；
底层由一个**预训练、冻结的 locomotion 策略**把速度指令翻译成关节动作。
核心创新 SRU（空间增强循环单元）让 RNN 隐式记住"我在哪、我见过什么"，从而实现长距离无地图导航。

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

## 3. 什么在训练、什么是冻结的

| 组件 | 状态 | 说明 |
|---|---|---|
| 高层 ActorCriticSRU（actor + critic） | ✅ 训练中 | MDPO 模式下是**两份**相同结构、各自独立参数 |
| 低层 locomotion 策略 | ❄️ 冻结 | `torch.jit.load` 加载，只前向推理，不参与梯度 |
| VAE 深度编码器 | ❄️ 冻结 | 自监督预训练（见 `sru-depth-pretraining` 仓库），这里只做特征提取 + 噪声仿真 |

所以"复现训练"= 只训练高层导航策略；locomotion 和视觉编码器是随 sim 仓库一起发布的资产：
- locomotion：`isaaclab_nav_task/navigation/assets/data/Policies/locomotion/b2w/policy_b2w_new_2.pt`、`.../aow_d/policy_blind_3_1.pt`
- 深度编码器：`.../Policies/depth_encoder/vae_pretrain_new.pth`

## 4. Locomotion（机器狗行走）是怎么做的

### 4.1 分层控制原理

高层策略**不直接控制关节**。它输出的速度指令会进入低层策略的观测里，低层策略负责"怎么走"：

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

文件名里的 **blind** 表示低层策略是"盲走"的——只用本体感知、不看视觉。这是刻意为之：
纯本体感知的 locomotion 更鲁棒、与具体场景无关，可以冻结后跨地形复用；所有"看路"的智能都压在高一层。

### 4.3 Locomotion 策略从哪来

它不是本仓库训练的，而是预先用 RL 单独训练好的速度跟踪（velocity tracking）locomotion 策略
（输入速度指令 + 本体感知 → 输出关节动作），以 TorchScript 形式随仓库发布。
复现导航训练时**不需要也无法**重新训练它；换机器人平台时只需换一个对应的 `.pt` 文件。

## 5. 感知管线：深度图是怎么进网络的

1. **相机**：`RayCasterCamera`（射线检测，非渲染），ZedX 内参（fx=fy≈72.7, cx≈94.4, cy≈62.5），
   原始 192×120 **下采样 3 倍 → 64×40**，最大距离 11 m（超出置 0）。
2. **VAE 编码 + 噪声仿真**：`depth_image_noisy_delayed()` 把 `[B,1,64,40]` 原始深度送入
   JIT 编译的 `DepthNoiseEncoder`（RegNet-X-400MF 骨干 + FPN + VAE 采样头，
   自监督预训练权重 `vae_pretrain_new.pth`），一次前向同时得到：
   - **编码特征 `[B,64,5,8]`**（2560 维）→ 这就是策略看到的"深度图"；
   - 仿真了立体相机噪声的深度图（仅用于可视化/研究）。
3. **观测延迟**：每个环境在 episode 重置时为每路信号随机抽一个 0~2 步的延迟
   （`ObservationDelayManager`），模拟真实传感器链路。actor 看到的是"带噪 + 延迟"的特征，
   critic 额外能看到"完美无延迟"的深度（特权信息）。

## 6. 高层导航策略：观测与网络结构

### 6.1 Actor 观测（"policy" 组，拼接成向量）

| 项 | 含义 | 噪声/延迟 |
|---|---|---|
| `base_lin_vel` (3) | 机体线速度 | ±0.2，延迟≤2步 |
| `base_ang_vel` (3) | 机体角速度 | ±0.1，延迟≤2步 |
| `projected_gravity` (3) | 重力投影（姿态） | ±0.1，延迟≤2步 |
| `last_action` (3) | 上一步速度指令 | — |
| `target_position` (4) | 到目标的**归一化方向向量**(3维,机体坐标系) + log(1+距离) | 随机 Δ 变换噪声（旋转 0.1 / 平移 0.5，概率 0.1），延迟≤2步 |
| `depth_image` (2560) | VAE 深度特征 64×5×8 | 带噪 + 延迟 |

注意：**没有关节位置/速度**——高层策略只关心机体级状态，这正是它能在不同平台间泛化的原因之一。

### 6.2 Critic 观测（非对称 actor-critic）

critic 额外拿到 actor 看不到的"作弊信息"：
- `height_scan_critic`：头顶 10 m × 10 m 射线网格（0.2 m 分辨率）→ 编码成 64×7×7（3136 维）高度图特征；
- **完美**深度特征（无噪声、无延迟）；
- `time_normalized`：episode 剩余时间归一化值。

非对称设计让价值函数估计更准，而部署时只需要 actor。

### 6.3 网络数据流（`rsl_rl/modules/actor_critic_sru.py`）

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

### 6.4 SRU 单元：核心创新（`rsl_rl/networks/sru_memory/lstm_sru.py`）

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

### 6.5 TC-Dropout（时间一致 dropout）

普通 dropout 每步独立采样掩码，会在时序上"随机擦掉记忆流"。
TC-Dropout 在**一条轨迹的所有时间步共享同一组掩码**（存储里保存 `dropout_masks`），
保证训练时的正则化不破坏时间一致性。

## 7. 任务定义：奖励、终止、目标与地形

### 7.1 奖励（`navigation_env_cfg.py` → RewardsCfg）

| 项 | 权重 | 说明 |
|---|---|---|
| `reach_goal_xy_soft` | +0.25 | 距离目标的软奖励，sigmoid(2.5) 平滑衰减，T_r=1 m |
| `reach_goal_xy_tight` | +1.5 | 到达 0.1 m 内给大额 flat 奖励（主要驱动） |
| `lateral_movement` | -0.1 | 惩罚侧向移动（鼓励直行） |
| `action_rate_l1` | -0.1 | 惩罚速度指令突变（平滑） |
| `rot_movement` | -1e-5 | 轻微惩罚原地旋转 |
| `joint_acc_l2_joint` | -1e-7 | 惩罚关节加速度（保护执行器） |
| `episode_termination` | -50 | 摔倒/非法接触等失败终止的重罚 |

另有奖励平移 `reward_shifting_value = 0.05`。

### 7.2 终止条件

| 条件 | 判定 | 语义 |
|---|---|---|
| `at_goal` | 距目标 < 0.5 m | ✅ 成功（提前结束，按 timeout 处理） |
| `time_out` | 60 s 超时且距离 > 0.5 m | ❌ 失败 |
| `base_contact` | base/髋/大腿接触力 > 1 N | ❌ 摔倒 |
| `large_pitch_angle` | pitch > 40° | ❌ 翻倒 |
| `terrain_fall` | 坠落高度 < -2 m | ❌ 掉坑 |

### 7.3 目标点

- 每个 episode **固定一个目标**（resampling_time 设为 1e9，episode 内不重采样）；
- 从地形的"有效可达格子"中均匀采样（预计算 valid_mask，含安全 padding：目标 0.5 m、出生点 0.6 m），
  保证目标可走通、出生点放得下机器人。

### 7.4 迷宫地形与课程

- 180 个地形 = **6 个难度行 × 30 个变体列**，每个 30 m × 30 m、0.1 m 分辨率；
- 构成比例：约 30% 迷宫 / 20% 随机 / 30% 楼梯 / 20% 深坑；
- B2W 当前配置：`curriculum = False`，初始地形等级 `max_init_terrain_level = 10`（直接在高难度上训练）；
- AoW-D 类似。详见 `sru-navigation-sim/docs/TERRAIN_AND_GOALS.md`。

## 8. 域随机化清单（sim-to-real 的关键）

| 事件 | 时机 | 内容 |
|---|---|---|
| `physics_material` | 启动 | 摩擦系数 μs 0.8~1.2, μd 0.7~1.0, 恢复系数 0~0.1 |
| `randomize_camera_height` | 重置 | 相机高度 ±0.1 m，roll/pitch/yaw ±5°（外参标定误差） |
| `randomize_action_scale` | 重置 | 速度指令增益 x:0.8~1.2, y:0.6~1.0, θ:0.8~1.2 + 偏置 |
| `randomize_low_pass_filter_alpha` | 重置 | 低通 α 每环境每维度随机（B2W: 0.1~0.6） |
| `reset_delay_buffer` | 重置 | 每路观测延迟 0~2 步随机 |
| `push_robot` | 间隔 0.2~5 s | 随机水平冲量 ±0.5 m/s（抗扰动） |
| 观测噪声 | 持续 | 速度/姿态/关节/目标位置均加均匀噪声 |

## 9. 训练算法

### 9.1 通用设置（B2W MDPO/PPO 配置相同部分）

- `num_steps_per_env = 16`：每个环境每次 rollout 收集 16 个高层步（= 3.2 s 仿真时间）；
- `num_learning_epochs = 5`，`num_mini_batches = 4`（mini-batch 按轨迹切分、保留时序）；
- `max_iterations = 15000`，每 500 次迭代存 checkpoint；seed = 60；
- 默认 `--num_envs 2048~4096`（MDPO 下两个网络各管一半环境）。

### 9.2 PPO（基线）

标准 PPO：clip=0.2、GAE γ=0.995 λ=0.95、自适应学习率（desired_kl=0.01）、
entropy_coef=0.00375、value_loss_coef=0.02。

### 9.3 MDPO（论文主算法，`rsl_rl/algorithms/mdpo.py`）

**双网络 + 深度互学习**：

1. 两份结构相同的 ActorCriticSRU（参数独立），环境对半分：
   策略 1 只控制前一半环境的动作，策略 2 控制另一半；
2. 每次更新时，每个策略先算自己的 PPO loss；
3. **互蒸馏**：把策略 2 拿到策略 1 的数据上前向（作为"教师"），
   计算 `KL(π_teacher ‖ π_student)`，反之亦然。**梯度同时穿过两个网络**——
   两个策略互相拉近、互相正则化，相当于用双倍参数做隐式集成，
   提高探索多样性与样本效率；
4. 总损失：`0.5 × [ (PPO₁ + 0.02·KL₂→₁) + (PPO₂ + 0.02·KL₁→₂) ]`（`distill_coef = 0.02`）；
5. γ=0.999（比 PPO 更看重长期回报，适合长 horizon 导航）。

### 9.4 MUON 优化器（可选增强）

对 RNN/MLP 的**隐藏层权重**用 Muon（Newton-Schulz 正交化动量，bfloat16），
bias/gain 等仍用 AdamW。降低显存、训练更稳。（README 注明此项为论文之外的新增。）

### 9.5 一次迭代到底发生了什么

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

## 10. 训练时该看什么指标

- **reach rate / success**：`metrics.in_goal`（距目标 <0.5 m）；tensorboard/wandb 里看 episode 内到达率随迭代上升；
- `Episode_Reward/Time_out`、`Episode_Termination`：成功 vs 摔倒比例；
- `Losses/policy_loss`、`KL`（MDPO 的蒸馏 KL 应缓慢下降后平稳）、entropy（不应过早塌缩）；
- 每 2000 步可录视频（`--video --video_interval 2000`）直观检查行为。

日志目录：`logs/rsl_rl/<experiment_name>/<时间戳>/`，checkpoint 为 `model_*.pt`。

## 11. Sim-to-Real 与部署

- 论文主张 **zero-shot**：仿真训好的策略直接上真机，不微调。靠的是第 8 节的域随机化 +
  分层架构（locomotion 本身也是 sim 预训练后零样本迁移的）。
- 导出：`scripts/play.py --export_onnx / --export_jit`，或代码里 `policy.export_onnx(...)`。
- **循环网络的 ONNX 约定**：隐状态显式暴露为输入/输出 `h_in, c_in → h_out, c_out`；
  episode 开始时置零，之后每步把上一步的 `h_out/c_out` 回传。
- 真机部署代码在独立仓库 `leggedrobotics/sru-robot-deployment`（ROS2 + Gazebo），
  用导出的 ONNX 跑高层策略，底层同样调用冻结的 locomotion 策略。

## 12. 附录：关键文件索引

| 内容 | 路径 |
|---|---|
| 环境基类（频率/场景/奖励/终止/事件） | `sru-navigation-sim/isaaclab_nav_task/navigation/navigation_env_cfg.py` |
| B2W / AoW-D 具体配置 | `.../navigation/config/b2w/navigation_env_cfg.py`、`.../aow_d/navigation_env_cfg.py` |
| 分层动作（SE2 + 低层策略调用） | `.../mdp/navigation/actions/navigation_se2_actions.py` |
| 观测项（深度编码/延迟等） | `.../mdp/observations.py`、`.../mdp/delay_manager.py` |
| 深度编码器（VAE+噪声） | `.../mdp/depth_noise_encoder.py`、`.../mdp/depth_utils/camera_config.py` |
| 地形生成与目标采样 | `sru-navigation-sim/isaaclab_nav_task/terrains/`、`docs/TERRAIN_AND_GOALS.md` |
| SRU actor-critic | `sru-navigation-learning/rsl_rl/modules/actor_critic_sru.py` |
| SRU 单元（空间变换门） | `sru-navigation-learning/rsl_rl/networks/sru_memory/lstm_sru.py` |
| 注意力融合（3D 位置编码） | `sru-navigation-learning/rsl_rl/networks/sru_memory/attention.py` |
| MDPO / PPO 算法 | `sru-navigation-learning/rsl_rl/algorithms/mdpo.py`、`ppo.py` |
| 训练/回放脚本 | `sru-navigation-sim/scripts/train.py`、`scripts/play.py` |
| 附录 A 空间记忆实验 | `sru-pytorch-spatial-learning/run_pointcloud.py` |

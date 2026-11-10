# Unitree B2W 导航目标朝向控制 (Goal Heading Control) 微调交接文档

---

## 1. 项目背景与目标

本任务旨在基于开源的 Unitree B2W 轮足机器人视觉导航策略（`nav_policy.onnx`），在保持原有优秀地形通过能力的前提下，微调实现**到点机身目标朝向（Goal Arrival Heading）精准控制**。

- **核心需求**：机器人在到达终点坐标 $(x, y)$ 的同时，机身朝向与目标给定的航向角 $\psi_{\text{goal}}$ 对齐（误差 $< 0.35\text{ rad} \approx 20^\circ$），并在终点平稳停驻。
- **基线约束**：回归原版官方 B2W 开源导航奖励基准，不引入多余外来复杂项，保持策略在复杂地形上的基础移动能力与步态稳定性。

---

## 2. 系统架构与关键文件结构

### 2.1 观测空间扩展 (Observation Space)
原版开源策略的本体感知（Proprioceptive）为 16 维，扩容后为 18 维：
$$\text{Policy Obs (2578 维)} = [\underbrace{\text{base\_lin\_vel}}_{3}, \underbrace{\text{base\_ang\_vel}}_{3}, \underbrace{\text{projected\_gravity}}_{3}, \underbrace{\text{last\_action}}_{3}, \underbrace{\text{target\_position}}_{4}, \underbrace{\mathbf{target\_heading}}_{2}, \underbrace{\text{depth\_image}}_{2560}]$$
- `target_position (4维)`：目标在机身系下的单位方向向量与对数距离：$[d_x, d_y, d_z, \ln(\text{dist} + 1.0)]$。
- `target_heading (2维)`：目标朝向在机身系下的相对夹角编码：$[\cos(\Delta\psi), \sin(\Delta\psi)]$，其中 $\Delta\psi = \psi_{\text{goal}} - \psi_{\text{robot}}$。

### 2.2 核心文件映射表

| 文件路径 | 职责说明 |
|:---|:---|
| [`scripts/surgery_expand_heading_from_onnx.py`](file:///home/yichangfeng/sru-navigation-learning/scripts/surgery_expand_heading_from_onnx.py) | 网络权重手术脚本：将 16 维 ONNX 权重扩容为 18 维 PyTorch 权重 |
| [`IsaacLab/logs/warmstart_nav_policy_heading.pt`](file:///home/yichangfeng/sru-navigation-learning/IsaacLab/logs/warmstart_nav_policy_heading.pt) | 微调热启动检查点（包含匹配 target_pos 尺度的敏感化朝向初值） |
| [`isaaclab_nav_task/navigation/config/b2w/navigation_env_cfg.py`](file:///home/yichangfeng/sru-navigation-learning/IsaacLab/source/isaaclab_nav_task/isaaclab_nav_task/navigation/config/b2w/navigation_env_cfg.py) | B2W 朝向任务配置：官方 B2W 原生奖励基线 + 平滑朝向与终末结算项 |
| [`isaaclab_nav_task/navigation/config/b2w/agents/rsl_rl_cfg.py`](file:///home/yichangfeng/sru-navigation-learning/IsaacLab/source/isaaclab_nav_task/isaaclab_nav_task/navigation/config/b2w/agents/rsl_rl_cfg.py) | MDPO 算法微调超参：小学习率 `3e-5`、`freeze_iters = 2000`、`max_iterations = 6000` |
| [`isaaclab_nav_task/navigation/mdp/curriculums.py`](file:///home/yichangfeng/sru-navigation-learning/IsaacLab/source/isaaclab_nav_task/isaaclab_nav_task/navigation/mdp/curriculums.py) | 三阶段课程学习：`b2w_heading_curriculum_phase` 调度权重与判定门限 |
| [`isaaclab_nav_task/navigation/mdp/rewards.py`](file:///home/yichangfeng/sru-navigation-learning/IsaacLab/source/isaaclab_nav_task/isaaclab_nav_task/navigation/mdp/rewards.py) | 核心奖励实现：平滑朝向对齐引导 `target_heading`、成功/超时终末结算（含 dt 稀释补偿） |
| [`isaaclab_nav_task/navigation/mdp/terminations.py`](file:///home/yichangfeng/sru-navigation-learning/IsaacLab/source/isaaclab_nav_task/isaaclab_nav_task/navigation/mdp/terminations.py) | 核心终止判定：单向锁存到点调姿状态机、5.0s $\to$ 2.0s 自适应课程退火窗口 |
| [`isaaclab_nav_task/navigation/mdp/navigation/goal_commands.py`](file:///home/yichangfeng/sru-navigation-learning/IsaacLab/source/isaaclab_nav_task/isaaclab_nav_task/navigation/mdp/navigation/goal_commands.py) | 导航目标生成器：离墙 $\ge 0.8\text{m}$ 安全旋转采样保障、航向随机采样与指标统计 |
| [`rsl_rl/modules/actor_critic_sru.py`](file:///home/yichangfeng/sru-navigation-learning/rsl_rl/modules/actor_critic_sru.py) | 策略网络结构：`set_freeze_spatial_and_sru` 局部冻结时空记忆单元 |
| [`rsl_rl/runners/on_policy_runner.py`](file:///home/yichangfeng/sru-navigation-learning/rsl_rl/runners/on_policy_runner.py) | 算法训练运行器：分阶段参数解冻调度、当前学习迭代数向环境课程透传 |
| [`rsl_rl/algorithms/optim/muon.py`](file:///home/yichangfeng/sru-navigation-learning/rsl_rl/algorithms/optim/muon.py) | Muon / AdamW 优化器：支持部分参数冻结（`p.grad is None` 安全跳过与衰减保护） |
| [`IsaacLab/train_b2w_heading_finetune.sh`](file:///home/yichangfeng/sru-navigation-learning/IsaacLab/train_b2w_heading_finetune.sh) | 微调一键启动脚本 |
| [`isaaclab_nav_task/scripts/verify_refactor.py`](file:///home/yichangfeng/sru-navigation-learning/IsaacLab/source/isaaclab_nav_task/scripts/verify_refactor.py) | 环境配置自动化断言与验证脚本 |

---

## 3. 历史问题深度复盘与核心机理

### 问题 1：为什么微调初期 target_heading 奖励在 0.5 左右且无法提升？
- **数学假象**：当机器人静止于目标点，到达航向角 $\Delta\psi \sim U[-\pi, \pi]$ 为完全随机分布时，$\mathbb{E}[\cos(\Delta\psi)] = 0$。因此 $R = 0.5 \times (1 + \cos(\Delta\psi)) = 0.5$ 正好是无控制机器人的**随机基线期望**，并不代表策略学会了转向。
- **终点挂机刷分漏洞**：在原版任务中，进入终点圈后可获得高达 15 分/步的位置奖励。机器人发现只要在终点静止不动，就能稳稳挂机刷满步数获得数千累积回报，而转向不仅有风险，且提早终止反而损失后续挂机收益。

### 问题 2：为什么此前加入驻留超时后，成功率反而从 16.5% 崩跌至 3.5%？
- **原地转向的位置扣分远大于朝向加分（收益倒挂）**：四轮足原地踏步调头时伴随 $0.15 \sim 0.2\text{m}$ 抖动，位置分骤降 6 分/步，远超微弱的朝向加分。
- **冲线逃逸恶性循环**：策略发现进圈必死，学出了高速冲过终点蹭瞬时分后逃逸漫游的病态行为。
- **权重 0 初始化导致朝向输入失明**：低学习率下权重范数爬升极其缓慢。

### 问题 3：为什么引入饱和平台保护后，成功率冲到 22% 又开始掉头下跌？
经过对训练动态（TensorBoard 第 70 ~ 315 步全量数据）的深度剖析，定位到此前过度打补丁引发的次生博弈漏洞：
1. **强化学习的“早死扣分（提前终止惩罚）”悖论**：
   - 只要对齐成功，就会触发 `early_termination` 立即重启，切断了后续回报流；
   - 故意不对齐，可以在圈内赖满 80 步刷满位置分（$80 \times 15 = 1200$ 分）；
   - 成功没有额外大奖，驻留超时被杀也没有惩罚，Critic 价值网络算清这笔账后，**梯度严重惩罚对齐动作**。
2. **驻留计时器出圈清零漏洞**：
   - 此前代码在出圈时重置计数器（`time_in_goal_circle_in_steps[~in_circle] = 0`），狗在 80 步快耗尽时只要晃出圈 1 步就能刷新计时器，诱发了在边界进出卡 bug 的漫游行为。

---

## 4. 当前生产级极简方案与参数配置（回归官方基线）

坚决摒弃复杂外来项，回归官方纯粹 B2W 基准，做纯粹的减法，构建**时序解耦、动态退火、闭环奖惩**的到点姿态调整机制：

### 4.1 全面回归官方 B2W 原生奖励基线
[`b2w/navigation_env_cfg.py`](file:///home/yichangfeng/sru-navigation-learning/IsaacLab/source/isaaclab_nav_task/isaaclab_nav_task/navigation/config/b2w/navigation_env_cfg.py) 中 `B2WHeadingRewardsCfg` 完全继承官方 `RewardsCfg`：
- 保留官方原生参数：`reach_goal_xy_tight`（权重 1.5，sigmoid 0.25，T_r 0.1）、`reach_goal_xy_soft`（权重 0.25）、`action_rate_l1`（-0.1）、`lateral_movement`（-0.1）、`joint_acc_l2_joint`（-1e-7）、`episode_termination`（-50.0）；
- **彻底清除多余项**：删除 `velocity_at_goal_linear_penalty`、删除 `target_yaw_progress`，恢复纯净官方基线。

### 4.2 到点姿态自适应课程退火窗口 (5.0s $\to$ 2.0s)
在 [`terminations.py`](file:///home/yichangfeng/sru-navigation-learning/IsaacLab/source/isaaclab_nav_task/isaaclab_nav_task/navigation/mdp/terminations.py) 的 `at_goal_with_heading_navigation` 中实现：
1. **单向锁存机制**：机器狗首次进入目标圈（$d < 0.4\text{m}$）后，`is_adjusting_heading` 置为 `True` 并单向锁定，出圈绝不清零，彻底杜绝卡 bug 刷新计时。
2. **随成功率动态退火课程**：
   $$T_{\text{adjust}} = 5.0 - 3.0 \times \text{clamp}\left(\frac{\text{SuccessRate} - 0.20}{0.50}, 0.0, 1.0\right)$$
   - **初期给足 5.0s（50 步）**：初学阶段轮足原地踏步较慢，给足 5 秒充裕时间探索转向动作并拿到正向反馈；
   - **随成功率退火至 2.0s（20 步）**：随着成功率从 20% 爬升至 70%，调整窗口自动线性收缩至 2.0s，迫使策略更快、更利索地收敛对齐。

### 4.3 闭环终末结算机制 (Terminal Outcome Bonus & Penalty)
彻底消除“挂机多拿分、早死受惩罚”的数学矛盾，在 [`rewards.py`](file:///home/yichangfeng/sru-navigation-learning/IsaacLab/source/isaaclab_nav_task/isaaclab_nav_task/navigation/mdp/rewards.py) 中配置 `target_heading_terminal_outcome`：
- **分支 A（成功结算）**：连续 3 步满足误差 $< 20^\circ$ 且线速度 $< 0.45\text{ m/s}$，发放**对齐成功终末大奖 $+30.0$**，记为 `Success (1.0)` 并提前终止；
- **分支 B（超时处决）**：耗尽当前调整窗口（5s $\to$ 2s）仍未对齐，施加**超时失败终末惩罚 $-15.0$**，记为 `Fail (0.0)` 并提前处决。
- **数学效果**：成功对齐总收益显著压倒赖着不走，Advantage 梯度彻底变为积极对齐。

### 4.4 平滑朝向过程引导 (Target Heading Alignment)
在调整窗口期内，基于实时姿态误差提供平滑非负余弦引导（权重 2.0）：
$$R_{\text{heading}} = 2.0 \times \frac{1 + \cos(\Delta\psi)}{2}$$
机头正对目标为满分 2.0，垂直为 1.0，背对为 0.0，平滑无奇点。

### 4.5 敏感化权重热启动 (Warmstart Surgery)
- 在 [`scripts/surgery_expand_heading_from_onnx.py`](file:///home/yichangfeng/sru-navigation-learning/scripts/surgery_expand_heading_from_onnx.py) 中，采用与 `target_position` 统计分布匹配的高斯分布初始化 heading 相关的权重列（$\text{std} \sim 0.05 \sim 0.10$）。
- 生成的 `warmstart_nav_policy_heading.pt` 从第 1 步起即可向网络传递朝向特征前向输出与反向梯度。

### 4.6 学习率微调与空间/时序记忆网络分阶段冻结保护 (Partial Freezing)
SRU 网络对高维视觉深度图的时空记忆极其敏感。若在微调初期全参数大学习率微调，SRU 会迅速发生灾难性遗忘，破坏避障与建图能力：
1. **学习率衰减**：
   - 微调学习率设为 `3e-5`（相比从头训练的 `3e-4` 降低一个数量级），采用 Cosine Schedule 平滑退火至 `1e-6`。
2. **前 600 个 Iterations 局部冻结**：
   - **冻结层**：深度编码器（Depth Encoder Conv）、空间注意力层（Spatial Attention: `self_attn`, `cross_attn`, `norm`, `ffn`）以及 SRU 时序记忆单元（`memory_a`, `memory_c`）。
   - **可训练层**：仅保留 Policy Head（MLP `actor`, `linear_dropout_actor`, `critic`）和本体感知投影层（`info_proj`）。
   - **机理**：让 MLP Head 和 Observation Embedding 率先学会理解和映射新的 18 维输入（尤其是新增的 $\Delta\psi$ 航向观测），在此期间绝对不污染 SRU 已学好的空间特征。
3. **600 Iterations 自动全参数解冻与 Phase 1.5 梯度适应**：
   - 达到 600 步后，`OnPolicyRunner` 自动调用 `set_freeze_spatial_and_sru(False)`，解冻全网络参数联合微调。
   - 随之进入 **Phase 1.5（400 轮全解冻纯导航适应期）**：保持 `align_weight = 0.0`，让 SRU 在小学习率（`3e-5`）下平滑适应梯度流动，彻底消除解冻瞬间的特征漂移，且不引入朝向新任务的干扰。
4. **Muon / AdamW 优化器冻结鲁棒性修复**：
   - 修复了 `rsl_rl/algorithms/optim/muon.py` 中的 `SingleDeviceMuonWithAuxAdam`，对 `p.grad is None` 的冻结参数进行安全跳过，防止梯度计算报错，并防止冻结参数被错误应用 Weight Decay。

### 4.7 四阶段课程学习与自动化衔接 (4-Stage Curriculum & Anti-Milking Balance)
在 [`b2w_heading_curriculum_phase`](file:///home/yichangfeng/sru-navigation-learning/IsaacLab/source/isaaclab_nav_task/isaaclab_nav_task/navigation/mdp/curriculums.py) 中定义了四阶段课程演进机制（总步数 6000 iters）：
- **Phase 1 (进度 0% ~ 10%，0 ~ 600 iters)**：
  - **核心目标**：温和复习与 18 维输入投影适应（SRU 冻结保护）。
  - **参数设置**：`align_weight = 0.0`，`target_heading_terminal = 0.0`，`velocity_linear_penalty = 0.0`。
  - **终止判定**：宽松到点条件（$d < 0.4\text{m}$，航向不限 $\approx 3.14\text{ rad}$），**固定宽松窗口 `fixed_dwell_time = 5.0s`（50 步）**，取消自适应压缩，杜绝过早紧逼。
  - **效果**：网络在小学习率下复习到点避障与进圈稳定，建立基线信心。
- **Phase 1.5 (进度 10% ~ 16.7%，600 ~ 1000 iters，共 400 轮)**：
  - **核心目标**：全网络解冻，SRU 适应小学习率梯度流动，以压倒性收益差破除“圈内高速晃悠赖分”肌肉记忆。
  - **参数设置**：保持 `align_weight = 0.0`（暂不引入朝向调头干扰，专注刹车停稳）；**引入温和终末结算 `target_heading_terminal = 0.3333`（成功减速退出净奖励 $+10.0$，超时扣除 $-5.0$）**；**加重终点线速度惩罚 `velocity_linear_penalty = -2.0`**（当 $d < 0.4\text{m}$ 且线速度 $v > 0.2\text{ m/s}$ 时强力扣分，但**对原地调头角速度 $\omega_z$ 施加 0 惩罚**）。
  - **终止判定**：维持宽松到点条件（$d < 0.4\text{m}$，航向不限 $\approx 3.14\text{ rad}$），固定停留窗口 `fixed_dwell_time = 5.0s`。
  - **博弈收益测算**：
    - 机器人若在圈内以 $0.6\text{m/s}$ 晃悠 50 步：获 $+15.0$ 位置分，但吃 $-5.0$ 超时罚分及 $-8.0$ 线速度惩罚，**净收益仅 $+2.0$**；
    - 机器人若入圈立即减速平稳停住（3步满足 $v<0.45\text{m/s}$）：瞬间夺得 $+10.0$ 成功大奖，**净收益达 $+10.9$**；
    - 减速退出收益（$+10.9$）是晃悠赖分（$+2.0$）的 **5 倍以上**，彻底杜绝策略故意不减速的刷分行为！
  - **效果**：解冻后的 SRU 与 Attention 在纯到点任务下稳固空间表征；建立“入圈主动平稳刹车”的运动习惯，为 Phase 2 开启朝向调整打下完美基础。
- **Phase 2 (进度 16.7% ~ 75%，1000 ~ 4500 iters)**：
  - **核心目标**：开启姿态探索与原地调头学习。
  - **参数设置**：激活朝向密集奖励 `align_weight = 2.0`；保持温和终末结算 `target_heading_terminal = 0.3333`（成功对齐额外净奖励 $+10.0$，超时扣除 $-5.0$）；持续施加 `velocity_linear_penalty = -2.0`。
  - **终止判定**：标准到点调姿门限（$d < 0.4\text{m}$，$|\Delta\psi| < 0.35\text{ rad} \approx 20^\circ$），固定给足 `fixed_dwell_time = 5.0s`（50 步）充裕调姿窗口。
  - **效果**：航向角误差从随机分布快速收敛至 $0.3 \sim 0.5\text{ rad}$。
- **Phase 3 (进度 75% ~ 100%，4500 ~ 6000 iters)**：
  - **核心目标**：严苛收敛考核与精准高确定性停驻。
  - **参数设置**：保持 `align_weight = 2.0`，全面激活终末大奖与超时处决 `target_heading_terminal = 1.0`（净奖励 $+30.0$ / 超时处决 $-15.0$），`velocity_linear_penalty = -2.0`。
  - **终止判定**：双重标准极度收紧至 $d < 0.3\text{m}$ 且 $|\Delta\psi| < 0.15\text{ rad} (\approx 8.6^\circ)$，停留窗口收敛至 `4.0s`。
  - **效果**：倒逼策略在到达目标时干净利索地刹车对准，彻底消除贴靠晃动。

### 4.8 自动监控与 600 轮无缝接力守护进程 (Auto-Transition Daemon)
为了不打断当前正在进行的 Phase 1 训练，部署了专用守护进程 [`IsaacLab/auto_transition_at_600.py`](file:///home/yichangfeng/sru-navigation-learning/IsaacLab/auto_transition_at_600.py)：
1. 实时轮询当前训练目录（`2026-11-04_13-31-19_mdpo_b2w_heading_finetune`）；
2. 当第 600 轮训练完成且 `model_600.pt` 写入落盘后，守护进程自动安全终止原训练进程（PID 2798930）；
3. 校验并等待 GPU 显存完全释放后，自动以 `model_600.pt` 为检查点拉起后续 5400 轮（覆盖 Phase 1.5 $\to$ Phase 2 $\to$ Phase 3）微调任务，日志重定向至 `IsaacLab/logs/phase1_5_to_end.log`。

### 4.9 目标随机化与 360° 无障碍旋转空间保障 (Clearance & Goal Randomization)
- **避免贴墙死局**：四轮足机器人原地旋转对齐需要至少半径 $0.7 \sim 0.8\text{m}$ 的无碰撞旋转包络。若目标生成在死角或贴墙处，机器人转向必然撞墙，导致策略被迫放弃转向。
- **实现方式**：在 [`navigation/mdp/navigation/goal_commands.py`](file:///home/yichangfeng/sru-navigation-learning/IsaacLab/source/isaaclab_nav_task/isaaclab_nav_task/navigation/mdp/navigation/goal_commands.py) 中，当检测到朝向控制激活时，目标点位置采样强制限定在具有充足离墙净距的 `spawn_mask` 区域（安全边缘 $>0.8\text{m}$），确保机器人拥有完整的 360° 无障碍转向通道。

### 4.10 IsaacLab `dt` 稀释补偿 (Terminal Reward dt-Scaling Compensation)
- **机理**：IsaacLab `RewardManager` 默认会对环境的所有奖励项乘以仿真步长 $\Delta t = 0.1\text{s}$。原本设定的 $+30.0$ 终末大奖会被缩放成 $+3.0$，导致无法压倒每步 $+1.5 \times 0.1$ 的位置挂机分。
- **修复**：在 [`target_heading_terminal_outcome`](file:///home/yichangfeng/sru-navigation-learning/IsaacLab/source/isaaclab_nav_task/isaaclab_nav_task/navigation/mdp/rewards.py) 内部自动除以 `env.step_dt` 进行补偿，保证策略实际净获得的结算分值严格为真实的 $+30.0$ 和 $-15.0$。

---

## 5. 关键训练指标理想变化与监控手册

### 5.1 各训练阶段核心指标理想演变规律 (Phase-by-Phase Expected Metrics)

下表总结了在四阶段课程机制下，各项关键指标在健康/理想状态下的数值区间与演变趋势：

| 监控指标 (TensorBoard 路径) | Phase 1 (0~600 iters) <br> **复习与输入适应 (冻结)** | Phase 1.5 (600~1000 iters) <br> **全解冻梯度适应 (无朝向)** | Phase 2 (1000~4500 iters) <br> **姿态探索 (密集奖励)** | Phase 3 (4500~6000 iters) <br> **严苛收敛与终末结算** | 异常危险信号 (Red Flags) |
|:---|:---|:---|:---|:---|:---|
| **`Metrics/robot_goal/success_rate`** | **快速回升至 85% ~ 95%**<br>(仅考核 $d<0.4\text{m}$) | **保持在 85% ~ 92% 高位**<br>(SRU 解冻适应，无新任务干扰) | **稳定在 80% ~ 90%**<br>(开始原地调头，轻微抖动) | **初期回踩至 50%~60%，随后拉升至 75% ~ 85%+**<br>(收紧至 $0.3\text{m}, 8.6^\circ$) | Phase 1/1.5 跌破 60% (遗忘导航)；Phase 3 跌破 30% 且无法回升 |
| **`Metrics/robot_goal/final_yaw_error`** | **维持在 ~1.57 rad (90°)**<br>(完全随机均匀分布基线) | **维持在 ~1.57 rad (90°)**<br>(完全不施加朝向偏好) | **从 1.57 rad 单调下降至 0.3 ~ 0.5 rad (17°~28°)** | **逼入核心精度带：< 0.15 rad (8.6°)**<br>平均值收敛在 0.08 ~ 0.12 rad | Phase 2 末期仍高于 1.0 rad (未能学会利用航向输入) |
| **`Episode_Reward/target_heading`** | **恒等于 0.0**<br>(`align_weight=0`) | **恒等于 0.0**<br>(`align_weight=0`) | **由 0.5~1.0 快速上升至 1.6 ~ 1.9**<br>(满分 2.0，反映对齐质量提升) | **维持在 1.7 ~ 1.95 高位** | 开启后长期停留在 0.5 (随机朝向挂机) |
| **`Episode_Reward/target_heading_terminal`** | **恒等于 0.0**<br>(未激活) | **温和结算：+3.0 ~ +8.0**<br>(`weight=0.33`，净值 $+10/-5$) | **温和结算：+3.0 ~ +8.0**<br>(`weight=0.33`，净值 $+10/-5$) | **初期约 +5 ~ +15，最终收敛至 +22.0 ~ +28.0**<br>(反映大部分 episode 拿下 +30) | 持续为负值 (< 0，说明大量超时被处决) |
| **`Episode_Reward/velocity_linear_penalty`** | **恒等于 0.0**<br>(未激活) | **由 -0.5 上升至 -0.05**<br>(快速学会入圈减速刹车) | **稳定在 -0.05 ~ 0.0**<br>(圈内保持原地自旋，无额外惩罚) | **保持在 -0.05 ~ 0.0**<br>(原地精准自旋对齐) | 持续较负 (<-0.5，说明仍在高速晃悠) |
| **`Train/mean_episode_length`** | **稳定在 50 ~ 80 步**<br>(直奔终点入圈即成功结算) | **稳定在 50 ~ 80 步**<br>(保持敏捷奔跑直达终点) | **略微延长至 80 ~ 110 步**<br>(奔跑 50~70 步 + 原地调姿 20~40 步) | **平稳缩短至 70 ~ 90 步**<br>(调姿熟练，快速触发早终止) | 持续等于 episode 上限 (如 200 步，说明卡死或赖圈) |
| **`Loss/surrogate` (Policy Loss)** | **极小且平稳 (-0.01 ~ +0.01)** | **解冻瞬间有小脉冲后平复** | **平稳推进** | **在 Phase 3 切入时因终末大奖短期波动** | 解冻后出现 NaN、Inf 或剧烈发散 (> 1.0) |
| **`Loss/value_function` (Critic Loss)** | **平滑下行** | **平滑下行或适度波动** | **随动作探索轻微上升后收敛** | **因引入终末结算有明显脉冲，随后快速平滑** | 持续剧烈震荡无法收敛 |
| **`Policy/entropy`** | **平缓下降 (0.002 -> 0.0018)** | **平缓下降 (0.0018 -> 0.0015)** | **继续下降 (0.0015 -> 0.0008)** | **收敛至下限 (0.0005)**<br>(策略行为高度确定化) | 提前跌至 0 (过早失去探索能力) |

---

### 5.2 如何监控训练过程 (Monitoring Guide)

#### 方法 1：TensorBoard Web 仪表盘（核心监控手段）
在服务器后台启动 TensorBoard：
```bash
tensorboard --logdir IsaacLab/logs/rsl_rl/b2w_navigation_heading_mdpo --port 6006
```
本地浏览器访问 `http://localhost:6006`，建议重点关注并收藏以下 Panels：
1. **Nav Performance 组（看成功率与精度）**：
   - `Metrics/robot_goal/success_rate`
   - `Metrics/robot_goal/final_yaw_error`
   - `Metrics/robot_goal/dist_to_goal`
2. **Reward Balance 组（看各奖励项是否处于健康博弈）**：
   - `Episode_Reward/target_heading`（朝向密集过程分，期望 1.6~1.9）
   - `Episode_Reward/target_heading_terminal`（终末大奖净值，Phase 2 期望 +3~+8，Phase 3 期望 +20 以上）
   - `Episode_Reward/velocity_linear_penalty`（入圈刹车惩罚，期望平稳趋近 0.0）
   - `Episode_Reward/reach_goal_xy_tight`（到点位置分）
3. **Training Dynamics 组（看策略收敛与稳定性）**：
   - `Train/mean_episode_length`（健康区间 70 ~ 100 步）
   - `Loss/surrogate` 与 `Loss/value_function`
   - `Policy/entropy`

#### 方法 2：终端实时进度输出 (CLI Log Stream)
训练过程中，终端每隔若干 iterations 会打印训练摘要。重点观察：
- **`[OnPolicyRunner] Partial freezing enabled`**：确保第 0 步成功提示冻结生效。
- **`[OnPolicyRunner] Unfreezing Depth/Attention/SRU layers at iteration 600`**：确保在第 600 步成功提示全解冻。
- **Mean Reward 与 Mean Episode Length**：观察每 100 iters 的平滑变化。

#### 方法 3：关键故障排查红线与应急处置
- **红线 1：Phase 1 到点成功率跌破 60%**
  - **可能诱因**：Checkpoint 加载失败，或者学习率过大破坏了原有网络。
  - **对策**：检查启动日志中是否有 `[INFO] Loading checkpoint from: ...`，确认 `learning_rate = 3e-5`。
- **红线 2：Iter 2000 解冻瞬间 Loss 爆炸 / 报错**
  - **可能诱因**：Muon 优化器对解冻参数的动量缓存初始化不平滑。
  - **对策**：检查 `muon.py` 中的 `SingleDeviceMuonWithAuxAdam` 是否已打上 `p.grad is None` 补丁。
- **红线 3：Phase 3 步长持续等于上限（200步）且终末奖励长期为负（-15）**
  - **可能诱因**：机器人虽然进圈，但无法在 0.15 rad 精度内停稳，持续被判定超时处决。
  - **对策**：检查目标采样是否卡在墙角；若无贴墙，可适当将 Phase 3 的对齐阈值放宽至 `0.20 rad`（约 11.5°）。

---

## 6. 训练启动与常用命令

### 6.1 运行配置自动化断言
```bash
cd /home/yichangfeng/sru-navigation-learning/IsaacLab
CONDA_PREFIX="/home/yichangfeng/miniforge3/envs/sru-learning" \
PATH="/home/yichangfeng/miniforge3/envs/sru-learning/bin:$PATH" \
TERM=xterm ./isaaclab.sh -p source/isaaclab_nav_task/scripts/verify_refactor.py --headless
```

### 6.2 一键启动正式微调
```bash
cd /home/yichangfeng/sru-navigation-learning/IsaacLab
./train_b2w_heading_finetune.sh
```
若需要自定义环境并行数或总步数（默认 6000 iters）：
```bash
NUM_ENVS=1024 MAX_ITER=6000 GUI=0 ./train_b2w_heading_finetune.sh
```

---

## 7. 常见维护问题排查 (FAQ)

- **Q1: 训练启动报 CUDA Out of Memory (OOM)?**
  - 原因：上一轮训练或仿真后台进程（Python / Isaac Sim Kit）未完全释放显存。
  - 解决：`fuser -v /dev/nvidia*` 查看占用进程 PID，执行 `kill -9 <PID>` 彻底释放后重启。
- **Q2: 后续实机部署（Sim-to-Real）时如何对齐？**
  - 实机部署代码在输入给策略网络时，必须将本体感知特征拼接为 18 维：
    第 16 维为 $\cos(\Delta\psi)$，第 17 维为 $\sin(\Delta\psi)$，其中 $\Delta\psi$ 为机器人基体机头与目标终点航向的平面夹角。

# 训练命令速查与 TensorBoard 指标解读

> 合并自原 `CLI_REFERENCE.md` 与 `TENSORBOARD_METRICS_GUIDE.md`。
> 任务设计原理见 `docs/structure/NAVIGATION_MDP.md`；训练流程与超参见 `docs/structure/go2w_nav_refactor_plan.md`。
> 奖励权重以代码 `navigation_env_cfg.py` 为准（Go2W 与 B2W 有差异，见 §3 表下注解）。

---

## 第一部分：常用命令速查

### 1. TensorBoard（查看训练曲线）

```bash
# 查看所有 RL 训练日志（推荐）
tensorboard --logdir IsaacLab/logs/rsl_rl --port 6006

# 只看某个任务的曲线，例如 MDPO
tensorboard --logdir IsaacLab/logs/rsl_rl/go2w_navigation_mdpo --port 6006

# 只看某一次具体 run
tensorboard --logdir IsaacLab/logs/rsl_rl/go2w_navigation_mdpo/2026-10-22_09-33-21_stage1_flat --port 6006
```

打开浏览器访问 `http://localhost:6006`。

**端口被占用**：换端口即可

```bash
tensorboard --logdir IsaacLab/logs/rsl_rl --port 6007
```

**远程机器（SSH）**：本地终端做端口转发

```bash
ssh -L 6006:localhost:6006 user@remote-host
# 然后本地浏览器打开 http://localhost:6006
```

**后台运行 + 指定 host（允许局域网访问）**

```bash
tensorboard --logdir IsaacLab/logs/rsl_rl --host 0.0.0.0 --port 6006 &
```

### 2. 日志目录结构

```
IsaacLab/logs/rsl_rl/
├── b2w_navigation_ppo/
│   └── 2026-10-19_14-16-10/
│       └── events.out.tfevents.*
├── go2w_navigation_mdpo/
│   ├── 2026-10-21_11-10-00/
│   └── 2026-10-22_09-33-21_stage1_flat/
└── go2w_navigation_finetune/
    ├── 2026-10-21_16-17-46_from_nav_policy_new/
    └── 2026-10-21_16-46-38_warmup_ppo/
```

每个子目录对应一次训练 run，文件夹名是 `YYYY-MM-DD_HH-MM-SS` 时间戳。

### 3. WandB（如果启用了）

```bash
wandb login
wandb offline   # 离线模式，不上传
ls IsaacLab/wandb/   # 本地查看 wandb 日志（无需联网）
```

### 4. 训练相关

```bash
# 激活 conda 环境
conda activate sru_nav_learning

# 进入 IsaacLab 目录
cd IsaacLab

# 运行导航训练脚本（示例，具体参数看各任务 README）
python scripts/rsl_rl/train.py --task Go2W-Navigation-MDPo
```

Go2W 的 5-Phase 课程训练命令见 `docs/structure/go2w_nav_refactor_plan.md`（`train_go2w_scratch.sh` / `continue_go2w.sh`）。

### 5. 其他常用

```bash
nvidia-smi                # 查看 GPU 占用
pkill -f tensorboard     # 杀掉占用的 tensorboard 进程
lsof -i :6006             # 查看某端口被谁占用
```

---

## 第二部分：TensorBoard 指标全解析

涵盖 IsaacLab + RSL-RL（MDPO/PPO）训练框架下 TensorBoard 面板记录的全部关键指标：物理含义、代码定义、健康区间与异常诊断。

### 1. 核心速度指标对比

这两个指标共同描绘机器狗在二维平面上的运动效率与意图对齐度。

```
           目标点 G (Goal)
              ^
              |  方向向量 u_goal
              |
              |     / 实际速度向量 v_2d (模长: velocity_magnitude)
              |    /
              |   /  夹角 θ
              |  /
              | /
              +----------------->
            机器狗当前位置 R
              |
              +---> 投影长度: velocity_toward_goal = |v_2d| * cos(θ)
```

#### 1.1 `Metrics/robot_goal/velocity_magnitude`（全向运动合速度）

- **物理含义**：机器狗质心在世界坐标系水平地面（X-Y 平面）上的**线速度绝对大小（模长）**。
- **代码定义**（`goal_commands.py`）：`velocity_magnitude = ‖v_2D‖ = √(v_x² + v_y²)`，单位 m/s。
- **理想状态**：
  - 初期（0~30 轮）：0.3 ~ 0.5 m/s（盲目探索或抖动）；
  - 中期与收敛期：稳定在 0.4 ~ 0.8 m/s（受限于 `policy_scaling`，平均合速度 0.5 ~ 0.7 m/s 属优良步态）。
- **异常诊断**：< 0.1 m/s = 卡死或原地瘫倒；> 1.8 m/s = 轮部打滑或物理飞车爆散。

#### 1.2 `Metrics/robot_goal/velocity_toward_goal`（向目标有效投影速度）

- **物理含义**：水平速度向量在**目标点视线连线方向上的标量投影（有效位移速率）**。
- **代码定义**：`u_goal = (p_goal − p_robot)/‖·‖`，`velocity_toward_goal = v_2D · u_goal = ‖v_2D‖ cos(θ)`。
- **符号含义**：正数 = 正在逼近目标；负数 = 背离/倒车；为零 = 静止或绕目标做切向圆周运动。
- **理想状态**：初期 −0.05 ~ 0.05 m/s；收敛期**持续稳定正值 0.05 ~ 0.25 m/s**。
  - 为什么平均只有 0.1 左右而不是 1.0？因为快到终点减速、转向调整、避障绕行、停在目标圈内时投影速度都降为 0，
    全局平均在 0.05 ~ 0.15 m/s 即代表高效率直扑目标。

#### 1.3 两者结合分析（导航运动效率比 η）

`η = velocity_toward_goal / velocity_magnitude = cos(θ)`

- magnitude 很大但 toward_goal ≈ 0 → 高速画圆或无头乱撞；
- 两者相近且均为正 → 笔直冲向目标，轨迹高效。

### 2. 导航任务表现指标（`Metrics/robot_goal/*`）

| 指标名称 | 含义 | 理想状态 |
| :--- | :--- | :--- |
| `success_rate` | 滑动窗口内触发终点半径（0.5 m）的回合占比 | 随训练上升，收敛期应显著 > 0（论文级 > 50~90%，视任务/阶段而定） |
| `velocity_magnitude` | 质心水平合速度 | 0.4 ~ 0.8 m/s |
| `velocity_toward_goal` | 向目标有效投影速率 | 正数，稳定 0.05 ~ 0.25 m/s |

### 3. 奖励项详解（`Episode_Reward/*`）

> TensorBoard 中展示的数值是该奖励项**加权后的每步单回合均值**。
> 下列权重为当前代码配置；若你的 run 里出现 `progress` / `heading` / `reach_goal` / `time_cost` 等名字，
> 说明那是**重构前的旧奖励方案**（已废弃），不要按旧文档解读。

| 奖励项名称 | 权重 | 含义与设计目的 | 理想曲线形态 |
| :--- | :--- | :--- | :--- |
| `reach_goal_xy_tight` | **+1.5** | 到点附近强吸引（sigmoid 0.25，T_r 0.1），d→0 时峰值 **+15** | 与 `success_rate` 正相关，成功时抬升 |
| `reach_goal_xy_soft` | **+0.25** | 宽域平滑 shaping（sigmoid 2.5，T_r 1.0，3D 误差） | 长程缓慢正向抬升 |
| `lateral_movement` | **−0.1** | 惩罚侧向移动 | 稳定在 −0.005 ~ −0.02 |
| `action_rate_l1` | **−0.1** | 惩罚速度指令突变（平滑） | −0.01 ~ −0.05，越平缓越丝滑 |
| `rot_movement` | **−1e-5** | 惩罚原地自旋 | 极小负值（约 −1e-5） |
| `joint_acc_l2_joint` | **−1e-7** | 关节加速度阻尼（保护电机） | −1e-4 ~ −1e-6 |
| `episode_termination` | **−50.0** | 撞墙/跌落/翻滚等灾难性终止 | **趋近于 0**（撞击越少越好） |
| `backward_movement_penalty` | **−0.0**（B2W）/ **−1.0**（Go2W） | 惩罚倒车 | 接近 0 |

> 注解：B2W 当前把 `backward_movement_penalty` 设为 −0.0（关闭，仅保留课程钩子）；
> Go2W 配置为 −1.0（对齐 SRU-Odin 原版）。若两者数值不一致，先确认你跑的是哪个机型的配置。

### 4. 回合终止原因统计（`Episode_Termination/*`）

该组指标是排查策略失败原因的最直接依据（各项值表示该类终止在当前采集环境中的比例）：

| 终止项名称 | 触发物理条件 | 理想状态 | 异常排查提示 |
| :--- | :--- | :--- | :--- |
| `early_termination` | **成功到达目标**（进入 0.5 m 目标圈触发） | 越高越好 | **注意：此项在代码中代表“成功通关”！** |
| `time_out` | 超时（300 步 / 60 秒仍未抵达） | 随成功率上升而下降 | 若高居不下，说明卡死或迷失在迷宫中 |
| `base_contact` | 碰撞（base / hip / thigh 受到 > 1.0 N 撞击） | 低 | 偏高说明感知避障不足，撞墙或绊倒在台阶上 |
| `large_pitch_angle` | 翻倒（俯仰/侧倾 > 40°） | 0 | 出现正值说明下层底盘失稳或斜坡后仰侧翻 |
| `terrain_fall` | 跌落坑洞（低于坠落阈值） | 0 | 第一阶段（平地）应恒为 0；坑洞阶段需重点监控 |

### 5. 优化与损失函数指标（`Loss/*`）

| 损失项名称 | 算法含义 | 理想状态 |
| :--- | :--- | :--- |
| `value_function` | 价值网络（Critic）对未来总折扣回报的 MSE | 先升后降、平缓收敛，不发散即可 |
| `surrogate` | PPO 策略网络（Actor）的重要性采样裁剪目标损失 | 在 0 附近微幅波动（−0.01 ~ +0.01） |
| `kl_divergence` | **MDPO 专属**：Policy 1 与 Policy 2 的双向 KL 散度 | 稳定在 0.005 ~ 0.02；持续为 0 说明两策略塌缩，MDPO 失效 |
| `learning_rate` | 当前优化器学习率 | 按调度平滑下降（exponential 或 adaptive KL） |

### 6. 策略与探索诊断指标（`Policy/*`）

#### `Policy/mean_noise_std`（高斯动作分布标准差 σ）

- **物理含义**：Actor 在连续速度动作空间（v_x, v_y, ω_z）输出分布的随机探索噪声标准差。
- **理想曲线形态**：
  - 初期（0~50 轮）：从初始值 1.0 缓慢回落至 0.8 ~ 0.9；
  - 中期（50~500 轮）：维持 **0.6 ~ 0.85**，保持探索多样性；
  - 后期（1000+ 轮）：逐步收敛至 0.3 ~ 0.5，形成确定、精准的轨迹。
- **致命陷阱（警惕坍塌）**：前 100 轮断崖式下跌至 < 0.3，说明策略因缺乏正向反馈陷入“摆烂死锁”。
- **防护**：代码底层已加入 `log_std.clamp(min=-2.5, max=0.2)`，对应 σ ∈ [0.082, 1.221]，杜绝后期发散与早熟坍塌。

### 7. 训练概览与计算吞吐（`Train/*` & `Perf/*`）

#### 7.1 `Train/mean_episode_length`（平均单回合存活步数）

- **物理含义**：从复位到回合终止的平均交互步数（单步高层时间 Δt = 0.2 s）。
- **理想趋势**：
  - 第一阶段（0~10 轮）：极低（10~30 步），大量出生即碰撞/失稳死亡；
  - 第二阶段（10~30 轮）：大幅拉长（200~280 步），学会站立求生但还不会去终点，大量超时；
  - 第三阶段（30+ 轮，成功率爆发）：**迅速缩短并稳定**——步数缩短意味着最快速度直扑终点提前通关。

#### 7.2 `Train/mean_reward`（单回合累积总回报）

随 `success_rate` 提升，从早期负数（碰撞惩罚主导）一路单调上升。

#### 7.3 `Perf/total_fps`（仿真与训练吞吐速率）

物理仿真 + 神经网络前后向每秒处理的总步数。1024 并行环境下 RTX 4090 通常 1400 ~ 1800 FPS。

### 8. 训练阶段健康曲线速查表

| 指标 (Tag) | 初始阶段 | 攀升阶段 | 收敛稳定期 | 异常告警线 |
| :--- | :--- | :--- | :--- | :--- |
| `success_rate` | 低 | 快速上升 | 高位平稳 | 长期不涨且 `closest_distance` 不缩小 |
| `time_out` | 高 | 下降 | 低 | 持续高（死锁） |
| `mean_episode_length` | 短（早死）→ 长（超时） | 逐步缩短 | 稳定在较短区间 | 恒等于 300 步（全超时） |
| `velocity_toward_goal` | ≈ 0 或负 | 转正 | 持续正 | 持续负值（背道而驰） |
| `mean_noise_std` | 1.0 → 0.9 | 0.9 → 0.8 | 0.6 ~ 0.8 | < 0.3（过早坍塌） |
| `kl_divergence`（MDPO） | 0.02 ~ 0.05 | 0.01 ~ 0.02 | < 0.015 | > 0.1（双策略严重分歧） |

---

## 第三部分：典型失败模式排查

| 现象 | 可能原因 | 排查方向 |
|---|---|---|
| 长期成功率不涨、`closest_distance` 不缩小 | 深度特征未真正接入 / γ 被改小 / 目标 Δ 噪声过大 | 检查 VAE 权重路径、`gamma`、`DeltaTransformationNoiseCfg` |
| `mean_noise_std` 前 100 轮断崖下跌 | 策略缺乏正向反馈，方差坍塌 | 确认 `log_std` 钳制存在；检查奖励门控是否过疏 |
| `Train/mean_reward` 峰值后单调下跌 + 动作剧烈抖动 | 学习率未闭环 / 方差发散 | 确认 `schedule` 配置；确认 `action_rate_l1`、`lateral_movement` 为 −0.1（不是 −0.02） |
| MDPO `KL` 持续为 0 | 两策略塌缩成同一份 | 检查互蒸馏损失是否生效、`distill_coef` |
| 成功但动作抖动 | 低通滤波 α 过大 / 动作尺度随机化过宽 | 检查 `randomize_low_pass_filter_alpha`、`policy_scaling` |

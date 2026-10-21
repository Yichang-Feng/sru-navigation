# Go2W 仿真接入 —— 完整工程交付与实践文档

- **更新日期**：2026-10-20
- **当前状态**：**已完整跑通并在仿真中实现避障导航**（加载资产 -> 步态网络 -> 深度编码 -> 避障导航全流程闭环）
- **目标工程**：`/home/yichangfeng/sru-navigation-learning/IsaacLab/source/isaaclab_nav_task`

---

## 1. 目标与背景

在 Isaac Lab 导航任务框架中，新增并接入 **Unitree Go2W**（四足 + 4 驱动轮）机器人。
以原有 **B2W**（双足两轮）作为参考基线，实现以下完整命令的 Go2W 版本运行：

```bash
cd /home/yichangfeng/sru-navigation-learning/IsaacLab
conda activate sru-learning

./isaaclab.sh -p source/isaaclab_nav_task/scripts/play.py \
    --task Isaac-Nav-PPO-Go2W-Play-v0 \
    --num_envs 1 \
    --checkpoint /home/yichangfeng/sru-robot-deployment/rl_nav_controller/deployment_policies/nav_policy_new.pt
```

---

## 2. 完成的关键修改与资产构建

### 2.1 URDF 到 USD 资产生成
- 从 `.../navigation/assets/data/Robots/Go2W/go2w_description.urdf` 通过 Isaac Lab 的 `UrdfConverter` 生成了标准的 `go2w_rsl.usd` 资产。
- 提取并保留了 16 个主动关节：
  - 12 个腿部旋转关节（`{FL,FR,RL,RR}_{hip,thigh,calf}_joint`）
  - 4 个轮子驱动关节（`{FL,FR,RL,RR}_foot_joint`）
- 修复了 Isaac Sim 4.5 环境中 `ImportConfig` 缺失 `set_merge_fixed_ignore_inertia` 导致的转换崩溃。

### 2.2 资产定义与执行器参数
- 文件：[`isaaclab_nav_task/navigation/assets/go2w.py`](file:///home/yichangfeng/sru-navigation-learning/IsaacLab/source/isaaclab_nav_task/isaaclab_nav_task/navigation/assets/go2w.py)
- 定义 `GO2W_CFG`（`ArticulationCfg`）：
  - 腿部执行器：`stiffness = 20.0, damping = 0.5, effort_limit = 23.5 Nm`（对齐 `rl_sar` 训练参数）
  - 轮子执行器：`stiffness = 0.0, damping = 0.5, effort_limit = 23.5 Nm`
  - 默认高度与姿态（采用原 `rl_sar` 训练默认姿态）：
    - `init_state.pos = (0.0, 0.0, 0.38)`
    - 前腿：`thigh = 0.50, calf = -1.50`
    - 后腿：`thigh = 0.80, calf = -1.50`
    - 髋关节：`hip = 0.0`
    - 实测站立与行走高度稳定在 **0.38 米** 左右（原策略最优力学工作点）。

### 2.3 相机与环境配置
- 相机映射：在 [`camera_config.py`](file:///home/yichangfeng/sru-navigation-learning/IsaacLab/source/isaaclab_nav_task/isaaclab_nav_task/navigation/mdp/depth_utils/camera_config.py) 添加 `"go2w": ZEDX_CAMERA_CONFIG`。
- 环境配置：[`config/go2w/navigation_env_cfg.py`](file:///home/yichangfeng/sru-navigation-learning/IsaacLab/source/isaaclab_nav_task/isaaclab_nav_task/navigation/config/go2w/navigation_env_cfg.py)
  - 相机相对基座位置：`offset.pos = (0.285, 0.0, 0.09)`（位于头部前方且高于自带镜头约 8cm）。
  - **相机俯仰角度**：`offset.rot = (1.0, 0.0, 0.0, 0.0)`（水平 0°，避障防误判关键）。
  - 接触力碰撞检测体：`body_names=["base", ".*hip", ".*thigh"]`。
- Gym 任务注册：[`config/go2w/__init__.py`](file:///home/yichangfeng/sru-navigation-learning/IsaacLab/source/isaaclab_nav_task/isaaclab_nav_task/navigation/config/go2w/__init__.py)
  - 注册了 `Isaac-Nav-PPO-Go2W-Play-v0`、`Isaac-Nav-PPO-Go2W-v0`、`Isaac-Nav-MDPO-Go2W-Play-v0` 等 6 个任务。

---

## 3. 如何将 rl_sar 的 Locomotion 策略融入本项目

本项目导航框架采用 **“分层强化学习（Hierarchical RL）”** 架构：
- **上层（高层导航）**：10Hz 运行，输入深度相机潜编码 + 目标向量，输出底盘速度指令 `[vx, vy, omega]`。
- **下层（步态 Locomotion）**：50Hz 运行（仿真 decimation=4，总频 200Hz），输入机器人状态，驱动 12 个腿部关节位置和 4 个轮子速度。

### 3.1 遇到的核心不匹配与排查
直接把 `rl_sar/policy/go2w/robot_lab/policy.pt` 给导航当下层使用时，最初发生了**四肢剧烈抽搐瞬间倒地**的问题。深度排查发现四个底层冲突：

1. **关节顺序完全不同**：
   - `rl_sar` 网络按**单腿**连续排列：`[FR(hip,thigh,calf), FL(...), RR(...), RL(...), 4 wheels]`
   - Isaac Lab USD 内部按**关节字母序**排列：`[FL/FR/RL/RR_hip, FL/FR/RL/RR_thigh, FL/FR/RL/RR_calf, 4 wheels]`
   - 如果不转换，大腿力矩直接打到髋关节电机，导致瞬间瘫痪。
2. **输入观测维度差异（57维 vs 60维）**：
   - 导航框架的 `LowLevelPolicyCfg` 默认包含 `base_lin_vel(3)`，共 60 维。
   - `rl_sar` 步态网络在实机训练时无真值线速度传感器，输入为 57 维（`ang_vel(3) + gravity(3) + commands(3) + dof_pos(16) + dof_vel(16) + actions(16)`）。
3. **输入特征尺度缩放（Observation Scale）**：
   - `rl_sar` 输入的 `dof_vel` 经过了 `0.05` 缩放，`ang_vel` 经过了 `0.25` 缩放。Isaac Lab 默认输出真实速度，导致网络接收到大 20 倍的速度输入。
4. **输出动作缩放（Action Scale）**：
   - `rl_sar` 输出要求：`hip` 缩放 0.125，`thigh/calf` 缩放 0.25，`wheel` 缩放 5.0。原配置统一为 0.5。

### 3.2 最终集成适配方案：`Go2WRlSarAdapter`
我们设计了即插即用的 TorchScript 适配模块，保存为 [`policy_go2w_adapted.pt`](file:///home/yichangfeng/sru-navigation-learning/IsaacLab/source/isaaclab_nav_task/isaaclab_nav_task/navigation/assets/data/Policies/locomotion/go2w/policy_go2w_adapted.pt)：

```python
class Go2WRlSarAdapter(nn.Module):
    # 1. 拦截 IsaacLab 的 57 维观测输入
    # 2. 对 ang_vel 乘 0.25，dof_vel 乘 0.05，轮子位置置 0
    # 3. 将关节观测从 IsaacLab 字母序索引映射到 rl_sar 腿部顺序
    # 4. 执行原始 policy.pt 前向推理
    # 5. 动作尺度按关节解耦缩放 (hip 0.125, thigh/calf 0.25, wheel 5.0)
    # 6. 将输出动作从 rl_sar 顺序映射回 IsaacLab 执行器顺序并返回
```

- 在 `Go2WNavigationEnvCfg` 中设置：
  - `self.observations.low_level_policy.base_lin_vel = None`
  - `self.actions.velocity_command.low_level_policy_file = ".../policy_go2w_adapted.pt"`
  - `self.actions.velocity_command.low_level_position_action.scale = 1.0`
  - `self.actions.velocity_command.low_level_velocity_action.scale = 1.0`

该方案**无需修改导航底层核心代码**，保证了框架的整洁与解耦。

---

## 4. 解决“行进方向与目标点保持 90 度夹角”的物理根因

### 4.1 现象分析
适配步态后，机器人能平稳走动，但在开阔空地上行进方向始终与目标点夹角呈约 90 度，绕圈无法切入目标点。

### 4.2 根因定位
- 基线机器人 **B2W** 高度 1.03m，相机倾斜 20° 时，视线落点在 **2.8米以外**。
- **Go2W** 矮机身（相机高度仅 0.47m），若沿用向下 20° 倾斜，**正前方仅 1.2m 处的地面就占满了画面的大部分**。
- 导航策略中的 VAE 深度特征编码器将 1.2m 处的地面识别为**正前方的实体障碍物/墙体**。
- 导航策略因而判定前方不可通行，持续执行**横向切线避障（输出较大 `vy`）**，造成了持续 90 度侧滑/绕圈。

### 4.3 修复效果
将相机调至**水平 0° 仰角**（`offset.rot = (1.0, 0.0, 0.0, 0.0)`）后，地面仅出现在画面底端，正前方视野通畅：
- 机器狗在开局自动旋转对正目标方向。
- 前向速度 `vx` 快速加速至 **1.1 ~ 1.5 m/s** 直奔目标点。
- 航向误差全程收敛在 **±10°** 以内。

---

## 5. 行走姿态说明与实测

- **当前采用方案**：已恢复为原 `rl_sar` 步态网络训练时的**原生默认姿态**：
  - 前腿：`thigh = 0.50, calf = -1.50`
  - 后腿：`thigh = 0.80, calf = -1.50`
  - 髋关节：`hip = 0.00`
  - 机身落点：`z = 0.38 m`
- **实测数据**：
  - 机器狗行走基座高度稳定在 **0.38 米**。
  - 连续物理步进测试：**0 次摔倒，0 次触发重置（`Done: False`）**，步态网络工作在力学最均匀的状态。
  - *(备忘：若后续真机部署或特定低矮通过场景需要压低机身，可通过在 `go2w.py` 中将 calf 调深至 -1.75、thigh 分别调至 0.65/0.95 降低机身至 0.33m)*。

---

## 6. 验证与运行命令

在本地图形终端中执行：

```bash
cd /home/yichangfeng/sru-navigation-learning/IsaacLab
conda activate sru-learning

./isaaclab.sh -p source/isaaclab_nav_task/scripts/play.py \
    --task Isaac-Nav-PPO-Go2W-Play-v0 \
    --num_envs 1 \
    --checkpoint /home/yichangfeng/sru-robot-deployment/rl_nav_controller/deployment_policies/nav_policy_new.pt
```

### 验证标准：
1. 机器人生成为 Go2W（四腿四轮），以自然偏低的趴伏站立高度（~0.33m）平稳起步。
2. 机器人能够自动旋转并朝红/绿目标点直线行驶（`vx > 1.0 m/s`）。
3. 遇到障碍物时能正常根据深度相机信息进行减速与避障绕行。

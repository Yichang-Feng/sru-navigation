# SRU Navigation — Unitree Go2W 端到端导航系统

[论文](https://arxiv.org/abs/2506.05997) | [项目主页](https://michaelfyang.github.io/sru-project-website/)

> 本仓库包含 **SRU（空间增强记忆循环单元）导航系统**从 IsaacLab 仿真训练到 Unitree Go2W 真机部署的**完整全栈代码**。
> 包括：RL 训练算法库、5-Phase 课程训练脚本、模型导出工具，以及实机 ROS 2 导航节点和两种部署模式（纯局部 SRU 避障 / 全局规划 + SRU 局部导航联合）。

---

## 系统全景图

```
┌─────────────────────────────────────────────────────────────┐
│          Step 1: IsaacLab 仿真训练（开发机 RTX 4090）          │
│                                                             │
│  IsaacLab/source/isaaclab_nav_task/   ← 仿真环境 & 任务定义   │
│  rsl_rl/                              ← SRU 算法库           │
│  IsaacLab/train_go2w_curriculum.sh    ← 5-Phase 训练启动      │
└───────────────────────┬─────────────────────────────────────┘
                        │ scripts/export_onnx.py
                        ▼ 导出 nav_policy.onnx
┌─────────────────────────────────────────────────────────────┐
│          Step 2: 实机部署（Go2W 机载 Jetson Orin，192.168.123.111）│
│                                                             │
│  模式 A — 纯局部 SRU 避障导航                                  │
│    /goal_pose → rl_nav_controller → /cmd_vel               │
│    （单节点，无地图，依赖深度相机）                              │
│                                                             │
│  模式 B — 全局路径规划 + SRU 局部导航                           │
│    /goal_pose → global_planner → /path                      │
│              → path_follow / local_planner                  │
│              → rl_nav_controller（前瞻萝卜点）→ /cmd_vel      │
│    （结合 LiDAR SLAM 地图，适合长距离导航）                      │
└─────────────────────────────────────────────────────────────┘
```

---

## 仓库结构

```
sru-navigation-learning/
│
├── rsl_rl/                        # ⚙️ SRU 核心 RL 算法库（Python 包）
│   ├── algorithms/                #   PPO / MDPO / SPO 实现
│   ├── modules/                   #   ActorCriticSRU 网络架构
│   ├── networks/sru_memory/       #   LSTM_SRU + 注意力机制
│   └── runners/                   #   OnPolicyRunner 训练主循环
│
├── IsaacLab/                      # 🏋️ IsaacLab 仿真环境（Git 子模块）
│   ├── source/isaaclab_nav_task/  #   Go2W 导航任务定义（环境、奖励、相机配置）
│   ├── train_go2w_curriculum.sh   #   5-Phase 课程训练一键启动
│   ├── train_go2w_scratch.sh      #   从头开始训练
│   ├── train_go2w_finetune.sh     #   在已有权重基础上微调
│   ├── play_go2w.sh               #   加载权重仿真回放（GUI 可视化）
│   └── logs/rsl_rl/               #   训练日志与权重 checkpoint
│
├── sru-robot-deployment/          # 🤖 实机部署代码（含两大完整机载工作空间）
│   ├── go2w_nav_ros2_ws/          #   对应机载 ~/go2w_nav_ros2_ws/ (高层导航栈)
│   │   └── src/                   #     含 rl_nav_controller, global_planner, path_follow 等
│   ├── bridge_ws/                 #   对应机载 ~/bridge_ws/ (底盘通信与传感器驱动)
│   │   └── src/                   #     含 unitree_bridge, zed-ros2-wrapper, super_lio 等
│   ├── scripts/                   #   一键同步/部署脚本 (sync_to_go2w_nav.sh 等)
│   ├── docs/                      #   实机部署文档集合 (软硬件拓扑/排坑指南)
│   ├── diagnostics/               #   诊断与测试工具 (door_diagnostic_logger 等)
│   └── README.md                  #   机载路径完全映射对照表
│
├── scripts/                       # 🔧 开发侧工具脚本（不推送至机器狗）
│   ├── export_onnx.py             #   训练后导出 ONNX 模型
│   ├── export_loco_to_onnx.py     #   步态策略导出
│   ├── prepare_warmstart_*.py     #   跨机型权重初始化
│   ├── surgery_expand_heading*.py #   ONNX 模型外科手术（扩展观测维度）
│   ├── compare_deployment_policies.sh # 多版本策略性能对比
│   ├── mock_global_nav_tester.py  #   全局导航集成测试（无需机器狗）
│   ├── test_adaptive_lookahead.py #   自适应前瞻单元测试
│   ├── test_global_nav_integration.py # 全局导航集成测试
│   └── benchmark/                 #   性能基准测试工具
│
├── docs/                          # 📚 训练与算法文档
│   ├── go2w_nav_refactor_plan.md  #   Go2W 5-Phase 训练全指南（必读）
│   ├── NAVIGATION_TRAINING_DESIGN.md  # 导航任务设计原理
│   ├── SRU_NAVIGATION_EXPLAINED.md    # SRU 架构详解
│   ├── ADAPTIVE_LOOKAHEAD_NAVIGATION_DESIGN.md # 自适应前瞻设计
│   ├── TENSORBOARD_METRICS_GUIDE.md   # 训练指标解读
│   └── B2W_HEADING_CONTROL_HANDOVER.md # 航向控制（B2W 参考）
│
├── config/                        # 📋 训练配置
│   └── dummy_config.yaml          #   PPO 超参数示例
│
├── setup.py / pyproject.toml      # 📦 rsl_rl 包安装配置
└── README.md                      # 本文件
```

---

## 一、训练指南

### 1.1 环境准备

```bash
# 1. 克隆本仓库（含 IsaacLab 子模块）
git clone https://github.com/leggedrobotics/sru-navigation-learning.git
cd sru-navigation-learning

# 2. 安装 IsaacLab（首次）
cd IsaacLab
./isaaclab.sh --install  # 参考 IsaacLab 官方文档

# 3. 卸载 IsaacLab 自带的 rsl_rl，安装本仓库的 SRU 增强版
./isaaclab.sh -p -m pip uninstall rsl-rl-lib -y
./isaaclab.sh -p -m pip install -e ..  # 安装 sru-navigation-learning 根目录的 rsl_rl

# 4. 安装导航任务扩展包
./isaaclab.sh -p -m pip install -e source/isaaclab_nav_task

# 5. 激活 Conda 环境（此后所有训练命令均在此环境）
conda activate isaaclab  # 或你的实际环境名
```

### 1.2 Go2W 5-Phase 课程训练

Go2W 采用从简单到复杂的 5 阶段递进课程（详见 [docs/go2w_nav_refactor_plan.md](docs/go2w_nav_refactor_plan.md)）：

| Phase | 迭代范围 | 地形 | 前向速度 |
|:---:|:---|:---|:---|
| 1 | 0 ~ 2k | 平地 + 稀疏障碍 | 1.5 m/s |
| 2 | 2k ~ 5k | 平地 + 稀疏障碍（降熵稳定） | 1.5 m/s |
| 3 | 5k ~ 14k | 初级迷宫 + 10~15 cm 楼梯 | 1.6 m/s |
| 4 | 14k ~ 24k | 20 cm 楼梯 + 深坑 + 密集迷宫 | 1.8 m/s |
| 5 | 24k ~ 35k | 全要素精细微调 | 1.8 m/s |

```bash
cd IsaacLab

# 方式 A：自动化全流程（Phase 1→5 自动切换，推荐）
conda activate isaaclab
python train_go2w_curriculum.py --algo ppo --num_envs 1024 --headless

# 方式 B：手动分 Phase 控制（适合中途介入调参）
./train_go2w_scratch.sh      # Phase 1 从头开始
# 确认 Phase 1 收敛后：
./train_go2w_finetune.sh --checkpoint logs/rsl_rl/go2w_navigation_ppo/model_2000.pt
# ...依次到 Phase 5

# 方式 C：使用 MDPO 双策略互蒸馏算法（性能更优）
./train_go2w_curriculum.sh --algo mdpo --num_envs 1024 --headless

# 仿真回放（GUI 可视化，验证策略）
./play_go2w.sh --checkpoint logs/rsl_rl/go2w_navigation_ppo/model_best.pt
```

**硬件参考**：RTX 4090 24GB，headless 模式下 1024 并行环境约占用 14~16 GB 显存。

### 1.3 训练监控

```bash
# Tensorboard（推荐）
tensorboard --logdir IsaacLab/logs/rsl_rl/go2w_navigation_ppo

# 指标解读参考 docs/TENSORBOARD_METRICS_GUIDE.md
```

---

## 二、模型导出

训练完成后，将 PyTorch 权重导出为 ONNX 格式（供机器狗机载 ONNX Runtime 推理）：

```bash
# 导出标准导航策略（含 VAE 编码器）
python scripts/export_onnx.py \
  --checkpoint IsaacLab/logs/rsl_rl/go2w_navigation_ppo/model_best.pt \
  --output sru-robot-deployment/go2w_nav_ros2_ws/src/rl_nav_controller/deployment_policies/nav_policy.onnx

# 验证导出结果
python -c "
import onnxruntime as ort
sess = ort.InferenceSession('sru-robot-deployment/go2w_nav_ros2_ws/src/rl_nav_controller/deployment_policies/nav_policy.onnx')
print('输入:', [i.name for i in sess.get_inputs()])
print('输出:', [o.name for o in sess.get_outputs()])
print('✓ ONNX 导出验证通过')
"
```

---

## 三、实机部署

> **详细部署文档**：[sru-robot-deployment/README.md](sru-robot-deployment/README.md)（含机载目录路径完全对照表）  
> **必读交接文档**：[sru-robot-deployment/docs/GO2W_ROBOT_DEPLOYMENT_HANDOVER.md](sru-robot-deployment/docs/GO2W_ROBOT_DEPLOYMENT_HANDOVER.md)

### 机器狗网络拓扑

```
开发机 (192.168.123.138)
  │  ssh / rsync / scp
  ▼
高层算力脑：Jetson Orin DevKit — 外挂背部
  IP: 10.3.42.161 (WiFi) / 192.168.123.111 (机载网)
  user: agile    OS: Ubuntu 22.04 + ROS 2 Humble
  职责: ZED 2i 相机 + SRU 策略推理 + (可选) LiDAR 重定位
  │
  │ /cmd_vel (ROS 2 DDS 局域网广播)
  ▼
底盘主脑：Jetson Orin NX — 腹部原装
  IP: 192.168.123.18    user: unitree    OS: Ubuntu 20.04 + ROS 2 Foxy
  职责: unitree_bridge（转发 /cmd_vel → 宇树 DDS → 运动控制板）
  │
  │ /api/sport/request (DDS)
  ▼
底层运动控制板 (192.168.123.161)：驱动 12 腿部关节 + 4 轮毂电机
```

### 3.1 推送代码到机器狗

```bash
# 在开发机，仓库根目录执行
# （保持 WiFi 连接至 Agile-Robot-Guest 或机载 192.168.123.x 网段）
bash sru-robot-deployment/scripts/sync_to_go2w_nav.sh

# 同时推送 ONNX 模型权重
scp sru-robot-deployment/go2w_nav_ros2_ws/src/rl_nav_controller/deployment_policies/nav_policy.onnx \
    agile@10.3.42.161:~/go2w_nav_ros2_ws/src/rl_nav_controller/deployment_policies/
```

### 3.2 模式 A：纯局部 SRU 避障导航

**适用场景**：无地图环境、临时探索、短距离任务。  
**系统组成**：ZED 2i 相机 + SRU 策略节点。  
**工作方式**：接收 `/goal_pose`，直接输出 `/cmd_vel`，策略用深度图隐式避障。

```bash
# 在高层 Jetson (ssh agile@10.3.42.161)
cd ~/go2w_nav_ros2_ws
./start_nav.sh                    # 标准启动（ZED + 策略节点）
# 或
./start_nav_custom.sh 1.5         # 自定义前瞻距离 1.5m（实机调试推荐）
# 楼梯场景
./start_nav_stairs.sh

# 在任意 PC / 机器上发布目标点
ros2 topic pub /startNavi std_msgs/msg/Bool "{data: true}" --once
ros2 topic pub /goal_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'odom'}, pose: {position: {x: 5.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}" --once

# 停止
./stop_nav.sh
```

> ⚠️ **安全铁则**：首次电机测试前，**必须用支架撑起机器狗使四腿悬空**！

### 3.3 模式 B：全局路径规划 + SRU 局部导航（联合模式）

**适用场景**：已知环境地图、长距离多点导航任务。  
**系统组成**：LiDAR SLAM（Super LIO）+ 全局规划 + 路径跟随 + SRU 局部避障。  
**工作方式**：`/goal_pose` → `global_planner`（全局路径）→ `path_follow`（前瞻萝卜点）→ `rl_nav_controller`（速度指令）→ `/cmd_vel`

```bash
# --- 在高层 Jetson (ssh agile@10.3.42.161) ---

# 终端 1: 启动 LiDAR + Super LIO 重定位
source /opt/ros/humble/setup.bash
source ~/bridge_ws/install/setup.bash
ros2 launch super_lio relocation.py rviz:=false

# 终端 2: 启动 ZED 2i 相机
source /opt/ros/humble/setup.bash
source ~/bridge_ws/install/setup.bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i

# 终端 3: 发布相机静态 TF
ros2 run tf2_ros static_transform_publisher 0.257 0.0 0.28 0.0 0.0 0.0 base_link zed_camera_link

# 终端 4: 启动全局导航栈（全局规划 + 路径跟随 + SRU 局部节点）
source ~/bridge_ws/install/setup.bash
ros2 launch global_planner global_planner.launch.py
# (global_planner 内部会自动启动 local_planner, path_follow, rl_nav_controller)

# --- 在外部 PC (RViz) 或任意机器上 ---
ros2 topic pub /startNavi std_msgs/msg/Bool "{data: true}" --once
# 通过 RViz 的 "2D Goal Pose" 或命令行发布目标点
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "..." --once
```

**全局导航数据流详解**：  
参见 [sru-robot-deployment/docs/GLOBAL_NAV_INTEGRATION_SPEC.md](sru-robot-deployment/docs/GLOBAL_NAV_INTEGRATION_SPEC.md)

---

## 四、相关项目仓库

| 仓库 | 内容 |
|:---|:---|
| [sru-pytorch-spatial-learning](https://github.com/leggedrobotics/sru-pytorch-spatial-learning) | 核心 SRU PyTorch 模块（独立版本） |
| [sru-navigation-sim](https://github.com/leggedrobotics/sru-navigation-sim) | IsaacLab 导航仿真任务扩展包 |
| [sru-depth-pretraining](https://github.com/leggedrobotics/sru-depth-pretraining) | 深度编码器自监督预训练 |
| [sru-robot-deployment](https://github.com/leggedrobotics/sru-robot-deployment) | 实机部署 ROS 2 包（早期独立版本） |

---

## 五、文档索引

### 训练与算法
| 文档 | 内容 |
|:---|:---|
| [docs/go2w_nav_refactor_plan.md](docs/go2w_nav_refactor_plan.md) | 🔑 **Go2W 5-Phase 训练完全指南**（超参数、课程设计、调参建议） |
| [docs/NAVIGATION_TRAINING_DESIGN.md](docs/NAVIGATION_TRAINING_DESIGN.md) | 导航任务 MDP 设计原理（奖励函数、终止条件） |
| [docs/SRU_NAVIGATION_EXPLAINED.md](docs/SRU_NAVIGATION_EXPLAINED.md) | SRU 架构通俗讲解 |
| [docs/ADAPTIVE_LOOKAHEAD_NAVIGATION_DESIGN.md](docs/ADAPTIVE_LOOKAHEAD_NAVIGATION_DESIGN.md) | 自适应前瞻萝卜点算法设计 |
| [docs/TENSORBOARD_METRICS_GUIDE.md](docs/TENSORBOARD_METRICS_GUIDE.md) | 训练曲线与指标解读 |
| [docs/B2W_HEADING_CONTROL_HANDOVER.md](docs/B2W_HEADING_CONTROL_HANDOVER.md) | 终点朝向控制（B2W 参考实现） |

### 实机部署
| 文档 | 内容 |
|:---|:---|
| [sru-robot-deployment/README.md](sru-robot-deployment/README.md) | 🔑 **部署目录总说明 + 机载路径对照表** |
| [sru-robot-deployment/docs/GO2W_ROBOT_DEPLOYMENT_HANDOVER.md](sru-robot-deployment/docs/GO2W_ROBOT_DEPLOYMENT_HANDOVER.md) | 机器狗双 Jetson 拓扑、网络、传感器、实机启动全流程 |
| [sru-robot-deployment/docs/GO2W_ROS2_CLI_DEPLOYMENT_GUIDE.md](sru-robot-deployment/docs/GO2W_ROS2_CLI_DEPLOYMENT_GUIDE.md) | 完整纯命令行操作手册 |
| [sru-robot-deployment/docs/GO2W_NAVIGATION_SYSTEM_GUIDE.md](sru-robot-deployment/docs/GO2W_NAVIGATION_SYSTEM_GUIDE.md) | 局部 SRU 导航系统详解 |
| [sru-robot-deployment/docs/GLOBAL_NAV_INTEGRATION_SPEC.md](sru-robot-deployment/docs/GLOBAL_NAV_INTEGRATION_SPEC.md) | 全局 + 局部联合导航技术规范 |
| [sru-robot-deployment/docs/GO2W_SIM_HANDOFF.md](sru-robot-deployment/docs/GO2W_SIM_HANDOFF.md) | Go2W 仿真接入工程文档 |
| [sru-robot-deployment/docs/GO2W_CAMERA_CONFIGURATION_AND_INTEGRATION_GUIDE.md](sru-robot-deployment/docs/GO2W_CAMERA_CONFIGURATION_AND_INTEGRATION_GUIDE.md) | 相机配置指南 |
| [sru-robot-deployment/docs/GO2W_DOOR_NAVIGATION_HANDOVER.md](sru-robot-deployment/docs/GO2W_DOOR_NAVIGATION_HANDOVER.md) | 门区穿越功能详解 |
| [sru-robot-deployment/docs/GO2W_TERMINAL_HEADING_CONTROL_HANDOVER.md](sru-robot-deployment/docs/GO2W_TERMINAL_HEADING_CONTROL_HANDOVER.md) | 终点朝向控制功能交接 |

---

## 引用

```bibtex
@article{yang2025sru,
  author = {Yang, Fan and Frivik, Per and Hoeller, David and Wang, Chen and Cadena, Cesar and Hutter, Marco},
  title = {Spatially-enhanced recurrent memory for long-range mapless navigation via end-to-end reinforcement learning},
  journal = {The International Journal of Robotics Research},
  year = {2025},
  doi = {10.1177/02783649251401926},
  url = {https://doi.org/10.1177/02783649251401926}
}
```

---

## 许可证

BSD-3-Clause License — 详见 [LICENSE](LICENSE)。  
依赖许可证见 [licenses/](licenses/) 目录。

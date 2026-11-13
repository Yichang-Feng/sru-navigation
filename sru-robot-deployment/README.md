# sru-robot-deployment — 实机部署代码目录

> **本目录的定位**：存放所有在 Unitree Go2W 真实机器人上运行的 ROS 2 代码、启动脚本和部署文档。  
> **阅读前置条件**：先阅读 [GO2W_ROBOT_DEPLOYMENT_HANDOVER.md](docs/GO2W_ROBOT_DEPLOYMENT_HANDOVER.md) 了解机器狗双 Jetson 硬件拓扑。

---

## 目录结构与机载路径对照表

以下是本目录每个子文件夹与机器狗机载 Jetson（高层脑，`192.168.123.111`，user: `agile`）上真实目录的对应关系。

| 本仓库路径（开发机） | 机器狗机载路径（`agile@192.168.123.111`） | 说明 |
|:---|:---|:---|
| `ros2_ws/src/rl_nav_controller/` | `~/go2w_nav_ros2_ws/src/rl_nav_controller/` | **主导航 ROS 2 功能包**（推荐工作空间） |
| `ros2_ws/src/rl_nav_controller/rl_nav_controller/rl_nav_controller.py` | `~/go2w_nav_ros2_ws/src/rl_nav_controller/rl_nav_controller/rl_nav_controller.py` | 标准 SRU 局部避障导航节点（主控文件）|
| `ros2_ws/src/rl_nav_controller/rl_nav_controller/rl_nav_controller_adaptive.py` | `~/go2w_nav_ros2_ws/src/rl_nav_controller/rl_nav_controller/rl_nav_controller_adaptive.py` | 自适应前瞻萝卜点变体 |
| `ros2_ws/src/rl_nav_controller/rl_nav_controller/rl_nav_controller_stairs.py` | `~/go2w_nav_ros2_ws/src/rl_nav_controller/rl_nav_controller/rl_nav_controller_stairs.py` | 楼梯/台阶场景变体 |
| `ros2_ws/src/rl_nav_controller/rl_nav_controller/adaptive_path_tracker.py` | `~/go2w_nav_ros2_ws/src/rl_nav_controller/rl_nav_controller/adaptive_path_tracker.py` | 自适应前瞻路径跟踪器 |
| `ros2_ws/src/rl_nav_controller/rl_nav_controller/path_tracker.py` | `~/go2w_nav_ros2_ws/src/rl_nav_controller/rl_nav_controller/path_tracker.py` | 分段路径跟踪器 |
| `ros2_ws/src/rl_nav_controller/rl_nav_controller/stairs_depth_processor.py` | `~/go2w_nav_ros2_ws/src/rl_nav_controller/rl_nav_controller/stairs_depth_processor.py` | 楼梯深度图预处理模块 |
| `ros2_ws/src/rl_nav_controller/rl_nav_controller/constants.py` | `~/go2w_nav_ros2_ws/src/rl_nav_controller/rl_nav_controller/constants.py` | 全局常量（话题名、阈值等） |
| `ros2_ws/src/rl_nav_controller/launch/rl_nav_controller.launch.py` | `~/go2w_nav_ros2_ws/src/rl_nav_controller/launch/rl_nav_controller.launch.py` | 标准 launch 文件 |
| `ros2_ws/src/rl_nav_controller/launch/rl_nav_controller_adaptive.launch.py` | `~/go2w_nav_ros2_ws/src/rl_nav_controller/launch/rl_nav_controller_adaptive.launch.py` | 自适应变体 launch 文件 |
| `ros2_ws/src/rl_nav_controller/config/zed_nav_override.yaml` | `~/go2w_nav_ros2_ws/src/rl_nav_controller/config/zed_nav_override.yaml` | ZED 2i 相机参数覆盖配置 |
| `ros2_ws/src/rl_nav_controller/config/realsense_nav_override.yaml` | `~/go2w_nav_ros2_ws/src/rl_nav_controller/config/realsense_nav_override.yaml` | RealSense 相机参数覆盖配置 |
| `ros2_ws/src/rl_nav_controller/deployment_policies/` | `~/go2w_nav_ros2_ws/src/rl_nav_controller/deployment_policies/` | **ONNX 模型权重**（`nav_policy.onnx`、`vae_encoder.onnx`；**不入 git**，从训练导出后 scp） |
| `ros2_ws/src/rl_nav_controller/setup.py` | `~/go2w_nav_ros2_ws/src/rl_nav_controller/setup.py` | ROS 2 功能包安装配置 |
| `scripts/start_nav.sh` | `~/go2w_nav_ros2_ws/start_nav.sh` | 🟢 **一键启动**（ZED 相机 + 策略节点） |
| `scripts/start_nav_custom.sh` | `~/go2w_nav_ros2_ws/start_nav_custom.sh` | 自定义前瞻距离启动（`./start_nav_custom.sh 1.5`） |
| `scripts/start_nav_adaptive.sh` | `~/go2w_nav_ros2_ws/start_nav_adaptive.sh` | 自适应前瞻变体启动 |
| `scripts/start_nav_stairs.sh` | `~/go2w_nav_ros2_ws/start_nav_stairs.sh` | 楼梯场景专用启动 |
| `scripts/stop_local_sru.sh` | `~/go2w_nav_ros2_ws/stop_nav.sh` | 停止所有导航节点 |
| `scripts/start_go2w_system.sh` | `~/sru_ws/start_go2w_system.sh` | （旧版 sru_ws 工作空间）系统启动脚本 |
| `scripts/stop_go2w_system.sh` | `~/sru_ws/stop_go2w_system.sh` | （旧版 sru_ws 工作空间）系统停止脚本 |
| `config/rviz.rviz` | `~/go2w_nav_ros2_ws/rviz.rviz` | RViz 可视化配置文件 |
| `diagnostics/door_diagnostic_logger.py` | `~/go2w_nav_ros2_ws/door_diagnostic_logger.py` | 门区通过诊断日志工具 |

> **注意**：`deployment_policies/` 目录下的 ONNX 模型文件（`nav_policy.onnx`、`vae_encoder.onnx`）不加入 git 版本控制（体积过大），需在训练结束后通过 `scripts/export_onnx.py` 导出，再通过 `scp` 或 `scripts/sync_to_go2w_nav.sh` 推送到机器狗。

---

## 机载其他 ROS 2 功能包（本仓库中无源码）

以下功能包存在于机器狗机载工作空间 `~/go2w_nav_ros2_ws/src/`，源码**不在本仓库**维护（属于第三方或独立仓库）：

| 机载路径 | 功能 | 来源 |
|:---|:---|:---|
| `~/go2w_nav_ros2_ws/src/global_planner/` | 全局路径规划（占据栅格地图 Dijkstra/A*） | 独立 ROS 2 包 |
| `~/go2w_nav_ros2_ws/src/local_planner/` | 局部速度规划器接口 | 独立 ROS 2 包 |
| `~/go2w_nav_ros2_ws/src/path_follow/` | 路径跟随控制器 | 独立 ROS 2 包 |
| `~/go2w_nav_ros2_ws/src/go2w_bridge/` | Go2W 底盘桥（`/cmd_vel` → 宇树 DDS） | 独立 ROS 2 包 |
| `~/go2w_nav_ros2_ws/src/super_lio/` | LiDAR-IMU 里程计与重定位 | 独立 ROS 2 包 |
| `~/bridge_ws/src/zed-ros2-wrapper/` | ZED 2i 相机 ROS 2 驱动 | Stereolabs 官方 |
| `~/bridge_ws/src/livox_ros_driver2/` | Livox MID-360 雷达驱动 | Livox 官方 |
| `~/bridge_ws/src/unitree_bridge/` | 宇树 DDS 通信桥（C++，订阅 `/cmd_vel` 转发给运动控制板） | 宇树官方 |

---

## 一键推送代码到机器狗

### 推送导航功能包（go2w_nav_ros2_ws）

```bash
# 在开发机上，仓库根目录执行（默认目标 IP：10.3.42.161）
bash sru-robot-deployment/scripts/sync_to_go2w_nav.sh

# 或指定 IP
bash sru-robot-deployment/scripts/sync_to_go2w_nav.sh 10.3.42.161
```

脚本将自动：
1. 同步 `ros2_ws/src/rl_nav_controller/` → 机载 `~/go2w_nav_ros2_ws/src/rl_nav_controller/`
2. 同步 `scripts/start_nav*.sh` → 机载 `~/go2w_nav_ros2_ws/`
3. 询问是否自动在机载端执行 `colcon build`

### 推送训练好的 ONNX 模型权重

```bash
# 1. 在开发机导出 ONNX
python scripts/export_onnx.py \
  --checkpoint IsaacLab/logs/rsl_rl/go2w_navigation_ppo/model_best.pt \
  --output sru-robot-deployment/ros2_ws/src/rl_nav_controller/deployment_policies/nav_policy.onnx

# 2. 推送至机器狗
scp sru-robot-deployment/ros2_ws/src/rl_nav_controller/deployment_policies/nav_policy.onnx \
  agile@10.3.42.161:~/go2w_nav_ros2_ws/src/rl_nav_controller/deployment_policies/
```

---

## 文档导航

| 文档 | 内容 |
|:---|:---|
| [GO2W_ROBOT_DEPLOYMENT_HANDOVER.md](docs/GO2W_ROBOT_DEPLOYMENT_HANDOVER.md) | 🔑 **必读**：机器狗双 Jetson 拓扑、网络、传感器、实机启动全流程 |
| [GO2W_ROS2_CLI_DEPLOYMENT_GUIDE.md](docs/GO2W_ROS2_CLI_DEPLOYMENT_GUIDE.md) | 完整纯命令行操作手册（从零开机到发目标点） |
| [GO2W_NAVIGATION_SYSTEM_GUIDE.md](docs/GO2W_NAVIGATION_SYSTEM_GUIDE.md) | 局部 SRU 导航系统架构详解 |
| [GLOBAL_NAV_INTEGRATION_SPEC.md](docs/GLOBAL_NAV_INTEGRATION_SPEC.md) | 全局路径规划 + SRU 局部导航联合模式技术规范 |
| [GO2W_TERMINAL_HEADING_CONTROL_HANDOVER.md](docs/GO2W_TERMINAL_HEADING_CONTROL_HANDOVER.md) | 终点朝向控制功能交接 |
| [GO2W_DOOR_NAVIGATION_HANDOVER.md](docs/GO2W_DOOR_NAVIGATION_HANDOVER.md) | 门区穿越功能详解 |
| [GO2W_SIM_HANDOFF.md](docs/GO2W_SIM_HANDOFF.md) | Go2W 仿真接入工程文档（IsaacLab 中运行 Go2W 的完整流程） |
| [GO2W_CAMERA_CONFIGURATION_AND_INTEGRATION_GUIDE.md](docs/GO2W_CAMERA_CONFIGURATION_AND_INTEGRATION_GUIDE.md) | 相机配置与深度图集成指南 |

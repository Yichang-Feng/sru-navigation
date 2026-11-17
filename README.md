# sru-robot-deployment — 实机部署工程中心

> **本目录定位**：存放所有在 Unitree Go2W 真实机器人（双 Jetson 架构）上运行的 ROS 2 工作空间、驱动、节点、配置与启动脚本。  
> **机载对应目标**：背部高层主脑（NVIDIA Jetson Orin DevKit，IP: `192.168.123.111` / `10.3.42.161`，用户: `agile`）。  
> **前置必读**：请先阅读 [GO2W_DEPLOYMENT_GUIDE.md](docs/GO2W_DEPLOYMENT_GUIDE.md) 了解硬件接线、网络拓扑与两机通信。

---

## 一、目录结构与机载路径完全对照表

本目录完整集成了机器狗机载的**两大 ROS 2 工作空间**（`go2w_nav_ros2_ws` 与 `bridge_ws`）、局部模式工作空间 `sru_ws` 的工作空间级文件，以及配套脚本与文档：

```
sru-robot-deployment/
│
├── go2w_nav_ros2_ws/              ← 🚀 对应狗机机载 ~/go2w_nav_ros2_ws/ (高层导航栈，与狗上一致)
│   ├── src/
│   │   ├── rl_nav_controller/     ← 核心 SRU 局部强化学习避障与前瞻跟踪节点 (Python)
│   │   │   ├── rl_nav_controller/ (主控 / 自适应 / 楼梯三个变体 + tracker + utils)
│   │   │   ├── launch/  config/  scripts/
│   │   │   ├── deployment_policies/  (ONNX 模型目录；*.onnx/*.pt 不进 git)
│   │   │   └── setup.py  package.xml
│   │   ├── global_planner/        ← 全局路径规划 C++ 包 (可通行点云上的 Dijkstra/A*)
│   │   │   └── map/trav_map.pcd   (先验可通行地图；*.pcd 不进 git，需单独 rsync)
│   │   ├── local_planner/         ← 局部规划器 C++ 包 (B 样条轨迹优化)
│   │   ├── path_follow/           ← 路径跟随与动态胡萝卜点 C++ 包
│   │   ├── go2w_bridge/           ← 底盘通信与手柄控制接口包
│   │   ├── task_manager/          ← 任务/状态调度包
│   │   ├── rviz_tools/            ← RViz 自定义目标点工具插件
│   │   └── Super-LIO-ros2/        ← Super-LIO LiDAR 定位/建图源码
│   │       └── src/super_lio/     (map_localization / relocation 节点, config, rviz, map)
│   ├── start_nav.sh               ← 导航一键启动 (tmux 分屏)
│   ├── start_nav_custom.sh        ← 自定义前瞻距离启动
│   ├── start_nav_adaptive.sh      ← 自适应前瞻变体
│   ├── start_nav_stairs.sh        ← 楼梯场景变体
│   ├── stop_nav.sh                ← 导航停止脚本
│   ├── rviz.rviz / test.rviz      ← 导航监控 RViz 视角配置
│   ├── door_diagnostic_logger.py  ← 窄门诊断记录工具
│   └── GO2W_DOOR_NAVIGATION_HANDOVER.md
│
├── sru_ws/                        ← 🔁 对应狗机 ~/sru_ws/ 的工作空间级文件快照
│   ├── start_local_sru.sh         ← 局部 SRU 模式启动脚本
│   ├── stop_local_sru.sh
│   ├── realsense_nav_override.yaml / zed_nav_override.yaml
│   ├── local_sru.rviz             ← 局部模式预调 RViz
│   ├── mock_global_nav_tester.py / test_*.py
│   ├── benchmark/                 ← 性能基准套件
│   └── README.md                  ← 快照说明 (狗上 src/sru-robot-deployment 是本仓库旧克隆)
│
├── bridge_ws/                     ← 🔌 对应狗机机载 ~/bridge_ws/ (底盘通信与传感器驱动)
│   ├── src/
│   │   ├── unitree_bridge/        ← 宇树官方 C++ 底盘桥接节点 (/cmd_vel → 宇树 DDS)
│   │   ├── livox_ros_driver2/     ← Livox MID-360 激光雷达 ROS 2 驱动
│   │   ├── zed-ros2-wrapper/      ← Stereolabs ZED 2i 双目深度相机 ROS 2 驱动
│   │   ├── super_lio/             ← Super LIO 激光惯导里程计与重定位功能包 (含 map.pcd)
│   │   ├── rviz_multi_goals_manager_plugins/
│   │   ├── nmea_msgs/             ← GPS / 惯导消息定义
│   │   ├── basic/                 ← 基础工具包
│   │   └── rl_nav_controller/     ← 备份引用节点
│   ├── bridge.sh                  ← 底盘桥单键启动
│   ├── bridge.yaml                ← 底盘参数配置
│   ├── lio.sh                     ← 雷达定位启动
│   ├── start.sh                   ← 策略启动
│   ├── complex.sh                 ← colcon 一键编译
│   └── zed_nav_override.yaml      ← 相机覆盖参数
│
├── scripts/                       ← 🛠️ 远程一键部署/同步/控制脚本
│   ├── sync_to_go2w_nav.sh        ← 一键将本地修改推送到机载 ~/go2w_nav_ros2_ws/
│   ├── sync_adaptive_to_go2w.sh   ← 推送自适应变体至机器狗
│   ├── sync_stairs_to_go2w.sh     ← 推送楼梯变体至机器狗
│   ├── sync_to_robot.sh           ← 底层一键同步脚本
│   ├── start_nav.sh               ← 本地备用启动脚本
│   ├── start_nav_custom.sh        ← 自定义前瞻距离启动脚本
│   ├── start_nav_adaptive.sh      ← 自适应前瞻启动脚本
│   ├── start_nav_stairs.sh        ← 楼梯场景启动脚本
│   └── stop_local_sru.sh          ← 停止脚本
│
├── docs/                          ← 📖 实机部署技术与交接文档
│   ├── GO2W_DEPLOYMENT_GUIDE.md           (🔑 拓扑 + 安全 + 一键启动 + 排错)
│   ├── GO2W_NAVIGATION_SYSTEM_GUIDE.md    (局部 SRU 导航系统详解，v3.3)
│   ├── GLOBAL_NAV_INTERFACE.md            (模式对比 + 话题/状态机/参数接口)
│   ├── GO2W_CAMERA_CONFIGURATION_AND_INTEGRATION_GUIDE.md (相机深度图参数调校)
│   └── GO2W_SIM_HANDOFF.md                (仿真接入工程交接)
│
├── diagnostics/                   ← 🩺 诊断与实机记录工具
│   ├── door_diagnostic_logger.py  (窄门通过热力图与特征诊断记录工具)
│   ├── test_door_diagnostic_logger.py
│   └── test_realsense_max_distance.py
│
└── config/                        ← 配置文件 (rviz.rviz 等)
```

---

## 二、两大工作空间的职责分工

| 工作空间 | 机载路径 | 主要节点与职责 | 启动命令 |
|:---|:---|:---|:---|
| **`bridge_ws`** | `agile@111:~/bridge_ws` | **硬件驱动与底层中枢**：<br>1. `unitree_bridge`：监听 `/cmd_vel` 转换下发 DDS 给底盘。<br>2. `livox_ros_driver2`：发布雷达点云。<br>3. `zed_wrapper`：发布深度图与相机里程计。<br>4. `super_lio`：雷达全局重定位。 | `cd ~/bridge_ws && ./bridge.sh`<br>`ros2 launch super_lio relocation.py` |
| **`go2w_nav_ros2_ws`** | `agile@111:~/go2w_nav_ros2_ws` | **高层自主避障与导航栈**：<br>1. `rl_nav_controller`：运行 SRU ONNX 模型进行端到端局部避障。<br>2. `global_planner`：基于先验可通行点云生成全局路径 `/global_path`（`map` 系）。<br>3. `path_follow`：计算路径前瞻萝卜点并送入强化学习策略。<br>4. `Super-LIO-ros2`：`map_localization.py` 发布 `map -> world` 与 `/loc/robo/odom`。 | `cd ~/go2w_nav_ros2_ws && ./start_nav.sh`<br>首次定位须在 RViz 给 `2D Pose Estimate`（发 `/initialpose`），否则节点停在 `WAIT_INIT` |
| **`sru_ws`** | `agile@111:~/sru_ws` | **局部 SRU 模式**：只跑底盘桥 + 雷达定位 + 相机 + `rl_nav_controller`，目标来自 RViz `/goal_pose`。其中 `src/sru-robot-deployment` 是本仓库在狗上的旧克隆。 | `cd ~/sru_ws && ./start_local_sru.sh` |

---

## 三、一键推送代码到机器狗

在开发机上执行以下命令，将本地修改一键同步到机载：

```bash
# 1. 确保连入机器狗网络（如 WiFi: Agile-Robot-Guest）
# 2. 推送导航工作空间代码：
bash sru-robot-deployment/scripts/sync_to_go2w_nav.sh 10.3.42.161

# 3. 推送训练好的最新 ONNX 模型权重：
scp sru-robot-deployment/go2w_nav_ros2_ws/src/rl_nav_controller/deployment_policies/nav_policy.onnx \
    agile@10.3.42.161:~/go2w_nav_ros2_ws/src/rl_nav_controller/deployment_policies/
```

---

## 四、部署文档导航

| 领域 | 核心文档 |
|:---|:---|
| **开机与通信** | [GO2W_DEPLOYMENT_GUIDE.md](docs/GO2W_DEPLOYMENT_GUIDE.md) |
| **命令行操作** | [GO2W_DEPLOYMENT_GUIDE.md](docs/GO2W_DEPLOYMENT_GUIDE.md) |
| **架构与联合模式** | [GLOBAL_NAV_INTERFACE.md](docs/GLOBAL_NAV_INTERFACE.md) |
| **相机调优** | [GO2W_CAMERA_CONFIGURATION_AND_INTEGRATION_GUIDE.md](docs/GO2W_CAMERA_CONFIGURATION_AND_INTEGRATION_GUIDE.md) |
| **场景适配** | [GO2W_SIM_HANDOFF.md](docs/GO2W_SIM_HANDOFF.md) |
| **VLN 上层接入** | [VLN_BASE_FRAME_GOALS.md](docs/VLN_BASE_FRAME_GOALS.md) |

# sru-robot-deployment — 实机部署工程中心

> **本目录定位**：存放所有在 Unitree Go2W 真实机器人（双 Jetson 架构）上运行的 ROS 2 工作空间、驱动、节点、配置与启动脚本。  
> **机载对应目标**：背部高层主脑（NVIDIA Jetson Orin DevKit，IP: `192.168.123.111` / `10.3.42.161`，用户: `agile`）。  
> **前置必读**：请先阅读 [GO2W_ROBOT_DEPLOYMENT_HANDOVER.md](docs/GO2W_ROBOT_DEPLOYMENT_HANDOVER.md) 了解硬件接线、网络拓扑与两机通信。

---

## 一、目录结构与机载路径完全对照表

本目录完整集成了机器狗机载的**两大 ROS 2 工作空间**（`go2w_nav_ros2_ws` 与 `bridge_ws`）以及配套脚本与文档：

```
sru-robot-deployment/
│
├── go2w_nav_ros2_ws/              ← 🚀 对应狗机机载 ~/go2w_nav_ros2_ws/ (高层导航栈)
│   ├── src/
│   │   ├── rl_nav_controller/     ← 核心 SRU 局部强化学习避障与前瞻跟踪节点
│   │   │   ├── rl_nav_controller/
│   │   │   │   ├── rl_nav_controller.py          (标准主控节点)
│   │   │   │   ├── rl_nav_controller_adaptive.py (自适应前瞻变体)
│   │   │   │   ├── rl_nav_controller_stairs.py   (楼梯穿越变体)
│   │   │   │   ├── adaptive_path_tracker.py      (自适应前瞻胡萝卜点算法)
│   │   │   │   ├── path_tracker.py               (分段路径跟踪器)
│   │   │   │   ├── stairs_depth_processor.py     (台阶高程提取处理器)
│   │   │   │   └── constants.py                  (遥控器手柄与状态常量)
│   │   │   ├── launch/                           (启动 launch 文件)
│   │   │   ├── config/                           (ZED / RealSense 参数覆盖)
│   │   │   ├── deployment_policies/              (ONNX 模型目录, 模型文件从训练导出)
│   │   │   └── setup.py
│   │   ├── global_planner/        ← 全局路径规划功能包 (Dijkstra / A* on 栅格地图)
│   │   ├── local_planner/         ← 局部规划器功能包
│   │   ├── path_follow/           ← 路径跟随与动态胡萝卜点提取节点
│   │   ├── go2w_bridge/           ← 底盘通信与手柄控制接口包
│   │   └── super_lio/             ← LiDAR 里程计与重定位配置包
│   ├── start_nav.sh               ← 导航一键启动脚本
│   ├── stop_nav.sh                ← 导航停止脚本
│   ├── rviz.rviz                  ← 导航监控 RViz 视角配置
│   └── test.rviz
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
│   ├── GO2W_ROBOT_DEPLOYMENT_HANDOVER.md        (🔑 硬件拓扑与开机排坑手册)
│   ├── GO2W_ROS2_CLI_DEPLOYMENT_GUIDE.md        (完整纯命令行执行指南)
│   ├── GO2W_NAVIGATION_SYSTEM_GUIDE.md          (局部 SRU 导航系统详解)
│   ├── GLOBAL_NAV_INTEGRATION_SPEC.md           (全局规划 + SRU 联合导航规范)
│   ├── GO2W_LOCAL_SRU_VS_GLOBAL_NAV_ARCHITECTURE.md (两种导航模式架构对比)
│   ├── GO2W_TERMINAL_HEADING_CONTROL_HANDOVER.md(终点朝向控制交接)
│   ├── GO2W_DOOR_NAVIGATION_HANDOVER.md         (窄门通过与前瞻自适应)
│   ├── GO2W_CAMERA_CONFIGURATION_AND_INTEGRATION_GUIDE.md (相机深度图参数调校)
│   └── GO2W_SIM_HANDOFF.md                      (仿真接入工程交接)
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
| **`go2w_nav_ros2_ws`** | `agile@111:~/go2w_nav_ros2_ws` | **高层自主避障与导航栈**：<br>1. `rl_nav_controller`：运行 SRU ONNX 模型进行端到端局部避障。<br>2. `global_planner`：基于雷达先验地图生成全局全局路径 `/path`。<br>3. `path_follow`：计算路径前瞻萝卜点并送入强化学习策略。 | `cd ~/go2w_nav_ros2_ws && ./start_nav.sh` |

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
| **开机与通信** | [GO2W_ROBOT_DEPLOYMENT_HANDOVER.md](docs/GO2W_ROBOT_DEPLOYMENT_HANDOVER.md) |
| **命令行操作** | [GO2W_ROS2_CLI_DEPLOYMENT_GUIDE.md](docs/GO2W_ROS2_CLI_DEPLOYMENT_GUIDE.md) |
| **架构与联合模式** | [GLOBAL_NAV_INTEGRATION_SPEC.md](docs/GLOBAL_NAV_INTEGRATION_SPEC.md)<br>[GO2W_LOCAL_SRU_VS_GLOBAL_NAV_ARCHITECTURE.md](docs/GO2W_LOCAL_SRU_VS_GLOBAL_NAV_ARCHITECTURE.md) |
| **相机调优** | [GO2W_CAMERA_CONFIGURATION_AND_INTEGRATION_GUIDE.md](docs/GO2W_CAMERA_CONFIGURATION_AND_INTEGRATION_GUIDE.md) |
| **场景适配** | [GO2W_DOOR_NAVIGATION_HANDOVER.md](docs/GO2W_DOOR_NAVIGATION_HANDOVER.md)<br>[GO2W_TERMINAL_HEADING_CONTROL_HANDOVER.md](docs/GO2W_TERMINAL_HEADING_CONTROL_HANDOVER.md) |

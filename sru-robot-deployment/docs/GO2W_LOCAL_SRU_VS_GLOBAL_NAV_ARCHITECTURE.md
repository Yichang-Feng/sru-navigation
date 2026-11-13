# Unitree Go2W 导航系统：单独启动局部 SRU 测试 vs 接入全局导航部署 调用架构与文件对照指南

本文档全面梳理 **Unitree Go2W 机器狗** 在两种典型运行模式下的软件架构、调用文件清单、话题交互拓扑与参数对齐规范：
1. **模式一：单独启动局部 SRU 测试模式 (`start_local_sru.sh`)**
2. **模式二：接入全局导航全系统部署模式 (`start_nav.sh`)**

---

## 1. 架构概览与核心对齐原则

```
+--------------------------------------------------------------------------------------------------+
|                                    Unitree Go2W 统一控制基座                                      |
|                                                                                                  |
|   遥控手柄控制统一: [B 键] -> 紧急制动 / 暂停 (/motion_enabled=False)                              |
|                    [A 键] -> 开闸启动 / 恢复推进 (/motion_enabled=True)                             |
|                                                                                                  |
|   到达判定距离统一: 单点导航 ARRIVE_GOAL_THRESHOLD = 0.20m                                        |
|                    全局路径 SEGMENT_ARRIVE_THRESHOLD = 0.20m                                     |
+--------------------------------------------------------------------------------------------------+
                                        |
                 +----------------------+----------------------+
                 |                                             |
                 v                                             v
  【模式一：单独启动局部 SRU 测试】               【模式二：接入全局导航部署】
  • 适用：局部避障策略推理、单点直连导航         • 适用：长距离全自主巡航、跨楼层楼梯导航
  • 启动脚本：./start_local_sru.sh              • 启动脚本：./start_nav.sh
  • 停止脚本：./stop_local_sru.sh               • 停止脚本：./stop_nav.sh
  • 目标来源：RViz 2D Nav Goal (/goal_pose)     • 目标来源：全局规划器动态下发 (/global_path)
  • 控制节点：rl_nav_controller                 • 控制节点：rl_nav_controller_stairs
```

---

## 2. 模式一：单独启动局部 SRU 测试 (`start_local_sru.sh`)

### 2.1 定位与用途
不依赖上层全局地图规划器与调度状态机，单独拉起底盘通信桥、激光雷达重定位、深度相机感知以及机载 SRU 强化学习避障策略，用于快速验证：
- 策略网络的端到端动态避障敏捷度；
- 深度相机点云/深度图与激光雷达定位实时性；
- 遥控手柄 A/B 键即时启停与刹停保底机制；
- RViz 2D Nav Goal 单点目标响应与 0.20m 精准到点。

### 2.2 启动与停止指令
```bash
# 进入机载 sru_ws 目录
cd ~/sru_ws

# 默认使用 Intel RealSense D435/D435i 相机启动
./start_local_sru.sh

# 或切换使用 Stereolabs ZED 2i 相机启动
./start_local_sru.sh --zed

# （兼容旧指令別名）
./start_go2w_system.sh

# 停止局部导航系统
./stop_local_sru.sh
# 或在运行终端直接按 Ctrl+C
```

### 2.3 调用的具体文件与功能角色

| 序号 | 调用文件 / 模块 | 路径位置 | 功能描述 |
| :--- | :--- | :--- | :--- |
| **1** | **启动主脚本** | `~/sru_ws/start_local_sru.sh`<br>*(兼容软链接: `start_go2w_system.sh`)* | 系统级编排脚本：自动加载环境、拉起各后台支撑节点、最后在前台启动避障策略。 |
| **2** | **停止脚本** | `~/sru_ws/stop_local_sru.sh`<br>*(兼容软链接: `stop_go2w_system.sh`)* | 向 `/motion_enabled` 发布 `false` 锁死底盘，强行终止所有后台局部感知与控制器节点。 |
| **3** | **底盘与手柄桥** | `~/go2w_nav_ros2_ws/src/go2w_bridge/launch/go2w_bridge.launch.py`<br>或 `~/sru_ws/src/go2w_bridge/launch/go2w_bridge.launch.py` | 拉起 `cmd_vel_bridge` 与 `wireless_controller`：<br>• 监听宇树手柄 `/wirelesscontroller`，按 B 发布 `/motion_enabled=False`，按 A 发布 `/motion_enabled=True`；<br>• 仅当使能时转发 `/cmd_vel` 到 `/api/sport/request`。 |
| **4** | **激光雷达与定位** | `livox_ros_driver2 msg_MID360_launch.py`<br>& `super_lio map_localization.py` | 启动 Livox MID-360 雷达硬件驱动与 Super-LIO 地图重定位（与全局启动完全统一），稳定输出 `/lio/robo/odom` 并在后台按需拉起 RViz。 |
| **5** | **深度相机感知** | `realsense2_camera rs_launch.py`<br>或 `zed_wrapper zed_camera.launch.py` | 启动深度相机驱动流水线，输出原生深度图（`/camera/camera/depth/image_rect_raw` 或 `/zed/zed_node/depth/depth_registered`）。 |
| **6** | **相机覆盖配置** | `~/sru_ws/realsense_nav_override.yaml`<br>或 `~/sru_ws/zed_nav_override.yaml` | 针对移动避障优化的相机配置（开启空间/时间/空洞滤波，关闭点云和RGB开销）。 |
| **7** | **相机静态 TF** | `tf2_ros static_transform_publisher` | 发布 `base_link -> camera_link`（RealSense: `0.34, 0.0, 0.25`）水平安装外参。 |
| **8** | **强化学习局部控制器** | `~/sru_ws/src/sru-robot-deployment/rl_nav_controller/rl_nav_controller/rl_nav_controller.py` | SRU 避障核心节点：读取深度图与里程计，接收单点目标推理出 `cmd_vel`，到点距离 0.20m。 |
| **9** | **控制器常量定义** | `~/sru_ws/src/sru-robot-deployment/rl_nav_controller/rl_nav_controller/constants.py` | 定义 `ARRIVE_GOAL_THRESHOLD = 0.20m`、`BUTTON_START = 0`、`BUTTON_PAUSE = 1` 等全局常量。 |
| **10**| **策略模型权重** | `~/sru_ws/model/Odin_nav_policy.onnx` | 机载端到端 ONNX 强化学习神经网络权重文件。 |

---

## 3. 模式二：接入全局导航全系统部署 (`start_nav.sh`)

### 3.1 定位与用途
接入高层全局拓扑地图规划器（Global Planner）、分段胡萝卜点自适应追踪、3D 楼梯坡度门控（Stairs Slope Gate）与全自主状态机调度，用于：
- 全局大范围自主导航巡航；
- 窄门、双向走廊穿行（`curvature_only` 转弯前瞻自适应，防止墙面误判）；
- 跨楼层上下楼梯感知与平滑推进；
- 逐段任务推进调度与停滞重规划上报（`/local_planner/status`）。

### 3.2 启动与停止指令
```bash
# 进入机载全局导航工作空间
cd ~/go2w_nav_ros2_ws

# 默认启动当前最优导航系统 (曲率自适应 + 楼梯感知 0.4m 前瞻)
./start_nav.sh

# 可选：需要弹出 RViz 监控界面时附加 --rviz 参数
./start_nav.sh --rviz

# 在任意终端执行停止命令
./stop_nav.sh
```

### 3.3 调用的具体文件与功能角色

| 序号 | 调用文件 / 模块 | 路径位置 | 功能描述 |
| :--- | :--- | :--- | :--- |
| **1** | **全局启动主脚本** | `~/go2w_nav_ros2_ws/start_nav.sh` | 基于 tmux 动态分屏拉起多节点集群，负责按依赖顺序拉起驱动、SLAM、感知、全局规划与局部规划。 |
| **2** | **全局停止脚本** | `~/go2w_nav_ros2_ws/stop_nav.sh` | 发布零速制动并发布 `/motion_enabled: false`，发送 `C-c` 并关闭 tmux 会话与进程。 |
| **3** | **雷达硬件驱动** | `livox_ros_driver2 msg_MID360_launch.py` | 独立后台驱动 MID-360 激光雷达。 |
| **4** | **激光里程计与重定位** | `super_lio map_localization.py` | Super-LIO 地图定位节点，发布高频雷达里程计 `/lio/robo/odom` 与 `map -> base_link` TF。 |
| **5** | **深度相机节点** | `realsense2_camera rs_launch.py`<br>或 `zed_wrapper zed_camera.launch.py` | 后台运行深度相机节点，由分屏监控曝光与帧率。 |
| **6** | **底盘与手柄安全桥** | `go2w_bridge/launch/go2w_bridge.launch.py` | 运行 `go2w_cmd_vel_bridge` 与 `go2w_wireless_controller`，实现按 B 刹停、按 A 恢复。 |
| **7** | **全局规划器** | `global_planner/launch/global_planner.launch.py` | 基于 3D 点云栅格地图生成全局最优无碰撞路径，向 `/global_path` 逐段下发。 |
| **8** | **楼梯感知局部控制器** | `src/rl_nav_controller/rl_nav_controller/rl_nav_controller_stairs.py` | 集成楼梯 3D 坡度门控（下坡 0.40m、上坡 0.45m 前瞻）、曲率前瞻自适应、终点航向对齐与状态机。 |
| **9** | **几何路径追踪器** | `src/rl_nav_controller/rl_nav_controller/adaptive_path_tracker.py` | 计算当前投影进度、胡萝卜点插值、曲率估计与终点到达判定（`check_arrived`，阈值 0.20m）。 |
| **10**| **楼梯深度图处理器** | `src/rl_nav_controller/rl_nav_controller/stairs_depth_processor.py` | 楼梯斜坡点云与深度图重映射适配工具。 |
| **11**| **核心避障基类** | `src/rl_nav_controller/rl_nav_controller/rl_nav_controller.py`<br>& `rl_nav_controller_adaptive.py` | 提供底层推理引擎、TF 坐标变换、手柄 A/B 恢复逻辑与底盘启闭安全总闸。 |

---

## 4. 关键特性与参数对照矩阵 (Comparison Matrix)

| 关键特性 | 模式一：单独启动局部 SRU (`start_local_sru.sh`) | 模式二：接入全局导航部署 (`start_nav.sh`) | 状态对齐结果 |
| :--- | :--- | :--- | :--- |
| **启动命令** | `cd ~/sru_ws && ./start_local_sru.sh` | `cd ~/go2w_nav_ros2_ws && ./start_nav.sh` | **独立区分，清晰明确** |
| **停止命令** | `cd ~/sru_ws && ./stop_local_sru.sh` | `cd ~/go2w_nav_ros2_ws && ./stop_nav.sh` | **各自独立，互不污染** |
| **进程管理方式** | 后台 `nohup / &` + 信号陷阱（`trap cleanup`） | `tmux` 动态分屏交互式多会话管理 | **适配不同使用场景** |
| **手柄遥控暂停** | **按 [B] 键**：进入 `STOPPED`，制动停机，保留目标 | **按 [B] 键**：进入 `STOPPED`，制动停机，保留路径 | **完全对齐统一** |
| **手柄遥控启动** | **按 [A] 键**：开闸解禁，恢复向单点目标推进 | **按 [A] 键**：开闸解禁，重对齐并恢复路径追踪 | **完全对齐统一** |
| **底盘硬件桥** | `go2w_bridge`（优先）/ `unitree_bridge`（兜底） | `go2w_bridge` | **统一使用 go2w_bridge** |
| **底盘安全总闸** | `/motion_enabled` & `/startNavi` 联动 | `/motion_enabled` & `/startNavi` 联动 | **完全对齐统一** |
| **目标到达判定** | `arrive_goal_threshold = 0.20m` | `segment_arrive_threshold = 0.20m` | **完全对齐为 0.20m** |
| **原地漂移容差** | `0.40m`（终点原地转弯航向对齐容差） | `0.40m`（终点原地转弯航向对齐容差） | **完全对齐统一** |
| **路径规划器** | 无（直连单点目标或手柄目标） | `global_planner` 自动全局路径生成 | **按需分层** |
| **楼梯/坡度感知** | 依赖纯深度图原生策略感知 | 动态 3D 路径坡度门控（下坡 0.4m / 上坡 0.45m） | **全局层增强** |
| **状态机上报** | 瞬发 `idle`, `running`, `goal_reached`, `stop` | 3Hz 周期 + 瞬发 `idle`, `running`, `arrived`, `blocked`, `stop` | **向下完全兼容** |

---

## 5. 部署与同步脚本指引

在本地开发机完成修改后，可通过以下两组脚本分别推送到机器狗：

### 5.1 推送局部 SRU 测试环境（机载 `~/sru_ws`）
```bash
# 在本地开发机执行 (默认 IP: 192.168.31.186，支持传参指定新 IP)
bash scripts/sync_to_robot.sh [ROBOT_IP]
```
该脚本自动执行：
1. 将 `rl_nav_controller.py`, `rl_nav_controller_adaptive.py`, `constants.py` 推送至 `~/sru_ws/src/sru-robot-deployment/rl_nav_controller/`；
2. 推送 `start_local_sru.sh` 与 `stop_local_sru.sh`，并在机载建立 `start_go2w_system.sh` 软链接；
3. 在机载端自动执行 `colcon build --packages-select rl_nav_controller --symlink-install` 编译生效。

### 5.2 推送全局导航系统环境（机载 `~/go2w_nav_ros2_ws`）
```bash
# 在本地开发机执行
bash scripts/sync_to_go2w_nav.sh [ROBOT_IP]
```
该脚本自动执行：
1. 推送全套控制器、楼梯处理器与几何跟踪器至 `~/go2w_nav_ros2_ws/src/rl_nav_controller/`；
2. 推送 `start_nav.sh`, `stop_nav.sh`, `start_nav_custom.sh` 等全局部署文件；
3. 同步清理并同步 `~/sru_ws` 脚本；
4. 在机载端自动执行 `colcon build --packages-select rl_nav_controller --symlink-install`。

# Unitree Go2W 实机部署手册（部署指南）

> 合并自原 `GO2W_ROBOT_DEPLOYMENT_HANDOVER.md`（2026-11-02）与 `GO2W_ROS2_CLI_DEPLOYMENT_GUIDE.md`（v2.0, 2026-10-26）。
> 硬件拓扑、网络、传感器、启动与排错的一站式手册。相机调优见 `GO2W_CAMERA_CONFIGURATION_AND_INTEGRATION_GUIDE.md`；接口规范见 `GLOBAL_NAV_INTERFACE.md`；系统算法详解见 `GO2W_NAVIGATION_SYSTEM_GUIDE.md`。

---

## 1. 硬件拓扑与网络

全系统运行在机载局域网（网段：`192.168.123.0/24`）。外部连接可通过 **Agile-Robot-Guest WiFi**（`10.3.42.x/24`，当前最常用）或 `192.168.31.0/24` 网段接入。

| 节点 IP | 角色与硬件定位 | OS / 环境 | 登录方式 | 职能分工 |
|---|---|---|---|---|
| `10.3.42.161`（WiFi，机载网 `192.168.123.111`） | **高层视觉、雷达 SLAM 与强化学习核心脑**（背部外挂 NVIDIA Jetson Orin DevKit） | Ubuntu 22.04 LTS / JetPack 6.x / CUDA 12.x / Py3.10 / **ROS 2 Humble** | `ssh agile@10.3.42.161`（备选 `192.168.31.186` ），密码 `1` | **高层算法主战场（可单机跑全套）**：直连 realsense 与 Livox MID-360，运行 Super LIO 重定位与 `rl_nav_controller`，输出 `/cmd_vel` |

### 1.1 传感器与物理接口

```
[Stereolabs ZED 2i 双目相机] ──USB 3.0──┐
                                       ├──> [高层核心算力脑 (192.168.123.111, agile)]
[Livox MID-360 激光雷达]   ──以太网───┘     │ (ROS 2 话题 /cmd_vel)
                                            ▼
                                   [底盘机载主脑 (192.168.123.18) 或 111 本地 bridge]
                                            │ (DDS 发送 /api/sport/request)
                                            ▼
                                   [底层运动主控板 (192.168.123.161)]
```

- **ZED 2i**：USB 3.0 插在外挂 Jetson（111），USB ID `2b03:f880`（Video）、`2b03:f881`（IMU MCU）；ROS 2 Humble 环境启动。
- **Livox MID-360**：通过机载交换机直连，在 18 上运行，广播 `/lio_sam_ros2/mapping/odometry`（或 `/lio/robo/odom`）。
- **两机通信**：标准 ROS 2 CycloneDDS 组网，话题天然互通，无需显式桥接。

### 1.2 机载工作空间（与本仓库对照）

高层机 `192.168.123.111`（`agile`），位于 `/home/agile/`（本地快照：`~/go2w_ref/`，2026-11-16 复制自狗上的 `go2w_nav_ros2_ws` 与 `sru_ws`）：

```
/home/agile/
├── go2w_nav_ros2_ws/            # 高层导航栈 (ROS 2 Humble)
│   ├── src/                     # rl_nav_controller / global_planner / local_planner / path_follow /
│   │                            # go2w_bridge / task_manager / rviz_tools / Super-LIO-ros2
│   │                            #   (Super-LIO-ros2/src/basic, src/super_lio, map/loc_map)
│   ├── start_nav.sh  start_nav_custom.sh  start_nav_adaptive.sh  start_nav_stairs.sh
│   └── stop_nav.sh  rviz.rviz  test.rviz
├── sru_ws/                      # 局部 SRU 模式工作空间；src/sru-robot-deployment 是本仓库在狗上的旧克隆
│   ├── src/sru-robot-deployment/rl_nav_controller/   # 局部模式使用的控制器包
│   ├── model/  map/  benchmark/
│   └── start_local_sru.sh  stop_local_sru.sh  *_nav_override.yaml  local_sru.rviz
├── bridge_ws/                   # 通用底盘桥与导航接口工作空间
├── realworld/                   # 路试、Waypoints、场景配置
├── onnxruntime_gpu-...whl       # Py3.10 GPU 推理加速包
└── unitree_ros2/                # 宇树官方 ROS 2 消息定义与 setup.sh
```

底盘机 `192.168.123.18`（`unitree`），位于 `/home/unitree/`（本地备份在 `/home/yichangfeng/go2w_robot_ref/`）：

```
/home/unitree/
├── bridge_ws/src/unitree_bridge/   # C++ 底盘桥接：订阅 /cmd_vel → 转发 /api/sport/request
├── autonomy_stack_go2/             # 雷达建图与 LIO-SAM 里程计
└── unitree_sdk2/                   # 宇树底层 DDS 通信 SDK
```

---

## 2. 安全规范（必须遵守）

> [!CAUTION]
> **安全防翻滚铁则**：
> 1. 首次测试电机关节运动前，**必须用结实的板凳或专用支架撑起机器狗腹部，使四肢和四个车轮完全悬空离地**。
> 2. 手边必须时刻准备好宇树无线遥控手柄，随时准备切换回手动控制。
> 3. 遥控手柄的 **B 键** 是底盘控制软开关，按一下可在使能与失能之间切换。

---

## 3. 启动方式

### 3.1 方式 A：一键脚本（推荐）

**全局导航模式**（`~/go2w_nav_ros2_ws`）：

```bash
cd ~/go2w_nav_ros2_ws
./start_nav.sh                # 默认 curv-only + 楼梯感知，不弹 RViz
./start_nav.sh --rviz         # 同上并自动弹出 RViz
./start_nav.sh --zed          # 切换为 ZED 2i 相机
```

**局部 SRU 模式**（`~/sru_ws`）：

```bash
cd ~/sru_ws
./start_local_sru.sh          # 默认 Intel RealSense D435/D435i
./start_local_sru.sh --zed    # 切换 ZED 2i
# 兼容旧命令别名
./start_go2w_system.sh
```

`start_nav.sh` 默认参数（当前实测最佳）：`curvature_only` 平地前瞻、前瞻上界 2.0m、下楼锁存 0.40m、上楼锁存 0.45m、到达判定 0.45m、终点朝向对齐开启（进入转身 0.50m / 放弃 0.60m，tol 0.26 rad）、停滞报错开启、深度重映射关闭。参数列表见 `start_nav.sh -h`。

### 3.2 方式 B：分步启动（排错与调试备查）

在 NoMachine 桌面按顺序开启终端：

**终端 1 — 底盘通信桥**
```bash
ros2 node list | grep unitree_bridge   # 若已自启则跳过
ros2 run unitree_bridge unitree_bridge  # 未自启时手动启动
```

**终端 2 — 雷达 + Super-LIO 地图定位**
```bash
ros2 launch super_lio map_localization.py
# 节点启动后停在 WAIT_INIT，必须先给初始位姿，否则既不广播 map -> world，也不发 /loc/robo/odom：
#   A) RViz 里按 p（2D Pose Estimate）在地图上点出狗的位置并拉出朝向；
#   B) 或直接发（位姿换成狗在地图中的实际位置/朝向）：
# ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
#   "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"
# 本 launch 不自动弹 RViz；需要时：ros2 run rviz2 rviz2 -d ~/go2w_nav_ros2_ws/rviz.rviz
# 旧流程 relocation.py 仍可用（静态广播 map->world），但它的 rviz:=true 参数实际不生效（rviz 节点未加入 LaunchDescription）。
```

**终端 3 — ZED 2i 深度相机**
```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
ros2 topic hz /zed/zed_node/depth/depth_registered   # 确认 15~30Hz
```

**终端 4 — 相机水平静态 TF**
```bash
ros2 run tf2_ros static_transform_publisher 0.257 0.0 0.28 0.0 0.0 0.0 base_link zed_camera_link
```

**终端 5 — SRU 强化学习策略**
```bash
ros2 launch rl_nav_controller rl_nav_controller.launch.py
# 默认 odom_topic=/lio/robo/odom（world 系前端里程计），同时额外订阅 /loc/robo/odom（map 系定位输出）
# 输出 "Navigation policy node is ready." + "First odometry message received, odom is ready." 即待命
# 常用参数：lookahead_distance:=2.0  depth_topic:=<深度话题>  report_failure:=false
```

### 3.3 一键脚本部署到 111

```bash
# 本地推送脚本
scp scripts/start_go2w_system.sh scripts/stop_go2w_system.sh agile@192.168.123.111:~/
# 本地一键同步并编译
bash scripts/sync_to_go2w_nav.sh 10.3.42.161
```

---

## 4. 激活导航与发布目标点

### 4.1 解锁底盘导航锁（必须执行）

```bash
ros2 topic pub /startNavi std_msgs/msg/Bool "{data: true}" --once
# 暂停：ros2 topic pub /startNavi std_msgs/msg/Bool "{data: false}" --once
```

### 4.2 健康自检

```bash
ros2 topic hz /lio/robo/odom            # 前端雷达里程计 (world 系) 10~50Hz
ros2 topic hz /loc/robo/odom            # 地图定位里程计 (map 系) 约 10Hz；无输出说明还在 WAIT_INIT
ros2 topic hz /zed/zed_node/depth/depth_registered  # 相机深度 15~30Hz
ros2 topic echo /cmd_vel                # 此时应全为 0（未发目标）
```

### 4.3 下发导航目标点

**方法 A：RViz 鼠标点击发点（推荐）**：工具栏 `2D Goal Pose`（快捷键 `g`），在地图中按住左键拖拽出箭头（箭头朝向 = 到达后的最终朝向）。

**方法 B：命令行发布绝对坐标**

```bash
ros2 topic pub /goal_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'world'}, pose: {position: {x: 1.5, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}" --once
```

悬空安全测试建议先发近距离单次点（如 x=0.8m）。

### 4.4 运行机制与机身应对

- **避障**：ZED 2i 持续捕捉 0.3~5.0m 视场；深度经 VAE 压缩为 2560 维特征实时喂给 LSTM；遇到障碍时输出横向偏移 + 角速度，绕行后重新对准目标。
- **颠簸**：MID-360 内置 IMU 重力对齐（`kf_align_gravity: true`）自动消除机身姿态变化对坐标系的影响。

---

## 5. 紧急接管与安全停止

1. **宇树遥控手柄（第一顺位物理接管）**：拨动手柄摇杆，底层 MCU 自动优先响应；或按 **B 键** 切断自动运动控制。
2. **终端发布停车锁**：`ros2 topic pub /startNavi std_msgs/msg/Bool "{data: false}" --once`
3. **全系统一键停止**：`./stop_nav.sh`（或 `./stop_go2w_system.sh` / `./stop_local_sru.sh`），立刻锁死底盘并终止所有节点。

---

## 6. 模型热替换与 GPU 推理验证

- 将 IsaacLab 训练好的最新权重导出为 `nav_policy.onnx`，scp 覆盖到 `~/go2w_nav_ros2_ws/src/rl_nav_controller/deployment_policies/`；走局部 SRU 模式时另同步到 `~/sru_ws/src/sru-robot-deployment/rl_nav_controller/deployment_policies/`。
- 在 111 上验证 ONNX Runtime 是否启用 CUDA EP：

```bash
python3 -c "import onnxruntime as ort; print(ort.get_available_providers())"
# 应包含 ['CUDAExecutionProvider', 'CPUExecutionProvider']
```

---

## 7. 常见问题与排坑速查

| # | 问题 | 排查 |
|---|---|---|
| 1 | 启动后发 `/goal_pose` 机器狗不动 | 是否发布 `/startNavi=true`；手柄使能状态；策略是否打印 `First odometry message received, odom is ready` |
| 2 | RViz 地图与激光点倾斜 | 点击 `2D Pose Estimate`（`p`）在全局地图上机器狗真实位置点击并拉出朝向 |
| 3 | RViz Image 全黑 | 机器狗趴下时镜头距地面 < 0.2m 处于 ZED 盲区；悬空撑起或站立（离地 > 0.3m），话题设为 `/zed/zed_node/rgb/image_rect_color` 或勾选深度图 `Normalize Range` |
| 4 | 跨机话题发现（DDS） | 两机使用相同 `ROS_DOMAIN_ID`（默认 0）；111 终端必须先 `source unitree_ros2/setup.sh` 加载宇树 DDS 接口与 QoS 配置 |
| 5 | 模型热替换 | 见 §6 |
| 6 | RViz 报 `Fixed Frame [map] does not exist`，且 `ros2 topic hz /loc/robo/odom` 无输出 | 定位节点停在 `WAIT_INIT`，还没收到 `/initialpose`（`map_localization.yaml` 里 `init_pose_source: topic`，`publish_when_invalid: false`）。给一次初始位姿后，动态 `map -> world` TF 与 `/loc/robo/odom` 会同时出现。给过仍无输出：看 Super-LIO 日志有无 `map_dir` 报错 / `LOST` / `FAILED`，`ros2 topic hz /lio/robo/odom` 确认前端在跑，`ros2 run tf2_tools view_frames` 看树里有没有 `world` |
| 7 | 单独跑 `relocation.py` 时 `rviz:=true` 不弹窗 | 该 launch 里 rviz 节点创建后未加入 `LaunchDescription`，手动起：`ros2 run rviz2 rviz2 -d <super_lio 包>/rviz/relocation.rviz` |
| 8 | 局部模式 RViz 正常、全局模式 map 帧报错 | 局部模式 `~/sru_ws/start_local_sru.sh` 额外广播了静态 `map -> world` 恒等 TF；全局 `start_nav.sh` 不广播，必须给 `/initialpose` 让定位节点自己发动态 TF |

---

## 8. 机载路径与部署文档导航

| 领域 | 核心文档 |
|---|---|
| 开机与通信 | 本手册 |
| 架构与联合模式 | `GLOBAL_NAV_INTERFACE.md` |
| 相机调优 | `GO2W_CAMERA_CONFIGURATION_AND_INTEGRATION_GUIDE.md` |
| 场景适配 | `ADAPTIVE_LOOKAHEAD.md`（门区/前瞻）、`GO2W_NAVIGATION_SYSTEM_GUIDE.md`（楼梯 + 终点朝向） |
| 仿真接入 | `GO2W_SIM_HANDOFF.md` |

### 8.1 一键推送代码到机器狗

```bash
# 推送导航工作空间代码
bash sru-robot-deployment/scripts/sync_to_go2w_nav.sh 10.3.42.161
# 同时推送 ONNX 模型权重
scp sru-robot-deployment/go2w_nav_ros2_ws/src/rl_nav_controller/deployment_policies/nav_policy.onnx \
    agile@10.3.42.161:~/go2w_nav_ros2_ws/src/rl_nav_controller/deployment_policies/
```

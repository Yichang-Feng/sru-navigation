# 局部 SRU × VLN 点跟随模块 — 交接文档

> 日期：2026-11-17 ｜ 平台：Unitree Go2W（Jetson Orin DevKit / Ubuntu 22.04 / ROS 2 Humble）
> 本文档只覆盖**局部 SRU 作为 VLN 下层跟随模块**的部分；全局导航（`start_nav.sh`）不在范围内。

---

## 1. 模块定位与本次修改目的

上层将接入一个 **VLN（Vision-Language Navigation）模型**。本模块的职责被收敛为：

- **只做点的跟随**：VLN 通过现有 `/goal_pose` 话题下发**相对机器人 base 的位置点**；
- VLN 判定到点后**停止发点**，SRU 完成最后一个点（10 cm 内）后刹停并等待下一个点；
- **不新增任何话题/节点**，不走 RViz 下点（`--vln` 模式默认关闭 RViz）；
- 里程计照常使用：位姿来自 Super-LIO（`/lio/robo/odom`），无地图依赖。

模块对外就是一个"给相对点 → 走到点 → 停下等下一个点"的跟随器，VLN 只负责决定"下一个相对点在哪 / 何时停发"。

## 2. 硬件、网络与数据流

### 2.1 硬件与网络环境

| 项目 | 说明 |
|:---|:---|
| 高层计算机 | Jetson Orin DevKit（Ubuntu 22.04 + ROS 2 Humble） |
| 深度相机 | **Intel RealSense D435i**（USB 连接，原生深度图 640×480@15Hz + spatial/temporal/hole-filling 滤波）。本项目**不涉及 ZED** |
| 激光雷达 | Livox MID-360（Super-LIO 里程计） |
| 底盘 / 手柄 | Go2W 底盘 + 无线手柄：**[A] 启动/恢复运动，[B] 暂停刹停** |
| 网络 | 机器人自带 WiFi `GL-MT3000-ef3-5G` → **192.168.123.111**；`agile_guest` WiFi → **10.3.42.161**。NoMachine / SSH 任选其一接入 |

### 2.2 数据流

```
VLN 模型（上层）
   │  /goal_pose (geometry_msgs/PoseStamped, frame_id=base_link)
   │  pose.position = 相对当前 base 的位移 [x, y, z] (m)
   ▼
rl_nav_controller (NavigationPolicyNode)
   │  ① 收点瞬间用当前里程计位姿把相对点"钉"到 world 帧（re-latch）
   │  ② SRU 策略网络端到端避障跟随（RealSense 深度图输入）
   │  ③ 到点判定：与钉住的 world 点 2D 水平距离 ≤ 10 cm → 刹停 → IDLE
   ▼
/cmd_vel ──► go2w_bridge ──► 底盘

Super-LIO (map_localization.py) ──► /lio/robo/odom ──► ①③ 的位姿来源（无图依赖）
RealSense D435i ──► /camera/camera/depth/image_rect_raw ──► 策略网络观测
```

## 3. 接口规范（VLN 侧需要知道的）

| 项目 | 内容 |
|:---|:---|
| 话题 | `/goal_pose`，`geometry_msgs/msg/PoseStamped` |
| frame_id | `base_link`（或 `base` / `base_footprint` / `body` / `imu`）；`--vln` 模式下**所有** `/goal_pose` 都按 base 相对点处理 |
| 位置语义 | 相对**收点瞬间**机器人 base 的位移，单位 m |
| 朝向语义 | 可选。orientation norm≈0 表示不约束朝向（保持收点时的车头朝向）；非零时解释为相对当前车头的偏航增量 |
| 距离钳位 | 相对点模长超过 `base_goal_max_distance`（默认 5.0 m）会被按比例钳回，并打警告日志 |
| 到点/停止 | 位置到点半径 `base_goal_arrive_threshold`（**默认 10 cm**）；VLN 判定到点后停发即可 |
| 恢复 | VLN 再次发新点，SRU 自动从 IDLE 回到 RUNNING 继续跟随 |

详细示例与行为细节见 [VLN_BASE_FRAME_GOALS.md](VLN_BASE_FRAME_GOALS.md)。

**发点策略的重要约定（re-latch 语义）**：每一个收到的相对点都会用**收点那一刻**的里程计位姿重新钉到 world 帧，机器人始终跟随**最新**的点。因此 VLN 应满足其一：

1. **分段式**：一次发一个目标点（"前方 2 m 处"），等 SRU 停下后再发下一个；或
2. **收敛式**：VLN 自己跟踪进度，持续发布**剩余位移**（越接近越小，到点前 ≈0）。

⚠️ 不要周期性重复发布同一个非零常数偏移（例如每周期都发 `[1.2, 0]`）——那样目标会一直钉在"当前位姿 + 1.2 m"处，机器人永远追不上、永远不会触发 10 cm 到点。

## 4. 关键行为说明

### 4.1 相对点 → world 点（re-latch）

```
p_world = R(robot_yaw/pitch/roll @ t_receipt) · p_base + p_robot(t_receipt)
target_yaw = robot_yaw(t_receipt) + rel_yaw(可选)
```

实现：`rl_nav_controller.py::_handle_base_frame_goal()`。

### 4.2 到点判定（10 cm）

- `arrive_threshold = base_goal_arrive_threshold`（相对点专用，默认 **0.10 m**；world 绝对点仍用旧的 `arrive_goal_threshold`=0.20 m）；
- 距离为 **world 帧下 2D 水平距离**（不含 z）：`dist = ‖target_pos_w[:2] − robot_pos_w[:2]‖ ≤ 0.10`；
- 策略模型带朝向输出时还有航向检查：到位但航向偏差超过 `arrive_heading_threshold` 会继续原地对头，直到对齐才判到点；
- 到点后：刹停 → `GOAL_REACHED` → `IDLE`，**保持 VLN 模式**（日志：`VLN point reached: waiting for the next base-frame goal.`），下一个 base 相对点直接重启跟随。

### 4.3 "VLN 停发"时的行为（看门狗默认关闭）

- 默认 `goal_stream_timeout = 0`（`--vln` 启动即此值）：**"没有输入"本身不会让 SRU 停下**；机器人会把最后一个收到的点走完（10 cm 内）再刹停。这与"VLN 判定到点后停发"的用法是自洽的——两者都指向同一段路的终点；
- 若希望"VLN 一旦停发就立即急停"（例如 VLN 崩溃/丢包保护），启动时加 `--goal-timeout <秒>`（如 1.5）：RUNNING 状态下超过该时长没有新 base 点 → 立即刹停 → `STOPPED`，丢弃过期目标，等新点恢复。

### 4.4 手动暂停优先级

按 B / motion disable 产生的 STOPPED 是**锁存**的：VLN 继续发点也不会自动恢复（日志 `robot stays STOPPED (manual pause latched)`），需按 A / resume 恢复。防止 VLN 点流覆盖人工急停。

## 5. "10 cm 是怎么算出来的"——没有图，靠里程计吗？

**是的，纯里程计计算，不依赖任何地图。** 具体：

1. VLN 在 t₀ 发相对点 `p_b` → 用 t₀ 时刻的里程计位姿 `(p_robot(t₀), R(t₀))` 一次性钉成 world 点 `p_goal`（§4.1）；
2. 之后每个控制周期，取**当前**里程计位姿 `p_robot(now)`，算 world 帧下 2D 距离 `‖p_robot(now) − p_goal‖`；
3. 因为 `p_goal` 和 `p_robot` 都在同一个里程计（world）坐标系里，这个距离本质上就是"**自收点那一刻起，机器人在里程计意义下累计走了多远**"——不需要图、不需要绝对定位。

误差来源 = 该段路径上的 **LIO 漂移**（Super-LIO 是激光-惯性里程计，短程几米内漂移通常在厘米级），所以 10 cm 到点半径是可实现的。注意两点：

- 距离只算水平面（x, y），z 不参与；
- re-latch 意味着"累计"的参考点是**最后一次收点**的时刻——VLN 每发一个新点，参考系原点就刷新一次（见 §3 的发点策略约定）。

## 6. 部署与启动

### 6.1 一键启动

一键脚本拉起局部 SRU 全链路（底盘桥 → Livox + Super-LIO → RealSense → SRU 策略），**无需单独解锁底盘导航锁**，启动后按手柄 [A] 开始运动。脚本**默认即 VLN 模式**（等价于 `--vln`）：

```bash
cd ~/sru_ws
./start_local_sru.sh          # base 相对点 + 关 RViz + 10 cm 到点（默认行为，无需参数）
```

停止：在启动终端 `Ctrl+C`（自动安全清理全部节点），或另开终端执行 `./stop_local_sru.sh`。

**拉取仓库后的部署步骤**（脚本硬编码机载路径 `~/sru_ws/` 与 `~/go2w_nav_ros2_ws/`，拉取后需把文件放到约定位置一次）：

```bash
cd <meta-repo>/src/sru-navigation          # 例如 ~/Downloads/agile_nav_ws-vln/src/sru-navigation
bash scripts/sync_to_go2w_nav.sh <机器人IP>   # 推送控制器/启动脚本/yaml 到机载并 colcon build（默认 IP 192.168.31.186）

# 模型文件已随仓库提供；若机载还没有，补推以下三个（vae_encoder.onnx 缺失则节点起不来）：
ssh agile@<机器人IP> "mkdir -p ~/sru_ws/model ~/go2w_nav_ros2_ws/src/rl_nav_controller/deployment_policies"
scp go2w_nav_ros2_ws/src/rl_nav_controller/deployment_policies/vae_encoder.onnx \
    go2w_nav_ros2_ws/src/rl_nav_controller/deployment_policies/nav_policy.onnx \
    agile@<机器人IP>:~/go2w_nav_ros2_ws/src/rl_nav_controller/deployment_policies/
scp sru_ws/model/Odin_nav_policy.onnx agile@<机器人IP>:~/sru_ws/model/
# 机载再编译一次，把模型装入 install 目录：
ssh agile@<机器人IP> "bash -c 'source /opt/ros/humble/setup.bash && cd ~/go2w_nav_ros2_ws && colcon build --packages-select rl_nav_controller --symlink-install'"
```

> 若车端是全新机器（尚无 `~/go2w_nav_ros2_ws`），先把仓库里 `go2w_nav_ros2_ws/src/{rl_nav_controller, go2w_bridge, Super-LIO-ros2}` 三个包拷到机载工作空间并 `colcon build`，再执行上述步骤。

### 6.2 start_local_sru.sh 命令行可调参数

| 参数 | 说明 | 默认值 |
|:---|:---|:---|
| `--vln` | VLN 模式：所有 `/goal_pose` 一律按 base 相对点处理，并自动关闭 RViz。**已默认开启**，保留该参数用于显式声明 | **默认开** |
| `--realsense` / `--d435` / `--d435i` | 使用 RealSense D435i 深度相机 | **默认** |
| `--no-rviz` / `--rviz` | 手动关闭/打开 RViz 可视化（默认关闭；旧式 RViz 下点用 `--goal-mode auto --rviz` 恢复） | 关 |
| `--goal-mode <auto\|base\|world>` | 目标帧模式：`auto`=按 frame_id 判别；`base`=全部按相对点（VLN 用法）；`world`=全部按世界绝对点（旧行为） | `auto` |
| `--goal-timeout <秒>` | "无新点"看门狗：RUNNING 状态下超过该时长没收到新 base 点 → 立即刹停。**0 = 关闭**（只靠 10 cm 位置到点） | `0.0`（关） |
| `--goal-arrive <米>` | VLN 相对点的位置到点半径 | `0.10` m |
| 透传参数 | 其余参数原样透传给 `rl_nav_controller` 节点，如 `--policy-path <路径>`（自定义 ONNX 模型）、`--arrive-threshold`、`--odom-topic`、`--lookahead-distance`、`--blocked-timeout` | — |

常用组合示例：

```bash
./start_local_sru.sh                                       # 标准 VLN 部署（默认行为）
./start_local_sru.sh --goal-mode auto --rviz                # 恢复旧行为：RViz 2D Nav Goal 下点调试
./start_local_sru.sh --vln --goal-timeout 1.5                # 额外开启"停发即刹停"看门狗
./start_local_sru.sh --vln --goal-arrive 0.15                # 到点半径调为 15 cm
./start_local_sru.sh --vln --policy-path ~/sru_ws/model/Odin_nav_policy.onnx   # 显式指定模型（不指定时自动加载该路径）
```

### 6.3 VLN 侧下点示例

`--vln` 模式下任意 `/goal_pose` 都按 base 相对点解释。下面这条命令表示"从当前位置向前方 1.5 m 走，到点（10 cm 内）即停"：

```bash
ros2 topic pub /goal_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'world'}, pose: {position: {x: 1.5, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}" --once
```

## 7. 交接注意事项 / TODO

1. **模型文件已随仓库提交**（此前被 .gitignore 忽略，现已显式放行）：`go2w_nav_ros2_ws/src/rl_nav_controller/deployment_policies/{vae_encoder.onnx, nav_policy.onnx}` 与 `sru_ws/model/Odin_nav_policy.onnx`（默认脚本存在时自动加载该模型，缺失则回退 `nav_policy.onnx`）。机载若还没有，按 §6.1 scp 补推；**`vae_encoder.onnx` 启动时无条件加载，缺失则节点起不来**；
2. `sru_ws/` 目录是机载 `~/sru_ws/` 的快照（已含最新 `start_local_sru.sh`）：行为改动请先改 `scripts/` 再同步，或直接以 `scripts/` 为准；
3. 看门狗默认关闭是有意为之（到点主判据是 10 cm 位置判定）；VLN 侧如需"停发即停"语义，用 `--goal-timeout` 显式开启，不要改默认值；
4. VLN 发点务必遵守 §3 的收敛/分段约定，否则会出现"永远追不上目标点"的现象（表现为机器人一直匀速前进不停）；
5. 10 cm 到点精度受 LIO 漂移影响：若实机发现"差一点就停 / 冲过头"，优先调 `--goal-arrive`，其次检查 Super-LIO 定位质量。

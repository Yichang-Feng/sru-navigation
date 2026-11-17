# 相机对比诊断记录器（schema 2.0）使用说明

本目录提供两个工具：

| 文件 | 作用 |
|---|---|
| `door_diagnostic_logger.py` | 实机录制：深度、位姿、目标/路径、控制指令、导航状态、LiDAR 净空、相机元数据、控制参数快照 |
| `compare_camera_sessions.py` | 离线对比两个 session（例如 RealSense vs ZED），输出 Markdown/JSON 报告 |

> 旧版的楼梯深度重映射（`/nav/remapped_depth`、`depth_remapped/`、`heatmaps_remapped/`）已全部移除。
> 新的 CSV 为 113 列，schema 版本见 `session_summary.json` 的 `schema_version`。

---

## 1. 快速开始

在 Jetson 高层脑（`agile@192.168.123.111`）上，先 source 环境：

```bash
source /opt/ros/humble/setup.bash
source ~/bridge_ws/install/setup.bash
source ~/go2w_nav_ros2_ws/install/setup.bash
```

### 1.1 RealSense 运行

```bash
python3 ~/go2w_nav_ros2_ws/door_diagnostic_logger.py \
  --camera realsense \
  --scene S3_door \
  --note "RS run1, default params" \
  --min-depth 0.15 \
  --max-depth 10.0
```

### 1.2 ZED 运行

```bash
python3 ~/go2w_nav_ros2_ws/door_diagnostic_logger.py \
  --camera zed \
  --scene S3_door \
  --note "ZED run1, default params" \
  --min-depth 0.25 \
  --max-depth 10.0
```

### 1.3 自动识别相机

```bash
python3 ~/go2w_nav_ros2_ws/door_diagnostic_logger.py \
  --scene S1_corridor --note "auto camera"
```

自动模式会同时监听 RealSense 与 ZED 深度话题；**同一时间只应有一路相机出图**，否则 `active_depth_topic` 会在两路之间切换。

### 1.4 离线对比

```bash
python3 ~/go2w_nav_ros2_ws/compare_camera_sessions.py \
  --a ~/door_diagnostics/session_..._realsense_S3_door \
  --b ~/door_diagnostics/session_..._zed_S3_door \
  --label-a RealSense \
  --label-b ZED \
  --out ~/door_diagnostics/compare_RS_vs_ZED_S3
```

输出：

- `comparison_report.md`：人读报告
- `comparison_report.json`：完整指标
- `progress_series.csv`：按路径进度 0..1 归一化的时序，便于画图

---

## 2. 录制前检查

1. 机器人已站立/悬空确认，急停/手柄 B 键可用。
2. 深度话题有数据：
   ```bash
   ros2 topic hz /camera/camera/depth/image_rect_raw   # RealSense
   ros2 topic hz /zed/zed_node/depth/depth_registered  # ZED
   ```
3. 定位正常：`ros2 topic hz /lio/robo/odom`，RViz 中机器人位姿不跳变。
4. 控制器已启动，且日志里能看到 `controller_params_*.yaml` 生成（启动后约 4–8 秒）。
5. 起点贴胶带标记；RViz 目标点尽量固定在同一个世界坐标。
6. 同一组对比尽量使用同一电量区间、同一光照、同一地面；A/B 交错跑（RS→ZED→ZED→RS）。

> 路径由 RViz 手动选择，两次不可能完全一致。对比脚本会自动按起点对齐，并输出两条轨迹的平均最近距离；如果 >0.5 m，报告会给出警告，此时应优先看“匹配位姿”与“进度归一化”指标，而不是直接比较总时间。

---

## 3. 数据目录结构

```
~/door_diagnostics/
└── session_YYYYMMDD_HHMMSS_<scene>_<camera>/
    ├── trajectory.csv              # 113 列时序汇总（主表）
    ├── session_meta.json           # 相机、话题、TF、CameraInfo、控制节点参数快照信息
    ├── session_summary.json        # 会话级聚合指标
    ├── session_args.json           # 本次命令行参数
    ├── controller_params_*.yaml    # ros2 param dump 的控制器参数（若可用）
    ├── camera_info.json            # fx/fy/cx/cy/width/height/distortion
    ├── tf_base_to_camera.json      # base_link -> camera TF
    ├── tf_base_to_lidar.json       # base_link -> lidar TF（若可用）
    ├── goals.jsonl                 # 每次 /goal_pose（及 /cur_goal、/goal_3d）事件
    ├── paths/
    │   └── path_0001_<hash>.json   # /global_path 快照（去重后保存）
    ├── depth/
    │   └── frame_000001_depth_40x64.npy   # 策略实际输入（40x64 float32）
    ├── depth_raw/
    │   └── frame_000001_raw_depth.png     # 原始深度 16-bit PNG（默认最大宽 320）
    ├── heatmaps/
    │   └── frame_000001_heatmap.png       # 可视化热力图
    ├── meta/
    │   └── frame_000001_meta.json         # 单帧完整嵌套元数据
    └── rgb/                        # 仅 --record-rgb 时
        └── frame_000001_rgb.jpg
```

`latest` 软链接指向最近一次 session。

---

## 4. 主要记录字段

### 4.1 相机与深度健康（新增，任务一核心）

| 字段 | 含义 |
|---|---|
| `camera_id` | `realsense` / `zed` |
| `depth_topic` / `depth_encoding` | 实际话题与编码 |
| `depth_width` / `depth_height` | 原生分辨率 |
| `depth_fps_est` / `depth_period_ms` | 相机实际帧率/周期 |
| `depth_latency_ms` / `depth_age_ms` | 图像时间戳到回调的年龄（ms） |
| `depth_valid_ratio_raw` | 原始深度有效率（有限且 ≥min_depth 且 ≤max_depth） |
| `depth_nan_ratio` | NaN/Inf 占比（ZED 常见） |
| `depth_zero_ratio` | 精确 0 占比（RealSense 无回波/遮挡常见） |
| `depth_near_ratio` | 0<d<min_depth 占比（近距盲区） |
| `depth_far_ratio` | d>max_depth 占比 |
| `depth_hole_ratio` | NaN/Inf/0 合计占比 |
| `depth_min/p10/median/mean/p90/max` | 有效深度分位数（米） |
| `depth_valid_ratio_40x64` | 预处理后 40×64 输入的有效比例 |
| `min_depth_param` / `max_depth_param` | 记录器复现策略预处理时使用的阈值 |
| `controller_min_depth` / `controller_max_depth` | 从控制器参数快照解析到的阈值（若有） |

### 4.2 目标、路径与导航状态

| 字段 | 含义 |
|---|---|
| `carrot_*` / `target_*` | 前瞻点、目标向量、体坐标系分量、方位象限 |
| `goal_*` | 最近一次目标点、朝向、距离、年龄 |
| `path_*` | 路径点数、长度、终点距离、前方坡度、哈希 |
| `nav_state` / `nav_state_msgs` | `/local_planner/status` 解析结果 |
| `motion_enabled` / `start_navi` | 底盘使能/开闸状态 |
| `cmd_*` / `raw_cmd_*` | 最终 `/cmd_vel` 与控制器原始输出（`/path_manager/.../nav_vel`） |

### 4.3 LiDAR 真值

| 字段 | 含义 |
|---|---|
| `lidar_min_dist_front` | 机体前向走廊（x 0.2–3 m，|y|≤0.6 m）最近点距离 |
| `lidar_min_dist_any` | 更宽走廊（|y|≤1.5 m）最近点距离 |
| `lidar_nearest_x/y/z` | 最近点坐标（base_link，若 TF 可用） |
| `lidar_valid` / `lidar_tf_ok` | 是否收到有效点云 / TF 是否成功 |
| `lidar_frame` / `lidar_points` | 原始坐标系 / 点数 |

LiDAR 支持两种输入：

- `--lidar-topic`（默认 `/livox/points`）`sensor_msgs/PointCloud2`
- `--livox-topic`（默认 `/livox/lidar`）`livox_ros_driver2/CustomMsg`

若当前环境没有 `livox_ros_driver2` Python 包，CustomMsg 分支会自动跳过；可改用 PointCloud2 话题，或 `--no-lidar` 关闭。

### 4.5 IMU 原始量（碰撞/楼梯动态复核）

| 字段 | 含义 |
|---|---|
| `imu_ax/ay/az` | 原始线加速度（m/s²） |
| `imu_gx/gy/gz` | 原始角速度（rad/s） |
| `imu_valid` | 最近 0.5 s 内是否收到 IMU |

默认订阅 `/livox/imu`（MID-360 内置 IMU）。IMU 冲击尖峰可用于离线辅助判断碰撞/磕碰；楼梯场景可用于复核俯仰动态。`--no-imu` 关闭。

### 4.4 自适应/楼梯内部量（可选 topic）

记录器预留了以下字段，若控制器发布对应 JSON topic 则自动记录：

- `/nav/adaptive_debug`：`lookahead`、`l_curv`、`l_clear`、`l_target`、`turn_deg`、`in_narrow`、`clearance_w`
- `/nav/stairs_debug`：`slope`、`dz`、`remap_active`

当前部署代码尚未发布这两个 topic；若需要更细的自适应/楼梯调参，可在控制器里增加一个 `std_msgs/String` JSON 发布器，记录器无需再改。

---

## 5. 参数快照与量程一致性

启动后，记录器会尝试：

```bash
ros2 param dump /rl_nav_controller
ros2 param dump /rl_nav_controller_adaptive
ros2 param dump /rl_nav_controller_stairs
```

第一个成功的节点参数会保存为 `controller_params_<node>.yaml`，并写入 `session_meta.json` / `session_summary.json`。

- 记录器会用控制器参数中的 `min_depth` / `max_depth` 校准自己的预处理（仅在**第一帧之前**生效）。
- 如果记录已开始才发现量程不一致，记录器会打印警告并保持当前值；下次请显式传 `--min-depth`/`--max-depth`。
- 为保证 40×64 输入与策略完全一致，建议始终显式传入与启动脚本相同的量程。

---

## 6. 推荐实验流程（同一段路径）

1. 固定起点（胶带）和目标点（RViz 中同一世界坐标）。
2. 先用 RealSense 跑 3–5 次，再用 ZED 跑 3–5 次；顺序可交错，减少时间漂移。
3. 每次运行单独一个 session，`--note` 写明相机、轮次、异常。
4. 运行结束后检查 `session_summary.json`：
   - `dropped_queue_frames` 应为 0 或很小；
   - `depth_health_mean.valid_ratio_raw` / `hole_ratio_raw` / `latency_ms` 是否合理；
   - `controller_params_node` 是否非空；
   - `camera_info` 是否包含 fx/fy。
5. 用 `compare_camera_sessions.py` 生成报告；先看轨迹相似度，再看匹配位姿动作差与深度健康。

---

## 7. 常用选项

| 选项 | 说明 |
|---|---|
| `--rate 5` | 录制频率（默认 5 Hz，与策略同频） |
| `--no-raw-depth` | 不保存原始深度 PNG（节省空间） |
| `--raw-depth-width 320` | 原始深度 PNG 最大宽度，等比缩放 |
| `--raw-depth-every 5` | 每 N 帧保存一次原始深度 PNG（长时录制省空间） |
| `--record-rgb --rgb-topic ...` | 同时保存 RGB（可选） |
| `--no-lidar` | 关闭 LiDAR 订阅 |
| `--imu-topic /livox/imu` | IMU 话题 |
| `--no-imu` | 关闭 IMU 订阅 |
| `--param-node /your_node` | 指定控制器节点名 |
| `--camera-frame camera_link` | 指定相机 TF child frame |
| `--camera-info-topic ...` | 指定 CameraInfo 话题 |

---

## 8. 已知限制

- 路径由 RViz 手动选择，不能保证完全一致；对比脚本用“起点对齐 + 最近点距离 + 进度归一化”降低影响，但不能消除。
- `goal_pose` 可能被控制器/waypoint 管理器重复发布，`goals.jsonl` 会记录每次事件；如需严格去重可后续按时间/位移过滤。
- LiDAR 的 `min_dist_front` 依赖 `base_link <- lidar_frame` TF；TF 不可用时仍记录原始坐标系下的距离，并在 `lidar_tf_ok=0` 标记。
- 记录器不保存完整分辨率深度（默认 320 宽 PNG）；若需要原始 640×480，可改 `--raw-depth-width 640`。
- 记录器只记录，不判断碰撞/成功率；这些指标由离线分析或人工复核得出。

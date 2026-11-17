# MapLoc 有图定位模块 — 系统说明书

> 版本 v1.0，2026-09-09。面向使用者与维护者。
>
> 本文是**系统说明**：讲清楚它是什么、怎么设计的、参数怎么调、怎么用。
> 设计过程中的决策与实测记录在 `docs/map_localization_design.md`（§17~§21，
> 按轮次记录），本文不重复那些推导，只给结论和用法。

---

## 1. 这是什么

MapLoc 在 Super-LIO 前端之外增加一个**低频后端线程**：用 voxel-slam 建的先验地图
做扫描配准，估计并维护 **odom→map 的漂移修正 `T_M_O`**，对外以里程计频率输出
`T_M_B = T_M_O · T_O_B`。

**前端算法一行没改。** 对 `SuperLIO` 的改动只有 18 行（一个虚析构、一个 IMU 频率钩子），
外加一处数值修复（见 §7.3）。定位逻辑全部在新增的 `loc/` 模块里。

实测效果（真实 bag，2343 s / 2836 m 全程）：

| | 纯里程计 `/lio/odom` | **MapLoc `/loc/odom`** |
|---|---|---|
| ATE RMSE | 7.97 m | **0.0162 m** |
| 中位 | 6.85 m | **0.0099 m** |
| p95 | 14.53 m | **0.0241 m** |
| 垂直均值 | 4.96 m | **0.0044 m** |
| 旋转中位 | 3.28° | **0.129°** |

全程 0 次 LOST、观测拒绝率 0.57%、前端 `[Observe]` 耗时不受影响。

---

## 2. 核心思想：估计漂移，不是估计位姿

这是整个设计的支点，其他结论都从它推出来。

后端估计的是 **`T_M_O`（odom 系相对 map 系的漂移）**，不是机器人位姿。因为：

1. **配准耗时不造成延迟。** `T_M_O` 是慢变量，配准跑 80 ms 也好 5 ms 也好，
   算出来的修正对"现在"依然成立。配准用的是**随帧快照**的 `T_O_B(t_k)`，
   所以观测的定义与配准何时结束无关。输出延迟只取决于前端。
2. **输出频率不受后端限制。** 后端每 0.5 m 匹配一次，输出仍是前端的 10 Hz
   （以及 IMU 频率）。
3. **失效时优雅退化。** 后端拒绝观测 / LOST 时，`T_M_O` 冻结，输出退化为
   "纯里程计 + 最后一次已知修正"，不会跳。

推论：**初始化、重定位、一致性检查全部在漂移域里做，不在位姿域。**
两次匹配的位姿本来就该不同（机器人在走），只有它们隐含的漂移才应该一致。
第一轮在这里栽过——把位姿域的差当成不一致，正常初始化永远无法确认。

---

## 3. 架构

```
┌─ 线程 T0：ROS 执行器 + SuperLIO::process()（既有，2 ms 定时器）──────────┐
│                                                                          │
│  IESKF 前端 ──→ T_O_B, scan_undistort_full_                              │
│       │                                                                  │
│       ├─ 深拷贝点云 + T_O_B + t ──→ FrameQueue（容量 10，满则丢最旧）     │
│       │                                                                  │
│       └─ 读 T_M_O_est（shared_mutex）──→ T_M_B = T_M_O_est · T_O_B       │
│                                     └──→ /loc/odom, TF map→world         │
└──────────────────────────────────────────────────────────────────────────┘
┌─ 线程 T1：MapLocalizer 后端（新增，nice +5）─────────────────────────────┐
│                                                                          │
│  ScanAccumulator ──→ SubmapManager ──→ IRegistrar ──→ DriftEstimator     │
│  (最近 5 帧→Bk 系)   (KD树取20帧拼子图)  (small_gicp)   (门控 + SE3 EKF)  │
│                            ↑                                             │
│                       KeyframeMap（KD树 + LRU 缓存）←── loc_map/          │
└──────────────────────────────────────────────────────────────────────────┘
```

### 3.1 线程契约

| | 规则 |
|---|---|
| T0 | **永不阻塞**。每帧只做：深拷贝约 8k 点（< 0.2 ms）、共享锁读 7 个浮点、合成发布 |
| T1 | **绝不触碰前端成员**（`kf_`、`ivox_`、`measures_`）。唯一输入是被推进来的帧 |
| 交换 | `FrameQueue`（mutex+deque）、`T_M_O_est`（shared_mutex）、`init_request_`、`status_` |
| CPU | 配准跑在 `tbb::task_arena(2)` 里，抢不走前端的核 |

### 3.2 模块与职责

`include/loc/` 下每个模块都**不依赖 ROS**，可以单独单测。

| 模块 | 职责 | 接口要点 |
|---|---|---|
| `KeyframeMap` | 读 `loc_map/`，3D KD 树检索，LRU 按需读点云 | `load / queryNear / cloud / nearestIdXY` |
| `SubmapManager` | 维护机器人周围的拼接子图 | **只有 `update(p_pred)`**，自己判断要不要重建 |
| `ScanAccumulator` | T0→T1 交接 + 多帧合并成 source | `FrameQueue` + `build()` |
| `IRegistrar` | **配准的接缝**，两个适配器 | `setTarget(cloud, version)` / `align()` |
| `DriftEstimator` | 全部后端状态：一个 SE(3) + 6×6 协方差 + 门控链 | `predict / update / reset`，返回 `UpdateReport` |
| `PoseComposer` | 可选平滑层（**默认关闭**） | `compose()` |
| `MapLocalizer` | 状态机 + T1 线程 + 初始化搜索 | 对外仅 5 个方法 |
| `se3_math.h` | double 精度 SE(3) 与**正交化** | 见 §7.3 |
| `MatchLogger` | 逐次匹配 CSV（诊断用，默认关闭） | |

ROS 只出现在 `ros/loc_publisher.*`（全部 `/loc/*` I/O）和
`lio/super_lio_maploc.*`（前端子类）。

### 3.3 几个刻意的设计选择

- **`SubmapManager` 不暴露 small_gicp 类型。** 它只给出 M 系点云 + `version`；
  目标结构（体素图 / KD 树 / 协方差）由 `IRegistrar` 自己按 version 缓存构建。
  换配准后端只动一个文件。
- **`RegResult` 没有 `ok()`。** 拆成 `hasPose()`（有解）与 `converged`（优化器收敛判据）。
  两者混用曾导致"位姿正确但报未收敛"的候选被丢弃，初始化随机失败。
- **`DriftEstimator::update()` 返回结果而不是记日志。** 拒绝时也报告本体修正量，
  测试和状态话题看到的是同一份事实。
- **子图无效时返回无效子图**，不返回上一个。返回旧子图会让调用方对着 150 m 外的
  几何做配准，表现为"配准坏了"而不是"机器人不在地图上"。

---

## 4. 关键机制

### 4.1 观测与 EKF

```
Z_k = T_M_Bk · T_O_Bk⁻¹                      对 T_M_O 的直接观测
Σ_Z = Ad(T_O_Bk) · Σ_gicp · Ad(T_O_Bk)ᵀ      协方差从 Bk 系搬到 O 系
ν   = Log(T̂⁻¹ · Z_k)                         6 维新息
K   = P S⁻¹,  S = P + Σ_Z
T̂  ← T̂ · Exp(Kν),   P ← (I − K) P
```

约定：**右扰动 `T ← T·Exp(δ)`，δ = [ω(3); ρ(3)] 旋转在前**。
这与 `BASIC::hat6/vee6`、`SE3::adjoint()` 和 small_gicp 的 `se3_exp` **完全一致**，
所以 small_gicp 的 Hessian 可以直接当协方差用，全程不需要任何重排。

### 4.2 门控链（按顺序）

分两类，顺序有讲究：

**质量门**（判断"这次配准本身好不好"）
1. `hasPose()` — 有对应点吗
2. `inlier_ratio ≥ track_inlier_min`
3. `rmse ≤ track_rmse_max`（公制，对目标**点云**算，不是对体素图）
4. 退化检测 — `degenerate_t_dims` / `degenerate_r_dims`

**一致性门**（判断"和现有估计合不合"）

5. 本体修正量 `‖e_t‖ < gate_body_t`、`‖e_R‖ < gate_body_r`
6. **速率门**（不依赖 P）— 见下
7. 卡方门 `d² < chi2_gate`

未初始化时一致性门无意义，所以 **bootstrap 分支放在两类之间**。

**速率门**是第二道防线：卡方门随 P 缩放，而 P 随过程噪声缩放，
过程噪声一放大卡方门就形同虚设。速率门直接比较相邻两次被接受的观测——
漂移不可能每走 1 m 就移动超过 `rate_gate_t`，无论滤波器当前怎么想。
**超限的观测是"暂存"而不是丢弃**：下一次若与暂存的一致，说明是真跳变，一起接受；
否则丢掉。代价是真跳变时延迟一次匹配。

### 4.3 初始化与重定位

搜索空间只有 **(x, y, yaw)**：

- `z` 取自地图最近关键帧（**水平最近邻**——3D 查询需要它正要求解的那个 z，
  猜错会取到另一层并自我印证）
- roll/pitch 取自前端（O、M 都重力对齐，本体倾角在两个系下相同）

流程：粗配所有假设 → 按 inlier 排序 → top-K 精配 → **连续两帧漂移一致才确认**。

两个救命的细节：
- 猜测与**同一时刻**的里程计配对（在 `requestInit()` 里快照）。用处理时刻的
  里程计会把中间走过的距离全烤进去——绑架场景实测偏 11 m。
- 每次尝试都**重新落地**（z 取自地图、roll/pitch 取自当前里程计），
  只有 (x,y,yaw) 沿用传播值。否则失败重试会把误差累积出去，实测 z 漂到 −69 m。
- 粗配没有可用候选时，**直接用跟踪配准器从原始假设精配**。
  有些位置粗配完全不工作（2 m 体素），而跟踪配准器能从 1 m/10° 直接收回来。

### 4.4 状态机

```
WAIT_ODOM → WAIT_INIT → INITIALIZING → TRACKING ⇄ LOST
                ↑                          │
                └──────────────────────────┘（人工 initialpose 随时可插）
```

- LOST 判据：连续拒绝 `lost_reject_count` 次，或 `lost_timeout` 秒无接受观测
- LOST 恢复同样要求**两帧漂移一致**（重复结构里进错 TRACKING 比多 LOST 一会儿糟得多）
- 人工 `/initialpose` **永远生效**，包括 TRACKING 中

---

## 5. 使用说明

### 5.1 编译

```bash
cd ~/super-lio
source /home/zhipengli/ws_livox/install/setup.bash   # livox 驱动（必需）
colcon build --packages-select basic super_lio
```

带单测：加 `--cmake-args -DBUILD_TESTING=ON`，产出 `test_loc`、`loc_replay`、`loc_probe_reg`。

> `super_lio` 依赖 `livox_ros_driver2`，编译和运行都要先 source 它的 workspace，
> 否则 CMake 找不到包，或运行时缺 typesupport 库而崩。

### 5.2 第一步：建定位地图（离线，一次）

把 voxel-slam 的 session 转成板上直接用的紧凑地图：

```bash
./build/super_lio/loc_map_builder \
    --session /home/zhipengli/localization/map_self/mid360_init \
    --out     /home/zhipengli/localization/loc_map \
    --edges   /home/zhipengli/localization/map_self/edge.txt
```

`loc_map_builder` **不依赖 ROS**（只要 PCL + Eigen），可以在任何有地图的机器上跑。

输出：

```
loc_map/
  meta.yaml        建图参数、外参、重力、包围盒
  keyframes.txt    id t x y z qx qy qz qw src_begin src_end n_points
  kf/000000.pcd    关键帧点云，各自的 IMU 体系（binary，只有 xyz）
  global_ds.pcd    显示用全局图（M 系）
  edges.txt        原样拷贝，运行时不读
```

自检输出要对得上（偏差超 10% 说明工具或数据变了）：

```
thinned 23372 frames to 2811 keyframes
  gravity from session: [0.0133 0.0109 -9.7470] (tilt 0.101 deg)
  edge check: 107 edges, mean residual 0.0125 m / 0.0934 deg
  on disk: 279.9 MB   (built in 4.7 s)
```

**关键帧点云存在各自的体系**（不是 M 系），这样将来重新 PGO 只需重写
`keyframes.txt`，2811 个 pcd 文件不用动。

调参：`--kf_dist 1.0 --kf_angle 15 --kf_range 60 --kf_voxel 0.25 --global_voxel 0.4`。

> **注意**：`sync-real.sh` 排除了 `*.pcd`，`loc_map/` 要单独 rsync 到板上。

### 5.3 运行

```bash
source /home/zhipengli/ws_livox/install/setup.bash
source install/setup.bash
ros2 launch super_lio map_localization.py \
    map_dir:=/home/zhipengli/localization/loc_map
```

launch 参数：

| 参数 | 默认 | 说明 |
|---|---|---|
| `map_dir` | 用 yaml 里的值 | 覆盖 `lio.maploc.map_dir` |
| `livox` | true | 回放 bag 时设 `false` |
| `rviz` | false | 开 RViz |

**这个 launch 不发静态 `map→world` TF**（`relocation.py` 发了）。
`map→world` 是动态的、就是漂移估计本身，静态发布器会和它打架。

### 5.4 给初值

三种方式，任选：

1. **yaml**（默认）：`init_pose_source: "yaml"` + `init_pose: [x, y, z, roll, pitch, yaw]`
   — **base 系在 map 系下的位姿**，米和度。z 会被地图覆盖。
2. **RViz**：`init_pose_source: "topic"`，用 "2D Pose Estimate" 发 `/initialpose`。
3. **服务**：`ros2 service call /loc/reinit std_srvs/srv/Trigger` — 用 yaml 里的初值重来。

> **yaw 约定容易填反**：`g_odom_robo` 在当前配置里是 180° 翻转（IMU 倒装），
> 所以建图原点处 base 的 yaw 是 **180°**，不是 0。拿不准就设
> `init_yaw_known: false`，让 360° 搜索自己解决（实测 0.5~0.8 s 搞定）。

### 5.5 输出

| 话题 | 类型 | 频率 | 说明 |
|---|---|---|---|
| `/loc/odom` | Odometry | 10 Hz | **主输出**，frame `map`，child `imu` |
| `/loc/robo/odom` | Odometry | 10 Hz | child `base` |
| `/loc/imu/odom`, `/loc/imu/robo/odom` | Odometry | IMU 频率 | `publish_imu_rate` |
| TF `map → world` | | 10 Hz | 动态；下游经 `world→base` 链得到 `map→base` |
| `/loc/status` | super_lio/LocStatus | 每次匹配 + 1 Hz | 状态、inlier、rmse、耗时、拒绝原因 |
| `/loc/map` | PointCloud2 | 启动一次（latched） | 先验地图 |
| `/loc/keyframes` | PoseArray | 启动一次（latched） | 关键帧位姿 |
| `/loc/submap` | PointCloud2 | 子图刷新时 | 当前在匹配的几何 |
| `/loc/odom_raw` | Odometry | 10 Hz | 回归对照，应与 `/loc/odom` 完全相等 |
| `/loc/odom_smooth` | Odometry | 10 Hz | 平滑层输出，`smooth_output` 开启时才有 |

`/loc/map` 和 `/loc/keyframes` 是 **transient_local（latched）**，
后启的 RViz 也能收到。

### 5.6 怎么看它工作得好不好

```bash
ros2 topic echo /loc/status
```

| 字段 | 健康值 | 不对劲时 |
|---|---|---|
| `state` | 3 (TRACKING) | 4 = LOST |
| `inlier_ratio` | > 0.9 | < 0.6 说明地图与现场不符 / 动态物体 |
| `rmse` | 0.09~0.10 m | > 0.2 m 要查 |
| `last_reject` | `accepted` | 见 §6.3 |
| `match_time_ms` | x86 约 3 ms | 板上预期 3~6 倍 |
| `queue_dropped` | 0 | 非 0 = 后端跟不上 |
| `jump_seq` | 初始化后不再变 | 变了说明重定位过 |

RViz 里：橙色的 `/lio/cloud_world` 应该**贴在**青色的 `/loc/submap` 上。

---

## 6. 参数

全部在 `lio.maploc.*` 命名空间，共 73 个。启动时会把**生效值**全部打一遍
INFO 日志——C++ 默认值和 yaml 双写，只改一边等于没改（这个坑踩过一次，
yaml 里的旧值把代码改动全盖掉了，而离线测试用的是 C++ 默认值所以测不出来）。

### 6.1 常用的（按调整频率排）

| 参数 | 默认 | 说明 |
|---|---|---|
| `map_dir` | "" | **必填**，`loc_map/` 路径 |
| `init_pose` | [0×6] | base 系初值，x y z roll pitch yaw（米/度） |
| `init_pose_source` | "yaml" | `yaml` / `topic` |
| `init_yaw_known` | true | false = 360° 搜索，yaw 拿不准时用 |
| `preload_all` / `lru_capacity` | false / 300 | 内存充裕可以全载 |
| `num_threads` | 2 | 后端 tbb arena 大小 |

### 6.2 精度与频率

| 参数 | 默认 | 依据 |
|---|---|---|
| `match_min_dist` | 0.5 m | 实测误差**不随匹配间隔增长**（<0.25 s 中位 0.0107 m，>2 s 中位 0.0097 m），缩短没有收益 |
| `accum_frames` | 5 | 5/3/1 精度基本相同，但拒绝率 0.43%→1.15%→4.87% |
| `source_voxel` / `target_voxel` | 0.25 / 0.5 m | |
| `max_corr_dist` | 1.0 m | **别为了"总能匹配上"调大**，只会引入错误修正 |
| `submap_radius` / `submap_max_k` | 15 m / 20 | |

### 6.3 门控（拒绝率高时看这里）

| 参数 | 默认 | `last_reject` |
|---|---|---|
| `track_inlier_min` | 0.5 | `low_inlier` |
| `track_rmse_max` | 0.3 m | `high_rmse` |
| `gate_body_t` / `gate_body_r` | 1.0 m / 5° | `body_gate` |
| `chi2_gate` | 16.8 | `chi2` |
| `rate_gate_t` / `rate_gate_r` | 0.6 m/m / 0.05 rad/m | `rate_gate` |
| `max_degenerate_t_dims` / `_r_dims` | 1 / 0 | `degenerate` |

### 6.4 过程噪声

```yaml
lio.maploc.sigma_t_per_m: 0.05      # [m] 每走 1 m
lio.maploc.sigma_r_per_m: 0.002     # [rad] 每走 1 m
lio.maploc.sigma_t_per_s: 0.02      # [m] 每秒（静止时也在漂）
lio.maploc.sigma_r_per_s: 0.0005    # [rad] 每秒
```

**这四个值是量出来的，不是猜的**：把匹配时刻快照的里程计位姿与 PGO 轨迹按实际
匹配间隔比对，中位 0.0185 m/m、p95 0.051。按时间的那两项是因为本数据 9% 的时间
机器人静止，静止时距离项不增长，P 会重新变得过度自信。

> **踩过的坑**：曾经把 0.5 m 窗口的统计（0.18 m/m）当成漂移率，那是短窗口除以
> 时间戳配准噪声造成的假象。设成 0.25 之后卡方门形同虚设——全程 0 次拒绝
> 看着健康，其实是门开着。**0 拒绝率不是成功指标。**

### 6.5 输出

```yaml
lio.maploc.smooth_output: false     # 平滑层，默认关闭
lio.maploc.publish_imu_rate: true
lio.maploc.save_trajectory: ""      # TUM 轨迹（IMU 位姿），退出时写
lio.maploc.match_log: ""            # 逐次匹配 CSV，诊断用
```

**`/loc/odom` 直接输出估计值，中间没有限速层。** 实测限速层作为主输出时
ATE RMSE 0.601 vs 0.190 m、旋转 1.637° vs 0.159°，而且它自己制造了全程最大的
单帧跳变（4.86 m）——它在"精度"和"平滑"两件事上同时更差。

现在默认路径每帧修正 **p95 5.8 mm、超 2 cm 仅 0.1%**，已经足够平滑。
`/loc/odom_smooth` 接口留着，下游真有抖动投诉再启用并单独验收。

---

## 7. 已知的坑与注意事项

### 7.1 数据相关

1. **地图强度全为 0** — voxel-slam 没写 intensity，配准和地图都不能用它。
2. **外参必须一致** — 地图点云已含建图时的 lidar→imu 外参，前端点云用
   `lio.extrinsic.lidar_imu`。不一致会引入常值偏差，启动时会 WARNING。
3. **`scan_undistort_full_` 每帧原地复用** — 交给 T1 前必须深拷贝。
4. **`+0.79` 与静态 TF** — 新节点不走 `set_initial_data`，launch 不发静态
   `map→world`。老的 `relocation.py` 两样都有。

### 7.2 参数相关

5. **C++ 默认值与 yaml 双写** — 只改一边等于没改。启动日志会打全部生效值。
6. **杠杆臂** — odom 原点离当前位置几百米时，`T_M_O` 的平移协方差被 yaw 放大，
   这是物理事实。**不要用 `T_M_O` 的平移差直接判跳变**，门控和限速都在本体视角下评估。

### 7.3 数值：旋转矩阵不正交（重要）

**代码库里没有任何地方对旋转矩阵做正交化。** `SO3::operator*` 是普通的 float
3×3 乘法，`ESKF::R_` 每个 IMU 样本乘一次（39 分钟约 47 万次），
`‖RᵀR−I‖` 只增不减——实测跑到 **6e-3**。

发布出去的四元数是干净的（`quaternion()` 输出时归一化），**所以这个问题被掩盖了**，
但内部矩阵不是。而 `SE3::inverse()` 用转置当逆：`R = k·Q` 时
`R·Rᵀ = k²·I`，任何做共轭的地方每帧被乘一个 `k²`，**指数累积**。
实测 `k = 1+1e-5` 时 2 万帧后 `det` 到 1.41。

**现在的处理**：
- `MapLocalizer::pushFrame` 对每个进入后端的 `T_O_B` 做四元数往返
- `DriftEstimator` 每次状态更新后正交化
- `PoseComposer` 共轭后正交化
- **`ESKF`**：`Update()` 每次、`Predict()` 每 200 个 IMU 样本各做一次

修完 `‖RᵀR−I‖` 全程 < 3e-7 且不再增长。顺带前端自己也变好了：
`/lio/odom` 后半程旋转中位 4.28° → 3.85°。

> 维护提醒：`SE3::inverse()` / `SO3::inverse()` 用转置当逆，**前提是 R 正交**。
> 现在两侧都做了正交化，前提成立。新增任何长期迭代旋转的代码，记得也要正交化。

---

## 8. 测试

### 8.1 单元测试（不依赖 ROS 和地图）

```bash
colcon build --packages-select super_lio --cmake-args -DBUILD_TESTING=ON
./build/super_lio/test_loc          # 35 项
```

覆盖：SE(3) 约定与伴随、EKF 收敛、每一道门、速率门的"暂存-确认"、
GPS 位置观测、限速器、队列与累积、**旋转正交性累积**。

### 8.2 合成回放（需要 session PCD，不需要 ROS）

```bash
./build/super_lio/loc_replay /home/zhipengli/localization/loc_map
```

注入随距离增长的漂移 + 一次里程计跳变 + 一次绑架，检验跟踪/LOST/重定位。

| 环境变量 | 用途 |
|---|---|
| `MAPLOC_SLEEP_MS=100` | 实时节奏。**测恢复延迟必须用它**——默认 6 ms/帧下一次搜索等于 40 个"帧" |
| `MAPLOC_NO_DRIFT=1` | 不注入漂移，剩下的就是配准精度本身 |
| `MAPLOC_CORRUPT_EVERY=20` | 每 N 帧整体平移 source，测门的包络 |
| `MAPLOC_ROT_SCALE=1e-5` | 给 `T_O_B` 旋转块乘 (1+ε)，测数值累积 |
| `MAPLOC_INFO=1` | 打开后端 INFO 日志 |

### 8.3 配准探针

```bash
./build/super_lio/loc_probe_reg /home/zhipengli/localization/loc_map 12062
```

在某一帧上从一组偏差初值出发，打印收敛性/inlier/rmse/位姿误差。
**判断粗配参数够不够用就靠它。**

### 8.4 真 bag 评测

```bash
src/super_lio/test/tools/run_bag_eval.sh <outdir> <params.yaml> [rate] [domain_id]
python3 src/super_lio/test/tools/wp3_accuracy.py <outdir>
```

建图那趟 bag 的 `alidarState.txt` 就是它自己的真值（同一时钟、同一传感器、
PGO 优化过），可以直接按时间戳逐帧比，**不要做 Umeyama 对齐**——对齐会把要测的偏差抹掉。

`test/tools/` 下还有 `wp1_diag.py`（D1~D6 诊断）、`ate.py`、`compare_runs.py`、
`odom_to_tum.py` 等。

> 并行跑多个配置时**必须给不同的 `ROS_DOMAIN_ID`**，否则多个节点的 `/loc/*`
> 挤在同一个图里，录制脚本会串台。`run_bag_eval.sh` 也会拒绝在已有运行时启动。

---

## 9. 性能

x86 开发机（32 核），全程实测：

| 环节 | 耗时 |
|---|---|
| T0 拷贝帧 + 合成输出 | < 0.2 ms |
| VGICP 单次配准 | **3 ms**（1.1 万 source，17 万点子图，2 线程） |
| 子图重建（缓存命中） | 0.8~1.2 ms |
| 子图重建（含协方差估计） | 约 35 ms |
| `ScanAccumulator::build()` | 0.47 ms |
| 初始化（360° 搜索） | 0.5~0.8 s |

后端每 0.5 m 匹配一次，1.2 m/s 步行约 2.4 Hz。
`build()` 只在真要匹配时才做（4999 帧进来只做了 1091 次）。

内存：位姿 + KD 树 < 1 MB，LRU 300 帧约 50 MB，子图 + 体素图约 20 MB。

**板上（Jetson Orin NX 级）尚未实测**，预期为 x86 的 3~6 倍。

---

## 10. 未做 / 下一步

1. **板上验证** — 编译（GCC 9 / TBB 2020 / Foxy）与耗时都没测过，
   这是目前最大的未知。
2. **平滑层 v2** — 接口在，默认关闭。下游真有抖动需求再按设计文档 §21.8 做。
3. **GPS** — `DriftEstimator::addPositionMeasurement()` 已实现并有单测，
   但 `T_ENU_M` 的标定和 ROS 接线没做。
4. `/loc/odom_raw` 确认无用后可以删。

---

## 11. 文件索引

```
src/super_lio/
  3rdparty/small_gicp/              vendored（MIT，commit 2c6c783）
  include/loc/                      后端模块（无 ROS 依赖）
  include/lio/super_lio_maploc.h    前端子类
  include/ros/loc_publisher.h       全部 /loc ROS I/O
  src/apps/map_localization_node.cpp   定位节点
  src/apps/loc_map_builder.cpp         离线建图工具（无 ROS）
  msg/LocStatus.msg
  config/map_localization.yaml         实机配置
  config/map_localization_bag.yaml     bag 回放配置
  launch/map_localization.py
  rviz/map_localization.rviz
  test/test_loc.cpp                    35 项单测
  test/tools/                          回放、探针、评测脚本

docs/
  maploc_manual.md                  本文
  map_localization_design.md        设计与实测记录（§17~§21）
  maploc_test_plan.md               第一轮测试方案
  maploc_round2_plan.md             第二轮
  maploc_round3_plan.md             第三轮
```

# Unitree Go2W 双深度相机（RealSense D435 与 ZED 2i）配置与感知集成全景参考指南

**版本**: v1.0  
**适用机型**: Unitree Go2W 轮足四足机器人  
**对应模块**: `rl_nav_controller.py`, `rl_nav_controller_adaptive.py`, `rl_nav_controller_stairs.py`, `start_nav.sh`, `constants.py`, `door_diagnostic_logger.py`, `test_realsense_max_distance.py`  
**维护状态**: 最新生效文档（适用于实车部署与离线排错）

---

## 一、 系统架构与相机选型背景

在 Go2W 自主导航系统中，局部避障与路径跟踪完全依赖基于强化学习（RL）端到端训练的策略网络（`nav_policy.onnx` + `vae_encoder.onnx`）。该策略在仿真（Isaac Sim / IsaacLab）中以**纯深度图像（$40\times 64$ 浮点矩阵）**为唯一环境感知输入，不依赖也不接收任何 RGB 彩色图像。

为了满足不同任务场景与传感器条件，系统构建了**双相机无缝切换架构**，并在启动层与控制层原生适配了两种不同原理的立体深度相机：
1. **Intel RealSense D435 / D435i**（主动红外散斑双目，轻量、低延迟、功耗小、近距避障极佳）；
2. **Stereolabs ZED 2i**（被动自然光长基线双目，长距离测距、大视场角、穿透力强）。

```
           ┌──────────────────────────────────────┐
           │     Unitree Go2W 视觉传感器选型      │
           └──────────────────┬───────────────────┘
                              │
          ┌───────────────────┴───────────────────┐
          ▼                                       ▼
 ┌───────────────────────┐             ┌───────────────────────┐
 │ Intel RealSense D435i │             │    Stereolabs ZED 2i  │
 │  主动红外双目 (50mm基线) │             │  被动立体双目 (120mm基线)│
 │  原生视场: 87° x 58°   │             │  超广视场: 110° x 70° │
 └──────────┬────────────┘             └──────────┬────────────┘
            │                                     │
            ▼ [/camera/camera/depth/image_rect_raw]▼ [/zed/zed_node/depth/depth_registered]
            │ (16UC1: 毫米整型)                   │ (32FC1: 米制浮点)
            └──────────────────┬──────────────────┘
                               │
                               ▼
        ┌──────────────────────────────────────────────┐
        │       深度图标准化预处理 (depth_callback)      │
        │ 1. 统一转为 float32 (米)                      │
        │ 2. 盲区与近距截断 (< min_depth 夹取为 min)    │
        │ 3. 远景与超量程映射 (> max_depth 映射为 0.0)  │
        │ 4. 双线性重采样为 40x64 (INTER_LINEAR)       │
        └──────────────────────┬───────────────────────┘
                               │
                               ▼
        ┌──────────────────────────────────────────────┐
        │   VAE 深度特征编码器 (vae_encoder.onnx)       │
        │        输入: [1, 1, 40, 64] -> 输出: 2560 维  │
        └──────────────────────┬───────────────────────┘
                               │
                               ▼
        ┌──────────────────────────────────────────────┐
        │  RL 策略网络与动作推理 (nav_policy.onnx)      │
        │        输出: 3 维高层动作指令 (cmd_vel)        │
        └──────────────────────────────────────────────┘
```

---

## 二、 两款相机核心硬件与成像特性对比

| 指标维度 | **Intel RealSense D435 / D435i** | **Stereolabs ZED 2i** | 实车导航影响与工程考量 |
| :--- | :--- | :--- | :--- |
| **立体测距原理** | **主动红外双目散斑 (Active IR Stereo)** | **被动双目视觉 (Passive Stereo)** | D435 靠投射红外点阵；ZED 靠自然光与纹理 |
| **物理基线 (Baseline)** | **$50\,\text{mm}$ (5 cm)** | **$120\,\text{mm}$ (12 cm)** | ZED 基线长 2.4 倍，远距误差增长明显慢于 D435 |
| **深度镜头原生 FOV** | **$87^\circ \text{H} \times 58^\circ \text{V}$ (大广角)** | **$110^\circ \text{H} \times 70^\circ \text{V}$ (超广角)** | 原生深度视野均很开阔 |
| **RGB 彩色镜头 FOV** | **$69^\circ \text{H} \times 42^\circ \text{V}$ (偏窄)** | 与深度共用物理镜头 ($110^\circ \times 70^\circ$) | **D435 严禁对齐到 RGB，否则视野会被切掉近 20°！** |
| **标准有效量程** | **$0.15\,\text{m} \sim 10.0\,\text{m}$** (理想 $0.3\sim 3\text{m}$) | **$0.25\,\text{m} \sim 10.0\,\text{m}$** (理想 $0.5\sim 15\text{m}$) | D435 经滤波增强后室内走廊可稳定测至 6~8m |
| **ROS 2 图像编码** | `16UC1` / `mono16` (16位无符号整型毫米) | `32FC1` (32位单精度浮点米) | 预处理根据 `encoding` 自动进行 $/1000.0$ 换算 |
| **机载算力与依赖** | 极低（板载 D4 芯片直接硬件计算视差） | 依赖 NVIDIA Jetson GPU 运行 TensorRT | D435 极大减轻机载 GPU 压力，释放算力给雷达定位 |
| **外参安装坐标** | `[0.34, 0.0, 0.25]` (距底盘 base_link) | `[0.34, 0.0, 0.25]` (相近支架) | 通过 TF 广播静态外参保持位姿一致 |

---

## 三、 Intel RealSense D435 / D435i 参数配置与特殊处理规范

### 1. 订阅话题配置
* **标准生效话题**：**`/camera/camera/depth/image_rect_raw`**
* **图像编码格式**：`16UC1`（像素值为毫米，预处理自动除以 1000 转换为浮点米）
* **备用/兼容话题**：`/camera/depth/image_rect_raw`（无多级命名空间时的备用通道）

### 2. 为什么绝对不能使用 `aligned_depth_to_color`（深度对齐到彩色）？
前期测试中机器人“在门前停滞不前、视野两侧过早变黑”的核心根源，即在于错误订阅了 `/camera/camera/aligned_depth_to_color/image_raw`。原因如下：
1. **视场角强行阉割（左右丢掉近 20°，上下丢掉 16°）**：
   D435 红外深度视场角为 $87^\circ \times 58^\circ$，而 RGB 彩色镜头仅为 $69^\circ \times 42^\circ$。对齐算法会把超出 RGB 视场的深度数据全部裁剪赋零（刷黑），导致机器人近地和通道两侧过早失去视差；
2. **跨镜头视差遮挡黑洞（Parallax Occlusion Holes）**：
   RGB 镜头与红外镜头横向相隔 $17.5\,\text{mm}$。门框、立柱、墙角等边缘区域，深度镜头看得见但 RGB 镜头被遮挡，重投影无法匹配，导致门洞四周产生大面积纯黑无效孔洞，直接触发 RL 策略的断崖/障碍避障拒止；
3. **远距视差二次插值丢失**：
   重投影光栅化插值会导致微弱的远距视差舍入丢失；
4. **算力空耗与时延增加**：
   策略网络根本不需要彩色图，开启对齐白白增加 15~30ms 时延与机载 CPU/GPU 内存拷贝开销。

### 3. 驱动层参数覆盖配置 (`realsense_nav_override.yaml`)
配置文件位于：[`src/rl_nav_controller/config/realsense_nav_override.yaml`](file:///home/yichangfeng/sru-navigation-learning/scripts/realsense_nav_override.yaml)
```yaml
/**:
    ros__parameters:
        depth_module:
            emitter_enabled: 1         # 开启激光投影散斑
            laser_power: 360.0         # 激光器功率调至最大 360mW，强化远距室内反射
            visual_preset: 4           # 4 = High Density (高密度模式，降低视差阈值填补空洞)
            enable_auto_exposure: true # 开启自动曝光
            hdr_enabled: false

        # 空间平滑滤波 (去除平面高频视差噪点)
        spatial_filter:
            enable: true
            filter_magnitude: 2
            filter_smooth_alpha: 0.5
            filter_smooth_delta: 20
            holes_fill: 2

        # 时间累积滤波 (跨帧稳定远景走廊，大幅降低远距噪点抖动)
        temporal_filter:
            enable: true
            filter_smooth_alpha: 0.4
            filter_smooth_delta: 20
            holes_fill: 3

        # 空洞填平滤波 (利用邻近几何填平反光/弱纹理黑洞，防止虚空误判)
        hole_filling_filter:
            enable: true
            holes_fill: 2              # 2 = nearest

        clip_distance: -1.0            # -1.0 禁用底层驱动距离截断限制
```

### 4. ROS 2 Launch 节点启动命令行 (`start_nav.sh`)
```bash
DEPTH_TOPIC="/camera/camera/depth/image_rect_raw"
CAMERA_MIN_DEPTH="0.15"
CAMERA_MAX_DEPTH="10.0"

# 广播相机外参 (base_link -> camera_link)
nohup ros2 run tf2_ros static_transform_publisher 0.34 0.0 0.25 0.0 0.0 0.0 base_link camera_link >/dev/null 2>&1 &

# 启动 RealSense 原生深度流，关闭对齐，加载高密度与滤波流水线
RS_CFG="$WS/src/rl_nav_controller/config/realsense_nav_override.yaml"
ros2 launch realsense2_camera rs_launch.py \
    config_file:="$RS_CFG" \
    depth_module.profile:=640x480x15 \
    rgb_camera.profile:=640x480x15 \
    align_depth.enable:=false \
    pointcloud.enable:=false \
    clip_distance:=-1.0 \
    filters:=spatial,temporal,hole_filling
```

---

## 四、 Stereolabs ZED 2i 参数配置与特殊处理规范

### 1. 订阅话题配置
* **标准生效话题**：**`/zed/zed_node/depth/depth_registered`**
* **图像编码格式**：`32FC1`（单通道 32 位浮点，单位：米）

### 2. 算力减负与防丢帧专属优化
ZED 2i 原生驱动全开时包含 RGB、点云、视觉 SLAM（VIO）、空间测绘等大量派生流，单帧处理延迟高达 ~75ms，极易因 Jetson GPU 争抢导致深度卡顿撞障。系统通过专用参数文件实现彻底轻量化减负：

### 3. 驱动层参数覆盖配置 (`zed_nav_override.yaml`)
配置文件位于：[`src/rl_nav_controller/config/zed_nav_override.yaml`](file:///home/yichangfeng/sru-navigation-learning/scripts/zed_nav_override.yaml)
```yaml
/**:
    ros__parameters:
        general:
            grab_resolution: 'HD720'          # 1280x720 分辨率采集
            grab_frame_rate: 15               # 15 FPS 稳定帧率
            pub_frame_rate: 15.0              # 广播频率锁定 15.0 Hz
            pub_resolution: 'CUSTOM'          # 自定义缩放广播
            pub_downscale_factor: 2.0         # 缩放至 640x360 降低 DDS 传输开销

        # 彻底关闭 RGB 与双目图像发布 (策略纯靠深度)
        video:
            publish_rgb: false
            publish_left_right: false
            publish_raw: false
            publish_gray: false
            publish_stereo: false

        # 深度图与模型配置
        depth:
            depth_mode: 'NEURAL_PLUS'         # 深度模型
            depth_stabilization: 0            # 禁用深度稳定防背景拉扯
            publish_depth_map: true           # 仅保留深度图
            publish_point_cloud: false        # 极关键：关闭点云释放数十兆带宽
            publish_depth_info: false
            publish_depth_confidence: false
            publish_disparity: false

        # 彻底关闭自带 VIO/视觉里程计 (位姿由 Super-LIO 雷达负责，避免冲突)
        pos_tracking:
            pos_tracking_enabled: false
            publish_tf: false
            publish_odom_pose: false
            publish_cam_path: false

        sensors:
            publish_imu: false                # 关闭 ZED IMU (使用 MID360 内部 IMU)
```

### 4. ROS 2 Launch 节点启动命令行 (`start_nav.sh`)
```bash
DEPTH_TOPIC="/zed/zed_node/depth/depth_registered"
CAMERA_MIN_DEPTH="0.25"
CAMERA_MAX_DEPTH="10.0"

ZED_CFG="$WS/src/rl_nav_controller/config/zed_nav_override.yaml"
ros2 launch zed_wrapper zed_camera.launch.py \
    camera_model:=zed2i \
    ros_params_override_path:="$ZED_CFG"
```

---

## 五、 控制器内部统一预处理与量程处理逻辑 (`depth_callback`)

在 [`rl_nav_controller.py`](file:///home/yichangfeng/sru-navigation-learning/rl_nav_controller.py#L1220-L1250) 中，无论接入哪款相机，均通过以下严密机制完成无缝标准化：

```python
# 1. 自动格式识别与单位统一至浮点“米”
depth_raw = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
if getattr(depth_msg, 'encoding', '') in ('16UC1', 'mono16') or depth_raw.dtype == np.uint16:
    depth_array = depth_raw.astype(np.float32) / 1000.0   # RealSense D435 (毫米 -> 米)
else:
    depth_array = depth_raw.astype(np.float32)            # ZED 2i (直接为米)

# 2. 解决近墙盲区与远距开阔地的“0.0 歧义性问题”
is_nan = np.isnan(depth_array)
is_posinf = np.isposinf(depth_array)
is_neginf = np.isneginf(depth_array)

# 远景与超量程 (+inf 或 > max_depth) 映射为 0.0 (与 VAE 训练集开阔远景特征对齐)
far_mask = is_posinf | (depth_array > self.max_depth)

# 近距离盲区、负无穷、或非远景异常点 (< min_depth) 强行截断为 min_depth (如 0.15m)
# 作用：防止贴墙或障碍物近身盲区被误判为 0.0 虚空通透地带！
close_mask = is_neginf | ((depth_array < self.min_depth) & ~is_nan & ~far_mask)
nan_mask = is_nan

depth_array[far_mask] = 0.0
depth_array[close_mask] = self.min_depth
depth_array[nan_mask] = self.min_depth

# 3. 双线性下采样为强化学习标准输入规格 (40x64)
depth_40x64 = cv2.resize(depth_array, (64, 40), interpolation=cv2.INTER_LINEAR)
```

---

## 六、 常用操作、切换与诊断指令速查

### 1. 启动相机与导航系统
```bash
# 模式 A: 使用 Intel RealSense D435/D435i 相机启动 (默认)
bash start_nav.sh --realsense

# 模式 B: 使用 Stereolabs ZED 2i 相机启动
bash start_nav.sh --zed
```

### 2. 实时测距能力与深度质量检测
在机器人或机载终端执行专用检测工具，实时查看深度有效覆盖率、最远探测距离及走廊穿透能力：
```bash
# 默认监听 RealSense 原生深度图 (/camera/camera/depth/image_rect_raw)
python3 scripts/test_realsense_max_distance.py

# 显式测试 ZED 深度图
python3 scripts/test_realsense_max_distance.py --topic /zed/zed_node/depth/depth_registered
```

### 3. 一键同步部署至真机
在本地开发机修改参数后，推送并编译至机器狗机载电脑：
```bash
bash scripts/sync_to_go2w_nav.sh [ROBOT_IP]
```
*(默认目标 IP 为 `192.168.31.186`，机载用户 `agile`)*

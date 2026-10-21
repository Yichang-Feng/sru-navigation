# Unitree Go2-W 实机部署 —— 详细执行计划与方案

- **更新日期**：2026-10-20
- **前置条件**：Isaac Lab 仿真中 `Isaac-Nav-PPO-Go2W-Play-v0` 已完整跑通（见 [`GO2W_SIM_HANDOFF.md`](../sru-navigation-learning/GO2W_SIM_HANDOFF.md)）
- **目标**：将验证过的高层 SRU 导航策略部署到真实 Unitree Go2-W 轮足四足机器人，实现视觉自主避障导航

---

## 1. 系统架构总览

### 1.1 分层控制架构

```
┌─────────────────────────────────────────────────────────────┐
│              感知输入 (Perception & Odometry)                │
│  - ZED-X 深度相机 (0° 水平): /zed/zed_node/depth/...        │
│  - 激光惯性里程计 (DLIO/Fast-LIO): /dlio/odom_node/odom     │
│  - 目标点: /goal_pose    - 手柄安全开关: /rsl_joy           │
└──────────────────────────────┬──────────────────────────────┘
                               │
                               ▼
┌─────────────────────────────────────────────────────────────┐
│   高层导航节点: rl_nav_controller (5 Hz)                     │
│  - VAE 深度编码: vae_encoder.onnx (40×64 → 2560维)          │
│  - SRU 循环策略: nav_policy.onnx (2576维输入 → [vx,vy,ωz])   │
│  - 输出: /path_manager/path_manager_ros/nav_vel (Twist)     │
└──────────────────────────────┬──────────────────────────────┘
                               │ cmd_vel (vx, vy, omega_z)
                               ▼
┌─────────────────────────────────────────────────────────────┐
│   底层运动控制 —— Unitree Sport Mode (官方 SDK)              │
│  - unitree_sdk2py SportClient.Move(vx, vy, vyaw)           │
│  - 内置 MPC/WBC 步态 + 防滑 + 跌倒自恢复                     │
└─────────────────────────────────────────────────────────────┘
```

### 1.2 核心策略模型（机器人无关）

| 模型 | 文件 | 输入 | 输出 | 频率 |
|:---|:---|:---|:---|:---|
| VAE 深度编码器 | `rl_nav_controller/deployment_policies/vae_encoder.onnx` | `[1,1,40,64]` 深度图 | `[1,2560]` 潜表征 | 5 Hz |
| SRU 导航策略 | `rl_nav_controller/deployment_policies/nav_policy.onnx` | obs: `[1,2576]`, h/c: `[1,1,512]` | actions: `[1,3]` (vx,vy,ωz) | 5 Hz |

> **关键认知**：高层导航策略是**机器人无关（robot-agnostic）**的——它只看深度图和目标点，输出标准速度指令。B2W 和 Go2W 共用同一套权重，无需重新训练。

---

## 2. 部署方案选择

### 方案 A（推荐）：宇树官方 Sport Mode

**适用场景**：快速落地验证、日常使用、安全优先

| 维度 | 说明 |
|:---|:---|
| 原理 | 宇树出厂固件内置成熟的轮足运动控制器（MPC/WBC + 动态阻尼），接收 Twist 指令即可行走 |
| 优势 | 零摔机风险；落地周期 1~2 小时；可用遥控器一键接管；无需额外训练/部署底层策略 |
| 劣势 | 无法使用自研 RL 步态；速度上限受官方固件限制（通常 vx ≤ 2.0 m/s） |
| 依赖 | `unitree_sdk2py` Python 包 + 网线连接 Go2W 内部网络 |

### 方案 B（备选）：自研 RL 步态 (rl_sar)

**适用场景**：课题要求全流程端到端 RL、需要超越官方固件的运动性能

| 维度 | 说明 |
|:---|:---|
| 原理 | 部署 `rl_sar` 低层 RL 策略（50 Hz 推理 / 200 Hz 指令），直接驱动 16 个关节电机 |
| 优势 | 全流程端到端 RL；可定制步态特性；速度/负载能力可能更优 |
| 劣势 | 部署复杂度高（需 CAN 总线直连）；存在摔机风险；调试周期长 |
| 依赖 | `rl_sar` C++ 运行时 + Go2W 底层 CAN 接口 + 适配后的 `policy_go2w_adapted.pt` |

**执行计划以方案 A 为主线，方案 B 作为后续扩展。**

---

## 3. 硬件与网络准备

### 3.1 硬件清单

| 组件 | 规格/型号 | 用途 |
|:---|:---|:---|
| Unitree Go2-W | 轮足四足机器人本体 | 被控平台 |
| ZED-X 深度相机 | Stereolabs ZED-X，安装于头部前方 | 感知输入（深度图） |
| 上位机 | Jetson Orin NX/AGX 或 NUC (i7+) / 笔记本 | 运行 ROS 2 + ONNX Runtime |
| 手柄 | PS5 DualSense / Xbox 无线手柄 | 安全死人开关 + 遥控 |
| 网线 | Cat5e 及以上 | 上位机 ↔ Go2W 内部网络 |

### 3.2 安装位置与外参（关键）

ZED-X 相机安装在 Go2W **头部前方**，相对 `base` 坐标系：

```
平移: x=0.285m, y=0.0m, z=0.09m   (前 28.5cm, 高 9cm)
旋转: yaw=0°, pitch=0°, roll=0°   (水平 0°，与仿真一致)
```

> ⚠️ **重要**：Go2W 机身矮（相机高度仅 ~0.47m），必须保持 **0° 水平仰角**。若沿用 B2W 的 20° 俯角，地面在 1.2m 处就会占满画面，导致策略误判为前方有墙。

### 3.3 网络拓扑

```
┌──────────────┐         ┌───────────────────┐
│   上位机      │  网线    │   Go2W 内部网络    │
│ 192.168.123.x│────────▶│ 192.168.123.161  │
│ (eth0)       │         │ (Go2W 运动控制器)  │
└──────────────┘         └───────────────────┘
```

- Go2W 出厂默认网段：`192.168.123.x/24`
- 上位机 `eth0` 设置静态 IP：`192.168.123.x`（x ≠ 161）
- 验证连通性：`ping 192.168.123.161`

---

## 4. 软件环境部署步骤

### 阶段 0：仿真验证确认（已完成 ✅）

```bash
# 确认仿真中策略工作正常
cd /home/yichangfeng/sru-navigation-learning/IsaacLab
conda activate sru-learning
./isaaclab.sh -p source/isaaclab_nav_task/scripts/play.py \
    --task Isaac-Nav-PPO-Go2W-Play-v0 \
    --num_envs 1 \
    --checkpoint /home/yichangfeng/sru-robot-deployment/rl_nav_controller/deployment_policies/nav_policy_new.pt
```

**验收标准**：机器狗能自动对正目标 → 直线前进 (vx > 1.0) → 遇障绕行。

---

### 阶段 1：上位机环境搭建（约 30 分钟）

#### 步骤 1.1：安装基础系统

- **OS**：Ubuntu 22.04 LTS
- **ROS 2**：Jazzy（或 Humble，需与 `unitree_sdk2py` 兼容版本匹配）
- **Python**：3.10+（推荐 3.12）

```bash
# 安装 ROS 2 Jazzy (如尚未安装)
sudo apt update && sudo apt install ros-jazzy-desktop -y

# 安装手柄驱动
sudo apt install ros-jazzy-joy -y
```

#### 步骤 1.2：创建 Python 虚拟环境

```bash
cd ~/sru-robot-deployment
chmod +x setup_ros2_onnx.sh
./setup_ros2_onnx.sh
# → 自动创建 ~/ros2_onnx venv，安装 onnxruntime + numpy/scipy/opencv/pyyaml
```

#### 步骤 1.3：安装宇树 SDK

```bash
source ~/ros2_onnx/bin/activate
pip install unitree_sdk2py==0.4.3  # 或最新版
# 验证
python -c "from unitree_sdk2py.go2.sport.sport_client import SportClient; print('OK')"
```

#### 步骤 1.4：编译 ROS 2 导航工作空间

```bash
mkdir -p ~/sru_ws/src
cp -r ~/sru-robot-deployment/rl_nav_controller ~/sru_ws/src/

cd ~/sru_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select rl_nav_controller --cmake-args -DCMAKE_BUILD_TYPE=Release
# ⚠️ 不要加 --symlink-install（与 venv 不兼容）
```

---

### 阶段 2：代码修改与适配（约 1~2 小时）

#### 步骤 2.1：修改 TF 外参配置

**文件**：`~/sru_ws/src/rl_nav_controller/launch/rl_nav_controller.launch.py`

将 `static_tf_node` 从 B2W 参数改为 Go2W 实机参数：

```python
# 原 B2W 参数 (需替换):
# "0.387", "0.0", "0.28", "0", "0.349", "0", "base_link", "zed_camera_link"

# Go2W 实机参数:
static_tf_node = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    output="screen",
    arguments=[
        "0.285", "0.0", "0.09",   # x, y, z 相对 base (前28.5cm, 高9cm)
        "0", "0", "0",            # yaw, pitch, roll — 必须全部为 0°(水平)
        "base_link", "zed_camera_link"
    ],
)
```

> ⚠️ **关键**：pitch 必须为 0。若 Go2W 实际安装有微小偏差（±2°以内），可微调此值，但绝不可回到 B2W 的 0.349 rad (20°)。

#### 步骤 2.2：创建速度桥接节点 `unitree_nav_bridge.py`

**文件**：`~/sru_ws/src/rl_nav_controller/scripts/unitree_nav_bridge.py`（新建）

```python
#!/usr/bin/env python3
"""Unitree Go2W Sport Mode bridge: subscribes to nav_vel and forwards to robot."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from unitree_sdk2py.go2.sport.sport_client import SportClient


class UnitreeGo2WBridge(Node):
    def __init__(self):
        super().__init__('unitree_go2w_bridge')

        # 初始化宇树 SDK (通过 eth0 连接 Go2W)
        self.sport_client = SportClient()
        self.sport_client.SetTimeout(10.0)
        self.sport_client.Init()
        self.get_logger().info("SportClient initialized.")

        # 启用运动模式
        self.sport_client.SwitchMode(3)  # 3 = Sport Mode (站立行走)
        self.get_logger().info("Switched to Sport Mode.")

        # 订阅导航节点输出的速度指令
        self.sub = self.create_subscription(
            Twist,
            '/path_manager/path_manager_ros/nav_vel',
            self.cmd_callback,
            10
        )
        self.get_logger().info("Subscribed to /path_manager/path_manager_ros/nav_vel")

    def cmd_callback(self, msg: Twist):
        """将导航策略输出的 Twist 转发给 Go2W Sport 模式。"""
        vx = max(-1.5, min(1.5, msg.linear.x))      # 限幅保护
        vy = max(-0.8, min(0.8, msg.linear.y))
        wz = max(-2.0, min(2.0, msg.angular.z))
        self.sport_client.Move(vx, vy, wz)


def main():
    rclpy.init()
    node = UnitreeGo2WBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 安全停机
        node.sport_client.Move(0.0, 0.0, 0.0)
        node.get_logger().info("Robot stopped.")
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

#### 步骤 2.3：修改 `setup.py` 注册入口点（可选）

**文件**：`~/sru_ws/src/rl_nav_controller/setup.py`

在 `entry_points` 中添加桥接节点：
```python
'console_scripts': [
    'rl_nav_controller = rl_nav_controller.rl_nav_controller:main',
    'unitree_go2w_bridge = rl_nav_controller.unitree_nav_bridge:main',  # 新增
]
```

重新编译：
```bash
cd ~/sru_ws && colcon build --packages-select rl_nav_controller
```

#### 步骤 2.4：确认模型文件就位

确保以下文件在正确位置（相对 `rl_nav_controller` 包根目录）：

```
rl_nav_controller/
├── deployment_policies/
│   ├── vae_encoder.onnx          # ~21 MB
│   └── nav_policy.onnx           # ~7 MB
└── launch/
    └── rl_nav_controller.launch.py  (已修改 TF 外参)
```

验证：
```bash
ls -la ~/sru_ws/src/rl_nav_controller/deployment_policies/*.onnx
# 应看到两个 .onnx 文件，大小分别约 21MB 和 7MB
```

---

### 阶段 3：传感器驱动启动（约 30 分钟）

#### 步骤 3.1：ZED-X 深度相机

```bash
# 安装 ZED SDK & ROS 2 wrapper (如尚未安装)
sudo apt install ros-jazzy-zed-ros2-wrapper -y

# 启动相机节点 (发布 /zed/zed_node/depth/depth_registered)
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedx
```

**验证**：
```bash
ros2 topic hz /zed/zed_node/depth/depth_registered
# 应显示 ~15-30 Hz
```

#### 步骤 3.2：激光惯性里程计 (DLIO/Fast-LIO)

Go2W 自带 LiDAR + IMU，使用宇树官方或开源 DLIO 驱动：

```bash
# 方式 A: 宇树官方 SDK 内置里程计 (推荐，最简单)
# Go2W Sport Mode 自动发布 /sportmodestate 中包含位姿信息
# 需编写一个简单节点将 sportmodestate 转换为 nav_msgs/Odometry

# 方式 B: 独立运行 DLIO/Fast-LIO
ros2 launch dlio dlio.launch.py
```

**验证**：
```bash
ros2 topic hz /dlio/odom_node/odom
# 应显示 ~50 Hz
```

> 💡 **简化方案**：若使用宇树 Sport Mode，运动控制器自身会发布 `/sportmodestate` 话题包含机体位姿与速度。可编写一个轻量转换节点将其映射为 `nav_msgs/Odometry` 格式发布到 `/dlio/odom_node/odom`，省去独立 SLAM 的复杂性。

#### 步骤 3.3：手柄驱动

```bash
# 连接 PS5/Xbox 手柄后自动检测
ros2 run joy joy_node
# 验证: ros2 topic hz /joy
```

---

### 阶段 4：实机联调（约 1~2 小时）

#### 步骤 4.1：安全准备

- [ ] Go2W 放在**平坦开阔场地**，周围 3m 内无障碍物/人员
- [ ] 确认手柄已连接且功能正常
- [ ] 确保 ZED-X 相机视野无遮挡、镜头清洁
- [ ] **手动遥控测试**：先用 `unitree_go2w_bridge` + 手动 Twist 发布，验证机器狗能正确响应前后左右转向

```bash
# 手动测试 (在另一个终端)
ros2 topic pub /path_manager/path_manager_ros/nav_vel geometry_msgs/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --rate 10
# 机器狗应缓慢前移
```

#### 步骤 4.2：启动全系统（三个终端）

**终端 1 — 传感器与里程计**：
```bash
source /opt/ros/jazzy/setup.bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedx
# + DLIO/里程计节点 (见步骤 3.2)
```

**终端 2 — 速度桥接节点**：
```bash
source ~/ros2_onnx/bin/activate
source /opt/ros/jazzy/setup.bash
source ~/sru_ws/install/setup.bash
python3 ~/sru_ws/src/rl_nav_controller/scripts/unitree_nav_bridge.py
# 输出: "SportClient initialized." + "Switched to Sport Mode."
```

**终端 3 — 高层导航策略节点**：
```bash
source ~/ros2_onnx/bin/activate
source /opt/ros/jazzy/setup.bash
source ~/sru_ws/install/setup.bash
ros2 launch rl_nav_controller rl_nav_controller.launch.py sim:=false
# 输出: "Using device: CPU (ONNX Runtime)" + "RL Navigation Controller started"
```

#### 步骤 4.3：发布目标点并观察

```bash
# 在 3m 前方设置目标
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{
  header: {frame_id: 'odom'},
  pose: {position: {x: 3.0, y: 0.0, z: 0.0}}
}" --once
```

**预期行为**：
1. 机器狗自动旋转对正目标方向（~2-3s）
2. 加速至 ~1.0 m/s 直线前进
3. 接近目标 0.75m 内停止
4. 手柄松开即立即刹车（死人开关保护）

#### 步骤 4.4：避障测试

在路径上放置障碍物（纸箱、锥桶），观察机器狗是否能：
- 减速 → 横向绕行 → 重新对正目标

---

## 5. 关键参数调优表

| 参数 | 文件/位置 | 默认值 | Go2W 建议值 | 说明 |
|:---|:---|:---|:---|:---|
| `POLICY_SCALE` | `constants.py` | `[1.5, 1.0, 1.0]` | `[1.2, 0.8, 1.0]` | Go2W 速度上限低于 B2W，降低 vx/vy 防止打滑 |
| `LOW_PASS_FILTER_COEF` | `constants.py` | `[0.9, 0.5, 0.5]` | `[0.7, 0.4, 0.4]` | 增加平滑度，减少 Sport Mode 响应抖动 |
| `DEFAULT_MIN_DEPTH` | `constants.py` | `0.25 m` | `0.30 m` | Go2W 矮机身，近处噪声更大 |
| `ARRIVE_GOAL_THRESHOLD` | `constants.py` | `0.75 m` | `0.75 m` | 保持不变 |
| TF pitch | launch 文件 | `0.0 rad` | `0.0 rad` | **必须为 0**（水平） |
| TF xyz | launch 文件 | `(0.285, 0, 0.09)` | 实测校准 | 用棋盘格标定精确值 |

---

## 6. 安全机制

### 6.1 多层保护

| 层级 | 机制 | 说明 |
|:---|:---|:---|
| L1 | 手柄死人开关 | 松开手柄按键 → `rl_nav_controller` 立即停止发布速度 → 机器狗原地停稳 |
| L2 | 速度限幅 | `unitree_nav_bridge.py` 中硬编码 vx≤1.5, vy≤0.8, ωz≤2.0 |
| L3 | 宇树固件保护 | Sport Mode 内置：急停、跌倒自恢复、碰撞检测后减速 |
| L4 | 手动遥控接管 | 随时可切换到 `b2w_joystick_control` 纯手动模式（同一话题） |

### 6.2 紧急停止

- **手柄**：松开使能键 → 立即停止
- **命令行**：`ros2 topic pub /path_manager/path_manager_ros/nav_vel geometry_msgs/Twist --once` (零值)
- **物理**：Go2W 背部急停按钮（硬件级断电）

---

## 7. 调试与故障排除

| 症状 | 可能原因 | 解决方案 |
|:---|:---|:---|
| 机器狗不动 | 手柄未使能 / 桥接节点未启动 | 检查 `/rsl_joy` 话题有数据；确认终端 2 无报错 |
| 持续侧滑绕圈 | TF pitch ≠ 0 (地面被识别为墙) | 回到步骤 2.1 确认 pitch=0；检查相机安装角度 |
| 速度过猛/打滑 | POLICY_SCALE 过大 | 降低 `constants.py` 中 vx/vy scale |
| 深度图全黑/异常 | ZED-X 未正确启动 / TF 缺失 | `ros2 topic echo /zed/zed_node/depth/depth_registered --no-arr -n1`；检查 TF tree |
| ONNX 推理慢 (>200ms) | CPU 性能不足 / 模型路径错误 | 确认使用 venv 中的 onnxruntime；考虑 GPU 版 |
| 无法连接 Go2W | 网络不通 / IP 冲突 | `ping 192.168.123.161`；检查 eth0 静态 IP 配置 |

---

## 8. 后续扩展路线（方案 B）

当方案 A 验证通过后，若课题要求部署自研 RL 步态：

1. **移植 `rl_sar`** 到 Go2W 实机（C++ 运行时 + CAN 总线直连）
2. **使用仿真中已适配的策略**：`policy_go2w_adapted.pt`（含关节顺序映射、观测缩放、动作解耦缩放）
3. **替换桥接层**：将 `unitree_nav_bridge.py` 的 SportClient 调用替换为 `rl_sar` 低层控制接口
4. **Sim-to-Real Gap 处理**：
   - 关节摩擦/延迟补偿
   - IMU 噪声与仿真差异
   - 地面摩擦系数校准

> 方案 B 预计额外工作量：**2~3 周**（含安全测试迭代）

---

## 9. 里程碑与时间线

| 阶段 | 内容 | 预计耗时 | 交付物 |
|:---|:---|:---|:---|
| M0 ✅ | Isaac Lab 仿真验证 Go2W 导航 | (已完成) | `GO2W_SIM_HANDOFF.md` |
| M1 | 上位机环境搭建 + SDK 安装 | 30 min | 可运行的 ROS 2 + ONNX Runtime 环境 |
| M2 | 代码适配（TF + 桥接节点） | 1~2 h | 修改后的 launch 文件 + `unitree_nav_bridge.py` |
| M3 | 传感器驱动联调 | 30 min | 深度图 + 里程计正常发布 |
| M4 | 实机首次行走测试 | 1~2 h | 机器狗响应 Twist 指令正确移动 |
| M5 | 自主导航全链路测试 | 1~2 h | 发目标 → 机器狗自主避障到达 |
| M6 | 参数调优 + 鲁棒性验证 | 半天 | 最终参数配置 + 测试报告 |

**总计（方案 A）**：从环境搭建到实机跑通约 **1~1.5 个工作日**。

---

## 10. 文件清单与修改摘要

### 需要修改的文件
| 文件 | 修改内容 |
|:---|:---|
| `rl_nav_controller/launch/rl_nav_controller.launch.py` | TF 外参：B2W → Go2W (pitch 0.349→0, xyz 更新) |
| `rl_nav_controller/rl_nav_controller/constants.py` | POLICY_SCALE / LOW_PASS_FILTER_COEF 微调 |

### 需要新建的文件
| 文件 | 用途 |
|:---|:---|
| `rl_nav_controller/scripts/unitree_nav_bridge.py` | Go2W Sport Mode 速度桥接节点 |
| (可选) `rl_nav_controller/scripts/sport_odom_bridge.py` | 将 `/sportmodestate` 转为 `nav_msgs/Odometry` |

### 无需修改的文件（直接复用）
| 文件 | 说明 |
|:---|:---|
| `deployment_policies/nav_policy.onnx` | 高层导航策略（机器人无关） |
| `deployment_policies/vae_encoder.onnx` | VAE 深度编码器（机器人无关） |
| `rl_nav_controller.py` 核心逻辑 | 输入输出接口与 B2W 完全一致 |
| `setup_ros2_onnx.sh` | 环境安装脚本直接可用 |

---

## 附录 A：实机相机标定的快速方法

若需精确标定 ZED-X 相对 base 的外参：

1. 在 Go2W 头部放置一个已知尺寸的棋盘格（如 9×6, 方格 30mm）
2. 用 `ros2 run camera_calibration_parsers` + ZED SDK 的 `grabCameraPose()` 获取相机内参
3. 手动测量安装距离作为初始值，通过 RViz 中叠加深度图与激光点云进行微调
4. 最终确认：在 RViz 中 `zed_camera_link` frame 的朝向应与 Go2W 前进方向一致（x 轴朝前）

## 附录 B：宇树 Sport Mode API 速查

```python
from unitree_sdk2py.go2.sport.sport_client import SportClient

client = SportClient()
client.Init()

# 模式切换: 0=躺下, 1=坐下, 2=趴下, 3=站立(Sport), 4=运动(奔跑)
client.SwitchMode(3)

# 速度控制 (vx: 前后 m/s, vy: 横移 m/s, vyaw: 转向 rad/s)
client.Move(vx=0.5, vy=0.0, vyaw=0.0)

# 停止
client.Stop()

# 获取状态 (包含位姿、速度、IMU 等)
state = client.GetSportState()
```

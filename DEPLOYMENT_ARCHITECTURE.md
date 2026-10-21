# SRU Robot Deployment 策略模型与系统架构技术参考文档

本文档全面梳理了 `~/sru-robot-deployment`（Unitree B2W 轮足四足机器人部署系统）中所使用的策略模型、关键代码文件、分层控制架构以及节点间的通信拓扑与数据流向。

---

## 1. 系统分层控制架构 (System Architecture)

整个系统采用**分层强化学习（Hierarchical RL）**与经典机器人控制相结合的架构：
- **高层导航策略（High-Level Navigation Policy）**：运行频率 **5 Hz**，负责“看路与规划”。以深度相机图像和目标点为输入，输出期望的机体线速度和角速度 $v_x, v_y, \omega_z$。
- **底层运动控制策略（Low-Level Locomotion Policy）**：运行频率 **50 Hz 推理 / 200 Hz 指令下发**，负责“平衡与行走”。以机体状态、关节状态和高层速度指令为输入，输出具体的 16 个关节伺服控制量（12 腿关节位置 + 4 轮电机速度）。

```mermaid
flowchart TD
    subgraph Perception_Goal [感知与目标输入]
        CAM["ZED-X 深度相机 / Gazebo 深度传感<br/>Topic: /zed/zed_node/depth/depth_registered"]
        ODOM["DLIO 激光里程计 / 仿真里程计<br/>Topic: /dlio/odom_node/odom"]
        GOAL["目标位置输入 (RViz / 航点管理)<br/>Topic: /goal_pose"]
        JOY["手柄死人开关 & 遥控<br/>Topic: /rsl_joy"]
    end

    subgraph High_Level_Node [高层导航节点: rl_nav_controller (5 Hz)]
        direction TB
        PRE["VAE 深度图像编码器<br/>vae_encoder.onnx<br/>(40×64 深度图 → 2560维 特征)"]
        NORM["状态拼接与归一化<br/>16维状态 + 2560维深度 = 2576维向量"]
        NAV_NET["SRU 循环导航网络<br/>nav_policy.onnx<br/>(Cross-Attention + LSTM_SRU)"]
        LPF["低通平滑滤波 (Low-Pass Filter)"]
        
        CAM --> PRE
        PRE --> NORM
        ODOM --> NORM
        GOAL --> NORM
        NORM --> NAV_NET
        NAV_NET --> LPF
    end

    subgraph Command_Bridge [速度指令通道 (Twist)]
        CMD_VEL["Topic: /path_manager/path_manager_ros/nav_vel<br/>(geometry_msgs/Twist: vx, vy, omega)"]
        LPF -->|自主导航模式| CMD_VEL
        JOY_TELEOP["遥控节点: b2w_joystick_control"] -.->|纯手动遥控模式| CMD_VEL
    end

    subgraph Low_Level_Node [底层运动控制器: b2w_controllers (50 Hz / 200 Hz)]
        direction TB
        LOCO_NET["步态运动策略<br/>policy_force_new.onnx<br/>(ONNX Runtime C++ 加载)"]
        ACT_CONV["动作反归一化与重排序<br/>Position Scale: 0.5, Velocity Scale: 5.0"]
        
        CMD_VEL --> LOCO_NET
        ODOM --> LOCO_NET
        J_STATE["关节状态反馈<br/>Topic: /joint_states"] --> LOCO_NET
        LOCO_NET --> ACT_CONV
    end

    subgraph Robot_Actuation [机器人执行机构 (Gazebo / 真实机器狗)]
        LEG_CMD["12 个腿关节位置指令<br/>/*_hip, thigh, calf_joint_position_cmd"]
        WHEEL_CMD["4 个轮电机角速度指令<br/>/*_foot_joint_velocity_cmd"]
        ACT_CONV --> LEG_CMD
        ACT_CONV --> WHEEL_CMD
    end
```

---

## 2. 核心策略模型详解 (Policy Models)

该系统协同使用了 **3 个核心神经网络模型**：

| 模型名称 | 文件位置 | 模型格式 | 运行框架 | 频率 | 输入规格 | 输出规格 | 功能与说明 |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **VAE 深度编码器** | `rl_nav_controller/deployment_policies/vae_encoder.onnx` <br/> *(对应 `vae_pretrain_new_jit.pt`)* | ONNX / TorchScript | ONNX Runtime (Python) | 5 Hz | 单通道深度图 `[1, 1, 40, 64]` | 压缩潜在表征 `[1, 2560]` (`64×5×8`) | 将 ZED-X 深度图进行空间特征压缩，去除噪声并提取几何障碍物特征 |
| **SRU 高层导航策略** | `rl_nav_controller/deployment_policies/nav_policy.onnx` <br/> *(对应 `nav_policy_new.pt`)* | ONNX / TorchScript | ONNX Runtime (Python) | 5 Hz | - `obs`: `[1, 2576]`<br/>- `h_in`: `[1, 1, 512]`<br/>- `c_in`: `[1, 1, 512]` | - `actions`: `[1, 3]`<br/>- `h_out`: `[1, 1, 512]`<br/>- `c_out`: `[1, 1, 512]` | 基于注意力机制与循环记忆网络（SRU），规划机器人避障并朝向目标点的机体速度 $[v_x, v_y, \omega_z]$ |
| **底层轮足运动策略** | `b2w_sim/b2w_controllers/src/policy/policy_force_new.onnx` | ONNX | ONNX Runtime (C++) | 50 Hz | 机体姿态、速度、历史动作、关节反馈及高层下发的目标 Twist | 16 个关节动作残差 `[1, 16]` | 控制 4 条腿共 12 个自由度的位置平衡以及 4 个足端驱动轮的连续转速 |

### 2.1 高层导航策略输入组成（2576 维）
高层策略的观测输入向量由 16 维机体状态与 2560 维视觉潜在表征拼接构成：
1. **机体线速度 (3 维)**：$[v_x, v_y, v_z]$（机体局部坐标系，来自里程计）
2. **机体角速度 (3 维)**：$[\omega_x, \omega_y, \omega_z]$（机体局部坐标系）
3. **投影重力向量 (3 维)**：$[g_x, g_y, g_z]$（世界重力在机体坐标系下的投影）
4. **上一时刻控制动作 (3 维)**：$[v_{x, prev}, v_{y, prev}, \omega_{z, prev}]$
5. **目标相对极坐标与距离 (4 维)**：机体指向目标的方向单位向量 $[d_x, d_y, d_z]$ 以及对数距离 $\ln(\text{dist} + 1.0)$
6. **VAE 视觉潜在特征 (2560 维)**：深度图经 VAE 提取的稠密障碍物表征

---

## 3. 涉及的核心代码与文件分布 (Key Files)

整个部署工程位于 `~/sru-robot-deployment`，分为以下核心功能包：

### 3.1 高层导航包：`rl_nav_controller`
- [`rl_nav_controller/rl_nav_controller.py`](file:///home/yichangfeng/sru-robot-deployment/rl_nav_controller/rl_nav_controller/rl_nav_controller.py)：
  - **`LearningModel` 类**：负责利用 `onnxruntime` 加载 `vae_encoder.onnx` 和 `nav_policy.onnx`，负责深度图尺寸缩放（双线性插值到 $40 \times 64$）、隐状态维护（$h, c$）和前向推理。
  - **`NavigationPolicyNode` 类**：ROS 2 核心节点。订阅深度图与里程计，以 5Hz 周期运行 `predict()`，经过一阶低通滤波后将速度发布到 `/path_manager/path_manager_ros/nav_vel`。
- [`rl_nav_controller/constants.py`](file:///home/yichangfeng/sru-robot-deployment/rl_nav_controller/rl_nav_controller/constants.py)：定义了控制周期（5.0 Hz）、速度动作缩放比例（`POLICY_SCALE = [1.5, 1.0, 1.0]`）、低通滤波系数（`[0.9, 0.5, 0.5]`）及手柄键位映射。
- [`rl_nav_controller/waypoint_manager.py`](file:///home/yichangfeng/sru-robot-deployment/rl_nav_controller/rl_nav_controller/waypoint_manager.py)：管理多航点导航、路径记录与循环巡航。
- [`rl_nav_controller/visualization.py`](file:///home/yichangfeng/sru-robot-deployment/rl_nav_controller/rl_nav_controller/visualization.py)：在 RViz 中发布当前速度箭头、目标方向向量及航点 Marker。
- [`launch/rl_nav_controller.launch.py`](file:///home/yichangfeng/sru-robot-deployment/rl_nav_controller/launch/rl_nav_controller.launch.py)：启动手柄驱动节点、静态 TF 变换（`base_link` $\to$ `zed_camera_link`）及导航节点。

### 3.2 底层运动控制包：`b2w_sim/b2w_controllers`
- [`b2w_controllers.cpp`](file:///home/yichangfeng/sru-robot-deployment/b2w_sim/b2w_controllers/src/b2w_controllers.cpp)：基于 C++ 与内嵌 ONNX Runtime 编写的高性能实时控制节点，维护 50Hz 策略循环和 200Hz 控制指令发布。
- [`config/b2w_controllers.yaml`](file:///home/yichangfeng/sru-robot-deployment/b2w_sim/b2w_controllers/config/b2w_controllers.yaml)：
  - 配置加载的模型相对路径：`policy/policy_force_new.onnx`
  - 配置输入话题（`/dlio/odom_node/odom`, `/path_manager/path_manager_ros/nav_vel`, `/joint_states`）
  - 配置关节默认角度（站姿：thigh=0.4, calf=-1.3）、关节输出缩放（腿关节 0.5，轮关节 5.0）。

### 3.3 仿真与物理环境：`b2w_sim/b2w_gazebo_ros2` 与 `b2w_description_ros2`
- [`b2w_gazebo.launch.py`](file:///home/yichangfeng/sru-robot-deployment/b2w_sim/b2w_gazebo_ros2/launch/b2w_gazebo.launch.py)：一键启动 Gazebo Harmonic、加载机器人 URDF、启动 46 个 ROS-Gazebo 桥接通道，并启动 `b2w_controllers`。
- `config/b2w_gz_bridge.yaml`：定义 Gazebo 物理引擎与 ROS 2 之间的双向通信（传感器数据如 IMU、雷达、相机输入，以及电机控制指令输出）。

---

## 4. ROS 2 通信与话题拓扑清单 (Communication & Topics)

| 话题名称 (Topic Name) | 消息类型 (Message Type) | 发布者 (Publisher) | 订阅者 (Subscriber) | 默认频率 | 作用与说明 |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `/goal_pose` | `geometry_msgs/PoseStamped` | RViz (Publish Point) / 航点管理器 | `rl_nav_controller` | 触发式 / 1 Hz | 机器人全局期望导航目标点位置 |
| `/dlio/odom_node/odom` | `nav_msgs/Odometry` | 里程计节点 / Gazebo 仿真里程计 | `rl_nav_controller` & `b2w_controllers` | 50 Hz | 机器人基座世界位姿、线速度与角速度反馈 |
| `/zed/zed_node/depth/depth_registered` | `sensor_msgs/Image` (32FC1) | ZED-X 驱动 / Gazebo 深度相机插件 | `rl_nav_controller` | 15~30 Hz | 前向深度相机采集的原始距离图像 |
| `/rsl_joy` | `sensor_msgs/Joy` | `joy_node` (手柄驱动) | `rl_nav_controller` | 50 Hz | 安全死人开关保护（按住使能）与模式切换 |
| `/path_manager/path_manager_ros/nav_vel` | `geometry_msgs/Twist` | `rl_nav_controller` *(或 `b2w_joystick_control`)* | `b2w_controllers` | 5 Hz | **上下层桥梁**：高层策略规划出的目标线速度与转向角速度 |
| `/joint_states` | `sensor_msgs/JointState` | Gazebo Bridge / 硬件下位机驱动 | `b2w_controllers` | 50~200 Hz | 16 个关节的当前角度、角速度与反馈力矩 |
| `/*_hip, thigh, calf_joint_position_cmd` | `std_msgs/Float64` | `b2w_controllers` | Gazebo / 伺服电机驱动 | 200 Hz | 12 个腿部活动关节的位置期望控制指令 |
| `/*_foot_joint_velocity_cmd` | `std_msgs/Float64` | `b2w_controllers` | Gazebo / 轮毂电机驱动 | 200 Hz | 4 个轮足电机的转速期望控制指令 |
| `vis/twist_cmd_marker` 等 | `visualization_msgs/Marker` | `rl_nav_controller` | RViz2 | 5 Hz | 调试可视化：速度箭头、目标方向与航点标记 |

---

## 5. 两种仿真运行方式对比 (Execution Guide)

| 对比维度 | 方式 A：当前 IsaacLab 仿真中直接加载 | 方式 B：ROS 2 + Gazebo 完整部署仿真 |
| :--- | :--- | :--- |
| **运行技术栈** | Isaac Lab (Omniverse) + Python | ROS 2 Jazzy + Gazebo Harmonic + C++ Controller |
| **调用的策略文件** | `rl_nav_controller/deployment_policies/nav_policy_new.pt` | `nav_policy.onnx` + `vae_encoder.onnx` + `policy_force_new.onnx` |
| **系统依赖** | 仅需现有 Python / IsaacLab 环境，无需额外依赖 | 需要配置完整的 ROS 2、Gazebo、`ros_gz_bridge`、手柄驱动 |
| **手柄要求** | 否（自主寻优环境内置目标） | 是（需连接手柄作为安全死人开关） |
| **适用场景** | 快速验证作者训练出的高层避障导航行为与强化学习表现 | 检验包含 ROS 通信、坐标变换、底层 C++ 控制器在内的软硬件全链路表现 |

### 5.1 运行方式 A（在 IsaacLab 中直接体验）
```bash
cd /home/yichangfeng/sru-navigation-learning/IsaacLab

./isaaclab.sh -p source/isaaclab_nav_task/scripts/play.py \
    --task Isaac-Nav-PPO-B2W-Play-v0 \
    --num_envs 1 \
    --checkpoint /home/yichangfeng/sru-robot-deployment/rl_nav_controller/deployment_policies/nav_policy_new.pt
```

### 5.2 运行方式 B（在 ROS 2 + Gazebo 中体验）
```bash
# 终端 1：启动 Gazebo 仿真、物理桥接与底层控制器
ros2 launch b2w_gazebo_ros2 b2w_gazebo.launch.py enable_rviz:=true

# 终端 2：启动高层视觉导航节点（加载 VAE 与 ONNX 策略）
ros2 launch rl_nav_controller rl_nav_controller.launch.py sim:=true

# 终端 3：在 RViz 中通过 "Publish Point" 或命令行下发目标
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{
  header: {frame_id: 'odom'},
  pose: {position: {x: 5.0, y: 0.0, z: 0.0}}
}"
```

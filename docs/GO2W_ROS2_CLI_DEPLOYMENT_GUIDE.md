# Unitree Go2W 实机自主避障导航从 0 到 1 实操全景手册

> **版本**：v2.0 (2026-10-26 更新)  
> **适用机型**：Unitree Go2W 轮足四足机器人  
> **操作方式**：通过 **NoMachine** 远程桌面连接背部高层主脑（Agile-Robot-Guest WiFi：`10.3.42.161`，备选无线 IP：`192.168.31.186`，机载网：`192.168.123.111`，用户: `agile`）  
> **核心改进**：
> 1. **无需单独启动 RViz**：启动 Super LIO 时指定 `rviz:=true` 即可自动拉起预调好的官方 RViz 界面；
> 2. **一键启动全栈脚本**：提供 `start_go2w_system.sh`，一条命令拉起底盘桥、雷达 SLAM、RViz、ZED 深度感知与强化学习避障策略；
> 3. **全闭环导航指引**：详述第 6 终端解锁导航、RViz 鼠标交互发点、命令行发点及动态避障全流程。

---

## 目录
1. [系统架构与前置环境](#1-系统架构与前置环境)
2. [安全规范与前置检查（必须遵守）](#2-安全规范与前置检查必须遵守)
3. [方式一：一键脚本启动全栈（推荐日常使用）](#3-方式一一键脚本启动全栈推荐日常使用)
4. [方式二：分步终端启动全流程（排错与调试备查）](#4-方式二分步终端启动全流程排错与调试备查)
   - [【终端 1】底盘通信桥 (unitree_bridge)](#终端-1底盘通信桥-unitree_bridge)
   - [【终端 2】雷达驱动、Super LIO 重定位与自动 RViz (rviz:=true)](#终端-2雷达驱动super-lio-重定位与自动-rviz-rviztrue)
   - [【终端 3】ZED 2i 双目深度相机感知](#终端-3zed-2i-双目深度相机感知)
   - [【终端 4】发布相机水平静态 TF](#终端-4发布相机水平静态-tf)
   - [【终端 5】启动 SRU 强化学习端到端避障策略](#终端-5启动-sru-强化学习端到端避障策略)
5. [【终端 6】激活底盘导航锁与自主避障发点](#5-终端-6激活底盘导航锁与自主避障发点)
   - [第一步：解锁底盘导航运动锁（必须执行）](#第一步解锁底盘导航运动锁必须执行)
   - [第二步：健康自检与状态确认](#第二步健康自检与状态确认)
   - [第三步：如何下发导航目标点 (Goal Pose)](#第三步如何下发导航目标点-goal-pose)
     - [方法 A：RViz 界面中鼠标点击发点（最直观）](#方法-arviz-界面中鼠标点击发点最直观)
     - [方法 B：CLI 命令行直接发布绝对坐标](#方法-bcli-命令行直接发布绝对坐标)
   - [第四步：自主避障运行机制与机身应对](#第四步自主避障运行机制与机身应对)
6. [紧急接管与安全停止机制](#6-紧急接管与安全停止机制)
7. [关键常见问题与排坑速查 (FAQ)](#7-关键常见问题与排坑速查-faq)

---

## 1. 系统架构与前置环境

### 1.1 硬件与网络节点
全系统运行于机载局域网（Agile-Robot-Guest WiFi 网段：`10.3.42.0/24`（当前最常用），备选无线网段：`192.168.31.0/24`，机载内网网段：`192.168.123.0/24`）：
- **`10.3.42.161` (Agile-Robot-Guest WiFi，机载内网 `192.168.123.111`，user: agile / 密码: `1` 或 `agile`)**：背部外挂 NVIDIA Jetson Orin DevKit (Ubuntu 22.04 / ROS 2 Humble)。直连 Livox MID-360 雷达与 ZED 2i 双目相机。**全套算法均在单机上完整闭环运行**。备选 IP：`192.168.31.186`（旧无线网段）。
- **`192.168.123.18` (unitree / 密码: `123`)**：腹部 Jetson Orin NX (Ubuntu 20.04 / Foxy)。
- **`192.168.123.161`**：底层运动控制 MCU。

### 1.2 环境变量配置（已配置确认）
在 `111` 的 `~/.bashrc` 末尾已注入以下 4 行环境加载，新打开的终端均自动生效：
```bash
source /opt/ros/humble/setup.bash
source ~/unitree_ros2/setup.sh
source ~/bridge_ws/install/setup.bash 2>/dev/null
source ~/sru_ws/install/setup.bash 2>/dev/null
```

---

## 2. 安全规范与前置检查（必须遵守）

> [!CAUTION]
> **安全防翻滚铁则**：
> 1. **首次测试电机关节运动前，必须使用结实的板凳或专用支架撑起机器狗腹部，使四肢和四个车轮完全悬空离地！**
> 2. 手边必须时刻准备好宇树无线遥控手柄，随时准备切换回手动控制。
> 3. 遥控手柄的 **`B` 键** 是底盘控制软开关，按一下可在使能与失能之间切换。

---

## 3. 方式一：一键脚本启动全栈（推荐日常使用）

为了避免每次登录需要开 5 个终端，工程中已提供全自动一键启动脚本 `start_go2w_system.sh`。

### 3.1 部署脚本到 111
若脚本尚未在 `111` 主目录下，直接将本项目脚本拷贝至 `111`：
```bash
scp scripts/start_go2w_system.sh scripts/stop_go2w_system.sh agile@192.168.123.111:~/
```

### 3.2 在 NoMachine 桌面一键启动
在 NoMachine 的桌面打开**第 1 个终端**，执行：
```bash
./start_go2w_system.sh
```

**脚本自动执行流程**：
1. 自动检查 `unitree_bridge` 是否已开机自启，若未启动则在后台自动唤醒；
2. 自动启动 Livox MID-360 驱动并运行 Super LIO，**自动弹出配置好的官方 RViz 界面**（`rviz:=true`）；
3. 自动拉起 ZED 2i 深度相机驱动并等待相机自检就绪；
4. 自动广播相机到机身的水平静态坐标变换（Static TF）；
5. 在前台启动 `rl_nav_controller` 强化学习端到端导航策略。

**启动成功标志**：
终端看到以下提示即代表整套系统全部准备完毕：
```text
[INFO] [navigation_policy_node]: Navigation policy node is ready.
[INFO] [navigation_policy_node]: RL Navigation launched in REAL-HARDWARE mode.
[INFO] [navigation_policy_node]: First odometry message received, odom is ready.
```
> **退出方法**：在运行该脚本的终端按下 `Ctrl+C`，脚本内部会自动安全终止所有后台节点与进程。

---

## 4. 方式二：分步终端启动全流程（排错与调试备查）

如需逐个模块深入排查或查看各节点的即时输出，可在 NoMachine 中按顺序分别开启以下 5 个终端。

### 【终端 1】底盘通信桥 (unitree_bridge)
先检查底盘桥是否已自启：
```bash
ros2 node list | grep unitree_bridge
```
- 若已输出 `/unitree_bridge`：直接跳过本终端！
- 若未自启：手动启动：
```bash
ros2 run unitree_bridge unitree_bridge
```

---

### 【终端 2】雷达驱动、Super LIO 重定位与自动 RViz (rviz:=true)
```bash
ros2 launch super_lio relocation.py rviz:=true
```
- **关键变动**：这里设置 `rviz:=true`，系统会**自动弹出预置好的 RViz2 窗口**。
- **窗口内容包含**：
  - `/lio/cloud_world`：实时雷达扫描点云；
  - `/lio/robo/odom`：机身实时定位与运动姿态；
  - `/trajectory_path`：实时移动轨迹；
  - `/zed/zed_node/rgb/...`：前向相机画面；
  - `/goal_marker`：导航目标路标点。
> 若首次重定位与地图稍有错位，在弹出的 RViz 顶部点击 **`2D Pose Estimate`**（快捷键 `p`），在地图上狗的当前位置点击并拉出朝向，即可瞬间对齐。

---

### 【终端 3】ZED 2i 双目深度相机感知
```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
```
- 相机为强化学习策略提供前向避障深度图；
- 启动后可在新终端使用 `ros2 topic hz /zed/zed_node/depth/depth_registered` 确认有稳定的 15~30Hz 输出。

---

### 【终端 4】发布相机水平静态 TF
```bash
ros2 run tf2_ros static_transform_publisher 0.257 0.0 0.28 0.0 0.0 0.0 base_link zed_camera_link
```
- 将相机基座刚性绑定到机身基座，保持水平 0 度仰角，防止地面被误判为障碍物。

---

### 【终端 5】启动 SRU 强化学习端到端避障策略
```bash
ros2 run rl_nav_controller rl_nav_controller \
  --ros-args \
  -r /dlio/odom_node/odom:=/lio/robo/odom \
  -r /path_manager/path_manager_ros/nav_vel:=/cmd_vel
```
- 输出 `Navigation policy node is ready` 以及 `First odometry message received, odom is ready` 标志策略进入待命状态。

---

## 5. 【终端 6】激活底盘导航锁与自主避障发点

前 5 个组件启动完成后，在 NoMachine 打开**第 6 个终端**，执行以下导航与避障控制操作：

### 第一步：解锁底盘导航运动锁（必须执行）
`unitree_bridge` 出于安全保护机制，默认将 `start_nav_` 设为 `false`（忽略一切自动控制速度指令）。必须通过发布使能话题解锁运动控制：

```bash
ros2 topic pub /startNavi std_msgs/msg/Bool "{data: true}" --once
```
> **提示**：如果在后续任何时候想暂停机器狗，可发布 `"{data: false}"` 重新上锁。

---

### 第二步：健康自检与状态确认
在终端 6 中快速运行以下检查，确认数据链路全通：
```bash
# 1. 确认雷达里程计正在高频发布 (约 10~50Hz)
ros2 topic hz /lio/robo/odom

# 2. 确认相机深度图正常发布 (约 15~30Hz)
ros2 topic hz /zed/zed_node/depth/depth_registered

# 3. 监控底盘下发速度 (此时速度应全为 0，因为还没发目标)
ros2 topic echo /cmd_vel
```

---

### 第三步：如何下发导航目标点 (Goal Pose)

#### 方法 A：RViz 界面中鼠标点击发点（最直观、推荐）
在终端 2 自动弹出的 RViz2 窗口中：
1. **选择工具**：点击顶部工具栏的 **`2D Goal Pose`** 按钮（或直接按键盘快捷键 **`g`**）；
2. **在地图中发点**：
   - 鼠标移动到点云地图中你想让狗走去的目标走廊或开阔区域；
   - **按住鼠标左键单击，并按住拖拽**拉出一个箭头（箭头的朝向就是机器狗到达目的地后的最终朝向）；
   - **松开鼠标**。
3. **观察响应**：
   - RViz 自动向 `/goal_pose` 发送目标；
   - 策略节点终端打印：`Received new goal pose`；
   - 四足车轮开始转动，机器狗开始原地调整朝向，并全自主向前巡航！

#### 方法 B：CLI 命令行直接发布绝对坐标
如果不通过图形界面，也可以在终端 6 中直接指定目标位置（例如：以世界坐标系向前方 $x=1.5$ 米发点）：
```bash
ros2 topic pub /goal_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'world'}, pose: {position: {x: 1.5, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}" --once
```
- 在**悬空安全测试**时，建议优先使用该命令行发送近距离单次点（如 $x=0.8$ 米），观察四肢与轮毂驱动响应。

---

### 第四步：自主避障运行机制与机身应对

1. **避障感知机制**：
   - 机器狗奔向目标点的途中，ZED 2i 双目相机持续捕捉前方 0.3m ~ 5.0m 视场内的障碍物（人体、椅子、纸箱等）；
   - 深度信息经 VAE 压缩为 2560 维特征实时喂给 LSTM 策略模型；
   - 当遇到障碍物阻挡直线路径时，模型输出包含横向偏移与角速度的偏航矢量，机器狗会自动流畅绕开障碍，绕过后再重新朝目标点前进。
2. **运动颠簸与机身起伏**：
   - 机器狗在奔跑与轮足切换时会产生自然的俯仰颠簸，MID-360 内置的高频 IMU 重力对齐机制（`kf_align_gravity: true`）会自动消除机身姿态变化对坐标系的影响，无需人为干预。

---

## 6. 紧急接管与安全停止机制

在实机行进测试过程中，若发生意外、轨迹跑偏或需要立即接管，可通过以下三种方式实现毫秒级急停：

1. **宇树遥控手柄（第一顺位物理接管）**：
   - 拨动手柄摇杆，底层 MCU 自动优先响应手柄输入；
   - 或快速按一下遥控手柄的 **`B` 键**，底盘桥立刻切断自动运动控制。
2. **终端发布停车锁**：
   ```bash
   ros2 topic pub /startNavi std_msgs/msg/Bool "{data: false}" --once
   ```
3. **全系统一键停止脚本**：
   在 NoMachine 任意新终端执行：
   ```bash
   ./stop_go2w_system.sh
   ```
   该脚本会立刻锁死底盘，并终止所有后台运行的 SLAM、感知与策略节点。

---

## 7. 关键常见问题与排坑速查 (FAQ)

1. **问：为什么启动后发了 `/goal_pose`，机器狗四肢和轮子完全不转？**  
   - **排查 1**：是否遗漏了步骤 5 的第一步？必须发布一次 `/startNavi` 为 `true`。
   - **排查 2**：手柄使能状态。检查遥控手柄是否处于待命使能模式。
   - **排查 3**：查看终端 5 的策略输出，是否显示 `First odometry message received, odom is ready`。若没有，检查 Super LIO 是否就绪。

2. **问：RViz 自动弹出来了，但地图和激光点是倾斜的？**  
   - 点击 RViz 顶部工具栏的 **`2D Pose Estimate`**（快捷键 `p`），在全局地图上机器狗当前真实位置点击并拉出箭头。重定位节点会瞬间执行全局 ICP 配准，将点云完全水平吸附到地图上。

3. **问：RViz 中 Image 插件为什么经常全黑？**  
   - 机器狗趴下时镜头距离地面极近（< 0.2m），处于 ZED 双目相机的物理盲区，视差失效会导致全黑。将机身悬空撑起或站立（离地 > 0.3m），并将话题设为 `/zed/zed_node/rgb/image_rect_color` 或勾选深度图的 `Normalize Range` 即可恢复正常。

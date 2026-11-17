# VLN 相对 base 目标点接入规范

> 适用对象：上层 VLN（Vision-Language-Navigation）模块开发者。
> 目标：上层为 VLN 模型，本模块**只做点的跟随**——VLN 在**现有** `/goal_pose` 话题上发布“相对机器人 base 的位置点”，SRU 负责跟随；VLN 判定到点后停止发点，SRU 完成最后一个点（10 cm 内）后刹停并等待下一个点。
> 不新增任何输入话题、不新增桥接节点、不走 RViz 下点；里程计照常使用（Super-LIO `/lio/robo/odom`，无地图依赖）。
> 平台：Unitree Go2W（Jetson Orin DevKit / Ubuntu 22.04 / ROS 2 Humble）｜深度相机：**Intel RealSense D435i**（本项目不涉及 ZED）
> 机载接入：用户 `agile`；机器人自带 WiFi `GL-MT3000-ef3-5G` → **192.168.123.111**，`agile_guest` WiFi → **10.3.42.161**（SSH / NoMachine 任选）

---

## 1. 一句话接口

| 项目 | 内容 |
|:---|:---|
| 话题 | `/goal_pose`（`geometry_msgs/msg/PoseStamped`，与 RViz 2D Goal Pose 相同） |
| 坐标系 | `header.frame_id: "base_link"`（或 `base` / `base_footprint` / `body` / `imu`） |
| 位置语义 | `pose.position` 为**相对当前机器人 base 的位移**，单位 m |
| 朝向语义 | `pose.orientation` 可选：相对当前车头朝向的偏航增量；全零/norm≈0 表示不约束朝向 |
| 发点节奏 | VLN 在未到点前持续周期发布（建议 2~5 Hz） |
| 到点/停止 | 机器人进入目标 **0.10 m** 半径内即刹停并进入 `IDLE` |
| 恢复 | VLN 再发布任意新点，SRU 自动回到 `RUNNING` 继续跟随 |
| 重复发点 | 不做去重；每条消息都按收到时的机器人位姿重新锁定世界目标，允许 VLN 在同一点附近微调 |

示例：

```bash
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'base_link'}, pose: {position: {x: 1.2, y: 0.0, z: 0.0}, orientation: {w: 0.0}}}" --rate 5
```

> 上例每次消息都表示“收到消息这一刻，车头正前方 1.2 m”。每条点都会按当时位姿重新锁定世界目标；机器人进入目标 0.10 m 内即停。持续重复同一相对点会持续把目标保持在车前 1.2 m（相对语义）。

### 1.1 坐标系与话题速查

| 来源 | 话题 | `frame_id` | 说明 |
|:---|:---|:---|:---|
| RViz 2D Goal Pose | `/goal_pose` | `map` | `local_sru.rviz` / `rviz.rviz` 的 Fixed Frame 为 `map` |
| VLN（本功能） | `/goal_pose` | `base_link` 等 base 帧 | 位置相对当前机器人 base |
| Super-LIO 里程计 | `/lio/robo/odom` | `world`（child `base`） | `start_local_sru.sh` 广播恒等静态 TF `map -> world` |

因此 RViz 下点（`map`）与 SRU 的 `world` 里程计是一致的；VLN 下点则必须先经 base→world 变换（节点内部完成）。

---

## 2. 实现位置

改动直接集成在 SRU 控制器内部（未新增节点/话题）：

- `go2w_nav_ros2_ws/src/rl_nav_controller/rl_nav_controller/rl_nav_controller.py`
- `go2w_nav_ros2_ws/src/rl_nav_controller/rl_nav_controller/rl_nav_controller_adaptive.py`
  （`rl_nav_controller_stairs.py` 继承自 adaptive，自动生效）

关键函数：

| 函数 | 作用 |
|:---|:---|
| `target_position_callback()` | 按 `goal_frame_mode` 区分 base 相对点 / 世界绝对点 |
| `_is_base_frame_goal()` | 判断 `frame_id` 是否属于 base 帧 |
| `_handle_base_frame_goal()` | 用当前 odom 位姿把相对点转换到 world，设置目标与朝向 |
| `check_goal_stream_timeout()` | 可选的 10 Hz 看门狗（默认关闭）：无新点超时 → 刹停 → `STOPPED` |

---

## 3. 行为细节

### 3.1 相对点 → 世界点

每次收到 base 相对点时，用当前里程计位姿做一次刚体变换：

```
p_world = R(robot_yaw/pitch/roll) * p_base + p_robot
```

- 位姿来源：`/lio/robo/odom`（参数 `odom_topic`），与 SRU 策略输入使用同一份里程计。
- **一次性变换**：点收到后固定在世界系；VLN 下一次发点会覆盖为上一点的新世界位置。
- z 分量：`p_base.z = 0` 时结果约为机器人当前高度；需要抬升/俯仰目标时可给非零 z。
- 距离钳位：`base_goal_max_distance`（默认 5 m）防止异常大点把机器人带飞。

### 3.2 朝向

- `orientation` 有效（四元数范数 > 0.5）：解析为**相对车头的偏航增量**，
  `target_yaw_world = robot_yaw + relative_yaw`。
- `orientation` 全零：继承机器人当前朝向，即“只跟点、不额外转向”。

### 3.3 到点与停止（位置判定，10 cm）

对 base 相对点使用**位置到点判定**：

1. 收到点后立即按当前位姿转换到世界系并进入 `RUNNING`；
2. 机器人进入目标 **`base_goal_arrive_threshold`（默认 0.10 m）** 内：
   - 主动发布零速刹停；
   - 清空当前目标，状态 `GOAL_REACHED` → `IDLE`；
3. VLN 再发布新点（包括在同一点附近的微调）→ 自动回到 `RUNNING`，继续跟随。

**不做同点去重**：每一条 base 点都会按收到时的机器人位姿重新锁定世界目标。
因此如果持续发布**同一个相对点**（如恒定的“车头前方 1.2 m”），目标会随车前进，机器人会持续行走——这是相对坐标的语义决定的。需要到点停止时，VLN 应在到达后发布接近当前机器人位置的点或停止发布。

**可选的“无新点”看门狗**：`goal_stream_timeout` 默认 **0（关闭）**。如需上游长时间断流时兜底刹停，可设为 2~3 倍发点周期；超时后进入 `STOPPED`，新点到达自动恢复 `RUNNING`。

### 3.4 手动暂停优先

为避免持续以 5 Hz 发点的 VLN 把人工急停“顶回去”，以下操作会锁存暂停状态：

- 遥控器 B 键（`/rsl_joy`）；
- `/motion_enabled = false`（宇树遥控 B / 桥接急停）；
- `/mission/control: "pause"`。

暂停期间 VLN 继续发点只会更新目标，机器人保持 `STOPPED` 不动；按遥控器 A、发布 `/motion_enabled = true` 或 `/mission/control: "resume"` 解除锁存，下一帧 VLN 点即恢复 `RUNNING`。

### 3.5 与全局路径 / RViz 共存

- 收到 `frame_id` 为 `world` / `map` / `odom`（或与 odom 帧一致）的点：按世界绝对点处理，退出 VLN 流模式；
- 收到 `/global_path` 路径：退出 VLN 流模式，走原有分段路径跟踪；
- 因此同一套 SRU 既可以接 VLN，也可以接 RViz / 全局规划，不需要重启。

---

## 4. 参数

| 参数 | 默认 | 说明 |
|:---|:---:|:---|
| `goal_frame_mode` | `auto` | `auto`：`frame_id` 属于 base 帧才按相对点处理；`base`：所有 `/goal_pose` 都按相对点；`world`：全部按绝对点（旧行为） |
| `base_frame_ids` | `base,base_link,base_footprint,body,imu` | `auto` 模式下识别为 base 帧的 ID 列表 |
| `base_goal_arrive_threshold` | `0.10` | VLN 相对点的位置到点半径（m） |
| `goal_stream_timeout` | `0.0` | 可选的无新点看门狗（s）；`0` = 关闭（默认，按位置到点） |
| `base_goal_max_distance` | `5.0` | 相对点最大距离（m），超出按方向等比钳位；`0` 关闭 |
| `odom_topic` | `/lio/robo/odom` | 相对点转换使用的里程计话题 |

默认 `goal_stream_timeout=0`（关闭），到点完全由 10 cm 位置判定。仅在上游可能异常断流时，才把它设为 VLN 发点周期的 2~3 倍作为兜底。

---

## 5. 拉取、编译与部署

### 5.1 需要部署的文件

| 仓库内路径 | 机载目标位置 | 说明 |
|:---|:---|:---|
| `go2w_nav_ros2_ws/src/rl_nav_controller/`（整包） | `~/go2w_nav_ros2_ws/src/rl_nav_controller/` | 控制器源码 + `deployment_policies/` 下随仓库提交的 ONNX 模型（`vae_encoder.onnx`、`nav_policy.onnx`） |
| `sru_ws/start_local_sru.sh`、`stop_local_sru.sh`、`realsense_nav_override.yaml` | `~/sru_ws/` | 一键启动/停止脚本与 RealSense 相机覆盖配置（仓库内 `sru_ws/` 即机载 `~/sru_ws/` 的规范副本） |

> 模型说明：默认策略模型为 **`nav_policy.onnx`**（随仓库提交于 `deployment_policies/`），`colcon build` 时由 `setup.py` 拷入 install share，控制器启动时默认加载；`vae_encoder.onnx` 启动时无条件加载，缺失则节点起不来。

### 5.2 拉取代码

团队元仓库（vcstool，含 `agile_nav.repos`）中本仓库位于 `src/sru-navigation`，跟踪分支 `sru-robot-deployment`：

```bash
cd ~/agile_nav_ws                 # 团队元仓库目录（含 agile_nav.repos）
vcs pull src                      # 更新全部子仓库；只更新本仓库可执行: cd src/sru-navigation && git pull
```

### 5.3 同步到机载并编译

在开发机上（拉取完成后）执行：

```bash
WS=~/agile_nav_ws                 # 团队元仓库目录
REPO=$WS/src/sru-navigation
ROBOT_IP=192.168.123.111          # 机器人自带 WiFi (GL-MT3000-ef3-5G)；走 agile_guest 时用 10.3.42.161

# 1) 同步控制器整包（含 deployment_policies/ 下的 ONNX 模型）
rsync -avz $REPO/go2w_nav_ros2_ws/src/rl_nav_controller/ \
    agile@$ROBOT_IP:~/go2w_nav_ros2_ws/src/rl_nav_controller/

# 2) 同步启动脚本与相机配置
rsync -avz $REPO/sru_ws/start_local_sru.sh $REPO/sru_ws/stop_local_sru.sh \
    $REPO/sru_ws/realsense_nav_override.yaml agile@$ROBOT_IP:~/sru_ws/
ssh agile@$ROBOT_IP "chmod +x ~/sru_ws/start_local_sru.sh ~/sru_ws/stop_local_sru.sh"

# 3) 机载编译（模型随包装入 install share）
ssh agile@$ROBOT_IP "source /opt/ros/humble/setup.bash && cd ~/go2w_nav_ros2_ws && \
    colcon build --packages-select rl_nav_controller --symlink-install"
```

> 若车端是全新机器（尚无 `~/go2w_nav_ros2_ws`）：除上表外，还需把仓库里 `go2w_nav_ros2_ws/src/`（含 Super-LIO、go2w_bridge）与 `bridge_ws/src/livox_ros_driver2`（激光雷达驱动）同步到机载对应工作空间并整体 `colcon build`；另外 `realsense2_camera` 不在本仓库内，需机载预装（`sudo apt install ros-humble-realsense2-camera`）。

## 6. 启动（机载端）

### 6.1 一键启动局部 SRU（默认即 VLN 模式）

```bash
cd ~/sru_ws
./start_local_sru.sh          # 默认 = --vln：base 相对点 + 关 RViz + 10 cm 到点，RealSense D435i
```

停止：启动终端 `Ctrl+C`（自动安全清理全部节点），或另开终端执行 `./stop_local_sru.sh`。

手动发一个测试点（表示“从当前位置向前走 1.5 m，进入 10 cm 内即停”）：

```bash
ros2 topic pub /goal_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'world'}, pose: {position: {x: 1.5, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}" --once
```

> VLN（base）模式下 `frame_id` 被忽略，任意值均可；VLN 判定到点后停止发点即可。

### 6.2 start_local_sru.sh 命令行可调参数

| 参数 | 说明 | 默认 |
|:---|:---|:---|
| `--vln` | VLN 模式：所有 `/goal_pose` 一律按 base 相对点处理，并关闭 RViz。**默认行为已等价于该参数**，保留用于显式声明 | 默认开 |
| `--realsense` / `--d435` / `--d435i` | 使用 RealSense D435i 深度相机（本项目唯一相机） | 默认 |
| `--zed` / `--camera <type>` | 切换相机类型（本项目不涉及 ZED，仅兼容保留） | realsense |
| `--rviz` / `--no-rviz` | 手动打开/关闭 RViz 可视化 | 关 |
| `--goal-mode <auto\|base\|world>` | 目标帧模式：`auto`=按 frame_id 判别；`base`=全部按相对点（VLN 用法）；`world`=全部按世界绝对点（旧行为） | `base` |
| `--goal-timeout <秒>` | “无新点”看门狗：RUNNING 下超过该时长未收到新点 → 刹停；**0 = 关闭**（只靠 10 cm 位置到点） | `0.0`（关） |
| `--goal-arrive <米>` | VLN 相对点的位置到点半径 | `0.10` m |
| 透传参数 | 其余参数原样透传给 `rl_nav_controller`，如 `--policy-path <路径>`（自定义 ONNX 策略模型；不指定时用仓库自带 `nav_policy.onnx`）、`--odom-topic`、`--blocked-timeout` 等 | — |

常用组合：

```bash
./start_local_sru.sh                                   # 标准 VLN 部署（默认行为）
./start_local_sru.sh --goal-mode auto --rviz           # 恢复旧行为：RViz 2D Nav Goal 下点调试
./start_local_sru.sh --goal-timeout 1.5                 # 额外开启“停发即刹停”看门狗
./start_local_sru.sh --goal-arrive 0.15                 # 到点半径调为 15 cm
./start_local_sru.sh --policy-path /path/to/model.onnx  # 指定自定义策略模型
```

### 6.3 全局导航栈 / 其他启动方式

若使用 `start_nav.sh`、自适应 launch 或 stairs 版本，直接给节点加参数：

```bash
ros2 run rl_nav_controller rl_nav_controller_adaptive ... \
  --ros-args -p goal_frame_mode:=base -p base_goal_arrive_threshold:=0.10
```

或持续向 `/goal_pose` 发布 `frame_id: base_link` 的消息（`auto` 模式会自动识别）。

---

## 7. VLN 侧发布示例（Python）

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class VlnGoalPublisher(Node):
    def __init__(self):
        super().__init__('vln_goal_publisher')
        self.pub = self.create_publisher(PoseStamped, '/goal_pose', 1)
        self.timer = self.create_timer(0.2, self.tick)  # 5 Hz

    def tick(self):
        if self.arrived:          # VLN 判断到点后不再发布
            return
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.pose.position.x = self.next_x   # 相对 base 的前向位移 (m)
        msg.pose.position.y = self.next_y   # 相对 base 的左向位移 (m)
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 0.0        # 不约束朝向
        self.pub.publish(msg)
```

要点：

- 按 VLN 的决策节奏发布即可；每条点都会按收到时位姿重新锁定目标；
- 到达后停止发布（或不发点），SRU 已在 10 cm 内自行刹停；
- 每次只发一个点，SRU 收到即覆盖上一目标，无需清空/取消接口；
- 如需紧急停止，直接向 `/mission/control` 发布 `std_msgs/String: "pause"`。

---

## 8. 调试

```bash
# 观察 SRU 是否收到相对点并转换
# 日志关键字：[VLN] base goal=... -> world=...
ros2 topic echo /local_planner/status
# running / stopped / idle / error

# 确认里程计就绪（未就绪时相对点会被忽略）
ros2 topic hz /lio/robo/odom
```

预期日志序列：

```
Goal input: /goal_pose | frame_mode=base | base_frames=[...] | base_arrive=0.10m | stream_timeout=0.00s | ...
[VLN] base goal=[1.20, 0.00, 0.00] (1.20m) -> world=[x, y, z], target_yaw: ... Arrival radius: 0.10m.
...
Arrived at the goal position and heading. Distance: 0.087m <= 0.100m
VLN point reached: waiting for the next base-frame goal.
```

（仅在手动开启 `goal_stream_timeout > 0` 时才会出现 `[VLN Watchdog] ... state -> STOPPED`。）

---

## 9. 注意事项

1. **里程计必须先就绪**：收到相对点前若 `/lio/robo/odom` 尚无数据，该点会被忽略（日志会提示）。
2. **超时参数要匹配 VLN 发点频率**，否则会出现“点间隔过长 → 误刹停”或“到点后停得太晚”。
3. **相对点不是全局规划**：只适合短时局部目标；长距离导航请使用全局路径模式。
4. **一次只看一个点**：后到的点覆盖先到的点；VLN 无需撤销旧点。
5. **到点判定完全交给 VLN**：SRU 不再因距离近而自行结束任务，避免 VLN 与 SRU 两套到点标准互相打架。

---

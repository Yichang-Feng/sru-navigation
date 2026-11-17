# VLN 相对 base 目标点接入规范（复用 `/goal_pose`）

> 适用对象：上层 VLN（Vision-Language-Navigation）模块开发者。
> 目标：VLN 只需在**现有** `/goal_pose` 话题上发布“相对机器人 base 的位置点”，SRU 导航节点负责跟随；VLN 到点后停止发点，SRU 自动刹停。
> 不新增任何输入话题、不新增桥接节点。

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

## 5. 启动

### 5.1 单独启动局部 SRU（VLN 模式）

```bash
cd ~/sru-robot-deployment   # 或开发机上的 scripts 目录
./scripts/start_local_sru.sh --vln
```

`--vln` 会：

- 给 SRU 传入 `goal_frame_mode=base`、`base_goal_arrive_threshold=0.10`，无新点看门狗保持关闭；
- **不启动 RViz**（VLN 场景无需可视化）；
- 启动脚本优先加载 `~/go2w_nav_ros2_ws` 中同步后的控制器源码，并检查其中是否包含 `goal_frame_mode` 支持，避免误加载 `~/sru_ws` 下的旧副本。

也可以手工指定：

```bash
./scripts/start_local_sru.sh --goal-mode base --goal-timeout 1.5
```

`--goal-mode auto`（默认）时，VLN 必须在消息里写 `frame_id: base_link`；
如果 VLN 不方便设置 `frame_id`，使用 `--vln` 强制 base 模式即可。

### 5.2 全局导航栈 / 其他启动方式

若使用 `start_nav.sh`、自适应 launch 或 stairs 版本，直接给节点加参数：

```bash
ros2 run rl_nav_controller rl_nav_controller_adaptive ... \
  --ros-args -p goal_frame_mode:=base -p base_goal_arrive_threshold:=0.10
```

或持续向 `/goal_pose` 发布 `frame_id: base_link` 的消息（`auto` 模式会自动识别）。

---

## 6. VLN 侧发布示例（Python）

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

## 7. 调试

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

## 8. 注意事项

1. **里程计必须先就绪**：收到相对点前若 `/lio/robo/odom` 尚无数据，该点会被忽略（日志会提示）。
2. **超时参数要匹配 VLN 发点频率**，否则会出现“点间隔过长 → 误刹停”或“到点后停得太晚”。
3. **相对点不是全局规划**：只适合短时局部目标；长距离导航请使用全局路径模式。
4. **一次只看一个点**：后到的点覆盖先到的点；VLN 无需撤销旧点。
5. **到点判定完全交给 VLN**：SRU 不再因距离近而自行结束任务，避免 VLN 与 SRU 两套到点标准互相打架。

---

## 9. 验证记录

可回归的单元测试：`go2w_nav_ros2_ws/src/rl_nav_controller/test/`（VLN 目标逻辑、PathSegmentTracker API、B 暂停/恢复路径）

```bash
cd go2w_nav_ros2_ws/src/rl_nav_controller
source /opt/ros/humble/setup.bash
python3 -m pytest test/ -q   # 19 passed
```

覆盖：

- base/world 帧识别与模式切换；
- base→world 位置/朝向转换（含 yaw=90° 场景）；
- 距离钳位；
- 无里程计保护；
- 10 cm 位置到点判定；
- 可选超时看门狗刹停 / 新点恢复 / 超时关闭；
- 同一点按新位姿重新锁定（无去重）；
- 绝对点透传与退出 VLN 模式；
- 手动暂停锁存不被连续发点覆盖。

另：`colcon build --packages-select rl_nav_controller` 编译通过。

---

## 10. 常见问题

### Q1：日志报 `Target frame_id "base_link" does not match odometry frame_id "world"`

这表示机载运行的还是**旧版控制器**（旧版只接受 `world` / `map` / `odom` / 空帧）。
新版启动时会打印一行：

```
Goal input: /goal_pose | frame_mode=base | base_frames=['base', 'base_link', ...] | stream_timeout=2.00s | ...
```

排查：

1. 看 `start_local_sru.sh` 输出的 `直接加载最新策略源码: <路径>`，确认加载的是刚同步的文件
   （脚本现在优先 `~/go2w_nav_ros2_ws`，并会跳过不含 `goal_frame_mode` 的旧副本）；
2. 如果使用 `ros2 run rl_nav_controller ...`，必须在机载端重新 `colcon build --packages-select rl_nav_controller`；
3. `~/sru_ws/src/sru-robot-deployment/...` 与 `~/sru_ws/src/rl_nav_controller/...` 下的旧副本也要更新
   （新版 `sync_to_go2w_nav.sh` 已自动同步存在的旧副本目录）。

### Q2：`--vln` 启动后没有 RViz

这是预期行为：`--vln` 会关闭 RViz。如需强制打开，在命令中追加 `--rviz`（参数顺序在后即可生效）。

### Q3：不让 VLN 设置 `frame_id` 怎么办？

用 `--vln`（等价于 `goal_frame_mode=base`），此时所有 `/goal_pose` 消息都按 base 相对点解释，`frame_id` 内容被忽略。

### Q4：为什么之前刹停后整个节点/脚本会退出？

旧版 `main()` 用 `except (KeyboardInterrupt, Exception): pass` 吞掉了 `rclpy.spin()` 抛出的异常，随后执行 `finally` 关机，表现为“自己退出”。现已：

- 拆分异常处理，非 Ctrl+C 异常会打印完整 traceback；
- 看门狗回调整体加异常保护，单个定时器异常不再终止导航；
- 默认关闭“无新点”看门狗，停止由 10 cm 位置判定完成（不再触发该路径）。

如果仍然退出，请把终端里的 traceback 发来，可据此定位具体回调。


### Q5：行进中按 B 或触发 STOPPED 后节点退出

根因：单独 SRU 使用的 `path_tracker.py` 中 `PathSegmentTracker` 缺少 `has_path()`；
而 `motion_enable_callback`、`joy_callback` 的 B 分支以及 `generate_cmd_vel` 的
STOPPED 分支都会调用它。一旦状态切到 `STOPPED`，下一次深度回调即抛
`AttributeError`，`rclpy.spin` 退出，表现为按 B 就退出（无新点看门狗刹停后退出也是同一原因）。

修复：`path_tracker.py` 已补齐 `has_path()`、`get_terminal_pose()`、
`realign_to_robot()`，并让 `set_path()` 接受 `frame_id`；`sync_to_go2w_nav.sh`
现在会同步 `path_tracker.py` / `adaptive_path_tracker.py` / `stairs_depth_processor.py`。

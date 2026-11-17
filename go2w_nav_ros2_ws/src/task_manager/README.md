# task_manager：逐点任务执行器

定位成功确认后，接收 `/mission/command` 中已经展开的 steps，逐点向 `/goal_3d` 发布
`PoseStamped`。全局或局部报告当前点到达后，记录一次 reached，完成 dwell 再下发下一点。
仅修改本功能包，不使用 action、不设置目标执行或控制确认超时，不发布 `/cmd_vel`。

本文和 [导航交接单](navigation-integration-handoff.md) 描述当前实现；
`mission-protocol-handoff.md` 保留原文，其中旧 action、控制、超时及终态保留条款由当前说明覆盖。

## 接口

| 方向 | 默认话题 | 类型及内容 |
|---|---|---|
| 收 | /mission/command | std_msgs/msg/String，JSON，command 为 start/pause/resume/stop |
| 发 | /mission/status | std_msgs/msg/String，完整 JSON 状态快照 |
| 发 | /mission/control | std_msgs/msg/String，纯字符串 pause 或 resume |
| 发 | /goal_3d | geometry_msgs/msg/PoseStamped，当前 step 的 xyz/yaw/frame |
| 收 | /global_planner/status | std_msgs/msg/String，全局规划状态原文 |
| 收 | /local_planner/status | std_msgs/msg/String，JSON：state 和字符串数组 msgs |
| 收 | /localization/status | std_msgs/msg/String，Super-LIO JSON：state 和字符串数组 msgs |
| 收 | /global_path | nav_msgs/msg/Path，匹配当前目标的新全局路径 |
| 收 | /loc/robo/odom | nav_msgs/msg/Odometry，机器人位置 |
| 收 | /estop | std_msgs/msg/Bool，独立安全中止输入 |

局部消息示例：`{"state":"error","msgs":["astar_no_path","unknown_reason"]}`。
仅接受 idle/running/goal_reached/error/stopped，msgs 必须存在且每项都是字符串。
非法 JSON、未知 state、缺失字段或字段类型错误只记日志，不修改任务或诊断状态。
原因不设枚举白名单，多条原因及未知原因原样保留。

`/mission/status` 保持 schema_version=1，增加 `local_planner_status`（解析后的
`{state, msgs}` 或 null），保留 `global_planner_status`（原文字符串或 null），
新增 `localization_status`（最近有效定位 `{state, msgs}` 或 null）。
状态、原因变化立即发布，心跳重复携带最新诊断。每次准备新目标清空全局请求原文；
局部诊断跨任务保留，包含 stopped 状态，以便新目标安全恢复。

命令、任务状态、控制及局部状态订阅/发布为 reliable、volatile、depth 10。
即使局部发布者为 transient_local，也不重放启动前缓存的到达结果。
全局请求订阅同样为 volatile；另有 reliable、transient_local、depth 1 订阅仅处理
ready/map_load_failed，发现缓存地图故障且忽略旧请求结果。
Path 为 reliable、transient_local、depth 1；odom 为 SensorDataQoS。

## 定位初始化门槛与心跳（2026-11-05）

启动立即发布 `state=uninitialized_location`，run/target/step_state 等任务字段为空，
`localization_status=null`，以 status_rate_hz（默认 2 Hz）持续上报；不提前发布普通 idle。
收到 ready 后保持未初始化，并提示等待初始化。定位节点未启动或没有传感器输入时，也保持等待。
任务自身/地图加载故障的锁存 error 优先，定位消息不能清除这些故障。

定位订阅为 **reliable、transient_local、depth 1**，接收后启动时的缓存状态。
只有连续两条有效 `succeed`、且间隔不超过 localization_status_timeout_sec（默认 3 秒），
才进入 idle 并允许任务；首次 succeed 可能只是历史缓存，需要等后续成功心跳确认。
成功后上游应持续按 1 Hz 发布 succeed，健康心跳本身不会额外重复发布任务快照。
校验 JSON 对象、五种 state 及字符串数组 msgs，非法消息只记日志，不能刷新成功心跳。

| 定位 state | 处理 |
|---|---|
| ready | 关闭门槛，等待建立定位，不要求 ready 周期上报 |
| succeed | 第一条等待后续心跳；第二条及时到达后放行，之后持续刷新成功时间 |
| error | 关闭门槛，完整保留可恢复原因 |
| initialized_failed | 关闭门槛，保留初始化失败原因，等待重新初始化成功 |
| failed | 关闭门槛，保留上游终态原因；需重启定位程序后再做成功确认 |

任何非 succeed 都重置连续成功确认。成功心跳间隔**超过** 3 秒同样关闭门槛，
包括首次确认期间；ready/error/initialized_failed/failed 是事件状态，不要求心跳，
长时间无消息不会被误判为成功心跳超时。计时使用单调时钟，与 ROS 仿真时间暂停无关。

未就绪时新 start 被拒绝：`command.accepted=false, reason=localization_not_ready`，
不保存任务、不下发目标或控制；定位就绪后前端必须重新发送 start。
已经接受的同一 run/request 重试仍保持幂等，不会重新执行已失败任务。
即使 TF 和 odom 持续有输出，也不能替代定位 succeed 确认。

运行中收到任何非 succeed，或成功心跳超时：立即请求局部 pause，发布一次带完整运行信息的
failed，然后清空任务信息并回到 **uninitialized_location**，不经过普通 idle。
原任务不在定位恢复后自动继续；定位重获两条成功消息后，只允许重新提交新 run。
旧导航占用仍须收到 stopped 或原有请求结束反馈才释放，定位恢复本身不是停车确认。
若暂停/取消已经请求 pause，则不重复发 pause；定位失效的安全中止优先于待确认取消。

`localization_status` 持续携带最后有效原文对象及全部 msgs，跨任务保留。
超时时保留最后的 succeed，不伪造上游 error，通过 message 说明 `succeed heartbeat timed out`。
在未初始化期间，全局/局部反馈仍可更新诊断或释放旧占用，但不能开始或推进任务。
定位恢复不解除 estop 或任务/地图锁存故障；上游 failed 不额外锁存为任务节点自身故障，
重启定位程序并重新满足两次成功确认后即可重新提交任务。

## 任务控制和状态迁移

| 前端 command | 下游控制及任务行为 |
|---|---|
| start | 定位未就绪时拒绝；就绪后只发布当前目标，不发 start，保持 starting 等规划/执行确认 |
| pause | 发 pause，进入 canceling；收到局部 stopped 后才进入 paused |
| resume | 已到达时重计完整 dwell；需要导航时发 resume，等待局部 running 后进入 running/navigating |
| stop | 发 pause，进入 canceling；收到 stopped 后发布一次 canceled，紧接着回 idle |

canceling 保留 run、索引、目标，step_state=null，不推进列表，不接受另一 run。
没有 stopped 确认就持续等待并在 message 中说明原因，无控制确认超时。
重复 pause/resume 不重复发控制；stop 可覆盖待确认的 pause，仍以 canceled 结束。

暂停或取消后的**新目标**先发 `/goal_3d`，收到与当前目标匹配的新 `/global_path` 后才发 resume。
全局 path_found 本身不能解除 stopped，也不能替代路径匹配；发送 resume 后仍保持 starting，
直到收到局部 running。恢复同一个未完成目标时可使用本节点已验证的该目标路径；
规划期间暂停且尚未收到路径时继续等路径，不重复发布目标。

暂停与到达交错时仍记录 reached，但保持 canceling 直到 stopped。
暂停后已到达的点在 resume 时重计完整 dwell，期间不发送 resume 恢复旧轨迹；
下一点下发并收到匹配路径后才解除局部 stopped。取消意图即使收到到达仍以 canceled 结束。
没有待确认请求时收到 stopped，若任务正在执行则作为外部暂停处理。

| 局部 state | 任务行为 |
|---|---|
| idle | 只更新诊断，不判定完成 |
| running | 当前目标开始导航、清除局部故障；不解除人工暂停或待确认控制，也不绕过恢复路径门控 |
| goal_reached | 无需先有 running，和全局到达共用一次完成入口，距离置 0，进入 dwell 或等暂停确认/继续 |
| error | 对外报告可恢复 error，保留 run、目标和原 step_state，阻止列表推进，不记录 failed、不锁存 fault |
| stopped | 确认 pause/stop；或触发外部暂停 |

局部 error 恢复到 running 后继续等当前点到达，恢复到 goal_reached 则直接处理到达。
人工暂停不因恢复反馈而解除；paused/canceling 优先显示控制阶段，局部故障仍在诊断字段中可见。
局部 error 期间继续检查独立 TF 和 estop；若 dwell 期间出现 error，暂不推进，恢复后继续原计时。

任务自身错误及地图故障为**锁存 error**，不会因局部 running/ready 或新 start 清除，修复后需重启。
局部可恢复 error 的 message 前缀为 `local planner error:`；不能将所有 error 都解释为必须重启。

## 全局规划与安全

| 全局原文 | 任务行为 |
|---|---|
| ready | 更新诊断，不清除故障 |
| planning | 等规划时保持 starting，不重复发目标，不解除暂停/取消 |
| path_found | 普通执行进入 running/navigating；等待恢复确认时仍保持 starting |
| goal_reached | 与局部共用到达入口及去重逻辑 |
| planning_failed: ... | 当前点 failed、任务 failed 一次后 idle，完整原因保留，释放目标占用 |
| map_load_failed: ... | 即使空闲也进入锁存 error |
| 其他 | 只更新原文 |

pause 待确认时发生全局规划失败，先等 stopped 再上报 failed；stop 待确认时仍以 canceled 结束。
空闲、已完成点及终态后的请求反馈只更新诊断，不复活任务或重复记录结果。

独立 `/estop` 或 TF 定位丢失仍立即发布一次 failed 后回 idle，并请求下游 pause。
旧执行未确认结束时内部保留占用，新的 run 被拒绝；stopped 或旧请求到达/规划失败可释放占用。
急停持续激活时仍拒绝 start。局部 error 中的 emergency_safety 原因不等同于独立 `/estop`。

succeeded/canceled/failed 各只发布一次完整终态，包含 run、索引及最近单点结果；定位就绪时紧接着发布 idle，
否则发布 uninitialized_location。两者均清空任务信息、target、step_state 和距离；未初始化时保留定位提示。
idle 固定 1 Hz，其余状态（含 uninitialized_location）默认 2 Hz。
前端应保存单次终态事件。幂等历史保留，重复 start 不重新执行已经接受的 run/request。
节点重启不恢复任务。拒绝回执、严格命令校验及逐点结果去重保留。

stopped 是局部软件状态确认，未新增底盘速度判断；实车停车仍须验证。
本节点不使用 `/motion_enabled`，不写 `/global_path`，不增加其他目标后备路径。

## 剩余距离

机器人位置转换到路径坐标系，投影到路径线段的 Q 点：

```text
remaining_distance_m = |机器人位置 - Q| + 路径总长 - Q 的累计路径长度
```

使用三维路径弧长和偏离路径的连接距离，不是到终点的直线距离。
曲线路径 (0,0)→(3,0)→(3,4) 中机器人在 (1,0) 时剩余 6 米。
该估计不包含局部绕障额外路程，不保证连接段可通行，只作显示，不判定到达。

每次发目标清空旧路径；路径时间戳不得早于当前目标发布时间，后续路径时间戳必须递增。
路径末点与当前目标 XYZ 误差不超过 0.05 米，Z 考虑 global_path_z_offset；异帧目标通过 TF 转换。
空路径、非法坐标、帧不一致、终点不符均不用于距离或触发 resume。
odom 按 header.frame_id 和自身时间戳转换，数据时间戳和接收时间均不得超过 tf_timeout_sec。
第一次按最近投影定位进度，后续在至少前后 3 米、随位移扩大的窗口搜索，减少交叉路径跳变。
数据缺失、陈旧或 TF 不可用时为 null，到达后为 0；新目标、终态及 idle 清空。

## 构建与启动

```bash
cd /home/jiewang/a2_nav_ros2_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select task_manager
source install/setup.bash
ros2 launch task_manager task_manager.launch.py
```

| 参数 | 默认值 |
|---|---|
| robot_id | a2 |
| command_topic / status_topic / control_topic | /mission/command / /mission/status / /mission/control |
| goal_topic | /goal_3d |
| global_planner_status_topic / local_planner_status_topic | /global_planner/status / /local_planner/status |
| localization_status_topic | /localization/status |
| localization_status_timeout_sec | 3.0，成功心跳有效期及两次成功确认的最大间隔 |
| global_path_topic / odom_topic | /global_path / /loc/robo/odom |
| global_path_z_offset | 0.5，需匹配全局 path_z_offset |
| frame_id / robot_frame | map / base |
| status_rate_hz | 2.0，idle 固定 1 Hz |
| tf_timeout_sec | 2.0，定位及距离数据有效期，不是执行超时 |
| dwell_default_sec | 1.0；v1 每个 step 仍必须明确提供 dwell_sec |

所有参数支持同名 launch 覆盖，仅在启动时读取；命令 frame_id 优先。
旧 planner_status_topic、goal_timeout_sec 和 action 参数均不使用。
yaw 写入目标四元数，最终朝向由导航执行链负责。

实机应核对 map→robot_frame：当前默认适配 map→world→base。
缺失、陈旧和未来时间戳 TF 会输出具体警告，不能靠模拟 running 代替定位。

```bash
ros2 run tf2_ros tf2_echo map base
ros2 launch task_manager task_manager.launch.py robot_frame:=base
```

## 命令和局部模拟示例

以下目标示例仅在隔离环境中测试；实机坐标由实际 mission 提供。

```bash
ros2 topic pub --once /mission/command std_msgs/msg/String '{data: "{\"schema_version\":1,\"robot_id\":\"a2\",\"run_id\":\"demo1\",\"request_id\":\"start1\",\"command\":\"start\",\"frame_id\":\"map\",\"mission\":{\"id\":\"m1\",\"name\":\"demo\",\"map_id\":\"map1\",\"mode\":\"once\",\"loop_count\":1,\"steps_total\":1,\"steps\":[{\"step_index\":0,\"lap\":1,\"point_index\":0,\"point_total\":1,\"name\":\"p1\",\"waypoint_id\":null,\"x\":1.0,\"y\":2.0,\"z\":0.0,\"yaw\":0.0,\"dwell_sec\":1.0}]}}"}'
ros2 topic pub --once /mission/command std_msgs/msg/String '{data: "{\"schema_version\":1,\"robot_id\":\"a2\",\"run_id\":\"demo1\",\"request_id\":\"pause1\",\"command\":\"pause\"}"}'
ros2 topic pub --once /local_planner/status std_msgs/msg/String '{data: "{\"state\":\"stopped\",\"msgs\":[]}"}'
ros2 topic pub --once /local_planner/status std_msgs/msg/String '{data: "{\"state\":\"running\",\"msgs\":[]}"}'
ros2 topic pub --once /local_planner/status std_msgs/msg/String '{data: "{\"state\":\"error\",\"msgs\":[\"astar_no_path\",\"optimize_failed\"]}"}'
ros2 topic pub --once /local_planner/status std_msgs/msg/String '{data: "{\"state\":\"goal_reached\",\"msgs\":[]}"}'
ros2 topic echo /mission/status std_msgs/msg/String
ros2 topic echo /mission/control std_msgs/msg/String
```

继续/取消使用同一 run_id，将 command 改为 resume/stop 并使用对应 request_id。
取消后重新开始必须使用新的 run_id/request_id；重复 start 只返回当前快照。

### 定位状态模拟

仅在隔离 ROS 测试环境使用。先启动任务节点，再模拟 ready：

```bash
ros2 topic pub --once /localization/status std_msgs/msg/String \
  --qos-reliability reliable --qos-durability transient_local \
  '{data: "{\"state\":\"ready\",\"msgs\":[]}"}'
```

在另一个终端持续模拟成功心跳，等待第二条消息后再发送任务；停止发布超过 3 秒会关闭门槛并中止活动任务。

```bash
ros2 topic pub --rate 1 /localization/status std_msgs/msg/String \
  --qos-reliability reliable --qos-durability transient_local \
  '{data: "{\"state\":\"succeed\",\"msgs\":[]}"}'
```

模拟定位丢失前先停止成功心跳发布者，避免两个模拟发布者交替改变状态：

```bash
ros2 topic pub --once /localization/status std_msgs/msg/String \
  --qos-reliability reliable --qos-durability transient_local \
  '{data: "{\"state\":\"error\",\"msgs\":[\"localization_lost\",\"sensor_data_timeout\"]}"}'
```

初始化/重新初始化由 Super-LIO 的 `/initialpose` 或其已有服务完成，本包不代发初值、不调用重定位服务。
定位状态没有时间戳、序号或进程标识，无法严格识别缓存所属的进程；两次成功和心跳超时是接收侧检查。
实机必须验证定位异常后的实际停车，以及定位程序重启、重新初始化后提交新任务的完整流程。

## 测试

```bash
source /opt/ros/foxy/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src/task_manager:$PYTHONPATH python3 -m pytest -q -rs src/task_manager/test
python3 -m flake8 src/task_manager/task_manager src/task_manager/test src/task_manager/launch src/task_manager/setup.py --max-line-length=100
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon test --packages-select task_manager --python-testing pytest --event-handlers console_direct+
colcon test-result --test-result-base build/task_manager --verbose
TASK_MANAGER_ROS_TESTS=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=178 \
  ROS_LOG_DIR=/tmp/task-manager-ros-tests PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  PYTHONPATH=src/task_manager:$PYTHONPATH python3 -m pytest -q src/task_manager/test/test_ros_integration.py
```

本机旧 pytest 与全局插件冲突，禁用自动加载；colcon 显式选择 pytest 避免跑零项。
DDS 测试默认跳过，单独显式运行；使用随机隔离话题、robot_frame 和模拟 TF，不向实车发目标。
具体通过项、接口限制和待实机验收项见交接单。

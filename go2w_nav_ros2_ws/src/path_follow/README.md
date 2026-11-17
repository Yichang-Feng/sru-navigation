# path_follow

ROS 2 Foxy Python 路径跟随器，订阅全局和局部 `nav_msgs/Path`，由配置决定哪一种生成 `/cmd_vel`（`geometry_msgs/Twist`）。默认跟随局部路径。两种来源互不覆盖，输入失效时停车，不自动切换来源。

## 编译和运行

```bash
cd /home/jiewang/a2_nav_ros2_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select path_follow
source install/setup.bash
ros2 launch path_follow path_follow.launch.py
```

编辑 `config/path_follow.yaml` 并重新编译，或复制 YAML 到自定义位置：

```bash
ros2 launch path_follow path_follow.launch.py params_file:=/absolute/path_follow.yaml
```

选择全局路径：

```yaml
path_follower:
  ros__parameters:
    path_source: global
    global_path_topic: /global_path
    local_path_topic: /local_path
    odom_topic: /lio/robo/odom
```

选择局部路径时设置 `path_source: local`。参数启动时读取且只读，修改后重启；无效来源或非法控制参数会拒绝启动。全局模式直接执行全局路径，实时绕障需要选择 local 并运行 `local_planner`。

回放 rosbag 时追加 `use_sim_time:=true`。标准入口是 `path_follow_node`，保留 `path_follow_py_node` 兼容旧命令。例如：

```bash
ros2 run path_follow path_follow_node --ros-args \
  --params-file /absolute/path_follow.yaml
ros2 run path_follow path_follow_py_node --ros-args \
  -p path_source:=global -p odom_topic:=/loc/robo/odom -p odom_frame:=map
```

兼容旧 `path_topic` 参数：非空时覆盖所选来源的话题，建议改用 `global_path_topic` / `local_path_topic`。原来的速度、前视距离等同名参数继续可用，默认速度调整为 0.30 m/s。

## 接口与坐标

| 接口 | 默认话题 | 坐标 / QoS |
|---|---|---|
| 全局路径输入 | `/global_path` | `map`；可靠、transient_local、depth=1 |
| 局部路径输入 | `/local_path` | `world`；可靠、volatile、depth=1 |
| 里程计输入 | `/lio/robo/odom` | `world`；SensorDataQoS |
| 速度输出 | `/cmd_vel` | 机体坐标系：前进 linear.x、偏航 angular.z |

全局路径可只发布一次，但发布端须使用 transient_local。局部路径须连续刷新；空 Path 立即取消所选路径。全局与局部话题必须不同。Path 的 frame_id 必须匹配对应配置，内部 PoseStamped 可省略 frame_id，否则须与 Path 一致。

里程计原点与路径参考点须对应同一机器人位置和离地高度。本节点不调整路径 Z；当前 global_planner / local_planner 输出均使用约 0.50 米参考高度。配置中的 `goal_height_tolerance` 防止在不同楼层或高度不匹配时继续跟随。

控制周期内将最新机器人姿态通过 TF 转到所选路径坐标系，位置和姿态一起变换。全局模式通常需要有效的 `map` / `world` 连接；局部路径和 odom 均为 world 时无需 TF。不会用单位变换替代缺失 TF。动态 TF 过期停车；全静态链不受动态超时约束。TF 明显修正时先输出零速度，再重新匹配路径进度。

## 控制与停车行为

- 沿折线段投影机器人位置，按路径弧长插值前视目标，支持稀疏点路径。采用纯追踪曲率产生转向；大方向误差时原地转向；输出不包含横移、倒车和高度控制。
- 全局路径按已完成进度限制后续搜索范围，降低回环/交叉路段跳段风险。初始匹配按三维距离选择；完全重叠的路段初始选择最先出现的一段。每条新路径重新匹配进度，适应局部路径持续重规划。
- 限制前进和角速度，限制前进加速度和角加速度；转弯时联动降低前进速度，避免角速度响应受限时切弯；接近路径终点减速，满足剩余弧长、水平距离和高度容差后停车。不要求达到终点姿态。
- 默认 odom 超时 0.30 秒、局部路径超时 0.50 秒、动态 TF 超时 0.30 秒。既检查 ROS 时间戳，也以单调时钟检查接收间隔；旧消息反复发送不能绕过时间戳检查。全局路径默认不过期（`global_path_timeout: 0.0`），可配置正数启用。
- 未收到所选路径/odom、空路径、非法位置/姿态、frame 不符、输入超时、TF 不可用、偏离路径过大时，直接发布零速度，不经过缓慢减速。有效输入恢复后自动跟随。ROS 时间回退时清空路径和 odom，需重新接收。
- 启动后等待输入期间持续发布零速度；正常退出时尽力发送零速度。进程强制终止或失去通信时应由底层速度看门狗停车；现有 `a2_bridge` 已提供 `cmd_vel_timeout`。

所有参数及单位见 `config/path_follow.yaml`。仅运行一个向同一 `/cmd_vel` 发布控制指令的跟随器，先按实机调整限速与跟踪误差参数。

## 验证

```bash
colcon test --packages-select path_follow --event-handlers console_direct+
colcon test-result --test-result-base build/path_follow --verbose
```

纯算法测试覆盖稀疏路径投影、前视插值、左右转向、原地转向、角度归一化、终点减速、高度检查、回环进度、速度与加速度限制和非法输入。ROS 集成测试同时运行两种配置，验证来源隔离、全局持久 QoS、TF 修正、局部/odom/TF 超时、取消、恢复及旧入口兼容性。

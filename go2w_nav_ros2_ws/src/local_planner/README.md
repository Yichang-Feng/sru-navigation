# local_planner

独立的 ROS 2 Foxy 局部规划包，算法移植自 SCAN-Planner。包内包含滑动栅格建图、A* 搜索、B 样条优化和重规划状态机，仅接收外部全局参考路径；默认由包内闭环控制器直接跟踪 B 样条并输出 `/cmd_vel`。`/local_path` 保留用于显示和可选几何跟踪，当前不使用 `path_follow`。

## 构建和启动

```bash
cd /home/jiewang/go2w_nav_ros2_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select local_planner --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch local_planner run.launch.py
```

只依赖 ROS 2、Eigen、PCL、OpenCV 等标准库，无需构建原 SCAN-Planner 中的算法、消息或仿真包。配置在 `config/planner.yaml`。可用 `params_file:=/绝对路径/planner.yaml` 替换；配置的节点键必须是 `local_planner_node`。

默认启动 `local_planner_node` 和 `closed_loop_controller` 两个节点，不启动或依赖 `path_follow`。定位、点云、全局规划和 TF 由其他节点提供。已有 `path_follow` 实例应先停止，避免与新控制器同时发布 `/cmd_vel`。可选可视化单独启动：

```bash
ros2 launch local_planner rviz.launch.py
```

启动参数 `body_pose_topic`、`sensor_pose_topic`、`cloud_topic`、`depth_topic` 和 `global_path_topic` 可重映射输入；`local_path_topic` 可重映射局部路径输出，`cmd_vel_topic` 可重映射闭环速度输出。例如：

```bash
ros2 launch local_planner run.launch.py global_path_topic:=/global_path body_pose_topic:=/lio/robo/odom
```

默认 `sensor_type:=lidar`，也支持 `sensor_type:=depth`；depth 模式需要时间同步的深度图与传感器里程计，并应按相机修改 YAML 中的内参和深度比例。`grid_map.need_extrinsic: true` 沿用原实机的传感器外参处理，`grid_map.cloud_is_world: true` 表示点云坐标已在规划坐标系中；接入不同传感器时应核对这些配置和代码中的原实机外参。

## 输入输出

| 方向 | 默认话题 | 消息类型 | 用途 |
| --- | --- | --- | --- |
| 输入 | `/global_path` | `nav_msgs/msg/Path` | 唯一参考路径入口 |
| 输入 | `/lio/robo/odom` | `nav_msgs/msg/Odometry` | 机身位置、姿态和速度 |
| 输入 | `/lio/imu/odom` | `nav_msgs/msg/Odometry` | 雷达/相机位姿 |
| 输入 | `/lio/cloud_world` | `sensor_msgs/msg/PointCloud2` | lidar 模式点云 |
| 输入 | `/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/msg/Image` | depth 模式深度图 |
| 内部反馈 | `/planning/go2_execution_frozen` | `std_msgs/msg/Bool` | 闭环控制器暂停执行时通知规划器同步冻结轨迹计时 |
| 输出 | `/cmd_vel` | `geometry_msgs/msg/Twist` | 闭环控制器输出的机体 X/Y 速度和偏航角速度 |
| 输出 | `/planning/bspline` | `local_planner/msg/Bspline` | 正常轨迹或紧急停车轨迹 |
| 输出 | `/local_path` | `nav_msgs/msg/Path` | 同一位置 B 样条的几何采样；默认 10 Hz 心跳，停车时发布空路径 |
| 输出 | `/planning/data_display` | `local_planner/msg/DataDisp` | 沿用原诊断消息 |
| 输出 | `/grid_map/*` | 点云、Marker、Odometry | 局部地图、边界与传感器外参可视化 |
| 输出 | `/optimal_list`、`/global_list` 等 | `visualization_msgs/msg/Marker` | 轨迹可视化 |

未收到里程计时忽略路径；无有效路径时等待。路径至少应包含两个不同的有限坐标点，`header.frame_id` 不得为空，各 pose 的非空 frame 必须一致。沿用原算法对路径进行三维降采样并保留末点，Z 加上 `grid_map.body_height`（默认 0，避免重复叠加全局规划器的路径高度偏移）。有效新路径会替换参考路线并触发规划或重规划。无效路径被忽略，已有有效参考路线继续保留。

机身里程计、传感器位姿和世界点云应处于配置的规划坐标系 `grid_map.frame_id`（默认 `world`）中。外部路径可在 `map` 等其他坐标系中，需提供到规划坐标系的最新 TF；保留原参考路径动态 TF 转换、短时丢失宽限、超时停车、跳变重规划以及重规划失败停车行为。相关阈值为 `fsm.reference_tf_*`。路径订阅默认可靠、volatile、深度 1，外部发布器应在里程计就绪后发布，过早发送的路径需要重新发布。

`Bspline` 和 `DataDisp` 字段保持原定义，但 ROS 消息类型已属于本包。直接执行样条的节点应改用 `local_planner/msg/Bspline`，通过 `/local_path` 跟踪的节点只需标准 `nav_msgs/msg/Path`。`Bspline` 不包含坐标系字段，其控制点位于规划坐标系。

## 默认使用 B 样条闭环控制

闭环控制器由原 `scan_planner` 迁移，使用本包 `local_planner/msg/Bspline`，不经过几何路径跟踪器：

```text
/global_path → local_planner_node → /planning/bspline → closed_loop_controller → /cmd_vel
                    ↑                                      │
                    └──── /planning/go2_execution_frozen ──┘
                    └──── /local_path（辅助显示/可选输出）
```

`run.launch.py` 默认 `enable_controller:=true`，一条启动命令即可运行完整链路。控制参数在 `config/controllers.yaml`，可通过 `controller_params_file:=/绝对路径/controllers.yaml` 替换，节点键为 `closed_loop_controller`。启动时控制器的 `frame_id` 和 `use_sim_time` 与规划器同步，机身里程计及全局路径也使用相同重映射。

控制器以 100 Hz 执行原有控制逻辑：

- 样条一阶导数提供世界坐标系速度前馈，叠加 `kp_pos` 位置误差反馈，再旋转到机体坐标系。
- 速度限值由 `controllers.yaml` 的 `max_vx`（m/s）、`max_vy`（m/s）和 `max_vyaw`（rad/s）设置，支持前进、后退、横移和偏航。偏航角速度另有 1.0 rad/s 的代码上限；Z 方向不输出控制量。
- 根据 `time_forward` 前视方向估计偏航，误差超过 `heading_error_threshold` 时先原地转向，同时发布冻结反馈，使控制器和规划器暂停各自的轨迹时间，朝向收敛后恢复。
- 局部轨迹执行结束后按原逻辑收敛到局部终点；到达全局终点后继续对齐全局路径末端四元数给定的朝向，达到位置/偏航容差后保持零速度。
- 收到所有控制点相同的停车样条时立即清零速度，不把停车点作为位置目标追赶。未收到有效轨迹、机身里程计失效/坐标系不匹配或超时（默认 `odom_timeout: 0.3` s）时同样停车。
- 只接受本包生成的三次样条；在恢复样条前检查点数、节点跨度、时间戳和有限性。非法或到达时已过期的轨迹会被拒绝并停车。

保留了原控制器在局部末端的位置保持行为；控制器不使用 `/local_path` 的空路径作为停车输入。规划器通过停车 B 样条通知它停止，若规划进程异常中断，仍需依赖系统层监控及底盘的指令超时保护。当前修改已进行接口联动验证，实机速度与控制增益仍需现场验证。

只规划、不输出速度时使用：

```bash
ros2 launch local_planner run.launch.py enable_controller:=false
```

## 保留的 /local_path 接口

当前闭环链路不读取 `/local_path`。将来如切换回 `path_follow`，应先设置 `enable_controller:=false`，然后单独启动 `path_follow`，保持同一速度话题只有一个控制器。它原有的局部路径输入模式仍可使用，包内不再注册依赖它的联动测试。

接口约定与转换细节：

- 转换调用实际优化位置样条的 De Boor 求值，保留完整起终点和 Z 坐标；不把控制点折线当作路径。默认 `local_path.sample_dt: 0.05` 秒，按速度不同对应不同空间间距，最多允许 10000 个采样区间。
- 每个 Pose 使用样条水平切线生成单位偏航四元数，端点零速度时用相邻点估计方向；`path_follow` 当前主要使用位置点进行几何跟踪。
- Header 和每个 Pose 的坐标系均为 `grid_map.frame_id`，默认 `world`。若修改坐标系或里程计话题，需同步修改 `path_follow` 的 `local_frame`、`odom_frame`、`odom_topic`，或提供它需要的 TF。
- 默认 `local_path.publish_rate: 10.0` Hz，可靠、volatile、深度 1。新样条生成后立即发布，并在其有效期内刷新路径时间戳；与跟踪器默认 `local_path_timeout: 0.50` 秒匹配。若降低心跳频率，应同步增大跟踪器的超时阈值并留出通信裕量。
- 急停、等待新参考路径、执行冻结或当前轨迹过期时输出空 Path，让 `path_follow` 清空路径并发布零速度；轨迹恢复需有有效规划。冻结解除可恢复尚未过期的当前轨迹。规划进程中断时由跟踪器的路径超时检测负责停车。

将来使用 Path 跟踪时实现的是**几何路径跟踪**：`Path` 不表达样条的速度、加速度与执行时序，`path_follow` 会使用自己的 `linear_vel`、速度上限和加速度上限，仅输出机体前进速度 `linear.x` 与偏航速度 `angular.z`，不执行三维高度运动、横移或倒车。规划器仍按照原样条时间推进状态机，因此必须结合实测跟踪速度、重规划阈值、前视距离和跟踪误差联合调参。若要求严格执行样条时间曲线或保证原样条的动力学指标，应使用带时间前馈/反馈的样条跟踪器；仅转换为 Path 不能提供这些保证。实机闭环避障与速度匹配尚需验证。

## 中文代码导读

- `plan_env`：滑动地图、射线占据融合与双圆柱机身碰撞模型；`path_searching`：A* 与参考高度。
- `bspline_opt`：B 样条求值、优化代价、时间调整；通用 L-BFGS 文件补充中文入口说明并保留原英文算法说明。
- `plan_manage`：参考路径输入、TF 监控、规划/执行/停车状态机，以及迁移的 `closed_loop_controller.cpp`；`traj_utils/bspline_path.h`：样条到 Path 的采样。
- `config/planner.yaml` 中每个参数都附中文含义和单位；launch 启动参数、消息字段及主要函数也有中文注释。

## 重规划资源限制

局部初值与密集 QR 拟合最多允许 512 个采样点，时间采样间隔不得小于 `1e-4` 秒，多项式初始化最多细分 16 次。复用旧轨迹前会检查其有效期、有限性和剩余区间；过期、零弧长或超限时改用一次有界多项式初始化，仍不可用则让本轮规划受控失败。时间重分配采用固定数量的采样，不按浮点误差项扩张点列。超过限制会输出 `[DIAG][INIT]` 日志，动力学限值和原有失败停车策略保持有效。

若局部重规划内部仍抛出 `std::bad_alloc` 或容器长度异常，会记录 `[DIAG][RESOURCE]` 并请求停车、清空局部路径及等待新的全局参考路径。该处理不能保证在整机内存完全耗尽、进程被系统 OOM 杀死或其他回调异常时继续运行；现场仍应核查内存和内核日志。参数过小导致受控拒绝时应修正参数，而不是调大动力学容差。

## 验证

```bash
colcon test --packages-select local_planner --event-handlers console_direct+
colcon test-result --verbose
```

包含 B 样条、参考路径、参考高度、坐标变换及 Path 采样单元测试；接口测试检查轨迹更新、TF 超时撤销路径，并测试闭环速度输出、偏航冻结/恢复、里程计超时、停车样条和终点朝向对齐。默认启动文件也有规划器与控制器联动验证。测试使用独立 ROS 域及测试速度话题，不依赖 `path_follow`，不连接实机速度接口。

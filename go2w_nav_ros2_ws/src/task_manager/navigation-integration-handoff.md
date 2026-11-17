# 导航话题对接交接单（2026-11-05）

本次只修改 task_manager，新增 Super-LIO 定位初始化门槛及成功心跳检查，保留按 `local_planner_status_diagram.html` 描述的 JSON 状态及
pause/resume 控制适配现有局部规划器。global_planner、scan_planner 和其他功能包保持不变；Super-LIO 也保持不变。
不接入 action，不增加目标执行或控制确认超时。
历史 `mission-protocol-handoff.md` 保留原文，当前行为以本交接单和 README 为准。

## 定位初始化与异常处理

接口参考 `../Super-LIO-ros2/readme.md`。新增定位订阅参数 localization_status_topic，
默认 /localization/status，String 内为 JSON `{state,msgs}`，可靠、transient_local、depth 1。
新增 localization_status_timeout_sec=3.0，配置和 launch 均支持覆盖，必须为有限正数。

- 启动立即上报 uninitialized_location；ready 表示传感器就绪，仍不能执行任务。
- 两次连续 succeed、间隔不超过 3 秒才解锁为 idle；第一条缓存成功不能单独放行。
- 正常定位后，ready/error/initialized_failed/failed 均关闭门槛；成功心跳超过 3 秒也关闭。
- 仅成功状态检查心跳，其他事件状态没有周期发布要求；使用单调时钟，非法消息不刷新时间。
- 新 start 在未就绪时拒绝为 localization_not_ready，不排队；解锁后必须重新提交。
- 若有活动任务则请求局部 pause，发布一次 failed 后立即回 uninitialized_location。
  恢复两次成功确认不会自动恢复旧任务，也不会释放尚未确认停止的旧执行占用。
- 定位失败优先于等待中的暂停/取消。原 TF 和 estop 中止、局部恢复、新路径门控保持。
- task_manager 的锁存 error 优先保留，定位消息不能清除任务/地图故障。
  上游 failed 要求重启定位程序，但不额外锁存为 task_manager 自身 fault。

/mission/status 新增 localization_status，为最近有效定位对象或 null，保留全部未知原因。
超时时原文字段可仍为 succeed，通过 message 区分成功心跳超时，不伪造来源状态。
未初始化状态按 status_rate_hz（默认 2 Hz）心跳；规划反馈可更新诊断但不能解除定位门槛。

前端需支持新 state=uninitialized_location，显示等待状态及定位原因；
不能将 ready 当作可启动任务，不能将定位恢复当作旧任务恢复或车辆停车证明。
该实现不调用重定位服务，也不提供初值；初始化仍走 Super-LIO 已有流程。
定位 JSON 没有时间戳/序号/进程 ID，两次成功只验证接收侧持续更新，不提供严格跨进程关联。

## 接线和迁移

```text
/mission/command (JSON: start/pause/resume/stop)
        ↓
 task_manager ── /goal_3d (PoseStamped) ── global_planner
        │                                       │
        │ /mission/control (String: pause/resume)│ /global_path
        ↓                                       ↓
 local_planner ←──────────────────────────────────┘
        │ /local_planner/status (String containing JSON: state,msgs)
        ↓
 task_manager ── /mission/status (JSON)
        ↑
 /localization/status、/global_planner/status、/loc/robo/odom、TF、/estop
```

- 前端四种 command 与下游两种控制不同：start 只发目标，stop 转为 pause。
- pause/stop 先进入 canceling，保留当前 run/目标，step_state=null。
  局部 stopped 确认后，分别进入 paused 或 canceled 一次后 idle。
  没有 stopped 就持续等待，不释放占用，不接受其他 run，无确认超时。
- 到达与控制交错时仍记录一次 reached；暂停仍等 stopped，取消仍以 canceled 结束。
- resume 同一未到达目标时不重复发目标；已有该目标的匹配路径可发送 resume，
  尚未收到路径则继续等待。发送 resume 后等局部 running 才进入导航。
- dwell 暂停恢复重计完整 dwell，不发送 resume 恢复旧轨迹；下一点需要导航时才恢复局部。
- 取消完成回 idle 后保留局部 stopped；新 start 必须先发目标、收到匹配的新路径，再发 resume。
- 全局 path_found 不能替代路径匹配或局部 running 恢复确认。
- 没有待确认请求时，执行期间收到 stopped 作为外部暂停。

## 局部 JSON 和任务故障

只接受 `{"state":"...","msgs":["..."]}`：state 为 idle/running/goal_reached/error/stopped，
msgs 为字符串数组。原因不做枚举过滤，多条及未知原因完整保留。
非法消息只记日志，不修改任务或最新有效诊断。
局部订阅显式 reliable、volatile、depth 10，兼容局部 transient_local 发布者，
避免重放节点启动前缓存的 goal_reached。

`/mission/status.local_planner_status` 为解析对象或 null，状态/原因变化立即发布并随心跳携带。
局部 error 为可恢复故障：保留 run、目标及原执行阶段，对外报告 error，暂停列表推进，
不生成单点 failed，不设置锁存 fault。running/goal_reached 可恢复原任务。
人工暂停与取消不因恢复反馈解除；paused/canceling 优先显示，故障原因仍在诊断字段。
任务本身错误与 map_load_failed 仍锁存，局部 running/全局 ready 不能清除。

独立 TF 超时和 `/estop` 仍执行安全中止，发布一次 failed 后 idle，并请求 pause。
旧执行占用在 stopped 或旧请求结束后释放；estop 未解除仍拒绝新任务。
局部 msgs 中 emergency_safety 只是可恢复局部原因，不替代独立急停输入。

全局 ready/planning/path_found/goal_reached/planning_failed/map_load_failed 映射保留，
错误原文进入 global_planner_status、message、单点 reason。
pause 待确认期间出现 planning_failed，等 stopped 后 failed；stop 意图优先，仍 canceled。
全局 volatile 请求订阅忽略缓存旧请求结果；另一路 durable 订阅只消费 ready/map_load_failed。

succeeded/canceled/failed 单次完整快照后，定位就绪时回 idle，否则回 uninitialized_location；前端需保存终态事件。
定位/局部诊断和全局原文跨空闲及等待保留，任务字段清空，idle 1 Hz，其他状态默认 2 Hz。

## 新路径恢复的边界

匹配路径要求时间戳不早于当前目标发布时间、后续时间戳递增、路径有限且非空、帧一致，
末点与转换后的当前目标误差不超过 0.05 米（包含 global_path_z_offset，默认 0.5 米）。
每个新目标重置路径匹配标志；旧时间戳、错误终点、空路径和单独 path_found 都不能触发 resume。
匹配路径不依赖 odom 是否已收到，距离不可用不会阻止恢复。

**已有接口无法严格证明局部已缓存本节点刚收到的新路径。** 两个订阅者对路径和控制的
跨话题接收顺序没有确认机制。本次按已确认方案采用“匹配新路径后再 resume”，
不引入路径接收确认接口；必须在实机验证局部“新路径优先”，不能把单元/话题测试当作该保证。

状态没有目标 ID，任务期间必须由 task_manager 独占目标控制；必须避免下一点下发后
再发送上一点到达事件。重复到达在已完成点/dwell 内会被去重，跨目标迟到无法严格识别。
HTML 提到 goal_reached 锁存及状态心跳，联调需确认新目标会及时清除旧到达状态，
保证任务侧不收到可误归属到下一目标的旧到达事件。volatile 只屏蔽历史缓存，不屏蔽实时重复事件。

stopped 是局部软件状态确认，不代表本节点测量过底盘零速。
任务节点不发布 cmd_vel、不使用 motion_enabled；进程退出、失联、控制链中断时停车仍由
导航/底盘和独立看门狗负责。yaw 仅传入目标姿态，最终朝向仍需实机导航链验证。

## 本次模拟验收

本地测试：178 项通过；隔离 ROS/DDS 测试：22 项通过。
全部启用时共 200 项；默认运行跳过 22 项显式 DDS 测试。
测试采用独立 ROS_DOMAIN_ID=178、随机话题、模拟 TF、JSON 局部规划发布者，无实车目标发送。

| 范围 | 模拟覆盖 |
|---|---|
| JSON 输入 | 五种状态、多原因、未知原因、非法 JSON、缺失字段、错误类型 |
| 故障恢复 | error→running/goal_reached、无 failed 结果及 fault、暂停不被恢复反馈解除 |
| 控制确认 | pause→canceling→stopped→paused，stop→pause→stopped→canceled 一次→idle |
| 竞态 | 全局/局部到达与暂停/取消交错、重复命令、dwell 重计、重复完成去重 |
| 新路径 | 取消后新 run、旧路径/错误终点不恢复、path_found 不恢复、匹配路径后 resume、running 确认 |
| QoS | 缓存局部到达不重放、缓存全局请求失败不误伤、缓存地图故障仍可发现 |
| 定位门槛 | 启动等待、缓存 ready/succeed、两次成功确认、提前 start 拒绝、异常原因、三秒断流、TF 仍新鲜时中止、重启后新任务 |
| 保留能力 | 三点逐点下发、TF/estop、距离计算、幂等、拒绝、单次终态、空闲心跳 |

对原协议第 10 节的对应记录：

| 原用例 | 当前模拟结论及差异 |
|---|---|
| 1～4：idle/三点/重试/并发拒绝 | 定位确认后通过；定位确认前改为 uninitialized_location |
| 5：pause | 状态及 stopped 软件确认已通过，实车停车待验 |
| 6：resume | 已按最新约定验证：保留当前目标、等待路径和 running；到达点重计 dwell |
| 7：stop | stopped 后 canceled 单次、idle 已通过 |
| 8、15：目标超时/skip | 已按用户要求移除，不执行旧超时/skip 验收；全局失败仍 abort |
| 9、10：TF/estop | 模拟中止及新任务限制已通过，实车停车待验 |
| 11、12：robot/schema 拒绝 | 已通过 |
| 13：页面刷新 | 完整心跳已测试；浏览器/后端恢复未纳入本包测试 |
| 14、16：dwell/逐点结果 | 已通过，重复到达不重复记录 |
| 17：浏览器关闭 | 执行器不依赖浏览器；页面回写与后端清理未纳入本包测试 |

代码检查 flake8（最大行长 100）、包级构建及 colcon pytest 检查通过。
本机 setuptools 的 tests_require 警告不影响构建，colcon 显式指定 pytest 避免零测试。
对修改前后 src 下 task_manager 以外文件逐一 SHA-256 核对一致；两规划器保持不变。

## 待实机联调

1. 验证真实 stopped 确认对应停止执行及停车，暂停/取消等待期间不会继续推进列表。
2. 验证新路径优先：取消后重新启动、暂停后切下一点时，resume 绝不恢复旧缓存轨迹。
3. 验证三点、重复坐标/起点即终点、到达朝向、dwell 暂停继续、外部暂停。
4. 验证局部 error 多原因及自动恢复；人工暂停期间恢复反馈不自行开动。
5. 验证独立急停、定位丢失、控制进程失联/退出时底盘停车和看门狗。
6. 核对 map→robot_frame、global_path_z_offset、目标独占发布、到达锁存/实时重复事件。
7. 验证定位 error/initialized_failed/failed 和成功心跳消失时实际停车；定位程序重启、重新初始化后只能重新提交任务，旧 run 不自动执行。
8. 核对 bridge 白名单：前端 mission/command 与 mission/status；mission/control 为局部执行接口。

以上实机验收本阶段未宣称通过。

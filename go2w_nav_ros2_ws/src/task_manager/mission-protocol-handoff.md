# 任务执行协议交接文档（v1）

> 面向：导航组（`mission_executor` 实现方）
> 来源：`docs/implementation-plan-v2.md` §5.2 / §5.5 / §5.6
> 状态：待导航组确认；确认后本文件是唯一契约，计划文档只保留摘要。

---

## 1. 范围

我们要导航侧提供的能力：**接收一个已经展开好的点列表，按顺序跟随，并把运行状态持续上报**。

不需要导航侧做的事：

- 不需要实现 `once / loop / pingpong` 的循环语义 —— **模式已由后端展开成 steps**；
- 不需要知道“激活地图”是哪一个 —— ROS 侧没有这个概念，地图一致性由后端/前端在启动前校验；
- 不需要读写 SQLite 或调用网页后端 —— 数据落库由前端通过 REST 完成。

需要导航侧做的事：

- 两个话题：`/mission/command`（收）、`/mission/status`（发）；
- 状态机、暂停/继续/停止、dwell、失败重试；
- 三条自动中止：`/estop`、定位（TF）丢失、连续 goal timeout；
- 即使空闲，也按 1 Hz 发心跳（前端靠它判断“执行器在线”）。

## 2. 数据流

```
浏览器（Vue）
  │  1. POST /api/missions/:id/runs        → 后端展开 steps，返回 { run_id, plan }
  │  2. publish /mission/command (JSON)    → 本节点
  │  3. 订阅 /mission/status                ← 本节点（运行中 2 Hz / idle 1 Hz）
  │  4. PUT /api/robots/:id/mission-state   → 后端快照（刷新/多标签页恢复用）
  ▼
Rust 后端（SQLite）        ROS 桥（foxglove / rosbridge）        mission_executor
```

- 浏览器是唯一的指令发起方；后端不直接连 ROS。
- 状态以本节点上报为准；后端快照只是镜像。
- 浏览器可能同时有多个（多标签页），它们都订阅同一个状态话题。

## 3. 话题约定

| 名称 | 类型 | 方向 | QoS | 频率 |
|------|------|------|-----|------|
| `/mission/command` | `std_msgs/String`（JSON） | 浏览器 → 节点 | reliable, depth 10 | 事件驱动 |
| `/mission/status` | `std_msgs/String`（JSON） | 节点 → 浏览器 | reliable, depth 10 | 运行中 2 Hz；idle 1 Hz；状态变化立即发 |

话题名可由参数覆盖（`command_topic` / `status_topic`），默认值见上表。
节点启动后必须**立刻**开始发 idle 心跳，不要只在收到命令后才发。

### 3.1 封装方式（`std_msgs/msg/String` + JSON 文本）

两条话题的消息类型都是 `std_msgs/msg/String`，JSON 文本放在 `data` 字段里：

```
message: std_msgs/msg/String
  data: "{\"schema_version\":1,\"robot_id\":\"a2\",...}"   # 整个 JSON 作为一个字符串
```

rosbridge 发布（注意 `msg` 里只有 `data`，不要把 JSON 对象直接当消息体）：

```json
{ "op": "publish", "topic": "/mission/command", "type": "std_msgs/msg/String",
  "msg": { "data": "{\"schema_version\":1,\"robot_id\":\"a2\",\"command\":\"pause\",...}" } }
```

读取方式：

- rosbridge 订阅（`cbor-raw`）：原始 CDR → 现有 `stringMsg.ts` 解码 → `JSON.parse(msg.data)`；
- foxglove：用内置 `string data` schema（`src/lib/ros/controlSchemas.ts`）advertise / CDR 编解码，
  与现有建图指令（`/mapping/command`、`/voxel_slam/command`）完全一致；
- 命令行调试：`ros2 topic echo /mission/status` 直接看到转义后的 JSON 文本。

为什么不用自定义 msg：

- Foxy 与 Humble 要各编一次接口包，改字段就得两台狗重新构建；
- `ros2 topic echo` 直接可读，现场排障成本低；
- 用 `schema_version` 做协议演进，比 msg 定义更容易灰度。

代价：没有静态类型与字段级校验。因此节点侧必须按 §6.3 严格校验并回
`command.accepted=false` + `reason`，前端也必须把解析失败当作“执行器无响应/协议不兼容”处理，
而不是猜测字段。
心跳带 `target` + `last_step_result` 时单条约 0.5–1 KB，2 Hz 下约 2 KB/s，可以忽略。

升级路径：字段稳定后可新增 `MissionCommand.msg` / `MissionStatus.msg`，话题名与字段语义不变，
前端换成 `MessageWriter` 即可（`src/lib/ros/` 已有 CDR 编解码基础设施）。

## 4. 命令协议

公共字段：

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `schema_version` | int | 是 | 当前固定 `1` |
| `robot_id` | string | 是 | 与部署 identity / `robots.id` 一致；不一致直接忽略并回一条 rejected 状态 |
| `run_id` | string | 是 | 后端 `mission_runs.id`，一次运行唯一 |
| `request_id` | string | 是 | 前端一次启动尝试的幂等键，重试时不变 |
| `command` | string | 是 | `start` / `pause` / `resume` / `stop` |

### 4.1 `start`

```json
{
  "schema_version": 1,
  "robot_id": "a2",
  "run_id": "01J9ZP...",
  "request_id": "01J9ZK...",
  "frame_id": "map",
  "command": "start",
  "mission": {
    "id": "01J9ZM...",
    "name": "巡检 A",
    "map_id": "01J9ZQ...",
    "mode": "once",
    "loop_count": 1,
    "steps_total": 3,
    "steps": [
      { "step_index": 0, "lap": 1, "point_index": 0, "point_total": 3,
        "waypoint_id": "01J9ZR...", "name": "门口",
        "x": 1.24, "y": 3.41, "z": 0, "yaw": 1.57, "dwell_sec": 2.0 }
    ]
  }
}
```

`steps[]` 字段：

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `step_index` | int | 是 | 从 0 开始，必须单调递增且与数组下标一致 |
| `lap` | int | 是 | 第几圈，仅用于展示；`once` 恒为 1 |
| `point_index` | int | 是 | 该 step 对应原点位列表中的序号（从 0 开始） |
| `point_total` | int | 是 | 原点位总数（不随循环放大） |
| `waypoint_id` | string/null | 否 | 原点位 id；点位已删除时仍保留，仅用于展示 |
| `name` | string | 是 | 显示名，可能为空字符串 |
| `x` / `y` / `z` | float | 是 | 米，`frame_id` 坐标系；必须是有限值 |
| `yaw` | float | 是 | 弧度，`[-π, π]`；**参与导航**，到位后转向该朝向 |
| `dwell_sec` | float | 是 | 到点后停留时间，`0–600` |

`steps` 上限 1000 条；超出视为非法命令（回 `failed`，见 §6.3）。

> `mode` / `loop_count` 只是元信息，节点**不要**据此再做循环。`steps_total` 必须等于
> `steps.length`，不一致时以 `steps.length` 为准并记 warning。

### 4.2 `pause` / `resume` / `stop`

```json
{ "schema_version": 1, "robot_id": "a2", "run_id": "01J9ZP...",
  "request_id": "01J9ZX...", "command": "pause" }
```

- `mission_id` 可选，仅用于日志。
- 只接受与当前活动 `run_id` 匹配的 pause/resume/stop；不匹配时忽略（见 §6.3）。

## 5. 状态协议

### 5.1 示例

```json
{
  "schema_version": 1,
  "robot_id": "a2",
  "run_id": "01J9ZP...",
  "request_id": "01J9ZK...",
  "mission_id": "01J9ZM...",
  "state": "running",
  "step_state": "navigating",
  "step_index": 2,
  "step_total": 6,
  "current_index": 1,
  "total": 3,
  "lap": 1,
  "current_waypoint_id": "01J9ZS...",
  "target": {
    "step_index": 2, "point_index": 1, "waypoint_id": "01J9ZS...",
    "name": "仓库门", "x": 4.2, "y": -1.1, "z": 0, "yaw": 3.14
  },
  "last_step_result": {
    "step_index": 1, "point_index": 0, "outcome": "reached",
    "reason": "", "finished_at": { "sec": 1759999980, "nanosec": 0 }
  },
  "remaining_distance_m": 8.4,
  "message": "navigating to 仓库门",
  "stamp": { "sec": 1760000000, "nanosec": 0 }
}
```

idle 心跳（没有运行时）：

```json
{
  "schema_version": 1,
  "robot_id": "a2",
  "run_id": null,
  "mission_id": null,
  "state": "idle",
  "step_state": null,
  "step_index": null,
  "step_total": null,
  "current_index": null,
  "total": null,
  "lap": null,
  "current_waypoint_id": null,
  "target": null,
  "last_step_result": null,
  "remaining_distance_m": null,
  "message": "",
  "stamp": { "sec": 1760000000, "nanosec": 0 }
}
```

### 5.2 字段语义

| 字段 | 说明 |
|------|------|
| `run_id` | 当前运行的 id；idle 时为 `null`。**命令与状态必须双向携带** |
| `request_id` | 最近一次被接受的 `start` 的 `request_id` |
| `state` | 见 §6.1 |
| `step_index` / `step_total` | 展开后步序列的当前步 / 总步数（含循环） |
| `current_index` / `total` | 直接回填当前 step 的 `point_index` / `point_total`，用于显示“第 2/3 个点” |
| `lap` | 当前第几圈；`once` 恒为 1 |
| `step_state` | `navigating`（正在前往 target）/ `dwelling`（已到达、停留中）/ `null`（idle、暂停、取消中、终态） |
| `target` | 当前目标点快照；`starting/running/paused/canceling` 时非空，`idle` 与终态为 `null`。前端用它做 3D 高亮与“当前目标”卡片，不反查 `plan_json` |
| `last_step_result` | 最近一个完成的 step；没有结果时为 `null`。每完成一个 step 更新一次，之后保持不变（心跳重复携带），客户端按 `(run_id, step_index)` 去重 |
| `remaining_distance_m` | 到**当前 step 目标点**的剩余距离（不是全程剩余）；未知时为 `null` |
| `message` | 人可读的当前动作/原因；失败时必须给出原因 |
| `stamp` | 节点时刻；前端只用于显示与排序，不与本地时钟比较 |

> 状态里的 `current_index` / `total` 直接来自当前 step 的 `point_index` / `point_total`，
> 不要自己反推。

### 5.3 单点结果（`last_step_result`）

节点在**每个 step 真正结束时**更新一次 `last_step_result`，随后所有心跳重复携带该值，
直到出现下一条结果。语义：

| 字段 | 说明 |
|------|------|
| `step_index` | 完成的 step 序号，必须与 plan 中的一致 |
| `point_index` | 该 step 的 `point_index`（原点位序号） |
| `outcome` | `reached`（已到达目标点）/ `skipped`（失败策略跳过）/ `failed`（任务因此失败） |
| `reason` | 失败/跳过的原因；`reached` 时为空字符串 |
| `finished_at` | 该 step 结束的节点时刻 |

规则：

- `reached` 在**到达目标点时**判定，`dwell` 被 pause/stop 打断不影响这条结果；
- `skip` 策略下，重试耗尽的点会发一条 `outcome=skipped`，然后继续下一个 step；
- `abort` 策略下，失败的点发 `outcome=failed`，随后状态进入 `failed`；
- 一条结果只代表一个 step；同一个 step 可能被重试多次，但**只发一条**最终结果；
- 客户端（前端）按 `(run_id, step_index)` 去重累积；中途加入的客户端会通过
  `GET /api/runs/:id` 的 `results[]` 补齐历史；
- 节点重启会丢失内存中的结果（不落盘），后端保留前端已经回写过的那部分。

### 5.4 命令回执（可选字段）

命令被拒绝或忽略时，状态里追加：

```json
"command": { "request_id": "01J9ZK...", "command": "start", "accepted": false, "reason": "run_active" }
```

`reason` 取值：`run_active` / `run_id_mismatch` / `robot_mismatch` / `invalid` /
`unsupported_version` / `estop_active`。
未出现该字段表示没有需要回执的拒绝。**不要**因为一条被拒绝的命令就改变当前状态。

## 6. 状态机与行为

### 6.1 状态枚举

`uninitialized location | idle | starting | running | canceling | paused | succeeded | failed | canceled | error`

- `idle`：没有活动 run（心跳里 `run_id = null`）。
- `starting`：已接受 `start`，正在下发第一个 goal。
- `canceling`：已收到 pause/stop，正在等待 cancel 完成（过渡态）。
- `paused`：cancel 已完成，等待 `resume`。
- `succeeded` / `failed` / `canceled`：终态，保持到下一次 `start`；终态后继续发心跳（run_id 仍在），
  以便前端刷新后能显示“上一次结束”的结果。下一次 `start` 到达时切到新的 `run_id`。
- `error`：节点自身故障（如参数非法、无法连接 action server）；前端提示“执行器异常”。

后端映射：节点 `error` → 后端 `failed`；节点 `idle` 且后端有非终态 run → 后端 `unknown`。

### 6.2 迁移

```
IDLE --start--> STARTING --ok--> RUNNING
                                  ├--pause--> CANCELING --ok--> PAUSED --resume--> STARTING
                                  ├--stop---> CANCELING --ok--> CANCELED
                                  ├--完成----> SUCCEEDED
                                  └--失败/中止--> FAILED
```

- **一次只能有一个活动 goal**。在 cancel 完成回调到达之前，不要下发下一个 goal。
- `resume` 重新下发当前 `step_index` 的 goal（重新规划），并**重新计时 dwell**（不保留剩余）。
- `stop` 后索引不归零；节点只上报 `canceled`，索引由前端/后端重置。

### 6.3 幂等与拒绝

| 情况 | 行为 |
|------|------|
| 同一 `request_id` + 同一 `run_id` 的重复 `start`（前端重试） | 不重启任务，立即重发当前状态 |
| 运行中收到不同 `run_id` 的 `start` | 不改变状态，回一条 `command.accepted=false, reason=run_active` |
| pause/resume/stop 的 `run_id` 与当前不符 | 忽略并回 `reason=run_id_mismatch` |
| `robot_id` 与本节点参数不符 | 忽略并回 `reason=robot_mismatch` |
| `schema_version` 不是 1 | 忽略并回 `reason=unsupported_version`，不要猜测字段 |
| `steps` 为空 / 超过 1000 / 坐标非有限值 / `yaw` 越界 | 不进入 running，状态不变，回 `reason=invalid`；前端据此把该 run 标为 `failed` |
| JSON 解析失败 | 只打日志，不回执（无法获知 `request_id`），不崩溃 |

### 6.4 dwell

- 到达当前点后先发一条 `last_step_result.outcome=reached`，再按 `dwell_sec` 停留；
  停留期间 `state` 仍为 `running`，`step_state=dwelling`，`target` 仍指向当前点，
  `remaining_distance_m` 为 0。
- dwell 结束、切到下一个 step 时，`target` / `step_index` / `step_state` 一起更新。
- dwell 期间收到 `pause`：cancel 当前 goal（此时已到达，cancel 可能是空操作），进入 `canceling → paused`。
- dwell 期间收到 `stop`：同样进入 `canceling → canceled`。

### 6.5 失败策略

`failure_policy`（节点参数，不是每条任务单独配置）：

- `abort`：该点重试 `waypoint_retries` 次仍失败 → `failed`，任务结束；
- `skip`：重试后仍失败 → 跳过该点继续下一个，最终 `succeeded` 但 `message` 里列出被跳过的点。

`goal_timeout_sec` 到达后视为一次失败，进入重试。

## 7. 安全与自动中止

| 条件 | 行为 |
|------|------|
| `/estop` 收到 `true` | 立即取消当前 goal，`state=failed`，`message="estop"`；在 `/estop=false` 之前不接受新的 `start`（回 `reason=estop_active`） |
| `map → robot_frame` TF 超过 `tf_timeout_sec`（默认 2 s） | 取消 goal，`state=failed`，`message="localization lost"` |
| 连续 `consecutive_timeout_limit`（默认 2）次 goal timeout | 按 `failure_policy` 处理 |
| action server 不可用 | `state=error`，`message` 说明缺少哪个 action |

补充：

- 节点不发布 `/cmd_vel`；停车由导航栈在 cancel 时保证。若导航栈不保证，
  需要 `teleop_guard` 的看门狗兜底（P5）。
- 节点不使用 `/motion_enabled`（那是网页跟随开关，与任务执行无关）。
- 低电量、底盘故障不在 v1 范围（P7 再补）。

## 8. 时序

**正常任务**

```
浏览器/后端                    桥                    mission_executor
   │ POST runs → {run_id, plan}                       │
   │──── start(run_id, steps) ───────────────────────>│
   │<─── status: starting ───────────────────────────│  立即
   │<─── status: running, step 0 ────────────────────│  2 Hz
   │<─── status: running, step 0, distance 3.2 ──────│
   │<─── status: running, step 1 ... ────────────────│
   │<─── status: succeeded ──────────────────────────│  变化即发
   │ PATCH /api/runs/:id (succeeded)                  │
```

**暂停 / 继续**

```
   │──── pause(run_id) ──────────────────────────────>│
   │<─── status: canceling ──────────────────────────│
   │<─── status: paused ─────────────────────────────│  cancel 完成
   │──── resume(run_id) ─────────────────────────────>│
   │<─── status: starting (同 step) ─────────────────│
   │<─── status: running ────────────────────────────│
```

**刷新页面**

```
浏览器启动
  → GET /api/robots/:id/mission-state   # 先渲染“最后已知 2/6”
  → connect bridge → subscribe /mission/status
  → 收到同 run_id 的心跳 → 校正为实时状态
```

## 9. 导航侧实现清单

1. 参数：`robot_id`、`command_topic`、`status_topic`、`navigate_action`、`frame_id`、
   `status_rate_hz`、`goal_timeout_sec`、`consecutive_timeout_limit`、`tf_timeout_sec`、
   `waypoint_retries`、`failure_policy`、`dwell_default_sec`。
2. 订阅命令话题、发布状态话题（reliable）。
3. 启动即发 idle 心跳（1 Hz），不要等第一条命令。
4. `start` 校验（§6.3）→ 发 `starting` → 逐 step 调用 `NavigateToPose`。
5. goal 用 `frame_id` 与 `yaw` 构造 `PoseStamped`（四元数由 yaw 计算）。
6. 反馈 `distance_remaining` 映射到 `remaining_distance_m`。
7. 每个 step 必须上报 `target`（当前目标坐标/名称）与 `step_state`（`navigating` / `dwelling`）；
   进入 dwell 时切到 `dwelling`，不要提前把 `target` 指向下一个点。
8. 每个 step 结束时上报一条 `last_step_result`（`reached` / `skipped` / `failed` + `reason`），
   随后心跳重复携带，直到下一条结果。
9. pause/stop 走 cancel 流程，`canceling` 过渡态必须如实上报。
10. 三条自动中止（§7）。
11. 状态变化立即发 + 运行中 2 Hz 心跳。
12. 单测覆盖：状态机、幂等、拒绝回执、dwell、skip/abort、TF 超时、estop、
    `target`/`step_state` 切换、`last_step_result` 只发一次。

## 10. 联调与验收

前置：identity 已含 `missionCommand` / `missionStatus`，且这两个话题已进入 bridge 白名单
（`launch/bridge_filter` 会自动生成；rosbridge 看 `topics_glob`，foxglove 看 `client_topic_whitelist`）。

| # | 用例 | 期望 |
|---|------|------|
| 1 | 节点空转 | 1 Hz `idle` 心跳，`run_id=null` |
| 2 | start 3 点 `once` | `starting → running` 逐点推进 → `succeeded` |
| 3 | 前端重试同一 `request_id` | 不重启任务，状态连续 |
| 4 | 运行中再发不同 `run_id` 的 start | 状态不变，`command.accepted=false, reason=run_active` |
| 5 | pause | `canceling → paused`；车停下 |
| 6 | resume | 重新下发当前点，dwell 重新计时 |
| 7 | stop | `canceling → canceled` |
| 8 | 单点不可达（goal timeout） | 按 `skip`/`abort` 行为，`message` 有原因 |
| 9 | TF 断 3 s | `failed`，`message` 说明定位丢失 |
| 10 | `/estop=true` 运行中 | 立即 cancel + `failed`，之后拒绝新 start |
| 11 | `robot_id` 不匹配 | 忽略 + `reason=robot_mismatch` |
| 12 | `schema_version=2` | 忽略 + `reason=unsupported_version`；不崩溃 |
| 13 | 运行中刷新浏览器 | 状态在 1 s 内恢复（心跳），当前目标坐标与逐点结果从 `mission_state` + `GET /api/runs/:id` 恢复 |
| 14 | 到点进入 dwell | `step_state=dwelling`，`target` 仍指向当前点；离开 dwell 后切下一个 `target` |
| 15 | 某点被 skip | `last_step_result.outcome=skipped` 且 `reason` 非空；任务继续下一个点 |
| 16 | 到达一个点 | 只产生一条 `reached`；后续心跳重复携带同一条，客户端去重 |
| 17 | 浏览器关闭，任务继续 | 任务完成；后端 run 终态由重新打开页面回写，或启动清理标 unknown |

## 11. 字段归属（谁负责写什么）

| 字段 | 产生方 | 备注 |
|------|--------|------|
| `run_id` / `plan_json` / steps | 后端（`POST /api/missions/:id/runs`） | 节点只消费，不生成 |
| `request_id` | 前端 | 一次启动尝试一个；重试不变 |
| `state` / `step_state` / `step_index` / `target` / `last_step_result` / `distance` / `message` | 节点 | 权威来源 |
| `mission_state` 快照 | 前端 | 收到状态后写后端，1 Hz 节流（含 `target`、`step_state`） |
| `mission_runs.results_json` | 前端 | 累积 `last_step_result` 后 PATCH，全量替换、按 `step_index` 去重 |
| `mission_runs` 终态 | 前端 | 幂等 PATCH；节点重启后也可能没人写，由启动清理兜底 |

## 12. 变更记录

| 日期 | 变更 |
|------|------|
| 2026-10-26 | 初版：两话题契约、状态机、三条自动中止、幂等/拒绝、联调清单 |
| 2026-10-26 | 状态增加 `target`（当前目标点坐标/名称）、`step_state`（navigating/dwelling）、`last_step_result`（逐点 reached/skipped/failed）；前端累积为 `mission_runs.results_json` |

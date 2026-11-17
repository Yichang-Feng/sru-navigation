# 定位状态接口对接说明

本文面向下游 monitor 和集成方，说明 Super-LIO 有图定位节点的状态接口。
项目总体介绍见 [README.md](README.md)。

## 1. 接口概览

| 项目 | 约定 |
|---|---|
| 发布程序 | `map_localization_node` |
| 状态话题 | `/localization/status` |
| ROS 消息类型 | `std_msgs/msg/String` |
| 消息内容 | `data` 字段中存放 JSON 字符串 |
| Reliability | `RELIABLE` |
| Durability | `TRANSIENT_LOCAL` |
| History / Depth | `KEEP_LAST` / `1` |
| 发布方式 | 状态变化时发布；正常定位的 `succeed` 另外按 1 Hz 上报 |

JSON 格式：

```json
{"state":"error","msgs":["localization_lost","consecutive_rejects","high_rmse"]}
```

| JSON 字段 | 类型 | 含义 |
|---|---|---|
| `state` | string | 当前定位状态，取值为 `ready`、`succeed`、`error`、`initialized_failed`、`failed` |
| `msgs` | string[] | 原因码及必要的可读提示；没有原因时为 `[]` |

两个字段都会提供。下游对 `msg.data` 做一次 JSON 解析即可。原因码和提示均为字符串，
应逐项处理，避免依赖数组位置或把整个数组与某一个固定示例比较。

## 2. 状态与上报规则

| state | 含义 | 触发与频率 | 下游处理 |
|---|---|---|---|
| `ready` | 传感器输入已具备，等待建立地图定位 | 首次获得非空、可同步的 LiDAR 和 IMU 数据后发送；等待初值时不周期重发 | 参与 monitor 的就绪汇总，尚不能当作定位成功 |
| `succeed` | 已完成定位并正常跟踪 | 初始化或 LOST 恢复确认成功时立即发送，随后按 1 Hz 上报 | 认为定位正常，并检查后续成功心跳是否持续 |
| `error` | LOST、输入暂时中断或已有定位的人工重新初始化 | 事件触发；重复相同状态和原因不重发 | 可恢复；LOST 时 monitor 停止移动，定位端继续尝试匹配 |
| `initialized_failed` | 初始化尝试耗尽，尚未获得可靠初始定位 | 事件触发；重试期间保持此状态 | 按提示给新初值，或移动后重试；成功后转为 `succeed` |
| `failed` | 前端失效或 LOST 已持续 5 秒 | 事件触发一次；之后状态和原因锁定 | 终态，必须重启定位程序 |

定位程序不发布 `idle`。monitor 收齐各程序的 `ready` 后，自行汇总为 `idle`。
当前接口也不发布 `warning`。

本接口按下面的业务时序区分 `error` 和 `failed`：LOST 首先以 `error` 通知
monitor 停车；在 5 秒内恢复可以回到 `succeed`。连续 LOST 满 5 秒则转报
`failed`，进入只能重启解除的终态。前端失效立即进入同样的终态。
初始化尝试耗尽使用独立的 `initialized_failed`，与终态 `failed` 区分。

尚未收到可用的同步传感器数据时，定位节点不会提前发布 `ready`。
节点没有启动、地图加载未完成、或始终没有输入时，也不能依靠此话题得到就绪确认。

### 2.1 正常启动与初始化

1. 启动定位节点，配置 `lio.maploc.init_pose_source: "topic"`。
2. 首次获得可用的同步 LiDAR / IMU 数据，发布 `ready`。
3. monitor 汇总各程序的就绪状态；定位节点等待用户提供 `/initialpose`。
4. 首次初始化搜索期间保持原状态，不因每次搜索尝试而发送新消息。
5. 初始化经过连续两帧一致性确认后，立即发布 `succeed`，随后按 1 Hz 上报。

```json
{"state":"ready","msgs":[]}
```

```json
{"state":"succeed","msgs":[]}
```

### 2.2 LOST 与恢复

前端里程计仍正常、地图匹配丢失时，上报 `error`。
单次配准被拒绝不会直接触发此状态；沿用定位后端现有的连续拒绝次数和匹配超时判据。

`msgs` 同时包含：

- `localization_lost`：说明地图定位已经丢失。
- `consecutive_rejects` 和 / 或 `match_timeout`：说明进入 LOST 的触发条件。
- 最近一次跟踪失败的具体原因，例如 `high_rmse`、`low_inlier` 或 `no_submap`。

连续匹配残差过大导致 LOST：

```json
{"state":"error","msgs":["localization_lost","consecutive_rejects","high_rmse"]}
```

当前位置附近没有可用子图，最终触发匹配超时：

```json
{"state":"error","msgs":["localization_lost","match_timeout","no_submap"]}
```

如果两个触发条件同时满足，数组会同时包含 `consecutive_rejects` 和 `match_timeout`。
LOST 期间停止 `succeed` 心跳。monitor 收到 `error` 后停止机器人移动，
定位后端仍持续尝试重定位；如果在 5 秒内恢复确认成功，立即发送
`{"state":"succeed","msgs":[]}`，并恢复 1 Hz 上报。

如果连续 LOST 满 5 秒，发送一次 `failed`，保留原 LOST 原因，并追加
`lost_duration_exceeded`：

```json
{"state":"failed","msgs":["localization_lost","consecutive_rejects","high_rmse","lost_duration_exceeded"]}
```

计时从进入 LOST 开始，使用主机单调时钟，不受传感器时间戳或 ROS 时间跳变影响。
该时长由 `lio.maploc.monitor_lost_fail_after` 配置，默认 `5.0` 秒。
转报 `failed` 后停止定位输出并锁定状态，不再允许重新定位恢复，必须重启程序。
如果在进入 `failed` 之前恢复，之后再次发生 LOST，会重新开始计时。

这些原因描述进入 LOST 时的触发条件和最近一次跟踪失败，不是逐次重定位搜索的日志。

### 2.3 初始化失败与人工重新初始化

初始化尝试次数耗尽，或在尝试上限内始终不能完成连续两帧一致性确认时，
后端进入新增的 `INITIALIZED_FAILED` 状态，并发送一次：

```json
{"state":"initialized_failed","msgs":["localization_init_failed","初始化失败，请重新提供初始定位，或移动机器人到结构丰富的场景重新初始化"]}
```

此状态停止原地重复搜索，并提供两条恢复路径：

1. 用户重新发送 `/initialpose`，立即开始新一轮初始化。
2. 将机器人移动到结构丰富的区域：相对初始化失败时的位置移动达到
   `lio.maploc.init_retry_min_dist`（默认 `0.5` 米）后，自动开始新一轮初始化。
   新一轮沿用已有初值，并用里程计传播到当前位置。

静止等待、仅时间流逝，或小于移动阈值的抖动，不会触发新一轮搜索。
每轮仍受 `lio.maploc.init_max_attempts` 限制；失败后重新停在 `INITIALIZED_FAILED`。
移动只能改善扫描结构，不能保证修正一个错误的初值；初值不准时应重新提供初值。
收到新初值或移动后重试时，继续保持 `initialized_failed` 及原提示，
不再发送 `error` 或其他中间状态；只有定位确认成功后才发送 `succeed`。

如果之前已经成功定位，收到人工重新初始化请求时，立即停止成功心跳，并发送：

```json
{"state":"error","msgs":["localization_reinitializing"]}
```

重新初始化成功后恢复 `succeed`；尝试耗尽则上报 `initialized_failed`，
`msgs` 包含 `localization_init_failed` 和中文提示。

### 2.4 输入中断

节点曾获得可用同步输入，但之后超过超时阈值未再获得非空同步数据组时，发送一次：

```json
{"state":"error","msgs":["sensor_data_timeout"]}
```

此时停止 `succeed` 心跳。如果输入中断与 LOST 重叠，保留 LOST 当前阶段的
`state` 和原因，并追加 `sensor_data_timeout`，例如尚未达到 5 秒时：

```json
{"state":"error","msgs":["localization_lost","match_timeout","no_submap","sensor_data_timeout"]}
```

输入恢复后，根据后端当前阶段更新状态：仍在等待首次定位则恢复 `ready`，
仍为 LOST 且未进入终态时保持 `error`，后端为正常跟踪则恢复 `succeed`。
如果已经上报 `initialized_failed`，输入恢复和重新尝试仍保持该状态，直到定位成功。
如果已经进入终态 `failed`，输入恢复也不会改变状态。
输入恢复本身不代表 LOST 已经恢复。

超时参数：

```yaml
lio.maploc.monitor_input_timeout: 2.0
lio.maploc.monitor_lost_fail_after: 5.0
lio.maploc.init_retry_min_dist: 0.5
```

`monitor_input_timeout` 单位为秒，默认 `2.0`，按主机单调时钟计算。
`monitor_lost_fail_after` 单位为秒，控制 LOST 从 `error` 转报 `failed` 的延时；
它与后端判断进入 LOST 的 `lost_timeout` 是两个独立参数。
`init_retry_min_dist` 单位为米，控制初始化失败后触发自动重试的移动距离。
这里的输入超时检查的是可用同步数据组，
所以仅有某一个传感器继续发消息，也可能触发输入超时。
暂停或慢速播放 bag 时，此参数仍按实际经过的时间计时。

### 2.5 前端异常与终态锁定

前端输出非有限位姿、连续超速或里程计位移超出地图尺度限制时，发送 `failed`，例如：

```json
{"state":"failed","msgs":["frontend_non_finite"]}
```

进入 `failed` 后停止定位位姿和 TF 输出，锁定首次失败时的状态和原因。
即使前端随后恢复合理运动、收到新初值或移动机器人，也不会离开此状态。
`/initialpose` 会被忽略，`/loc/reinit`（YAML 模式）返回失败并提示重启。
只有重启定位程序才能清除锁定，重新走 `ready → 初始化 → succeed`。

此规则适用于所有 `failed`，包括前端异常和 LOST 持续 5 秒后的失败。
初始化耗尽的 `initialized_failed` 不属于这个终态。

## 3. 原因码表

| 原因码 | 含义 |
|---|---|
| `localization_lost` | 地图定位丢失，前端里程计仍可用于外推 |
| `consecutive_rejects` | 连续被拒绝的匹配次数达到 LOST 阈值 |
| `match_timeout` | 距离上次接受地图匹配已超过 LOST 超时阈值 |
| `lost_duration_exceeded` | 进入 LOST 后已持续达到上报阈值，默认 5 秒，由 `error` 转报 `failed` |
| `not_converged` | 配准没有得到可用位姿；此码不单指优化器迭代未收敛 |
| `low_inlier` | 配准内点比例低于阈值 |
| `high_rmse` | 配准残差超过阈值 |
| `degenerate` | 配准几何约束退化，退化维数超过允许范围 |
| `body_gate` | 配准要求的本体位置或姿态修正过大 |
| `rate_gate` | 相邻接受观测之间的漂移变化超过速率门预算 |
| `chi2` | 观测与当前估计不一致，未通过卡方门 |
| `bad_covariance` | 配准观测协方差异常 |
| `no_submap` | 预测位置附近没有可用地图子图 |
| `localization_init_failed` | 初始化搜索 / 确认尝试耗尽，等待新初值或移动后重试 |
| `localization_reinitializing` | 人工重新初始化，或故障后开始新一轮初始化 |
| `localization_waiting_init` | 已有定位记录，但当前需要重新提供初始定位（`error`） |
| `sensor_data_timeout` | 超时未获得可用的同步 LiDAR / IMU 数据组 |
| `frontend_non_finite` | 前端位姿含 NaN 或 Inf |
| `frontend_speed` | 前端隐含运动速度连续超过允许上限 |
| `frontend_odom_radius` | 前端位姿距里程计原点的距离超过地图尺度限制 |

下游以 `state` 决定状态等级，以 `msgs` 展示或细分原因。
新增未知原因码时，应保留并展示原字符串，不应因此把 `error` 当成正常。
初始化失败的数组还包含中文操作提示，可直接展示给用户；做程序判断时使用原因码。

## 4. ROS 2 订阅示例

### 4.1 命令行

```bash
ros2 topic echo /localization/status std_msgs/msg/String \
  --qos-reliability reliable \
  --qos-durability transient_local \
  --qos-history keep_last \
  --qos-depth 1
```

查看发布端和订阅端 QoS：

```bash
ros2 topic info /localization/status --verbose
```

### 4.2 Python / rclpy

```python
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String


class LocalizationMonitor(Node):
    def __init__(self):
        super().__init__("localization_status_monitor")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.subscription = self.create_subscription(
            String, "/localization/status", self.on_status, qos
        )

    def on_status(self, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f"无效状态 JSON: {exc}")
            return

        if not isinstance(payload, dict):
            self.get_logger().error("状态 JSON 必须是对象")
            return

        state = payload.get("state")
        reasons = payload.get("msgs")
        if (
            not isinstance(state, str)
            or not isinstance(reasons, list)
            or not all(isinstance(reason, str) for reason in reasons)
        ):
            self.get_logger().error("字段类型错误：需要 state:string、msgs:string[]")
            return

        if state not in {"ready", "succeed", "error", "initialized_failed", "failed"}:
            self.get_logger().warning(f"未知定位状态: {state}")
            return

        # 在此接入 monitor 的状态存储、显示和恢复策略。
        self.get_logger().info(f"localization: state={state}, msgs={reasons}")


def main():
    rclpy.init()
    node = LocalizationMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
```

运行环境需要已安装 `rclpy` 和 `std_msgs`，并已加载 ROS 2 环境。
示例仅演示订阅、JSON 解析和字段校验；monitor 的心跳超时策略需自行接入。

## 5. 初始化位置输入

| 项目 | 约定 |
|---|---|
| 话题 | `/initialpose` |
| 类型 | `geometry_msgs/msg/PoseWithCovarianceStamped` |
| 位姿含义 | 机器人 `base` 在地图坐标系 `map` 下的位姿 |
| 姿态格式 | 四元数，ROS 字段顺序为 `x, y, z, w` |

上游应先把初值转换到地图坐标系，再填写 `header.frame_id: map`。
当前回调不会根据 `header.frame_id` 做 TF 坐标变换。
常规二维初始化给出 `(x, y, yaw)` 即可；默认高度取地图，roll / pitch 取前端姿态。
收到初值或服务请求被接受，都不代表初始化成功，必须等待 `succeed`。

也可调用 `/loc/reinit`，类型为 `std_srvs/srv/Trigger`，但它仅在
`lio.maploc.init_pose_source: "yaml"` 时使用 YAML 初值重新初始化。
正常等待用户初值的 `"topic"` 模式应通过 `/initialpose` 发起定位。

## 6. 对接时的时序约定

- **只有 `succeed` 有 1 Hz 心跳。** `ready`、`error`、`initialized_failed`、`failed` 长时间没有重发是正常行为。
  monitor 应分别设置启动就绪等待和成功心跳超时策略，不能对所有状态统一要求每秒有消息。
- **事件消息会去重。** 同一 `state` 与同一 `msgs` 持续存在时不重发；原因变化、异常恢复时会发送新消息。
- **后启动订阅者要使用 transient_local。** 这样才能收到发布者仍存活时缓存的最后一条状态。
  深度为 1，只能得到当前缓存，不能回放完整状态历史。
- **缓存成功状态不等于新鲜定位。** JSON 没有时间戳、序号或节点运行标识。
  后启动的 monitor 应等待后续成功心跳，并结合自身超时策略确认持续更新。
- **节点退出不会自动发送 failed。** `failed` 表示程序检测到前端异常或持续 LOST；进程退出、网络中断和未启动，
  需要 monitor 自己检测。首次收到缓存状态后，不应仅凭该消息断言节点持续在线。
- **区分可恢复状态与终态。** `error` 可恢复为 `succeed`；`initialized_failed` 在重试期间保持，
  定位成功后清除。任何 `failed` 都必须重启定位程序，不能通过新初值解除。
- **后端状态与 monitor 状态分开理解。** `/loc/status.state=6` 表示新增的
  `INITIALIZED_FAILED`；JSON 使用对应的小写状态 `initialized_failed`。
  后端 `FAILED`（数值 5）与 JSON 的终态 `failed` 对应。

## 7. 与其他定位输出的关系

`/localization/status` 是供 monitor 使用的 JSON 状态接口。
原有 `/loc/status`（`super_lio/msg/LocStatus`）提供配准质量、修正量和耗时等详细诊断。
机器人定位输出可使用 `/loc/robo/odom`，类型为 `nav_msgs/msg/Odometry`。

LOST 期间，现有定位输出会使用最后一次地图修正和前端里程计外推；
位姿话题仍有数据不能代替状态判断。后端处于 FAILED 时会停止定位位姿输出及
`map → world` TF 发布。初始化尝试耗尽进入 `INITIALIZED_FAILED` 时，也会清除旧定位
的有效性，停止定位位姿与 TF 输出，直到新一轮初始化确认成功。

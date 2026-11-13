# Unitree Go2W 导航终点姿态（朝向）传统对齐控制交接文档
# (GO2W Terminal Goal Heading Alignment Handover Guide)

- **创建时间**：2026-11-11
- **当前仓库**：`/home/yichangfeng/sru-navigation-learning`
- **Git 当前分支**：`main`
- **最新提交基线**：`490f211 feat(stairs): add 3D slope gate and 0.45m lookahead for stair ascent`
- **适用机型**：Unitree Go2W 轮足/四足机器人（机载工控机 ROS 2 Humble 环境）

---

## 1. 当前项目整体状况与最新技术基线

当前仓库已建立起完整的 **Unitree Go2W 视觉端到端 + 几何自适应楼梯导航系统**。在启动新任务前，请先熟知以下基础模块与已验证的工程特性：

### 1.1 核心算法与网络架构
* **网络策略**：基于 SRU (Spatial Recurrent Unit) 空间循环记忆的视觉强化学习策略网络（`nav_policy.onnx` + `vae_encoder.onnx`）。
* **输入维度**：本体感知 16 维（线速度、角速度、重力投影向量、上一时刻动作、3D 目标相对机身向量与对数距离）+ 深度特征 2560 维。
* **平地弯道自适应（`--curv-only`）**：依据 Path 局部曲率半径在 `0.5m ~ 2.0m` 动态调节前瞻点（Carrot Point），大曲率直道高速巡航，小曲率弯道自动贴近。
* **楼梯全地形 3D 坡度门控（`StairsSlopeGate`）**：
  * **下楼梯**：路径坡度 $\le -0.20$ 且高程落差 $\le -0.15\text{m}$ 时，提前约 0.5m 锁存前瞻点为 `0.40m`，采用原始深度图无损下楼，消除虚拟障碍误判。
  * **上楼梯（最新上线，Commit 490f211）**：路径坡度 $\ge +0.20$ 且爬升高程 $\ge +0.15\text{m}$ 时，提前锁存前瞻点为 `0.45m`，消除大前瞻目标梯度稀释引发的拒止倒车与原地打转，实测推进速度稳定在 $\approx 0.38\text{m/s}$。
* **标准 6 状态有限状态机（FSM）**：
  定义在 [`rl_nav_controller.py`](file:///home/yichangfeng/sru-navigation-learning/rl_nav_controller.py)：`IDLE`, `RUNNING`, `GOAL_REACHED`, `STOPPED`, `FAILED`, `ERROR`。

### 1.2 关键代码文件清单

| 文件路径 | 模块定位与职责 |
| :--- | :--- |
| [`rl_nav_controller_stairs.py`](file:///home/yichangfeng/sru-navigation-learning/rl_nav_controller_stairs.py) | **当前实车运行主入口类**（继承自自适应控制器）。承载 3D 楼梯门控仲裁、前瞻点计算、安全防倒退守护、诊断发布。 |
| [`rl_nav_controller_adaptive.py`](file:///home/yichangfeng/sru-navigation-learning/rl_nav_controller_adaptive.py) | 双因子自适应控制器基类（曲率自适应 + 门洞净空自适应）。 |
| [`rl_nav_controller.py`](file:///home/yichangfeng/sru-navigation-learning/rl_nav_controller.py) | 核心基础控制器。包含 ROS 2 订阅/发布、TF2 转换、Path 接收、ONNX 模型前向推断、标准 6 状态机。 |
| [`adaptive_path_tracker.py`](file:///home/yichangfeng/sru-navigation-learning/adaptive_path_tracker.py) | 路径几何跟踪器。负责投影进度更新、Carrot 前瞻点插值、到达终点判定（`check_arrived`）、曲率计算。 |
| [`stairs_depth_processor.py`](file:///home/yichangfeng/sru-navigation-learning/stairs_depth_processor.py) | 3D 路径坡度门控类 `StairsSlopeGate`。双向检测上下楼梯斜率。 |
| [`scripts/start_nav.sh`](file:///home/yichangfeng/sru-navigation-learning/scripts/start_nav.sh) | 实车标准一键启动脚本（默认采用 `--curv-only` + 楼梯感知双门控）。 |
| [`tests/test_stairs_goal_stopping.py`](file:///home/yichangfeng/sru-navigation-learning/tests/test_stairs_goal_stopping.py) | 自动化单元回归测试脚本。 |

---

## 2. 新任务需求：终点姿态（朝向）传统控制

### 2.1 任务目标与约束原则
* **核心需求**：当机器狗到达上层发布的全局路径最后一个点时，机身朝向与该点给定的航向角 $\theta_{\text{target}}$ 对齐（默认容差 $|\Delta \theta| \le 0.08\text{ rad} \approx 4.5^\circ \sim 5.0^\circ$），并在对齐后平稳刹车停机。
* **三大黄金约束（切勿违背）**：
  1. **不微调、不重新训练 RL 神经网络**：采用**纯传统控制算法**（带死区补偿与限幅的 P 控制器）接管。
  2. **两阶段完全时序解耦**：
     * **过程阶段（0% ~ 100% 路径长）**：100% 由原有的 SRU 策略网络和自适应前瞻模块接管，过程中**严禁**施加任何强制的朝向对齐或横向角速度强加，确保动态避障、走廊通行、上下楼梯等优秀的地形越障能力不受丝毫破坏。
     * **终点对齐阶段**：仅在平面距离到达终点阈值（`check_arrived(robot_xy, arrive_threshold=0.20m)` 为 `True`）后触发。
  3. **严格的状态机向后兼容**：
     * 保持对外标准的 6 状态机规范。在进行原地朝向对齐时，外部状态仍维持 `RUNNING`；对齐收敛或超时后，再触发 `GOAL_REACHED` 并回到 `IDLE`。

---

## 3. 上层 Path 朝向信息与坐标系深度解析

### 3.1 上层 Path 是否包含朝向信息？
* 上层通过 `/nav/global_path`（或配置的 `path_topic`）发布的类型是标准 ROS 2 `nav_msgs/msg/Path`。
* `Path` 包含 `poses: geometry_msgs/msg/PoseStamped[]` 数组。每个点包含 `pose.position (x, y, z)` 与 `pose.orientation (x, y, z, w)`。
* **解析判定准则**：
  在 [`rl_nav_controller.py:L945-953`](file:///home/yichangfeng/sru-navigation-learning/rl_nav_controller.py#L945-L953) 中已有判定逻辑：
  ```python
  final_q = path_msg.poses[-1].pose.orientation
  norm_sq = final_q.x**2 + final_q.y**2 + final_q.z**2 + final_q.w**2
  terminal_yaw = None
  if norm_sq > 0.5:  # 有效四元数（已归一化，非未初始化的 [0,0,0,0]）
      terminal_yaw = float(np.arctan2(
          2.0 * (final_q.w * final_q.z + final_q.x * final_q.y),
          1.0 - 2.0 * (final_q.y**2 + final_q.z**2)
      ))
  ```
  * **情况 A（有效朝向）**：`norm_sq > 0.5`，上层显式指定了终点姿态，成功提取到 `terminal_yaw`。
  * **情况 B（无朝向需求）**：若上层只填坐标、未填姿态导致四元数全为 0（`norm_sq <= 0.5`），则 `terminal_yaw = None`。此时控制器**直接跳过对齐阶段**，到达位置立即完成。

### 3.2 坐标系基准与转换流
* **基准坐标系**：
  * 上层路径的位姿定义在 `path_msg.header.frame_id`（通常为全局地图系 `'map'` 或 `'world'`）。
  * 机器狗当前的位姿 `robot_pos_w, robot_orientation_w`（由机载激光 SLAM 或轮式/惯性里程计解算）同样位于 `map_frame_id`（通常为 `'map'`）。
* **当前朝向解算**：
  若当前全局四元数为 $[w, x, y, z]$，机器狗在全局系的实际偏航角为：
  $$\theta_{\text{robot}} = \text{atan2}\big(2.0 \cdot (w z + x y), \; 1.0 - 2.0 \cdot (y^2 + z^2)\big)$$
* **航向误差角 $\Delta \theta$**：
  在全局系下做角度差，并通过环形解缠绕映射至 $[-\pi, \pi]$：
  $$\Delta \theta = \text{atan2}\big(\sin(\theta_{\text{target}} - \theta_{\text{robot}}), \; \cos(\theta_{\text{target}} - \theta_{\text{robot}})\big)$$
* **控制指令输出系**：
  误差 $\Delta \theta$ 算出的角速度 $\omega_z$ 输出到 `/cmd_vel`（`geometry_msgs/msg/Twist`），属于机器狗机体坐标系（`base_link`）的自转速度，线速度 $v_x = 0.0, v_y = 0.0$。

---

## 4. 传统到点朝向控制器的数学建模与工程设计

### 4.1 控制律设计（带摩擦死区补偿与限幅的 P 控制器）

Unitree Go2W 在静止状态下进行原地踏步自转时，足底与地面存在静摩擦力矩。若直接使用线性 P 控制，在小误差区间输出的极小角速度将无法克服静摩擦，导致机器狗原地踏步震颤而不旋转；而在大误差区间若速度过大，又会产生剧烈侧晃。

因此，采用如下分段非线性控制律：

$$\omega_z = \begin{cases} 
0.0, & \text{if } |\Delta \theta| \le \theta_{\text{tol}} \\
\text{sgn}(\Delta \theta) \cdot \text{clamp}\Big( |\Delta \theta| \cdot K_p + \omega_{\text{deadband}}, \; \omega_{\min}, \; \omega_{\max} \Big), & \text{if } |\Delta \theta| > \theta_{\text{tol}}
\end{cases}$$

### 4.2 推荐工程参数表

| 参数名称 | 变量名 / 默认值 | 物理意义与工程取值依据 |
| :--- | :--- | :--- |
| **对齐容差阈值** | `terminal_heading_tol = 0.08 rad` ($\approx 4.58^\circ$) | 允许的最终对齐误差。四足踏步不宜低于 $3^\circ$（避免因步态离散产生微震），$5^\circ$ 是工程最佳平衡点。 |
| **比例增益** | `terminal_heading_kp = 1.2` | 航向误差比例响应增益。 |
| **摩擦死区补偿** | `terminal_heading_deadband = 0.15 rad/s` | 确保进入转动时立刻给予基础自转扭矩，利落转动。 |
| **最小角速度钳位** | `terminal_heading_min_vel = 0.18 rad/s` | 防止旋转末端角速度过小卡滞。 |
| **最大角速度钳位** | `terminal_heading_max_vel = 0.60 rad/s` | 限制最大自转速度，防止机身晃动或踏空。 |
| **对齐超时保护** | `terminal_heading_timeout = 5.0 s` | 原地对齐最大允许耗时。防打滑或卡死看门狗。 |
| **平滑低通滤波** | $\alpha = 0.3$ | 沿用已有指令低通滤波器，加减速平滑不顿挫。 |

---

## 5. 五重安全与异常防护机制

1. **四元数未配置直接跳过（Graceful Fallback）**：
   若 `path_tracker.terminal_yaw is None`，则说明上层无需对齐朝向。位置一旦满足 `< 0.20m`，立刻原路退出并报 `GOAL_REACHED`。
2. **楼梯/陡坡自转禁绝保护（Stair/Slope Spin Guard）**：
   若位置到达时，坡度门控显示当前仍处于楼梯上（`is_stair_down == True` 或 `is_stair_up == True`），**严格禁止原地大幅度旋转**！必须直接以当前姿态就地刹车完成，防止机器狗在陡峭楼梯台阶上踩空摔落。
3. **超时看门狗保护（Timeout Watchdog）**：
   开始执行对齐时记录 `heading_align_start_time = now()`。若超过 `5.0s` 仍未进入容差范围，强制输出 `0` 速度并中止对齐，记录警告日志并流转至 `GOAL_REACHED`，防止无限原地踏步耗尽电池。
4. **踏步位置漂移保护（Position Drift Guard）**：
   四足原地踏步可能产生轻微平移。每周期检测当前位置与终点坐标的距离：若漂移量超过 `0.40m`，立即终止旋转并刹停，防止漂出安全通行区。
5. **稳定驻留确认（Stabilization Window）**：
   连续满足 $|\Delta \theta| \le \theta_{\text{tol}}$ 达到 `0.2s`（约 2~3 个控制周期）才正式判定完成，避免瞬时穿过目标朝向的动态过冲假象。

---

## 6. 实施路线与具体修改步骤（下一步任务指引）

接手的 Agent 或工程师可按以下步骤有序实施并验证：

### 步骤 1：扩展 [`adaptive_path_tracker.py`](file:///home/yichangfeng/sru-navigation-learning/adaptive_path_tracker.py)
- **目标**：方便控制器一键获取终点坐标和终点朝向。
- **改动**：在 `AdaptivePathTracker` 类中添加 `get_terminal_pose()` 方法：
  ```python
  def get_terminal_pose(self):
      """Return (terminal_pt, terminal_yaw) if active, else (None, None)."""
      if not self.is_active() or len(self.points) == 0:
          return None, None
      return self.points[-1].tolist(), self.terminal_yaw
  ```

### 步骤 2：在 [`rl_nav_controller_stairs.py`](file:///home/yichangfeng/sru-navigation-learning/rl_nav_controller_stairs.py) 中接入对齐状态与逻辑
- **参数声明（在 `__init__` 中）**：
  ```python
  self.declare_parameter('enable_terminal_heading_align', True)
  self.declare_parameter('terminal_heading_tol', 0.08)       # ~4.6 deg
  self.declare_parameter('terminal_heading_kp', 1.2)
  self.declare_parameter('terminal_heading_timeout', 5.0)     # seconds
  self.declare_parameter('terminal_heading_max_vel', 0.6)
  self.declare_parameter('terminal_heading_min_vel', 0.18)

  # 内部运行状态
  self.is_aligning_heading = False
  self.heading_align_start_time = None
  self.heading_align_target_yaw = None
  self.heading_align_terminal_pos = None
  self.heading_align_stable_count = 0
  ```
- **重构 `depth_callback` 中的到达逻辑**：
  当 `self.path_tracker.check_arrived(...)` 触发时：
  1. 检查 `self.enable_terminal_heading_align`、`is_stair_down`、`is_stair_up` 以及 `self.path_tracker.terminal_yaw`。
  2. 若在楼梯上或 `terminal_yaw is None`，执行现有的直接刹车并 `set_state(NavState.GOAL_REACHED)`。
  3. 若条件满足，置 `self.is_aligning_heading = True`，记录 `heading_align_start_time`、`heading_align_target_yaw` 和 `heading_align_terminal_pos`，清空 `path_tracker`（防止路径跟踪再次触发），保持 `self.state = NavState.RUNNING`。
- **在控制循环中执行对齐分支**：
  若 `self.is_aligning_heading` 为 `True`：
  1. 检查超时（`> 5.0s`）与位置漂移（`> 0.4m`），若触发则主动刹停并转入 `GOAL_REACHED`。
  2. 计算 $\Delta \theta = \text{atan2}(\sin(\theta_{\text{target}} - \theta_{\text{robot}}), \cos(\theta_{\text{target}} - \theta_{\text{robot}}))$。
  3. 若 $|\Delta \theta| \le \text{tol}$，增加稳定计数器；当稳定保持后，刹停机器人，置 `self.is_aligning_heading = False`，正式 `set_state(NavState.GOAL_REACHED)` 然后切为 `IDLE`。
  4. 若未达标，按公式计算 $\omega_z$，组装 `Twist(linear.x=0.0, linear.y=0.0, angular.z=omega_z)` 并发布。

### 步骤 3：CLI 与启动脚本参数支持
- 在 [`scripts/start_nav.sh`](file:///home/yichangfeng/sru-navigation-learning/scripts/start_nav.sh) 与 [`scripts/start_nav_stairs.sh`](file:///home/yichangfeng/sru-navigation-learning/scripts/start_nav_stairs.sh) 中：
  - 增加 `--no-heading-align` 选项（默认开启对齐）。
  - 增加 `--heading-tol <弧度>` 选项。
  - 将相应参数注入 ROS 2 启动命令。

### 步骤 4：单元测试与离线回归
- 编写单元测试（可直接扩展 `tests/test_stairs_goal_stopping.py` 或新建 `tests/test_terminal_heading_alignment.py`）：
  * **用例 1（无朝向直接完成）**：`terminal_yaw = None` 时，位置到达立即刹车退出。
  * **用例 2（正反向角速度与最短转角）**：测试目标航向在 $+90^\circ$、$-90^\circ$、$+170^\circ$、$-170^\circ$ 时计算出的 $\Delta \theta$ 正负号与 $\omega_z$ 方向正确。
  * **用例 3（死区与限幅）**：验证当误差极小时不低于 `min_vel`，极大时不高于 `max_vel`。
  * **用例 4（收敛终止）**：连续给入误差 $\le \text{tol}$ 的当前航向，验证稳定切换到 `GOAL_REACHED`。
  * **用例 5（超时看门狗保护）**：模拟时间推进超过 5.0s，验证强制刹停退出。
  * **用例 6（楼梯上保护）**：`is_stair_down=True` 时即使有 `terminal_yaw` 也严禁进入转向。

### 步骤 5：真机部署与验证步骤
1. 推送代码至实车：`./scripts/sync_stairs_to_go2w.sh <ROBOT_IP>`
2. 登录机器狗编译：`colcon build --packages-select rl_nav_controller --symlink-install`
3. 使用带朝向的测试脚本发布路径（或 RViz 拖动 Goal Pose）：
   ```bash
   # 测试给入终点朝向 90 度
   python3 scripts/mock_global_nav_tester.py --send-test-path --terminal-yaw 1.57
   ```
4. 观察机器狗：跑完全程后，到达位置刹停线速度，原地优雅踏步转向至指定朝向，误差进入 $5^\circ$ 后彻底刹停。

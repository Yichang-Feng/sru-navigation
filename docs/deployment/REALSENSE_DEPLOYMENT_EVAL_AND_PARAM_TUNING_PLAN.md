# RealSense 部署效果量化对比与部署参数整定 —— 设计与实施方案

> 适用平台：Unitree Go2W + Jetson Orin（高层脑 `192.168.123.111`）+ Livox MID-360 + Super-LIO
>
> 代码基线：`sru-navigation-learning`（训练/仿真）与 `sru-robot-deployment`（实机部署）
>
> 文档性质：实施方案，不改变现有默认参数；所有"建议值/搜索范围"均需通过本文实验流程确认后落地。

---

## 0. 目标与交付物

本方案同时覆盖两个任务：

| 任务 | 核心问题 | 最终交付物 |
|---|---|---|
| **任务一：RealSense 对比与提升** | 换用 RealSense D435 后，与 ZED 2i 的差距到底在哪里？如何量化并消除？ | 分层评测报告、差距归因、RealSense 配置 v1（必要时含虚拟相机适配层/仿真微调权重）、相机 A/B 结论 |
| **任务二：部署参数整定** | 前瞻、曲率、窄门、楼梯、终点、安全等参数如何系统调优？ | 参数注册表、分场景参数 profile、调参实验数据、最终推荐参数与验证报告 |

两个任务不是串行关系：**任务一先固定"相机呈现给策略的画面"，任务二再在这个画面基础上调控制器参数**。否则相机的 FOV/量程/无效像素处理与控制器参数会相互混淆，调出来的参数不可复现。

---

## 1. 现状与关键事实（代码级）

### 1.1 策略观测与推理链路

- 高层策略 5 Hz 运行（`DEFAULT_CONTROL_FREQUENCY = 5.0`），ONNX 推理，输入为：
  `[本体感觉 16/18 维] + [VAE 深度特征 2560 维]`，带 LSTM `h/c` 状态；输出经 `tanh` 与 `POLICY_SCALE = [1.5, 1.0, 1.0]` 缩放，再经低通 `[0.9, 0.5, 0.5]`。
- 深度预处理（`rl_nav_controller.py: depth_callback`）：
  1. RealSense `16UC1/mono16` 自动 `/1000.0` 转米；ZED `32FC1` 直接为米；
  2. `> max_depth` 或 `+inf` → `0.0`（视为远处开阔）；
  3. `< min_depth`、`NaN`、`-inf` → `min_depth`（视为近处障碍）；
  4. `cv2.resize(..., (64, 40), INTER_LINEAR)`，**不保持长宽比、不做裁剪**。
- 推理入口：`rl_nav_controller.py: predict()`；观测组装在 `state_input + depth_embedding`。

### 1.2 仿真相机基准（策略"见过的世界"）

来自 `IsaacLab/source/isaaclab_nav_task/isaaclab_nav_task/navigation/navigation_env_cfg.py` 与 `config/go2w/navigation_env_cfg.py`：

| 项目 | 仿真值 | 说明 |
|---|---|---|
| 相机安装（相对 base） | `pos = (0.285, 0.0, 0.09)`，`rot = (1,0,0,0)` | Go2W 覆盖为**水平 0°**，不是基类默认的 20° 下俯 |
| 分辨率/内参 | 192×120，`fx=fy=72.7025`，`cx=94.4457`，`cy=62.5424` | 3 倍下采样 → 64×40 |
| 有效视场 | 水平 ≈ **105.8°**，垂直 ≈ **79.1°** | 由内参和分辨率推算 |
| 最大距离 | `max_distance = 11.0 m` | 超出置 0 |
| 深度类型 | `distance_to_image_plane` | 平面 z 深度，不是欧氏距离 |
| 噪声模型 | `initialize_depth_noise_generator(robot_name="go2w")` | 当前映射到 `ZEDX_CAMERA_CONFIG` |

`camera_config.py` 中**已经存在** `REALSENSE_D435_CAMERA_CONFIG`（focal 35.0、baseline 0.05、min 0.15、max 4.0），但 `ROBOT_CAMERA_CONFIGS` 中没有 Go2W+D435 的任务注册，当前训练/评测仍走 ZED X 噪声模型。这是任务一 P3 的现成抓手。

### 1.3 两种相机的部署链路

| 项目 | RealSense D435/D435i（当前默认） | ZED 2i |
|---|---|---|
| 深度话题 | `/camera/camera/depth/image_rect_raw` | `/zed/zed_node/depth/depth_registered` |
| 编码 | `16UC1`（毫米） | `32FC1`（米） |
| 分辨率/帧率 | 640×480 @ 15 Hz | 640×360 @ 15 Hz（`pub_downscale_factor=2.0`） |
| 深度 FOV | 87° H × 58° V | 110° H × 70° V |
| 基线 | 50 mm | 120 mm |
| 有效量程 | 0.15–10 m（理想 0.3–3 m） | 0.25–10 m（理想 0.5–15 m） |
| 部署 `min_depth` / `max_depth` | 0.15 / 10.0 | 0.25 / 10.0 |
| 静态 TF（`base_link` → 相机） | `0.34 0.0 0.25 0 0 0` | `0.257 0.0 0.28 0 0 0` |
| 驱动优化 | 主动 IR、laser 360 mW、High Density、spatial+temporal+hole filling | NEURAL_PLUS、关闭 RGB/点云/VIO/IMU，15 Hz |
| 算力特征 | 板载 D4 深度引擎，CPU/GPU 压力小 | Jetson GPU TensorRT，深度延迟与资源占用更高 |

启动脚本事实：`start_nav.sh` 与 `start_local_sru.sh` 的 `CAMERA_TYPE` 默认均为 `realsense`。

### 1.4 三个高风险不一致（本方案重点验证）

1. **视场/角度映射不一致**：仿真水平 105.8°、垂直 79.1°；RealSense 87°×58°；ZED 110°×70°。三者都直接 resize 到 64×40，导致同一物理角度落在不同像素列/行。RealSense 的角分辨率最"密"，但视野最窄。
2. **安装高度/俯仰与仿真基线不同（属于机型差异，不是缺陷）**：策略的训练/仿真基线是 B2W，实机是 Go2W，两者站高与相机安装点本身就不同；代码中的 Go2W 环境另有 `offset.pos=(0.285,0,0.09)`、0° 水平配置，需确认实际训练用的是 B2W 还是 Go2W 那套。部署 TF 为 base 上方 0.25 m（RealSense）/0.28 m（ZED）。因此不能把"比仿真高/低"直接当成 bug，但仍应实测相机光心离地高度、俯仰和横向偏置，并在虚拟相机层按训练时的相机模型对齐。高度差会改变近地盲区、低障碍和台阶边缘在画面中的位置，是 RealSense 与 ZED 对比时必须控制的变量。
3. **无效像素语义与量程不一致**：RealSense 的无效像素常为 `0`（无回波/遮挡阴影），ZED 常为 `NaN`。当前预处理把 `0`（`< min_depth`）和 `NaN` 都映射为 `min_depth`（近障碍），语义上偏保守；但若大面积空洞被填成"近墙"，会造成假障碍、窄门拒入。另外 RealSense 部署 `max_depth=10.0`，而仿真 D435 配置是 `4.0`，远距噪声会进入策略。

### 1.5 已有工具与可复用资产

| 工具 | 位置 | 可复用点 |
|---|---|---|
| 相机对比诊断记录器（schema 2.0，113 列 CSV + 原始深度 PNG + CameraInfo/TF/控制参数快照 + LiDAR/IMU） | `sru-robot-deployment/diagnostics/door_diagnostic_logger.py` | 5 Hz 同步记录、深度健康/延迟、路径/目标快照、离线对比 |
| RealSense 深度质量测试 | `sru-robot-deployment/diagnostics/test_realsense_max_distance.py` | 有效率、空洞率、分位数、门洞穿透、离线 session 分析 |
| 实机评测采集/调度/分析 | `sru-navigation-learning/scripts/benchmark/` | DTO/LiDAR 真值、CSV/JSON、SPL、反应距离、自动发点 |
| 自适应前瞻单测 | `sru-navigation-learning/scripts/test_adaptive_lookahead.py` | 曲率/门洞/锁存的离线场景验证 |
| 楼梯消融数据 | `docs/test/stairs_ablation_results_frame183.csv` | 前瞻/Query 距离对下楼稳定性的既有结论 |
| 仿真回放 | `IsaacLab/source/isaaclab_nav_task/scripts/play.py` | 参数/相机模型仿真筛选 |
| 全局导航 Mock | `scripts/mock_global_nav_tester.py` | 无机器狗时联调路径/状态机 |

---

## 2. 总体技术路线

采用"**三层评测 + 两阶段优化**"：

```
L0 传感器台架（离线/静态）
   ↓ 解释"相机看到了什么"
L1 匹配状态策略响应（离线回放）
   ↓ 解释"策略对同一场景反应差多少"
L2 闭环导航（实机）
   ↓ 解释"最终任务完成度差多少"

阶段 A：先冻结相机呈现（标定 + 预处理 + 必要的虚拟相机适配）
阶段 B：再分场景整定控制器参数（前瞻/门/楼梯/终点/安全）
```

原则：

1. **先测量，后优化**：没有基线数据不调参。
2. **一次只改变一个层面**：相机配置未冻结前，不调前瞻/楼梯参数；控制器参数未冻结前，不做最终 A/B。
3. **安全优先于性能**：调参期间限制速度、保留所有看门狗与停滞检测、楼梯必须有人守护。
4. **可复现**：每次运行记录相机型号、TF、profile 哈希、代码 commit、电池电压、场景 ID、操作者。
5. **统计而非个例**：每组 ≥5 次初筛、≥10–20 次确认；A/B 交错进行；报告置信区间。

---

## 3. 任务一：RealSense 与 ZED 的量化对比与提升

### 3.1 评测分层与指标体系

#### L0 传感器台架（不动机器人策略）

目的：回答"两台相机在相同场景下，深度图质量差多少；差异是否足以解释策略行为"。

| 指标 | 定义 | 采集方式 |
|---|---|---|
| 有效率 `valid_ratio` | 有效深度像素 / 总像素 | 离线 npy / 在线统计 |
| 空洞率 `hole_ratio` | `0`/`NaN`/`inf` 占比 | 同上 |
| 距离偏差 `bias` | `mean(d_meas - d_ref)` | 平面靶标 + 激光测距仪/LiDAR |
| 距离误差 `RMSE/P95` | 对参考距离的均方根/95 分位绝对误差 | 同上 |
| 时间噪声 `temporal_std` | 同一像素 100 帧标准差 | 静态场景录像 |
| 闪烁率 `flicker_rate` | 相邻帧深度跳变 >5 cm 的像素比例 | 同上 |
| 边缘保真 `edge_error` | 物体边界处"飞点"比例与边界深度梯度 | 斜置平面/门框 |
| 最远可靠距离 `d_max_rel` | P95 相对误差 <5% 且有效率 >80% 的最远距离 | 距离阶梯 |
| 最近可靠距离 `d_min_rel` | 近距离有效且不饱和的最近距离 | 距离阶梯 |
| 端到端延迟 | 图像 header 时间戳 → 回调处理时间 | 在线打点 |
| 帧率/抖动 | 实际 Hz、帧间隔 P95 | `ros2 topic hz` + 日志 |
| 机载负载 | CPU/GPU/内存、推理耗时 | `tegrastats` + 节点计时 |
| 有效 FOV | 有效像素边界角 | 由内参与有效区域反算 |
| 地面盲区 `D_blind` | `h / tan(θ_bottom)` | 相机高度 h + 下视半角 |
| 低障碍可见性 | 距离 d 处、高度 `h - d·tan(θ_bottom)` 以下的障碍不可见 | 同上 |

场景矩阵（L0）：

| 场景 | 内容 | 关注点 |
|---|---|---|
| T1 平面靶标 | 白板/棋盘格，0.3/0.5/0.8/1/1.5/2/3/4/5/6/8 m | 偏差、RMSE、量程 |
| T2 长走廊 | 纹理弱、重复结构 | 远距噪声、空洞、抖动 |
| T3 窄门 | 0.8–0.9 m 门洞，两侧门框 | 门框边缘、门洞穿透、空洞 |
| T4 楼梯 | 下行长楼梯、台阶边缘 | 台阶边缘、深渊、近地盲区 |
| T5 低障碍 | 10 cm 纸箱/横杆 | 可见距离、下视盲区 |
| T6 光照 | 正常室内 / 暗光 <10 lux / 窗边逆光 / 阳光直射 | IR 散斑有效性、曝光 |
| T7 材质 | 白墙、黑绒布、玻璃、金属、反光地面 | 无效像素、多路径 |
| T8 动态 | 行人横穿、移动障碍 | 时间滤波拖影/延迟 |

#### L1 匹配状态策略响应（离线回放）

目的：把"相机差异"与"轨迹差异"解耦，回答"同一机器人位姿、同一目标下，两台相机的深度会让策略输出差多少"。

数据要求（5 Hz，与策略同频）：

- 预处理后的 `depth_40x64`（float32，米）；
- `robot_pos/quat`、`linear_vel`、`angular_vel`、`gravity_vector`；
- `last_action`、`target_pos_w`/`carrot`、`target_yaw_w`；
- 原始网络输出 `raw_action`、滤波后 `cmd_vel`；
- 时间戳、相机 ID、profile ID、trial ID。

方法：

1. **回放一致性校验**：从 episode 起点 reset LSTM，按记录顺序回放 ONNX，要求与线上 `raw_action` 最大误差 <1e-4；否则检查观测字段/归一化/隐藏状态。
2. **匹配状态对比**：从两台相机各自的运行中，筛选位姿差 <2 cm、航向差 <2°、目标差 <5 cm 的帧对，比较 `raw_action`。
3. **跨相机反事实输入**：在匹配状态处，把 A 相机的深度嵌入替换为 B 相机的深度嵌入，测 `Δaction`；这是最直接的"深度差异→动作差异"度量。
4. **指标**：
   - `mean |Δvx|`、`|Δvy|`、`|Δwz|`；
   - 动作符号一致率（尤其 `vx` 前进/停止、`wz` 左/右）；
   - 安全关键分歧率：一台输出前进、另一台输出停止/急转的帧占比；
   - 动作与真实净空的相关性（用 LiDAR 真值）。

> 注意：L1 只能解释"同一状态下的瞬时差异"，不能替代闭环。闭环轨迹会因动作不同而分叉，因此 L1 用于归因与筛选，L2 用于最终结论。

#### L2 闭环导航（实机，主指标）

| 指标 | 定义 | 备注 |
|---|---|---|
| 成功率 `SR` | 成功次数 / 总次数 | 成功 = 到点（<0.20 m）且无碰撞/无人工接管 |
| 碰撞率 `CR` | 发生物理接触的次数 / 总次数 | LiDAR/接触/IMU + 视频复核 |
| 近失率 `NMR` | 最小净空 <0.10 m 的次数 / 总次数 | 与碰撞分开统计 |
| 到达时间 `TTG` | 发点到 `goal_reached` 的秒数 | 超时单独计 |
| 路径长度 `PL` | 里程计积分或定位轨迹长度 | 与最短路比 |
| SPL | `(1/N) Σ S_i · l_i / max(p_i, l_i)` | 标准导航指标 |
| 最小净空 `d_min` | 全程 LiDAR 机体前向走廊最小距离 | 5 分位/最小值 |
| 人工接管率 | 操作者接管/急停次数 / 总次数 | 安全关键 |
| 平均/最大速度 | `odom` 或 `/cmd_vel` | 分场景看 |
| 速度平滑度 | `vx` 的 RMS jerk、`wz` 的 RMS | 舒适性/机械磨损 |
| 航向振荡 | `wz` 过零率、yaw 误差 RMS | 直道/门内 |
| 恢复次数 | `blocked`/重规划/掉头次数 | 策略鲁棒性 |
| 窄门通过时间 | latch 进入到释放的时长 | 门场景 |
| 楼梯通过时间 | 门控进入到退出的时长 | 楼梯场景 |
| 终点航向误差 | 到达后最终 yaw 与目标 yaw 差 | 终点场景 |
| 定位误差 | 与已知地图标记/回环点对比 | 可选 |

### 3.2 实验设计与统计

#### 场景定义（实机，建议固定在同一楼层/走廊）

| ID | 场景 | 起点→终点 | 关键考核 |
|---|---|---|---|
| S1 | 长直走廊（≥10 m） | 直线 | 巡航速度、无假减速、无蛇形 |
| S2 | 90° 直角转弯 | 直行→右/左转 | 转弯完成时间、切内角、最小净空 |
| S3 | 窄门（0.85 m） | 门前 3 m→门后 2 m | 通过率、门框碰撞、latch 时序 |
| S4 | 长楼梯下行（≥8 级） | 平台→楼梯底 | 下楼成功率、拒绝下行、俯仰/ roll |
| S5 | 楼梯上行（如具备） | 楼梯底→平台 | 上行成功率、卡住 |
| S6 | 密集障碍/家具 | 绕障到点 | 碰撞、恢复、最小净空 |
| S7 | 低矮障碍（10 cm） | 直行逼近 | 探测距离、碰撞 |
| S8 | 走廊+门+转弯组合 | 端到端 | 模式切换、总时间 |
| S9 | 终点航向对齐 | 1 m 短程 | 航向误差、对齐时间、漂移 |
| S10 | 光照/材质专项 | 暗光/逆光/反光 | 感知退化下的通过率 |

#### 重复数与顺序

- **初筛**：每个（相机 × 场景 × profile）5 次；用于排除明显失败配置。
- **确认**：每个关键对比 10–15 次；最终结论 20 次以上。
- **交错 A/B**：按 `A B B A` 或随机区组交错，避免电池衰减、地面温度、人员走动等时间漂移。
- **随机化**：场景顺序、相机顺序、profile 顺序均随机；每次运行前重定位/重置。
- **统计**：
  - 成功率：配对 McNemar 检验（同一场景 A/B 配对）；
  - 连续指标：配对 Wilcoxon signed-rank，报告 bootstrap 95% CI；
  - 效应量：Cliff's delta / Cohen's d；
  - 多场景汇总：分层报告，不把不同难度场景简单平均；必要时混合效应模型。
- **预注册**：先写死主指标（成功率、碰撞率、TTG、d_min）与停止规则，避免事后挑指标。

#### 真值与数据质量

- **LiDAR 真值**：MID-360 点云转 `base_link`，取机体前向 1.2 m 宽、0–2 m 高走廊内的最近点；同时记录最近点坐标，便于复核。
- **碰撞**：优先用足端力/接触传感器（若 bridge 暴露）；否则用 IMU 冲击 + odom 速度突变 + 视频人工复核三重判定。
- **人工接管**：操作者按下 B 键/急停即计一次；记录原因。
- **数据质量门控**：帧率 <10 Hz、定位丢失、TF 缺失、CSV 缺列、深度大面积空洞的 run 标记为 invalid，不进入统计。
- **同步**：所有日志带 ROS 时间戳与 monotonic 时间；深度、odom、cmd、carrot 同一 CSV 行；保留原始 bag 以便复核。

### 3.3 差距归因方法

对每个失败/退化 run，按以下顺序归因并计数：

1. **感知层**：深度空洞、远距噪声、近地盲区、FOV 缺失、光照/材质失效；
2. **几何层**：相机高度/俯仰/横向安装误差、TF 错误、FOV 与训练不一致；
3. **时间层**：深度延迟、掉帧、时间滤波拖影、控制 5 Hz 与相机 15 Hz 的相位；
4. **策略层**：同状态动作分歧、LSTM 状态发散、对特定深度模式的偏好；
5. **控制层**：前瞻/门/楼梯/终点参数不适配；
6. **执行层**：底盘打滑、轮足状态、电池、地面。

产出：`failure_taxonomy.csv`（run_id、场景、相机、失败类别、发生时刻、关键帧路径、视频时间戳、复核人）。

### 3.4 提升路线（按成本/收益排序）

#### P0：标定与预处理对齐（0.5–1 周，必须先做）

1. **确认相机型号与安装**：D435 / D435i / D455？序列号、固件、镜头是否干净；用卷尺/水平仪实测相机光心离地高度、俯仰角、横向偏置；确认 `base_link` 原点定义。
2. **统一 TF**：把实测外参写死到启动脚本；RealSense 与 ZED 的 x/z 差异要有依据（同一支架？不同支架？）。在 RViz 中叠加 LiDAR 点云与深度点云，检查地面/墙面是否重合。
3. **量程与无效像素策略**：
   - 测试 RealSense `max_depth` = 4.0 / 6.0 / 10.0（仿真 D435 配置是 4.0）；
   - 测试无效像素填充：`min_depth`（当前，保守）/ 邻域中值 / `0.0`（开阔）；
   - 测试 `min_depth` = 0.15 / 0.20 / 0.25；
   - 观察假障碍、窄门拒入、远距抖动。
4. **滤波与预设消融**：High Density / High Accuracy / Default；spatial/temporal/hole filling 开关；temporal alpha；laser power；emitter on/off；记录边缘保真与延迟。
5. **分辨率/帧率**：640×480@15 / 640×480@30 / 848×480@15；权衡延迟、空洞、CPU。
6. **长宽比与 FOV**：当前直接 resize 到 64×40。至少测试"中心裁剪到 16:9 再 resize"与"保持角分辨率的 letterbox/padding"；用 T1/T2/T3 对比。

**P0 验收**：在 T1–T5 上给出两台相机的量化表；确定 RealSense 的 `min/max_depth`、滤波组合、FPS、裁剪/填充策略 v1。

#### P1：虚拟相机适配层（1 周，解决 FOV/安装不一致）

目标：让任意相机输出"仿真虚拟相机"所看到的深度图，消除 RealSense/ZED/仿真三者的角度映射差异。

- 虚拟相机参数直接取仿真：`fx=fy=72.7025/3`、`cx=94.4457/3`、`cy=62.5424/3`（在 64×40 网格上），安装位姿取仿真 `(0.285, 0, 0.09)`、0°；或先用实测安装位姿，再做"仿真等效"重投影。
- 实现 `depth_adapter.py`：
  1. 读取相机 `CameraInfo` 与 `base_link→camera_link` TF；
  2. 对虚拟相机每个像素生成射线；
  3. 将射线变换到实际相机坐标系，投影到实际深度图，取 z 深度（RealSense/ZED 都是沿光轴 z 深度，与仿真 `distance_to_image_plane` 一致）；
  4. 越界/无效像素按策略填充（0.0 或 `min_depth`，作为可测参数）；
  5. 输出 64×40 浮点米制深度，替换原 `depth_callback` 的 resize 结果；
  6. 发布 `depth_health`（有效率、空洞率、延迟、填充比例）供诊断与安全限速。
- **验证**：T1 平面靶标上，重投影后的深度应与解析平面一致（误差 <2 cm）；T2 走廊上检查墙面直线度；T3 门框边缘不出现大面积飞点。
- **A/B**：原 resize vs 虚拟相机；在 S1/S2/S3/S4/S7 上各 10 次。
- **俯仰选项**：在虚拟相机框架下测试 RealSense 物理俯仰 0° / -5° / -10°（向下），看近地盲区、低障碍、楼梯边缘的收益；注意这需要机械调整或支架。

> 如果 P1 收益显著，它应成为 RealSense 的默认预处理；ZED 也可走同一层，保证两者可比。

#### P2：相机专属控制器 profile（与任务二联动，1 周）

- RealSense 有效量程短、FOV 窄：`lookahead_max` 可能应从 2.0 降到 1.5–1.8；`stair_lookahead` 可能需略增或调整门控；`max_depth` 与远距噪声相关。
- ZED 保留原 profile。
- 产出：`profiles/realsense_v1.yaml`、`profiles/zed_v1.yaml`，并在 S1–S9 上验证。

#### P3：仿真域适配/微调（2–3 周，可选但推荐）

1. 把 `go2w_d435` 接入仿真：`initialize_depth_noise_generator(robot_name="go2w_d435")` 或新增 D435 噪声生成器；注册 `Isaac-Nav-*-Go2W-D435-*` 任务。
2. 用当前策略在 D435 噪声/FOV 下评测（无需训练），量化 sim 内的"相机换型损失"。
3. 若损失显著：从 Phase 4/5 checkpoint 微调 2k–5k iter，加入：
   - 相机外参随机化（±2 cm、±3°、pitch ±5°）；
   - 深度空洞/丢帧/噪声（D435 参数）；
   - 可选：ZED/D435 混合随机化，训练一个相机无关策略。
4. 导出 ONNX，先在仿真评测，再实机 A/B；只有实机优于 P1/P2 才采用。

#### P4：部署侧安全增强（可选，0.5 周）

- **置信度限速**：深度有效率/空洞率低于阈值时，限制 `vx` 上限或增大安全距离；不改变策略输入。
- **LiDAR 近场守门**：用 MID-360 检测机体前向近场障碍，若小于制动距离则限速/急停；作为独立安全层，不喂给策略。需测误报率（尤其楼梯边缘、玻璃、反光地面）。
- **时间戳门控**：深度帧年龄 >100 ms 时降速；记录 `depth_age_ms`。

### 3.5 任务一验收标准

以下阈值在基线测量后固定；先给出建议目标：

| 维度 | 建议目标 |
|---|---|
| 感知 | RealSense 在 T2 走廊有效率达到 ZED 的 90% 以上；0.5–3 m RMSE ≤5 cm；最远可靠距离 ≥4 m |
| 平面/门/走廊 | RealSense 成功率不低于 ZED 基线 5 个百分点；碰撞率不高于 ZED +1 pp |
| 楼梯 | RealSense 下楼成功率不低于 ZED 基线 5 pp；无新增"拒绝下楼" |
| 低障碍 | 10 cm 障碍在 ≥1.0 m 处可见；碰撞率不高于 ZED +2 pp |
| 效率 | TTG 不高于 ZED +10%；平均速度不低于 ZED 的 90% |
| 安全 | 人工接管率不高于 ZED；最小净空 ≥0.10 m 的 run 占比 ≥95% |
| 可复现 | 同一配置跨两天重复，成功率差异 ≤10 pp |

---

## 4. 任务二：部署参数整定

### 4.1 参数清单与分组

#### A. 基础/状态机（`rl_nav_controller.py`、`constants.py`）

| 参数 | 当前默认 | 作用 | 是否首轮调 |
|---|---:|---|---|
| `control_frequency` | 5.0 Hz | 策略频率，训练对齐 | 否（保持 5 Hz） |
| `lookahead_distance` | 2.0 m | 非自适应/基类前瞻 | 仅作对照 |
| `segment_arrive_threshold` / `arrive_goal_threshold` | 0.20 m | 到点判定 | 是（0.15–0.35） |
| `blocked_timeout` | 4.0 s | 停滞判定 | 是（2–8） |
| `blocked_dist_threshold` | 0.1 m | 停滞进尺 | 是（0.05–0.2） |
| `blocked_vel_threshold` | 0.1 m/s | 停滞速度 | 是（0.05–0.2） |
| `path_start_max_dist` | 2.5 m | 路径起点容差 | 低优先 |
| `camera_timeout` | 2.0 s | 相机丢失看门狗 | 否（安全） |

#### B. 自适应前瞻（`rl_nav_controller_adaptive.py`、`adaptive_path_tracker.py`）

| 参数 | 当前默认 | 搜索范围（建议） | 主要场景/KPI |
|---|---:|---|---|
| `lookahead_min` | **0.40 m** | 0.35–0.55（0.7 已证明毁窄门） | S2/S3，门内速度/碰撞 |
| `lookahead_max` | 2.00 m | 1.50–2.50 | S1，巡航速度/时间 |
| `switch_mode` | **`corner_dist`** | `corner_dist` / `curvature` / `hybrid` | 切换规则，见下 |
| `corner_switch_thresh_deg` | 25° | 20–35 | 航向偏离多少度算拐角 |
| `corner_switch_margin` | 0.25 m | 0.20–0.45 | carrot 停在拐角前多远 |
| `adaptive_window_dist` | 1.60 m | 1.0–2.4 | 仅 `curvature`/`hybrid` 有效 |
| `turn_angle_low_deg` | 20° | 10–30 | S1/S2，假触发 |
| `turn_angle_high_deg` | 50° | 40–70 | S2，收缩幅度 |
| `slew_rate_decrease` | 2.5 m/s | 1.5–4.0 | S2/S3，响应 |
| `slew_rate_increase` | 0.8 m/s | 0.4–1.2 | 出弯/出门，平顺 |
| `passage_latch_dist` | 1.20 m | 0.8–1.8 | S3，出画锁存 |
| `clearance_threshold` | 1.05 m | 0.9–1.3 | S3，门洞判定（需暴露为 ROS 参数） |
| `door_speed_boost` | 0.35（节点）/0.0（部署） | 0–0.45 | S3，门内速度 |

#### C. 楼梯（`rl_nav_controller_stairs.py`、`stairs_depth_processor.py`）

| 参数 | 当前默认 | 搜索范围（建议） | 主要 KPI |
|---|---:|---|---|
| `stair_slope_threshold` | -0.20 | -0.30–-0.12 | 下楼入口提前量/误触发 |
| `stair_lookahead_eval_dist` | 1.20 m | 0.8–1.6 | 门控时序 |
| `stair_exit_slope_threshold` | -0.10 | -0.15–-0.05 | 退出时机 |
| `stair_exit_dist_threshold` | 0.60 m | 0.4–1.0 | 防过早恢复 |
| `stair_min_elevation_drop` | 0.15 m | 0.10–0.25 | 单级台阶确认 |
| `stair_lookahead_dist` | 0.40 m | 0.30–0.60 | 下楼速度/稳定性 |
| `stair_up_lookahead_dist` | 0.45 m | 0.35–0.65 | 上楼 |
| `stair_up_slope_threshold` | 0.20 | 0.12–0.30 | 上楼入口 |
| `stair_up_min_elevation_gain` | 0.15 m | 0.10–0.25 | 上楼确认 |
| `stair_up_exit_slope_threshold` | 0.10 | 0.05–0.15 | 上楼退出 |
| `enable_stair_remapping` | False | False/True | 重映射收益 |
| `stair_ramp_base` | 1.0 m | 0.8–1.4 | 重映射 |
| `stair_ramp_step` | 0.046 m | 0.03–0.07 | 重映射 |
| `stair_abyss_threshold` | 2.2 m | 1.5–3.0 | 深渊判定 |
| `stair_side_min_clamp` | 0.50 m | 0.3–0.7 | 扶手/侧墙 |
| `stair_left_col` / `right_col` | 16 / 48 | 12–20 / 44–52 | 中央走廊 |
| `stair_min_forward_vel` | 0.0 | 0–0.3 | 楼梯速度保底 |

#### D. 安全/舒适（`constants.py`、控制器）

| 参数 | 当前默认 | 搜索范围 | KPI |
|---|---:|---|---|
| `POLICY_SCALE` | [1.5, 1.0, 1.0] | 仅评估，不改训练语义 | 速度/碰撞 |
| `LATERAL_VELOCITY_SCALE` | 0.6 | 0.4–0.8 | 侧移/蛇形 |
| `LOW_PASS_FILTER_COEF` | [0.9, 0.5, 0.5] | 0.7–0.95 / 0.3–0.7 | 平滑/延迟 |
| 楼梯角速度限幅 | ±0.5 rad/s | 0.4–0.7 | 甩尾/扶手碰撞 |

#### E. 相机/预处理（与任务一共用）

| 参数 | 当前 | 搜索范围 |
|---|---|---|
| RealSense `min_depth` / `max_depth` | 0.15 / 10.0 | 0.15–0.30 / 3.0–10.0 |
| ZED `min_depth` / `max_depth` | 0.25 / 10.0 | 0.20–0.30 / 6.0–10.0 |
| FPS / 分辨率 | 15 Hz / 640×480(RS)、640×360(ZED) | 15/30；640/848 |
| 滤波组合 | spatial+temporal+hole | 开关/预设 |
| 无效像素填充 | `min_depth` | `min_depth` / 邻域中值 / 0.0 |
| 裁剪/重采样 | 直接 resize | 中心裁剪 / 虚拟相机 |

### 4.2 场景—KPI 对应矩阵

| 场景 | 主 KPI | 次 KPI | 重点参数组 |
|---|---|---|---|
| S1 长直走廊 | 平均速度、TTG | 速度平滑、假减速次数 | B（L_max、window、angle_low） |
| S2 直角转弯 | 转弯时间、d_min | 切内角、wz 振荡 | B（L_min、angle_high、slew） |
| S3 窄门 | 通过率、通过时间 | 门框碰撞、latch 时序、假 latch | B（L_min、clearance、latch、boost） |
| S4 楼梯下行 | 下楼成功率 | 拒绝下行、pitch/roll、门控时序 | C（slope、eval、stair_L、exit） |
| S5 楼梯上行 | 上行成功率 | 卡住、roll | C（up 参数） |
| S6 密集障碍 | 成功率、碰撞 | 恢复次数、d_min | A（blocked）、B、D |
| S7 低障碍 | 碰撞、探测距离 | d_min | E（相机/预处理）+ A |
| S8 组合路线 | 端到端成功率、TTG | 模式切换次数 | B+C+D |
| S9 终点 | 航向误差、对齐时间 | 漂移、超时 | 终点参数 |
| S10 光照/材质 | 通过率 | 空洞率、接管率 | E + P4 安全层 |

### 4.3 分阶段整定流程

#### Stage 0：离线筛选（不动机器人，1–2 天）

- **路径回放**：用诊断日志中的 `/global_path` 与机器人位姿，离线重算不同（L_min、L_max、window、angle_low/high、slew、latch）下的 `L(t)`、胡萝卜点、`ΔΘ`、latch 状态。
- 检查：
  - 直道是否稳定在 `L_max`，是否被噪声误触发；
  - 90° 弯前是否提前 0.8–1.2 m 开始收缩；
  - 门内是否稳定锁存、出门后是否平滑恢复；
  - 收缩/恢复速率是否产生胡萝卜点瞬移。
- 产出：每个参数的可行域、明显不稳定组合、推荐初值。
- 工具：扩展现有 `scripts/test_adaptive_lookahead.py` 与 `adaptive_path_tracker.py` 的离线模式；输入为 CSV/JSON 路径与位姿。

> Stage 0 只能筛"路径几何层"参数（B 组大部分），不能替代楼梯门控与闭环安全。

#### Stage 1：实机粗调（分组坐标下降，1 周）

1. 固定相机 profile（任务一 P0/P1 结论）。
2. 从默认参数出发，按组调：
   - **组 B1：前瞻几何**（S1/S2/S6）：`lookahead_max` → `adaptive_window_dist` → `turn_angle_high` → `turn_angle_low` → `lookahead_min` → `slew`；
   - **组 B2：窄门**（S3）：`clearance_threshold` → `passage_latch_dist` → `lookahead_min` → `door_speed_boost`；
   - **组 C：楼梯**（S4/S5）：`stair_slope_threshold`/`eval_dist` → `stair_lookahead_dist` → `exit` 参数 → remap；
   - **组 A/D：安全与终点**（S6/S9）：`blocked_*`、终点航向、低通/限幅。
3. 每个参数 3–5 个值，每值 5 次；一次只动一个参数；每组结束后回到该组最优再进下一组。
4. 观察交互：若某参数在两组间最优值冲突，记录下来进入 Stage 2。

#### Stage 2：交互 DOE（1 周）

对强交互组做小规模设计实验：

- **前瞻组**：`lookahead_min × lookahead_max × adaptive_window_dist × turn_angle_high`，2 水平全因子 16 组或 Latin hypercube 12 组；N=10。
- **窄门组**：`clearance_threshold × passage_latch_dist × door_speed_boost`，3×3×3 取 9–12 组；N=10。
- **楼梯组**：`stair_slope_threshold × stair_lookahead_eval_dist × stair_lookahead_dist × stair_exit_dist`，2 水平 16 组；N=10。
- 分析主效应/交互效应；画 Pareto 图；选稳健平台而非尖峰最优点。

#### Stage 3：贝叶斯优化（0.5–1 周，可选）

- 仅对 Stage 2 中 2–3 个最敏感参数做 BO；目标函数带安全约束：
  `maximize SR - λ1·CR - λ2·intervention - λ3·normalized_TTG`，约束 `d_min ≥ 0.10 m`。
- 每轮 10 次试验，期望改进（EI）选点；15–20 轮或连续 2 轮无提升停止。
- 不建议对楼梯参数做纯 BO：安全风险高，优先用物理约束与保守平台。

#### Stage 4：留出验证（1 周）

- 用未参与调参的场景/日期/起点验证；N=15–20。
- 与默认 profile、ZED profile 做配对对比。
- 检查跨天重复性：同配置两天成功率差异 ≤10 pp。

### 4.4 优化目标与停止条件

推荐**字典序**而非加权和：

1. 安全约束：碰撞率 ≤ 基线；人工接管 = 0（名义场景）；`d_min ≥ 0.10 m`；
2. 成功率最大化；
3. TTG 最小化；
4. 平滑度/舒适性。

若必须单一分数：

```
J = 1.0·(1 - SR) + 2.0·CR + 1.0·IR + 0.3·(TTG / TTG_ref) + 0.1·RMS_jerk_norm
```

停止条件（满足任一）：

- 连续 2 个批次（每批 ≥10 次）主指标无显著提升；
- 成功率 95% CI 半宽 <5 pp；
- 参数已处于物理约束边界，继续调整会牺牲安全；
- 达到预定试验预算。

### 4.5 实验执行协议（实机安全）

**每次运行前检查**：

- 电池 >80%，四轮/腿部无松动，镜头干净；
- TF、深度话题、`/lio/robo/odom`、`/local_planner/status` 正常；
- 定位质量（重定位误差/点云匹配）正常；
- 场景标记（起点胶带、门宽、台阶）未变；
- 急停/手柄 B 键可用，守护人员在位；
- 记录环境（光照、地面、人员）。

**运行中**：

- 调参初期速度上限 0.6 m/s（可通过参数/外部限速）；
- 楼梯：安全带/软牵引，先单级台阶验证门控，再整段；
- 窄门：门框包软质材料，禁止玻璃门；
- 任何异常（定位跳变、深度丢失、异常转向）立即接管并标记 run invalid；
- 单次超时 60 s（与训练一致），超时即失败。

**运行后**：

- 保存 bag、CSV、深度 npy、meta、视频；
- 填写 run 元数据（profile、相机、场景、天气/光照、异常）；
- 当天备份并计算指标，避免数据堆积。

### 4.6 配置与版本管理

建议目录：

```
sru-robot-deployment/config/
├── param_registry.yaml          # 参数元数据：默认值、范围、单位、场景、约束
├── profiles/
│   ├── realsense_default.yaml
│   ├── realsense_p0.yaml
│   ├── realsense_v1.yaml
│   ├── zed_default.yaml
│   ├── zed_v1.yaml
│   └── tuning/
│       ├── rs_lookahead_001.yaml
│       └── ...
└── experiments/
    └── 2026-11-xx_baseline/
        ├── plan.yaml
        ├── runs.csv
        └── report.md
```

- profile 只包含**显式覆盖**的参数，其余走默认，避免"复制一份默认值后失同步"。
- 每个 profile 计算 SHA256，写入每次 run 的 meta；报告里引用 hash。
- 启动脚本新增 `--profile <path>`，内部转成 `--ros-args -p key:=value`；也可用 `ros2 param load`。
- 参数注册表同时生成文档表格，避免代码/文档漂移。

### 4.7 任务二验收标准

| 维度 | 建议目标 |
|---|---|
| 安全性 | 所有确认 run 无碰撞、无接管；楼梯无跌落 |
| 成功率 | 关键场景（S1/S2/S3/S4/S8）≥90%，组合路线 ≥85% |
| 效率 | 相比默认 profile，TTG 降低 ≥10% 或平均速度提升 ≥10%，且不牺牲安全 |
| 稳定性 | 同配置跨天成功率差异 ≤10 pp；无参数尖峰依赖 |
| 可解释性 | 每个改动参数有物理依据与消融数据 |
| 回归 | 原有单测（自适应前瞻、终点航向、楼梯门控）全部通过 |

---

## 5. 工具与代码改动清单

### 5.1 `sru-robot-deployment`（P0）

| 文件 | 改动 | 目的 |
|---|---|---|
| `diagnostics/door_diagnostic_logger.py` | **已实现（schema 2.0）**：移除 remap；新增 `camera_id`、深度健康/延迟/分位数、原始深度 PNG、CameraInfo、TF、导航状态、目标/路径快照、LiDAR 净空、控制参数快照；新增 `diagnostics/compare_camera_sessions.py` | L0/L1 数据完整、双 session 对比 |
| `scripts/experiment_runner.py`（新） | 读 profile/场景/相机；启动/等待/发点/监控/急停/记录/重置；随机化与重复 | 自动化 L2 |
| `scripts/analyze_experiments.py`（新） | 汇总 CSV/JSON；算 SR/CR/SPL/TTG/d_min/jerk；配对统计；出图 | 自动报告 |
| `config/param_registry.yaml`（新） | 参数默认/范围/单位/场景/约束 | 单一事实源 |
| `config/profiles/*.yaml`（新） | 默认/v1/调参中间 profile | 可复现 |
| `scripts/start_nav.sh` / `start_local_sru.sh` | `--profile`、`--camera`、统一 TF 输出、记录 profile hash | 启动可复现 |
| `rl_nav_controller/rl_nav_controller/rl_nav_controller.py` | 发布 `depth_health`；记录 raw_action/obs 摘要；可选 `depth_age` 限速 | 诊断/安全 |
| `rl_nav_controller/rl_nav_controller/adaptive_path_tracker.py` | `clearance_threshold` 暴露为 ROS 参数；发布 `L_curv`、`L_clear`、`latch`、`ΔΘ` | 调参可观测 |
| `rl_nav_controller/rl_nav_controller/rl_nav_controller_stairs.py` | 发布 gate 内部量（slope、dz、entry/exit）；所有参数可注入 | 楼梯调参 |
| `rl_nav_controller/rl_nav_controller/depth_adapter.py`（新） | 虚拟相机重投影、填充策略、健康统计 | P1 |

### 5.2 `sru-navigation-learning`（P1/P3）

| 文件 | 改动 | 目的 |
|---|---|---|
| `scripts/benchmark/benchmark_collector.py` | 增加相机/profile/trial 元数据、深度延迟、LiDAR 机体前向最近点、接触事件 | 真值与归因 |
| `scripts/benchmark/benchmark_runner.py` | 场景定义、自动重置、安全包络、随机化 A/B | 自动 L2 |
| `scripts/benchmark/analyze_benchmark.py` | 增加 SPL、jerk、门/楼梯指标、配对统计 | 报告 |
| `scripts/depth_benchmark/`（新） | L0 深度质量分析、L1 ONNX 回放/匹配状态对比 | 分层评测 |
| `IsaacLab/.../config/go2w/navigation_env_cfg.py` | 接入 `go2w_d435` 噪声/FOV 的 D435 评测任务 | P3 |
| `IsaacLab/.../mdp/depth_utils/camera_config.py` | 确认 D435 参数（focal/baseline/max_depth）与实机一致 | P3 |
| `docs/` | 本方案、实验报告、最终参数表 | 文档 |

### 5.3 运行命令示例

```bash
# L0：RealSense 深度质量（在线）
python3 ~/go2w_nav_ros2_ws/test_realsense_max_distance.py \
  --topic /camera/camera/depth/image_rect_raw

# L0：离线分析已录 session
python3 ~/go2w_nav_ros2_ws/test_realsense_max_distance.py \
  --session ~/door_diagnostics/session_xxx

# L2：相机 A/B 录制（先用当前部署参数跑同一段手动路径）
python3 ~/go2w_nav_ros2_ws/door_diagnostic_logger.py \
  --camera realsense --scene S3_door --note "RS run1"
python3 ~/go2w_nav_ros2_ws/door_diagnostic_logger.py \
  --camera zed --scene S3_door --note "ZED run1"

# L2：两个 session 离线对比
python3 ~/go2w_nav_ros2_ws/compare_camera_sessions.py \
  --a ~/door_diagnostics/session_..._realsense_S3_door \
  --b ~/door_diagnostics/session_..._zed_S3_door \
  --label-a RealSense --label-b ZED

# L2：自动化多 profile/多重复（计划中的工具）
python3 ~/sru-robot-deployment/scripts/experiment_runner.py \
  --profile ~/sru-robot-deployment/config/profiles/realsense_p0.yaml \
  --camera realsense --scene S3_door --trials 10 --out ~/runs

# L2：汇总分析（计划中的工具）
python3 ~/sru-robot-deployment/scripts/analyze_experiments.py ~/runs \
  --baseline realsense_default --compare zed_default
```

---

## 6. 实施计划与里程碑

### 6.1 完整版（6 周）

| 周 | 工作 | 产出 |
|---|---|---|
| W1 | 仪器化（logger/runner/registry）；相机标定；L0 台架；基线 L2（2 相机 × 6 场景 × 5 次） | 基线报告、失败分类、数据 schema |
| W2 | P0 预处理/滤波/量程消融；FOV/长宽比实验；差距归因 | RealSense 配置 v1、对比报告 |
| W3 | P1 虚拟相机适配层；S1/S2/S3/S4/S7 A/B；任务二 Stage 0 | 适配层 + 离线筛选结果 |
| W4 | 任务二 Stage 1（前瞻/门/楼梯/终点粗调）；P2 相机专属 profile | profile v1 |
| W5 | Stage 2 交互 DOE；Stage 3 BO；P3 仿真 D435 评测（如需要） | 最优/稳健参数、Pareto |
| W6 | Stage 4 留出验证；最终相机 A/B；文档与默认值更新 | 最终报告、profile、回归测试 |

### 6.2 最小可行版（4 周，时间紧时）

- W1：仪器化 + 基线（S1/S2/S3/S4，2 相机 × 5 次）。
- W2：P0（标定 + 量程/滤波/裁剪） + A/B。
- W3：任务二 Stage 1（只调前瞻/门/楼梯三组）。
- W4：留出验证 + 报告。

**取舍**：暂不做 P1 虚拟相机、P3 微调、BO；只做"标定 + 预处理 + 粗调"，也能拿到可量化结论与可用 profile。

### 6.3 里程碑判定

| 里程碑 | 判定 |
|---|---|
| M1 基线可复现 | 同一配置连续 3 次运行，主指标波动 <10 pp |
| M2 感知差距可解释 | 至少 80% 的失败 run 能归入失败分类 |
| M3 相机配置 v1 达标 | P0 验收表通过 |
| M4 参数 profile v1 达标 | 关键场景 ≥90%，TTG 改善 ≥10% |
| M5 最终验证 | 留出场景/跨天重复通过 |

---

## 7. 风险与对策

| 风险 | 影响 | 对策 |
|---|---|---|
| 实机时间稀缺 | 试验做不完 | 先离线筛选；小 N 初筛 + 大 N 确认；批量运行；预注册指标 |
| 场景/光照变化 | 结论不可复现 | 固定场景标记；交错 A/B；记录环境；跨天验证 |
| 参数过拟合单一场景 | 换场景失效 | 留出场景；优先稳健平台；Pareto 而非单点最优 |
| 楼梯调参危险 | 跌落/损坏 | 安全带、守护、单级先验、速度上限、保留所有看门狗 |
| 相机安装差异混淆结论 | 归因错误 | 实测外参；P1 虚拟相机；必要时双相机同支架 |
| 深度延迟/掉帧 | 碰撞 | 记录 `depth_age`；P4 限速；禁用高延迟滤波 |
| RealSense 远距噪声 | 假障碍/减速 | `max_depth` 收敛到 4–6 m；滤波；P2 profile |
| 仿真微调过拟合仿真 | 实机不升反降 | 仿真仅筛选；实机 A/B 决定是否采用 |
| 数据量 | 存储/分析压力 | 5 Hz、深度压缩、按会话保留；自动清理 |
| 统计功效不足 | 结论不稳 | 配对设计；置信区间；必要时增加 N |

---

## 8. 需要确认的信息

在开工前请确认以下事实（直接影响实验设计）：

1. **RealSense 具体型号**：D435 / D435i / D455？三者基线、FOV、量程不同；D455 基线 95 mm，远距更好。
2. **ZED 是否仍可用**：能否与 RealSense 交替安装/同时安装？能否做一次双相机同支架标定？
3. **实际安装外参**：相机光心离地高度、俯仰角、相对 `base_link` 的 x/y/z；`base_link` 原点定义（机身中心还是地面）。
4. **测试场地**：是否有固定走廊/窄门/长楼梯/低障碍？能否贴标记、封场、放置软质门框？
5. **安全装备**：楼梯守护、软牵引、急停/手柄、场地保险。
6. **算力与时间**：是否允许 P3 的仿真微调（GPU 时长）？每周实机可用小时数？
7. **任务范围**：楼梯上行是否必须？全局导航模式 B 是否纳入评测，还是只测模式 A？
8. **现有基线**：是否有历史 ZED 运行数据/视频可用于回溯对比？

---

## 附录 A：核心指标公式

```
SR   = N_success / N_total
CR   = N_collision / N_total
NMR  = N_near_miss / N_total,  near_miss: d_min < 0.10 m
SPL  = (1/N) Σ_i S_i · l_i / max(p_i, l_i)
TTG  = t_goal - t_start
RMS_jerk = sqrt(mean((Δa/Δt)^2))
d_min = min_t min_{p ∈ P_front(t)} ||p||_xy
```

感知：

```
bias   = mean(d_meas - d_ref)
RMSE   = sqrt(mean((d_meas - d_ref)^2))
P95err = percentile(|d_meas - d_ref|, 95)
valid_ratio = N_valid / N_total
hole_ratio  = N_zero_or_nan_or_inf / N_total
temporal_std = std_t(d_pixel)
D_blind = h_cam / tan(θ_bottom)
h_visible(d) = h_cam - d · tan(θ_bottom)
```

统计：

```
配对成功率：McNemar test
配对连续量：Wilcoxon signed-rank + bootstrap 95% CI
效应量：Cliff's delta / Cohen's d
```

---

## 附录 B：建议新增日志字段

在现有 46 列 CSV 基础上追加：

| 字段 | 说明 |
|---|---|
| `run_id` / `trial_id` | 唯一运行/试验 ID |
| `camera_id` | `realsense_d435` / `zed2i` |
| `profile_id` / `profile_hash` | 参数 profile |
| `commit_hash` | 代码版本 |
| `scene_id` / `scene_phase` | 场景与阶段 |
| `depth_age_ms` | 深度帧年龄 |
| `depth_valid_ratio` / `depth_hole_ratio` | 深度健康 |
| `depth_fill_ratio` | 预处理填充比例 |
| `raw_vx` / `raw_vy` / `raw_wz` | 网络原始输出（滤波前） |
| `gravity_x/y/z` | 观测中的重力向量 |
| `last_action_vx/vy/wz` | 上一动作 |
| `obs_hash` | 观测摘要（可选） |
| `lidar_min_dist_front` / `lidar_near_x/y/z` | LiDAR 真值 |
| `contact_event` | 碰撞/接触事件 |
| `intervention` | 人工接管标记 |
| `battery_voltage` | 电池 |
| `operator` / `notes` | 运行备注 |

---

## 附录 C：参数调优优先级（建议）

| 优先级 | 参数组 | 理由 |
|---|---|---|
| P0 | 相机标定、TF、`min/max_depth`、无效像素填充、滤波 | 影响所有场景，先固定 |
| P1 | `lookahead_min/max`、`adaptive_window_dist`、`turn_angle_*` | 影响直道/转弯/门，收益最大 |
| P2 | `clearance_threshold`、`passage_latch_dist`、`door_speed_boost` | 窄门专项 |
| P3 | 楼梯 slope/eval/lookahead/exit | 安全关键，需守护 |
| P4 | `blocked_*`、终点航向、低通/限幅 | 稳定性/舒适性 |
| P5 | 重映射参数、`stair_min_forward_vel` | 收益不确定，最后做 |

---

## 附录 D：与现有文档的关系

- 相机硬件/驱动细节：`sru-robot-deployment/docs/GO2W_CAMERA_CONFIGURATION_AND_INTEGRATION_GUIDE.md`
- 自适应前瞻算法：`docs/structure/ADAPTIVE_LOOKAHEAD.md`
- 全局导航接口：`sru-robot-deployment/docs/GLOBAL_NAV_INTERFACE.md`
- 实机部署流程：`sru-robot-deployment/docs/GO2W_DEPLOYMENT_GUIDE.md`
- 仿真接入：`sru-robot-deployment/docs/GO2W_SIM_HANDOFF.md`
- 评测工具：`scripts/benchmark/README.md`

本文不重复上述文档的硬件细节，只定义**如何测量、如何对比、如何调参、如何验收**。

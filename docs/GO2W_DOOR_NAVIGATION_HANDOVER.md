# Unitree Go2W 狭窄通道（小门）穿行与动态前瞻实机导航交接文档

---

## 1. 文档概述与背景

* **项目名称**：Unitree Go2W 轮足机器人全局路径跟踪与 SRU 端到端避障导航系统
* **机载硬件平台**：Jetson Orin DevKit (Ubuntu 22.04 / ROS 2 Humble)
* **核心软件栈**：
  * 激光雷达驱动：Livox MID-360 (`livox_ros_driver2`)
  * SLAM 定位与建图：Super-LIO (`map_localization.py`)
  * 深度视觉感知：ZED 2i 相机 (`zed_wrapper`)
  * 底盘控制通信：`go2w_bridge`
  * 全局路径规划：`global_planner`（广播 `/global_path` 与 `/global_planner/raw_map`）
  * 局部端到端策略：`rl_nav_controller`（基于 SRU/LSTM 视觉强化学习策略网络）
* **编写目的**：记录实机调试中遇到的“小门拒绝通行与探索性过强”问题、验证成功的实验现象、底层核心机理与矛盾分析、完整的实机操作规范，以及后续接交人员可直接落地的改进方案。

---

## 2. 问题现象与实验验证结论

### 2.1 初始问题现象（前瞻距离 $L = 2.0\text{m}$）
在实机多障碍大场景导航中，全局规划器规划了一条准确穿过狭小门洞（开阔宽度约 $0.8\text{m} \sim 0.9\text{m}$）的可行路径。然而机器狗运动到门前约 $0.5\text{m} \sim 1.0\text{m}$ 位置时，**出现严重的“拒绝进门”与过度探索行为**：
- 机器狗突然放弃对全局路径的跟踪；
- 机身开始大幅打舵转向，在门前开阔区反复绕圈、寻找旁支通道漫游；
- 若无人工干预，机器狗绝不进入小门，判定此处为不可通行区域。

### 2.2 关键实验验证结论（调整前瞻距离为 $L = 0.5\text{m}$）
通过开发支持命令行动态前瞻控制的启动脚本 [`start_nav_custom.sh`](file:///home/yichangfeng/sru-navigation-learning/scripts/start_nav_custom.sh)，将前瞻胡萝卜点距离调整为 **$0.5\text{m}$** 再次测试：
$$\text{命令：}\quad \texttt{./start_nav_custom.sh 0.5}$$
* **实验结果**：**机器狗成功且平稳地正对门洞，顺畅穿过了狭小门！**
* **归因证实**：该实验现象**100% 证实**了此前拒绝进门的核心根因——**并非传感器故障或硬件损坏，而是前瞻距离（Lookahead Distance）与狭窄几何环境之间的矛盾**。

### 2.3 伴生次生问题（当前的核心技术矛盾）
虽然 $L = 0.5\text{m}$ 解决了小门穿行问题，但固定小前瞻引出了严重的系统级负面效应：
1. **行进速度极慢（Speed Degradation）**：
   - 强化学习策略网络的线速度输出与机身坐标系下目标相对距离 $\ln(\text{dist} + 1.0)$ 高度正相关。当目标点始终锁定在眼前仅 $0.5\text{m}$ 时，策略网络“认为目标已近在咫尺”，自适应输出的前进线速度被极度压低（线速度由正常的 $0.7 \sim 0.9\text{ m/s}$ 骤降到 $0.15 \sim 0.25\text{ m/s}$），整体巡检通行效率极低。
2. **几乎丧失开阔区动态避障能力（Loss of Obstacle Avoidance Horizon）**：
   - 端到端强化学习视觉避障的核心机理，是利用**决策视界（Planning Horizon）**感知“机器人与目标点之间”的障碍分布，并提前计算平滑绕障动作弧线。
   - 现实环境中大量动态/静态障碍物（如行人、箱体、立柱）本身的物理尺寸就超过 $0.5\text{m}$。
   - 当前瞻点仅为 $0.5\text{m}$ 时，目标点往往落在了障碍物近前端甚至机身下部视野盲区，机器人直奔 $0.5\text{m}$ 的目标而去，等到到达目标点时，距真实障碍物已不足 $0.2\text{m}$。此时即使网络觉察到障碍，由于运动惯性与侧向空间受限，根本无法完成转弯绕行，极易发生直接刮蹭或逼停。

---

## 3. 核心矛盾与深层机理解剖

```
                                  前瞻距离两难困境 (Dilemma)
                                              │
              ┌───────────────────────────────┴───────────────────────────────┐
              ▼                                                               ▼
   【小前瞻模式：L = 0.5m】                                        【大前瞻模式：L = 2.0m】
  优点：胡萝卜点落在门缝正中央，                                  优点：决策视界开阔，提前预判绕障，
        目标向量笔直穿门，不撞门框。                                    线速度拉满 (0.8~1.0 m/s)。
  缺点：速度极慢 (0.2 m/s)，视界过窄，                            缺点：进门前目标穿透门后落在侧墙，
        障碍物还没看清就已撞上。                                        引发严重碰撞恐慌与掉头逃逸。
```

### 3.1 为什么大前瞻（$L = 2.0\text{m}$）在门前会“拒进门”？
1. **“隔墙指路”效应**：
   - 设机器狗当前在门外 $0.5\text{m}$ 处，若 $L = 2.0\text{m}$，前瞻胡萝卜点已延伸至门后 $1.5\text{m}$。
   - 只要机器狗在进门前机头稍微有 $10^\circ \sim 15^\circ$ 的微小偏角，在机身基坐标系（Base Link）下投影的目标向量就会直接穿过门框侧面的厚墙。
2. **价值函数（Value Function）碰撞惩罚压制**：
   - 端到端导航策略在仿真中对“撞墙/碰撞”设置了极高的惩罚项（例如 $-10$ 或直接 Terminate）。
   - 当目标向量指向墙壁后方，而视觉深度图在前方 $0.5\text{m}$ 处看到了坚硬的门框侧墙，网络评估出“向前直冲的碰撞风险巨大”，此时“向侧后方大角度打舵寻找开阔地绕行”在数学期望上收益更高，从而表现为“放弃路径、向其他地方探索”。

### 3.2 为什么小前瞻（$L = 0.5\text{m}$）会导致“速度极慢且无法避障”？
1. **状态输入中的对数距离缩减**：
   - 观察量中包含 $[d_x, d_y, d_z, \ln(\text{dist} + 1.0)]$。当 $\text{dist} = 0.5$ 时，$\ln(1.5) \approx 0.405$；而当 $\text{dist} = 2.0$ 时，$\ln(3.0) \approx 1.098$。距离特征缩减近 3 倍，网络自然收敛至终末逼近减速行为。
2. **感知前瞻与物理避障死区冲突**：
   - ZED 2i 相机的最小盲区为 $0.25\text{m}$，若目标仅在 $0.5\text{m}$ 处，留给感知与控制回环进行反应的有效空间纵深仅剩 $0.25\text{m}$。以机器狗车体长度（约 $0.6\text{m}$）计算，当发现障碍物时，机身已经处于不可逆的碰撞几何区。

---

## 4. 实机实验操作与调试流程规范

### 4.1 代码同步操作（已剔除 RViz 冗余编译）
在主机开发环境终端，运行专属一键同步脚本：
```bash
# 自动同步纯 Python 控制器与启动脚本至狗机 (默认 IP: 10.3.42.161)
bash scripts/sync_to_go2w_nav.sh 10.3.42.161
```
* **注意**：脚本已剔除 `rviz.rviz` 与任何可视化大包的推送，并在狗机上严格执行：
  ```bash
  colcon build --packages-select rl_nav_controller --symlink-install
  ```
  该命令在 1~2 秒内秒级完成，**坚决不会触发 `rviz_tools` 等大型 C++ 插件的漫长重编**。

### 4.2 启动导航节点（动态控制前瞻）
登录机器狗车载终端：
```bash
ssh agile@10.3.42.161
cd ~/go2w_nav_ros2_ws
```
根据测试场景选择前瞻距离：
```bash
# 场景 A: 专测小门通过性（小前瞻 0.5m ~ 0.8m）
./start_nav_custom.sh 0.5

# 场景 B: 推荐折中验证（0.8m ~ 1.0m，兼顾过门与低速避障）
./start_nav_custom.sh 0.8

# 场景 C: 开阔区高速大范围巡航（默认 2.0m）
./start_nav_custom.sh 2.0
```

### 4.3 实机手柄操作规范
- **`[按钮 A (Button 0)]`**：启动 / 恢复导航（Start / Resume）。
  - 开闸底盘使能并恢复路径跟踪。
- **`[按钮 B (Button 1)]`**：暂停 / 紧急刹停（Pause / Stop）。
  - 机器狗立即主动制动刹停（`cmd_vel = 0`），并在内存中安全保留当前未走完的全局路径，防止失控。

### 4.4 停止系统
在任一终端运行：
```bash
cd ~/go2w_nav_ros2_ws && ./stop_nav.sh
```

### 4.5 运行诊断记录器（抓取 40×64 深度图与热力图）
若需复现或录制进门瞬间网络眼里的世界（诊断感知是否将门洞抹平、前瞻目标向量是否被墙体拉偏）：
```bash
ssh agile@10.3.42.161

# 方式 1：默认启动（按当前时间戳自动创建独立会话文件夹）
python3 ~/go2w_nav_ros2_ws/door_diagnostic_logger.py

# 方式 2：附带会话标签启动（推荐！如记录 0.5m 成功通过与 2.0m 绕路失败的对照试验）
python3 ~/go2w_nav_ros2_ws/door_diagnostic_logger.py pass_0.5m
python3 ~/go2w_nav_ros2_ws/door_diagnostic_logger.py fail_2.0m
```

#### 数据存储结构与分析：
每次录制都会在 `~/door_diagnostics/` 下生成独立的 `session_YYYYMMDD_HHMMSS[_tag]/` 目录，并自动建立 `~/door_diagnostics/latest` 快捷软链接：
* **`heatmaps/*.png`**：直观伪彩色深度热力图（320×200），图上标有门洞中央检测窗口（矩形框）、前瞻目标方向与夹角（白色指示箭头）、实时控制速度 `[vx, wz]` 以及门洞正中深度。
* **`depth/*.npy`**：网络实际输入的 $40\times 64$ 原始浮点深度矩阵，用于离线送入 VAE 评估隐空间表征（Latent Vectors）。
* **`meta/*.json`**：单帧里程计位姿、全局前瞻点坐标、相对机器人航向角误差等诊断元数据。
* **`trajectory.csv`**：**全过程时序一览表**（包含时间戳、机器狗位姿、前瞻点坐标、距角、控制指令、门中净深）。可直接复制到电脑端用 Excel 或 Python Pandas 快速绘制曲线。
* **`session_summary.json`**：本次会话记录的总帧数、录制时长、平均帧率与丢包统计。

> [!TIP]
> 记录器内部采用**异步非阻塞写盘队列**与**目标离线保护**：即使目标点短暂丢失或停滞，深度图仍会持续录制；且由于磁盘 I/O 移至后台线程，完全消除了因写盘延迟导致的 ROS 掉帧断流。按 `Ctrl+C` 即可优雅退出并保存完整概况。

---

## 5. 推荐技术演进与解决路径（给接交人员的方案）

为了彻底解决“既要以 $2.0\text{m}$ 保持开阔区高速避障，又要以 $0.6\text{m}$ 顺畅穿门”的矛盾，推荐后续开发人员按以下优先级实施优化：

### 方案 1：基于局部净空横截面的自适应动态前瞻距离（工程推荐优先级：最高 ★★★★★）
* **原理**：无需重新训练网络，在 [`rl_nav_controller.py`](file:///home/yichangfeng/sru-navigation-learning/rl_nav_controller.py) 的前瞻提取模块加入**空间净空（Clearance）自适应函数**。
* **实现逻辑**：
  1. 订阅激光雷达局部点云或读取 ZED 深度图正前方横向障碍距离；
  2. 计算当前通道可用净空宽度 $W_{\text{clearance}}$（例如门洞两侧立柱间距）：
     $$L = \begin{cases} 
     0.6\text{ m}, & W_{\text{clearance}} \le 1.0\text{ m (检测到进入窄道/门洞)} \\
     2.0\text{ m}, & W_{\text{clearance}} \ge 2.0\text{ m (处于开阔环境)} \\
     0.6 + 1.4 \times \frac{W_{\text{clearance}} - 1.0}{1.0}\text{ m}, & \text{其他（平滑线性插值过渡）}
     \end{cases}$$
* **效果**：在开阔平原全速巡航避障；一靠近小门自动平滑收缩前瞻点，贴在门洞正中顺畅穿过，出门后自动拉升恢复高速。

### 方案 2：全局规划器路径属性赋权（Semantic Path Tagging）（优先级：★★★★☆）
* **原理**：修改上层 `global_planner`。
* **实现逻辑**：全局规划器基于先验 PCD 地图已知哪些坐标是狭窄门洞。规划路径时，在门洞前后的 `nav_msgs/msg/Path` 点集添加标记（或将路径点间距加密至 $0.05\text{m}$）。局部控制器若检测到当前路径处于“门洞走廊区域”，临时切换为小前瞻跟踪模式。

### 方案 3：强化学习模型微调（Fine-Tuning with Narrow Passages）（优先级：★★★☆☆）
* **原理**：在 Isaac Lab 仿真环境中针对性增加门洞地形。
* **实现逻辑**：
  1. 在障碍物生成器中加入开口宽为 $0.7\text{m} \sim 0.9\text{m}$ 的双侧障碍墙；
  2. 适度降低极近距离侧向微弱贴合的惩罚权重；
  3. 微调让策略网络学会“两边虽然都是障碍，只要正中无障碍，依然可以全速穿越”。

---

## 6. 核心代码与工程文件映射清单

| 文件路径（开发机） | 机载路径（agile@10.3.42.161） | 职责说明 |
|:---|:---|:---|
| [`rl_nav_controller.py`](file:///home/yichangfeng/sru-navigation-learning/rl_nav_controller.py) | `~/go2w_nav_ros2_ws/src/rl_nav_controller/rl_nav_controller/rl_nav_controller.py` | 核心强化学习避障与路径跟踪节点（包含前瞻提取、容错不上报） |
| [`scripts/constants.py`](file:///home/yichangfeng/sru-navigation-learning/scripts/constants.py) | `~/go2w_nav_ros2_ws/src/rl_nav_controller/rl_nav_controller/constants.py` | 手柄按键映射（A=启动，B=暂停刹停）与参数基准 |
| [`scripts/rl_nav_controller.launch.py`](file:///home/yichangfeng/sru-navigation-learning/scripts/rl_nav_controller.launch.py) | `~/go2w_nav_ros2_ws/src/rl_nav_controller/launch/rl_nav_controller.launch.py` | ROS 2 Launch 启动脚本（支持 `lookahead_distance` 与 `report_failure` 传参） |
| [`scripts/start_nav_custom.sh`](file:///home/yichangfeng/sru-navigation-learning/scripts/start_nav_custom.sh) | `~/go2w_nav_ros2_ws/start_nav_custom.sh` | 自定义前瞻距离 & 容错启动脚本（默认后台多节点 tmux 架构） |
| [`scripts/door_diagnostic_logger.py`](file:///home/yichangfeng/sru-navigation-learning/scripts/door_diagnostic_logger.py) | `~/go2w_nav_ros2_ws/door_diagnostic_logger.py` | 窄门导航诊断记录工具（保存 40x64 热力图与目标向量） |
| [`scripts/sync_to_go2w_nav.sh`](file:///home/yichangfeng/sru-navigation-learning/scripts/sync_to_go2w_nav.sh) | - | 开发机一键向狗机推流同步与自动化构建脚本 |
| [`docs/GO2W_DOOR_NAVIGATION_HANDOVER.md`](file:///home/yichangfeng/sru-navigation-learning/docs/GO2W_DOOR_NAVIGATION_HANDOVER.md) | - | 本技术交接文档 |

---

## 7. 实机消融测试最新进展与问题追踪 (2026-11-06)

* **记录时间**：`2026-11-06 18:02:09`
* **测试场景**：电梯间往返路段（包含直道巡航、转弯过窄门、长楼梯混合工况）
* **消融测试核心结论**：
  1. **`--curv-only` 模式（仅 Path 曲率转弯判断）**：
     - **直道避障效果明显更好**：前瞻稳定保持 $2.0\text{m}$，决策视界充分，且 100% 免疫走廊墙壁与立柱噪点带来的假门洞误锁存问题；
     - **转弯穿窄门表现**：对转弯过窄门稍有负面影响，**容易发生碰/刮蹭门框**，需重新审视出弯恢复时机与曲率前瞻阈值；
  2. **`--door-only` 模式（仅视觉窄门净空判断）**：
     - **直道避障困难**：走廊两侧及前方物体极易误触发窄门检测与空间锁存，前瞻持续被压在 $0.5\text{m}$，网络视距不足导致难以躲避直道障碍；
  3. **长楼梯地形**：
     - **两种模式均仍无法下长楼梯**（下视视野断崖与高程阶梯步态缺失，需专门楼梯步态与感知处理接管）。
* **详细算法与分析记录**：详见技术设计规范文档 [`docs/ADAPTIVE_LOOKAHEAD_NAVIGATION_DESIGN.md`](file:///home/yichangfeng/sru-navigation-learning/docs/ADAPTIVE_LOOKAHEAD_NAVIGATION_DESIGN.md) 第 9 节。

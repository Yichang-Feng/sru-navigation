# Unitree Go2W 导航避障实机标准化评测与诊断套件

本套件用于对 Unitree Go2W 强化学习端到端导航策略进行**量化、可复现、标准化的实机评测与缺陷归因诊断**。

---

## 一、 当前策略两大痛点的代码级机理定位

结合 `~/go2w_robot_ref`（实机部署镜像）与仿真环境配置代码，现定位导致 **“低矮物体不避障”** 与 **“近距离不避障”** 的关键原因如下：

### 1. 近距离不避障：深度图截断逻辑的“零值陷阱”与传感器物理死区
- **代码位置**：[`rl_nav_controller.py`](file:///home/yichangfeng/sru-navigation-learning/sru-robot-deployment/go2w_nav_ros2_ws/src/rl_nav_controller/rl_nav_controller/rl_nav_controller.py#L496-L500)
  ```python
  depth_array = np.nan_to_num(depth_array, nan=0.0, posinf=self.max_depth * 2.0, neginf=0.0)
  depth_array[depth_array > self.max_depth] = 0.0
  depth_array[depth_array < self.min_depth] = 0.0
  ```
- **机理剖析**：
  1. **物理死区与立体匹配失效**：ZED 2i 双目基线为 12cm，物理测距极限盲区约为 $0.25\sim 0.35\text{m}$。当障碍物距离机头小于 0.3m 时，双目视差失配，相机抛出大量 `NaN` 或 `< 0.25m` 的数值。
  2. **致命语义歧义（0.0 歧义）**：代码将 `> max_depth`（10米以上极其开阔的远景）和 `< min_depth`（贴脸极其危险的障碍）**全部赋值为 `0.0`**！
  3. **策略决策失误**：在 VAE 和策略网络眼中，障碍物越逼近机头，障碍物区域反而大面积“溶解”变成了代表无限远空旷的 `0.0`，策略误以为前方畅通无阻，进而全速推杆前冲撞击。
  4. **动力学响应延迟**：策略运行在 5Hz（周期 200ms），加上动作低通滤波系数 `[0.9, 0.5, 0.5]`。机器狗以 $1.0\sim 1.5\text{m/s}$ 运动时，制动延迟距离至少需 $0.8\sim 1.0\text{m}$。一旦在 $0.5\text{m}$ 以内才触发决策，物理碰撞不可避免。

### 2. 低矮物体不避障：水平俯仰角平视导致视场盲锥 + 40×64 分辨率抹平
- **代码位置**：
  - 实机启动脚本 [`start_go2w_system.sh:174`](file:///home/yichangfeng/sru-navigation-learning/sru-robot-deployment/scripts/start_go2w_system.sh#L174)：
    `ros2 run tf2_ros static_transform_publisher 0.257 0.0 0.28 0.0 0.0 0.0 base_link zed_camera_link`（俯仰角 Pitch = 0.0，完全平视）。
  - 仿真相机配置 [`navigation_env_cfg.py:123`](file:///home/yichangfeng/sru-navigation-learning/IsaacLab/source/isaaclab_nav_task/isaaclab_nav_task/navigation/config/go2w/navigation_env_cfg.py#L123)：
    `offset.rot = (1.0, 0.0, 0.0, 0.0)`（水平 0°）。
- **机理剖析**：
  1. **几何地面视场盲锥**：Go2W 站立高度约 0.38m，相机离地高度约 0.47m。ZED 2i 垂直视场角为 $60^\circ$（向下视角 $\approx 30^\circ$）。
     - 地面视野盲区前向距离：$D_{\text{blind}} = 0.47 / \tan(30^\circ) \approx \mathbf{0.81\text{米}}$。
     - 高度为 10cm 的障碍物：距机头约 $\mathbf{0.64\text{米}}$ 处便已彻底**从相机画面底部滑出视野**！
  2. **双线性降采样平滑（40×64）**：
     预处理直接用 `cv2.resize` 将图像压缩为 40 行 × 64 列。一个 10cm 的障碍物在 2 米开外在 40 行中仅占 1~2 个像素行，极易被双线性插值融入周围地面深度。
  3. **训练地形缺乏孤立低矮杂物**：
     检查 `go2w_stage_terrains.py` 可见，Phase 1~2 课程地形主要为高耸迷宫墙面（1m+），无平地孤立矮块（5~15cm）。

---

## 二、 测试与数据采集套件组件

本套件包含三个核心脚本，位于 `scripts/benchmark/`：

1. **[`benchmark_collector.py`](file:///home/yichangfeng/sru-navigation-learning/scripts/benchmark/benchmark_collector.py)**：
   - 部署在机载电脑（Jetson `111`）或局域网开发机上的 ROS 2 实时数据记录节点。
   - 实时同步记录：高精度定位里程计、`/cmd_vel` 指令、ZED 深度图盲区/零值统计、MID-360 激光雷达计算出的机头前方最近障碍物欧氏距离（DTO - Ground Truth）。
   - 自动生成按时间戳命名的 CSV 时序表与 JSON 元数据。

2. **[`benchmark_runner.py`](file:///home/yichangfeng/sru-navigation-learning/scripts/benchmark/benchmark_runner.py)**：
   - 自动化实验调度执行器。
   - 协调发点、计时、软急停保护、终点触达判定、碰撞计数、SPL 计算。

3. **[`analyze_benchmark.py`](file:///home/yichangfeng/sru-navigation-learning/scripts/benchmark/analyze_benchmark.py)**：
   - 离线分析与画图脚本（支持在开发机 Python/Conda 环境下独立运行，无需 ROS 2 依赖）。
   - 自动计算量化指标表格（成功率、碰撞率、SPL、反应距离 $D_{\text{react}}$、最小安全裕度 $d_{\min}$、危险区侵入时间比）。
   - 自动绘制诊断应力图（障碍物距离 vs 下发速度、近距离零值死区缺陷比对）。

---

## 三、 实机测试与数据采集执行步骤

### 步骤 1：启动全机基础感知与策略节点
在 Jetson `111`（或通过 NoMachine 终端）执行常规一键启动：
```bash
cd ~/sru-navigation-learning/scripts
bash start_go2w_system.sh
```

### 步骤 2：启动实时数据记录器
在另一个终端启动记录器（建议将输出保存到 `~/benchmark_logs`）：
```bash
# 激活 ROS 2 环境
source /opt/ros/humble/setup.bash
source ~/sru_ws/install/setup.bash

# 运行记录节点
python3 ~/sru-navigation-learning/scripts/benchmark/benchmark_collector.py \
    --test-name low_obs_10cm \
    --obstacle-height 0.10 \
    --obstacle-distance 3.0 \
    --output-dir ~/benchmark_logs
```

### 步骤 3：执行标准化测试套件
在第三个终端触发自动化测试用例，例如单障正向逼近测试（重复 3 次）：
```bash
python3 ~/sru-navigation-learning/scripts/benchmark/benchmark_runner.py \
    --suite frontal_approach \
    --obstacle-height 0.10 \
    --distance 3.5 \
    --trials 3 \
    --output ~/benchmark_logs/summary_low_obs_10cm.json
```

### 步骤 4：离线出具量化分析报表与诊断图
在外部开发电脑（或已安装 matplotlib 的环境）执行：
```bash
conda activate sru-learning
python scripts/benchmark/analyze_benchmark.py ~/benchmark_logs/ --output-dir ~/benchmark_results/
```
终端将自动输出量化评价表格，并在 `~/benchmark_results/` 下生成诊断曲线图。

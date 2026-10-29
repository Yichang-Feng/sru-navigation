#!/usr/bin/env bash
# ==============================================================================
# Unitree Go2W 自主避障导航全系统一键启动脚本 (start_go2w_system.sh)
#
# 适用硬件：高层核心机 Jetson Orin DevKit (192.168.123.111, 用户: agile)
# 适用环境：Ubuntu 22.04 / ROS 2 Humble (通过 NoMachine 远程桌面或本地终端运行)
#
# 启动内容：
#   [1] unitree_bridge (底盘控制桥接，自动检测自启状态)
#   [2] super_lio relocation.py (Livox MID-360 雷达驱动 + SLAM 重定位 + 自动拉起 RViz)
#   [3] zed_camera (ZED 2i 双目深度相机感知)
#   [4] static TF (base_link -> zed_camera_link 水平相机外参标定)
#   [5] rl_nav_controller (SRU 强化学习端到端避障导航策略节点)
#
# 退出机制：按 Ctrl+C 自动安全关闭并终止所有后台进程
# ==============================================================================

set -e

# 颜色输出定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

echo -e "${CYAN}================================================================${NC}"
echo -e "${CYAN}      Unitree Go2W 自主避障导航全系统一键启动器 (One-Click Launch)  ${NC}"
echo -e "${CYAN}================================================================${NC}"

# 1. 加载 ROS 2 及机载工程环境
echo -e "${BLUE}[1/5] 加载 ROS 2 环境与工程工作空间...${NC}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 自动定位 ZED 导航轻量化覆盖配置文件
ZED_CONFIG_OVERRIDE=""
CANDIDATE_CONFIGS=(
    "${SCRIPT_DIR}/zed_nav_override.yaml"
    "$HOME/sru_ws/zed_nav_override.yaml"
    "$HOME/bridge_ws/zed_nav_override.yaml"
    "$HOME/realworld/zed_nav_override.yaml"
    "$HOME/zed_nav_override.yaml"
)
for cfg in "${CANDIDATE_CONFIGS[@]}"; do
    if [ -f "$cfg" ]; then
        ZED_CONFIG_OVERRIDE="$cfg"
        break
    fi
done

# 若外部未找到预置 yaml，则自动生成临时配置，确保 100% 即插即用免维护
if [ -z "$ZED_CONFIG_OVERRIDE" ] || [ ! -f "$ZED_CONFIG_OVERRIDE" ]; then
    ZED_CONFIG_OVERRIDE="/tmp/zed_nav_override.yaml"
    cat << 'EOF' > "$ZED_CONFIG_OVERRIDE"
/**:
    ros__parameters:
        general:
            grab_resolution: 'HD720'
            grab_frame_rate: 15
            pub_frame_rate: 15.0
            pub_resolution: 'CUSTOM'
            pub_downscale_factor: 2.0
        video:
            publish_rgb: false
            publish_left_right: false
            publish_raw: false
            publish_gray: false
            publish_stereo: false
        depth:
            depth_mode: 'NEURAL_PLUS'
            depth_stabilization: 0
            publish_depth_map: true
            publish_point_cloud: false
            publish_depth_info: false
            publish_depth_confidence: false
            publish_disparity: false
        pos_tracking:
            pos_tracking_enabled: false
            publish_tf: false
            publish_map_tf: false
            publish_odom_pose: false
            publish_cam_path: false
            area_memory: false
        sensors:
            publish_imu: false
            publish_imu_tf: false
EOF
fi

source /opt/ros/humble/setup.bash 2>/dev/null || true
if [ -f "$HOME/unitree_ros2/setup.sh" ]; then
    source "$HOME/unitree_ros2/setup.sh" 2>/dev/null || true
fi
if [ -f "$HOME/bridge_ws/install/setup.bash" ]; then
    source "$HOME/bridge_ws/install/setup.bash" 2>/dev/null || true
fi
if [ -f "$HOME/sru_ws/install/setup.bash" ]; then
    source "$HOME/sru_ws/install/setup.bash" 2>/dev/null || true
fi

# 记录所有后台进程 PID
PIDS=()

# 退出清理函数
cleanup() {
    echo -e "\n${YELLOW}[!] 接收到退出信号，正在安全停止所有后台节点...${NC}"
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null || true
        fi
    done
    sleep 1
    # 强制清理可能残留的节点进程
    pkill -f "relocation_node" 2>/dev/null || true
    pkill -f "livox_ros_driver2_node" 2>/dev/null || true
    pkill -f "zed_wrapper" 2>/dev/null || true
    pkill -f "rl_nav_controller" 2>/dev/null || true
    pkill -f "static_transform_publisher.*zed_camera_link" 2>/dev/null || true
    echo -e "${GREEN}[✓] 全系统所有节点已安全终止。${NC}"
    exit 0
}

# 注册捕获信号
trap cleanup SIGINT SIGTERM EXIT

# 2. 检查或启动底盘桥 unitree_bridge
echo -e "\n${BLUE}[2/5] 检查底盘通信桥 (unitree_bridge)...${NC}"
if ros2 node list 2>/dev/null | grep -q "unitree_bridge"; then
    echo -e "${GREEN}[✓] 检测到 unitree_bridge 已在后台运行，无需重复启动。${NC}"
else
    echo -e "${YELLOW}[i] 未检测到自启的 unitree_bridge，正在启动...${NC}"
    ros2 run unitree_bridge unitree_bridge > /tmp/unitree_bridge.log 2>&1 &
    BRIDGE_PID=$!
    PIDS+=($BRIDGE_PID)
    sleep 1
    if kill -0 "$BRIDGE_PID" 2>/dev/null; then
        echo -e "${GREEN}[✓] unitree_bridge 启动成功 (PID: $BRIDGE_PID)${NC}"
    else
        echo -e "${RED}[✗] unitree_bridge 启动失败，请检查 /tmp/unitree_bridge.log${NC}"
    fi
fi

# 3. 启动 Livox MID-360 雷达、Super LIO 重定位与自带 RViz (rviz:=true)
echo -e "\n${BLUE}[3/5] 启动 Livox MID-360、Super LIO 重定位并拉起预调 RViz (rviz:=true)...${NC}"
ros2 launch super_lio relocation.py rviz:=true > /tmp/super_lio.log 2>&1 &
SUPER_LIO_PID=$!
PIDS+=($SUPER_LIO_PID)
echo -e "${GREEN}[✓] Super LIO 与 RViz 已在后台拉起 (PID: $SUPER_LIO_PID)${NC}"
echo -e "${CYAN}    等待 4 秒等待雷达点云与地图加载...${NC}"
sleep 4

# 4. 启动 ZED 2i 双目深度相机 (加载轻量级导航专属配置)
echo -e "\n${BLUE}[4/5] 启动 ZED 2i 双目深度相机感知 (纯深度图加速模式)...${NC}"
if [ -n "$ZED_CONFIG_OVERRIDE" ] && [ -f "$ZED_CONFIG_OVERRIDE" ]; then
    echo -e "${GREEN}[✓] 已加载轻量化覆盖配置: ${ZED_CONFIG_OVERRIDE}${NC}"
    echo -e "${CYAN}    (已关闭 RGB/点云/视觉SLAM/IMU，深度模式: NEURAL，预期 15Hz 稳定满频)${NC}"
    ros2 launch zed_wrapper zed_camera.launch.py \
        camera_model:=zed2i \
        ros_params_override_path:="$ZED_CONFIG_OVERRIDE" > /tmp/zed_camera.log 2>&1 &
else
    echo -e "${YELLOW}[!] 未指定覆盖配置，使用默认参数启动相机...${NC}"
    ros2 launch zed_wrapper zed_camera.launch.py \
        camera_model:=zed2i > /tmp/zed_camera.log 2>&1 &
fi
ZED_PID=$!
PIDS+=($ZED_PID)
echo -e "${GREEN}[✓] ZED 2i 相机节点已在后台拉起 (PID: $ZED_PID)${NC}"
echo -e "${CYAN}    等待 4 秒等待相机曝光与立体匹配初始化...${NC}"
sleep 4

# 5. 发布相机静态坐标系变换 (TF)
echo -e "\n${BLUE}[5/5] 发布 ZED 相机水平静态 TF (base_link -> zed_camera_link)...${NC}"
ros2 run tf2_ros static_transform_publisher 0.257 0.0 0.28 0.0 0.0 0.0 base_link zed_camera_link > /dev/null 2>&1 &
TF_PID=$!
PIDS+=($TF_PID)
echo -e "${GREEN}[✓] 相机静态 TF 广播中 (PID: $TF_PID)${NC}"
sleep 1

# 6. 前台启动 SRU 强化学习端到端避障导航策略
echo -e "\n${CYAN}================================================================${NC}"
echo -e "${GREEN}  前 4 个系统组件已就绪，正在启动第 5 步：SRU 强化学习导航策略！  ${NC}"
echo -e "${CYAN}================================================================${NC}"
echo -e "${YELLOW}[提示] 当看到 'First odometry message received, odom is ready' 时，${NC}"
echo -e "${YELLOW}       系统即全部就绪！此时在 NoMachine 打开新终端发布激活命令：${NC}"
echo -e "${GREEN}       ros2 topic pub /startNavi std_msgs/msg/Bool \"{data: true}\" --once${NC}"
echo -e "${CYAN}================================================================${NC}\n"

# 前台运行 rl_nav_controller，保持用户能够实时观察策略推理输出
POLICY_MODEL_ARG=()
HAS_CUSTOM_POLICY=false
for arg in "$@"; do
    if [ "$arg" = "--policy-path" ]; then
        HAS_CUSTOM_POLICY=true
        break
    fi
done

if [ "$HAS_CUSTOM_POLICY" = "false" ] && [ -f "$HOME/sru_ws/model/Odin_nav_policy.onnx" ]; then
    echo -e "${GREEN}[✓] 未显式指定模型，自动载入默认自定义模型: $HOME/sru_ws/model/Odin_nav_policy.onnx${NC}"
    POLICY_MODEL_ARG=("--policy-path" "$HOME/sru_ws/model/Odin_nav_policy.onnx")
fi

exec ros2 run rl_nav_controller rl_nav_controller "${POLICY_MODEL_ARG[@]}" "$@" \
  --ros-args \
  -r /dlio/odom_node/odom:=/lio/robo/odom \
  -r /path_manager/path_manager_ros/nav_vel:=/cmd_vel

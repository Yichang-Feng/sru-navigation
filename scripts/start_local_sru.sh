#!/usr/bin/env bash
# ==============================================================================
# Unitree Go2W 单独启动局部 SRU 避障导航系统脚本 (start_local_sru.sh)
#
# 说明：用于不依赖全局路径规划、单独启动并测试机载 SRU 局部强化学习避障导航。
#       支持 RViz 2D Nav Goal 发送单点目标直接避障推进。
#       手柄按键统一对齐全局模式：[A] 启动/恢复运动，[B] 暂停刹停。
#       到点判定距离与全局对齐为 0.20m。
#
# 适用硬件：高层核心机 Jetson Orin DevKit (192.168.123.111, 用户: agile)
# 适用环境：Ubuntu 22.04 / ROS 2 Humble (通过 NoMachine 远程桌面或本地终端运行)
#
# 启动内容：
#   [1] go2w_bridge (底盘控制与手柄通信桥，按 B 停止，按 A 启动)
#   [2] super_lio relocation.py (Livox MID-360 雷达驱动 + SLAM 重定位 + 自动拉起 RViz)
#   [3] 深度相机 (默认: Intel RealSense D435/D435i 原生深度图；可选: --zed 使用 ZED 2i)
#   [4] static TF (base_link -> camera_link 水平相机外参标定)
#   [5] rl_nav_controller (SRU 强化学习端到端避障导航策略节点，单点到点判定 0.20m)
#
# 用法：
#   ./start_local_sru.sh                 # 默认使用 RealSense D435/D435i 深度相机
#   ./start_local_sru.sh --realsense     # 显式指定 RealSense
#   ./start_local_sru.sh --zed           # 切换使用 Stereolabs ZED 2i 相机
#
# 退出机制：按 Ctrl+C 自动安全关闭并终止所有后台进程，或在另一终端执行 ./stop_local_sru.sh
# ==============================================================================

set -e

# 颜色输出定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# 命令行参数解析：默认使用 RealSense 相机
CAMERA_TYPE="realsense"
ENABLE_RVIZ="true"
FORWARD_ARGS=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        --realsense|--d435|--d435i)
            CAMERA_TYPE="realsense"
            shift
            ;;
        --zed|--zed2i|--zedx)
            CAMERA_TYPE="zed"
            shift
            ;;
        --camera)
            CAMERA_TYPE="$2"
            shift 2
            ;;
        --rviz)
            ENABLE_RVIZ="true"
            shift
            ;;
        --no-rviz)
            ENABLE_RVIZ="false"
            shift
            ;;
        *)
            FORWARD_ARGS+=("$1")
            shift
            ;;
    esac
done

echo -e "${CYAN}================================================================${NC}"
echo -e "${CYAN}   Unitree Go2W 单独启动局部 SRU 避障导航系统 (Standalone Local Launch) ${NC}"
echo -e "${CYAN}   感知相机: ${GREEN}${CAMERA_TYPE}${CYAN} (可通过 --zed 或 --realsense 切换)         ${NC}"
echo -e "${CYAN}   控制对齐: ${GREEN}按 [B] 停止刹停，按 [A] 启动/恢复运动${CYAN}                  ${NC}"
echo -e "${CYAN}   到点阈值: ${GREEN}0.20m (与全局导航统一)${CYAN}                                ${NC}"
echo -e "${CYAN}================================================================${NC}"

# 1. 加载 ROS 2 及机载工程环境
echo -e "${BLUE}[1/5] 加载 ROS 2 环境与工程工作空间...${NC}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 定位相机导航配置文件
if [ "$CAMERA_TYPE" = "realsense" ]; then
    DEPTH_TOPIC="/camera/camera/depth/image_rect_raw"
    CAMERA_MIN_DEPTH="0.15"
    CAMERA_MAX_DEPTH="10.0"

    RS_CONFIG_OVERRIDE=""
    CANDIDATE_RS_CONFIGS=(
        "${SCRIPT_DIR}/realsense_nav_override.yaml"
        "${SCRIPT_DIR}/config/realsense_nav_override.yaml"
        "$HOME/sru_ws/realsense_nav_override.yaml"
        "$HOME/bridge_ws/realsense_nav_override.yaml"
        "$HOME/realsense_nav_override.yaml"
    )
    for cfg in "${CANDIDATE_RS_CONFIGS[@]}"; do
        if [ -f "$cfg" ]; then
            RS_CONFIG_OVERRIDE="$cfg"
            break
        fi
    done

    # 若未找到则生成临时的高性能配置
    if [ -z "$RS_CONFIG_OVERRIDE" ] || [ ! -f "$RS_CONFIG_OVERRIDE" ]; then
        RS_CONFIG_OVERRIDE="/tmp/realsense_nav_override.yaml"
        cat << 'EOF' > "$RS_CONFIG_OVERRIDE"
/**:
    ros__parameters:
        depth_module:
            emitter_enabled: 1
            laser_power: 360.0
            visual_preset: 4
            enable_auto_exposure: true
        spatial_filter:
            enable: true
            filter_magnitude: 2
            filter_smooth_alpha: 0.5
            filter_smooth_delta: 20
            holes_fill: 2
        temporal_filter:
            enable: true
            filter_smooth_alpha: 0.4
            filter_smooth_delta: 20
            holes_fill: 3
        hole_filling_filter:
            enable: true
            holes_fill: 2
        clip_distance: -1.0
EOF
    fi
else
    DEPTH_TOPIC="/zed/zed_node/depth/depth_registered"
    CAMERA_MIN_DEPTH="0.25"
    CAMERA_MAX_DEPTH="10.0"

    ZED_CONFIG_OVERRIDE=""
    CANDIDATE_CONFIGS=(
        "${SCRIPT_DIR}/zed_nav_override.yaml"
        "${SCRIPT_DIR}/config/zed_nav_override.yaml"
        "$HOME/sru_ws/zed_nav_override.yaml"
        "$HOME/bridge_ws/zed_nav_override.yaml"
        "$HOME/zed_nav_override.yaml"
    )
    for cfg in "${CANDIDATE_CONFIGS[@]}"; do
        if [ -f "$cfg" ]; then
            ZED_CONFIG_OVERRIDE="$cfg"
            break
        fi
    done

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
fi

source /opt/ros/humble/setup.bash 2>/dev/null || true
if [ -f "$HOME/unitree_ros2/setup.sh" ]; then
    source "$HOME/unitree_ros2/setup.sh" 2>/dev/null || true
fi
if [ -f "$HOME/bridge_ws/install/setup.bash" ]; then
    source "$HOME/bridge_ws/install/setup.bash" 2>/dev/null || true
fi
if [ -f "$HOME/go2w_nav_ros2_ws/install/setup.bash" ]; then
    source "$HOME/go2w_nav_ros2_ws/install/setup.bash" 2>/dev/null || true
fi
if [ -f "$HOME/sru_ws/install/setup.bash" ]; then
    source "$HOME/sru_ws/install/setup.bash" 2>/dev/null || true
fi

# 记录所有后台进程 PID
PIDS=()
CLEANUP_DONE=0

# 退出清理函数
cleanup() {
    [ "${CLEANUP_DONE:-0}" -eq 1 ] && return
    CLEANUP_DONE=1
    trap - SIGINT SIGTERM EXIT
    echo -e "\n${YELLOW}[!] 接收到退出信号，正在快速安全停止所有后台节点...${NC}"
    # 非阻塞后台发布底盘制动锁与停止信号
    (ros2 topic pub /motion_enabled std_msgs/msg/Bool "{data: false}" --once >/dev/null 2>&1 &) || true
    (ros2 topic pub /startNavi std_msgs/msg/Bool "{data: false}" --once >/dev/null 2>&1 &) || true
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -9 "$pid" 2>/dev/null || true
        fi
    done
    pkill -9 -f "rl_nav_controller" 2>/dev/null || true
    pkill -9 -f "cmd_vel_bridge" 2>/dev/null || true
    pkill -9 -f "wireless_controller" 2>/dev/null || true
    pkill -9 -f "unitree_bridge" 2>/dev/null || true
    pkill -9 -f "map_localization_node" 2>/dev/null || true
    pkill -9 -f "relocation_node" 2>/dev/null || true
    pkill -9 -f "livox_ros_driver2_node" 2>/dev/null || true
    pkill -9 -f "realsense2_camera" 2>/dev/null || true
    pkill -9 -f "zed_wrapper" 2>/dev/null || true
    pkill -9 -f "static_transform_publisher.*camera_link" 2>/dev/null || true
    pkill -9 -f "static_transform_publisher.*zed_camera_link" 2>/dev/null || true
    pkill -9 -f "static_transform_publisher.*map.*world" 2>/dev/null || true
    pkill -9 -f "rviz2" 2>/dev/null || true
    echo -e "${GREEN}[✓] 全系统所有节点已安全终止，终端已就绪。${NC}"
    exit 0
}

# 注册捕获信号
trap cleanup SIGINT SIGTERM EXIT

# 2. 启动底盘控制与手柄通信桥 (统一对齐全局启动: go2w_bridge，支持按 B 停止、按 A 启动)
echo -e "\n${BLUE}[2/5] 启动底盘控制与手柄桥 (按 B 停止，按 A 启动)...${NC}"
pkill -f "unitree_bridge" 2>/dev/null || true
pkill -f "cmd_vel_bridge" 2>/dev/null || true
pkill -f "wireless_controller" 2>/dev/null || true

if ros2 pkg prefix go2w_bridge >/dev/null 2>&1; then
    echo -e "${GREEN}[✓] 检测到 go2w_bridge 功能包，启动高层桥接 (cmd_vel_bridge + wireless_controller)...${NC}"
    ros2 launch go2w_bridge go2w_bridge.launch.py > /tmp/go2w_bridge.log 2>&1 &
    BRIDGE_PID=$!
    PIDS+=($BRIDGE_PID)
    sleep 1
    if kill -0 "$BRIDGE_PID" 2>/dev/null; then
        echo -e "${GREEN}[✓] go2w_bridge 启动成功 (PID: $BRIDGE_PID，已使能手柄 A/B 键控制)${NC}"
    else
        echo -e "${RED}[✗] go2w_bridge 启动失败，请检查 /tmp/go2w_bridge.log${NC}"
    fi
else
    echo -e "${YELLOW}[i] 未检测到 go2w_bridge 功能包，降级启动 unitree_bridge...${NC}"
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

# 3. 启动 Livox MID-360 雷达与 Super LIO 地图重定位 (100% 对齐全局 start_nav.sh)
echo -e "\n${BLUE}[3/5] 启动 Livox MID-360 激光雷达硬件驱动与 Super LIO 定位系统...${NC}"

# 3.1 启动 Livox MID-360 激光雷达硬件驱动
pkill -f "livox_ros_driver2_node" 2>/dev/null || true
echo -e "${GREEN}[✓] 启动 Livox MID-360 雷达驱动 (ros2 launch livox_ros_driver2 msg_MID360_launch.py)...${NC}"
ros2 launch livox_ros_driver2 msg_MID360_launch.py > /tmp/livox_lidar.log 2>&1 &
LIVOX_PID=$!
PIDS+=($LIVOX_PID)
sleep 2

# 3.2 启动 Super-LIO 定位 (使用与全局 start_nav.sh 完全一致的 map_localization.py 发布 /lio/robo/odom)
pkill -f "map_localization_node" 2>/dev/null || true
pkill -f "relocation_node" 2>/dev/null || true

if ros2 launch super_lio map_localization.py --show-args >/dev/null 2>&1; then
    echo -e "${GREEN}[✓] 启动 Super-LIO 地图定位节点 (ros2 launch super_lio map_localization.py)...${NC}"
    ros2 launch super_lio map_localization.py > /tmp/super_lio.log 2>&1 &
    SUPER_LIO_PID=$!
    PIDS+=($SUPER_LIO_PID)
else
    echo -e "${YELLOW}[i] 降级启动 super_lio relocation.py...${NC}"
    ros2 launch super_lio relocation.py > /tmp/super_lio.log 2>&1 &
    SUPER_LIO_PID=$!
    PIDS+=($SUPER_LIO_PID)
fi

# 3.3 广播 map -> world 基础静态变换 (打通全域坐标系，确保点云在 map/world 均能实时渲染)
pkill -f "static_transform_publisher.*map.*world" 2>/dev/null || true
nohup ros2 run tf2_ros static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 map world > /dev/null 2>&1 &
MAP_TF_PID=$!
PIDS+=($MAP_TF_PID)
echo -e "${GREEN}[✓] 已广播坐标基准静态 TF: map -> world (PID: $MAP_TF_PID)${NC}"

# 3.4 可选拉起 RViz 监控 (优先加载局部 SRU 专属预调界面 local_sru.rviz / relocation.rviz)
if [ "$ENABLE_RVIZ" = "true" ] && [ -n "${DISPLAY:-}" ]; then
    RVIZ_CFG=""
    CANDIDATE_RVIZS=(
        "${SCRIPT_DIR}/local_sru.rviz"
        "${SCRIPT_DIR}/../config/local_sru.rviz"
        "$HOME/sru_ws/local_sru.rviz"
        "$HOME/bridge_ws/src/super_lio/super_lio/rviz/relocation.rviz"
        "$HOME/bridge_ws/install/super_lio/share/super_lio/rviz/relocation.rviz"
        "$HOME/go2w_nav_ros2_ws/src/super_lio/super_lio/rviz/relocation.rviz"
        "$HOME/bridge_ws/src/super_lio/super_lio/rviz/lio.rviz"
        "$HOME/go2w_nav_ros2_ws/rviz.rviz"
    )
    for r in "${CANDIDATE_RVIZS[@]}"; do
        if [ -f "$r" ]; then
            RVIZ_CFG="$r"
            break
        fi
    done
    if [ -n "$RVIZ_CFG" ]; then
        echo -e "${GREEN}[✓] 后台拉起预调 RViz 可视化界面 (配置文件: $RVIZ_CFG)...${NC}"
        nohup ros2 run rviz2 rviz2 -d "$RVIZ_CFG" >/dev/null 2>&1 &
        RVIZ_PID=$!
        PIDS+=($RVIZ_PID)
    fi
fi

echo -e "${CYAN}    等待 4 秒等待激光雷达建流与先验地图匹配加载...${NC}"
sleep 4

# 4. 启动深度相机感知节点
if [ "$CAMERA_TYPE" = "realsense" ]; then
    echo -e "\n${BLUE}[4/5] 启动 Intel RealSense D435i 深度相机 (原生深度 640x480 + 滤波流水线)...${NC}"
    pkill -f "static_transform_publisher.*camera_link" 2>/dev/null || true
    nohup ros2 run tf2_ros static_transform_publisher 0.34 0.0 0.25 0.0 0.0 0.0 base_link camera_link > /dev/null 2>&1 &
    TF_PID=$!
    PIDS+=($TF_PID)
    echo -e "${GREEN}[✓] 已广播 RealSense 静态 TF: base_link -> camera_link (PID: $TF_PID)${NC}"

    if [ -n "$RS_CONFIG_OVERRIDE" ] && [ -f "$RS_CONFIG_OVERRIDE" ]; then
        echo -e "${GREEN}[✓] 已加载 RealSense 远距覆盖配置: ${RS_CONFIG_OVERRIDE}${NC}"
        ros2 launch realsense2_camera rs_launch.py \
            config_file:="$RS_CONFIG_OVERRIDE" \
            depth_module.profile:=640x480x15 \
            rgb_camera.profile:=640x480x15 \
            align_depth.enable:=false \
            pointcloud.enable:=false \
            clip_distance:=-1.0 \
            filters:=spatial,temporal,hole_filling > /tmp/realsense_camera.log 2>&1 &
    else
        ros2 launch realsense2_camera rs_launch.py \
            depth_module.profile:=640x480x15 \
            rgb_camera.profile:=640x480x15 \
            align_depth.enable:=false \
            pointcloud.enable:=false \
            clip_distance:=-1.0 \
            filters:=spatial,temporal,hole_filling > /tmp/realsense_camera.log 2>&1 &
    fi
    CAM_PID=$!
    PIDS+=($CAM_PID)
    echo -e "${GREEN}[✓] RealSense D435i 相机节点已在后台拉起 (PID: $CAM_PID)${NC}"
    echo -e "${CYAN}    等待 3 秒等待相机曝光与驱动流水线初始化...${NC}"
    sleep 3
else
    echo -e "\n${BLUE}[4/5] 启动 ZED 2i 双目深度相机感知 (纯深度图加速模式)...${NC}"
    pkill -f "static_transform_publisher.*zed_camera_link" 2>/dev/null || true
    nohup ros2 run tf2_ros static_transform_publisher 0.257 0.0 0.28 0.0 0.0 0.0 base_link zed_camera_link > /dev/null 2>&1 &
    TF_PID=$!
    PIDS+=($TF_PID)
    echo -e "${GREEN}[✓] 已广播 ZED 静态 TF: base_link -> zed_camera_link (PID: $TF_PID)${NC}"

    if [ -n "$ZED_CONFIG_OVERRIDE" ] && [ -f "$ZED_CONFIG_OVERRIDE" ]; then
        echo -e "${GREEN}[✓] 已加载轻量化覆盖配置: ${ZED_CONFIG_OVERRIDE}${NC}"
        ros2 launch zed_wrapper zed_camera.launch.py \
            camera_model:=zed2i \
            ros_params_override_path:="$ZED_CONFIG_OVERRIDE" > /tmp/zed_camera.log 2>&1 &
    else
        echo -e "${YELLOW}[!] 未指定覆盖配置，使用默认参数启动相机...${NC}"
        ros2 launch zed_wrapper zed_camera.launch.py \
            camera_model:=zed2i > /tmp/zed_camera.log 2>&1 &
    fi
    CAM_PID=$!
    PIDS+=($CAM_PID)
    echo -e "${GREEN}[✓] ZED 2i 相机节点已在后台拉起 (PID: $CAM_PID)${NC}"
    echo -e "${CYAN}    等待 4 秒等待相机曝光与立体匹配初始化...${NC}"
    sleep 4
fi

# 5. 前台启动 SRU 强化学习端到端避障导航策略
echo -e "\n${CYAN}================================================================${NC}"
echo -e "${GREEN}  前置系统组件已就绪，正在启动：SRU 强化学习导航策略！  ${NC}"
echo -e "${CYAN}  深度图像话题: ${GREEN}${DEPTH_TOPIC}${NC}"
echo -e "${CYAN}  深度量程范围: [${CAMERA_MIN_DEPTH}m, ${CAMERA_MAX_DEPTH}m]${NC}"
echo -e "${CYAN}================================================================${NC}"
echo -e "${YELLOW}[提示] 当看到 'First odometry message received, odom is ready' 时系统即全部就绪！${NC}"
echo -e "${GREEN}>>> 手柄操作: [A] 启动/恢复导航  |  [B] 暂停刹停${NC}"
echo -e "${CYAN}================================================================${NC}\n"

# 前台运行 rl_nav_controller，保持用户能够实时观察策略推理输出
POLICY_MODEL_ARG=()
HAS_CUSTOM_POLICY=false
for arg in "${FORWARD_ARGS[@]}"; do
    if [ "$arg" = "--policy-path" ]; then
        HAS_CUSTOM_POLICY=true
        break
    fi
done

if [ "$HAS_CUSTOM_POLICY" = "false" ] && [ -f "$HOME/sru_ws/model/Odin_nav_policy.onnx" ]; then
    echo -e "${GREEN}[✓] 未显式指定模型，自动载入默认自定义模型: $HOME/sru_ws/model/Odin_nav_policy.onnx${NC}"
    POLICY_MODEL_ARG=("--policy-path" "$HOME/sru_ws/model/Odin_nav_policy.onnx")
fi

# 优先直接使用源码执行，彻底规避 colcon build setuptools entry_points 缓存脱节
CANDIDATE_SCRIPTS=(
    "$HOME/sru_ws/src/sru-robot-deployment/rl_nav_controller/rl_nav_controller/rl_nav_controller.py"
    "$HOME/sru_ws/src/rl_nav_controller/rl_nav_controller/rl_nav_controller.py"
    "$HOME/go2w_nav_ros2_ws/src/rl_nav_controller/rl_nav_controller/rl_nav_controller.py"
    "${SCRIPT_DIR}/../rl_nav_controller.py"
    "${SCRIPT_DIR}/rl_nav_controller.py"
)
SRC_SCRIPT=""
for s in "${CANDIDATE_SCRIPTS[@]}"; do
    if [ -f "$s" ]; then
        SRC_SCRIPT="$s"
        break
    fi
done

if [ -n "$SRC_SCRIPT" ]; then
    SRC_PKG_DIR="$(dirname "$(dirname "$SRC_SCRIPT")")"
    echo -e "${GREEN}[✓] 直接加载最新策略源码: ${SRC_SCRIPT}${NC}"
    export PYTHONPATH="${SRC_PKG_DIR}:${PYTHONPATH:-}"
    python3 -u "$SRC_SCRIPT" "${POLICY_MODEL_ARG[@]}" "${FORWARD_ARGS[@]}" \
      --ros-args \
      -p depth_topic:=${DEPTH_TOPIC} \
      -p min_depth:=${CAMERA_MIN_DEPTH} \
      -p max_depth:=${CAMERA_MAX_DEPTH} \
      -p segment_arrive_threshold:=0.20 \
      -p arrive_goal_threshold:=0.20 \
      -r /dlio/odom_node/odom:=/lio/robo/odom \
      -r /path_manager/path_manager_ros/nav_vel:=/cmd_vel
    RET_CODE=$?
else
    ros2 run rl_nav_controller rl_nav_controller "${POLICY_MODEL_ARG[@]}" "${FORWARD_ARGS[@]}" \
      --ros-args \
      -p depth_topic:=${DEPTH_TOPIC} \
      -p min_depth:=${CAMERA_MIN_DEPTH} \
      -p max_depth:=${CAMERA_MAX_DEPTH} \
      -p segment_arrive_threshold:=0.20 \
      -p arrive_goal_threshold:=0.20 \
      -r /dlio/odom_node/odom:=/lio/robo/odom \
      -r /path_manager/path_manager_ros/nav_vel:=/cmd_vel
    RET_CODE=$?
fi

cleanup
exit $RET_CODE

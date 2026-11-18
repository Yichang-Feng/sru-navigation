#!/usr/bin/env bash
# ==============================================================================
# Unitree Go2W 单独启动局部 SRU 避障导航系统脚本 (start_local_sru.sh)
#
# 说明：用于不依赖全局路径规划、单独启动并测试机载 SRU 局部强化学习避障导航。
#       支持 RViz 2D Nav Goal 发送单点目标直接避障推进。
#       手柄按键统一对齐全局模式：[A] 启动/恢复运动，[B] 暂停刹停。
#       到点判定距离与全局对齐为 0.20m。
#
# 适用环境（自适应）：
#   - ROS 2 Foxy / Humble / Jazzy 自动检测 (/opt/ros/<distro>)，机载端实测为 Foxy
#   - 所有节点均为单独启动 (ros2 run 逐节点拉起，每节点独立进程、独立 /tmp 日志)，
#     不使用 ros2 launch 打包启动
#   - onnxruntime 自动预检：缺失时自动安装（优先 pip --user 免 sudo；
#     失败则建立继承 ROS 系统 site-packages 的专用 venv），无需手动处理
#
# 机载部署布局（本脚本位于 <deploy_root>/src/sru-local/sru_ws/）：
#   ~/agile_nav_ws/src/sru-local/sru_ws/start_local_sru.sh    (本脚本)
#   ~/agile_nav_ws/src/sru-local/go2w_nav_ros2_ws/            (rl_nav_controller / super_lio 等)
#   ~/agile_nav_ws/src/sru-local/bridge_ws/                   (livox_ros_driver2 / unitree_bridge 等，可选)
#
# 启动内容（全部单独启动）：
#   [1] ROS 环境加载 (distro 自动检测 + 各工作空间 install/setup.bash) + onnxruntime 预检
#   [2] 底盘桥: go2w_bridge (cmd_vel_bridge + wireless_controller 两个独立进程) 或 unitree_bridge 降级
#   [3] Livox MID-360 驱动 (livox_ros_driver2_node) + Super-LIO 定位
#       (优先 map_localization_node，降级 relocation_node；relocation 模式补发静态 map->world TF)
#   [4] 深度相机 (默认 RealSense D435i: realsense2_camera 单进程；可选 --zed)
#   [5] rl_nav_controller (SRU 强化学习端到端避障策略节点，前台运行，单点到点判定 0.20m)
#
# 用法：
#   ./start_local_sru.sh                 # 默认即 VLN 模式（等价于 --vln）：base 相对点 + 关 RViz + 10cm 到点，RealSense 相机
#   ./start_local_sru.sh --vln           # 显式声明 VLN 模式（与默认行为相同）
#   ./start_local_sru.sh --goal-mode auto --rviz   # 恢复旧行为：按 frame_id 判别目标帧 + RViz 下点
#   ./start_local_sru.sh --goal-timeout 1.5        # 开启"无新点刹停"看门狗（默认关闭）
#   ./start_local_sru.sh --goal-arrive 0.15        # 指定 VLN 相对点的位置到点半径 (默认 10cm)
#   ./start_local_sru.sh --policy-path /path/to/model.onnx  # 指定自定义策略模型（默认用仓库自带 deployment_policies/nav_policy.onnx）
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
ENABLE_RVIZ="false"       # 默认关闭 RViz（VLN 部署不需要；--rviz 可打开）
FORWARD_ARGS=()

# VLN 相对 base 目标点模式 (见 docs/VLN_BASE_FRAME_GOALS.md)
# auto: frame_id 为 base_link 等 base 帧时按相对点处理
# base: 所有 /goal_pose 都按相对 base 点处理 (默认，等价于 --vln)
# world: 全部按世界绝对点处理 (旧行为)
GOAL_FRAME_MODE="base"
GOAL_STREAM_TIMEOUT="0.0"     # 0 = 关闭“无新点刹停”看门狗，按位置到点
BASE_GOAL_ARRIVE="0.10"       # VLN 相对点按位置到点的半径 (m)

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
        --vln)
            # 上层 VLN 直接向 /goal_pose 下发相对 base 的点；VLN 模式不启动 RViz
            GOAL_FRAME_MODE="base"
            ENABLE_RVIZ="false"
            shift
            ;;
        --goal-mode)
            GOAL_FRAME_MODE="$2"
            shift 2
            ;;
        --goal-timeout)
            GOAL_STREAM_TIMEOUT="$2"
            shift 2
            ;;
        --goal-arrive)
            BASE_GOAL_ARRIVE="$2"
            shift 2
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
echo -e "${CYAN}   目标模式: ${GREEN}${GOAL_FRAME_MODE}${CYAN} (base=VLN 相对点，按位置 ${BASE_GOAL_ARRIVE}m 到点) ${NC}"
echo -e "${CYAN}   无新点看门狗: ${GREEN}${GOAL_STREAM_TIMEOUT}s${CYAN} (0=关闭)${NC}"
echo -e "${CYAN}   RViz 可视化: ${GREEN}${ENABLE_RVIZ}${CYAN} (--vln 模式默认关闭)${NC}"
echo -e "${CYAN}   启动方式: ${GREEN}全部节点单独启动 (ros2 run，独立进程/日志)${NC}"
echo -e "${CYAN}   到点阈值: ${GREEN}0.20m (与全局导航统一；VLN 相对点由发点流决定到点)${NC}  ${NC}"
echo -e "${CYAN}================================================================${NC}"

# ------------------------------------------------------------------------------
# 1. 加载 ROS 2 (distro 自动检测: Foxy/Humble/Jazzy) 及机载工程环境
# ------------------------------------------------------------------------------
echo -e "\n${BLUE}[1/5] 加载 ROS 2 环境与工程工作空间...${NC}"
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

# --- ROS distro 自动检测 (Foxy 优先，兼容 Humble/Jazzy) ---
if [ -z "${ROS_DISTRO:-}" ]; then
    for _d in foxy humble jazzy; do
        if [ -f "/opt/ros/${_d}/setup.bash" ]; then
            ROS_DISTRO="$_d"
            break
        fi
    done
fi
if [ -z "${ROS_DISTRO:-}" ] || [ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    echo -e "${RED}[✗] 未找到 ROS 2 安装 (/opt/ros/{foxy,humble,jazzy})，请检查机载 ROS 环境。${NC}"
    exit 1
fi
echo -e "${GREEN}[✓] 检测到 ROS 2 ${ROS_DISTRO}，加载 /opt/ros/${ROS_DISTRO}/setup.bash...${NC}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"

# --- 机载工作空间 (兼容 ~/agile_nav_ws/src/sru-local/ 与旧版 $HOME 布局) ---
WS_SETUP_CANDIDATES=(
    "$HOME/unitree_ros2/setup.sh"
    "$HOME/bridge_ws/install/setup.bash"
    "$HOME/go2w_nav_ros2_ws/install/setup.bash"
    "$HOME/sru_ws/install/setup.bash"
    "${SCRIPT_DIR}/install/setup.bash"
    "${SCRIPT_DIR}/../go2w_nav_ros2_ws/install/setup.bash"
    "${SCRIPT_DIR}/../bridge_ws/install/setup.bash"
    "$(cd "${SCRIPT_DIR}/../.." 2>/dev/null && pwd)/install/setup.bash"
)
for ws in "${WS_SETUP_CANDIDATES[@]}"; do
    if [ -f "$ws" ]; then
        # shellcheck disable=SC1090
        source "$ws" || true   # 单个工作空间加载失败不中断，由后续 ros2 可用性检查兜底
        echo -e "${GREEN}[✓] 已加载工作空间环境: ${ws}${NC}"
    fi
done

if ! command -v ros2 >/dev/null 2>&1; then
    echo -e "${RED}[✗] ros2 命令不可用，请手动 source ROS 2 与机载工作空间后重试。${NC}"
    exit 1
fi

# --- onnxruntime / Python 环境预检 (策略节点依赖: rclpy + onnxruntime) ---
# 优先级：
#   1) 系统 python3 已具备 rclpy+onnxruntime → 直接用
#   2) 既有 venv (~/ros2_onnx, 本目录 onnx_venv, ~/sru_onnx_venv) 已具备 → 用之
#   3) pip --user 安装 onnxruntime (免 sudo，装到 ~/.local)
#   4) 建立专用 venv (--system-site-packages 继承 ROS 的 rclpy/numpy/cv2)，仅在其中装 onnxruntime
CTRL_PYTHON=""
_setup_log="/tmp/sru_onnx_setup.log"

python_env_ok() { # $1=解释器路径；要求同时可导入 rclpy 与 onnxruntime
    "$1" -c "import rclpy, onnxruntime" >/dev/null 2>&1
}

find_controller_python() {
    if python_env_ok python3; then
        CTRL_PYTHON="python3"
        return 0
    fi
    local venv
    for venv in "$HOME/ros2_onnx/bin/python" "${SCRIPT_DIR}/onnx_venv/bin/python" "$HOME/sru_onnx_venv/bin/python"; do
        if [ -x "$venv" ] && python_env_ok "$venv"; then
            CTRL_PYTHON="$venv"
            return 0
        fi
    done

    echo -e "${YELLOW}[i] 当前 Python 缺少 onnxruntime，尝试自动安装 (pip --user，免 sudo)...${NC}"
    if python3 -m pip install --user onnxruntime >"$_setup_log" 2>&1 && python_env_ok python3; then
        CTRL_PYTHON="python3"
        echo -e "${GREEN}[✓] onnxruntime 已安装到用户目录 (~/.local)，系统 python3 可用。${NC}"
        return 0
    fi

    local venv_dir="${SCRIPT_DIR}/onnx_venv"
    echo -e "${YELLOW}[i] pip --user 不可用，改用专用 venv: ${venv_dir} (继承 ROS 系统 site-packages)...${NC}"
    if python3 -m venv --system-site-packages "$venv_dir" >>"$_setup_log" 2>&1 \
        && "$venv_dir/bin/pip" install --upgrade pip setuptools wheel >>"$_setup_log" 2>&1 \
        && "$venv_dir/bin/pip" install onnxruntime >>"$_setup_log" 2>&1 \
        && python_env_ok "$venv_dir/bin/python"; then
        CTRL_PYTHON="${venv_dir/bin/python}"
        echo -e "${GREEN}[✓] 专用 venv 创建完成: ${venv_dir}${NC}"
        return 0
    fi

    echo -e "${RED}[✗] onnxruntime 自动安装失败，详见日志: ${_setup_log}${NC}"
    echo -e "${RED}    请手动执行: pip3 install --user onnxruntime   (或 sudo apt install python3-pip / python3-venv 后重试)${NC}"
    return 1
}

if ! find_controller_python; then
    exit 1
fi
echo -e "${GREEN}[✓] 策略推理 Python: ${CTRL_PYTHON} (onnxruntime $(${CTRL_PYTHON} -c 'import onnxruntime;print(onnxruntime.__version__)' 2>/dev/null || echo '?'))${NC}"

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

# --- 工具函数：包 share 目录 / 可执行文件探测 / 后台进程存活检查 ---
pkg_share() { # $1=包名 → 输出 share/<pkg> 路径 (未找到则空)
    local prefix
    prefix="$(ros2 pkg prefix "$1" 2>/dev/null || true)"
    if [ -n "$prefix" ] && [ -d "${prefix}/share/$1" ]; then
        echo "${prefix}/share/$1"
    fi
}

pkg_has_exe() { # $1=包名 $2=可执行文件名 → 0/1
    local prefix d
    prefix="$(ros2 pkg prefix "$1" 2>/dev/null || true)"
    [ -z "$prefix" ] && return 1
    for d in "${prefix}/lib/$1" "${prefix}/lib"; do
        if [ -x "${d}/$2" ]; then
            return 0
        fi
    done
    return 1
}

check_alive() { # $1=pid $2=节点名 $3=日志文件
    if kill -0 "$1" 2>/dev/null; then
        echo -e "${GREEN}[✓] $2 启动成功 (PID: $1，日志: $3)${NC}"
    else
        echo -e "${RED}[✗] $2 启动失败，日志尾部 ($3):${NC}"
        tail -n 15 "$3" 2>/dev/null | sed 's/^/    /' || true
    fi
}

# ------------------------------------------------------------------------------
# 2. 启动底盘控制与手柄通信桥 (单独启动: go2w_bridge 两节点独立进程，或 unitree_bridge 降级)
# ------------------------------------------------------------------------------
echo -e "\n${BLUE}[2/5] 启动底盘控制与手柄桥 (按 B 停止，按 A 启动)...${NC}"
pkill -f "unitree_bridge" 2>/dev/null || true
pkill -f "cmd_vel_bridge" 2>/dev/null || true
pkill -f "wireless_controller" 2>/dev/null || true

if ros2 pkg prefix go2w_bridge >/dev/null 2>&1; then
    echo -e "${GREEN}[✓] 检测到 go2w_bridge 功能包，单独启动 cmd_vel_bridge + wireless_controller...${NC}"
    ros2 run go2w_bridge cmd_vel_bridge > /tmp/go2w_cmd_vel_bridge.log 2>&1 &
    BRIDGE_PID=$!
    PIDS+=($BRIDGE_PID)
    ros2 run go2w_bridge wireless_controller > /tmp/go2w_wireless_controller.log 2>&1 &
    WIRELESS_PID=$!
    PIDS+=($WIRELESS_PID)
    sleep 1
    check_alive "$BRIDGE_PID" "go2w_bridge/cmd_vel_bridge" /tmp/go2w_cmd_vel_bridge.log
    check_alive "$WIRELESS_PID" "go2w_bridge/wireless_controller" /tmp/go2w_wireless_controller.log
else
    echo -e "${YELLOW}[i] 未检测到 go2w_bridge 功能包，降级单独启动 unitree_bridge...${NC}"
    if pkg_has_exe unitree_bridge unitree_bridge; then
        ros2 run unitree_bridge unitree_bridge > /tmp/unitree_bridge.log 2>&1 &
        BRIDGE_PID=$!
        PIDS+=($BRIDGE_PID)
        sleep 1
        check_alive "$BRIDGE_PID" "unitree_bridge" /tmp/unitree_bridge.log
    else
        echo -e "${RED}[✗] unitree_bridge 可执行文件不存在 (请确认 bridge_ws 已 colcon build 并 source)。${NC}"
        BRIDGE_PID=""
    fi
fi

# ------------------------------------------------------------------------------
# 3. 启动 Livox MID-360 雷达驱动与 Super LIO 定位系统 (全部单独启动)
# ------------------------------------------------------------------------------
echo -e "\n${BLUE}[3/5] 启动 Livox MID-360 激光雷达硬件驱动与 Super LIO 定位系统...${NC}"

# 3.1 启动 Livox MID-360 激光雷达硬件驱动 (单独进程)
pkill -f "livox_ros_driver2_node" 2>/dev/null || true
LIVOX_SHARE="$(pkg_share livox_ros_driver2)"
MID360_CFG=""
CANDIDATE_MID360=(
    "${LIVOX_SHARE}/config/MID360_config.json"
    "${SCRIPT_DIR}/../bridge_ws/src/livox_ros_driver2/config/MID360_config.json"
    "$HOME/bridge_ws/src/livox_ros_driver2/config/MID360_config.json"
)
for cfg in "${CANDIDATE_MID360[@]}"; do
    if [ -n "$cfg" ] && [ -f "$cfg" ]; then
        MID360_CFG="$cfg"
        break
    fi
done

if pkg_has_exe livox_ros_driver2 livox_ros_driver2_node; then
    LIVOX_ARGS=(--ros-args
        -p xfer_format:=1
        -p multi_topic:=0
        -p data_src:=0
        -p publish_freq:=10.0
        -p output_data_type:=0
        -p frame_id:=livox_frame)
    [ -n "$MID360_CFG" ] && LIVOX_ARGS+=(-p "user_config_path:=${MID360_CFG}")
    echo -e "${GREEN}[✓] 单独启动 Livox MID-360 雷达驱动 (ros2 run livox_ros_driver2 livox_ros_driver2_node)...${NC}"
    [ -n "$MID360_CFG" ] && echo -e "${CYAN}    MID360 配置: ${MID360_CFG}${NC}"
    ros2 run livox_ros_driver2 livox_ros_driver2_node "${LIVOX_ARGS[@]}" > /tmp/livox_lidar.log 2>&1 &
    LIVOX_PID=$!
    PIDS+=($LIVOX_PID)
    sleep 2
    check_alive "$LIVOX_PID" "livox_ros_driver2_node" /tmp/livox_lidar.log
else
    echo -e "${RED}[✗] livox_ros_driver2 可执行文件不存在 (请确认 bridge_ws 已 colcon build 并 source)。${NC}"
fi

# 3.2 启动 Super-LIO 定位 (单独进程: 优先 map_localization_node，降级 relocation_node)
pkill -f "map_localization_node" 2>/dev/null || true
pkill -f "relocation_node" 2>/dev/null || true

SLIO_SHARE="$(pkg_share super_lio)"
SLIO_MODE=""
SLIO_CFG=""
if pkg_has_exe super_lio map_localization_node; then
    SLIO_MODE="map_localization_node"
    for cfg in "${SLIO_SHARE}/config/map_localization.yaml" \
               "$(cd "${SCRIPT_DIR}/../go2w_nav_ros2_ws/src/Super-LIO-ros2/src/super_lio/config" 2>/dev/null && pwd)/map_localization.yaml"; do
        if [ -n "$cfg" ] && [ -f "$cfg" ]; then
            SLIO_CFG="$cfg"
            break
        fi
    done
elif pkg_has_exe super_lio relocation_node; then
    SLIO_MODE="relocation_node"
    for cfg in "${SLIO_SHARE}/config/relocation.yaml"; do
        if [ -n "$cfg" ] && [ -f "$cfg" ]; then
            SLIO_CFG="$cfg"
            break
        fi
    done
fi

if [ -z "$SLIO_MODE" ]; then
    echo -e "${RED}[✗] super_lio 的 map_localization_node / relocation_node 均不存在 (请确认 go2w_nav_ros2_ws 已 colcon build 并 source)。${NC}"
else
    # 与 launch 文件一致：节点启动前给雷达驱动 1 秒缓冲，保证首批 IMU 数据就位
    sleep 1
    SLIO_ARGS=(--ros-args --log-level info)
    [ -n "$SLIO_CFG" ] && SLIO_ARGS+=(-p "$SLIO_CFG")
    echo -e "${GREEN}[✓] 单独启动 Super-LIO 定位 (ros2 run super_lio ${SLIO_MODE})...${NC}"
    [ -n "$SLIO_CFG" ] && echo -e "${CYAN}    定位配置: ${SLIO_CFG}${NC}"
    ros2 run super_lio "$SLIO_MODE" "${SLIO_ARGS[@]}" > /tmp/super_lio.log 2>&1 &
    SUPER_LIO_PID=$!
    PIDS+=($SUPER_LIO_PID)
    sleep 2
    check_alive "$SUPER_LIO_PID" "super_lio/${SLIO_MODE}" /tmp/super_lio.log

    # 3.3 静态 map -> world TF：仅 relocation 模式需要 (relocation.py 即如此)。
    #     map_localization_node 会*动态*广播该变换(漂移估计 T_M_O)，再发静态恒等变换会产生双发布者冲突，
    #     与全局 start_nav.sh 行为保持一致。
    if [ "$SLIO_MODE" = "relocation_node" ]; then
        pkill -f "static_transform_publisher.*map.*world" 2>/dev/null || true
        nohup ros2 run tf2_ros static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 map world > /dev/null 2>&1 &
        MAP_TF_PID=$!
        PIDS+=($MAP_TF_PID)
        echo -e "${GREEN}[✓] 已广播坐标基准静态 TF: map -> world (PID: $MAP_TF_PID，relocation 模式)${NC}"
    else
        echo -e "${CYAN}[i] map_localization_node 会动态发布 map -> world TF，无需静态恒等变换。${NC}"
    fi
fi

# 3.4 可选拉起 RViz 监控 (单独进程；优先加载局部 SRU 专属预调界面 local_sru.rviz / relocation.rviz)
if [ "$ENABLE_RVIZ" = "true" ] && [ -n "${DISPLAY:-}" ]; then
    RVIZ_CFG=""
    CANDIDATE_RVIZS=(
        "${SCRIPT_DIR}/local_sru.rviz"
        "${SCRIPT_DIR}/../config/local_sru.rviz"
        "$HOME/sru_ws/local_sru.rviz"
        "$HOME/bridge_ws/src/super_lio/super_lio/rviz/relocation.rviz"
        "$HOME/bridge_ws/install/super_lio/share/super_lio/rviz/relocation.rviz"
        "$HOME/go2w_nav_ros2_ws/src/super_lio/super_lio/rviz/relocation.rviz"
        "${SCRIPT_DIR}/../go2w_nav_ros2_ws/src/Super-LIO-ros2/src/super_lio/rviz/map_localization.rviz"
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

# ------------------------------------------------------------------------------
# 4. 启动深度相机感知节点 (单独进程)
# ------------------------------------------------------------------------------
if [ "$CAMERA_TYPE" = "realsense" ]; then
    echo -e "\n${BLUE}[4/5] 启动 Intel RealSense D435i 深度相机 (原生深度 640x480 + 滤波流水线，单独进程)...${NC}"
    pkill -f "static_transform_publisher.*camera_link" 2>/dev/null || true
    nohup ros2 run tf2_ros static_transform_publisher 0.34 0.0 0.25 0.0 0.0 0.0 base_link camera_link > /dev/null 2>&1 &
    TF_PID=$!
    PIDS+=($TF_PID)
    echo -e "${GREEN}[✓] 已广播 RealSense 静态 TF: base_link -> camera_link (PID: $TF_PID)${NC}"

    if [ -n "$RS_CONFIG_OVERRIDE" ] && [ -f "$RS_CONFIG_OVERRIDE" ]; then
        echo -e "${GREEN}[✓] 已加载 RealSense 远距覆盖配置: ${RS_CONFIG_OVERRIDE}${NC}"
        ros2 run realsense2_camera realsense2_camera --ros-args \
            --params-file "$RS_CONFIG_OVERRIDE" \
            -p depth_module.profile:=640x480x15 \
            -p rgb_camera.profile:=640x480x15 \
            -p align_depth.enable:=false \
            -p pointcloud.enable:=false \
            -p clip_distance:=-1.0 > /tmp/realsense_camera.log 2>&1 &
    else
        ros2 run realsense2_camera realsense2_camera --ros-args \
            -p depth_module.profile:=640x480x15 \
            -p rgb_camera.profile:=640x480x15 \
            -p align_depth.enable:=false \
            -p pointcloud.enable:=false \
            -p clip_distance:=-1.0 \
            -p spatial_filter.enable:=true \
            -p temporal_filter.enable:=true \
            -p hole_filling_filter.enable:=true > /tmp/realsense_camera.log 2>&1 &
    fi
    CAM_PID=$!
    PIDS+=($CAM_PID)
    sleep 3
    check_alive "$CAM_PID" "realsense2_camera" /tmp/realsense_camera.log
else
    echo -e "\n${BLUE}[4/5] 启动 ZED 2i 双目深度相机感知 (纯深度图加速模式，单独进程)...${NC}"
    pkill -f "static_transform_publisher.*zed_camera_link" 2>/dev/null || true
    nohup ros2 run tf2_ros static_transform_publisher 0.257 0.0 0.28 0.0 0.0 0.0 base_link zed_camera_link > /dev/null 2>&1 &
    TF_PID=$!
    PIDS+=($TF_PID)
    echo -e "${GREEN}[✓] 已广播 ZED 静态 TF: base_link -> zed_camera_link (PID: $TF_PID)${NC}"

    if [ -n "$ZED_CONFIG_OVERRIDE" ] && [ -f "$ZED_CONFIG_OVERRIDE" ]; then
        echo -e "${GREEN}[✓] 已加载轻量化覆盖配置: ${ZED_CONFIG_OVERRIDE}${NC}"
        ros2 run zed_wrapper zed_camera --ros-args \
            -p camera_model:=zed2i \
            -p "ros_params_override_path:=${ZED_CONFIG_OVERRIDE}" > /tmp/zed_camera.log 2>&1 &
    else
        echo -e "${YELLOW}[!] 未指定覆盖配置，使用默认参数启动相机...${NC}"
        ros2 run zed_wrapper zed_camera --ros-args \
            -p camera_model:=zed2i > /tmp/zed_camera.log 2>&1 &
    fi
    CAM_PID=$!
    PIDS+=($CAM_PID)
    sleep 4
    check_alive "$CAM_PID" "zed_camera" /tmp/zed_camera.log
fi

# ------------------------------------------------------------------------------
# 5. 前台启动 SRU 强化学习端到端避障导航策略 (使用预检通过的 Python 环境)
# ------------------------------------------------------------------------------
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

# 默认策略模型：仓库自带 deployment_policies/nav_policy.onnx（colcon build 时由 setup.py 装入 install share，控制器默认加载）。
# 需要其他模型时显式传 --policy-path <路径>（经 FORWARD_ARGS 透传给节点）。
if [ "$HAS_CUSTOM_POLICY" = "false" ]; then
    echo -e "${CYAN}[i] 未指定 --policy-path，使用默认策略模型: deployment_policies/nav_policy.onnx${NC}"
fi

# 优先直接使用源码执行，彻底规避 colcon build setuptools entry_points 缓存脱节
CANDIDATE_SCRIPTS=(
    # ~/agile_nav_ws/src/sru-local/ 布局 (Foxy 机载)：go2w_nav_ros2_ws 与本脚本同级
    "${SCRIPT_DIR}/../go2w_nav_ros2_ws/src/rl_nav_controller/rl_nav_controller/rl_nav_controller.py"
    # 旧版 $HOME 直接布局
    "$HOME/go2w_nav_ros2_ws/src/rl_nav_controller/rl_nav_controller/rl_nav_controller.py"
    "$HOME/sru_ws/src/sru-robot-deployment/rl_nav_controller/rl_nav_controller/rl_nav_controller.py"
    "$HOME/sru_ws/src/rl_nav_controller/rl_nav_controller/rl_nav_controller.py"
    "${SCRIPT_DIR}/../rl_nav_controller.py"
    "${SCRIPT_DIR}/rl_nav_controller.py"
)
SRC_SCRIPT=""
FALLBACK_SCRIPT=""
for s in "${CANDIDATE_SCRIPTS[@]}"; do
    if [ -f "$s" ]; then
        [ -z "$FALLBACK_SCRIPT" ] && FALLBACK_SCRIPT="$s"
        # 优先选择包含 VLN base 相对点支持的版本，避免误加载旧副本
        if grep -q "goal_frame_mode" "$s"; then
            SRC_SCRIPT="$s"
            break
        fi
    fi
done
if [ -z "$SRC_SCRIPT" ] && [ -n "$FALLBACK_SCRIPT" ]; then
    SRC_SCRIPT="$FALLBACK_SCRIPT"
    if [ "$GOAL_FRAME_MODE" != "world" ]; then
        echo -e "${RED}[!] 警告: 找到的控制器源码不包含 VLN base 目标支持: ${SRC_SCRIPT}${NC}"
        echo -e "${RED}    运行中的代码可能是旧版本，请重新同步并在机载端 colcon build。${NC}"
    fi
fi

if [ -n "$SRC_SCRIPT" ]; then
    SRC_PKG_DIR="$(dirname "$(dirname "$SRC_SCRIPT")")"
    echo -e "${GREEN}[✓] 直接加载最新策略源码: ${SRC_SCRIPT}${NC}"
    export PYTHONPATH="${SRC_PKG_DIR}:${PYTHONPATH:-}"
    set +e   # 前台节点退出码需完整捕获，交给 cleanup/RET_CODE 处理
    "$CTRL_PYTHON" -u "$SRC_SCRIPT" "${POLICY_MODEL_ARG[@]}" "${FORWARD_ARGS[@]}" \
      --ros-args \
      -p depth_topic:=${DEPTH_TOPIC} \
      -p min_depth:=${CAMERA_MIN_DEPTH} \
      -p max_depth:=${CAMERA_MAX_DEPTH} \
      -p segment_arrive_threshold:=0.20 \
      -p arrive_goal_threshold:=0.20 \
      -p goal_frame_mode:=${GOAL_FRAME_MODE} \
      -p goal_stream_timeout:=${GOAL_STREAM_TIMEOUT} \
      -p base_goal_arrive_threshold:=${BASE_GOAL_ARRIVE} \
      -r /dlio/odom_node/odom:=/lio/robo/odom \
      -r /path_manager/path_manager_ros/nav_vel:=/cmd_vel
    RET_CODE=$?
    set -e
else
    ros2 run rl_nav_controller rl_nav_controller "${POLICY_MODEL_ARG[@]}" "${FORWARD_ARGS[@]}" \
      --ros-args \
      -p depth_topic:=${DEPTH_TOPIC} \
      -p min_depth:=${CAMERA_MIN_DEPTH} \
      -p max_depth:=${CAMERA_MAX_DEPTH} \
      -p segment_arrive_threshold:=0.20 \
      -p arrive_goal_threshold:=0.20 \
      -p goal_frame_mode:=${GOAL_FRAME_MODE} \
      -p goal_stream_timeout:=${GOAL_STREAM_TIMEOUT} \
      -p base_goal_arrive_threshold:=${BASE_GOAL_ARRIVE} \
      -r /dlio/odom_node/odom:=/lio/robo/odom \
      -r /path_manager/path_manager_ros/nav_vel:=/cmd_vel
    set +e
    RET_CODE=$?
fi

cleanup
exit $RET_CODE

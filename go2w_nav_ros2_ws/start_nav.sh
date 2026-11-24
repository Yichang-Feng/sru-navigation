#!/usr/bin/env bash

# ==============================================================================
# Unitree Go2W 全局导航系统默认启动器 (start_nav.sh)
#
# 默认配置（当前实测最佳设计）：
#   1. 导航策略：RL 楼梯感知与双因子自适应控制器 (rl_nav_controller_stairs)
#   2. 平地前瞻：--curv-only + --min 0.4 + --switch-mode corner_dist
#      (按"到下一个拐角的距离"收缩前瞻；避免走廊双侧墙壁被深度误判为窄门)
#   3. 楼梯感知模式：3D 路径坡度门控，下楼前瞻锁存 0.40m，原始深度图直接下楼 (无虚假阻挡)
#   4. 容错与状态机：规范 6 状态流转，保留停滞超时报错 (REPORT_FAILURE="true")
#   5. RViz 可视化：默认不自动弹出 (ENABLE_RVIZ=false)，支持通过 --rviz 参数开启
#
# 用法：
#   ./start_nav.sh                     # 默认启动当前最优导航系统 (curv-only + corner_dist + 楼梯感知 0.4m)
#   ./start_nav.sh --rviz              # 启动并自动弹出 RViz
#   ./start_nav.sh --remapping         # 开启楼梯深度图重映射处理
#   ./start_nav.sh -h                  # 查看帮助
#
# 停止导航：在任何终端执行 ./stop_nav.sh
# ==============================================================================

set -e

SESSION="go2w_nav"
WS="$HOME/go2w_nav_ros2_ws"
ROS_SETUP="/opt/ros/humble/setup.bash"

# ==========================================
# 默认参数配置
# ==========================================
CAMERA_TYPE="realsense"         # 相机选择: "realsense" (默认, Intel RealSense D435/D435i) | "zed" (Stereolabs ZED 2i)
ENABLE_ADAPTIVE="true"
ADAPTIVE_MODE="curvature_only"  # 默认使用 curvature_only，避免窄走廊双侧墙壁被视觉误判为窄门导致前瞻锁死在 0.5m
LOOKAHEAD_MIN="0.4"
LOOKAHEAD_MAX="1.0"
WINDOW_DIST="1.6"
TURN_ANGLE_LOW="20.0"   # 曲率自适应下阈值 (deg)：小于它认为是直道
TURN_ANGLE_HIGH="50.0"  # 曲率自适应上阈值 (deg)：大于它前瞻直接收到 lookahead_min
SWITCH_MODE="corner_dist" # 前瞻切换规则: corner_dist (默认) | curvature | hybrid
CORNER_THRESH="25.0"    # corner_dist: 路径航向偏离该角度即视为"拐角" (deg)
CORNER_MARGIN="0.25"    # corner_dist: 在拐角前多少米开始收缩前瞻 (m)
SLEW_INC="0.8"          # 前瞻恢复速率 (m/s)：决定出弯/出窄门后多快回到长前瞻
SLEW_DEC="2.5"          # 前瞻收缩速率 (m/s)
ALIGN_ENTER="0.20"      # 到点转身：距离 <= 该值才允许转身
ALIGN_ABORT="0.40"      # 到点转身：距离 > 该值（扣除纯旋转漂移后）则放弃转身
ALIGN_LEVER="0.55"      # 到点转身：里程计原点相对旋转中心的杠杆臂 (m)，用于扣除原地旋转的位置漂移
DOOR_SPEED_BOOST="0.0"
REPORT_FAILURE="true"           # 默认启用停滞报错上报 (标准状态机)
ENABLE_RVIZ=false               # 默认不自动弹出 RViz (按要求关闭)

# 到达判定与楼梯感知特定参数
SEGMENT_ARRIVE_THRESHOLD="0.20"
ENABLE_STAIRS="true"
ENABLE_STAIR_REMAPPING="false"  # 默认关闭深度重映射，仅保留楼梯前瞻点 0.4m 切换测试
STAIR_LOOKAHEAD="0.4"
STAIR_UP_LOOKAHEAD="0.45"
STAIR_MIN_VEL="0.0"
STAIR_SLOPE_THRESH="-0.20"
STAIR_UP_SLOPE_THRESH="0.20"
STAIR_SIDE_CLAMP="0.50"
STAIR_MIN_DROP="0.15"

# 终点航向对齐参数
ENABLE_HEADING_ALIGN="true"     # 默认开启终点姿态对齐
HEADING_TOL="0.26"              # 默认约 15 度容差 (弧度)

# ==========================================
# 命令行参数解析
# ==========================================
show_help() {
    echo "=================================================================="
    echo "Unitree Go2W 楼梯感知与双因子自适应导航系统启动器"
    echo "用法:"
    echo "  $0 [选项]"
    echo ""
    echo "选项:"
    echo "  --realsense | --d435 使用 Intel RealSense D435/D435i 深度相机 (默认)"
    echo "  --zed               使用 Stereolabs ZED 2i 深度相机 (可选)"
    echo "  --camera <类型>     显式指定相机类型: zed | realsense"
    echo "  --curv-only         仅保留基于路径的自适应前瞻 + 楼梯感知 (防平地深度误判)"
    echo "  --door-only         仅保留局部窄门净空感知 + 楼梯感知"
    echo "  --mode <模式>        显式指定自适应模式: both | curvature_only | clearance_only | none"
    echo "  --min <距离>        平地自适应前瞻下界 (默认 0.4m)"
    echo "  --max <距离>        平地自适应前瞻上界 (默认 2.0m)"
    echo "  --window <距离>      曲率自适应前瞻窗口长度 (默认 1.6m, 建议不要 >2.0)"
    echo "  --turn-low <角度>     曲率自适应下阈值 deg (默认 20, 小于它视为直道)"
    echo "  --turn-high <角度>    曲率自适应上阈值 deg (默认 50, 大于它前瞻直接收到 min)"
    echo "  --switch-mode <规则>  前瞻切换规则: corner_dist (默认) | curvature | hybrid"
    echo "  --corner-thresh <角度> corner_dist: 路径航向偏离该角度视为拐角 (默认 25 deg)"
    echo "  --corner-margin <距离> corner_dist: 拐角前多少米开始收缩前瞻 (默认 0.25m)"
    echo "  --slew-inc <速率>     前瞻恢复速率 m/s (默认 0.8, 出弯恢复快慢的关键)"
    echo "  --slew-dec <速率>     前瞻收缩速率 m/s (默认 2.5)"
    echo "  --align-enter <距离>  到点转身进入距离 (默认 0.20m, 需 --arrive-thresh <= 此值)"
    echo "  --align-abort <距离>  到点转身放弃距离 (默认 0.40m, 已扣除原地旋转漂移)"
    echo "  --align-lever <距离>  里程计杠杆臂 (默认 0.55m, 原地旋转位置漂移补偿)"
    echo "  --arrive-thresh <距离> 终端目标到达判定距离阈值 (默认 0.20m)"
    echo "  --heading-align     开启终端航向对齐 (默认开启)"
    echo "  --no-heading-align  禁用终端航向对齐 (到达坐标即完成)"
    echo "  --heading-tol <弧度> 终端航向对齐容差 (默认 0.26 rad ≈ 15°)"
    echo "  --stair-lookahead <距离> 楼梯下楼前瞻锁存距离 (默认 0.4m, 策略前进最优响应点)"
    echo "  --stair-up-lookahead <距离> 楼梯上楼前瞻锁存距离 (默认 0.45m, 避免上楼目标梯度稀释)"
    echo "  --remapping         开启楼梯深度图重映射处理 (默认关闭: 纯原始深度图测试)"
    echo "  --no-remapping      关闭楼梯深度图重映射处理 (默认: 纯原始深度图测试)"
    echo "  --stair-min-vel <速度>   下楼保底推进线速度 (默认 0.0m/s, 纯依靠策略网络输出)"
    echo "  --stair-slope <斜率>     触发下楼判定坡度阈值 (默认 -0.20, 提前约 0.5m 判定下楼)"
    echo "  --stair-up-slope <斜率>  触发上楼判定坡度阈值 (默认 +0.20, 提前约 0.5m 判定上楼)"
    echo "  --stair-side-clamp <距离> 扶手侧边栏杆距离钳位 (默认 0.50m)"
    echo "  --stair-min-drop <高度>   触发下楼最小落差阈值 (默认 0.15m, 单阶落差)"
    echo "  --no-stairs         禁用楼梯深度图重映射与坡度门控"
    echo "  --boost <速度>      平地过门保底速度 (默认 0.35m/s, 楼梯上严格禁用)"
    echo "  --no-rviz           后台运行，不自动弹出 RViz 窗口"
    echo "  --report-failure    开启严格报错模式 (停滞报错)"
    echo "  -h | --help         显示帮助"
    echo "=================================================================="
    exit 0
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        -h|--help)
            show_help
            ;;
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
        --curv-only|--curvature-only|--curv)
            ADAPTIVE_MODE="curvature_only"
            shift
            ;;
        --door-only|--clearance-only|--door)
            ADAPTIVE_MODE="clearance_only"
            shift
            ;;
        --mode)
            ADAPTIVE_MODE="$2"
            shift 2
            ;;
        --min)
            LOOKAHEAD_MIN="$2"
            shift 2
            ;;
        --max)
            LOOKAHEAD_MAX="$2"
            shift 2
            ;;
        --arrive-thresh|--arrive-threshold)
            SEGMENT_ARRIVE_THRESHOLD="$2"
            shift 2
            ;;
        --heading-align)
            ENABLE_HEADING_ALIGN="true"
            shift
            ;;
        --no-heading-align)
            ENABLE_HEADING_ALIGN="false"
            shift
            ;;
        --heading-tol)
            HEADING_TOL="$2"
            shift 2
            ;;
        --stair-lookahead)
            STAIR_LOOKAHEAD="$2"
            shift 2
            ;;
        --stair-up-lookahead)
            STAIR_UP_LOOKAHEAD="$2"
            shift 2
            ;;
        --remapping|--enable-remapping)
            ENABLE_STAIR_REMAPPING="true"
            shift
            ;;
        --no-remapping)
            ENABLE_STAIR_REMAPPING="false"
            shift
            ;;
        --stair-slope)
            STAIR_SLOPE_THRESH="$2"
            shift 2
            ;;
        --stair-up-slope)
            STAIR_UP_SLOPE_THRESH="$2"
            shift 2
            ;;
        --stair-side-clamp)
            STAIR_SIDE_CLAMP="$2"
            shift 2
            ;;
        --stair-min-drop)
            STAIR_MIN_DROP="$2"
            shift 2
            ;;
        --stair-min-vel)
            STAIR_MIN_VEL="$2"
            shift 2
            ;;
        --no-stairs)
            ENABLE_STAIRS="false"
            shift
            ;;
        --boost)
            DOOR_SPEED_BOOST="$2"
            shift 2
            ;;
        --window)
            WINDOW_DIST="$2"
            shift 2
            ;;
        --turn-low)
            TURN_ANGLE_LOW="$2"
            shift 2
            ;;
        --turn-high)
            TURN_ANGLE_HIGH="$2"
            shift 2
            ;;
        --switch-mode)
            SWITCH_MODE="$2"
            shift 2
            ;;
        --corner-thresh)
            CORNER_THRESH="$2"
            shift 2
            ;;
        --corner-margin)
            CORNER_MARGIN="$2"
            shift 2
            ;;
        --slew-inc)
            SLEW_INC="$2"
            shift 2
            ;;
        --slew-dec)
            SLEW_DEC="$2"
            shift 2
            ;;
        --align-enter)
            ALIGN_ENTER="$2"
            shift 2
            ;;
        --align-abort)
            ALIGN_ABORT="$2"
            shift 2
            ;;
        --align-lever)
            ALIGN_LEVER="$2"
            shift 2
            ;;
        --no-adaptive)
            ENABLE_ADAPTIVE="false"
            ADAPTIVE_MODE="none"
            shift
            ;;
        --no-rviz)
            ENABLE_RVIZ=false
            shift
            ;;
        --rviz)
            ENABLE_RVIZ=true
            shift
            ;;
        --report-failure)
            REPORT_FAILURE="true"
            shift
            ;;
        --no-report-failure)
            REPORT_FAILURE="false"
            shift
            ;;
        *)
            echo "[!] 警告: 未知参数 '$1'，将被忽略。使用 -h 查看帮助。"
            shift
            ;;
    esac
done

# ==========================================
# 节点启动开关
# ==========================================
ENABLE_LIVOX=true
ENABLE_SUPER_LIO=true
ENABLE_ZED=true
ENABLE_GO2W_BRIDGE=true
ENABLE_GLOBAL_PLANNER=true
ENABLE_LOCAL_PLANNER=false
ENABLE_RL_STAIRS_PLANNER=true

# ==========================================
# 环境检查
# ==========================================
if ! command -v tmux &>/dev/null; then
    echo "错误：未安装 tmux"
    exit 1
fi

if [ ! -f "$ROS_SETUP" ]; then
    echo "错误：ROS2 环境不存在：$ROS_SETUP"
    exit 1
fi

if [ ! -f "$WS/install/setup.bash" ]; then
    echo "错误：工作空间环境不存在：$WS/install/setup.bash"
    exit 1
fi

if tmux has-session -t "=$SESSION" 2>/dev/null; then
    echo "[!] 提示: 导航系统已经在后台运行 (tmux 会话: $SESSION)。"
    echo "    若想重新以新参数启动，请先在终端运行: ./stop_nav.sh"
    echo "    正在连接现有会话..."
    if [ -z "${TMUX:-}" ]; then
        tmux attach-session -t "$SESSION"
    else
        tmux switch-client -t "$SESSION"
    fi
    exit 0
fi

echo "=================================================================="
echo ">>> Unitree Go2W 楼梯感知与双因子自适应导航系统启动器"
echo ">>> 感知相机: $CAMERA_TYPE (可在启动时通过 --realsense 或 --zed 切换)"
echo ">>> 楼梯感知使能: $ENABLE_STAIRS [下楼前瞻: ${STAIR_LOOKAHEAD}m, 上楼前瞻: ${STAIR_UP_LOOKAHEAD}m, 深度重映射: ${ENABLE_STAIR_REMAPPING}, 下坡阈值: ${STAIR_SLOPE_THRESH}, 上坡阈值: ${STAIR_UP_SLOPE_THRESH}, 到达阈值: ${SEGMENT_ARRIVE_THRESHOLD}m]"
echo ">>> 平地前瞻自适应使能: $ENABLE_ADAPTIVE [模式: $ADAPTIVE_MODE, 范围: ${LOOKAHEAD_MIN}m ~ ${LOOKAHEAD_MAX}m]"
echo ">>> 前瞻切换规则: $SWITCH_MODE [拐角阈值: ${CORNER_THRESH}°, 提前量: ${CORNER_MARGIN}m, 窗口: ${WINDOW_DIST}m, 恢复/收缩: ${SLEW_INC}/${SLEW_DEC} m/s]"
echo ">>> 容错模式: $REPORT_FAILURE, 自动打开 RViz: $ENABLE_RVIZ"
echo "=================================================================="

NAMES=()
COMMANDS=()
DELAYS=()

add_node() {
    NAMES+=("$1")
    COMMANDS+=("$2")
    DELAYS+=("$3")
}

if [ "$ENABLE_LIVOX" = true ]; then
    add_node "Livox MID360" \
        "ros2 launch livox_ros_driver2 msg_MID360_launch.py" 3
fi

if [ "$ENABLE_SUPER_LIO" = true ]; then
    add_node "Super-LIO" \
        "ros2 launch super_lio map_localization.py" 3
fi

if [ "$CAMERA_TYPE" = "realsense" ]; then
    DEPTH_TOPIC="/camera/camera/depth/image_rect_raw"
    CAMERA_MIN_DEPTH="0.15"
    CAMERA_MAX_DEPTH="10.0"
    # 后台静默发布 RealSense 静态坐标变换 (base_link -> camera_link)，不占用额外 tmux 分屏
    pkill -f "static_transform_publisher.*camera_link" 2>/dev/null || true
    nohup ros2 run tf2_ros static_transform_publisher 0.34 0.0 0.25 0.0 0.0 0.0 base_link camera_link >/dev/null 2>&1 &
    RS_CFG="$WS/src/rl_nav_controller/config/realsense_nav_override.yaml"
    if [ -f "$RS_CFG" ]; then
        add_node "RealSense D435i Camera" \
            "ros2 launch realsense2_camera rs_launch.py config_file:='$RS_CFG' depth_module.profile:=640x480x15 rgb_camera.profile:=640x480x15 align_depth.enable:=false pointcloud.enable:=false clip_distance:=-1.0 filters:=spatial,temporal,hole_filling" 3
    else
        add_node "RealSense D435i Camera" \
            "ros2 launch realsense2_camera rs_launch.py depth_module.profile:=640x480x15 rgb_camera.profile:=640x480x15 align_depth.enable:=false pointcloud.enable:=false clip_distance:=-1.0 filters:=spatial,temporal,hole_filling" 3
    fi
else
    DEPTH_TOPIC="/zed/zed_node/depth/depth_registered"
    CAMERA_MIN_DEPTH="0.25"
    CAMERA_MAX_DEPTH="10.0"
    ZED_CFG="$WS/src/rl_nav_controller/config/zed_nav_override.yaml"
    if [ -f "$ZED_CFG" ]; then
        add_node "ZED 2i Camera" \
            "ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i ros_params_override_path:='$ZED_CFG'" 3
    else
        add_node "ZED 2i Camera" \
            "ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i" 3
    fi
fi

if [ "$ENABLE_GO2W_BRIDGE" = true ]; then
    add_node "Go2W Bridge" \
        "ros2 launch go2w_bridge go2w_bridge.launch.py" 1
fi

if [ "$ENABLE_GLOBAL_PLANNER" = true ]; then
    add_node "Global Planner" \
        "ros2 launch global_planner global_planner.launch.py" 2
fi

if [ "$ENABLE_LOCAL_PLANNER" = true ]; then
    add_node "Local Planner" \
        "ros2 launch local_planner run.launch.py" 2
fi

if [ "$ENABLE_RL_STAIRS_PLANNER" = true ]; then
    RL_ARGS="--ros-args \
        -p depth_topic:=${DEPTH_TOPIC} \
        -p min_depth:=${CAMERA_MIN_DEPTH} \
        -p max_depth:=${CAMERA_MAX_DEPTH} \
        -p enable_stairs_mode:=${ENABLE_STAIRS} \
        -p enable_stair_remapping:=${ENABLE_STAIR_REMAPPING} \
        -p stair_lookahead_dist:=${STAIR_LOOKAHEAD} \
        -p stair_up_lookahead_dist:=${STAIR_UP_LOOKAHEAD} \
        -p stair_min_forward_vel:=${STAIR_MIN_VEL} \
        -p stair_slope_threshold:=${STAIR_SLOPE_THRESH} \
        -p stair_up_slope_threshold:=${STAIR_UP_SLOPE_THRESH} \
        -p stair_side_min_clamp:=${STAIR_SIDE_CLAMP} \
        -p stair_min_elevation_drop:=${STAIR_MIN_DROP} \
        -p enable_adaptive_lookahead:=${ENABLE_ADAPTIVE} \
        -p adaptive_mode:=${ADAPTIVE_MODE} \
        -p lookahead_min:=${LOOKAHEAD_MIN} \
        -p lookahead_max:=${LOOKAHEAD_MAX} \
        -p adaptive_window_dist:=${WINDOW_DIST} \
        -p turn_angle_low_deg:=${TURN_ANGLE_LOW} \
        -p turn_angle_high_deg:=${TURN_ANGLE_HIGH} \
        -p switch_mode:=${SWITCH_MODE} \
        -p corner_switch_thresh_deg:=${CORNER_THRESH} \
        -p corner_switch_margin:=${CORNER_MARGIN} \
        -p slew_rate_increase:=${SLEW_INC} \
        -p slew_rate_decrease:=${SLEW_DEC} \
        -p terminal_align_enter_dist:=${ALIGN_ENTER} \
        -p terminal_align_abort_dist:=${ALIGN_ABORT} \
        -p terminal_align_lever_arm:=${ALIGN_LEVER} \
        -p door_speed_boost:=${DOOR_SPEED_BOOST} \
        -p segment_arrive_threshold:=${SEGMENT_ARRIVE_THRESHOLD} \
        -p enable_terminal_heading_align:=${ENABLE_HEADING_ALIGN} \
        -p terminal_heading_tol:=${HEADING_TOL} \
        -p report_failure:=${REPORT_FAILURE}"

    # 直接执行 Python 脚本源码并显式注入包路径，彻底规避 setuptools entry_points 元数据脱节与包名遮蔽
    RL_CMD="PYTHONPATH=\"\$WS/src/rl_nav_controller:\${PYTHONPATH:-}\" python3 -u src/rl_nav_controller/rl_nav_controller/rl_nav_controller_stairs.py $RL_ARGS"
    add_node "RL Stairs-Aware Local Planner" "$RL_CMD" 0
fi

COUNT=${#NAMES[@]}
if [ "$COUNT" -eq 0 ]; then
    echo "没有启用任何 ROS2 节点！"
    exit 1
fi

# ==========================================
# 创建 tmux 会话和动态分屏
# ==========================================
tmux new-session -d -s "$SESSION" -n navigation
tmux set-option -t "$SESSION" mouse on
tmux set-option -t "$SESSION" pane-border-status top
tmux set-option -t "$SESSION" pane-border-format " #{pane_title} "
tmux set-option -t "$SESSION" remain-on-exit on

PANES=()
FIRST_PANE=$(tmux display-message -p -t "$SESSION:0.0" '#{pane_id}')
PANES+=("$FIRST_PANE")

for ((i=1; i<COUNT; i++)); do
    NEW_PANE=$(tmux split-window -t "$SESSION:0" -P -F '#{pane_id}')
    PANES+=("$NEW_PANE")
    tmux select-layout -t "$SESSION:0" tiled
done

if [ "$COUNT" -gt 1 ]; then
    tmux select-layout -t "$SESSION:0" tiled
fi

# ==========================================
# 启动 ROS2 节点
# ==========================================
for ((i=0; i<COUNT; i++)); do
    NAME="${NAMES[$i]}"
    CMD="${COMMANDS[$i]}"
    PANE="${PANES[$i]}"
    DELAY="${DELAYS[$i]}"

    echo "[$((i+1))/$COUNT] 正在启动: $NAME"
    tmux select-pane -t "$PANE" -T "$NAME"
    tmux send-keys -t "$PANE" \
        "source '$ROS_SETUP' && cd '$WS' && source install/setup.bash && $CMD" C-m

    if [ "$DELAY" -gt 0 ] && [ "$i" -lt $((COUNT-1)) ]; then
        sleep "$DELAY"
    fi
done

tmux select-pane -t "${PANES[0]}"

# ==========================================
# 自动启动 RViz (可选)
# ==========================================
if [ "$ENABLE_RVIZ" = true ]; then
    RVIZ_CFG="$WS/rviz.rviz"
    if [ ! -f "$RVIZ_CFG" ]; then
        RVIZ_CFG="$WS/src/rl_nav_controller/rviz/rviz.rviz"
    fi
    if [ -f "$RVIZ_CFG" ]; then
        echo "[✓] 正在后台启动 RViz 可视化界面 (配置文件: $RVIZ_CFG)..."
        nohup bash -c "source '$ROS_SETUP' && source '$WS/install/setup.bash' && ros2 run rviz2 rviz2 -d '$RVIZ_CFG'" >/dev/null 2>&1 &
    fi
fi

echo ""
echo "=================================================================="
echo ">>> Unitree Go2W 楼梯感知与双因子自适应导航系统启动完成！"
echo ">>> 楼梯诊断监控: ros2 topic echo /nav/is_descending_stairs  |  ros2 topic echo /nav/is_ascending_stairs"
echo ">>> 手柄操作: [A] 启动/恢复导航  |  [B] 暂停刹停"
echo ">>> 停止运行: ./stop_nav.sh"
echo "=================================================================="

if [ -z "${TMUX:-}" ]; then
    tmux attach-session -t "$SESSION"
else
    tmux switch-client -t "$SESSION"
fi

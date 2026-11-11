#!/usr/bin/env bash

# ==============================================================================
# Unitree Go2W 全局导航系统默认启动器 (start_nav.sh)
#
# 默认配置（当前实测最佳设计）：
#   1. 导航策略：RL 楼梯感知与双因子自适应控制器 (rl_nav_controller_stairs)
#   2. 平地前瞻模式：--curv-only (Path 曲率转弯自适应，避免走廊双侧墙壁被视觉误判)
#   3. 楼梯感知模式：3D 路径坡度门控，下楼前瞻锁存 0.40m，原始深度图直接下楼 (无虚假阻挡)
#   4. 容错与状态机：规范 6 状态流转，保留停滞超时报错 (REPORT_FAILURE="true")
#   5. RViz 可视化：默认不自动弹出 (ENABLE_RVIZ=false)，支持通过 --rviz 参数开启
#
# 用法：
#   ./start_nav.sh                     # 默认启动当前最优导航系统 (curv-only + 楼梯感知 0.4m)
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
ENABLE_ADAPTIVE="true"
ADAPTIVE_MODE="curvature_only"  # 默认使用 curvature_only，避免窄走廊双侧墙壁被视觉误判为窄门导致前瞻锁死在 0.5m
LOOKAHEAD_MIN="0.5"
LOOKAHEAD_MAX="2.0"
WINDOW_DIST="1.6"
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
    echo "  --curv-only         仅保留 Path 曲率转弯判断 + 楼梯感知 (防平地视觉误判)"
    echo "  --door-only         仅保留局部窄门净空感知 + 楼梯感知"
    echo "  --mode <模式>        显式指定自适应模式: both | curvature_only | clearance_only | none"
    echo "  --min <距离>        平地自适应前瞻下界 (默认 0.5m)"
    echo "  --max <距离>        平地自适应前瞻上界 (默认 2.0m)"
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
echo ">>> 楼梯感知使能: $ENABLE_STAIRS [下楼前瞻: ${STAIR_LOOKAHEAD}m, 上楼前瞻: ${STAIR_UP_LOOKAHEAD}m, 深度重映射: ${ENABLE_STAIR_REMAPPING}, 下坡阈值: ${STAIR_SLOPE_THRESH}, 上坡阈值: ${STAIR_UP_SLOPE_THRESH}, 到达阈值: ${SEGMENT_ARRIVE_THRESHOLD}m]"
echo ">>> 平地前瞻自适应使能: $ENABLE_ADAPTIVE [模式: $ADAPTIVE_MODE, 范围: ${LOOKAHEAD_MIN}m ~ ${LOOKAHEAD_MAX}m]"
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

if [ "$ENABLE_ZED" = true ]; then
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
        -p door_speed_boost:=${DOOR_SPEED_BOOST} \
        -p segment_arrive_threshold:=${SEGMENT_ARRIVE_THRESHOLD} \
        -p enable_terminal_heading_align:=${ENABLE_HEADING_ALIGN} \
        -p terminal_heading_tol:=${HEADING_TOL} \
        -p report_failure:=${REPORT_FAILURE}"

    # 自动兼容：优先检测 ros2 entrypoint，若尚未编译注册则无缝回退到 python3 源码直运
    RL_CMD="if ros2 pkg executables rl_nav_controller 2>/dev/null | grep -q 'rl_nav_controller_stairs'; then ros2 run rl_nav_controller rl_nav_controller_stairs $RL_ARGS; else python3 src/rl_nav_controller/rl_nav_controller/rl_nav_controller_stairs.py $RL_ARGS; fi"
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

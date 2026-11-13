#!/usr/bin/env bash

# ==============================================================================
# Unitree Go2W 双因子自适应动态前瞻导航系统启动器 (start_nav_adaptive.sh)
#
# 特色能力：
#   1. 双因子自适应前瞻 (Dual-Factor Adaptive Lookahead)：
#      - 因子一：Path 前向滑动窗口曲率自适应 (急弯提前降速至 0.5m，防切内角撞门框)
#      - 因子二：深度图局部通道净空自适应与空间锁存 (过窄门锁定 0.5m，出关平滑恢复)
#   2. 门洞对准推力补偿 (Door Speed Boost)：
#      - 解决 L=0.5m 时对数距离引起的蠕动爬行，当朝向对正门洞时赋予保底速度 (0.35m/s)
#   3. 零侵入原有系统：完全独立启动，原版 start_nav.sh 与 start_nav_custom.sh 保持不变
#
# 用法：
#   ./start_nav_adaptive.sh                     # 默认启动双因子自适应 (0.5m ~ 2.0m)
#   ./start_nav_adaptive.sh --min 0.4 --max 1.8 # 自定义前瞻范围
#   ./start_nav_adaptive.sh --no-rviz           # 仅启动后台节点，不打开 RViz
#   ./start_nav_adaptive.sh -h                  # 查看帮助
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
ADAPTIVE_MODE="both"  # "both" | "curvature_only" | "clearance_only" | "none"
LOOKAHEAD_MIN="0.5"
LOOKAHEAD_MAX="2.0"
WINDOW_DIST="1.5"
DOOR_SPEED_BOOST="0.35"
REPORT_FAILURE="false"
ENABLE_RVIZ=true

# ==========================================
# 命令行参数解析
# ==========================================
show_help() {
    echo "=================================================================="
    echo "Unitree Go2W 双因子自适应动态前瞻导航系统启动器"
    echo "用法:"
    echo "  $0 [选项]"
    echo ""
    echo "选项:"
    echo "  --curv-only         【测试推荐】仅保留 Path 曲率转弯判断 (彻底关闭视觉窄门检测，防视觉误判)"
    echo "  --door-only         【测试模式】仅保留局部窄门净空感知 (关闭 Path 曲率预判)"
    echo "  --mode <模式>        显式指定模式: both | curvature_only | clearance_only | none"
    echo "  --min <距离>        设置自适应前瞻下界 (默认 0.5m)"
    echo "  --max <距离>        设置自适应前瞻上界 (默认 2.0m)"
    echo "  --boost <速度>      设置过门保底速度 (默认 0.35m/s)"
    echo "  --window <距离>     设置曲率前瞻滑动窗口距离 (默认 1.5m)"
    echo "  --no-adaptive       禁用自适应前瞻 (使用固定 2.0m)"
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
ENABLE_RL_LOCAL_PLANNER=true

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
echo ">>> Unitree Go2W 双因子自适应动态前瞻导航系统启动器"
echo ">>> 前瞻自适应使能: $ENABLE_ADAPTIVE [模式: $ADAPTIVE_MODE]"
echo ">>> 前瞻范围: ${LOOKAHEAD_MIN}m ~ ${LOOKAHEAD_MAX}m"
echo ">>> 前向曲率探查窗口: ${WINDOW_DIST}m, 窄门推力保底: ${DOOR_SPEED_BOOST}m/s"
echo ">>> 容错模式 (report_failure): $REPORT_FAILURE"
echo ">>> 自动打开 RViz: $ENABLE_RVIZ"
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

if [ "$ENABLE_RL_LOCAL_PLANNER" = true ]; then
    RL_ARGS="--ros-args -p enable_adaptive_lookahead:=${ENABLE_ADAPTIVE} -p adaptive_mode:=${ADAPTIVE_MODE} -p lookahead_min:=${LOOKAHEAD_MIN} -p lookahead_max:=${LOOKAHEAD_MAX} -p adaptive_window_dist:=${WINDOW_DIST} -p door_speed_boost:=${DOOR_SPEED_BOOST} -p report_failure:=${REPORT_FAILURE}"
    # 自动兼容：优先检测 ros2 entrypoint，若尚未编译注册则无缝回退到 python3 源码直运，100% 防止 'no executable found'
    RL_CMD="if ros2 pkg executables rl_nav_controller 2>/dev/null | grep -q 'rl_nav_controller_adaptive'; then ros2 run rl_nav_controller rl_nav_controller_adaptive $RL_ARGS; else python3 src/rl_nav_controller/rl_nav_controller/rl_nav_controller_adaptive.py $RL_ARGS; fi"
    add_node "RL Adaptive Local Planner" "$RL_CMD" 0
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
echo ">>> Unitree Go2W 双因子自适应导航系统启动完成！"
echo ">>> 手柄操作: [A] 启动/恢复导航  |  [B] 暂停刹停"
echo ">>> 停止运行: ./stop_nav.sh"
echo "=================================================================="

if [ -z "${TMUX:-}" ]; then
    tmux attach-session -t "$SESSION"
else
    tmux switch-client -t "$SESSION"
fi

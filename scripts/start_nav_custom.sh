#!/usr/bin/env bash

# ==============================================================================
# Unitree Go2W 自主避障导航系统启动器 (自定义前瞻距离 & 容错增强版)
#
# 基础架构：基于原版稳定运行的 start_nav.sh 全局多节点 tmux 协同系统
# 新增功能：
#   1. 命令行参数配置前瞻动态胡萝卜点距离 (lookahead carrot distance，默认 2.0m)
#   2. 容错运行：默认不上报 error / failed，避障停滞不锁死，深度帧短暂延迟不报错
#   3. 手柄映射：[按钮 A] 启动/恢复导航；[按钮 B] 暂停/刹停
#   4. RViz 可视化：自动挂载 rviz.rviz (含全局 PCD 地图、实时激光、橙色胡萝卜点、下点工具)
#
# 使用方式：
#   ./start_nav_custom.sh              # 默认前瞻 2.0m，不上报 error/failed
#   ./start_nav_custom.sh 1.5          # 设置前瞻 1.5m
#   ./start_nav_custom.sh -l 2.5       # 设置前瞻 2.5m
#   ./start_nav_custom.sh --no-rviz    # 不自动打开 RViz 窗口 (纯后台运行)
#   ./start_nav_custom.sh -h           # 查看详细帮助信息
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
LOOKAHEAD_DIST="2.0"
REPORT_FAILURE="false"
ENABLE_RVIZ=true

# ==========================================
# 命令行参数解析
# ==========================================
show_help() {
    echo "=================================================================="
    echo "Unitree Go2W 导航系统一键启动脚本 (容错与前瞻距离控制版)"
    echo "用法:"
    echo "  $0 [前瞻距离(米)] [选项]"
    echo ""
    echo "示例:"
    echo "  $0                 # 默认前瞻 2.0m，不上报 error/failed"
    echo "  $0 1.5             # 设置前瞻胡萝卜点距离为 1.5m"
    echo "  $0 -l 2.5          # 设置前瞻胡萝卜点距离为 2.5m"
    echo "  $0 --no-rviz       # 仅启动后台节点，不弹 RViz 窗口"
    echo "  $0 --report-failure # 恢复严格模式 (上报 error/failed 并在停滞时自锁)"
    echo "  $0 -h | --help     # 显示此帮助"
    echo "=================================================================="
    exit 0
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        -h|--help)
            show_help
            ;;
        -l|--lookahead|--lookahead-distance)
            LOOKAHEAD_DIST="$2"
            shift 2
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
            # 如果是纯数字或浮点数，直接作为前瞻距离
            if [[ "$1" =~ ^[0-9]+(\.[0-9]+)?$ ]]; then
                LOOKAHEAD_DIST="$1"
                shift
            else
                echo "[!] 警告: 未知参数 '$1'，将被忽略。使用 -h 查看帮助。"
                shift
            fi
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

# 若已有同名 session 运行中，提示用户
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

# ==========================================
# 打印启动摘要信息
# ==========================================
echo "=================================================================="
echo ">>> Unitree Go2W 自主避障导航系统启动器"
echo ">>> 萝卜点前瞻距离 (Lookahead Distance) : ${LOOKAHEAD_DIST} 米"
echo ">>> 失败与错误状态上报 (Report Failure) : ${REPORT_FAILURE} (不上报/停滞不锁死)"
echo ">>> 手柄控制说明:"
echo "    - [按钮 A (Button 0)] : 启动 / 恢复导航 (Start / Resume)"
echo "    - [按钮 B (Button 1)] : 暂停 / 即时刹停 (Pause / Stop)"
echo "=================================================================="

# ==========================================
# 配置启用的节点
# ==========================================
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
    add_node "RL Local Planner" \
        "ros2 launch rl_nav_controller rl_nav_controller.launch.py lookahead_distance:=${LOOKAHEAD_DIST} report_failure:=${REPORT_FAILURE}" 0
fi

# RViz 窗口拉起 (仅在检测到图形界面 DISPLAY 时启动)
if [ "$ENABLE_RVIZ" = true ]; then
    if [ -n "${DISPLAY:-}" ]; then
        if [ -f "$WS/rviz.rviz" ]; then
            add_node "RViz2 Visualizer" \
                "rviz2 -d '$WS/rviz.rviz'" 0
        else
            add_node "RViz2 Visualizer" \
                "rviz2" 0
        fi
    else
        echo "[i] 未检测到 DISPLAY 显示环境变量 (可能为纯 SSH 登录)，已跳过自动拉起 RViz 窗口。"
    fi
fi

COUNT=${#NAMES[@]}
if [ "$COUNT" -eq 0 ]; then
    echo "错误：没有启用任何 ROS2 节点！"
    exit 1
fi

echo "========================================"
echo "ROS2 导航系统启动中... 启用节点数：$COUNT"
echo "========================================"

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
# 依次启动 ROS2 节点
# ==========================================
for ((i=0; i<COUNT; i++)); do
    NAME="${NAMES[$i]}"
    CMD="${COMMANDS[$i]}"
    PANE="${PANES[$i]}"
    DELAY="${DELAYS[$i]}"

    echo "[$((i+1))/$COUNT] 启动 $NAME"
    tmux select-pane -t "$PANE" -T "$NAME"
    tmux send-keys -t "$PANE" \
        "source '$ROS_SETUP' && cd '$WS' && source install/setup.bash && $CMD" C-m

    if [ "$DELAY" -gt 0 ] && [ "$i" -lt $((COUNT-1)) ]; then
        sleep "$DELAY"
    fi
done

tmux select-pane -t "${PANES[0]}"

echo ""
echo "========================================"
echo "导航启动命令已全部发送！"
echo "tmux 会话：$SESSION"
echo "========================================"

# ==========================================
# 进入 tmux
# ==========================================
if [ -z "${TMUX:-}" ]; then
    tmux attach-session -t "$SESSION"
else
    tmux switch-client -t "$SESSION"
fi

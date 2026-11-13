
#!/usr/bin/env bash

# ==========================================
# ROS2 导航系统动态分屏启动脚本
# ==========================================

SESSION="go2w_nav"
WS="$HOME/go2w_nav_ros2_ws"
ROS_SETUP="/opt/ros/humble/setup.bash"

# ==========================================
# 1. 节点启动开关
# true  = 启动
# false = 不启动
# ==========================================

ENABLE_LIVOX=true
ENABLE_SUPER_LIO=true
ENABLE_ZED=true
ENABLE_GO2W_BRIDGE=true
ENABLE_GLOBAL_PLANNER=true
ENABLE_LOCAL_PLANNER=false
ENABLE_RL_LOCAL_PLANNER=true

# ==========================================
# 2. 环境检查
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
    echo "错误：工作空间环境不存在"
    exit 1
fi

# 已有会话则进入，不重复启动
if tmux has-session -t "=$SESSION" 2>/dev/null; then
    echo "导航系统已经运行"
    if [ -z "${TMUX:-}" ]; then
        tmux attach-session -t "$SESSION"
    else
        tmux switch-client -t "$SESSION"
    fi
    exit 0
fi

# ==========================================
# 3. 配置启用的节点
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
        "ros2 launch rl_nav_controller rl_nav_controller.launch.py" 0
fi

COUNT=${#NAMES[@]}

if [ "$COUNT" -eq 0 ]; then
    echo "没有启用任何 ROS2 节点！"
    exit 1
fi

echo "========================================"
echo "ROS2 导航系统"
echo "启用节点数量：$COUNT"
echo "========================================"

# ==========================================
# 4. 创建 tmux 会话和动态分屏
# ==========================================

tmux new-session -d -s "$SESSION" -n navigation

# 启用鼠标
tmux set-option -t "$SESSION" mouse on

# 显示分屏标题
tmux set-option -t "$SESSION" pane-border-status top
tmux set-option -t "$SESSION" \
    pane-border-format " #{pane_title} "

# 节点退出后保留日志
tmux set-option -t "$SESSION" remain-on-exit on

# 先记录首个 pane ID
PANES=()
FIRST_PANE=$(tmux display-message -p \
    -t "$SESSION:0.0" '#{pane_id}')
PANES+=("$FIRST_PANE")

# 动态创建其余 pane
for ((i=1; i<COUNT; i++)); do
    # 使用新 pane ID，避免 pane 索引变化
    NEW_PANE=$(tmux split-window \
        -t "$SESSION:0" \
        -P -F '#{pane_id}')

    PANES+=("$NEW_PANE")

    # 根据节点数量自动排列
    tmux select-layout -t "$SESSION:0" tiled
done

# 最终统一排列
if [ "$COUNT" -gt 1 ]; then
    tmux select-layout -t "$SESSION:0" tiled
fi

# ==========================================
# 5. 依次启动 ROS2 节点
# ==========================================

for ((i=0; i<COUNT; i++)); do

    NAME="${NAMES[$i]}"
    CMD="${COMMANDS[$i]}"
    PANE="${PANES[$i]}"
    DELAY="${DELAYS[$i]}"

    echo "[$((i+1))/$COUNT] 启动 $NAME"

    # 设置分屏标题
    tmux select-pane -t "$PANE" -T "$NAME"

    # 在对应 pane 中启动 ROS2
    tmux send-keys -t "$PANE" \
        "source '$ROS_SETUP' && cd '$WS' && source install/setup.bash && $CMD" C-m

    # 启动间隔（不是就绪检测）
    if [ "$DELAY" -gt 0 ] && [ "$i" -lt $((COUNT-1)) ]; then
        sleep "$DELAY"
    fi
done

# 选中第一个分屏
tmux select-pane -t "${PANES[0]}"

echo ""
echo "========================================"
echo "导航启动命令已全部发送！"
echo "tmux 会话：$SESSION"
echo "========================================"

# ==========================================
# 6. 进入 tmux
# ==========================================

if [ -z "${TMUX:-}" ]; then
    tmux attach-session -t "$SESSION"
else
    tmux switch-client -t "$SESSION"
fi


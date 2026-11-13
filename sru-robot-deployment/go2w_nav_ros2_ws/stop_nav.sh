#!/usr/bin/env bash

# ==========================================
# ROS2 导航系统安全停止脚本
# ==========================================

SESSION="go2w_nav"
ROS_SETUP="/opt/ros/humble/setup.bash"

if [ -f "$ROS_SETUP" ]; then
    source "$ROS_SETUP" 2>/dev/null || true
fi

echo "正在安全停止 ROS2 导航系统与底盘运动..."

# 1. 主动发布置零速度与锁定指令，确保机器狗底盘物理制动安全
timeout 1 ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once >/dev/null 2>&1 || true
timeout 1 ros2 topic pub /motion_enabled std_msgs/msg/Bool "{data: false}" --once >/dev/null 2>&1 || true
timeout 1 ros2 topic pub /startNavi std_msgs/msg/Bool "{data: false}" --once >/dev/null 2>&1 || true

if ! tmux has-session -t "=$SESSION" 2>/dev/null; then
    echo "tmux 导航会话未运行"
    exit 0
fi

# 2. 获取会话中的所有分屏并发送 Ctrl+C
mapfile -t PANES < <(
    tmux list-panes -s -t "$SESSION" -F '#{pane_id}'
)

for PANE in "${PANES[@]}"; do
    tmux send-keys -t "$PANE" C-c
done

# 等待进程响应 Ctrl+C
sleep 2

# 3. 关闭整个会话
tmux kill-session -t "=$SESSION" 2>/dev/null || true

# 4. 兜底清理可能残留的节点进程
pkill -f "map_localization_node" 2>/dev/null || true
pkill -f "livox_ros_driver2_node" 2>/dev/null || true
pkill -f "zed_wrapper" 2>/dev/null || true
pkill -f "realsense2_camera" 2>/dev/null || true
pkill -f "rl_nav_controller" 2>/dev/null || true
pkill -f "global_planner_node" 2>/dev/null || true
pkill -f "cmd_vel_bridge" 2>/dev/null || true
pkill -f "wireless_controller" 2>/dev/null || true

echo "导航系统已安全停止，底盘已锁定，tmux 会话已关闭。"


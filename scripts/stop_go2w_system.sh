#!/usr/bin/env bash
# ==============================================================================
# Unitree Go2W 自主避障导航全系统一键停止脚本 (stop_go2w_system.sh)
# ==============================================================================

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${YELLOW}[!] 正在紧急停止 Go2W 导航系统所有节点...${NC}"

# 1. 立即发布底盘制动锁，确保机器人立刻停止运动
source /opt/ros/humble/setup.bash 2>/dev/null || true
ros2 topic pub /startNavi std_msgs/msg/Bool "{data: false}" --once >/dev/null 2>&1 || true

# 2. 终止各节点
pkill -f "rl_nav_controller" 2>/dev/null && echo -e "${GREEN}[✓] rl_nav_controller 已停止${NC}" || true
pkill -f "relocation_node" 2>/dev/null && echo -e "${GREEN}[✓] super_lio relocation_node 已停止${NC}" || true
pkill -f "livox_ros_driver2_node" 2>/dev/null && echo -e "${GREEN}[✓] livox_ros_driver2 已停止${NC}" || true
pkill -f "zed_wrapper" 2>/dev/null && echo -e "${GREEN}[✓] zed_camera 已停止${NC}" || true
pkill -f "static_transform_publisher.*zed_camera_link" 2>/dev/null && echo -e "${GREEN}[✓] 相机静态 TF 已停止${NC}" || true
pkill -f "rviz2" 2>/dev/null && echo -e "${GREEN}[✓] RViz2 已关闭${NC}" || true

echo -e "${GREEN}[✓] Go2W 导航系统已全部安全关闭。${NC}"

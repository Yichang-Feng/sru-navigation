#!/usr/bin/env bash
# ==============================================================================
# Unitree Go2W 单独启动局部 SRU 避障导航系统停止脚本 (stop_local_sru.sh)
#
# 说明：用于安全停止单独启动的局部 SRU 避障导航系统所有节点，并向底盘发布制动锁
# ==============================================================================

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${YELLOW}[!] 正在安全停止局部 SRU 导航系统所有节点...${NC}"

# 1. 立即发布底盘制动锁，确保机器人立刻停止运动
source /opt/ros/humble/setup.bash 2>/dev/null || true
ros2 topic pub /motion_enabled std_msgs/msg/Bool "{data: false}" --once >/dev/null 2>&1 || true
ros2 topic pub /startNavi std_msgs/msg/Bool "{data: false}" --once >/dev/null 2>&1 || true

# 2. 终止各节点
pkill -f "rl_nav_controller" 2>/dev/null && echo -e "${GREEN}[✓] rl_nav_controller 已停止${NC}" || true
pkill -f "cmd_vel_bridge" 2>/dev/null && echo -e "${GREEN}[✓] cmd_vel_bridge 已停止${NC}" || true
pkill -f "wireless_controller" 2>/dev/null && echo -e "${GREEN}[✓] wireless_controller 已停止${NC}" || true
pkill -f "unitree_bridge" 2>/dev/null && echo -e "${GREEN}[✓] unitree_bridge 已停止${NC}" || true
pkill -f "map_localization_node" 2>/dev/null && echo -e "${GREEN}[✓] super_lio map_localization_node 已停止${NC}" || true
pkill -f "relocation_node" 2>/dev/null && echo -e "${GREEN}[✓] super_lio relocation_node 已停止${NC}" || true
pkill -f "livox_ros_driver2_node" 2>/dev/null && echo -e "${GREEN}[✓] livox_ros_driver2 已停止${NC}" || true
pkill -9 -f "realsense2_camera" 2>/dev/null && echo -e "${GREEN}[✓] realsense2_camera 已停止 (硬件已释放)${NC}" || true
pkill -9 -f "zed_wrapper" 2>/dev/null && echo -e "${GREEN}[✓] zed_camera 已停止 (硬件已释放)${NC}" || true
pkill -9 -f "static_transform_publisher.*camera_link" 2>/dev/null && echo -e "${GREEN}[✓] 相机静态 TF 已停止${NC}" || true
pkill -9 -f "static_transform_publisher.*zed_camera_link" 2>/dev/null && echo -e "${GREEN}[✓] ZED 静态 TF 已停止${NC}" || true
pkill -9 -f "rviz2" 2>/dev/null && echo -e "${GREEN}[✓] RViz2 已关闭${NC}" || true

echo -e "${GREEN}[✓] Go2W 导航系统已全部安全关闭。${NC}"

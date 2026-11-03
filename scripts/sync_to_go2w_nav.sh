#!/usr/bin/env bash
# ==============================================================================
# Unitree Go2W 导航系统一键同步与真机部署脚本 (sync_to_go2w_nav.sh)
#
# 用途：将本地已完成对齐的 rl_nav_controller 以及更新后的 start_nav.sh / stop_nav.sh
#       一键推送到机器狗车载算力平台 (agile@<ROBOT_IP>:~/go2w_nav_ros2_ws/) 并编译生效
# ==============================================================================

set -e

# 支持通过命令行传参覆盖目标 IP: bash scripts/sync_to_go2w_nav.sh [IP]
ROBOT_IP="${1:-${ROBOT_IP:-10.3.42.161}}"
ROBOT_USER="agile"

LOCAL_WS="/home/yichangfeng/go2w_robot_ref/go2w_nav_ros2_ws"
SRC_PKG_DIR="${LOCAL_WS}/src/rl_nav_controller"

echo "=================================================================="
echo ">>> Unitree Go2W 导航系统一键同步器"
echo ">>> 目标机器人: ${ROBOT_USER}@${ROBOT_IP}"
echo ">>> 目标工作空间: ~/go2w_nav_ros2_ws"
echo "=================================================================="

# 1. 检查网络连通性
if ! ping -c 1 -W 1 "$ROBOT_IP" > /dev/null 2>&1; then
    echo "[!] 警告: 无法 ping 通 $ROBOT_IP，请确认是否已连接机器狗车载局域网。"
    echo "    若网络正常但 ping 被机载防火墙禁用，将直接尝试 SSH/RSYNC 传输..."
fi

# 2. 检查本地源码存在性
if [ ! -d "$SRC_PKG_DIR" ]; then
    echo "[✗] 错误: 本地未找到 $SRC_PKG_DIR"
    exit 1
fi

# 3. 同步 rl_nav_controller 完整功能包
echo -e "\n[1/3] 同步 rl_nav_controller 功能包（含模型权重、配置与启动文件）..."
rsync -avz --progress \
    --exclude='__pycache__' \
    --exclude='*.pyc' \
    --exclude='.git' \
    --exclude='*.egg-info' \
    "$SRC_PKG_DIR/" "$ROBOT_USER@$ROBOT_IP:~/go2w_nav_ros2_ws/src/rl_nav_controller/"

echo "[✓] rl_nav_controller 源码与模型同步完成。"

# 4. 同步更新后的全局启动和停止脚本及 RViz 配置
echo -e "\n[2/3] 同步 start_nav.sh / stop_nav.sh 与 rviz.rviz 到机载工作空间根目录..."
scp "${LOCAL_WS}/start_nav.sh" "$ROBOT_USER@$ROBOT_IP:~/go2w_nav_ros2_ws/start_nav.sh"
scp "${LOCAL_WS}/stop_nav.sh" "$ROBOT_USER@$ROBOT_IP:~/go2w_nav_ros2_ws/stop_nav.sh"
scp "${LOCAL_WS}/rviz.rviz" "$ROBOT_USER@$ROBOT_IP:~/go2w_nav_ros2_ws/rviz.rviz"
ssh "$ROBOT_USER@$ROBOT_IP" "chmod +x ~/go2w_nav_ros2_ws/start_nav.sh ~/go2w_nav_ros2_ws/stop_nav.sh"
echo "[✓] 启动脚本与 RViz 配置同步完成。"

# 5. 提示或自动在机载平台编译
echo -e "\n[3/3] 在机器狗机载平台上编译 rl_nav_controller..."
read -p "是否立即在机载端自动执行 colcon build? [Y/n] " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]] || [[ -z $REPLY ]]; then
    ssh "$ROBOT_USER@$ROBOT_IP" "bash -c 'source /opt/ros/humble/setup.bash && cd ~/go2w_nav_ros2_ws && colcon build --packages-select rl_nav_controller --symlink-install'"
    echo -e "\n[✓] 机载端编译成功完成！"
else
    echo "已跳过远程自动编译。请在狗机上手动执行："
    echo "  ssh ${ROBOT_USER}@${ROBOT_IP}"
    echo "  cd ~/go2w_nav_ros2_ws"
    echo "  source /opt/ros/humble/setup.bash"
    echo "  colcon build --packages-select rl_nav_controller --symlink-install"
fi

echo "=================================================================="
echo ">>> 部署全部就绪！在狗机上启动导航只需执行："
echo "    cd ~/go2w_nav_ros2_ws && ./start_nav.sh"
echo ">>> 停止导航执行："
echo "    cd ~/go2w_nav_ros2_ws && ./stop_nav.sh"
echo "=================================================================="

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

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [ -f "$WORKSPACE_DIR/rl_nav_controller.py" ]; then
    scp "$WORKSPACE_DIR/rl_nav_controller.py" "$ROBOT_USER@$ROBOT_IP:~/go2w_nav_ros2_ws/src/rl_nav_controller/rl_nav_controller/rl_nav_controller.py"
fi
if [ -f "$WORKSPACE_DIR/scripts/constants.py" ]; then
    scp "$WORKSPACE_DIR/scripts/constants.py" "$ROBOT_USER@$ROBOT_IP:~/go2w_nav_ros2_ws/src/rl_nav_controller/rl_nav_controller/constants.py"
fi
if [ -f "$WORKSPACE_DIR/scripts/rl_nav_controller.launch.py" ]; then
    scp "$WORKSPACE_DIR/scripts/rl_nav_controller.launch.py" "$ROBOT_USER@$ROBOT_IP:~/go2w_nav_ros2_ws/src/rl_nav_controller/launch/rl_nav_controller.launch.py"
fi

echo "[✓] rl_nav_controller 源码与模型同步完成。"

# 4. 同步更新后的全局启动和停止脚本 (不再推送任何 rviz 相关文件，避免触发 C++ 工具链)
echo -e "\n[2/4] 同步 start_nav.sh / start_nav_custom.sh / stop_nav.sh 到机载 go2w_nav_ros2_ws..."
scp "${LOCAL_WS}/start_nav.sh" "$ROBOT_USER@$ROBOT_IP:~/go2w_nav_ros2_ws/start_nav.sh"
scp "${LOCAL_WS}/stop_nav.sh" "$ROBOT_USER@$ROBOT_IP:~/go2w_nav_ros2_ws/stop_nav.sh"

if [ -f "$WORKSPACE_DIR/scripts/start_nav_custom.sh" ]; then
    scp "$WORKSPACE_DIR/scripts/start_nav_custom.sh" "$ROBOT_USER@$ROBOT_IP:~/go2w_nav_ros2_ws/start_nav_custom.sh"
    ssh "$ROBOT_USER@$ROBOT_IP" "cd ~/go2w_nav_ros2_ws && ln -sf start_nav_custom.sh start_nav_carrot.sh && chmod +x start_nav_custom.sh"
fi

if [ -f "$WORKSPACE_DIR/scripts/door_diagnostic_logger.py" ]; then
    scp "$WORKSPACE_DIR/scripts/door_diagnostic_logger.py" "$ROBOT_USER@$ROBOT_IP:~/go2w_nav_ros2_ws/door_diagnostic_logger.py"
    ssh "$ROBOT_USER@$ROBOT_IP" "chmod +x ~/go2w_nav_ros2_ws/door_diagnostic_logger.py"
fi

if [ -f "$WORKSPACE_DIR/docs/GO2W_DOOR_NAVIGATION_HANDOVER.md" ]; then
    scp "$WORKSPACE_DIR/docs/GO2W_DOOR_NAVIGATION_HANDOVER.md" "$ROBOT_USER@$ROBOT_IP:~/go2w_nav_ros2_ws/GO2W_DOOR_NAVIGATION_HANDOVER.md" 2>/dev/null || true
fi

ssh "$ROBOT_USER@$ROBOT_IP" "chmod +x ~/go2w_nav_ros2_ws/start_nav.sh ~/go2w_nav_ros2_ws/stop_nav.sh"
echo "[✓] go2w_nav_ros2_ws 启动脚本与文档同步完成 (已跳过所有 rviz 文件推送)。"

# 5. 自动恢复与清理机载 ~/sru_ws 目录文件
echo -e "\n[3/4] 恢复与清理机载 ~/sru_ws 目录 (恢复原版 start_go2w_system.sh，清理临时文件)..."
if [ -d "$HOME/go2w_robot_ref/sru_ws" ] || [ -f "$WORKSPACE_DIR/scripts/start_go2w_system.sh" ]; then
    scp "$WORKSPACE_DIR/scripts/start_go2w_system.sh" "$ROBOT_USER@$ROBOT_IP:~/sru_ws/start_go2w_system.sh"
    ssh "$ROBOT_USER@$ROBOT_IP" "rm -f ~/sru_ws/start_go2w_map_nav.sh ~/sru_ws/go2w_local_nav.rviz ~/sru_ws/map/map.pcd && chmod +x ~/sru_ws/start_go2w_system.sh 2>/dev/null || true"
    if [ -f "$WORKSPACE_DIR/rl_nav_controller.py" ]; then
        scp "$WORKSPACE_DIR/rl_nav_controller.py" "$ROBOT_USER@$ROBOT_IP:~/sru_ws/src/sru-robot-deployment/rl_nav_controller/rl_nav_controller/rl_nav_controller.py" 2>/dev/null || true
    fi
    if [ -f "$WORKSPACE_DIR/scripts/constants.py" ]; then
        scp "$WORKSPACE_DIR/scripts/constants.py" "$ROBOT_USER@$ROBOT_IP:~/sru_ws/src/sru-robot-deployment/rl_nav_controller/rl_nav_controller/constants.py" 2>/dev/null || true
    fi
    echo "[✓] 机载 ~/sru_ws 目录已成功恢复为原版状态！"
fi

# 6. 提示或自动在机载平台编译
echo -e "\n[4/4] 在机器狗机载平台上编译 rl_nav_controller..."
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
echo ">>> 部署全部就绪！在狗机上启动导航方式："
echo "    [方式 1] 自定义前瞻萝卜点距离且不上报错误 (推荐实机调试):"
echo "             cd ~/go2w_nav_ros2_ws && ./start_nav_custom.sh 1.5"
echo "             (支持: ./start_nav_custom.sh [距离] 或 -l [距离]，默认 2.0m)"
echo "    [方式 2] 原版全系统启动 (原参数原逻辑完全不受影响):"
echo "             cd ~/go2w_nav_ros2_ws && ./start_nav.sh"
echo ">>> 停止导航执行："
echo "    cd ~/go2w_nav_ros2_ws && ./stop_nav.sh"
echo "=================================================================="

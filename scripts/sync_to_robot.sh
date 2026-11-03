#!/usr/bin/env bash
# ==============================================================================
# Unitree Go2W 导航控制器与评测套件一键同步脚本 (sync_to_robot.sh)
# 
# 用途：将开发机上修改后的 rl_nav_controller.py 及 benchmark 评测套件
#       直接推送到机器狗高层算力平台 (192.168.123.111, agile)
# ==============================================================================

set -e

# 可通过命令行参数传入新 IP: bash scripts/sync_to_robot.sh <IP>
ROBOT_IP="${1:-${ROBOT_IP:-10.3.42.161}}"
ROBOT_USER="agile"
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "=================================================================="
echo ">>> 正在连接机器狗高层主板: ${ROBOT_USER}@${ROBOT_IP}..."
echo "=================================================================="

# 检查网络连通性
if ! ping -c 1 -W 1 "$ROBOT_IP" > /dev/null 2>&1; then
    echo "[!] 警告: 无法 ping 通 $ROBOT_IP，请确认是否已连接机器狗车载 WiFi/局域网。"
    echo "    若网络正常但 ping 被禁用，将尝试直接进行 SSH/SCP 传输..."
fi

# 1. 同步修改后的 rl_nav_controller 包到机载 sru_ws
echo -e "\n[1/3] 同步全局导航重构后的 rl_nav_controller 代码..."
SRC_PKG_DIR="/home/yichangfeng/go2w_robot_ref/sru_ws/src/sru-robot-deployment/rl_nav_controller"

if [ -d "$SRC_PKG_DIR" ]; then
    rsync -avz --exclude='__pycache__' --exclude='*.pyc' \
        "$SRC_PKG_DIR/" "$ROBOT_USER@$ROBOT_IP:~/sru_ws/src/sru-robot-deployment/rl_nav_controller/"
    echo "[✓] sru_ws/rl_nav_controller 全部代码已同步完成。"
else
    echo "[!] 警告: 未找到 $SRC_PKG_DIR，尝试单独同步核心文件..."
    ssh "$ROBOT_USER@$ROBOT_IP" "mkdir -p ~/sru_ws/src/sru-robot-deployment/rl_nav_controller/rl_nav_controller/"
    scp "$WORKSPACE_DIR/scripts/test_global_nav_integration.py" "$ROBOT_USER@$ROBOT_IP:~/"
fi

# 2. 同步测试脚本与模拟调度器
echo -e "\n[2/3] 同步全局导航测试套件与规范文档..."
scp "$WORKSPACE_DIR/scripts/mock_global_nav_tester.py" "$ROBOT_USER@$ROBOT_IP:~/"
scp "$WORKSPACE_DIR/scripts/test_global_nav_integration.py" "$ROBOT_USER@$ROBOT_IP:~/"
scp "$WORKSPACE_DIR/docs/GLOBAL_NAV_INTEGRATION_SPEC.md" "$ROBOT_USER@$ROBOT_IP:~/"
echo "[✓] 测试脚本与规范文档同步完成。"

# 3. 提示在机载端生效与编译
echo -e "\n[3/3] 部署就绪！"
echo "------------------------------------------------------------------"
echo "提示：请在机器人上重新编译生效（推荐 symlink-install 免二次编译）："
echo "  ssh agile@$ROBOT_IP"
echo "  cd ~/sru_ws"
echo "  source /opt/ros/humble/setup.bash"
echo "  colcon build --packages-select rl_nav_controller --symlink-install"
echo "=================================================================="

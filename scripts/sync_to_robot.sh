#!/usr/bin/env bash
# ==============================================================================
# Unitree Go2W 导航控制器与评测套件一键同步脚本 (sync_to_robot.sh)
# 
# 用途：将开发机上修改后的 rl_nav_controller.py 及 benchmark 评测套件
#       直接推送到机器狗高层算力平台 (192.168.123.111, agile)
# ==============================================================================

set -e

# 可通过命令行参数传入新 IP: bash scripts/sync_to_robot.sh <IP>
ROBOT_IP="${1:-${ROBOT_IP:-192.168.31.186}}"
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

# 1. 同步修改后的 rl_nav_controller.py 到机载 sru_ws 与 bridge_ws
echo -e "\n[1/3] 同步核心控制器代码 rl_nav_controller.py..."
ssh "$ROBOT_USER@$ROBOT_IP" "mkdir -p ~/sru_ws/src/sru-robot-deployment/rl_nav_controller/rl_nav_controller/ ~/bridge_ws/src/rl_nav_controller/rl_nav_controller/"

scp "$WORKSPACE_DIR/rl_nav_controller.py" "$ROBOT_USER@$ROBOT_IP:~/sru_ws/src/sru-robot-deployment/rl_nav_controller/rl_nav_controller/rl_nav_controller.py"
scp "$WORKSPACE_DIR/rl_nav_controller.py" "$ROBOT_USER@$ROBOT_IP:~/bridge_ws/src/rl_nav_controller/rl_nav_controller/rl_nav_controller.py"
echo "[✓] rl_nav_controller.py 同步完成。"

# 2. 同步实机 benchmark 评测与数据采集套件
echo -e "\n[2/3] 同步 benchmark 自动化测试与数据采集套件..."
ssh "$ROBOT_USER@$ROBOT_IP" "mkdir -p ~/benchmark_scripts/ ~/benchmark_logs/"
scp -r "$WORKSPACE_DIR/scripts/benchmark/"* "$ROBOT_USER@$ROBOT_IP:~/benchmark_scripts/"
echo "[✓] benchmark 套件同步完成。"

# 3. 提示在机载端生效
echo -e "\n[3/3] 部署就绪！"
echo "------------------------------------------------------------------"
echo "提示：Python 节点修改后直接生效（若工作空间为 symlink-install）。"
echo "如需彻底重新编译机载工作空间，可在机器人上运行："
echo "  ssh agile@$ROBOT_IP"
echo "  cd ~/sru_ws && colcon build --packages-select rl_nav_controller --symlink-install"
echo "=================================================================="

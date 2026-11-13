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

# 1. 同步修改后的 rl_nav_controller 包到机载 sru_ws
echo -e "\n[1/4] 同步局部避障导航 rl_nav_controller 源码与 RealSense 默认配置..."
SRC_PKG_DIR="/home/yichangfeng/go2w_robot_ref/sru_ws/src/sru-robot-deployment/rl_nav_controller"

if [ -d "$SRC_PKG_DIR" ]; then
    rsync -avz --exclude='__pycache__' --exclude='*.pyc' \
        "$SRC_PKG_DIR/" "$ROBOT_USER@$ROBOT_IP:~/sru_ws/src/sru-robot-deployment/rl_nav_controller/"
fi

ssh "$ROBOT_USER@$ROBOT_IP" "mkdir -p ~/sru_ws/src/sru-robot-deployment/rl_nav_controller/rl_nav_controller"
if [ -f "$WORKSPACE_DIR/rl_nav_controller.py" ]; then
    scp "$WORKSPACE_DIR/rl_nav_controller.py" "$ROBOT_USER@$ROBOT_IP:~/sru_ws/src/sru-robot-deployment/rl_nav_controller/rl_nav_controller/rl_nav_controller.py"
fi
if [ -f "$WORKSPACE_DIR/rl_nav_controller_adaptive.py" ]; then
    scp "$WORKSPACE_DIR/rl_nav_controller_adaptive.py" "$ROBOT_USER@$ROBOT_IP:~/sru_ws/src/sru-robot-deployment/rl_nav_controller/rl_nav_controller/rl_nav_controller_adaptive.py"
fi
if [ -f "$WORKSPACE_DIR/path_tracker.py" ]; then
    scp "$WORKSPACE_DIR/path_tracker.py" "$ROBOT_USER@$ROBOT_IP:~/sru_ws/src/sru-robot-deployment/rl_nav_controller/rl_nav_controller/path_tracker.py"
fi
if [ -f "$WORKSPACE_DIR/adaptive_path_tracker.py" ]; then
    scp "$WORKSPACE_DIR/adaptive_path_tracker.py" "$ROBOT_USER@$ROBOT_IP:~/sru_ws/src/sru-robot-deployment/rl_nav_controller/rl_nav_controller/adaptive_path_tracker.py"
fi
if [ -f "$WORKSPACE_DIR/scripts/constants.py" ]; then
    scp "$WORKSPACE_DIR/scripts/constants.py" "$ROBOT_USER@$ROBOT_IP:~/sru_ws/src/sru-robot-deployment/rl_nav_controller/rl_nav_controller/constants.py"
fi
if [ -f "$WORKSPACE_DIR/scripts/rl_nav_controller.launch.py" ]; then
    scp "$WORKSPACE_DIR/scripts/rl_nav_controller.launch.py" "$ROBOT_USER@$ROBOT_IP:~/sru_ws/src/sru-robot-deployment/rl_nav_controller/launch/rl_nav_controller.launch.py"
fi
# 同步到备用路径 (若存在)
ssh "$ROBOT_USER@$ROBOT_IP" "if [ -d ~/sru_ws/src/rl_nav_controller/rl_nav_controller ]; then cp -f ~/sru_ws/src/sru-robot-deployment/rl_nav_controller/rl_nav_controller/*.py ~/sru_ws/src/rl_nav_controller/rl_nav_controller/ 2>/dev/null || true; fi"
echo "[✓] sru_ws/rl_nav_controller 核心代码与默认参数同步完成。"

# 2. 同步 start_local_sru.sh / stop_local_sru.sh 及 RealSense 覆盖配置
echo -e "\n[2/4] 同步 ~/sru_ws/start_local_sru.sh (默认 RealSense D435) 与相机参数..."
if [ -f "$WORKSPACE_DIR/scripts/start_local_sru.sh" ]; then
    scp "$WORKSPACE_DIR/scripts/start_local_sru.sh" "$ROBOT_USER@$ROBOT_IP:~/sru_ws/start_local_sru.sh"
    ssh "$ROBOT_USER@$ROBOT_IP" "cd ~/sru_ws && chmod +x start_local_sru.sh && ln -sf start_local_sru.sh start_go2w_system.sh"
fi
if [ -f "$WORKSPACE_DIR/scripts/stop_local_sru.sh" ]; then
    scp "$WORKSPACE_DIR/scripts/stop_local_sru.sh" "$ROBOT_USER@$ROBOT_IP:~/sru_ws/stop_local_sru.sh"
    ssh "$ROBOT_USER@$ROBOT_IP" "cd ~/sru_ws && chmod +x stop_local_sru.sh && ln -sf stop_local_sru.sh stop_go2w_system.sh"
fi
if [ -f "$WORKSPACE_DIR/scripts/realsense_nav_override.yaml" ]; then
    scp "$WORKSPACE_DIR/scripts/realsense_nav_override.yaml" "$ROBOT_USER@$ROBOT_IP:~/sru_ws/realsense_nav_override.yaml"
fi
if [ -f "$WORKSPACE_DIR/config/local_sru.rviz" ]; then
    scp "$WORKSPACE_DIR/config/local_sru.rviz" "$ROBOT_USER@$ROBOT_IP:~/sru_ws/local_sru.rviz"
fi
if [ -f "$WORKSPACE_DIR/scripts/test_realsense_max_distance.py" ]; then
    scp "$WORKSPACE_DIR/scripts/test_realsense_max_distance.py" "$ROBOT_USER@$ROBOT_IP:~/sru_ws/test_realsense_max_distance.py"
    ssh "$ROBOT_USER@$ROBOT_IP" "chmod +x ~/sru_ws/test_realsense_max_distance.py"
fi
echo "[✓] start_local_sru.sh / stop_local_sru.sh, local_sru.rviz 与 realsense_nav_override.yaml 同步完成。"

# 3. 同步测试脚本与模拟调度器
echo -e "\n[3/4] 同步全局导航测试套件与规范文档..."
if [ -f "$WORKSPACE_DIR/scripts/mock_global_nav_tester.py" ]; then
    scp "$WORKSPACE_DIR/scripts/mock_global_nav_tester.py" "$ROBOT_USER@$ROBOT_IP:~/"
fi
if [ -f "$WORKSPACE_DIR/scripts/test_global_nav_integration.py" ]; then
    scp "$WORKSPACE_DIR/scripts/test_global_nav_integration.py" "$ROBOT_USER@$ROBOT_IP:~/"
fi
if [ -f "$WORKSPACE_DIR/docs/GLOBAL_NAV_INTEGRATION_SPEC.md" ]; then
    scp "$WORKSPACE_DIR/docs/GLOBAL_NAV_INTEGRATION_SPEC.md" "$ROBOT_USER@$ROBOT_IP:~/"
fi
echo "[✓] 测试脚本与规范文档同步完成。"

# 4. 机载端重新编译生效
echo -e "\n[4/4] 编译与验证..."
echo "------------------------------------------------------------------"
echo "正在机载端执行 colcon build (rl_nav_controller)..."
ssh "$ROBOT_USER@$ROBOT_IP" "bash -c 'source /opt/ros/humble/setup.bash && cd ~/sru_ws && colcon build --packages-select rl_nav_controller --symlink-install'"
echo -e "\n[✓] 机载 ~/sru_ws 编译成功完成！"
echo "=================================================================="
echo ">>> 启动方式 (默认使用 RealSense D435)："
echo "    ssh ${ROBOT_USER}@${ROBOT_IP}"
echo "    cd ~/sru_ws && ./start_local_sru.sh"
echo ">>> 若需要切回 ZED 2i 测试："
echo "    cd ~/sru_ws && ./start_local_sru.sh --zed"
echo ">>> 停止导航执行："
echo "    cd ~/sru_ws && ./stop_local_sru.sh"
echo "=================================================================="

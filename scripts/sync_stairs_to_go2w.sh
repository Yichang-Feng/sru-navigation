#!/usr/bin/env bash
# ==============================================================================
# Unitree Go2W 楼梯感知与自适应导航一键同步与构建脚本 (sync_stairs_to_go2w.sh)
#
# 用途：只推送新增的楼梯感知模块与启动文件，完全不污染原有任何基准代码！
# ==============================================================================

set -e

ROBOT_IP="${1:-${ROBOT_IP:-10.3.42.161}}"
ROBOT_USER="agile"
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "=================================================================="
echo ">>> Unitree Go2W 楼梯感知导航一键同步器"
echo ">>> 目标机器人: ${ROBOT_USER}@${ROBOT_IP}"
echo ">>> 目标工作空间: ~/go2w_nav_ros2_ws"
echo "=================================================================="

# 1. 检查网络连通性
if ! ping -c 1 -W 1 "$ROBOT_IP" > /dev/null 2>&1; then
    echo "[!] 警告: 无法 ping 通 $ROBOT_IP，尝试直接传输..."
fi

# 2. 推送独立 Python 核心模块
echo -e "\n[1/3] 推送核心模块 (rl_nav_controller.py, adaptive_path_tracker.py, rl_nav_controller_adaptive.py, stairs_depth_processor.py, rl_nav_controller_stairs.py)..."
scp "$WORKSPACE_DIR/rl_nav_controller.py" \
    "$ROBOT_USER@$ROBOT_IP:~/go2w_nav_ros2_ws/src/rl_nav_controller/rl_nav_controller/rl_nav_controller.py"

scp "$WORKSPACE_DIR/adaptive_path_tracker.py" \
    "$ROBOT_USER@$ROBOT_IP:~/go2w_nav_ros2_ws/src/rl_nav_controller/rl_nav_controller/adaptive_path_tracker.py"

scp "$WORKSPACE_DIR/rl_nav_controller_adaptive.py" \
    "$ROBOT_USER@$ROBOT_IP:~/go2w_nav_ros2_ws/src/rl_nav_controller/rl_nav_controller/rl_nav_controller_adaptive.py"

scp "$WORKSPACE_DIR/stairs_depth_processor.py" \
    "$ROBOT_USER@$ROBOT_IP:~/go2w_nav_ros2_ws/src/rl_nav_controller/rl_nav_controller/stairs_depth_processor.py"

scp "$WORKSPACE_DIR/rl_nav_controller_stairs.py" \
    "$ROBOT_USER@$ROBOT_IP:~/go2w_nav_ros2_ws/src/rl_nav_controller/rl_nav_controller/rl_nav_controller_stairs.py"

if [ -f "$WORKSPACE_DIR/scripts/constants.py" ]; then
    scp "$WORKSPACE_DIR/scripts/constants.py" \
        "$ROBOT_USER@$ROBOT_IP:~/go2w_nav_ros2_ws/src/rl_nav_controller/rl_nav_controller/constants.py"
fi

# 3. 推送启动脚本与诊断工具
echo -e "\n[2/4] 推送 start_nav.sh, start_nav_stairs.sh 与 door_diagnostic_logger.py..."
if [ -f "$WORKSPACE_DIR/scripts/start_nav.sh" ]; then
    scp "$WORKSPACE_DIR/scripts/start_nav.sh" \
        "$ROBOT_USER@$ROBOT_IP:~/go2w_nav_ros2_ws/start_nav.sh"
    ssh "$ROBOT_USER@$ROBOT_IP" "chmod +x ~/go2w_nav_ros2_ws/start_nav.sh"
    echo "[✓] 已推送最新默认启动器 start_nav.sh"
fi

if [ -f "$WORKSPACE_DIR/scripts/start_nav_stairs.sh" ]; then
    scp "$WORKSPACE_DIR/scripts/start_nav_stairs.sh" \
        "$ROBOT_USER@$ROBOT_IP:~/go2w_nav_ros2_ws/start_nav_stairs.sh"
    ssh "$ROBOT_USER@$ROBOT_IP" "chmod +x ~/go2w_nav_ros2_ws/start_nav_stairs.sh"
fi

if [ -f "$WORKSPACE_DIR/scripts/door_diagnostic_logger.py" ]; then
    scp "$WORKSPACE_DIR/scripts/door_diagnostic_logger.py" \
        "$ROBOT_USER@$ROBOT_IP:~/go2w_nav_ros2_ws/door_diagnostic_logger.py"
    ssh "$ROBOT_USER@$ROBOT_IP" "chmod +x ~/go2w_nav_ros2_ws/door_diagnostic_logger.py"
    echo "[✓] 已推送增强版 door_diagnostic_logger.py"
fi

# 4. 在机载端注册 setup.py entry_point (若尚未注册)
echo -e "\n[3/4] 检查并在机载 setup.py 中注册 rl_nav_controller_stairs 可执行文件..."
ssh "$ROBOT_USER@$ROBOT_IP" python3 - << 'EOF'
setup_path = "/home/agile/go2w_nav_ros2_ws/src/rl_nav_controller/setup.py"
try:
    with open(setup_path, "r", encoding="utf-8") as f:
        content = f.read()
    if "rl_nav_controller_stairs" not in content:
        target = "'rl_nav_controller_adaptive = rl_nav_controller.rl_nav_controller_adaptive:main'"
        fallback_target = "'rl_nav_controller = rl_nav_controller.rl_nav_controller:main'"
        
        replacement = "'rl_nav_controller_stairs = rl_nav_controller.rl_nav_controller_stairs:main'"
        
        if target in content:
            new_content = content.replace(target, target + ",\n            " + replacement)
            with open(setup_path, "w", encoding="utf-8") as f:
                f.write(new_content)
            print("[✓] 成功在机载 setup.py 注册 rl_nav_controller_stairs 入口点！")
        elif fallback_target in content:
            new_content = content.replace(fallback_target, fallback_target + ",\n            " + replacement)
            with open(setup_path, "w", encoding="utf-8") as f:
                f.write(new_content)
            print("[✓] 成功在机载 setup.py 注册 rl_nav_controller_stairs 入口点！")
        else:
            print("[!] 未找到默认入口点标记，跳过文本替换。")
    else:
        print("[✓] 机载 setup.py 已包含 rl_nav_controller_stairs。")
except Exception as e:
    print(f"[!] 更新 setup.py 异常: {e}")
EOF

# 5. 在机载平台编译 rl_nav_controller
echo -e "\n[4/4] 在机器狗机载平台上增量构建 rl_nav_controller..."
read -p "是否立即在机载端自动执行 colcon build? [Y/n] " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]] || [[ -z $REPLY ]]; then
    ssh "$ROBOT_USER@$ROBOT_IP" "bash -c 'source /opt/ros/humble/setup.bash && cd ~/go2w_nav_ros2_ws && colcon build --packages-select rl_nav_controller --symlink-install'"
    echo -e "\n[✓] 机载端楼梯感知模块构建完成！"
else
    echo "已跳过远程自动编译。"
fi

echo "=================================================================="
echo ">>> 楼梯感知导航部署就绪！"
echo ">>> 启动楼梯感知自适应导航："
echo "    ssh ${ROBOT_USER}@${ROBOT_IP}"
echo "    cd ~/go2w_nav_ros2_ws && ./start_nav_stairs.sh"
echo ">>> 任何时候回退到原版自适应导航："
echo "    cd ~/go2w_nav_ros2_ws && ./start_nav_adaptive.sh"
echo ">>> 任何时候回退到原版基准导航："
echo "    cd ~/go2w_nav_ros2_ws && ./start_nav_custom.sh 0.5"
echo "=================================================================="

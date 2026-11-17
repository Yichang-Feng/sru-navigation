#!/usr/bin/env bash
# ==============================================================================
# Unitree Go2W 导航系统一键同步与真机部署脚本 (sync_to_go2w_nav.sh)
#
# 用途：将本地已完成对齐的 rl_nav_controller 以及更新后的 start_nav.sh / stop_nav.sh
#       一键推送到机器狗车载算力平台 (agile@<ROBOT_IP>:~/go2w_nav_ros2_ws/) 并编译生效
# ==============================================================================

set -e

# 支持通过命令行传参覆盖目标 IP: bash scripts/sync_to_go2w_nav.sh [IP]
ROBOT_IP="${1:-${ROBOT_IP:-192.168.31.186}}"
ROBOT_USER="agile"

DEPLOY_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [ -d "${DEPLOY_DIR}/go2w_nav_ros2_ws/src/rl_nav_controller" ]; then
    LOCAL_WS="${DEPLOY_DIR}/go2w_nav_ros2_ws"
else
    LOCAL_WS="/home/yichangfeng/go2w_robot_ref/go2w_nav_ros2_ws"
fi
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

# 2. 仅同步本次改动涉及的文件（depth_fill_mode / V2）
echo -e "\n[1/3] 同步本次改动文件（depth_fill_mode=nearest）..."

# rl_nav_controller 核心与 adaptive（stairs 继承 adaptive，必须同步）
scp "${LOCAL_WS}/src/rl_nav_controller/rl_nav_controller/rl_nav_controller.py" \
    "$ROBOT_USER@$ROBOT_IP:~/go2w_nav_ros2_ws/src/rl_nav_controller/rl_nav_controller/rl_nav_controller.py"
scp "${LOCAL_WS}/src/rl_nav_controller/rl_nav_controller/rl_nav_controller_adaptive.py" \
    "$ROBOT_USER@$ROBOT_IP:~/go2w_nav_ros2_ws/src/rl_nav_controller/rl_nav_controller/rl_nav_controller_adaptive.py"

# launch 参数
scp "${LOCAL_WS}/src/rl_nav_controller/launch/rl_nav_controller.launch.py" \
    "$ROBOT_USER@$ROBOT_IP:~/go2w_nav_ros2_ws/src/rl_nav_controller/launch/rl_nav_controller.launch.py"

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [ -f "$WORKSPACE_DIR/rl_nav_controller.py" ]; then
    scp "$WORKSPACE_DIR/rl_nav_controller.py" "$ROBOT_USER@$ROBOT_IP:~/go2w_nav_ros2_ws/src/rl_nav_controller/rl_nav_controller/rl_nav_controller.py"
fi
if [ -f "$WORKSPACE_DIR/rl_nav_controller_adaptive.py" ]; then
    scp "$WORKSPACE_DIR/rl_nav_controller_adaptive.py" "$ROBOT_USER@$ROBOT_IP:~/go2w_nav_ros2_ws/src/rl_nav_controller/rl_nav_controller/rl_nav_controller_adaptive.py"
fi
if [ -f "$WORKSPACE_DIR/rl_nav_controller_stairs.py" ]; then
    scp "$WORKSPACE_DIR/rl_nav_controller_stairs.py" "$ROBOT_USER@$ROBOT_IP:~/go2w_nav_ros2_ws/src/rl_nav_controller/rl_nav_controller/rl_nav_controller_stairs.py"
fi
# 路径跟踪与台阶模块（workspace 源目录为准；B 暂停路径依赖 path_tracker.has_path 等 API）
for MODULE in path_tracker.py adaptive_path_tracker.py stairs_depth_processor.py; do
    SRC_MODULE="${LOCAL_WS}/src/rl_nav_controller/rl_nav_controller/${MODULE}"
    if [ -f "$SRC_MODULE" ]; then
        scp "$SRC_MODULE" "$ROBOT_USER@$ROBOT_IP:~/go2w_nav_ros2_ws/src/rl_nav_controller/rl_nav_controller/${MODULE}"
    elif [ -f "$WORKSPACE_DIR/${MODULE}" ]; then
        scp "$WORKSPACE_DIR/${MODULE}" "$ROBOT_USER@$ROBOT_IP:~/go2w_nav_ros2_ws/src/rl_nav_controller/rl_nav_controller/${MODULE}"
    fi
done
if [ -f "$WORKSPACE_DIR/scripts/setup_rl_nav_controller.py" ]; then
    scp "$WORKSPACE_DIR/scripts/setup_rl_nav_controller.py" "$ROBOT_USER@$ROBOT_IP:~/go2w_nav_ros2_ws/src/rl_nav_controller/setup.py"
elif [ -f "${LOCAL_WS}/src/rl_nav_controller/setup.py" ]; then
    scp "${LOCAL_WS}/src/rl_nav_controller/setup.py" "$ROBOT_USER@$ROBOT_IP:~/go2w_nav_ros2_ws/src/rl_nav_controller/setup.py"
fi
if [ -f "$WORKSPACE_DIR/scripts/constants.py" ]; then
    scp "$WORKSPACE_DIR/scripts/constants.py" "$ROBOT_USER@$ROBOT_IP:~/go2w_nav_ros2_ws/src/rl_nav_controller/rl_nav_controller/constants.py"
fi
if [ -f "$WORKSPACE_DIR/scripts/rl_nav_controller.launch.py" ]; then
    scp "$WORKSPACE_DIR/scripts/rl_nav_controller.launch.py" "$ROBOT_USER@$ROBOT_IP:~/go2w_nav_ros2_ws/src/rl_nav_controller/launch/rl_nav_controller.launch.py"
fi
if [ -f "$WORKSPACE_DIR/scripts/realsense_nav_override.yaml" ]; then
    ssh "$ROBOT_USER@$ROBOT_IP" "mkdir -p ~/go2w_nav_ros2_ws/src/rl_nav_controller/config" 2>/dev/null || true
    scp "$WORKSPACE_DIR/scripts/realsense_nav_override.yaml" "$ROBOT_USER@$ROBOT_IP:~/go2w_nav_ros2_ws/src/rl_nav_controller/config/realsense_nav_override.yaml"
fi

echo "[✓] rl_nav_controller 源码与模型同步完成。"

# 4. 同步更新后的全局启动和停止脚本 (不再推送任何 rviz 相关文件，避免触发 C++ 工具链)
echo -e "\n[2/4] 同步 start_nav.sh / start_nav_custom.sh / stop_nav.sh 到机载 go2w_nav_ros2_ws..."
if [ -f "$WORKSPACE_DIR/scripts/start_nav.sh" ]; then
    scp "$WORKSPACE_DIR/scripts/start_nav.sh" "$ROBOT_USER@$ROBOT_IP:~/go2w_nav_ros2_ws/start_nav.sh"
else
    scp "${LOCAL_WS}/start_nav.sh" "$ROBOT_USER@$ROBOT_IP:~/go2w_nav_ros2_ws/start_nav.sh"
fi
scp "${LOCAL_WS}/stop_nav.sh" "$ROBOT_USER@$ROBOT_IP:~/go2w_nav_ros2_ws/stop_nav.sh"

if [ -f "$WORKSPACE_DIR/scripts/start_nav_custom.sh" ]; then
    scp "$WORKSPACE_DIR/scripts/start_nav_custom.sh" "$ROBOT_USER@$ROBOT_IP:~/go2w_nav_ros2_ws/start_nav_custom.sh"
    ssh "$ROBOT_USER@$ROBOT_IP" "chmod +x ~/go2w_nav_ros2_ws/start_nav_custom.sh"
fi

if [ -f "$WORKSPACE_DIR/diagnostics/door_diagnostic_logger.py" ]; then
    scp "$WORKSPACE_DIR/diagnostics/door_diagnostic_logger.py" "$ROBOT_USER@$ROBOT_IP:~/go2w_nav_ros2_ws/door_diagnostic_logger.py"
    ssh "$ROBOT_USER@$ROBOT_IP" "chmod +x ~/go2w_nav_ros2_ws/door_diagnostic_logger.py"
fi
if [ -f "$WORKSPACE_DIR/diagnostics/test_realsense_max_distance.py" ]; then
    scp "$WORKSPACE_DIR/diagnostics/test_realsense_max_distance.py" "$ROBOT_USER@$ROBOT_IP:~/go2w_nav_ros2_ws/test_realsense_max_distance.py"
    ssh "$ROBOT_USER@$ROBOT_IP" "chmod +x ~/go2w_nav_ros2_ws/test_realsense_max_distance.py"
fi
if [ -f "$WORKSPACE_DIR/diagnostics/compare_camera_sessions.py" ]; then
    scp "$WORKSPACE_DIR/diagnostics/compare_camera_sessions.py" "$ROBOT_USER@$ROBOT_IP:~/go2w_nav_ros2_ws/compare_camera_sessions.py"
    ssh "$ROBOT_USER@$ROBOT_IP" "chmod +x ~/go2w_nav_ros2_ws/compare_camera_sessions.py"
fi
if [ -f "$WORKSPACE_DIR/diagnostics/README.md" ]; then
    scp "$WORKSPACE_DIR/diagnostics/README.md" "$ROBOT_USER@$ROBOT_IP:~/go2w_nav_ros2_ws/DOOR_DIAGNOSTIC_README.md" 2>/dev/null || true
fi

if [ -f "$WORKSPACE_DIR/docs/GO2W_DOOR_NAVIGATION_HANDOVER.md" ]; then
    scp "$WORKSPACE_DIR/docs/GO2W_DOOR_NAVIGATION_HANDOVER.md" "$ROBOT_USER@$ROBOT_IP:~/go2w_nav_ros2_ws/GO2W_DOOR_NAVIGATION_HANDOVER.md" 2>/dev/null || true
fi
if [ -f "$WORKSPACE_DIR/docs/GO2W_CAMERA_CONFIGURATION_AND_INTEGRATION_GUIDE.md" ]; then
    scp "$WORKSPACE_DIR/docs/GO2W_CAMERA_CONFIGURATION_AND_INTEGRATION_GUIDE.md" "$ROBOT_USER@$ROBOT_IP:~/go2w_nav_ros2_ws/GO2W_CAMERA_CONFIGURATION_AND_INTEGRATION_GUIDE.md" 2>/dev/null || true
fi

ssh "$ROBOT_USER@$ROBOT_IP" "chmod +x ~/go2w_nav_ros2_ws/start_nav.sh ~/go2w_nav_ros2_ws/stop_nav.sh"
echo "[✓] go2w_nav_ros2_ws 启动脚本与文档同步完成 (已跳过所有 rviz 文件推送)。"

# 5. 自动恢复与同步机载 ~/sru_ws 目录脚本与配置
echo -e "\n[3/4] 同步机载 ~/sru_ws 目录 (start_local_sru.sh / stop_local_sru.sh 与控制器)..."
if [ -f "$WORKSPACE_DIR/scripts/start_local_sru.sh" ]; then
    scp "$WORKSPACE_DIR/scripts/start_local_sru.sh" "$ROBOT_USER@$ROBOT_IP:~/sru_ws/start_local_sru.sh"
    ssh "$ROBOT_USER@$ROBOT_IP" "cd ~/sru_ws && chmod +x start_local_sru.sh && ln -sf start_local_sru.sh start_go2w_system.sh"
fi
if [ -f "$WORKSPACE_DIR/scripts/stop_local_sru.sh" ]; then
    scp "$WORKSPACE_DIR/scripts/stop_local_sru.sh" "$ROBOT_USER@$ROBOT_IP:~/sru_ws/stop_local_sru.sh"
    ssh "$ROBOT_USER@$ROBOT_IP" "cd ~/sru_ws && chmod +x stop_local_sru.sh && ln -sf stop_local_sru.sh stop_go2w_system.sh"
fi
ssh "$ROBOT_USER@$ROBOT_IP" "rm -f ~/sru_ws/start_go2w_map_nav.sh ~/sru_ws/go2w_local_nav.rviz ~/sru_ws/map/map.pcd 2>/dev/null || true"

# 同步控制器到机载 ~/sru_ws 下的旧副本目录（若存在），避免 start_local_sru.sh 误加载旧代码
for REMOTE_PKG_DIR in \
    "~/sru_ws/src/sru-robot-deployment/rl_nav_controller/rl_nav_controller" \
    "~/sru_ws/src/rl_nav_controller/rl_nav_controller"; do
    if ssh "$ROBOT_USER@$ROBOT_IP" "test -d $REMOTE_PKG_DIR" 2>/dev/null; then
        scp "${LOCAL_WS}/src/rl_nav_controller/rl_nav_controller/rl_nav_controller.py" \
            "$ROBOT_USER@$ROBOT_IP:$REMOTE_PKG_DIR/rl_nav_controller.py"
        scp "${LOCAL_WS}/src/rl_nav_controller/rl_nav_controller/rl_nav_controller_adaptive.py" \
            "$ROBOT_USER@$ROBOT_IP:$REMOTE_PKG_DIR/rl_nav_controller_adaptive.py"
        for MODULE in path_tracker.py adaptive_path_tracker.py; do
            if [ -f "${LOCAL_WS}/src/rl_nav_controller/rl_nav_controller/${MODULE}" ]; then
                scp "${LOCAL_WS}/src/rl_nav_controller/rl_nav_controller/${MODULE}" \
                    "$ROBOT_USER@$ROBOT_IP:$REMOTE_PKG_DIR/${MODULE}"
            fi
        done
        echo "[✓] 已更新机载旧副本目录: $REMOTE_PKG_DIR"
    fi
done
if [ -f "$WORKSPACE_DIR/scripts/constants.py" ]; then
    scp "$WORKSPACE_DIR/scripts/constants.py" "$ROBOT_USER@$ROBOT_IP:~/sru_ws/src/sru-robot-deployment/rl_nav_controller/rl_nav_controller/constants.py" 2>/dev/null || true
fi
if [ -f "$WORKSPACE_DIR/scripts/rl_nav_controller.launch.py" ]; then
    scp "$WORKSPACE_DIR/scripts/rl_nav_controller.launch.py" "$ROBOT_USER@$ROBOT_IP:~/sru_ws/src/sru-robot-deployment/rl_nav_controller/launch/rl_nav_controller.launch.py" 2>/dev/null || true
fi
if [ -f "$WORKSPACE_DIR/scripts/realsense_nav_override.yaml" ]; then
    scp "$WORKSPACE_DIR/scripts/realsense_nav_override.yaml" "$ROBOT_USER@$ROBOT_IP:~/sru_ws/realsense_nav_override.yaml" 2>/dev/null || true
fi
echo "[✓] 机载 ~/sru_ws 目录已成功同步为最新 start_local_sru.sh / stop_local_sru.sh！"

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

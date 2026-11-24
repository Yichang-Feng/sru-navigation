#!/usr/bin/env bash
# =============================================================================
# record_nav_bag.sh — 录制 RGB + 深度 + 定位/控制/TF，供后续处理与网页展示
# =============================================================================
#
# 用法:
#   ./record_nav_bag.sh                  # 自动识别相机（ZED / RealSense）
#   ./record_nav_bag.sh realsense        # 强制 RealSense
#   ./record_nav_bag.sh zed              # 强制 ZED
#   ./record_nav_bag.sh auto my_run      # 自动识别，并指定本次会话名
#
# 环境变量:
#   BAG_DIR=~/bags          输出目录（默认 ~/bags，建议指向外接 SSD）
#   REC_MODE=rgb|full       录制模式。rgb=只录 RGB + /loc/robo/odom + /global_path + /tf + /tf_static（默认）；
#                           full=深度+RGB+定位/控制/TF
#   REC_RATE=0              图像录制频率 Hz（默认 0 = 相机原始帧率；设 5/3 等会降频录制）
#   REC_NICE=10             录制/节流进程的 nice 优先级（越大越让路给导航）
#   REC_CPUS=""             可选，例如 "4-7"，用 taskset 把录制进程绑到指定 CPU
#   SKIP_RGB=1              不录 RGB（自动切到 full 模式，只录深度 + 定位）
#   NO_COMPRESS=1           关闭 zstd 压缩（CPU 紧张时用，磁盘占用会变大）
#   STORAGE=auto|mcap|sqlite3  录制存储格式（默认 auto，优先 mcap）
#   RAW_LIDAR=1             额外录 /livox/lidar 与 /livox/imu（后续重跑 SLAM 用，很重）
#   REC_LIO_PATH=1          额外录 /lio/path（消息随轨迹增长，默认不录）
#   EXTRA_TOPICS="..."      额外话题，空格分隔，例如 "/vis/lookahead_goal"
#   MAX_CACHE_MB=256        写缓存大小（MB，默认 256；越大越平滑但占内存）
#
# 性能提示:
#   默认 rgb 模式 + 原始帧率，只录 RGB/定位/路径/TF，负载比 full 小得多。
#   若仍卡顿：REC_RATE=5 把 RGB 降到 5 Hz，或 SKIP_RGB=1 只录深度。
#   录制进程用 nice/ionice 给导航让路；/lio/path 默认不录，需要时 REC_LIO_PATH=1。
#
# 示例:
#   ./record_nav_bag.sh auto rgb_full             # 默认：只录 RGB + 定位 + 路径 + TF（原始帧率）
#   REC_RATE=5 ./record_nav_bag.sh auto rgb_5hz   # RGB 降到 5 Hz
#   REC_MODE=full ./record_nav_bag.sh auto full_run    # 深度 + RGB + 控制 + TF
#   REC_MODE=full SKIP_RGB=1 ./record_nav_bag.sh auto depth_only  # 只录深度
#   BAG_DIR=/mnt/ssd/bags ./record_nav_bag.sh
#
# 录制结束后:
#   ros2 bag info <输出目录>
#   ros2 bag play <输出目录> --clock
#
# 注意:
#   - ZED 默认 publish_rgb=false，要录 RGB 请先把
#     go2w_nav_ros2_ws/src/rl_nav_controller/config/zed_nav_override.yaml
#     里的 general.publish_rgb 改为 true。
#   - RealSense 的 depth/image_rect_raw 未与 RGB 对齐（align_depth=false），
#     并排展示没问题；要叠图请额外录 aligned_depth_to_color。
#   - 传感器话题多为 BEST_EFFORT，脚本已写好 QoS 覆盖文件；
#     /tf_static 用 TRANSIENT_LOCAL 才能拿到静态外参。
# =============================================================================

set -euo pipefail

CAMERA="${1:-auto}"
NAME="${2:-}"
BAG_DIR="${BAG_DIR:-$HOME/bags}"
MAX_CACHE_MB="${MAX_CACHE_MB:-256}"
REC_RATE="${REC_RATE:-0}"
REC_NICE="${REC_NICE:-10}"
REC_CPUS="${REC_CPUS:-}"
REC_MODE="${REC_MODE:-}"   # 空 = 自动：默认 rgb；SKIP_RGB=1 时 full

die() { echo "ERROR: $*" >&2; exit 1; }
warn() { echo "WARN: $*" >&2; }

if [[ -z "$REC_MODE" ]]; then
    if [[ "${SKIP_RGB:-0}" == "1" ]]; then
        REC_MODE=full
    else
        REC_MODE=rgb
    fi
fi

case "$REC_MODE" in
    full|rgb) ;;
    *) die "REC_MODE 必须是 full 或 rgb（当前: $REC_MODE）" ;;
esac
if [[ "$REC_MODE" == "rgb" && "${SKIP_RGB:-0}" == "1" ]]; then
    die "REC_MODE=rgb 与 SKIP_RGB=1 冲突：rgb 模式本身就是要录 RGB。"
fi

if ! [[ "$REC_RATE" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
    die "REC_RATE 必须是数字（Hz），当前: $REC_RATE"
fi
if ! [[ "$REC_NICE" =~ ^-?[0-9]+$ ]]; then
    die "REC_NICE 必须是整数，当前: $REC_NICE"
fi

if [[ "$CAMERA" == "-h" || "$CAMERA" == "--help" ]]; then
    awk 'NR>1 && /^set -euo pipefail/{exit} NR>1{sub(/^# ?/,""); print}' "$0"
    exit 0
fi

# ROS 环境
if ! command -v ros2 >/dev/null 2>&1; then
    for _d in "${ROS_DISTRO:-}" humble foxy jazzy iron; do
        if [[ -n "$_d" && -f "/opt/ros/${_d}/setup.bash" ]]; then
            # shellcheck disable=SC1090
            set +u
            source "/opt/ros/${_d}/setup.bash"
            set -u
            break
        fi
    done
fi

# 自动加载机载工作空间（非交互 SSH 下 .bashrc 可能不会 source）
for _ws in \
    "${WS_SETUP:-}" \
    "$HOME/bridge_ws/install/setup.bash" \
    "$HOME/go2w_nav_ros2_ws/install/setup.bash" \
    "$HOME/sru_ws/install/setup.bash" \
    "$HOME/agile_nav_ws/install/setup.bash" \
    "$HOME/agile_nav_ws/src/sru-local/go2w_nav_ros2_ws/install/setup.bash" \
    "$HOME/agile_nav_ws/src/sru-local/bridge_ws/install/setup.bash" \
    "$HOME/agile_nav_ws/src/sru-local/sru_ws/install/setup.bash" \
    "$HOME/agile_nav_ws/src/sru-local/install/setup.bash"; do
    if [[ -n "$_ws" && -f "$_ws" ]]; then
        # shellcheck disable=SC1090
        set +u
        source "$_ws" || warn "加载工作空间失败: $_ws"
        set -u
    fi
done

command -v ros2 >/dev/null 2>&1 \
    || die "ros2 未找到。请先 source /opt/ros/<distro>/setup.bash 以及工作空间 install/setup.bash（或用 WS_SETUP 指定）。"

if ! TOPICS="$(ros2 topic list 2>/dev/null)"; then
    die "ros2 topic list 执行失败。请确认 ROS 环境已正确 source（ROS_DISTRO=${ROS_DISTRO:-未设置}）。"
fi
have() { grep -qxF "$1" <<<"$TOPICS"; }

# 相机自动识别（rgb 模式按 RGB 话题，full 模式按深度话题）
if [[ "$CAMERA" == "auto" ]]; then
    if [[ "$REC_MODE" == "rgb" ]]; then
        if have "/zed/zed_node/rgb/image_rect_color"; then
            CAMERA=zed
        elif have "/camera/camera/color/image_raw" || have "/camera/color/image_raw"; then
            CAMERA=realsense
        else
            die "rgb 模式下未检测到 RGB 话题。请确认相机驱动已启动（ZED 需 publish_rgb: true），或手动指定 realsense|zed。"
        fi
    elif have "/zed/zed_node/depth/depth_registered"; then
        CAMERA=zed
    elif have "/camera/camera/depth/image_rect_raw" || have "/camera/depth/image_rect_raw"; then
        CAMERA=realsense
    else
        die "未检测到 ZED / RealSense 深度话题。请先启动相机驱动，或手动指定 realsense|zed。"
    fi
fi

case "$CAMERA" in
    realsense)
        DEPTH="/camera/camera/depth/image_rect_raw"
        have "$DEPTH" || DEPTH="/camera/depth/image_rect_raw"
        DEPTH_INFO="/camera/camera/depth/camera_info"
        have "$DEPTH_INFO" || DEPTH_INFO="/camera/depth/camera_info"
        RGB="/camera/camera/color/image_raw"
        have "$RGB" || RGB="/camera/color/image_raw"
        RGB_INFO="/camera/camera/color/camera_info"
        have "$RGB_INFO" || RGB_INFO="/camera/color/camera_info"
        ;;
    zed)
        DEPTH="/zed/zed_node/depth/depth_registered"
        DEPTH_INFO="/zed/zed_node/depth/camera_info"
        RGB="/zed/zed_node/rgb/image_rect_color"
        RGB_INFO="/zed/zed_node/rgb/camera_info"
        ;;
    *)
        die "相机参数必须是 auto|realsense|zed（当前: $CAMERA）"
        ;;
esac

if [[ "$REC_MODE" == "rgb" ]]; then
    if ! have "$RGB"; then
        die "RGB 话题 $RGB 不存在。请确认相机驱动已启动（ZED 需 publish_rgb: true），或用 ros2 topic list 检查。"
    fi
    SENSORS=("$RGB")
    echo "==> REC_MODE=rgb：只录 RGB + /loc/robo/odom + /global_path + /tf(/tf_static)，不录深度"
else
    if ! have "$DEPTH"; then
        die "深度话题 $DEPTH 不存在。请确认相机驱动已启动，或用 ros2 topic list 检查实际话题名。"
    fi
    SENSORS=("$DEPTH" "$DEPTH_INFO")
    if [[ "${SKIP_RGB:-0}" != "1" ]]; then
        if have "$RGB"; then
            SENSORS+=("$RGB" "$RGB_INFO")
        else
            warn "未发现 RGB 话题 $RGB，本次只录深度。"
            [[ "$CAMERA" == "zed" ]] && warn "ZED 需把 zed_nav_override.yaml 的 general.publish_rgb 设为 true。"
        fi
    else
        echo "==> SKIP_RGB=1，跳过 RGB"
    fi
fi

# 定位 / 控制 / TF
if [[ "$REC_MODE" == "rgb" ]]; then
    if have "/loc/robo/odom"; then
        CANDIDATE=(/loc/robo/odom /global_path /tf /tf_static)
    elif have "/lio/robo/odom"; then
        warn "/loc/robo/odom 不存在，rgb 模式改用 /lio/robo/odom（world 系）"
        CANDIDATE=(/lio/robo/odom /global_path /tf /tf_static)
    else
        warn "/loc/robo/odom 与 /lio/robo/odom 都不存在，本次不录里程计"
        CANDIDATE=(/global_path /tf /tf_static)
    fi
else
    CANDIDATE=(
        /lio/robo/odom
        /loc/robo/odom
        /global_path
        /cmd_vel
        /motion_enabled
        /startNavi
        /local_planner/status
        /tf
        /tf_static
    )
    # /lio/path 消息会随轨迹不断变长，默认不录；需要时 REC_LIO_PATH=1
    if [[ "${REC_LIO_PATH:-0}" == "1" ]]; then
        CANDIDATE+=(/lio/path)
    fi
fi
OPTIONAL=()
MISSING=()
for t in "${CANDIDATE[@]}"; do
    if have "$t"; then OPTIONAL+=("$t"); else MISSING+=("$t"); fi
done
if (( ${#MISSING[@]} )); then
    warn "以下话题当前不存在，录制时跳过："
    printf '   %s\n' "${MISSING[@]}" >&2
fi

# 可选原始雷达（仅 full 模式；重跑 Super-LIO 用）
if [[ "$REC_MODE" == "full" && "${RAW_LIDAR:-0}" == "1" ]]; then
    for t in /livox/lidar /livox/imu; do
        if have "$t"; then OPTIONAL+=("$t"); else warn "RAW_LIDAR=1 但 $t 不存在，跳过"; fi
    done
fi

# 额外话题
EXTRA=()
if [[ -n "${EXTRA_TOPICS:-}" ]]; then
    read -r -a EXTRA <<< "$EXTRA_TOPICS"
    for t in "${EXTRA[@]}"; do
        have "$t" || warn "EXTRA_TOPICS 中的 $t 当前不存在"
    done
fi

# ---------------------------------------------------------------------------
# 图像降频：默认只把图像录到 REC_RATE Hz，实时策略仍拿相机原始帧率
# ---------------------------------------------------------------------------
THROTTLE_PID=""
THROTTLE_SCRIPT=""
REC_PID=""
cleanup() {
    if [[ -n "$THROTTLE_PID" ]] && kill -0 "$THROTTLE_PID" 2>/dev/null; then
        kill "$THROTTLE_PID" 2>/dev/null || true
        wait "$THROTTLE_PID" 2>/dev/null || true
    fi
    [[ -n "$THROTTLE_SCRIPT" ]] && rm -f "$THROTTLE_SCRIPT"
}
trap cleanup EXIT

if [[ "$REC_RATE" != "0" ]]; then
    PAIRS=()
    NEW_SENSORS=()
    if [[ "$REC_MODE" == "rgb" ]]; then
        if have "$RGB"; then
            PAIRS+=(--pair "$RGB:/rec/color/image_raw")
            NEW_SENSORS+=("/rec/color/image_raw")
        fi
    else
        if have "$DEPTH"; then
            PAIRS+=(--pair "$DEPTH:/rec/depth/image_rect_raw")
            NEW_SENSORS+=("/rec/depth/image_rect_raw")
        fi
        have "$DEPTH_INFO" && NEW_SENSORS+=("$DEPTH_INFO")
        if [[ "${SKIP_RGB:-0}" != "1" ]] && have "$RGB"; then
            PAIRS+=(--pair "$RGB:/rec/color/image_raw")
            NEW_SENSORS+=("/rec/color/image_raw")
            have "$RGB_INFO" && NEW_SENSORS+=("$RGB_INFO")
        fi
    fi

    if (( ${#PAIRS[@]} > 0 )); then
        THROTTLE_SCRIPT="$(mktemp "${TMPDIR:-/tmp}/image_throttle_XXXXXX.py")"
        cat > "$THROTTLE_SCRIPT" <<'PY'
#!/usr/bin/env python3
"""把图像话题按固定频率转发；实时链路不变，只降低录制负载。"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Image


class ImageThrottle(Node):
    def __init__(self, pairs, rate):
        super().__init__('record_image_throttle')
        self.period = 1.0 / rate
        self.last = {}
        self.pubs = {}
        for src, dst in pairs:
            self.pubs[dst] = self.create_publisher(Image, dst, qos_profile_sensor_data)
            self.last[dst] = 0.0
            self.create_subscription(
                Image, src, lambda msg, d=dst: self.cb(msg, d),
                qos_profile_sensor_data)
        self.get_logger().info(
            'throttle %.2f Hz: %s' % (rate, ', '.join('%s -> %s' % p for p in pairs)))

    def cb(self, msg, dst):
        now = time.monotonic()
        if now - self.last[dst] < self.period:
            return
        self.last[dst] = now
        self.pubs[dst].publish(msg)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--rate', type=float, required=True)
    ap.add_argument('--pair', action='append', required=True, help='src:dst')
    args = ap.parse_args()
    pairs = []
    for item in args.pair:
        src, sep, dst = item.partition(':')
        if not sep or not src or not dst:
            print('bad --pair: %s' % item, file=sys.stderr)
            return 2
        pairs.append((src, dst))
    rclpy.init()
    node = ImageThrottle(pairs, args.rate)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # 被 kill 时 context 可能已经关闭，忽略二次 shutdown 异常
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
PY
        PYTHON_BIN=/usr/bin/python3
        [[ -x "$PYTHON_BIN" ]] || PYTHON_BIN=python3
        "$PYTHON_BIN" -c 'import rclpy' 2>/dev/null \
            || die "python3 无法 import rclpy，无法启动图像降频节点（可 REC_RATE=0 关闭降频）。"
        nice -n "$REC_NICE" "$PYTHON_BIN" "$THROTTLE_SCRIPT" --rate "$REC_RATE" "${PAIRS[@]}" &
        THROTTLE_PID=$!
        sleep 1.5
        if ! kill -0 "$THROTTLE_PID" 2>/dev/null; then
            THROTTLE_PID=""
            die "图像降频节点启动失败（REC_RATE=$REC_RATE）。可先用 REC_RATE=0 排查。"
        fi
        SENSORS=("${NEW_SENSORS[@]}")
        echo "==> 图像录制频率: ${REC_RATE} Hz（实时策略仍为相机原始帧率）"
    else
        warn "没有可降频的图像话题，按原始话题录制。"
    fi
else
    echo "==> REC_RATE=0：按相机原始帧率录制图像"
fi

# 输出目录
STAMP="$(date +%Y%m%d_%H%M%S)"
if [[ -n "$NAME" ]]; then
    BAG="$BAG_DIR/${NAME}_${STAMP}"
else
    BAG="$BAG_DIR/nav_${CAMERA}_${REC_MODE}_${STAMP}"
fi
mkdir -p "$BAG_DIR"
QOS="$BAG_DIR/qos_overrides.yaml"

cat > "$QOS" <<'YAML'
"/camera/camera/color/image_raw":
  reliability: best_effort
  durability: volatile
  history: keep_last
  depth: 5
"/camera/color/image_raw":
  reliability: best_effort
  durability: volatile
  history: keep_last
  depth: 5
"/camera/camera/depth/image_rect_raw":
  reliability: best_effort
  durability: volatile
  history: keep_last
  depth: 5
"/camera/depth/image_rect_raw":
  reliability: best_effort
  durability: volatile
  history: keep_last
  depth: 5
"/zed/zed_node/rgb/image_rect_color":
  reliability: best_effort
  durability: volatile
  history: keep_last
  depth: 5
"/zed/zed_node/depth/depth_registered":
  reliability: best_effort
  durability: volatile
  history: keep_last
  depth: 5
"/rec/depth/image_rect_raw":
  reliability: best_effort
  durability: volatile
  history: keep_last
  depth: 5
"/rec/color/image_raw":
  reliability: best_effort
  durability: volatile
  history: keep_last
  depth: 5
"/tf_static":
  reliability: reliable
  durability: transient_local
  history: keep_last
  depth: 10
YAML

# 存储格式
STORAGE="${STORAGE:-auto}"
if [[ "$STORAGE" == "auto" ]]; then
    if ros2 pkg prefix rosbag2_storage_mcap >/dev/null 2>&1; then
        STORAGE=mcap
    else
        STORAGE=sqlite3
    fi
fi

# 按当前 rosbag2 版本探测可用参数（Foxy/Humble 差异）
BAG_HELP="$(ros2 bag record --help 2>/dev/null || true)"

QOS_ARGS=()
if grep -q -- "--qos-profile-overrides-path" <<<"$BAG_HELP" 2>/dev/null; then
    QOS_ARGS=(--qos-profile-overrides-path "$QOS")
else
    warn "当前 rosbag2 不支持 --qos-profile-overrides-path，BEST_EFFORT 话题可能录不到。"
fi

STORAGE_ARGS=()
if grep -q -- "--storage" <<<"$BAG_HELP" 2>/dev/null; then
    STORAGE_ARGS=(--storage "$STORAGE")
else
    STORAGE=_default_
fi

CACHE_ARGS=()
if grep -q -- "--max-cache-size" <<<"$BAG_HELP" 2>/dev/null; then
    CACHE_ARGS=(--max-cache-size $(( MAX_CACHE_MB * 1024 * 1024 )))
fi

COMP_ARGS=()
if [[ "${NO_COMPRESS:-0}" != "1" ]]; then
    if grep -q -- "--compression-mode" <<<"$BAG_HELP" 2>/dev/null \
       && ros2 pkg prefix rosbag2_compression_zstd >/dev/null 2>&1; then
        COMP_ARGS=(--compression-mode file --compression-format zstd)
    else
        warn "当前 rosbag2 无 zstd 压缩支持，本次不压缩。"
    fi
fi

_STORAGE_SHOW="$STORAGE"
[[ "$STORAGE" == "_default_" ]] && _STORAGE_SHOW="rosbag2默认"
_RATE_SHOW="原始帧率(不降频)"
[[ "$REC_RATE" != "0" ]] && _RATE_SHOW="${REC_RATE} Hz（实时链路不降频）"

echo "============================================================"
echo " 相机      : $CAMERA"
echo " 输出      : $BAG"
echo " 图像频率  : $_RATE_SHOW"
echo " 存储/压缩 : $_STORAGE_SHOW $([[ ${#COMP_ARGS[@]} -gt 0 ]] && echo '+ zstd' || echo '(无压缩)')"
echo " 话题      :"
printf '   %s\n' "${SENSORS[@]}" "${OPTIONAL[@]}" "${EXTRA[@]}"
echo "------------------------------------------------------------"
echo " 按 Ctrl+C 停止并保存；结束后可用:"
echo "   ros2 bag info \"$BAG\""
echo "============================================================"

# 让录制进程给导航让路：nice + 低优先级 IO
PRIO_CMD=()
if command -v nice >/dev/null 2>&1; then
    PRIO_CMD=(nice -n "$REC_NICE")
fi
if command -v ionice >/dev/null 2>&1; then
    PRIO_CMD+=(ionice -c 2 -n 7)
else
    warn "ionice 不可用（可 apt install util-linux），IO 抢占可能仍会影响导航。"
fi
if [[ -n "$REC_CPUS" ]]; then
    if command -v taskset >/dev/null 2>&1; then
        PRIO_CMD=(taskset -c "$REC_CPUS" "${PRIO_CMD[@]}")
    else
        warn "taskset 不可用，忽略 REC_CPUS=$REC_CPUS"
    fi
fi

REC_CMD=(
    "${PRIO_CMD[@]}"
    ros2 bag record
    "${SENSORS[@]}" "${OPTIONAL[@]}" "${EXTRA[@]}"
    "${QOS_ARGS[@]}"
    "${CACHE_ARGS[@]}"
    "${STORAGE_ARGS[@]}"
    "${COMP_ARGS[@]}"
    -o "$BAG"
)

if [[ -n "$THROTTLE_PID" ]]; then
    # 有降频节点时不能 exec，否则节流进程会变成孤儿
    "${REC_CMD[@]}" &
    REC_PID=$!
    trap 'wait "$REC_PID" 2>/dev/null; exit 130' INT TERM
    if wait "$REC_PID"; then
        REC_STATUS=0
    else
        REC_STATUS=$?
    fi
    trap - INT TERM
    exit "$REC_STATUS"
else
    exec "${REC_CMD[@]}"
fi

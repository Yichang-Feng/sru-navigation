#!/usr/bin/env python3
"""RealSense D435/D435i 最大测距能力与深度质量实时测试诊断工具 (test_realsense_max_distance.py)

功能：
1. 实时订阅 RealSense 深度图像话题 (默认 /camera/camera/aligned_depth_to_color/image_raw)
2. 逐帧统计：
   - 画面有效像素率 (Valid Ratio %) 与 零值空洞率 (Zero/Void Ratio %)
   - 最近探测距离 (Min Depth)、平均距离 (Mean)、中位数距离 (Median)
   - 90% 分位深度、95% 分位深度、99% 分位深度与绝对最远探测距离 (Max Depth)
   - 中央通道/门洞窗口 (Doorway ROI) 的测距情况与穿透深度
   - 终端动态直方图 (ASCII Range Distribution)
3. 支持离线测试模式：直接传入 --session 或 --file 分析录制的诊断数据集

使用方法：
1. 机器人真机在线实时测试：
   python3 scripts/test_realsense_max_distance.py
   python3 scripts/test_realsense_max_distance.py --topic /camera/camera/aligned_depth_to_color/image_raw

2. 离线历史诊断数据分析：
   python3 scripts/test_realsense_max_distance.py --session door_diagnostics/session_20261110_152256_stairs_test5
   python3 scripts/test_realsense_max_distance.py --file door_diagnostics/session_20261110_152256_stairs_test5/depth/frame_000060_depth_40x64.npy
"""

import os
import sys
import time
import argparse
import numpy as np

# 尝试导入 ROS 2 依赖（离线模式下即使无 ROS 2 也可运行）
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False


def analyze_depth_array(depth_m: np.ndarray, min_valid: float = 0.15, max_clip: float = 20.0):
    """统计单帧深度矩阵的各项距离指标与分布"""
    # 过滤非数与无穷大
    valid_mask = np.isfinite(depth_m) & (depth_m > min_valid) & (depth_m <= max_clip)
    valid_count = int(np.sum(valid_mask))
    total_count = depth_m.size
    valid_ratio = valid_count / total_count if total_count > 0 else 0.0
    zero_count = int(np.sum(depth_m == 0.0) + np.sum(np.isnan(depth_m)))
    zero_ratio = zero_count / total_count if total_count > 0 else 0.0

    if valid_count > 0:
        valid_depths = depth_m[valid_mask]
        min_val = float(np.min(valid_depths))
        mean_val = float(np.mean(valid_depths))
        median_val = float(np.median(valid_depths))
        p90 = float(np.percentile(valid_depths, 90))
        p95 = float(np.percentile(valid_depths, 95))
        p99 = float(np.percentile(valid_depths, 99))
        max_val = float(np.max(valid_depths))
    else:
        min_val = mean_val = median_val = p90 = p95 = p99 = max_val = 0.0

    # 中央视野/门洞走廊窗口分析 (中央高度 30%~70%, 宽度 35%~65%)
    h, w = depth_m.shape
    c_y1, c_y2 = int(h * 0.35), int(h * 0.65)
    c_x1, c_x2 = int(w * 0.35), int(w * 0.65)
    center_roi = depth_m[c_y1:c_y2, c_x1:c_x2]
    c_valid = center_roi[np.isfinite(center_roi) & (center_roi > min_valid) & (center_roi <= max_clip)]
    c_max = float(np.max(c_valid)) if len(c_valid) > 0 else 0.0
    c_avg = float(np.mean(c_valid)) if len(c_valid) > 0 else 0.0
    c_void = float(np.mean((center_roi <= min_valid) | (~np.isfinite(center_roi))))

    # 分段直方图统计
    bins = [0.0, 1.0, 2.0, 3.0, 5.0, 8.0, 12.0]
    hist_counts, _ = np.histogram(depth_m[valid_mask], bins=bins) if valid_count > 0 else ([0]*6, bins)
    hist_ratios = [c / total_count for c in hist_counts]

    return {
        "valid_count": valid_count,
        "total_count": total_count,
        "valid_ratio": valid_ratio,
        "zero_count": zero_count,
        "zero_ratio": zero_ratio,
        "min": min_val,
        "mean": mean_val,
        "median": median_val,
        "p90": p90,
        "p95": p95,
        "p99": p99,
        "max": max_val,
        "center_max": c_max,
        "center_avg": c_avg,
        "center_void": c_void,
        "hist_ratios": hist_ratios,
    }


def format_stats_line(idx: int, stats: dict) -> str:
    """生成单行终端诊断输出与分布条"""
    hr = stats["hist_ratios"]
    dist_str = (
        f"<1m:{hr[0]*100:2.0f}% "
        f"1-2m:{hr[1]*100:2.0f}% "
        f"2-3m:{hr[2]*100:2.0f}% "
        f"3-5m:{hr[3]*100:2.0f}% "
        f"5-8m:{hr[4]*100:2.0f}% "
        f">8m:{hr[5]*100:2.0f}%"
    )
    return (
        f"[Frame {idx:04d}] "
        f"有效率:{stats['valid_ratio']*100:4.1f}% | "
        f"最远Max:\033[92m{stats['max']:5.2f}m\033[0m | "
        f"95%分位:\033[96m{stats['p95']:5.2f}m\033[0m | "
        f"中位:\033[93m{stats['median']:4.2f}m\033[0m | "
        f"门洞穿透:{stats['center_max']:5.2f}m(空洞{stats['center_void']*100:2.0f}%) | "
        f"分布:[{dist_str}]"
    )


class RealSenseDistanceBenchmarkNode(Node if HAS_ROS2 else object):
    def __init__(self, topic: str):
        super().__init__('realsense_distance_benchmark')
        self.bridge = CvBridge()
        self.frame_idx = 0
        self.max_reach_overall = 0.0
        self.p95_reach_list = []
        self.valid_ratio_list = []

        self.subscription = self.create_subscription(
            Image, topic, self.depth_callback, 10
        )
        self.get_logger().info(f"已监听深度话题: {topic}，正在评估相机最大测距能力... (按 Ctrl+C 退出并查看汇总)")

    def depth_callback(self, msg):
        self.frame_idx += 1
        try:
            depth_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            if getattr(msg, 'encoding', '') in ('16UC1', 'mono16') or depth_raw.dtype == np.uint16:
                depth_m = depth_raw.astype(np.float32) / 1000.0
            else:
                depth_m = depth_raw.astype(np.float32)
        except Exception as e:
            self.get_logger().error(f"深度图像解析异常: {e}")
            return

        stats = analyze_depth_array(depth_m)
        self.max_reach_overall = max(self.max_reach_overall, stats["max"])
        self.p95_reach_list.append(stats["p95"])
        self.valid_ratio_list.append(stats["valid_ratio"])

        line = format_stats_line(self.frame_idx, stats)
        sys.stdout.write(f"\r{line}   ")
        sys.stdout.flush()

    def print_summary(self):
        print("\n" + "=" * 70)
        print(">>> RealSense D435 测距能力测试汇总")
        print(f"总处理帧数: {self.frame_idx}")
        if self.frame_idx > 0:
            avg_valid = float(np.mean(self.valid_ratio_list)) * 100
            avg_p95 = float(np.mean(self.p95_reach_list))
            print(f"全局最远探测极限 (Peak Max): \033[92m{self.max_reach_overall:.2f} 米\033[0m")
            print(f"稳定有效测距范围 (Avg 95%): \033[96m{avg_p95:.2f} 米\033[0m")
            print(f"平均画面有效覆盖率 (Valid Ratio): {avg_valid:.1f}%")
            if self.max_reach_overall >= 6.0:
                print("\033[92m[结论] 远距探测正常：相机已成功探测至 6 米以上，能够满足室内大厅与长走廊导航需求！\033[0m")
            elif self.max_reach_overall >= 4.0:
                print("\033[93m[结论] 测距中等：测距在 4~6 米之间，请检查是否已开启高密度预设 (visual_preset:=4) 与红外发射器。\033[0m")
            else:
                print("\033[91m[结论] 测距受限：最远不足 4 米，请确认 laser_power 是否调至 360，且滤镜 (filters) 已生效。\033[0m")
        print("=" * 70)


def run_offline_session(session_path: str):
    """离线分析已录制的数据集"""
    depth_dir = os.path.join(session_path, "depth")
    if not os.path.isdir(depth_dir):
        print(f"[✗] 未在 {session_path} 中找到 depth 子目录")
        return

    npy_files = sorted([os.path.join(depth_dir, f) for f in os.listdir(depth_dir) if f.endswith(".npy")])
    if not npy_files:
        print(f"[✗] {depth_dir} 中未找到任何 .npy 文件")
        return

    print(f"=== 开始离线分析 Session 数据: {os.path.basename(session_path)} (共 {len(npy_files)} 帧) ===")
    max_all = 0.0
    p95_all = []
    valid_all = []

    for idx, fpath in enumerate(npy_files):
        arr = np.load(fpath)
        stats = analyze_depth_array(arr)
        max_all = max(max_all, stats["max"])
        p95_all.append(stats["p95"])
        valid_all.append(stats["valid_ratio"])
        if idx % 10 == 0 or idx == len(npy_files) - 1:
            print(format_stats_line(idx, stats))

    print("\n" + "=" * 70)
    print(f">>> 离线分析结论: {os.path.basename(session_path)}")
    print(f"全局最远探测极限 (Peak Max): \033[92m{max_all:.2f} 米\033[0m")
    print(f"稳定有效测距范围 (Avg 95%): \033[96m{float(np.mean(p95_all)):.2f} 米\033[0m")
    print(f"平均画面有效覆盖率: {float(np.mean(valid_all))*100:.1f}%")
    print("=" * 70)


def run_offline_file(file_path: str):
    """离线分析单个 .npy 文件"""
    if not os.path.isfile(file_path):
        print(f"[✗] 文件不存在: {file_path}")
        return
    arr = np.load(file_path)
    stats = analyze_depth_array(arr)
    print(f"=== 单帧深度分析: {os.path.basename(file_path)} ===")
    print(f"矩阵尺寸: {arr.shape}, 数据类型: {arr.dtype}")
    print(f"有效像素数: {stats['valid_count']} / {stats['total_count']} ({stats['valid_ratio']*100:.1f}%)")
    print(f"零值空洞数: {stats['zero_count']} ({stats['zero_ratio']*100:.1f}%)")
    print(f"最近有效距离: {stats['min']:.3f} m")
    print(f"平均有效距离: {stats['mean']:.3f} m")
    print(f"中位数距离  : {stats['median']:.3f} m")
    print(f"90% 分位距离: {stats['p90']:.3f} m")
    print(f"95% 分位距离: {stats['p95']:.3f} m")
    print(f"最远探测距离: \033[92m{stats['max']:.3f} m\033[0m")
    print(f"门洞/中央最远: \033[96m{stats['center_max']:.3f} m\033[0m (均值: {stats['center_avg']:.3f} m, 虚空率: {stats['center_void']*100:.1f}%)")
    print("=" * 70)


def main():
    parser = argparse.ArgumentParser(description="RealSense D435 最大测距能力测试工具")
    parser.add_argument("--topic", type=str, default="/camera/camera/depth/image_rect_raw",
                        help="ROS 2 深度图像话题名称 (默认原始深度图: /camera/camera/depth/image_rect_raw)")
    parser.add_argument("--session", type=str, default=None,
                        help="离线评估历史 session 目录路径")
    parser.add_argument("--file", type=str, default=None,
                        help="离线评估单个 depth_40x64.npy 文件路径")

    args, unknown = parser.parse_known_args()

    if args.file:
        run_offline_file(args.file)
        return

    if args.session:
        run_offline_session(args.session)
        return

    if not HAS_ROS2:
        print("[!] 当前 Python 环境中未检测到 rclpy，无法运行在线实时 ROS 2 测试。")
        print("    可在有 ROS 2 环境的终端运行: source /opt/ros/humble/setup.bash && python3 scripts/test_realsense_max_distance.py")
        print("    或者使用离线模式: python3 scripts/test_realsense_max_distance.py --session <目录>")
        return

    rclpy.init(args=unknown)
    node = RealSenseDistanceBenchmarkNode(topic=args.topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.print_summary()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

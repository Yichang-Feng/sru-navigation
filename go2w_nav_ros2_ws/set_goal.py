#!/usr/bin/env python3
"""一次性固定目标点下发（参数扫描用）。

等价于在 RViz 里用 Goal3DTool 点一次：向 /goal_3d 发布**单条** PoseStamped。
这一条消息同时被两个节点消费：
  1. global_planner：以最新 odom 为起点重新 A* 规划 -> /global_path
     （同一张地图 + 同一个 goal + 同一个起点 => 每轮路径完全一致）；
  2. rl_nav_controller(_stairs)：设置终端目标（到达判定 + 终点航向对齐）。

默认值是 run4 (session_20261110_144216_realsense, goals.jsonl) 的精确 goal，
map 系：(-0.849791, 10.228565, -4.624915) yaw=139.1865°。
run4 对应路径参考：91 点、长 34.0 m、终点 (-0.8498, 10.2286, -4.1249)。

用法（机器狗上，导航已启动、机器人已摆到胶带起点之后，按 A 之前）：
    python3 set_goal.py                      # 用 run4 固定 goal
    python3 set_goal.py --yaw 140            # 覆盖单个分量
退出码 0 = 发布成功且收到新路径；1 = 10 s 内没有新路径（规划失败/无起点）。
"""
import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def path_stats(msg):
    pts = msg.poses
    n = len(pts)
    if n == 0:
        return None
    length = 0.0
    for a, b in zip(pts[:-1], pts[1:]):
        pa, pb = a.pose.position, b.pose.position
        dx = pb.x - pa.x
        dy = pb.y - pa.y
        dz = pb.z - pa.z
        length += math.sqrt(dx * dx + dy * dy + dz * dz)
    f, l = pts[0].pose.position, pts[-1].pose.position
    return {
        'n': n,
        'length': length,
        'first': (f.x, f.y, f.z),
        'last': (l.x, l.y, l.z),
    }


def main():
    ap = argparse.ArgumentParser(description='One-shot fixed goal on /goal_3d')
    ap.add_argument('--x', type=float, default=-0.8497912287712097)
    ap.add_argument('--y', type=float, default=10.228565216064453)
    ap.add_argument('--z', type=float, default=-4.62491512298584)
    ap.add_argument('--yaw', type=float, default=139.18651855020798, help='deg')
    ap.add_argument('--frame', default='map')
    ap.add_argument('--wait-path-sec', type=float, default=10.0)
    args = ap.parse_args()

    rclpy.init()
    node = Node('set_goal_fixed')
    path_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.TRANSIENT_LOCAL)
    state = {'count': 0, 'last': None}

    def path_cb(msg):
        state['count'] += 1
        state['last'] = msg

    node.create_subscription(Path, '/global_path', path_cb, path_qos)
    pub = node.create_publisher(
        PoseStamped, '/goal_3d', QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))

    # 先收一次 latched 旧路径（planner 用 transient_local），区分"新规划"
    t0 = time.time()
    while state['count'] == 0 and time.time() - t0 < 3.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    old = path_stats(state['last']) if state['count'] else None
    if old:
        print(f'[set_goal] 旧路径: n={old["n"]} len={old["length"]:.2f} m '
              f'end=({old["last"][0]:.3f}, {old["last"][1]:.3f}, {old["last"][2]:.3f})')

    msg = PoseStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = args.frame
    yaw = math.radians(args.yaw)
    msg.pose.position.x = args.x
    msg.pose.position.y = args.y
    msg.pose.position.z = args.z
    msg.pose.orientation.x = 0.0
    msg.pose.orientation.y = 0.0
    msg.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.orientation.w = math.cos(yaw / 2.0)
    pub.publish(msg)
    print(f'[set_goal] 已发布 /goal_3d x1: frame={args.frame} '
          f'pos=({args.x:.4f}, {args.y:.4f}, {args.z:.4f}) yaw={args.yaw:.2f} deg')

    t1 = time.time()
    ok = False
    while time.time() - t1 < args.wait_path_sec:
        rclpy.spin_once(node, timeout_sec=0.1)
        if state['count'] > (1 if old else 0):
            st = path_stats(state['last'])
            print(f'[set_goal] 新路径 OK: n={st["n"]} len={st["length"]:.2f} m '
                  f'start=({st["first"][0]:.3f}, {st["first"][1]:.3f}, {st["first"][2]:.3f}) '
                  f'end=({st["last"][0]:.3f}, {st["last"][1]:.3f}, {st["last"][2]:.3f})')
            ok = True
            break

    node.destroy_node()
    rclpy.shutdown()
    if not ok:
        print('[set_goal] 警告: 发布后未收到新 /global_path（起点未就绪/不可达/规划超时？）',
              file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())

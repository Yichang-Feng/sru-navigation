#!/usr/bin/env python3
"""Controller-side latency probe (run on the robot, not on the dev machine).

Purpose
-------
Stage 0 found that the carrot the policy receives sits ~0.35-0.5 m *behind*
where the current pose would put it (equivalent pose delay tau ~= 1.2 s), and on
the stairs the commanded 0.40 m lookahead is cancelled entirely. Three very
different causes produce that identical symptom:

  (A) the controller's path progress uses an old map pose (control loop /
      executor behind), so `progress` and hence the carrot lag;
  (B) the controller is current but `/vis/lookahead_goal` is recorded late;
  (C) the deployed tracker / parameters differ from the repository.

This script separates (A) from (B) **without touching the controller**: it
records, for every message, both the host arrival time and the header stamp.

Key relationship
----------------
`/vis/lookahead_goal` is published with `header.stamp = self.robot_odom_time`,
i.e. the stamp of the *odom message the controller last processed*. So:

    marker_age = arrival(marker) - stamp(marker)
               = (age of the controller's pose at publish) + (transport delay)

and for odometry, measured in this same process:

    odom_age   = arrival(odom) - stamp(odom)
               = (odom's own latency) + (this process's scheduling delay)

If `marker_age ~= odom_age`, the controller is current -> the delay is in the
publish/record path (B). If `marker_age ~= odom_age + 1 s`, the controller is
processing a stale pose -> (A).

This process must be cheap; keep it in its own terminal and do not run RViz
on the same machine while measuring.

Topics
------
  /loc/robo/odom        nav_msgs/Odometry         map-frame pose (what the
                                                  controller prefers for 'map')
  /lio/robo/odom        nav_msgs/Odometry         odom-frame pose (rate check)
  /vis/lookahead_goal   visualization_msgs/Marker carrot, stamp = ctrl odom time
  /cmd_vel              geometry_msgs/Twist       command stream timing

Usage
-----
    python3 diag_ctrl_latency.py --seconds 90 --out ~/latency_$(date +%H%M%S).csv
    # drive: 30 s straight at ~0.4 m/s, then a 90 deg turn, then stop

Then analyse offline (on the dev machine):

    python3 scripts/analyze_ctrl_latency.py ~/latency_XXXX.csv

Output CSV columns:
    host_time, topic, stamp, age_ms, x, y, z, lin_x, ang_z
"""

import argparse
import csv
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from visualization_msgs.msg import Marker

TOPICS = ["/loc/robo/odom", "/lio/robo/odom", "/vis/lookahead_goal", "/cmd_vel"]


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class LatencyProbe(Node):
    def __init__(self, out_path, seconds):
        super().__init__("ctrl_latency_probe")
        self.out_path = out_path
        self.t_end = time.time() + seconds
        self.t0 = time.time()
        self.f = open(out_path, "w", newline="", encoding="utf-8")
        self.w = csv.writer(self.f)
        self.w.writerow(["host_time", "topic", "stamp", "age_ms", "x", "y", "z", "lin_x", "ang_z"])
        self.count = {t: 0 for t in TOPICS}
        self.max_age = {t: 0.0 for t in TOPICS}

        self.create_subscription(
            Odometry, "/loc/robo/odom", lambda m: self.odom_cb(m, "/loc/robo/odom"), 50)
        self.create_subscription(
            Odometry, "/lio/robo/odom", lambda m: self.odom_cb(m, "/lio/robo/odom"), 50)
        self.create_subscription(Marker, "/vis/lookahead_goal", self.marker_cb, 50)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_cb, 50)
        self.create_timer(1.0, self.progress)
        self.get_logger().info(
            f"probing {TOPICS} for {seconds:.0f}s -> {out_path}")

    def _row(self, topic, stamp, x=0.0, y=0.0, z=0.0, lin_x=0.0, ang_z=0.0):
        now = time.time()
        age_ms = (now - stamp) * 1000.0 if stamp > 0 else float("nan")
        self.w.writerow([f"{now:.6f}", topic, f"{stamp:.6f}",
                         f"{age_ms:.1f}" if math.isfinite(age_ms) else "",
                         f"{x:.4f}", f"{y:.4f}", f"{z:.4f}",
                         f"{lin_x:.4f}", f"{ang_z:.4f}"])
        if math.isfinite(age_ms):
            self.count[topic] += 1
            self.max_age[topic] = max(self.max_age[topic], age_ms / 1000.0)

    def odom_cb(self, msg: Odometry, topic: str):
        p = msg.pose.pose.position
        self._row(topic, stamp_to_sec(msg.header.stamp), p.x, p.y, p.z)

    def marker_cb(self, msg: Marker):
        p = msg.pose.position
        self._row("/vis/lookahead_goal", stamp_to_sec(msg.header.stamp), p.x, p.y, p.z)

    def cmd_cb(self, msg: Twist):
        self._row("/cmd_vel", time.time(), 0.0, 0.0, 0.0, msg.linear.x, msg.angular.z)

    def progress(self):
        el = time.time() - self.t0
        if int(el) % 5 == 0:
            self.get_logger().info(
                f"t={el:5.1f}s  " + "  ".join(
                    f"{t.split('/')[-1]}:{self.count[t]}" for t in TOPICS))
        self.f.flush()
        if time.time() >= self.t_end:
            self.get_logger().info(
                "done. max age (s): " + "  ".join(
                    f"{t.split('/')[-1]}={self.max_age[t]:.3f}" for t in TOPICS))
            self.f.close()
            raise SystemExit(0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--seconds", type=float, default=90.0)
    args = ap.parse_args()
    rclpy.init()
    node = LatencyProbe(args.out, args.seconds)
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        try:
            node.f.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

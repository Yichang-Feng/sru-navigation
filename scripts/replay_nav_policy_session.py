#!/usr/bin/env python3
"""Replay the deployed RL navigation policy on a recorded door_diagnostics session.

The input assembly mirrors LearningModel.predict in rl_nav_controller_adaptive.py:
  obs = [linear_vel(3), angular_vel(3), gravity(3), last_action(3),
         target_dir(3), log1p(target_dist)(1), VAE(depth_40x64).flatten(2560)]

The 40x64 depth snapshots saved by door_diagnostic_logger are already the exact
clipped/resized image the controller passes to the VAE encoder. Replay starts at
`--replay-start` (default: first RUNNING frame) so the LSTM state and last action
are carried forward the same way as in the live controller.

The logger stores pose in `map` but the odom twist in `odom` frame; converting the
twist with the odom-frame yaw is required. A ~1 s pose lag (`--pose-lag-frames 4`
at the 4 Hz recording rate) reproduces the logged commands best on this session.
"""

import argparse
import csv
import json
import os

import numpy as np
import onnxruntime as ort
from scipy.spatial.transform import Rotation as R

POLICY_SCALE = np.array([1.5, 1.0, 1.0], dtype=np.float32)
STATE_DIM = 16
LSTM_HIDDEN = 512


def quat_wxyz_from_meta(att, yaw_deg=None):
    """ROS Euler attitude (roll,pitch,yaw degrees) -> quaternion [w,x,y,z]."""
    yaw = att["yaw_deg"] if yaw_deg is None else yaw_deg
    r = R.from_euler("ZYX", [yaw, att["pitch_deg"], att["roll_deg"]], degrees=True)
    x, y, z, w = r.as_quat()
    return np.array([w, x, y, z], dtype=np.float32)


def odom_yaw_deg(all_meta, idx, window=4):
    """Estimate the yaw of the odom message from smoothed /lio/robo/odom positions.

    The logger records pose in map frame but velocity twist in odom frame; the
    controller converts that twist with the quaternion from the same odom msg.
    """
    lo = max(0, idx - window)
    hi = min(len(all_meta), idx + window + 1)
    t = np.arange(lo, hi, dtype=float)
    p = np.array([all_meta[j]["robot_pos_odom"][:2] for j in range(lo, hi)], dtype=float)
    dx = np.polyfit(t, p[:, 0], 1)[0]
    dy = np.polyfit(t, p[:, 1], 1)[0]
    if np.hypot(dx, dy) < 1e-4:
        return None
    return float(np.degrees(np.arctan2(dy, dx)))


def inv_apply(q_wxyz, v):
    q_xyzw = np.array([q_wxyz[1], q_wxyz[2], q_wxyz[3], q_wxyz[0]], dtype=np.float64)
    return R.from_quat(q_xyzw).inv().apply(np.asarray(v, dtype=np.float64))


def load_session(sess_dir):
    rows = list(csv.DictReader(open(os.path.join(sess_dir, "trajectory.csv"))))
    by_frame = {}
    for row in rows:
        by_frame[int(float(row["frame_idx"]))] = row
    frames = sorted(by_frame)
    all_meta = []
    for fi in frames:
        p = os.path.join(sess_dir, "meta", f"frame_{fi:06d}_meta.json")
        all_meta.append(json.load(open(p)) if os.path.exists(p) else None)
    return rows, by_frame, frames, all_meta


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--session", required=True)
    ap.add_argument("--start", type=int, default=385)
    ap.add_argument("--end", type=int, default=395)
    ap.add_argument("--replay-start", type=int, default=0,
                    help="first frame to feed to the policy; 0 = first RUNNING frame")
    ap.add_argument("--vae", default="/home/yichangfeng/sru-robot-deployment/go2w_nav_ros2_ws/"
                                     "src/rl_nav_controller/deployment_policies/vae_encoder.onnx")
    ap.add_argument("--policy", default="/home/yichangfeng/sru-robot-deployment/go2w_nav_ros2_ws/"
                                        "src/rl_nav_controller/deployment_policies/nav_policy.onnx")
    ap.add_argument("--vel-yaw-source", choices=["map", "odom", "base"], default="odom",
                    help="yaw used to rotate the logged odom twist into base frame")
    ap.add_argument("--pose-lag-frames", type=int, default=4,
                    help="use pose/velocity from N frames earlier (controller latency)")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    sess_dir = os.path.abspath(args.session)
    rows, by_frame, frames, all_meta = load_session(sess_dir)

    if args.replay_start:
        start_frame = args.replay_start
    else:
        start_frame = next((f for f in frames
                            if by_frame[f].get("nav_state") == "running"
                            and by_frame[f].get("carrot_source") != "none"), None)
        if start_frame is None:
            raise RuntimeError("no RUNNING frame with a carrot found")

    vae = ort.InferenceSession(args.vae, providers=["CPUExecutionProvider"])
    policy = ort.InferenceSession(args.policy, providers=["CPUExecutionProvider"])
    vae_in = vae.get_inputs()[0].name
    vae_out = vae.get_outputs()[0].name

    h = np.zeros((1, 1, LSTM_HIDDEN), dtype=np.float32)
    c = np.zeros((1, 1, LSTM_HIDDEN), dtype=np.float32)
    last_action = np.zeros(3, dtype=np.float32)

    results = []
    for idx, fi in enumerate(frames):
        if fi < start_frame:
            continue
        row = by_frame[fi]
        # The controller stops invoking the policy outside RUNNING; in this session
        # frames 43..515 are the active navigation segment containing 385..395.
        active = row.get("nav_state") == "running" and row.get("carrot_source") != "none"

        meta = all_meta[idx]
        meta_pose = all_meta[max(0, idx - args.pose_lag_frames)]
        depth_path = os.path.join(sess_dir, "depth", f"frame_{fi:06d}_depth_40x64.npy")
        if meta is None or meta_pose is None or not os.path.exists(depth_path):
            continue
        depth = np.load(depth_path).astype(np.float32, copy=False)
        if depth.shape != (40, 64):
            raise RuntimeError(f"frame {fi}: expected depth (40,64), got {depth.shape}")

        emb = vae.run([vae_out], {vae_in: depth[np.newaxis, np.newaxis]})[0].flatten()

        att = meta_pose["robot_attitude"]
        q_map = quat_wxyz_from_meta(att)
        if args.vel_yaw_source == "map":
            q_vel = q_map
        elif args.vel_yaw_source == "odom":
            yaw_odom = odom_yaw_deg(all_meta, max(0, idx - args.pose_lag_frames))
            q_vel = q_map if yaw_odom is None else quat_wxyz_from_meta(att, yaw_deg=yaw_odom)
        else:  # velocity already in base frame
            q_vel = None
        pos = np.array(meta_pose["robot_pos"], dtype=np.float64)
        target = np.array(meta["carrot_pos"], dtype=np.float64)
        vel_w = np.array([meta_pose["odom_velocity"][k] for k in ("vx", "vy", "vz")], dtype=np.float64)
        omega_w = np.array([0.0, 0.0, meta_pose["odom_velocity"]["wz"]], dtype=np.float64)

        linear = vel_w.astype(np.float32) if q_vel is None else inv_apply(q_vel, vel_w).astype(np.float32)
        angular = omega_w.astype(np.float32) if q_vel is None else inv_apply(q_vel, omega_w).astype(np.float32)
        gravity = inv_apply(q_vel if q_vel is not None else q_map, [0.0, 0.0, -9.81])
        gravity = (gravity / np.linalg.norm(gravity)).astype(np.float32)

        target_b = inv_apply(q_map, target - pos)
        dist = float(np.linalg.norm(target_b) + 1e-6)
        target_state = np.concatenate([target_b / dist, [np.log(dist + 1.0)]]).astype(np.float32)

        state = np.concatenate([linear, angular, gravity, last_action, target_state]).astype(np.float32)
        assert state.shape == (STATE_DIM,)
        obs = np.concatenate([state, emb])[np.newaxis].astype(np.float32)

        outputs = policy.run(None, {"obs": obs, "h_in": h, "c_in": c})
        raw_action, h, c = outputs[0][0], outputs[1], outputs[2]
        cmd = np.tanh(raw_action) * POLICY_SCALE
        last_action = raw_action.astype(np.float32)

        rec = {
            "frame": fi,
            "active": active,
            "cmd_vx": float(cmd[0]), "cmd_vy": float(cmd[1]), "cmd_wz": float(cmd[2]),
            "raw_vx": float(raw_action[0]), "raw_vy": float(raw_action[1]), "raw_wz": float(raw_action[2]),
            "target_b": [float(x) for x in target_b],
            "target_yaw_deg": float(np.degrees(np.arctan2(target_b[1], target_b[0]))),
            "linear_base": [float(x) for x in linear],
            "angular_base": [float(x) for x in angular],
            "gravity_base": [float(x) for x in gravity],
            "recorded_cmd": [float(row["cmd_vx"]), float(row["cmd_vy"]), float(row["cmd_wz"])],
        }
        results.append(rec)

    selected = [r for r in results if args.start <= r["frame"] <= args.end]
    report = {
        "session": sess_dir,
        "policy": args.policy,
        "vae": args.vae,
        "replay_start_frame": start_frame,
        "frames": selected,
        "all_frame_count": len(results),
    }
    if args.out:
        os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
        json.dump(report, open(args.out, "w"), indent=2, ensure_ascii=False)
    print(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()

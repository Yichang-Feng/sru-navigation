#!/usr/bin/env python3
"""Ablation Study: Stairs Descent Lookahead Distance & Depth Image Processing.

Tests policy behavior on Session 20261103_184007_stairs_test4 Frame 185:
- Comparing Raw Depth vs Remapped Depth
- Across lookahead distances from very small (0.05m ~ 0.3m) to standard (0.4m ~ 0.5m) to long (1.0m ~ 2.0m)
- Evaluating forward velocity (vx), lateral drift (vy), angular turn rate (wz), and descent viability.
"""

import os
import sys
import json
import math
import numpy as np
import cv2
import onnxruntime as ort

# Project paths
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

SESSION_DIR = os.path.join(PROJECT_ROOT, "door_diagnostics", "session_20261103_184007_stairs_test4")
VAE_PATH = "/home/yichangfeng/go2w_robot_ref/go2w_nav_ros2_ws/src/rl_nav_controller/deployment_policies/vae_encoder.onnx"
POLICY_PATH = "/home/yichangfeng/go2w_robot_ref/go2w_nav_ros2_ws/src/rl_nav_controller/deployment_policies/nav_policy.onnx"

POLICY_SCALE = np.array([1.5, 1.0, 1.0], dtype=np.float32)
LATERAL_SCALE = 0.60
LPF_COEF = np.array([0.9, 0.5, 0.5], dtype=np.float32)


class StairsAblationTester:
    def __init__(self):
        providers = ["CPUExecutionProvider"]
        self.vae_sess = ort.InferenceSession(VAE_PATH, providers=providers)
        self.policy_sess = ort.InferenceSession(POLICY_PATH, providers=providers)
        self.vae_in_name = self.vae_sess.get_inputs()[0].name
        self.vae_out_name = self.vae_sess.get_outputs()[0].name

    def encode_depth(self, depth_img: np.ndarray) -> np.ndarray:
        resized = cv2.resize(depth_img, (64, 40), interpolation=cv2.INTER_LINEAR)
        tensor = resized.astype(np.float32)[np.newaxis, np.newaxis, :, :]
        out = self.vae_sess.run([self.vae_out_name], {self.vae_in_name: tensor})[0]
        return out.flatten()

    def run_policy_step(self, depth_emb, target_vec_b, lin_vel, ang_vel, grav, last_act, h_in, c_in):
        dist = np.linalg.norm(target_vec_b) + 1e-6
        target_norm = target_vec_b / dist
        target_pos_log = np.concatenate([target_norm, [np.log(dist + 1.0)]])

        state_list = list(lin_vel) + list(ang_vel) + list(grav) + list(last_act) + target_pos_log.tolist()
        obs = np.concatenate([state_list, depth_emb])[np.newaxis].astype(np.float32)
        outputs = self.policy_sess.run(None, {"obs": obs, "h_in": h_in, "c_in": c_in})
        raw_act, h_out, c_out = outputs
        raw_act = raw_act.squeeze(0)
        cmd = np.tanh(raw_act) * POLICY_SCALE
        return cmd, raw_act, h_out, c_out

    def multi_step_eval(self, depth_emb, target_vec_b, total_steps=10, initial_vel=None):
        h_state = np.zeros((1, 1, 512), dtype=np.float32)
        c_state = np.zeros((1, 1, 512), dtype=np.float32)
        last_act = np.zeros(3, dtype=np.float32)

        if initial_vel is not None:
            lin_vel = list(initial_vel[:3])
            ang_vel = [0.0, 0.0, initial_vel[3] if len(initial_vel) > 3 else 0.0]
        else:
            lin_vel = [0.0, 0.0, 0.0]
            ang_vel = [0.0, 0.0, 0.0]
        grav = [0.0, 0.0, -1.0]
        prev_cmd = np.zeros(3, dtype=np.float32)

        history = []
        for s in range(total_steps):
            cmd, raw_act, h_state, c_state = self.run_policy_step(
                depth_emb, target_vec_b, lin_vel, ang_vel, grav, last_act, h_state, c_state
            )
            last_act = raw_act.copy()

            # Controller post-processing
            model_cmd = np.array([cmd[0], cmd[1] * LATERAL_SCALE, cmd[2]])
            # Filter
            filt_cmd = LPF_COEF * model_cmd + (1.0 - LPF_COEF) * prev_cmd
            prev_cmd = filt_cmd.copy()

            # Simulated body velocity feedback
            lin_vel = [float(filt_cmd[0] * 0.5), float(filt_cmd[1] * 0.5), 0.0]
            ang_vel = [0.0, 0.0, float(filt_cmd[2] * 0.5)]

            history.append({
                "raw_cmd": cmd,
                "filt_cmd": filt_cmd,
                "vx": filt_cmd[0],
                "vy": filt_cmd[1],
                "wz": filt_cmd[2]
            })

        return history

    def simulate_2d_rollout(self, depth_emb, target_generator_fn, dt=0.2, steps=15):
        """Simulate closed-loop 2D path integration over a time horizon."""
        h_state = np.zeros((1, 1, 512), dtype=np.float32)
        c_state = np.zeros((1, 1, 512), dtype=np.float32)
        last_act = np.zeros(3, dtype=np.float32)
        lin_vel = [0.0, 0.0, 0.0]
        ang_vel = [0.0, 0.0, 0.0]
        grav = [0.0, 0.0, -1.0]
        prev_cmd = np.zeros(3, dtype=np.float32)

        x, y, theta = 0.0, 0.0, 0.0
        trajectory = [(0.0, 0.0, 0.0)]

        for step in range(steps):
            # Target relative to robot's current local frame
            target_vec_b = target_generator_fn(x, y, theta)

            cmd, raw_act, h_state, c_state = self.run_policy_step(
                depth_emb, target_vec_b, lin_vel, ang_vel, grav, last_act, h_state, c_state
            )
            last_act = raw_act.copy()

            model_cmd = np.array([cmd[0], cmd[1] * LATERAL_SCALE, cmd[2]])
            if model_cmd[0] < 0.0:
                model_cmd[0] = 0.0  # Stair anti-reverse guard
            model_cmd[2] = float(np.clip(model_cmd[2], -0.5, 0.5))  # Stair wz clamp

            filt_cmd = LPF_COEF * model_cmd + (1.0 - LPF_COEF) * prev_cmd
            prev_cmd = filt_cmd.copy()

            vx = filt_cmd[0]
            vy = filt_cmd[1]
            wz = filt_cmd[2]

            # Integrate robot motion
            dx_world = (vx * math.cos(theta) - vy * math.sin(theta)) * dt
            dy_world = (vx * math.sin(theta) + vy * math.cos(theta)) * dt
            dtheta = wz * dt

            x += dx_world
            y += dy_world
            theta += dtheta
            trajectory.append((x, y, theta))

            lin_vel = [float(vx * 0.5), float(vy * 0.5), 0.0]
            ang_vel = [0.0, 0.0, float(wz * 0.5)]

        return trajectory


def assess_viability(vx: float, wz: float) -> str:
    """Assess whether the output command will successfully descend stairs."""
    if vx >= 0.35 and abs(wz) <= 0.30:
        return "EXCELLENT (Goes Down Smoothly)"
    elif vx >= 0.25 and abs(wz) <= 0.50:
        return "GOOD (Descends with slight turn)"
    elif vx >= 0.15 and abs(wz) <= 0.60:
        return "MARGINAL (Slow / drifting)"
    elif abs(wz) >= 0.70 or vx < 0.10:
        return "FAILED (Spins / Refuses descent)"
    else:
        return "POOR"


def main():
    print("=" * 80)
    print("       Ablation Study: Stairs Descent Lookahead & Raw vs Remapped Depth       ")
    print("       Diagnostic Frame: session_20261103_184007_stairs_test4 / Frame 185      ")
    print("=" * 80)

    tester = StairsAblationTester()

    # Load Frame 185 data
    meta_path = os.path.join(SESSION_DIR, "meta", "frame_000185_meta.json")
    raw_depth_path = os.path.join(SESSION_DIR, "depth", "frame_000185_depth_40x64.npy")
    remap_depth_path = os.path.join(SESSION_DIR, "depth_remapped", "frame_000185_remapped_40x64.npy")

    with open(meta_path, "r") as f:
        meta = json.load(f)

    raw_depth = np.load(raw_depth_path)
    remap_depth = np.load(remap_depth_path)

    raw_emb = tester.encode_depth(raw_depth)
    remap_emb = tester.encode_depth(remap_depth)

    slope_dz_dx = -0.3110 / 0.4218  # -0.7373 from frame 185 geometry
    heading_drift_dy_dx = 0.0410 / 0.4218  # 0.0972

    print(f"\n[Frame 185 Ground Truth]")
    print(f"  Robot World Pos: {meta['robot_pos']}")
    print(f"  Robot Yaw: {meta['robot_attitude']['yaw_deg']:.1f}°")
    print(f"  Carrot World Pos: {meta['carrot_pos']}")
    print(f"  Recorded target_3d: dx={meta['target_3d']['dx_b']:.3f}m, dy={meta['target_3d']['dy_b']:.3f}m, dz={meta['target_3d']['dz_b']:.3f}m")
    print(f"  Recorded cmd_vel:   vx={meta['cmd_vel'][0]:.3f}m/s, vy={meta['cmd_vel'][1]:.3f}m/s, wz={meta['cmd_vel'][2]:.3f}rad/s")
    print(f"  Depth remapped active: {meta['stairs_diagnostics']['remapped_active']}")

    lookahead_list = [0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.42, 0.45, 0.50, 0.60, 0.70, 0.80, 1.00, 1.20, 1.50, 2.00]

    # --- EXPERIMENT 1: Full Sweep with 3D Stairs Slope (dz = slope * L) ---
    print("\n" + "=" * 80)
    print("EXPERIMENT 1: Lookahead Sweep under 3D Stairs Geometry (dz = -0.737 * L)")
    print("=" * 80)
    print(f"{'L (m)':<7} | {'RAW vx':<7} {'RAW vy':<7} {'RAW wz':<7} {'RAW Status':<22} | {'REMAP vx':<8} {'REMAP vy':<8} {'REMAP wz':<8} {'REMAP Status':<22}")
    print("-" * 105)

    exp1_raw_results = []
    exp1_remap_results = []

    for L in lookahead_list:
        dx = L
        dy = heading_drift_dy_dx * L
        dz = slope_dz_dx * L
        target_vec = np.array([dx, dy, dz], dtype=np.float32)

        # Evaluate RAW depth
        hist_raw = tester.multi_step_eval(raw_emb, target_vec, total_steps=10)
        vx_r = hist_raw[-1]["vx"]
        vy_r = hist_raw[-1]["vy"]
        wz_r = hist_raw[-1]["wz"]
        status_r = assess_viability(vx_r, wz_r)
        exp1_raw_results.append((L, vx_r, vy_r, wz_r, status_r))

        # Evaluate REMAPPED depth
        hist_m = tester.multi_step_eval(remap_emb, target_vec, total_steps=10)
        vx_m = hist_m[-1]["vx"]
        vy_m = hist_m[-1]["vy"]
        wz_m = hist_m[-1]["wz"]
        status_m = assess_viability(vx_m, wz_m)
        exp1_remap_results.append((L, vx_m, vy_m, wz_m, status_m))

        print(f"{L:<7.2f} | {vx_r:<+7.3f} {vy_r:<+7.3f} {wz_r:<+7.3f} {status_r:<22} | {vx_m:<+8.3f} {vy_m:<+8.3f} {wz_m:<+8.3f} {status_m:<22}")

    # --- EXPERIMENT 2: Step 1 (Immediate Cold-Start Reaction) Comparison ---
    print("\n" + "=" * 80)
    print("EXPERIMENT 2: Immediate Step 1 Reaction (Cold-Start Impulse)")
    print("=" * 80)
    print(f"{'L (m)':<7} | {'RAW Step 1 (vx, wz)':<24} {'RAW Tendency':<18} | {'REMAP Step 1 (vx, wz)':<24} {'REMAP Tendency':<18}")
    print("-" * 100)

    for L in [0.10, 0.20, 0.30, 0.40, 0.50, 0.80, 1.00, 1.50, 2.00]:
        dx = L
        dy = heading_drift_dy_dx * L
        dz = slope_dz_dx * L
        target_vec = np.array([dx, dy, dz], dtype=np.float32)

        hist_raw = tester.multi_step_eval(raw_emb, target_vec, total_steps=1)
        hist_m = tester.multi_step_eval(remap_emb, target_vec, total_steps=1)

        v1_r = hist_raw[0]["vx"]
        w1_r = hist_raw[0]["wz"]
        v1_m = hist_m[0]["vx"]
        w1_m = hist_m[0]["wz"]

        tend_r = "Forward Surge" if v1_r > 0.8 else ("Forward Drive" if v1_r > 0.2 else ("Reverse/Brake" if v1_r < 0 else "Spinning"))
        tend_m = "Forward Surge" if v1_m > 0.8 else ("Forward Drive" if v1_m > 0.2 else ("Reverse/Brake" if v1_m < 0 else "Spinning"))

        print(f"{L:<7.2f} | vx={v1_r:<+6.3f}, wz={w1_r:<+6.3f}   {tend_r:<18} | vx={v1_m:<+6.3f}, wz={w1_m:<+6.3f}   {tend_m:<18}")

    # --- EXPERIMENT 3: Closed-Loop Dynamic Rollout over 3 Seconds (15 steps) ---
    print("\n" + "=" * 80)
    print("EXPERIMENT 3: Dynamic Closed-Loop Simulation (3.0s Horizon / 15 steps @ 5Hz)")
    print("=" * 80)

    def make_target_gen(lookahead_dist):
        def target_gen(x_rob, y_rob, theta_rob):
            # Target on stair path: path runs along world X direction
            # Robot starts at (0,0,0) facing along X.
            # Lookahead point is at distance L along path.
            # Since the path is straight down stairs: target is at (x_rob + L, 0)
            dx_w = (x_rob + lookahead_dist) - x_rob
            dy_w = 0.0 - y_rob
            # Rotate into body frame
            dx_b = dx_w * math.cos(-theta_rob) - dy_w * math.sin(-theta_rob)
            dy_b = dx_w * math.sin(-theta_rob) + dy_w * math.cos(-theta_rob)
            dz_b = slope_dz_dx * lookahead_dist
            return np.array([dx_b, dy_b, dz_b], dtype=np.float32)
        return target_gen

    print(f"{'Condition':<25} | {'Lookahead':<10} | {'Net Forward (m)':<15} | {'Lateral Drift (m)':<18} | {'Heading Dev (deg)':<18} | {'Result':<15}")
    print("-" * 110)

    rollout_configs = [
        ("Raw Depth", raw_emb, 0.10),
        ("Raw Depth", raw_emb, 0.20),
        ("Raw Depth", raw_emb, 0.30),
        ("Raw Depth", raw_emb, 0.40),
        ("Raw Depth", raw_emb, 0.50),
        ("Raw Depth", raw_emb, 1.00),
        ("Raw Depth", raw_emb, 2.00),
        ("Remapped Depth", remap_emb, 0.40),
        ("Remapped Depth", remap_emb, 0.50),
        ("Remapped Depth", remap_emb, 2.00),
    ]

    for label, emb, L in rollout_configs:
        traj = tester.simulate_2d_rollout(emb, make_target_gen(L), dt=0.2, steps=15)
        final_x, final_y, final_th = traj[-1]
        deg = math.degrees(final_th)
        if final_x > 0.8 and abs(deg) < 25.0:
            res = "DESCENDS CLEANLY"
        elif final_x > 0.5 and abs(deg) < 45.0:
            res = "DESCENDS"
        elif final_x < 0.3 or abs(deg) > 60.0:
            res = "FAILS / TURNS AWAY"
        else:
            res = "MARGINAL"
        print(f"{label:<25} | L={L:<7.2f}m | x = {final_x:<+10.3f}m | y = {final_y:<+13.3f}m | yaw = {deg:<+11.1f}° | {res:<15}")

    # --- EXPERIMENT 4: Test across multiple consecutive stair frames (182, 185, 195, 210) ---
    print("\n" + "=" * 80)
    print("EXPERIMENT 4: Verification Across Multiple Stair Descent Frames (182, 185, 195, 210)")
    print("=" * 80)
    test_frames = [182, 185, 195, 210]
    print(f"{'Frame':<6} {'Phase':<20} | {'L=0.2m (RAW)':<14} {'L=0.4m (RAW)':<14} {'L=0.5m (RAW)':<14} | {'L=0.4m (REMAP)':<14} {'L=2.0m (REMAP)':<14}")
    print("-" * 105)

    phase_names = {
        182: "Stair Top Inception",
        185: "First Step Descent",
        195: "Mid-Stairway Step 3",
        210: "Continuous Flight"
    }

    for f_idx in test_frames:
        d_raw_path = os.path.join(SESSION_DIR, "depth", f"frame_{f_idx:06d}_depth_40x64.npy")
        d_map_path = os.path.join(SESSION_DIR, "depth_remapped", f"frame_{f_idx:06d}_remapped_40x64.npy")
        if not os.path.exists(d_raw_path):
            continue
        e_raw = tester.encode_depth(np.load(d_raw_path))
        e_map = tester.encode_depth(np.load(d_map_path))

        def get_v_w(emb, L):
            tgt = np.array([L, 0.0, slope_dz_dx * L], dtype=np.float32)
            h = tester.multi_step_eval(emb, tgt, total_steps=10)
            return f"{h[-1]['vx']:+.2f}/{h[-1]['wz']:+.2f}"

        raw_02 = get_v_w(e_raw, 0.20)
        raw_04 = get_v_w(e_raw, 0.40)
        raw_05 = get_v_w(e_raw, 0.50)
        map_04 = get_v_w(e_map, 0.40)
        map_20 = get_v_w(e_map, 2.00)

        phase = phase_names.get(f_idx, "Stairs")
        print(f"{f_idx:<6} {phase:<20} | {raw_02:<14} {raw_04:<14} {raw_05:<14} | {map_04:<14} {map_20:<14}")

    print("\n" + "=" * 80)
    print("Ablation Study Completed.")
    print("=" * 80)


if __name__ == "__main__":
    main()

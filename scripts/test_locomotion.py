#!/usr/bin/env python3
# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: MIT

"""Go2W Locomotion Stair Climbing Test Bench (rl_sar environment and control design).

Key Features:
1. Dedicated Straight Flight of Stairs (modelled after rl_sar/worlds/stairs.world):
   - 8-meter flat runway before stairs (X in [-8.0, 0.0]m, Y in [-5.0, +5.0]m).
   - Ascending stairs starting at X = 0.0m (tread depth=0.28m, riser height=0.15m, 8 steps).
   - Flat top platform on top of stairs.
   - Guaranteed fixed spawn at X = -2.0m, Y = 0.0m, facing straight into the stairs (+X).
2. Elimination of Navigation Timeouts and Random Jumps:
   - Episode timeouts and early terminations removed.
   - Resets are strictly manual (pressing 'R') or on demand.
3. rl_sar-style Keyboard Teleoperation (modelled after rl_sar rl_sdk.cpp):
   - W / Up Arrow   : Vx += 0.1 m/s
   - S / Down Arrow : Vx -= 0.1 m/s
   - A / Left Arrow : Vy += 0.1 m/s
   - D / Right Arrow: Vy -= 0.1 m/s
   - Q              : Yaw += 0.2 rad/s
   - E              : Yaw -= 0.2 rad/s
   - Space / X      : Stop (0, 0, 0)
   - R              : Reset robot pose in front of stairs
   - Dual-input support: Works seamlessly from BOTH Linux terminal and Omniverse GUI window!
4. Direct Policy Feeding:
   - Raw velocity setpoints are passed 1:1 to the low-level locomotion policy observation buffer without tanh compression.
5. In-Place Terminal HUD:
   - Clean 1-line updating status line without terminal flooding.

Usage:
    # 1. Interactive GUI test (spawns 2m before stairs, user controls with W/S/A/D/Space/R):
    ./isaaclab.sh -p source/isaaclab_nav_task/scripts/test_locomotion.py --gui

    # 2. Test different step heights (e.g., 0.12m or 0.10m):
    ./isaaclab.sh -p source/isaaclab_nav_task/scripts/test_locomotion.py --gui --step-height 0.12

    # 3. Test on flat ground:
    ./isaaclab.sh -p source/isaaclab_nav_task/scripts/test_locomotion.py --gui --terrain-type flat

    # 4. Headless video recording:
    ./isaaclab.sh -p source/isaaclab_nav_task/scripts/test_locomotion.py --video --video_length 400 --vx 0.5
"""

from __future__ import annotations

import argparse
import math
import os
import select
import sys
import threading
import time

# Handle '--headless False' or '--headless=False' gracefully before argparse parses it
for i, arg in enumerate(sys.argv):
    if arg in ["--headless=False", "--headless=0"]:
        sys.argv[i] = "--gui"
    elif arg == "--headless" and i + 1 < len(sys.argv) and sys.argv[i + 1].lower() in ["false", "0"]:
        sys.argv[i] = "--gui"
        sys.argv.pop(i + 1)
        break

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Go2W Locomotion Test Bench (rl_sar style).")
parser.add_argument("--task", type=str, default="Isaac-Nav-PPO-Go2W-Play-v0", help="Task name.")
parser.add_argument("--gui", action="store_true", default=False, help="Launch with graphical 3D viewport window.")
parser.add_argument("--terrain-type", type=str, default="stairs", choices=["stairs", "flat"], help="Terrain type: 'stairs' or 'flat'.")
parser.add_argument("--step-height", type=float, default=0.15, help="Step riser height in meters (default: 0.15m from rl_sar).")
parser.add_argument("--step-width", type=float, default=0.28, help="Step tread depth in meters (default: 0.28m from rl_sar).")
parser.add_argument("--num-steps", type=int, default=8, help="Number of stairs steps (default: 8).")
parser.add_argument("--stair-start-x", type=float, default=0.0, help="World X coordinate where stairs begin (default: 0.0m).")
parser.add_argument("--spawn-dist", type=float, default=2.0, help="Distance before stairs where robot spawns (default: 2.0m).")
parser.add_argument("--vx", type=float, default=0.0, help="Initial forward speed (m/s). Default: 0.0 (stopped).")
parser.add_argument("--vy", type=float, default=0.0, help="Initial lateral speed (m/s). Default: 0.0.")
parser.add_argument("--omega", type=float, default=0.0, help="Initial turn rate (rad/s). Default: 0.0.")
parser.add_argument("--policy", type=str, default="dreamwaq", choices=["dreamwaq", "rl_sar"], help="Locomotion policy: 'dreamwaq' or 'rl_sar'. Default: dreamwaq.")
parser.add_argument("--max-vx", type=float, default=1.5, help="Full speed forward (m/s). Default: 1.5 m/s.")
parser.add_argument("--max-vx-rev", type=float, default=0.8, help="Full speed reverse (m/s). Default: 0.8 m/s.")
parser.add_argument("--max-vy", type=float, default=0.6, help="Full speed lateral strafe (m/s). Default: 0.6 m/s.")
parser.add_argument("--max-yaw", type=float, default=1.0, help="Full speed turn rate (rad/s). Default: 1.0 rad/s.")
parser.add_argument("--incremental", action="store_true", default=False, help="Use incremental speed stepping (+/-) instead of direct full speed.")
parser.add_argument("--video", action="store_true", default=False, help="Record video of the run.")
parser.add_argument("--video_length", type=int, default=400, help="Video length in steps.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments.")
parser.add_argument("--max_steps", type=int, default=1000000, help="Maximum steps to run.")

AppLauncher.add_app_launcher_args(parser)
args_cli, hydra_args = parser.parse_known_args()

if args_cli.gui:
    args_cli.headless = False
    os.environ["HEADLESS"] = "0"

if args_cli.video:
    args_cli.enable_cameras = True

# Launch Omniverse Simulation App
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym
import numpy as np
import torch

import isaaclab_tasks  # noqa: F401
import isaaclab_nav_task  # noqa: F401

from isaaclab.terrains.height_field.utils import height_field_to_mesh
from isaaclab.terrains.height_field.hf_terrains_cfg import HfTerrainBaseCfg
from isaaclab.utils import configclass
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper


# =============================================================================
# Custom Straight Stairs Heightfield Terrain (rl_sar stairs.world model)
# =============================================================================

@height_field_to_mesh
def straight_stairs_terrain(difficulty: float, cfg: HfStraightStairsTerrainCfg) -> np.ndarray:
    """Generate a clean straight staircase terrain.

    Tile is centered at (0, 0):
      X ranges from -size[0]/2 to +size[0]/2.
      Y ranges from -size[1]/2 to +size[1]/2.
    - Flat runway: from -size[0]/2 to stair_start_x (flat ground at z=0).
    - Ascending steps: num_steps steps ascending forward along +X.
    - Top platform: from top of stairs to +size[0]/2 (flat at max step height).
    """
    width_pixels = int(cfg.size[0] / cfg.horizontal_scale)
    length_pixels = int(cfg.size[1] / cfg.horizontal_scale)
    hf = np.zeros((width_pixels, length_pixels), dtype=np.int16)

    if cfg.num_steps <= 0:
        return hf  # Completely flat terrain

    half_w = cfg.size[0] / 2.0
    start_pixel = int((cfg.stair_start_x + half_w) / cfg.horizontal_scale)
    step_width_pixels = max(1, int(round(cfg.step_width / cfg.horizontal_scale)))

    cur_px = max(0, start_pixel)
    current_h = 0
    for i in range(1, cfg.num_steps + 1):
        next_px = min(width_pixels, cur_px + step_width_pixels)
        current_h = int(round(i * cfg.step_height / cfg.vertical_scale))
        hf[cur_px:next_px, :] = current_h
        cur_px = next_px

    if cur_px < width_pixels:
        hf[cur_px:, :] = current_h

    return hf


@configclass
class HfStraightStairsTerrainCfg(HfTerrainBaseCfg):
    """Configuration for straight flight of stairs terrain."""
    function = straight_stairs_terrain
    stair_start_x: float = 0.0
    step_width: float = 0.28
    step_height: float = 0.15
    num_steps: int = 8


# =============================================================================
# rl_sar Keyboard Controller
# =============================================================================

class RLSarKeyboardController:
    """rl_sar-style keyboard controller supporting Terminal and Omniverse GUI."""

    def __init__(
        self,
        default_vx: float = 0.0,
        default_vy: float = 0.0,
        default_omega: float = 0.0,
        max_vx: float = 1.0,
        max_vx_rev: float = 0.8,
        max_vy: float = 0.6,
        max_yaw: float = 1.0,
        incremental: bool = False,
    ):
        self.cmd_vx = float(default_vx)
        self.cmd_vy = float(default_vy)
        self.cmd_yaw = float(default_omega)
        self.target_vx = float(default_vx)
        self.target_vy = float(default_vy)
        self.target_yaw = float(default_omega)
        self.max_vx = float(max_vx)
        self.max_vx_rev = float(max_vx_rev)
        self.max_vy = float(max_vy)
        self.max_yaw = float(max_yaw)
        self.incremental = incremental
        self.reset_requested = False
        self.running = True
        self.lock = threading.Lock()
        self._pressed_keys = set()
        self._key_timestamps = {}
        self._last_time = time.time()
        self.accel_limit = 2.5   # m/s^2 forward acceleration
        self.decel_limit = 3.5   # m/s^2 braking deceleration
        self.yaw_rate_limit = 3.0 # rad/s^2 yaw acceleration

        # Terminal raw mode setup
        self.is_terminal = sys.stdin.isatty()
        self.orig_term_settings = None
        if self.is_terminal:
            try:
                import termios
                import tty
                self.orig_term_settings = termios.tcgetattr(sys.stdin)
                tty.setcbreak(sys.stdin.fileno())
                self.term_thread = threading.Thread(target=self._terminal_read_loop, daemon=True)
                self.term_thread.start()
            except Exception as e:
                print(f"[INFO] Terminal stdin listener disabled: {e}")

        # Omniverse GUI Keyboard subscription
        self._carb_input = None
        self._appwindow = None
        self._gui_sub = None
        try:
            import carb.input
            import omni.appwindow
            self._appwindow = omni.appwindow.get_default_app_window()
            self._carb_input = carb.input.acquire_input_interface()
            if self._appwindow is not None and self._carb_input is not None:
                keyboard = self._appwindow.get_keyboard()
                self._gui_sub = self._carb_input.subscribe_to_keyboard_events(
                    keyboard, self._on_gui_keyboard_event
                )
        except Exception:
            pass

    def _terminal_read_loop(self):
        """Dedicated thread reading keystrokes from stdin."""
        while self.running:
            try:
                r, _, _ = select.select([sys.stdin], [], [], 0.05)
                if not r:
                    continue
                ch = sys.stdin.read(1)
                act = None
                if ch == '\x1b':
                    r2, _, _ = select.select([sys.stdin], [], [], 0.05)
                    if r2:
                        seq1 = sys.stdin.read(1)
                        if seq1 == '[':
                            r3, _, _ = select.select([sys.stdin], [], [], 0.05)
                            if r3:
                                seq2 = sys.stdin.read(1)
                                if seq2 == 'A': act = 'w'
                                elif seq2 == 'B': act = 's'
                                elif seq2 == 'C': act = 'd'
                                elif seq2 == 'D': act = 'a'
                else:
                    k = ch.lower()
                    if k in ('w', 'up'): act = 'w'
                    elif k in ('s', 'down'): act = 's'
                    elif k in ('a', 'left'): act = 'a'
                    elif k in ('d', 'right'): act = 'd'
                    elif k in ('q', 'z'): act = 'q'
                    elif k in ('e', 'c'): act = 'e'
                    elif k in (' ', 'x'): act = 'space'
                    elif k == 'r': act = 'r'

                if act:
                    with self.lock:
                        if act == 'space':
                            self._pressed_keys.clear()
                            self._key_timestamps.clear()
                            self.cmd_vx = 0.0
                            self.cmd_vy = 0.0
                            self.cmd_yaw = 0.0
                        elif act == 'r':
                            self.reset_requested = True
                        else:
                            if self.incremental:
                                self._handle_incremental_action(act)
                            else:
                                self._pressed_keys.add(act)
                                self._key_timestamps[act] = time.time()
                                self._recompute_cmd_locked()
            except Exception:
                break

    def _on_gui_keyboard_event(self, event):
        """Callback for Omniverse 3D window keyboard events."""
        try:
            import carb.input
            name = event.input.name.lower()
            action_map = {
                'w': 'w', 'up': 'w',
                's': 's', 'down': 's',
                'a': 'a', 'left': 'a',
                'd': 'd', 'right': 'd',
                'q': 'q', 'z': 'q',
                'e': 'e', 'c': 'e',
                'r': 'r',
                'space': 'space', 'x': 'space'
            }
            act = action_map.get(name)
            if not act:
                return

            with self.lock:
                if event.type in (carb.input.KeyboardEventType.KEY_PRESS, carb.input.KeyboardEventType.KEY_REPEAT):
                    if act == 'space':
                        self._pressed_keys.clear()
                        self._key_timestamps.clear()
                        self.cmd_vx = 0.0
                        self.cmd_vy = 0.0
                        self.cmd_yaw = 0.0
                    elif act == 'r':
                        self.reset_requested = True
                    else:
                        if self.incremental:
                            self._handle_incremental_action(act)
                        else:
                            self._pressed_keys.add(act)
                            self._recompute_cmd_locked()
                elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
                    if not self.incremental:
                        self._pressed_keys.discard(act)
                        self._key_timestamps.pop(act, None)
                        self._recompute_cmd_locked()
        except Exception:
            pass

    def _handle_incremental_action(self, action: str):
        if action == 'w':
            self.cmd_vx = round(min(self.max_vx, self.cmd_vx + 0.1), 2)
            self.target_vx = self.cmd_vx
        elif action == 's':
            self.cmd_vx = round(max(-self.max_vx_rev, self.cmd_vx - 0.1), 2)
            self.target_vx = self.cmd_vx
        elif action == 'a':
            self.cmd_vy = round(min(self.max_vy, self.cmd_vy + 0.1), 2)
            self.target_vy = self.cmd_vy
        elif action == 'd':
            self.cmd_vy = round(max(-self.max_vy, self.cmd_vy - 0.1), 2)
            self.target_vy = self.cmd_vy
        elif action == 'q':
            self.cmd_yaw = round(min(self.max_yaw, self.cmd_yaw + 0.2), 2)
            self.target_yaw = self.cmd_yaw
        elif action == 'e':
            self.cmd_yaw = round(max(-self.max_yaw, self.cmd_yaw - 0.2), 2)
            self.target_yaw = self.cmd_yaw

    def _recompute_cmd_locked(self):
        """Recompute target velocities from active held keys (Press=Full Speed, Release=Stop)."""
        if self.incremental:
            return

        vx = 0.0
        if 'w' in self._pressed_keys:
            vx += self.max_vx
        if 's' in self._pressed_keys:
            vx -= self.max_vx_rev

        vy = 0.0
        if 'a' in self._pressed_keys:
            vy += self.max_vy
        if 'd' in self._pressed_keys:
            vy -= self.max_vy

        yaw = 0.0
        if 'q' in self._pressed_keys:
            yaw += self.max_yaw
        if 'e' in self._pressed_keys:
            yaw -= self.max_yaw

        self.target_vx = round(vx, 2)
        self.target_vy = round(vy, 2)
        self.target_yaw = round(yaw, 2)

    def get_cmd(self) -> list[float]:
        with self.lock:
            now = time.time()
            dt = min(max(now - self._last_time, 0.001), 0.1)
            self._last_time = now

            # Auto-decay terminal keystrokes when key is released (no repeat within 0.25s)
            if self.is_terminal and not self.incremental and self._key_timestamps:
                expired = [k for k, t in self._key_timestamps.items() if now - t > 0.25]
                for k in expired:
                    self._pressed_keys.discard(k)
                    del self._key_timestamps[k]
                if expired:
                    self._recompute_cmd_locked()

            if not self.incremental:
                # Smooth rate limiting for linear X (prevents front wheel lift on accel and rear wheel lift on decel)
                if (self.cmd_vx >= 0 and self.target_vx >= self.cmd_vx) or (self.cmd_vx <= 0 and self.target_vx <= self.cmd_vx):
                    max_dv = self.accel_limit * dt
                else:
                    max_dv = self.decel_limit * dt

                diff_x = self.target_vx - self.cmd_vx
                if abs(diff_x) > max_dv:
                    self.cmd_vx += math.copysign(max_dv, diff_x)
                else:
                    self.cmd_vx = self.target_vx

                diff_y = self.target_vy - self.cmd_vy
                max_dv_y = self.accel_limit * dt
                if abs(diff_y) > max_dv_y:
                    self.cmd_vy += math.copysign(max_dv_y, diff_y)
                else:
                    self.cmd_vy = self.target_vy

                diff_yaw = self.target_yaw - self.cmd_yaw
                max_dyaw = self.yaw_rate_limit * dt
                if abs(diff_yaw) > max_dyaw:
                    self.cmd_yaw += math.copysign(max_dyaw, diff_yaw)
                else:
                    self.cmd_yaw = self.target_yaw

            return [round(self.cmd_vx, 3), round(self.cmd_vy, 3), round(self.cmd_yaw, 3)]

    def stop(self):
        with self.lock:
            self._pressed_keys.clear()
            self._key_timestamps.clear()
            self.target_vx = 0.0
            self.target_vy = 0.0
            self.target_yaw = 0.0
            self.cmd_vx = 0.0
            self.cmd_vy = 0.0
            self.cmd_yaw = 0.0

    def close(self):
        self.running = False
        if self._gui_sub is not None and self._carb_input is not None and self._appwindow is not None:
            try:
                self._carb_input.unsubscribe_to_keyboard_events(self._appwindow.get_keyboard(), self._gui_sub)
            except Exception:
                pass
        if self.orig_term_settings is not None:
            try:
                import termios
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.orig_term_settings)
            except Exception:
                pass


def quat_to_euler_angles(q: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    """Convert quaternion (w, x, y, z) to roll, pitch, yaw in degrees."""
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = torch.atan2(sinr_cosp, cosr_cosp) * (180.0 / math.pi)

    sinp = 2.0 * (w * y - z * x)
    sinp = torch.clamp(sinp, -1.0, 1.0)
    pitch = torch.asin(sinp) * (180.0 / math.pi)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = torch.atan2(siny_cosp, cosy_cosp) * (180.0 / math.pi)

    return roll, pitch, yaw


# =============================================================================
# Main Simulation Routine
# =============================================================================

def main():
    spawn_x = args_cli.stair_start_x - args_cli.spawn_dist
    spawn_y = 0.0
    spawn_yaw = 0.0  # Facing East (+X), directly into the stairs

    print(f"\n{'='*75}")
    print(f"       Go2W Locomotion Test Bench (rl_sar Design)")
    print(f"{'='*75}")
    print(f" Terrain Type          : {args_cli.terrain_type.upper()}")
    if args_cli.terrain_type == "stairs":
        print(f" Stair Riser Height    : {args_cli.step_height:.2f} m ({args_cli.step_height*100:.0f} cm)")
        print(f" Stair Tread Depth     : {args_cli.step_width:.2f} m ({args_cli.step_width*100:.0f} cm)")
        print(f" Number of Steps       : {args_cli.num_steps}")
        print(f" Total Staircase Rise  : {args_cli.num_steps * args_cli.step_height:.2f} m")
    print(f" Staircase Start (X)   : {args_cli.stair_start_x:+.2f} m")
    print(f" Robot Spawn Pose      : X={spawn_x:+.2f} m, Y={spawn_y:+.2f} m (Faces +X directly into stairs)")
    print(f" Distance to Steps     : {args_cli.spawn_dist:.2f} m (on flat runway)")
    print(f" GUI Viewport Window   : {'ENABLED' if not args_cli.headless else 'DISABLED (Headless)'}")
    print(f"{'='*75}\n")

    # Load task spec
    spec = gym.spec(args_cli.task)
    env_cfg = spec.kwargs.get("env_cfg_entry_point")()

    # Configure locomotion policy (DreamWaQ or rl_sar)
    from isaaclab_nav_task.navigation import ISAACLAB_NAV_TASKS_ASSETS_DIR
    if args_cli.policy == "dreamwaq":
        policy_file = os.path.join(
            ISAACLAB_NAV_TASKS_ASSETS_DIR, "Policies", "locomotion", "go2w", "policy_go2w_dreamwaq_adapted.pt"
        )
        env_cfg.actions.velocity_command.low_level_policy_file = policy_file
        # Match DreamWaQ actuator stiffness & damping (kp=50.0, kd=1.0 per g2w.yaml)
        if hasattr(env_cfg.scene.robot, "actuators"):
            if "legs" in env_cfg.scene.robot.actuators:
                env_cfg.scene.robot.actuators["legs"].stiffness = 50.0
                env_cfg.scene.robot.actuators["legs"].damping = 1.0
            if "legs_calf" in env_cfg.scene.robot.actuators:
                env_cfg.scene.robot.actuators["legs_calf"].stiffness = 50.0
                env_cfg.scene.robot.actuators["legs_calf"].damping = 1.0
        # Set symmetric posture and standing height matching DreamWaQ g2w.yaml (thigh=0.67, calf=-1.30)
        env_cfg.scene.robot.init_state.pos = (0.0, 0.0, 0.40)
        env_cfg.scene.robot.init_state.joint_pos = {
            ".*hip_joint": 0.0,
            ".*thigh_joint": 0.67,
            ".*calf_joint": -1.30,
            ".*foot_joint": 0.0,
        }
        spawn_standing_z = 0.40
    else:
        policy_file = os.path.join(
            ISAACLAB_NAV_TASKS_ASSETS_DIR, "Policies", "locomotion", "go2w", "policy_go2w_adapted.pt.bak_rl_sar"
        )
        env_cfg.actions.velocity_command.low_level_policy_file = policy_file
        spawn_standing_z = 0.38
    print(f" Locomotion Policy     : {args_cli.policy.upper()} ({os.path.basename(policy_file)})")

    # 1. CONFIGURE CLEAN rl_sar-STYLE TERRAIN
    env_cfg.scene.terrain.terrain_generator.sub_terrains = {
        "straight_stairs": HfStraightStairsTerrainCfg(
            function=straight_stairs_terrain,
            proportion=1.0,
            size=(40.0, 40.0) if args_cli.terrain_type == "flat" else (16.0, 10.0),
            horizontal_scale=0.1,
            vertical_scale=0.005,
            border_width=0.0,
            stair_start_x=args_cli.stair_start_x,
            step_width=args_cli.step_width,
            step_height=args_cli.step_height if args_cli.terrain_type == "stairs" else 0.0,
            num_steps=args_cli.num_steps if args_cli.terrain_type == "stairs" else 0,
        )
    }
    env_cfg.scene.terrain.terrain_generator.num_rows = 1
    env_cfg.scene.terrain.terrain_generator.num_cols = 1
    env_cfg.scene.terrain.terrain_generator.curriculum = False
    env_cfg.scene.terrain.max_init_terrain_level = 0
    env_cfg.scene.num_envs = args_cli.num_envs

    # 2. DISABLE ALL TIMEOUTS AND UNWANTED TERMINATIONS (NO PREMATURE RESETS)
    env_cfg.terminations.time_out = None
    env_cfg.terminations.early_termination = None
    env_cfg.terminations.base_contact = None
    env_cfg.terminations.terrain_fall = None
    env_cfg.terminations.large_pitch_angle = None
    env_cfg.episode_length_s = 1e6  # Infinite horizon

    # 3. DIRECT ACTION FEEDING (Match rl_sar: no tanh squashing, no low-pass lag)
    env_cfg.actions.velocity_command.enable_low_pass_filter = False
    env_cfg.actions.velocity_command.use_raw_actions = True

    # Disable random disturbances, events, and sensor delays for pure locomotion testing
    env_cfg.observations.policy.enable_corruption = False
    if hasattr(env_cfg.observations, "low_level_policy"):
        env_cfg.observations.low_level_policy.enable_corruption = False
    if hasattr(env_cfg, "events"):
        env_cfg.events.reset_base = None
        env_cfg.events.reset_robot_joints = None
        env_cfg.events.push_robot = None
        env_cfg.events.base_external_force_torque = None
        env_cfg.events.randomize_action_scale = None
        env_cfg.events.randomize_low_pass_filter_alpha = None
        env_cfg.events.randomize_camera_height = None
    if hasattr(env_cfg, "delay_cfg"):
        env_cfg.delay_cfg.enabled = False

    # Disable remote nucleus/S3 asset downloads and debug markers
    env_cfg.scene.terrain.visual_material = None
    if hasattr(env_cfg, "commands") and hasattr(env_cfg.commands, "robot_goal"):
        env_cfg.commands.robot_goal.debug_vis = False
        env_cfg.commands.robot_goal.robot_to_goal_line_vis = False

    # Monkey patch RobotNavigationGoalCommand before gym.make/env.reset so no maze masks are required
    from isaaclab_nav_task.navigation.mdp.navigation.goal_commands import RobotNavigationGoalCommand
    def fixed_resample_command(self, env_ids):
        terrain = self.env.scene.terrain
        if isinstance(env_ids, torch.Tensor):
            idx = env_ids
        else:
            idx = torch.as_tensor(env_ids, device=self.device, dtype=torch.long)
        terrain.env_origins[idx, 0] = spawn_x
        terrain.env_origins[idx, 1] = spawn_y
        terrain.env_origins[idx, 2] = 0.0
        self.goal_position_world[idx, 0] = args_cli.stair_start_x + args_cli.num_steps * args_cli.step_width + 1.0
        self.goal_position_world[idx, 1] = 0.0
        self.goal_position_world[idx, 2] = args_cli.num_steps * args_cli.step_height + 0.5
        self.spawn_position_world[idx, 0] = spawn_x
        self.spawn_position_world[idx, 1] = spawn_y
        self.spawn_position_world[idx, 2] = 0.0
    RobotNavigationGoalCommand._resample_command = fixed_resample_command

    # Create Gym Environment
    render_mode = "rgb_array" if args_cli.video else None
    env = gym.make(args_cli.task, cfg=env_cfg, render_mode=render_mode)

    if args_cli.video:
        video_dir = os.path.join("logs", "locomotion_test", "videos")
        os.makedirs(video_dir, exist_ok=True)
        print(f"[INFO] Recording video to: {video_dir}")
        env = gym.wrappers.RecordVideo(
            env,
            video_folder=video_dir,
            step_trigger=lambda s: s == 0,
            video_length=args_cli.video_length,
            disable_logger=True,
        )

    env = RslRlVecEnvWrapper(env)
    device = env.unwrapped.device
    num_envs = env.unwrapped.num_envs
    robot = env.unwrapped.scene["robot"]

    # Bypass action processing tanh to feed direct raw velocity commands into policy observations
    action_term = env.unwrapped.action_manager.get_term("velocity_command")
    def direct_process_actions(actions):
        action_term._raw_navigation_velocity_actions[:] = actions
        action_term._processed_navigation_velocity_actions[:] = actions
    action_term.process_actions = direct_process_actions

    # Position 3D viewport camera directly behind robot looking at the stairs
    try:
        cam_eye = [spawn_x - 3.0, spawn_y + 2.0, 1.8]
        cam_target = [spawn_x + 1.5, spawn_y, 0.4]
        env.unwrapped.sim.set_camera_view(eye=cam_eye, target=cam_target)
        print("[CAMERA] Viewport camera positioned behind Go2W focusing on stairs ahead.\n")
    except Exception:
        pass

    # Initialize Keyboard Controller
    controller = RLSarKeyboardController(
        default_vx=args_cli.vx,
        default_vy=args_cli.vy,
        default_omega=args_cli.omega,
        max_vx=args_cli.max_vx,
        max_vx_rev=args_cli.max_vx_rev,
        max_vy=args_cli.max_vy,
        max_yaw=args_cli.max_yaw,
        incremental=args_cli.incremental,
    )

    def reset_robot_pose():
        """Reset robot root state to fixed spawn pose in front of stairs."""
        root_state = robot.data.default_root_state.clone()
        for i in range(num_envs):
            root_state[i, 0] = spawn_x
            root_state[i, 1] = spawn_y
            root_state[i, 2] = spawn_standing_z  # Standing height (m)
            root_state[i, 3] = math.cos(spawn_yaw / 2.0)
            root_state[i, 4] = 0.0
            root_state[i, 5] = 0.0
            root_state[i, 6] = math.sin(spawn_yaw / 2.0)
            root_state[i, 7:] = 0.0  # Zero initial velocities
        robot.write_root_state_to_sim(root_state)
        # Reset joint positions and velocities to default
        joint_pos = robot.data.default_joint_pos.clone()
        joint_vel = robot.data.default_joint_vel.clone()
        robot.write_joint_state_to_sim(joint_pos, joint_vel)

    # Initial reset
    env.reset()
    reset_robot_pose()

    print("=" * 80)
    print(f"             Go2W Locomotion Teleop Controls [Policy: {args_cli.policy.upper()}]")
    print("=" * 80)
    if args_cli.incremental:
        print(f"  Mode                : Incremental Speed Stepping (+/-)")
        print(f"  [W] / [Up Arrow]    : Linear X Forward (+0.1 m/s, max {args_cli.max_vx:.2f})")
        print(f"  [S] / [Down Arrow]  : Linear X Reverse (-0.1 m/s, max -{args_cli.max_vx_rev:.2f})")
        print(f"  [A] / [Left Arrow]  : Linear Y Left    (+0.1 m/s, max {args_cli.max_vy:.2f})")
        print(f"  [D] / [Right Arrow] : Linear Y Right   (-0.1 m/s, max -{args_cli.max_vy:.2f})")
        print(f"  [Q]                 : Yaw Turn Left    (+0.2 rad/s, max {args_cli.max_yaw:.2f})")
        print(f"  [E]                 : Yaw Turn Right   (-0.2 rad/s, max -{args_cli.max_yaw:.2f})")
    else:
        print(f"  Mode                : Direct Full-Speed Keystroke (WASDQE)")
        print(f"  [W] / [Up Arrow]    : Direct Full Forward (+{args_cli.max_vx:.2f} m/s)")
        print(f"  [S] / [Down Arrow]  : Direct Full Reverse (-{args_cli.max_vx_rev:.2f} m/s)")
        print(f"  [A] / [Left Arrow]  : Direct Full Strafe Left (+{args_cli.max_vy:.2f} m/s)")
        print(f"  [D] / [Right Arrow] : Direct Full Strafe Right (-{args_cli.max_vy:.2f} m/s)")
        print(f"  [Q]                 : Direct Full Turn Left (+{args_cli.max_yaw:.2f} rad/s)")
        print(f"  [E]                 : Direct Full Turn Right (-{args_cli.max_yaw:.2f} rad/s)")
    print("  [Space] / [X]       : Emergency Stop   (Vx=0, Vy=0, Yaw=0)")
    print("  [R]                 : Reset Robot Pose in front of stairs")
    print("  [Ctrl+C]            : Exit Test")
    print("=" * 80 + "\n")

    step_count = 0
    traj_x = []
    traj_y = []
    traj_yaw = []
    cur_x = spawn_x
    cur_y = spawn_y
    yaw_deg = spawn_yaw
    act_vx = 0.0

    try:
        while simulation_app.is_running() and step_count < args_cli.max_steps:
            # Handle user requested reset (pressing 'R')
            if controller.reset_requested:
                controller.reset_requested = False
                controller.stop()
                reset_robot_pose()
                traj_x.clear()
                traj_y.clear()
                traj_yaw.clear()
                print(f"\n[RESET] Robot repositioned at X={spawn_x:.2f}m facing stairs! Commands set to 0.\n")

            # Get velocity command from rl_sar controller
            cmd = controller.get_cmd()
            actions = torch.tensor([cmd], dtype=torch.float32, device=device).repeat(num_envs, 1)

            # Step environment
            obs, rews, dones, infos = env.step(actions)
            step_count += 1

            # Telemetry of Environment 0
            pos_w = robot.data.root_pos_w[0]        # [x, y, z]
            quat_w = robot.data.root_quat_w[0:1]    # [w, x, y, z]
            vel_b = robot.data.root_lin_vel_b[0]    # [vx, vy, vz] in body frame

            roll, pitch, yaw = quat_to_euler_angles(quat_w)
            act_vx = vel_b[0].item()
            cur_x = pos_w[0].item()
            cur_y = pos_w[1].item()
            height_z = pos_w[2].item()
            pitch_deg = pitch[0].item()
            yaw_deg = yaw[0].item()
            dist_to_stairs = args_cli.stair_start_x - cur_x

            traj_x.append(cur_x)
            traj_y.append(cur_y)
            traj_yaw.append(yaw_deg)

            if cur_x < args_cli.stair_start_x:
                status = f"\033[96mRunway ({dist_to_stairs:.1f}m to steps)\033[0m"
            else:
                step_idx = int((cur_x - args_cli.stair_start_x) / args_cli.step_width) + 1
                if step_idx <= args_cli.num_steps:
                    status = f"\033[92mClimbing Step {step_idx}/{args_cli.num_steps}\033[0m"
                else:
                    status = "\033[95mTop Platform Reached!\033[0m"

            if abs(pitch_deg) > 40.0:
                status = "\033[91mFallen / Tilted (>40°) - Press [R] to Reset\033[0m"

            # Single-line in-place refresh HUD
            hud_line = (
                f"\r[{args_cli.policy.upper()} Teleop] Cmd:[Vx:{cmd[0]:+4.2f} Vy:{cmd[1]:+4.2f} Yaw:{cmd[2]:+4.2f}] | "
                f"Robot:[X:{cur_x:+5.2f} Y:{cur_y:+5.2f} Yaw:{yaw_deg:+5.1f}° Vx:{act_vx:+4.2f} Z:{height_z:4.2f}m Pitch:{pitch_deg:+5.1f}°] | {status}\033[K"
            )
            if sys.stdout.isatty():
                sys.stdout.write(hud_line)
                sys.stdout.flush()
            elif step_count % 10 == 0 or step_count == 1:
                clean_line = (
                    f"Step {step_count:4d} | Cmd:[Vx:{cmd[0]:+4.2f} Vy:{cmd[1]:+4.2f} Yaw:{cmd[2]:+4.2f}] | "
                    f"Robot:[X:{cur_x:+5.2f} Y:{cur_y:+5.2f} Yaw:{yaw_deg:+5.1f}° Vx:{act_vx:+4.2f} Z:{height_z:4.2f}m Pitch:{pitch_deg:+5.1f}°]"
                )
                print(clean_line)

        # Print Benchmark Summary Table
        delta_x = cur_x - spawn_x
        delta_y = cur_y - spawn_y
        max_abs_y = max((abs(y - spawn_y) for y in traj_y), default=abs(delta_y))
        duration_s = max(step_count * 0.02, 1e-3)
        avg_speed_x = delta_x / duration_s

        print("\n" + "=" * 80)
        print(f"            STRAIGHT-LINE LOCOMOTION BENCHMARK RESULTS")
        print("=" * 80)
        print(f"  Policy Name             : {args_cli.policy.upper()}")
        print(f"  Total Steps / Duration  : {step_count} steps ({duration_s:.2f} s)")
        print(f"  Command Setpoint        : Vx={cmd[0]:+.2f} m/s, Vy={cmd[1]:+.2f} m/s, Yaw={cmd[2]:+.2f} rad/s")
        print(f"  Spawn Pose              : X={spawn_x:+.3f} m, Y={spawn_y:+.3f} m, Yaw={spawn_yaw:+.2f}°")
        print(f"  Final Pose              : X={cur_x:+.3f} m, Y={cur_y:+.3f} m, Yaw={yaw_deg:+.2f}°")
        print(f"  Forward Distance (ΔX)   : {delta_x:+.3f} m")
        print(f"  Lateral Drift (ΔY)      : {delta_y:+.3f} m")
        print(f"  Max Lateral Error       : {max_abs_y:.3f} m")
        print(f"  Final Heading Error     : {yaw_deg:+.2f}°")
        print(f"  Average Forward Speed   : {avg_speed_x:.3f} m/s")
        print("=" * 80 + "\n")

    except KeyboardInterrupt:
        print("\n\n[INFO] Locomotion test interrupted by user.")
    finally:
        controller.close()
        print("\n[INFO] Locomotion test finished.")
        env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()

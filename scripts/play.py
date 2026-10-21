#!/usr/bin/env python3
# Copyright (c) 2022-2025, Fan Yang and Per Frivik, ETH Zurich.
# All rights reserved.
#
# SPDX-License-Identifier: MIT

"""Play a trained navigation policy (PPO/MDPO) with automatic checkpoint loading.

Usage:
    python scripts/play.py --task <task_name> [options]

Arguments:
    --task                   Task name (required, typically *-Play-v0 variant)
    --checkpoint             Path to model checkpoint (.pt file)
    --use_last_checkpoint    Use latest checkpoint from logs (default behavior)
    --num_envs              Number of parallel environments
    --video                 Enable video recording
    --video_length          Video length in steps (default: 200)

Examples:
    python scripts/play.py --task Isaac-Navigation-B2W-Play-v0
    python scripts/play.py --task Isaac-Navigation-B2W-Play-v0 --checkpoint path/to/model.pt
    python scripts/play.py --task Isaac-Navigation-B2W-Play-v0 --video --num_envs 16

Note: Automatically finds latest checkpoint if --checkpoint not specified.
"""

from __future__ import annotations

import argparse
import sys

from isaaclab.app import AppLauncher

# Add argparse arguments
parser = argparse.ArgumentParser(description="Play a trained navigation policy with RSL-RL.")
parser.add_argument("--video", action="store_true", default=False, help="Record videos during play.")
parser.add_argument("--video_length", type=int, default=200, help="Length of the recorded video (in steps).")
parser.add_argument("--num_envs", type=int, default=None, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
parser.add_argument("--checkpoint", type=str, default=None, help="Path to model checkpoint.")
parser.add_argument("--use_last_checkpoint", action="store_true", help="Use last checkpoint from logs.")
parser.add_argument("--export_jit", action="store_true", default=False, help="Export policy as JIT module.")
parser.add_argument("--export_onnx", action="store_true", default=False, help="Export policy as ONNX model.")
parser.add_argument("--num_episodes", type=int, default=None, help="Number of episodes to evaluate before exiting.")

# Append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
args_cli, hydra_args = parser.parse_known_args()

# Always enable cameras
args_cli.enable_cameras = True

# Launch simulation
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# Import after launching simulation
import gymnasium as gym
import os
import re
import torch

from rsl_rl.runners import OnPolicyRunner

# Import Isaac Lab extensions
import isaaclab_tasks  # noqa: F401
import isaaclab_nav_task  # noqa: F401

from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlVecEnvWrapper, export_policy_as_onnx


def find_latest_checkpoint(log_path: str, checkpoint_pattern: str = "model_.*.pt") -> str:
    """Find the latest checkpoint file in the log directory.

    Args:
        log_path: Base log directory path
        checkpoint_pattern: Regex pattern for checkpoint files

    Returns:
        Path to the latest checkpoint file
    """
    # Find all run directories
    if not os.path.exists(log_path):
        raise ValueError(f"Log path does not exist: {log_path}")

    run_dirs = []
    for entry in os.scandir(log_path):
        if entry.is_dir() and re.match(r"\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2}", entry.name):
            run_dirs.append(entry.name)

    if not run_dirs:
        raise ValueError(f"No run directories found in: {log_path}")

    # Sort to get latest run
    run_dirs.sort()
    latest_run = run_dirs[-1]
    run_path = os.path.join(log_path, latest_run)

    # Find checkpoint files
    checkpoint_files = []
    for f in os.listdir(run_path):
        if re.match(checkpoint_pattern, f):
            checkpoint_files.append(f)

    if not checkpoint_files:
        raise ValueError(f"No checkpoint files matching '{checkpoint_pattern}' found in: {run_path}")

    # Sort to get latest checkpoint
    checkpoint_files.sort(key=lambda m: f"{m:0>15}")
    latest_checkpoint = checkpoint_files[-1]

    return os.path.join(run_path, latest_checkpoint)


def load_checkpoint_with_fallback(runner: OnPolicyRunner, checkpoint_path: str, load_optimizer: bool = True):
    """Load checkpoint with fallback for PyTorch compatibility issues.

    Args:
        runner: RSL-RL runner instance
        checkpoint_path: Path to checkpoint file
        load_optimizer: Whether to load optimizer state
    """
    print(f"[INFO] Loading checkpoint from: {checkpoint_path}")

    # Load checkpoint to CPU first for compatibility
    loaded_dict = torch.load(checkpoint_path, map_location='cpu', weights_only=False)

    # Load model state - handle both standard algorithms (PPO) and MDPO
    if runner.is_mdpo:
        # MDPO uses two actor-critics, load same state into both
        runner.alg.actor_critic_1.load_state_dict(loaded_dict["model_state_dict"], strict=True)
        runner.alg.actor_critic_2.load_state_dict(loaded_dict["model_state_dict"], strict=True)
    else:
        # Standard algorithms use one actor-critic
        runner.alg.actor_critic.load_state_dict(loaded_dict["model_state_dict"], strict=True)

    # Load normalizers if using empirical normalization
    if runner.empirical_normalization:
        runner.obs_normalizer.load_state_dict(loaded_dict["obs_norm_state_dict"])
        runner.critic_obs_normalizer.load_state_dict(loaded_dict["critic_obs_norm_state_dict"])

    # Load optimizer if requested
    if load_optimizer:
        if runner.is_mdpo:
            runner.alg.optimizer_1.load_state_dict(loaded_dict["optimizer_state_dict"])
        else:
            runner.alg.optimizer.load_state_dict(loaded_dict["optimizer_state_dict"])

    runner.current_learning_iteration = loaded_dict["iter"]
    print(f"[INFO] Loaded checkpoint from iteration {loaded_dict['iter']}")


def export_policy_jit(runner: OnPolicyRunner, checkpoint_path: str):
    """Export policy as JIT module to an 'export' folder next to the checkpoint.

    Args:
        runner: RSL-RL runner instance with loaded policy
        checkpoint_path: Path to the checkpoint file (used to determine export location)
    """
    # Determine export directory (create 'export' folder in the same directory as checkpoint)
    checkpoint_dir = os.path.dirname(checkpoint_path)
    export_dir = os.path.join(checkpoint_dir, "export")

    # Get the actor-critic module
    if runner.is_mdpo:
        actor_critic = runner.alg.actor_critic_1
    else:
        actor_critic = runner.alg.actor_critic

    # Get normalizer if using empirical normalization
    normalizer = runner.obs_normalizer if runner.empirical_normalization else None

    # Export using the module's export_jit method
    print(f"[INFO] Exporting JIT policy to: {export_dir}")
    actor_critic.export_jit(path=export_dir, filename="policy.pt", normalizer=normalizer)
    print(f"[INFO] JIT export complete!")


def export_policy_onnx(runner: OnPolicyRunner, checkpoint_path: str):
    """Export policy as ONNX model to an 'export' folder next to the checkpoint.

    Args:
        runner: RSL-RL runner instance with loaded policy
        checkpoint_path: Path to the checkpoint file (used to determine export location)
    """
    # Determine export directory (create 'export' folder in the same directory as checkpoint)
    checkpoint_dir = os.path.dirname(checkpoint_path)
    export_dir = os.path.join(checkpoint_dir, "export")

    # Get the actor-critic module
    if runner.is_mdpo:
        actor_critic = runner.alg.actor_critic_1
    else:
        actor_critic = runner.alg.actor_critic

    # Get normalizer if using empirical normalization
    normalizer = runner.obs_normalizer if runner.empirical_normalization else None

    # Check if the module has export_onnx method
    if not hasattr(actor_critic, "export_onnx"):
        raise NotImplementedError(
            f"ONNX export not implemented for {type(actor_critic).__name__}. "
            "Please add an export_onnx method to this module."
        )

    # Export using the module's export_onnx method
    print(f"[INFO] Exporting ONNX policy to: {export_dir}")
    actor_critic.export_onnx(path=export_dir, filename="policy.onnx", normalizer=normalizer)
    print(f"[INFO] ONNX export complete!")


def main():
    """Play navigation policy with RSL-RL."""
    # Parse command-line arguments
    spec = gym.spec(args_cli.task)
    env_cfg_class = spec.kwargs.get("env_cfg_entry_point")
    agent_cfg_class = spec.kwargs.get("rsl_rl_cfg_entry_point")

    # Instantiate the configs
    env_cfg: ManagerBasedRLEnvCfg = env_cfg_class()
    agent_cfg: RslRlOnPolicyRunnerCfg = agent_cfg_class()

    # Override config from command line
    if args_cli.num_envs is not None:
        env_cfg.scene.num_envs = args_cli.num_envs

    # Get checkpoint path
    if args_cli.checkpoint:
        resume_path = args_cli.checkpoint
    else:
        # Get last checkpoint from log directory
        log_root_path = os.path.join("logs", "rsl_rl", agent_cfg.experiment_name)
        resume_path = find_latest_checkpoint(log_root_path, checkpoint_pattern="model_.*.pt")

    # Create the environment
    env = gym.make(args_cli.task, cfg=env_cfg, render_mode="rgb_array" if args_cli.video else None)
    if args_cli.video:
        video_kwargs = {
            "video_folder": os.path.join(os.path.dirname(resume_path), "videos", "play"),
            "step_trigger": lambda step: step == 0,
            "video_length": args_cli.video_length,
            "disable_logger": True,
        }
        print(f"[INFO] Recording videos during play to: {video_kwargs['video_folder']}")
        env = gym.wrappers.RecordVideo(env, **video_kwargs)

    # Wrap the environment
    env = RslRlVecEnvWrapper(env)

    # Check if checkpoint is a TorchScript JIT policy or standard checkpoint dict
    is_jit = False
    try:
        jit_policy = torch.jit.load(resume_path, map_location=env.unwrapped.device)
        is_jit = True
    except Exception:
        is_jit = False

    if is_jit:
        print(f"[INFO] Loaded pre-trained TorchScript JIT policy directly from: {resume_path}")
        jit_policy.eval()

        # Adapt recurrent hidden states to match environment count
        num_envs = env.unwrapped.num_envs
        device = env.unwrapped.device
        if hasattr(jit_policy, "hidden_state") and hasattr(jit_policy, "cell_state"):
            num_layers = jit_policy.hidden_state.shape[0]
            hidden_size = jit_policy.hidden_state.shape[2]
            jit_policy.hidden_state = torch.zeros(num_layers, num_envs, hidden_size, device=device)
            jit_policy.cell_state = torch.zeros(num_layers, num_envs, hidden_size, device=device)
            print(f"[INFO] Adapted JIT policy recurrent states for num_envs={num_envs} (shape: [{num_layers}, {num_envs}, {hidden_size}]).")

        def policy(obs):
            if hasattr(obs, "keys"):
                obs = obs["policy"]
            return jit_policy(obs)

        def policy_reset(dones):
            if isinstance(dones, torch.Tensor) and (dones == 1).any():
                if hasattr(jit_policy, "hidden_state") and hasattr(jit_policy, "cell_state"):
                    done_mask = (dones == 1)
                    jit_policy.hidden_state[:, done_mask] = 0.0
                    jit_policy.cell_state[:, done_mask] = 0.0
    else:
        # Create runner
        runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)

        # Load checkpoint with compatibility handling
        load_checkpoint_with_fallback(runner, resume_path)

        # Export JIT if requested
        if args_cli.export_jit:
            export_policy_jit(runner, resume_path)

        # Export ONNX if requested
        if args_cli.export_onnx:
            export_policy_onnx(runner, resume_path)

        # Obtain policy for inference
        policy = runner.get_inference_policy(device=env.unwrapped.device)
        policy_reset = runner.get_policy_reset(device=env.unwrapped.device) if hasattr(runner, "get_policy_reset") else None

    # Reset environment
    obs = env.get_observations()
    if isinstance(obs, tuple):
        obs = obs[0]
    if hasattr(obs, "keys"):
        obs = obs["policy"]

    # Episode metrics tracker
    num_envs = env.unwrapped.num_envs
    step_dt = env.unwrapped.step_dt
    ep_steps = torch.zeros(num_envs, dtype=torch.long, device=env.unwrapped.device)

    # Goal command generator for initial distance tracking
    goal_cmd = env.unwrapped.command_manager._terms.get("robot_goal", None) if hasattr(env.unwrapped, "command_manager") else None
    initial_distances = torch.zeros(num_envs, device=env.unwrapped.device)
    if goal_cmd is not None and hasattr(goal_cmd, "initial_distance_to_goal"):
        initial_distances = goal_cmd.initial_distance_to_goal.clone()

    total_episodes = 0
    success_episodes = 0
    collision_episodes = 0
    timeout_episodes = 0
    fall_episodes = 0
    success_times = []
    success_distances = []

    def print_summary():
        if total_episodes == 0:
            print("\n[INFO] No completed episodes to summarize.")
            return
        sr = (success_episodes / total_episodes) * 100.0
        cr = (collision_episodes / total_episodes) * 100.0
        tr = (timeout_episodes / total_episodes) * 100.0
        fr = (fall_episodes / total_episodes) * 100.0
        other_episodes = total_episodes - (success_episodes + collision_episodes + timeout_episodes + fall_episodes)
        other_rate = (other_episodes / total_episodes) * 100.0

        avg_time = (sum(success_times) / len(success_times)) if success_times else 0.0
        avg_dist = (sum(success_distances) / len(success_distances)) if success_distances else 0.0

        print("\n" + "=" * 68)
        print("                 NAVIGATION EVALUATION SUMMARY")
        print("=" * 68)
        print(f" Total Evaluated Episodes : {total_episodes}")
        if resume_path:
            print(f" Checkpoint               : {resume_path}")
        print("-" * 68)
        print(f" Success Rate (SR)        : {sr:6.2f}% ({success_episodes}/{total_episodes})")
        print(f" Collision Rate (CR)      : {cr:6.2f}% ({collision_episodes}/{total_episodes})")
        print(f" Timeout Rate (TR)        : {tr:6.2f}% ({timeout_episodes}/{total_episodes})")
        print(f" Fall / Flip Rate         : {fr:6.2f}% ({fall_episodes}/{total_episodes})")
        if other_episodes > 0:
            print(f" Other Failures           : {other_rate:6.2f}% ({other_episodes}/{total_episodes})")
        print("-" * 68)
        if success_episodes > 0:
            print(f" Avg Time to Success      : {avg_time:.2f} s")
            print(f" Avg Goal Distance        : {avg_dist:.2f} m")
            avg_speed = avg_dist / avg_time if avg_time > 0 else 0.0
            print(f" Avg Effective Speed      : {avg_speed:.2f} m/s")
        print("=" * 68 + "\n")

    # Simulate environment
    timestep = 0
    try:
        while simulation_app.is_running():
            # Run policy
            with torch.inference_mode():
                actions = policy(obs)
            # Step environment
            ep_steps += 1
            obs, _, dones, _ = env.step(actions)
            if isinstance(obs, tuple):
                obs = obs[0]
            if hasattr(obs, "keys"):
                obs = obs["policy"]
            if policy_reset is not None:
                policy_reset(dones)

            # Check episode completion and record termination statistics
            if isinstance(dones, torch.Tensor) and (dones == 1).any():
                done_ids = (dones == 1).nonzero(as_tuple=False).flatten().tolist()
                term_mgr = getattr(env.unwrapped, "termination_manager", None)
                term_map = getattr(term_mgr, "_term_name_to_term_idx", {}) if term_mgr is not None else {}
                term_dones = getattr(term_mgr, "_term_dones", None) if term_mgr is not None else None

                for env_id in done_ids:
                    total_episodes += 1
                    duration_s = ep_steps[env_id].item() * step_dt
                    init_dist = initial_distances[env_id].item()

                    is_success = bool(term_dones[env_id, term_map["early_termination"]].item()) if (term_dones is not None and "early_termination" in term_map) else False
                    is_collision = bool(term_dones[env_id, term_map["base_contact"]].item()) if (term_dones is not None and "base_contact" in term_map) else False
                    is_timeout = bool(term_dones[env_id, term_map["time_out"]].item()) if (term_dones is not None and "time_out" in term_map) else False
                    is_large_angle = bool(term_dones[env_id, term_map["large_pitch_angle"]].item()) if (term_dones is not None and "large_pitch_angle" in term_map) else False
                    is_fall = bool(term_dones[env_id, term_map["terrain_fall"]].item()) if (term_dones is not None and "terrain_fall" in term_map) else False

                    if is_success:
                        status = "\033[92mSUCCESS (Reached Goal)\033[0m"
                        success_episodes += 1
                        success_times.append(duration_s)
                        success_distances.append(init_dist)
                    elif is_collision:
                        status = "\033[91mCOLLISION (base_contact)\033[0m"
                        collision_episodes += 1
                    elif is_large_angle:
                        status = "\033[93mFAIL (tilted/flipped)\033[0m"
                        fall_episodes += 1
                    elif is_fall:
                        status = "\033[93mFAIL (terrain fall)\033[0m"
                        fall_episodes += 1
                    elif is_timeout:
                        status = "\033[94mTIMEOUT (exceeded 60s)\033[0m"
                        timeout_episodes += 1
                    else:
                        status = "FAIL (other)"

                    sr = (success_episodes / total_episodes) * 100.0
                    print(
                        f"[Episode {total_episodes:4d} | Env {env_id:2d}] {status:<34} | "
                        f"Time: {duration_s:5.1f}s | Goal Dist: {init_dist:4.1f}m | "
                        f"Current SR: {sr:5.1f}% ({success_episodes}/{total_episodes})"
                    )

                    # Reset per-env tracking state
                    ep_steps[env_id] = 0
                    if goal_cmd is not None and hasattr(goal_cmd, "initial_distance_to_goal"):
                        initial_distances[env_id] = goal_cmd.initial_distance_to_goal[env_id]

                    if args_cli.num_episodes is not None and total_episodes >= args_cli.num_episodes:
                        break

            if args_cli.num_episodes is not None and total_episodes >= args_cli.num_episodes:
                print(f"\n[INFO] Reached requested evaluation episodes: {args_cli.num_episodes}")
                break

            if timestep % 25 == 0:
                act_0 = actions[0].tolist()
                num_dones = dones.sum().item() if isinstance(dones, torch.Tensor) else 0
                print(f"[Step {timestep:5d}] Action (Env 0): vx={act_0[0]:.3f}, vy={act_0[1]:.3f}, omega={act_0[2]:.3f} | Done envs: {num_dones}/{env.num_envs}")

            timestep += 1
            if args_cli.video and timestep >= args_cli.video_length:
                break
    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user (Ctrl+C). Generating evaluation summary...")
    finally:
        print_summary()

    # Close the environment
    env.close()


if __name__ == "__main__":
    # Run the main function
    main()
    # Close simulation
    simulation_app.close()

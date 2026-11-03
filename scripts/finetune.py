#!/usr/bin/env python3
# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: MIT

"""Fine-tune a navigation policy on Go2W using YAML configuration.
Includes Critic Warm-up mechanism to protect pre-trained Actor weights from random Critic noise.

Usage:
    ./isaaclab.sh -p source/isaaclab_nav_task/scripts/finetune.py --config source/isaaclab_nav_task/config/go2w_finetune.yaml

Examples:
    # Fine-tune with default config (PPO with 60-iteration Critic Warmup)
    ./isaaclab.sh -p source/isaaclab_nav_task/scripts/finetune.py

    # Override number of envs or seed
    ./isaaclab.sh -p source/isaaclab_nav_task/scripts/finetune.py --num_envs 180 --seed 42
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import yaml
from datetime import datetime

from isaaclab.app import AppLauncher

# Add argparse arguments
parser = argparse.ArgumentParser(description="Fine-tune a navigation policy with RSL-RL.")
parser.add_argument(
    "--config",
    type=str,
    default="source/isaaclab_nav_task/config/go2w_finetune.yaml",
    help="Path to the YAML fine-tuning configuration file.",
)
parser.add_argument("--num_envs", type=int, default=None, help="Number of parallel environments to simulate.")
parser.add_argument("--seed", type=int, default=None, help="Random seed.")
parser.add_argument("--max_iterations", type=int, default=None, help="Max fine-tuning iterations.")
parser.add_argument("--video", action="store_true", default=False, help="Record videos during training.")
parser.add_argument("--video_length", type=int, default=200, help="Video length in steps.")
parser.add_argument("--video_interval", type=int, default=500, help="Interval between video recordings.")

# Append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
args_cli, hydra_args = parser.parse_known_args()

if args_cli.video:
    args_cli.enable_cameras = True

# Launch simulation
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# Import packages after app launch
import gymnasium as gym
import torch

from rsl_rl.runners import OnPolicyRunner

# Import Isaac Lab tasks and navigation extension
import isaaclab_tasks  # noqa: F401
import isaaclab_nav_task  # noqa: F401

from isaaclab.utils.io import dump_yaml
from isaaclab_tasks.utils.parse_cfg import load_cfg_from_registry
from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlVecEnvWrapper

# Enable performance flags
torch.backends.cuda.matmul.allow_tf32 = True
torch.backends.cudnn.allow_tf32 = True
torch.backends.cudnn.deterministic = False
torch.backends.cudnn.benchmark = False


def map_jit_to_actor(jit_name: str) -> str:
    """Map TorchScript Actor parameter names to ActorCriticSRU parameter names."""
    if jit_name.startswith("rnn."):
        return "memory_a." + jit_name
    elif jit_name.startswith("linear."):
        return "linear_dropout_actor." + jit_name
    return jit_name


def inject_actor_weights(ac_module: torch.nn.Module, jit_params: dict, init_noise_std: float) -> int:
    """Inject pre-trained actor weights from TorchScript into an ActorCriticSRU module."""
    state_dict = ac_module.state_dict()
    loaded_count = 0
    for k, v in jit_params.items():
        mapped_k = map_jit_to_actor(k)
        if mapped_k in state_dict:
            state_dict[mapped_k] = v.data.clone()
            loaded_count += 1

    # Initialize log_std to target low exploration noise
    state_dict["log_std"] = torch.full((3,), math.log(init_noise_std))
    ac_module.load_state_dict(state_dict)
    return loaded_count


def setup_critic_warmup(runner: OnPolicyRunner, warmup_iterations: int, actor_lr: float, critic_lr: float):
    """Setup Critic Warm-up: Freeze Actor during warmup iterations so Critic learns without corrupting Actor."""
    if warmup_iterations <= 0:
        print("[INFO] Critic warmup disabled (warmup_iterations <= 0).")
        return

    actor_ratio = actor_lr / critic_lr

    if runner.is_mdpo:
        actor_params_1 = runner.alg.actor_critic_1.get_actor_parameters()
        critic_params_1 = runner.alg.actor_critic_1.get_critic_parameters()
        actor_params_2 = runner.alg.actor_critic_2.get_actor_parameters()
        critic_params_2 = runner.alg.actor_critic_2.get_critic_parameters()

        # Re-initialize optimizers with separated actor/critic param groups
        runner.alg.optimizer_1 = torch.optim.Adam([
            {"params": actor_params_1, "lr": actor_lr, "is_actor": True},
            {"params": critic_params_1, "lr": critic_lr, "is_actor": False},
        ], weight_decay=runner.alg.weight_decay)

        runner.alg.optimizer_2 = torch.optim.Adam([
            {"params": actor_params_2, "lr": actor_lr, "is_actor": True},
            {"params": critic_params_2, "lr": critic_lr, "is_actor": False},
        ], weight_decay=runner.alg.weight_decay)

        all_actor_params = actor_params_1 + actor_params_2
        all_optimizers = [runner.alg.optimizer_1, runner.alg.optimizer_2]
    else:
        actor_params = runner.alg.actor_critic.get_actor_parameters()
        critic_params = runner.alg.actor_critic.get_critic_parameters()

        runner.alg.optimizer = torch.optim.Adam([
            {"params": actor_params, "lr": actor_lr, "is_actor": True},
            {"params": critic_params, "lr": critic_lr, "is_actor": False},
        ])

        all_actor_params = actor_params
        all_optimizers = [runner.alg.optimizer]

    # 1. Freeze actor initially
    for p in all_actor_params:
        p.requires_grad = False

    print(f"\n{'*'*75}")
    print(f"[CRITIC WARM-UP ENABLED] For the first {warmup_iterations} iterations:")
    print(f"  - Pre-trained Actor is 100% FROZEN (no gradient corruption).")
    print(f"  - 80% success-rate Actor will collect real trajectories for the Critic.")
    print(f"  - Critic will train at lr={critic_lr:.2e} to fit the value function V(s).")
    print(f"  - At iteration {warmup_iterations}, Actor will unfreeze at gentle lr={actor_lr:.2e}.")
    print(f"{'*'*75}\n")

    # 2. Hook into alg.update to unfreeze actor when warmup completes
    orig_update = runner.alg.update
    unfrozen = False

    def warmup_hook_update(it, tot_iter):
        nonlocal unfrozen
        if it < warmup_iterations:
            # Keep actor frozen
            for p in all_actor_params:
                p.requires_grad = False
        elif not unfrozen:
            # Unfreeze actor
            unfrozen = True
            for p in all_actor_params:
                p.requires_grad = True
            print(f"\n{'='*75}")
            print(f"[CRITIC WARM-UP COMPLETE at iteration {it}!] Value function V(s) is now converged.")
            print(f"[UNFREEZING ACTOR] Actor unfrozen with gentle learning rate: {actor_lr:.2e}")
            print(f"{'='*75}\n")

        # Call original update
        result = orig_update(it, tot_iter)

        # Scale actor learning rate proportionally if adaptive schedule changes base lr
        if unfrozen:
            for opt in all_optimizers:
                for g in opt.param_groups:
                    if g.get("is_actor"):
                        g["lr"] = runner.alg.learning_rate * actor_ratio

        return result

    runner.alg.update = warmup_hook_update


def main():
    # 1. Load YAML configuration
    config_path = args_cli.config
    if not os.path.isabs(config_path):
        config_path = os.path.abspath(config_path)

    if not os.path.exists(config_path):
        raise FileNotFoundError(f"Configuration file not found: {config_path}")

    print(f"\n[INFO] Loading fine-tuning configuration from: {config_path}")
    with open(config_path, "r", encoding="utf-8") as f:
        finetune_cfg = yaml.safe_load(f)

    task_name = finetune_cfg.get("task", {}).get("name", "Isaac-Nav-PPO-Go2W-v0")
    print(f"[INFO] Target Task: {task_name}")

    # 2. Load default environment and agent configurations
    env_cfg = load_cfg_from_registry(task_name, "env_cfg_entry_point")
    agent_cfg: RslRlOnPolicyRunnerCfg = load_cfg_from_registry(task_name, "rsl_rl_cfg_entry_point")

    # 3. Apply YAML overrides to agent_cfg
    alg_cfg = finetune_cfg.get("algorithm", {})
    critic_lr = float(alg_cfg.get("learning_rate", 2.0e-4))
    actor_lr = float(alg_cfg.get("actor_learning_rate", 3.0e-5))
    warmup_iters = int(alg_cfg.get("critic_warmup_iterations", 60))

    agent_cfg.algorithm.learning_rate = critic_lr
    if "schedule" in alg_cfg:
        agent_cfg.algorithm.schedule = str(alg_cfg["schedule"])
    if "desired_kl" in alg_cfg:
        agent_cfg.algorithm.desired_kl = float(alg_cfg["desired_kl"])
    if "entropy_coef" in alg_cfg:
        agent_cfg.algorithm.entropy_coef = float(alg_cfg["entropy_coef"])
    if "value_loss_coef" in alg_cfg:
        agent_cfg.algorithm.value_loss_coef = float(alg_cfg["value_loss_coef"])
    if "clip_param" in alg_cfg:
        agent_cfg.algorithm.clip_param = float(alg_cfg["clip_param"])
    if "gamma" in alg_cfg:
        agent_cfg.algorithm.gamma = float(alg_cfg["gamma"])

    pol_cfg = finetune_cfg.get("policy", {})
    init_noise_std = float(pol_cfg.get("init_noise_std", 0.25))
    agent_cfg.policy.init_noise_std = init_noise_std
    if "dropout" in pol_cfg:
        agent_cfg.policy.dropout = float(pol_cfg["dropout"])

    train_cfg = finetune_cfg.get("training", {})
    agent_cfg.max_iterations = int(train_cfg.get("max_iterations", 1500))
    agent_cfg.save_interval = int(train_cfg.get("save_interval", 100))
    agent_cfg.experiment_name = str(train_cfg.get("experiment_name", "go2w_navigation_finetune"))
    agent_cfg.run_name = str(train_cfg.get("run_name", "warmup_ppo"))
    agent_cfg.logger = str(train_cfg.get("logger", "tensorboard"))

    # CLI overrides
    if args_cli.num_envs is not None:
        env_cfg.scene.num_envs = args_cli.num_envs
    elif "num_envs" in finetune_cfg.get("task", {}):
        env_cfg.scene.num_envs = int(finetune_cfg["task"]["num_envs"])

    if args_cli.seed is not None:
        agent_cfg.seed = args_cli.seed
    elif "seed" in finetune_cfg.get("task", {}):
        agent_cfg.seed = int(finetune_cfg["task"]["seed"])

    if args_cli.max_iterations is not None:
        agent_cfg.max_iterations = args_cli.max_iterations

    # 4. Apply YAML overrides to env_cfg
    env_yaml_cfg = finetune_cfg.get("environment", {})
    rewards_yaml = env_yaml_cfg.get("rewards", {})
    if "joint_acc_l2_weight" in rewards_yaml and hasattr(env_cfg.rewards, "joint_acc_l2_joint"):
        env_cfg.rewards.joint_acc_l2_joint.weight = float(rewards_yaml["joint_acc_l2_weight"])
    if "action_rate_l1_weight" in rewards_yaml and hasattr(env_cfg.rewards, "action_rate_l1"):
        env_cfg.rewards.action_rate_l1.weight = float(rewards_yaml["action_rate_l1_weight"])
    if "reach_goal_soft_prob" in rewards_yaml and hasattr(env_cfg.rewards, "reach_goal_xy_soft"):
        env_cfg.rewards.reach_goal_xy_soft.params["probability"] = float(rewards_yaml["reach_goal_soft_prob"])

    terrain_yaml = env_yaml_cfg.get("terrain", {})
    if "curriculum" in terrain_yaml:
        env_cfg.scene.terrain.terrain_generator.curriculum = bool(terrain_yaml["curriculum"])
    if "max_init_terrain_level" in terrain_yaml:
        env_cfg.scene.terrain.max_init_terrain_level = int(terrain_yaml["max_init_terrain_level"])
    if "difficulty_range" in terrain_yaml:
        env_cfg.scene.terrain.terrain_generator.difficulty_range = list(terrain_yaml["difficulty_range"])

    # 5. Create environment & runner
    env = gym.make(task_name, cfg=env_cfg, render_mode="rgb_array" if args_cli.video else None)
    env = RslRlVecEnvWrapper(env)

    log_root_path = os.path.join("logs", "rsl_rl", agent_cfg.experiment_name)
    log_root_path = os.path.abspath(log_root_path)
    log_dir = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    if agent_cfg.run_name:
        log_dir += f"_{agent_cfg.run_name}"
    log_dir = os.path.join(log_root_path, log_dir)
    print(f"[INFO] Logging experiment in directory: {log_dir}")

    runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=log_dir, device=agent_cfg.device)
    runner.add_git_repo_to_log(__file__)

    # Save running configs for reproducibility
    os.makedirs(os.path.join(log_dir, "params"), exist_ok=True)
    dump_yaml(os.path.join(log_dir, "params", "env.yaml"), env_cfg)
    dump_yaml(os.path.join(log_dir, "params", "agent.yaml"), agent_cfg)
    dump_yaml(os.path.join(log_dir, "params", "finetune_config.yaml"), finetune_cfg)

    # 6. Load pretrained policy
    ckpt_cfg = finetune_cfg.get("checkpoint", {})
    pretrained_path = ckpt_cfg.get("pretrained_path")

    if pretrained_path and os.path.exists(pretrained_path):
        print(f"\n{'='*70}\n[INFO] Loading pre-trained checkpoint: {pretrained_path}")
        is_jit = False
        try:
            jit_model = torch.jit.load(pretrained_path, map_location="cpu")
            is_jit = True
        except Exception:
            is_jit = False

        if is_jit:
            jit_params = dict(jit_model.named_parameters())
            print(f"[INFO] Detected TorchScript JIT archive with {len(jit_params)} parameters.")

            if runner.is_mdpo:
                n1 = inject_actor_weights(runner.alg.actor_critic_1, jit_params, init_noise_std)
                n2 = inject_actor_weights(runner.alg.actor_critic_2, jit_params, init_noise_std)
                print(f"[SUCCESS] Injected {n1} actor weights into MDPO Policy 1 & Policy 2!")
            else:
                n = inject_actor_weights(runner.alg.actor_critic, jit_params, init_noise_std)
                print(f"[SUCCESS] Injected {n} actor weights into ActorCritic network!")

            print(f"[INFO] Exploration noise std initialized to {init_noise_std:.3f}.")
            print(f"{'='*70}\n")
        else:
            print("[INFO] Loading standard checkpoint dict via runner.load...")
            runner.load(pretrained_path, load_optimizer=False)
            print(f"[SUCCESS] Loaded checkpoint from {pretrained_path}!\n{'='*70}\n")
    else:
        print(f"[WARNING] Pretrained checkpoint path not found or not provided: {pretrained_path}")

    # 7. Setup Critic Warm-up (Freeze Actor for N iterations)
    setup_critic_warmup(runner, warmup_iters, actor_lr, critic_lr)

    # 8. Start fine-tuning
    print(f"[INFO] Starting fine-tuning for {agent_cfg.max_iterations} iterations...\n")
    runner.learn(num_learning_iterations=agent_cfg.max_iterations, init_at_random_ep_len=True)

    # 9. Close environment
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()

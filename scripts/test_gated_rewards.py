#!/usr/bin/env python3
"""Test script for verifying Go2W gated rewards and environment stepping."""

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Test Go2W gated rewards.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# Launch headless simulation app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym
import torch
import isaaclab_tasks  # noqa: F401
import isaaclab_nav_task  # noqa: F401
from isaaclab_tasks.utils.parse_cfg import load_cfg_from_registry

print("=" * 60)
print("1. Checking Environment and Reward Configurations")
print("=" * 60)

env_cfg_p1 = load_cfg_from_registry("Isaac-Nav-MDPO-Go2W-Phase1-v0", "env_cfg_entry_point")
agent_cfg_p1 = load_cfg_from_registry("Isaac-Nav-MDPO-Go2W-Phase1-v0", "rsl_rl_cfg_entry_point")

# Verify policy scaling
print("Policy scaling:", env_cfg_p1.actions.velocity_command.policy_scaling)
assert env_cfg_p1.actions.velocity_command.policy_scaling[0] == 1.5, "Expected vx scaling 1.5"
assert env_cfg_p1.actions.velocity_command.policy_scaling[1] == 0.3, "Expected vy scaling 0.3"
assert env_cfg_p1.actions.velocity_command.policy_scaling[2] == 1.0, "Expected wz scaling 1.0"
print("[PASS] Policy scaling: [1.5, 0.3, 1.0]")

# Verify rewards
rewards = env_cfg_p1.rewards
print("\nReward terms:")
for k, v in rewards.__dict__.items():
    if v is not None and hasattr(v, "weight"):
        print(f"  - {k}: func={v.func.__name__}, weight={v.weight}")

assert rewards.progress.func.__name__ == "gated_progress_to_goal"
assert rewards.progress.weight == 1.5
assert rewards.heading_progress.func.__name__ == "heading_progress_to_goal"
assert rewards.heading_progress.weight == 0.3
assert rewards.heading.func.__name__ == "heading_alignment"
assert rewards.heading.weight == 0.5
assert rewards.lateral_movement.func.__name__ == "lateral_movement_l2"
assert rewards.lateral_movement.weight == -0.5
print("\n[PASS] All reward functions and weights verified!")

print("\n" + "=" * 60)
print("2. Instantiating Environment (num_envs=4) and Stepping")
print("=" * 60)

env_cfg_p1.scene.num_envs = 4
env = gym.make("Isaac-Nav-MDPO-Go2W-Phase1-v0", cfg=env_cfg_p1)

obs, _ = env.reset()
print(f"Obs reset successful. Policy obs shape: {obs['policy'].shape}")

for step in range(5):
    # Test random actions [-1, 1]
    actions = torch.randn(4, 3, device=env.unwrapped.device)
    obs, reward, terminated, truncated, info = env.step(actions)
    assert not torch.isnan(reward).any(), f"NaN in rewards at step {step}!"
    assert not torch.isinf(reward).any(), f"Inf in rewards at step {step}!"
    print(f"Step {step+1}: mean reward = {reward.mean().item():.4f}, terminated = {terminated.sum().item()}")

print("\n[SUCCESS] Environment stepped 5 times without error. All rewards finite!")
env.close()
simulation_app.close()

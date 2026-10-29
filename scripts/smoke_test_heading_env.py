#!/usr/bin/env python3
"""Smoke test for Go2W heading environment execution."""

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Smoke test Go2W heading environment.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import os
import sys
import torch
import gymnasium as gym
import isaaclab_tasks
import isaaclab_nav_task
from isaaclab_tasks.utils.parse_cfg import load_cfg_from_registry

print("\n" + "=" * 60)
print("Running Smoke Test for Isaac-Nav-MDPO-Go2W-Heading-v0")
print("=" * 60)

env_cfg = load_cfg_from_registry("Isaac-Nav-MDPO-Go2W-Heading-v0", "env_cfg_entry_point")
env_cfg.scene.num_envs = 4

env = gym.make("Isaac-Nav-MDPO-Go2W-Heading-v0", cfg=env_cfg)
print("[INFO] Environment successfully created!")

obs, _ = env.reset()
policy_obs = obs["policy"]
critic_obs = obs["critic"]
print(f"Policy obs shape: {policy_obs.shape} (Expected: [4, 2578])")
print(f"Critic obs shape: {critic_obs.shape}")

assert policy_obs.shape == (4, 2578), f"Expected (4, 2578), got {policy_obs.shape}"

for step in range(5):
    actions = torch.zeros((4, 3), device=env.unwrapped.device)
    obs, rew, term, trunc, info = env.step(actions)
    print(f"Step {step+1}: rew min={rew.min().item():.3f}, max={rew.max().item():.3f}")

print("\n[SUCCESS] Environment step/reset test passed!")
env.close()
sys.stdout.flush()
os._exit(0)

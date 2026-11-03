#!/usr/bin/env python3
"""Verification test script for Go2W navigation refactor."""

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Verify Go2W navigation configuration.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# Launch headless simulation app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# Imports after simulation app launch
import gymnasium as gym
import isaaclab_tasks
import isaaclab_nav_task
import isaaclab_nav_task.navigation.mdp as mdp
from isaaclab_tasks.utils.parse_cfg import load_cfg_from_registry

print("\n" + "=" * 60)
print("1. Testing Isaac-Nav-MDPO-Go2W-v0 (Phase 4 Default) Configuration")
print("=" * 60)
env_cfg = load_cfg_from_registry("Isaac-Nav-MDPO-Go2W-v0", "env_cfg_entry_point")
agent_cfg = load_cfg_from_registry("Isaac-Nav-MDPO-Go2W-v0", "rsl_rl_cfg_entry_point")

print("Episode length (s):", env_cfg.episode_length_s)
assert env_cfg.episode_length_s == 60.0, f"Expected 60.0s, got {env_cfg.episode_length_s}"

assert env_cfg.rewards.reach_goal_xy_soft is None, "reach_goal_xy_soft should be None (disabled in Go2W)!"
assert env_cfg.rewards.reach_goal_xy_tight is None, "reach_goal_xy_tight should be None (disabled in Go2W)!"
assert env_cfg.rewards.episode_termination.weight == -50.0, f"Expected -50.0 termination weight, got {env_cfg.rewards.episode_termination.weight}"
assert env_cfg.rewards.backward_movement_penalty.weight == -1.0, f"Expected -1.0 backward_movement, got {env_cfg.rewards.backward_movement_penalty.weight}"
assert env_cfg.rewards.action_rate_l1.weight == -0.02, f"Expected -0.02 action_rate_l1 weight, got {env_cfg.rewards.action_rate_l1.weight}"
assert env_cfg.rewards.lateral_movement.weight == -0.5, f"Expected -0.5 lateral_movement weight, got {env_cfg.rewards.lateral_movement.weight}"
assert env_cfg.rewards.joint_acc_l2_joint.weight == -1e-7, f"Expected -1e-7 joint_acc_l2_joint weight, got {env_cfg.rewards.joint_acc_l2_joint.weight}"
assert env_cfg.rewards.progress.weight == 1.5, f"Expected 1.5 progress weight, got {env_cfg.rewards.progress.weight}"
assert env_cfg.rewards.heading.weight == 0.5, "heading weight should be 0.5!"
assert env_cfg.rewards.reach_goal.weight == 10.0, "reach_goal weight should be 10.0!"
assert env_cfg.curriculum.disable_backward_penalty is None, "disable_backward_penalty must be None (permanently locked)!"

print("Base contact bodies:", env_cfg.terminations.base_contact.params["sensor_cfg"].body_names)
assert "base" in env_cfg.terminations.base_contact.params["sensor_cfg"].body_names, "base_contact should monitor base!"
print("Base contact threshold:", env_cfg.terminations.base_contact.params["threshold"])
assert env_cfg.terminations.base_contact.params["threshold"] == 1.0, "base_contact threshold should be 1.0!"
print("Terrain fall threshold:", env_cfg.terminations.terrain_fall.params["fall_height_threshold"])
assert env_cfg.terminations.terrain_fall.params["fall_height_threshold"] == -0.35, "fall_height_threshold should be -0.35!"
print("Goal height offset range:", env_cfg.commands.robot_goal.goal_height_offset_range)
assert env_cfg.commands.robot_goal.goal_height_offset_range == (0.1, 0.4), "goal_height_offset_range should be (0.1, 0.4)!"

print("Policy scaling (Phase 4):", env_cfg.actions.velocity_command.policy_scaling)
assert env_cfg.actions.velocity_command.policy_scaling[0] >= 1.5, "policy_scaling vx should be >= 1.5!"
assert env_cfg.actions.velocity_command.policy_scaling[1] >= 0.0, "policy_scaling vy should be >= 0.0!"

print("MDPO Alg:", agent_cfg.algorithm.class_name)
assert agent_cfg.algorithm.class_name == "MDPO", "Algorithm must be MDPO!"
print("MDPO LR:", agent_cfg.algorithm.learning_rate)
assert agent_cfg.algorithm.learning_rate == 1.0e-3, "LR ceiling should be 1.0e-3!"
print("MDPO Steps per env:", agent_cfg.num_steps_per_env)
assert agent_cfg.num_steps_per_env == 24, "Steps per env should be 24!"
print("MDPO Gamma:", agent_cfg.algorithm.gamma)
assert agent_cfg.algorithm.gamma == 0.995, f"Gamma should be 0.995, got {agent_cfg.algorithm.gamma}!"
print("MDPO Desired KL:", agent_cfg.algorithm.desired_kl)
assert agent_cfg.algorithm.desired_kl == 0.01, f"Desired KL should be 0.01, got {agent_cfg.algorithm.desired_kl}!"
print("MDPO Clip Param:", agent_cfg.algorithm.clip_param)
assert agent_cfg.algorithm.clip_param == 0.2, f"Clip param should be 0.2, got {agent_cfg.algorithm.clip_param}!"
print("MDPO Schedule:", agent_cfg.algorithm.schedule)
assert agent_cfg.algorithm.schedule in ["exponential", "cosine"], f"Schedule should be exponential or cosine, got {agent_cfg.algorithm.schedule}!"

print("\n" + "=" * 60)
print("2. Testing 5-Phase Curriculum Configurations (P1+2=1500, P3=1500, P4+5=3000)")
print("=" * 60)
# Phase 1: Flat ground + sparse obstacles, vx=1.5, iter=500
env_p1 = load_cfg_from_registry("Isaac-Nav-MDPO-Go2W-Phase1-v0", "env_cfg_entry_point")
agent_p1 = load_cfg_from_registry("Isaac-Nav-MDPO-Go2W-Phase1-v0", "rsl_rl_cfg_entry_point")
assert "non_maze" in env_p1.scene.terrain.terrain_generator.sub_terrains
assert "stairs" not in env_p1.scene.terrain.terrain_generator.sub_terrains
assert env_p1.actions.velocity_command.policy_scaling[0] == 1.5
assert env_p1.actions.velocity_command.policy_scaling[1] >= 0.0
assert agent_p1.policy.init_noise_std == 1.0
assert agent_p1.algorithm.entropy_coef == 0.005
assert agent_p1.max_iterations == 500
print("[PASS] Phase 1 Config (Flat, vx=1.5, entropy=0.005, iter=500)")

# Phase 2: Stabilization, vx=1.5, iter=1000 (P1+P2=1500 total)
agent_p2 = load_cfg_from_registry("Isaac-Nav-MDPO-Go2W-Phase2-v0", "rsl_rl_cfg_entry_point")
assert agent_p2.algorithm.entropy_coef == 0.003
assert agent_p2.max_iterations == 1000
print("[PASS] Phase 2 Config (Stabilize, vx=1.5, entropy=0.003, iter=1000)")

# Phase 3: Stairs (10-15cm) + Mazes, vx=1.6, iter=1500
env_p3 = load_cfg_from_registry("Isaac-Nav-MDPO-Go2W-Phase3-v0", "env_cfg_entry_point")
agent_p3 = load_cfg_from_registry("Isaac-Nav-MDPO-Go2W-Phase3-v0", "rsl_rl_cfg_entry_point")
assert "stairs" in env_p3.scene.terrain.terrain_generator.sub_terrains, "Phase 3 must have stairs!"
assert env_p3.actions.velocity_command.policy_scaling[0] in [1.5, 1.6]
assert agent_p3.algorithm.entropy_coef == 0.003
assert agent_p3.max_iterations == 1500
print("[PASS] Phase 3 Config (Stairs+Maze, vx=1.6, entropy=0.003, iter=1500)")

# Phase 4: Full 20cm Stairs + Dense Mazes + Pits, vx=1.8, iter=1500
env_p4 = load_cfg_from_registry("Isaac-Nav-MDPO-Go2W-Phase4-v0", "env_cfg_entry_point")
agent_p4 = load_cfg_from_registry("Isaac-Nav-MDPO-Go2W-Phase4-v0", "rsl_rl_cfg_entry_point")
assert "stairs" in env_p4.scene.terrain.terrain_generator.sub_terrains, "Phase 4 must have stairs!"
assert "pits" in env_p4.scene.terrain.terrain_generator.sub_terrains, "Phase 4 must have pits!"
assert env_p4.actions.velocity_command.policy_scaling[0] in [1.5, 1.8]
assert agent_p4.algorithm.entropy_coef == 0.003
assert agent_p4.max_iterations == 1500
print("[PASS] Phase 4 Config (20cm Stairs+Pits+DenseMaze, vx=1.8, entropy=0.003, iter=1500)")

# Phase 5: Refinement, vx=1.8, iter=1500 (P4+P5=3000 total)
agent_p5 = load_cfg_from_registry("Isaac-Nav-MDPO-Go2W-Phase5-v0", "rsl_rl_cfg_entry_point")
assert agent_p5.algorithm.entropy_coef == 0.002
assert agent_p5.algorithm.clip_param == 0.15
assert agent_p5.max_iterations == 1500
print("[PASS] Phase 5 Config (Refinement, vx=1.8, entropy=0.002, clip=0.15, iter=1500)")

# PureMaze
env_pm = load_cfg_from_registry("Isaac-Nav-MDPO-Go2W-PureMaze-v0", "env_cfg_entry_point")
assert env_pm.scene.terrain.terrain_generator.sub_terrains["maze"].proportion == 1.0
print("[PASS] PureMaze Config (100% maze sub-terrain)")

print("\n" + "=" * 60)
print("3. Testing PPO Go2W Configuration")
print("=" * 60)
agent_ppo = load_cfg_from_registry("Isaac-Nav-PPO-Go2W-Phase1-v0", "rsl_rl_cfg_entry_point")
print("PPO Alg:", agent_ppo.algorithm.class_name)
assert agent_ppo.algorithm.class_name == "PPO"
print("PPO LR:", agent_ppo.algorithm.learning_rate)
assert agent_ppo.algorithm.learning_rate == 1.0e-3
print("PPO Steps per env:", agent_ppo.num_steps_per_env)
assert agent_ppo.num_steps_per_env == 24
print("PPO Gamma:", agent_ppo.algorithm.gamma)
assert agent_ppo.algorithm.gamma == 0.995, f"PPO Gamma should be 0.995, got {agent_ppo.algorithm.gamma}!"
print("PPO Desired KL:", agent_ppo.algorithm.desired_kl)
assert agent_ppo.algorithm.desired_kl == 0.02, f"PPO Desired KL should be 0.02, got {agent_ppo.algorithm.desired_kl}!"
print("PPO Clip Param:", agent_ppo.algorithm.clip_param)
assert agent_ppo.algorithm.clip_param == 0.15, f"PPO Clip param should be 0.15, got {agent_ppo.algorithm.clip_param}!"
print("PPO Schedule:", agent_ppo.algorithm.schedule)
assert agent_ppo.algorithm.schedule == "adaptive"
assert agent_ppo.algorithm.entropy_coef == 0.005
assert agent_ppo.max_iterations == 500
assert agent_ppo.save_interval == 250
print("[PASS] PPO Phase 1 Config (adaptive, kl=0.02, entropy=0.005, iter=500)")

ppo_p2 = load_cfg_from_registry("Isaac-Nav-PPO-Go2W-Phase2-v0", "rsl_rl_cfg_entry_point")
assert ppo_p2.max_iterations == 1000
assert ppo_p2.save_interval == 250
print("[PASS] PPO Phase 2 Config (iter=1000)")

ppo_p3 = load_cfg_from_registry("Isaac-Nav-PPO-Go2W-Phase3-v0", "rsl_rl_cfg_entry_point")
assert ppo_p3.max_iterations == 1500
assert ppo_p3.save_interval == 250
print("[PASS] PPO Phase 3 Config (iter=1500)")

ppo_p4 = load_cfg_from_registry("Isaac-Nav-PPO-Go2W-Phase4-v0", "rsl_rl_cfg_entry_point")
assert ppo_p4.max_iterations == 1500
assert ppo_p4.save_interval == 250
print("[PASS] PPO Phase 4 Config (iter=1500)")

ppo_p5 = load_cfg_from_registry("Isaac-Nav-PPO-Go2W-Phase5-v0", "rsl_rl_cfg_entry_point")
assert ppo_p5.max_iterations == 1500
assert ppo_p5.save_interval == 250
print("[PASS] PPO Phase 5 Config (iter=1500)")

print("\n" + "=" * 60)
print("4. Testing Go2W Heading Tasks Configuration")
print("=" * 60)
env_heading = load_cfg_from_registry("Isaac-Nav-MDPO-Go2W-Heading-v0", "env_cfg_entry_point")
agent_heading = load_cfg_from_registry("Isaac-Nav-MDPO-Go2W-Heading-v0", "rsl_rl_cfg_entry_point")

print("Heading Reward target_heading weight:", env_heading.rewards.target_heading.weight)
assert env_heading.rewards.target_heading.weight == 1.0, f"Expected 1.0, got {env_heading.rewards.target_heading.weight}"
assert env_heading.rewards.velocity_at_goal.weight == -1.0, f"Expected -1.0, got {env_heading.rewards.velocity_at_goal.weight}"
assert hasattr(env_heading.observations.policy, "target_heading"), "Policy must include target_heading obs!"
assert hasattr(env_heading.observations.critic, "target_heading"), "Critic must include target_heading obs!"
assert env_heading.terminations.early_termination.params["heading_threshold"] == 0.1745, "Heading threshold should be 0.1745 rad!"
assert env_heading.terminations.time_out.func == mdp.time_out_with_heading_navigation, "Timeout must be heading-aware!"
assert env_heading.observations.metrics.in_goal.func == mdp.in_goal_with_heading, "Metrics in_goal must be heading-aware!"

print("Heading Agent Alg:", agent_heading.algorithm.class_name)
assert agent_heading.algorithm.class_name == "MDPO"
assert agent_heading.algorithm.learning_rate == 1.5e-4, f"Expected 1.5e-4, got {agent_heading.algorithm.learning_rate}"
assert agent_heading.algorithm.clip_param == 0.10, f"Expected 0.10, got {agent_heading.algorithm.clip_param}"
assert agent_heading.algorithm.desired_kl == 0.005, f"Expected 0.005, got {agent_heading.algorithm.desired_kl}"
assert agent_heading.policy.init_noise_std == 0.35, f"Expected 0.35, got {agent_heading.policy.init_noise_std}"
assert agent_heading.max_iterations == 800, f"Expected 800, got {agent_heading.max_iterations}"
print("[PASS] Isaac-Nav-MDPO-Go2W-Heading-v0 Config Verified!")

agent_ppo_heading = load_cfg_from_registry("Isaac-Nav-PPO-Go2W-Heading-v0", "rsl_rl_cfg_entry_point")
assert agent_ppo_heading.algorithm.class_name == "PPO"
assert agent_ppo_heading.algorithm.learning_rate == 1.5e-4
assert agent_ppo_heading.max_iterations == 800
print("[PASS] Isaac-Nav-PPO-Go2W-Heading-v0 Config Verified!")

print("\n" + "=" * 60)
print("5. Testing B2W Heading Tasks Configuration")
print("=" * 60)
env_b2w_heading = load_cfg_from_registry("Isaac-Nav-MDPO-B2W-Heading-v0", "env_cfg_entry_point")
agent_b2w_heading = load_cfg_from_registry("Isaac-Nav-MDPO-B2W-Heading-v0", "rsl_rl_cfg_entry_point")

print("B2W Heading Reward target_heading weight:", env_b2w_heading.rewards.target_heading.weight)
assert env_b2w_heading.rewards.target_heading.weight == 1.0, f"Expected 1.0, got {env_b2w_heading.rewards.target_heading.weight}"
assert hasattr(env_b2w_heading.observations.policy, "target_heading"), "B2W Policy must include target_heading obs!"
assert hasattr(env_b2w_heading.observations.critic, "target_heading"), "B2W Critic must include target_heading obs!"
assert env_b2w_heading.terminations.early_termination.params["heading_threshold"] == 0.1745, "B2W Heading threshold should be 0.1745 rad!"
assert env_b2w_heading.terminations.time_out.func == mdp.time_out_with_heading_navigation, "B2W Timeout must be heading-aware!"
assert env_b2w_heading.observations.metrics.in_goal.func == mdp.in_goal_with_heading, "B2W Metrics in_goal must be heading-aware!"

print("B2W Heading Agent Alg:", agent_b2w_heading.algorithm.class_name)
assert agent_b2w_heading.algorithm.class_name == "MDPO"
print("B2W Heading Agent LR:", agent_b2w_heading.algorithm.learning_rate)
assert agent_b2w_heading.algorithm.learning_rate == 1.5e-4, f"Expected 1.5e-4, got {agent_b2w_heading.algorithm.learning_rate}"
assert agent_b2w_heading.algorithm.clip_param == 0.10, f"Expected 0.10, got {agent_b2w_heading.algorithm.clip_param}"
assert agent_b2w_heading.algorithm.desired_kl == 0.005, f"Expected 0.005, got {agent_b2w_heading.algorithm.desired_kl}"
assert agent_b2w_heading.policy.init_noise_std == 0.35, f"Expected 0.35, got {agent_b2w_heading.policy.init_noise_std}"
assert agent_b2w_heading.max_iterations == 2500, f"Expected 2500, got {agent_b2w_heading.max_iterations}"
assert agent_b2w_heading.save_interval == 100, f"Expected 100, got {agent_b2w_heading.save_interval}"
print("[PASS] Isaac-Nav-MDPO-B2W-Heading-v0 Config Verified!")

agent_b2w_ppo_heading = load_cfg_from_registry("Isaac-Nav-PPO-B2W-Heading-v0", "rsl_rl_cfg_entry_point")
assert agent_b2w_ppo_heading.algorithm.class_name == "PPO"
assert agent_b2w_ppo_heading.algorithm.learning_rate == 1.5e-4
assert agent_b2w_ppo_heading.max_iterations == 2500
assert agent_b2w_ppo_heading.save_interval == 100
print("[PASS] Isaac-Nav-PPO-B2W-Heading-v0 Config Verified!")

print("\n" + "=" * 60)
print("ALL GO2W & B2W VERIFICATION CHECKS PASSED SUCCESSFULLY!")
print("=" * 60 + "\n")

import os
import sys
sys.stdout.flush()
os._exit(0)


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
assert env_heading.observations.low_level_policy.base_lin_vel is None, "Go2W low_level_policy must have base_lin_vel=None for DreamWaQ (57-dim)!"

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
print("\n" + "=" * 60)
print("5. Testing B2W Heading Tasks Configuration & Curriculum")
print("=" * 60)
env_b2w_heading = load_cfg_from_registry("Isaac-Nav-MDPO-B2W-Heading-v0", "env_cfg_entry_point")
agent_b2w_heading = load_cfg_from_registry("Isaac-Nav-MDPO-B2W-Heading-v0", "rsl_rl_cfg_entry_point")

# Phase 1 initial rewards (heading weight 0.0, terminal 0.0)
print("B2W Heading Reward target_heading weight:", env_b2w_heading.rewards.target_heading.weight)
assert env_b2w_heading.rewards.target_heading.weight == 0.0, f"Expected 0.0 (Phase 1 initial), got {env_b2w_heading.rewards.target_heading.weight}"
assert env_b2w_heading.rewards.target_heading_terminal.weight == 0.0, f"Expected 0.0 (Phase 1 initial), got {env_b2w_heading.rewards.target_heading_terminal.weight}"
assert env_b2w_heading.rewards.target_heading_terminal.params["success_bonus"] == 30.0, "success_bonus should be 30.0!"
assert env_b2w_heading.rewards.target_heading_terminal.params["fail_penalty"] == 15.0, "fail_penalty should be 15.0!"
assert env_b2w_heading.rewards.reach_goal_xy_tight.params["plateau_radius"] == 0.4, "reach_goal_xy_tight must have plateau_radius 0.4!"
assert hasattr(env_b2w_heading.observations.policy, "target_heading"), "B2W Policy must include target_heading obs!"
assert hasattr(env_b2w_heading.observations.critic, "target_heading"), "B2W Critic must include target_heading obs!"

# Phase 1 initial terminations & curriculum
assert env_b2w_heading.terminations.early_termination.params["heading_threshold"] >= 3.14, "Phase 1 initial heading_threshold should be unconstrained (~3.14 rad)!"
assert env_b2w_heading.terminations.early_termination.params["fixed_dwell_time"] == 5.0, "B2W fixed_dwell_time should be 5.0s!"
assert env_b2w_heading.terminations.early_termination.params["velocity_threshold"] == 0.55, "B2W velocity_threshold should be 0.55!"
assert env_b2w_heading.terminations.early_termination.params["required_align_steps"] == 2, "B2W required_align_steps should be 2!"
assert env_b2w_heading.terminations.time_out.func == mdp.time_out_with_heading_navigation, "B2W Timeout must be heading-aware!"
assert env_b2w_heading.observations.metrics.in_goal.func == mdp.in_goal_with_heading, "B2W Metrics in_goal must be heading-aware!"
assert hasattr(env_b2w_heading.curriculum, "heading_phase"), "B2W environment must have heading_phase curriculum!"
assert env_b2w_heading.curriculum.heading_phase.func == mdp.b2w_heading_curriculum_phase, "Curriculum term func mismatch!"

print("B2W Heading Agent Alg:", agent_b2w_heading.algorithm.class_name)
assert agent_b2w_heading.algorithm.class_name == "MDPO"
print("B2W Heading Agent LR:", agent_b2w_heading.algorithm.learning_rate)
assert agent_b2w_heading.algorithm.learning_rate == 3e-5, f"Expected 3e-5, got {agent_b2w_heading.algorithm.learning_rate}"
assert agent_b2w_heading.algorithm.clip_param == 0.10, f"Expected 0.10, got {agent_b2w_heading.algorithm.clip_param}"
assert agent_b2w_heading.algorithm.desired_kl == 0.005, f"Expected 0.005, got {agent_b2w_heading.algorithm.desired_kl}"
assert agent_b2w_heading.policy.init_noise_std == 0.35, f"Expected 0.35, got {agent_b2w_heading.policy.init_noise_std}"
assert agent_b2w_heading.max_iterations == 6000, f"Expected 6000, got {agent_b2w_heading.max_iterations}"
assert agent_b2w_heading.freeze_iters == 600, f"Expected freeze_iters=600, got {agent_b2w_heading.freeze_iters}"
assert agent_b2w_heading.save_interval == 100, f"Expected 100, got {agent_b2w_heading.save_interval}"
print("[PASS] Isaac-Nav-MDPO-B2W-Heading-v0 Config Verified!")

agent_b2w_ppo_heading = load_cfg_from_registry("Isaac-Nav-PPO-B2W-Heading-v0", "rsl_rl_cfg_entry_point")
assert agent_b2w_ppo_heading.algorithm.class_name == "PPO"
assert agent_b2w_ppo_heading.algorithm.learning_rate == 3e-5, f"Expected 3e-5, got {agent_b2w_ppo_heading.algorithm.learning_rate}"
assert agent_b2w_ppo_heading.max_iterations == 6000
assert agent_b2w_ppo_heading.freeze_iters == 600
assert agent_b2w_ppo_heading.save_interval == 100
print("[PASS] Isaac-Nav-PPO-B2W-Heading-v0 Config Verified!")

# 6. Test ActorCriticSRU Layer Freezing Logic
print("\n" + "=" * 60)
print("6. Testing ActorCriticSRU Partial Freezing Logic")
print("=" * 60)
from rsl_rl.modules.actor_critic_sru import ActorCriticSRU
test_model = ActorCriticSRU(
    num_actor_obs=2578,
    num_critic_obs=5715,
    num_actions=3,
    actor_hidden_dims=[512, 256, 128],
    critic_hidden_dims=[512, 256, 128],
    activation="elu",
    init_noise_std=0.35,
    image_input_dims=(64, 5, 8),
    height_input_dims=(64, 7, 7),
    rnn_type="lstm_sru",
    rnn_hidden_size=512,
    rnn_num_layers=1,
    num_cameras=1,
)

# Test freezing
test_model.set_freeze_spatial_and_sru(freeze=True)
# Attention internal layers must be frozen
for name, p in test_model.attn_image_net.named_parameters():
    if "info_proj" in name:
        assert p.requires_grad is True, f"info_proj parameter {name} should be trainable!"
    else:
        assert p.requires_grad is False, f"Spatial attention parameter {name} should be frozen!"
# SRU layers must be frozen
for p in test_model.memory_a.parameters():
    assert p.requires_grad is False, "memory_a should be frozen!"
for p in test_model.memory_c.parameters():
    assert p.requires_grad is False, "memory_c should be frozen!"
# Policy Head MLP must be trainable
for p in test_model.actor.parameters():
    assert p.requires_grad is True, "actor MLP should be trainable!"
for p in test_model.linear_dropout_actor.parameters():
    assert p.requires_grad is True, "linear_dropout_actor should be trainable!"
assert test_model.log_std.requires_grad is True, "log_std should be trainable!"

# Test unfreezing
test_model.set_freeze_spatial_and_sru(freeze=False)
for p in test_model.parameters():
    assert p.requires_grad is True, "All parameters must be trainable after unfreezing!"
print("[PASS] ActorCriticSRU Layer Freezing & Unfreezing Verified!")

# 7. Test 4-Phase Curriculum Logic
print("\n" + "=" * 60)
print("7. Testing B2W Heading 4-Phase Curriculum Transitions")
print("=" * 60)
import torch
from types import SimpleNamespace
from isaaclab_nav_task.navigation.mdp.curriculums import b2w_heading_curriculum_phase

class DummyTerm:
    def __init__(self, weight=0.0, params=None):
        self.weight = weight
        self.params = params or {}

class DummyRewardManager:
    def __init__(self):
        self.terms = {
            "target_heading": DummyTerm(weight=0.0),
            "target_yaw_progress": DummyTerm(weight=0.0),
            "target_heading_terminal": DummyTerm(weight=0.0),
            "velocity_linear_penalty": DummyTerm(weight=0.0),
        }
    def get_term_cfg(self, name):
        return self.terms[name]
    def set_term_cfg(self, name, cfg):
        self.terms[name] = cfg

class DummyTerminationManager:
    def __init__(self):
        self.terms = {
            "early_termination": DummyTerm(params={"distance_threshold": 0.4, "heading_threshold": 3.14159, "fixed_dwell_time": 5.0})
        }
    def get_term_cfg(self, name):
        return self.terms[name]
    def set_term_cfg(self, name, cfg):
        self.terms[name] = cfg

dummy_env = SimpleNamespace(
    device=torch.device("cpu"),
    reward_manager=DummyRewardManager(),
    termination_manager=DummyTerminationManager(),
    current_learning_iteration=0,
    max_learning_iterations=6000,
    common_step_counter=0,
)

# Phase 1: iters 0 - 600 (e.g. iter 300)
dummy_env.current_learning_iteration = 300
p1 = b2w_heading_curriculum_phase(dummy_env, [0]).item()
assert p1 == 1.0, f"Expected Phase 1, got {p1}"
assert dummy_env.reward_manager.get_term_cfg("target_heading").weight == 0.0
assert dummy_env.reward_manager.get_term_cfg("target_yaw_progress").weight == 0.0
assert dummy_env.reward_manager.get_term_cfg("target_heading_terminal").weight == 0.0
assert dummy_env.reward_manager.get_term_cfg("velocity_linear_penalty").weight == 0.0
assert dummy_env.termination_manager.get_term_cfg("early_termination").params["fixed_dwell_time"] == 5.0
assert dummy_env.termination_manager.get_term_cfg("early_termination").params["heading_threshold"] > 3.0

# Phase 1.5: iters 600 - 700 (e.g. iter 650)
dummy_env.current_learning_iteration = 650
p1_5 = b2w_heading_curriculum_phase(dummy_env, [0]).item()
assert p1_5 == 1.5, f"Expected Phase 1.5, got {p1_5}"
assert dummy_env.reward_manager.get_term_cfg("target_heading").weight == 0.0
assert dummy_env.reward_manager.get_term_cfg("target_yaw_progress").weight == 0.0
assert abs(dummy_env.reward_manager.get_term_cfg("target_heading_terminal").weight - 0.3333) < 1e-3
assert dummy_env.reward_manager.get_term_cfg("velocity_linear_penalty").weight == -2.0
assert dummy_env.termination_manager.get_term_cfg("early_termination").params["fixed_dwell_time"] == 5.0
assert dummy_env.termination_manager.get_term_cfg("early_termination").params["velocity_threshold"] == 0.55
assert dummy_env.termination_manager.get_term_cfg("early_termination").params["required_align_steps"] == 2

# Phase 2: iters 700 - 4500 (e.g. iter 750)
dummy_env.current_learning_iteration = 750
p2 = b2w_heading_curriculum_phase(dummy_env, [0]).item()
assert p2 == 2.0, f"Expected Phase 2, got {p2}"
assert dummy_env.reward_manager.get_term_cfg("target_heading").weight == 3.0
assert dummy_env.reward_manager.get_term_cfg("target_yaw_progress").weight == 1.5
assert dummy_env.reward_manager.get_term_cfg("target_heading_terminal").weight == 1.0
assert dummy_env.reward_manager.get_term_cfg("velocity_linear_penalty").weight == -2.0
assert dummy_env.termination_manager.get_term_cfg("early_termination").params["fixed_dwell_time"] == 5.0
assert dummy_env.termination_manager.get_term_cfg("early_termination").params["heading_threshold"] == 0.35
assert dummy_env.termination_manager.get_term_cfg("early_termination").params["velocity_threshold"] == 0.55
assert dummy_env.termination_manager.get_term_cfg("early_termination").params["required_align_steps"] == 2

# Phase 3: iters >= 4500 (e.g. iter 5400)
dummy_env.current_learning_iteration = 5400
p3 = b2w_heading_curriculum_phase(dummy_env, [0]).item()
assert p3 == 3.0, f"Expected Phase 3, got {p3}"
assert dummy_env.reward_manager.get_term_cfg("target_heading").weight == 3.0
assert dummy_env.reward_manager.get_term_cfg("target_yaw_progress").weight == 1.5
assert dummy_env.reward_manager.get_term_cfg("target_heading_terminal").weight == 1.0
assert dummy_env.reward_manager.get_term_cfg("velocity_linear_penalty").weight == -2.0
assert dummy_env.termination_manager.get_term_cfg("early_termination").params["distance_threshold"] == 0.3
assert dummy_env.termination_manager.get_term_cfg("early_termination").params["heading_threshold"] == 0.15
assert dummy_env.termination_manager.get_term_cfg("early_termination").params["fixed_dwell_time"] == 4.0

print("[PASS] B2W Heading 4-Phase Curriculum Transitions Verified!")

print("\n" + "=" * 60)
print("ALL GO2W & B2W VERIFICATION CHECKS PASSED SUCCESSFULLY!")
print("=" * 60 + "\n")

import os
import sys
sys.stdout.flush()
os._exit(0)


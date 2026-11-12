# Copyright (c) 2022-2025, Fan Yang and Per Frivik, ETH Zurich.
# All rights reserved.
#
# SPDX-License-Identifier: MIT

"""Reward functions for navigation tasks.

These functions can be passed to :class:`isaaclab.managers.RewardTermCfg`
to specify the reward function and its parameters.
"""
from __future__ import annotations

from typing import TYPE_CHECKING

import math
import torch

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg

from isaaclab_nav_task.navigation.mdp.navigation.goal_commands import RobotNavigationGoalCommand

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def action_rate_l1(env: "ManagerBasedRLEnv") -> torch.Tensor:
    """Penalize the rate of change of the actions using L1 kernel."""
    return torch.sum(torch.abs(env.action_manager.action - env.action_manager.prev_action), dim=1)


def lateral_movement(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Reward the agent for moving laterally using L1-Kernel.

    Args:
        env: The learning environment.
        asset_cfg: The name of the robot asset.

    Returns:
        Dense reward [0, +1] based on the lateral velocity.
    """
    asset: Articulation = env.scene[asset_cfg.name]
    # compute the reward
    lateral_velocity = asset.data.root_lin_vel_b[:, 1]
    reward = torch.abs(lateral_velocity)
    return reward


def rot_movement(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Reward the agent for rotating around the z-axis using an L2-Kernel.

    Args:
        env: The learning environment.
        asset_cfg: The name of the robot asset.

    Returns:
        Dense reward [0, +1] based on the rotational velocity.
    """
    asset: Articulation = env.scene[asset_cfg.name]
    rot_vel_norm = torch.norm(asset.data.root_ang_vel_b, dim=1)
    return rot_vel_norm


def reach_goal_xyz(
    env: ManagerBasedRLEnv,
    command_name: str,
    sigmoid: float,
    T_r: float,
    probability: float,
    flat: bool,
    ratio: bool,
    plateau_radius: float = 0.0,
) -> torch.Tensor:
    """Reward goal reaching with configurable sigmoid shaping and optional flat plateau.

    Args:
        env: The learning environment.
        command_name: Name of the goal command.
        sigmoid: Sigmoid parameter for shaping.
        T_r: Time reward scaling factor.
        probability: Probability of random sampling.
        flat: Whether to only consider xy error (ignore z).
        ratio: Whether to scale by travel distance ratio.
        plateau_radius: Distance within which reward is locked at maximum saturation (1.0 / T_r).
            Protects robots from losing position reward during in-place turning steps at the goal.

    Returns:
        Dense reward based on distance to goal.
    """
    goal_cmd_generator: RobotNavigationGoalCommand = env.command_manager._terms[command_name]

    t = env.episode_length_buf
    T = env.max_episode_length

    if flat:
        xyz_error = torch.norm(goal_cmd_generator._get_unscaled_command()[:, :2], dim=1)
    else:
        xyz_error = torch.norm(goal_cmd_generator._get_unscaled_command(), dim=1)

    if plateau_radius > 0.0:
        dist_for_reward = torch.clamp(xyz_error - plateau_radius, min=0.0)
    else:
        dist_for_reward = xyz_error

    reward = 1 / (1 + torch.square(dist_for_reward / sigmoid)) / T_r

    timeup_mask = t > (T - goal_cmd_generator.required_time_at_goal_in_steps)
    random_mask = torch.rand_like(t.float()) < probability
    timeup_mask = torch.logical_or(timeup_mask, random_mask)

    arrive_mask = goal_cmd_generator.time_at_goal > 0.0
    reward_mask = torch.logical_or(timeup_mask, arrive_mask)

    if ratio:
        # Calculate the travel distance ratio relative to the initial goal distance
        travel_distance = torch.max(
            goal_cmd_generator.distance_traveled, goal_cmd_generator.initial_distance_to_goal
        )
        travel_distance_ratio = goal_cmd_generator.initial_distance_to_goal / (travel_distance + 1e-6)
    else:
        travel_distance_ratio = torch.ones_like(reward)

    reward = reward * reward_mask.float() * travel_distance_ratio

    return reward


def backward_movement_penalty(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Small penalty for backward movement as a regularization term.

    Args:
        env: The learning environment.
        asset_cfg: The name of the robot asset.

    Returns:
        Penalty [0, +1] based on backward velocity (to be used with negative weight).
    """
    asset: Articulation = env.scene[asset_cfg.name]
    # compute the penalty
    forward_velocity = asset.data.root_lin_vel_b[:, 0]
    # Only penalize negative forward velocity (backward movement)
    backward_velocity = torch.clamp(-forward_velocity, min=0.0, max=1.0)
    return backward_velocity


def progress_to_goal(env: ManagerBasedRLEnv, command_name: str = "robot_goal") -> torch.Tensor:
    """Potential-based dense reward for progress toward the goal.

    Returns positive reward proportional to distance reduction per unit time.
    Handles episode resets by zeroing progress for newly-reset environments.
    """
    goal_cmd: RobotNavigationGoalCommand = env.command_manager.get_term(command_name)
    current_dist = torch.norm(goal_cmd._get_unscaled_command()[:, :2], dim=1)

    # Lazy-initialize state tensor on the env (persists across steps within an episode)
    if not hasattr(env, "_progress_prev_dist"):
        env._progress_prev_dist = current_dist.clone()
        return torch.zeros_like(current_dist)

    progress = env._progress_prev_dist - current_dist
    env._progress_prev_dist = current_dist.clone()

    # Zero out progress for environments that just reset (avoids spurious reward from stale state)
    just_reset = env.episode_length_buf <= 1
    if just_reset.any():
        progress[just_reset] = 0.0

    # Clamp progress to avoid numerical spikes
    progress = torch.clamp(progress, min=-1.0, max=1.0)
    return progress / env.step_dt


def heading_alignment(
    env: ManagerBasedRLEnv, command_name: str = "robot_goal", distance_threshold: float = 0.5
) -> torch.Tensor:
    """Reward for facing toward the goal in robot body frame.

    Returns cos(angle between robot forward axis and direction to goal).
    Range: [-1, 1]. Positive when facing goal, negative when facing away.
    Fades to 0 inside goal radius to prevent spinning singularity.
    """
    goal_cmd: RobotNavigationGoalCommand = env.command_manager.get_term(command_name)
    # goal_command_body[:, 0] is the X component of the unit direction vector in body frame
    cos_heading = goal_cmd.goal_command_body[:, 0]
    cos_heading = torch.clamp(cos_heading, min=-1.0, max=1.0)

    # Fade out heading alignment inside goal zone to prevent singularity spinning
    dist = torch.norm(goal_cmd._get_unscaled_command()[:, :2], dim=1)
    fade = torch.clamp((dist - 0.2) / max(distance_threshold - 0.2, 1e-4), min=0.0, max=1.0)
    return fade * cos_heading


def velocity_at_goal_penalty(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    command_name: str = "robot_goal",
    distance_threshold: float = 0.5,
) -> torch.Tensor:
    """Penalize base linear and angular velocity when inside the goal radius.

    Encourages the robot to decelerate and come to a complete, stable stop at the goal.
    """
    goal_cmd: RobotNavigationGoalCommand = env.command_manager.get_term(command_name)
    dist = torch.norm(goal_cmd._get_unscaled_command()[:, :2], dim=1)
    at_goal = (dist < distance_threshold).float()

    asset: Articulation = env.scene[asset_cfg.name]
    lin_vel_2d = torch.norm(asset.data.root_lin_vel_b[:, :2], dim=1)
    ang_vel_z = torch.abs(asset.data.root_ang_vel_b[:, 2])

    return at_goal * (lin_vel_2d + 0.5 * ang_vel_z)


def reach_goal_direct(
    env: ManagerBasedRLEnv, command_name: str = "robot_goal", distance_threshold: float = 0.5
) -> torch.Tensor:
    """Definite sparse bonus awarded when the robot enters the goal radius."""
    goal_cmd: RobotNavigationGoalCommand = env.command_manager.get_term(command_name)
    dist = torch.norm(goal_cmd._get_unscaled_command()[:, :2], dim=1)
    return (dist < distance_threshold).float()


def time_step_cost(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Constant per-step time cost to encourage efficient travel."""
    return torch.ones(env.num_envs, device=env.device)


def gated_progress_to_goal(
    env: ManagerBasedRLEnv,
    command_name: str = "robot_goal",
    power: float = 2.0,
    distance_threshold: float = 0.5,
) -> torch.Tensor:
    """Potential-based dense reward for progress toward the goal with cosine heading gating.

    Only awards positive progress when the robot's heading aligns with the goal direction.
    Penalizes moving away from the goal unconditionally (asymmetry protection).
    Fades out gating inside distance_threshold to prevent near-goal singularity jitter.
    """
    goal_cmd: RobotNavigationGoalCommand = env.command_manager.get_term(command_name)
    goal_in_body = goal_cmd._get_unscaled_command()
    current_dist = torch.norm(goal_in_body[:, :2], dim=1)

    if not hasattr(env, "_progress_prev_dist"):
        env._progress_prev_dist = current_dist.clone()
        return torch.zeros_like(current_dist)

    progress = env._progress_prev_dist - current_dist
    env._progress_prev_dist = current_dist.clone()

    just_reset = env.episode_length_buf <= 1
    if just_reset.any():
        progress[just_reset] = 0.0

    progress = torch.clamp(progress, min=-1.0, max=1.0) / env.step_dt

    # 2D heading cosine in robot body horizontal plane
    cos_heading = goal_in_body[:, 0] / torch.clamp(current_dist, min=1e-6)
    cos_heading = torch.clamp(cos_heading, min=-1.0, max=1.0)
    gating = torch.clamp(cos_heading, min=0.0) ** power

    # Fade out gating restriction near goal (dist < distance_threshold) to prevent singularity
    fade = torch.clamp((current_dist - 0.2) / max(distance_threshold - 0.2, 1e-4), min=0.0, max=1.0)
    effective_gating = fade * gating + (1.0 - fade) * 1.0

    # Decouple positive progress (gated) from negative progress (unconditionally penalized)
    pos_reward = torch.clamp(progress, min=0.0) * effective_gating
    neg_penalty = torch.clamp(progress, max=0.0)

    return pos_reward + neg_penalty


def heading_progress_to_goal(
    env: ManagerBasedRLEnv,
    command_name: str = "robot_goal",
    distance_threshold: float = 0.5,
) -> torch.Tensor:
    """Potential-based reward for reducing absolute heading error toward the goal.

    Awards positive reward for rotating toward the goal and penalizes turning away.
    Mathematically immune to infinite spinning exploits (PBRS conservative property).
    Fades out near goal to avoid singularity spinning.
    """
    goal_cmd: RobotNavigationGoalCommand = env.command_manager.get_term(command_name)
    goal_in_body = goal_cmd._get_unscaled_command()
    current_dist = torch.norm(goal_in_body[:, :2], dim=1)

    # Compute angle error in [-pi, pi]
    heading_error = torch.atan2(goal_in_body[:, 1], goal_in_body[:, 0])
    abs_heading_err = torch.abs(heading_error)

    if not hasattr(env, "_prev_abs_heading_err"):
        env._prev_abs_heading_err = abs_heading_err.clone()
        return torch.zeros_like(abs_heading_err)

    # Reduction in heading error (positive when turning toward goal)
    heading_progress = env._prev_abs_heading_err - abs_heading_err
    env._prev_abs_heading_err = abs_heading_err.clone()

    just_reset = env.episode_length_buf <= 1
    if just_reset.any():
        heading_progress[just_reset] = 0.0

    heading_progress = torch.clamp(heading_progress, min=-1.0, max=1.0) / env.step_dt

    # Fade out inside goal radius to prevent singularity jitter
    fade = torch.clamp((current_dist - 0.2) / max(distance_threshold - 0.2, 1e-4), min=0.0, max=1.0)

    # Only activate when there is a meaningful angle error (> 0.2 rad ~ 11.5 deg)
    active_mask = (abs_heading_err > 0.2).float()

    return fade * active_mask * heading_progress


def lateral_movement_l2(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    deadzone: float = 0.05,
) -> torch.Tensor:
    """Penalize lateral velocity using an L2 quadratic kernel with a small deadzone.

    Allows tiny natural lateral jitter/stepping (<= deadzone) without penalty,
    but imposes quadratic punishment on deliberate crab-walking / lateral drift.
    """
    asset: Articulation = env.scene[asset_cfg.name]
    lateral_velocity = torch.abs(asset.data.root_lin_vel_b[:, 1])
    excess_lat_vel = torch.clamp(lateral_velocity - deadzone, min=0.0)
    return torch.square(excess_lat_vel)


def _compute_realtime_yaw_error(
    env: ManagerBasedRLEnv, goal_cmd: RobotNavigationGoalCommand
) -> tuple[torch.Tensor, torch.Tensor]:
    """Compute real-time yaw error and cosine directly from robot quaternion without command lag."""
    if hasattr(goal_cmd, "goal_heading_world"):
        q = env.scene["robot"].data.root_quat_w
        robot_yaw = torch.atan2(2.0 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]), 1.0 - 2.0 * (q[:, 2] ** 2 + q[:, 3] ** 2))
        yaw_err = (goal_cmd.goal_heading_world - robot_yaw + math.pi) % (2.0 * math.pi) - math.pi
        return yaw_err, torch.cos(yaw_err)
    yaw_err = getattr(goal_cmd, "target_yaw_error", torch.zeros(env.num_envs, device=env.device))
    return yaw_err, getattr(goal_cmd, "target_yaw_cos", torch.ones_like(yaw_err))


def target_heading_alignment(
    env: ManagerBasedRLEnv,
    command_name: str = "robot_goal",
    distance_threshold: float = 1.0,
) -> torch.Tensor:
    """Reward for aligning with the final target heading when approaching the goal or in adjustment phase.

    Uses smooth distance gating: 0 when dist > distance_threshold, increasing to 1.0 as dist -> 0.
    Once the robot enters the goal circle and starts adjusting orientation, gate is locked at 1.0.
    Uses non-negative half-wave cosine mapping [0, 1]:
    - Exactly aligned (0 deg): 1.0
    - Perpendicular (90 deg): 0.5
    - Opposite (180 deg): 0.0
    """
    goal_cmd: RobotNavigationGoalCommand = env.command_manager.get_term(command_name)
    dist = torch.norm(goal_cmd._get_unscaled_command()[:, :2], dim=1)
    gate = torch.clamp(1.0 - dist / distance_threshold, min=0.0, max=1.0)
    is_adjusting = getattr(goal_cmd, "is_adjusting_heading", None)
    if is_adjusting is not None:
        gate = torch.where(is_adjusting, torch.ones_like(gate), gate)

    _, cos_yaw = _compute_realtime_yaw_error(env, goal_cmd)
    heading_reward = 0.5 * (1.0 + cos_yaw)
    return gate * heading_reward


def target_heading_terminal_outcome(
    env: ManagerBasedRLEnv,
    command_name: str = "robot_goal",
    success_bonus: float = 30.0,
    fail_penalty: float = 15.0,
) -> torch.Tensor:
    """Award terminal bonus for successful heading alignment and penalty for dwell timeout.

    Note: IsaacLab's RewardManager implicitly scales all term returns by env.step_dt (0.1s).
    To ensure that the agent receives genuine net return of +success_bonus and -fail_penalty,
    we divide by env.step_dt so that value = (bonus / dt) * dt = bonus.
    """
    goal_cmd: RobotNavigationGoalCommand = env.command_manager.get_term(command_name)
    outcome = getattr(goal_cmd, "heading_terminal_outcome", None)
    if outcome is None:
        return torch.zeros(env.num_envs, device=env.device)
    reward = torch.zeros_like(outcome)
    dt = getattr(env, "step_dt", 0.1)
    eff_success = success_bonus / max(1e-4, dt)
    eff_fail = fail_penalty / max(1e-4, dt)
    reward[outcome > 0.5] = eff_success
    reward[outcome < -0.5] = -eff_fail
    return reward


def target_yaw_progress(
    env: ManagerBasedRLEnv,
    command_name: str = "robot_goal",
    distance_threshold: float = 1.0,
) -> torch.Tensor:
    """Potential-based reward for reducing absolute target yaw error when approaching the goal.

    Awards positive reward when rotating toward the target heading and penalizes turning away.
    Smoothly gates in as dist < distance_threshold.
    Mathematically conservative (telescoping sum), immune to spinning exploits.
    """
    goal_cmd: RobotNavigationGoalCommand = env.command_manager.get_term(command_name)
    dist = torch.norm(goal_cmd._get_unscaled_command()[:, :2], dim=1)
    gate = torch.clamp(1.0 - dist / distance_threshold, min=0.0, max=1.0)

    yaw_err, _ = _compute_realtime_yaw_error(env, goal_cmd)
    abs_yaw_err = torch.abs(yaw_err)

    if not hasattr(env, "_prev_target_yaw_err"):
        env._prev_target_yaw_err = abs_yaw_err.clone()
        return torch.zeros_like(abs_yaw_err)

    progress = env._prev_target_yaw_err - abs_yaw_err
    env._prev_target_yaw_err = abs_yaw_err.clone()

    just_reset = env.episode_length_buf <= 1
    if just_reset.any():
        progress[just_reset] = 0.0

    progress = torch.clamp(progress, min=-1.0, max=1.0) / env.step_dt

    return gate * progress


def velocity_at_goal_heading_penalty(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    command_name: str = "robot_goal",
    distance_threshold: float = 0.5,
    heading_tolerance: float = 0.1745,
) -> torch.Tensor:
    """Penalize base velocity in goal circle, decoupling linear vs angular velocity.

    Always penalizes linear velocity inside distance_threshold to encourage stopping.
    Penalizes angular velocity ONLY when heading is already aligned within heading_tolerance,
    allowing the robot to freely rotate in place to align with the target heading.
    """
    goal_cmd: RobotNavigationGoalCommand = env.command_manager.get_term(command_name)
    dist = torch.norm(goal_cmd._get_unscaled_command()[:, :2], dim=1)
    at_goal = (dist < distance_threshold).float()

    asset: Articulation = env.scene[asset_cfg.name]
    lin_vel_2d = torch.norm(asset.data.root_lin_vel_b[:, :2], dim=1)
    ang_vel_z = torch.abs(asset.data.root_ang_vel_b[:, 2])

    yaw_err, _ = _compute_realtime_yaw_error(env, goal_cmd)
    heading_aligned = (torch.abs(yaw_err) < heading_tolerance).float()

    return at_goal * (lin_vel_2d + 0.5 * heading_aligned * ang_vel_z)


def velocity_at_goal_linear_penalty(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    command_name: str = "robot_goal",
    distance_threshold: float = 0.5,
    speed_deadzone: float = 0.2,
) -> torch.Tensor:
    """Penalize excessive linear velocity inside goal radius to promote stable braking.

    Leaves angular velocity completely unpenalized so the robot is free to rotate in place.
    """
    goal_cmd: RobotNavigationGoalCommand = env.command_manager.get_term(command_name)
    dist = torch.norm(goal_cmd._get_unscaled_command()[:, :2], dim=1)
    at_goal = (dist < distance_threshold).float()

    asset: Articulation = env.scene[asset_cfg.name]
    lin_vel_2d = torch.norm(asset.data.root_lin_vel_b[:, :2], dim=1)
    excess_lin_vel = torch.clamp(lin_vel_2d - speed_deadzone, min=0.0)

    return at_goal * excess_lin_vel



# Copyright (c) 2022-2025, Fan Yang and Per Frivik, ETH Zurich.
# All rights reserved.
#
# SPDX-License-Identifier: MIT

"""Common functions that can be used to activate certain terminations.

The functions can be passed to the :class:`isaaclab.managers.TerminationTermCfg` object to enable
the termination introduced by the function.
"""

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import ContactSensor
from isaaclab.utils.math import quat_inv, yaw_quat, quat_mul, euler_xyz_from_quat

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv
    from isaaclab_nav_task.navigation.mdp.navigation.goal_commands import RobotNavigationGoalCommand


def euler_xyz_from_quat_wrapped(quat: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    """Convert quaternion to Euler angles (XYZ convention) with wrapping to [-pi, pi].

    Args:
        quat: Quaternion tensor of shape (..., 4) in (w, x, y, z) format.

    Returns:
        Tuple of (roll, pitch, yaw) tensors.
    """
    roll, pitch, yaw = euler_xyz_from_quat(quat)
    # Wrap to [-pi, pi]
    roll = torch.remainder(roll + torch.pi, 2 * torch.pi) - torch.pi
    pitch = torch.remainder(pitch + torch.pi, 2 * torch.pi) - torch.pi
    yaw = torch.remainder(yaw + torch.pi, 2 * torch.pi) - torch.pi
    return roll, pitch, yaw


def time_out_navigation(
    env: "ManagerBasedRLEnv",
    goal_cmd_name: str = "robot_goal",
    distance_threshold: float = 0.5
) -> torch.Tensor:
    """Terminate the episode when the episode length exceeds the maximum episode length.

    This also tracks success metrics by checking if the robot reached the goal before timeout.
    """
    from isaaclab_nav_task.navigation.mdp.navigation.goal_commands import RobotNavigationGoalCommand

    goal_cmd_generator: RobotNavigationGoalCommand = env.command_manager._terms[goal_cmd_name]

    termination = env.episode_length_buf >= env.max_episode_length

    env_ids = torch.where(termination)[0]

    distance_goal = torch.norm(goal_cmd_generator._get_unscaled_command()[:, :2], dim=1)

    # update time at goal
    goal_cmd_generator.time_at_goal[distance_goal < distance_threshold] += 1 * env.step_dt

    if env_ids.numel() > 0:  # Check if env_ids is not empty
        success_masks = goal_cmd_generator.time_at_goal > 0.0
        value_buffer = torch.zeros_like(distance_goal)  # init with 0: Fail
        value_buffer[success_masks] = 1.0  # Success
        goal_cmd_generator.goal_reached_buffer.add(value_buffer, env_ids)

    return termination


def time_out_with_heading_navigation(
    env: "ManagerBasedRLEnv",
    goal_cmd_name: str = "robot_goal",
    distance_threshold: float = 0.4,
    heading_threshold: float = 0.1745,  # ~10 deg
) -> torch.Tensor:
    """Terminate the episode when timeout occurs, recording success only if BOTH distance and heading match."""
    from isaaclab_nav_task.navigation.mdp.navigation.goal_commands import RobotNavigationGoalCommand

    goal_cmd_generator: RobotNavigationGoalCommand = env.command_manager._terms[goal_cmd_name]

    termination = env.episode_length_buf >= env.max_episode_length

    env_ids = torch.where(termination)[0]

    distance_goal = torch.norm(goal_cmd_generator._get_unscaled_command()[:, :2], dim=1)
    yaw_err = getattr(goal_cmd_generator, "target_yaw_error", torch.zeros_like(distance_goal))

    # Keep track of time spent at goal with correct heading
    at_goal_with_heading = (distance_goal < distance_threshold) & (torch.abs(yaw_err) < heading_threshold)
    if not hasattr(goal_cmd_generator, "heading_time_at_goal"):
        goal_cmd_generator.heading_time_at_goal = torch.zeros(env.num_envs, device=env.device)

    goal_cmd_generator.heading_time_at_goal[at_goal_with_heading] += 1 * env.step_dt

    # Also update legacy time_at_goal for distance-based rewards if needed
    goal_cmd_generator.time_at_goal[distance_goal < distance_threshold] += 1 * env.step_dt

    if env_ids.numel() > 0:
        # Success is strictly awarded ONLY if heading was also satisfied at the goal!
        success_masks = goal_cmd_generator.heading_time_at_goal > 0.0
        value_buffer = torch.zeros_like(distance_goal)
        value_buffer[success_masks] = 1.0
        goal_cmd_generator.goal_reached_buffer.add(value_buffer, env_ids)
        goal_cmd_generator.heading_time_at_goal[env_ids] = 0.0

    return termination


def illegal_contact_navigation(
    env: "ManagerBasedRLEnv",
    threshold: float,
    sensor_cfg: SceneEntityCfg,
    goal_cmd_name: str = "robot_goal",
) -> torch.Tensor:
    """Terminate when the contact force on the sensor exceeds the force threshold."""
    from isaaclab_nav_task.navigation.mdp.navigation.goal_commands import RobotNavigationGoalCommand

    # extract the used quantities (to enable type-hinting)
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    net_contact_forces = contact_sensor.data.net_forces_w_history
    goal_cmd_generator: RobotNavigationGoalCommand = env.command_manager._terms[goal_cmd_name]

    termination = torch.any(
        torch.max(torch.norm(net_contact_forces[:, :, sensor_cfg.body_ids], dim=-1), dim=1)[0] > threshold, dim=1
    )

    env_ids = torch.where(termination)[0]

    if env_ids.numel() > 0:  # Check if env_ids is not empty
        goal_cmd_generator.goal_reached_buffer.add(torch.zeros_like(termination, dtype=torch.float), env_ids)

    return termination


def large_angle_termination_navigation(
    env: "ManagerBasedRLEnv",
    threshold: float,
    goal_cmd_name: str = "robot_goal",
) -> torch.Tensor:
    """Terminate when the robot exceeds a pitch or roll angle threshold."""
    from isaaclab_nav_task.navigation.mdp.navigation.goal_commands import RobotNavigationGoalCommand

    goal_cmd_generator: RobotNavigationGoalCommand = env.command_manager._terms[goal_cmd_name]

    # degree to rad
    threshold_rad = threshold * torch.pi / 180.0

    robot = env.scene["robot"]
    yaw_q = yaw_quat(robot.data.root_quat_w)
    base_quat_b = quat_mul(quat_inv(yaw_q), robot.data.root_quat_w)
    robot_roll, robot_pitch, _ = euler_xyz_from_quat_wrapped(base_quat_b)

    termination = torch.logical_or(torch.abs(robot_pitch) > threshold_rad, torch.abs(robot_roll) > threshold_rad)

    env_ids = torch.where(termination)[0]

    if env_ids.numel() > 0:  # Check if env_ids is not empty
        goal_cmd_generator.goal_reached_buffer.add(torch.zeros_like(termination, dtype=torch.float), env_ids)

    return termination


def at_goal_navigation(
    env: "ManagerBasedRLEnv",
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    distance_threshold: float = 0.5,
    goal_cmd_name: str = "robot_goal",
) -> torch.Tensor:
    """Terminate the episode when the goal is reached.

    Args:
        env: The learning environment.
        asset_cfg: The name of the robot asset.
        distance_threshold: The distance threshold to the goal.
        goal_cmd_name: The name of the goal command term.

    Returns:
        Boolean tensor indicating whether the goal is reached.
    """
    from isaaclab_nav_task.navigation.mdp.navigation.goal_commands import RobotNavigationGoalCommand

    # Extract the used quantities
    asset: Articulation = env.scene[asset_cfg.name]
    goal_cmd_generator: RobotNavigationGoalCommand = env.command_manager._terms.get(goal_cmd_name)

    # Calculate distance to goal
    xy_error = torch.norm(goal_cmd_generator._get_unscaled_command()[:, :2], dim=1)

    # Check conditions for termination
    at_goal = xy_error < distance_threshold

    # already at goal
    already_at_goal = goal_cmd_generator.time_at_goal > 0.0
    at_goal = torch.logical_or(at_goal, already_at_goal)

    # Update the time at goal in steps
    goal_cmd_generator.time_at_goal_in_steps[at_goal] += 1  # Increment if at goal

    # Determine if termination condition is met
    termination = goal_cmd_generator.time_at_goal_in_steps > goal_cmd_generator.required_time_at_goal_in_steps

    # Update goal reached buffer if termination condition is met
    env_ids = torch.where(termination)[0]
    if env_ids.numel() > 0:  # Check if any environments have met the termination condition
        goal_cmd_generator.goal_reached_buffer.add(torch.ones_like(termination, dtype=torch.float), env_ids)

    return termination


def at_goal_with_heading_navigation(
    env: "ManagerBasedRLEnv",
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    distance_threshold: float = 0.4,
    heading_threshold: float = 0.35,  # ~20 degrees
    goal_cmd_name: str = "robot_goal",
    velocity_threshold: float = 0.45,  # Relaxed linear velocity threshold for arrival
    dwell_time_range: tuple[float, float] = (5.0, 2.0),  # Curriculum: 5.0s -> 2.0s
    curriculum_success_range: tuple[float, float] = (0.2, 0.7),  # Dynamic interpolation based on success rate
    required_align_steps: int = 3,  # Consecutive steps required to confirm alignment (0.3s)
    fixed_dwell_time: float | None = 5.0,  # Fixed generous adjustment window (e.g. 5.0s = 50 steps), overrides dynamic shrinking
) -> torch.Tensor:
    """Terminate the episode when both distance and heading criteria are met at the goal (success),
    or when the arrival adjustment window expires without aligning heading (dwell failure).

    Curriculum: When fixed_dwell_time is None, the allowed adjustment window starts at dwell_time_range[0] (5.0s)
    and automatically scales down to dwell_time_range[1] (2.0s) as success rate rises.
    When fixed_dwell_time is provided (e.g. 5.0s), it provides a constant generous window for in-place braking & alignment.
    """
    from isaaclab_nav_task.navigation.mdp.navigation.goal_commands import RobotNavigationGoalCommand
    import math

    asset: Articulation = env.scene[asset_cfg.name]
    goal_cmd_generator: RobotNavigationGoalCommand = env.command_manager._terms.get(goal_cmd_name)

    # 1. Compute distance and real-time yaw error directly from quaternion
    xy_error = torch.norm(goal_cmd_generator._get_unscaled_command()[:, :2], dim=1)
    if hasattr(goal_cmd_generator, "goal_heading_world"):
        q = asset.data.root_quat_w
        robot_yaw = torch.atan2(2.0 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]), 1.0 - 2.0 * (q[:, 2] ** 2 + q[:, 3] ** 2))
        yaw_err = (goal_cmd_generator.goal_heading_world - robot_yaw + math.pi) % (2.0 * math.pi) - math.pi
    else:
        yaw_err = getattr(goal_cmd_generator, "target_yaw_error", torch.zeros_like(xy_error))
    lin_vel_2d = torch.norm(asset.data.root_lin_vel_b[:, :2], dim=1)

    # 2. Check circle entry and lock one-way adjustment state (cannot be reset by boundary jitter)
    in_circle = xy_error < distance_threshold
    if hasattr(goal_cmd_generator, "is_adjusting_heading"):
        goal_cmd_generator.is_adjusting_heading |= in_circle
        is_adjusting = goal_cmd_generator.is_adjusting_heading
    else:
        is_adjusting = in_circle

    # 3. Dynamic adjustment window duration or fixed generous window
    if fixed_dwell_time is not None and fixed_dwell_time > 0:
        allowed_time_s = fixed_dwell_time
    else:
        sr = torch.mean(goal_cmd_generator.success_tracker.get_success_rate()).item()
        sr_min, sr_max = curriculum_success_range
        progress = max(0.0, min(1.0, (sr - sr_min) / max(1e-4, sr_max - sr_min)))
        allowed_time_s = dwell_time_range[0] - progress * (dwell_time_range[0] - dwell_time_range[1])
    max_dwell_steps = max(10, int(allowed_time_s / env.step_dt))

    # 4. Advance dwell counter for environments currently adjusting
    if hasattr(goal_cmd_generator, "adjust_dwell_steps"):
        goal_cmd_generator.adjust_dwell_steps[is_adjusting] += 1
        curr_dwell = goal_cmd_generator.adjust_dwell_steps
    else:
        curr_dwell = torch.zeros(env.num_envs, device=env.device)

    # 5. Check heading alignment condition
    aligned = is_adjusting & (torch.abs(yaw_err) < heading_threshold) & (lin_vel_2d < velocity_threshold)
    if hasattr(goal_cmd_generator, "align_consecutive_steps"):
        goal_cmd_generator.align_consecutive_steps[aligned] += 1
        goal_cmd_generator.align_consecutive_steps[~aligned] = 0
        consecutive_aligned = goal_cmd_generator.align_consecutive_steps
    else:
        consecutive_aligned = torch.zeros(env.num_envs, device=env.device)

    # 6. Branch A: Success Termination
    success_termination = consecutive_aligned >= required_align_steps

    # 7. Branch B: Dwell Timeout Failure Termination
    dwell_termination = is_adjusting & (curr_dwell >= max_dwell_steps) & (~success_termination)

    # Handle outcomes (recorded in buffers for metrics and reward terms)
    success_ids = torch.where(success_termination)[0]
    if success_ids.numel() > 0:
        goal_cmd_generator.goal_reached_buffer.add(torch.ones_like(success_termination, dtype=torch.float), success_ids)
        if hasattr(goal_cmd_generator, "heading_terminal_outcome"):
            goal_cmd_generator.heading_terminal_outcome[success_ids] = 1.0

    dwell_fail_ids = torch.where(dwell_termination)[0]
    if dwell_fail_ids.numel() > 0:
        goal_cmd_generator.goal_reached_buffer.add(torch.zeros_like(dwell_termination, dtype=torch.float), dwell_fail_ids)
        if hasattr(goal_cmd_generator, "heading_terminal_outcome"):
            goal_cmd_generator.heading_terminal_outcome[dwell_fail_ids] = -1.0

    return success_termination | dwell_termination



def terrain_fall(
    env: "ManagerBasedRLEnv",
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    fall_height_threshold: float = -1.0,
    goal_cmd_name: str = "robot_goal",
) -> torch.Tensor:
    """Terminate when the robot falls below a certain height threshold.

    This termination is triggered when the robot's z-position falls below a
    specified threshold, indicating that the robot has fallen off the terrain
    or into a deep pit.

    Args:
        env: The learning environment.
        asset_cfg: The configuration for the robot asset.
        fall_height_threshold: The z-height below which the robot is considered fallen (in meters).
            Default is -1.0m to account for pits which can be ~1.5m deep.
        goal_cmd_name: The name of the goal command term.

    Returns:
        Boolean tensor indicating whether the robot has fallen.
    """
    # Direct tensor access for z-coordinate (avoids intermediate variable allocation)
    termination = env.scene[asset_cfg.name].data.root_pos_w[:, 2] < fall_height_threshold

    # Early exit if no terminations (common case - avoids torch.where overhead)
    if not termination.any():
        return termination

    # Update goal reached buffer with failure
    goal_cmd = env.command_manager._terms.get(goal_cmd_name)
    if goal_cmd is not None:
        env_ids = termination.nonzero(as_tuple=False).squeeze(-1)
        goal_cmd.goal_reached_buffer.add(torch.zeros(env.num_envs, dtype=torch.float, device=env.device), env_ids)

    return termination

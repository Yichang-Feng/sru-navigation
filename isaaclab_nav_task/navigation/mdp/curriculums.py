# Copyright (c) 2022-2025, Fan Yang and Per Frivik, ETH Zurich.
# All rights reserved.
#
# SPDX-License-Identifier: MIT

"""Curriculum functions for navigation tasks.

The functions can be passed to the :class:`isaaclab.managers.CurriculumTermCfg` object to enable
the curriculum introduced by the function.
"""
from __future__ import annotations

import os
from collections.abc import Sequence
from typing import TYPE_CHECKING

import torch

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def disable_backward_penalty_after_steps(
    env: ManagerBasedRLEnv, 
    env_ids: Sequence[int], 
    term_name: str = "backward_movement_penalty", 
    num_steps: int = 1000
) -> torch.Tensor:
    """Curriculum that disables the backward movement penalty after a certain number of steps.
    
    This helps with early training by preventing backward movement, but removes the constraint
    later to allow more natural movement patterns.

    Args:
        env: The learning environment.
        env_ids: Not used since all environments are affected.
        term_name: The name of the backward movement penalty term.
        num_steps: The number of steps after which the penalty should be disabled.
        
    Returns:
        Current step counter as float for logging purposes.
    """
    if env.common_step_counter > num_steps:
        # Check if the term exists and has a non-zero weight
        if hasattr(env.reward_manager, 'get_term_cfg'):
            try:
                term_cfg = env.reward_manager.get_term_cfg(term_name)
                if term_cfg.weight != 0.0:
                    # Disable the penalty by setting weight to 0
                    term_cfg.weight = 0.0
                    env.reward_manager.set_term_cfg(term_name, term_cfg)
                    print(f"Disabled backward movement penalty at step {env.common_step_counter}")
            except KeyError:
                # Term doesn't exist, which is fine
                pass
    
    return torch.tensor(float(env.common_step_counter))


def b2w_heading_curriculum_phase(
    env: ManagerBasedRLEnv,
    env_ids: Sequence[int],
    phase_iters: tuple[int, int, int] = (600, 700, 4500),
    phase_ratios: tuple[float, float, float] = (0.10, 0.1167, 0.75),
    total_steps: int = 144000,
) -> torch.Tensor:
    """4-Stage Curriculum for B2W Goal Heading Fine-tuning.

    - Phase 1 (iters 0 - 600, ~0-10%):
        align_weight = 0.0 (no heading reward), target_heading_terminal = 0.0.
        SRU, Attention, Depth Encoder FROZEN.
        early_termination heading_threshold is relaxed (3.14159 rad).
        Agent adapts MLP and 18-dim input without interfering with SRU.
    - Phase 1.5 (iters 600 - 700, 100 iters):
        align_weight = 0.0 (still NO heading reward), target_heading_terminal = 0.3333.
        All layers UNFROZEN (SRU adapts to gradient flow under small LR).
        Eliminates feature drift before introducing heading reward.
    - Phase 2 (iters 700 - 4500):
        align_weight = 2.0 (reward_yaw_dense active), target_heading_terminal = 0.3333.
        early_termination: distance_threshold = 0.4m, heading_threshold = 0.35 rad (~20 deg).
        velocity_threshold = 0.55 m/s, required_align_steps = 2.
        Allows the agent to explore turning and aligning with dense reward.
    - Phase 3 (iters 4500 - 6000):
        align_weight = 2.0, target_heading_terminal weight = 1.0 (dt-compensated Success Bonus).
        early_termination criteria tightened: distance_threshold = 0.3m, heading_threshold = 0.15 rad (~8.6 deg).
        velocity_threshold = 0.50 m/s, required_align_steps = 2.
    """
    phase_env = os.environ.get("B2W_HEADING_PHASE", "").strip()
    if phase_env:
        current_phase = float(phase_env)
        if current_phase <= 1.0:
            phase_name = "Phase 1 (Frozen SRU Review, align_weight=0)"
        elif current_phase <= 1.5:
            phase_name = "Phase 1.5 (Unfrozen SRU Adaptation, align_weight=0)"
        elif current_phase <= 2.0:
            phase_name = "Phase 2 (Dense Heading Alignment, align_weight=3.0)"
        else:
            phase_name = "Phase 3 (Strict Convergence & Terminal Bonus, align_weight=3.0, terminal=1.0)"
        progress = 1.0
    elif hasattr(env, "current_learning_iteration") and env.current_learning_iteration is not None:
        it = env.current_learning_iteration
        if it < phase_iters[0]:
            current_phase = 1.0
            phase_name = "Phase 1 (Frozen SRU Review, align_weight=0)"
        elif it < phase_iters[1]:
            current_phase = 1.5
            phase_name = "Phase 1.5 (Unfrozen SRU Adaptation, align_weight=0)"
        elif it < phase_iters[2]:
            current_phase = 2.0
            phase_name = "Phase 2 (Dense Heading Alignment, align_weight=2.0)"
        else:
            current_phase = 3.0
            phase_name = "Phase 3 (Strict Convergence & Terminal Bonus, align_weight=2.0, terminal=1.0)"
        progress = it / float(getattr(env, "max_learning_iterations", 6000) or 6000)
    else:
        progress = min(1.0, float(env.common_step_counter) / max(1.0, float(total_steps)))
        if progress < phase_ratios[0]:
            current_phase = 1.0
            phase_name = "Phase 1 (Frozen SRU Review, align_weight=0)"
        elif progress < phase_ratios[1]:
            current_phase = 1.5
            phase_name = "Phase 1.5 (Unfrozen SRU Adaptation, align_weight=0)"
        elif progress < phase_ratios[2]:
            current_phase = 2.0
            phase_name = "Phase 2 (Dense Heading Alignment, align_weight=2.0)"
        else:
            current_phase = 3.0
            phase_name = "Phase 3 (Strict Convergence & Terminal Bonus, align_weight=2.0, terminal=1.0)"

    prev_phase = getattr(env, "_current_b2w_heading_phase", None)
    if prev_phase != current_phase:
        env._current_b2w_heading_phase = current_phase
        it_display = getattr(env, 'current_learning_iteration', int(progress * 6000))
        print(f"\n[B2W Heading Curriculum] >>> Entering {phase_name} at Iteration {it_display} (Progress: {progress*100:.1f}%) <<<")

        # 1. Update Rewards
        if hasattr(env, "reward_manager"):
            try:
                term_heading = env.reward_manager.get_term_cfg("target_heading")
                if current_phase in (1.0, 1.5):
                    term_heading.weight = 0.0
                else:
                    term_heading.weight = 3.0  # Increased to 3.0 for decisive heading alignment incentive
                env.reward_manager.set_term_cfg("target_heading", term_heading)
            except KeyError:
                pass

            try:
                term_yaw_prog = env.reward_manager.get_term_cfg("target_yaw_progress")
                if current_phase in (1.0, 1.5):
                    term_yaw_prog.weight = 0.0
                else:
                    term_yaw_prog.weight = 1.5  # Directional angular velocity potential: awards rotating toward goal, penalizes turning away
                env.reward_manager.set_term_cfg("target_yaw_progress", term_yaw_prog)
            except KeyError:
                pass

            try:
                term_terminal = env.reward_manager.get_term_cfg("target_heading_terminal")
                if current_phase == 1.0:
                    term_terminal.weight = 0.0
                elif current_phase == 1.5:
                    term_terminal.weight = 0.3333  # Mild terminal outcome: net +10.0 on success, -5.0 on timeout
                else:  # Phase 2 & Phase 3
                    term_terminal.weight = 1.0     # Full terminal outcome: net +30.0 on success, -15.0 on timeout (kills lingering exploit)
                env.reward_manager.set_term_cfg("target_heading_terminal", term_terminal)
            except KeyError:
                pass

            try:
                term_vel = env.reward_manager.get_term_cfg("velocity_linear_penalty")
                if current_phase == 1.0:
                    term_vel.weight = 0.0
                else:
                    term_vel.weight = -2.0  # Decisive in-circle braking guidance (penalizes excess linear velocity, 0 penalty on ang_vel)
                env.reward_manager.set_term_cfg("velocity_linear_penalty", term_vel)
            except KeyError:
                pass

        # 2. Update Terminations
        if hasattr(env, "termination_manager"):
            try:
                term_early = env.termination_manager.get_term_cfg("early_termination")
                if current_phase in (1.0, 1.5):
                    term_early.params["distance_threshold"] = 0.4
                    term_early.params["heading_threshold"] = 3.14159
                    term_early.params["fixed_dwell_time"] = 5.0
                    term_early.params["velocity_threshold"] = 0.55
                    term_early.params["required_align_steps"] = 2
                elif current_phase == 2.0:
                    term_early.params["distance_threshold"] = 0.4
                    term_early.params["heading_threshold"] = 0.35
                    term_early.params["fixed_dwell_time"] = 5.0  # Generous 5.0s window for in-place turning exploration
                    term_early.params["velocity_threshold"] = 0.55
                    term_early.params["required_align_steps"] = 2
                else:  # Phase 3.0
                    term_early.params["distance_threshold"] = 0.3
                    term_early.params["heading_threshold"] = 0.15
                    term_early.params["fixed_dwell_time"] = 4.0  # Tightened 4.0s window for high-precision convergence
                    term_early.params["velocity_threshold"] = 0.50
                    term_early.params["required_align_steps"] = 2
                env.termination_manager.set_term_cfg("early_termination", term_early)
            except KeyError:
                pass

        terminal_weight_str = '0.0' if current_phase == 1.0 else ('0.33 (mild +10/-5)' if current_phase == 1.5 else '1.0 (full +30/-15)')
        vel_penalty_str = '0.0' if current_phase == 1.0 else '-2.0'
        yaw_prog_str = '0.0' if current_phase in (1.0, 1.5) else '1.5'
        dwell_s = 4.0 if current_phase == 3.0 else 5.0
        print(f"[B2W Heading Curriculum] Configured: "
              f"target_heading.weight={'0.0' if current_phase in (1.0, 1.5) else '3.0'}, "
              f"target_yaw_progress.weight={yaw_prog_str}, "
              f"target_heading_terminal.weight={terminal_weight_str}, "
              f"velocity_linear_penalty.weight={vel_penalty_str}, "
              f"early_termination thresholds=(dist:{0.3 if current_phase==3.0 else 0.4}m, heading:{0.15 if current_phase==3.0 else (0.35 if current_phase==2.0 else 3.14)}rad, dwell:{dwell_s}s)\n")

    return torch.tensor(float(current_phase), device=env.device)

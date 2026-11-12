# Copyright (c) 2022-2025, Fan Yang and Per Frivik, ETH Zurich.
# All rights reserved.
#
# SPDX-License-Identifier: MIT

"""B2W specific configuration for navigation environment."""

import os

from isaaclab.utils import configclass
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import CurriculumTermCfg as CurrTerm

from isaaclab_nav_task.navigation.navigation_env_cfg import NavigationEnvCfg, RewardsCfg, CurriculumCfg
import isaaclab_nav_task.navigation.mdp as mdp
from isaaclab_nav_task.navigation.config.go2w.navigation_env_cfg import ObservationsHeadingCfg

from isaaclab_nav_task.navigation.assets import B2W_CFG, ISAACLAB_NAV_TASKS_ASSETS_DIR  # isort: skip


LEG_JOINT_NAMES = [".*hip_joint", ".*thigh_joint", ".*calf_joint"]
WHEEL_JOINT_NAMES = [".*foot_joint"]

@configclass
class B2WNavigationEnvCfg(NavigationEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        from isaaclab_nav_task.navigation.mdp.observations import initialize_depth_noise_generator
        from isaaclab_nav_task.navigation.mdp.depth_utils.camera_config import get_camera_config

        initialize_depth_noise_generator(robot_name="b2w", use_jit_precompiled=False)

        camera_config = get_camera_config("b2w")
        CAMERA_RESOLUTION = camera_config.resolution

        self.scene.robot = B2W_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        self.scene.raycast_camera.prim_path = "{ENV_REGEX_NS}/Robot/base_link"
        self.scene.raycast_camera.offset.pos = (0.387, 0.0, 0.28)
        self.scene.height_scanner_critic.prim_path = "{ENV_REGEX_NS}/Robot/base_link"

        self.terminations.base_contact.params = {"sensor_cfg": SceneEntityCfg("contact_forces", body_names=["base_link", ".*hip", ".*thigh"]), "threshold": 1.0}

        self.actions.velocity_command.low_level_position_action = mdp.JointPositionActionCfg(asset_name="robot", joint_names=[".*hip_joint", ".*thigh_joint", ".*calf_joint"], scale=0.5, use_default_offset=True)
        self.actions.velocity_command.low_level_velocity_action = mdp.JointVelocityActionCfg(asset_name="robot", joint_names=[".*foot_joint"], scale=5.0, use_default_offset=True)
        self.actions.velocity_command.low_level_policy_file = os.path.join(ISAACLAB_NAV_TASKS_ASSETS_DIR, "Policies", "locomotion", "b2w", "policy_b2w_new_2.pt")

        self.rewards.joint_acc_l2_joint.params = {"asset_cfg": SceneEntityCfg("robot", joint_names=LEG_JOINT_NAMES+WHEEL_JOINT_NAMES)}

        self.terminations.base_contact.params = {"sensor_cfg": SceneEntityCfg("contact_forces", body_names=["base_link", ".*hip", ".*thigh"]), "threshold": 1.0}

        self.events.randomize_low_pass_filter_alpha.params = {
            "alpha_range": (0.1, 0.6),
            "action_term": "velocity_command",
            "per_dimension": True,
            "alpha_range_vx": (0.1, 0.6),
            "alpha_range_vy": (0.1, 0.6),
            "alpha_range_omega": (0.1, 0.6),
        }

        self.commands.robot_goal.debug_vis = False
        self.commands.robot_goal.goal_height_offset_range = (0.2, 0.6)

        self.scene.terrain.max_init_terrain_level = 10
        self.scene.terrain.terrain_generator.difficulty_range = [0.5, 1.0]
        self.scene.terrain.terrain_generator.curriculum = False

@configclass
class B2WNavigationEnvCfg_DEV(B2WNavigationEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.terrain.terrain_generator.num_rows = 2
        self.scene.terrain.terrain_generator.num_cols = 30
        self.scene.terrain.max_init_terrain_level = 10
        self.scene.terrain.terrain_generator.difficulty_range = [0.5, 1.0]
        self.scene.terrain.terrain_generator.curriculum = False

@configclass
class B2WNavigationEnvCfg_PLAY(B2WNavigationEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        self.scene.num_envs = 20
        self.scene.env_spacing = 2.5
        self.scene.terrain.max_init_terrain_level = None
        if self.scene.terrain.terrain_generator is not None:
            self.scene.terrain.terrain_generator.num_rows = 2
            self.scene.terrain.terrain_generator.num_cols = 2

        self.observations.policy.enable_corruption = False
        self.events.base_external_force_torque = None
        self.events.push_robot = None


# =============================================================================
# B2W Goal Heading Fine-Tuning Task Configuration
# =============================================================================

@configclass
class B2WHeadingRewardsCfg(RewardsCfg):
    """Reward terms for B2W arrival heading control (Official B2W baseline + 3-Phase curriculum)."""

    # 1. 到达目标点机身朝向对齐过程引导 (Phase 1 为 0.0，Phase 2 & 3 开启为 2.0)
    target_heading = RewTerm(
        func=mdp.target_heading_alignment,
        weight=0.0,
        params={"command_name": "robot_goal", "distance_threshold": 1.0},
    )

    # 2. 到达目标点朝向终末结算大奖与超时惩罚 (Phase 1 & 1.5 为 0.0，Phase 2 温和开启 0.33，Phase 3 开启 1.0)
    # success_bonus 经过 dt 补偿后到手净值为 +30.0，fail_penalty 净值为 -15.0
    target_heading_terminal = RewTerm(
        func=mdp.target_heading_terminal_outcome,
        weight=0.0,
        params={"command_name": "robot_goal", "success_bonus": 30.0, "fail_penalty": 15.0},
    )

    # 3. 终点圈内超额线速度惩罚 (Phase 1.5, 2, 3 激活，对原地自旋角速度 0 惩罚，促进入圈主动刹车)
    velocity_linear_penalty = RewTerm(
        func=mdp.velocity_at_goal_linear_penalty,
        weight=0.0,
        params={"command_name": "robot_goal", "distance_threshold": 0.4, "speed_deadzone": 0.2},
    )

    # 4. 到达目标点机身朝向角速度势能引导 (Phase 2 & 3 开启为 1.5，主动旋转调头直接奖励，逆向旋转直接扣分)
    target_yaw_progress = RewTerm(
        func=mdp.target_yaw_progress,
        weight=0.0,
        params={"command_name": "robot_goal", "distance_threshold": 1.0},
    )


@configclass
class B2WHeadingCurriculumCfg(CurriculumCfg):
    """Curriculum terms for B2W arrival heading fine-tuning."""

    disable_backward_penalty = None
    heading_phase = CurrTerm(
        func=mdp.b2w_heading_curriculum_phase,
        params={"phase_iters": (600, 700, 4500), "phase_ratios": (0.10, 0.1167, 0.75), "total_steps": 144000},
    )


@configclass
class B2WNavigationEnvCfg_Heading(B2WNavigationEnvCfg):
    """B2W navigation environment for arrival heading control fine-tuning."""

    rewards: B2WHeadingRewardsCfg = B2WHeadingRewardsCfg()
    observations: ObservationsHeadingCfg = ObservationsHeadingCfg()
    curriculum: B2WHeadingCurriculumCfg = B2WHeadingCurriculumCfg()

    def __post_init__(self):
        super().__post_init__()
        self.rewards = B2WHeadingRewardsCfg()
        self.observations = ObservationsHeadingCfg()
        self.curriculum = B2WHeadingCurriculumCfg()
        self.rewards.joint_acc_l2_joint.params = {
            "asset_cfg": SceneEntityCfg("robot", joint_names=LEG_JOINT_NAMES + WHEEL_JOINT_NAMES)
        }
        # 终点位置饱和平台保护 (0.4m)：四轮足原地踏步自旋时机身 0.15m 抖动不扣位置分
        self.rewards.reach_goal_xy_tight.params["plateau_radius"] = 0.4

        # Heading-aware arrival termination:
        # Phase 1: 0.4m distance, heading unconstrained (3.14 rad), fixed 5.0s window for arrival review
        # Phase 2: 0.4m distance, 0.35 rad (~20 deg), fixed 5.0s window for in-place turning exploration
        # Phase 3: 0.3m distance, 0.15 rad (~8.6 deg) tightened, 4.0s window
        self.terminations.early_termination.func = mdp.at_goal_with_heading_navigation
        self.terminations.early_termination.params = {
            "distance_threshold": 0.4,
            "heading_threshold": 3.14159,  # Initial Phase 1: unconstrained heading for arrival review
            "goal_cmd_name": "robot_goal",
            "velocity_threshold": 0.55,  # Relaxed for quadruped suspension balancing jitter
            "fixed_dwell_time": 5.0,  # Fixed generous 5.0s (50 steps) window, no aggressive shrinking
            "required_align_steps": 2,  # 2 consecutive steps (0.4s) to prevent single-step jitter reset
        }
        # Heading-aware timeout termination: only counts success if heading was aligned
        self.terminations.time_out.func = mdp.time_out_with_heading_navigation
        self.terminations.time_out.params = {
            "distance_threshold": 0.4,
            "heading_threshold": 0.35,  # ~20 degrees
            "goal_cmd_name": "robot_goal",
        }
        if hasattr(self.observations.metrics, "in_goal"):
            self.observations.metrics.in_goal.params["heading_threshold"] = 0.35
        # Allow terrain level override for fine-tuning
        init_level = os.environ.get("B2W_INIT_LEVEL", "").strip()
        if init_level:
            self.scene.terrain.max_init_terrain_level = int(init_level)


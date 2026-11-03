# Copyright (c) 2022-2025, Fan Yang and Per Frivik, ETH Zurich.
# All rights reserved.
#
# SPDX-License-Identifier: MIT

"""B2W specific configuration for navigation environment."""

import os

from isaaclab.utils import configclass
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import RewardTermCfg as RewTerm

from isaaclab_nav_task.navigation.navigation_env_cfg import NavigationEnvCfg, RewardsCfg
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
    """Reward terms for B2W arrival heading control."""

    # 1. 到达目标点机身朝向对齐门控奖励 (d <= 1.0m 时平滑开启，d->0 时权重为 1.0)
    target_heading = RewTerm(
        func=mdp.target_heading_alignment,
        weight=1.0,
        params={"command_name": "robot_goal", "distance_threshold": 1.0},
    )


@configclass
class B2WNavigationEnvCfg_Heading(B2WNavigationEnvCfg):
    """B2W navigation environment for arrival heading control fine-tuning."""

    rewards: B2WHeadingRewardsCfg = B2WHeadingRewardsCfg()
    observations: ObservationsHeadingCfg = ObservationsHeadingCfg()

    def __post_init__(self):
        super().__post_init__()
        self.rewards = B2WHeadingRewardsCfg()
        self.observations = ObservationsHeadingCfg()
        self.rewards.joint_acc_l2_joint.params = {
            "asset_cfg": SceneEntityCfg("robot", joint_names=LEG_JOINT_NAMES + WHEEL_JOINT_NAMES)
        }
        # Heading-aware arrival termination: requires both distance < 0.4m and heading error < 10 deg (0.1745 rad)
        self.terminations.early_termination.func = mdp.at_goal_with_heading_navigation
        self.terminations.early_termination.params = {
            "distance_threshold": 0.4,
            "heading_threshold": 0.1745,  # ~10 degrees
            "goal_cmd_name": "robot_goal",
        }
        # Heading-aware timeout termination: only counts success if heading was aligned
        self.terminations.time_out.func = mdp.time_out_with_heading_navigation
        self.terminations.time_out.params = {
            "distance_threshold": 0.4,
            "heading_threshold": 0.1745,  # ~10 degrees
            "goal_cmd_name": "robot_goal",
        }
        # Allow terrain level override for fine-tuning
        init_level = os.environ.get("B2W_INIT_LEVEL", "").strip()
        if init_level:
            self.scene.terrain.max_init_terrain_level = int(init_level)


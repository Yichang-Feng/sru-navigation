# Copyright (c) 2022-2026, Fan Yang and Per Frivik, ETH Zurich.
# All rights reserved.
#
# SPDX-License-Identifier: MIT

"""Gym environment registration for Go2W navigation tasks (SRU-Odin 5-Phase Curriculum)."""

import gymnasium as gym

from . import agents, navigation_env_cfg

##
# Register Gym environments.
##

# =============================================================================
# MDPO 5-Phase Environments (Primary Algorithm)
# =============================================================================

# Phase 1: Cold Start (0 -> 2,000 iter, flat ground + sparse obstacles)
gym.register(
    id="Isaac-Nav-MDPO-Go2W-Phase1-v0",
    entry_point="isaaclab_nav_task.navigation:NavigationEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": navigation_env_cfg.Go2WNavigationEnvCfg_Phase1,
        "rsl_rl_cfg_entry_point": agents.rsl_rl_cfg.Go2WNavMDPORunnerPhase1Cfg,
    },
)

# Phase 2: Stabilization & precision stopping (2,000 -> 5,000 iter)
gym.register(
    id="Isaac-Nav-MDPO-Go2W-Phase2-v0",
    entry_point="isaaclab_nav_task.navigation:NavigationEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": navigation_env_cfg.Go2WNavigationEnvCfg_Phase2,
        "rsl_rl_cfg_entry_point": agents.rsl_rl_cfg.Go2WNavMDPORunnerPhase2Cfg,
    },
)

# Phase 3: Stairs (10-15cm) & moderate mazes (5,000 -> 14,000 iter)
gym.register(
    id="Isaac-Nav-MDPO-Go2W-Phase3-v0",
    entry_point="isaaclab_nav_task.navigation:NavigationEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": navigation_env_cfg.Go2WNavigationEnvCfg_Phase3,
        "rsl_rl_cfg_entry_point": agents.rsl_rl_cfg.Go2WNavMDPORunnerPhase3Cfg,
    },
)

# Phase 4: Full 20cm stairs, pits, and dense mazes (14,000 -> 24,000 iter)
gym.register(
    id="Isaac-Nav-MDPO-Go2W-Phase4-v0",
    entry_point="isaaclab_nav_task.navigation:NavigationEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": navigation_env_cfg.Go2WNavigationEnvCfg_Phase4,
        "rsl_rl_cfg_entry_point": agents.rsl_rl_cfg.Go2WNavMDPORunnerPhase4Cfg,
    },
)

# Phase 5: Trajectory smoothing & fine refinement (24,000+ iter)
gym.register(
    id="Isaac-Nav-MDPO-Go2W-Phase5-v0",
    entry_point="isaaclab_nav_task.navigation:NavigationEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": navigation_env_cfg.Go2WNavigationEnvCfg_Phase5,
        "rsl_rl_cfg_entry_point": agents.rsl_rl_cfg.Go2WNavMDPORunnerPhase5Cfg,
    },
)

# Default Standard Full-Complexity Task (Phase 4)
gym.register(
    id="Isaac-Nav-MDPO-Go2W-v0",
    entry_point="isaaclab_nav_task.navigation:NavigationEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": navigation_env_cfg.Go2WNavigationEnvCfg_Phase4,
        "rsl_rl_cfg_entry_point": agents.rsl_rl_cfg.Go2WNavMDPORunnerPhase4Cfg,
    },
)

# PureMaze Refinement Task
gym.register(
    id="Isaac-Nav-MDPO-Go2W-PureMaze-v0",
    entry_point="isaaclab_nav_task.navigation:NavigationEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": navigation_env_cfg.Go2WNavigationEnvCfg_PureMaze,
        "rsl_rl_cfg_entry_point": agents.rsl_rl_cfg.Go2WNavMDPORunnerPureMazeCfg,
    },
)

# Play/Evaluation Task
gym.register(
    id="Isaac-Nav-MDPO-Go2W-Play-v0",
    entry_point="isaaclab_nav_task.navigation:NavigationEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": navigation_env_cfg.Go2WNavigationEnvCfg_PLAY,
        "rsl_rl_cfg_entry_point": agents.rsl_rl_cfg.Go2WNavMDPORunnerPhase4Cfg,
    },
)

# Development/Debug Task
gym.register(
    id="Isaac-Nav-MDPO-Go2W-Dev-v0",
    entry_point="isaaclab_nav_task.navigation:NavigationEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": navigation_env_cfg.Go2WNavigationEnvCfg_DEV,
        "rsl_rl_cfg_entry_point": agents.rsl_rl_cfg.Go2WNavMDPORunnerDevCfg,
    },
)

# Backward Compatibility Stage Aliases
gym.register(
    id="Isaac-Nav-MDPO-Go2W-Stage1-v0",
    entry_point="isaaclab_nav_task.navigation:NavigationEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": navigation_env_cfg.Go2WNavigationEnvCfg_Phase1,
        "rsl_rl_cfg_entry_point": agents.rsl_rl_cfg.Go2WNavMDPORunnerPhase1Cfg,
    },
)

gym.register(
    id="Isaac-Nav-MDPO-Go2W-Stage2-v0",
    entry_point="isaaclab_nav_task.navigation:NavigationEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": navigation_env_cfg.Go2WNavigationEnvCfg_Phase3,
        "rsl_rl_cfg_entry_point": agents.rsl_rl_cfg.Go2WNavMDPORunnerPhase3Cfg,
    },
)


# =============================================================================
# PPO 5-Phase Environments (Baseline Algorithm)
# =============================================================================

# Phase 1: Cold Start (0 -> 2,000 iter)
gym.register(
    id="Isaac-Nav-PPO-Go2W-Phase1-v0",
    entry_point="isaaclab_nav_task.navigation:NavigationEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": navigation_env_cfg.Go2WNavigationEnvCfg_Phase1,
        "rsl_rl_cfg_entry_point": agents.rsl_rl_cfg.Go2WNavPPORunnerPhase1Cfg,
    },
)

# Phase 2: Stabilization & precision stopping (2,000 -> 5,000 iter)
gym.register(
    id="Isaac-Nav-PPO-Go2W-Phase2-v0",
    entry_point="isaaclab_nav_task.navigation:NavigationEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": navigation_env_cfg.Go2WNavigationEnvCfg_Phase2,
        "rsl_rl_cfg_entry_point": agents.rsl_rl_cfg.Go2WNavPPORunnerPhase2Cfg,
    },
)

# Phase 3: Stairs (10-15cm) & moderate mazes (5,000 -> 14,000 iter)
gym.register(
    id="Isaac-Nav-PPO-Go2W-Phase3-v0",
    entry_point="isaaclab_nav_task.navigation:NavigationEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": navigation_env_cfg.Go2WNavigationEnvCfg_Phase3,
        "rsl_rl_cfg_entry_point": agents.rsl_rl_cfg.Go2WNavPPORunnerPhase3Cfg,
    },
)

# Phase 4: Full 20cm stairs, pits, and dense mazes (14,000 -> 24,000 iter)
gym.register(
    id="Isaac-Nav-PPO-Go2W-Phase4-v0",
    entry_point="isaaclab_nav_task.navigation:NavigationEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": navigation_env_cfg.Go2WNavigationEnvCfg_Phase4,
        "rsl_rl_cfg_entry_point": agents.rsl_rl_cfg.Go2WNavPPORunnerPhase4Cfg,
    },
)

# Phase 5: Trajectory smoothing & fine refinement (24,000+ iter)
gym.register(
    id="Isaac-Nav-PPO-Go2W-Phase5-v0",
    entry_point="isaaclab_nav_task.navigation:NavigationEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": navigation_env_cfg.Go2WNavigationEnvCfg_Phase5,
        "rsl_rl_cfg_entry_point": agents.rsl_rl_cfg.Go2WNavPPORunnerPhase5Cfg,
    },
)

# Default Standard Full-Complexity Task (Phase 4)
gym.register(
    id="Isaac-Nav-PPO-Go2W-v0",
    entry_point="isaaclab_nav_task.navigation:NavigationEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": navigation_env_cfg.Go2WNavigationEnvCfg_Phase4,
        "rsl_rl_cfg_entry_point": agents.rsl_rl_cfg.Go2WNavPPORunnerPhase4Cfg,
    },
)

# PureMaze Refinement Task
gym.register(
    id="Isaac-Nav-PPO-Go2W-PureMaze-v0",
    entry_point="isaaclab_nav_task.navigation:NavigationEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": navigation_env_cfg.Go2WNavigationEnvCfg_PureMaze,
        "rsl_rl_cfg_entry_point": agents.rsl_rl_cfg.Go2WNavPPORunnerPureMazeCfg,
    },
)

# Play/Evaluation Task
gym.register(
    id="Isaac-Nav-PPO-Go2W-Play-v0",
    entry_point="isaaclab_nav_task.navigation:NavigationEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": navigation_env_cfg.Go2WNavigationEnvCfg_PLAY,
        "rsl_rl_cfg_entry_point": agents.rsl_rl_cfg.Go2WNavPPORunnerPhase4Cfg,
    },
)

# Development/Debug Task
gym.register(
    id="Isaac-Nav-PPO-Go2W-Dev-v0",
    entry_point="isaaclab_nav_task.navigation:NavigationEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": navigation_env_cfg.Go2WNavigationEnvCfg_DEV,
        "rsl_rl_cfg_entry_point": agents.rsl_rl_cfg.Go2WNavPPORunnerDevCfg,
    },
)

# Backward Compatibility Stage Aliases
gym.register(
    id="Isaac-Nav-PPO-Go2W-Stage1-v0",
    entry_point="isaaclab_nav_task.navigation:NavigationEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": navigation_env_cfg.Go2WNavigationEnvCfg_Phase1,
        "rsl_rl_cfg_entry_point": agents.rsl_rl_cfg.Go2WNavPPORunnerPhase1Cfg,
    },
)

gym.register(
    id="Isaac-Nav-PPO-Go2W-Stage2-v0",
    entry_point="isaaclab_nav_task.navigation:NavigationEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": navigation_env_cfg.Go2WNavigationEnvCfg_Phase3,
        "rsl_rl_cfg_entry_point": agents.rsl_rl_cfg.Go2WNavPPORunnerPhase3Cfg,
    },
)

# Warm-Start Fine-Tuning Task (Porting from B2W, strictly aligned with SRU-Odin)
gym.register(
    id="Isaac-Nav-PPO-Go2W-FineTune-v0",
    entry_point="isaaclab_nav_task.navigation:NavigationEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": navigation_env_cfg.Go2WNavigationEnvCfg_Phase1,
        "rsl_rl_cfg_entry_point": agents.rsl_rl_cfg.Go2WNavPPOFineTuneRunnerCfg,
    },
)

# 40-80cm Low Obstacle Fine-Tuning Task (Real-aligned camera: body center +10cm, pitch=0)
gym.register(
    id="Isaac-Nav-PPO-Go2W-LowObs-v0",
    entry_point="isaaclab_nav_task.navigation:NavigationEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": navigation_env_cfg.Go2WLowObsNavigationEnvCfg,
        "rsl_rl_cfg_entry_point": agents.rsl_rl_cfg.Go2WNavPPOLowObsRunnerCfg,
    },
)

# 40-80cm Low Obstacle Evaluation/Play Task
gym.register(
    id="Isaac-Nav-PPO-Go2W-LowObs-Play-v0",
    entry_point="isaaclab_nav_task.navigation:NavigationEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": navigation_env_cfg.Go2WLowObsNavigationEnvCfg_PLAY,
        "rsl_rl_cfg_entry_point": agents.rsl_rl_cfg.Go2WNavPPOLowObsRunnerCfg,
    },
)

# Arrival Heading Control Fine-Tuning Tasks (MDPO & PPO)
gym.register(
    id="Isaac-Nav-MDPO-Go2W-Heading-v0",
    entry_point="isaaclab_nav_task.navigation:NavigationEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": navigation_env_cfg.Go2WNavigationEnvCfg_Heading,
        "rsl_rl_cfg_entry_point": agents.rsl_rl_cfg.Go2WNavMDPOHeadingRunnerCfg,
    },
)

gym.register(
    id="Isaac-Nav-PPO-Go2W-Heading-v0",
    entry_point="isaaclab_nav_task.navigation:NavigationEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": navigation_env_cfg.Go2WNavigationEnvCfg_Heading,
        "rsl_rl_cfg_entry_point": agents.rsl_rl_cfg.Go2WNavPPOHeadingRunnerCfg,
    },
)



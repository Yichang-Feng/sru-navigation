# Copyright (c) 2022-2025, Fan Yang and Per Frivik, ETH Zurich.
# All rights reserved.
#
# SPDX-License-Identifier: MIT

"""Maze terrain generation for navigation tasks.

This module provides:
- Maze terrain generation with various obstacle types
- Stair/platform terrain for elevation training
- Pit terrain with negative obstacles
- Monkey-patches for Isaac Lab terrain system integration
- Optimized mesh generation for reduced GPU memory

Key files:
- terrain_constants.py: Centralized height values and thresholds
- hf_terrains_maze.py: Terrain generation functions
- hf_terrains_maze_cfg.py: Terrain configurations
- patches.py: Isaac Lab integration patches
"""

# Patches first (no Isaac Sim required)
from .patches import apply_terrain_patches

# Constants (no Isaac Sim required)
from .terrain_constants import (
    HEIGHTS,
    THRESHOLDS,
    PADDING,
    STAIRS,
    HeightValues,
    SamplingThresholds,
    VERTICAL_SCALE,
    HORIZONTAL_SCALE,
    CELL_SIZE,
    CELL_PIXELS,
)

# Terrain generation (requires Isaac Sim)
from .hf_terrains_maze_cfg import HfMazeTerrainCfg
from .hf_terrains_maze import maze_terrain
from .maze_config import MAZE_TERRAIN_CFG
from .maze_low_obs_config import LOW_OBS_MAZE_TERRAIN_CFG
from .go2w_stage_terrains import (
    GO2W_PHASE1_TERRAIN_CFG,
    GO2W_PHASE2_TERRAIN_CFG,
    GO2W_PHASE3_TERRAIN_CFG,
    GO2W_PHASE4_TERRAIN_CFG,
    GO2W_PHASE5_TERRAIN_CFG,
    GO2W_STAGE1_TERRAIN_CFG,
    GO2W_STAGE2_TERRAIN_CFG,
    GO2W_STAGE3_TERRAIN_CFG,
    apply_go2w_terrain_env_overrides,
)


__all__ = [
    # Patches
    "apply_terrain_patches",
    # Constants
    "HEIGHTS",
    "THRESHOLDS",
    "PADDING",
    "STAIRS",
    "HeightValues",
    "SamplingThresholds",
    "VERTICAL_SCALE",
    "HORIZONTAL_SCALE",
    "CELL_SIZE",
    "CELL_PIXELS",
    # Terrain configs
    "HfMazeTerrainCfg",
    "MAZE_TERRAIN_CFG",
    "LOW_OBS_MAZE_TERRAIN_CFG",
    "GO2W_PHASE1_TERRAIN_CFG",
    "GO2W_PHASE2_TERRAIN_CFG",
    "GO2W_PHASE3_TERRAIN_CFG",
    "GO2W_PHASE4_TERRAIN_CFG",
    "GO2W_PHASE5_TERRAIN_CFG",
    "GO2W_STAGE1_TERRAIN_CFG",
    "GO2W_STAGE2_TERRAIN_CFG",
    "GO2W_STAGE3_TERRAIN_CFG",
    "apply_go2w_terrain_env_overrides",
    # Terrain functions
    "maze_terrain",
]


# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: MIT

"""Phase-specific terrain configurations for Go2W navigation curriculum learning (SRU-Odin 5-Phase).

Phase 1 (0 -> 2,000 iter): Flat ground + sparse obstacles (establishing straight forward motion & basic avoidance)
Phase 2 (2,000 -> 5,000 iter): Flat ground + sparse obstacles with auto-curriculum promotion
Phase 3 (5,000 -> 14,000 iter): Moderate mazes + 10-15cm stairs (moderate step climbing & cornering)
Phase 4 (14,000 -> 24,000 iter): Full complexity maze + 20cm stairs + pits (deep pit avoidance & 20cm step climbing)
Phase 5 (24,000+ iter): Full complexity refinement
"""

import os
import copy
from isaaclab.terrains.terrain_generator_cfg import TerrainGeneratorCfg
from .hf_terrains_maze_cfg import HfMazeTerrainCfg


# =============================================================================
# Helper function for dynamic environment variable overrides
# =============================================================================

def apply_go2w_terrain_env_overrides(terrain_cfg: TerrainGeneratorCfg, default_diff: tuple[float, float] = (0.0, 0.4)) -> TerrainGeneratorCfg:
    """Apply environment variable overrides (GO2W_DIFFICULTY, GO2W_CELL_SIZE) to a terrain generator config."""
    cfg = copy.deepcopy(terrain_cfg)

    # Difficulty override
    _diff = os.environ.get("GO2W_DIFFICULTY", "").strip()
    if _diff:
        parts = _diff.replace(" ", "").split(",")
        cfg.difficulty_range = (float(parts[0]), float(parts[1]))
    else:
        cfg.difficulty_range = default_diff

    # Cell size override (affects corridor width and tile size)
    _cell = os.environ.get("GO2W_CELL_SIZE", "").strip()
    if _cell:
        new_cell = float(_cell)
        for sub in cfg.sub_terrains.values():
            sub.cell_size = new_cell
        tile_m = 15 * new_cell
        cfg.size = (tile_m, tile_m)

    return cfg


# =============================================================================
# Phase 1: Cold Start (Flat ground + sparse obstacles, [0.0, 0.4])
# =============================================================================

GO2W_PHASE1_TERRAIN_CFG = TerrainGeneratorCfg(
    size=(30.0, 30.0),
    border_width=30.0,
    num_rows=6,
    num_cols=30,
    horizontal_scale=0.1,
    vertical_scale=0.005,
    slope_threshold=0.75,
    use_cache=False,
    curriculum=True,
    difficulty_range=(0.0, 0.4),
    sub_terrains={
        "non_maze": HfMazeTerrainCfg(
            proportion=0.8,
            open_probability=0.95,
            grid_size=(15, 15),
            cell_size=2.0,
            add_noise_to_flat=False,
            add_goal=True,
            randomize_wall=True,
            random_wall_ratio=0.8,
            non_maze_terrain=True,
        ),
        "maze_simple": HfMazeTerrainCfg(
            proportion=0.2,
            open_probability=0.95,
            grid_size=(15, 15),
            cell_size=2.0,
            add_noise_to_flat=False,
            add_goal=True,
            randomize_wall=True,
            random_wall_ratio=0.3,
            add_stairs_to_maze=False,
        ),
    },
)
"""Go2W Phase 1 Terrain: Flat ground with sparse obstacles for cold-start navigation."""


# =============================================================================
# Phase 2: Stabilization (Same layout as Phase 1, auto-promotes with curriculum)
# =============================================================================

GO2W_PHASE2_TERRAIN_CFG = copy.deepcopy(GO2W_PHASE1_TERRAIN_CFG)
"""Go2W Phase 2 Terrain: Exploration annealing and stabilization on easy terrain."""


# =============================================================================
# Phase 3: Intermediate Mazes + 10-15cm Stairs ([0.2, 0.6])
# =============================================================================

GO2W_PHASE3_TERRAIN_CFG = TerrainGeneratorCfg(
    size=(30.0, 30.0),
    border_width=30.0,
    num_rows=6,
    num_cols=30,
    horizontal_scale=0.1,
    vertical_scale=0.005,
    slope_threshold=0.75,
    use_cache=False,
    curriculum=True,
    difficulty_range=(0.2, 0.6),
    sub_terrains={
        "maze": HfMazeTerrainCfg(
            proportion=0.4,
            open_probability=0.9,
            grid_size=(15, 15),
            cell_size=2.0,
            add_noise_to_flat=False,
            add_goal=True,
            randomize_wall=True,
            random_wall_ratio=0.5,
            add_stairs_to_maze=True,
        ),
        "stairs": HfMazeTerrainCfg(
            proportion=0.3,
            open_probability=0.9,
            grid_size=(15, 15),
            cell_size=2.0,
            add_noise_to_flat=False,
            add_goal=True,
            randomize_wall=False,
            random_wall_ratio=1.0,
            non_maze_terrain=False,
            stairs=True,
        ),
        "non_maze": HfMazeTerrainCfg(
            proportion=0.3,
            open_probability=0.9,
            grid_size=(15, 15),
            cell_size=2.0,
            add_noise_to_flat=False,
            add_goal=True,
            randomize_wall=True,
            random_wall_ratio=1.0,
            non_maze_terrain=True,
        ),
    },
)
"""Go2W Phase 3 Terrain: Introducing stairs (10-15cm) and moderate mazes."""


# =============================================================================
# Phase 4: High Difficulty Full Terrain + 20cm Stairs + Pits ([0.3, 0.8])
# =============================================================================

GO2W_PHASE4_TERRAIN_CFG = TerrainGeneratorCfg(
    size=(30.0, 30.0),
    border_width=30.0,
    num_rows=6,
    num_cols=30,
    horizontal_scale=0.1,
    vertical_scale=0.005,
    slope_threshold=0.75,
    use_cache=False,
    curriculum=True,
    difficulty_range=(0.3, 0.8),
    sub_terrains={
        "maze": HfMazeTerrainCfg(
            proportion=0.3,
            open_probability=0.9,
            grid_size=(15, 15),
            cell_size=2.0,
            add_noise_to_flat=False,
            add_goal=True,
            randomize_wall=True,
            random_wall_ratio=0.5,
            add_stairs_to_maze=True,
        ),
        "stairs": HfMazeTerrainCfg(
            proportion=0.3,
            open_probability=0.9,
            grid_size=(15, 15),
            cell_size=2.0,
            add_noise_to_flat=False,
            add_goal=True,
            randomize_wall=False,
            random_wall_ratio=1.0,
            non_maze_terrain=False,
            stairs=True,
        ),
        "pits": HfMazeTerrainCfg(
            proportion=0.2,
            open_probability=0.9,
            grid_size=(15, 15),
            cell_size=2.0,
            add_noise_to_flat=False,
            add_goal=True,
            randomize_wall=True,
            random_wall_ratio=1.0,
            non_maze_terrain=True,
            dynamic_obstacles=True,
        ),
        "non_maze": HfMazeTerrainCfg(
            proportion=0.2,
            open_probability=0.9,
            grid_size=(15, 15),
            cell_size=2.0,
            add_noise_to_flat=False,
            add_goal=True,
            randomize_wall=True,
            random_wall_ratio=1.0,
            non_maze_terrain=True,
        ),
    },
)
"""Go2W Phase 4 Terrain: High-difficulty 20cm stairs, pits, and dense mazes."""


# =============================================================================
# Phase 5: Fine-Grained Refinement on Full Complexity ([0.3, 0.8])
# =============================================================================

GO2W_PHASE5_TERRAIN_CFG = copy.deepcopy(GO2W_PHASE4_TERRAIN_CFG)
"""Go2W Phase 5 Terrain: Final refinement on full complexity terrain."""


# =============================================================================
# Backward Compatibility Aliases
# =============================================================================

GO2W_STAGE1_TERRAIN_CFG = GO2W_PHASE1_TERRAIN_CFG
GO2W_STAGE2_TERRAIN_CFG = GO2W_PHASE3_TERRAIN_CFG
GO2W_STAGE3_TERRAIN_CFG = GO2W_PHASE4_TERRAIN_CFG

# Copyright (c) 2022-2025, Fan Yang and Per Frivik, ETH Zurich.
# All rights reserved.
#
# SPDX-License-Identifier: MIT

"""Configuration for maze terrains with low obstacles (40cm to 80cm).

Dedicated configuration for fine-tuning obstacle avoidance against low-height
obstacles without modifying the upstream standard maze terrain settings.
"""

from isaaclab.terrains.terrain_generator_cfg import TerrainGeneratorCfg
from .hf_terrains_maze_cfg import HfMazeTerrainCfg

LOW_OBS_MAZE_TERRAIN_CFG = TerrainGeneratorCfg(
    size=(30.0, 30.0),
    border_width=30.0,
    num_rows=6,
    num_cols=30,
    horizontal_scale=0.1,
    vertical_scale=0.005,
    slope_threshold=0.75,
    use_cache=False,
    curriculum=False,
    difficulty_range=(0.5, 1.0),
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
            wall_height=0.6,
            wall_height_range=(0.40, 0.80),
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
            wall_height=0.6,
            wall_height_range=(0.40, 0.80),
        ),
        "stairs": HfMazeTerrainCfg(
            proportion=0.2,
            open_probability=0.9,
            grid_size=(15, 15),
            cell_size=2.0,
            add_noise_to_flat=False,
            add_goal=True,
            randomize_wall=False,
            random_wall_ratio=1.0,
            non_maze_terrain=False,
            stairs=True,
            wall_height=0.6,
            wall_height_range=(0.40, 0.80),
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
            wall_height=0.6,
            wall_height_range=(0.40, 0.80),
        ),
    },
)
"""Maze terrain configuration with 40-80cm obstacles for fine-tuning."""

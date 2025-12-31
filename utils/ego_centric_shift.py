# Copyright (c) 2025 Fan Yang, Robotic Systems Lab, ETH Zurich
# Licensed under the MIT License (see LICENSE file)
#
# Author: Fan Yang (fanyang1@ethz.ch)
# Robotic Systems Lab, ETH Zurich
# 2025
#
# Description: Utility functions for ego-centric coordinate transformations
# and map reordering for robotics applications.

import numpy as np

def reorder_map_ego_centric(map_array, x, y, rec_size):
    """
    Reorder a 2D binary map to be ego-centric around a given position (x, y), padding out-of-bound areas with zeros.
    Args:
        map_array: 2D numpy array representing the original map
        x, y: Coordinates of the center of the ego-centric map
        rec_size: Size of the reconstructed ego-centric map
    """
    m, n = map_array.shape
    shifted_map = np.zeros((rec_size, rec_size), dtype=map_array.dtype)

    # Determine ranges on the original map
    start_x = max(0, x - rec_size // 2)
    end_x = min(m, x + (rec_size + 1) // 2)
    start_y = max(0, y - rec_size // 2)
    end_y = min(n, y + (rec_size + 1) // 2)

    # Determine ranges on the new shifted map
    shifted_start_x = max(0, rec_size // 2 - x)
    shifted_end_x = shifted_start_x + (end_x - start_x)
    shifted_start_y = max(0, rec_size // 2 - y)
    shifted_end_y = shifted_start_y + (end_y - start_y)

    # Copy the visible part of the original map to the shifted map
    shifted_map[shifted_start_x:shifted_end_x, shifted_start_y:shifted_end_y] = \
        map_array[start_x:end_x, start_y:end_y]

    return shifted_map

def inverse_reorder_map_ego_centric(ego_centric_map, x, y, original_shape):
    """
    Reverse the ego-centric transformation by shifting the map back to its original configuration.
    Notes the ego-centric map size can be different from the original map size.
    Args:
        ego_centric_map: 2D numpy array representing the ego-centric map
        x, y: Coordinates of the center of the ego-centric map
        original_shape: Shape of the original map
    """
    m = original_shape[0] # assuming the map is square
    rec_size = ego_centric_map.shape[0]
    
    # Calculate the amount of shift applied to center (x, y)
    shift_x = (rec_size // 2) - x
    shift_y = (rec_size // 2) - y

    # Reverse the shift using np.roll
    # np.roll shifts in the opposite direction, so we negate the shifts
    original_map = np.roll(ego_centric_map, -shift_x, axis=0)
    original_map = np.roll(original_map, -shift_y, axis=1)
    
    # padding with zeros for smaller ego-centric map with respect to the original map
    if rec_size < m:
        pad_x = (m - rec_size) // 2
        pad_y = (m - rec_size) // 2
        original_map = np.pad(original_map, ((pad_x, pad_x), (pad_y, pad_y)), mode='constant', constant_values=0)

    return original_map
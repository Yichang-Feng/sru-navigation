# Copyright (c) 2025 Fan Yang, Robotic Systems Lab, ETH Zurich
# Licensed under the MIT License (see LICENSE file)
#
# Author: Fan Yang (fanyang1@ethz.ch)
# Robotic Systems Lab, ETH Zurich
# 2025
#
# Description: General visualization utilities for plotting training progress
# and model performance metrics.

import torch
import numpy as np
import matplotlib.pyplot as plt

import os
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from utils.ego_centric_shift import inverse_reorder_map_ego_centric


def visualize_map(env_map, obs, positions, preditions, obs_masks, ground_truth, obs_size, ego_centric=False):
    fig, axs = plt.subplots(3, len(obs) + 1, figsize=(15, 10))
    # Display the full environment map
    axs[0, 0].imshow(env_map, cmap='gray')
    axs[1, 0].imshow(np.zeros_like(env_map), cmap='gray')
    axs[2, 0].imshow(np.zeros_like(env_map), cmap='gray')
    axs[0, 0].set_title('Environment Map')
    
    rec_size = preditions[0].shape[0] 

    # Loop through each observation and position
    for i, (o, p) in enumerate(zip(obs, positions)):
        x, y = p.int()
        # Calculate the top-left corner of the observation in the environment map
        x1 = x - obs_size // 2
        y1 = y - obs_size // 2
        
        # Create an overlay that's the size of the environment map initialized to NaNs
        overlay = np.full(env_map.shape, np.nan)

        # Place the observation in the overlay at the correct position
        for dx in range(obs_size):
            for dy in range(obs_size):
                map_x = x1 + dx
                map_y = y1 + dy
                if 0 <= map_x < env_map.shape[0] and 0 <= map_y < env_map.shape[1]:
                    overlay[map_x, map_y] = o[dx, dy]
        
        # draw a red bounding box for the origianl map
        rect = plt.Rectangle((-.5+y1-rec_size//2+obs_size//2, 
                              -.5+x1-rec_size//2+obs_size//2), 
                             rec_size, 
                             rec_size, 
                             edgecolor='r', 
                             facecolor='none', 
                             linewidth=2)

        # Display the observation on a new subplot
        axs[0, i + 1].imshow(env_map, cmap='gray')
        axs[0, i + 1].add_patch(rect)
        axs[0, i + 1].imshow(overlay, cmap='cool', alpha=0.5)  # Overlay with transparency
        axs[0, i + 1].set_title(f'Step: {i+1}')
        
        if ego_centric:
            pred_viz = inverse_reorder_map_ego_centric(preditions[i], x, y, env_map.shape)
            gt_viz = inverse_reorder_map_ego_centric(ground_truth[i], x, y, env_map.shape)
            mask_viz = inverse_reorder_map_ego_centric(obs_masks[i], x, y, env_map.shape)
        else:
            pred_viz = preditions[i]
            gt_viz = ground_truth[i]
            mask_viz = obs_masks[i]
        
        # Display the observation mask and ground truth on the second and third rows
        axs[1, i + 1].imshow(pred_viz, cmap='gray', alpha=0.8)
        axs[1, i + 1].imshow(mask_viz, cmap='viridis', alpha=0.2)
        axs[1, i + 1].set_title(f'Step: {i+1}')
        
        axs[2, i + 1].imshow(pred_viz, cmap='cool', alpha=0.5)
        axs[2, i + 1].imshow(gt_viz, cmap='hot', alpha=0.5)
        axs[2, i + 1].set_title(f'Step: {i+1}')

    plt.show()
    

def modify_original_grid(original_grid, grid_coords, patch_value):
    """
    Note: This function modifies the original grid in place.
    Modify the original grid using advanced indexing.
    """
    grid_coords = grid_coords.squeeze(0)
    H, W = original_grid.shape[0], original_grid.shape[1]
    h, w = grid_coords.shape[0], grid_coords.shape[1]
    for i in range(h):
        for j in range(w):
            y, x = grid_coords[i, j]
            if 0 <= x < H and 0 <= y < W:
                original_grid[x, y] = patch_value[i, j]
    return # inplace modification


def visualize_map_bbox(env_map, obs, obs_grids, rec, rec_masks, rec_bboxs, rec_gt):
    fig, axs = plt.subplots(3, len(obs) + 1, figsize=(15, 10))
    # Display the full environment map
    axs[0, 0].imshow(env_map, cmap='gray')
    axs[1, 0].imshow(np.zeros_like(env_map), cmap='gray')
    axs[2, 0].imshow(np.zeros_like(env_map), cmap='gray')
    axs[0, 0].set_title('Environment Map')
    
    # Loop through steps
    for i, (o, grids, bbox) in enumerate(zip(obs, obs_grids, rec_bboxs)):        
        # Create an overlay that's the size of the environment map initialized to NaNs
        overlay = np.full(env_map.shape, np.nan)

        # Place the observation in the overlay at the correct position
        modify_original_grid(overlay, grids, o)

        # Display the observation on a new subplot
        axs[0, i + 1].imshow(env_map, cmap='gray')
        axs[0, i + 1].plot([bbox[i][0] for i in [0, 1, 2, 3, 0]], [bbox[i][1] for i in [0, 1, 2, 3, 0]], 'r-')
        axs[0, i + 1].imshow(overlay, cmap='cool', alpha=0.5)  # Overlay with transparency
        axs[0, i + 1].set_title(f'Step: {i+1}')
        

        pred_viz = rec[i]
        mask_viz = rec_masks[i]
        gt_viz = rec_gt[i] * rec_masks[i] # only show the ground truth where the mask is valid
        
        # Display the observation mask and ground truth on the second and third rows
        axs[1, i + 1].imshow(pred_viz, cmap='gray', alpha=0.8)
        axs[1, i + 1].imshow(mask_viz, cmap='viridis', alpha=0.2)
        axs[1, i + 1].set_title(f'Step: {i+1}')
        
        axs[2, i + 1].imshow(pred_viz, cmap='cool', alpha=0.5)
        axs[2, i + 1].imshow(gt_viz, cmap='hot', alpha=0.5)
        axs[2, i + 1].set_title(f'Step: {i+1}')

    plt.show() 

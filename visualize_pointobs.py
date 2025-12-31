# Copyright (c) 2025 Fan Yang, Robotic Systems Lab, ETH Zurich
# Licensed under the MIT License (see LICENSE file)
#
# Author: Fan Yang (fanyang1@ethz.ch)
# Robotic Systems Lab, ETH Zurich
# 2025
#
# Description: Visualization utilities for 3D point cloud prediction results,
# including ground truth, predictions, robot trajectories, and error visualization.

import os
import torch
import pypose as pp
import matplotlib as mpl
import matplotlib.pyplot as plt
from torch.utils.data import DataLoader

def visualize_dataset(robot_poses: pp.LieTensor, obs_coord_info: torch.Tensor, ego_obs: torch.Tensor, last_obs: torch.Tensor, delta_robot_poses: pp.LieTensor, rnn_type='rnn'):
    batch_size = robot_poses.shape[0]
    # accumulate the delta poses to check against the final robot poses
    accumulated_robot_poses = [pp.identity_SE3(batch_size)]
    for i in range(delta_robot_poses.shape[1]):
        accumulated_robot_poses.append(accumulated_robot_poses[-1] @ delta_robot_poses[:, i])
    accumulated_robot_poses = torch.stack(accumulated_robot_poses[1:], dim=1)
    
     # check the observations in the last robot frame
    last_obs = accumulated_robot_poses[:, -1:].Act(last_obs[..., :3])
    
    # check the ego-observations
    ego_obs = accumulated_robot_poses.Act(ego_obs[..., :3])
    
    # Plotting for each sample in the batch
    for batch_sample in range(batch_size):
        # Plot the trajectory of the agents and robot in 3D
        fig = plt.figure(figsize=(10, 6))
        ax = fig.add_subplot(111, projection='3d')

        # Loop over the different agents
        ax.scatter(obs_coord_info[batch_sample, :, 0].numpy(),
                   obs_coord_info[batch_sample, :, 1].numpy(),
                   obs_coord_info[batch_sample, :, 2].numpy(), color='r', label='Ground Truth')
            
        ax.scatter(last_obs[batch_sample, :, 0].numpy(),
                   last_obs[batch_sample, :, 1].numpy(),
                   last_obs[batch_sample, :, 2].numpy() + 0.2, color='g', label=f'Last Observed')
            
        ax.scatter(ego_obs[batch_sample, :, 0].numpy(),
                   ego_obs[batch_sample, :, 1].numpy(),
                   ego_obs[batch_sample, :, 2].numpy() - 0.2, color='b', label=f'Ego Observed')
        
        # Plot the robot trajectory
        robot_positions = robot_poses[batch_sample].translation().numpy()
        ax.plot(robot_positions[:, 0], robot_positions[:, 1], robot_positions[:, 2], 'k--', label='Robot')
        
        # plot the accumulated robot poses
        accumulated_robot_positions = accumulated_robot_poses[batch_sample].translation().numpy()
        ax.plot(accumulated_robot_positions[:, 0], accumulated_robot_positions[:, 1], accumulated_robot_positions[:, 2] + 0.2, 'r--', label='Accumulated Robot')
        
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title(f'Observations of the Robot - Sample {batch_sample}')
        ax.grid(True)
        plt.show()
        
def visualize_predict(obs_gt: torch.Tensor, obs_pred_last: torch.Tensor, delta_robot_poses: pp.LieTensor, break_first=False, model_name='RNN'):
    
    mpl.rcParams.update({'font.size': 18, 'axes.labelsize': 20, 'axes.titlesize': 24})
    
    # Optimized color palette with more vibrant and balanced colors
    colors = {
        'predicted_points': "#FF6969",    
        'ground_truth': "#6A0C3D",        
        'error_line': "#48A9A6",          
        'robot_trajectory': "#595B72",   
        'annotation': "#0C1844"
    }
    
    batch_size = obs_gt.shape[0]
    
    # Accumulate the delta poses to check against the final robot poses
    accumulated_robot_poses = [pp.identity_SE3(batch_size)]
    for i in range(delta_robot_poses.shape[1]):
        accumulated_robot_poses.append(accumulated_robot_poses[-1] @ delta_robot_poses[:, i])
    accumulated_robot_poses = torch.stack(accumulated_robot_poses[1:], dim=1)
    
    # Predicted observations in the last robot frame -> convert to world frame
    obs_pred = accumulated_robot_poses[:, -1:].Act(obs_pred_last[..., :3])
    
    # Increase the point size for a fuller appearance
    point_size = 200  # Increased from 150
    
    # Loop through each sample in the batch for visualization
    for batch_sample in range(batch_size):
        fig = plt.figure(figsize=(9, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        scatter_handles = {}
        seq_len = obs_pred_last.shape[1]
        error_line_handle = None  # to store the first error line for legend
        
        for i in range(seq_len):
            alpha_value = 0.8  # Increased opacity for clarity
            
            # Choose markers: diamond or square for predicted based on confidence, circle for ground truth
            pred_marker = 'D' if torch.sigmoid(obs_pred_last[batch_sample, i, 3]).item() < 0.5 else 's'
            gt_marker = 'o'
            
            # Plot predicted point with a black outline for clarity
            s_pred = ax.scatter(obs_pred[batch_sample, i, 0].item(),
                                obs_pred[batch_sample, i, 1].item(),
                                obs_pred[batch_sample, i, 2].item(),
                                color=colors['predicted_points'], marker=pred_marker, s=point_size, alpha=alpha_value,
                                edgecolors='k', linewidth=0.7)
            if 'Predicted' not in scatter_handles:
                scatter_handles['Predicted'] = s_pred
            
            # Plot ground truth point with a black outline
            s_gt = ax.scatter(obs_gt[batch_sample, i, 0].item(),
                              obs_gt[batch_sample, i, 1].item(),
                              obs_gt[batch_sample, i, 2].item(),
                              color=colors['ground_truth'], marker=gt_marker, s=point_size, alpha=alpha_value,
                              edgecolors='k', linewidth=0.7)
            if 'Ground Truth' not in scatter_handles:
                scatter_handles['Ground Truth'] = s_gt
            
            # Draw a dotted error line connecting predicted and ground truth points.
            # Only label the first error line so that it appears once in the legend.
            if i == 0:
                error_line_handle, = ax.plot([obs_pred[batch_sample, i, 0].item(), obs_gt[batch_sample, i, 0].item()],
                                             [obs_pred[batch_sample, i, 1].item(), obs_gt[batch_sample, i, 1].item()],
                                             [obs_pred[batch_sample, i, 2].item(), obs_gt[batch_sample, i, 2].item()],
                                             '-.', linewidth=2, color=colors['error_line'], alpha=0.7)
            else:
                ax.plot([obs_pred[batch_sample, i, 0].item(), obs_gt[batch_sample, i, 0].item()],
                        [obs_pred[batch_sample, i, 1].item(), obs_gt[batch_sample, i, 1].item()],
                        [obs_pred[batch_sample, i, 2].item(), obs_gt[batch_sample, i, 2].item()],
                        '-.', linewidth=2, color=colors['error_line'], alpha=0.7)
            
            # Annotate the ground truth point with the sequence index using a highlighted box
            ax.text(obs_gt[batch_sample, i, 0].item() + 0.5,
                    obs_gt[batch_sample, i, 1].item() + 0.5,
                    obs_gt[batch_sample, i, 2].item() + 1.0,
                    f'{i}', color=colors['annotation'], fontsize=16, fontweight='bold',
                    bbox=dict(facecolor='white', alpha=0.6, pad=1, edgecolor='none'))
        
        # Plot the accumulated robot trajectory as a dashed line with thicker width
        robot_positions = accumulated_robot_poses[batch_sample].translation().numpy()
        robot_traj_handle, = ax.plot(robot_positions[:, 0],
                                     robot_positions[:, 1],
                                     robot_positions[:, 2],
                                     '--', linewidth=5, alpha=0.9, color=colors['robot_trajectory'], label='Robot Trajectory')
        
        # Add arrows to indicate the direction of the robot trajectory movement
        for j in range(len(robot_positions) - 1):
            start = robot_positions[j]
            delta = robot_positions[j + 1] - robot_positions[j]
            ax.quiver(start[0], start[1], start[2],
                      delta[0], delta[1], delta[2],
                      length=0.2, normalize=True, color=colors['robot_trajectory'], arrow_length_ratio=0.3)
        
        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.set_zlabel('Z [m]')
        
        # A compact box aspect ratio with slight compression on the Z-axis
        ax.set_box_aspect([1, 1, 0.6])
        
        # Dashed grid for a finer look
        ax.grid(True, linestyle='--', linewidth=0.5)
        
        # Consolidate legend entries without duplicates
        handles = list(scatter_handles.values())
        labels = list(scatter_handles.keys())
        if error_line_handle is not None:
            handles.append(error_line_handle)
            labels.append('Error Line')
        handles.append(robot_traj_handle)
        labels.append('Robot Trajectory')
        ax.legend(handles, labels, loc='best', fontsize=20, frameon=False)
        
        # Adjust camera view if desired (e.g. fixed view initialization)
        ax.view_init(elev=30, azim=45)
        fig.tight_layout(pad=0.0)
        # Save figure as PNG in the 'figures' folder
        os.makedirs('figures', exist_ok=True)
        fig.savefig(f'figures/plot/{model_name}_spatial_map.png', format='png', dpi=300)
        plt.show()
        
        if break_first:
            break

# Test the trajectory generator
if __name__ == "__main__":
    import yaml
    from torch.utils.data import DataLoader
    from dataloader.spiral_dataset import RobotDataset
    
    # Load parameters from YAML file
    with open('params/pointcloud.yaml', 'r') as file:
        params = yaml.safe_load(file)
        
    # Parameters
    rot_scale = params['rot_scale']
    batch_size = params['batch_size']
    sequence_length = params['sequence_length']
        
    # Dataset and DataLoader
    dataset = RobotDataset(num_samples=1000, sequence_length=sequence_length, scale=rot_scale)
    dataloader = DataLoader(dataset, batch_size=batch_size, shuffle=True, num_workers=4)
    
    # Get a batch from the dataloader
    out_dict = next(iter(dataloader))
    
    # Extract the data
    robot_poses = out_dict['transformations']
    obs_coord_info = out_dict['transformed_points']
    ego_obs = out_dict['observed_points']
    last_obs = out_dict['last_transformed_points']
    delta_robot_poses = out_dict['delta_transformations']
    
    batch_size, num_steps = robot_poses.shape[:2]
    
    robot_poses = pp.from_matrix(robot_poses.view(batch_size, num_steps, 3, 4), ltype=pp.SE3_type)
    delta_robot_poses = pp.from_matrix(delta_robot_poses.view(batch_size, num_steps, 3, 4), ltype=pp.SE3_type)

    visualize_predict(obs_coord_info, last_obs, delta_robot_poses)
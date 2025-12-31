# Copyright (c) 2025 Fan Yang, Robotic Systems Lab, ETH Zurich
# Licensed under the MIT License (see LICENSE file)
#
# Author: Fan Yang (fanyang1@ethz.ch)
# Robotic Systems Lab, ETH Zurich
# 2025
#
# Description: Spiral trajectory dataset for evaluation - generates spiral
# trajectories for testing point cloud prediction models.

import torch
from torch.utils.data import Dataset, DataLoader
import numpy as np

def spiral_trajectory(sequence_length, theta_step=0.8, radius_step=0.3, z_step=0.2):
    """
    Generates a spiral trajectory in 3D as a list of 4x4 homogeneous transformation matrices.
    The trajectory starts at (0,0,0) with the first transformation being identity.
    For subsequent frames, the translation follows a spiral defined by the parameters theta_step,
    radius_step, and z_step, and the rotation is computed so that the x-axis (forward) points along
    the direction of motion.
    """
    positions = []
    for i in range(sequence_length):
        theta = i * theta_step
        radius = i * radius_step
        z = i * z_step
        # Spiral parameterization: position on the spiral.
        pos = np.array([radius * np.cos(theta), radius * np.sin(theta), z])
        positions.append(pos)
    
    transformations = []
    for i in range(sequence_length):
        if i == 0:
            # Force the first transformation to be identity (zero translation and identity rotation)
            T = np.eye(4)
        else:
            # Determine the forward direction using the difference from the previous position.
            forward = positions[i] - positions[i-1]
            forward = forward / np.linalg.norm(forward)
            
            # Define the nominal 'up' vector.
            up = np.array([0, 0, 1])
            # If forward is nearly parallel to up, choose an alternative up.
            if np.abs(np.dot(forward, up)) > 0.99:
                up = np.array([0, 1, 0])
            right = np.cross(forward, up)
            right = right / np.linalg.norm(right)
            new_up = np.cross(forward, right)  # Conventional right-hand rule
            
            # Assemble rotation matrix.
            # We use the convention that the first column is the forward direction.
            R = np.column_stack([forward, right, new_up])
            
            # Build the 4x4 transformation matrix.
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = positions[i]
        transformations.append(T)
        
    return transformations

def random_point():
    return np.random.uniform(-5, 5, 3)

def transform_points(points, transformation):
    homogeneous_points = np.hstack([points, np.ones((points.shape[0], 1))])
    transformed_points = (transformation @ homogeneous_points.T).T[:, :3]
    return transformed_points

class RobotDataset(Dataset):
    def __init__(self, num_samples, sequence_length, scale=1.0):
        self.num_samples = num_samples
        self.sequence_length = sequence_length

    def __len__(self):
        return self.num_samples

    @torch.no_grad()
    def __getitem__(self, idx):
        # Generate random points (observations)
        points = np.array([random_point() for _ in range(self.sequence_length)])
        
        # Generate spiral trajectory transformations for each frame.
        transformations = np.array(spiral_trajectory(self.sequence_length))
        
        # Generate some binary information per frame.
        binary_info = np.random.randint(0, 2, (self.sequence_length, 1))
        
        # Compute delta transformations between consecutive frames.
        # Note: The first transformation is the identity.
        delta_transformations = np.empty_like(transformations)
        delta_transformations[0] = np.eye(4)
        for i in range(1, self.sequence_length):
            delta_transformations[i] = np.linalg.inv(transformations[i-1]) @ transformations[i]

        # Apply transformations to the observed points.
        transformed_points = np.array([
            transform_points(points[i:i + 1], transformations[i])
            for i in range(self.sequence_length)
        ]).squeeze(1)

        # Calculate the inverse of the last frame transformation and transform all observed points into that frame.
        last_frame_inv = np.linalg.inv(transformations[-1])
        last_frame_points = transform_points(transformed_points, last_frame_inv)
        
        # Append the binary information to the points.
        points = np.concatenate([points, binary_info], axis=1)
        transformed_points = np.concatenate([transformed_points, binary_info], axis=1)
        last_frame_points = np.concatenate([last_frame_points, binary_info], axis=1)
        
        # Convert arrays to tensors.
        points_tensor = torch.tensor(points, dtype=torch.float32)
        transformed_points_tensor = torch.tensor(transformed_points, dtype=torch.float32)
        last_frame_points_tensor = torch.tensor(last_frame_points, dtype=torch.float32)
        
        # Reshape the transformation matrices into a 12-dimensional vector per frame (3x4).
        transformations_tensor = torch.tensor(transformations, dtype=torch.float32).reshape(-1, 16)[:, :12]
        delta_transformations_tensor = torch.tensor(delta_transformations, dtype=torch.float32).reshape(-1, 16)[:, :12]

        sample = {
            'observed_points': points_tensor,
            'transformations': transformations_tensor,
            'delta_transformations': delta_transformations_tensor,
            'transformed_points': transformed_points_tensor,
            'last_transformed_points': last_frame_points_tensor
        }

        return sample


# Example usage
if __name__ == '__main__':
    dataset = RobotDataset(num_samples=10, sequence_length=5)
    dataloader = DataLoader(dataset, batch_size=2, shuffle=True)

    for data in dataloader:
        # Sanity check on data shapes
        print("Observed points shape:", data['observed_points'].shape)
        print("Transformations shape:", data['transformations'].shape)
        print("Delta transformations shape:", data['delta_transformations'].shape)
        print("Transformed points shape:", data['transformed_points'].shape)
        print("Last transformed points shape:", data['last_transformed_points'].shape)

        # Verify if accumulated delta transformations match the final transformations
        delta_transformations = data['delta_transformations']
        transformations = data['transformations']
        sequence_length = transformations.shape[1]

        accumulated_transformations = torch.eye(4).repeat(transformations.shape[0], 1, 1)
        for i in range(sequence_length):
            homogeneous_transform = delta_transformations[:, i, :].view(-1, 3, 4)
            last_row = torch.tensor([0, 0, 0, 1], dtype=torch.float32).unsqueeze(0).repeat(homogeneous_transform.shape[0], 1, 1)
            homogeneous_transform = torch.cat([homogeneous_transform, last_row], dim=1)
            accumulated_transformations = accumulated_transformations @ homogeneous_transform

        # Check if the accumulated transformation is the same as the last transformation
        is_close = torch.allclose(accumulated_transformations[:, :3, :], transformations[:, -1, :].view(-1, 3, 4), atol=1e-6)
        print("Accumulated transformations match the last transformations:", is_close)
        break
# Copyright (c) 2025 Fan Yang, Robotic Systems Lab, ETH Zurich
# Licensed under the MIT License (see LICENSE file)
#
# Author: Fan Yang (fanyang1@ethz.ch)
# Robotic Systems Lab, ETH Zurich
# 2025
#
# Description: Point cloud dataset loader for training - generates synthetic
# point cloud sequences with robot pose transformations.

import torch
from torch.utils.data import Dataset, DataLoader
import numpy as np
from scipy.spatial.transform import Rotation

def random_transformation(scale): # Rotation in 3D
    rotation = Rotation.random()
    angle = rotation.magnitude() * scale
    rotation = Rotation.from_rotvec(rotation.as_rotvec() * angle)
    R = rotation.as_matrix()
    t = np.random.uniform(-2, 2, 3)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T

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
        self.scale = scale

    def __len__(self):
        return self.num_samples

    @torch.no_grad()
    def __getitem__(self, idx):
        points = np.array([random_point() for _ in range(self.sequence_length)])
        transformations = np.array([random_transformation(self.scale) for _ in range(self.sequence_length - 1)])
        binary_info = np.random.randint(0, 2, (self.sequence_length, 1))
        
        # Append identity transformation to the first frame
        transformations = np.concatenate([np.eye(4)[None, :, :], transformations], axis=0)
        # Get the delta transformations from previous frame to current frame
        delta_transformations = np.linalg.inv(transformations[:-1]) @ transformations[1:]
        # Append identity transformation to the initial frame
        delta_transformations = np.concatenate([np.eye(4)[None, :, :], delta_transformations], axis=0)

        # Apply transformations to the observed points
        transformed_points = np.array([transform_points(points[i:i + 1], transformations[i])
                                       for i in range(self.sequence_length)]).squeeze(1)

        # Calculate the inverse of the last frame transformation
        last_frame_inv = np.linalg.inv(transformations[-1])
        # Transform all observed points to the last robot frame
        last_frame_points = transform_points(transformed_points, last_frame_inv)
        
        # Concatenate the binary information to the points
        points = np.concatenate([points, binary_info], axis=1)
        transformed_points = np.concatenate([transformed_points, binary_info], axis=1)
        last_frame_points = np.concatenate([last_frame_points, binary_info], axis=1)
        
        # Reorder the transformed points and last frame points, first points to the last
        transformed_points = torch.tensor(transformed_points, dtype=torch.float32)
        last_frame_points = torch.tensor(last_frame_points, dtype=torch.float32)

        # Convert arrays to tensors
        points_tensor = torch.tensor(points, dtype=torch.float32)
        transformations_tensor = torch.tensor(transformations, dtype=torch.float32).reshape(-1, 16)[:, :12]
        delta_transformations = torch.tensor(delta_transformations, dtype=torch.float32).reshape(-1, 16)[:, :12]

        sample = {
            'observed_points': points_tensor,
            'transformations': transformations_tensor,
            'delta_transformations': delta_transformations,
            'transformed_points': transformed_points,
            'last_transformed_points': last_frame_points
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
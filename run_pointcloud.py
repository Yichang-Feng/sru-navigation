# Copyright (c) 2025 Fan Yang, Robotic Systems Lab, ETH Zurich
# Licensed under the MIT License (see LICENSE file)
#
# Author: Fan Yang (fanyang1@ethz.ch)
# Robotic Systems Lab, ETH Zurich
# 2025
#
# Description: Main training and evaluation script for point cloud prediction
# using various recurrent architectures (LSTM, GRU, SRU_LSTM, SRU_GRU,
# SRU_LSTM_Gated, MambaNet, S4).

import os
import yaml
import torch
import wandb
import argparse
import pypose as pp
import torch.nn as nn
from datetime import datetime
import torch.optim as optim
import matplotlib.pyplot as plt
from torch.utils.data import DataLoader
from network.vanilla_mamab import MambaNet
from network.lstm_sru import LSTM_SRU
from network.lstm_sru_gate import LSTM_SRU_Gate
from network.gru_sru import GRU_SRU
from network.s4_utils.s4d_net import S4Model

from dataloader.points_dataset import RobotDataset
from utils.load_weight import load_pretrained_weights

from visualize_pointobs import visualize_predict


class Loss_Function(nn.Module):
    def __init__(self):
        super(Loss_Function, self).__init__()
        self.coord_loss = nn.MSELoss()
        self.info_loss = nn.BCEWithLogitsLoss()

    def forward(self, pred_coord, pred_info, target_points):
        coord_loss = self.coord_loss(pred_coord, target_points[..., :3])
        info_loss = self.info_loss(pred_info, target_points[..., 3]) * 10.0
        loss_dict = {'coord_loss': coord_loss.item(), 'info_loss': info_loss.item()}
        return coord_loss + info_loss, loss_dict

class ForwardLayer(nn.Module):
    def __init__(self, input_size, hidden_size, info_size, sequence_length):
        super(ForwardLayer, self).__init__()
        self.hidden_size = hidden_size
        self.info_size = info_size
        self.sequence_length = sequence_length
        
        coord_dim = 3 * sequence_length
        self.fc = nn.Sequential(
            nn.Linear(input_size, hidden_size),
            nn.LayerNorm(hidden_size),
            nn.ELU(inplace=True),
        )
        
        self.coord_fc = nn.Linear(hidden_size, coord_dim)
        
        self.info_fc = nn.Linear(hidden_size, self.sequence_length)
        
    def forward(self, x):
        x = self.fc(x)
        coord = self.coord_fc(x)
        info = self.info_fc(x)
        return coord.view(-1, self.sequence_length, 3), info
    
class PoseEncoder(nn.Module):
    def __init__(self, pose_size, pose_embed_size):
        super(PoseEncoder, self).__init__()
        self.fc = nn.Linear(pose_size, pose_embed_size)
        
        # Initialize weights with identity matrix
        self.fc.weight.data.copy_(torch.eye(pose_size))
        
    def forward(self, x):
        return self.fc(x)
    
class TransformBaseModel(nn.Module):
    def __init__(self, model, input_size, hidden_size, num_layers, pose_size, info_size, sequence_length, pretrain_path=None):
        super(TransformBaseModel, self).__init__()
        self.sequence_length = sequence_length
        self.pose_embed = PoseEncoder(pose_size, pose_size)
        self.rnn = model(input_size, hidden_size, pose_size, num_layers=num_layers, batch_first=True)
        self.fc = ForwardLayer(hidden_size, hidden_size, info_size, sequence_length)
        
        # Load pre-trained weights
        load_pretrained_weights(self.rnn, pretrain_path)

    def forward(self, x, pose):
        pose = self.pose_embed(pose)
        x, _ = self.rnn(x, pose)
        coord, info = self.fc(x[:, -1, :])
        return coord, info
    
class RNNBaseModel(nn.Module):
    def __init__(self, model, input_size, hidden_size, num_layers, pose_size, info_size, sequence_length, pretrain_path=None):
        super(RNNBaseModel, self).__init__()
        self.sequence_length = sequence_length
        self.pose_embed = PoseEncoder(pose_size, pose_size)
        self.rnn = model(input_size + pose_size, hidden_size, num_layers=num_layers, batch_first=True)
        self.fc = ForwardLayer(hidden_size, hidden_size, info_size, sequence_length)
        
        # Load pre-trained weights
        load_pretrained_weights(self.rnn, pretrain_path)

    def forward(self, x, pose):
        pose = self.pose_embed(pose)
        x = torch.cat((x, pose), dim=2)
        x, _ = self.rnn(x)
        coord, info = self.fc(x[:, -1, :])
        return coord, info
    
class LSTMModel(RNNBaseModel):
    def __init__(self, input_size, hidden_size, num_layers, pose_size, info_size, sequence_length, pretrain_path=None):
        super(LSTMModel, self).__init__(nn.LSTM, input_size, hidden_size, num_layers, pose_size, info_size, sequence_length, pretrain_path=pretrain_path)

class GRUModel(RNNBaseModel):
    def __init__(self, input_size, hidden_size, num_layers, pose_size, info_size, sequence_length, pretrain_path=None):
        super(GRUModel, self).__init__(nn.GRU, input_size, hidden_size, num_layers, pose_size, info_size, sequence_length, pretrain_path=pretrain_path)

class LSTMSRUModel(RNNBaseModel):
    def __init__(self, input_size, hidden_size, num_layers, pose_size, info_size, sequence_length, pretrain_path=None):
        super(LSTMSRUModel, self).__init__(LSTM_SRU, input_size, hidden_size, num_layers, pose_size, info_size, sequence_length, pretrain_path=pretrain_path)

class GRUSRUModel(RNNBaseModel):
    def __init__(self, input_size, hidden_size, num_layers, pose_size, info_size, sequence_length, pretrain_path=None):
        super(GRUSRUModel, self).__init__(GRU_SRU, input_size, hidden_size, num_layers, pose_size, info_size, sequence_length, pretrain_path=pretrain_path)

class LSTMSRUGateModel(RNNBaseModel):
    def __init__(self, input_size, hidden_size, num_layers, pose_size, info_size, sequence_length, pretrain_path=None):
        super(LSTMSRUGateModel, self).__init__(LSTM_SRU_Gate, input_size, hidden_size, num_layers, pose_size, info_size, sequence_length, pretrain_path=pretrain_path)

class MambaNetModel(RNNBaseModel):
    def __init__(self, input_size, hidden_size, num_layers, pose_size, info_size, sequence_length, pretrain_path=None):
        super(MambaNetModel, self).__init__(MambaNet, input_size, hidden_size, num_layers, pose_size, info_size, sequence_length, pretrain_path=pretrain_path)

class S4ModelModel(RNNBaseModel):
    def __init__(self, input_size, hidden_size, num_layers, pose_size, info_size, sequence_length, pretrain_path=None):
        super(S4ModelModel, self).__init__(S4Model, input_size, hidden_size, num_layers, pose_size, info_size, sequence_length, pretrain_path=pretrain_path)

def create_optimizer(model, optimizer_params=None):
    """
    Create NAdam optimizer for the model.

    Args:
        model: The model to optimize
        optimizer_params: Dictionary containing optimizer parameters

    Returns:
        optimizer: The configured optimizer
    """
    # Use default lr from config or fallback
    default_lr = optimizer_params.get('lr', 2e-3) if optimizer_params else 2e-3
    default_weight_decay = optimizer_params.get('weight_decay', 1e-5) if optimizer_params else 1e-5

    optimizer = optim.NAdam(model.parameters(), lr=default_lr, weight_decay=default_weight_decay)
    print("Using NAdam optimizer")
    print(f"  - NAdam learning rate: {default_lr}, weight decay: {default_weight_decay}")

    return optimizer

def train_model(model, task_name, model_name, dataloader, criterion, optimizer, num_epochs, device, is_wandb=False):
    model.train()
    losses = []
    losses_dict = []
    
    if is_wandb:
        init_wandb(task_name, model_name)

    for epoch in range(num_epochs):
        epoch_losses = []
        epoch_losses_dict = []

        for data in dataloader:
            obs_points = data['observed_points'].to(device)
            delta_transformations = data['delta_transformations'].to(device)
            last_points = data['last_transformed_points'].to(device)

            optimizer.zero_grad()
            coord, info = model(obs_points, delta_transformations)
            loss, loss_dict = criterion(coord, info, last_points)
            loss.backward()
            nn.utils.clip_grad_norm_(model.parameters(), 2.0)
            optimizer.step()
            
            epoch_losses.append(loss.item())
            epoch_losses_dict.append(loss_dict)

        avg_loss = sum(epoch_losses) / len(epoch_losses)
        avg_loss_dict = {key: sum(d[key] for d in epoch_losses_dict) / len(epoch_losses_dict) for key in epoch_losses_dict[0]}
        losses.append(avg_loss)
        losses_dict.append(avg_loss_dict)
        # Log output
        print(f'Epoch [{epoch+1}/{num_epochs}], Loss: {avg_loss:.4f}, Coord Loss: {avg_loss_dict["coord_loss"]:.4f}, Info Loss: {avg_loss_dict["info_loss"]:.4f}')
        # Log to wandb
        if is_wandb:
            wandb.log({'Loss': avg_loss, 'Coord Loss': avg_loss_dict['coord_loss'], 'Info Loss': avg_loss_dict['info_loss']})
            
    wandb.finish()
    
    return losses, losses_dict


def evaluate_model(model, dataloader, criterion, device, break_first=False, model_name='RNN'):
    model.eval()
    losses = []

    for data in dataloader:
        obs_points = data['observed_points'].to(device)
        delta_transformations = data['delta_transformations'].to(device)
        last_points = data['last_transformed_points'].to(device)
        
        with torch.no_grad():
            coord, info = model(obs_points, delta_transformations)
            loss, _ = criterion(coord, info, last_points)
            losses.append(loss.item())
            
        # Visualize the predicted points
        obs_coord_info = data['transformed_points']
        last_obs_pred = torch.cat((coord, info.unsqueeze(2)), dim=-1).cpu().detach()
        delta_transformations = delta_transformations.cpu().detach()
        delta_pose = pp.from_matrix(delta_transformations.view(delta_transformations.shape[0], delta_transformations.shape[1], 3, 4), ltype=pp.SE3_type)
        visualize_predict(obs_coord_info, last_obs_pred, delta_pose, break_first=break_first, model_name=model_name)
        
        if break_first:
            break
        
    avg_loss = sum(losses) / len(losses)
    print(f'Evaluation Loss: {avg_loss:.4f}')
    
    return avg_loss


def init_wandb(task_name, model_name):
    wandb.require("core")
    # Convert to string in the format you prefer
    date_time_str = datetime.now().strftime("_%d-%m-%Y-%H-%M-%S_")
    # Initialize wandb
    wandb.init(
        # set the wandb project where this run will be logged
        project="srt_memory_unit_pointcloud",
        # Set the run name to current date and time
        name=model_name + date_time_str + task_name,
        config={
            "architecture": model_name,  # Replace with your actual architecture
        }
    )

if __name__ == '__main__':
    # Model parameters
    # Load parameters from YAML file
    with open('params/pointcloud.yaml', 'r') as file:
        params = yaml.safe_load(file)

    # Update parameters
    learning_rate = params['lr']
    input_size = params['input_size']
    pose_size = params['pose_size']
    hidden_size = params['hidden_size']
    info_size = params['info_size']
    sequence_length = params['sequence_length']
    batch_size = params['batch_size']
    num_epochs = params['num_epochs']
    num_layers = params['num_layers']
    rot_scale = params['rot_scale']
    is_ablation = params['is_ablation']
    load_pretrain = params['load_pretrain']
    save_pretrain = params['save_pretrain']
    pretrain_type = params['pretrain_type']
    task_name = params['task_name']
    
    parser = argparse.ArgumentParser()
    parser.add_argument('--train', action='store_true', help='Flag to indicate training')
    parser.add_argument('--wandb', action='store_true', help='Flag to indicate logging to wandb')
    parser.add_argument('--euler', action='store_true', help='Flag to indicate using euler cluster')
    args = parser.parse_args()

    is_euler = args.euler
    is_train = args.train
    is_wandb = args.wandb
    save_dir = params['save_dir_euler'] if is_euler else params['save_dir_local']
    timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    
    if load_pretrain:
        pretrain_path_lstm = f'{save_dir}/pretrain/{pretrain_type}/lstm_{hidden_size}_{num_layers}.pth'
        pretrain_path_gru = f'{save_dir}/pretrain/{pretrain_type}/gru_{hidden_size}_{num_layers}.pth'
        pretrain_path_lstma = f'{save_dir}/pretrain/{pretrain_type}/lstma_{hidden_size}_{num_layers}.pth'
        pretrain_path_grua = f'{save_dir}/pretrain/{pretrain_type}/grua_{hidden_size}_{num_layers}.pth'
        pretrain_path_lstmag = f'{save_dir}/pretrain/{pretrain_type}/lstmag_{hidden_size}_{num_layers}.pth'
        pretrain_path_mamba = f'{save_dir}/pretrain/{pretrain_type}/mambanet_{hidden_size}_{num_layers}.pth'
    else:
        pretrain_path_lstm = None
        pretrain_path_gru = None
        pretrain_path_lstma = None
        pretrain_path_grua = None
        pretrain_path_lstmag = None
        pretrain_path_mamba = None

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    
    print(f'Training on {device}')
    # Print all the parameters
    print(f'Input Dimension: {input_size}\n'
        f'Pose Dimension: {pose_size}\n'
        f'Hidden Dimension: {hidden_size}\n'
        f'Information Dimension: {info_size}\n'
        f'Sequence Length: {sequence_length}\n'
        f'Batch Size: {batch_size}\n'
        f'Number of Epochs: {num_epochs}')
    
    if is_ablation:
        print('Running Ablation Study')
        # Initialize the models
        model_dicts = {
            'LSTM': LSTMModel(input_size, hidden_size, num_layers, pose_size, info_size, sequence_length).to(device),
            'GRU': GRUModel(input_size, hidden_size, num_layers, pose_size, info_size, sequence_length).to(device),
            'SRU_LSTM': LSTMSRUModel(input_size, hidden_size, num_layers, pose_size, info_size, sequence_length).to(device),
            'SRU_GRU': GRUSRUModel(input_size, hidden_size, num_layers, pose_size, info_size, sequence_length).to(device),
            'SRU_LSTM_Gated': LSTMSRUGateModel(input_size, hidden_size, num_layers, pose_size, info_size, sequence_length).to(device)
        }
    else:
        print('Running Full Study')
        # Initialize the models
        model_dicts = {
            'LSTM': LSTMModel(input_size, hidden_size, num_layers, pose_size, info_size, sequence_length, pretrain_path=pretrain_path_lstm).to(device),
            'GRU': GRUModel(input_size, hidden_size, num_layers, pose_size, info_size, sequence_length, pretrain_path=pretrain_path_gru).to(device),
            'SRU_LSTM': LSTMSRUModel(input_size, hidden_size, num_layers, pose_size, info_size, sequence_length, pretrain_path=pretrain_path_lstma).to(device),
            'SRU_GRU': GRUSRUModel(input_size, hidden_size, num_layers, pose_size, info_size, sequence_length, pretrain_path=pretrain_path_grua).to(device),
            'SRU_LSTM_Gated': LSTMSRUGateModel(input_size, hidden_size, num_layers, pose_size, info_size, sequence_length, pretrain_path=pretrain_path_lstmag).to(device),
            'MambaNet': MambaNetModel(input_size, hidden_size, num_layers, pose_size, info_size, sequence_length, pretrain_path=pretrain_path_mamba).to(device),
            'S4': S4ModelModel(input_size, hidden_size, num_layers, pose_size, info_size, sequence_length).to(device),
        }
    
    # check the number of parameters in the model
    for model_name, model in model_dicts.items():
        num_params = sum(p.numel() for p in model.parameters()) / 1e6
        print(f'{model_name} has {num_params} million parameters')
    
    # Dataset and DataLoader
    dataset = RobotDataset(num_samples=1000, sequence_length=sequence_length, scale=rot_scale)
    dataloader = DataLoader(dataset, batch_size=batch_size, shuffle=True, num_workers=4)
    
    if is_train:
        # Train the models
        training_losses = {}
        detailed_losses = {}
        for model_name, model in model_dicts.items():
            print(f'Training {model_name}')
            criterion = Loss_Function()
            optimizer = create_optimizer(model, params)
            losses, loss_dict = train_model(model, task_name, model_name, dataloader, criterion, optimizer, num_epochs, device, is_wandb)
            # save the final model
            if not os.path.exists(f'{save_dir}/models/cloud/{timestamp}'):
                os.makedirs(f'{save_dir}/models/cloud/{timestamp}')
            torch.save(model.state_dict(), f'{save_dir}/models/cloud/{timestamp}/cloud_{model_name}.pth')
            if save_pretrain:
                # save pretrain model
                if not os.path.exists(f'{save_dir}/pretrain/{pretrain_type}'):
                    os.makedirs(f'{save_dir}/pretrain/{pretrain_type}')
                torch.save(model.rnn.state_dict(), f'{save_dir}/pretrain/{pretrain_type}/{model_name.lower()}_{hidden_size}_{num_layers}.pth')
            # store the training losses
            training_losses[model_name] = losses
            detailed_losses[model_name] = loss_dict
            print(f'{model_name} training completed')

        # save the training losses in a file in data folder
        # check if the folder exists
        if not os.path.exists(f'{save_dir}/data/cloud'):
            os.makedirs(f'{save_dir}/data/cloud')
        with open(f'{save_dir}/data/cloud/{timestamp}_cloud_losses.yaml', 'w') as file:
            yaml.dump(training_losses, file)
        with open(f'{save_dir}/data/cloud/{timestamp}_detailed_cloud_losses.yaml', 'w') as file:
            yaml.dump(detailed_losses, file)
                
        # Plotting the total loss
        plt.figure(figsize=(10, 5))
        plt.xlabel('Epoch')
        plt.ylabel('Loss')
        plt.title('Total Loss Overview')

        for model_name in model_dicts.keys():
            losses = [loss for loss in training_losses[model_name]]
            plt.plot(losses, label=f'Total Loss - {model_name}')

        # Add grid and legend
        plt.grid()
        plt.legend()
        plt.ylim(bottom=0)
        # Save the plot to figures folder
        if not os.path.exists(f'{save_dir}/figures/cloud/{timestamp}'):
            os.makedirs(f'{save_dir}/figures/cloud/{timestamp}')
        plt.savefig(f'{save_dir}/figures/cloud/{timestamp}/cloud_total_loss.png')

        # Plotting the detailed loss
        plt.figure(figsize=(10, 5))
        plt.xlabel('Epoch')
        plt.ylabel('Loss')
        plt.title('Detailed Loss Overview')

        for model_name in model_dicts.keys():
            coord_losses = [detail_loss['coord_loss'] for detail_loss in detailed_losses[model_name]]
            info_losses = [detail_loss['info_loss'] for detail_loss in detailed_losses[model_name]]
            
            plt.plot(coord_losses, label=f'Spatial - {model_name}', linestyle='--')
            plt.plot(info_losses, label=f'Temporal - {model_name}', linestyle='-.')
            
        # Add grid and legend
        plt.grid()
        plt.legend()
        plt.ylim(bottom=0)
        # Save the plot to figures folder
        if not os.path.exists(f'{save_dir}/figures/cloud/{timestamp}'):
            os.makedirs(f'{save_dir}/figures/cloud/{timestamp}')
        plt.savefig(f'{save_dir}/figures/cloud/{timestamp}/cloud_detailed_loss.png')
    else:
        # check what is the latest timestamp in the models folder
        timestamp = sorted(os.listdir(f'{save_dir}/models/cloud'), reverse=True)[0]
        
        print("Loading the existing models")
        for model_name, model in model_dicts.items():
            model.load_state_dict(torch.load(f'{save_dir}/models/cloud/{timestamp}/cloud_{model_name}.pth'))
        
        # Evaluate the models
        for model_name, model in model_dicts.items():
            print(f'Evaluating {model_name}')
            criterion = Loss_Function()
            evaluate_model(model, dataloader, criterion, device, break_first=True, model_name=model_name)
            print(f'{model_name} evaluation completed')
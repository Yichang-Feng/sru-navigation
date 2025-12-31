# Copyright (c) 2025 Fan Yang, Robotic Systems Lab, ETH Zurich
# Licensed under the MIT License (see LICENSE file)
#
# Author: Fan Yang (fanyang1@ethz.ch)
# Robotic Systems Lab, ETH Zurich
# 2025
#
# Description: Utility functions for loading pretrained RNN weights with
# flexible handling of model architecture changes.

import torch

def load_pretrained_weights(self, pretrain_path=None):
    """
    Load pre-trained weights into the model, allowing flexibility in input_size and pose_size.
    Only loads the weights that match the current model's layers.
    """
    try:
        state_dict = torch.load(pretrain_path)
        model_dict = self.state_dict()
        pretrained_dict = {k: v for k, v in state_dict.items() if k in model_dict and v.size() == model_dict[k].size()}
        model_dict.update(pretrained_dict)
        self.load_state_dict(model_dict, strict=True)
        print(f'\033[92mLoaded {len(pretrained_dict)} pre-trained weights from {pretrain_path}\033[0m')
    except Exception:
        pass
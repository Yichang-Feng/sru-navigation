#!/usr/bin/env python3
"""Build and export DreamWaQ adapter for Go2W in Isaac Lab."""

import os
import torch
import torch.nn as nn


class Go2WDreamWaqAdapter(nn.Module):
    def __init__(self, encoder: nn.Module, latent_mu: nn.Module, vel_mu: nn.Module, actor: nn.Module):
        super().__init__()
        self.encoder = encoder
        self.latent_mu = latent_mu
        self.vel_mu = vel_mu
        self.actor = actor

        # Joint mapping indices: Isaac Lab alphabetical -> DreamWaQ sim order
        # Isaac: [FL, FR, RL, RR] x [hip, thigh, calf, foot]
        # DreamWaQ: [FL, FR, RL, RR] where each leg has [hip, thigh, calf, foot]
        self.register_buffer('isaac_to_dwaq', torch.tensor([0, 4, 8, 12, 1, 5, 9, 13, 2, 6, 10, 14, 3, 7, 11, 15], dtype=torch.long))
        
        # DreamWaQ leg actions to Isaac Lab legs: [FL, FR, RL, RR] x [hip, thigh, calf]
        self.register_buffer('dwaq_to_isaac_legs', torch.tensor([0, 4, 8, 12, 1, 5, 9, 13, 2, 6, 10, 14], dtype=torch.long))
        
        # DreamWaQ wheel actions to Isaac Lab wheels: [FL, FR, RL, RR]
        self.register_buffer('dwaq_to_isaac_wheels', torch.tensor([3, 7, 11, 15], dtype=torch.long))
        
        # Mask wheels for position errors and positions (wheels at 3, 7, 11, 15 in DreamWaQ)
        self.register_buffer('wheel_mask', torch.tensor([1.0, 1.0, 1.0, 0.0, 1.0, 1.0, 1.0, 0.0, 1.0, 1.0, 1.0, 0.0, 1.0, 1.0, 1.0, 0.0], dtype=torch.float32))
        
        # Default joint angles in DreamWaQ sim order: [FL(0, 0.67, -1.3, 0), FR(...), RL(...), RR(...)]
        # Strictly matches /home/yichangfeng/DreamWaQ_Go2W/deploy/deploy_real/configs/g2w.yaml
        self.register_buffer('default_dof_pos', torch.tensor([
            0.0, 0.67, -1.3, 0.0,
            0.0, 0.67, -1.3, 0.0,
            0.0, 0.67, -1.3, 0.0,
            0.0, 0.67, -1.3, 0.0
        ], dtype=torch.float32))
        
        self.register_buffer('cmd_scale', torch.tensor([2.0, 2.0, 0.25], dtype=torch.float32))

        self.register_buffer('obs_hist_buf', torch.zeros(1, 365, dtype=torch.float32))
        self.register_buffer('prev_actions', torch.zeros(1, 16, dtype=torch.float32))

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        B = obs.shape[0]
        ang_vel = obs[:, 0:3] * 0.25
        gravity_vec = obs[:, 3:6]
        commands = obs[:, 6:9] * self.cmd_scale

        dof_pos_isaac_rel = obs[:, 9:25]
        dof_vel_isaac = obs[:, 25:41]

        # Reorder to DreamWaQ sim joint order
        dof_err = dof_pos_isaac_rel[:, self.isaac_to_dwaq] * self.wheel_mask
        dof_abs = (dof_pos_isaac_rel[:, self.isaac_to_dwaq] + self.default_dof_pos) * self.wheel_mask
        dof_vel_dwaq = dof_vel_isaac[:, self.isaac_to_dwaq] * 0.05

        if self.prev_actions.shape[0] != B:
            self.prev_actions = torch.zeros(B, 16, device=obs.device, dtype=obs.dtype)
            self.obs_hist_buf = torch.zeros(B, 365, device=obs.device, dtype=obs.dtype)

        # 73-dim observation matching DreamWaQ compute_observations():
        # [ang_vel(3), gravity(3), cmd(3), dof_err(16), dof_vel(16), dof_abs(16), last_action(16)]
        curr_obs = torch.cat([
            ang_vel,
            gravity_vec,
            commands,
            dof_err,
            dof_vel_dwaq,
            dof_abs,
            self.prev_actions
        ], dim=-1)

        self.obs_hist_buf = torch.cat([self.obs_hist_buf[:, 73:], curr_obs], dim=-1)

        h = self.encoder(self.obs_hist_buf)
        latent = self.latent_mu(h)
        vel = self.vel_mu(h)
        code = torch.cat([latent, vel], dim=-1)
        obs_all = torch.cat([code, curr_obs], dim=-1)

        raw_actions = self.actor(obs_all)

        self.prev_actions.copy_(raw_actions)

        leg_actions = raw_actions[:, self.dwaq_to_isaac_legs] * 0.25
        wheel_actions = raw_actions[:, self.dwaq_to_isaac_wheels] * 10.0

        return torch.cat([leg_actions, wheel_actions], dim=-1)


def main():
    base_dir = "/home/yichangfeng/DreamWaQ_Go2W/deploy/pre_train/g2wDWAQ"
    encoder = torch.jit.load(os.path.join(base_dir, "encoder_dwaq.pt"))
    latent_mu = torch.jit.load(os.path.join(base_dir, "latent_mu_dwaq.pt"))
    vel_mu = torch.jit.load(os.path.join(base_dir, "vel_mu_dwaq.pt"))
    actor = torch.jit.load(os.path.join(base_dir, "actor_dwaq.pt"))

    adapter = Go2WDreamWaqAdapter(encoder, latent_mu, vel_mu, actor)
    adapter.eval()

    save_path = "/home/yichangfeng/sru-navigation-learning/policy_go2w_dreamwaq_adapted.pt"
    scripted = torch.jit.script(adapter)
    scripted.save(save_path)
    print(f"Successfully saved scripted model to: {save_path}")
    print(f"File size: {os.path.getsize(save_path)} bytes")

    # Verify loading
    loaded = torch.jit.load(save_path)
    dummy_in = torch.randn(1, 57)
    dummy_out = loaded(dummy_in)
    print("Verification forward pass successful! Output shape:", dummy_out.shape)


if __name__ == "__main__":
    main()

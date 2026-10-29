#!/usr/bin/env python3
"""Convert official B2W TorchScript baseline into a complete RSL-RL checkpoint for Go2W.

Loads actor weights from official ETH B2W baseline (b2w_nav_policy_new.pt),
loads critic weights and template structure from a full ActorCriticSRU checkpoint,
sets exploration noise std to 0.5 (init_noise_std for fine-tuning),
resets iter to 0, and saves a ready-to-load checkpoint for train.py.
"""

import math
import os
import sys
import torch

def prepare_warmstart_checkpoint(
    b2w_ts_path: str = "/home/yichangfeng/SRU-Odin-main/Train/assets/baselines/b2w_nav_policy_new.pt",
    template_ckpt_path: str = "/home/yichangfeng/sru-navigation-learning/IsaacLab/logs/rsl_rl/b2w_navigation_ppo/2026-10-19_14-16-10/model_14999.pt",
    output_path: str = "/home/yichangfeng/sru-navigation-learning/IsaacLab/logs/warmstart_b2w.pt",
    init_noise_std: float = 0.5,
):
    print("=" * 70)
    print("[prepare_warmstart] Generating warmstart checkpoint for Go2W fine-tuning...")
    print(f"  B2W TorchScript source: {b2w_ts_path}")
    print(f"  Critic template source:  {template_ckpt_path}")
    print(f"  Target output path:      {output_path}")
    print(f"  Target init_noise_std:   {init_noise_std}")
    print("=" * 70)

    assert os.path.exists(b2w_ts_path), f"File not found: {b2w_ts_path}"
    assert os.path.exists(template_ckpt_path), f"File not found: {template_ckpt_path}"

    # 1. Load template checkpoint
    template = torch.load(template_ckpt_path, map_location="cpu", weights_only=False)
    state_dict = template["model_state_dict"]

    # 2. Load TorchScript baseline
    ts_model = torch.jit.load(b2w_ts_path, map_location="cpu")
    ts_sd = ts_model.state_dict()

    # 3. Map TorchScript keys to ActorCriticSRU keys
    def map_jit_to_actor(name: str) -> str:
        if name.startswith("rnn."):
            return "memory_a." + name
        elif name.startswith("linear."):
            return "linear_dropout_actor." + name
        return name

    injected_count = 0
    for k, v in ts_sd.items():
        target_k = map_jit_to_actor(k)
        if target_k in state_dict:
            if state_dict[target_k].shape == v.shape:
                state_dict[target_k] = v.data.clone()
                injected_count += 1
            else:
                print(f"  [WARN] Shape mismatch for {target_k}: {state_dict[target_k].shape} vs {v.shape}")
        else:
            if k not in ["hidden_state", "cell_state", "attn_image_net.position_embedding.penc.inv_freq"]:
                print(f"  [WARN] Key not in state_dict: {k} -> {target_k}")

    # 4. Set exploration noise to init_noise_std
    state_dict["log_std"] = torch.full((3,), math.log(init_noise_std))

    # 5. Build clean output dict
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    out_dict = {
        "model_state_dict": state_dict,
        "iter": 0,
        "infos": None,
    }

    torch.save(out_dict, output_path)
    print(f"[SUCCESS] Injected {injected_count} actor weights and saved checkpoint to:")
    print(f"  {output_path} ({os.path.getsize(output_path) / (1024*1024):.2f} MB)")
    print("=" * 70)


if __name__ == "__main__":
    b2w_src = sys.argv[1] if len(sys.argv) > 1 else "/home/yichangfeng/SRU-Odin-main/Train/assets/baselines/b2w_nav_policy_new.pt"
    out_file = sys.argv[2] if len(sys.argv) > 2 else "/home/yichangfeng/sru-navigation-learning/IsaacLab/logs/warmstart_b2w.pt"
    prepare_warmstart_checkpoint(b2w_ts_path=b2w_src, output_path=out_file)

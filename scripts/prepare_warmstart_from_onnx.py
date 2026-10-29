#!/usr/bin/env python3
"""Convert SRU-Odin nav_policy.onnx into a valid RSL-RL checkpoint for fine-tuning.

This script:
1. Loads template weights (including Critic) from an existing ActorCriticSRU checkpoint.
2. Extracts all Actor weights (cross-attention, SRU memory, linear_dropout, actor MLP)
   from nav_policy.onnx and injects them into the state dict.
3. Sets exploration noise std to a conservative value (default 0.35) for stable fine-tuning.
4. Resets iteration counter to 0.
5. Verifies forward numerical parity between the ONNX runtime and PyTorch model.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import numpy as np
import onnx
from onnx import numpy_helper
import onnxruntime as ort
import torch


def prepare_warmstart_from_onnx(
    onnx_path: str = "/home/yichangfeng/SRU-Odin-main/Deployment/models/nav_policy.onnx",
    template_path: str = "/home/yichangfeng/sru-navigation-learning/IsaacLab/logs/rsl_rl/go2w_navigation_mdpo_phase1/2026-10-22_18-12-19_mdpo_phase1_anneal/model_1999.pt",
    output_path: str = "/home/yichangfeng/sru-navigation-learning/IsaacLab/logs/warmstart_from_nav_policy_onnx.pt",
    init_noise_std: float = 0.35,
    verify: bool = True,
):
    print("=" * 70)
    print("[prepare_warmstart] Generating warmstart checkpoint from ONNX...")
    print(f"  Source ONNX:       {onnx_path}")
    print(f"  Template Checkpoint: {template_path}")
    print(f"  Output Checkpoint:   {output_path}")
    print(f"  Init noise std:      {init_noise_std}")
    print("=" * 70)

    assert os.path.exists(onnx_path), f"ONNX file not found: {onnx_path}"
    assert os.path.exists(template_path), f"Template file not found: {template_path}"

    # 1. Load template checkpoint
    template = torch.load(template_path, map_location="cpu", weights_only=False)
    state_dict = template["model_state_dict"]

    # 2. Load ONNX and parse initializers
    onnx_model = onnx.load(onnx_path)
    init_map = {init.name: numpy_helper.to_array(init).copy() for init in onnx_model.graph.initializer}

    # 3. Direct mapping table (ONNX initializer name -> PyTorch state_dict key)
    direct_maps = {
        "attn_image_net.info_proj.0.weight": "attn_image_net.info_proj.0.weight",
        "attn_image_net.info_proj.0.bias": "attn_image_net.info_proj.0.bias",
        "attn_image_net.info_proj.2.weight": "attn_image_net.info_proj.2.weight",
        "attn_image_net.info_proj.2.bias": "attn_image_net.info_proj.2.bias",
        "attn_image_net.norm1.weight": "attn_image_net.norm1.weight",
        "attn_image_net.norm1.bias": "attn_image_net.norm1.bias",
        "attn_image_net.self_attn.in_proj_bias": "attn_image_net.self_attn.in_proj_bias",
        "attn_image_net.self_attn.out_proj.weight": "attn_image_net.self_attn.out_proj.weight",
        "attn_image_net.self_attn.out_proj.bias": "attn_image_net.self_attn.out_proj.bias",
        "attn_image_net.norm2.weight": "attn_image_net.norm2.weight",
        "attn_image_net.norm2.bias": "attn_image_net.norm2.bias",
        "attn_image_net.ffn.0.bias": "attn_image_net.ffn.0.bias",
        "attn_image_net.ffn.2.bias": "attn_image_net.ffn.2.bias",
        "attn_image_net.cross_attn.in_proj_weight": "attn_image_net.cross_attn.in_proj_weight",
        "attn_image_net.cross_attn.in_proj_bias": "attn_image_net.cross_attn.in_proj_bias",
        "attn_image_net.cross_attn.out_proj.weight": "attn_image_net.cross_attn.out_proj.weight",
        "attn_image_net.cross_attn.out_proj.bias": "attn_image_net.cross_attn.out_proj.bias",
        "rnn.cells.0.linear_all.weight": "memory_a.rnn.cells.0.linear_all.weight",
        "rnn.cells.0.linear_all.bias": "memory_a.rnn.cells.0.linear_all.bias",
        "rnn.cells.0.transform_gate.weight": "memory_a.rnn.cells.0.transform_gate.weight",
        "rnn.cells.0.transform_gate.bias": "memory_a.rnn.cells.0.transform_gate.bias",
        "linear.weight": "linear_dropout_actor.linear.weight",
        "linear.bias": "linear_dropout_actor.linear.bias",
        "actor.0.weight": "actor.0.weight",
        "actor.0.bias": "actor.0.bias",
        "actor.2.weight": "actor.2.weight",
        "actor.2.bias": "actor.2.bias",
        "actor.4.weight": "actor.4.weight",
        "actor.4.bias": "actor.4.bias",
    }

    injected_count = 0
    for onnx_k, pt_k in direct_maps.items():
        if onnx_k in init_map and pt_k in state_dict:
            val = torch.from_numpy(init_map[onnx_k])
            assert state_dict[pt_k].shape == val.shape, (
                f"Shape mismatch for {pt_k}: {state_dict[pt_k].shape} vs {val.shape}"
            )
            state_dict[pt_k] = val
            injected_count += 1
        else:
            print(f"  [WARN] Missing key pair: {onnx_k} -> {pt_k}")

    # 4. Handle unnamed transposed linear weights from ONNX export
    if "onnx::MatMul_558" in init_map:
        # self_attn.in_proj_weight is (192, 64)
        state_dict["attn_image_net.self_attn.in_proj_weight"] = torch.from_numpy(init_map["onnx::MatMul_558"].T)
        injected_count += 1
    if "onnx::MatMul_566" in init_map:
        # ffn.0.weight is (128, 64)
        state_dict["attn_image_net.ffn.0.weight"] = torch.from_numpy(init_map["onnx::MatMul_566"].T)
        injected_count += 1
    if "onnx::MatMul_567" in init_map:
        # ffn.2.weight is (64, 128)
        state_dict["attn_image_net.ffn.2.weight"] = torch.from_numpy(init_map["onnx::MatMul_567"].T)
        injected_count += 1

    # 5. Set conservative exploration noise
    state_dict["log_std"] = torch.full((3,), math.log(init_noise_std))

    # 6. Save clean checkpoint
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    out_dict = {
        "model_state_dict": state_dict,
        "iter": 0,
        "infos": None,
    }
    torch.save(out_dict, output_path)
    print(f"[SUCCESS] Injected {injected_count} actor weights into checkpoint:")
    print(f"  Saved to: {output_path} ({os.path.getsize(output_path) / (1024*1024):.2f} MB)")

    # 7. Verification if requested
    if verify:
        print("\n[VERIFICATION] Comparing forward outputs between ONNX and PyTorch...")
        from rsl_rl.modules.actor_critic_sru import ActorCriticSRU

        model = ActorCriticSRU(
            num_actor_obs=2576,
            num_critic_obs=2576 + 64 * 7 * 7 + 1,
            num_actions=3,
            actor_hidden_dims=[512, 256, 128],
            critic_hidden_dims=[512, 256, 128],
            activation="elu",
            init_noise_std=init_noise_std,
            image_input_dims=(64, 5, 8),
            height_input_dims=(64, 7, 7),
            rnn_type="lstm_sru",
            rnn_hidden_size=512,
            rnn_num_layers=1,
            num_cameras=1,
        )
        model.load_state_dict(state_dict)
        model.eval()

        sess = ort.InferenceSession(onnx_path)
        np.random.seed(123)
        test_obs = np.random.randn(2, 2576).astype(np.float32)
        test_h = np.zeros((1, 2, 512), dtype=np.float32)
        test_c = np.zeros((1, 2, 512), dtype=np.float32)

        ort_out = sess.run(None, {"obs": test_obs, "h_in": test_h, "c_in": test_c})
        ort_action = ort_out[0]

        with torch.no_grad():
            pt_obs = torch.from_numpy(test_obs)
            pt_action = model.act_inference(pt_obs)
            max_diff = np.abs(ort_action - pt_action.numpy()).max()
            print(f"  Batch max diff: {max_diff:.8e}")
            assert max_diff < 1e-4, f"Verification failed! Max diff {max_diff} exceeds tolerance."
            print("  [PASSED] Perfect parity verified!\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Prepare warmstart checkpoint from ONNX")
    parser.add_argument("--onnx", default="/home/yichangfeng/SRU-Odin-main/Deployment/models/nav_policy.onnx")
    parser.add_argument("--template", default="/home/yichangfeng/sru-navigation-learning/IsaacLab/logs/rsl_rl/go2w_navigation_mdpo_phase1/2026-10-22_18-12-19_mdpo_phase1_anneal/model_1999.pt")
    parser.add_argument("--output", default="/home/yichangfeng/sru-navigation-learning/IsaacLab/logs/warmstart_from_nav_policy_onnx.pt")
    parser.add_argument("--noise-std", type=float, default=0.35)
    parser.add_argument("--no-verify", action="store_true")
    args = parser.parse_args()

    prepare_warmstart_from_onnx(
        onnx_path=args.onnx,
        template_path=args.template,
        output_path=args.output,
        init_noise_std=args.noise_std,
        verify=not args.no_verify,
    )

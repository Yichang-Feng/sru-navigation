#!/usr/bin/env python3
"""Export trained navigation checkpoints (.pt) to ONNX for real-robot deployment.

Usage:
    python scripts/export_onnx.py --checkpoint /path/to/model_5500.pt

    # Export and automatically deploy to robot workspace:
    python scripts/export_onnx.py --checkpoint /path/to/model_5500.pt --deploy

    # Specify custom output directory:
    python scripts/export_onnx.py --checkpoint /path/to/model_5500.pt --output-dir ./my_export
"""

import argparse
import os
import shutil
import sys
import time
from pathlib import Path
from typing import Optional, Tuple

import numpy as np
import torch
import yaml

# Ensure project root is in sys.path
REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from rsl_rl.modules.actor_critic_sru import ActorCriticSRU

DEFAULT_DEPLOY_DIR = Path("/home/yichangfeng/sru-robot-deployment/rl_nav_controller/deployment_policies")


def parse_args():
    parser = argparse.ArgumentParser(description="Export trained navigation policy to ONNX format.")
    parser.add_argument(
        "--checkpoint",
        type=str,
        required=True,
        help="Path to trained PyTorch checkpoint (.pt file)",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default=None,
        help="Directory to save exported ONNX model (defaults to '<run_dir>/export')",
    )
    parser.add_argument(
        "--filename",
        type=str,
        default="nav_policy.onnx",
        help="Filename for exported ONNX model (default: nav_policy.onnx)",
    )
    parser.add_argument(
        "--agent-cfg",
        type=str,
        default=None,
        help="Path to agent.yaml config file (default: auto-detected in run_dir/params/agent.yaml)",
    )
    parser.add_argument(
        "--deploy",
        action="store_true",
        help=f"Copy exported ONNX model to robot deployment package ({DEFAULT_DEPLOY_DIR})",
    )
    parser.add_argument(
        "--opset",
        type=int,
        default=17,
        help="ONNX opset version (default: 17)",
    )
    return parser.parse_args()


def load_agent_config(checkpoint_path: Path, agent_cfg_arg: Optional[str] = None) -> dict:
    """Locate and load agent.yaml for the checkpoint."""
    if agent_cfg_arg and os.path.isfile(agent_cfg_arg):
        cfg_path = Path(agent_cfg_arg)
    else:
        # Check standard locations
        run_dir = checkpoint_path.parent
        candidates = [
            run_dir / "params" / "agent.yaml",
            run_dir / "agent.yaml",
            run_dir.parent / "params" / "agent.yaml",
        ]
        cfg_path = None
        for cand in candidates:
            if cand.is_file():
                cfg_path = cand
                break

    if cfg_path and cfg_path.is_file():
        print(f"[INFO] Loaded agent config from: {cfg_path}")
        with open(cfg_path, "r", encoding="utf-8") as f:
            return yaml.unsafe_load(f)
    else:
        print("[WARN] No agent.yaml found, using fallback defaults for Go2W Phase 5.")
        return {}


def export_checkpoint_to_onnx(
    checkpoint_path: str,
    output_dir: Optional[str] = None,
    filename: str = "nav_policy.onnx",
    agent_cfg_path: Optional[str] = None,
    deploy: bool = False,
    opset_version: int = 17,
) -> Path:
    ckpt_file = Path(checkpoint_path).resolve()
    if not ckpt_file.is_file():
        raise FileNotFoundError(f"Checkpoint file not found: {ckpt_file}")

    if output_dir is None:
        out_dir = ckpt_file.parent / "export"
    else:
        out_dir = Path(output_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    out_file = out_dir / filename

    print("=" * 80)
    print("Go2W Navigation Policy ONNX Exporter")
    print(f"Checkpoint  : {ckpt_file}")
    print(f"Output ONNX : {out_file}")
    print("=" * 80)

    # 1. Load checkpoint
    ckpt = torch.load(ckpt_file, map_location="cpu", weights_only=False)
    if "model_state_dict" in ckpt:
        state_dict = ckpt["model_state_dict"]
    elif "model_state_dict_1" in ckpt:
        state_dict = ckpt["model_state_dict_1"]
    else:
        state_dict = ckpt

    # 2. Load agent config
    agent_cfg = load_agent_config(ckpt_file, agent_cfg_path)
    pol_cfg = agent_cfg.get("policy", {})

    # 3. Infer / verify network dimensions
    if "attn_image_net.info_proj.0.weight" in state_dict:
        proprio_dim = state_dict["attn_image_net.info_proj.0.weight"].shape[1]
    else:
        proprio_dim = 16

    if "actor.4.weight" in state_dict:
        num_actions = state_dict["actor.4.weight"].shape[0]
    else:
        num_actions = 3

    img_dims = tuple(pol_cfg.get("image_input_dims", (64, 5, 8)))
    height_dims = tuple(pol_cfg.get("height_input_dims", (64, 7, 7)))
    num_cameras = pol_cfg.get("num_cameras", 1)

    num_image_features = num_cameras * img_dims[0] * img_dims[1] * img_dims[2]
    num_height_features = height_dims[0] * height_dims[1] * height_dims[2]

    num_actor_obs = proprio_dim + num_image_features
    num_critic_obs = proprio_dim + num_image_features + num_height_features + 1

    actor_hidden_dims = pol_cfg.get("actor_hidden_dims", [512, 256, 128])
    critic_hidden_dims = pol_cfg.get("critic_hidden_dims", [512, 256, 128])
    rnn_hidden_size = pol_cfg.get("rnn_hidden_size", 512)
    rnn_num_layers = pol_cfg.get("rnn_num_layers", 1)
    activation = pol_cfg.get("activation", "elu")

    print("[INFO] Model Architecture Specs:")
    print(f"  - Proprioceptive State Dim : {proprio_dim}")
    print(f"  - Depth Image Feature Dim : {num_image_features} (shape: {img_dims}, {num_cameras} cam)")
    print(f"  - Total Policy Input Obs   : {num_actor_obs}")
    print(f"  - Policy Output Actions    : {num_actions} ([vx, vy, omega])")
    print(f"  - LSTM/SRU Hidden State Dim: {rnn_hidden_size} (layers: {rnn_num_layers})")

    # 4. Instantiate ActorCriticSRU
    model = ActorCriticSRU(
        num_actor_obs=num_actor_obs,
        num_critic_obs=num_critic_obs,
        num_actions=num_actions,
        actor_hidden_dims=actor_hidden_dims,
        critic_hidden_dims=critic_hidden_dims,
        activation=activation,
        init_noise_std=pol_cfg.get("init_noise_std", 1.0),
        rnn_type=pol_cfg.get("rnn_type", "lstm_sru"),
        rnn_hidden_size=rnn_hidden_size,
        rnn_num_layers=rnn_num_layers,
        dropout=pol_cfg.get("dropout", 0.2),
        num_cameras=num_cameras,
        image_input_dims=img_dims,
        height_input_dims=height_dims,
    )

    load_result = model.load_state_dict(state_dict, strict=True)
    print(f"[INFO] Successfully loaded model weights: {load_result}")

    # 5. Export to ONNX
    print(f"\n[INFO] Exporting actor policy to: {out_file} (opset {opset_version})...")
    model.export_onnx(path=str(out_dir), filename=filename, normalizer=None)

    # 6. Verify exported model
    print("\n[INFO] Validating exported ONNX model...")
    try:
        import onnx
        onnx_model = onnx.load(str(out_file))
        onnx.checker.check_model(onnx_model)
        print("[✓] ONNX checker: Model syntax and graph topology valid.")
    except Exception as e:
        print(f"[WARN] ONNX check warning: {e}")

    try:
        import onnxruntime as ort
        sess = ort.InferenceSession(str(out_file), providers=["CPUExecutionProvider"])
        inputs_info = {inp.name: inp.shape for inp in sess.get_inputs()}
        outputs_info = {out.name: out.shape for out in sess.get_outputs()}
        print(f"[✓] ONNX Runtime Session created successfully.")
        print(f"    Inputs : {inputs_info}")
        print(f"    Outputs: {outputs_info}")

        # Test dummy inference
        dummy_obs = np.zeros((1, num_actor_obs), dtype=np.float32)
        dummy_h = np.zeros((rnn_num_layers, 1, rnn_hidden_size), dtype=np.float32)
        dummy_c = np.zeros((rnn_num_layers, 1, rnn_hidden_size), dtype=np.float32)

        start_t = time.perf_counter()
        actions, h_out, c_out = sess.run(None, {"obs": dummy_obs, "h_in": dummy_h, "c_in": dummy_c})
        latency_ms = (time.perf_counter() - start_t) * 1000.0

        print(f"[✓] Dummy inference succeeded in {latency_ms:.2f} ms:")
        print(f"    Action output shape: {actions.shape}, values: {actions[0]}")
        print(f"    Hidden state shape : {h_out.shape}")
        print(f"    Cell state shape   : {c_out.shape}")
    except Exception as e:
        print(f"[WARN] ONNXRuntime validation failed: {e}")

    # 7. Optional deployment copy
    if deploy:
        if DEFAULT_DEPLOY_DIR.exists():
            target_dest = DEFAULT_DEPLOY_DIR / filename
            if target_dest.exists():
                backup = DEFAULT_DEPLOY_DIR / f"{filename}.bak_{int(time.time())}"
                shutil.copy2(target_dest, backup)
                print(f"[DEPLOY] Backed up existing model to: {backup}")
            shutil.copy2(out_file, target_dest)
            print(f"[DEPLOY] Successfully deployed new model to: {target_dest}")
        else:
            print(f"[DEPLOY ERROR] Target deployment directory does not exist: {DEFAULT_DEPLOY_DIR}")

    print("\n" + "=" * 80)
    print("Export Complete!")
    print(f"Saved ONNX Policy: {out_file} ({out_file.stat().st_size / (1024 * 1024):.2f} MB)")
    print("=" * 80)
    return out_file


def main():
    args = parse_args()
    export_checkpoint_to_onnx(
        checkpoint_path=args.checkpoint,
        output_dir=args.output_dir,
        filename=args.filename,
        agent_cfg_path=args.agent_cfg,
        deploy=args.deploy,
        opset_version=args.opset,
    )


if __name__ == "__main__":
    main()

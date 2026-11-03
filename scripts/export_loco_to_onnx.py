#!/usr/bin/env python3
"""Export Go2W Locomotion TorchScript model (policy.pt) to ONNX (policy.onnx)."""

import os
import argparse
import torch
import onnx
import onnxruntime as ort
import numpy as np

def convert(pt_path: str, onnx_path: str):
    print(f"Loading TorchScript model from: {pt_path}")
    model = torch.jit.load(pt_path)
    model.eval()

    dummy_input = torch.randn(1, 57, dtype=torch.float32)

    os.makedirs(os.path.dirname(os.path.abspath(onnx_path)), exist_ok=True)
    print(f"Exporting ONNX model to: {onnx_path}")
    torch.onnx.export(
        model,
        dummy_input,
        onnx_path,
        export_params=True,
        opset_version=13,
        do_constant_folding=True,
        input_names=["input"],
        output_names=["output"],
        dynamic_axes={
            "input": {0: "batch_size"},
            "output": {0: "batch_size"}
        }
    )

    # Verification
    onnx_model = onnx.load(onnx_path)
    onnx.checker.check_model(onnx_model)

    session = ort.InferenceSession(onnx_path, providers=["CPUExecutionProvider"])
    test_input = np.random.randn(1, 57).astype(np.float32)
    torch_out = model(torch.from_numpy(test_input)).detach().numpy()
    onnx_out = session.run(["output"], {"input": test_input})[0]

    diff = np.max(np.abs(torch_out - onnx_out))
    print(f"✓ Export verified! Max difference between PyTorch and ONNX: {diff:.6e}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert policy.pt to policy.onnx")
    parser.add_argument(
        "--pt",
        default="/home/yichangfeng/rl_sar/policy/go2w/robot_lab/policy.pt",
        help="Path to policy.pt"
    )
    parser.add_argument(
        "--onnx",
        default="/home/yichangfeng/rl_sar/policy/go2w/robot_lab/policy.onnx",
        help="Path to output policy.onnx"
    )
    args = parser.parse_args()
    convert(args.pt, args.onnx)

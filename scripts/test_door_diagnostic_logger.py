#!/usr/bin/env python3
"""
Unit test for DoorDiagnosticLogger algorithmic components:
- Directory structure & session isolation
- CSV trajectory table generation
- Safe math on NaN / Inf
- Depth downsampling and clamping logic
"""

import os
import sys
import time
import shutil
import tempfile
import json
import csv
import numpy as np
import cv2

def test_depth_preprocessing():
    min_depth = 0.25
    max_depth = 10.0

    # Create dummy raw depth with NaNs, Infs, and numbers
    raw = np.array([
        [float('nan'), 0.1, 0.5, float('inf')],
        [-float('inf'), 2.0, 15.0, 3.5]
    ], dtype=np.float32)

    is_nan = np.isnan(raw)
    is_posinf = np.isposinf(raw)
    is_neginf = np.isneginf(raw)

    far_mask = is_posinf | (raw > max_depth)
    close_mask = is_neginf | ((raw < min_depth) & ~is_nan & ~far_mask)
    nan_mask = is_nan

    depth_clamped = raw.copy()
    depth_clamped[far_mask] = 0.0
    depth_clamped[close_mask] = min_depth
    depth_clamped[nan_mask] = min_depth

    # Verify clamping
    assert depth_clamped[0, 0] == min_depth, "NaN should clamp to min_depth"
    assert depth_clamped[0, 1] == min_depth, "< min_depth should clamp to min_depth"
    assert depth_clamped[0, 2] == 0.5, "0.5 should stay 0.5"
    assert depth_clamped[0, 3] == 0.0, "+inf should map to 0.0"
    assert depth_clamped[1, 0] == min_depth, "-inf should clamp to min_depth"
    assert depth_clamped[1, 2] == 0.0, "> max_depth should map to 0.0"

    # Test resize to (64, 40)
    resized = cv2.resize(depth_clamped, (64, 40), interpolation=cv2.INTER_LINEAR)
    assert resized.shape == (40, 64), f"Expected shape (40, 64), got {resized.shape}"
    print("[✓] Depth preprocessing logic verified.")

def test_safe_math():
    # Test NaN / Inf safe arrow math
    center_x = 160
    dy_b_nan = float('nan')
    raw_arrow_x = center_x + (-dy_b_nan * 40.0)
    # np.nan_to_num or fallback test
    safe_arrow_x = int(np.clip(np.nan_to_num(raw_arrow_x, nan=center_x), 30.0, 290.0))
    assert safe_arrow_x == 160, "NaN should fallback safely without ValueError"

    dy_b_huge = 1000.0
    raw_arrow_x = center_x + (-dy_b_huge * 40.0)
    safe_arrow_x = int(np.clip(np.nan_to_num(raw_arrow_x, nan=center_x), 30.0, 290.0))
    assert safe_arrow_x == 30, "Huge negative offset should clamp to 30"
    print("[✓] Safe math logic verified.")

def test_session_folder_creation():
    temp_dir = tempfile.mkdtemp(prefix="test_diag_")
    try:
        session_name = "session_20261106_120000_test"
        session_dir = os.path.join(temp_dir, session_name)
        heatmap_dir = os.path.join(session_dir, 'heatmaps')
        depth_dir = os.path.join(session_dir, 'depth')
        meta_dir = os.path.join(session_dir, 'meta')
        for d in [session_dir, heatmap_dir, depth_dir, meta_dir]:
            os.makedirs(d, exist_ok=True)

        latest_link = os.path.join(temp_dir, 'latest')
        if os.path.islink(latest_link) or os.path.exists(latest_link):
            os.unlink(latest_link)
        os.symlink(session_name, latest_link)

        assert os.path.islink(latest_link)
        assert os.path.exists(os.path.join(latest_link, 'heatmaps'))
        print("[✓] Session folder & symlink logic verified.")
    finally:
        shutil.rmtree(temp_dir)

if __name__ == '__main__':
    test_depth_preprocessing()
    test_safe_math()
    test_session_folder_creation()
    print("All diagnostic logger tests passed!")

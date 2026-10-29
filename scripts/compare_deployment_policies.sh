#!/usr/bin/env bash
# ==============================================================================
# Compare Success Rates of Three Deployment Navigation Policies on Go2W
# 1. ~/go2w_robot_ref/jetson_111/sru_ws_src/sru-robot-deployment/rl_nav_controller/deployment_policies/nav_policy.onnx
# 2. ~/SRU-Odin/Deployment/models/nav_policy.onnx
# 3. ~/sru-robot-deployment/rl_nav_controller/deployment_policies/nav_policy.onnx
#
# Standardized across identical episode count (--num_episodes) and seed
# ==============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
ISAACLAB_DIR="$REPO_ROOT/IsaacLab"
CONDA_ENV_DIR="/home/yichangfeng/miniforge3/envs/sru-learning"

TASK="${TASK:-Isaac-Nav-MDPO-Go2W-v0}"
NUM_ENVS="${NUM_ENVS:-16}"
NUM_EPISODES="${NUM_EPISODES:-100}"
MAX_STEPS="${MAX_STEPS:-2000}"
SEED="${SEED:-42}"

MODEL_GO2W_REF="/home/yichangfeng/go2w_robot_ref/jetson_111/sru_ws_src/sru-robot-deployment/rl_nav_controller/deployment_policies/nav_policy.onnx"
MODEL_SRU_ODIN="/home/yichangfeng/SRU-Odin/Deployment/models/nav_policy.onnx"
MODEL_SRU_DEPLOY="/home/yichangfeng/sru-robot-deployment/rl_nav_controller/deployment_policies/nav_policy.onnx"

OUT_DIR="$REPO_ROOT/benchmark_results"
mkdir -p "$OUT_DIR"
JSON_GO2W_REF="$OUT_DIR/eval_go2w_ref.json"
JSON_SRU_ODIN="$OUT_DIR/eval_sru_odin.json"
JSON_SRU_DEPLOY="$OUT_DIR/eval_sru_deploy.json"

cd "$ISAACLAB_DIR"
export CONDA_PREFIX="$CONDA_ENV_DIR"
export PATH="$CONDA_ENV_DIR/bin:$PATH"
export TERM=xterm

echo "======================================================================"
echo "[1/3] Evaluating Model 1: Go2W Robot Ref Policy (Target: $NUM_EPISODES eps)"
echo "      $MODEL_GO2W_REF"
echo "======================================================================"
./isaaclab.sh -p eval_go2_policy_on_go2w.py \
    --task "$TASK" \
    --num_envs "$NUM_ENVS" \
    --num_steps "$MAX_STEPS" \
    --num_episodes "$NUM_EPISODES" \
    --seed "$SEED" \
    --onnx_path "$MODEL_GO2W_REF" \
    --output_json "$JSON_GO2W_REF" \
    --headless

echo "======================================================================"
echo "[2/3] Evaluating Model 2: SRU-Odin Upstream Policy (Target: $NUM_EPISODES eps)"
echo "      $MODEL_SRU_ODIN"
echo "======================================================================"
./isaaclab.sh -p eval_go2_policy_on_go2w.py \
    --task "$TASK" \
    --num_envs "$NUM_ENVS" \
    --num_steps "$MAX_STEPS" \
    --num_episodes "$NUM_EPISODES" \
    --seed "$SEED" \
    --onnx_path "$MODEL_SRU_ODIN" \
    --output_json "$JSON_SRU_ODIN" \
    --headless

echo "======================================================================"
echo "[3/3] Evaluating Model 3: sru-robot-deployment Policy (Target: $NUM_EPISODES eps)"
echo "      $MODEL_SRU_DEPLOY"
echo "======================================================================"
./isaaclab.sh -p eval_go2_policy_on_go2w.py \
    --task "$TASK" \
    --num_envs "$NUM_ENVS" \
    --num_steps "$MAX_STEPS" \
    --num_episodes "$NUM_EPISODES" \
    --seed "$SEED" \
    --onnx_path "$MODEL_SRU_DEPLOY" \
    --output_json "$JSON_SRU_DEPLOY" \
    --headless

echo "======================================================================"
echo "All 3 evaluations finished! Generating comparison summary..."
echo "======================================================================"

python3 -c "
import json
import os

files = [
    ('go2w_robot_ref', '$JSON_GO2W_REF'),
    ('SRU-Odin', '$JSON_SRU_ODIN'),
    ('sru-robot-deployment', '$JSON_SRU_DEPLOY'),
]

data = {}
for name, p in files:
    if os.path.exists(p):
        with open(p) as f:
            data[name] = json.load(f)

print('\n' + '='*85)
print(f'{\"Metric\":<32} | {\"go2w_robot_ref\":<15} | {\"SRU-Odin\":<15} | {\"sru-robot-deploy\":<15}')
print('-'*85)

keys = [
    ('Total Episodes', lambda d: str(d.get('total_episodes'))),
    ('Success Goals Reached', lambda d: str(d.get('reach_goals'))),
    ('Success Rate (%)', lambda d: f\"{d.get('success_rate'):.1f}%\"),
    ('Collision/Failure Rate (%)', lambda d: f\"{d.get('collision_rate'):.1f}%\"),
    ('Mean Forward Speed vx (m/s)', lambda d: f\"{d.get('mean_vx'):.3f}\"),
    ('Mean Lateral Speed vy (m/s)', lambda d: f\"{d.get('mean_vy'):.3f}\"),
    ('Mean Yaw Rate wz (rad/s)', lambda d: f\"{d.get('mean_wz'):.3f}\"),
    ('Mean Goal Distance (m)', lambda d: f\"{d.get('mean_goal_distance'):.2f}\"),
]

for label, fn in keys:
    v1 = fn(data['go2w_robot_ref']) if 'go2w_robot_ref' in data else 'N/A'
    v2 = fn(data['SRU-Odin']) if 'SRU-Odin' in data else 'N/A'
    v3 = fn(data['sru-robot-deployment']) if 'sru-robot-deployment' in data else 'N/A'
    print(f'{label:<32} | {v1:<15} | {v2:<15} | {v3:<15}')

print('-'*85)
print('Termination Breakdown:')
terms = ['early_termination', 'base_contact', 'large_pitch_angle', 'terrain_fall', 'time_out']
for t in terms:
    v1 = str(data['go2w_robot_ref']['term_counts'].get(t, 0)) if 'go2w_robot_ref' in data else 'N/A'
    v2 = str(data['SRU-Odin']['term_counts'].get(t, 0)) if 'SRU-Odin' in data else 'N/A'
    v3 = str(data['sru-robot-deployment']['term_counts'].get(t, 0)) if 'sru-robot-deployment' in data else 'N/A'
    print(f'  - {t:<28} | {v1:<15} | {v2:<15} | {v3:<15}')
print('='*85 + '\n')
"

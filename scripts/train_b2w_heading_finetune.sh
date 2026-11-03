#!/usr/bin/env bash
# ==============================================================================
# Fine-tune B2W Navigation Policy for Goal Heading Control
# Hot-started from: logs/warmstart_nav_policy_heading.pt (expanded from nav_policy.onnx)
# ==============================================================================
set -e

ISAACLAB_DIR="/home/yichangfeng/sru-navigation-learning/IsaacLab"
CONDA_ENV_DIR="/home/yichangfeng/miniforge3/envs/sru-learning"

ALGO="${ALGO:-mdpo}"
ALGO_UPPER=$(echo "$ALGO" | tr '[:lower:]' '[:upper:]')
TASK="${TASK:-Isaac-Nav-${ALGO_UPPER}-B2W-Heading-v0}"
NUM_ENVS="${NUM_ENVS:-1024}"
MAX_ITER="${MAX_ITER:-800}"
SEED="${SEED:-42}"
RUN_NAME="${RUN_NAME:-${ALGO}_b2w_heading_finetune}"
GUI="${GUI:-0}"
CHECKPOINT="${CHECKPOINT:-/home/yichangfeng/sru-navigation-learning/IsaacLab/logs/warmstart_nav_policy_heading.pt}"

HEADLESS_FLAG="--headless"
if [[ "$GUI" == "1" ]]; then
  HEADLESS_FLAG=""
fi

CHECKPOINT_FLAG=""
if [[ -n "$CHECKPOINT" ]]; then
  CHECKPOINT_FLAG="--checkpoint $CHECKPOINT"
fi

cat <<INFO
==================================================================
 Starting B2W Goal Heading Fine-tuning (Warmstart)
 Algorithm   : $ALGO_UPPER
 Task        : $TASK
 Envs        : $NUM_ENVS
 Max Iter    : $MAX_ITER
 Seed        : $SEED
 Run Name    : $RUN_NAME
 Checkpoint  : $CHECKPOINT
 GUI         : $GUI
==================================================================
INFO

cd "$ISAACLAB_DIR"
export CONDA_PREFIX="$CONDA_ENV_DIR"
export PATH="$CONDA_ENV_DIR/bin:$PATH"
export TERM=xterm

./isaaclab.sh -p source/isaaclab_nav_task/scripts/train.py \
    --task "$TASK" \
    --num_envs "$NUM_ENVS" \
    --max_iterations "$MAX_ITER" \
    --seed "$SEED" \
    --run_name "$RUN_NAME" \
    $HEADLESS_FLAG \
    $CHECKPOINT_FLAG

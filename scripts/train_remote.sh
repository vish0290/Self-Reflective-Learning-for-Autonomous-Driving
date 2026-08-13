#!/bin/bash
# Remote training script for Mac mini
# Runs training in tmux session for 24/7 operation

set -e

# Configuration
DATASET_DIR="${1:-./datasets/dino_training_data}"
OUTPUT_DIR="${2:-./checkpoints/dino_driver}"
EPOCHS="${3:-50}"
BATCH_SIZE="${4:-32}"

if [ ! -d "$DATASET_DIR" ]; then
    echo "Dataset not found: $DATASET_DIR"
    echo "Usage: $0 <dataset_dir> [output_dir] [epochs] [batch_size]"
    exit 1
fi

# Activate venv
source venv/bin/activate

# Create tmux session
SESSION_NAME="dino-training"

# Kill existing session if it exists
tmux kill-session -t $SESSION_NAME 2>/dev/null || true

# Start new tmux session
tmux new-session -d -s $SESSION_NAME

# Run training in tmux
tmux send-keys -t $SESSION_NAME "source venv/bin/activate" C-m
tmux send-keys -t $SESSION_NAME "python 'model training/train_dino_driver.py' \\
  --data '$DATASET_DIR' \\
  --output '$OUTPUT_DIR' \\
  --epochs $EPOCHS \\
  --batch-size $BATCH_SIZE \\
  --mlflow \\
  --mlflow-uri './mlruns' \\
  --workers 4" C-m

echo "=== Training Started ==="
echo
echo "Session: $SESSION_NAME"
echo "Dataset: $DATASET_DIR"
echo "Output: $OUTPUT_DIR"
echo "Epochs: $EPOCHS"
echo "Batch size: $BATCH_SIZE"
echo
echo "Commands:"
echo "  Attach:  tmux attach -t $SESSION_NAME"
echo "  Detach:  Ctrl+B, then D"
echo "  Logs:    tail -f $OUTPUT_DIR/train.log"
echo "  MLflow:  mlflow ui --backend-store-uri ./mlruns"
echo

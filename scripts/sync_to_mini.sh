#!/bin/bash
# Sync code and dataset to Mac mini
# Usage: ./scripts/sync_to_mini.sh user@mac-mini-hostname

REMOTE="$1"

if [ -z "$REMOTE" ]; then
    echo "Usage: $0 user@hostname"
    echo "Example: $0 vish@mac-mini.local"
    exit 1
fi

REMOTE_DIR="~/server_side"

echo "=== Syncing to Mac mini ==="
echo "Remote: $REMOTE"
echo "Directory: $REMOTE_DIR"
echo

# Create remote directory
ssh "$REMOTE" "mkdir -p $REMOTE_DIR"

# Sync code (exclude datasets, checkpoints, cache)
echo "Syncing code..."
rsync -avz --progress \
  --exclude 'datasets/' \
  --exclude 'checkpoints/' \
  --exclude '*.pt' \
  --exclude '*.pth' \
  --exclude '__pycache__/' \
  --exclude '.git/' \
  --exclude 'mlruns/' \
  --exclude 'venv/' \
  --exclude 'backup_*/' \
  core/ "model training/" scripts/ \
  "$REMOTE:$REMOTE_DIR/"

echo
echo "Code synced. Now sync dataset:"
echo "  rsync -avz --progress datasets/dino_training_data/ $REMOTE:$REMOTE_DIR/datasets/dino_training_data/"
echo
echo "Then SSH and run setup:"
echo "  ssh $REMOTE"
echo "  cd $REMOTE_DIR"
echo "  ./scripts/setup_mac_mini.sh"
echo "  ./scripts/train_remote.sh ./datasets/dino_training_data"
echo

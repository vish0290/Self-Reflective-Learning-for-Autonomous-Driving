#!/bin/bash
# Setup script for Mac mini training environment
# Run this on the Mac mini after copying the code

set -e

echo "=== Mac Mini Training Setup ==="
echo

# Check Python
if ! command -v python3 &> /dev/null; then
    echo "Python 3 not found. Install it first."
    exit 1
fi

PYTHON_VERSION=$(python3 --version)
echo "Python: $PYTHON_VERSION"

# Create virtual environment
if [ ! -d "venv" ]; then
    echo "Creating virtual environment..."
    python3 -m venv venv
fi

source venv/bin/activate

# Install dependencies
echo "Installing dependencies..."
pip install --upgrade pip
pip install torch torchvision torchaudio  # MPS support on Apple Silicon
pip install pillow numpy mlflow

echo
echo "=== Setup Complete ==="
echo
echo "To start training:"
echo "  1. Copy dataset to Mac mini"
echo "  2. Run: ./scripts/train_remote.sh"
echo

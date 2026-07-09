#!/bin/bash
# Run Isaac Sim in headless mode

echo "🎮 Starting Isaac Sim (Headless)..."

cd /workspace

# Run Isaac Sim
/isaac-sim/python.sh train_env.py

echo "✓ Training complete"


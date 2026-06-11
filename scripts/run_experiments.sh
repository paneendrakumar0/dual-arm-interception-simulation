#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

TRIALS="${TRIALS:-24}"
SEED="${SEED:-20260611}"
OUTPUT="${OUTPUT:-outputs/experiments/latest}"

PYTHONPATH=src python3 -m dynamic_dual_arm_sim.experiments \
  --config configs/intercept_demo.json \
  --output "$OUTPUT" \
  --trials "$TRIALS" \
  --seed "$SEED"

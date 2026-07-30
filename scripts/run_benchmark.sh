#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

SEEDS="${SEEDS:-20260611,20260612,20260613,20260614,20260615,20260616,20260617,20260618,20260619,20260620}"
TRIALS_PER_SEED="${TRIALS_PER_SEED:-100}"
WORKERS="${WORKERS:-4}"
OUTPUT="${OUTPUT:-outputs/benchmarks/latest}"

PYTHONPATH=src python3 -m dynamic_dual_arm_sim.benchmark \
  --config configs/intercept_demo.json \
  --output "$OUTPUT" \
  --seeds "$SEEDS" \
  --trials-per-seed "$TRIALS_PER_SEED" \
  --workers "$WORKERS"

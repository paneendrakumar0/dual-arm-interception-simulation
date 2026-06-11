#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

PYTHONPATH=src python3 -m dynamic_dual_arm_sim.run \
  --config configs/intercept_demo.json \
  --output outputs \
  --render

"$ROOT_DIR/scripts/render_video.sh" outputs/frames outputs/dual_arm_interception.mp4

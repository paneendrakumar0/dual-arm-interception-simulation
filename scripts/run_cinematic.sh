#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

OUTPUT="${OUTPUT:-outputs/cinematic}"
VIDEO="${VIDEO:-outputs/dual_arm_interception_cinematic.mp4}"
FPS="${FPS:-24}"

PYTHONPATH=src python3 -m dynamic_dual_arm_sim.run \
  --config configs/intercept_demo.json \
  --output "$OUTPUT" \
  --render

FPS="$FPS" "$ROOT_DIR/scripts/render_video.sh" "$OUTPUT/frames" "$VIDEO"

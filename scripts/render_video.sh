#!/usr/bin/env bash
set -euo pipefail

FRAME_DIR="${1:-outputs/frames}"
OUTPUT_VIDEO="${2:-outputs/dual_arm_interception.mp4}"
FPS="${FPS:-24}"

if ! command -v ffmpeg >/dev/null 2>&1; then
  echo "ffmpeg is required to create the showcase video." >&2
  exit 1
fi

if [ ! -d "$FRAME_DIR" ]; then
  echo "Frame directory does not exist: $FRAME_DIR" >&2
  exit 1
fi

if ! ls "$FRAME_DIR"/frame_*.png >/dev/null 2>&1; then
  echo "No frame_*.png files found in: $FRAME_DIR" >&2
  exit 1
fi

mkdir -p "$(dirname "$OUTPUT_VIDEO")"

ffmpeg -y \
  -framerate "$FPS" \
  -i "$FRAME_DIR/frame_%04d.png" \
  -vf "scale=trunc(iw/2)*2:trunc(ih/2)*2,format=yuv420p" \
  -c:v libx264 \
  -preset medium \
  -crf 18 \
  "$OUTPUT_VIDEO"

echo "Wrote $OUTPUT_VIDEO"

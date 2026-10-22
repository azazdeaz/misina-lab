#!/usr/bin/env bash
# Records the straddler demo's video: films the robot driving down a row in
# Isaac Sim, headless, and writes an mp4 small enough to upload, plus a
# contact sheet to check it by. What it films is the top of video.py.
#
#   examples/straddler_demo/record.sh [out.mp4]
#
# Defaults to target/video/straddler.mp4 at the repo root. Needs uv, ffmpeg and
# an RTX GPU, but no display.
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
out=$(realpath -m "${1:-$here/../../target/video/straddler.mp4}")
mkdir -p "$(dirname "$out")"
raw=$(mktemp --suffix .mp4)
trap 'rm -f "$raw"' EXIT

cd "$here"
uv run video.py "$raw"
"$here/../../docs/video/encode.sh" "$raw" "$out" "scale=1280:-2:flags=lanczos"

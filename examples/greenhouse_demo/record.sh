#!/usr/bin/env bash
# Records the harvest demo's video: films the harvester cutting a path's due
# trusses in Isaac Sim, headless, and writes an mp4 small enough to upload, its
# captions burned in, plus a contact sheet to check it by. What it films is the
# top of video.py.
#
#   examples/greenhouse_demo/record.sh [out.mp4]
#
# Defaults to target/video/harvest.mp4 at the repo root. Needs uv, ffmpeg and
# an RTX GPU, but no display.
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
out=$(realpath -m "${1:-$here/../../target/video/harvest.mp4}")
mkdir -p "$(dirname "$out")"
raw=$(mktemp --suffix .mp4)
trap 'rm -f "$raw" "${raw%.mp4}.srt"' EXIT

cd "$here"
uv run video.py "$raw"
# Down to the README video's size, the captions on a box in the bottom left.
style="FontName=Noto Sans,FontSize=11,Bold=1,Shadow=0,Alignment=1,MarginL=16,MarginV=14"
style+=",BorderStyle=4,BackColour=&H70000000,OutlineColour=&H70000000,Outline=5"
"$here/../../docs/video/encode.sh" "$raw" "$out" \
    "scale=1280:-2:flags=lanczos,subtitles=${raw%.mp4}.srt:force_style='$style'"

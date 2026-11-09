#!/usr/bin/env bash
# Records a viewer video: plays a storyboard in a generator's viewer, frame by
# frame, and writes an mp4 small enough to upload plus a contact sheet to
# check it by. How to change what it shows: .claude/skills/readme-video/SKILL.md.
#
#   docs/video/record.sh [package] [tour.json] [out.mp4]
#
# `package` is the generator's crate, vinery-lab by default. The storyboard
# defaults to the one named after it beside this script, the video to
# target/video/{package}.mp4.
# Needs cargo and ffmpeg, and no display: the viewer opens no window, and
# draws into an image at the storyboard's size.
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
package=${1:-vinery-lab}
tour=$(realpath "${2:-$here/$package.json}")
out=$(realpath -m "${3:-$here/../../target/video/$package.mp4}")
mkdir -p "$(dirname "$out")"
raw=$(mktemp --suffix .mp4)
trap 'rm -f "$raw"' EXIT

# The viewer's variables are named after its Python package: VINERY_LAB_TOUR.
prefix=${package//-/_}
prefix=${prefix^^}
cd "$here/../.."
env "${prefix}_TOUR=$tour" "${prefix}_RECORD=$raw" \
    cargo run --release --quiet -p "$package" --bin "$package"
# The recorder's file is near-lossless and fragmented; this one is the upload's.
"$here/encode.sh" "$raw" "$out"

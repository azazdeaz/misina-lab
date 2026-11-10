#!/usr/bin/env bash
# Encodes a recording into a video GitHub takes, and a contact sheet to check
# it by. Every README video goes through it.
#
#   docs/video/encode.sh in.mp4 out.mp4 [filters]
#
# `filters` is an ffmpeg filter chain applied on the way, e.g. a scale. The
# sheet is out-sheet.png beside the video: a frame every two seconds from the
# first, four to a row.
set -euo pipefail
raw=$1 out=$2 filters=${3:-null}

# At a constant quality, so the size follows the picture: a minute of the
# greenhouse at 1080p is about 55 MB, and GitHub takes 100 on a paid plan.
# H.264, which every browser plays. It starts playing before it has loaded.
ffmpeg -y -loglevel warning -i "$raw" -vf "$filters" \
    -c:v libx264 -preset veryslow -crf 22 -pix_fmt yuv420p -movflags +faststart "$out"
secs=$(ffprobe -v error -show_entries format=duration -of csv=p=0 "$out")
rows=$(( (${secs%.*} / 2 + 4) / 4 ))
ffmpeg -y -loglevel warning -i "$out" \
    -vf "select='isnan(prev_selected_t)+gte(t-prev_selected_t,2)',scale=480:-2,tile=4x$rows" \
    -frames:v 1 -update 1 "${out%.*}-sheet.png"
echo "$out ($(numfmt --to=si --suffix=B "$(stat -c %s "$out")")), ${out%.*}-sheet.png"

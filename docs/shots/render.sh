#!/usr/bin/env bash
# Renders the docs' pictures from their shots into target/docs-assets, laid out
# as the assets repository is: docs/shots/A/B.json becomes A/B.webp, which
# publish.sh pushes. How to add or change a shot: .claude/skills/docs-shots/SKILL.md.
#
#   docs/shots/render.sh [shot.json ...]
#
# Renders every shot, into an emptied folder, when none is named. A shot is
# rendered by what its folder is named after: an example with a shot.py, in
# Isaac Sim, or else a crate's viewer, which plays the shot as a storyboard.
# Needs cargo, uv and ffmpeg, a display for the viewer and an RTX GPU for
# Isaac Sim, and for butom_capture's, BUTOM: see its shot.py.
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
repo=$(cd "$here/../.." && pwd)
out=$repo/target/docs-assets
if (( $# == 0 )); then
    rm -rf "$out"
    set -- "$here"/*/*.json
fi
png=$(mktemp --suffix .png)
trap 'rm -f "$png"' EXIT

for shot in "$@"; do
    shot=$(realpath "$shot")
    [[ $shot == "$here"/*/*.json ]] || { echo "$shot is not a shot in $here" >&2; exit 1; }
    name=${shot#"$here"/}
    name=${name%.json}
    app=${name%%/*}
    # Gone until the renderer writes it, so a run that saves nothing fails below.
    rm -f "$png"
    if [[ -f $repo/examples/$app/shot.py ]]; then
        (cd "$repo/examples/$app" && uv run shot.py "$shot" "$png")
    else
        # The viewer's variables are named after its package: VINERY_LAB_TOUR.
        prefix=$(tr 'a-z-' 'A-Z_' <<< "$app")
        (cd "$repo" && env "${prefix}_TOUR=$shot" "${prefix}_SHOT=$png" \
            cargo run --release --quiet -p "$app")
    fi
    mkdir -p "$(dirname "$out/$name")"
    ffmpeg -y -loglevel error -i "$png" -c:v libwebp -quality 90 "$out/$name.webp"
    echo "$out/$name.webp ($(numfmt --to=si --suffix=B "$(stat -c %s "$out/$name.webp")"))"
done

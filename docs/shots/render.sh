#!/usr/bin/env bash
# Renders the docs' pictures from their shots into target/docs-assets, laid out
# as the assets repository is: docs/shots/A/B.json becomes A/B.webp, which
# publish.sh pushes. How to add or change a shot: .claude/skills/docs-shots/SKILL.md.
#
#   docs/shots/render.sh [shot.json ...]
#
# Renders every shot, into an emptied folder, when none is named. A shot is
# rendered by what its folder is named after: an example with a shot.py, in
# Isaac Sim, or else a crate's viewer, which plays the shot as a storyboard;
# one in montage/ is other shots' pictures put together, see below.
# Needs cargo, uv, ffmpeg and jq, an RTX GPU for Isaac Sim, and for
# butom_capture's, BUTOM: see its shot.py.
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
repo=$(cd "$here/../.." && pwd)
out=$repo/target/docs-assets
if (( $# == 0 )); then
    rm -rf "$out"
    # Montages last, from the pictures rendered before them.
    for shot in "$here"/*/*.json; do [[ $shot == */montage/* ]] || set -- "$@" "$shot"; done
    set -- "$@" "$here"/montage/*.json
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
    if [[ $app == montage ]]; then
        # The pictures of the shots in `tiles`, `columns` to a row, each scaled
        # and cropped to `tile` pixels, `gap` apart with nothing between. A
        # tile's picture the folder lacks is rendered first; one it holds is
        # used as it is. A tile is a shot of an example or a crate, never a
        # montage.
        layout=$(jq -r '[.columns, .tile[], .gap] | @tsv' "$shot")
        read -r columns width height gap <<< "$layout"
        names=$(jq -r '.tiles[]' "$shot")
        mapfile -t names <<< "$names"
        for tile in "${names[@]}"; do
            [[ $tile != montage/* ]] || { echo "$shot: a tile is not a montage: $tile" >&2; exit 1; }
        done
        inputs=() graph= tiles= n=0
        for tile in "${names[@]}"; do
            [[ -f $out/$tile.webp ]] || "$0" "$here/$tile.json"
            inputs+=(-i "$out/$tile.webp")
            graph+="[$n]scale=$width:$height:force_original_aspect_ratio=increase,"
            graph+="crop=$width:$height,setsar=1,format=rgba[t$n];"
            tiles+="[t$n]"
            n=$((n + 1))
        done
        rows=$(((n + columns - 1) / columns))
        ffmpeg -y -loglevel error "${inputs[@]}" -filter_complex \
            "$graph${tiles}concat=n=$n,tile=${columns}x$rows:padding=$gap:color=0x00000000" \
            -frames:v 1 "$png"
    elif [[ -f $repo/examples/$app/shot.py ]]; then
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

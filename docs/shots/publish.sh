#!/usr/bin/env bash
# Publishes target/docs-assets, which render.sh fills, as the one commit on main
# of github.com/azazdeaz/misina-lab-docs-assets, force-pushed: the repository
# holds the current set of pictures and nothing older. The docs link each one
# as https://raw.githubusercontent.com/azazdeaz/misina-lab-docs-assets/main/<path>,
# which serves a new set within about five minutes, its cache time.
#
#   docs/shots/publish.sh
#
# A picture this checkout's Markdown links there that the folder lacks is kept
# as published, so only the shots a change affects need rendering. Refuses
# while a linked picture is neither rendered nor published, as a new shot's is
# until it is rendered. Needs push access.
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
repo=$(cd "$here/../.." && pwd)
assets=$repo/target/docs-assets
raw=https://raw.githubusercontent.com/azazdeaz/misina-lab-docs-assets/main/
remote=https://github.com/azazdeaz/misina-lab-docs-assets.git

cd "$repo"
missing= kept=
# Every picture the Markdown links: link targets, `](...)`, and not the
# example URLs the docs show in code.
while read -r link; do
    path=${link#*"$raw"}
    [[ -f $assets/$path ]] && continue
    # Not rendered here: the pages keep the copy they show now.
    if curl -fsSL --create-dirs -o "$assets/$path" "$raw$path"; then
        kept+=" $path"
    else
        rm -f "$assets/$path"
        missing+=$path$'\n'
    fi
done < <(git grep --untracked -ohE "[]][(]$raw[^)[:space:]]+" -- '*.md' | sort -u)
[[ -z $missing ]] || { printf 'neither in %s nor published:\n%s' "$assets" "$missing" >&2; exit 1; }
source=$(git describe --always --dirty)

# A throwaway repository over the folder, which stays a plain folder.
GIT_DIR=$(mktemp -d)
export GIT_DIR GIT_WORK_TREE=$assets
trap 'rm -rf "$GIT_DIR"' EXIT
cd "$assets"
git init -q -b main
git add -A
git commit -q -m "Rendered from misina-lab $source" ${kept:+-m "Kept as published:$kept"}
git push --force "$remote" main

#!/usr/bin/env bash
# Publishes target/docs-assets, which render.sh fills, as the one commit on main
# of github.com/azazdeaz/misina-lab-docs-assets, force-pushed: the repository
# holds the current set of pictures and nothing older. The docs link each one
# as https://raw.githubusercontent.com/azazdeaz/misina-lab-docs-assets/main/<path>,
# which serves a new set within about five minutes, its cache time.
#
#   docs/shots/publish.sh
#
# Refuses while the folder lacks a picture that this checkout's Markdown links
# there, since publishing would take it down. Needs push access.
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
repo=$(cd "$here/../.." && pwd)
assets=$repo/target/docs-assets
raw=https://raw.githubusercontent.com/azazdeaz/misina-lab-docs-assets/main/
remote=https://github.com/azazdeaz/misina-lab-docs-assets.git

cd "$repo"
# Link targets, `](...)`, and not the example URLs the docs show in code.
missing=$(git grep --untracked -ohE "[]][(]$raw[^)[:space:]]+" -- '*.md' | sort -u |
    while read -r link; do
        path=${link#*"$raw"}
        [[ -f $assets/$path ]] || echo "$path"
    done)
[[ -z $missing ]] || { printf 'missing from %s:\n%s\n' "$assets" "$missing" >&2; exit 1; }
source=$(git describe --always --dirty)

# A throwaway repository over the folder, which stays a plain folder.
GIT_DIR=$(mktemp -d)
export GIT_DIR GIT_WORK_TREE=$assets
trap 'rm -rf "$GIT_DIR"' EXIT
cd "$assets"
git init -q -b main
git add -A
git commit -q -m "Rendered from misina-lab $source"
git push --force "$remote" main

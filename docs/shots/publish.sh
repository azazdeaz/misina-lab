#!/usr/bin/env bash
# Publishes target/docs-assets, which render.sh fills, as the one commit on main
# of github.com/azazdeaz/misina-lab-docs-assets, force-pushed: the repository
# holds the current set of pictures and nothing older. The docs link each one
# as https://raw.githubusercontent.com/azazdeaz/misina-lab-docs-assets/main/<path>,
# which serves a new set within about five minutes, its cache time.
#
#   docs/shots/publish.sh
#
# A shot's picture, or one this checkout's Markdown links there, that the
# folder lacks is kept as published, so only the shots a change affects need
# rendering. Refuses while one is neither rendered nor published, as a new
# shot's is until it is rendered, and when main has moved since it started.
# Needs push access.
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
repo=$(cd "$here/../.." && pwd)
assets=$repo/target/docs-assets
raw=https://raw.githubusercontent.com/azazdeaz/misina-lab-docs-assets/main/
remote=https://github.com/azazdeaz/misina-lab-docs-assets.git

cd "$repo"
# The commit main is on now. Kept pictures are fetched at it, since main's own
# URL serves a copy cached for minutes, and the push expects it, so a publish
# started from an older set fails rather than undoing a newer one.
base=$(git ls-remote "$remote" refs/heads/main | cut -f1)
missing= kept=
while read -r path; do
    # A path in the assets repository, so one inside the folder.
    [[ $path != *..* ]] || { echo "not a picture's path: $path" >&2; exit 1; }
    [[ -f $assets/$path ]] && continue
    # Not rendered here: the pages keep the copy they show now.
    if curl -fsSL --create-dirs -o "$assets/$path" "${raw%main/}$base/$path"; then
        kept+=" $path"
    else
        rm -f "$assets/$path"
        missing+=$path$'\n'
    fi
# Every shot's picture, as render.sh names it, and every picture the Markdown
# links: link targets, `](...)`, up to any fragment, and not the example URLs
# the docs show in code.
done < <({
    (cd "$here" && printf '%s\n' */*.json | sed 's/json$/webp/')
    git grep --untracked -ohE "[]][(]${raw}[^)#?[:space:]]+" -- '*.md' | sed "s|^.*$raw||"
} | sort -u)
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
git push --force-with-lease="refs/heads/main:$base" "$remote" main

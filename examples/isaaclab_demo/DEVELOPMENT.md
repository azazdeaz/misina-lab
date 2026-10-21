## Isaac Lab release

`isaaclab[isaacsim,all]` is pinned to one release from NVIDIA's package index.
The straddler, pruning and greenhouse demos pin the same one; bump the four
together.

To move to a newer release:

1. Set the version in `dependencies`.
2. Copy `tools/wheel_builder/uv-overrides.txt`, at that release's tag, into
   `override-dependencies`. These are Isaac Lab's own resolver pins, which its
   wheel does not carry; without them the `isaacsim` extra pulls the stack back
   to versions Isaac Lab needs newer.
3. Re-lock and sync:

       uv lock
       uv sync

Extras can also move between releases.

A commit between releases builds the same package from Git: drop the version
from `dependencies`, add
`isaaclab = { git = "https://github.com/isaac-sim/IsaacLab.git", rev = "<commit>", subdirectory = "tools/wheel_builder" }`
to `[tool.uv.sources]`, and take the overrides from that commit.

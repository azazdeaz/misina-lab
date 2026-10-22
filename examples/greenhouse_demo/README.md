# Greenhouse demo

A [generated greenhouse](../../crates/greenhouse-lab/README.md) in Isaac Sim:
the smallest script that spawns `GreenhouseCfg`, which renders the
greenhouse's picture in Isaac Lab for the docs, and one that bends and cuts
its lowest trusses. No robot works it yet.

**Contents:** [Requirements](#requirements) · [Running it](#running-it) ·
[Layout](#layout)

## Requirements
 - [uv](https://docs.astral.sh/uv/getting-started/installation/) installed
 - [Isaac Sim 6.2](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/installation/requirements.html) compatible hardware
 - [Rust](https://www.rust-lang.org/tools/install) installed (this demo builds the scene generator from source)

## Running it

```bash
uv run shot.py ../../docs/shots/greenhouse_demo/rows.json
```

opens Kit on the docs' picture: the greenhouse, the viewport at the
picture's size and camera. Navigate the viewport and press `P`, and the
terminal prints the camera, ready to paste into the shot. With an output
file after the shot, `shot.py` renders the picture headless instead.

```bash
uv run bend.py
```

opens Kit on one short row whose trusses below 1.8 m are rods. They settle
under their fruit, a capsule standing on the floor sweeps down each side of
the row through them, and once they are back at rest each is cut by its
stem and falls. `--viz none` runs it without a window and prints how far
the trusses were pushed, how close to where they settled they came back,
and how many fell.

The first run of either generates its greenhouse and caches it; later runs
start straight up.

## Layout

| file | what it owns |
| --- | --- |
| `shot.py` | the greenhouse, the sky, and a still of them from a shot file in `docs/shots/greenhouse_demo/`; see [docs-shots](../../.claude/skills/docs-shots/SKILL.md) |
| `bend.py` | a row with bent trusses under VBD, the capsules that sweep through them, and the cut |

[`examples/isaaclab_demo/DEVELOPMENT.md`](../isaaclab_demo/DEVELOPMENT.md)
covers the pinned Isaac Lab release, which this demo pins too.

# Greenhouse demo

A [generated greenhouse](../../crates/greenhouse-lab/README.md) in Isaac Sim:
the smallest script that spawns `GreenhouseCfg`, which renders the
greenhouse's picture in Isaac Lab for the docs, one that bends and cuts its
lowest trusses, and a pipe-rail robot harvesting them.

![The harvester cutting a truss in Isaac Lab](https://raw.githubusercontent.com/azazdeaz/misina-lab-docs-assets/main/greenhouse_demo/harvest.webp)

**Contents:** [Requirements](#requirements) · [Running it](#running-it) ·
[Harvesting](#harvesting) · [Recording the video](#recording-the-video) ·
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

```bash
uv run harvest.py
```

opens Kit on two short rows and the harvester at their heads, which cuts
every due truss below 1.8 m, where the trusses are rods, and lets it fall,
as the next section tells.
`--paths` picks the paths it works, every one by default, and
`HEADLESS=1 uv run harvest.py --viz kit` runs it without a window and
prints the tally once the last truss has fallen.

The first run of each generates its greenhouse and caches it; later runs
start straight up.

## Harvesting

https://github.com/user-attachments/assets/1c498746-754b-422e-ab37-57b7c7bb456f

The robot has the layout of eternal.ag's Harvester and most robots of its
kind: a trolley as wide as the heating pipes are spaced, a lift on it, a
UR5e on the lift and a cutter in its hand. The trolley does not roll here: it
rides prismatic joints off a base fixed in the main path, across the row
heads to a path, down it and up. The arm is
[MuJoCo Menagerie's UR5e](https://github.com/google-deepmind/mujoco_menagerie/tree/main/universal_robots_ur5e),
read out of its MJCF; the first run downloads it from the
[misina-lab-sim-assets](https://huggingface.co/datasets/azazdeaz/misina-lab-sim-assets)
dataset on Hugging Face, and later runs find it in the local cache.

The plan comes from the scene's own ground truth, not from a camera:

1. **What is due.** A truss is due once its least ripe fruit is turning, by
   the stage the generator gives every fruit.
2. **The order.** The robot crosses to each path in turn, cuts the due
   trusses on its left going down it and the ones on its right coming back,
   folding the arm before any run longer than half a meter, so it never
   sweeps the crop wall.
3. **The approach.** Each truss is cut at its `Cut` frame, where the
   peduncle leaves the stem. The cutter's V comes in level and square to
   the peduncle, so the peduncle lies across it. The trolley runs to a
   station behind the cut, the arm moves to a stand-off 12 cm short of it,
   and closes in on a straight line.
4. **The cut.** Once the arm holds still with the mouth within 1.5 cm of the
   cut, the truss is cut there by rule, with `Shears`, at its rod's first
   joint, and drops whole to the floor. The blades are drawn only, so the
   truss falls past them. The forearm, the wrists and the cutter's head
   collide with the trusses they pass; the rest of the robot passes through
   them.

The tally counts the trusses due on the paths worked, how many the arm
reaches, cuts and sees fall, and the seconds per truss cut. On the demo's two
4 m rows, 36 trusses are due on the three paths, and the robot reaches and
cuts every one, each dropping whole to the floor: about 3 s of simulated
time a cut and 150 s for the whole run, which takes about six minutes on an
RTX 5070 Ti.

## Recording the video

```bash
./record.sh
```

films the harvester cutting Path_01's due trusses, the first three close up,
in Isaac Sim's RTX renderer and headless, and writes
`target/video/harvest.mp4` at the repo root, captioned and small enough to
upload, and a contact sheet beside it. Its storyboard -- the shots, the takes,
the pacing -- is the top of `video.py`, and the run fails if a take's truss
was not cut. Run from the repo root on the branch whose pull request changes
the video,
`docs/video/publish.sh target/video/harvest.mp4 examples/greenhouse_demo/README.md`
uploads it and points the video line above at it.

## Layout

| file | what it owns |
| --- | --- |
| `shot.py` | a still from a shot file in `docs/shots/greenhouse_demo/`: the greenhouse under a sky, or the harvester at work; see [docs-shots](../../.claude/skills/docs-shots/SKILL.md) |
| `bend.py` | a row with bent trusses under VBD, the capsules that sweep through them, and the cut |
| `harvest.py` | two rows and the harvester: the ground truth off the stage, the plan, the cuts, the tally |
| `harvester.py` | the robot: its dimensions, the UR5e read out of Menagerie's MJCF, the URDF built from both, the actuators |
| `video.py`, `record.sh` | the video: Path_01 harvested, filmed headless, and its encoding |
| `kinematics.py` | forward and inverse kinematics of the trolley and the arm; the pruning demo's module, kept the same |

[`examples/isaaclab_demo/DEVELOPMENT.md`](../isaaclab_demo/DEVELOPMENT.md)
covers the pinned Isaac Lab release, which this demo pins too.

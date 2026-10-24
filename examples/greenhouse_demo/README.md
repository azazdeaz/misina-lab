# Greenhouse demo

A [generated greenhouse](../../crates/greenhouse-lab/README.md) in Isaac Sim:
the smallest script that spawns `GreenhouseCfg`, which renders the
greenhouse's picture in Isaac Lab for the docs, one that bends and cuts its
lowest trusses, and a pipe-rail robot harvesting them.

![The harvester cutting a truss in Isaac Lab](https://raw.githubusercontent.com/azazdeaz/misina-lab-docs-assets/main/greenhouse_demo/harvest.webp)

**Contents:** [Requirements](#requirements) · [Running it](#running-it) ·
[Harvesting](#harvesting) · [Planning with cuRobo](#planning-with-curobo) ·
[Recording the video](#recording-the-video) · [Layout](#layout)

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
start straight up. The first harvest also fetches
[cuRobo](https://github.com/NVlabs/curobo) from GitHub and compiles its
kernels for the GPU it finds, into `~/.cache/curobo`; no CUDA toolkit is
needed.

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
   and folds the arm before every run of the trolley, so it never sweeps
   the crop wall.
3. **The approach.** Each truss is cut at its `Cut` frame, where the
   peduncle leaves the stem. The cutter's V comes in square to the
   peduncle, so the peduncle lies across it, and within 20° of level,
   square to the stem as well: a standing stem then crosses the V's plane
   at the mouth only and stays clear of the cutter's head. It comes in
   along the row, away from where the arm stands. The trolley runs to a
   station behind the cut, cuRobo plans the arm to a stand-off 12 cm short
   of it, and the arm closes in on a straight line.
4. **The cut.** Once the arm holds still with the mouth within 1.5 cm of the
   cut, the truss is cut there by rule, with `Shears`, at its rod's first
   joint, and drops whole to the floor. The blades are drawn only, so the
   truss falls past them. The forearm, the wrists and the cutter's head
   collide with the trusses they pass, and the arm, the cutter's head and
   the trolley's chassis and mast with each other.

The tally counts the trusses due on the paths worked, how many the arm
reaches, how many of those only by brushing a truss, how many it cuts and
sees fall, and the seconds per truss cut. On the demo's two 4 m rows, 36
trusses are due on the three paths, and the robot reaches and cuts every
one, each dropping whole to the floor, a quarter of them only by brushing a
truss: about 5 s of simulated time a cut and 200 s for the whole run, which
takes about four minutes on an RTX 5070 Ti.

## Planning with cuRobo

[cuRobo](https://github.com/NVlabs/curobo) plans every move of the arm: the
fold before each run of the trolley and the reach to each stand-off. It
plans the arm alone, from the platform out, while the trolley holds its
station. What it collides with is what the simulation collides with:

- **The robot.** `harvester.py` holds the arm's colliders, Menagerie's
  capsules, and the cutter's head as shapes. It writes each into the URDF
  Isaac Sim imports, a capsule as its cylinder and a sphere on each end, and
  hands cuRobo the spheres that cover it, spaced along a capsule no farther
  apart than its radius and grown to cover the dips between them, so a
  pose cuRobo finds clear is clear in the simulation too, the robot's
  collisions with itself included. The blades are covered as well, though
  the simulation lets a cut truss fall through them.
- **The world.** `planner.py` reads every collider of the running Newton
  model within the arm's reach, but the arm's own, and hands cuRobo the box
  round each where the simulation has it at that moment: the trolley's
  chassis and mast, the rails, the floor, and the trusses and their fruit,
  bent or fallen.

Bent trusses crowd the crop: a truss's fruit hang just below its cut and
the next truss up a few centimeters above. A cut is planned clear of every
truss first, but for the two segments the cutter closes on; failing that,
the arm may brush the truss it cuts, and failing that any truss, since the
rods bend aside as they do round a picker's arm. The trolley, the rails and
the floor it never touches. The straight line in is solved point by point
from the stand-off, not planned, so it stays on the branch of the arm cuRobo
reached the stand-off on.

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
| `harvester.py` | the robot: its dimensions, the UR5e read out of Menagerie's MJCF, its colliders, the URDF built from both and cuRobo's model of it, the actuators |
| `planner.py` | cuRobo: the world it plans in, read off the running simulation, the fold, the reach to a cut and the straight line in |
| `video.py`, `record.sh` | the video: Path_01 harvested, filmed headless, and its encoding |
| `kinematics.py` | forward and inverse kinematics of the trolley and the arm, for the URDF, the stations and the straight line in; the pruning demo's module, kept the same |

[`examples/isaaclab_demo/DEVELOPMENT.md`](../isaaclab_demo/DEVELOPMENT.md)
covers the pinned Isaac Lab release, which this demo pins too.

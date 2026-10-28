# Greenhouse demo

A [generated greenhouse](../../crates/greenhouse-lab/README.md) in Isaac Sim:
the smallest script that spawns `GreenhouseCfg`, which renders the
greenhouse's picture in Isaac Lab for the docs, one that bends and cuts its
lowest trusses, and a pipe-rail robot harvesting them.

![The harvester cutting a truss in Isaac Lab](https://raw.githubusercontent.com/azazdeaz/misina-lab-docs-assets/main/greenhouse_demo/harvest.webp)

**Contents:** [Requirements](#requirements) · [Running it](#running-it) ·
[Harvesting](#harvesting) · [Cutting and holding](#cutting-and-holding) ·
[Planning with cuRobo](#planning-with-curobo) ·
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
every due truss below 1.8 m, where the trusses are rods, holds it, and
carries it to the crate on the lift, as the next section tells.
`--paths` picks the paths it works, every one by default, and
`HEADLESS=1 uv run harvest.py --viz kit` runs it without a window and
prints the tally once the last truss has fallen. `--colliders` draws every
collider the physics steps, translucent over the scene and kept in step
with the ones moved or resized while it runs, as
[the pruning demo's](../pruning_demo/README.md#seeing-the-colliders) does.

The first run of each generates its greenhouse and caches it; later runs
start straight up. The first harvest also fetches
[cuRobo](https://github.com/NVlabs/curobo) from GitHub and compiles its
kernels for the GPU it finds, into `~/.cache/curobo`; no CUDA toolkit is
needed.

## Harvesting

https://github.com/user-attachments/assets/b080ef8e-e4ce-4aaa-8c71-36345494e7ea

The robot has the layout of eternal.ag's Harvester and most robots of its
kind: a trolley as wide as the heating pipes are spaced, a lift on it, a
UR5e and a crate on the lift and a cut-and-hold shear in its hand. The trolley does not roll here: it
rides prismatic joints off a base fixed in the main path, across the row
heads to a path, down it and up. The arm is
[MuJoCo Menagerie's UR5e](https://github.com/google-deepmind/mujoco_menagerie/tree/main/universal_robots_ur5e),
read out of its MJCF; the first run downloads it from the
[misina-lab-sim-assets](https://huggingface.co/datasets/azazdeaz/misina-lab-sim-assets)
dataset on Hugging Face, and later runs find it in the local cache.

The crate, `harvester.Crate`, is a solid-walled Euro stacking crate,
600 × 400 × 120 mm, standing on the lift behind the arm, turned across
it, under the folded shear and within the arm's reach. It is
drawn with the ribs down its walls, the rim round its top and the foot it
stacks on, and collides as five boxes: the floor, and each wall out to the
ribs' faces. The arm carries every truss it cuts to the crate and lets
it go over its middle.

The plan comes from the scene's own ground truth, not from a camera:

1. **What is due.** A truss is due once its least ripe fruit is turning, by
   the stage the generator gives every fruit.
2. **The order.** The robot crosses to each path in turn, cuts the due
   trusses on its left going down it and the ones on its right coming back,
   and folds the arm before every run of the trolley, so it never sweeps
   the crop wall.
3. **The approach.** Each truss is cut by the stem, at the first joint of
   its rod, which `Cut` marks within a centimetre: the blade crosses the
   peduncle half a centimetre past the joint, and the cut is made at it,
   so the whole truss falls. The blades' plane runs along the stem, a
   centimetre or so out from it, so the peduncle lies across the mouth,
   as square to it as the stem lets it be, and the shear slides past the
   stem rather than through it: a plane square to a peduncle that rises
   or droops off square to the stem would cross the stem a few
   centimetres from the cut. Tilted up to 30° off level, the stem runs behind
   the blades and clear of the shear's head. It comes in along the row, or
   turned 25° about the stem toward the path, with the hook
   above the peduncle, so the truss's weight hangs the peduncle off the
   hook into the mouth, rather than dragging it over the hook's edge; the
   trolley stands behind the cut that way, so the arm reaches away from its
   base. cuRobo plans the arm to a stand-off 12 cm short of the cut, and
   the arm closes in on a straight line 35° below the blades, so the
   peduncle comes in through the middle of the open mouth's entrance,
   under the hook's claw, and rises into the hook's crook; along the
   blades, the claw, curling down past the crook, would sweep through it.
   The robot's weight is compensated, as a cobot's controller compensates
   its own, so the arm keeps to the line rather than sagging a centimetre
   below it, and holds still at its end until the mouth is at the cut.
4. **The cut and the hold.** The shear closes, its moving blade lifting the
   peduncle onto the hook, and cuts it where the blade reaches it, as
   [the pruning demo's shear cuts a cane](../pruning_demo/README.md#how-the-shear-cuts).
   Beside the blades, on the truss's side of the cut, a sprung finger
   riding the cutting blade presses the peduncle against a jaw beside the
   hook as the blade cuts it. A shear that closes on nothing opens, backs
   out and tries once more.
5. **The release.** Held by its peduncle, the truss rides out on the line
   back to the stand-off and 20 cm across the path, clear of the crop
   wall, rising as it goes until the jaws are 42 cm above the crate's
   floor. The arm swings it round its base from there to over the crate's
   middle, the mouth turning about the upright alone, so the truss hangs
   in the jaws as it did when cut: turned so that its weight pulls toward
   the mouth's entrance, it slides out of them. The shear opens over the
   crate and the truss drops into it. One still lying on the open finger
   is tipped off it. The
   forearm, the wrists, the shear and the crate collide with the trusses
   they meet, and the arm, the shear, the crate and the trolley's chassis
   and mast with each other.

The tally counts the trusses due on the paths worked, how many the arm
reaches, how many of those only by brushing a truss, how many it cuts,
holds and lets go, how many end in the crate, and the seconds per truss
cut.
On the demo's two 4 m rows, 36 trusses are due on the three paths. The
robot reaches four in five, a third of those only by brushing a truss; the
stems of the plants beside the rest keep the arm off them. It cuts all but
a few of those it reaches, and four in five of the trusses it cuts ride in
the jaws to the crate and land in it; the rest slip out of the jaws on the
way, or are let go in the path where the shoulder cannot turn far enough
to swing them round (see [Cutting and holding](#cutting-and-holding)).
That is about 5 s of simulated time a cut and 290 s for the whole run,
which takes about seven minutes on an RTX 5070 Ti.

## Cutting and holding

The shear is the pruning demo's bypass shear, `shear.py`, which both demos
keep the same: a cutting blade swinging past a hook, colliding as convex
pieces along them, the cutting blade's in slices that stand at a rod while
the drawn blade closes on through it. The harvester's is smaller, a 6 cm
blade opening 35 degrees, and cuts and holds: 1.6 cm out along the pivot
from the hook, past the cutting blade, a rubber jaw stands beside the hook,
and a rubber finger rides the cutting blade on a joint of its own, held
against the blade by a spring.

A cane the pruning demo cuts is long and gives, and the moving blade
carries it across the mouth onto the hook. A peduncle is short and stiff
between its stem and the cut, so it is brought to the hook instead,
a radius and a millimetre off its edge, halfway along the blade, where
the cutting blade is widest: one the blade does not quite lift onto the
hook is still between the blades as they meet, and is cut then.

The cut is a rule; the hold is the simulation's own. The finger meets the
peduncle as the blade does and stops there, while the blade closes on
through and its spring winds up, so the peduncle is pressed between the
finger and the jaw, with about 10 N once the shear is shut, and friction
there carries the truss once it is cut. The finger is one of the robot's
bodies the solver bending the trusses sees, as the blades are, so the
peduncle pushes it back as it would a real one. The shear opening swings
the finger down and away from under the peduncle, and the truss drops; one
the rubber still holds up on the finger's slope is tipped off by turning
the blades down about the peduncle, as the pruning demo tips out a piece
of cane.

The blade crosses the peduncle half a centimetre past its rod's first
joint, and `Shears.cut` makes the cut at the joint. Now and then the
peduncle shifts under the closing blade far enough that the cut lands a
centimetre or more past the joint, in the segment the jaws hold; that
truss slips out of them and falls where it is cut. Over four runs of the
demo, 6 of the 106 trusses cut were lost from the jaws.

## Planning with cuRobo

[cuRobo](https://github.com/NVlabs/curobo) plans every move of the arm: the
fold before each run of the trolley, the reach to each stand-off and the
swing to the crate. It
plans the arm alone, from the platform out, while the trolley holds its
station. What it collides with is what the simulation collides with:

- **The robot.** `harvester.py` holds the arm's colliders, Menagerie's
  capsules, and the box round each part of the shear as shapes: the head,
  and each blade with the jaw or the finger beside it. It writes the arm's into the URDF Isaac Sim
  imports, a capsule as its cylinder and a sphere on each end, beside the
  shear's own pieces, and hands cuRobo the spheres that cover each shape,
  spaced along a capsule no farther apart than its radius and grown to
  cover the dips between them, so a pose cuRobo finds clear is clear in
  the simulation too, the robot's collisions with itself included. cuRobo
  plans with the shear held open and the finger against its blade.
- **The world.** `planner.py` reads every collider of the running Newton
  model within the arm's reach, but the arm's own, and hands cuRobo the box
  round each where the simulation has it at that moment: the trolley's
  chassis and mast, the crate, the rails, the floor, the stems, and the
  trusses and their fruit, bent or fallen. An upright stem's box reaches
  back to the middle of its row, a fin, so the arm keeps to the path's side
  of the stems and meets the crop only between two, where the trusses
  hang: a stand-off reached round the back of a stem is one cuRobo finds
  no way back out of. A stretch of stem left out by a cut leaves its fin
  out with it.

Bent trusses crowd the crop: a truss's fruit hang just below its cut and
the next truss up a few centimeters above. A cut is planned clear of every
truss first, but for the two segments the shear closes on, and clear of
every stem but the 20 cm either side of the cut of the one it is made
beside, which the shear slides along closer than the spheres cuRobo covers
it in allow; the fold that follows leaves that stretch out too. Failing
that, the arm may brush the truss it cuts, and failing that any truss,
since the rods bend aside as they do round a picker's arm. The trolley, the
rails, the floor and every other stretch of stem it never touches. The
straight lines in, out and across the path are solved point by point, not
planned, so they stay on the branch of the arm cuRobo reached the stand-off
on, and within the joint range cuRobo plans in, so the fold that follows
plans from where they end. The swing to the crate may brush any truss,
which leaves cuRobo fewer to keep the arm off and the mouth less to turn
on the way. Where the arm already touches what cuRobo keeps it off, a stem
say, cuRobo finds no fold, and the arm folds on straight lines in joint
space instead: out of the crop with the shoulder pan held, then round.
Turned while still out, it would sweep along the crop wall into the stems.

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
| `harvest.py` | two rows and the harvester: the ground truth off the stage, the plan, the cuts, the holds and releases, the tally |
| `harvester.py` | the robot: its dimensions, the UR5e read out of Menagerie's MJCF, its colliders, the shear on its flange, the URDF built from them and cuRobo's model of it, the actuators |
| `shear.py` | the cut-and-hold shear: its blades' geometry, their meshes and colliders, and `Stroke`, the blade closing through a peduncle, and the jaw and the sprung finger that hold the truss; the pruning demo's module, kept the same |
| `planner.py` | cuRobo: the world it plans in, read off the running simulation, the moves to joint positions, folded or over the crate, the reach to a cut and the straight lines of the mouth |
| `video.py`, `record.sh` | the video: Path_01 harvested, filmed headless, and its encoding |
| `kinematics.py` | forward and inverse kinematics of the trolley and the arm, for the URDF, the stations and the straight line in; the pruning demo's module, kept the same |

[`examples/isaaclab_demo/DEVELOPMENT.md`](../isaaclab_demo/DEVELOPMENT.md)
covers the pinned Isaac Lab release, which this demo pins too.

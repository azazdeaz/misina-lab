# Isaac Lab

> [!WARNING]
> **Under development.** This page may not reflect the current state of
> the source. We're working on it.

A generated scene goes into Isaac Lab as an ordinary scene asset, like a
robot or a USD file. Each [generator](generators.md), the code that
produces one kind of scene, comes with a **cfg class**, an Isaac Lab config
class. The first time the cfg class is used, it builds the scene and saves
it as a USD file for reuse. Then it spawns that file: Isaac Lab adds it to
the USD stage, the scene the simulation runs. You construct the cfg class
with the parameters you want and declare it in your scene config. The base
class and every helper on this page are in `misina_lab.isaaclab`, the
framework's Isaac Lab package.

The generator's package holds the cfg class, below the generated classes
of its fragments, such as `BoxCfg`:

```python
from typing import ClassVar

import boxlab
from isaaclab.utils.configclass import configclass
from misina_lab.isaaclab import GeneratedSceneCfg


@configclass
class BoxesCfg(GeneratedSceneCfg):
    PARAMS: ClassVar[type] = boxlab.BoxesParams

    # >>> generated: aggregate
    boxes: BoxCfg = BoxCfg()
    FRAGMENTS: ClassVar[tuple[str, ...]] = ("boxes",)
    # <<< generated: aggregate
```

Your scene config uses it:

```python
from boxlab.isaaclab import BoxCfg, BoxesCfg
from isaaclab.assets import AssetBaseCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils.configclass import configclass

BOXES_CFG = BoxesCfg(boxes=BoxCfg(count=5))


@configclass
class MySceneCfg(InteractiveSceneCfg):
    boxes = AssetBaseCfg(prim_path="{ENV_REGEX_NS}/Boxes", spawn=BOXES_CFG)
```

The cfg class extends `GeneratedSceneCfg`. It has one field per
[fragment](generators.md#the-params-aggregate). A fragment is the struct
of parameters of one [element](elements.md), the code that builds one kind
of thing in the scene: `BoxParams` holds the parameters of the boxes. Each
field of the cfg class holds an instance of a Python class with the same
fields as its Rust fragment: `BoxCfg` for `BoxParams`.

A code generator writes the fragment classes, and the lines between the
`# >>> generated` and `# <<< generated` comments, from the Rust structs
([editing-parameters.md](editing-parameters.md)). Do not edit them by
hand, since the next regeneration overwrites them. You write the rest of
the class. The viewer, a window on the scene with a control for every
parameter, has a **Copy Isaac Lab cfg** button. It copies a
[snippet](generators.md#what-the-framework-builds-from-it): Python code
that constructs this class with the viewer's current settings.

On first use, the spawner (`spawn_generated`, the function the cfg class
spawns with) builds the scene from the fragments. It writes the scene to a
USD file in [the cache](#the-cache), and spawns that file through Isaac
Lab's own USD-file spawner. So everything Isaac Lab's `FileCfg` offers
works here too: `scale`, `semantic_tags`, `rigid_props`,
`collision_props`, visual materials. The `prim_path` above, where the scene
goes on the stage, uses `{ENV_REGEX_NS}`. That pattern matches every
environment, Isaac Lab's parallel copies of the task. So the scene is
built once and copied into each environment.

Declare the scene in a scene config with `AssetBaseCfg`, and do not spawn
it onto the stage yourself. Isaac Lab can run its physics on Newton, a GPU
physics engine, and the physics config on this page ([Rods](#rods)) is a
Newton one. Under Newton, Isaac Lab simulates only the assets that a scene
config declares. A scene spawned onto the stage directly is rendered, but
nothing collides with it.

## The contract

The upper-case attributes in the table belong to the generator. Its
subclass sets them on the class, not on an instance: `copy()` and
`replace()` rebuild a cfg from its fields only, so they drop a value set
on an instance.

| Attribute | What it says |
| --- | --- |
| `PARAMS` | the Python class of the [aggregate](generators.md#the-params-aggregate), the struct that contains every fragment: `boxlab.BoxesParams`. The framework finds the rest from it. Each fragment's class comes from a default `PARAMS()`. The compiled extension module, `boxlab._core`, comes from `PARAMS.__module__`. The package, `boxlab`, comes from that module's name |
| `FRAGMENTS` | the names of the fragment fields, in the order the aggregate takes them. These fields are turned into params, and the cache key is made from them. The code generator writes this line |
| `PLANT` | the name every plant's prim takes before its index: `Plant` for `Plant_007`. Required for a scene with [rods](#rods). The rods and wood under one plant's prim form one collision group ([Rods](#rods)) |
| `ROD_STRETCH`, `ROD_DAMPING` | the rod tuning, [below](#rods). The defaults are `rods.ROD_STRETCH` and `rods.ROD_DAMPING` |
| `LONE_ROD_DAMPING` | the damping of a rod under no plant's prim, used in place of `ROD_DAMPING`. `None` damps it like any other rod |
| `without_rods()` | this cfg with nothing flexible in its scene, or `None` if its scene has no rod with these parameters. The default returns `None`; a generator whose scene can have rods overrides it |
| `cache_dir` | where the USD files go. Unset, it is `$<PACKAGE>_CACHE_DIR`, else `$XDG_CACHE_HOME/<package>/scenes`, else `~/.cache/<package>/scenes`: `BOXLAB_CACHE_DIR` and `~/.cache/boxlab/scenes` for `boxlab` |
| `force_regenerate` | build the scene again even when the cache has it, for working on the generator itself |

The fragment fields are plain `@configclass` dataclasses, not the Rust
classes, so `to_dict()`, `replace()` and Isaac Lab's YAML round trips work
on them. `to_params` turns them into the Rust classes when the scene is
built.

Two rules of `configclass` apply to the class you write. Every annotated
attribute needs a value, so the base class gives `PARAMS` the placeholder
`MISSING`, and your subclass replaces it. And a class attribute must not
hold a dataclass *class*, even inside a tuple, because `configclass` takes
such a class for an instance. So `FRAGMENTS` holds the fields' names as
strings.

## The cache

The same parameters give the same file, so a scene is built once and read
from disk after that. The file is `boxes_<key>.usd`. The key is a hash
over each fragment's `to_dict()` and over what identifies the generator:
the extension's version, the size and modification time of the
extension's file and of the USD builder's (`misina_lab.usd.build`), and the
OpenUSD version. So a rebuilt extension gets new files. The rest of the
cfg, such as `rigid_props` or `scale`, is applied to the prim after
spawning and does not change the USD, so it is not part of the key.
Nothing deletes an old file; clear the directory by hand.

Several processes can share a cache directory, such as the ranks of a
distributed training run. They use a lock file, so only one of them builds
a scene at a time. A scene is written under a temporary name and then
renamed, so no process reads a half-written file.

## Rods

A **rod** is how Newton simulates a flexible **organ**, one instance of an
element such as a long stem ([mesh-variations.md](mesh-variations.md)).
It is a chain of capsule-shaped rigid bodies, one per segment, joined by
joints that bend, twist, stretch and shear. Each capsule is as thick as
the organ is drawn there. The first segment is fixed to the plant's
**wood**: its rigid parts, such as a trunk, which are static colliders. A
generator makes an organ flexible by writing it into the scene as a
`Cable`, a curve that Newton's importer turns into a rod
([export.md](export.md#flexible-organs)). Nothing else in a generated
scene moves.

A **solver** is the part of Newton that computes how bodies move in each
step. Only VBD, Newton's vertex block descent solver, can simulate a rod,
and a robot needs MJWarp, Newton's GPU version of MuJoCo. So a scene with
rods and a robot runs both solvers as one coupled solver, made of named
parts. Its part `rigid`
advances the robot with MJWarp, and its part `rods` advances the rods with
VBD. The robot bodies that a rod may touch are copied into the `rods` part
as **proxies**: bodies that move as the robot's bodies move, and push the
rods. `make_physics_cfg_newton` builds this physics config. When
`cfg.without_rods()` is `None`, the scene has no rods, and it builds plain
MJWarp instead:

```python
physics = make_physics_cfg_newton(
    PLANTS_CFG,
    robot="/World/envs/env_.*/Robot",
    contact_bodies=[".*/Robot/gripper_.*"],
    substeps=4,
)
sim_cfg = sim_utils.SimulationCfg(physics=physics)
```

`robot` is a pattern for the robot's prim path, and `contact_bodies` are
patterns for the bodies a rod may touch. Only those bodies become proxies,
so nothing else can bend a rod: a leg not on the list passes through one.
`substeps` is the number of physics substeps in each simulation step. The
rod tuning below assumes four, 1.25 ms each. With one substep, a rod sags
about twice as far.

`broad_phase` is `"explicit"` by default, or `"nxn"` or `"sap"`. It picks
the **broad phase**, the method Newton uses to find which shapes may touch
([cutting.md](cutting.md)). It also decides what a cut piece of rod
collides with.

The physics config you build is not always the one that runs. Isaac Lab's
`--physics` option, a Hydra preset (Isaac Lab's config overrides) or your
own code can replace it before the scene is built. So the spawner checks
the scene against the physics in force when it spawns it:

- If nothing in force can simulate a rod, as under MJWarp alone or PhysX,
  the spawner logs a warning and spawns `without_rods()` instead. That
  scene has the same shapes, with the rods as static meshes, and it is
  cached as a separate file.
- If VBD runs, the spawner raises `TypeError` when the cfg sets no
  `PLANT`. Otherwise it calls `tune_rods` and [`drive_skins`](#skins).

`tune_rods` changes the model before Newton builds it:

- **It softens each rod joint's stretch and shear.** It multiplies their
  stiffness by `ROD_STRETCH`, 0.01. Without this, a rod bends over under
  its own weight and keeps moving instead of settling. Bend and twist keep
  their stiffness, and are damped by `ROD_DAMPING`, 0.1 s, times that
  stiffness. The numbers were measured on rods about a metre long,
  tapering from about a centimetre across to half that; `rods.py` says
  what each one does. To change them, set `ROD_STRETCH`, `ROD_DAMPING` or
  `LONE_ROD_DAMPING` on the cfg class.
- **It puts the shapes into collision groups.** A collision group is a
  number on each shape that decides which other shapes it may collide with
  ([cutting.md](cutting.md)). Each plant's rods and wood share one group,
  so they collide with each other. The static scene and each other body,
  such as a robot's, are in groups that collide with every group. So a rod
  collides with its own plant, the static scene and the robot, and passes
  through every other plant. A rod under no plant's prim counts as a plant
  of its own. Without groups, Newton would list every pair of rod segments
  as a possible contact: about 50 million pairs for ten thousand segments.
- **It attaches the colliders under a segment's prim to the segment's
  body**, with `carry_riders`. A collider placed as a child of a segment's
  prim, such as a leaf's, would otherwise be imported as static and stay
  where it was built. Attached, it moves and falls with the segment. It
  does not collide with its own rod.
- **It lets a rod through the wood it starts inside.** A wood collider,
  the collision shape of a piece of wood, is often thicker and straighter
  than the wood drawn. So the segments next to the fixed one can start
  inside it. The solver would then push them out of the shape the rod was
  built in, on the first step. `let_rods_out_of_their_wood` turns off
  collision between each such segment and that wood collider for the life
  of the model, after a cut too. The rod's other segments still collide
  with that wood.

With the `"explicit"` broad phase, a scene with rods must contain at
least two static collision shapes that are not a ground plane or a height
field, such as two posts. A cut adds an entry to Newton's list of possible
contacts, and the spare entries reserved for cuts are kept on two such
shapes until a cut uses them
([cutting.md](cutting.md#letting-the-two-faces-meet)). Without the two
shapes, the first reset, which builds the model, raises `ValueError`. The
`"nxn"` and `"sap"` broad phases do not need them.

`Shears`, a class in `misina_lab.isaaclab`, cuts a rod at any point while
the simulation runs. Newton cannot add a body once the model is built, so
a cut reuses bodies the model already has ([cutting.md](cutting.md)).
Build `Shears` after the first reset.

## Skins

A **skin** is one mesh along a whole rod that bends smoothly at every
joint, where a tube per segment shows a kink at each
([export.md](export.md#skins)). A skin is bent by its **skeleton**, a USD
joint per segment that says where that part of the mesh is. Newton moves
the segment prims, but nothing moves a skin's skeleton. So a skin keeps
the shape it was built in until something poses it, that is, moves its
skeleton's joints to the segments. `drive_skins` does that, and the
spawner calls it together with `tune_rods`.

From then on, each time Kit's viewport or an RTX camera is about to render
a frame, every skin whose segments moved since it was last posed is posed
again. A step that renders nothing poses nothing. Without Kit, Isaac Sim's
application framework, `drive_skins` does nothing. A uniform `scale` on
the scene is allowed. A skin under a prim that scales unevenly or skews
raises `ValueError` when it is posed.

When `Shears` cuts a rod, `skins.split` splits its skin at the cut. The
part of the mesh before the cut follows the joints before it, and the rest
follows the joints after it. Each part ends at the cut, closed by a cap.
Both parts stay where they were drawn, even on a bent rod.

## Moving a collider

`Shears.colliders(body)` lists a body's collision shapes: their indices in
the model, and each one's pose relative to the body. A pose is one array of
7 numbers, a position (x, y, z) and then a rotation quaternion
(x, y, z, w). `Shears.place(shape, pose)` moves one shape to a new pose
relative to its body while the simulation runs. It writes the pose into
every copy of the model that the solvers use, as a cut does.

A tool that closes on a rod, such as a blade or a gripper jaw, needs this.
A rigid jaw that keeps closing presses the rod against whatever is behind
it, and the solver pushes the rod out sideways. So the jaw's collider has
to stop at the rod, while the jaw's mesh keeps closing and passes through
it. Build the collider from slices, each a separate collision shape, and
move each one with `Shears.place`. The slices that reach a rod stop there,
and the others keep moving with the jaw. Which slice stops, and at what
pose, depends on the tool's shape, so that rule belongs in the tool's
code.

## Seeing the colliders

Kit can draw a PhysX scene's colliders, but not a Newton model's.
`ColliderView` draws them: every collision shape of the running model, as
a see-through shape of the same kind and size. The static scene is blue, a
rod green, and any other body, such as a robot's, magenta.

```python
from misina_lab.isaaclab import ColliderView

colliders = ColliderView()  # after the first reset, like Shears
while simulation_app.is_running():
    sim.step()
    colliders.update()
```

Each shape is drawn as a child of its body's prim, so it moves with the
body at no cost per frame. `update()` redraws only the shapes whose pose
relative to their body, or whose size, changed since the last call, such
as the two segments at a cut. A shape whose body has no prim on the stage
has nowhere to be drawn, so it is skipped, and `ColliderView.missing`
counts such shapes.

## A still of the scene

`misina_lab.isaaclab.shots` renders a picture of a generated scene in Isaac
Sim's RTX renderer, from a **shot file**: a JSON file with the picture's
size, the parameters to change, and the camera. Without an output path, it
opens Kit's viewport on the scene instead, so you can frame the shot by
hand.

The camera is given as in the viewer's storyboards, the JSON files the
viewer plays back to record videos and stills of itself: the point looked
at, a yaw, a pitch, a distance and a field of view (`Pose` in
[tour.rs](../src/tour.rs)). Press `P` in Kit's viewport, or in the viewer,
to print the current camera in that form. A view framed in one app then
gives the same view in the other. The module's docstring lists every field
of a shot. A script builds its own scene between the module's calls:

```python
from misina_lab.isaaclab import shots

args = shots.arguments("Renders a still of the boxes.")
shot = shots.load(args.shot)
shots.configure(BOXES_CFG, shot.get("set", {}))
with launch_simulation(sim_cfg, shots.launcher(args.out)):
    sim = sim_utils.SimulationContext(sim_cfg)
    BOXES_CFG.func("/World/Boxes", BOXES_CFG)
    camera = shots.sensor(shot) if args.out else None
    sim.reset()
    shots.finish(sim, shot, None, camera, args.out)
```

`arguments` reads the command line: the shot file, and the PNG to write if
there is one. `load` reads the shot and rejects a field it does not know. A
script that reads fields of its own passes their names as a second argument.
`configure` applies the shot's parameter changes to the cfg. `launcher`
gives the launch options: headless with cameras for a picture, or Kit's
window to frame one in. `sensor` makes the camera, before the first reset as
every Isaac Lab sensor is. `finish` places the camera and saves the picture,
or, with no camera, opens the viewport. Its third argument is the scene's
ground, a `misina_lab.usd.Ground`, which the height of the point looked at
is measured from; `None` puts the ground at height 0.

A demo that moves takes its picture at a chosen moment. The shot names the
moment with two more fields: `at`, what the demo is doing then, and
`warmup`, seconds of simulated time after that. The script names both in
its call to `load`: `shots.load(args.shot, {"at", "warmup"})`. It wraps the
demo's steps in `shots.Moments`, calls `reach(shot)` to step the demo to
that moment, and passes the `Moments` to `finish` as a sixth argument.

## A video of the scene

`misina_lab.isaaclab.film` films the running simulation in the same
renderer. The script takes a frame every so many physics steps, so the
video plays in simulated time, however long the steps take to compute.
`film.sensor` makes the camera, as `shots.sensor` does for a still. `Rig`
moves the camera from one shot to the next, starting and ending each move
slowly. If the next camera position is more than `cut` meters away, it
jumps there instead. `develop` renders each frame four times, so that
moving edges come out clean and do not flicker, and returns the picture.
`Film` sends the frames to ffmpeg, and writes their captions beside the
video as a SubRip (`.srt`) subtitle file. The module's docstring has the
loop a script runs.

## Where it lives

- [`misina_lab/isaaclab/spawn.py`](../../../python/misina-lab/misina_lab/isaaclab/spawn.py): `GeneratedSceneCfg`, `spawn_generated`, `resolve_usd_path`, `to_params`, and the cache key
- [`misina_lab/isaaclab/rods.py`](../../../python/misina-lab/misina_lab/isaaclab/rods.py): `make_physics_cfg_newton`, `tune_rods`, `carry_riders`, `let_rods_out_of_their_wood`, and the numbers
- [`misina_lab/isaaclab/cutting.py`](../../../python/misina-lab/misina_lab/isaaclab/cutting.py): `Shears.colliders` and `Shears.place` for a collider that moves; the cut itself is [cutting.md](cutting.md)
- [`misina_lab/isaaclab/skins.py`](../../../python/misina-lab/misina_lab/isaaclab/skins.py): `drive_skins`, `Skins`, `rig` and `split`
- [`misina_lab/isaaclab/colliders.py`](../../../python/misina-lab/misina_lab/isaaclab/colliders.py): `ColliderView`
- [`misina_lab/isaaclab/shots.py`](../../../python/misina-lab/misina_lab/isaaclab/shots.py): the shot file, `sensor` and `finish`
- [`misina_lab/isaaclab/film.py`](../../../python/misina-lab/misina_lab/isaaclab/film.py): `sensor`, `Rig`, `develop` and `Film`
- [`src/scene/mod.rs`](../src/scene/mod.rs): `cable`, `spawn_rod`, `skin_mesh` and `segment_frames`, what the Rust side authors

# Rods

A **rod** is a flexible **organ**: one stem or leaf that bends in the
simulation. An organ is one instance of what an [element](elements.md)
builds, and an element is the code that builds one kind of thing in the
scene. Only a rod, and what is placed on it, moves in a generated scene.

[Newton](https://newton-physics.github.io/newton/latest/), a GPU physics
engine that [Isaac
Lab](https://isaac-sim.github.io/IsaacLab/main/index.html) can run on,
simulates a rod as a chain of capsule-shaped rigid bodies, one per segment,
joined by
[joints](https://newton-physics.github.io/newton/latest/concepts/deformable_objects.html)
that bend, twist, stretch and shear. A capsule is a cylinder with a half
sphere at each end, and each one is as thick as the organ is drawn there.
The first segment is fixed to the plant's **wood**: its rigid parts, such as
a trunk, which are static colliders.

A rod is set up in two places. In the generator, the element describes
the organ as a curve ([Describing a rod](#describing-a-rod)). In Isaac
Lab, the physics config has to run the solver that moves rods, and the
spawner tunes the rods before the model is built
([Simulating rods](#simulating-rods)). Under physics that cannot simulate
a rod, the organ keeps the shape it was built in.

## Describing a rod

In place of one mesh and a capsule for its collider
([export.md](export.md#colliders)), an element describes a flexible organ
as a curve,
[`scene::cable(points, widths, loads, surface)`](https://docs.rs/misina-lab/latest/misina_lab/scene/fn.cable.html).
Newton's importer turns the curve into the rod, with one body per segment
of the curve. The arguments of `cable`:

- `points`: the curve, in the organ's own frame, starting at the held end.
  Give at least three. Space them evenly after the first segment: each
  point's mass is split between the two segments it joins, so uneven
  spacing gives segments the wrong mass.
- `widths`: the diameter at each point in meters, as drawn and simulated.
- `loads`: extra mass in kilograms hanging from each point, such as fruit.
  It weighs on the rod, but the fruit is not a simulated body. Leave it
  empty for none.
- `surface`: the colour and finish of the mesh the curve replaces, so the
  rod looks like that mesh.

The curve itself is not drawn.
[`scene::spawn_rod(organ, cable, frames, tubes, skin)`](https://docs.rs/misina-lab/latest/misina_lab/scene/fn.spawn_rod.html)
spawns the curve as a **prim** named `Cable` under the organ; a
[prim](https://openusd.org/release/glossary.html#usdglossary-prim) is one
node of the exported USD scene ([export.md](export.md)). Next to `Cable`,
`spawn_rod` spawns one prim per segment. Each is placed at its segment's
rest frame, the position and rotation `scene::segment_frames(&points)`
gives that segment. Newton turns each segment prim into that segment's
body, so what is drawn under the prim moves with the body.

The organ is drawn by `tubes`, one mesh per segment from
`scene::segment_tubes`, or by one skin ([Skins](#skins)). `spawn_rod`
returns the segment prims' entities, one per frame and in the same order,
so an index into `frames` is an index into the list returned. To put a
leaf on the segment at index 3, spawn it as a child of the entity at index
3, with the transform `scene::rebase(&frames[3], &placement)`. `placement`
is where the leaf sits in the organ's frame, and `rebase` gives that place
in the segment's frame. A collider under a segment's prim moves with the
segment once the Python function `rods.carry_riders` has attached it to
the segment's body. `tune_rods`, the framework's Isaac Lab helper that
sets up the rods, calls `carry_riders` ([The tuning](#the-tuning)).

> [!NOTE]
> **In Newton terms.** A segment's prim is named `scene::segment_name(i)`,
> the path Newton's importer gives that body; the importer keeps a prim
> already there, with its children. `cable` writes the curve's `normals`,
> each segment's +Y from `segment_frames`. The importer builds each body
> turned to match, so the body starts where its prim is. Without normals,
> a body could start turned from its prim by up to half a turn.

## Simulating rods

In Isaac Lab, a generated scene is spawned from its **cfg**, an instance of
the generator's Isaac Lab config class. The **spawner** is the function
that builds the scene on first use and adds it to the simulation's USD
stage ([isaac-lab.md](isaac-lab.md)). The functions on this page are in
`misina_lab.isaaclab`, the framework's Isaac Lab package.

A **solver** is the part of Newton that computes how bodies move in each
step. Only
[VBD](https://newton-physics.github.io/newton/latest/solvers/vbd.html),
Newton's vertex block descent solver, can simulate a rod, and a robot needs
[MJWarp](https://newton-physics.github.io/newton/latest/solvers/mujoco.html),
Newton's GPU version of MuJoCo. So a scene with rods and a robot runs both
solvers as one
[coupled solver](https://newton-physics.github.io/newton/latest/concepts/coupling.html),
made of named parts. Its part `rigid` advances the robot with MJWarp, and
its part `rods` advances the rods with VBD. The robot bodies that a rod may
touch are copied into the `rods` part as **proxies**: bodies that move as
the robot's bodies move, and push the rods. `make_physics_cfg_newton`
builds this physics config. When the cfg's `without_rods()` returns `None`,
the scene has no rods, and it builds plain MJWarp instead:

```python
physics = make_physics_cfg_newton(
    PLANTS_CFG,
    robot="/World/envs/env_.*/Robot",
    contact_bodies=[".*/Robot/gripper_.*"],
    substeps=4,
)
sim_cfg = sim_utils.SimulationCfg(dt=0.005, physics=physics)
```

`PLANTS_CFG` is the scene's cfg. `robot` is a pattern for the robot's prim
path, and `contact_bodies` are patterns for the bodies a rod may touch.
Only those bodies become proxies, so nothing else can bend a rod: a leg not
on the list passes through one. `substeps` is the number of physics
substeps in each simulation step. The rod tuning below assumes a substep of
1.25 ms: four substeps in a simulation step of 5 ms, which is `dt=0.005` on
the `SimulationCfg`. With one substep, a rod sags about twice as far.

`broad_phase` is `"explicit"` by default, or `"nxn"` or `"sap"`. It picks
the
[**broad phase**](https://newton-physics.github.io/newton/latest/concepts/collisions.html#broad-phase-and-shape-compatibility),
the method Newton uses to find which shapes may touch
([cutting.md](cutting.md)). It also decides what a cut piece of rod
collides with.

With the `"explicit"` broad phase, a scene with rods must contain at
least two static collision shapes that are not a ground plane or a height
field, such as two posts. A cut adds an entry to Newton's list of possible
contacts, and the spare entries reserved for cuts are kept on two such
shapes until a cut uses them
([cutting.md](cutting.md#letting-the-two-faces-meet)). Without the two
shapes, the first reset, which builds the model, raises `ValueError`. The
`"nxn"` and `"sap"` broad phases do not need them.

The physics config you build is not always the one that runs. Isaac Lab's
`--physics` option, a Hydra preset (Isaac Lab's config overrides) or your
own code can replace it before the scene is built. So when the spawner
spawns the scene, it checks it against the physics config that is in use:

- If nothing in that config can simulate a rod, as under MJWarp alone or
  PhysX, the spawner logs a warning and spawns `without_rods()` instead.
  That scene has the same shapes, with the rods as static meshes, and it
  is cached as a separate file.
- If VBD runs, the spawner raises `TypeError` when the cfg sets no
  `PLANT`, the name every plant's prim starts with
  ([isaac-lab.md](isaac-lab.md#the-contract)). Otherwise it calls
  `tune_rods` and [`drive_skins`](#skins).

## The tuning

`tune_rods` changes the model before Newton builds it:

- **It softens each rod joint's stretch and shear.** It multiplies their
  stiffness by `ROD_STRETCH`, 0.01. Without this, a rod bends over under
  its own weight and keeps moving instead of settling. Bend and twist keep
  their stiffness, and are damped by `ROD_DAMPING`, 0.1 s, times that
  stiffness. The numbers were measured on rods about a metre long,
  tapering from about a centimetre across to half that; `rods.py` says
  what each one does. To change them, set `ROD_STRETCH`, `ROD_DAMPING` or
  `LONE_ROD_DAMPING`, the damping of a rod under no plant's prim, on the
  cfg class ([isaac-lab.md](isaac-lab.md#the-contract)).
- **It puts the shapes into collision groups.** A
  [collision group](https://newton-physics.github.io/newton/latest/concepts/collisions.html#collision-groups)
  is a number on each shape that decides which other shapes it may collide
  with ([cutting.md](cutting.md)). Each plant's rods and wood share one
  group, so they collide with each other. The static scene and each other
  body, such as a robot's, are in groups that collide with every group. So
  a rod collides with its own plant, the static scene and the robot, and
  passes through every other plant. A rod under no plant's prim counts as a
  plant of its own. Without groups, Newton would list every pair of rod
  segments as a possible contact: about 50 million pairs for ten thousand
  segments.
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

## Skins

Tubes, one rigid mesh per segment, show a sharp bend where two segments
meet. A **skin** is one mesh along the whole organ that bends smoothly at
every joint. Build it with
[`scene::skin_mesh(mesh, points)`](https://docs.rs/misina-lab/latest/misina_lab/scene/fn.skin_mesh.html),
register the result as a **part**, a mesh in the scene's mesh library, with
`Library::part` ([export.md](export.md#references-to-shared-meshes)), and
pass the `Geometry` it returns to `spawn_rod` as `skin`, with `tubes` empty.

`skin_mesh` records for each vertex how far along the rod it lies, counted
in segments (`scene::ATTRIBUTE_ALONG`). The USD builder
([export.md](export.md#the-builder)) binds each vertex to three joints: its
own segment's and the segments' before and after it. The blend between the
three is what bends the skin smoothly.

The skin's prim, `Skin`, sits beside the segment prims. It is a
[`SkelRoot`](https://openusd.org/release/api/usd_skel_page_front.html),
USD's prim type for a mesh that a skeleton bends. The skeleton has one
joint per segment, named after the segment's prim and placed at its rest
frame. A skin is never instanceable, that is, never a shared copy of one
mesh ([export.md](export.md#references-to-shared-meshes)), since each skin
bends on its own. Leaves and the other things on the organ still go under
the segment prims, not under `Skin`.

Newton moves the segment prims, but nothing moves a skin's skeleton. So a
skin keeps the shape it was built in until something poses it, that is,
moves its skeleton's joints to the segments. In Isaac Lab, `drive_skins`
does that, and the spawner calls it together with `tune_rods`.

From then on, each time the viewport of Kit, Isaac Sim's application
framework, or an RTX camera is about to render a frame, every skin whose
segments moved since it was last posed is posed again. A step that renders
nothing poses nothing. Without Kit, `drive_skins` does nothing. A uniform
`scale` on the scene is allowed. A skin under a prim that scales unevenly
or skews raises `ValueError` when it is posed.

When a rod is cut ([cutting.md](cutting.md)), `skins.split` splits its skin
at the cut. The part of the mesh before the cut follows the joints before
it, and the rest follows the joints after it. Each part ends at the cut,
closed by a cap. Both parts stay where they were drawn, even on a bent rod.

## Moving a collider

`Shears`, the class that cuts rods ([cutting.md](cutting.md)), also moves
colliders. `Shears.colliders(body)` lists a body's collision shapes: their
indices in the model, and each one's pose relative to the body. A pose is
one array of 7 numbers, a position (x, y, z) and then a rotation quaternion
(x, y, z, w). `Shears.place(shape, pose)` moves one shape to a new pose
relative to its body while the simulation runs. It writes the pose into
every copy of the model that the solvers use, as a cut does.

A tool that closes on a rod, such as a blade or a gripper jaw, needs this.
A rigid jaw that keeps closing presses the rod against whatever is behind
it, and the solver pushes the rod out sideways. So the jaw's collider has
to stop at the rod, while the jaw's mesh keeps closing and passes through
it. Build the collider from slices, each a separate collision shape, and
move each one with `Shears.place`. The slices that reach a rod stop there,
and the others keep moving with the jaw. A slice's pose is relative to the
jaw's body, so a slice that has stopped is placed again on each step in
which the jaw moves. Which slice stops, and at what pose, depends on the
tool's shape, so that rule belongs in the tool's code.

## Where it lives

- [`src/scene/mod.rs`](../src/scene/mod.rs): `cable`, `spawn_rod`, `segment_frames`, `segment_tubes`, `segment_name`, `rebase` and `skin_mesh`, what the Rust side authors
- [`misina_lab/usd/build.py`](../../../python/misina-lab/misina_lab/usd/build.py): what a cable and a skin become in USD
- [`misina_lab/isaaclab/rods.py`](../../../python/misina-lab/misina_lab/isaaclab/rods.py): `make_physics_cfg_newton`, `tune_rods`, `carry_riders`, `let_rods_out_of_their_wood`, and the numbers
- [`misina_lab/isaaclab/skins.py`](../../../python/misina-lab/misina_lab/isaaclab/skins.py): `drive_skins`, `Skins`, `rig` and `split`
- [`misina_lab/isaaclab/cutting.py`](../../../python/misina-lab/misina_lab/isaaclab/cutting.py): `Shears.colliders` and `Shears.place` for a collider that moves; the cut itself is [cutting.md](cutting.md)

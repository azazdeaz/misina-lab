# Export

> [!WARNING]
> **Under development.** This page may not reflect the current state of
> the source. We're working on it.

**Export** is how a built scene leaves Rust. The scene is built in Bevy,
the game engine the framework runs on. Bevy holds each object of the scene
as an **entity**: a thing with a name, a position and often a mesh. The
export reads those entities and writes the **scene document**: plain JSON
that says what geometry exists, where it goes and which mesh each object
uses. Python then turns the document into a USD **stage**, the scene file
saved as `.usd`. A stage is a tree of **prims**, and a prim is one node of
that tree, such as a group, a mesh or a collider.

Rust decides what is in the scene. Python decides how it is written in
USD: the prim types, the physics properties, the references. The document,
`scene::doc::SceneDoc`, is the only thing the two sides share. The
**viewer** is a window that draws the scene, with a slider for every
parameter ([generators.md](generators.md#what-the-framework-builds-from-it)).
The export reads the same entities the viewer draws, so the file holds
what the viewer shows.

```text
Bevy entities (Transform, Mesh3d, Name, UsdReference)
      │
      ▼  src/scene/export.rs
SceneDoc, as JSON
      │
      ▼  python/misina-lab/misina_lab/usd/build.py
a .usd stage
```

## Coordinates

The scene is authored **Z-up, in meters**: REP-103, the convention for
units and axes that ROS and Isaac Lab use. Build every element in it. An
**element** is the code that builds one kind of thing in the scene, such
as the leaves ([elements.md](elements.md)).

The Python builder ([The builder](#the-builder)) writes `upAxis = "Z"` on
the stage. USD reads `upAxis` only from the file that is opened, not from
a file that file references. So a stage that references this one cannot
change the axis. Where `upAxis` is missing, USD assumes Y-up, so the
builder always writes it.

Bevy's axes are Y-up. So the viewer rotates the whole scene with one extra
parent entity above the scene root, turned by `scene::z_up_to_y_up()`. The
export starts at the scene root, below that parent, so the rotation never
reaches the file. No element has to know about Bevy's axes.

## Which entities become prims

The export starts at the **scene root**, the entity the framework spawns
for the generator. A **generator** is the code that produces one kind of
scene ([generators.md](generators.md)). The root is named after the
generator's `NAME` and marked with `scene::UsdRoot`. The `scene::PrimRoot`
resource holds it, and every element spawns its entities under it. The
root becomes the stage's default prim, the prim USD uses when another file
references the stage.

The export then visits every entity below the root, and each entity that
has a `Name` becomes a prim. The `Name` is the prim's name, and the
`Transform` is the prim's transform, left out when it is the identity. By
default the prim's USD type is `Xform`, a prim with a transform. An entity
can set another type with `scene::UsdType`, such as `Scope`, a group that
cannot have a transform.

Below is one plant of a scene whose generator is named `Plants`. An arrow
means the prim references a shared mesh in `/Plants/parts`, and
instanceable is explained [below](#references-to-shared-meshes):

```text
/Plants/Row_00/Plant_047    Xform
  /Wood                     -> parts/Plant_3, instanceable
  /Collision                Capsule, the plant's collider
  /Stem_00                  Xform
    /Tube                   -> parts/Stem_1, instanceable
    /Leaf_00                -> parts/Leaf_2, instanceable
```

An entity without a `Name` is left out, with everything under it. So the
camera, the lights, the viewer's panel of sliders and the parent entity
that rotates the axes stay out of the file. You do not have to flag them.

The export writes siblings in name order, not in the order they were
spawned. A scene is built in **layers**, each on what the one before
placed, such as the ground and then the plants on it
([generators.md](generators.md#stage-sets)). When a layer rebuilds and
spawns its entities again, they keep their place in the document, and the
same parameters give the same document. Also keep names the same from one
build to the next. An Isaac Lab config finds a prim by its path, so after
a name changes, the config no longer finds that prim.

### References to shared meshes

An element registers each mesh once in the **mesh library**, with
`scene::Library::part`, under a name such as `Leaf_2`. A registered mesh
is a **part**. An **organ** is one instance of an element, one leaf or one
stem. Many similar organs share one part: the mesh of a **representative**,
one organ chosen to stand for the ones like it
([mesh-variations.md](mesh-variations.md)).

`part` returns a `scene::Geometry`. Each entity that carries this
`Geometry` is drawn with the part's mesh in the viewer. In the document it
becomes a prim that references the part, not a copy of its points. The
library becomes `/<root>/parts` on the stage.

The export marks a prim that references a part as **instanceable**, which
lets USD keep one shared copy of the mesh for all the prims that reference
it. An instanceable prim can have no children, because USD ignores
children written under one. So the export fails on an entity that has both
a reference and children. A prim that only groups others, such as
`Stem_00` above, references no part and may have children. Spawn what
belongs to an object beside its mesh, as `Leaf_00` sits beside `Tube`.

The export also fails on a mesh that is not an indexed triangle list.
Rotations are stored in the document as quaternions in xyzw order. The
document carries a `format` number, and the builder refuses one that is
not its own `FORMAT`. So a document saved by an older version of the
framework, such as a cached scene, fails with an error.

## The builder

`misina_lab.usd.build_usd(doc, path)`, in the framework's Python package,
writes the document as a stage at `path`. The path must not exist yet.
`.usd` or `.usdc` gives a binary file, `.usda` a text file. A generator's
Python module has `write_usd(path)`, which builds the scene and passes the
document to `build_usd` ([generators.md](generators.md#the-python-module)).
In the viewer, the `S` key saves the document as `scene.json` in the
working directory. `python -m misina_lab.usd scene.json scene.usd` builds
a stage from it, and `--force` overwrites an existing file.

The module docstring of `build.py` lists every USD rule the builder
follows, and the reason for each. A rule left out of the builder raises no
error: the stage still loads, but draws or collides wrongly.

## Colliders

The document says what collides, and the builder adds the USD physics
properties that make a prim a collider. A flexible organ collides as a
chain of moving bodies ([Flexible organs](#flexible-organs)). Nothing else
in the scene is a rigid body, so every other collider is static: it never
moves. In the viewer, tick **Colliders** in the footer to draw them all.
There are two kinds of static collider.

**A part that is its own collider.** `Library::collide(&part,
TRIANGLE_MESH)` makes a part collide as its exact triangles. USD allows
that only on a static collider; in the file it is
`physics:approximation = "none"`. Use it only for the ground, where the
mesh is the shape. A prim that references such a part is not
instanceable, so the stage holds a full copy of the mesh for it. A scene
has one ground, so that is one copy.

`Library::heightfield(&part, resolution)` also marks the part as a height
field: a grid of ground heights, one every `resolution` meters. Newton, a
physics engine Isaac Lab can run on, then turns the mesh into a height
field at that spacing. Without the mark, the mesh goes to Newton's MuJoCo
solver as a plain mesh, and MuJoCo collides a mesh as its convex hull,
which fills in every dip of the ground. The framework's terrain element
calls both `collide` and `heightfield` for its ground.

**A capsule.** Anything else that collides gets a capsule, a cylinder with
a half sphere at each end, in place of its detailed mesh.
`scene::capsule(radius, z0, z1)` lays one along the organ's own Z axis,
from `z0` to `z1` in meters. `scene::capsule_between(radius, from, to)`
lays one along any line. Name the capsule `scene::COLLISION`, so a program
that reads the stage finds every collider by one name:

```rust
organ.with_child((Name::new(COLLISION), capsule(0.02, 0.0, 0.8)));
```

The span from `z0` to `z1` covers the whole capsule, half spheres
included. In USD a capsule's `height` is its straight middle part alone,
so a span shorter than two radii gives a sphere, which reaches past both
ends of the span. A capsule has the USD `guide` purpose, which renderers
skip, so no render draws it. The viewer's **Colliders** overlay does.

### Flexible organs

A flexible organ, such as a long thin stem, bends in the simulation. In
place of a mesh and a capsule, the element describes it as a curve,
`scene::cable(points, widths, loads, surface)`. Newton turns the curve
into a **rod**: a chain of capsule-shaped bodies, one per segment of the
curve, connected by joints. The first segment is held fixed. Only
Newton's VBD solver, the one that simulates rods, moves a rod. Under any
other solver the rod keeps the shape it was built in
([isaac-lab.md](isaac-lab.md#rods)).

The arguments of `cable`:

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

The curve itself is not drawn. `scene::spawn_rod(organ, cable, frames,
tubes, skin)` spawns it as `Cable` under the organ. Beside it, it spawns
one prim per segment. Each is placed at its segment's rest frame, the
position and rotation `scene::segment_frames(&points)` gives that segment.
Newton turns each segment prim into that segment's body, so everything
under the prim moves with the body.

The organ is drawn by `tubes`, one mesh per segment from
`scene::segment_tubes`, or by one skin ([Skins](#skins)). `spawn_rod`
returns the segment prims' entities, one per frame and in the same order,
so an index into `frames` is an index into the list returned. To put a
leaf on the segment at index 3, spawn it as a child of the entity at index
3, with the transform `scene::rebase(&frames[3], &placement)`. `placement`
is where the leaf sits in the organ's frame, and `rebase` gives that place
in the segment's frame. A collider under a segment's prim moves with the
segment once the Python function `rods.carry_riders` has attached it to
the segment's body. `tune_rods`, the Isaac Lab helper that sets up the
rods, calls `carry_riders` ([isaac-lab.md](isaac-lab.md#rods)).

> [!NOTE]
> **In Newton terms.** A segment's prim is named `scene::segment_name(i)`,
> the path Newton's importer gives that body; the importer keeps a prim
> already there, with its children. `cable` writes the curve's `normals`,
> each segment's +Y from `segment_frames`. The importer builds each body
> turned to match, so the body starts where its prim is. Without normals,
> a body could start turned from its prim by up to half a turn.

### Skins

Tubes, one rigid mesh per segment, show a sharp bend where two segments
meet. A **skin** is one mesh along the whole organ that bends smoothly at
every joint. Build it with `scene::skin_mesh(mesh, points)`, register the
result as a part with `Library::part`, and pass the `Geometry` it returns
to `spawn_rod` as `skin`, with no tubes.

`skin_mesh` records for each vertex how far along the rod it lies,
counted in segments (`scene::ATTRIBUTE_ALONG`). The builder binds each
vertex to three joints: its own segment's and the segments' before and
after it. The blend between the three is what bends the skin smoothly.

The skin's prim, `Skin`, sits beside the segment prims. It is a
`SkelRoot`, USD's prim type for a mesh that a skeleton bends. The skeleton
has one joint per segment, named after the segment's prim and placed at
its rest frame. A skin is never instanceable, since each skin bends on its
own. Leaves and the other things on the organ still go under the segment
prims, not under `Skin`. A skin bends only in a program that poses its
skeleton to follow the rod ([isaac-lab.md](isaac-lab.md#skins)). Anywhere
else it keeps its built shape.

## Custom attributes

An element can label a prim with **ground truth**: facts that a program
reading the stage can use, and that change nothing drawn or simulated,
such as whether a robot may grasp the object. Put `scene::UsdAttributes`
on the entity:

```rust
commands.spawn((
    Name::new("Box_00"),
    geometry.clone(),
    UsdAttributes::default().with("graspable", true).with("clearance", 0.12),
));
```

The builder writes each label as a custom attribute, one that no USD
schema declares. Its name starts with the generator's `PACKAGE`, the name
of its Python package: `boxlab:graspable` above. The prefix keeps a label
from clashing with a schema's attribute or another generator's. A value
is a `bool`, `i32`, `f32`, `[f32; 3]` or string. It is written as a plain
JSON value, and its USD type follows from that: `bool`, `int`, `float`,
`float3` or `string`.

A label is written on the prim itself, never inside the part it
references. So prims that share one mesh still each have their own value,
read with USD's ordinary `prim.GetAttribute(name)`.

## Reading the ground back

A program that places something on the terrain after the build, such as a
robot, needs the ground height the generator placed the scene on.
`misina_lab.usd.Ground(stage).height(x, y)` reads it from the stage.
Between grid points the height is interpolated from the four grid points
around it. A point outside the grid gets the height at the nearest edge.
Coordinates are world space. `Ground` finds the ground by its
`newton:heightfield:resolution` attribute, not by name, and raises
`ValueError` unless exactly one prim has it.

## Where it lives

- [`src/scene/doc.rs`](../src/scene/doc.rs): `SceneDoc`, `FORMAT`, `TRIANGLE_MESH`, and the conventions the builder relies on
- [`src/scene/export.rs`](../src/scene/export.rs): the export, `scene_doc`
- [`src/scene/mod.rs`](../src/scene/mod.rs): `UsdRoot`, `PrimRoot`, `UsdType`, `UsdReference`, `UsdAttributes`, `Library`, `capsule`, `cable`, `spawn_rod`, `segment_frames`, `skin_mesh`, `rebase`, `z_up_to_y_up`
- [`src/colliders.rs`](../src/colliders.rs): the viewer's collider overlay
- [`misina_lab/usd/build.py`](../../../python/misina-lab/misina_lab/usd/build.py): `build_usd`, and every USD rule with its reason
- [`misina_lab/usd/ground.py`](../../../python/misina-lab/misina_lab/usd/ground.py): `Ground`

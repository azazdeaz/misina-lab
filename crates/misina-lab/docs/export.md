# Export

**Export** is how a built scene leaves Rust. The scene is built in
[Bevy](https://bevy.org), the game engine the framework runs on. Bevy holds
each object of the scene as an
[**entity**](https://bevy.org/learn/quick-start/getting-started/ecs/): a
thing with a name, a position and often a mesh. The export reads those
entities and writes the **scene document**: plain JSON that says what
geometry exists, where it goes and which mesh each object uses. Python then
turns the document into a USD
[**stage**](https://openusd.org/release/glossary.html#usdglossary-stage),
the scene file saved as `.usd`. A stage is a tree of
[**prims**](https://openusd.org/release/glossary.html#usdglossary-prim),
and a prim is one node of that tree, such as a group, a mesh or a collider.

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

The scene is authored **Z-up, in meters**:
[REP-103](https://www.ros.org/reps/rep-0103.html), the convention for units
and axes that ROS and Isaac Lab use. Build every element in it. An
**element** is the code that builds one kind of thing in the scene, such as
the leaves ([elements.md](elements.md)).

The Python builder ([The builder](#the-builder)) writes
[`upAxis = "Z"`](https://openusd.org/release/api/group___usd_geom_up_axis__group.html)
on the stage. Where `upAxis` is missing, USD assumes Y-up, so the builder
always writes it. USD reads `upAxis` only from the file that is opened, not
from a file that file references, and it does not turn a referenced file to
match. So a Y-up stage that references this scene gets it lying on its
side, and has to add the rotation itself.

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
[`scene::Library::part`](https://docs.rs/misina-lab/latest/misina_lab/scene/struct.Library.html#method.part),
under a name such as `Leaf_2`. A registered mesh is a **part**. An
**organ** is one instance of an element, one leaf or one stem. Many similar
organs share one part: the mesh of a **representative**, one organ chosen
to stand for the ones like it ([mesh-variations.md](mesh-variations.md)).

`part` returns a `scene::Geometry`. Each entity that carries this
`Geometry` is drawn with the part's mesh in the viewer. In the document it
becomes a prim that references the part, not a copy of its points. The
library becomes `/<root>/parts` on the stage.

The export marks a prim that references a part as
[**instanceable**](https://openusd.org/release/glossary.html#usdglossary-instanceable),
which lets USD keep one shared copy of the mesh for all the prims that
reference it. The exceptions are a part that is its own collider
([Colliders](#colliders)) and a skin, one mesh that bends with a flexible
organ ([rods.md](rods.md#skins)). An instanceable prim can have no
children, because USD ignores children written under one. So the export
fails on an entity that has both a reference and children. A prim that only
groups others, such as `Stem_00` above, references no part and may have
children. Spawn what belongs to an object beside its mesh, as `Leaf_00`
sits beside `Tube`.

The export also fails on a mesh that is not an indexed triangle list.
Rotations are stored in the document as quaternions in xyzw order. The
document carries a `format` number, and the builder refuses one that is
not its own `FORMAT`. So a document saved by an older version of the
framework, such as a cached scene, fails with an error.

## The builder

`misina_lab.usd.build_usd(doc, path)`, in the framework's Python package,
writes the document as a stage at `path`. The path must not exist yet.
`.usd` or `.usdc` gives a binary file, `.usda` a text file. The params
aggregate in a generator's Python module has a method `write_usd(path)`,
which builds the scene and passes the document to `build_usd`
([generators.md](generators.md#the-python-module)).
In the viewer, the `S` key saves the document as `scene.json` in the
working directory. `python -m misina_lab.usd scene.json scene.usd` builds
a stage from it, and `--force` overwrites an existing file.

The module docstring of `build.py` lists every USD rule the builder
follows, and the reason for each. A rule left out of the builder raises no
error: the stage still loads, but draws or collides wrongly.

## Colliders

The document says what collides, and the builder adds the USD physics
properties that make a prim a collider. A flexible organ collides as a
chain of moving bodies, and a collider placed under one of its segments
moves with that segment ([rods.md](rods.md)). Nothing else in the scene is
a rigid body, so every other collider is static: it never moves. In the
viewer, tick **Colliders** in the footer to draw them all. There are two
kinds of static collider.

**A part that is its own collider.** `Library::collide(&part,
TRIANGLE_MESH)` makes a part collide as its exact triangles. USD allows
that only on a static collider; in the file it is
`physics:approximation = "none"`. Use it only for the ground, where the
mesh is the shape. A prim that references such a part is not
instanceable, so the stage holds a full copy of the mesh for it. A scene
has one ground, so that is one copy.

`Library::heightfield(&part, resolution)` also marks the part as a height
field: a grid of ground heights, one every `resolution` meters.
[Newton](https://newton-physics.github.io/newton/latest/), a physics engine
Isaac Lab can run on, then turns the mesh into a height field at that
spacing. Without the mark, the mesh goes to Newton's MuJoCo solver as a
plain mesh, and MuJoCo collides a mesh as its convex hull, which fills in
every dip of the ground. The framework's terrain element calls both
`collide` and `heightfield` for its ground.

**A capsule.** Anything else that collides gets a capsule, a cylinder with
a half sphere at each end, in place of its detailed mesh.
[`scene::capsule(radius, z0, z1)`](https://docs.rs/misina-lab/latest/misina_lab/scene/fn.capsule.html)
lays one along the organ's own Z axis, from `z0` to `z1` in meters.
`scene::capsule_between(radius, from, to)` lays one along any line. Name
the capsule `scene::COLLISION`, so a program that reads the stage finds
every collider by one name:

```rust
organ.with_child((Name::new(COLLISION), capsule(0.02, 0.0, 0.8)));
```

The span from `z0` to `z1` covers the whole capsule, half spheres included.
In USD a capsule's `height` is its straight middle part alone, so a span
shorter than two radii gives a sphere, which reaches past both ends of the
span. A capsule has the USD `guide`
[purpose](https://openusd.org/release/glossary.html#usdglossary-purpose),
which renderers skip, so no render draws it. The viewer's **Colliders**
overlay does.

## Custom attributes

An element can label a prim with **ground truth**: facts that a program
reading the stage can use, and that change nothing drawn or simulated, such
as whether a robot may grasp the object. Put
[`scene::UsdAttributes`](https://docs.rs/misina-lab/latest/misina_lab/scene/struct.UsdAttributes.html)
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
- [`src/scene/mod.rs`](../src/scene/mod.rs): `UsdRoot`, `PrimRoot`, `UsdType`, `UsdReference`, `UsdAttributes`, `Library`, `capsule`, `capsule_between`, `z_up_to_y_up`
- [`src/colliders.rs`](../src/colliders.rs): the viewer's collider overlay
- [`misina_lab/usd/build.py`](../../../python/misina-lab/misina_lab/usd/build.py): `build_usd`, and every USD rule with its reason
- [`misina_lab/usd/ground.py`](../../../python/misina-lab/misina_lab/usd/ground.py): `Ground`

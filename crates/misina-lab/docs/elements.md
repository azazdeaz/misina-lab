# Elements

A scene is the 3D world that a [generator](generators.md) builds and
exports to a simulator such as Isaac Lab. An **element** is the code that
builds one kind of thing in it: the posts, the plants, the leaves, a patch
of ground cover. A generator builds its scene from a list of elements, and
each element is one Rust module of the generator.

An **organ** is one instance of what an element builds: one leaf, one
stem. An element that sorts its organs into groups that share a mesh
holds five things ([mesh-variations.md](mesh-variations.md) explains
configs and metrics):

- a **fragment**, its own struct of parameters
  ([generators.md](generators.md#the-params-aggregate));
- a **config**, the few numbers that decide one organ's mesh, such as its
  length;
- a **metric**, the distance between two configs, which decides which
  organs are alike enough to share one mesh;
- a build system, the function that makes the meshes and creates the
  organs in the scene;
- a plugin, the function that adds the fragment and the systems to the
  app.

An element with one mesh for all its organs, such as a wire of one gauge,
or with a mesh for every organ, has no metric and skips the grouping.

> [!NOTE]
> **In Bevy terms.** Each organ is an
> [entity](https://bevy.org/learn/quick-start/getting-started/ecs/), and
> its config is a `Component` on it. The fragment is a
> [`Resource`](https://bevy.org/learn/quick-start/getting-started/resources/).
> A system is a function Bevy runs every frame, given the data it asks for,
> and `run_if` adds a run condition, a check that decides whether it runs
> this frame. A
> [plugin](https://bevy.org/learn/quick-start/getting-started/plugins/) is
> a function that adds resources and systems to the `App`.

To add an element:

1. Write its module, as in the example below.
2. Add its plugin to the generator's own `plugin` function.
3. Add its fragment as a field of the `generator!` call, the macro that
   declares the generator's parameters.
4. In the element's plugin, put its systems in one of the generator's
   stages ([The pipeline](#the-pipeline)). Add a new stage if no stage runs
   at the right point.

```rust
/// The prefix this element's meshes are named under in the mesh library:
/// `Leaf_0`, `Leaf_1`, ...
pub const PART: &str = "Leaf";

/// One leaf's shape, as the stem it grows on wrote it.
#[derive(Component, Clone, Copy, Debug, PartialEq)]
pub struct LeafConfig { pub length: f32, pub droop: f32, /* ... */ }

/// Two leaves share a mesh when their configs are close in every field that
/// changes the shape.
pub struct LeafMetric;
impl Metric<LeafConfig> for LeafMetric { /* weighted distance over the fields that build */ }

/// The struct's doc comment is the Python class docstring.
#[derive(Resource, Reflect, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "python", pyo3::pyclass(get_all, set_all, skip_from_py_object))]
pub struct LeafParams {
    /// Blade length, in meters. A field's first paragraph is its tooltip and docstring.
    #[reflect(@Slider { min: 0.02, max: 0.2, step: 0.005 })]
    pub length: f32,
    /* ... */
}

pub fn plugin(app: &mut App) {
    // `reauthor` writes edited params onto the leaves already placed. `build`
    // then rebuilds the meshes if a config or the params changed. Both run in
    // this element's stage, `Grow::Leaves`.
    app.init_resource::<LeafParams>().add_systems(PreUpdate, (
        reauthor.run_if(resource_changed::<LeafParams>),
        build.run_if(configs_changed::<LeafConfig>.or_eager(resource_changed::<LeafParams>)),
    ).chain().in_set(Grow::Leaves));
}

/// Writes the params onto every leaf already placed, in place.
fn reauthor(params: Res<LeafParams>, mut leaves: Query<&mut LeafConfig>) { /* ... */ }

fn build(commands: Commands, library: Library, /* ... */) -> Result<()> { /* ... */ }
```

[`Metric`](https://docs.rs/misina-lab/latest/misina_lab/quantize/trait.Metric.html),
[`configs_changed`](https://docs.rs/misina-lab/latest/misina_lab/scene/fn.configs_changed.html)
and
[`Library`](https://docs.rs/misina-lab/latest/misina_lab/scene/struct.Library.html)
come from the framework. `Grow` is the generator's own list of stages.

Several things are generated from the fragment alone
([editing-parameters.md](editing-parameters.md)). The viewer, the window
with a control for every parameter, gets a section in its panel: a slider
for each number, a dropdown for each choice, a checkbox for each flag. The
snippet, the Python code that constructs the Isaac Lab config
with the current settings, gets its fields. So do the Python class, its
type stub and the parameters page. `cargo test` fails if a field lacks
something one of these needs, such as the doc comment that becomes its
tooltip.

## The pipeline

A scene is built in **layers**. Each layer is placed on what the layer
before it created: first the ground, then the plants on the ground, then
the stems on the plants, then the leaves on the stems. Each element is one
layer.

A generator orders the layers with **stages**. It lists its stages in
build order, and each element runs in one of them
([generators.md](generators.md#stage-sets)). Elements that do not read each
other's output can share a stage, such as the leaves and the ground cover.

Every layer that groups its organs runs the same five steps:

1. **Collect** every config of its own kind, and sort them by
   [`scene::Order`](https://docs.rs/misina-lab/latest/misina_lab/scene/struct.Order.html),
   a number each organ is given when it is placed. The sort makes the
   result the same on every run ([Rules](#rules)).
2. **Cluster**: group similar configs, and pick a limited number of them,
   the **representatives**, to get a mesh. The limit is
   `params.variations`, which an element raises where it has to keep a mesh
   for each kind of organ
   ([mesh-variations.md](mesh-variations.md#the-budget)).
3. **Build** each representative's mesh once, into `Prototypes`, the
   **mesh library**. It holds every mesh any layer built, by name, such as
   `Leaf_2`, and every layer reads from it.
4. **Assign** each organ the mesh of the representative nearest to its
   config.
5. **Expand**: the representative's mesh has **growth points**, the
   places where the next layer's organs attach, such as its buds. For each
   organ, create a config for the next layer at every growth point. Organs
   that share a mesh still get configs of their own.

The first layer has no layer before it. So its configs come from the
generator's layout code, or from an element that creates its own
([Rules](#rules)). The last layer skips step 5.

**The places come from the representative; the configs written there come
from the organ.** In step 5, the growth points, such as how many branches
there are and where the buds sit, are read from the mesh that was built.
They must match the mesh the organ shows. Points taken from the organ's
own config would not, and a stem could float in the air beside the wood
it should grow from. The numbers in each new config are drawn for each
organ on its own. So two plants that share one mesh still carry different
leaves, and a hundred plants built from four meshes do not look like four
copies.

## The prims an element creates

The exported scene is a tree of USD
[**prims**](https://openusd.org/release/glossary.html#usdglossary-prim),
the nodes of a USD stage. An element creates two kinds:

- A **geometry prim** shows one mesh from the mesh library. It points to
  the mesh and holds no copy of its points. It has no children, and is
  exported
  [**instanceable**](https://openusd.org/release/glossary.html#usdglossary-instanceable):
  USD loads the mesh only once, however many prims point to it.
- A **structural prim**, such as a plant or a stem, only positions the
  prims that sit on it. It is an `Xform`, a prim that holds only a position
  and children, and each one is unique, not instanced.

```text
/Scene/Planting/Row_00/Plant_047     Xform, unique
  /Wood                               -> parts/Plant_3, instanceable
  /Collision                          Capsule, the trunk's physics proxy
  /Stem_00                            Xform, unique
    /Tube                             -> parts/Stem_1, instanceable
    /Leaf_00                          -> parts/Leaf_2, instanceable
```

A geometry prim must have no children, because USD cannot address the
children of an instanceable prim. The export stops with an error if one
does. An organ with nothing growing on it, such as a leaf, is itself the
geometry prim. Otherwise each leaf would need two prims, an `Xform` and a
geometry child. In a scene with hundreds of thousands of leaves, one prim
per leaf halves the number of prims.

Two kinds of part are exported not instanceable. One is a part that is
its own collider, the ground: physics cannot reach a collider inside a
shared copy ([export.md](export.md#colliders)). The other is a skin, one
mesh that bends with a flexible organ: each skin bends on its own, so it
cannot be a shared copy ([rods.md](rods.md#skins)).

> [!NOTE]
> **In Bevy terms.** A geometry prim is an entity that carries the
> `Geometry` bundle `Library::part` returns: the mesh handle, the material
> and a `UsdReference` naming the part. Every organ of one representative
> gets a clone of it, so they share one mesh and Bevy draws them together.

## What an element is built from

An element builds its meshes with the **geometry kernels**, the
framework's general-purpose mesh builders: tubes, flat outlines and folds,
bending under load, packing, and colour
([mesh-builders.md](mesh-builders.md)). They do not know whether a mesh is
a leaf or a stem.

The framework also ships one element, `terrain`. It builds the ground mesh
and `Ground`, a height field: it gives the ground's height at any (x, y),
so an element can place things on the ground. A generator that uses the
terrain puts it in its first stage. `terrain::build` is a system, not a
plugin. So the generator itself adds it to that stage, runs it when
`TerrainParams` changes, and initialises the `TerrainParams` and `Ground`
resources.

Code that belongs to one kind of scene but builds no mesh belongs to the
generator, not to an element: a layout solver, the code that gives every
plant its config, a named colour. It has no prefix in the mesh library. It
may still run in a stage of its own, as a generator's planting pass can.

## Rules

**Each element owns its part of the scene and rebuilds all of it every
time.** So if a rebuild makes fewer representatives, no old mesh is left
behind. Before it builds, a layer removes its meshes from the library,
`Library::clear(PART)`, and despawns (deletes) the entities it created.

**Elements share configs and library meshes, not prim paths.** A layer
reads the configs the layer before it created, and creates configs for the
next layer. To use a mesh another layer built, it looks the mesh up in the
library by its name, such as `Leaf_2`. No layer finds another layer's prims
by their path.

**Layers run in a fixed order.** Each layer must finish before the next one
reads what it created. The generator lists its stages in order once, in
its plugin.

> [!NOTE]
> **In Bevy terms.** A stage is a variant of a
> [`SystemSet`](https://docs.rs/bevy/latest/bevy/ecs/schedule/trait.SystemSet.html)
> enum, a named group of systems: `Grow::Leaves` in the example. The
> generator chains the variants once, inside `Build`, the framework's set
> for everything that builds the scene. An element adds its systems to its
> stage with `in_set`.

**Rebuild only on change.** A layer builds only when one of its configs
was added, changed or removed, or when its own params changed (the next
rule). Building meshes from curves is slow, so building every frame would
make the viewer slow to respond. Removing a config also counts as a
change. So when all of a layer's configs are removed, the layer runs one
last time and clears its meshes from the library. The run condition on the
configs is `configs_changed::<XConfig>`.

**Editing a parameter updates the existing configs in place.** When you
edit a leaf parameter, only the leaf meshes are rebuilt, not the whole
scene. The leaf configs were created by the layer before, so the leaf
layer does not create them again. Instead it runs a `reauthor` system,
which copies the layer's own params onto those configs. The rest of each
config stays as it was: its position and the random values it was made
from. A parameter that is copied into no config field, such as
`variations`, changes no config. So the layer's build also needs a
`resource_changed` condition on its params.

> [!NOTE]
> **In Bevy terms.** `reauthor` runs on `resource_changed::<LeafParams>`
> and writes with
> [`set_if_neq`](https://docs.rs/bevy/latest/bevy/ecs/change_detection/trait.DetectChangesMut.html#method.set_if_neq),
> which marks a component changed only when its value differs. So a
> `reauthor` that writes the values a config already has does not make
> `configs_changed` fire.

**Combine run conditions with `or_eager`, never `or_else`.** Both are
methods of Bevy's
[`SystemCondition`](https://docs.rs/bevy/latest/bevy/ecs/schedule/trait.SystemCondition.html)
that combine two conditions with a logical or. `or_else` skips the second
condition when the first is true, and `or_eager` always runs both. A
condition counts changes from the last time it ran, its `last_run`. So a
skipped condition reports the same change as new on the next frame, and one
edit rebuilds the layer twice.

**An element that sits on no layer before it creates its own configs.**
Ground cover is an example. Its `author` system creates configs from the
layout, the `Ground` and the scene params. It runs again whenever its own
params, the layout or the scene params change. It needs no `reauthor`,
because running `author` again already updates its configs.

**A parameter that must be one value from a fixed list is a string.** The
field declares the allowed values, `@Choices(&Kind::NAMES)`. The element
parses the string once into a Rust enum that has `ALL`, its variants,
`NAMES`, their names, and `parse`, from a name to its variant. When
Python hands the params to a build, an unknown name raises `ValueError`.
Inside the build the element reads an unknown name as the default and logs
a warning, because a build system must not fail. The viewer offers the
list as a dropdown, so it only writes a valid name.

**Determinism.** The same params must always give exactly the same scene,
byte for byte. The Isaac Lab spawner builds a scene once, saves it under a
key made from its params, and reuses the file after that
([isaac-lab.md](isaac-lab.md#the-cache)). That is only correct if the same
params always build the same scene. Bevy hands a system its entities in an
order that changes from run to run. So every organ carries a
`scene::Order`, given in the order organs are placed, and every layer
sorts by it before clustering.

## Where it lives

- [`src/scene/mod.rs`](../src/scene/mod.rs): the scene tree and the components that say what prim an entity becomes; `Library`, `Prototypes`, `Order`, `configs_changed`, `placed`, and the capsule and cable proxies
- [`src/geometry/`](../src/geometry/): the geometry kernels
- [`src/terrain.rs`](../src/terrain.rs): the ground and `Ground`
- [`src/palette.rs`](../src/palette.rs): colour, `Response`, `tissue`
- [`src/testing.rs`](../src/testing.rs): reading a built scene back in a test; `organs`, `prim`, `prim_path`

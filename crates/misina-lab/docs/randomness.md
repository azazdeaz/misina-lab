# Randomness

> [!WARNING]
> **Under development.** This page may not reflect the current state of
> the source. We're working on it.

The random numbers in a scene depend only on its params, the settings that
describe the scene ([generators.md](generators.md#the-params-aggregate)).
Most of them start from one number, the **seed**, which is itself one of
the params; the rest start from fixed numbers in the code. So the same
params give the same scene on every machine and in every run. The Isaac
Lab spawner, the code that puts a scene into the
simulator, relies on this. It saves each scene it builds as a USD file
under a key made from the params, and reuses that file when the same params
come again ([isaac-lab.md](isaac-lab.md#the-cache)).

By convention, a [generator](generators.md), the code that produces one
kind of scene, keeps the seed in the `seed` field of `SceneParams`. That is
the generator's own struct for the params that apply to the whole scene.

Each [element](elements.md), the code that builds one kind of thing in the
scene such as the leaves, takes its numbers from its own **streams**. A
stream is an `Rng`, the framework's random number generator. It starts from
the seed combined with a fixed number that belongs to the element
([Starting a stream](#starting-a-stream)). So the numbers one element
draws never shift the numbers another element draws: setting a plant to
carry more stems adds stems, and the trunk under them keeps its shape. A
stream also starts from a number that identifies what it is for, such as a
stem's place in the build order. The stem keeps its random numbers only
while that number stays the same.

Not every stream starts from the seed. The terrain, the ground element the
framework provides, has no seed parameter yet. It computes the height of
the ground from noise, a smooth random field, made from a fixed seed of its
own. Some elements also build a mesh from fixed numbers and the organ's
index or config alone. Changing the seed changes neither of these.

## Starting a stream

An element usually starts one stream for each mesh it builds and one for
each organ it places. An **organ** is one instance of an element, one leaf
or one stem. The stream's seed combines three numbers with XOR, the bitwise
exclusive-or:

```text
a representative's mesh   seed ^ LAYER_STREAM ^ salt(representative index)
an instance's placement   seed ^ CHILD_STREAM ^ salt(its Order)
```

`LAYER_STREAM` and `CHILD_STREAM` stand for the element's **stream
constants**: fixed numbers, chosen once, one per kind of draw. A **draw** is
one random number taken from a stream. A **representative** is an organ
whose mesh is built once and shared with the organs like it
([mesh-variations.md](mesh-variations.md)). `Order` numbers the organs in
the order they were placed ([elements.md](elements.md#rules)). `salt` turns
a small index such as 1, 2 or 3 into very different 64-bit numbers, so
organs with neighbouring indices get unrelated streams.

In code, where `scene` is the generator's `SceneParams` and `index` is the
representative's index:

```rust
use misina_lab::rng::{Rng, salt};

/// Any fixed number. Only that it never changes matters.
const STEM_STREAM: u64 = 0x589A_B41E_A1D2_F35B;

let mut rng = Rng::new(scene.seed ^ STEM_STREAM ^ salt(index as u64));
let bend = rng.range(-0.2, 0.2); // `unit()` draws in 0..1
```

Some elements work the other way round. They draw a seed for each organ
when they place it and keep it in the organ's **config**, the few numbers
that decide its mesh. The mesh is then built from a stream started from
that seed.

`Rng` is SplitMix64, a fixed algorithm written into the crate. It gives the
same numbers for a seed on every machine and in every version of the crate.
The default generator of another library may give different numbers after
a version update. Draw every random number from `Rng`.

## Rules

**Each element keeps its own stream constants**, one `*_STREAM: u64` per
kind of draw it makes. The framework's colour variation has its own
constant, `COLOR_STREAM`. `shade`, the framework's function that changes a
mesh's base colour slightly, takes a stream started from `COLOR_STREAM` and
the mesh's own seed. So changing the colours never changes the shapes.

**The order of the draws decides what an element builds.** A draw inserted
in the middle changes every number drawn after it, so the same seed gives a
different scene. Elements write down their draw order in a comment where a
reader might otherwise reorder the draws.

**Where a slot can stay empty, its draws still happen.** A slot is a place
an organ may fill or not, such as a bud that may grow a stem. If a bud
grows no stem, the element still draws that stem's numbers. So the other
stems get the same numbers as before: when one bud stops growing a stem,
only that stem goes away, and every other stem keeps its shape.

**If a value differs from organ to organ, draw it for each organ and keep
it in the organ's config.** The element builds only a limited number of
meshes, and gives each organ the mesh whose config is most like its own.
The element's metric decides what is compared, and it is given the organ's
config, sometimes with values the element computed for it. So two organs
that differ only in a value kept somewhere else, such as a scale applied
when the mesh is placed, get the same mesh
([mesh-variations.md](mesh-variations.md)).

## Where it lives

- [`src/rng.rs`](../src/rng.rs): `Rng`, `salt`
- [`src/palette.rs`](../src/palette.rs): `shade`, the per-mesh colour variation, and `COLOR_STREAM`
- [`src/terrain.rs`](../src/terrain.rs): `SEED`, the terrain's fixed seed

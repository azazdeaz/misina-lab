# Mesh variations

> [!WARNING]
> **Under development.** This page may not reflect the current state of
> the source. We're working on it.

A scene can hold ten thousand leaves, no two quite alike. Building a mesh
for each would cost too much memory and build time. So each
[element](elements.md), the code that builds one kind of thing such as the
leaves, builds only a limited number of meshes, and gives every leaf the
mesh that most resembles its own shape. Ten thousand leaves come out as a
few dozen meshes and ten thousand references to them. The limit is a
budget, by convention the element's `variations` parameter.

To decide which meshes to build, the element groups similar leaves. Each
**organ**, one leaf or one stem, has a **config**: the few numbers that
decide its mesh, such as its length and how far it droops. The element
picks some of the configs as **representatives**, at most as many as the
budget, and builds one mesh per representative. Each organ then uses the
mesh of the representative whose config is nearest to its own.

Replacing many values by the nearest of a few chosen ones is called vector
**quantization**, which is where the module's name, `quantize`, comes from.
In code the budget is the argument `k`, and the result, a `Codebook`, holds
the representatives and which one each config was given:

```rust
pub trait Metric<T> { fn distance(&self, a: &T, b: &T) -> f32; }

pub fn farthest_first<T: Clone, M: Metric<T>>(
    items: &[T], k: usize, max_radius: f32, metric: &M,
) -> Codebook<T>;
```

The element supplies the `Metric`, which says how different two configs
look, and `farthest_first` picks the representatives. It starts from the
first config and keeps adding the config farthest from every one picked so
far. It stops when it has `k` representatives, or when every config is
within `max_radius` of one.

Picking the farthest config each time keeps the *largest* distance from any
organ to its representative small: at most twice the smallest possible. A
method that keeps the average distance small, such as k-means, lets the
common shapes outweigh a rare one. Here a rare config is kept: one diseased
leaf among ten thousand healthy ones still gets a mesh of its own, provided
the metric puts it far from the rest. The method is Gonzalez's
farthest-first traversal for the **k-center** problem.

`quantize.rs` holds all of it, and depends on neither Bevy nor on what is
being grown.

## Writing a metric

- **Keep the config one plain struct, and let the metric decide what
  matters.** A `LeafConfig` holds all the fields, and a `LeafMetric` has a
  `distance` that chooses which fields count and multiplies each by a fixed
  weight. The weights turn a difference in a field into roughly how
  different the two meshes *look*.
- **Leave out a field that does not change the mesh.** The angle a leaf
  hangs at only places the mesh, and the code that builds the mesh never
  reads it. Two configs that differ only in that angle should share a mesh;
  each leaf still hangs at its own angle. If the metric counted the angle,
  the budget would be used up on configs that build the same mesh.
- **Put a large gap between categories.** A young plant and a mature one are
  different kinds of plant, not different sizes, so the metric puts them
  further apart than any two plants of one kind can be. The same goes for a
  leaf cut from one of five outline drawings: they are five shapes, not five
  nearby numbers, and with a gap that large a budget of five or more keeps
  all five.
- **Two configs that build no mesh are zero apart**, so that none of the
  budget goes to things that cost nothing.

## The budget

`variations` is an upper limit, the **budget**: the most representatives,
and so meshes, an element may keep. Raise it to spend memory on variety;
lower it to save memory at the cost of variety. To see what a budget cost,
read `radius` on the `Codebook`: how far the worst-matched organ is from the
representative whose mesh it uses.

A larger budget cannot create variety that is not in the configs. If every
config is identical there is one representative, however large the budget.
So each kind of organ needs at least one value that differs from organ to
organ: a girth drawn at random for each plant, a length and a node spacing
for each stem. Put such a value in the config, not in a scale applied when
the finished mesh is placed in the scene. Scaling a placed mesh needs no
extra mesh, but the clustering never sees the scale. Every mesh would then
be built at one arbitrary size, and not at the sizes that occur. Two
independent values give more visible variety than one, since the meshes
then differ in two ways and not along a single scale.

## Determinism

The same params must give the same scene on every machine and in every
run, because the Isaac Lab spawner caches a built scene under a key made
from its params and reuses it ([isaac-lab.md](isaac-lab.md#the-cache)).
`farthest_first` always starts from the first config in the list, and when
two configs are equally far it takes the one that comes first. So the
result depends only on the list it is given. Keeping the order of that list
the same from run to run is the element's job: give every organ a
`scene::Order` and sort by it before calling
([elements.md](elements.md#rules)).

## Where it lives

- [`src/quantize.rs`](../src/quantize.rs): `Metric`, `Codebook`, `farthest_first`

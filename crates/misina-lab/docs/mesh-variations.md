# Mesh variations

A scene can hold ten thousand leaves, no two quite alike. Building a mesh
for each would cost too much memory and build time. So the
[element](elements.md) that builds the leaves, the code for that one kind
of thing, builds only a limited number of meshes, and gives every leaf the
mesh that most resembles its own shape. Ten thousand leaves come out as a
few dozen meshes and ten thousand references to them. The limit is a
budget, by convention the element's `variations` parameter.

To decide which meshes to build, the element groups similar leaves. Each
**organ**, one leaf or one stem, has a **config**: the few numbers that
decide its mesh, such as its length and how far it droops. The element
picks some of the configs as **representatives**, as many as the budget
allows, and builds meshes only for them. Each organ then uses the mesh of
the representative whose config is nearest to its own.

Replacing many values by the nearest of a few chosen ones is called vector
**quantization**, which is where the module's name,
[`quantize`](https://docs.rs/misina-lab/latest/misina_lab/quantize/index.html),
comes from. In code the budget is the argument `k`, and the result, a
[`Codebook`](https://docs.rs/misina-lab/latest/misina_lab/quantize/struct.Codebook.html),
holds the representatives and which one each config was given:

```rust
pub trait Metric<T> { fn distance(&self, a: &T, b: &T) -> f32; }

pub fn farthest_first<T: Clone, M: Metric<T>>(
    items: &[T], k: usize, max_radius: f32, metric: &M,
) -> Codebook<T> { /* ... */ }
```

The element supplies the
[`Metric`](https://docs.rs/misina-lab/latest/misina_lab/quantize/trait.Metric.html),
which says how different two configs look, and
[`farthest_first`](https://docs.rs/misina-lab/latest/misina_lab/quantize/fn.farthest_first.html)
picks the representatives. It starts from the first config. Then it keeps
adding the config that is worst matched so far: the one farthest from its
nearest representative. It stops when it has `k` representatives, or when
every config is within `max_radius` of one. Pass 0 for `max_radius` to use
the whole budget: the picking then stops early only when every config left
is at distance zero from a representative.

Picking the farthest config each time keeps the *largest* distance from any
organ to its representative small. If `max_radius` does not stop the picking
first, and the metric is a true distance, that distance is at most twice
the smallest possible. A true distance is the same in both directions, and
going from one config to another by way of a third is never shorter than
going directly (the triangle inequality). A method that keeps the
average distance small, such as k-means, lets the common shapes outweigh a
rare one. Here a rare config is kept: one diseased leaf among ten thousand
healthy ones still gets a mesh of its own, provided the metric puts it far
from the rest, farther than `max_radius`, and the budget has room for it.
The method is Gonzalez's farthest-first traversal for the **k-center**
problem.

`quantize.rs` holds all of it, and depends on neither Bevy nor on what is
being grown.

## Writing a metric

- **Keep the config one plain struct, and let the metric decide what
  matters.** A `LeafConfig` holds all the fields, and a `LeafMetric` has a
  `distance` that chooses which fields count and multiplies each by a
  weight. The weights turn a difference in a field into roughly how
  different the two meshes *look*.
- **Leave out a field that does not change the mesh.** Take a leaf whose
  mesh is turned to the angle it hangs at only when it is placed, so the
  code that builds the mesh never reads the angle. Two configs that differ
  only in that angle should share a mesh; each leaf still hangs at its own
  angle. If the metric counted the angle, the budget would be used up on
  configs that build the same mesh. A field the mesh is built from
  normally counts, an angle included. Leaving such a field out is a choice:
  the organs that share a mesh then all show the representative's value
  for it, and the budget goes to the fields that count.
- **Put a large gap between categories.** A young plant and a mature one are
  different kinds of plant, not different sizes, so the metric puts them
  further apart than any two plants of one kind can be. The same goes for a
  leaf cut from one of five outline drawings: they are five shapes, not five
  nearby numbers, and with a gap that large a budget of five or more keeps
  all five.
- **Two configs that build no mesh are zero apart.** Then at most one of
  them is picked as a representative, so things that cost nothing take at
  most one place in the budget.

## The budget

`variations` is the **budget**, which the element passes to
`farthest_first` as `k`: the most representatives that call may keep. Raise
it to spend memory on variety; lower it to save memory at the cost of
variety. To see what a budget cost, read `radius` on the `Codebook`: how
far the worst-matched organ is from the representative whose mesh it uses.

An element whose organs are of several kinds can keep more representatives
than `variations`. It can raise `k` to the number of kinds present, so that
the budget has room for one of each, or cluster each kind on its own with
the whole budget. A `k` of zero gives an empty `Codebook`, with no
representative for any config, so an element keeps `k` at 1 or more, for
example with `params.variations.max(1) as usize`.

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

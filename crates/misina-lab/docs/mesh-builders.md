# Mesh builders

The **geometry kernels** are the framework's general-purpose mesh builders:
functions that make a tube along a line, fill a flat outline, bend a stalk
under its load, or move crowded fruit apart. An [element](elements.md) is
the code that builds one kind of thing in a scene, and it builds the mesh of
each **organ**, such as one leaf or one stem, from them. A kernel does not
know whether its mesh is a leaf or a stem, so every generator can use it.

The kernels are the `geometry` modules of the `misina_lab` crate, with
`palette`, the module for colour, beside them. The smallest use is the row
of boxes in the [README](../README.md): its `build` function calls
`geometry::mesh::box_mesh(size)` and registers the mesh it returns in the
mesh library, which holds every mesh of the scene by name
([elements.md](elements.md#the-pipeline)).

| Module | What it does |
| --- | --- |
| [`geometry::mesh`](https://docs.rs/misina-lab/latest/misina_lab/geometry/mesh/index.html) | `MeshData`, the mesh type that every kernel building a mesh returns, and simple shapes built in it, such as a box and a cylinder |
| [`geometry::strand`](https://docs.rs/misina-lab/latest/misina_lab/geometry/strand/index.html) | turns a polyline with a radius at each point into a tube. The tube can thicken all round at a node, where a branch or leaf joins (`Bulge`), and on one side where a stalk leaves it (`Collar`). Its rings are evenly spaced, or, given a tolerance (how far the mesh may lie from the true surface), placed only where the shape needs them |
| [`geometry::outline`](https://docs.rs/misina-lab/latest/misina_lab/geometry/outline/index.html) | a closed ring of points and the flat mesh that fills it ([Outlines](#outlines)) |
| [`geometry::shapes`](https://docs.rs/misina-lab/latest/misina_lab/geometry/shapes/index.html) | rings built in code, the pieces to build a lobed or toothed edge, and `bent`, which fills a ring and folds it like a leaf: curled about its midline and drooped at the tip |
| [`geometry::scatter`](https://docs.rs/misina-lab/latest/misina_lab/geometry/scatter/index.html) | spreads points over a band of ground, for things placed within an area rather than along a line |
| [`geometry::bending`](https://docs.rs/misina-lab/latest/misina_lab/geometry/bending/index.html) | `sag`: the shape a rod comes to rest in under the weight it carries. An element draws a stalk in the shape it grew into, carrying nothing, and `sag` bends it under its current load. So one shape works at every stage of growth: the heavier the load, the further it bends |
| [`geometry::packing`](https://docs.rs/misina-lab/latest/misina_lab/geometry/packing/index.html) | `pack`: moves a cluster of fruit on stalks apart, out of each other, the stems and each other's stalks, while every stalk keeps its length. A cluster too crowded to clear is left overlapping a little. `swing`: for stalks that were packed for larger fruit than they carry, which leaves the fruit hanging apart. It turns each fruit back toward the way it would hang alone, as far as there is room, so crowded fruit touch |
| [`palette`](https://docs.rs/misina-lab/latest/misina_lab/palette/index.html) | colour: `srgb` turns a hex colour into linear RGB, `shade` changes it a little for each mesh, `Response` says how a surface reflects light. `tissue` gives the colour of a mix of `Pigment` amounts, such as chlorophyll and lycopene, for a surface whose colour changes as the organ grows |

## Outlines

A flat organ, such as a leaf blade, is a ring of points filled with
triangles. `geometry::outline` does the fill. Every ring is built in the
same local frame: the point where the organ attaches is at the origin, the
shape runs along +X and its width across Y. So the fill works the same for
any ring, whatever built it.

`geometry::shapes` builds simple rings from a length, a width and, for a
leaf, how far along it is widest; a grass blade is one. A lobed, toothed
edge is built from curves with the same module's pieces. `cubic` samples a
Bézier curve, `serrate` cuts teeth along an edge, `simplify` drops points
that a tolerance does not need, and `untangle` cuts out loops. The generator
places the lobe tips from the organ's veins, the lines its blade is built
around, and joins the pieces into one ring. So one builder makes the organ
at any stage of its growth, and each of its meshes gets an edge of its own.

An organ that folds, such as a leaf along its veins, keeps each fold as
edges of its mesh. `outline::creased_mesh` takes lines across the ring and
makes each one a chain of mesh edges, cut into pieces as fine as the fill.
The element's own code, when it bends the mesh afterwards, can then fold
it sharply along those edges. A fold across the middle of triangles would
come out rounded over their width.

## Where it lives

- [`src/geometry/`](../src/geometry/): the geometry kernels, one module each
- [`src/palette.rs`](../src/palette.rs): colour, `Response`, `tissue`

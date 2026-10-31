# Garden Lab 🥕

Raised vegetable beds, generated for robotics simulation and authored to USD
for Isaac Lab: a level floor, a grid of timber-framed beds filled with soil a
path apart, and the crops in them, grown to the garden's week: lettuce, kale
and carrots so far, the toolset being built one species at a time. It is the
newest of [Misina Lab's generators](../../README.md#reference-generators).

> [!WARNING]
> **Under construction.** Three crops so far, none that bends yet. Expect
> breaking changes.

## What the garden holds

| Part | What it is | Knobs |
| --- | --- | --- |
| Floor | a level slab reaching a path's width past the outermost beds, its own collider | — |
| Beds | timber frames on posts, in columns along X and rows across Y, a path apart, each filled with soil up to below its rim; the frame and the soil each its own collider | width, length, height, soil below the rim, path width, columns, rows |
| Planting | every bed's soil dealt out in 30 cm squares to the crops by their shares, each square holding one, four, nine or sixteen plants as the crop's spacing asks, jittered off the grid | jitter, misses |
| Lettuce | a rosette of entire, ruffled blades one golden angle apart on a stub of a stem, the outer blades flat and drooping, the inner upright, a head closing over the crown when asked; every plant its own size, phase and vigour, red or green | leaves, blade length, ruffle, heading, tint; share, spacing, sowing week, bed, mesh variations |
| Kale | an upright stem, leaning and bowing a little, with a crown of large curled blades on long petioles up it one golden angle apart, upright at the head and spreading and drooping below, the nodes under the leaves kept stripped bare | height, leaves kept, leaf length, curl; the common knobs |
| Carrot | a rosette of compound leaves, a rachis with toothed leaflets in pairs, arching over a taproot whose orange shoulder pushes up through the soil as it swells; the root carries its ripeness and a `Cut` frame at the shoulder | leaves, leaf length, root length and diameter; the common knobs |
| Physics | what a robot bumps into besides the floor and the beds, scene-wide: every stem as a chain of capsules, and every fruit and root as a capsule | stems collide, fruit collides |

+Y is north, beds run east–west along X, the garden is centred on the origin
and the top of the floor is z = 0.

## The garden's week

The scene has one calendar, `scene.week`, and every crop a sowing week on
it. A plant's age is the difference, and its recipe grows it from a seedling
to harvest-ready over the crop's own weeks, eight for lettuce, ten for kale
and twelve for carrots, so one garden holds a bed just sown beside a bed
ready to pick. A bed is bare before its crop's sowing week.

## Running it

From the repository root,

```bash
cargo run --release -p garden-lab
```

opens the viewer and its parameter panel. From Python, after a checkout's
`uv sync`:

```python
import garden_lab

params = garden_lab.GardenParams()
params.bed.height = 0.6
params.write_usd("garden.usda")
```

In Isaac Lab, `garden_lab.isaaclab.GardenCfg` spawns the scene as an asset.
[docs/parameters.md](docs/parameters.md) lists every parameter, with its
default and range.

## Prims

```text
/Garden
  /Floor                          its own collider
  /Beds/Bed_00 …                  an Xform at the bed's centre on the floor
    /Frame                        the boards and the posts, its own collider
    /Soil                         the soil block, its own collider
    /Plant_000 …                  an Xform at the plant's foot on the soil
      /Crown                      a rosette's stub of stem, which its leaves sit on
      /Wood                       a stemmed plant's axes, one mesh of its own
      /Collision_00 …             a capsule along each straight span of its axes, when stems collide
      /Leaf_00 …                  one leaf each, the oldest first
      /Fruit_00                   a fruit or root: garden_lab:ripeness, :pick, :harvest_ready
        /Skin  /Cut  /Collision   the mesh, the frame a harvester takes it by, its capsule
```

Beds are numbered along the south row first, west to east. Every bed carries
`garden_lab:soil_z` and `garden_lab:rim_z`, the heights of its soil surface
and its rim above the floor, and `garden_lab:inner_size`, the soil's extent
inside the frame along X and Y with the rim's height. Plants are numbered by
planting place on a four-by-four lattice per square, so a place the miss rate
or a wider spacing left empty is a gap in the numbers
rather than a shift; every plant carries `garden_lab:species`,
`garden_lab:crop` and `garden_lab:age`, its weeks since sowing. A fruit or
root carries `garden_lab:ripeness`, 0 to 1, `garden_lab:pick`, how a
harvester takes it (`pull` for a root), and `garden_lab:harvest_ready`, true
from a ripeness of 0.9; its `Cut` frame sits where it is taken, +X up the
stalk.

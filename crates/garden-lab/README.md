# Garden Lab 🥕

Raised vegetable beds, generated for robotics simulation and authored to USD
for Isaac Lab: a level floor, a grid of timber-framed beds filled with soil a
path apart, and the crops in them, grown to the garden's week: lettuce, kale,
carrots, bush beans, pole beans on their trellis, zucchini and strawberries so
far, with the weeds between them, the toolset being built one species at a
time. It is the newest of
[Misina Lab's generators](../../README.md#reference-generators).

> [!WARNING]
> **Under construction.** Seven crops so far, two that bend. Expect breaking
> changes.

## What the garden holds

| Part | What it is | Knobs |
| --- | --- | --- |
| Floor | a level slab reaching a path's width past the outermost beds, its own collider | — |
| Beds | timber frames on posts, in columns along X and rows across Y, a path apart, each filled with soil up to below its rim; the frame and the soil each its own collider; a bed dealt a climber's squares gets a trellis on its north edge, two posts and a rail, its own collider | width, length, height, soil below the rim, path width, columns, rows, trellis height |
| Planting | every bed's soil dealt out in 30 cm squares to the crops by their shares, the climbers last so they take the north squares, each square holding one, four, nine or sixteen plants as the crop's spacing asks, or, for a crop spaced wider than a square, a plant every two or three squares on the lattice that fits the most into its block, jittered off the grid; every climber gets a string from its foot to the rail | jitter, misses |
| Weeds | the plants nobody sowed, on a 20 cm slot grid over every bed's soil and over the floor off the beds, so many to the square metre of each: a tuft of grass, a mat, a dandelion's rosette, a bolter sending up a wand and a sprawling broadleaf, each one mesh, at any stage whatever the week | bed pressure, path pressure, tall share, mesh variations, leaf detail |
| Lettuce | a rosette of entire, ruffled blades one golden angle apart on a stub of a stem, the outer blades flat and drooping, the inner upright, a head closing over the crown when asked; every plant its own size, phase and vigour, red or green | leaves, blade length, ruffle, heading, tint; share, spacing, sowing week, bed, mesh variations |
| Kale | an upright stem, leaning and bowing a little, with a crown of large curled blades on long petioles up it one golden angle apart, upright at the head and spreading and drooping below, the nodes under the leaves kept stripped bare | height, leaves kept, leaf length, curl; the common knobs |
| Carrot | a rosette of compound leaves, a rachis with toothed leaflets in pairs, arching over a taproot whose orange shoulder pushes up through the soil as it swells; the root carries its ripeness and a `Cut` frame at the shoulder | leaves, leaf length, root length and diameter; the common knobs |
| Bush bean | a short stem branching from its lowest nodes into a bush, a trifoliate leaf at every node of every axis, pods in clusters hanging under the leaves from the fourth node up once the plant is older than five weeks, growing out as they ripen | height, branches, leaf length, pod length; the common knobs |
| Pole bean | a vine twining anticlockwise up its string to the trellis rail, a large trifoliate leaf at every node and pods hanging from the fourth node up once the vine is older than six weeks; the vine is the garden's one bending stem | height, leaf length, pod length; the common knobs |
| Zucchini | a stout, leaning stem with a palmate five-lobed blade on a long petiole every 1.2 cm up it, the oldest leaves lying out near level and the youngest standing steep, each blade hinged down where its petiole ends; three fruit at most, on the fourth to sixth nodes, set a week apart once the plant is older than five weeks, lying out from the stem a little above level, growing as they ripen and cut to pick; one plant every three squares at its default spacing | height, leaf length, fruit length; the common knobs |
| Strawberry | a crown of trifoliate leaves with coarsely toothed leaflets on long petioles, every berry hanging from a stalk of its own arching out from the crown, set a week apart once the plant is older than six and a half weeks and growing out as they ripen from pale to red, cut to pick; a runner from a plant older than five weeks, humping over and down onto the soil and along it with a plantlet of three small leaves at its tip; the stalks are the garden's other bending stems | leaves, leaf length, fruits, fruit length, runners; the common knobs |
| Physics | what a robot bumps into besides the floor and the beds, and what bends, scene-wide: every stem as a chain of capsules, every fruit and root as a capsule, every leaf as a box round its blade, and the stems that bend as rods the solver moves, their leaves, pods and berries riding them, boxes and all | stems collide, fruit collides, leaves collide, flexible stems, segment length |

+Y is north, beds run east–west along X, the garden is centred on the origin
and the top of the floor is z = 0.

## The garden's week

The scene has one calendar, `scene.week`, and every crop a sowing week on
it. A plant's age is the difference, and its recipe grows it from a seedling
to harvest-ready over the crop's own weeks, eight for lettuce, bush beans and
zucchini, ten for kale and pole beans, eleven for strawberries and twelve for
carrots, so one garden holds a bed just sown beside a bed ready to pick. A
bed is bare before its crop's sowing week. The weeds take no notice of it.

## How it is meshed

Every rigid stem is a tube rung where its shape needs it, swelling at every
node an organ leaves it at and toward that organ's stalk, so a petiole grows
out of the stem rather than poking through it, and at the scar a stripped
kale leaf leaves; it rounds off to a point past its last organ, and a
zucchini's is ridged. A rod, with `flexible_stems`, is plain segment tubes
shared by size. A crown is a low dome the leaves leave from, a
strawberry's dark. A blade is
pleated on its veins, each vein a valley and the blade standing up to a crest
between two: a simple blade or a leaflet on secondary veins leaving its
midrib, a strawberry leaflet deeply, a loose-leaf lettuce and a curly kale
frilled at the margin, and a zucchini blade on the main veins fanning from
its middle. The veins and the crests are lines of edges through the fill, so
the pleats fold sharply however coarse the fill is. A zucchini blade is
toothed all round and a curly kale's notched along its edges; a simple blade
carries a tapered midrib. A strawberry sits under a calyx of seven sepals,
studded with achenes; a zucchini is ribbed and club-shaped, on a ridged
stalk flaring into it, with a blossom scar; a pod is flattened, swells at
its beans, hangs by a stalk under a calyx cup and ends in a curled beak; a
carrot is creased round and drawn out into a tail.

Leaves and fruit carry a colour per point, which USD carries as a vertex
`displayColor`: a strawberry colours from its tip as it ripens, and blades
pale along their midribs and veins.

`scene.detail` sets how finely the crop is meshed: at 1 a rigid stem's tube
strays about a pixel from its shape to a camera half a meter away and a
blade is a few hundred triangles, 4 is four times as fine and 0.1 the
coarsest at which every organ still shows. The weeds have a detail of their
own. The shots under `docs/shots/garden-lab/` frame one crop each at
harvest, close enough to judge all this;
`docs/shots/render.sh docs/shots/garden-lab/*.json` renders them.

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
With `physics.flexible_stems` the pole beans' vines and the strawberries'
fruit stalks are rods, which only Newton's VBD solver steps;
`GardenCfg.without_rods()` says whether a cfg authors any, and
`misina_lab.isaaclab.rods.make_physics_cfg_newton` picks the solver from it.
[docs/parameters.md](docs/parameters.md) lists every parameter, with its
default and range.

## Prims

```text
/Garden
  /Floor                          its own collider
  /Beds/Bed_00 …                  an Xform at the bed's centre on the floor
    /Frame                        the boards and the posts, its own collider
    /Soil                         the soil block, its own collider
    /Trellis                      two posts and a rail on the north edge, when a climber is dealt a square; its own collider
    /Plant_000 …                  an Xform at the plant's foot on the soil
      /Crown                      a rosette's stub of stem, which its leaves sit on
      /Wood                       a stemmed plant's rigid axes, one mesh of its own
      /Collision_00 …             a capsule along each straight span of its rigid axes, when stems collide
      /String                     a climber's string, an Xform turned to the rail
        /Cord  /Collision         its mesh and its capsule, in the plant's collision group
      /Rod_00 …                   one Xform per bending axis at the plant's frame, numbered by axis, when stems are flexible: a pole bean's vine, each of a strawberry's fruit stalks
        /Cable                    the curve the solver bends, held at its first segment
        /Cable_edge_body_0 …      a prim per segment, driven by the solver
          /Stem                   the segment's tube
          /Leaf_04  /Fruit_33 …   what rides the segment, placed in its frame
      /Leaf_00 …                  one leaf each, numbered by node; a branch's leaves, or a runner's plantlet's, from a thousand times its number
        /Blade  /Collision        when leaves collide: the mesh, and a box round the blade, a Cube scaled to its half extents
      /Fruit_00 …                 a fruit or root, a stemmed plant's eight to a node, a strawberry's by berry: garden_lab:ripeness, :pick, :harvest_ready
        /Skin  /Cut  /Collision   the mesh, the frame a harvester takes it by, its capsule
  /Weeds                          a Scope, absent when no weed grew
    /Bed_00/Weed_0017 …           a weed in the soil of Bed_00, one mesh, named by its slot; garden_lab:species, :crop false
    /Paths/Weed_0042 …            one on the floor off the beds
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
harvester takes it (`pull` for a root, `snap` for a pod, `cut` for a
zucchini or a strawberry), and `garden_lab:harvest_ready`, true from a
ripeness of 0.9; its `Cut` frame sits where it is taken, halfway down its
stalk or at its top without one, +X up the stalk.

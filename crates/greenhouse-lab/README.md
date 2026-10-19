# greenhouse-lab

The rows of a Western European high-wire tomato greenhouse, generated for
robotics simulation and authored to USD for Isaac Lab: the floor, the pipe
rails, the gutters with their slabs and blocks, the crop wires, and the
crop on them: stems, leaves, trusses and fruit. The building, its roof,
glass and columns, is left to an HDR or a Gaussian splat. It is built on
[misina-lab](../misina-lab/README.md), like the vineyard beside it.

The dimensions and the variations follow
[the research notes](../../docs/tomato-greenhouse-research.md), drawn from
Dutch research compartments, supplier sheets and one measured truss study.
A stem keeps its leaves from the head down and is stripped below them, as
growers leave it, so the trusses in the harvest band hang off bare stem.

## What a row holds

| Part | What it is | Knobs |
| --- | --- | --- |
| Floor | white film under the crop and concrete across the row heads, each its own collider | the main path's width |
| Rail | two heating pipes in every path, on a support every meter, each segment a capsule collider | gauge, pipe diameter, height, unevenness up to and past the 2° spec |
| Gutter | a steel channel, hanging or standing, with stone wool slabs and a block at every plant | height, mount, plant spacing |
| Wires | two crop wires over every gutter, for the V the stems are trained in | height, spread |
| Stems | two to a plant, each lowered along the gutter and rising to a hook on its wire, with its twine wound down it; a leaf at every node and a truss in every third internode, the leaves stripped below the top few with a stub left at each | harvest height, trusses on a stem, leaves kept, lowered length, extra stems, trusses turned to the path |
| Leaves | a compound leaf: a petiole and a rachis with a terminal leaflet, three or four pairs of laterals and small leaflets between them, each leaflet outlined from its veins; small and upright at the head, drooping further down. Visual only, with no collider | length, droop, mesh detail |
| Trusses | peduncle, rachis and a pedicel to every fruit, with a `Cut` and a `Bend` frame | fruit count and diameter, branched, peduncle length, diameter and angle, missing fruit |
| Fruit | every fruit its own prim, with a calyx of five or six sepals and a stage from fruit set to red: fruit grow toward the head, a stem ripens from the bottom up, a truss from its base to its tip, and a fruit from its blossom end | ripe trusses on a stem, the spread in a truss, the ripe colour, height to width, ribbing, jointed pedicels |

Rows run along +X from the row head at x = 0, the main path lies at
negative x, and z = 0 is the top of the floor. Every gutter has a path on
both sides, so every crop wall faces one a robot can drive.

## Running it

From the repository root,

```bash
cargo run --release -p greenhouse-lab
```

opens the viewer and its parameter panel. From Python, after
`pip install greenhouse-lab`:

```python
import greenhouse_lab

params = greenhouse_lab.GreenhouseParams()
params.rail.unevenness = 1.0
params.write_usd("greenhouse.usda")
```

In Isaac Lab, `greenhouse_lab.isaaclab.GreenhouseCfg` spawns the scene as
an asset. [docs/parameters.md](docs/parameters.md) lists every parameter,
with its default and range.

## Prims

```text
/Greenhouse
  /Floor/Film  /Floor/Concrete        each its own collider
  /Paths/Path_03/Rail
    /Pipe_L_007/Steel  /Collision     one segment, support to support
    /Support_L_007 …
  /Rows/Row_02
    /Gutter  /Hanger_00 …  /Slab_00 …  /Block_017 …
    /Wire_L  /Wire_R
    /Plant_017
      /Stem_0
        /Tube  /Hook  /Twine
        /Leaf_00 …                    the lowest still on the stem first
        /Truss_00                     the lowest still on the stem
          /Peduncle  /Cut  /Bend
          /Fruit_00 …
```

`Cut` sits at the peduncle's base and `Bend` where it bends most, each with
+X along the peduncle: where a cut-flush gripper and a clamp-and-cut one
take a truss. Every fruit carries `greenhouse_lab:stage`, its growth stage
or USDA colour class, and `greenhouse_lab:ripeness`, the share of its
surface that has coloured.

Left is +Y: the side on the left looking down a row from its head. A
standing gutter has `Stand_00 …` where a hanging one has hangers.

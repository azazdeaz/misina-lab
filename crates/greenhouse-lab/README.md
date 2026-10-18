# greenhouse-lab

The rows of a Western European high-wire tomato greenhouse, generated for
robotics simulation and authored to USD for Isaac Lab: the floor, the pipe
rails, the gutters with their slabs and blocks, and the crop wires. The
building, its roof, glass and columns, is left to an HDR or a Gaussian
splat. It is built on [misina-lab](../misina-lab/README.md), like the
vineyard beside it.

The dimensions and the variations follow
[the research notes](../../docs/tomato-greenhouse-research.md), drawn from
Dutch research compartments, supplier sheets and one measured truss study.
The stems, leaves, trusses and fruit come next.

## What a row holds

| Part | What it is | Knobs |
| --- | --- | --- |
| Floor | white film under the crop and concrete across the row heads, each its own collider | the main path's width |
| Rail | two heating pipes in every path, on a support every meter, each segment a capsule collider | gauge, pipe diameter, height, unevenness up to and past the 2° spec |
| Gutter | a steel channel, hanging or standing, with stone wool slabs and a block at every plant | height, mount, plant spacing |
| Wires | two crop wires over every gutter, for the V the stems are trained in | height, spread |

Rows run along +X from the row head at x = 0, the main path lies at
negative x, and z = 0 is the top of the floor. Every gutter has a path on
both sides, so every crop wall faces one a robot can drive.

## Running it

From the repository root,

```bash
cargo run --release -p greenhouse-lab
```

opens the viewer and its parameter panel. From Python:

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
```

Left is +Y: the side on the left looking down a row from its head. A
standing gutter has `Stand_00 …` where a hanging one has hangers.

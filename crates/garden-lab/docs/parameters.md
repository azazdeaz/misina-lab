<!-- Generated from the Rust params structs by `cargo test regen_params -- --ignored`.
     Edit the structs, not this file: misina-lab's docs/editing-parameters.md says how. -->

# Parameters

Every parameter is reachable three ways under one name: as a control in the viewer
(`cargo run --release`), as an attribute of a `garden_lab.GardenParams` fragment,
and as a field of the matching Isaac Lab `GardenCfg` fragment: `params.<fragment>.<field>`
in Python and `GardenCfg(<fragment>=<Fragment>Cfg(<field>=...))` in Isaac Lab set the
same thing. The slider range is what the viewer offers; Python takes any value the
generator can build. Lengths are in meters.

## Scene

`SceneParams` in Python, `SceneCfg` in Isaac Lab.

Scene-wide parameters: the seed, and how far into the season the garden is.

| Parameter | Type | Default | Slider range | Description |
|---|---|---|---|---|
| `seed` | int | `0` | 0 to 64 | The one seed the whole scene is generated from. Each element salts it with a constant of its own, so a new seed is a different garden and an edit to one element never re-rolls another. |
| `week` | float | `8.0` | 0 to 20 | The week on the garden's calendar. Each crop grows from its own sowing week toward its harvest, so one garden holds seedlings beside a bed ready to pick. |

## Physics

`PhysicsParams` in Python, `PhysicsCfg` in Isaac Lab.

What a robot bumps into, besides the floor and the beds, and what bends. Scene-wide, since a demo wants one switch.

| Parameter | Type | Default | Slider range | Description |
|---|---|---|---|---|
| `stems_collide` | bool | `True` |  | Whether every stem collides, as a chain of capsules along it. A rod collides as itself, whatever this says. |
| `fruit_collides` | bool | `False` |  | Whether every fruit and root collides, as a capsule each. |
| `flexible_stems` | bool | `False` |  | Whether the stems that bend, a pole bean's vine, are rods the solver moves, their leaves and pods riding them; off, they are wood like every other stem. |
| `segment` | float | `0.05` | 0.02 to 0.06 | Length of a rod's segments, in meters: shorter bends smoother and costs a body per segment. No longer than 6 cm, the most a pole bean's vine can be sampled at and still clear its string. |

## Beds

`BedParams` in Python, `BedCfg` in Isaac Lab.

The raised beds: their size, and how they are laid out.

Beds run east–west along X in `columns`, and the `rows` are numbered across Y from the south, with a path between any two beds and round the whole garden.

| Parameter | Type | Default | Slider range | Description |
|---|---|---|---|---|
| `width` | float | `1.2` | 0.6 to 1.5 | Width of a bed across Y, outside the frame, in meters. Up to 1.2 m is reachable from both sides; 1.5 m is FarmBot Genesis's bed. |
| `length` | float | `2.4` | 1.2 to 4.8 | Length of a bed along X, outside the frame, in meters. 4 by 8 feet is the common footprint, and lumber comes in 8 feet. |
| `height` | float | `0.3` | 0.15 to 0.9 | Height of the frame's rim above the floor, in meters. 15 to 30 cm is typical; 60 to 90 cm is an accessible bed, which a wheeled robot reaches too. Below 0.25 m the soil is shallower than the 25 cm growing guides ask for. |
| `rim` | float | `0.04` | 0 to 0.1 | How far the soil surface lies below the rim, in meters. |
| `path` | float | `0.9` | 0.45 to 1.5 | Width of the paths between the beds and round the garden, in meters. 45 to 60 cm is comfortable on foot; 90 cm and up takes a cart or a wheeled robot. |
| `columns` | int | `2` | 1 to 4 | Beds along X in each row. |
| `rows` | int | `3` | 1 to 6 | Rows of beds across Y. |
| `trellis` | float | `2.0` | 1.2 to 2.5 | Height of the trellis rail above the soil, in meters, on a bed that holds a climber: two posts at the bed's north corners and a rail between them, a string from it down to every climbing plant. |

## Planting

`PlantingParams` in Python, `PlantingCfg` in Isaac Lab.

How a bed is filled, beyond what each crop says for itself.

| Parameter | Type | Default | Slider range | Description |
|---|---|---|---|---|
| `jitter` | float | `0.2` | 0 to 0.5 | How far a plant stands off its exact place, as a fraction of the distance to its neighbours: 0 is a stamped grid. |
| `misses` | float | `0.05` | 0 to 0.5 | Share of planting places left empty: seeds that never came up. |

## Lettuce

`LettuceParams` in Python, `LettuceCfg` in Isaac Lab.

Lettuce, loose-leaf or heading: a rosette of entire blades, ruffled or smooth, round a stub of stem a couple of centimetres tall, grown to harvest over eight weeks.

| Parameter | Type | Default | Slider range | Description |
|---|---|---|---|---|
| `leaves` | int | `28` | 8 to 48 | Blades on a harvest-ready plant. |
| `leaf_length` | float | `0.15` | 0.08 to 0.3 | Length of the largest blade at harvest, in meters. |
| `ruffle` | float | `0.6` | 0 to 1 | How much the margin ruffles: 0 is a smooth butterhead, 1 a frilly loose-leaf. |
| `heading` | bool | `False` |  | Whether the inner blades close over the crown into a head. |
| `tint` | str | `"green"` | `green`, `red`, `mix` | Foliage colour: `green`, `red`, or a `mix` of plants of either across the bed. |
| `share` | float | `0.25` | 0 to 1 | Share of the garden's planting squares this crop gets, against the other crops'; 0 leaves it out. |
| `spacing` | float | `0.15` | 0.03 to 1 | Distance between plants, in meters, rounded to one, two, three or four per side of a 30 cm square. |
| `sown_week` | float | `0.0` | 0 to 20 | The week it was sown, on the scene's calendar: a plant's age is the scene's week less this, and a bed is bare before it. |
| `bed` | int | `0` | 0 to 24 | The one bed it is confined to, counted from 1; 0 plants it anywhere. |
| `variations` | int | `12` | 1 to 64 | How many distinct meshes each of its organs, its leaves and its fruit, may share. A budget, not a count: the organs are clustered and this is how many representatives the clustering may keep. |

## Kale

`KaleParams` in Python, `KaleCfg` in Isaac Lab.

Kale: an upright stem carrying a crown of large curled blades on long petioles, the nodes below the leaves kept stripped bare, grown to harvest over ten weeks.

| Parameter | Type | Default | Slider range | Description |
|---|---|---|---|---|
| `height` | float | `0.6` | 0.3 to 1 | Height of a harvest-ready stem, in meters. |
| `leaves_kept` | int | `10` | 4 to 20 | Leaves left on the stem, counted down from its head; the nodes below are stripped. A stem grows a node every 3.5 cm. |
| `leaf_length` | float | `0.35` | 0.15 to 0.6 | Length of a grown leaf, petiole and blade, in meters. |
| `curl` | float | `0.8` | 0 to 1 | How tightly the margin curls: 0 is a flat lacinato, 1 a curly kale. |
| `share` | float | `0.25` | 0 to 1 | Share of the garden's planting squares this crop gets, against the other crops'; 0 leaves it out. |
| `spacing` | float | `0.3` | 0.03 to 1 | Distance between plants, in meters, rounded to one, two, three or four per side of a 30 cm square. |
| `sown_week` | float | `0.0` | 0 to 20 | The week it was sown, on the scene's calendar: a plant's age is the scene's week less this, and a bed is bare before it. |
| `bed` | int | `0` | 0 to 24 | The one bed it is confined to, counted from 1; 0 plants it anywhere. |
| `variations` | int | `12` | 1 to 64 | How many distinct meshes each of its organs, its leaves and its fruit, may share. A budget, not a count: the organs are clustered and this is how many representatives the clustering may keep. |

## Carrot

`CarrotParams` in Python, `CarrotCfg` in Isaac Lab.

Carrot: a rosette of finely divided leaves over a taproot whose shoulder shows at the soil, pulled by its tops, grown to harvest over twelve weeks.

| Parameter | Type | Default | Slider range | Description |
|---|---|---|---|---|
| `leaves` | int | `9` | 4 to 16 | Leaves on a harvest-ready plant. |
| `leaf_length` | float | `0.3` | 0.15 to 0.5 | Length of a grown leaf, in meters. |
| `root_length` | float | `0.18` | 0.08 to 0.3 | Length of a harvest-ready root, in meters. |
| `root_diameter` | float | `0.03` | 0.015 to 0.06 | Diameter of a harvest-ready root at its shoulder, in meters. |
| `share` | float | `0.25` | 0 to 1 | Share of the garden's planting squares this crop gets, against the other crops'; 0 leaves it out. |
| `spacing` | float | `0.075` | 0.03 to 1 | Distance between plants, in meters, rounded to one, two, three or four per side of a 30 cm square. |
| `sown_week` | float | `0.0` | 0 to 20 | The week it was sown, on the scene's calendar: a plant's age is the scene's week less this, and a bed is bare before it. |
| `bed` | int | `0` | 0 to 24 | The one bed it is confined to, counted from 1; 0 plants it anywhere. |
| `variations` | int | `12` | 1 to 64 | How many distinct meshes each of its organs, its leaves and its fruit, may share. A budget, not a count: the organs are clustered and this is how many representatives the clustering may keep. |

## Bush bean

`BushBeanParams` in Python, `BushBeanCfg` in Isaac Lab.

Bush bean: a short stem branching from its lowest nodes into a bush of trifoliate leaves, pods hanging in clusters under the upper leaves, grown to harvest over eight weeks.

| Parameter | Type | Default | Slider range | Description |
|---|---|---|---|---|
| `height` | float | `0.4` | 0.2 to 0.6 | Height of a harvest-ready plant's main stem, in meters. |
| `branches` | int | `3` | 0 to 5 | Branches leaving the lowest nodes. |
| `leaf_length` | float | `0.18` | 0.08 to 0.3 | Length of a grown leaf, petiole and leaflets, in meters. |
| `pod_length` | float | `0.13` | 0.06 to 0.2 | Length of a ripe pod, in meters. |
| `share` | float | `0.25` | 0 to 1 | Share of the garden's planting squares this crop gets, against the other crops'; 0 leaves it out. |
| `spacing` | float | `0.1` | 0.03 to 1 | Distance between plants, in meters, rounded to one, two, three or four per side of a 30 cm square. |
| `sown_week` | float | `0.0` | 0 to 20 | The week it was sown, on the scene's calendar: a plant's age is the scene's week less this, and a bed is bare before it. |
| `bed` | int | `0` | 0 to 24 | The one bed it is confined to, counted from 1; 0 plants it anywhere. |
| `variations` | int | `12` | 1 to 64 | How many distinct meshes each of its organs, its leaves and its fruit, may share. A budget, not a count: the organs are clustered and this is how many representatives the clustering may keep. |

## Pole bean

`PoleBeanParams` in Python, `PoleBeanCfg` in Isaac Lab.

Pole bean: a vine twining up a string from its bed's trellis, a large trifoliate leaf at every node and pods hanging from the upper nodes, grown to harvest over ten weeks.

| Parameter | Type | Default | Slider range | Description |
|---|---|---|---|---|
| `height` | float | `2.2` | 1 to 3 | Length of a harvest-ready vine up its string, in meters; a vine stops at the string's top. |
| `leaf_length` | float | `0.25` | 0.1 to 0.4 | Length of a grown leaf, petiole and leaflets, in meters. |
| `pod_length` | float | `0.16` | 0.08 to 0.25 | Length of a ripe pod, in meters. |
| `share` | float | `0.25` | 0 to 1 | Share of the garden's planting squares this crop gets, against the other crops'; 0 leaves it out. |
| `spacing` | float | `0.15` | 0.03 to 1 | Distance between plants, in meters, rounded to one, two, three or four per side of a 30 cm square. |
| `sown_week` | float | `0.0` | 0 to 20 | The week it was sown, on the scene's calendar: a plant's age is the scene's week less this, and a bed is bare before it. |
| `bed` | int | `0` | 0 to 24 | The one bed it is confined to, counted from 1; 0 plants it anywhere. |
| `variations` | int | `12` | 1 to 64 | How many distinct meshes each of its organs, its leaves and its fruit, may share. A budget, not a count: the organs are clustered and this is how many representatives the clustering may keep. |

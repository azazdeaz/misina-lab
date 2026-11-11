<!-- Generated from the Rust params structs by `cargo test regen_params -- --ignored`.
     Edit the structs, not this file: misina-lab's docs/editing-parameters.md says how. -->

# Parameters

Every parameter is reachable three ways under one name: as a control in the viewer
(`cargo run --release`), as an attribute of a `greenhouse_lab.GreenhouseParams` fragment,
and as a field of the matching Isaac Lab `GreenhouseCfg` fragment: `params.<fragment>.<field>`
in Python and `GreenhouseCfg(<fragment>=<Fragment>Cfg(<field>=...))` in Isaac Lab set the
same thing. The slider range is what the viewer offers; Python takes any value the
generator can build. Lengths are in meters.

## Scene

`SceneParams` in Python, `SceneCfg` in Isaac Lab.

Scene-wide parameters: the seed, and how far into the season the crop is.

| Parameter | Type | Default | Slider range | Description |
|---|---|---|---|---|
| `seed` | int | `0` | 0 to 64 | The one seed the whole scene is generated from. Each element salts it with a constant of its own, so a new seed is a different greenhouse and an edit to one element never re-rolls another. |
| `week` | float | `16.0` | 0 to 40 | Weeks since the crop was planted. A stem grows about a truss a week up its twine, reaches the wire after about nine weeks at the defaults, when its first trusses ripen, and is lowered along the gutter from then on. |

## Rows

`LayoutParams` in Python, `LayoutCfg` in Isaac Lab.

Where the rows go.

Rows run along X from the row head, where the concrete main path crosses them, and are numbered across Y. Every gutter has a path on both sides, so there is one path more than there are rows.

| Parameter | Type | Default | Slider range | Description |
|---|---|---|---|---|
| `rows` | int | `6` | 1 to 12 | How many rows, one gutter each. |
| `pitch` | float | `1.6` | 1.5 to 1.8 | Distance between neighbouring gutters, in meters: the row pitch. |
| `row_length` | float | `20.0` | 5 to 120 | Length of a row from the main path to its far end, in meters. Farm rows run 90 to 115 m; a shorter one simulates faster. |
| `main_path` | float | `3.0` | 3 to 5 | Width of the concrete main path across the row heads, in meters: where a robot changes rows. |
| `plant_spacing` | float | `0.36` | 0.3 to 0.5 | Distance between plants along a gutter, in meters, each in a propagation block on a slab. A slab is as long as the whole number of plants that brings it nearest 1.2 m, so the plants stand this far apart the whole row along. |

## Pipe rail

`RailParams` in Python, `RailCfg` in Isaac Lab.

The pipe rail in every path: two heating pipes a trolley rides on.

The pipes rest on a support every meter, and `unevenness` moves the supports off level, each by a draw of its own: the pair tilts across and each pipe pitches along its length. Each pipe turns down into the floor at the row head, and a return bend joins the two at the far end.

| Parameter | Type | Default | Slider range | Description |
|---|---|---|---|---|
| `gauge` | float | `0.6` | 0.425 to 0.8 | Distance between the two pipes, centre to centre, in meters. Growers lay 425, 500, 550, 600 and 800 mm. |
| `pipe_diameter` | float | `0.051` | 0.045 to 0.051 | Outer diameter of a pipe, in meters: the slider stops at the 45 and 51 mm heating pipes. |
| `height` | float | `0.15` | 0.05 to 0.3 | Height of the pipes' axis above the floor, in meters. An estimate, to be set from a measured site. |
| `unevenness` | float | `0.5` | 0 to 2 | How far the supports stand off level. 0 is a perfect rail; at 1 the pair tilts across by up to the 2° the rail spec allows; 2 is twice out of spec. |

## Gutter

`GutterParams` in Python, `GutterCfg` in Isaac Lab.

The gutter each row's crop stands in, and what holds it up.

Stone wool slabs lie end to end in the gutter, and a propagation block sits on a slab at every plant; the rows' plant spacing places them.

| Parameter | Type | Default | Slider range | Description |
|---|---|---|---|---|
| `height` | float | `0.5` | 0.3 to 1 | Height of the gutter's floor above the ground, in meters: where the slabs rest. |
| `mount` | str | `"hanging"` | `hanging`, `standing` | What holds the gutter up: `hanging` from a rod every 2.5 m, which leaves the floor under it clear, or `standing` on a stand every 2.5 m. |

## Crop wires

`WireParams` in Python, `WireCfg` in Isaac Lab.

The two crop wires over every gutter, which the stems are trained up to in a V.

| Parameter | Type | Default | Slider range | Description |
|---|---|---|---|---|
| `height` | float | `4.5` | 3.5 to 5.5 | Height of the crop wires above the floor, in meters: how tall the crop stands. |
| `spread` | float | `0.35` | 0.1 to 0.6 | How far each of a row's two wires runs to the side of its gutter, in meters. An estimate, to be set from a measured site. |

## Stems

`StemParams` in Python, `StemCfg` in Isaac Lab.

The crop's stems: two to a plant, trained alternately to the left and the right wire, and lowered as they grow.

Each stem grows a truss a week from planting, up its twine to a hook on the wire. Once its head is a hook's length below the wire it is lowered, and all the stem the wire is too low for lies along the gutter. In full production the stem between `harvest_height` and the head carries `trusses` trusses with three leaves to each, so how much a stem grows in a week follows from the wire height rather than being set. The crop is deleafed about weekly, so the stem below its leaves is bare.

| Parameter | Type | Default | Slider range | Description |
|---|---|---|---|---|
| `harvest_height` | float | `1.2` | 0.8 to 1.8 | Height above the floor of the lowest truss still on a stem once the crop is in full production, in meters: where a harvesting arm reaches to. A stem's lowest hangs up to a week's growth above it, the higher the sooner after its last picking, and a stem that grows faster than the rest hangs it up to 5% of its live height lower. The stem below it has been picked. |
| `trusses` | int | `9` | 6 to 12 | Trusses on a stem in full production, from the lowest still on it up to the head. At a truss a week, also the weeks from a truss showing at the head to its picking. |
| `development_spread` | float | `0.2` | 0 to 0.5 | How far stems drift apart in their development, in weeks: each truss comes a little sooner or later than the last, so a stem `week` weeks old runs up to this times √`week` ahead of the crop or behind it, its trusses hanging that much of a week's growth lower or higher and ripening that much sooner or later. At 0.2, up to 0.8 of a week at week 16; at 0 every stem keeps in step with the crop, and its trusses stray from its neighbours' heights only as far as it grows up to 5% faster or slower. |
| `deleafed_to_truss` | int | `3` | 1 to 8 | The truss a deleafing pass strips a stem up to, counted up from the lowest still on it: every leaf below it goes. 1 strips the stem below the truss being picked, 2 up to the next truss, a week's growth higher, and 3 leaves the two lowest hanging on bare stem. Stripped up to its third, a stem keeps 18–20 leaves at the defaults, the small ones at its head among them, and three fewer for each truss higher; Dutch farms were measured carrying 19–24 in summer, counting every leaf over 5 cm long. |
| `days_since_deleafing` | float | `0.0` | 0 to 14 | Days since the crop was last deleafed; growers deleaf it weekly. Each stem has grown three leaves a week since, so it carries that many more than the pass left it: its lowest, which the next pass strips. A stem planted since has not been deleafed. |
| `extra_stems` | float | `0.0` | 0 to 0.5 | Share of plants grown on with a third stem, which growers keep to raise the stem density. |
| `turned_to_path` | float | `0.8` | 0 to 1 | Share of trusses turned to face the path. The rest face anywhere, into the crop as often as not, where a harvester has to skip them. |
| `head_diameter` | float | `0.01` | 0.006 to 0.014 | Diameter of a stem's head, in meters: how thick it is 20 cm below its tip, where growers measure it. About 10 mm on a balanced crop; under 8 mm the plant is spent on its fruit, over 12 mm it grows leaf, and beefsteak stems run thicker than cherry. Each stem draws its own about it, and is half as thick again from 0.8 m below its tip down. |
| `flexible_below` | float | `0.0` | 0 to 5.5 | Height above the floor, in meters, below which a truss bends: one whose node is lower is exported as a rod a physics engine bends and a robot can cut, carrying its fruit, rather than as a static mesh. 0 bends none. |
| `skinned` | bool | `False` |  | Whether a truss that bends is drawn as one skin bending with it, a mesh bound to every segment of its rod, rather than as a rigid tube on each segment, which kinks at every joint. A skin bends only where a consumer poses it from the rod's bodies, as `misina_lab.isaaclab` does under Newton; anywhere else it keeps the shape it was built in. |
| `detail` | float | `0.5` | 0.1 to 4 | How finely the stems and the trusses' stems are meshed. At 1 a stem's tube strays from its shape by about a pixel to a camera half a meter away, as a leaf does at its own detail of 1; at 4 by a quarter of that. Down to about 0.5 every node shows its swelling and the swelling round its leaf or truss; lower is cheaper, and at 0.1 a stem is a plain tube with its stubs. |

## Leaves

`LeafParams` in Python, `LeafCfg` in Isaac Lab.

The leaves: a compound leaf at each of a stem's top nodes, as many as the stem keeps.

A leaf is a petiole and a rachis with a terminal leaflet, three or four pairs of laterals and small leaflets between the pairs, every leaflet cut from its veins, folded along its midrib, puckered between its veins and ruffled at its margin. The leaves near the head are still growing, small and upright, their leaflets narrow and still folded shut from the bud; a couple of trusses down they are full grown and open, and the older they are the more they droop. The young leaves are a lighter green, and the oldest yellow. Every leaf draws its own size, angle, droop, leaflet count and yellowing about these. Leaves are visual only, with no collider.

| Parameter | Type | Default | Slider range | Description |
|---|---|---|---|---|
| `length` | float | `0.45` | 0.3 to 0.6 | Length of a full-grown leaf, from the stem to the tip of its terminal leaflet, in meters. Each leaf draws a little more or less. |
| `droop` | float | `90.0` | 0 to 150 | How far an old leaf's rachis bends down from where it leaves the stem to its tip, in degrees. The young leaves near the head droop less. |
| `lobing` | float | `0.85` | 0 to 1 | How deeply the leaflets lobe: 0 leaves every leaflet entire, as on a potato-leaf cultivar, 1 cuts a lateral's basal sinuses nearly to its midrib. The terminal leaflet lobes a little less, and the small leaflets between the pairs hardly at all. |
| `curl` | float | `1.0` | 0 to 2 | How far a grown leaflet's blade bends out of the flat shape it was cut as: puckered up between its veins and ruffled along its margin, both scaled together. 0 leaves it smooth but for its fold and its droop. |
| `fold` | float | `0.35` | -0.5 to 1 | How far a grown leaflet folds up along its midrib, in radians each side: 0 holds it flat across, more folds it into a V. A leaflet at the head is folded far harder whatever this is, its halves still closed up toward each other as they lay in the bud, and opens out as the leaf grows. |
| `asymmetry` | float | `0.5` | 0 to 1 | How lopsided a lateral leaflet is: 0 for halves that are mirror images, 1 for one side much the wider, its base oblique and the blade twisting about its midrib toward the tip. The small leaflets between the pairs lean a little, the terminal leaflet never. |
| `yellowing` | float | `1.0` | 0 to 2 | How far the oldest leaves have yellowed. A leaf starts to fade fourteen nodes below the head and may fade fully by eighteen, how far each one goes a draw of its own: at 1 a leaf that old is anywhere from green to a senescent leaf's pale yellow-green, at 2 half of them have faded all the way, and 0 keeps every leaf green. |
| `variations` | int | `40` | 1 to 128 | How many distinct leaf meshes the scene may hold. A budget, not a count: the leaves are clustered and this is how many representatives the clustering may keep. |
| `detail` | float | `1.0` | 0.1 to 4 | How finely a leaf is meshed, as a multiplier: at 1 a leaflet's margin keeps within 0.6 mm of its curves, about a pixel to a camera half a meter away, its blade is cut into triangles about 12 mm on a side, and a full-grown leaf comes to about 1500 triangles; about 500 at 0.1 and 4500 at 4. Coarser drops the finest teeth first, then rounds off the sinuses, and keeps the lobes; the pucker and the ruffle soften as the triangles across the blade grow, and the folds are on the veins and sharp at any detail. |

## Trusses

`TrussParams` in Python, `TrussCfg` in Isaac Lab.

The trusses: a peduncle off the stem, bending over into a rachis with the fruit hung along it.

The peduncle numbers default to the means measured on medium trusses in a high-wire house, and every truss draws its own about them; the rachis's internode to what those trusses' measured lengths leave for it (est.), and the pedicel to a genebank's measurements. A medium truss tomato is about 6 fruit of 60 mm; a cherry truss about 15 of 25 mm, branched.

| Parameter | Type | Default | Slider range | Description |
|---|---|---|---|---|
| `fruits` | int | `6` | 1 to 24 | Fruit on a truss. Each truss draws a few more or fewer. |
| `fruit_diameter` | float | `0.06` | 0.02 to 0.08 | Diameter of a fruit at the base of a truss, in meters; the fruit toward its tip come out a little smaller. |
| `branched` | bool | `False` |  | Whether the rachis forks in two at the first fruit, as a cherry truss does, rather than hanging as one. |
| `peduncle_length` | float | `0.079` | 0.03 to 0.14 | Length of the peduncle from the stem to the first fruit, in meters. |
| `peduncle_diameter` | float | `0.007` | 0.004 to 0.012 | Diameter of the peduncle, in meters. |
| `peduncle_angle` | float | `70.0` | 5 to 139 | Angle between the stem and the peduncle where it leaves, in degrees: 90 leaves level, less leaves rising. |
| `stiffness` | float | `70.0` | 20 to 1000 | How stiff the peduncle and rachis are, as a Young's modulus in MPa. A truss grows standing out from the stem, and its fruit's weight bends it down against this: one in flower barely bends, and one sags further as its fruit swell. Below green wood's, as a stalk under a load for weeks gives way to it more than it would at once. |
| `rachis_internode` | float | `0.014` | 0.005 to 0.05 | Length of the rachis between two pedicels under grown fruit, in meters: short, a truss's fruit crowd into a bunch, and long, they hang apart down the rachis. |
| `pedicel_length` | float | `0.031` | 0.01 to 0.06 | Length of a pedicel under a grown fruit, from the rachis to the fruit's calyx, in meters. A small-fruited truss's are about as long as a large-fruited one's, so its fruit hang further out for their size. |
| `pedicel_angle` | float | `45.0` | 0 to 50 | How far under the rachis its pedicels leave it, in degrees below its sides: 0 leaves them level out to the sides, and more hangs the fruit further under it. At 45 the two ranks, down alternate flanks, meet at a right angle, as they do on a tomato truss. |
| `jointed` | bool | `True` |  | Whether a pedicel has a knuckle, the joint a fruit breaks off at, as most truss varieties do. |
| `stalks_in_flower` | float | `0.35` | 0.1 to 1 | How long a pedicel, and the internode of rachis leading to it, are when its flower opens, as a share of their length under the grown fruit: about a third on a large-fruited truss, whose stalks grow to about three times their length in flower, and nearer 1 on a small-fruited one, whose barely grow on. |
| `missing_fruit` | float | `0.05` | 0 to 0.3 | Chance that a fruit is missing from its pedicel. |
| `variations` | int | `24` | 1 to 64 | How many distinct truss meshes the scene may hold. A budget, not a count: the trusses are clustered and this is how many representatives the clustering may keep. |

## Fruit

`FruitParams` in Python, `FruitCfg` in Isaac Lab.

The fruit and the flowers they set from: their shape, how ripe the crop is, and what colour ripe is.

Every fruit has a stage: bud and flower before it sets, set, expanding and mature green while it grows, then breaker, turning, pink, light red and red as it colours from its blossom end up. A truss shows at the head in bud, opens its flowers from its base to its tip from about 15 cm below the head, sets them over the week after, and they reach full size about five trusses further down. A stem colours from its lowest truss up and a truss from its base to its tip, so in full production the lowest `ripe_trusses` trusses on a stem are colouring and the ones above them still green; a young crop's trusses are all green until the first are old enough. A truss is due once its tip fruit is turning. Every stem runs a little ahead of or behind the rest.

A fruit has a lobe per locule, the chambers its seeds grow in, and a groove over each wall between two: deepest round its shoulders, where it pleats out from the calyx, and fading toward its blossom end. Its outline is rounded either side of where it is widest, blocky or round at its shoulders and at its blossom end, its stem scar sunk in a dip and its blossom end pointed, flat or dented. Every fruit draws its own lobe count, how wide and how full each lobe is and how deep each groove, its lean, how far its widest point, its blossom tip and the blockiness of each half stray from the params', and its five or six sepals and how each curls, within the ranges the research notes give; its flower has a petal between each two sepals.

| Parameter | Type | Default | Slider range | Description |
|---|---|---|---|---|
| `ripe_trusses` | float | `1.0` | 0 to 3 | Trusses per stem that have coloured in full production, on average, counted up from the lowest: a fraction is the next one part way. |
| `spread` | float | `0.2` | 0 to 1 | How far a truss's tip fruit lags behind its base fruit, as a share of the ripening from breaker to red: 0 colours a truss all at once. |
| `color` | str | `"red"` | `red`, `orange`, `yellow`, `brown` | The colour a ripe fruit turns: `red`, `orange`, `yellow` or `brown`. |
| `aspect` | float | `0.9` | 0.5 to 1.8 | Height of a fruit over its width: under 1 is oblate, as round truss tomatoes and beefsteaks are, and over 1 elongated, as plum tomatoes are. Each fruit draws a little either side. |
| `widest` | float | `0.5` | 0.35 to 0.65 | Where a fruit is widest, as a share of its height from its stem end: over 0.5 it is fuller toward its blossom end, as a plum tomato is. Each fruit strays a little either side. |
| `stem_dip` | float | `0.1` | 0 to 0.25 | How deep the dip a fruit's stem scar sits in is, as a fraction of its radius: about 0.1 on a round truss tomato, next to none on a cherry or a plum tomato, and about 0.2 on a beefsteak. |
| `stem_blockiness` | float | `0.3` | 0 to 1 | How blocky a fruit's stem half is: 0 is an ellipse, and 1 has broad, flat shoulders. Each fruit strays a little either side. |
| `blossom_blockiness` | float | `0.2` | 0 to 1 | How blocky a fruit's blossom half is: 0 is an ellipse, and 1 has a broad, flat blossom end. Each fruit strays a little either side. |
| `blossom_tip` | float | `0.0` | -0.15 to 0.2 | How far a fruit's blossom end is drawn out to a point, as a plum tomato's is, as a share of the height of its blossom half. Under 0 it is pushed in: flat at about -0.075, and dented past that, as a beefsteak's is. Each fruit strays a little either side. |
| `locules` | float | `4.0` | 2 to 12 | How many locules a fruit has, and so lobes: 2 in a cherry or a plum tomato, 3 to 6 in a round or a truss tomato, and more in a beefsteak. Each fruit draws up to one either side. |
| `ribbing` | float | `0.08` | 0 to 0.3 | Depth of the grooves between a fruit's lobes round its shoulders, where they are deepest, as a fraction of its radius: 0 is smooth, and a beefsteak's are about 0.2. |
| `rib_reach` | float | `0.35` | 0.05 to 0.5 | How far down a fruit its grooves reach: their depth at its girth, as a share of their depth round its shoulders. Below that they fade out before the blossom end. |
| `irregularity` | float | `1.0` | 0 to 2 | How irregular a fruit is: how far its lobes differ in width and in fullness and its grooves in depth, how far its blossom end leans, and how far its widest point, its blossom tip and its blockiness stray from the params'. 0 is a regular fruit; at 1 about one fruit in five leans visibly, and 2 is lopsided. |
| `petal_length` | float | `0.011` | 0.004 to 0.02 | Length of a flower's petals, in meters: an open tomato flower is under 25 mm across, and a cherry tomato's smaller. |
| `petal_reflex` | float | `125.0` | 20 to 170 | How far an open flower's petals sweep back, in degrees off pointing straight ahead: 90 opens it flat, and a tomato's sweep back past that toward its pedicel. |
| `variations` | int | `32` | 1 to 64 | How many distinct fruit and flower meshes the scene may hold, by size, shape and stage. A budget, not a count: the fruit are clustered and this is how many representatives the clustering may keep, though a crop in flower keeps one for its flowers and one for its fruit at least. A fruit is drawn at its representative's size, so on a small budget a truss's fruit, packed to touch at their own sizes, are drawn into each other. |
| `detail` | float | `3.0` | 0.1 to 8 | How finely a fruit's skin is meshed, as a multiplier: at 1 its rings and meridians keep within 0.6 mm of its shape, about a pixel to a camera half a meter away, as a leaf's margin does at its own detail of 1, and its faces within twice that; at 3 its rings and meridians keep within 0.2 mm and its faces within 0.4 mm, which leaves its outline round from close up. A full-grown 60 mm fruit comes to about 1250 triangles with its calyx at 3, about 400 at 1 and 3700 at 8; each fruit is meshed for its own size, so a young one costs less. |

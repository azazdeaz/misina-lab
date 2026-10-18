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

Scene-wide parameters: the seed.

| Parameter | Type | Default | Slider range | Description |
|---|---|---|---|---|
| `seed` | int | `0` | 0 to 64 | The one seed the whole scene is generated from. Each element salts it with a constant of its own, so a new seed is a different greenhouse and an edit to one element never re-rolls another. |

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
| `plant_spacing` | float | `0.36` | 0.3 to 0.5 | Distance between plants along a gutter, in meters, each in a propagation block on a slab. A slab carries the whole number of plants closest to this, so the real spacing comes out close to it rather than equal to it. |

## Pipe rail

`RailParams` in Python, `RailCfg` in Isaac Lab.

The pipe rail in every path: two heating pipes a trolley rides on.

The pipes rest on a support every meter, and `unevenness` moves the supports off level, each by a draw of its own: the pair tilts across and each pipe pitches along its length.

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

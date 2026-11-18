# Changelog

Every release of misina-lab, vinery-lab and greenhouse-lab, newest first. The
three release together under one version, so a version is one section, and
each entry starts with the package it is about. The layout follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/). Until 1.0.0, a minor
version may break what the one before it built.

Two kinds of change break a project that worked on the release before:

- **Scene document format.** Each version names the format of the scene
  document it writes: `generate_scene_json()`, the viewer's `S` key, the input
  of `python -m misina_lab.usd`. The builder refuses a document of any other
  format, so one saved by an earlier release has to be generated again. Scenes
  the Isaac Lab spawner cached regenerate on their own.
- **cfg** marks an entry that renames or removes a field of a generated Isaac
  Lab cfg class, `VineyardCfg`, `GreenhouseCfg` or one of their fragments, or
  changes what it means. The `*Params` classes change with them. A cfg written
  for an earlier release, or copied from its viewer, may need editing.

## [Unreleased]

The first release, 0.1.0. Scene document format 12.

### Added

- **misina-lab**: a framework for procedural scene generators. A generator
  declares its parameters once, as Rust structs, and builds its scene in Bevy
  0.20; the viewer's panel, the Python params classes, the Isaac Lab cfg
  classes, their type stubs and a parameters page are all generated from those
  declarations. It releases as a crate on crates.io and a Python package on
  PyPI.
- **misina-lab**: the viewer. Collapsible parameter sections with a tooltip on
  every parameter, sliders that take a typed number, a rebuild of only what an
  edit touches, a wireframe toggle, an overlay of every collider, and a footer
  with what the scene weighs. **Copy Isaac Lab cfg** puts the current settings
  on the clipboard as a cfg construction, only the fields that moved. A
  storyboard plays camera moves and panel edits and saves a still or records a
  video, with no window or display; `P` copies the view as a storyboard's
  `camera`.
- **misina-lab**: what a generator builds its organs from. Strands, tubes
  swept along a polyline that swell at their bulges and collars; outlines
  filled into sheets that fold on their veins; the pieces of a lobed, toothed
  margin; points scattered over a band of ground; `bending::sag`, where a rod
  settles under the loads on it; `packing::pack`, a cluster of fruit on stalks
  moved apart until none lies in another; and `palette::tissue`, the colour a
  tissue's pigments leave. `quantize` reduces a population of configs to a
  budget of meshes, and every element draws from a stream of the one seed.
- **misina-lab**: the USD builder, `misina_lab.usd`, and
  `python -m misina_lab.usd scene.json scene.usd` to build a saved document.
  Scenes are Z-up, in meters. A material per part, coloured per point where
  the part asks for it; static colliders; the ground as a height field, read
  back by `misina_lab.usd.Ground` to place a robot on it. Custom prim
  attributes carry ground truth, under the generator's namespace.
- **misina-lab**: the Isaac Lab spawner, `GeneratedSceneCfg`, on Isaac Lab
  3.0.0 and the Newton 1.6.1 it ships with. It generates a scene on first use,
  caches it as a USD file, and spawns it as an ordinary `FileCfg` asset, once
  however many envs there are.
- **misina-lab**: flexible rods on Newton: one capsule body per segment,
  tapering as the organ does and carrying the loads hung on it, stepped by the
  VBD solver beside MJWarp for the robot, and spawned static on a backend that
  cannot step them. Whatever is authored under a segment rides its body, and a
  skin bends with the rod where a tube per segment kinks at every joint.
  `Shears` cuts a rod anywhere along it while the simulation runs, and holds a
  cut piece as a hand does.
- **misina-lab**: Isaac Sim tools. `ColliderView` draws every collider of the
  running Newton model in Kit's viewport, `shots` renders a still of a scene
  from a shot file, at a moment picked by what the demo is doing, and `film`
  films a video of the running simulation.
- **vinery-lab**: a parametric vineyard. Rows on rolling terrain with bumps a
  wheel feels, posts and trellis wires, vines trained up to the fruiting wire
  and along it, leaves built from their veins and bent along them, a cover crop
  in the alleys and weeds under the vines. Every plant, shoot and leaf is its
  own prim, and the same seed gives the same vineyard. Level of detail and mesh
  variance trade fidelity for frame rate. Spawned with `VineyardCfg`.
- **vinery-lab**: stray shoots and weed stems as flexible rods a robot pushes
  aside and cuts, and a dormant mode: bare canes with a bud at every node,
  every one of them cuttable.
- **greenhouse-lab**: the rows of a high-wire tomato greenhouse. Floor, heating
  pipe loops, gutters with slabs and blocks, crop wires, and the crop on them,
  grown for `SceneCfg.week` weeks since planting: stems with their twine and
  hooks, lowered along the gutter as they grow; compound leaves that yellow
  with age and are stripped as growers strip them; and trusses that flower,
  set fruit and sag under it. Every fruit is its own prim, with a calyx, a
  shape of its own and a ripeness coloured from its pigments, and every truss
  has a `Cut` and a `Bend` frame for a gripper. Trusses low on the stem bend
  and are cut as rods, and fruit and stems collide. Spawned with
  `GreenhouseCfg`.
- Web playgrounds of both viewers, and the docs site, at
  [azazdeaz.github.io/misina-lab](https://azazdeaz.github.io/misina-lab/).
- Examples, each its own uv project: a quadruped walking the vineyard's
  alleys, a straddling robot driving its rows, an arm pruning a dormant row, a
  pipe-rail robot harvesting the greenhouse, and RGB-D sequences filmed as
  BUTom-ST21's were.

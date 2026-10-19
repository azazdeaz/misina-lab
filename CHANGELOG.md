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

### Added

- **misina-lab**: `ColliderView` draws every collider of the running Newton
  model in Kit's viewport, translucent, and follows the ones moved or resized
  while it runs. Kit draws only a PhysX scene's colliders.
- **greenhouse-lab**: leaves. A compound tomato leaf at every node of a stem
  from the head down, `leaves_kept` of them: a terminal leaflet, three or four
  pairs of laterals and small leaflets between them, each leaflet outlined
  from its veins. Leaves are small and upright at the head and droop further
  down. The stem below them is stripped, with a stub at each node, so the
  lowest two trusses always hang on bare stem. Leaves are visual only, with
  no collider. New: `StemCfg.leaves_kept` and `LeafCfg`.

### Changed

- **misina-lab**: `Shears.colliders(body)` replaces `Shears.collider(body)`
  and lists all of a body's collision shapes, so a part built of several, a
  blade in slices, can have each moved on its own with `Shears.place`.

### Fixed

- **misina-lab**: The ground's collider under Newton is the ground mesh again
  on a field whose two axes step differently. The height field Newton builds
  from it was sampled at the finer axis's spacing, which put its rows between
  the mesh's along the other axis: millimetres off the drawn ground on gentle
  ground, more on rough.

## [0.1.0] - Unreleased

Scene document format 8. The first release.

### Added

- **misina-lab**: a framework for procedural scene generators. A generator
  declares its parameters once, as Rust structs, and builds its scene in Bevy
  0.20; the viewer's panel, the Python params classes, the Isaac Lab cfg
  classes, their type stubs and a parameters page are all generated from those
  declarations.
- **misina-lab**: the viewer. Collapsible parameter sections with a tooltip on
  every parameter, sliders that take a typed number, a rebuild of only what an
  edit touches, a wireframe toggle, an overlay of every collider, a footer with
  what the scene weighs, and video recording of a run. **Copy Isaac Lab cfg**
  puts the current settings on the clipboard as a cfg construction, only the
  fields that moved.
- **misina-lab**: the USD builder, `misina_lab.usd`, and
  `python -m misina_lab.usd scene.json scene.usd` to build a saved document.
  Scenes are Z-up, in meters. A material per part, coloured per point where
  the part asks for it; static colliders; the ground as a height field, read
  back by `misina_lab.usd.Ground` to place a robot on it. Custom prim
  attributes carry ground truth, under the generator's namespace.
- **misina-lab**: the Isaac Lab spawner, `GeneratedSceneCfg`. It generates a
  scene on first use, caches it as a USD file, and spawns it as an ordinary
  `FileCfg` asset, once however many envs there are.
- **misina-lab**: flexible rods on Newton: one capsule body per segment,
  stepped by the VBD solver beside MJWarp for the robot, and spawned static on
  a backend that cannot step them. `misina_lab.isaaclab.cutting` cuts a rod
  anywhere along it while the simulation runs.
- **vinery-lab**: a parametric vineyard. Rows on rolling terrain with bumps a
  wheel feels, posts and trellis wires, vines trained up to the fruiting wire
  and along it, leaves as detailed meshes, a cover crop in the alleys and weeds
  under the vines. Every plant, shoot and leaf is its own prim, and the same
  seed gives the same vineyard. Level of detail and mesh variance trade
  fidelity for frame rate.
- **vinery-lab**: stray shoots as flexible rods a robot pushes aside and cuts,
  and a dormant mode: bare canes with a bud at every node, every one of them
  cuttable.
- **vinery-lab**: `VineyardCfg`, and the viewer as a
  [web playground](https://azazdeaz.github.io/vinery-lab/).
- **greenhouse-lab**: the rows of a high-wire tomato greenhouse. Floor, heating
  pipe rails, gutters with slabs and blocks, crop wires, and the crop on them:
  stems with their twine and hooks, trusses with a `Cut` and a `Bend` frame
  for a gripper, and every fruit its own prim, with a calyx, a growth stage and
  a ripeness. Spawned with `GreenhouseCfg`.

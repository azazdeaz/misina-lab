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

Scene document format 10.

### Added

- **misina-lab**: `geometry::outline::creased_mesh` fills an outline with
  lines across it kept as edges, split as finely as the fill, so a bend
  can fold the sheet sharply on them; the parts of a line outside the ring
  are left out.
- **misina-lab**: `geometry::shapes` gains `cubic`, `serrate`, `simplify`
  and `untangle`, the pieces a lobed, toothed margin is built from, so a
  generator strings its own ring from an organ's veins.
- **misina-lab**: `misina_lab.isaaclab.film` films a video of the running
  simulation in Isaac Sim: `Rig` eases a camera from shot to shot and cuts
  the long moves, and `Film` pipes the frames to ffmpeg and writes their
  captions as SubRip. Every demo's video is filmed through it.
- **greenhouse-lab**: Every fruit has a collider: a sphere round its body, a
  capsule on an elongated one, as `Fruit_nn/Collision`; the mesh moved under
  the fruit's prim as `Fruit_nn/Skin`. On a bent truss it rides the
  segment's body.
- **misina-lab**: `rods.carry_riders` seats a collider authored under a rod
  segment's prim on that segment's body, which the importer had left static,
  and lets it through its own rod; `tune_rods` calls it. `Shears` finds a
  body's rod capsule by its label, so a body may carry other shapes, and
  parks the colliders of whatever a cut hides.
- **greenhouse-lab**: Trusses bend. One whose node on its stem is below
  `StemCfg.flexible_below` is a rod a solver bends and `Shears` cuts, its
  peduncle and first rachis the rod and everything else riding the segment
  that carries it: the pedicels, the fruit, and `Bend`. The fruit weigh on
  the rod, so a truss hangs with its fruit's weight and falls with it once
  cut. `Cut` sits within a centimetre of the rod's first joint, so a flush
  cut there drops the whole truss. `GreenhouseCfg.PLANT` groups a plant's
  rods for collisions, and `GreenhouseCfg.without_rods()` spawns them rigid
  under a backend that cannot step them. The default, 0, bends none.
- **misina-lab**: `spawn_rod` authors a rod's curve and a prim per segment
  carrying its tube, for whatever else rides the rod; `segment_frames`,
  `segment_tubes`, `segment_of`, `rebase`, `segment_name` and `STEM` are
  what a generator places riders with. They were vinery-lab's.
- **misina-lab**: `ColliderView` draws every collider of the running Newton
  model in Kit's viewport, translucent, and follows the ones moved or resized
  while it runs. Kit draws only a PhysX scene's colliders.
- **greenhouse-lab**: leaves. A compound tomato leaf at every node of a stem
  from the head down, `leaves_kept` of them: a terminal leaflet, three or four
  pairs of laterals and small leaflets between them, each leaflet outlined
  from its veins. Leaves are small and upright at the head and droop further
  down. The stem below them is stripped, with a stub at each node, so the
  two trusses nearest their picking always hang on bare stem. Leaves are
  visual only, with no collider. New: `StemCfg.leaves_kept` and `LeafCfg`.
- **greenhouse-lab**: `LeafCfg.detail` sets how finely a leaf is meshed, as
  a multiplier: a full-grown leaf is about 1500 triangles at the default 1,
  500 at 0.1 and 4500 at 4. Coarser drops the leaflets' finest teeth first.
- **greenhouse-lab**: the crop's age. `SceneCfg.week`, the weeks since
  planting, grows every stem forward from a 0.4 m transplant at a truss a
  week. A young crop stands on its twines below the wire with green fruit
  only and keeps its lowest leaves; its heads reach the wire and its first
  trusses ripen about week 9 at the defaults; from then on its stems are
  lowered along the gutter as they grow. Which trusses hang, how ripe they
  are and how far their fruit have grown follow from each truss's age, and
  an extra stem is taken on six weeks after planting.
- The docs are a site,
  [azazdeaz.github.io/misina-lab](https://azazdeaz.github.io/misina-lab/):
  the framework's pages, both generators' parameters and the examples,
  searchable.
- **misina-lab**: The viewer's `P` key logs the camera as a storyboard's
  `camera`, and copies it: a view found by hand, ready to paste into a
  storyboard.
- **misina-lab**: A storyboard's camera takes `fov`, the vertical field of
  view. `{PACKAGE}_SHOT` saves the window to an image once a storyboard is
  over, so a storyboard with no steps is a still of its opening shot.
- **misina-lab**: `misina_lab.isaaclab.shots` renders a still of a generated
  scene in Isaac Sim from a shot file, its camera a storyboard's, or opens
  Kit's viewport on the scene, where `P` prints the camera in the same terms.
- **greenhouse-lab**: A web playground, the viewer in the browser, at
  [azazdeaz.github.io/misina-lab/playground/greenhouse](https://azazdeaz.github.io/misina-lab/playground/greenhouse/).

### Changed

- **vinery-lab**: Leaves are built from their veins rather than traced: five
  main veins from the petiole's junction, a lobe at the end of each, sinuses
  between them and teeth the whole way round, cut to a margin of its own for
  every blade mesh the budget keeps, where five drawn outlines were curled a
  few ways each. A leaf's shape now follows its growth: the young leaves near
  a shoot's tip are narrower and coarser toothed, with little blade yet at
  their base, folded up along their veins and a lighter yellow-green, where
  they were a grown blade scaled down. A leaf now takes about ten nodes
  below the tip to reach full size, 0.7 m at the default spacing where it
  took 0.25 m, so the leaves of a default shoot are all still filling out
  and the canopy's blades run a little smaller. New: `LeafCfg.lobing`, how
  deeply a grown blade is cut, which each leaf draws about;
  `LeafCfg.detail` now meshes the margin as well as the middle, so coarser
  drops the teeth first.
- **vinery-lab** (**cfg**): A leaf bends along its veins, which its mesh
  keeps as edges so it creases on them. The young leaves near a shoot's tip
  are folded on the midrib as they lay in the bud, opening out by the time
  they are half grown, where they drooped across it. A grown blade hangs off
  a petiole held up and out, bent down off it at the junction by new
  `LeafCfg.blade_angle`, and arches down off it, where the whole leaf
  drooped from the shoot; `ShootCfg.leaf_droop` is still the blade's pitch.
  The blade stands up between its main veins. And no leaf is symmetric: one
  half is the larger and the midrib curves away from it, the next leaf up
  the shoot leaning the other way about three times in four, as far as new
  `LeafCfg.asymmetry` lets it. `LeafCfg.curl` now arches, pleats and
  ruffles a grown blade and no longer troughs it across its midrib; new
  `LeafCfg.fold` folds it into a V. A full-grown leaf's area is new
  `LeafCfg.area`, 150 cm² as before. Every leaf prim carries its blade's
  area as placed, in m², and its maturity, as `vinery_lab:area` and
  `vinery_lab:maturity`. A blade is about a sixth more triangles at the same
  `detail`.
- **misina-lab**: `Outline::from_svg` is gone, and with it the `usvg`
  dependency: an outline is a ring built in code, which `geometry::outline`
  fills. A generator that traced its shapes in SVG builds the ring itself.
- **greenhouse-lab**: Trusses are modelled closer to life. The rachis is a
  chain of internodes that zigzags, kinking away from each pedicel, and
  hangs nearer vertical toward its tip; the peduncle bends over a longer
  arc. A pedicel is sized to its fruit, arches up and over and comes down
  onto the fruit vertically, with the knuckle just under its tip, where the
  fruit hangs a little off vertical. The sepals are half the fruit's
  diameter long, lift off the shoulders to spread, and are folded and
  twisted. Every representative truss draws its own kinks.
- **misina-lab**: A rod carries loads: `Cable.loads` is the mass hanging from
  each point, which the builder adds to the rod's own, and `cable()` takes
  it after the widths, empty for a rod that carries nothing.
- **misina-lab**: A rod tapers as its organ is drawn. The builder authors a
  cable's `widths` again as its `physics:thicknesses`, so each segment's
  capsule, mass and stiffness follow the taper, and the scene document's
  `Cable.thickness`, the one diameter a rod used to be simulated at, is
  gone. At its material's own stiffness a tapered
  1.4 m rod sags about 9 cm and settles in about a second, as the uniform
  one did stiffened threefold, so `tune_rods`, vinery-lab's `tune_shoots`
  and `rods` lose `stiffen` and `ROD_STIFFEN`.
- **misina-lab**: The Isaac Lab side runs on Isaac Lab 3.0.0 and the Newton
  1.6.1 it ships with. Under Newton, Isaac Lab now puts into the physics only
  the assets a scene declares, so declare a generated scene in a scene config,
  `AssetBaseCfg(prim_path=..., spawn=...)`: one spawned straight onto the
  stage with `cfg.func(...)` is drawn, but nothing collides with it. The
  viewer's **Copy Isaac Lab cfg** snippet says so.
- The examples run on the Isaac Lab 3.0.0 release and Isaac Sim 6.2, and
  build their scene as an `InteractiveScene`, which puts the robot in the
  scene's one environment, at `/World/envs/env_0/Robot`.
- **greenhouse-lab** (**cfg**): `StemCfg.lowered` is gone: how much stem
  lies along the gutter follows from `SceneCfg.week`.
  `StemCfg.harvest_height` and `StemCfg.trusses` now describe the crop in
  full production, and set how much a stem grows in a week.
- **misina-lab**: `Shears.colliders(body)` replaces `Shears.collider(body)`
  and lists all of a body's collision shapes, so a part built of several, a
  blade in slices, can have each moved on its own with `Shears.place`.
- **misina-lab**: The viewer's camera is `bevy_panorbit_camera` 0.36 from
  crates.io, so a generator in its own repository no longer copies a
  `[patch.crates-io]` section; delete it.
- **misina-lab**: A web build of a generator gets Bevy's WebGPU backend and
  `getrandom`'s `wasm_js` from misina-lab, so a generator in its own
  repository no longer lists them under its `wasm32` dependencies; delete
  them.
- **misina-lab**: A storyboard played without `{PACKAGE}_RECORD` hands the
  window back to the pointer once it is over, instead of quitting.
- The repository is now
  [azazdeaz/misina-lab](https://github.com/azazdeaz/misina-lab), and the web
  playground moved with it to
  [azazdeaz.github.io/misina-lab/playground/vinery](https://azazdeaz.github.io/misina-lab/playground/vinery/).
  The old repository URL redirects; the old playground URL does not.
- The README is misina-lab's front page: the framework's features first,
  then a section for each reference generator and its examples. The docs
  site lists the framework first too. The
  vineyard's page is now
  [crates/vinery-lab/README.md](crates/vinery-lab/README.md), which is also
  vinery-lab's description on PyPI.

### Fixed

- **misina-lab**: A cut piece rests on the stub it was cut from. The two
  segments either side of a cut were the two a joint had tied, and Newton
  never pairs those, so the piece fell through the stub's top segment and
  tipped off the one below. `tune_rods` now reserves a contact-pair row per
  rod joint as the model is built, and a cut writes the joint's pair into
  it, with each capsule pulled back a radius at the cut face so the two
  touch there instead of starting a thickness deep in each other. A scene
  with rods needs two static colliders, anywhere, to park the rows on.
- **misina-lab**: `configs_changed` also fires when a layer's configs are
  removed, so an edit that leaves a layer nothing to build clears its parts
  from the library. Such an edit used to leave the layer's old parts in the
  scene document, unreferenced.
- **misina-lab**: The viewer draws everything a rebuild spawns. When an edit
  rebuilt two layers in one frame, Bevy 0.20 could skip propagating the
  transforms of parts the second layer spawned into entity indices the first
  had just freed, and those parts drew at the origin: setting the
  greenhouse's week to 0 lost a whole row. A parent whose children changed
  is now marked moved, which Bevy always propagates.
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
  [web playground](https://azazdeaz.github.io/misina-lab/).
- **greenhouse-lab**: the rows of a high-wire tomato greenhouse. Floor, heating
  pipe rails, gutters with slabs and blocks, crop wires, and the crop on them:
  stems with their twine and hooks, trusses with a `Cut` and a `Bend` frame
  for a gripper, and every fruit its own prim, with a calyx, a growth stage and
  a ripeness. Spawned with `GreenhouseCfg`.

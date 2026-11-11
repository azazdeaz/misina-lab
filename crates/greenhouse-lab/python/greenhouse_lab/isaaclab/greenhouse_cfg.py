"""Spawner configuration for a generated greenhouse.

One `@configclass` fragment per element, mirroring the `*Params` pyclasses in
`greenhouse_lab._core` field for field. The fragments and `GreenhouseCfg`'s
fields are generated from the Rust params structs -- see
`crates/misina-lab/docs/editing-parameters.md`; the rest of `GreenhouseCfg` is
hand-written, over the core's `GeneratedSceneCfg`.
"""

from __future__ import annotations

from typing import ClassVar, Self

from isaaclab.utils.configclass import configclass

import greenhouse_lab
from misina_lab.isaaclab.spawn import GeneratedSceneCfg


# >>> generated: fragments
@configclass
class SceneCfg:
    """Scene-wide parameters: the seed, and how far into the season the crop is."""

    seed: int = 0
    """The one seed the whole scene is generated from. Each element salts it with a
    constant of its own, so a new seed is a different greenhouse and an edit to one
    element never re-rolls another.
    """
    week: float = 16.0
    """Weeks since the crop was planted. A stem grows about a truss a week up its twine,
    reaches the wire after about nine weeks at the defaults, when its first trusses
    ripen, and is lowered along the gutter from then on.
    """


@configclass
class LayoutCfg:
    """Where the rows go.

    Rows run along X from the row head, where the concrete main path crosses them,
    and are numbered across Y. Every gutter has a path on both sides, so there is one
    path more than there are rows.
    """

    rows: int = 6
    """How many rows, one gutter each."""
    pitch: float = 1.6
    """Distance between neighbouring gutters, in meters: the row pitch."""
    row_length: float = 20.0
    """Length of a row from the main path to its far end, in meters. Farm rows run 90 to
    115 m; a shorter one simulates faster.
    """
    main_path: float = 3.0
    """Width of the concrete main path across the row heads, in meters: where a robot
    changes rows.
    """
    plant_spacing: float = 0.36
    """Distance between plants along a gutter, in meters, each in a propagation block on
    a slab. A slab is as long as the whole number of plants that brings it nearest
    1.2 m, so the plants stand this far apart the whole row along.
    """


@configclass
class RailCfg:
    """The pipe rail in every path: two heating pipes a trolley rides on.

    The pipes rest on a support every meter, and `unevenness` moves the supports off
    level, each by a draw of its own: the pair tilts across and each pipe pitches
    along its length. Each pipe turns down into the floor at the row head, and a
    return bend joins the two at the far end.
    """

    gauge: float = 0.6
    """Distance between the two pipes, centre to centre, in meters. Growers lay 425,
    500, 550, 600 and 800 mm.
    """
    pipe_diameter: float = 0.051
    """Outer diameter of a pipe, in meters: the slider stops at the 45 and 51 mm heating
    pipes.
    """
    height: float = 0.15
    """Height of the pipes' axis above the floor, in meters. An estimate, to be set from
    a measured site.
    """
    unevenness: float = 0.5
    """How far the supports stand off level. 0 is a perfect rail; at 1 the pair tilts
    across by up to the 2° the rail spec allows; 2 is twice out of spec.
    """


@configclass
class GutterCfg:
    """The gutter each row's crop stands in, and what holds it up.

    Stone wool slabs lie end to end in the gutter, and a propagation block sits on a
    slab at every plant; the rows' plant spacing places them.
    """

    height: float = 0.5
    """Height of the gutter's floor above the ground, in meters: where the slabs rest."""
    mount: str = "hanging"
    """What holds the gutter up: `hanging` from a rod every 2.5 m, which leaves the
    floor under it clear, or `standing` on a stand every 2.5 m.
    """


@configclass
class WireCfg:
    """The two crop wires over every gutter, which the stems are trained up to in a V."""

    height: float = 4.5
    """Height of the crop wires above the floor, in meters: how tall the crop stands."""
    spread: float = 0.35
    """How far each of a row's two wires runs to the side of its gutter, in meters. An
    estimate, to be set from a measured site.
    """


@configclass
class StemCfg:
    """The crop's stems: two to a plant, trained alternately to the left and the right
    wire, and lowered as they grow.

    Each stem grows a truss a week from planting, up its twine to a hook on the wire.
    Once its head is a hook's length below the wire it is lowered, and all the stem
    the wire is too low for lies along the gutter. In full production the stem
    between `harvest_height` and the head carries `trusses` trusses with three leaves
    to each, so how much a stem grows in a week follows from the wire height rather
    than being set. The crop is deleafed about weekly, so the stem below its leaves
    is bare.
    """

    harvest_height: float = 1.2
    """Height above the floor of the lowest truss still on a stem once the crop is in
    full production, in meters: where a harvesting arm reaches to. A stem's lowest
    hangs up to a week's growth above it, the higher the sooner after its last
    picking, and a stem that grows faster than the rest hangs it up to 5% of its live
    height lower. The stem below it has been picked.
    """
    trusses: int = 9
    """Trusses on a stem in full production, from the lowest still on it up to the head.
    At a truss a week, also the weeks from a truss showing at the head to its
    picking.
    """
    development_spread: float = 0.2
    """How far stems drift apart in their development, in weeks: each truss comes a
    little sooner or later than the last, so a stem `week` weeks old runs up to this
    times √`week` ahead of the crop or behind it, its trusses hanging that much of
    a week's growth lower or higher and ripening that much sooner or later. At 0.2,
    up to 0.8 of a week at week 16; at 0 every stem keeps in step with the crop, and
    its trusses stray from its neighbours' heights only as far as it grows up to 5%
    faster or slower.
    """
    deleafed_to_truss: int = 3
    """The truss a deleafing pass strips a stem up to, counted up from the lowest still
    on it: every leaf below it goes. 1 strips the stem below the truss being picked,
    2 up to the next truss, a week's growth higher, and 3 leaves the two lowest
    hanging on bare stem. Stripped up to its third, a stem keeps 18–20 leaves at
    the defaults, the small ones at its head among them, and three fewer for each
    truss higher; Dutch farms were measured carrying 19–24 in summer, counting
    every leaf over 5 cm long.
    """
    days_since_deleafing: float = 0.0
    """Days since the crop was last deleafed; growers deleaf it weekly. Each stem has
    grown three leaves a week since, so it carries that many more than the pass left
    it: its lowest, which the next pass strips. A stem planted since has not been
    deleafed.
    """
    extra_stems: float = 0.0
    """Share of plants grown on with a third stem, which growers keep to raise the stem
    density.
    """
    turned_to_path: float = 0.8
    """Share of trusses turned to face the path. The rest face anywhere, into the crop
    as often as not, where a harvester has to skip them.
    """
    head_diameter: float = 0.01
    """Diameter of a stem's head, in meters: how thick it is 20 cm below its tip, where
    growers measure it. About 10 mm on a balanced crop; under 8 mm the plant is spent
    on its fruit, over 12 mm it grows leaf, and beefsteak stems run thicker than
    cherry. Each stem draws its own about it, and is half as thick again from 0.8 m
    below its tip down.
    """
    flexible_below: float = 0.0
    """Height above the floor, in meters, below which a truss bends: one whose node is
    lower is exported as a rod a physics engine bends and a robot can cut, carrying
    its fruit, rather than as a static mesh. 0 bends none.
    """
    skinned: bool = False
    """Whether a truss that bends is drawn as one skin bending with it, a mesh bound to
    every segment of its rod, rather than as a rigid tube on each segment, which
    kinks at every joint. A skin bends only where a consumer poses it from the rod's
    bodies, as `misina_lab.isaaclab` does under Newton; anywhere else it keeps the
    shape it was built in.
    """
    detail: float = 0.5
    """How finely the stems and the trusses' stems are meshed. At 1 a stem's tube strays
    from its shape by about a pixel to a camera half a meter away, as a leaf does at
    its own detail of 1; at 4 by a quarter of that. Down to about 0.5 every node
    shows its swelling and the swelling round its leaf or truss; lower is cheaper,
    and at 0.1 a stem is a plain tube with its stubs.
    """


@configclass
class LeafCfg:
    """The leaves: a compound leaf at each of a stem's top nodes, as many as the stem
    keeps.

    A leaf is a petiole and a rachis with a terminal leaflet, three or four pairs of
    laterals and small leaflets between the pairs, every leaflet cut from its veins,
    folded along its midrib, puckered between its veins and ruffled at its margin.
    The leaves near the head are still growing, small and upright, their leaflets
    narrow and still folded shut from the bud; a couple of trusses down they are full
    grown and open, and the older they are the more they droop. The young leaves are
    a lighter green, and the oldest yellow. Every leaf draws its own size, angle,
    droop, leaflet count and yellowing about these. Leaves are visual only, with no
    collider.
    """

    length: float = 0.45
    """Length of a full-grown leaf, from the stem to the tip of its terminal leaflet, in
    meters. Each leaf draws a little more or less.
    """
    droop: float = 90.0
    """How far an old leaf's rachis bends down from where it leaves the stem to its tip,
    in degrees. The young leaves near the head droop less.
    """
    lobing: float = 0.85
    """How deeply the leaflets lobe: 0 leaves every leaflet entire, as on a potato-leaf
    cultivar, 1 cuts a lateral's basal sinuses nearly to its midrib. The terminal
    leaflet lobes a little less, and the small leaflets between the pairs hardly at
    all.
    """
    curl: float = 1.0
    """How far a grown leaflet's blade bends out of the flat shape it was cut as:
    puckered up between its veins and ruffled along its margin, both scaled together.
    0 leaves it smooth but for its fold and its droop.
    """
    fold: float = 0.35
    """How far a grown leaflet folds up along its midrib, in radians each side: 0 holds
    it flat across, more folds it into a V. A leaflet at the head is folded far
    harder whatever this is, its halves still closed up toward each other as they lay
    in the bud, and opens out as the leaf grows.
    """
    asymmetry: float = 0.5
    """How lopsided a lateral leaflet is: 0 for halves that are mirror images, 1 for one
    side much the wider, its base oblique and the blade twisting about its midrib
    toward the tip. The small leaflets between the pairs lean a little, the terminal
    leaflet never.
    """
    yellowing: float = 1.0
    """How far the oldest leaves have yellowed. A leaf starts to fade fourteen nodes
    below the head and may fade fully by eighteen, how far each one goes a draw of
    its own: at 1 a leaf that old is anywhere from green to a senescent leaf's pale
    yellow-green, at 2 half of them have faded all the way, and 0 keeps every leaf
    green.
    """
    variations: int = 40
    """How many distinct leaf meshes the scene may hold. A budget, not a count: the
    leaves are clustered and this is how many representatives the clustering may
    keep.
    """
    detail: float = 1.0
    """How finely a leaf is meshed, as a multiplier: at 1 a leaflet's margin keeps
    within 0.6 mm of its curves, about a pixel to a camera half a meter away, its
    blade is cut into triangles about 12 mm on a side, and a full-grown leaf comes to
    about 1500 triangles; about 500 at 0.1 and 4500 at 4. Coarser drops the finest
    teeth first, then rounds off the sinuses, and keeps the lobes; the pucker and the
    ruffle soften as the triangles across the blade grow, and the folds are on the
    veins and sharp at any detail.
    """


@configclass
class TrussCfg:
    """The trusses: a peduncle off the stem, bending over into a rachis with the fruit
    hung along it.

    The peduncle numbers default to the means measured on medium trusses in a
    high-wire house, and every truss draws its own about them; the rachis's internode
    to what those trusses' measured lengths leave for it (est.), and the pedicel to a
    genebank's measurements. A medium truss tomato is about 6 fruit of 60 mm; a
    cherry truss about 15 of 25 mm, branched.
    """

    fruits: int = 6
    """Fruit on a truss. Each truss draws a few more or fewer."""
    fruit_diameter: float = 0.06
    """Diameter of a fruit at the base of a truss, in meters; the fruit toward its tip
    come out a little smaller.
    """
    branched: bool = False
    """Whether the rachis forks in two at the first fruit, as a cherry truss does,
    rather than hanging as one.
    """
    peduncle_length: float = 0.079
    """Length of the peduncle from the stem to the first fruit, in meters."""
    peduncle_diameter: float = 0.007
    """Diameter of the peduncle, in meters."""
    peduncle_angle: float = 70.0
    """Angle between the stem and the peduncle where it leaves, in degrees: 90 leaves
    level, less leaves rising.
    """
    stiffness: float = 70.0
    """How stiff the peduncle and rachis are, as a Young's modulus in MPa. A truss grows
    standing out from the stem, and its fruit's weight bends it down against this:
    one in flower barely bends, and one sags further as its fruit swell. Below green
    wood's, as a stalk under a load for weeks gives way to it more than it would at
    once.
    """
    rachis_internode: float = 0.014
    """Length of the rachis between two pedicels under grown fruit, in meters: short, a
    truss's fruit crowd into a bunch, and long, they hang apart down the rachis.
    """
    pedicel_length: float = 0.031
    """Length of a pedicel under a grown fruit, from the rachis to the fruit's calyx, in
    meters. A small-fruited truss's are about as long as a large-fruited one's, so
    its fruit hang further out for their size.
    """
    pedicel_angle: float = 45.0
    """How far under the rachis its pedicels leave it, in degrees below its sides: 0
    leaves them level out to the sides, and more hangs the fruit further under it. At
    45 the two ranks, down alternate flanks, meet at a right angle, as they do on a
    tomato truss.
    """
    jointed: bool = True
    """Whether a pedicel has a knuckle, the joint a fruit breaks off at, as most truss
    varieties do.
    """
    stalks_in_flower: float = 0.35
    """How long a pedicel, and the internode of rachis leading to it, are when its
    flower opens, as a share of their length under the grown fruit: about a third on
    a large-fruited truss, whose stalks grow to about three times their length in
    flower, and nearer 1 on a small-fruited one, whose barely grow on.
    """
    missing_fruit: float = 0.05
    """Chance that a fruit is missing from its pedicel."""
    variations: int = 24
    """How many distinct truss meshes the scene may hold. A budget, not a count: the
    trusses are clustered and this is how many representatives the clustering may
    keep.
    """


@configclass
class FruitCfg:
    """The fruit and the flowers they set from: their shape, how ripe the crop is, and
    what colour ripe is.

    Every fruit has a stage: bud and flower before it sets, set, expanding and mature
    green while it grows, then breaker, turning, pink, light red and red as it
    colours from its blossom end up. A truss shows at the head in bud, opens its
    flowers from its base to its tip from about 15 cm below the head, sets them over
    the week after, and they reach full size about five trusses further down. A stem
    colours from its lowest truss up and a truss from its base to its tip, so in full
    production the lowest `ripe_trusses` trusses on a stem are colouring and the ones
    above them still green; a young crop's trusses are all green until the first are
    old enough. A truss is due once its tip fruit is turning. Every stem runs a
    little ahead of or behind the rest.

    Every fruit draws its own lobe count, its five or six sepals and how each curls,
    and a slight lean, within the ranges the research notes give; its flower has a
    petal between each two sepals.
    """

    ripe_trusses: float = 1.0
    """Trusses per stem that have coloured in full production, on average, counted up
    from the lowest: a fraction is the next one part way.
    """
    spread: float = 0.2
    """How far a truss's tip fruit lags behind its base fruit, as a share of the
    ripening from breaker to red: 0 colours a truss all at once.
    """
    color: str = "red"
    """The colour a ripe fruit turns: `red`, `orange`, `yellow` or `brown`."""
    aspect: float = 0.9
    """Height of a fruit over its width: under 1 is oblate, as round truss tomatoes and
    beefsteaks are, and over 1 elongated, as plum tomatoes are. Each fruit draws a
    little either side.
    """
    ribbing: float = 0.03
    """Depth of the grooves between a fruit's lobes, as a fraction of its radius: 0 is
    smooth, and a beefsteak's are about 0.15.
    """
    petal_length: float = 0.011
    """Length of a flower's petals, in meters: an open tomato flower is under 25 mm
    across, and a cherry tomato's smaller.
    """
    petal_reflex: float = 125.0
    """How far an open flower's petals sweep back, in degrees off pointing straight
    ahead: 90 opens it flat, and a tomato's sweep back past that toward its pedicel.
    """
    variations: int = 32
    """How many distinct fruit and flower meshes the scene may hold, by size, shape and
    stage. A budget, not a count: the fruit are clustered and this is how many
    representatives the clustering may keep, though a crop in flower keeps one for
    its flowers and one for its fruit at least. A fruit is drawn at its
    representative's size, so on a small budget a truss's fruit, packed to touch at
    their own sizes, are drawn into each other.
    """
    detail: float = 3.0
    """How finely a fruit's skin is meshed, as a multiplier: at 1 its rings and
    meridians keep within 0.6 mm of its shape, about a pixel to a camera half a meter
    away, as a leaf's margin does at its own detail of 1, and its faces within twice
    that; at 3 its rings and meridians keep within 0.2 mm and its faces within 0.4
    mm, which leaves its outline round from close up. A full-grown 60 mm fruit comes
    to about 1250 triangles with its calyx at 3, about 450 at 1 and 3700 at 8; each
    fruit is meshed for its own size, so a young one costs less.
    """


# <<< generated: fragments


@configclass
class GreenhouseCfg(GeneratedSceneCfg):
    """Spawn a procedurally generated greenhouse.

    The scene is generated on first use and cached as a USD file keyed on the
    fragments below, then spawned through Isaac Lab's ordinary USD-file path;
    see `GeneratedSceneCfg` for what that leaves open.

    It comes with its own static colliders: the floor as its own mesh, and
    each pipe rail segment as a capsule.

    .. code-block:: python

        GREENHOUSE_CFG = GreenhouseCfg(
            layout=LayoutCfg(rows=4, row_length=40.0),
            rail=RailCfg(gauge=0.55, unevenness=1.0),
        )

        # in a scene cfg: Newton steps only the assets a scene declares
        greenhouse = AssetBaseCfg(prim_path="/World/Greenhouse", spawn=GREENHOUSE_CFG)

    A truss whose node is below `StemCfg.flexible_below` is authored as a
    deformable curve carrying its fruit, and bends under the coupled solver
    `rods.make_physics_cfg_newton` builds for a scene that has any; `Shears`
    cuts it. Under a backend that cannot step one it is spawned as a static
    mesh instead. Each bent truss is about ten rigid bodies, so keep the height
    to what a robot reaches, and the scene to the rows it works.
    """

    PARAMS: ClassVar[type] = greenhouse_lab.GreenhouseParams
    PLANT: ClassVar[str] = "Plant"
    """The plant prim `crates/greenhouse-lab/src/elements/stem.rs` names
    `Plant_NNN`: a plant's bent trusses collide as one group."""

    # >>> generated: aggregate
    scene: SceneCfg = SceneCfg()
    layout: LayoutCfg = LayoutCfg()
    rail: RailCfg = RailCfg()
    gutter: GutterCfg = GutterCfg()
    wire: WireCfg = WireCfg()
    stem: StemCfg = StemCfg()
    leaf: LeafCfg = LeafCfg()
    truss: TrussCfg = TrussCfg()
    fruit: FruitCfg = FruitCfg()

    FRAGMENTS: ClassVar[tuple[str, ...]] = (
        "scene",
        "layout",
        "rail",
        "gutter",
        "wire",
        "stem",
        "leaf",
        "truss",
        "fruit",
    )
    """The fragment fields, in the order `GreenhouseParams` takes them: what `to_params`
    converts and the cache key is built from. Everything else on the cfg is applied
    to the spawned prim rather than baked into the USD, and takes no part in the key.
    """
    # <<< generated: aggregate

    def without_rods(self) -> Self | None:
        """The greenhouse with every truss rigid, or `None` when none bends as
        it stands: `StemCfg.flexible_below` is 0.

        A height promises no truss below it -- a young crop's lowest trusses
        hang higher -- and the coupled solver refuses an entry that owns no
        body, at reset, with its own error. Keep the height at 0 for such a
        scene.
        """
        # ponytail: read off the cfg, as the scene itself is only generated
        # inside the app, after the physics config was built.
        stem = self.stem
        if stem.flexible_below <= 0.0:
            return None
        return self.replace(stem=stem.replace(flexible_below=0.0))

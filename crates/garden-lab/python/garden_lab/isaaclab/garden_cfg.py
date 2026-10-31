"""Spawner configuration for a generated garden.

One `@configclass` fragment per element, mirroring the `*Params` pyclasses in
`garden_lab._core` field for field. The fragments and `GardenCfg`'s fields
are generated from the Rust params structs -- see
`crates/misina-lab/docs/editing-parameters.md`; the rest of `GardenCfg` is
hand-written, over the core's `GeneratedSceneCfg`.
"""

from __future__ import annotations

from typing import ClassVar

from isaaclab.utils.configclass import configclass

import garden_lab
from misina_lab.isaaclab.spawn import GeneratedSceneCfg


# >>> generated: fragments
@configclass
class SceneCfg:
    """Scene-wide parameters: the seed, and how far into the season the garden is."""

    seed: int = 0
    """The one seed the whole scene is generated from. Each element salts it with a
    constant of its own, so a new seed is a different garden and an edit to one
    element never re-rolls another.
    """
    week: float = 8.0
    """The week on the garden's calendar. Each crop grows from its own sowing week
    toward its harvest, so one garden holds seedlings beside a bed ready to pick.
    """


@configclass
class PhysicsCfg:
    """What a robot bumps into, besides the floor and the beds. Scene-wide, since a demo
    wants one switch.
    """

    stems_collide: bool = True
    """Whether every stem collides, as a chain of capsules along it."""
    fruit_collides: bool = False
    """Whether every fruit and root collides, as a capsule each."""


@configclass
class BedCfg:
    """The raised beds: their size, and how they are laid out.

    Beds run east–west along X in `columns`, and the `rows` are numbered across Y
    from the south, with a path between any two beds and round the whole garden.
    """

    width: float = 1.2
    """Width of a bed across Y, outside the frame, in meters. Up to 1.2 m is reachable
    from both sides; 1.5 m is FarmBot Genesis's bed.
    """
    length: float = 2.4
    """Length of a bed along X, outside the frame, in meters. 4 by 8 feet is the common
    footprint, and lumber comes in 8 feet.
    """
    height: float = 0.3
    """Height of the frame's rim above the floor, in meters. 15 to 30 cm is typical; 60
    to 90 cm is an accessible bed, which a wheeled robot reaches too. Below 0.25 m
    the soil is shallower than the 25 cm growing guides ask for.
    """
    rim: float = 0.04
    """How far the soil surface lies below the rim, in meters."""
    path: float = 0.9
    """Width of the paths between the beds and round the garden, in meters. 45 to 60 cm
    is comfortable on foot; 90 cm and up takes a cart or a wheeled robot.
    """
    columns: int = 2
    """Beds along X in each row."""
    rows: int = 3
    """Rows of beds across Y."""


@configclass
class PlantingCfg:
    """How a bed is filled, beyond what each crop says for itself."""

    jitter: float = 0.2
    """How far a plant stands off its exact place, as a fraction of the distance to its
    neighbours: 0 is a stamped grid.
    """
    misses: float = 0.05
    """Share of planting places left empty: seeds that never came up."""


@configclass
class LettuceCfg:
    """Lettuce, loose-leaf or heading: a rosette of entire blades, ruffled or smooth,
    round a stub of stem a couple of centimetres tall, grown to harvest over eight
    weeks.
    """

    leaves: int = 28
    """Blades on a harvest-ready plant."""
    leaf_length: float = 0.15
    """Length of the largest blade at harvest, in meters."""
    ruffle: float = 0.6
    """How much the margin ruffles: 0 is a smooth butterhead, 1 a frilly loose-leaf."""
    heading: bool = False
    """Whether the inner blades close over the crown into a head."""
    tint: str = "green"
    """Foliage colour: `green`, `red`, or a `mix` of plants of either across the bed."""
    share: float = 0.25
    """Share of the garden's planting squares this crop gets, against the other crops';
    0 leaves it out.
    """
    spacing: float = 0.15
    """Distance between plants, in meters, rounded to one, two, three or four per side
    of a 30 cm square.
    """
    sown_week: float = 0.0
    """The week it was sown, on the scene's calendar: a plant's age is the scene's week
    less this, and a bed is bare before it.
    """
    bed: int = 0
    """The one bed it is confined to, counted from 1; 0 plants it anywhere."""
    variations: int = 12
    """How many distinct meshes each of its organs, its leaves and its fruit, may share.
    A budget, not a count: the organs are clustered and this is how many
    representatives the clustering may keep.
    """


@configclass
class KaleCfg:
    """Kale: an upright stem carrying a crown of large curled blades on long petioles,
    the nodes below the leaves kept stripped bare, grown to harvest over ten weeks.
    """

    height: float = 0.6
    """Height of a harvest-ready stem, in meters."""
    leaves_kept: int = 10
    """Leaves left on the stem, counted down from its head; the nodes below are
    stripped. A stem grows a node every 3.5 cm.
    """
    leaf_length: float = 0.35
    """Length of a grown leaf, petiole and blade, in meters."""
    curl: float = 0.8
    """How tightly the margin curls: 0 is a flat lacinato, 1 a curly kale."""
    share: float = 0.25
    """Share of the garden's planting squares this crop gets, against the other crops';
    0 leaves it out.
    """
    spacing: float = 0.3
    """Distance between plants, in meters, rounded to one, two, three or four per side
    of a 30 cm square.
    """
    sown_week: float = 0.0
    """The week it was sown, on the scene's calendar: a plant's age is the scene's week
    less this, and a bed is bare before it.
    """
    bed: int = 0
    """The one bed it is confined to, counted from 1; 0 plants it anywhere."""
    variations: int = 12
    """How many distinct meshes each of its organs, its leaves and its fruit, may share.
    A budget, not a count: the organs are clustered and this is how many
    representatives the clustering may keep.
    """


@configclass
class CarrotCfg:
    """Carrot: a rosette of finely divided leaves over a taproot whose shoulder shows at
    the soil, pulled by its tops, grown to harvest over twelve weeks.
    """

    leaves: int = 9
    """Leaves on a harvest-ready plant."""
    leaf_length: float = 0.3
    """Length of a grown leaf, in meters."""
    root_length: float = 0.18
    """Length of a harvest-ready root, in meters."""
    root_diameter: float = 0.03
    """Diameter of a harvest-ready root at its shoulder, in meters."""
    share: float = 0.25
    """Share of the garden's planting squares this crop gets, against the other crops';
    0 leaves it out.
    """
    spacing: float = 0.075
    """Distance between plants, in meters, rounded to one, two, three or four per side
    of a 30 cm square.
    """
    sown_week: float = 0.0
    """The week it was sown, on the scene's calendar: a plant's age is the scene's week
    less this, and a bed is bare before it.
    """
    bed: int = 0
    """The one bed it is confined to, counted from 1; 0 plants it anywhere."""
    variations: int = 12
    """How many distinct meshes each of its organs, its leaves and its fruit, may share.
    A budget, not a count: the organs are clustered and this is how many
    representatives the clustering may keep.
    """


# <<< generated: fragments


@configclass
class GardenCfg(GeneratedSceneCfg):
    """Spawn a procedurally generated raised-bed garden.

    The scene is generated on first use and cached as a USD file keyed on the
    fragments below, then spawned through Isaac Lab's ordinary USD-file path;
    see `GeneratedSceneCfg` for what that leaves open.

    It comes with its own static colliders: the floor, and every bed's frame
    and soil, each as its own mesh.

    .. code-block:: python

        GARDEN_CFG = GardenCfg(bed=BedCfg(columns=1, rows=2, height=0.6))

        # in a scene cfg: Newton steps only the assets a scene declares
        garden = AssetBaseCfg(prim_path="/World/Garden", spawn=GARDEN_CFG)
    """

    PARAMS: ClassVar[type] = garden_lab.GardenParams

    # >>> generated: aggregate
    scene: SceneCfg = SceneCfg()
    physics: PhysicsCfg = PhysicsCfg()
    bed: BedCfg = BedCfg()
    planting: PlantingCfg = PlantingCfg()
    lettuce: LettuceCfg = LettuceCfg()
    kale: KaleCfg = KaleCfg()
    carrot: CarrotCfg = CarrotCfg()

    FRAGMENTS: ClassVar[tuple[str, ...]] = (
        "scene",
        "physics",
        "bed",
        "planting",
        "lettuce",
        "kale",
        "carrot",
    )
    """The fragment fields, in the order `GardenParams` takes them: what `to_params`
    converts and the cache key is built from. Everything else on the cfg is applied
    to the spawned prim rather than baked into the USD, and takes no part in the key.
    """
    # <<< generated: aggregate

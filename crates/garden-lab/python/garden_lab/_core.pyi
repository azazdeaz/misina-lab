"""Type stubs for the `garden_lab._core` extension module.

The classes between the ``generated`` markers are rendered from the Rust
params structs by ``cargo test -p garden-lab regen_params -- --ignored``:
edit the structs, not this file -- `crates/misina-lab/docs/editing-parameters.md`
says how. The `py.typed` marker beside it makes the package's annotations
visible to type checkers.
"""

from typing import Literal

__version__: str

# >>> generated: fragments
LettuceTint = Literal["green", "red", "mix"]
"""What `LettuceParams.tint` may be set to."""

class SceneParams:
    """Scene-wide parameters: the seed, and how far into the season the garden is."""

    seed: int
    """The one seed the whole scene is generated from. Each element salts it with a
    constant of its own, so a new seed is a different garden and an edit to one
    element never re-rolls another.
    """
    week: float
    """The week on the garden's calendar. Each crop grows from its own sowing week
    toward its harvest, so one garden holds seedlings beside a bed ready to pick.
    """

    def __init__(
        self,
        *,
        seed: int = 0,
        week: float = 8.0,
    ) -> None: ...
    def __repr__(self) -> str: ...

class PhysicsParams:
    """What a robot bumps into, besides the floor and the beds, and what bends.
    Scene-wide, since a demo wants one switch.
    """

    stems_collide: bool
    """Whether every stem collides, as a chain of capsules along it. A rod collides as
    itself, whatever this says.
    """
    fruit_collides: bool
    """Whether every fruit and root collides, as a capsule each."""
    leaves_collide: bool
    """Whether every leaf collides, as a box round its blade, fitted to the blade's
    extent in the plane it lies flattest in; the petiole does not. A leaf riding a
    bending stem carries its box with it.
    """
    flexible_stems: bool
    """Whether the stems that bend, a pole bean's vine, are rods the solver moves, their
    leaves and pods riding them; off, they are wood like every other stem.
    """
    segment: float
    """Length of a rod's segments, in meters: shorter bends smoother and costs a body
    per segment. No longer than 6 cm, the most a pole bean's vine can be sampled at
    and still clear its string.
    """

    def __init__(
        self,
        *,
        stems_collide: bool = True,
        fruit_collides: bool = False,
        leaves_collide: bool = False,
        flexible_stems: bool = False,
        segment: float = 0.05,
    ) -> None: ...
    def __repr__(self) -> str: ...

class BedParams:
    """The raised beds: their size, and how they are laid out.

    Beds run east–west along X in `columns`, and the `rows` are numbered across Y
    from the south, with a path between any two beds and round the whole garden.
    """

    width: float
    """Width of a bed across Y, outside the frame, in meters. Up to 1.2 m is reachable
    from both sides; 1.5 m is FarmBot Genesis's bed.
    """
    length: float
    """Length of a bed along X, outside the frame, in meters. 4 by 8 feet is the common
    footprint, and lumber comes in 8 feet.
    """
    height: float
    """Height of the frame's rim above the floor, in meters. 15 to 30 cm is typical; 60
    to 90 cm is an accessible bed, which a wheeled robot reaches too. Below 0.25 m
    the soil is shallower than the 25 cm growing guides ask for.
    """
    rim: float
    """How far the soil surface lies below the rim, in meters."""
    path: float
    """Width of the paths between the beds and round the garden, in meters. 45 to 60 cm
    is comfortable on foot; 90 cm and up takes a cart or a wheeled robot.
    """
    columns: int
    """Beds along X in each row."""
    rows: int
    """Rows of beds across Y."""
    trellis: float
    """Height of the trellis rail above the soil, in meters, on a bed that holds a
    climber: two posts at the bed's north corners and a rail between them, a string
    from it down to every climbing plant.
    """

    def __init__(
        self,
        *,
        width: float = 1.2,
        length: float = 2.4,
        height: float = 0.3,
        rim: float = 0.04,
        path: float = 0.9,
        columns: int = 2,
        rows: int = 3,
        trellis: float = 2.0,
    ) -> None: ...
    def __repr__(self) -> str: ...

class PlantingParams:
    """How a bed is filled, beyond what each crop says for itself."""

    jitter: float
    """How far a plant stands off its exact place, as a fraction of the distance to its
    neighbours: 0 is a stamped grid.
    """
    misses: float
    """Share of planting places left empty: seeds that never came up."""

    def __init__(
        self,
        *,
        jitter: float = 0.2,
        misses: float = 0.05,
    ) -> None: ...
    def __repr__(self) -> str: ...

class WeedParams:
    """The plants that come up where nothing was sown: in the beds' soil and on the
    paths, so many to the square metre of each. Every plant is one mesh: `variations`
    is the budget of distinct ones and `detail` the triangles a leaf is cut into.
    """

    pressure: float
    """Plants per square metre of bed soil. Zero is a weeded bed; the ceiling is one
    plant per slot of the 20 cm grid, about twenty-five.
    """
    path_pressure: float
    """Plants per square metre of path, the floor off the beds."""
    tall: float
    """The share of plants that are tall, bolters and broadleaves, against low tufts,
    mats and rosettes.
    """
    variations: int
    """How many distinct plant meshes the scene may hold. A budget, not a count: the
    plants are clustered by species, stage and size and this is how many
    representatives the clustering may keep.
    """
    detail: int
    """Triangles a leaf is cut into."""

    def __init__(
        self,
        *,
        pressure: float = 3.0,
        path_pressure: float = 1.0,
        tall: float = 0.3,
        variations: int = 24,
        detail: int = 16,
    ) -> None: ...
    def __repr__(self) -> str: ...

class LettuceParams:
    """Lettuce, loose-leaf or heading: a rosette of entire blades, ruffled or smooth,
    round a stub of stem a couple of centimetres tall, grown to harvest over eight
    weeks.
    """

    leaves: int
    """Blades on a harvest-ready plant."""
    leaf_length: float
    """Length of the largest blade at harvest, in meters."""
    ruffle: float
    """How much the margin ruffles: 0 is a smooth butterhead, 1 a frilly loose-leaf."""
    heading: bool
    """Whether the inner blades close over the crown into a head."""
    tint: LettuceTint
    """Foliage colour: `green`, `red`, or a `mix` of plants of either across the bed."""
    share: float
    """Share of the garden's planting squares this crop gets, against the other crops';
    0 leaves it out.
    """
    spacing: float
    """Distance between plants, in meters: rounded to one, two, three or four per side
    of a 30 cm square, or, wider than a square, to a plant every two or three
    squares, on the lattice that fits the most into the crop's block.
    """
    sown_week: float
    """The week it was sown, on the scene's calendar: a plant's age is the scene's week
    less this, and a bed is bare before it.
    """
    bed: int
    """The one bed it is confined to, counted from 1; 0 plants it anywhere."""
    variations: int
    """How many distinct meshes each of its organs, its leaves and its fruit, may share.
    A budget, not a count: the organs are clustered and this is how many
    representatives the clustering may keep.
    """

    def __init__(
        self,
        *,
        leaves: int = 28,
        leaf_length: float = 0.15,
        ruffle: float = 0.6,
        heading: bool = False,
        tint: LettuceTint = "green",
        share: float = 0.25,
        spacing: float = 0.15,
        sown_week: float = 0.0,
        bed: int = 0,
        variations: int = 12,
    ) -> None: ...
    def __repr__(self) -> str: ...

class KaleParams:
    """Kale: an upright stem carrying a crown of large curled blades on long petioles,
    the nodes below the leaves kept stripped bare, grown to harvest over ten weeks.
    """

    height: float
    """Height of a harvest-ready stem, in meters."""
    leaves_kept: int
    """Leaves left on the stem, counted down from its head; the nodes below are
    stripped. A stem grows a node every 3.5 cm.
    """
    leaf_length: float
    """Length of a grown leaf, petiole and blade, in meters."""
    curl: float
    """How tightly the margin curls: 0 is a flat lacinato, 1 a curly kale."""
    share: float
    """Share of the garden's planting squares this crop gets, against the other crops';
    0 leaves it out.
    """
    spacing: float
    """Distance between plants, in meters: rounded to one, two, three or four per side
    of a 30 cm square, or, wider than a square, to a plant every two or three
    squares, on the lattice that fits the most into the crop's block.
    """
    sown_week: float
    """The week it was sown, on the scene's calendar: a plant's age is the scene's week
    less this, and a bed is bare before it.
    """
    bed: int
    """The one bed it is confined to, counted from 1; 0 plants it anywhere."""
    variations: int
    """How many distinct meshes each of its organs, its leaves and its fruit, may share.
    A budget, not a count: the organs are clustered and this is how many
    representatives the clustering may keep.
    """

    def __init__(
        self,
        *,
        height: float = 0.6,
        leaves_kept: int = 10,
        leaf_length: float = 0.35,
        curl: float = 0.8,
        share: float = 0.25,
        spacing: float = 0.3,
        sown_week: float = 0.0,
        bed: int = 0,
        variations: int = 12,
    ) -> None: ...
    def __repr__(self) -> str: ...

class CarrotParams:
    """Carrot: a rosette of finely divided leaves over a taproot whose shoulder shows at
    the soil, pulled by its tops, grown to harvest over twelve weeks.
    """

    leaves: int
    """Leaves on a harvest-ready plant."""
    leaf_length: float
    """Length of a grown leaf, in meters."""
    root_length: float
    """Length of a harvest-ready root, in meters."""
    root_diameter: float
    """Diameter of a harvest-ready root at its shoulder, in meters."""
    share: float
    """Share of the garden's planting squares this crop gets, against the other crops';
    0 leaves it out.
    """
    spacing: float
    """Distance between plants, in meters: rounded to one, two, three or four per side
    of a 30 cm square, or, wider than a square, to a plant every two or three
    squares, on the lattice that fits the most into the crop's block.
    """
    sown_week: float
    """The week it was sown, on the scene's calendar: a plant's age is the scene's week
    less this, and a bed is bare before it.
    """
    bed: int
    """The one bed it is confined to, counted from 1; 0 plants it anywhere."""
    variations: int
    """How many distinct meshes each of its organs, its leaves and its fruit, may share.
    A budget, not a count: the organs are clustered and this is how many
    representatives the clustering may keep.
    """

    def __init__(
        self,
        *,
        leaves: int = 9,
        leaf_length: float = 0.3,
        root_length: float = 0.18,
        root_diameter: float = 0.03,
        share: float = 0.25,
        spacing: float = 0.075,
        sown_week: float = 0.0,
        bed: int = 0,
        variations: int = 12,
    ) -> None: ...
    def __repr__(self) -> str: ...

class BushBeanParams:
    """Bush bean: a short stem branching from its lowest nodes into a bush of trifoliate
    leaves, pods hanging in clusters under the upper leaves, grown to harvest over
    eight weeks.
    """

    height: float
    """Height of a harvest-ready plant's main stem, in meters."""
    branches: int
    """Branches leaving the lowest nodes."""
    leaf_length: float
    """Length of a grown leaf, petiole and leaflets, in meters."""
    pod_length: float
    """Length of a ripe pod, in meters."""
    share: float
    """Share of the garden's planting squares this crop gets, against the other crops';
    0 leaves it out.
    """
    spacing: float
    """Distance between plants, in meters: rounded to one, two, three or four per side
    of a 30 cm square, or, wider than a square, to a plant every two or three
    squares, on the lattice that fits the most into the crop's block.
    """
    sown_week: float
    """The week it was sown, on the scene's calendar: a plant's age is the scene's week
    less this, and a bed is bare before it.
    """
    bed: int
    """The one bed it is confined to, counted from 1; 0 plants it anywhere."""
    variations: int
    """How many distinct meshes each of its organs, its leaves and its fruit, may share.
    A budget, not a count: the organs are clustered and this is how many
    representatives the clustering may keep.
    """

    def __init__(
        self,
        *,
        height: float = 0.4,
        branches: int = 3,
        leaf_length: float = 0.18,
        pod_length: float = 0.13,
        share: float = 0.25,
        spacing: float = 0.1,
        sown_week: float = 0.0,
        bed: int = 0,
        variations: int = 12,
    ) -> None: ...
    def __repr__(self) -> str: ...

class PoleBeanParams:
    """Pole bean: a vine twining up a string from its bed's trellis, a large trifoliate
    leaf at every node and pods hanging from the upper nodes, grown to harvest over
    ten weeks.
    """

    height: float
    """Length of a harvest-ready vine up its string, in meters; a vine stops at the
    string's top.
    """
    leaf_length: float
    """Length of a grown leaf, petiole and leaflets, in meters."""
    pod_length: float
    """Length of a ripe pod, in meters."""
    share: float
    """Share of the garden's planting squares this crop gets, against the other crops';
    0 leaves it out.
    """
    spacing: float
    """Distance between plants, in meters: rounded to one, two, three or four per side
    of a 30 cm square, or, wider than a square, to a plant every two or three
    squares, on the lattice that fits the most into the crop's block.
    """
    sown_week: float
    """The week it was sown, on the scene's calendar: a plant's age is the scene's week
    less this, and a bed is bare before it.
    """
    bed: int
    """The one bed it is confined to, counted from 1; 0 plants it anywhere."""
    variations: int
    """How many distinct meshes each of its organs, its leaves and its fruit, may share.
    A budget, not a count: the organs are clustered and this is how many
    representatives the clustering may keep.
    """

    def __init__(
        self,
        *,
        height: float = 2.2,
        leaf_length: float = 0.25,
        pod_length: float = 0.16,
        share: float = 0.25,
        spacing: float = 0.15,
        sown_week: float = 0.0,
        bed: int = 0,
        variations: int = 12,
    ) -> None: ...
    def __repr__(self) -> str: ...

class ZucchiniParams:
    """Zucchini: a stout, leaning stem under a mound of giant palmate leaves on long
    hollow petioles, the fruit lying low in the canopy and cut as they ripen, grown
    to harvest over eight weeks.
    """

    height: float
    """Height of a harvest-ready stem, in meters; the leaves stand well above it."""
    leaf_length: float
    """Length of a grown leaf, petiole and blade, in meters."""
    fruit_length: float
    """Length of a ripe fruit, in meters."""
    share: float
    """Share of the garden's planting squares this crop gets, against the other crops';
    0 leaves it out.
    """
    spacing: float
    """Distance between plants, in meters: rounded to one, two, three or four per side
    of a 30 cm square, or, wider than a square, to a plant every two or three
    squares, on the lattice that fits the most into the crop's block.
    """
    sown_week: float
    """The week it was sown, on the scene's calendar: a plant's age is the scene's week
    less this, and a bed is bare before it.
    """
    bed: int
    """The one bed it is confined to, counted from 1; 0 plants it anywhere."""
    variations: int
    """How many distinct meshes each of its organs, its leaves and its fruit, may share.
    A budget, not a count: the organs are clustered and this is how many
    representatives the clustering may keep.
    """

    def __init__(
        self,
        *,
        height: float = 0.14,
        leaf_length: float = 0.65,
        fruit_length: float = 0.2,
        share: float = 0.25,
        spacing: float = 0.9,
        sown_week: float = 0.0,
        bed: int = 0,
        variations: int = 12,
    ) -> None: ...
    def __repr__(self) -> str: ...

# <<< generated: fragments

class GardenParams:
    """Every element's params as one value. Each fragment is a live object:
    ``params.bed.height = 0.6`` changes the scene the next call generates."""

    # >>> generated: aggregate
    scene: SceneParams
    physics: PhysicsParams
    bed: BedParams
    planting: PlantingParams
    weeds: WeedParams
    lettuce: LettuceParams
    kale: KaleParams
    carrot: CarrotParams
    bush_bean: BushBeanParams
    pole_bean: PoleBeanParams
    zucchini: ZucchiniParams

    def __init__(
        self,
        scene: SceneParams | None = None,
        physics: PhysicsParams | None = None,
        bed: BedParams | None = None,
        planting: PlantingParams | None = None,
        weeds: WeedParams | None = None,
        lettuce: LettuceParams | None = None,
        kale: KaleParams | None = None,
        carrot: CarrotParams | None = None,
        bush_bean: BushBeanParams | None = None,
        pole_bean: PoleBeanParams | None = None,
        zucchini: ZucchiniParams | None = None,
    ) -> None: ...
    # <<< generated: aggregate
    def __repr__(self) -> str: ...
    def generate_scene_json(self) -> str:
        """Generates the scene and returns it as a JSON document, the whole
        contract with the USD builder."""
        ...
    def write_usd(self, path: str) -> None:
        """Generates the scene and writes it to `path` as USD.

        The extension picks the format: `.usd`/`.usdc` for binary, `.usda` for
        text. The file must not already exist. The stage is authored by
        `misina_lab.usd.build_usd`.
        """
        ...

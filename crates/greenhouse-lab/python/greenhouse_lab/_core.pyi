"""Type stubs for the `greenhouse_lab._core` extension module.

The classes between the ``generated`` markers are rendered from the Rust
params structs by ``cargo test regen_params -- --ignored``: edit the structs,
not this file -- `crates/misina-lab/docs/editing-parameters.md` says how. The
`py.typed` marker beside it makes the package's annotations visible to type
checkers.
"""

from typing import Literal

__version__: str

# >>> generated: fragments
GutterMount = Literal["hanging", "standing"]
"""What `GutterParams.mount` may be set to."""

class SceneParams:
    """Scene-wide parameters: the seed."""

    seed: int
    """The one seed the whole scene is generated from. Each element salts it with a
    constant of its own, so a new seed is a different greenhouse and an edit to one
    element never re-rolls another.
    """

    def __init__(
        self,
        *,
        seed: int = 0,
    ) -> None: ...
    def __repr__(self) -> str: ...

class LayoutParams:
    """Where the rows go.

    Rows run along X from the row head, where the concrete main path crosses them,
    and are numbered across Y. Every gutter has a path on both sides, so there is one
    path more than there are rows.
    """

    rows: int
    """How many rows, one gutter each."""
    pitch: float
    """Distance between neighbouring gutters, in meters: the row pitch."""
    row_length: float
    """Length of a row from the main path to its far end, in meters. Farm rows run 90 to
    115 m; a shorter one simulates faster.
    """
    main_path: float
    """Width of the concrete main path across the row heads, in meters: where a robot
    changes rows.
    """
    plant_spacing: float
    """Distance between plants along a gutter, in meters, each in a propagation block on
    a slab. A slab carries the whole number of plants closest to this, so the real
    spacing comes out close to it rather than equal to it.
    """

    def __init__(
        self,
        *,
        rows: int = 6,
        pitch: float = 1.6,
        row_length: float = 20.0,
        main_path: float = 3.0,
        plant_spacing: float = 0.36,
    ) -> None: ...
    def __repr__(self) -> str: ...

class RailParams:
    """The pipe rail in every path: two heating pipes a trolley rides on.

    The pipes rest on a support every meter, and `unevenness` moves the supports off
    level, each by a draw of its own: the pair tilts across and each pipe pitches
    along its length.
    """

    gauge: float
    """Distance between the two pipes, centre to centre, in meters. Growers lay 425,
    500, 550, 600 and 800 mm.
    """
    pipe_diameter: float
    """Outer diameter of a pipe, in meters: the slider stops at the 45 and 51 mm heating
    pipes.
    """
    height: float
    """Height of the pipes' axis above the floor, in meters. An estimate, to be set from
    a measured site.
    """
    unevenness: float
    """How far the supports stand off level. 0 is a perfect rail; at 1 the pair tilts
    across by up to the 2° the rail spec allows; 2 is twice out of spec.
    """

    def __init__(
        self,
        *,
        gauge: float = 0.6,
        pipe_diameter: float = 0.051,
        height: float = 0.15,
        unevenness: float = 0.5,
    ) -> None: ...
    def __repr__(self) -> str: ...

class GutterParams:
    """The gutter each row's crop stands in, and what holds it up.

    Stone wool slabs lie end to end in the gutter, and a propagation block sits on a
    slab at every plant; the rows' plant spacing places them.
    """

    height: float
    """Height of the gutter's floor above the ground, in meters: where the slabs rest."""
    mount: GutterMount
    """What holds the gutter up: `hanging` from a rod every 2.5 m, which leaves the
    floor under it clear, or `standing` on a stand every 2.5 m.
    """

    def __init__(
        self,
        *,
        height: float = 0.5,
        mount: GutterMount = "hanging",
    ) -> None: ...
    def __repr__(self) -> str: ...

class WireParams:
    """The two crop wires over every gutter, which the stems are trained up to in a V."""

    height: float
    """Height of the crop wires above the floor, in meters: how tall the crop stands."""
    spread: float
    """How far each of a row's two wires runs to the side of its gutter, in meters. An
    estimate, to be set from a measured site.
    """

    def __init__(
        self,
        *,
        height: float = 4.5,
        spread: float = 0.35,
    ) -> None: ...
    def __repr__(self) -> str: ...

# <<< generated: fragments

class GreenhouseParams:
    """Every element's params as one value. Each fragment is a live object:
    ``params.rail.gauge = 0.55`` changes the scene the next call generates."""

    # >>> generated: aggregate
    scene: SceneParams
    layout: LayoutParams
    rail: RailParams
    gutter: GutterParams
    wire: WireParams

    def __init__(
        self,
        scene: SceneParams | None = None,
        layout: LayoutParams | None = None,
        rail: RailParams | None = None,
        gutter: GutterParams | None = None,
        wire: WireParams | None = None,
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

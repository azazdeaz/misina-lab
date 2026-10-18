"""Spawner configuration for a generated greenhouse.

One `@configclass` fragment per element, mirroring the `*Params` pyclasses in
`greenhouse_lab._core` field for field. The fragments and `GreenhouseCfg`'s
fields are generated from the Rust params structs -- see
`crates/misina-lab/docs/editing-parameters.md`; the rest of `GreenhouseCfg` is
hand-written, over the core's `GeneratedSceneCfg`.
"""

from __future__ import annotations

from typing import ClassVar

from isaaclab.utils.configclass import configclass

import greenhouse_lab
from misina_lab.isaaclab.spawn import GeneratedSceneCfg


# >>> generated: fragments
@configclass
class SceneCfg:
    """Scene-wide parameters: the seed."""

    seed: int = 0
    """The one seed the whole scene is generated from. Each element salts it with a
    constant of its own, so a new seed is a different greenhouse and an edit to one
    element never re-rolls another.
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
    a slab. A slab carries the whole number of plants closest to this, so the real
    spacing comes out close to it rather than equal to it.
    """


@configclass
class RailCfg:
    """The pipe rail in every path: two heating pipes a trolley rides on.

    The pipes rest on a support every meter, and `unevenness` moves the supports off
    level, each by a draw of its own: the pair tilts across and each pipe pitches
    along its length.
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

        # manager-based scene cfg
        greenhouse = AssetBaseCfg(prim_path="/World/Greenhouse", spawn=GREENHOUSE_CFG)
    """

    PARAMS: ClassVar[type] = greenhouse_lab.GreenhouseParams

    # >>> generated: aggregate
    scene: SceneCfg = SceneCfg()
    layout: LayoutCfg = LayoutCfg()
    rail: RailCfg = RailCfg()
    gutter: GutterCfg = GutterCfg()
    wire: WireCfg = WireCfg()

    FRAGMENTS: ClassVar[tuple[str, ...]] = (
        "scene",
        "layout",
        "rail",
        "gutter",
        "wire",
    )
    """The fragment fields, in the order `GreenhouseParams` takes them: what `to_params`
    converts and the cache key is built from. Everything else on the cfg is applied
    to the spawned prim rather than baked into the USD, and takes no part in the key.
    """
    # <<< generated: aggregate

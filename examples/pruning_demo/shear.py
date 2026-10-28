"""A bypass shear for a robot's hand, and its stroke through the rods it cuts.

The shear's blades are a bypass pruner's: a cutting blade whose convex edge
is ground on its outer face swings past a hook whose concave edge holds a
rod, and whose claw reaches past the cutting blade's tip. `Shear` is its
geometry, `urdf` writes it as two links for the importer, drawn as meshes
and colliding as convex pieces along them, and `Stroke` closes it a control
tick at a time, cutting every rod it closes on with `Shears`. A cut-and-hold
shear, `Shear.hold`, has a rubber jaw beside the hook and a sprung finger
beside the cutting blade, which press what the blade cuts between them until
the shear opens.

The pruning demo and the greenhouse demo keep this module the same.
Nothing here imports Isaac Lab: `Stroke` reaches the simulation only through
the `Shears` it is handed.
"""

from __future__ import annotations

import dataclasses
import hashlib
import math
import pathlib
import tempfile
from typing import TYPE_CHECKING

import numpy as np

from kinematics import rpy_of, transform

if TYPE_CHECKING:
    from misina_lab.isaaclab import Shears

HEAD = "shear_link"
"""The link the head and the hook are, fixed to the hand."""
BLADE = "blade_link"
"""The link the cutting blade is, swinging on `SHEAR_JOINT`."""
SHEAR_JOINT = "shear"
"""The cutting blade's joint: 0 is shut, `Shear.opening` is open."""
FINGER = "finger_link"
"""The link a cut-and-hold shear's finger is, riding the cutting blade on
`FINGER_JOINT`."""
FINGER_JOINT = "finger"
"""The finger's joint off the cutting blade: 0 is against the blade, and
the finger turns open from there as a rod pushes it back."""

BLADE_STEP = 0.03
"""How far the blade's target turns per control tick, closing or opening, in
radians: shut in thirty ticks, the better part of the second an electric
pruner takes, and the plate advances a couple of millimetres between one look
at what it has reached and the next, well within its own width."""

SLACK = 0.05
"""How far short of fully shut or fully open the blade may stop, in radians,
to count as there: the drive settles a little off its target."""

GRIP = 0.001
"""How far off a rod's surface a slice of the moving blade's collider stands
once the blades hold it, in meters: a hair clear of the wood, not in it."""

FOUL = 0.02
"""How far off the blades' plane a point of a piece cut free may lie and
still be in the mouth, in meters: past the axis of a piece lying on a plate,
and past half the spacing of the points `Stroke.fouled` takes along a
capsule, so one crossing the plane has a point this close."""

MATERIALS = {
    "steel": "0.55 0.57 0.60 1",
    "polished": "0.86 0.87 0.89 1",
    "coated": "0.16 0.17 0.19 1",
    "rubber": "0.62 0.12 0.08 1",
}
"""Colours, as URDF rgba: the head's galvanized steel, the cutting blade's
polished steel, the hook's dark coat and the jaws' red rubber."""

_CACHE = pathlib.Path(tempfile.gettempdir()) / "shear"
"""Where the blades' meshes are written, each named for its contents."""


@dataclasses.dataclass(frozen=True)
class Shear:
    """A bypass shear: a fixed blade, the hook, and a cutting blade that
    swings past it.

    In the shear's own frame both blades stand out along +Z from the pivot,
    the X axis through its origin, in the YZ plane, with their cutting edges
    on one curve when shut: `edge`, bowed toward +Y. The cutting blade lies
    back from it toward -Y, so its edge is convex, and the hook toward +Y, so
    its edge is the concave crook a rod is held in. The cutting blade swings
    about X, so the mouth -- the curved wedge between the two edges -- opens
    toward -Y, and closing it sweeps whatever lies across the blades. The
    edges being one curve turned about the pivot, the open mouth spans the
    same angle at every distance out.
    """

    blade: float = 0.08
    """The cutting blade's length, from the pivot to its tip along +Z."""

    width: float = 0.02
    """How wide either blade is at its root, from the cutting edge back. The
    cutting blade narrows from there to its point, the hook rounds off into
    its claw."""

    thickness: float = 0.004
    """Plate thickness, along the pivot."""

    bow: float = 0.006
    """How far the edges stand off the straight line from the pivot to the
    cutting blade's tip, midway along it."""

    claw: float = 0.008
    """How far the hook reaches past the cutting blade's tip, its edge
    curling on into the mouth: a rod slid out to the tip is caught there."""

    opening: float = 0.9
    """How far the moving blade swings open, in radians: about fifty
    degrees, an electric pruner's. The mouth is then wide enough that a rod
    a couple of centimetres off the planned point still lies between the
    blades."""

    head: float = 0.05
    """The body the blades are set in: a cube of this edge behind the pivot,
    holding the drive a real shear has -- Bumblebee's takes about 320 N to
    part an 8 mm cane."""

    slices: int = 8
    """How many pieces the moving blade collides as, end to end along it. A
    piece that reaches a held rod stands there while the blade closes on
    through it, and the others go on with the blade, so a centimetre each at
    the default size -- about a rod across -- leaves the rest of the blade
    colliding where it is drawn."""

    hold: float = 0.0
    """How far along the pivot, past the cutting blade, a cut-and-hold
    shear's jaws are: a rubber jaw fixed beside the hook, and beside the
    cutting blade a rubber finger, which a spring on `FINGER_JOINT` holds
    against it. The finger meets a rod as the blade does and gives as the
    blade closes on through it, so it presses the piece the blade cuts
    against the jaw until the shear has opened past it. 0 is a shear
    without them."""

    jaw_thickness: float = 0.008
    """How thick the jaw and the finger are along the pivot: the length of
    rod they press."""

    @property
    def bypass(self) -> float:
        """The moving plate's offset from the fixed one along the pivot: a
        plate's thickness of clearance between them, and half of one more."""
        return 1.5 * self.thickness

    def edge(self, z):
        """Where the edges lie across, in Y, `z` out along the blades in the
        frame of either blade shut: a parabola through the pivot and the
        cutting blade's tip. Past the tip it curls on toward -Y, as the
        hook's claw."""
        u = np.asarray(z) / self.blade
        return 4 * self.bow * u * (1 - u)

    def back(self, z):
        """Where the cutting blade's back lies across, in Y, `z` out along
        it in its own frame: straight from its root to its point."""
        return -self.width * (1 - np.asarray(z) / self.blade)

    def edge_angle(self, out):
        """How far the edges are turned off the blades' axis toward the
        mouth, `out` from the pivot, in radians: negative, since they bow
        away from it."""
        z = np.linspace(0.0, self.blade, 65)
        y = self.edge(z)
        return np.interp(out, np.hypot(y, z), np.arctan2(-y, z))

    def clearance(self, y, z):
        """How far the point (y, z) of the shear's frame lies off the fixed
        blade's edge, into the mouth and across the line out from the pivot;
        negative behind the edge."""
        out = np.hypot(y, z)
        return out * np.sin(np.arctan2(-y, z) - self.edge_angle(out))

    @property
    def mouth(self) -> tuple[float, float]:
        """Where a rod is brought to, as (y, z) in the shear's own frame:
        the centre of the widest circle between the straight lines from the
        pivot to the hook's tip and to the open cutting blade's tip, and the
        cutting blade's reach. A rod that far off the planned point in any
        direction still comes in past the hook's tip, which a rod outside
        it slides off, and lies within the blade's reach: 2.4 cm at the
        default size. The bowed edges leave more room than that toward the
        hook, and less toward the cutting blade, which carries a rod it
        meets across onto the hook."""
        tip = self.blade + self.claw
        hook = math.atan2(-float(self.edge(tip)), tip)
        half = (self.opening - hook) / 2
        d = self.blade / (1 + math.sin(half))
        return -d * math.sin(hook + half), d * math.cos(hook + half)

    @property
    def spans(self) -> np.ndarray:
        """Where each of the moving blade's `slices` lies along it, as (from,
        to) out from the pivot, shape (slices, 2): each is the stretch of the
        blade between two of `slices + 1` points evenly spaced along Z."""
        z = np.linspace(0.0, self.blade, self.slices + 1)
        ends = np.hypot(self.edge(z), z)
        return np.column_stack([ends[:-1], ends[1:]])

    @property
    def reach(self) -> float:
        """How far across, in -Y, the mouth opens: past the moving edge at
        full opening, a blade's length out from the pivot."""
        return self.blade * math.tan(self.opening)

    def off_edge(self, out: float, off: float) -> tuple[float, float]:
        """The point `out` from the pivot and `off` off the fixed blade's edge
        into the mouth, as `clearance` measures it, as (y, z) in the shear's
        frame."""
        turned = float(self.edge_angle(out)) + math.asin(off / out)
        return -out * math.sin(turned), out * math.cos(turned)

    def pinch(self, y: float, z: float, radius: float) -> float:
        """The blade angle at which the moving edge is `radius` from the
        point (y, z) of the shear's frame, on the mouth's side of it: where
        the moving blade touches a rod of that radius lying across the
        mouth there, and shutting further would push it."""
        out = math.hypot(y, z)
        turned = math.atan2(-y, z) - float(self.edge_angle(out))
        return math.asin(min(1.0, radius / out)) + turned

    def blade_plane(self, angle: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """The moving blade's plane with the shear at `angle`, in the shear's
        frame, as the rectangle `Shears.crossing` takes round the blade: a
        corner and its two edges, across from the blade's back past its bowed
        edge, and along it. `covers` says which crossings the blade itself
        covers; a rod whose axis it covers is cut there."""
        plate = transform(rpy=(angle, 0.0, 0.0))
        corner = plate @ np.array([self.bypass, -self.width, 0.0, 1.0])
        across = plate[:3, :3] @ np.array([0.0, self.width + self.bow, 0.0])
        up = plate[:3, :3] @ np.array([0.0, 0.0, self.blade])
        return corner[:3], across, up

    def wedge(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """The mouth at full opening, in the shear's frame, as the rectangle
        `Shears.crossing` takes: in the moving plate's plane, from `reach`
        across to the fixed edge's deepest point and the blade's length out.
        `in_wedge` says where in the shear's frame a crossing of it lies."""
        corner = np.array([self.bypass, -self.reach, 0.0])
        return corner, np.array([0.0, self.reach + self.bow, 0.0]), np.array([0.0, 0.0, self.blade])

    def covers(self, on: np.ndarray, angle: float) -> tuple[np.ndarray, np.ndarray]:
        """Of the points at (a, b) on `blade_plane`'s rectangle, with the
        blade at `angle`: which the moving blade covers, behind its edge and
        short of its back; and how far each lies off the fixed blade's edge,
        as `clearance` measures it."""
        y = -self.width + on[:, 0] * (self.width + self.bow)
        z = on[:, 1] * self.blade
        out = np.hypot(y, z)
        behind = np.arctan2(-y, z) - self.edge_angle(out)
        return (behind >= 0) & (y >= self.back(z)), out * np.sin(behind + angle)

    def in_wedge(self, on: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Where the points at (a, b) on `wedge`'s rectangle lie, as their y
        and z in the shear's frame."""
        return -self.reach + on[:, 0] * (self.reach + self.bow), on[:, 1] * self.blade


class Stroke:
    """The shear's moving blade, driven a control tick at a time: shut,
    cutting every rod it closes on, or open again.

    The blade collides as `Shear.slices` pieces end to end along it. Each
    follows the blade until a rod it carries across the mouth lies against
    the fixed blade and the piece reaches it. From there that piece stands
    where it is while the blade closes on through the rod -- nothing is
    crushed between the blades, and nothing the rod leans on is pulled from
    under it -- and follows the blade again once that has opened back past
    it. The pieces clear of every rod close with the blade, so its tip and
    its root collide where they are drawn.

    Every rod against the fixed blade is cut as the blade's edge passes its
    axis, each at its own tick, and every rod between the blades as they
    meet. One the open blade was in already as the stroke began -- gone
    into, not closed on -- is cut only then, if it is between them still.
    """

    def __init__(self, shear: Shear, frame: np.ndarray, shears: Shears):
        """`shear` is in the hand at `frame`, its own frame in the frame of
        the body it is on; `shears` cuts the rods, and finds the moving
        blade's slices among the robot's bodies by `BLADE`."""
        self.shear, self.frame, self.shears = shear, frame, shears
        self.jaw = shear.opening
        """The blade angle to hold."""
        self.cuts = 0
        """How many rods the stroke has cut; cleared once the blade is sent
        open."""
        self.inside: np.ndarray | None = None
        """The rod bodies the open blade was in already as the stroke began;
        cleared with `cuts`."""
        self.angle = shear.opening
        """The blade's angle at the last tick."""
        blade = next(i for i, label in enumerate(shears.labels) if label.endswith("/" + BLADE))
        shapes, rest = shears.colliders(blade)
        if len(shapes) != shear.slices:
            raise ValueError(
                f"the moving blade collides as {len(shapes)} shapes, not its {shear.slices} slices"
            )
        # Out from the pivot, the order `Shear.spans` gives them in.
        order = np.argsort(rest[:, 2])
        self._shapes, self._rest = shapes[order], rest[order]
        self.held = np.full(len(shapes), np.nan)
        """The blade angle each slice stands at while the blade is shut past
        it, out from the pivot; NaN where it rides the blade."""
        self.turned = np.zeros(len(shapes))
        """How far back from the blade each slice was last placed, in
        radians."""

    def tick(self, hand: np.ndarray, shear: float, closing: bool) -> bool:
        """Turn the target a `BLADE_STEP` shut, or open, and place the
        slices. `hand` is the pose of the body the shear is on in world
        coordinates and `shear` the blade's angle: shutting, the plate at that
        angle cuts each rod it has reached. Returns whether the blade has got
        there -- the two blades met, or the mouth fully open."""
        s = self.shear
        # A slice is written once a tick and rides the blade until the next,
        # so it is placed for where the blade will be by then.
        ahead = shear + min(0.0, shear - self.angle)
        self.angle = shear
        if closing:
            self.jaw = max(self.jaw - BLADE_STEP, 0.0)
            self.held = np.where(np.isnan(self.held), self._pinched(hand, ahead, shear), self.held)
        else:
            self.jaw, self.cuts, self.inside = min(self.jaw + BLADE_STEP, s.opening), 0, None
            # Released once the blade has opened back to it, or fully open:
            # the drive need not reach an angle a slice was held at.
            self.held[shear >= np.minimum(self.held, s.opening - SLACK)] = np.nan
        turned = np.where(np.isnan(self.held), 0.0, np.maximum(0.0, self.held - ahead))
        for i in np.flatnonzero(turned != self.turned):
            self.shears.place(self._shapes[i], _swung(self._rest[i], turned[i]))
        self.turned = turned
        if not closing:
            return shear > s.opening - SLACK
        # The blades meeting cut a rod still between them: the last look is
        # along the plate at the stop, a thin rod's axis being closer to it
        # than the blade gets before it counts as shut.
        shut = shear < SLACK
        angle = 0.0 if shut else shear
        hits, at, on = self.shears.crossing(*_placed(hand @ self.frame, *s.blade_plane(angle)))
        covered, off = s.covers(on, angle)
        hits, at, off = hits[covered], at[covered], off[covered]
        bodies = self.shears.bodies[hits]
        if self.inside is None:
            self.inside = bodies
        if shut:
            due = np.ones(len(hits), dtype=bool)
        else:
            # Until then, a rod counts only lying against the fixed blade:
            # one the moving blade still carries across the mouth can sink
            # into it past its own axis.
            due = ~np.isin(bodies, self.inside) & (off <= self.shears.radius[hits] + GRIP)
        # In body order, nearest the wood first along a rod: a rod through the
        # plate twice is cut where it is nearer the wood, and the other
        # crossing falls with the piece.
        for i in np.flatnonzero(due):
            if not self.shears.loose[hits[i]]:
                self.cuts += self.shears.cut(int(bodies[i]), float(at[i]))
        return shut

    def _pinched(self, hand: np.ndarray, ahead: float, shear: float) -> np.ndarray:
        """The angle to stand each slice at, NaN for one to leave on the
        blade. Where a rod across the mouth, within the blades' reach, lies
        against the fixed blade and the moving edge reaches it by `ahead`,
        every slice along which the edge meets it stands at the angle it does
        -- or at the blade's own `shear`, if that is past it already; a slice
        two rods reach stands at the first the edge touches."""
        s = self.shear
        stand = np.full(s.slices, np.nan)
        hits, _, on = self.shears.crossing(*_placed(hand @ self.frame, *s.wedge()))
        for i, y, z in zip(hits, *s.in_wedge(on), strict=True):
            radius = self.shears.radius[i] + GRIP
            out = math.hypot(y, z)
            if s.clearance(y, z) > radius or out > s.blade:
                continue
            if (pinch := s.pinch(y, z, radius)) < ahead:
                continue
            # From first touch until it passes the axis, the edge meets the
            # rod within a radius of the axis's own distance out.
            along = (s.spans[:, 0] < out + radius) & (s.spans[:, 1] > out - radius)
            stand[along] = np.fmax(stand[along], min(pinch, shear))
        return stand

    def fouled(self, hand: np.ndarray) -> bool:
        """Whether a piece cut free lies in the mouth, `hand` being the pose
        of the body the shear is on: one that fell across it or onto the
        blades, which the blade does not cut and the arm would carry on to
        the next cut."""
        s = self.shear
        corner, u, v = _placed(hand @ self.frame, *s.wedge())
        start, end = self.shears.capsules()
        loose = self.shears.loose
        # Points along every loose capsule, from the wedge's corner: a piece
        # lying flat on a plate crosses no plane of the mouth's.
        along = np.linspace(0.0, 1.0, 5)[:, None]
        points = start[loose, None] + along * (end - start)[loose, None] - corner
        normal = np.cross(u, v)
        a, b = points @ u / (u @ u), points @ v / (v @ v)
        off = np.abs(points @ normal) / np.linalg.norm(normal)
        # Out to the back of either blade, across the mouth.
        w = s.width / (s.reach + s.bow)
        return bool(((-w <= a) & (a <= 1 + w) & (0 <= b) & (b <= 1) & (off <= FOUL)).any())


def urdf(
    s: Shear, parent: str, frame: np.ndarray, mass: float = 0.5, fixed: float | None = None
) -> str:
    """The shear as URDF: `HEAD` fixed to link `parent` at `frame` in it,
    holding the head, `mass` kilograms, behind the pivot and the hook, and
    `BLADE` on `SHEAR_JOINT` off it, the cutting blade -- or, with `fixed`,
    fixed at that angle and the finger against it, for a planner that holds
    the shear still. The hook lies back from `Shear.edge` along +Y, the
    cutting blade along -Y in its own frame, so their edges meet along it at
    zero. With `Shear.hold`, the jaw is on `HEAD` with the hook's outline,
    and `FINGER` on `FINGER_JOINT` off `BLADE`, with the cutting blade's."""

    def hook(z):
        # Its back rounds off from the root into the tip of the claw.
        return s.edge(z), s.edge(z) + s.width * np.sqrt(1 - z / (s.blade + s.claw))

    def cutting(z):
        return s.edge(z), s.back(z)

    head = [
        _box((0.0, 0.0, -s.head / 2), (s.head, s.head, s.head), "steel"),
        _blade(s, 0.0, hook, s.blade + s.claw, s.slices + 1, 0.0, "coated", "hook"),
    ]
    blade = _blade(s, s.bypass, cutting, s.blade, s.slices, 1.5 * s.thickness, "polished", "blade")
    xyz, rpy = (
        " ".join(str(float(v)) for v in each) for each in (frame[:3, 3], rpy_of(frame[:3, :3]))
    )
    turning = (
        f'<axis xyz="1 0 0"/>\n    <limit lower="0" upper="{s.opening}" effort="10" velocity="6"/>'
    )
    swing = turning if fixed is None else f'<origin xyz="0 0 0" rpy="{fixed} 0 0"/>'
    finger = ""
    if s.hold:
        thick = s.jaw_thickness
        head.append(
            _blade(s, s.hold, hook, s.blade + s.claw, s.slices + 1, 0.0, "rubber", "jaw", thick)
        )
        # The finger starts a third of the way out: nearer the pivot, its
        # back would swing into the head as the shear opens.
        start = s.blade / 3
        finger = f"""
  <joint name="{FINGER_JOINT}" type="{"revolute" if fixed is None else "fixed"}">
    <parent link="{BLADE}"/>
    <child link="{FINGER}"/>
    {turning if fixed is None else ""}
  </joint>
  <link name="{FINGER}">
{_blade(s, s.hold, cutting, s.blade, s.slices, 0.0, "rubber", "finger", thick, start)}
    <inertial>
      <origin xyz="{s.hold} 0 {(start + s.blade) / 2}"/>
      <mass value="0.02"/>
{_box_inertia(0.02, thick, s.width, s.blade - start)}
    </inertial>
  </link>"""

    return f"""
  <joint name="tool" type="fixed">
    <parent link="{parent}"/>
    <child link="{HEAD}"/>
    <origin xyz="{xyz}" rpy="{rpy}"/>
  </joint>
  <link name="{HEAD}">
{chr(10).join(head)}
    <inertial>
      <origin xyz="0 0 {-s.head / 2}"/>
      <mass value="{mass}"/>
{_box_inertia(mass, s.head, s.head, s.head)}
    </inertial>
  </link>
  <joint name="{SHEAR_JOINT}" type="{"revolute" if fixed is None else "fixed"}">
    <parent link="{HEAD}"/>
    <child link="{BLADE}"/>
    {swing}
  </joint>
  <link name="{BLADE}">
{blade}
    <inertial>
      <origin xyz="0 0 {s.blade / 2}"/>
      <mass value="0.05"/>
{_box_inertia(0.05, s.thickness, s.width, s.blade)}
    </inertial>
  </link>{finger}"""


def _blade(
    s: Shear,
    x: float,
    outline,
    length: float,
    pieces: int,
    bevel: float,
    material: str,
    name: str,
    thickness: float | None = None,
    start: float = 0.0,
) -> str:
    """One blade, centred `x` along the pivot and from `start` to `length`
    out from it, `thickness` thick along it, a plate's by default, its
    `outline` giving its edge and its back across in Y at each Z. Drawn as a
    mesh, its face toward +X ground back `bevel` from the edge to a sharp
    one, if `bevel`; colliding as `pieces` convex prisms end to end along
    it, each the blade between two points evenly spaced along Z, with the
    chord of the edge for its edge. Each piece's mesh is about its own
    middle, placed there by its origin, so its pose says where along the
    blade it is."""
    thickness = s.thickness if thickness is None else thickness
    low, high = x - thickness / 2, x + thickness / 2

    def sections(z, corners):
        """A polygon across the blade at each of `z`, from `corners`: each an
        x, and its y at each of `z`."""
        return np.stack([np.column_stack([np.full_like(z, a), b, z]) for a, b in corners], axis=1)

    z = np.linspace(start, length, 41)
    edge, back = outline(z)
    ground = edge - np.sign(edge - back) * np.minimum(bevel, np.abs(edge - back))
    # The edge is a twentieth of the plate thick where it is ground, the
    # plate's whole thickness where it is not.
    land = low + thickness / 20 if bevel else high
    drawn = sections(z, [(low, back), (low, edge), (land, edge), (high, ground), (high, back)])
    mesh = '      <geometry><mesh filename="{}"/></geometry>'
    parts = [_shaped(mesh.format(_stl(_loft(drawn), name)), ("visual",), material)]
    z = np.linspace(start, length, pieces + 1)
    edge, back = outline(z)
    for i in range(pieces):
        ends = slice(i, i + 2)
        piece = sections(
            z[ends], [(low, back[ends]), (low, edge[ends]), (high, edge[ends]), (high, back[ends])]
        )
        middle = piece.reshape(-1, 3).mean(axis=0)
        origin = f'      <origin xyz="{middle[0]} {middle[1]} {middle[2]}"/>\n'
        path = _stl(_loft(piece - middle), f"{name}_{i}")
        parts.append(_shaped(origin + mesh.format(path), ("collision",), material))
    return "\n".join(parts)


def _loft(sections: np.ndarray) -> np.ndarray:
    """The closed surface through `sections`, shape (stations, corners, 3):
    a polygon across the blade at each station along it, the corners of one
    matching those of the next. Returned as triangles, shape (n, 3, 3),
    wound outward."""
    s = sections
    # Wound one way round each section, the side quads face outward.
    widest = np.argmax(np.ptp(s[:, :, 1], axis=1))
    x, y = s[widest, :, 0], s[widest, :, 1]
    if np.dot(x, np.roll(y, -1)) - np.dot(np.roll(x, -1), y) < 0:
        s = s[:, ::-1]
    a, b = s[:-1], np.roll(s[:-1], -1, axis=1)
    c, d = np.roll(s[1:], -1, axis=1), s[1:]
    sides = np.concatenate([np.stack([a, b, c], 2), np.stack([a, c, d], 2)]).reshape(-1, 3, 3)
    fan = np.arange(1, s.shape[1] - 1)
    caps = (
        [s[0, [0] * len(fan)], s[0, fan + 1], s[0, fan]],
        [s[-1, [0] * len(fan)], s[-1, fan], s[-1, fan + 1]],
    )
    return np.concatenate([sides, *(np.stack(cap, 1) for cap in caps)])


def _stl(triangles: np.ndarray, name: str) -> str:
    """`triangles`, shape (n, 3, 3), written once as a binary STL file in
    `_CACHE` named for its contents, leaving out the degenerate ones; its
    path."""
    a, b, c = triangles[:, 0], triangles[:, 1], triangles[:, 2]
    normal = np.cross(b - a, c - a)
    length = np.linalg.norm(normal, axis=1)
    keep = length > 1e-12
    record = np.zeros(
        keep.sum(), dtype=[("normal", "<f4", 3), ("corners", "<f4", (3, 3)), ("attribute", "<u2")]
    )
    record["normal"] = normal[keep] / length[keep, None]
    record["corners"] = triangles[keep]
    data = bytes(80) + np.uint32(len(record)).tobytes() + record.tobytes()
    path = _CACHE / f"{name}_{hashlib.sha256(data).hexdigest()[:16]}.stl"
    if not path.exists():
        _CACHE.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
    return str(path)


def _box(xyz, size, material: str) -> str:
    """A box, drawn and colliding."""
    shape = (
        f'      <origin xyz="{xyz[0]} {xyz[1]} {xyz[2]}"/>\n'
        f'      <geometry><box size="{size[0]} {size[1]} {size[2]}"/></geometry>'
    )
    return _shaped(shape, ("visual", "collision"), material)


def _shaped(shape: str, tags: tuple[str, ...], material: str) -> str:
    """One geometry repeated as each of `tags`, coloured where it is a visual."""
    colour = f'\n      <material name="{material}"><color rgba="{MATERIALS[material]}"/></material>'
    return "\n".join(
        f"    <{tag}>\n{shape}{colour if tag == 'visual' else ''}\n    </{tag}>" for tag in tags
    )


def _box_inertia(mass: float, x: float, y: float, z: float) -> str:
    ixx, iyy, izz = (mass * (b * b + c * c) / 12 for b, c in ((y, z), (x, z), (x, y)))
    return (
        f'      <inertia ixx="{ixx:.6g}" iyy="{iyy:.6g}" izz="{izz:.6g}" ixy="0" ixz="0" iyz="0"/>'
    )


def _placed(frame: np.ndarray, corner, u, v) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """The rectangle `corner`, `u`, `v` given in `frame`, in world coordinates."""
    return frame[:3, 3] + frame[:3, :3] @ corner, frame[:3, :3] @ u, frame[:3, :3] @ v


def _swung(pose: np.ndarray, angle: float) -> np.ndarray:
    """`pose`, a position and (x, y, z, w) rotation in one array of 7, turned
    by `angle` about the x axis of the frame it is in."""
    turn = np.array([math.sin(angle / 2), 0.0, 0.0, math.cos(angle / 2)])
    v, w = pose[3:6], pose[6]
    rotation = [*(turn[3] * v + w * turn[:3] + np.cross(turn[:3], v)), turn[3] * w - turn[:3] @ v]
    return np.array([*_rotate(turn, pose[:3]), *rotation])


def _rotate(quat: np.ndarray, v: np.ndarray) -> np.ndarray:
    """`v` turned by the (x, y, z, w) rotation `quat`."""
    x, y, z, w = quat
    u = np.array([x, y, z])
    return v + 2 * np.cross(u, np.cross(u, v) + w * v)

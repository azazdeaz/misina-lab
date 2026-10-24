"""A Bumblebee-like pruning robot: a real Warthog and UR10, and the rest
generated from Bumblebee's published dimensions.

Bumblebee (Silwal et al., 2021) is a Clearpath Warthog driving the alley with
an aluminium extrusion frame on its back, a UR5 on a 1.35 m linear slide along
the frame, and a bypass shear in the arm's hand. This is that layout with a
UR10 in the UR5's place: the Warthog as Clearpath's own URDF draws and weighs
it, fetched from the demos' asset dataset; a frame on its deck and a rail
along the frame's row-side edge carrying a carriage on a prismatic joint,
from primitives; the stock UR10 Isaac Sim ships, standing on the carriage;
and a shear head whose one blade swings on a revolute joint. `urdf` writes
the robot from the Warthog's URDF, the fields of `Bumblebee` and the UR10's
own URDF, and `Bumblebee.chain` is the same joints for `kinematics`, so the
arm the solver poses is the arm the simulation steps.

The shear's blades are a bypass pruner's: a cutting blade with a convex,
ground edge swinging past a hook with a concave one, drawn as meshes `urdf`
writes beside the URDF and colliding as convex pieces along them.

The shear touches what it prunes. Its head and both blades collide with the
canes, so a cane the shear comes in on is pushed aside or funnelled into the
mouth, and the moving blade carries it across the mouth onto the fixed one.
Held there, it is cut where the moving blade's edge reaches its axis: tick by
tick while the shear closes, `pruner.Stroke` looks along `Bumblebee.blade` at
the blade's angle with `Shears.crossing` and cuts every cane it closes on.
The moving blade collides as `Shear.slices` pieces end to end along it, and
`Stroke` stands each piece that reaches a held cane at the cane while the
blade itself closes on through it; the rest go on with the blade.

The robot drives with the row on its left: the rail is on the +Y edge of the
frame, and the arm reaches out that way.
"""

from __future__ import annotations

import copy
import dataclasses
import functools
import hashlib
import importlib.util
import math
import pathlib
import tempfile
import xml.etree.ElementTree as ElementTree

import numpy as np

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg

from kinematics import Joint, rpy_of, transform

ASSETS = "azazdeaz/misina-lab-sim-assets"
"""The Hugging Face dataset the demos' larger assets live in. `huggingface_hub`
fetches what a demo needs from it on first use and caches it."""

ASSETS_REVISION = "7fca5f5bdadb2b5217d1e86726cf16431e1978b4"
"""The dataset commit the demo is pinned to."""

WARTHOG = "clearpath_warthog"
"""Clearpath's Warthog, as warthog-cpr/warthog@55555b6 describes it:
`warthog.urdf`, expanded from its xacro with the default options, its meshes
and its BSD licence."""

MOUNT = "top_chassis_link"
"""The Warthog's link the frame stands on: the top of its deck, where
Clearpath mount its accessories."""

CLEARED = "left_diff_unit_antenna_link"
"""The Warthog's link left off: its left antenna, which stands up through
where the rail runs."""

CORNERS = ("front_left", "front_right", "rear_left", "rear_right")
WHEEL_JOINTS = [f"{corner}_wheel_joint" for corner in CORNERS]
"""The drive joints, in `CORNERS` order, as the Warthog names them. A skid
steer has no steering joint: the left pair and the right pair turn at
different speeds."""

LEFT = [joint for joint in WHEEL_JOINTS if "left" in joint]
RIGHT = [joint for joint in WHEEL_JOINTS if "right" in joint]

SLIDE_JOINT = "slide"
SHEAR_JOINT = "shear"
"""The one joint of the cutter: 0 is closed, `Shear.opening` is open."""

ARM_BASE = "arm_base"
"""The link the arm stands on: the carriage, once the importer merges the
fixed joint between them."""

HAND = "wrist_3_link"
"""The body the shear is fixed to, and whose pose says where the mouth is."""

BLADE = "blade_link"
"""The body the moving blade is, swinging on `SHEAR_JOINT`."""

UR10_URDF = (
    pathlib.Path(importlib.util.find_spec("isaacsim").submodule_search_locations[0])
    / "exts/isaacsim.asset.importer.urdf/data/urdf/robots/ur10/urdf/ur10.urdf"
)
"""The UR10 that Isaac Sim's URDF importer ships: Universal Robots' meshes,
cylinder colliders, and the ur_description joint table. Zero everywhere is
the arm lying out along +X."""

_UR10 = ElementTree.parse(UR10_URDF).getroot()
_MOVING = [joint for joint in _UR10.findall("joint") if joint.get("type") == "revolute"]
_LINKS = [_MOVING[0].find("parent").get("link"), *(j.find("child").get("link") for j in _MOVING)]
"""The UR10's links from its base out, one per joint in `_MOVING` past the
base: its `ee_link`, a frame with no part of the arm in it, is left off."""


def _numbers(text: str) -> tuple[float, ...]:
    return tuple(float(v) for v in text.split())


def _read(joint: ElementTree.Element) -> Joint:
    """A URDF joint element as `kinematics` reads it."""
    origin, limit = joint.find("origin"), joint.find("limit")
    return Joint(
        joint.get("name"),
        _numbers(origin.get("xyz")),
        _numbers(origin.get("rpy")),
        _numbers(joint.find("axis").get("xyz")),
        limits=(float(limit.get("lower")), float(limit.get("upper"))),
        effort=float(limit.get("effort")),
        velocity=float(limit.get("velocity")),
    )


# The elbow is held to half a turn either way, as ur_description's limited
# arm has it, so the solver never folds the forearm through the upper arm.
UR10 = tuple(
    dataclasses.replace(joint, limits=(-math.pi, math.pi)) if joint.name == "elbow_joint" else joint
    for joint in map(_read, _MOVING)
)
ARM_JOINTS = [joint.name for joint in UR10]
SHOULDER_JOINTS, WRIST_JOINTS = ARM_JOINTS[:3], ARM_JOINTS[3:]

TOOL = transform(
    _numbers(_UR10.find("joint[@name='ee_fixed_joint']/origin").get("xyz")),
    (-math.pi / 2, 0.0, 0.0),
)
"""The flange frame off the last wrist: +Z out of the hand, where a tool
bolts on."""

ARM_HOME = (0.0, -2.0, 2.2, -1.8, -1.57, 0.0)
"""Joint positions the arm is folded to for driving: elbow up, hand over the
deck. The slide and the shear start at zero, mid-rail and closed."""

# -- Actuation. Position drives on the arm, the slide and the shear, and
# velocity drives on the wheels.
SHOULDER_STIFFNESS = 6000.0
SHOULDER_DAMPING = 600.0
"""The three joints carrying the arm's weight: stiff, so that weight sags
the mouth a couple of centimetres off its pose at most -- the shoulder holds
up to 90 N m of it with the arm reaching out. The mouth is a few centimetres
across, and a cane outside it when the blades meet is not cut."""
WRIST_STIFFNESS = 100.0
WRIST_DAMPING = 10.0
"""The three wrist joints, which carry little: gains a sixtieth of the
shoulder's, so a wrist's 56 N m is not saturated by a few tenths of a radian
of error."""
ARMATURE = {"shoulder": 0.1, "wrist": 0.02, "shear": 0.002}
"""Reflected rotor inertia added to each joint, in kg m^2, the way a geared
servo carries its motor's. Without it the last wrist -- a link of two
hundred grams on a 56 N m drive -- turns a tenth of a radian in one physics
substep and chatters about its target instead of settling on it."""
SLIDE_STIFFNESS = 4000.0
SLIDE_DAMPING = 400.0
SHEAR_STIFFNESS = 20.0
SHEAR_DAMPING = 1.0
DRIVE_DAMPING = 1500.0  # with zero stiffness this is the velocity drive's strength
DRIVE_EFFORT_PER_KG = 0.5
"""N m per kilogram of machine each wheel may push with: what a wheel can
push is what is standing on it."""

# -- The frame on the Warthog's back.
FENDERS = 0.85
"""The top of the Warthog's fenders, above the ground, as `urdf` stands it:
fenders.stl's highest point."""

POSTS = (0.6, 0.3)
"""Where the frame's four posts stand on `MOUNT`: this far either way along
the deck and across it."""

BEAM = 0.04
"""The side of the frame's square extrusion."""

RAIL = (0.08, 0.06)
"""The rail's width and height."""

FRAME_MASS = 20.0
"""The frame and the rail together."""

MATERIALS = {
    "steel": "0.55 0.57 0.60 1",
    "aluminium": "0.78 0.80 0.82 1",
    "polished": "0.86 0.87 0.89 1",
    "coated": "0.16 0.17 0.19 1",
}
"""Colours, as URDF rgba: galvanized steel, bare aluminium, the cutting
blade's polished steel and the hook's dark coat."""


@dataclasses.dataclass(frozen=True)
class Shear:
    """A bypass shear: a fixed blade, the hook, and a cutting blade that
    swings past it.

    Both blades stand out of the hand along its +Z, in its YZ plane, with
    their cutting edges on one curve when shut: `edge`, bowed toward +Y.
    The cutting blade lies back from it toward -Y, so its edge is convex,
    and the hook toward +Y, so its edge is the concave crook a cane is held
    in. The cutting blade swings about the hand's X axis at the blades'
    base, so the mouth -- the curved wedge between the two edges -- opens in
    the hand's -Y, and closing it sweeps whatever lies across the blades.
    The edges being one curve turned about the pivot, the open mouth spans
    the same angle at every distance out.
    """

    blade: float = 0.08
    """The cutting blade's length, from the pivot to its tip along the
    hand's +Z."""

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
    curling on into the mouth: a cane slid out to the tip is caught there."""

    opening: float = 0.9
    """How far the moving blade swings open, in radians: about fifty
    degrees, an electric pruner's. The mouth is then wide enough that a cane
    a couple of centimetres off the planned point still lies between the
    blades."""

    head: float = 0.05
    """The body the blades are set in: a cube of this edge, holding the drive
    a real shear has -- Bumblebee's takes about 320 N to part an 8 mm cane."""

    slices: int = 8
    """How many pieces the moving blade collides as, end to end along it. A
    piece that reaches a held cane stands there while the blade closes on
    through it, and the others go on with the blade, so a centimetre each at
    the default size -- about a cane across -- leaves the rest of the blade
    colliding where it is drawn."""

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
        """Where a cane is brought to, as (y, z) in the shear's own frame:
        the centre of the widest circle between the straight lines from the
        pivot to the hook's tip and to the open cutting blade's tip, and the
        cutting blade's reach. A cane that far off the planned point in any
        direction still comes in past the hook's tip, which a cane outside
        it slides off, and lies within the blade's reach: 2.4 cm at the
        default size. The bowed edges leave more room than that toward the
        hook, and less toward the cutting blade, which carries a cane it
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

    def pinch(self, y: float, z: float, radius: float) -> float:
        """The blade angle at which the moving edge is `radius` from the
        point (y, z) of the shear's frame, on the mouth's side of it: where
        the moving blade touches a cane of that radius lying across the
        mouth there, and shutting further would push it."""
        out = math.hypot(y, z)
        turned = math.atan2(-y, z) - float(self.edge_angle(out))
        return math.asin(min(1.0, radius / out)) + turned

    def covers(self, on: np.ndarray, angle: float) -> tuple[np.ndarray, np.ndarray]:
        """Of the points at (a, b) on `Bumblebee.blade`'s rectangle, with the
        blade at `angle`: which the moving blade covers, behind its edge and
        short of its back; and how far each lies off the fixed blade's edge,
        as `clearance` measures it."""
        y = -self.width + on[:, 0] * (self.width + self.bow)
        z = on[:, 1] * self.blade
        out = np.hypot(y, z)
        behind = np.arctan2(-y, z) - self.edge_angle(out)
        return (behind >= 0) & (y >= self.back(z)), out * np.sin(behind + angle)

    def in_wedge(self, on: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Where the points at (a, b) on `Bumblebee.wedge`'s rectangle lie,
        as their y and z in the shear's frame."""
        return -self.reach + on[:, 0] * (self.reach + self.bow), on[:, 1] * self.blade


@dataclasses.dataclass(frozen=True)
class Bumblebee:
    """One machine, in meters and kilograms.

    The chassis is a Clearpath Warthog whatever the machine, and its wheels
    and mass are read off its URDF; the slide is the paper's, and the arm the
    stock UR10. Everything here is free to move with `dataclasses.replace`.
    """

    travel: float = 1.35
    """The slide's stroke, along the row, centered on the chassis."""

    deck: float = 0.94
    """The frame's top, above the ground, which the rail sits on."""

    rail_offset: float = 0.45
    """The rail's centre line, out from the machine's toward the row: over
    the Warthog's left fenders."""

    carriage: float = 0.08
    """The carriage's height, which is what the arm stands on."""

    shear: Shear = Shear()
    max_speed: float = 1.0
    """m/s on the ground, flat out. A Warthog does five; an alley is not the
    place for it."""

    def __post_init__(self):
        x = max(x for x, _ in self.corners) + self.wheel_radius
        if self.travel >= 2 * x:
            raise ValueError(f"{self} slides past the ends of its chassis")
        if self.deck - BEAM <= FENDERS:
            raise ValueError(f"{self} has its frame in the Warthog's fenders")

    @property
    def corners(self) -> list[tuple[float, float]]:
        """Each wheel's `(x, y)` in the base frame, in `CORNERS` order."""
        return [
            tuple(float(v) for v in _placement(_warthog(), _child(joint))[:2, 3])
            for joint in WHEEL_JOINTS
        ]

    @property
    def track(self) -> float:
        """Wheel centre to wheel centre, across the machine."""
        return self.corners[0][1] - self.corners[1][1]

    @property
    def wheel_radius(self) -> float:
        """The radius of the Warthog's wheel colliders."""
        link = _child(WHEEL_JOINTS[0])
        return float(
            _warthog().find(f"link[@name='{link}']/collision/geometry/cylinder").get("radius")
        )

    @property
    def mass(self) -> float:
        """The Warthog alone, without the frame and the arm."""
        return sum(float(mass.get("value")) for mass in _warthog().iter("mass"))

    @property
    def max_wheel_speed(self) -> float:
        """rad/s at `max_speed`."""
        return self.max_speed / self.wheel_radius

    @property
    def drive_effort(self) -> float:
        """N m a wheel may exert. See `DRIVE_EFFORT_PER_KG`."""
        return DRIVE_EFFORT_PER_KG * self.mass

    @property
    def rail(self) -> tuple[float, float, float]:
        """Where the rail's top surface is, at mid-stroke, in the base frame."""
        return (0.0, self.rail_offset, self.deck + RAIL[1])

    @property
    def chain(self) -> tuple[Joint, ...]:
        """The slide and the arm, as `kinematics` reads them: seven joints from
        the base frame to the last wrist, `TOOL` still to come."""
        x, y, z = self.rail
        first = dataclasses.replace(UR10[0], xyz=(0.0, 0.0, self.carriage + UR10[0].xyz[2]))
        slide = Joint(
            SLIDE_JOINT,
            (x, y, z),
            axis=(1.0, 0.0, 0.0),
            prismatic=True,
            limits=(-self.travel / 2, self.travel / 2),
            effort=1000.0,
            velocity=0.5,
        )
        return (slide, first, *UR10[1:])

    def blade(self, angle: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """The moving blade's plane with the shear at `angle`, in the `HAND`
        frame, as the rectangle `Shears.crossing` takes round the blade: a
        corner and its two edges, across from the blade's back past its
        bowed edge, and along it. `Shear.covers` says which crossings the
        blade itself covers; a cane whose axis it covers is cut there."""
        s = self.shear
        plate = TOOL @ transform(rpy=(angle, 0.0, 0.0))
        corner = plate @ np.array([s.bypass, -s.width, 0.0, 1.0])
        across = plate[:3, :3] @ np.array([0.0, s.width + s.bow, 0.0])
        up = plate[:3, :3] @ np.array([0.0, 0.0, s.blade])
        return corner[:3], across, up

    def wedge(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """The mouth at full opening, in the `HAND` frame, as the rectangle
        `Shears.crossing` takes: in the moving plate's plane, from
        `Shear.reach` across to the fixed edge's deepest point and the
        blade's length out. `Shear.in_wedge` says where in the shear's frame
        a crossing of it lies."""
        s = self.shear
        corner = TOOL @ np.array([s.bypass, -s.reach, 0.0, 1.0])
        across = TOOL[:3, :3] @ np.array([0.0, s.reach + s.bow, 0.0])
        up = TOOL[:3, :3] @ np.array([0.0, 0.0, s.blade])
        return corner[:3], across, up

    @property
    def tool(self) -> np.ndarray:
        """The frame off the last wrist a target is posed for: the mouth, +Z
        along the blades and +X along the pivot -- which a cane has to lie
        along to be cut."""
        y, z = self.shear.mouth
        return TOOL @ transform((0.0, y, z))


def bumblebee_cfg(machine: Bumblebee, prim_path: str) -> ArticulationCfg:
    """An articulation spawning `machine`, converting its URDF on first use."""
    return ArticulationCfg(
        prim_path=prim_path,
        spawn=sim_utils.UrdfFileCfg(
            asset_path=_write_urdf(machine),
            usd_dir=str(_CACHE),
            fix_base=False,
            # Boxes, cylinders and the blades' convex pieces: a hull of each
            # is the shape itself.
            collision_type="Convex Hull",
            joint_drive=None,  # the actuators below own the gains
            activate_contact_sensors=False,
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            joint_pos=dict(zip(ARM_JOINTS, ARM_HOME, strict=True))
        ),
        actuators={
            "shoulder": ImplicitActuatorCfg(
                joint_names_expr=SHOULDER_JOINTS,
                stiffness=SHOULDER_STIFFNESS,
                damping=SHOULDER_DAMPING,
                effort_limit={joint.name: joint.effort for joint in UR10[:3]},
                armature=ARMATURE["shoulder"],
            ),
            "wrist": ImplicitActuatorCfg(
                joint_names_expr=WRIST_JOINTS,
                stiffness=WRIST_STIFFNESS,
                damping=WRIST_DAMPING,
                effort_limit=UR10[-1].effort,
                armature=ARMATURE["wrist"],
            ),
            "slide": ImplicitActuatorCfg(
                joint_names_expr=[SLIDE_JOINT],
                stiffness=SLIDE_STIFFNESS,
                damping=SLIDE_DAMPING,
                effort_limit=1000.0,
            ),
            "shear": ImplicitActuatorCfg(
                joint_names_expr=[SHEAR_JOINT],
                stiffness=SHEAR_STIFFNESS,
                damping=SHEAR_DAMPING,
                effort_limit=10.0,
                armature=ARMATURE["shear"],
            ),
            # Zero stiffness makes these velocity drives: the target is a
            # speed and `damping` is how hard they hold it.
            "drive": ImplicitActuatorCfg(
                joint_names_expr=WHEEL_JOINTS,
                stiffness=0.0,
                damping=DRIVE_DAMPING,
                effort_limit=machine.drive_effort,
            ),
        },
    )


##
# The Warthog.
##


@functools.cache
def warthog_folder() -> pathlib.Path:
    """The Warthog's folder, downloaded from `ASSETS` the first time."""
    from huggingface_hub import snapshot_download

    root = snapshot_download(
        ASSETS, repo_type="dataset", revision=ASSETS_REVISION, allow_patterns=f"{WARTHOG}/*"
    )
    return pathlib.Path(root) / WARTHOG


@functools.cache
def _warthog() -> ElementTree.Element:
    """The Warthog's URDF, its meshes found by absolute path. Its root,
    `base_link`, is between the axles at their height."""
    folder = warthog_folder()
    robot = ElementTree.parse(folder / "warthog.urdf").getroot()
    for mesh in robot.iter("mesh"):
        mesh.set("filename", str(folder / mesh.get("filename")))
    return robot


def _child(joint: str) -> str:
    return _warthog().find(f"joint[@name='{joint}']/child").get("link")


def _lift(m: Bumblebee) -> float:
    """How far `urdf` raises the Warthog's root, so its wheels stand on the
    ground."""
    return m.wheel_radius - _placement(_warthog(), _child(WHEEL_JOINTS[0]))[2, 3]


def _placement(robot: ElementTree.Element, link: str) -> np.ndarray:
    """`link`'s frame in that of the root of `robot`'s tree."""
    joint = robot.find(f"joint/child[@link='{link}']/..")
    if joint is None:
        return np.eye(4)
    origin = joint.find("origin")
    return _placement(robot, joint.find("parent").get("link")) @ transform(
        _numbers(origin.get("xyz")), _numbers(origin.get("rpy"))
    )


##
# URDF generation.
##

_CACHE = pathlib.Path(tempfile.gettempdir()) / "pruning_demo"
"""Where a generated robot is kept, named for its contents: two sizes are two
robots, and the URDF converter caches on the name too."""


def _write_urdf(machine: Bumblebee) -> str:
    """Build the URDF for one machine and return its path, writing it once."""
    text = urdf(machine)
    _CACHE.mkdir(parents=True, exist_ok=True)
    path = _CACHE / f"bumblebee_{hashlib.sha256(text.encode()).hexdigest()[:16]}.urdf"
    path.write_text(text)
    return str(path)


def urdf(machine: Bumblebee) -> str:
    """`machine` as a URDF document.

    `base_link` is the frame with its origin on the ground between the
    Warthog's wheels, the rail included, since nothing between them moves.
    The Warthog hangs off it, the carriage off the slide, and the arm stands
    on the carriage through a fixed joint the importer merges away.
    """
    m = machine
    x, y, z = m.rail
    px, py = POSTS
    mount = _lift(m) + _placement(_warthog(), MOUNT)[2, 3]
    rail_length = m.travel + 0.2
    beams = m.deck - BEAM / 2
    outer = y + RAIL[0] / 2
    frame = "\n".join(
        [
            *(
                _box(
                    (sx * px, sy * py, (mount + m.deck - BEAM) / 2),
                    (BEAM, BEAM, m.deck - BEAM - mount),
                    "aluminium",
                )
                for sx in (-1, 1)
                for sy in (-1, 1)
            ),
            # Across the deck and out to the rail, and along its far side.
            *(
                _box(
                    (sx * px, (outer - py - BEAM / 2) / 2, beams),
                    (BEAM, outer + py + BEAM / 2, BEAM),
                    "aluminium",
                )
                for sx in (-1, 1)
            ),
            _box((0.0, -py, beams), (2 * px - BEAM, BEAM, BEAM), "aluminium"),
            _box((x, y, z - RAIL[1] / 2), (rail_length, *RAIL), "steel"),
        ]
    )
    slide, *arm = m.chain
    return f"""<?xml version="1.0"?>
<!-- Generated by bumblebee.py; edit that, not this. -->
<robot name="bumblebee">
  <link name="base_link">
{frame}
    <inertial>
      <origin xyz="0 0 {(mount + m.deck) / 2}"/>
      <mass value="{FRAME_MASS}"/>
{_box_inertia(FRAME_MASS, rail_length, 2 * py, m.deck - mount)}
    </inertial>
  </link>
{_chassis(m)}
{_joint(slide, "base_link", "carriage")}
  <link name="carriage">
{_box((0.0, 0.0, m.carriage / 2), (0.15, 0.15, m.carriage), "steel")}
    <inertial>
      <origin xyz="0 0 {m.carriage / 2}"/>
      <mass value="5.0"/>
{_box_inertia(5.0, 0.15, 0.15, m.carriage)}
    </inertial>
  </link>
{_arm(m, arm)}
{_shear(m)}
</robot>
"""


def _chassis(m: Bumblebee) -> str:
    """The Warthog: its materials, links and joints as its URDF has them,
    raised so its wheels stand on the ground. Its rockers are fixed: on the
    real machine a differential bar holds the chassis level between them,
    which the URDF leaves out. Its wheels get the drive's limits."""
    robot = copy.deepcopy(_warthog())
    lift = _lift(m)
    for joint in robot.findall("joint"):
        if joint.get("type") == "revolute":
            joint.set("type", "fixed")
        elif joint.get("type") == "continuous":
            ElementTree.SubElement(
                joint, "limit", effort=str(m.drive_effort), velocity=str(m.max_wheel_speed)
            )
        if joint.find("parent").get("link") == "base_link":
            origin = joint.find("origin")
            x, y, z = _numbers(origin.get("xyz"))
            origin.set("xyz", f"{x} {y} {z + lift}")

    def carried(element: ElementTree.Element) -> str:
        """A link by its name, a joint by the link it carries."""
        return element.find("child").get("link") if element.tag == "joint" else element.get("name")

    # The Warthog's own `base_link` is empty: its chassis joint hangs off this
    # robot's instead.
    kept = [
        element
        for element in robot
        if element.tag in ("material", "link", "joint")
        and carried(element) not in ("base_link", CLEARED)
    ]
    return "\n".join("  " + ElementTree.tostring(e, encoding="unicode").rstrip() for e in kept)


def _arm(m: Bumblebee, joints: list[Joint]) -> str:
    """The UR10 on the carriage: its links as `UR10_URDF` draws and weighs
    them, its base renamed `ARM_BASE` and its meshes found by absolute path,
    joined by `joints`, the chain's."""
    parts = [
        f"""
  <joint name="arm_mount" type="fixed">
    <parent link="carriage"/>
    <child link="{ARM_BASE}"/>
    <origin xyz="0 0 {m.carriage}"/>
  </joint>"""
    ]
    names = [ARM_BASE, *_LINKS[1:]]
    for stock, name in zip(_LINKS, names, strict=True):
        link = copy.deepcopy(_UR10.find(f"link[@name='{stock}']"))
        link.set("name", name)
        for mesh in link.iter("mesh"):
            mesh.set("filename", str((UR10_URDF.parent / mesh.get("filename")).resolve()))
        parts.append("  " + ElementTree.tostring(link, encoding="unicode").rstrip())
    # The arm's first joint is `m.chain`'s, offset by the carriage the mount
    # above already climbs.
    first = dataclasses.replace(joints[0], xyz=(0.0, 0.0, joints[0].xyz[2] - m.carriage))
    for joint, parent, child in zip([first, *joints[1:]], names[:-1], names[1:], strict=True):
        parts.append(_joint(joint, parent, child))
    return "\n".join(parts)


def _shear(m: Bumblebee) -> str:
    """The shear head on the flange, and its one moving blade. The hook lies
    back from `Shear.edge` along +Y, the cutting blade along -Y in its own
    frame, so their edges meet along it at zero."""
    s = m.shear

    def hook(z):
        # Its back rounds off from the root into the tip of the claw.
        return s.edge(z), s.edge(z) + s.width * np.sqrt(1 - z / (s.blade + s.claw))

    def cutting(z):
        return s.edge(z), s.back(z)

    head = f"""
  <joint name="tool" type="fixed">
    <parent link="{HAND}"/>
    <child link="shear_link"/>
    <origin xyz="{TOOL[0, 3]} {TOOL[1, 3]} {TOOL[2, 3]}" rpy="{" ".join(str(a) for a in rpy_of(TOOL[:3, :3]))}"/>
  </joint>
  <link name="shear_link">
{_box((0.0, 0.0, -s.head / 2), (s.head, s.head, s.head), "steel")}
{_blade(s, 0.0, hook, s.blade + s.claw, s.slices + 1, 0.0, "coated", "hook")}
    <inertial>
      <origin xyz="0 0 {-s.head / 2}"/>
      <mass value="0.5"/>
{_box_inertia(0.5, s.head, s.head, s.head)}
    </inertial>
  </link>
  <joint name="{SHEAR_JOINT}" type="revolute">
    <parent link="shear_link"/>
    <child link="{BLADE}"/>
    <axis xyz="1 0 0"/>
    <limit lower="0" upper="{s.opening}" effort="10" velocity="6"/>
  </joint>
  <link name="{BLADE}">
{_blade(s, s.bypass, cutting, s.blade, s.slices, 1.5 * s.thickness, "polished", "blade")}
    <inertial>
      <origin xyz="0 0 {s.blade / 2}"/>
      <mass value="0.05"/>
{_box_inertia(0.05, s.thickness, s.width, s.blade)}
    </inertial>
  </link>"""
    return head


def _blade(
    s: Shear, x: float, outline, length: float, pieces: int, bevel: float, material: str, name: str
) -> str:
    """One blade, centred `x` along the pivot and `length` out from it, its
    `outline` giving its edge and its back across in Y at each Z. Drawn as a
    mesh, its face toward +X ground back `bevel` from the edge to a sharp
    one, if `bevel`; colliding as `pieces` convex prisms end to end along
    it, each the blade between two points evenly spaced along Z, with the
    chord of the edge for its edge. Each piece's mesh is about its own
    middle, placed there by its origin, so its pose says where along the
    blade it is."""
    low, high = x - s.thickness / 2, x + s.thickness / 2

    def sections(z, corners):
        """A polygon across the blade at each of `z`, from `corners`: each an
        x, and its y at each of `z`."""
        return np.stack([np.column_stack([np.full_like(z, a), b, z]) for a, b in corners], axis=1)

    z = np.linspace(0.0, length, 41)
    edge, back = outline(z)
    ground = edge - np.sign(edge - back) * np.minimum(bevel, np.abs(edge - back))
    # The edge is a twentieth of the plate thick where it is ground, the
    # plate's whole thickness where it is not.
    land = low + s.thickness / 20 if bevel else high
    drawn = sections(z, [(low, back), (low, edge), (land, edge), (high, ground), (high, back)])
    mesh = '      <geometry><mesh filename="{}"/></geometry>'
    parts = [_shaped(mesh.format(_stl(_loft(drawn), name)), ("visual",), material)]
    z = np.linspace(0.0, length, pieces + 1)
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


def _joint(joint: Joint, parent: str, child: str) -> str:
    kind = "prismatic" if joint.prismatic else "revolute"
    return f"""
  <joint name="{joint.name}" type="{kind}">
    <parent link="{parent}"/>
    <child link="{child}"/>
    <origin xyz="{" ".join(str(v) for v in joint.xyz)}" rpy="{" ".join(str(v) for v in joint.rpy)}"/>
    <axis xyz="{" ".join(str(v) for v in joint.axis)}"/>
    <limit lower="{joint.limits[0]}" upper="{joint.limits[1]}" effort="{joint.effort}" velocity="{joint.velocity}"/>
  </joint>"""


def _box(xyz, size, material: str, tags: tuple[str, ...] = ("visual", "collision")) -> str:
    shape = (
        f'      <origin xyz="{xyz[0]} {xyz[1]} {xyz[2]}"/>\n'
        f'      <geometry><box size="{size[0]} {size[1]} {size[2]}"/></geometry>'
    )
    return _shaped(shape, tags, material)


def _shaped(shape: str, tags: tuple[str, ...], material: str) -> str:
    """One geometry repeated as each of `tags`, coloured where it is a visual."""
    colour = f'\n      <material name="{material}"><color rgba="{MATERIALS[material]}"/></material>'
    return "\n".join(
        f"    <{tag}>\n{shape}{colour if tag == 'visual' else ''}\n    </{tag}>" for tag in tags
    )


def _box_inertia(mass: float, x: float, y: float, z: float) -> str:
    return _inertia(
        mass * (y * y + z * z) / 12, mass * (x * x + z * z) / 12, mass * (x * x + y * y) / 12
    )


def _inertia(ixx: float, iyy: float, izz: float) -> str:
    return (
        f'      <inertia ixx="{ixx:.6f}" iyy="{iyy:.6f}" izz="{izz:.6f}" ixy="0" ixz="0" iyz="0"/>'
    )

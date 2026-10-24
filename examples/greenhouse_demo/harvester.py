"""A pipe-rail truss harvester: a trolley, a lift, a UR5e and a cutter.

The layout of eternal.ag's Harvester and most robots of its kind: a trolley
the width of a path's heating pipes, a lift on it, a cobot arm on the lift
and a cutter in the arm's hand. Here the trolley does not roll. It rides
three prismatic joints off a base fixed in the main path: `AISLE` across the
row heads to a path, `RAIL` down it and `LIFT` up. The trolley, the lift and
the cutter are primitives, every dimension a field of `Harvester`; the arm
is MuJoCo Menagerie's model of Universal Robots' UR5e, fetched from `ASSETS`
on first use and read out of its MJCF, so its links are drawn and weighed
as Menagerie has them. `urdf` writes the robot, and `Harvester.chain` is the
same joints for `kinematics`.

The cutter is a head on the flange and two blades fixed open in a V in
front of it. Only the head collides: a truss is cut by rule, with
`Shears`, once the V's crotch is at its peduncle, and the blades let the
piece fall rather than catch it.

The robot collides with itself. Its colliders are the trolley's chassis and
mast, Menagerie's capsules on the arm and the cutter's head; the platform has
none, the arm's base standing flush on it. The arm's and the cutter's are
each a `Shape`: `urdf` writes it for Isaac Sim's importer, and `curobo_cfg`
hands cuRobo the spheres that cover it, so a pose cuRobo finds clear is clear
in the simulation. The trolley's reach cuRobo with the rest of the world, from
the simulation itself (see `planner`).
"""

from __future__ import annotations

import dataclasses
import functools
import hashlib
import itertools
import math
import pathlib
import tempfile
import xml.etree.ElementTree as ElementTree

import numpy as np

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg

import kinematics
from kinematics import Joint, rpy_of, transform

ASSETS = "azazdeaz/misina-lab-sim-assets"
"""The Hugging Face dataset the demos' larger assets live in. `huggingface_hub`
fetches what a demo needs from it on first use and caches it."""

ASSETS_REVISION = "ce53079b13989d0d1fdefde214fc617b1c08468e"
"""The dataset commit the demo is pinned to."""

UR5E = "universal_robots_ur5e"
"""MuJoCo Menagerie's UR5e, as google-deepmind/mujoco_menagerie@71f066a
publishes it: `ur5e.xml`, its meshes and its BSD-3-Clause licence."""

AISLE, RAIL, LIFT = "aisle", "rail", "lift"
"""The trolley's joints: across the main path, down a path, up."""

HAND = "wrist_3_link"
"""The body the cutter is fixed to, once the importer merges the fixed joint
between them, and whose pose says where the mouth is."""
CUTTER = "cutter"
"""The cutter's link in the URDF, at the flange, +Z out between the blades."""

ARM_HOME = (0.0, -math.pi / 2, math.pi / 2, -math.pi / 2, -math.pi / 2, 0.0)
"""Joint positions the arm rides folded to: the upper arm upright and the
forearm level back over the trolley, the cutter hanging down inside its
width."""

# -- Actuation: position drives on every joint.
TRAVEL = {AISLE: (1e5, 1.2e4, 1e4), RAIL: (1e5, 1.2e4, 1e4), LIFT: (2e5, 6e3, 5e3)}
"""Stiffness, damping and effort limit of each trolley joint, in N/m, N s/m
and N: about critically damped on the 400 kg the aisle and the rail carry and
the 50 the lift does, which holds the lift's load to a few millimetres of
sag."""
SHOULDER_STIFFNESS = 6000.0
SHOULDER_DAMPING = 600.0
"""The three joints carrying the arm's weight, three times Menagerie's gain:
the 60 N m an outstretched arm hangs on the shoulder then sags the mouth
under a centimetre."""
WRIST_STIFFNESS = 500.0
WRIST_DAMPING = 100.0
"""Menagerie's gains for the wrists."""
ARMATURE = 0.1
"""Reflected rotor inertia on each arm joint, in kg m^2: Menagerie's."""

HEAD = 0.06
"""How far the cutter's head stands out of the flange, in meters."""
HEAD_WIDTH = 0.05
"""The head across, either way."""
BLADE = 0.07
"""Length of a blade, out from the crotch of the V."""
OPENING = 0.7
"""Angle between the blades, in radians."""
MOUTH = 0.012
"""How far into the V from its crotch the mouth is: where a peduncle is cut."""
BLADE_THICKNESS, BLADE_WIDTH = 0.003, 0.015
"""A blade's plate: through it, and across from its inner, cutting edge."""

GRAIN = 0.015
"""The thinnest a box is cut into for its spheres, in meters: a blade is
covered a blade's width across, not its thickness."""

MATERIALS = {
    "chassis": "0.18 0.45 0.25 1",
    "steel": "0.55 0.57 0.60 1",
    "tire": "0.05 0.05 0.05 1",
}
"""Colours, as URDF rgba: a green trolley, galvanized steel and rubber."""


@dataclasses.dataclass(frozen=True)
class Shape:
    """A solid in its link's frame: a capsule or a cylinder along its z,
    `size` its radius and half length, or a box, `size` its extents."""

    kind: str
    xyz: tuple[float, ...]
    size: tuple[float, ...]
    rpy: tuple[float, ...] = (0.0, 0.0, 0.0)

    def geometry(self) -> str:
        """The URDF geometry of a box or a cylinder."""
        if self.kind == "box":
            return f'<box size="{" ".join(str(float(v)) for v in self.size)}"/>'
        return f'<cylinder radius="{self.size[0]}" length="{2 * self.size[1]}"/>'

    def collision(self) -> str:
        """As URDF `collision` elements. URDF has no capsule: one is written as
        its cylinder and a sphere on each end, which make it up exactly."""
        parts = [_tagged("collision", _origin(self.xyz, self.rpy), self.geometry())]
        if self.kind == "capsule":
            frame = transform(self.xyz, self.rpy)
            ball = f'<sphere radius="{self.size[0]}"/>'
            for end in (-self.size[1], self.size[1]):
                centre = frame[:3, :3] @ (0.0, 0.0, end) + frame[:3, 3]
                parts.append(_tagged("collision", _origin(centre, (0, 0, 0)), ball))
        return "\n".join(parts)

    def cover(self) -> list[tuple[np.ndarray, float]]:
        """Spheres whose union holds the shape, each a centre in the link's
        frame and a radius: along the axis of a capsule or a cylinder, at most
        its radius apart, and at the middle of each cell of a box cut into
        cells no thinner than `GRAIN`."""
        frame = transform(self.xyz, self.rpy)
        if self.kind == "box":
            size = np.array(self.size, dtype=float)
            counts = np.ceil(size / max(size.min(), GRAIN)).astype(int)
            cell = size / counts
            centres = [cell * (np.array(i) + 0.5) - size / 2 for i in np.ndindex(*counts)]
            radius = float(np.linalg.norm(cell)) / 2
        else:
            radius, half = self.size
            count = math.ceil(2 * half / radius) + 1
            centres = [np.array([0.0, 0.0, z]) for z in np.linspace(-half, half, count)]
            # A point midway between two centres is the farthest from either.
            radius = math.hypot(radius, half / (count - 1)) if count > 1 else radius
        return [(frame[:3, :3] @ centre + frame[:3, 3], radius) for centre in centres]


CUTTER_HEAD = Shape("box", (0.0, 0.0, HEAD / 2), (HEAD_WIDTH, HEAD_WIDTH, HEAD))
"""The cutter's head, standing out of the flange: the part of it that collides."""


@dataclasses.dataclass(frozen=True)
class Harvester:
    """One machine, in meters and kilograms."""

    gauge: float = 0.6
    """Pipe centre to pipe centre: the rail's, which sets the trolley's width."""

    rail_top: float = 0.1755
    """Top of the pipes above the floor, which the wheels stand on."""

    length: float = 1.9
    """The trolley, end to end."""

    deck: float = 0.54
    """The lift's platform above the floor, lowered."""

    lift: float = 1.2
    """How far the platform rises."""

    mass: float = 350.0
    """The trolley and its lift, without the arm."""

    across: tuple[float, float] = (-6.0, 6.0)
    """Where `AISLE` goes, across the main path."""

    down: float = 30.0
    """How far `RAIL` goes, out of the main path and down a path."""

    @property
    def width(self) -> float:
        """The trolley across, over its wheels."""
        return self.gauge + 0.14

    @property
    def chain(self) -> tuple[Joint, ...]:
        """The trolley's joints and the arm's, as `kinematics` reads them: nine
        from the base frame, on the main path's floor, to the last wrist,
        `tool` still to come."""
        arm = _ur5e()
        # The arm's base stands on the platform, turned as Menagerie turns
        # it, and its first joint is off the base.
        first = transform(rpy=arm.mount) @ transform(arm.joints[0].xyz, arm.joints[0].rpy)
        return (
            Joint(AISLE, (0.0, 0.0, 0.0), axis=(0, 1, 0), prismatic=True, limits=self.across),
            Joint(RAIL, (0.0, 0.0, 0.0), axis=(1, 0, 0), prismatic=True, limits=(0, self.down)),
            Joint(LIFT, (0, 0, self.deck), axis=(0, 0, 1), prismatic=True, limits=(0, self.lift)),
            dataclasses.replace(arm.joints[0], xyz=tuple(first[:3, 3]), rpy=rpy_of(first[:3, :3])),
            *arm.joints[1:],
        )

    @property
    def tool(self) -> np.ndarray:
        """The mouth's frame off the last wrist: +Z out between the blades,
        +X across the V, along which a peduncle has to lie to be cut."""
        return _ur5e().flange @ transform((0.0, 0.0, HEAD + MOUTH))


def arm_joints() -> list[str]:
    """The arm's joint names, from its base out."""
    return [joint.name for joint in _ur5e().joints]


def arm_links() -> list[str]:
    """The arm's moving links, from its base out."""
    return list(_ur5e().bodies[1:])


def spheres() -> dict[str, list[dict]]:
    """The arm's and the cutter's shapes as the spheres cuRobo collides, by
    link: each one's `Shape.cover`. The blades are covered too: the
    simulation lets a cut truss fall through them, and the planner keeps them
    off the crop all the same."""
    shapes = {**_ur5e().shapes, CUTTER: (CUTTER_HEAD, *blades())}
    return {
        link: [
            {"center": centre.tolist(), "radius": radius}
            for shape in solids
            for centre, radius in shape.cover()
        ]
        for link, solids in shapes.items()
    }


def curobo_cfg(machine: Harvester) -> dict:
    """The arm as cuRobo's robot config: from the platform out to the cutter,
    the tool it poses, colliding as `spheres`. Every pair of links is checked
    against the other but a joint's two and the pairs whose spheres meet at
    `ARM_HOME`: the last wrist spins the cutter about an axis the wrist before
    it shares, so those two meet however the arm is posed."""
    covers = spheres()
    arm = _ur5e()
    frames = kinematics.frames(machine.chain, [0.0, 0.0, 0.0, *ARM_HOME])
    placed = dict(zip(arm.bodies[1:], frames[3:], strict=True))
    placed[CUTTER] = placed[HAND] @ arm.flange

    def balls(link: str) -> tuple[np.ndarray, np.ndarray]:
        centres = np.array([ball["center"] for ball in covers[link]])
        radii = np.array([ball["radius"] for ball in covers[link]])
        return centres @ placed[link][:3, :3].T + placed[link][:3, 3], radii

    chain = [*arm.bodies[1:], CUTTER]
    ignore = {link: [after] for link, after in zip(chain, chain[1:])}
    for a, b in itertools.combinations(covers, 2):
        (ca, ra), (cb, rb) = balls(a), balls(b)
        gaps = np.linalg.norm(ca[:, None] - cb[None], axis=-1) - ra[:, None] - rb[None]
        if gaps.min() < 0 and b not in ignore.get(a, []):
            ignore.setdefault(a, []).append(b)
    return {
        "kinematics": {
            "urdf_path": write_urdf(machine),
            "base_link": "platform",
            "tool_frames": [CUTTER],
            "collision_link_names": list(covers),
            "collision_spheres": covers,
            "self_collision_ignore": ignore,
            "self_collision_buffer": {},
            "cspace": {
                "joint_names": arm_joints(),
                "default_joint_position": list(ARM_HOME),
                "cspace_distance_weight": [1.0] * len(ARM_HOME),
                "null_space_weight": [1.0] * len(ARM_HOME),
                # Every joint but the elbow turns two full turns; cut to one,
                # the planner never reaches for a pose a turn away. The tenth
                # of a radian more keeps a pose the arm overshoots in range.
                "position_limit_clip": [math.pi + 0.1, math.pi + 0.1, 0.1] + [math.pi + 0.1] * 3,
            },
        }
    }


def harvester_cfg(machine: Harvester, prim_path: str, position) -> ArticulationCfg:
    """An articulation spawning `machine` with its base at `position`,
    converting its URDF on first use."""
    arm = _ur5e().joints
    return ArticulationCfg(
        prim_path=prim_path,
        spawn=sim_utils.UrdfFileCfg(
            asset_path=write_urdf(machine),
            usd_dir=str(_CACHE),
            fix_base=True,
            joint_drive=None,  # the actuators below own the gains
            activate_contact_sensors=False,
            self_collision=True,
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=tuple(position), joint_pos=dict(zip(arm_joints(), ARM_HOME, strict=True))
        ),
        actuators={
            **{
                joint: ImplicitActuatorCfg(
                    joint_names_expr=[joint], stiffness=k, damping=d, effort_limit=effort
                )
                for joint, (k, d, effort) in TRAVEL.items()
            },
            "shoulder": ImplicitActuatorCfg(
                joint_names_expr=[joint.name for joint in arm[:3]],
                stiffness=SHOULDER_STIFFNESS,
                damping=SHOULDER_DAMPING,
                effort_limit={joint.name: joint.effort for joint in arm[:3]},
                armature=ARMATURE,
            ),
            "wrist": ImplicitActuatorCfg(
                joint_names_expr=[joint.name for joint in arm[3:]],
                stiffness=WRIST_STIFFNESS,
                damping=WRIST_DAMPING,
                effort_limit={joint.name: joint.effort for joint in arm[3:]},
                armature=ARMATURE,
            ),
        },
    )


##
# The UR5e, out of Menagerie's MJCF.
##


@dataclasses.dataclass(frozen=True)
class _Arm:
    joints: tuple[Joint, ...]
    """Body i to body i + 1 of `bodies`, as URDF writes a joint."""
    bodies: tuple[str, ...]
    """Menagerie's bodies from the base out, which name the links."""
    links: tuple[str, ...]
    """Each body as a URDF link: Menagerie's meshes, colliding as its
    capsules."""
    shapes: dict[str, tuple[Shape, ...]]
    """Each body's capsules, and the one cylinder on the last wrist."""
    flange: np.ndarray
    """The frame off the last wrist a tool bolts on, +Z out of the hand."""
    mount: tuple[float, float, float]
    """The turn the base stands at, as rpy."""


@functools.cache
def ur5e_folder() -> pathlib.Path:
    """Menagerie's UR5e folder, downloaded from `ASSETS` the first time."""
    from huggingface_hub import snapshot_download

    root = snapshot_download(
        ASSETS, repo_type="dataset", revision=ASSETS_REVISION, allow_patterns=f"{UR5E}/*"
    )
    return pathlib.Path(root) / UR5E


@functools.cache
def _ur5e() -> _Arm:
    """The UR5e, read out of Menagerie's MJCF."""
    folder = ur5e_folder()
    mjcf = ElementTree.parse(folder / "ur5e.xml").getroot()
    classes = _classes(mjcf.find("default"))
    colours = {m.get("name"): m.get("rgba") for m in mjcf.iter("material")}
    meshes = folder / mjcf.find("compiler").get("meshdir")
    actuated = {drive.get("joint"): drive for drive in mjcf.iter("general")}
    base = mjcf.find("worldbody/body")

    def attrs(element: ElementTree.Element, cls: str) -> dict[str, str]:
        """An element's attributes, its default class's under them."""
        return {**classes[element.get("class", cls)][element.tag], **element.attrib}

    joints, bodies, links, shapes = [], [], [], {}
    for body, cls in _bodies(base, "main"):
        bodies.append(body.get("name"))
        if (joint := body.find("joint")) is not None:
            spec = attrs(joint, cls)
            xyz, rpy = _placed(body)
            joints.append(
                Joint(
                    spec["name"],
                    xyz,
                    rpy,
                    _numbers(spec["axis"]),
                    limits=_numbers(spec["range"]),
                    effort=_numbers(attrs(actuated[spec["name"]], cls)["forcerange"])[1],
                    # UR's figure for every e-Series joint: 180 degrees a second.
                    velocity=math.pi,
                )
            )
        parts, solids = [], []
        for geom in body.findall("geom"):
            spec = attrs(geom, cls)
            xyz, rpy = _placed(geom)
            if spec.get("type") == "mesh":
                mesh = f'<mesh filename="{meshes / spec["mesh"]}.obj"/>'
                material = spec["material"]
                parts.append(
                    _tagged("visual", _origin(xyz, rpy), mesh, (material, colours[material]))
                )
            else:
                solids.append(Shape(spec["type"], xyz, _numbers(spec["size"]), rpy))
        parts += [solid.collision() for solid in solids]
        if solids:
            shapes[body.get("name")] = tuple(solids)
        inertial = body.find("inertial")
        parts.append(
            _inertial(
                float(inertial.get("mass")),
                _numbers(inertial.get("diaginertia")),
                *_placed(inertial),
            )
        )
        links.append(f'  <link name="{body.get("name")}">\n' + "\n".join(parts) + "\n  </link>")

    site = mjcf.find(".//site[@name='attachment_site']")
    return _Arm(
        tuple(joints),
        tuple(bodies),
        tuple(links),
        shapes,
        transform(*_placed(site)),
        _placed(base)[1],
    )


def _bodies(body: ElementTree.Element, cls: str):
    """`body` and every body under it, root first, each with the default class
    its elements take when they name none."""
    cls = body.get("childclass", cls)
    yield body, cls
    for child in body.findall("body"):
        yield from _bodies(child, cls)


def _classes(default: ElementTree.Element, inherited=None, out=None) -> dict:
    """Every MJCF default class by name, as the attributes it gives each
    element tag, its parents' included."""
    out = {} if out is None else out
    merged = {tag: dict(each) for tag, each in (inherited or {}).items()}
    for tag in ("joint", "geom", "general"):
        merged.setdefault(tag, {})
    for child in default:
        if child.tag != "default":
            merged.setdefault(child.tag, {}).update(child.attrib)
    out[default.get("class", "main")] = merged
    for child in default.findall("default"):
        _classes(child, merged, out)
    return out


def _placed(element: ElementTree.Element) -> tuple[tuple[float, ...], tuple[float, float, float]]:
    """An MJCF element's `pos` and (w, x, y, z) `quat` as URDF's xyz and rpy."""
    quat = np.array(_numbers(element.get("quat", "1 0 0 0")))
    w, x, y, z = quat / np.linalg.norm(quat)
    rotation = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )
    return _numbers(element.get("pos", "0 0 0")), rpy_of(rotation)


def _numbers(text: str) -> tuple[float, ...]:
    return tuple(float(v) for v in text.split())


##
# URDF generation.
##

_CACHE = pathlib.Path(tempfile.gettempdir()) / "greenhouse_demo"
"""Where a generated robot is kept, named for its contents: two sizes are two
robots, and the URDF converter caches on the name too."""


def write_urdf(machine: Harvester) -> str:
    """Build the URDF for one machine and return its path, writing it once."""
    text = urdf(machine)
    _CACHE.mkdir(parents=True, exist_ok=True)
    path = _CACHE / f"harvester_{hashlib.sha256(text.encode()).hexdigest()[:16]}.urdf"
    path.write_text(text)
    return str(path)


def urdf(machine: Harvester) -> str:
    """`machine` as a URDF document.

    `base_link` is fixed on the main path's floor. The aisle carriage rides
    across it, the trolley down a path off the carriage, and the platform up
    the trolley's mast; the arm stands on the platform through a fixed joint
    the importer merges away, and the cutter on the last wrist through
    another. The trolley's chassis and mast collide, the arm and the
    cutter's head; the platform, the wheels and the blades are drawn only.
    """
    m = machine
    aisle, rail, lift, *_ = m.chain
    arm = _ur5e()
    # The flanged wheels, a pair on each pipe.
    wheel = 0.05
    wheels = "\n".join(
        _cylinder(
            (x * (m.length / 2 - 0.2), y * m.gauge / 2, m.rail_top + wheel),
            (math.pi / 2, 0.0, 0.0),
            wheel,
            0.04,
            "tire",
        )
        for x in (-1, 1)
        for y in (-1, 1)
    )
    bottom = m.rail_top + 0.07
    chassis = m.deck - 0.04 - bottom
    mast = m.lift + 0.3
    rear = -m.length / 2 + 0.1
    front = 0.25
    joints = "".join(
        _joint(joint, parent, child)
        for joint, parent, child in zip(arm.joints, arm.bodies[:-1], arm.bodies[1:], strict=True)
    )
    return f"""<?xml version="1.0"?>
<!-- Generated by harvester.py; edit that, not this. -->
<robot name="harvester">
  <link name="base_link">
{_inertial(1.0, (0.01, 0.01, 0.01))}
  </link>
{_joint(aisle, "base_link", "carriage")}
  <link name="carriage">
{_inertial(10.0, _box_inertia(10.0, 0.3, 0.3, 0.1))}
  </link>
{_joint(rail, "carriage", "trolley")}
  <link name="trolley">
{_box((0.0, 0.0, bottom + chassis / 2), (m.length, m.width, chassis), "chassis", BOTH)}
{wheels}
{_box((rear, 0.0, m.deck + mast / 2), (0.12, 0.2, mast), "steel", BOTH)}
{_inertial(m.mass, _box_inertia(m.mass, m.length, m.width, chassis), (0, 0, bottom + chassis / 2))}
  </link>
{_joint(lift, "trolley", "platform")}
  <link name="platform">
{_box(((rear + front) / 2, 0.0, -0.02), (front - rear, 0.5, 0.04), "steel")}
{_inertial(25.0, _box_inertia(25.0, front - rear, 0.5, 0.04), ((rear + front) / 2, 0, -0.02))}
  </link>
  <joint name="arm_mount" type="fixed">
    <parent link="platform"/>
    <child link="{arm.bodies[0]}"/>
    {_origin((0.0, 0.0, 0.0), arm.mount)}
  </joint>
{chr(10).join(arm.links)}
{joints}
  <joint name="cutter_mount" type="fixed">
    <parent link="{HAND}"/>
    <child link="{CUTTER}"/>
    {_origin(tuple(arm.flange[:3, 3]), rpy_of(arm.flange[:3, :3]))}
  </joint>
  <link name="{CUTTER}">
{_tagged("visual", _origin(CUTTER_HEAD.xyz, CUTTER_HEAD.rpy), CUTTER_HEAD.geometry(), STEEL)}
{CUTTER_HEAD.collision()}
{chr(10).join(_tagged("visual", _origin(b.xyz, b.rpy), b.geometry(), STEEL) for b in blades())}
{_inertial(0.4, _box_inertia(0.4, HEAD_WIDTH, HEAD_WIDTH, HEAD), (0, 0, HEAD / 2))}
  </link>
</robot>
"""


def blades() -> tuple[Shape, Shape]:
    """The two blades, plates standing out of the head's face in the
    cutter's YZ plane, `OPENING` apart: the V a peduncle lying along X is
    cut in."""
    plates = []
    for side in (-1, 1):
        angle = side * OPENING / 2
        # Out along the blade from the crotch, and back from its inner edge.
        along = np.array([0.0, -math.sin(angle), math.cos(angle)])
        back = np.array([0.0, math.cos(angle), math.sin(angle)]) * side
        centre = np.array([0.0, 0.0, HEAD]) + along * BLADE / 2 + back * BLADE_WIDTH / 2
        size = (BLADE_THICKNESS, BLADE_WIDTH, BLADE)
        plates.append(Shape("box", tuple(centre), size, (angle, 0.0, 0.0)))
    return tuple(plates)


def _joint(joint: Joint, parent: str, child: str) -> str:
    kind = "prismatic" if joint.prismatic else "revolute"
    return f"""
  <joint name="{joint.name}" type="{kind}">
    <parent link="{parent}"/>
    <child link="{child}"/>
    {_origin(joint.xyz, joint.rpy)}
    <axis xyz="{" ".join(str(v) for v in joint.axis)}"/>
    <limit lower="{joint.limits[0]}" upper="{joint.limits[1]}" effort="{joint.effort}" velocity="{joint.velocity}"/>
  </joint>"""


def _origin(xyz, rpy) -> str:
    return (
        f'<origin xyz="{" ".join(str(float(v)) for v in xyz)}"'
        f' rpy="{" ".join(str(float(v)) for v in rpy)}"/>'
    )


BOTH = ("visual", "collision")
STEEL = ("steel", MATERIALS["steel"])


def _box(xyz, size, material: str, tags: tuple[str, ...] = ("visual",)) -> str:
    geometry = f'<box size="{size[0]} {size[1]} {size[2]}"/>'
    return "\n".join(
        _tagged(tag, _origin(xyz, (0, 0, 0)), geometry, (material, MATERIALS[material]))
        for tag in tags
    )


def _cylinder(xyz, rpy, radius: float, length: float, material: str) -> str:
    geometry = f'<cylinder radius="{radius}" length="{length}"/>'
    return _tagged("visual", _origin(xyz, rpy), geometry, (material, MATERIALS[material]))


def _tagged(tag: str, origin: str, geometry: str, colour: tuple[str, str] | None = None) -> str:
    """One geometry as a `visual` or `collision`; a visual is coloured
    `colour`, a material's name and rgba."""
    material = ""
    if colour and tag == "visual":
        material = f'\n      <material name="{colour[0]}"><color rgba="{colour[1]}"/></material>'

    return f"    <{tag}>\n      {origin}\n      <geometry>{geometry}</geometry>{material}\n    </{tag}>"


def _inertial(mass: float, inertia, xyz=(0, 0, 0), rpy=(0, 0, 0)) -> str:
    ixx, iyy, izz = inertia
    return (
        f'    <inertial>\n      {_origin(xyz, rpy)}\n      <mass value="{mass}"/>\n'
        f'      <inertia ixx="{ixx:.6g}" iyy="{iyy:.6g}" izz="{izz:.6g}" ixy="0" ixz="0" iyz="0"/>\n'
        "    </inertial>"
    )


def _box_inertia(mass: float, x: float, y: float, z: float) -> tuple[float, float, float]:
    return mass * (y * y + z * z) / 12, mass * (x * x + z * z) / 12, mass * (x * x + y * y) / 12

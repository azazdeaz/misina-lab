"""Harvest a greenhouse's ripe trusses with a pipe-rail robot, from ground truth.

Two short rows whose lowest trusses are rods Newton's VBD bends, and the
`harvester` in the main path at their heads. It crosses to each path in
turn and works down it and back: every due truss on its left going in, every
one on its right coming out. Each is cut at its peduncle, by its stem, held
by it, pulled out of the crop into the path, carried to the crate on the
lift and lowered into it where what lies there is lowest, then let go.

What is due and where to cut it come from the scene's own ground truth, not
from a camera. A truss is due once its least ripe fruit is turning, as the
generator stages them, and is cut by its `Cut` frame. For each, the arm folds
and the trolley runs to a station back from the cut on the rail and the lift.
cuRobo plans the arm from there, around the simulation's own colliders (see
`planner`), to a stand-off and in on a straight line, and once it holds
still the shear closes: `shear.Stroke` cuts the peduncle where the blade
reaches it, and the jaws take the truss as it is cut.

    uv run harvest.py                        # in Kit's window, running on once done
    HEADLESS=1 uv run harvest.py --viz kit   # without one, printing the tally
"""

from __future__ import annotations

import argparse
import dataclasses
import math
import re
from collections.abc import Iterator

import numpy as np
import torch

import isaaclab.sim as sim_utils
from isaaclab.app import add_launcher_args, launch_simulation
from isaaclab.assets import AssetBaseCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import instantiate
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

from greenhouse_lab.isaaclab import GreenhouseCfg, LayoutCfg, StemCfg
from harvester import (
    AISLE,
    ARM_HOME,
    HAND,
    LIFT,
    RAIL,
    Crate,
    Harvester,
    arm_joints,
    harvester_cfg,
)
from kinematics import frames, rotation, solve, transform
from misina_lab.isaaclab import ColliderView, rods
from shear import SHEAR_JOINT, Stroke

BELOW = 1.8
"""Height, in meters, below which a truss's node puts it on a rod: the band
the arm works, and the trusses a cut can drop."""

GREENHOUSE_CFG = GreenhouseCfg(
    layout=LayoutCfg(rows=2, row_length=4.0),
    # Skinned: each rod is drawn by one mesh bending with it, which keeps the
    # rachis's zigzag and its knots, where a tube per segment runs straight.
    # Deleafed up to the truss above the one picked: the lowest leaves join
    # the stem about 40 cm above a due truss, where the camera sees them.
    stem=StemCfg(flexible_below=BELOW, skinned=True, deleafed_to_truss=2),
)

GREENHOUSE_PATH = "/World/Greenhouse"
# Where `design_scene` puts the robot: its scene's one environment.
ROBOT_PATH = "/World/envs/env_0/Robot"
ROBOT_CONTACT = [rf"{ROBOT_PATH}/.*((forearm|wrist_[123]|blade|finger)_link|platform)"]
"""The parts of the robot a truss meets: the forearm, the wrists, the
shear's head, hook and jaw riding on the last, the cutting blade and the
finger on it, and the platform, whose colliders are the crate's. Each costs
a proxy in the solver that bends the trusses."""

SIM_DT = 0.005
SUBSTEPS = 4
"""Four substeps of the 1.25 ms the rod tuning was measured at."""
DECIMATION = 4
"""Physics steps per control tick: the robot is driven at 50 Hz."""

STAGES = (
    "bud",
    "flower",
    "set",
    "expanding",
    "mature_green",
    "breaker",
    "turning",
    "pink",
    "light_red",
    "red",
)
"""A fruit's `greenhouse_lab:stage`, from bud and flower to red."""
DUE = STAGES.index("turning")

STANDOFF = 0.12
"""How far short of the cut the straight line in starts, back the way the
mouth comes in (`Harvester.way_in`)."""
BACK = 0.3
"""How far back from the cut along the rail, against the way the shear comes
in, the arm's base is stationed."""
RISE = 0.15
"""How far above the shoulder the cut is held, where the lift reaches."""
TILTS = np.radians([0.0, 20.0, -20.0, 30.0, -30.0])
"""How far off level the shear may point, turned about the blades' normal,
square to the stem. Level, a standing stem runs behind the hook straight
across the mouth's height and nowhere nearer the head; tilted, it runs
toward the head, the nearer the further it is tilted."""
YAWS = np.radians([0.0, 25.0])
"""How far the shear and its blades' plane may be turned about the stem
toward the path, off coming in along the row: turned, the shear comes in
partly from the path, its wrist further out of the crop wall. The blades'
plane still runs along the stem, and crosses the peduncle aslant."""
STEPS = np.array([0.02, 0.02, 0.01])
"""How far each trolley joint's target may move per control tick on a run,
every joint pacing itself to arrive with the slowest: 1 m/s across and down a
path, 0.5 m/s up."""
ARRIVED = 0.05
"""How close every joint has to be to its goal, in radians or meters."""
STILL = 0.002
"""How little every joint may still move per control tick for the robot to
count as settled: a tenth of a radian a second."""
PAST = 0.005
"""How far past its rod's first joint, by the stem, the blade crosses a
peduncle, in meters. `Shears.cut` makes a cut within a centimetre of a
joint at the joint, so this one drops the whole truss, and the piece the
jaws hold runs straight from the cut to the next joint."""
ASKEW = 45.0
"""How far off the way its `Cut` was authored a peduncle may run for the
shear to be squared to it as it hangs, in degrees, as far as its stem lets
it be."""
TRIES = 2
"""How many times a truss's cut is tried before it is given up on: a shear
that closed on nothing opens, backs out and plans the cut afresh."""
PULL = 0.2
"""How far a truss cut and held is pulled out across the path, from the
stand-off, before it is carried to the crate: out of the crop wall, and
short of the trolley."""
CARRY = 0.42
"""How high above the crate's floor the jaws carry a truss to it, in
meters: the truss hanging from them clears the crate's rim."""
SPOTS = (0.0, 0.12, -0.12)
"""Where along the crate from its middle the jaws may lower a truss, in
meters, the middle first. The middle is straight behind the shoulder, at
the end of its range, and now and then out of reach. A truss let go falls
over from about 15 cm up, which from a spot off the crate's middle line
across it would reach past the 18.5 cm to the side wall."""
LOWER = 0.15
"""How fast the jaws lower a truss into the crate, in m/s."""
SINK = 0.03
"""How far the jaws go on down past where the truss's lowest fruit meets
what lies under it, in meters, so the truss rests its weight there before
the shear opens."""
CLEAR = 0.1
"""How far above the crate's rim the jaws stay as they lower a truss, in
meters: the mouth is level, so the wrist behind it rides at about the
jaws' height, and further down it meets the rim. A truss hangs about 15 cm
below the jaws, so one lowered into the empty crate stops about 6 cm short
of its floor."""
LEAVE = 0.06
"""How far the open mouth backs off a truss it lowered, in meters: from
the hook's crook out past the claw's tip."""
HELD = 0.015
"""How near the middle of the jaws, in meters, a truss's rod cut free
passes for the jaws to have it: a peduncle anywhere between them passes
within this. One they have as the shear shuts is held there until it
opens."""
TIP = 1.2
"""How far the blades are turned down about the peduncle, in radians, to
tip out a truss the open finger still bears, as it does where the shear is
tilted up: one lying across it slides off once it slopes past the 27
degrees of `harvester.HAND_FRICTION`."""
PATIENCE = 300
"""Control ticks a stage may wait on the robot before its job is given up
on: 6 s."""
FALL = 3.0
"""Seconds the last truss is given to fall before the tally."""

# A rod body's label ends in its segment's prim; segment 0 is the held one.
SEGMENT = re.compile(r"(.*)/Cable_edge_body_(\d+)$")
# A stem's collider's label is its prim: one capsule along each span of it.
STEM = re.compile(r"(.*/Stem_\d+)/Collision_\d+$")
# A fruit's collider rides a segment of its truss's rod.
FRUIT = re.compile(r".*/Fruit_\d+/Collision$")


@dataclasses.dataclass
class Truss:
    prim: str
    cut: np.ndarray
    """Its `Cut` frame in world coordinates: at the peduncle's base, +X along it."""
    due: bool
    side: np.ndarray = dataclasses.field(default_factory=lambda: np.zeros(3))
    """The horizontal direction from the path it is cut from to it."""
    bodies: list[int] = dataclasses.field(default_factory=list)
    """Its rod's bodies, by model index, from the stem out."""
    stem: np.ndarray = dataclasses.field(default_factory=lambda: np.array([0.0, 0.0, 1.0]))
    """Which way the stem it hangs from runs at its node."""
    axis: np.ndarray = dataclasses.field(default_factory=lambda: np.zeros(3))
    """A point on that stem's axis by its node."""
    stem_shapes: list[int] = dataclasses.field(default_factory=list)
    """Its stem's colliders, by model index."""
    fruit: list[tuple[int, np.ndarray, float, float]] = dataclasses.field(default_factory=list)
    """Its fruit's colliders: each one's body, its pose in the body's frame,
    its radius, and how far it is drawn out along its +Z either way: 0 for a
    sphere, more for a capsule."""


@dataclasses.dataclass(frozen=True)
class Point:
    """Where a `Gesture` takes the shear's tip next: `offset` from a point
    of the truss's fruit, in meters ahead the way the shear comes in to cut
    it, across the path into the crop -- along the row, where the shear
    comes in across the path -- and up. `anchor` names the point:
    "back", the near side of the fruit furthest back toward the arm, level
    with its middle; "low", the bottom of the lowest fruit; "middle", the
    middle of them all. The tip goes there on a straight line at `speed`,
    in m/s, and holds still `hold` seconds."""

    anchor: str
    offset: tuple[float, float, float] = (0.0, 0.0, 0.0)
    speed: float = 0.05
    hold: float = 0.0


@dataclasses.dataclass(frozen=True)
class Gesture:
    """The open shear moved against the first truss before it is cut, to
    show the crop giving way to the arm: its tip through `points`, the mouth
    turned as a cut's is (see `turned`), `yaw` radians toward the path and
    `tilt` up. The arm is planned to the first point keeping clear of
    everything, the shear too, so it should stand clear of the truss; it
    goes on to the others in straight lines. Turned toward the path, the arm
    reaches in from it, out of the line of stems along the row. The shear
    cuts only as it closes, so the truss is only pushed. The last point
    should leave the shear clear of the truss, for the cut to be planned
    from."""

    name: str
    points: tuple[Point, ...]
    yaw: float = 0.0
    tilt: float = 0.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Harvests a greenhouse's ripe trusses.")
    parser.add_argument(
        "--paths",
        type=int,
        nargs="+",
        help="The paths to work, as the scene numbers them -- Path_00 first. Every one by default.",
    )
    parser.add_argument(
        "--colliders",
        action="store_true",
        help=(
            "Draw every collider the physics steps, translucent over the scene, kept in step "
            "with the ones moved or resized while it runs: the shear's slices, a cut peduncle."
        ),
    )
    add_launcher_args(parser)
    parser.set_defaults(visualizer=["kit"])
    return parser.parse_args()


def design_scene(machine: Harvester):
    """The greenhouse, a sky, and the robot in the main path at the row heads.

    Declared as a scene, since Newton steps only what a scene declares.
    """
    scene = InteractiveSceneCfg(num_envs=1, env_spacing=0.0)
    scene.light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(
            intensity=750.0,
            texture_file=f"{ISAAC_NUCLEUS_DIR}/Materials/Textures/Skies/PolyHaven/kloofendal_43d_clear_puresky_4k.hdr",
        ),
    )
    # Bare ground around the floor, its top a centimeter under the floor's:
    # past the floor's edge a camera sees ground, not the underside of the sky.
    scene.ground = AssetBaseCfg(
        prim_path="/World/Ground",
        spawn=sim_utils.CuboidCfg(
            size=(200.0, 200.0, 0.1),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.3, 0.27, 0.22)),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -0.06)),
    )
    scene.greenhouse = AssetBaseCfg(prim_path=GREENHOUSE_PATH, spawn=GREENHOUSE_CFG)
    # Its base on the main path's centre line; the rows run from x = 0.
    base = (-GREENHOUSE_CFG.layout.main_path / 2, 0.0, 0.0)
    scene.robot = harvester_cfg(machine, "{ENV_REGEX_NS}/Robot", base)
    # Importing the scene class loads pip's `pxr`, and Kit's own has to be
    # the first one loaded.
    return instantiate(scene)["robot"]


def read_trusses(stage, root: str) -> list[Truss]:
    """Every truss on a rod under `root`, with its `Cut` frame and whether it
    is due."""
    from pxr import Usd, UsdGeom

    trusses = []
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
        if not (prim.GetName().startswith("Truss_") and prim.GetChild("Cable")):
            continue
        cut = UsdGeom.Xformable(prim.GetChild("Cut")).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()
        )
        stages = [
            STAGES.index(each.Get())
            for fruit in Usd.PrimRange(prim)
            if (each := fruit.GetAttribute("greenhouse_lab:stage")) and each.HasValue()
        ]
        due = bool(stages) and min(stages) >= DUE
        # USD's matrices are row vectors.
        trusses.append(Truss(str(prim.GetPath()), np.array(cut).T, due))
    return trusses


def stem_axes(model) -> dict[str, tuple[np.ndarray, np.ndarray, np.ndarray]]:
    """Each stem's colliders, by the stem's prim: their model indices and
    the two ends of each one's axis in the world."""
    poses, scales = model.shape_transform.numpy(), model.shape_scale.numpy()
    found: dict[str, list[int]] = {}
    for shape, label in enumerate(model.shape_label):
        if match := STEM.match(label):
            found.setdefault(match[1], []).append(shape)
    axes = {}
    for stem, shapes in found.items():
        shapes = np.array(shapes)
        frames = [frame_of(pose[:3], pose[3:]) for pose in poses[shapes]]
        reach = np.array([f[:3, 2] for f in frames]) * scales[shapes, 1, None]
        middles = poses[shapes, :3]
        axes[stem] = (shapes, middles - reach, middles + reach)
    return axes


def distances(point: np.ndarray, starts: np.ndarray, ends: np.ndarray) -> np.ndarray:
    """How far `point` is from each segment from `starts[i]` to `ends[i]`."""
    span = ends - starts
    t = np.clip(np.einsum("ij,ij->i", point - starts, span) / (span * span).sum(1), 0.0, 1.0)
    return np.linalg.norm(point - (starts + span * t[:, None]), axis=1)


def joint_line(start, end) -> np.ndarray:
    """The arm's joint positions on a straight line in joint space from
    `start` to `end`, no joint moving more than 0.04 rad from one to the
    next: both ends, however near, so it has a gradient."""
    start, end = np.asarray(start, dtype=float), np.asarray(end, dtype=float)
    return np.linspace(start, end, 2 + int(np.abs(end - start).max() / 0.04))


def path_ys(stage, root: str) -> list[float]:
    """Each path's centre line, by its number."""
    from pxr import Usd, UsdGeom

    paths = sorted(stage.GetPrimAtPath(f"{root}/Paths").GetChildren(), key=lambda p: p.GetName())
    return [
        UsdGeom.Xformable(path)
        .ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        .ExtractTranslation()[1]
        for path in paths
    ]


def plan(trusses: list[Truss], ys: list[float], paths: list[int], pitch: float) -> list:
    """The jobs that harvest `paths` in order, as trusses to cut and joint
    positions to go to: for each path, out to the main path with the arm
    folded and across to it, then its due trusses on the left in driving
    order and the ones on the right on the way back; and out again at the
    end."""
    jobs: list = []
    y = 0.0
    for number in paths:
        jobs += [np.array([y, 0.0, 0.0, *ARM_HOME]), np.array([ys[number], 0.0, 0.0, *ARM_HOME])]
        y = ys[number]
        here = [t for t in trusses if t.due and abs(t.cut[1, 3] - y) < pitch / 2]
        for truss in here:
            truss.side = np.array([0.0, np.sign(truss.cut[1, 3] - y), 0.0])
        jobs += sorted((t for t in here if t.side[1] > 0), key=lambda t: t.cut[0, 3])
        jobs += sorted((t for t in here if t.side[1] < 0), key=lambda t: -t.cut[0, 3])
    jobs.append(np.array([y, 0.0, 0.0, *ARM_HOME]))
    return jobs


def peduncle(truss: Truss, shears) -> tuple[np.ndarray, np.ndarray]:
    """Where the blade crosses `truss`'s peduncle, `PAST` its rod's first
    joint, as it hangs now, and the way square to the stem nearest the way
    the peduncle runs there: the normal of the blades' plane, which runs
    along the stem. A plane square to the peduncle itself would cross the
    stem a few centimetres from the cut wherever the peduncle rises or
    droops off square to it, where the hook sweeps it.

    The peduncle runs the way the segment past that joint runs. The truss's
    weight bends the rod at that joint, off the way `Cut` was authored. One
    running more than `ASKEW` off that way, pushed along the row by a
    neighbour, is taken to run as authored."""
    authored = truss.cut[:3, 0] / np.linalg.norm(truss.cut[:3, 0])
    start, end = shears.capsules()
    first = np.searchsorted(shears.bodies, truss.bodies[1])
    hangs = (end[first] - start[first]) / np.linalg.norm(end[first] - start[first])
    along = hangs if hangs @ authored > np.cos(np.radians(ASKEW)) else authored
    square = along - (along @ truss.stem) * truss.stem
    return start[first] + PAST * along, square / np.linalg.norm(square)


def heading(along: np.ndarray, side: np.ndarray) -> np.ndarray:
    """The level way the shear comes in to a peduncle running `along` on the
    `side` of the path: square to it, and of the two ways, the one that turns
    the shear's +Y up, so the hook is above the peduncle and the truss's
    weight hangs it off the hook into the mouth, from where the cutting blade
    lifts it onto the hook. Hung the other way, on the hook, the weight drags
    it over the hook's thin edge, out of the blade's reach. A peduncle
    running along the row is come at from the path all the same, the other
    way being out of the crop wall."""
    way = np.cross(along, [0.0, 0.0, 1.0])
    way /= np.linalg.norm(way)
    return -way if way @ side < -0.5 else way


def approaches(
    truss: Truss, at: np.ndarray, along: np.ndarray, machine: Harvester
) -> list[np.ndarray]:
    """The mouth's poses that cut `truss`'s peduncle at `at`, the blades'
    plane square to `along` (see `peduncle`): through `at`, +X `along`, so
    the peduncle lies across the mouth with the jaws on the truss's side of
    the cut, and +Z the way the shear points, `heading`; each turned
    `YAWS` about the stem toward the path, and tilted up to `TILTS` off
    level about its +X. A turned plane crosses the peduncle further out,
    where it is as far off the stem's axis as the unturned one."""
    off = at - truss.axis
    reach = np.linalg.norm(off - (off @ truss.stem) * truss.stem)
    poses = []
    for yaw in YAWS:
        through = at + (reach / np.cos(yaw) - reach) * along
        for tilt in TILTS:
            pose = np.eye(4)
            pose[:3, :3] = turned(truss, along, yaw, tilt)
            pose[:3, 3] = through - machine.shear.bypass * pose[:3, 0]
            poses.append(pose)
    return poses


def turned(truss: Truss, along: np.ndarray, yaw: float, tilt: float) -> np.ndarray:
    """The mouth's rotation at `truss`'s peduncle running `along`: +X along
    it and +Z level the way the shear points, `heading`, turned `yaw` about
    the stem toward the path and then tilted `tilt` up about its +X."""
    level = heading(along, truss.side)
    toward = np.sign(np.cross(truss.stem, level) @ truss.side)
    turn = rotation(truss.stem, toward * yaw)
    x, ahead = turn @ along, turn @ level
    approach = np.cos(tilt) * ahead + np.sin(tilt) * np.cross(ahead, x)
    return np.column_stack([x, np.cross(approach, x), approach])


def gesture_poses(gesture: Gesture, truss: Truss, shears, machine: Harvester) -> list[np.ndarray]:
    """The mouth's world poses that put the shear's tip on each of
    `gesture`'s points, about `truss`'s fruit as they hang now."""
    _, along = peduncle(truss, shears)
    level, up = heading(along, truss.side), np.array([0.0, 0.0, 1.0])
    # Into the crop; where the shear comes in across the path, square to a
    # peduncle along the row, this runs along the row instead.
    across = np.cross(up, level)
    if across @ truss.side < 0:
        across = -across
    fruit = placed_fruit(truss.fruit, shears)
    middles, axes, radii, halves = fruit
    # How far each fruit reaches from its middle toward the arm, level with
    # its middle: out through its side, or through a round end where the
    # side ends first. And how far it reaches down.
    cos = np.abs(axes @ level)
    sin = np.sqrt(np.maximum(1.0 - cos**2, 1e-12))
    behind = np.where(
        radii / sin * cos <= halves,
        radii / sin,
        halves * cos + np.sqrt(np.maximum(radii**2 - (halves * sin) ** 2, 0.0)),
    )
    _, under = reaches(fruit)
    back, low = np.argmin(middles @ level - behind), np.argmin(middles[:, 2] - under)
    anchors = {
        "back": middles[back] - behind[back] * level,
        # The bottom of its lower round end.
        "low": middles[low] - halves[low] * np.sign(axes[low, 2]) * axes[low] - radii[low] * up,
        "middle": middles.mean(axis=0),
    }
    mouth = turned(truss, along, gesture.yaw, gesture.tilt)
    s = machine.shear
    claw = np.linalg.inv(machine.tool) @ machine.shear_frame
    tip = mouth @ (claw @ [0.0, s.edge(s.blade + s.claw), s.blade + s.claw, 1.0])[:3]
    poses = []
    for point in gesture.points:
        pose = np.eye(4)
        pose[:3, :3] = mouth
        offset = np.column_stack([level, across, up]) @ point.offset
        pose[:3, 3] = anchors[point.anchor] + offset - tip
        poses.append(pose)
    return poses


Fruit = tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]
"""Fruit colliders where they lie: each one's middle, its +Z, its radius and
how far it is drawn out along its +Z either way."""


def placed_fruit(fruit: list, shears) -> Fruit:
    """Fruit colliders, listed as `Truss.fruit` lists them, where they lie
    now in the world."""
    placed = np.zeros((len(fruit), 4, 4))
    for i, (body, offset, _, _) in enumerate(fruit):
        pose = shears.pose(body)
        placed[i] = frame_of(pose[:3], pose[3:]) @ offset
    radii, halves = np.array([each[2:] for each in fruit]).reshape(-1, 2).T
    return placed[:, :3, 3], placed[:, :3, 2], radii, halves


def moved(fruit: Fruit, frame: np.ndarray) -> Fruit:
    """`fruit` moved by the transform `frame`."""
    middles, axes, radii, halves = fruit
    return middles @ frame[:3, :3].T + frame[:3, 3], axes @ frame[:3, :3].T, radii, halves


def reaches(fruit: Fruit) -> tuple[np.ndarray, np.ndarray]:
    """How far each of `fruit` reaches from its middle level, and up or down."""
    _, axes, radii, halves = fruit
    upright = np.abs(axes[:, 2])
    return radii + halves * np.sqrt(np.maximum(1.0 - upright**2, 0.0)), radii + halves * upright


def room(held: Fruit, pile: Fruit, crate: Crate) -> float:
    """How far the fruit `held` can come straight down before one of them
    meets one of `pile`, `crate`'s floor, or its rim where it reaches out
    over a wall. Both are in the crate's frame, each fruit taken as the
    upright cylinder round it: it meets the highest it overlaps seen from
    above that reaches below its bottom."""
    (low, *_), (high, *_) = held, pile
    (wide, under), (broad, over) = reaches(held), reaches(pile)
    inside = np.array([crate.length, crate.width]) / 2 - crate.rib - crate.wall
    out = (np.abs(low[:, :2]) + wide[:, None] > inside).any(axis=1)
    floor = np.where(out, crate.height, crate.floor)
    bottoms = low[:, 2] - under
    below = np.linalg.norm(low[:, None, :2] - high[None, :, :2], axis=2) < wide[:, None] + broad
    below &= high[None, :, 2] - over < bottoms[:, None]
    tops = np.where(below, high[:, 2] + over, -np.inf).max(axis=1, initial=-np.inf)
    # With no fruit held, nothing stops it.
    return float((bottoms - np.maximum(floor, tops)).min(initial=np.inf))


def station(machine: Harvester, cut: np.ndarray, way: np.ndarray, aisle: float) -> np.ndarray:
    """Where the trolley stands for a cut at `cut` the shear comes in to
    `way`, both in the robot's base frame: across at `aisle`, the arm's base
    `BACK` back from the cut along the rail, so the shear comes in away from
    it and the arm reaches past the near side of the cut, not round its far
    side, and the lift holding the cut `RISE` above the shoulder. The three
    joints' positions."""
    chain = machine.chain
    shoulder = chain[2].xyz[2] + chain[3].xyz[2]
    wanted = [aisle, cut[0] - BACK * np.sign(way[0]), cut[2] - RISE - shoulder]
    return np.clip(wanted, *zip(*(joint.limits for joint in chain[:3]), strict=True))


@dataclasses.dataclass
class Tally:
    due: int = 0
    reachable: int = 0
    cut: int = 0
    crowded: int = 0
    """Reachable only by brushing a truss."""
    held: int = 0
    """Cut, and still in the jaws where it is let go."""
    missed: int = 0
    crated: int = 0
    """Cut, and lying in the crate at the end."""
    seconds: float = 0.0

    def summary(self) -> str:
        share = lambda part: f"{100 * part / self.due:.0f}%" if self.due else "n/a"  # noqa: E731
        return (
            f"{self.due} trusses due: {share(self.reachable)} reachable, "
            f"{self.crowded} of them only by brushing a truss, "
            f"{share(self.cut)} cut, {share(self.held)} held and let go, "
            f"{share(self.crated)} in the crate, {self.missed} given up on, "
            f"{self.seconds / max(self.cut, 1):.1f} s per truss cut"
        )


class Harvest:
    """The jobs `plan` lists, done one at a time, a control tick at a time.

    `control` takes where the joints are, the hand's world pose and the
    shear's angle, and returns every joint's position and velocity targets
    and the shear's. The trolley runs only with the arm folded, and `planner`
    plans the fold from wherever the arm is. A joint position is a run to
    it. A truss is a run to its station, and then the cut `planner` plans
    from there: the move to a stand-off and the straight line in, holding
    still until the mouth is at the cut, the shear closing, cutting and
    taking the truss, the line back out, a straight line `PULL` out across
    the path, a swing round to over the crate, a straight line down until
    the truss rests on what lies there, and the shear opening there, letting
    the truss go, and backing off it; a truss let fall instead, short of
    the crate, is tipped out if it still lies on the finger. A
    shear that cut nothing opens and backs out, and the cut is planned
    afresh, `TRIES` times in all. `stroke` is the shear. The robot's base is
    at `base` in the world.

    A cut is planned clear of everything first, but for the held segment and
    the next, which the shear closes on, and the stretch of its stem by the
    cut, which the shear slides along (`planner.BESIDE`). Failing that, the
    arm may brush the truss it cuts, and failing that any truss: the rods
    bend aside, as the crop does round a picker's arm. The trolley, the
    rails, the floor and every other stretch of stem it never touches.

    Before the first truss's cut is planned, the arm makes `gestures` at it,
    each a stage `reach` to its first point and a stage `touch` through the
    rest, leaving out one it cannot make or finish. The tally counts the time
    they take toward that truss's cut.
    """

    def __init__(
        self,
        machine: Harvester,
        planner,
        jobs: list,
        base: np.ndarray,
        q,
        shears,
        tally: Tally,
        gestures=(),
    ):
        self.machine, self.planner, self.base = machine, planner, base
        self.shears, self.tally = shears, tally
        self.gestures: list[Gesture] = list(gestures)
        """The gestures still to make, at the first truss."""
        self.touching: tuple[np.ndarray, np.ndarray] | None = None
        """The gesture under way, past its first point."""
        self.made = 0
        """How many of `gestures` the arm has made to their last point."""
        self.stroke = Stroke(machine.shear, machine.shear_frame, shears)
        self.jobs = list(jobs)
        self.q = np.array(q, dtype=float)
        self.target = self.q.copy()
        self.velocity = np.zeros_like(self.q)
        self.cut: list[Truss] = []
        """The trusses cut, in order."""
        self.held: int | None = None
        """The body of the rod cut free that the jaws hold, from the shut until
        the shear opens or the job is given up: `run` has `Shears.hold` carry
        it with the hand."""
        self.lowered = False
        """Whether the job's truss was lowered into the crate, for the open
        shear to back off it (`_leave`)."""
        self.time = 0.0
        self.soft: set[int] = set()
        """The rod bodies the arm may brush, those the last cut was planned through."""
        self.beside: tuple[list[int], np.ndarray] = ([], np.zeros(3))
        """The stem the last cut was planned beside, its colliders, and where
        it was cut: the planner leaves the stem out of its world there."""
        self._next()

    @property
    def done(self) -> bool:
        return self.stage == "done"

    @property
    def severed(self) -> bool:
        """Whether the job's truss is cut."""
        return bool(self.cut) and self.cut[-1] is self.job

    def platform(self) -> np.ndarray:
        """The platform's pose in the world, the trolley where it is held: a
        run overshoots its station by centimetres before it settles there."""
        return self.base @ frames(self.machine.chain[:3], self.target[:3])[-1]

    def _next(self) -> None:
        """Start on the next job, or finish."""
        self.held, self.lowered = None, False
        while self.jobs:
            self.job = self.jobs.pop(0)
            if not isinstance(self.job, Truss):
                self.goal = self.job[:3]
            elif self.shears.loose[np.searchsorted(self.shears.bodies, self.job.bodies)].any():
                continue
            else:
                local = np.linalg.inv(self.base)
                _, along = peduncle(self.job, self.shears)
                way = local[:3, :3] @ heading(along, self.job.side)
                cut = local @ self.job.cut[:, 3]
                self.goal = station(self.machine, cut, way, self.target[0])
                self.started, self.tries = self.time, 0
            self._fold()
            return
        self.job, self.stage = None, "done"

    def _fold(self) -> None:
        """Fold the arm where the trolley stands, unless it is folded, to run."""
        self.stage, self.ticks = "run", 0
        if np.abs(self.q[3:] - ARM_HOME).max() < ARRIVED:
            return
        folding = self.planner.move(self.q[3:], ARM_HOME, self.platform(), self.soft, self.beside)
        if folding is None:
            # From among the trusses, the arm may brush any on the way out.
            folding = self.planner.move(
                self.q[3:], ARM_HOME, self.platform(), set(self.shears.bodies), self.beside
            )
        if folding is None:
            # From a pose already touching what it may not brush, a fold
            # finds no way. The arm folds on straight lines in joint space
            # instead: out of the crop with the shoulder pan held, then
            # round. Turned while still out, it would sweep along the crop
            # wall into the stems.
            print(f"[INFO]: no clear way to fold from {np.round(self.q[3:], 2)}")
            out = [self.q[3], *ARM_HOME[1:]]
            line = np.concatenate([joint_line(self.q[3:], out), joint_line(out, ARM_HOME)])
            folding = (line, np.gradient(line, self.planner.dt, axis=0))
        self._play("fold", folding)

    def _plan(self) -> None:
        """Plan the cut of the truss the trolley stands for, or give it up."""
        truss, platform = self.job, self.platform()
        at, along = peduncle(truss, self.shears)
        poses = approaches(truss, at, along, self.machine)
        beside = (truss.stem_shapes, at)
        for soft in (set(truss.bodies[:2]), set(truss.bodies), set(self.shears.bodies)):
            planned = self.planner.cut(self.q[3:], platform, poses, STANDOFF, soft, beside)
            if planned is not None:
                break
        if planned is None:
            print(f"[INFO]: {truss.prim}: out of reach")
            self.tally.missed += self.tries > 0
            self._next()
            return
        self.tries += 1
        self.tally.reachable += self.tries == 1
        self.tally.crowded += self.tries == 1 and len(soft) > 2
        self.planned, self.soft, self.beside = planned, soft, beside
        self._play("approach", planned.approach)

    def _touch(self) -> None:
        """Make the next of `gestures` at the truss the trolley stands for, or
        plan its cut once none is left. A truss with no fruit to place them
        about passes every one over, so none is left for a later truss."""
        while self.gestures:
            gesture = self.gestures.pop(0)
            if self.job.fruit and (planned := self._gesture(gesture)) is not None:
                reach, self.touching = planned
                self._play("reach", reach)
                return
            print(f"[INFO]: {self.job.prim}: no way to {gesture.name}")
        self._plan()

    def _gesture(
        self, gesture: Gesture
    ) -> tuple[tuple[np.ndarray, np.ndarray], tuple[np.ndarray, np.ndarray]] | None:
        """`gesture` at the job's truss, as the arm's joint positions and
        velocities every `planner.dt` on its way to the first point and then
        through the rest; or None if the arm cannot make it. The way to the
        first point keeps clear of the truss, but for the stretch of its stem
        by that point, and every straight line after it is solved before any
        is made."""
        platform = self.platform()
        poses = gesture_poses(gesture, self.job, self.shears, self.machine)
        beside = (self.job.stem_shapes, poses[0][:3, 3])
        reached = self.planner.reach(self.q[3:], platform, poses[0], set(), beside)
        if reached is None:
            return None
        stretches, q = [], reached[0][-1]
        for i, point in enumerate(gesture.points):
            if i:
                line = self.planner.line(q, platform, poses[i - 1], poses[i], point.speed)
                if line is None:
                    return None
                stretches.append(line)
                q = line[0][-1]
            # Held for one point at least, so the gesture never ends empty.
            still = np.repeat(q[None], max(1, round(point.hold / self.planner.dt)), axis=0)
            stretches.append((still, np.zeros_like(still)))
        return reached, tuple(np.concatenate(part) for part in zip(*stretches, strict=True))

    def _play(self, stage: str, trajectory: tuple[np.ndarray, np.ndarray]) -> None:
        self.stage, self.ticks, self.clock = stage, 0, 0.0
        self.trajectory = trajectory

    def _shut(self, hand: np.ndarray) -> None:
        """Count the truss cut if its rod is, now the shear is shut, hold it
        if the jaws have it, and back out with what they hold."""
        if self.shears.loose[np.searchsorted(self.shears.bodies, self.job.bodies)].any():
            print(f"[INFO]: {self.job.prim}: cut after {self.time - self.started:.1f} s")
            self.cut.append(self.job)
            self.tally.cut += 1
            self.tally.seconds += self.time - self.started
            gap, body = self._between(hand)
            if gap < HELD:
                self.held = body
        else:
            missed = "a neighbour" if self.stroke.cuts else "nothing"
            print(f"[INFO]: {self.job.prim}: the shear cut {missed}")
            self.tally.missed += self.tries == TRIES
        self._play("outward", self.planned.outward)

    def _opened(self) -> None:
        """Plan the cut afresh if the shear closed on nothing and it has
        tries left; else move on, the arm brushing the truss let go as it
        falls past the shear."""
        if not self.severed and self.tries < TRIES:
            self._plan()
            return
        self.soft |= set(self.job.bodies)
        self._next()

    def _pull(self, hand: np.ndarray) -> None:
        """Pull the truss held out across the path, rising to the height it
        is carried at, or let it go where it is if the arm cannot reach out
        that way."""
        start = hand @ self.machine.tool
        end = start.copy()
        end[:3, 3] -= PULL * self.job.side
        end[2, 3] += (self.platform() @ [*self._over_crate(), 1.0])[2] - start[2, 3]
        pulling = self.planner.line(self.q[3:], self.platform(), start, end)
        if pulling is None:
            print(f"[INFO]: {self.job.prim}: let go at the stand-off")
            self._release(hand)
        else:
            self._play("pull", pulling)

    def _carry(self, hand: np.ndarray) -> None:
        """Swing the truss held round the arm's base to over the crate, the
        jaws `CARRY` above its floor, at the one of `SPOTS` where the truss
        would come down furthest before it meets what lies in the crate
        (`room`), or let it go where it is if the arm can get over none of
        them. The mouth ends turned about the upright alone, so the truss
        hangs in the jaws as it did when cut, its weight pulling away from the
        mouth's entrance. On the way the arm may brush any truss, which leaves
        `planner` fewer to keep off and the mouth less to turn."""
        platform, crate = self.platform(), self.machine.crate_frame
        mouth = np.linalg.inv(platform) @ hand @ self.machine.tool
        hold = np.array([self.machine.shear.hold, 0.0, 0.0, 1.0])
        jaws = (mouth @ hold)[:3]
        held, pile = self._crated()
        spots = []
        for along in SPOTS:
            there = self._over_crate(along)
            # Round the shoulder's axis, the platform's upright: the short
            # way, unless that takes the shoulder past its range.
            turn = math.atan2(there[1], there[0]) - math.atan2(jaws[1], jaws[0])
            over = transform(rpy=(0.0, 0.0, turn)) @ mouth
            over[:3, 3] += there - (over @ hold)[:3]
            seed = self.q[3:].copy()
            seed[0] = (seed[0] + turn + math.pi) % (2 * math.pi) - math.pi
            drop, distance, angle = solve(self.planner.chain, self.machine.tool, over, seed)
            # Unsolved where even the long way round takes the shoulder past
            # its range.
            if distance < 0.005 and angle < 0.02:
                # The truss swung there with the mouth, in the crate's frame.
                swung = np.linalg.inv(crate) @ over @ np.linalg.inv(mouth) @ crate
                spots.append((round(room(moved(held, swung), pile, self.machine.crate), 2), drop))
        carrying = None
        # Furthest down first; of spots as far to the centimetre, the first.
        for _, drop in sorted(spots, key=lambda spot: -spot[0]):
            carrying = self.planner.move(
                self.q[3:], drop, platform, set(self.shears.bodies), self.beside
            )
            if carrying is not None:
                break
        if carrying is None:
            print(f"[INFO]: {self.job.prim}: let go short of the crate")
            self._release(hand)
        else:
            self._play("carry", carrying)

    def _lower(self, hand: np.ndarray) -> None:
        """Lower the truss held straight down at `LOWER` until its lowest
        fruit is `SINK` past where it meets what lies under it in the crate,
        the jaws `CLEAR` above the rim at the lowest; three quarters or half
        as far where the arm would meet the crate or itself (`_clear`); or
        let it go where it is if none of those will do."""
        start = hand @ self.machine.tool
        crate = np.linalg.inv(self.platform() @ self.machine.crate_frame)
        jaws = (crate @ start @ [self.machine.shear.hold, 0.0, 0.0, 1.0])[2]
        down = room(*self._crated(), self.machine.crate) + SINK
        down = np.clip(down, 0.0, jaws - self.machine.crate.height - CLEAR)
        # Less far where the arm would meet the crate or itself.
        for depth in down * np.array([1.0, 0.75, 0.5]):
            end = start.copy()
            end[2, 3] -= depth
            lowering = self.planner.line(self.q[3:], self.platform(), start, end, LOWER)
            if self._clear(lowering):
                self.lowered = True
                self._play("lower", lowering)
                return
        print(f"[INFO]: {self.job.prim}: let go over the crate")
        self._release(hand)

    def _leave(self, hand: np.ndarray) -> None:
        """Back the open mouth `LEAVE` off the truss it lowered, the way it
        came in to the cut, so the peduncle passes out under the hook's claw:
        the truss rests in the crate, so tipping the blades about the
        peduncle would not free it. Where the arm would meet the crate or
        itself, move on from where it is. Neither way does the mouth get
        `_fallen`'s check: `Stroke.fouled` counts every piece cut free,
        those in the crate under the lowered mouth too, and tipping turns
        the blades down into them."""
        start = hand @ self.machine.tool
        end = start @ transform(-LEAVE * self.machine.way_in)
        leaving = self.planner.line(self.q[3:], self.platform(), start, end)
        if self._clear(leaving):
            self._play("leave", leaving)
        else:
            self._opened()

    def _fallen(self, hand: np.ndarray) -> None:
        """Tip out a truss let go that still lies on the finger, or move on."""
        if self.severed and self.stroke.fouled(hand):
            self._tip(hand)
        else:
            self._opened()

    def _clear(self, line: tuple[np.ndarray, np.ndarray] | None) -> bool:
        """Whether the arm can follow a straight `line` of the mouth's, and
        `planner` finds the arm at its end clear of everything but the
        trusses: `Planner.line` solves its points alone, so it passes the
        arm through the crate and the robot's own links as readily. Only the
        end is checked: a line down into the crate is deepest there, the
        wrist nearest the rim."""
        return (
            line is not None
            and self.planner.move(
                self.q[3:], line[0][-1], self.platform(), set(self.shears.bodies), self.beside
            )
            is not None
        )

    def _crated(self) -> tuple[Fruit, Fruit]:
        """The job's truss's fruit and those of the trusses cut before it,
        where they lie now in the crate's frame."""
        inverse = np.linalg.inv(self.platform() @ self.machine.crate_frame)
        before = [fruit for truss in self.cut if truss is not self.job for fruit in truss.fruit]
        return tuple(
            moved(placed_fruit(fruit, self.shears), inverse) for fruit in (self.job.fruit, before)
        )

    def _over_crate(self, along: float = 0.0) -> np.ndarray:
        """Where the jaws are carried to, in the platform's frame: `CARRY`
        over the crate's floor, `along` its length from its middle."""
        return (self.machine.crate_frame @ [along, 0.0, self.machine.crate.floor + CARRY, 1.0])[:3]

    def _between(self, hand: np.ndarray) -> tuple[float, int]:
        """How near the middle of the jaws the job's rod cut free passes, and
        the body of it that passes nearest."""
        jaws = (hand @ self.machine.tool @ transform((self.machine.shear.hold, 0.0, 0.0)))[:3, 3]
        rod = np.searchsorted(self.shears.bodies, self.job.bodies)
        loose = rod[self.shears.loose[rod]]
        start, end = (each[loose] for each in self.shears.capsules())
        axis = end - start
        along = np.clip(((jaws - start) * axis).sum(1) / (axis * axis).sum(1), 0.0, 1.0)
        gaps = np.linalg.norm(start + along[:, None] * axis - jaws, axis=1)
        nearest = int(gaps.argmin())
        return float(gaps[nearest]), int(self.shears.bodies[loose[nearest]])

    def _release(self, hand: np.ndarray) -> None:
        """Open the shear and let go, counting the truss held if its rod cut
        free still passes within `HELD` of the middle of the jaws."""
        self.tally.held += self._between(hand)[0] < HELD
        self.held = None
        self.stage, self.ticks = "open", 0

    def _tip(self, hand: np.ndarray) -> None:
        """Turn the blades `TIP` down about the peduncle, on a straight line
        in joint space."""
        platform = self.base @ frames(self.machine.chain[:3], self.q[:3])[-1]
        tipped = hand @ self.machine.tool @ transform(rpy=(TIP, 0.0, 0.0))
        q, _, _ = solve(
            self.planner.chain, self.machine.tool, np.linalg.inv(platform) @ tipped, self.q[3:]
        )
        line = joint_line(self.q[3:], q)
        self._play("tip", (line, np.gradient(line, self.planner.dt, axis=0)))

    def control(
        self, q: np.ndarray, hand: np.ndarray, angle: float
    ) -> tuple[np.ndarray, np.ndarray, float]:
        """Advance one tick, `q` every joint's position, `hand` the last
        wrist's world pose and `angle` the shear's. Returns every joint's
        position and velocity targets, and the shear's angle to hold."""
        moved = float(np.abs(q - self.q).max())
        self.q = np.array(q, dtype=float)
        self.velocity[:] = 0.0
        if self.done:
            return self.target, self.velocity, self.stroke.jaw
        tick = DECIMATION * SIM_DT
        self.time += tick
        self.ticks += 1
        settled = np.abs(self.target - self.q).max() < ARRIVED and moved < STILL
        if self.ticks > PATIENCE and self.stage == "close":
            # Backed out along the line it came in on, clear of the crop, to
            # try again if it may.
            print(f"[INFO]: {self.job.prim}: gave up at the {self.stage}")
            self.tally.missed += self.tries == TRIES
            self._play("outward", self.planned.outward)
        elif self.ticks > PATIENCE and self.stage in ("reach", "touch"):
            # A gesture the arm cannot finish is left, not the truss.
            print(f"[INFO]: {self.job.prim}: gave up at the {self.stage}")
            self._touch()
        elif self.ticks > PATIENCE and self.stage == "lower":
            # Held up on the way down, it lets go where it is.
            print(f"[INFO]: {self.job.prim}: let go on the way down")
            self._release(hand)
        elif self.ticks > PATIENCE:
            print(f"[INFO]: {getattr(self.job, 'prim', 'a run')}: gave up at {self.stage}")
            self.tally.missed += isinstance(self.job, Truss)
            self._next()
        elif self.stage == "run":
            gap = self.goal - self.target[:3]
            if np.abs(gap).any():
                # Every joint paced to arrive with the slowest; patience
                # counts only the wait for the robot to catch up.
                step = gap / max(np.abs(gap / STEPS).max(), 1.0)
                self.target[:3] += step
                self.velocity[:3] = step / tick
                self.ticks = 0
            elif settled and isinstance(self.job, Truss):
                self._touch()
            elif settled:
                self._next()
        elif self.stage == "close":
            if self.stroke.tick(hand, angle, closing=True):
                self._shut(hand)
        elif self.stage == "open":
            if self.stroke.tick(hand, angle, closing=False):
                if self.lowered:
                    self._leave(hand)
                else:
                    self._fallen(hand)
        else:
            # A planned stretch, played on the simulation's clock, its end
            # held still until the robot is there and still.
            positions, velocities = self.trajectory
            self.clock += tick
            at = self.clock / self.planner.dt
            if at < len(positions) - 1:
                i, share = int(at), at - int(at)
                self.target[3:] = (1 - share) * positions[i] + share * positions[i + 1]
                self.velocity[3:] = (1 - share) * velocities[i] + share * velocities[i + 1]
                self.ticks = 0
            else:
                self.target[3:] = positions[-1]
                if settled and self.stage == "reach":
                    self._play("touch", self.touching)
                elif settled and self.stage == "touch":
                    self.made += 1
                    self._touch()
                elif settled and self.stage == "approach":
                    self._play("inward", self.planned.inward)
                elif settled and self.stage == "outward" and self.severed:
                    self._pull(hand)
                elif settled and self.stage == "pull":
                    self._carry(hand)
                elif settled and self.stage == "carry":
                    self._lower(hand)
                elif settled and self.stage == "lower":
                    self._release(hand)
                elif settled and self.stage == "leave":
                    self._opened()
                elif settled and self.stage == "tip" and not self.stroke.fouled(hand):
                    self._opened()
                elif settled and self.stage in ("fold", "inward", "outward"):
                    after = {"fold": "run", "inward": "close", "outward": "open"}
                    self.stage, self.ticks = after[self.stage], 0
        return self.target, self.velocity, self.stroke.jaw


def frame_of(position: np.ndarray, quat: np.ndarray) -> np.ndarray:
    """A position and an (x, y, z, w) rotation as a transform."""
    x, y, z, w = quat
    frame = np.eye(4)
    frame[:3, :3] = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )
    frame[:3, 3] = position
    return frame


def simulation_cfg() -> sim_utils.SimulationCfg:
    """Newton coupled with VBD, the robot's forearm and wrists among what a
    truss collides with, under a sweep-and-prune broad phase: a cut truss meets every
    other, so the crate fills with a pile rather than a layer."""
    return sim_utils.SimulationCfg(
        dt=SIM_DT,
        physics=rods.make_physics_cfg_newton(
            GREENHOUSE_CFG, ROBOT_PATH, ROBOT_CONTACT, SUBSTEPS, broad_phase="sap"
        ),
    )


def make_machine() -> Harvester:
    """The harvester for `GREENHOUSE_CFG`'s rail."""
    rail = GREENHOUSE_CFG.rail
    return Harvester(gauge=rail.gauge, rail_top=rail.height + rail.pipe_diameter / 2)


def run(
    sim: sim_utils.SimulationContext,
    robot,
    machine: Harvester,
    paths: list[int] | None = None,
    render: bool = True,
    gestures: tuple[Gesture, ...] = (),
) -> Iterator[Harvest]:
    """Reset the simulation of the scene `design_scene` laid out and harvest
    `paths`, every one by default, for as long as the run lasts, yielding the
    `Harvest` after every physics step, the arm making `gestures` at the first
    truss before it is cut. `FALL` seconds after its last job the tally is
    printed, and a run without a window ends. Without `render` no step
    renders, and what does is up to the caller."""
    from isaaclab_newton.physics import NewtonManager

    from misina_lab.isaaclab.cutting import Shears
    from planner import Planner

    stage = sim_utils.get_current_stage()
    sim.reset()
    # The joints spawn at zero until written: the robot starts folded.
    robot.write_joint_position_to_sim_index(position=robot.data.default_joint_pos.torch.clone())
    robot.write_joint_velocity_to_sim_index(velocity=robot.data.default_joint_vel.torch.clone())
    robot.reset()
    shears = Shears()
    chains: dict[str, list[tuple[int, int]]] = {}
    for body, label in enumerate(shears.labels):
        if match := SEGMENT.match(label):
            chains.setdefault(match[1], []).append((int(match[2]), body))
    trusses = read_trusses(stage, GREENHOUSE_PATH)
    model = NewtonManager.get_model()
    stems = stem_axes(model)
    rows: dict[str, list[int]] = {}
    for stem, (shapes, _, _) in stems.items():
        # A stem's row is its plant's parent.
        rows.setdefault(stem.rsplit("/", 2)[0], []).extend(shapes)
    shape_body, scale = model.shape_body.numpy(), model.shape_scale.numpy()
    placed = model.shape_transform.numpy()
    fruit = [shape for shape, label in enumerate(model.shape_label) if FRUIT.match(label)]
    for truss in trusses:
        truss.bodies = [body for _, body in sorted(chains[truss.prim])]
        shapes, starts, ends = stems[truss.prim.rsplit("/", 1)[0]]
        nearest = distances(truss.cut[:3, 3], starts, ends).argmin()
        truss.stem = (ends - starts)[nearest] / np.linalg.norm((ends - starts)[nearest])
        truss.axis = starts[nearest]
        truss.stem_shapes = list(shapes)
        truss.fruit = [
            (int(shape_body[s]), frame_of(placed[s, :3], placed[s, 3:]), *map(float, scale[s, :2]))
            for s in fruit
            if shape_body[s] in truss.bodies
        ]
    ys = path_ys(stage, GREENHOUSE_PATH)
    paths = paths if paths is not None else list(range(len(ys)))
    if not all(0 <= path < len(ys) for path in paths):
        raise ValueError(f"no path {paths}: the greenhouse has {len(ys)} paths")
    jobs = plan(trusses, ys, paths, GREENHOUSE_CFG.layout.pitch)
    tally = Tally(due=sum(isinstance(job, Truss) for job in jobs))

    joints, _ = robot.find_joints(
        [AISLE, RAIL, LIFT, *arm_joints(), SHEAR_JOINT], preserve_order=True
    )
    hand, _ = robot.find_bodies([HAND])
    base = frame_of(
        robot.data.root_pos_w.torch[0].cpu().numpy(),
        robot.data.root_quat_w.torch[0].cpu().numpy(),
    )

    def hold(targets, velocities) -> None:
        command = robot.actuators.target_command
        for set_index, value in (
            (command.set_position_index, targets),
            (command.set_velocity_index, velocities),
        ):
            set_index(
                value=torch.tensor(value, dtype=torch.float32, device=robot.device).unsqueeze(0),
                joint_ids=joints,
            )

    def hand_at() -> np.ndarray:
        """The hand's world pose, as a position and (x, y, z, w) rotation in one
        array of 7."""
        return np.concatenate(
            [
                robot.data.body_link_pos_w.torch[0, hand[0]].cpu().numpy(),
                robot.data.body_link_quat_w.torch[0, hand[0]].cpu().numpy(),
            ]
        )

    # A drive with no target holds zero, so the arm is held home and the
    # shear open first.
    hold([0.0, 0.0, 0.0, *ARM_HOME, machine.shear.opening], np.zeros(len(joints)))
    # Where the robot starts.
    spawned = [0.0, 0.0, 0.0, *ARM_HOME]
    planner = Planner(machine, ROBOT_PATH, list(rows.values()))
    harvest = Harvest(machine, planner, jobs, base, spawned, shears, tally, gestures)
    print(f"[INFO]: {len(trusses)} trusses bend, {tally.due} due on paths {paths}", flush=True)
    sim.set_camera_view(eye=[-3.5, -2.5, 2.8], target=[1.5, 0.0, 1.0])
    step, after = 0, 0
    while sim.is_running():
        q = robot.data.joint_pos.torch[0, joints].cpu().numpy()
        at = hand_at()
        if step % DECIMATION == 0:
            targets, velocities, jaw = harvest.control(
                q[:-1], frame_of(at[:3], at[3:]), float(q[-1])
            )
            hold([*targets, jaw], [*velocities, 0.0])
        shears.hold(harvest.held, at)
        robot.write_data_to_sim()
        sim.step(render=render)
        robot.update(SIM_DT)
        step += 1
        yield harvest
        after = after + 1 if harvest.done else 0
        if after == round(FALL / SIM_DT):
            # A truss is in the crate if every body cut free is over its
            # footprint and above its underside.
            crate = np.linalg.inv(harvest.platform() @ machine.crate_frame)
            points = NewtonManager.get_state_0().body_q.numpy()[:, :3] @ crate[:3, :3].T
            points += crate[:3, 3]
            footprint = np.array([machine.crate.length, machine.crate.width]) / 2
            inside = (np.abs(points[:, :2]) < footprint).all(axis=1) & (points[:, 2] > 0.0)
            loose = shears.bodies[shears.loose]
            tally.crated = sum(inside[np.intersect1d(t.bodies, loose)].all() for t in harvest.cut)
            print(f"[INFO]: done in {harvest.time:.0f} s. {tally.summary()}", flush=True)
            if not sim.has_gui:
                return


def main():
    args = parse_args()
    sim_cfg = simulation_cfg()
    with launch_simulation(sim_cfg, args):
        # The crate holds hundreds of contacts once it fills.
        rods.widen_contact_threads()
        sim = sim_utils.SimulationContext(sim_cfg)
        machine = make_machine()
        robot = design_scene(machine)
        colliders = None
        for step, _ in enumerate(run(sim, robot, machine, args.paths)):
            # Colliders only move on a control tick, when the stroke places
            # the shear's slices or a cut reshapes a peduncle.
            if args.colliders and step % DECIMATION == 0:
                if colliders is None:
                    colliders = ColliderView()
                else:
                    colliders.update()


if __name__ == "__main__":
    main()

"""Harvest a greenhouse's ripe trusses with a pipe-rail robot, from ground truth.

Two short rows whose lowest trusses are rods Newton's VBD bends, and the
`harvester` in the main path at their heads. It crosses to each path in
turn and works down it and back: every due truss on its left going in, every
one on its right coming out. Each is cut at its peduncle, by its stem, held
by it, pulled out of the crop into the path, carried to the crate on the
lift and let go over its middle.

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
from harvester import AISLE, ARM_HOME, HAND, LIFT, RAIL, Harvester, arm_joints, harvester_cfg
from kinematics import frames, rotation, solve, transform
from misina_lab.isaaclab import ColliderView, rods
from shear import SHEAR_JOINT, Stroke

BELOW = 1.8
"""Height, in meters, below which a truss's node puts it on a rod: the band
the arm works, and the trusses a cut can drop."""

GREENHOUSE_CFG = GreenhouseCfg(
    layout=LayoutCfg(rows=2, row_length=4.0),
    stem=StemCfg(flexible_below=BELOW),
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

STAGES = ("set", "expanding", "mature_green", "breaker", "turning", "pink", "light_red", "red")
"""A fruit's `greenhouse_lab:stage`, from fruit set to red."""
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
DROP = 0.42
"""How high above the crate's floor the jaws carry a truss and let it go,
over its middle, in meters: the truss hanging from them clears the crate's
rim."""
ASIDE = 0.12
"""How far along the crate from its middle the jaws let a truss go where
the middle is out of the arm's reach, in meters: the middle is straight
behind the shoulder, at the end of its range."""
HELD = 0.015
"""How near the middle of the jaws, in meters, a truss's rod cut free
passes for the jaws to have it: a peduncle anywhere between them passes
within this. One they have as the shear shuts is held there until it
opens."""
TIP = 1.2
"""How far the blades are turned down about the peduncle, in radians, to
tip out a truss the open finger still bears: one lying across it slides
off once it slopes past the rubber's 45 degrees."""
PATIENCE = 300
"""Control ticks a stage may wait on the robot before its job is given up
on: 6 s."""
FALL = 3.0
"""Seconds the last truss is given to fall before the tally."""

# A rod body's label ends in its segment's prim; segment 0 is the held one.
SEGMENT = re.compile(r"(.*)/Cable_edge_body_(\d+)$")
# A stem's collider's label is its prim: one capsule along each span of it.
STEM = re.compile(r"(.*/Stem_\d+)/Collision_\d+$")


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
    level = heading(along, truss.side)
    off = at - truss.axis
    reach = np.linalg.norm(off - (off @ truss.stem) * truss.stem)
    toward = np.sign(np.cross(truss.stem, level) @ truss.side)
    poses = []
    for yaw in YAWS:
        turn = rotation(truss.stem, toward * yaw)
        x, ahead = turn @ along, turn @ level
        up = np.cross(ahead, x)
        through = at + (reach / np.cos(yaw) - reach) * along
        for tilt in TILTS:
            approach = np.cos(tilt) * ahead + np.sin(tilt) * up
            pose = np.eye(4)
            pose[:3, 0], pose[:3, 1], pose[:3, 2] = x, np.cross(approach, x), approach
            pose[:3, 3] = through - machine.shear.bypass * x
            poses.append(pose)
    return poses


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
    the path, a swing round to over the crate, and the shear opening there,
    letting the truss go, and tipped down if it still lies on the finger. A
    shear that cut nothing opens and backs out, and the cut is planned
    afresh, `TRIES` times in all. `stroke` is the shear. The robot's base is
    at `base` in the world.

    A cut is planned clear of everything first, but for the held segment and
    the next, which the shear closes on, and the stretch of its stem by the
    cut, which the shear slides along (`planner.BESIDE`). Failing that, the
    arm may brush the truss it cuts, and failing that any truss: the rods
    bend aside, as the crop does round a picker's arm. The trolley, the
    rails, the floor and every other stretch of stem it never touches.
    """

    def __init__(
        self, machine: Harvester, planner, jobs: list, base: np.ndarray, q, shears, tally: Tally
    ):
        self.machine, self.planner, self.base = machine, planner, base
        self.shears, self.tally = shears, tally
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
        self.held = None
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
        """Swing the truss held round the arm's base to over the crate's
        middle, or `ASIDE` off it either way, the jaws `DROP` above its floor,
        or let it go where it is if the arm can get to none of them. The mouth
        ends turned about the upright alone, so the truss hangs in the jaws as
        it did when cut, its weight pulling away from the mouth's entrance. On
        the way the arm may brush any truss, which leaves `planner` fewer to
        keep off and the mouth less to turn."""
        platform = self.platform()
        mouth = np.linalg.inv(platform) @ hand @ self.machine.tool
        hold = np.array([self.machine.shear.hold, 0.0, 0.0, 1.0])
        jaws = (mouth @ hold)[:3]
        carrying = None
        for along in (0.0, ASIDE, -ASIDE):
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

    def _over_crate(self, along: float = 0.0) -> np.ndarray:
        """Where the jaws let a truss go, in the platform's frame: `DROP` over
        the crate's floor, `along` its length from its middle."""
        return (self.machine.crate_frame @ [along, 0.0, self.machine.crate.floor + DROP, 1.0])[:3]

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
                self._plan()
            elif settled:
                self._next()
        elif self.stage == "close":
            if self.stroke.tick(hand, angle, closing=True):
                self._shut(hand)
        elif self.stage == "open":
            if self.stroke.tick(hand, angle, closing=False):
                # A truss let go still lying on the finger is tipped out.
                if self.severed and self.stroke.fouled(hand):
                    self._tip(hand)
                else:
                    self._opened()
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
                if settled and self.stage == "approach":
                    self._play("inward", self.planned.inward)
                elif settled and self.stage == "outward" and self.severed:
                    self._pull(hand)
                elif settled and self.stage == "pull":
                    self._carry(hand)
                elif settled and self.stage == "carry":
                    self._release(hand)
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
    truss collides with."""
    return sim_utils.SimulationCfg(
        dt=SIM_DT,
        physics=rods.make_physics_cfg_newton(GREENHOUSE_CFG, ROBOT_PATH, ROBOT_CONTACT, SUBSTEPS),
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
) -> Iterator[Harvest]:
    """Reset the simulation of the scene `design_scene` laid out and harvest
    `paths`, every one by default, for as long as the run lasts, yielding the
    `Harvest` after every physics step. `FALL` seconds after its last job the
    tally is printed, and a run without a window ends. Without `render` no
    step renders, and what does is up to the caller."""
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
    stems = stem_axes(NewtonManager.get_model())
    rows: dict[str, list[int]] = {}
    for stem, (shapes, _, _) in stems.items():
        # A stem's row is its plant's parent.
        rows.setdefault(stem.rsplit("/", 2)[0], []).extend(shapes)
    for truss in trusses:
        truss.bodies = [body for _, body in sorted(chains[truss.prim])]
        shapes, starts, ends = stems[truss.prim.rsplit("/", 1)[0]]
        nearest = distances(truss.cut[:3, 3], starts, ends).argmin()
        truss.stem = (ends - starts)[nearest] / np.linalg.norm((ends - starts)[nearest])
        truss.axis = starts[nearest]
        truss.stem_shapes = list(shapes)
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
    harvest = Harvest(machine, planner, jobs, base, spawned, shears, tally)
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

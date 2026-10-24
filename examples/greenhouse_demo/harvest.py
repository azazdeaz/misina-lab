"""Harvest a greenhouse's ripe trusses with a pipe-rail robot, from ground truth.

Two short rows whose lowest trusses are rods Newton's VBD bends, and the
`harvester` in the main path at their heads. It crosses to each path in
turn and works down it and back: every due truss on its left going in, every
one on its right coming out. Each is cut at its peduncle, by its stem, and
drops to the floor; nothing catches it.

What is due and where to cut it come from the scene's own ground truth, not
from a camera. A truss is due once its least ripe fruit is turning, as the
generator stages them, and is cut at its `Cut` frame. For each, the arm folds
and the trolley runs to a station behind the cut on the rail and the lift.
cuRobo plans the arm from there, around the simulation's own colliders (see
`planner`), to a stand-off and in on a straight line, and once it holds still
with the mouth within `TOLERANCE` of the cut, `Shears` cuts the truss there.

    uv run harvest.py                        # in Kit's window, running on once done
    HEADLESS=1 uv run harvest.py --viz kit   # without one, printing the tally
"""

from __future__ import annotations

import argparse
import dataclasses
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
from kinematics import frames
from misina_lab.isaaclab import rods

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
ROBOT_CONTACT = [rf"{ROBOT_PATH}/.*(forearm|wrist_[123])_link"]
"""The parts of the robot a truss meets: the forearm and the wrists, the
cutter's head riding on the last. Each costs a proxy in the solver that bends
the trusses."""

SIM_DT = 0.005
SUBSTEPS = 4
"""Four substeps of the 1.25 ms the rod tuning was measured at."""
DECIMATION = 4
"""Physics steps per control tick: the robot is driven at 50 Hz."""

STAGES = ("set", "expanding", "mature_green", "breaker", "turning", "pink", "light_red", "red")
"""A fruit's `greenhouse_lab:stage`, from fruit set to red."""
DUE = STAGES.index("turning")

STANDOFF = 0.12
"""How far short of the cut, back along the cutter, the straight line in starts."""
BACK = 0.3
"""How far behind the cut, down the rail, the arm's base is stationed."""
RISE = 0.15
"""How far above the shoulder the cut is held, where the lift reaches."""
TILTS = np.radians([0.0, 20.0, -20.0])
"""How far off level the cutter may come in. Square to the peduncle, it has
the peduncle across the V. Level, it is square to a standing stem too, which
then crosses the V's plane at the mouth and nowhere nearer the head; tilted,
the stem crosses toward the head, and 20 degrees keeps it off its corner."""
STEPS = np.array([0.02, 0.02, 0.01])
"""How far each trolley joint's target may move per control tick on a run,
every joint pacing itself to arrive with the slowest: 1 m/s across and down a
path, 0.5 m/s up."""
ARRIVED = 0.05
"""How close every joint has to be to its goal, in radians or meters."""
STILL = 0.002
"""How little every joint may still move per control tick for the robot to
count as settled: a tenth of a radian a second."""
TOLERANCE = 0.015
"""How close the mouth has to hold to the cut for the truss to be cut, in
meters."""
PATIENCE = 300
"""Control ticks a stage may wait on the robot before its job is given up
on: 6 s."""
FALL = 3.0
"""Seconds the last truss is given to fall before the tally."""
FELL = 0.2
"""How far, in meters, every body of a cut truss has dropped once it fell."""

# A rod body's label ends in its segment's prim; segment 0 is the held one.
SEGMENT = re.compile(r"(.*)/Cable_edge_body_(\d+)$")


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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Harvests a greenhouse's ripe trusses.")
    parser.add_argument(
        "--paths",
        type=int,
        nargs="+",
        help="The paths to work, as the scene numbers them -- Path_00 first. Every one by default.",
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


def approaches(truss: Truss, base: np.ndarray) -> list[np.ndarray]:
    """The mouth's poses that cut `truss`: at its `Cut`, +X along the
    peduncle either way, so it lies across the V, and +Z the way the cutter
    comes in, square to the peduncle and within `TILTS` of level. It comes in
    along the row away from `base`, where the arm stands, so the arm reaches
    past the near side of the cut, not round its far side."""
    along = truss.cut[:3, 0] / np.linalg.norm(truss.cut[:3, 0])
    heading = np.cross([0.0, 0.0, 1.0], along)
    heading /= np.linalg.norm(heading)
    if heading @ (truss.cut[:3, 3] - base) < 0:
        heading = -heading
    up = np.cross(along, heading)
    poses = []
    for tilt in TILTS:
        approach = np.cos(tilt) * heading + np.sin(tilt) * up
        for x in (along, -along):
            pose = np.eye(4)
            pose[:3, 0], pose[:3, 1], pose[:3, 2] = x, np.cross(approach, x), approach
            pose[:3, 3] = truss.cut[:3, 3]
            poses.append(pose)
    return poses


def station(machine: Harvester, cut: np.ndarray, aisle: float) -> np.ndarray:
    """Where the trolley stands for a cut at `cut`, a point in the robot's
    base frame: across at `aisle`, the arm's base `BACK` behind the cut down
    the rail, and the lift holding the cut `RISE` above the shoulder. The
    three joints' positions."""
    chain = machine.chain
    shoulder = chain[2].xyz[2] + chain[3].xyz[2]
    wanted = [aisle, cut[0] - BACK, cut[2] - RISE - shoulder]
    return np.clip(wanted, *zip(*(joint.limits for joint in chain[:3]), strict=True))


@dataclasses.dataclass
class Tally:
    due: int = 0
    reachable: int = 0
    cut: int = 0
    crowded: int = 0
    """Reachable only by brushing a truss."""
    missed: int = 0
    fell: int = 0
    seconds: float = 0.0

    def summary(self) -> str:
        share = lambda part: f"{100 * part / self.due:.0f}%" if self.due else "n/a"  # noqa: E731
        return (
            f"{self.due} trusses due: {share(self.reachable)} reachable, "
            f"{self.crowded} of them only by brushing a truss, "
            f"{share(self.cut)} cut, {share(self.fell)} fell, {self.missed} given up on, "
            f"{self.seconds / max(self.cut, 1):.1f} s per truss cut"
        )


class Harvest:
    """The jobs `plan` lists, done one at a time, a control tick at a time.

    `control` takes where the joints are and the mouth's world pose, and
    returns every joint's position and velocity targets. The trolley runs
    only with the arm folded, and `planner` plans the fold from wherever the
    arm is. A joint position is a run to it. A truss is a run to its station,
    and then the cut `planner` plans from there: the move to a stand-off and
    the straight line in, holding still until the mouth is at the cut, the
    cut, and the line back out. The robot's base is at `base` in the world.

    A cut is planned clear of everything first, but for the held segment and
    the next, which the cutter closes on. Failing that, the arm may brush
    the truss it cuts, and failing that any truss: the rods bend aside, as
    the crop does round a picker's arm. The trolley, the rails and the floor
    it never touches.
    """

    def __init__(
        self, machine: Harvester, planner, jobs: list, base: np.ndarray, q, shears, tally: Tally
    ):
        self.machine, self.planner, self.base = machine, planner, base
        self.shears, self.tally = shears, tally
        self.jobs = list(jobs)
        self.q = np.array(q, dtype=float)
        self.target = self.q.copy()
        self.velocity = np.zeros_like(self.q)
        self.cut: list[Truss] = []
        """The trusses cut, in order."""
        self.time = 0.0
        self.soft: set[int] = set()
        """The rod bodies the arm may brush, those the last cut was planned through."""
        self._next()

    @property
    def done(self) -> bool:
        return self.stage == "done"

    def platform(self) -> np.ndarray:
        """The platform's pose in the world, the trolley where it is held: a
        run overshoots its station by centimetres before it settles there."""
        return self.base @ frames(self.machine.chain[:3], self.target[:3])[-1]

    def _next(self) -> None:
        """Start on the next job, or finish."""
        while self.jobs:
            self.job = self.jobs.pop(0)
            if not isinstance(self.job, Truss):
                self.goal = self.job[:3]
            elif self.shears.loose[np.searchsorted(self.shears.bodies, self.job.bodies[1])]:
                continue
            else:
                cut = np.linalg.inv(self.base) @ self.job.cut[:, 3]
                self.goal = station(self.machine, cut, self.target[0])
                self.started = self.time
            self._fold()
            return
        self.job, self.stage = None, "done"

    def _fold(self) -> None:
        """Fold the arm where the trolley stands, unless it is folded, to run."""
        self.stage, self.ticks = "run", 0
        if np.abs(self.q[3:] - ARM_HOME).max() < ARRIVED:
            return
        folding = self.planner.fold(self.q[3:], self.platform(), self.soft)
        if folding is None:
            # From a pose already touching what it may not brush, a fold
            # finds no way; the arm folds on a straight line instead.
            print(f"[INFO]: no clear way to fold from {np.round(self.q[3:], 2)}")
            line = np.linspace(
                self.q[3:], ARM_HOME, 1 + int(np.abs(ARM_HOME - self.q[3:]).max() / 0.04)
            )
            folding = (line, np.gradient(line, self.planner.dt, axis=0))
        self._play("fold", folding)

    def _plan(self) -> None:
        """Plan the cut of the truss the trolley stands for, or give it up."""
        truss, platform = self.job, self.platform()
        poses = approaches(truss, platform[:3, 3])
        for soft in (set(truss.bodies[:2]), set(truss.bodies), set(self.shears.bodies)):
            planned = self.planner.cut(self.q[3:], platform, poses, STANDOFF, soft)
            if planned is not None:
                break
        if planned is None:
            print(f"[INFO]: {truss.prim}: out of reach")
            self._next()
            return
        self.tally.reachable += 1
        self.tally.crowded += len(soft) > 2
        self.planned, self.soft = planned, soft
        self._play("approach", planned.approach)

    def _play(self, stage: str, trajectory: tuple[np.ndarray, np.ndarray]) -> None:
        self.stage, self.ticks, self.clock = stage, 0, 0.0
        self.trajectory = trajectory

    def control(self, q: np.ndarray, mouth: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Advance one tick, `q` every joint's position and `mouth` the mouth's
        world pose. Returns every joint's position and velocity targets."""
        moved = float(np.abs(q - self.q).max())
        self.q = np.array(q, dtype=float)
        self.velocity[:] = 0.0
        if self.done:
            return self.target, self.velocity
        tick = DECIMATION * SIM_DT
        self.time += tick
        self.ticks += 1
        settled = np.abs(self.target - self.q).max() < ARRIVED and moved < STILL
        if self.ticks > PATIENCE and self.stage == "cut":
            # Backed out along the line it came in on, clear of the crop.
            print(f"[INFO]: {self.job.prim}: gave up at the cut")
            self.tally.missed += 1
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
        elif self.stage == "cut":
            # Held still with the mouth at the cut, the truss is cut at its
            # first joint, by the stem, which `Cut` sits within a centimetre of.
            if moved < STILL and np.linalg.norm(mouth[:3, 3] - self.job.cut[:3, 3]) <= TOLERANCE:
                self.shears.cut(self.job.bodies[1], 0.0)
                print(f"[INFO]: {self.job.prim}: cut after {self.time - self.started:.1f} s")
                self.cut.append(self.job)
                self.tally.cut += 1
                self.tally.seconds += self.time - self.started
                self._play("outward", self.planned.outward)
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
                elif settled and self.stage == "outward":
                    self._next()
                elif settled:
                    self.stage, self.ticks = {"fold": "run", "inward": "cut"}[self.stage], 0
        return self.target, self.velocity


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
    shears = Shears()
    chains: dict[str, list[tuple[int, int]]] = {}
    for body, label in enumerate(shears.labels):
        if match := SEGMENT.match(label):
            chains.setdefault(match[1], []).append((int(match[2]), body))
    trusses = read_trusses(stage, GREENHOUSE_PATH)
    for truss in trusses:
        truss.bodies = [body for _, body in sorted(chains[truss.prim])]
    ys = path_ys(stage, GREENHOUSE_PATH)
    paths = paths if paths is not None else list(range(len(ys)))
    if not all(0 <= path < len(ys) for path in paths):
        raise ValueError(f"no path {paths}: the greenhouse has {len(ys)} paths")
    jobs = plan(trusses, ys, paths, GREENHOUSE_CFG.layout.pitch)
    tally = Tally(due=sum(isinstance(job, Truss) for job in jobs))

    joints, _ = robot.find_joints([AISLE, RAIL, LIFT, *arm_joints()], preserve_order=True)
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

    def mouth() -> np.ndarray:
        hand_pose = frame_of(
            robot.data.body_link_pos_w.torch[0, hand[0]].cpu().numpy(),
            robot.data.body_link_quat_w.torch[0, hand[0]].cpu().numpy(),
        )
        return hand_pose @ machine.tool

    # A drive with no target holds zero, so the arm is held home first.
    hold([0.0, 0.0, 0.0, *ARM_HOME], np.zeros(len(joints)))
    # Where the robot spawns: its joints read zero until the first step.
    spawned = [0.0, 0.0, 0.0, *ARM_HOME]
    harvest = Harvest(machine, Planner(machine, ROBOT_PATH), jobs, base, spawned, shears, tally)
    print(f"[INFO]: {len(trusses)} trusses bend, {tally.due} due on paths {paths}", flush=True)
    sim.set_camera_view(eye=[-3.5, -2.5, 2.8], target=[1.5, 0.0, 1.0])
    rest = NewtonManager.get_state_0().body_q.numpy()[:, 2].copy()
    step, after = 0, 0
    while sim.is_running():
        if step % DECIMATION == 0:
            hold(*harvest.control(robot.data.joint_pos.torch[0, joints].cpu().numpy(), mouth()))
        robot.write_data_to_sim()
        sim.step(render=render)
        robot.update(SIM_DT)
        step += 1
        yield harvest
        after = after + 1 if harvest.done else 0
        if after == round(FALL / SIM_DT):
            drop = rest - NewtonManager.get_state_0().body_q.numpy()[:, 2]
            tally.fell = sum(drop[t.bodies[1:]].min() > FELL for t in harvest.cut)
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
        for _ in run(sim, robot, machine, args.paths):
            pass


if __name__ == "__main__":
    main()

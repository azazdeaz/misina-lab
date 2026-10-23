"""Harvest a greenhouse's ripe trusses with a pipe-rail robot, from ground truth.

Two short rows whose lowest trusses are rods Newton's VBD bends, and the
`harvester` in the main path at their heads. It crosses to each path in
turn and works down it and back: every due truss on its left going in, every
one on its right coming out. Each is cut at its peduncle, by its stem, and
drops to the floor; nothing catches it.

What is due and where to cut it come from the scene's own ground truth, not
from a camera. A truss is due once its least ripe fruit is turning, as the
generator stages them, and is cut at its `Cut` frame. For each, the trolley
runs to a station behind the cut on the rail and the lift, the arm moves to a
stand-off and closes in on a straight line, and once it holds still with the
mouth within `TOLERANCE` of the cut, `Shears` cuts the truss there.

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
from kinematics import solve
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
"""How far short of the cut, along the approach, the planned move ends."""
BACK = 0.3
"""How far behind the cut, along the approach, the arm's base is stationed."""
RISE = 0.15
"""How far above the shoulder the cut is held, where the lift reaches."""
HOP = 0.5
"""How far the trolley may run with the arm out, in meters. A longer run
folds the arm where the trolley stands first and reaches out again at the
end, so the arm never sweeps the crop wall."""
STEPS = np.array([0.02, 0.02, 0.01, *[0.04] * 6])
"""How far each joint's target may move per control tick on a planned move,
every joint pacing itself to arrive with the slowest: 1 m/s across and down
a path, 0.5 m/s up, and 2 rad/s on the arm, under the UR5e's 3.14."""
CLOSE_IN = 0.008
"""How far the mouth moves per control tick on the straight line: 0.4 m/s."""
ARRIVED = 0.05
"""How close every joint has to be to its goal, in radians or meters."""
STILL = 0.002
"""How little every joint may still move per control tick for the robot to
count as settled: a tenth of a radian a second."""
TOLERANCE = 0.015
"""How close the mouth has to hold to the cut for the truss to be cut, in
meters."""
REACH_TOLERANCE = (0.005, 0.05)
"""How close the solver has to get a pose, in meters and radians, for it to
count as reached."""
PATIENCE = 300
"""Control ticks a stage may wait on the robot before its job is given up
on: 6 s."""
FALL = 3.0
"""Seconds the last truss is given to fall before the tally."""
FELL = 0.2
"""How far, in meters, every body of a cut truss has dropped once it fell."""

SEEDS = (
    (0.0, -1.2, 1.6, -0.4, 1.57, 0.0),
    (np.pi, -1.2, 1.6, -0.4, 1.57, 0.0),
    (0.0, -1.9, -1.6, 0.4, -1.57, 0.0),
    (np.pi, -1.9, -1.6, 0.4, -1.57, 0.0),
)
"""Arm postures the solver restarts from when the one the arm is in folds it
away from a cut: reaching out to either side, elbow up or down."""

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


def squared(truss: Truss) -> np.ndarray:
    """The mouth's frame at the cut: +X along the peduncle, so it lies across
    the V, and +Z the approach, level and square to the peduncle, from down
    the row -- or from the path, where the peduncle runs along the row."""
    tangent = truss.cut[:3, 0] / np.linalg.norm(truss.cut[:3, 0])
    approach = np.cross([0.0, 0.0, 1.0], tangent)
    approach /= np.linalg.norm(approach)
    if approach @ (truss.side + [0.5, 0.0, 0.0]) < 0:
        approach = -approach
    pose = np.eye(4)
    pose[:3, 0] = tangent
    pose[:3, 1] = np.cross(approach, tangent)
    pose[:3, 2] = approach
    pose[:3, 3] = truss.cut[:3, 3]
    return pose


def station(machine: Harvester, pose: np.ndarray, aisle: float) -> np.ndarray:
    """Where the trolley stands for a cut at `pose`, in the robot's base
    frame: across at `aisle`, the arm's base `BACK` behind the cut down the
    rail, and the lift holding the cut `RISE` above the shoulder. The three
    joints' positions."""
    chain = machine.chain
    shoulder = chain[2].xyz[2] + chain[3].xyz[2]
    wanted = [aisle, pose[0, 3] - BACK * pose[0, 2], pose[2, 3] - RISE - shoulder]
    return np.clip(wanted, *zip(*(joint.limits for joint in chain[:3]), strict=True))


def held(chain, q) -> tuple:
    """`chain` with the trolley's joints held where `q` has them."""
    return tuple(
        dataclasses.replace(joint, limits=(value, value)) if joint.prismatic else joint
        for joint, value in zip(chain, q, strict=True)
    )


def reach(machine: Harvester, pose: np.ndarray, q: np.ndarray) -> tuple[np.ndarray, bool]:
    """Joint positions for `pose`, in the robot's base frame, with the trolley
    held where `q` has it: from the arm's own posture or, failing that, from
    each of `SEEDS`. The closest try, and whether any reached it."""
    chain = held(machine.chain, q)
    best = None
    for seed in (q[3:], *SEEDS):
        result = solve(chain, machine.tool, pose, [*q[:3], *seed])
        if result[1] <= REACH_TOLERANCE[0] and result[2] <= REACH_TOLERANCE[1]:
            return result[0], True
        if best is None or result[1] < best[1]:
            best = result
    return best[0], False


@dataclasses.dataclass
class Tally:
    due: int = 0
    reachable: int = 0
    cut: int = 0
    missed: int = 0
    fell: int = 0
    seconds: float = 0.0

    def summary(self) -> str:
        share = lambda part: f"{100 * part / self.due:.0f}%" if self.due else "n/a"  # noqa: E731
        return (
            f"{self.due} trusses due: {share(self.reachable)} reachable, "
            f"{share(self.cut)} cut, {share(self.fell)} fell, {self.missed} given up on, "
            f"{self.seconds / max(self.cut, 1):.1f} s per truss cut"
        )


class Harvest:
    """The jobs `plan` lists, done one at a time, a control tick at a time.

    `control` takes where the joints are and the mouth's world pose, and
    returns every joint's target. A joint position is a planned move; a truss
    is a planned move to its station and stand-off -- the arm folded first
    when the rail has far to run -- a straight line in, holding still until
    the mouth is at the cut, the cut, and the line back out. Poses are solved
    in the robot's base frame, `base` in the world.
    """

    def __init__(self, machine: Harvester, jobs: list, base: np.ndarray, q, shears, tally: Tally):
        self.machine, self.base, self.shears, self.tally = machine, base, shears, tally
        self.jobs = list(jobs)
        self.q = np.array(q, dtype=float)
        self.target = self.q.copy()
        self.cut: list[Truss] = []
        """The trusses cut, in order."""
        self.time = 0.0
        self._next()

    @property
    def done(self) -> bool:
        return self.stage == "done"

    def _next(self) -> None:
        """Start on the next job, or finish."""
        self.ticks = 0
        while self.jobs:
            self.job = self.jobs.pop(0)
            if not isinstance(self.job, Truss):
                self.stage, self.goals = "move", self._route(self.job)
                return
            if self.shears.loose[np.searchsorted(self.shears.bodies, self.job.bodies[1])]:
                continue
            self.pose = np.linalg.inv(self.base) @ squared(self.job)
            self.standoff = self.pose.copy()
            self.standoff[:3, 3] -= STANDOFF * self.pose[:3, 2]
            here = self.q.copy()
            here[:3] = station(self.machine, self.pose, self.q[0])
            goal, ok = reach(self.machine, self.standoff, here)
            if not (ok and reach(self.machine, self.pose, goal)[1]):
                print(f"[INFO]: {self.job.prim}: out of reach")
                continue
            self.tally.reachable += 1
            self.stage, self.goals, self.started = "move", self._route(goal), self.time
            self.line = self.standoff[:3, 3].copy()
            return
        self.job, self.stage = None, "done"

    def _route(self, goal: np.ndarray) -> list[np.ndarray]:
        """The joint positions a planned move to `goal` goes through: `goal`
        alone for a hop, or for a longer run the arm folded where the trolley
        stands, the trolley run with it folded, and `goal`."""
        if np.abs(goal[:2] - self.target[:2]).max() <= HOP:
            return [goal]
        return [np.array([*self.target[:3], *ARM_HOME]), np.array([*goal[:3], *ARM_HOME]), goal]

    def control(self, q: np.ndarray, mouth: np.ndarray) -> np.ndarray:
        """Advance one tick, `q` every joint's position and `mouth` the mouth's
        world pose. Returns every joint's target."""
        moved = float(np.abs(q - self.q).max())
        self.q = np.array(q, dtype=float)
        if self.done:
            return self.target
        self.time += DECIMATION * SIM_DT
        self.ticks += 1
        if self.ticks > PATIENCE:
            print(f"[INFO]: {getattr(self.job, 'prim', 'a move')}: gave up at {self.stage}")
            self.tally.missed += isinstance(self.job, Truss)
            self._next()
            return self.target
        if self.stage == "move":
            gap = self.goals[0] - self.target
            if np.abs(gap).any():
                # Every joint paced to arrive with the slowest; patience
                # counts only the wait for the robot to catch up.
                self.target = self.target + gap / max(np.abs(gap / STEPS).max(), 1.0)
                self.ticks = 0
                return self.target
            if np.abs(self.goals[0] - self.q).max() >= ARRIVED:
                return self.target
            self.goals.pop(0)
            if self.goals:
                return self.target
            if not isinstance(self.job, Truss):
                self._next()
                return self.target
            self.stage, self.ticks = "approach", 0
        if self.stage in ("approach", "retract"):
            # The straight line, a step of the mouth at a time, each solved
            # from where the arm is; the end of it held until the robot is
            # there and still.
            end = (self.pose if self.stage == "approach" else self.standoff)[:3, 3]
            gap = end - self.line
            distance = float(np.linalg.norm(gap))
            if distance > 1e-6:
                self.line = self.line + gap * min(CLOSE_IN / distance, 1.0)
                pose = self.pose.copy()
                pose[:3, 3] = self.line
                chain = held(self.machine.chain, self.target)
                start = [*self.target[:3], *self.q[3:]]
                self.target, _, _ = solve(chain, self.machine.tool, pose, start, iterations=50)
            elif np.abs(self.target - self.q).max() < ARRIVED and moved < STILL:
                if self.stage == "retract":
                    self._next()
                    return self.target
                self.stage, self.ticks = "cut", 0
        if self.stage == "cut":
            # Held still with the mouth at the cut, the truss is cut at its
            # first joint, by the stem, which `Cut` sits within a centimetre of.
            if np.linalg.norm(mouth[:3, 3] - self.job.cut[:3, 3]) <= TOLERANCE:
                self.shears.cut(self.job.bodies[1], 0.0)
                print(f"[INFO]: {self.job.prim}: cut after {self.time - self.started:.1f} s")
                self.cut.append(self.job)
                self.tally.cut += 1
                self.tally.seconds += self.time - self.started
                self.stage, self.ticks = "retract", 0
        return self.target


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

    def hold(targets) -> None:
        robot.actuators.target_command.set_position_index(
            value=torch.tensor(targets, dtype=torch.float32, device=robot.device).unsqueeze(0),
            joint_ids=joints,
        )

    def mouth() -> np.ndarray:
        hand_pose = frame_of(
            robot.data.body_link_pos_w.torch[0, hand[0]].cpu().numpy(),
            robot.data.body_link_quat_w.torch[0, hand[0]].cpu().numpy(),
        )
        return hand_pose @ machine.tool

    # A drive with no target holds zero, so the arm is held home first.
    hold([0.0, 0.0, 0.0, *ARM_HOME])
    harvest = Harvest(
        machine, jobs, base, robot.data.joint_pos.torch[0, joints].cpu().numpy(), shears, tally
    )
    print(f"[INFO]: {len(trusses)} trusses bend, {tally.due} due on paths {paths}", flush=True)
    sim.set_camera_view(eye=[-3.5, -2.5, 2.8], target=[1.5, 0.0, 1.0])
    rest = NewtonManager.get_state_0().body_q.numpy()[:, 2].copy()
    step, after = 0, 0
    while sim.is_running():
        if step % DECIMATION == 0:
            hold(harvest.control(robot.data.joint_pos.torch[0, joints].cpu().numpy(), mouth()))
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

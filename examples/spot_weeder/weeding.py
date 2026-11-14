"""The paper's weeding pipeline over planted feet, its §3.2-3.4, with oracle
detection: the weeds' positions come from the scene, as the L515 would
measure them from where it is mounted, instead of from its pictures.

The robot scans twice, yawed right and then left, as its camera sees only
part of E at a time. After each scan the removal planner keeps the weeds
found in E (the paper's Eq. 5) that the robot can also reach and lean over,
and sorts them right to left. It removes them one by one: it moves the bit
just above and behind the weed, finds the weed again standing still,
corrects once (Eq. 6) to aim 2 cm into the ground under it, and mills for a
second.

Points are in the ground frame of `spot_weeder`, the paper's initial body
frame, and poses are as `pose` has them. The robots of a batch, one scene
each, work in step: each move takes them all the same time, and a robot with
nothing to do in one holds still.

    python weeding.py   # weeds one scene on flat ground, headless, and says what became of each weed
"""

import math

import isaaclab.sim as sim_utils
import numpy as np
import pose
import spot_weeder
from isaaclab.app import launch_simulation
from spot_weeder import TIP, E, inside, pose_for, reachable

WEEDS = 5
"""Weeds per scene, all in E: the paper's density, 5 per workspace, which it
calls realistic."""

SPACING = 0.1
"""How far apart the weeds are at the least, so that the weed found nearest
to the one aimed at is that one."""

SCANS = (-math.pi / 18, math.pi / 18)
"""The yaws the robot scans from, right and then left."""

CAMERA = TIP + np.array([0.0, 0.0, spot_weeder.LENS])
"""Where the L515's lens is in the body's frame."""

VIEW = np.tan(np.radians(spot_weeder.FIELD_OF_VIEW[::-1]) / 2)
"""How far the camera sees to either side for each metre down, along the
body's x and y."""

NOISE = 0.003
"""The detection's error in each direction, in metres, for the runs with
Spot's errors. The paper gives none; 3 mm is about two pixels of its
320 x 240 pictures from 30 cm up."""

AIM = 0.02
"""How far under the weed the bit is aimed, into the ground, as in the
paper."""

APPROACH = np.array([-0.03, 0.0, 0.04])
"""Where the bit waits while the robot finds the weed again, from the weed:
just behind and above it, where the tool doesn't hide it."""

SAME = 0.1
"""How far from the weed aimed at the nearest weed found again may be and
still be taken for it: the paper's 10 cm."""

DONE = 0.05
"""How close to a weed the robot aimed at before a weed the second scan
finds is taken for that one, and left."""

HIT = 0.011
"""How close the bit's centre has to come to a weed's stem, in the ground,
to mill it out: half the 22 mm bit."""

LEAN = 0.04
"""How far inside the outline of its feet the planner keeps the robot's
centre of mass, in metres. With none, robots tip over their front feet
while they mill at the front of E; 2 cm keeps them standing as the
controller tracks, and Spot's errors take 2 cm more."""

FOOTING = np.array([0.05, 0.05, 0.05, 0.1, 0.1, 0.1])
"""How far, in metres and radians, a body may be from the pose it was sent
after it held still before the robot counts as having lost its footing."""

CORRECT = 1.0
"""How long the correction takes, in seconds. With the move before it, it
makes `pose.MOVE`, the paper's 4.8 s of movement per weed."""

REACH = pose.MOVE - CORRECT
"""How long a move to a weed, or to a scan, takes, in seconds."""

DETECT = 2.0
"""How long the robot stands still for a detection, in seconds: the paper's
1.994."""

MILL = 1.0
"""How long the bit mills, in seconds, as in the paper."""

UNSEEN, OUTSIDE, FAR, LOST, MISSED, REMOVED = range(6)
"""What becomes of a weed, each over the ones before: not found; found
outside E; found in E but out of the robot's reach or balance; aimed at,
not found again and missed; aimed at, found again and missed; milled out."""

OUTCOMES = (
    "not found",
    "found outside E",
    "out of reach",
    "missed, not found again",
    "missed",
    "removed",
)


def scatter(rng: np.random.Generator) -> np.ndarray:
    """`WEEDS` weeds at random in E, `SPACING` apart: (WEEDS, 3)."""
    weeds = []
    while len(weeds) < WEEDS:
        x, y = rng.uniform(E.min(axis=0), E.max(axis=0))
        if inside(E, x, y) and all(math.dist((x, y), weed[:2]) >= SPACING for weed in weeds):
            weeds.append((x, y, 0.0))
    return np.array(weeds)


def frame(poses) -> tuple[np.ndarray, np.ndarray]:
    """Each body's turn and position in the ground frame, (..., 3, 3) and
    (..., 3), for `poses`, (..., 6)."""
    x, y, z, roll, pitch, yaw = np.moveaxis(np.asarray(poses, dtype=float), -1, 0)
    cr, sr, cp, sp = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    # Yawed, pitched and rolled, as `spot_weeder.fk` turns the bit.
    turn = np.stack(
        [
            np.stack([cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr], axis=-1),
            np.stack([sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], axis=-1),
            np.stack([-sp, cp * sr, cp * cr], axis=-1),
        ],
        axis=-2,
    )
    return turn, np.stack([x, y, spot_weeder.Z_NOM + z], axis=-1)


def to_ground(poses, points) -> np.ndarray:
    """Where `points`, (..., 3) in the body's frame, are in the ground frame
    with the body in `poses`, (..., 6)."""
    turn, at = frame(poses)
    return at + np.einsum("...ij,...j->...i", turn, points)


def body_pose(points) -> np.ndarray:
    """The poses that put the bit on `points`, (..., 3), by the paper's
    Eqs. 2-4: (..., 6), NaN where none does."""
    with np.errstate(invalid="ignore"):
        x, pitch, yaw = pose_for(np.moveaxis(np.asarray(points, dtype=float), -1, 0))
    zero = np.zeros_like(x)
    return np.stack([x, zero, zero, zero, pitch, yaw], axis=-1)


def detect(actual, sent, weeds, offset, noise, rng: np.random.Generator) -> np.ndarray:
    """Where the robots find the weeds `weeds`, (n, w, 3): (n, w, 3), NaN for
    the weeds out of the camera's view. Each body stands in `actual`, (n, 6),
    with its tool mounted `offset`, (n, 3), from where the pipeline takes it
    to be, and its camera measures each weed with an error of `noise`, (n,).
    The pipeline places what it measures through the camera's nominal
    mounting from the pose it sent, `sent`: it knows no better where the
    body is."""
    turn, at = frame(actual)
    lens = at + np.einsum("nij,nj->ni", turn, CAMERA + offset)
    # The weeds from the lens, along the body's axes.
    seen = np.einsum("nji,nwj->nwi", turn, weeds - lens[:, None])
    visible = (np.abs(seen[..., :2]) <= -seen[..., 2:] * VIEW).all(axis=-1)
    seen = seen + rng.normal(0.0, 1.0, seen.shape) * np.asarray(noise)[:, None, None]
    turn, at = frame(sent)
    found = at[:, None] + np.einsum("nij,nwj->nwi", turn, CAMERA + seen)
    return np.where(visible[..., None], found, np.nan)


class Pipeline:
    """Weeds a scene for each robot `controller` drives, the weeds of each in
    `weeds`, (n, w, 3). `spot_like` gives each robot the errors of Spot's own
    controller to add to the poses it is sent, or None for none. Each tool is
    mounted `offset`, (n, 3), from where the pipeline takes it to be, and
    each detection errs by `noise`, (n,). `raised` are the feet's blocks.

    `status` says what became of each weed, `aimed` where each robot aimed
    the bit, `closest` how close the bit came to each weed it aimed at, and
    `fell` which robots lost their footing."""

    def __init__(
        self, sim, scene, controller, weeds, spot_like=None, offset=None, noise=None, raised=None
    ):
        n = len(weeds)
        self.sim, self.scene, self.controller, self.weeds = sim, scene, controller, weeds
        self.spot_like = spot_like or [None] * n
        self.offset = np.zeros((n, 3)) if offset is None else offset
        self.noise = np.zeros(n) if noise is None else noise
        self.raised = raised or {}
        self.rng = np.random.default_rng(0)
        self.sent = self.held = np.zeros((n, 6))
        self.status = np.full(weeds.shape[:2], UNSEEN)
        self.aimed = [[] for _ in range(n)]
        self.closest = [[] for _ in range(n)]
        self.fell = np.zeros(n, dtype=bool)

    def run(self) -> None:
        """Scans right and removes what it finds, then left, and stands up
        again."""
        for yaw in SCANS:
            self.remove(self.scan(yaw))
        self.send(np.zeros_like(self.sent), REACH)

    def send(self, poses, seconds: float) -> None:
        """Sends each robot to its pose in `poses`, (n, 6), with its errors,
        and waits while it moves."""
        self.sent = poses
        errors = [
            np.zeros(6) if errs is None else errs(p) for errs, p in zip(self.spot_like, poses)
        ]
        self.held = poses + np.array(errors)
        self.controller.move(self.held, seconds)
        pose.run(self.sim, self.scene, self.controller, seconds)
        self.check()

    def check(self) -> None:
        """Notes the robots that are off the poses they hold."""
        self.fell |= (np.abs(self.controller.pose() - self.held) > FOOTING).any(axis=1)

    def look(self) -> np.ndarray:
        """Stands still for a detection, then finds the weeds, as `detect`."""
        pose.run(self.sim, self.scene, self.controller, DETECT)
        actual = self.controller.pose()
        return detect(actual, self.sent, self.weeds, self.offset, self.noise, self.rng)

    def mark(self, which, status) -> None:
        """Gives the weeds `which`, a mask or indices, `status`, unless they
        came further."""
        self.status[which] = np.maximum(self.status[which], status)

    def scan(self, yaw: float) -> list[list[tuple[int, np.ndarray]]]:
        """Scans from `yaw`, and plans: for each robot, the weeds it is to
        remove, in order, each with where it was found."""
        n, w = self.status.shape
        self.stand = np.tile([0.0, 0.0, 0.0, 0.0, 0.0, yaw], (n, 1))
        self.send(self.stand, REACH)
        found = self.look()
        self.controller.replant()
        seen = ~np.isnan(found[..., 0])
        self.mark(seen, OUTSIDE)
        # Eq. 5, leaving the weeds the robot aimed at before.
        fresh = seen & inside(E, found[..., 0], found[..., 1])
        for i, aimed in enumerate(self.aimed):
            for point in aimed:
                fresh[i] &= np.linalg.norm(found[i] - point, axis=-1) >= DONE
        aim, near = found - (0, 0, AIM), found + APPROACH
        able = fresh & reachable(*np.moveaxis(aim, -1, 0), self.raised)
        able &= reachable(*np.moveaxis(near, -1, 0), self.raised)
        for j in range(w):
            for points in (aim[:, j], near[:, j]):
                able[:, j] &= self.controller.balanced(body_pose(np.nan_to_num(points)), LEAN)
        self.mark(fresh & ~able, FAR)
        # Right to left.
        yaws = body_pose(aim)[..., 5]
        return [
            [(j, found[i, j]) for j in sorted(np.flatnonzero(able[i]), key=lambda j: yaws[i, j])]
            for i in range(n)
        ]

    def remove(self, plans: list[list[tuple[int, np.ndarray]]]) -> None:
        """Removes each robot's weeds in the order `plans` has them. A robot
        done with its weeds pulls the bit out and waits where it scanned."""
        n = len(plans)
        rows = np.arange(n)
        for k in range(max(map(len, plans))):
            busy = np.array([k < len(plan) for plan in plans])
            weed = np.array([plan[k][0] if k < len(plan) else 0 for plan in plans])
            target = np.array([plan[k][1] if k < len(plan) else np.zeros(3) for plan in plans])
            self.send(np.where(busy[:, None], body_pose(target + APPROACH), self.stand), REACH)
            # The weed found nearest to the one aimed at is that one.
            found = self.look()
            gap = np.nan_to_num(np.linalg.norm(found - target[:, None], axis=-1), nan=np.inf)
            nearest = gap.argmin(axis=1)
            again = gap[rows, nearest] <= SAME
            target = np.where(again[:, None], found[rows, nearest], target)
            # Eq. 6: the bit, where the pose sent puts it, moved on by what
            # separates it from the weed found again, which brings it to that
            # weed as the pose sent places it.
            self.send(np.where(busy[:, None], body_pose(target - (0, 0, AIM)), self.stand), CORRECT)
            closest = self.mill()
            self.mark(closest <= HIT, REMOVED)
            for i in np.flatnonzero(busy):
                self.mark((i, weed[i]), MISSED if again[i] else LOST)
                self.aimed[i].append(target[i])
                self.closest[i].append(closest[i, weed[i]])

    def mill(self) -> np.ndarray:
        """Mills for `MILL` seconds: how close the bit's centre came to each
        weed in the ground, (n, w), inf where it stayed out of it."""
        closest = np.full(self.status.shape, np.inf)
        for _ in range(round(MILL / pose.SIM_DT)):
            pose.run(self.sim, self.scene, self.controller, pose.SIM_DT)
            bit = to_ground(self.controller.pose(), TIP + self.offset)
            gap = np.linalg.norm(self.weeds[..., :2] - bit[:, None, :2], axis=-1)
            closest = np.minimum(closest, np.where(bit[:, None, 2] <= 0.0, gap, np.inf))
        self.check()
        return closest


def main():
    """Weeds one scene on flat ground as the controller tracks, prints what
    became of each weed, and checks that each one aimed at came out."""
    with launch_simulation(
        sim_utils.SimulationCfg(physics=pose.PHYSICS), {"headless": True, "visualizer": []}
    ):
        sim, scene, controller = pose.launch()
        weeds = scatter(np.random.default_rng(0))[None]
        pipeline = Pipeline(sim, scene, controller, weeds)
        pipeline.run()
        for (x, y, _), status in zip(weeds[0], pipeline.status[0]):
            print(f"[INFO]: the weed at ({x:.3f}, {y:+.3f}): {OUTCOMES[status]}", flush=True)
        assert not np.isin(pipeline.status, (LOST, MISSED)).any(), pipeline.status


if __name__ == "__main__":
    main()

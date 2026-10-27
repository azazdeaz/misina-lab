"""The harvester's arm, planned by cuRobo in the world the simulation steps.

cuRobo plans the arm alone, from the platform out, while the trolley's three
joints hold where `harvest` stations them. Its robot is
`harvester.curobo_cfg`: the arm's and the shear's shapes, each covered in
spheres, the shear held open. Its world is every collider of the running Newton model within the
arm's reach but the arm's own, each as the box round it, placed where the
simulation has it at the moment of planning: the trolley's chassis and mast,
the rails, the floor, and the trusses and their fruit, bent or fallen. So the
planner keeps off whatever the simulation would collide with, the robot's own
trolley included.

A cut is planned the way cuRobo plans a grasp: of a set of cut poses it
picks one it can reach, the shear's own spheres off, since the shear has to
touch the truss at the end, and plans the move to a stand-off back from it
the way the mouth comes in (`harvester.Harvester.way_in`). From there the
arm closes in on a straight line, solved point by point, and backs out the
same way; `line` solves any other straight line of the mouth's the same way.

Import this only once Kit is up: cuRobo loads Warp, and Newton brings `pxr`.
"""

from __future__ import annotations

import dataclasses
import math

import numpy as np
import torch
from scipy.spatial.transform import Rotation

from curobo.motion_planner import MotionPlanner, MotionPlannerCfg
from curobo.scene import Cuboid, Scene
from curobo.types import GoalToolPose, JointState
from isaaclab_newton.physics import NewtonManager
from newton import GeoType, ShapeFlags

import harvester
from kinematics import solve, transform
from shear import BLADE, FINGER, HEAD

REACH = 1.3
"""How far from the platform's origin a collider is kept in the world, in
meters: past the arm's reach with the shear on it."""
OBSTACLES = 1000
"""The most boxes the world holds; the nearest are kept. cuRobo checks every
sphere against every box in one kernel launch, which caps their product."""
GOALS = 6
"""The most cut poses a cut is planned to at once."""
SPEED = 0.4
"""How fast the mouth moves on the straight line in and out, in m/s."""
INSIDE = 0.02
"""How far inside cuRobo's joint range a straight line keeps the arm, in
radians: cuRobo plans from no pose on the bound itself."""
LINE_TOLERANCE = (0.002, 0.02)
"""How close to the straight line every point of it has to be solved, in
meters and radians."""


@dataclasses.dataclass
class Cut:
    """A planned cut: the arm's joint positions and velocities every `dt`
    seconds on the way to the stand-off, in to the cut and back out."""

    approach: tuple[np.ndarray, np.ndarray]
    inward: tuple[np.ndarray, np.ndarray]
    outward: tuple[np.ndarray, np.ndarray]


class Planner:
    """cuRobo for `machine`, whose articulation is at `robot`.

    Build one after the first `SimulationContext.reset()`, once Newton's
    model exists. Two cuRobo planners are kept, one for the cuts and one for
    the folds: cuRobo captures each kind of problem in CUDA graphs of its own
    shape, and replaying one planner's graphs for the other kind fails. The
    fold planner captures none; a fold plans in a third of a second without.

    It folds once before the cut planner captures its graphs. A fold builds
    a kernel of its own into a Warp module the cut planner's graphs launch
    from, and Warp reloads a module that gains a kernel: a graph captured
    before that runs code that is gone, and crashes once Kit's renderer has
    reused its memory.
    """

    def __init__(self, machine: harvester.Harvester, robot: str):
        self._machine = machine
        cfg = harvester.curobo_cfg(machine)
        cache = {"cuboid": OBSTACLES}
        self._folding = MotionPlanner(
            MotionPlannerCfg.create(robot=cfg, collision_cache=cache, use_cuda_graph=False)
        )
        home = np.array(harvester.ARM_HOME)
        self._folding.plan_cspace(
            self._state(self._folding, home), self._state(self._folding, home + 0.1)
        )
        self._cutting = MotionPlanner(
            MotionPlannerCfg.create(robot=cfg, collision_cache=cache, max_goalset=GOALS)
        )
        self._cutting.warmup(enable_graph=True, num_warmup_iterations=2)
        self.dt = self._cutting.trajopt_solver.config.interpolation_dt
        """Seconds between two points of a planned trajectory."""
        clips = np.array(cfg["kinematics"]["cspace"]["position_limit_clip"]) + INSIDE
        self.chain = tuple(
            dataclasses.replace(joint, limits=(joint.limits[0] + clip, joint.limits[1] - clip))
            for joint, clip in zip(machine.chain[3:], clips, strict=True)
        )
        """The arm's joints from the platform out, held inside the range
        cuRobo plans in: a straight line solved within it ends where cuRobo
        can plan on from."""
        self._tool = np.linalg.inv(machine.tool) @ machine.shear_frame
        """The shear's frame, which cuRobo poses, in the mouth's."""

        model = NewtonManager.get_model()
        arm = {*harvester.arm_links(), BLADE, FINGER}
        labels = model.body_label
        body = model.shape_body.numpy()
        kind = model.shape_type.numpy()
        colliding = (model.shape_flags.numpy() & int(ShapeFlags.COLLIDE_SHAPES)) != 0
        shapes = []
        self._offset = {}
        """A mesh's box, as its middle in the shape's frame, and its size."""
        for shape in np.flatnonzero(colliding):
            owner = int(body[shape])
            if (
                owner >= 0
                and labels[owner].startswith(robot)
                and labels[owner].rsplit("/", 1)[-1] in arm
            ):
                continue
            if kind[shape] in (GeoType.MESH, GeoType.CONVEX_MESH):
                points = (
                    np.asarray(model.shape_source[shape].vertices)
                    * model.shape_scale.numpy()[shape]
                )
                low, high = points.min(axis=0), points.max(axis=0)
                self._offset[int(shape)] = ((low + high) / 2, high - low)
            elif kind[shape] not in (
                GeoType.BOX,
                GeoType.SPHERE,
                GeoType.CAPSULE,
                GeoType.CYLINDER,
            ):
                continue
            shapes.append(int(shape))
        self._shapes = np.array(shapes)
        self._body = body[self._shapes]
        self._kind = kind[self._shapes]

    def cut(
        self, q: np.ndarray, platform: np.ndarray, poses: list[np.ndarray], standoff: float, skip
    ) -> Cut | None:
        """A cut from arm joint positions `q` to one of the mouth's `poses`, in
        the world, the platform at `platform`, the bodies in `skip` left out of
        the world; or None if none is reachable. Its line in starts `standoff`
        back from the pose along `Harvester.way_in`.

        cuRobo plans to the set as a whole first. Planning to a set it seeds
        no paths from its graph, so it can miss a pose it reaches alone, and
        each pose is tried alone after."""
        planner = self._cutting
        self._world(planner, platform, skip)
        # The stand-off, in the mouth's frame at the cut. cuRobo backs off from
        # the pose it is given along the shear's +Z alone, so it is given the
        # cut pose moved the rest of the way.
        back = -standoff * self._machine.way_in
        moved = transform((0.0, back[1], 0.0))
        for chosen in (poses, *([pose] for pose in poses)):
            goals = np.array(
                [np.linalg.inv(platform) @ pose @ moved @ self._tool for pose in chosen]
            )
            position = torch.tensor(goals[:, :3, 3], dtype=torch.float32, device="cuda")
            quaternion = Rotation.from_matrix(goals[:, :3, :3]).as_quat(scalar_first=True)
            quaternion = torch.tensor(quaternion, dtype=torch.float32, device="cuda")
            result = planner.plan_grasp(
                GoalToolPose(
                    tool_frames=planner.tool_frames,
                    position=position.view(1, 1, 1, len(chosen), 3),
                    quaternion=quaternion.view(1, 1, 1, len(chosen), 4),
                ),
                self._state(planner, q),
                grasp_approach_offset=back[2],
                plan_approach_to_grasp=False,
                plan_grasp_to_lift=False,
                disable_collision_links=[HEAD, BLADE],
            )
            if not result.success.any():
                continue
            goal = chosen[int(result.goalset_index.view(-1)[0])]
            pose = np.linalg.inv(platform) @ goal
            approach = _trajectory(
                result.approach_interpolated_trajectory, result.approach_interpolated_last_tstep
            )
            inward = self._line(approach[0][-1], pose @ transform(back), pose)
            if inward is not None:
                return Cut(approach, inward, (inward[0][::-1], -inward[1][::-1]))
        return None

    def line(
        self, q: np.ndarray, platform: np.ndarray, start: np.ndarray, end: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray] | None:
        """The arm's joint positions and velocities every `dt` taking the mouth
        from world pose `start` to `end`, turned as `start` is, on a straight
        line at `SPEED`, from arm joint positions `q` with the platform at
        `platform`; or None if the arm cannot follow it."""
        inverse = np.linalg.inv(platform)
        return self._line(q, inverse @ start, inverse @ end)

    def _line(
        self, q: np.ndarray, start: np.ndarray, end: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray] | None:
        """`line`, `start` and `end` in the platform's frame.

        cuRobo's own straight line is a cost it weighs against reaching the
        end, and it reaches the end round another branch of the arm as
        readily. Here every point is solved from the one before, from `q`,
        so the line stays on the branch the arm is on."""
        chain, tool = self.chain, self._machine.tool
        length = np.linalg.norm(end[:3, 3] - start[:3, 3])
        positions = []
        for share in np.linspace(0.0, 1.0, math.ceil(length / (SPEED * self.dt)) + 1):
            point = start.copy()
            point[:3, 3] += share * (end[:3, 3] - start[:3, 3])
            q, distance, angle = solve(chain, tool, point, q, iterations=50)
            if distance > LINE_TOLERANCE[0] or angle > LINE_TOLERANCE[1]:
                return None
            positions.append(q)
        positions = np.array(positions)
        return positions, np.gradient(positions, self.dt, axis=0)

    def fold(
        self, q: np.ndarray, platform: np.ndarray, skip
    ) -> tuple[np.ndarray, np.ndarray] | None:
        """A move from arm joint positions `q` to `ARM_HOME`, the platform at
        `platform`, the bodies in `skip` left out of the world; or None if
        there is none."""
        planner = self._folding
        self._world(planner, platform, skip)
        home = self._state(planner, harvester.ARM_HOME)
        result = planner.plan_cspace(home, self._state(planner, q))
        if result is None or not result.success.any():
            return None
        return _trajectory(result.interpolated_trajectory, result.interpolated_last_tstep)

    def _world(self, planner: MotionPlanner, platform: np.ndarray, skip) -> None:
        """Hand `planner` the colliders round the platform, at `platform` in
        the world, as boxes in its frame."""
        model = NewtonManager.get_model()
        bodies = NewtonManager.get_state_0().body_q.numpy()
        placed = model.shape_transform.numpy()[self._shapes]
        scale = model.shape_scale.numpy()[self._shapes]
        poses = np.tile(np.eye(4), (len(self._shapes), 1, 1))
        poses[:, :3, :3] = Rotation.from_quat(placed[:, 3:]).as_matrix()
        poses[:, :3, 3] = placed[:, :3]
        moving = self._body >= 0
        frames = np.tile(np.eye(4), (int(moving.sum()), 1, 1))
        frames[:, :3, :3] = Rotation.from_quat(bodies[self._body[moving], 3:]).as_matrix()
        frames[:, :3, 3] = bodies[self._body[moving], :3]
        poses[moving] = frames @ poses[moving]
        poses = np.linalg.inv(platform) @ poses

        # The box round each: a box's own, and a round shape's along its z.
        radius, half = scale[:, 0], scale[:, 1]
        size = np.where(
            (self._kind == GeoType.BOX)[:, None],
            2 * scale,
            np.stack([2 * radius, 2 * radius, 2 * half], axis=1),
        )
        sphere, capsule = self._kind == GeoType.SPHERE, self._kind == GeoType.CAPSULE
        size[sphere] = 2 * radius[sphere, None]
        size[capsule, 2] += 2 * radius[capsule]
        for i, shape in enumerate(self._shapes):
            if shape in self._offset:
                middle, size[i] = self._offset[shape]
                poses[i] = poses[i] @ transform(middle)

        reach = np.linalg.norm(poses[:, :3, 3], axis=1) - np.linalg.norm(size, axis=1) / 2
        kept = [i for i in np.argsort(reach) if reach[i] < REACH and self._body[i] not in skip]
        kept = kept[:OBSTACLES]
        quaternions = Rotation.from_matrix(poses[kept, :3, :3]).as_quat(scalar_first=True)
        boxes = [
            Cuboid(
                name=f"shape_{self._shapes[i]}",
                dims=size[i].tolist(),
                pose=[*poses[i, :3, 3].tolist(), *quaternion.tolist()],
            )
            for i, quaternion in zip(kept, quaternions, strict=True)
        ]
        planner.update_world(Scene(cuboid=boxes))

    @staticmethod
    def _state(planner: MotionPlanner, q) -> JointState:
        position = torch.tensor([list(q)], dtype=torch.float32, device="cuda")
        return JointState.from_position(position, joint_names=planner.joint_names)


def _trajectory(plan: JointState, last: torch.Tensor) -> tuple[np.ndarray, np.ndarray]:
    """A planned stretch's joint positions and velocities, trimmed to its end."""
    end = int(last.view(-1)[0]) + 1
    dof = plan.position.shape[-1]
    return (
        plan.position.view(-1, dof)[:end].cpu().numpy(),
        plan.velocity.view(-1, dof)[:end].cpu().numpy(),
    )

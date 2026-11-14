"""The weeder's body pose over planted feet, held and moved in Newton: a
controller standing in for Spot's own, the physics it runs on, and the
grounds of the paper's indoor tests.

The paper sends Spot only the pose it wants, and Spot's own controller finds
its way there and holds it (§3.4). `Controller` does that over planted feet:
it moves the body along a smooth path in pose space, solving
`spot_weeder.stance` for the legs at every step, and an integrator on the
measured body pose takes out the sag of the leg drives under the tool's
weight. `SpotLike` gives the errors of Spot's own controller, to add to the
poses sent, for runs that should miss the way Spot misses.

A pose here is (x, y, z, roll, pitch, yaw) in the ground frame of
`spot_weeder`: the body's shift from where it stands at rest, and its turn.

    python pose.py      # checks that the controller holds a pose, headless
"""

import math

import isaaclab.sim as sim_utils
import numpy as np
import spot_weeder
import torch
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.utils.math import (
    euler_xyz_from_quat,
    quat_apply,
    quat_apply_inverse,
    quat_from_euler_xyz,
)
from isaaclab_newton.physics import (
    MJWarpSolverCfg,
    NewtonCfg,
    NewtonCollisionPipelineCfg,
    NewtonShapeCfg,
)

SIM_DT = 0.002
"""Spot's physics step, as Isaac Lab trains it."""

MARGIN = 0.002
"""Every shape's contact margin, and the gap beyond it where contacts are
found early. Margins add up, so two shapes touch from 4 mm apart: the feet
stand a few millimetres into that. The ground of `design_scene` lets the bit
in; the demo's touches it from 4 mm away, and brakes the spindle."""

PHYSICS = NewtonCfg(
    solver_cfg=MJWarpSolverCfg(
        njmax=130,
        nconmax=40,
        cone="pyramidal",
        impratio=1,
        integrator="implicitfast",
        use_mujoco_contacts=False,
    ),
    collision_cfg=NewtonCollisionPipelineCfg(max_triangle_pairs=2_500_000),
    num_substeps=2,
    default_shape_cfg=NewtonShapeCfg(margin=MARGIN, gap=MARGIN),
)
"""Isaac Lab's Newton settings for Spot, from its Spot task, with `MARGIN`
for its 1 cm. No collider in the robot or the ground authors a margin of
its own, so this one is every shape's."""

LEGS = ImplicitActuatorCfg(joint_names_expr=[".*_h[xy]", ".*_kn"], stiffness=800.0, damping=20.0)
"""Spot's legs on stiff drives, in place of the ones Isaac Lab trains Spot's
policies with, which fold under the tool's weight. They keep the torque
limits Spot's USD authors, 45 N m at the hips and 115 N m at the knees."""

MOVE = 4.8
"""How long a move takes, in seconds: the paper's Table 5 gives 4.795 s of
movement per weed."""

GAIN = 4.0
"""How fast the integrator takes out a pose error, per second."""

WINDUP = np.array([0.06, 0.06, 0.06, 0.1, 0.1, 0.1])
"""The most the integrator adds to a pose: 6 cm and 0.1 rad. With the body
shifted 28 cm back, the legs give 5 cm under the tool."""

BALANCE = 0.06
"""How far inside the outline of its feet the robot's centre of mass has
to stay in a pose that is safe to send. The controller holds poses up to
the edge of the feet; this leaves room for `SpotLike`'s errors."""

SOFT = 0.04
"""The soft ground's contact time constant, in seconds, twice Newton's
default. MuJoCo's contacts sink in proportion to its square: Spot's front
feet stand 27 mm into this ground and its hind feet 14 mm, like the mattress
of the paper's main indoor test, where on the flat ground they stand 4 mm
and 0.5 mm in."""

RAISED = {"fl": 0.05, "hr": 0.10}
"""The raised ground's blocks under the feet, in metres: the paper's Table 3
case 4, the front left foot 5 cm up and the hind right 10 cm."""

GROUNDS = ("flat", "soft", "raised")
"""The grounds of the paper's indoor tests (its Fig. 7)."""


def soft_ground(prim_path: str, cfg: sim_utils.GroundPlaneCfg, *args, **kwargs):
    """Isaac Lab's ground plane, with the contact of `SOFT`."""
    from isaaclab.sim.spawners.from_files import spawn_ground_plane
    from pxr import Gf, Sdf

    prim = spawn_ground_plane(prim_path, cfg, *args, **kwargs)
    # Authored before Isaac Lab hands the stage to Newton, which reads them
    # when the scene is built. MuJoCo averages two geoms' contact parameters
    # weighted by their `solmix`, so the plane's outweighs the feet's; a
    # higher priority would also give the contact the plane's friction, and
    # with pyramidal friction cones that changes how far the feet sink too.
    plane = prim.GetStage().GetPrimAtPath(f"{prim_path}/GroundPlane/CollisionPlane")
    plane.CreateAttribute("mjc:solref", Sdf.ValueTypeNames.Float2).Set(Gf.Vec2f(SOFT, 1.0))
    plane.CreateAttribute("mjc:solmix", Sdf.ValueTypeNames.Float).Set(1e6)
    return prim


def let_the_bit_in(_event) -> None:
    """Keeps the ground plane from touching the bit and its shaft in Newton's
    model, so the bit goes into the ground as it does into soil when it mills.
    A filtered pair authored in USD is dropped: Isaac Lab builds the robot
    apart from the rest of the stage."""
    from isaaclab_newton.physics import NewtonBuilderCfg

    sim = sim_utils.SimulationContext.instance()
    builder = sim.get_or_create_backend(NewtonBuilderCfg(physics_cfg=sim.cfg.physics))
    labels = builder.shape_label
    ground = [i for i, label in enumerate(labels) if label.startswith("/World/ground/")]
    for i, label in enumerate(labels):
        if f"/Robot/{spot_weeder.BIT}/" in label:
            for plane in ground:
                builder.add_shape_collision_filter_pair(i, plane)


def design_scene(ground: str = "flat", num_envs: int = 1, spacing: float = 2.0):
    """One of `GROUNDS`, with a standing robot in each of `num_envs`
    environments, its legs on `LEGS`. The ground lets the bit in."""
    # Imported here: they bring `pxr`, and Kit's own has to be the first one.
    from isaaclab.assets import AssetBaseCfg
    from isaaclab.physics import PhysicsEvent
    from isaaclab.scene import InteractiveSceneCfg
    from isaaclab.utils import instantiate
    from isaaclab_newton.physics import NewtonManager

    cfg = InteractiveSceneCfg(num_envs=num_envs, env_spacing=spacing)
    plane = sim_utils.GroundPlaneCfg(color=(0.3, 0.2, 0.12))
    if ground == "soft":
        plane = plane.replace(func=soft_ground)
    cfg.ground = AssetBaseCfg(prim_path="/World/ground", spawn=plane)
    raised = RAISED if ground == "raised" else {}
    for leg, height in raised.items():
        x, y = spot_weeder.HIPS[leg]
        block = sim_utils.CuboidCfg(
            size=(0.2, 0.2, height), collision_props=sim_utils.UsdPhysicsCollisionCfg()
        )
        at = (x, y + math.copysign(spot_weeder.HIP, y), height / 2)
        setattr(
            cfg,
            f"block_{leg}",
            AssetBaseCfg(
                prim_path=f"{{ENV_REGEX_NS}}/block_{leg}",
                spawn=block,
                init_state=AssetBaseCfg.InitialStateCfg(pos=at),
            ),
        )
    robot = spot_weeder.robot_cfg()
    cfg.robot = robot.replace(
        prim_path="{ENV_REGEX_NS}/Robot",
        actuators={"legs": LEGS, spot_weeder.SPINDLE: robot.actuators[spot_weeder.SPINDLE]},
        init_state=robot.init_state.replace(joint_pos=spot_weeder.stance(raised=raised)),
    )
    # Newton's model is built when the simulation resets, after this scene.
    NewtonManager.register_callback(let_the_bit_in, PhysicsEvent.MODEL_INIT, name="let_the_bit_in")
    return instantiate(cfg)


class Controller:
    """Holds the body of each robot in `robot` in a pose over its planted
    feet, and moves it to the next pose along a smooth path, as Spot's own
    controller does with the paper's body pose command. `origins` are the
    environments' origins, and `raised` the feet's blocks, as
    `spot_weeder.stance` takes them.

    The legs are solved for the feet where they were planted, and as deep as
    they sink into the ground. Legs that reach short of a sunk foot push the
    feet against each other, and MuJoCo's friction gives way under a lasting
    push, so they slide; legs solved for wherever the feet are would let
    them splay. `replant` takes the feet where they stand for planted, as
    the robot stands still."""

    def __init__(self, robot, origins: torch.Tensor, raised: dict[str, float] | None = None):
        self.robot, self.origins, self.raised = robot, origins, raised or {}
        names = list(spot_weeder.stance())
        self.legs, _ = robot.find_joints(names, preserve_order=True)
        self.low, self.high = np.array([spot_weeder.LIMITS[name[-2:]] for name in names]).T
        self.feet, _ = robot.find_bodies(
            [f"{leg}_foot" for leg in spot_weeder.HIPS], preserve_order=True
        )
        n = robot.num_instances
        self.start = self.goal = self.integral = np.zeros((n, 6))
        self.time = self.seconds = 0.0
        self.targets = robot.data.default_joint_pos.torch[:, self.legs].cpu().numpy().astype(float)
        self.slid = {}

    def replant(self) -> None:
        """Takes the feet where they stand now for planted. On the soft ground
        each move slides them a little, and legs solved for where they stood
        before push them on."""
        feet = self.robot.data.body_pos_w.torch[:, self.feet, :2] - self.origins[:, None, :2]
        feet = feet.cpu().numpy()
        self.slid = {
            leg: (feet[:, i, 0] - x, feet[:, i, 1] - y - math.copysign(spot_weeder.HIP, y))
            for i, (leg, (x, y)) in enumerate(spot_weeder.HIPS.items())
        }

    def pose(self) -> np.ndarray:
        """Each body's pose as it stands, (n, 6)."""
        data = self.robot.data
        position = (data.root_pos_w.torch - self.origins).cpu().numpy()
        position[:, 2] -= spot_weeder.Z_NOM
        turn = torch.stack(euler_xyz_from_quat(data.root_quat_w.torch), dim=-1).cpu().numpy()
        return np.concatenate([position, turn], axis=1)

    def balanced(self, pose, margin: float = BALANCE) -> np.ndarray:
        """Whether each robot can hold `pose`, (n, 6) or one pose for all:
        whether its centre of mass, carried along with its body from where it
        is now, stays `margin` inside the outline of its feet where they
        stand now."""
        data = self.robot.data
        mass = data.body_mass.torch[..., None]
        com = (mass * data.body_com_pos_w.torch).sum(dim=1) / mass.sum(dim=1)
        center = quat_apply_inverse(data.root_quat_w.torch, com - data.root_pos_w.torch).cpu()
        pose = torch.tensor(np.broadcast_to(pose, (len(center), 6)), dtype=torch.float32)
        shifted = pose[:, :3] + quat_apply(quat_from_euler_xyz(*pose[:, 3:].T), center)
        # The feet clockwise seen from above: front left, front right, hind
        # right, hind left. The inside is to the right of each edge.
        ring = [self.feet[i] for i in (0, 1, 3, 2)]
        corners = (data.body_pos_w.torch[:, ring, :2] - self.origins[:, None, :2]).cpu()
        edges = corners.roll(-1, dims=1) - corners
        to = shifted[:, None, :2] - corners
        left = (edges[..., 0] * to[..., 1] - edges[..., 1] * to[..., 0]) / edges.norm(dim=-1)
        return (-left >= margin).all(dim=1).numpy()

    def reference(self) -> np.ndarray:
        """Where the path has each body now: a minimum-jerk blend from the
        pose a move started at to the one it was sent to."""
        s = min(self.time / self.seconds, 1.0) if self.seconds else 1.0
        s = s**3 * (10 - 15 * s + 6 * s**2)
        return self.start + s * (self.goal - self.start)

    def move(self, pose, seconds: float = MOVE) -> None:
        """Sends each body to `pose`, (n, 6) or one pose for all, along a
        path that takes `seconds`."""
        self.start = self.reference()
        self.goal = np.broadcast_to(pose, self.start.shape).copy()
        self.time, self.seconds = 0.0, seconds

    def step(self, dt: float = SIM_DT) -> None:
        """Advances the path `dt` seconds, and sends the legs the joint angles
        of the pose there, corrected by the integrated error."""
        self.time += dt
        reference = self.reference()
        self.integral = np.clip(
            self.integral + GAIN * dt * (reference - self.pose()), -WINDUP, WINDUP
        )
        x, y, z, roll, pitch, yaw = (reference + self.integral).T
        height = self.robot.data.body_pos_w.torch[:, self.feet, 2] - self.origins[:, None, 2]
        sunk = (height.cpu().numpy() - spot_weeder.FOOT).T
        raised = {
            leg: np.minimum(h, self.raised.get(leg, 0.0)) for leg, h in zip(spot_weeder.HIPS, sunk)
        }
        with np.errstate(invalid="ignore"):
            joints = spot_weeder.stance(x, pitch, yaw, raised, y=y, z=z, roll=roll, slid=self.slid)
        joints = np.stack(list(joints.values()), axis=-1)
        # A pose out of reach, or past a joint's range, keeps the last angles
        # that reached it.
        with np.errstate(invalid="ignore"):
            reached = ((self.low <= joints) & (joints <= self.high)).all(axis=-1)
        self.targets[reached] = joints[reached]
        target = torch.tensor(self.targets, dtype=torch.float32, device=self.origins.device)
        self.robot.actuators.target_command.set_position_index(value=target, joint_ids=self.legs)


def run(sim, scene, controller: Controller, seconds: float, render: bool = False) -> None:
    """Steps the simulation `seconds`, the controller driving the legs, or
    until its viewer is closed."""
    for _ in range(round(seconds / SIM_DT)):
        if not sim.is_running():
            return
        controller.step()
        scene.write_data_to_sim()
        sim.step(render=render)
        scene.update(SIM_DT)


def launch(ground: str = "flat", num_envs: int = 1):
    """The simulation and the scene of `design_scene`, standing still for a
    second, and a controller for it."""
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=SIM_DT, physics=PHYSICS))
    scene = design_scene(ground, num_envs)
    robot = scene["robot"]
    sim.reset()
    robot.write_joint_position_to_sim_index(position=robot.data.default_joint_pos.torch.clone())
    controller = Controller(robot, scene.env_origins, RAISED if ground == "raised" else None)
    run(sim, scene, controller, 1.0)
    return sim, scene, controller


class SpotLike:
    """The errors of Spot's own controller, as the paper's Table 1 gives them,
    to add to the poses the controller is sent: `pose + spot_like(pose)`.

    They are a smooth random function of where the pose puts the bit over
    the ground, scaled so that over the paper's E they have Table 1's mean
    and standard deviation. A move and its small correction to a nearby
    point then come out with nearly the same error, as the paper found
    beside its Table 2, which is why one re-detection fixes most misses.
    `length` is how far apart, at the bit, two errors are mostly unrelated:
    40 cm by default, at which `weeding`'s one correction, from 3 cm behind
    the weed, mills out 85 to 89% of the weeds it aims at, where the paper's
    Table 4 has 86%."""

    MEAN = np.array([1.11e-3, 4.79e-3, 0.33e-3, 0.84e-3, 3.34e-3, -1.78e-3])
    STD = np.array([6.33e-3, 27.88e-3, 2.04e-3, 4.23e-3, 8.14e-3, 2.23e-3])
    """Table 1, in metres and radians: (x, y, z, roll, pitch, yaw)."""

    def __init__(self, seed: int = 0, length: float = 0.4, waves: int = 64):
        # A sum of plane waves of random direction and phase, with wave
        # numbers spread as for a Gaussian process of that length scale.
        rng = np.random.default_rng(seed)
        self.k = rng.normal(0.0, 1.0 / length, (6, waves, 2))
        self.phase = rng.uniform(0.0, 2 * np.pi, (6, waves))
        (x0, y0), (x1, y1) = spot_weeder.E.min(axis=0), spot_weeder.E.max(axis=0)
        x, y = np.meshgrid(np.linspace(x0, x1, 100), np.linspace(y0, y1, 200))
        inside = spot_weeder.inside(spot_weeder.E, x, y)
        field = self._field(np.stack([x[inside], y[inside]], axis=-1))
        self.center, self.scale = field.mean(axis=0), field.std(axis=0)

    def _field(self, xy: np.ndarray) -> np.ndarray:
        return np.cos(np.einsum("...d,kwd->...kw", xy, self.k) + self.phase).sum(axis=-1)

    def __call__(self, pose: np.ndarray) -> np.ndarray:
        x, y, z, roll, pitch, yaw = np.moveaxis(np.asarray(pose), -1, 0)
        bit = np.moveaxis(spot_weeder.fk(x, pitch, yaw, y=y, z=z, roll=roll)[:2], 0, -1)
        return self.MEAN + self.STD * (self._field(bit) - self.center) / self.scale


def main():
    """Stands the robot on flat ground, pitches it to put the bit 4 cm up and
    holds it there, and checks that the body holds the pose."""
    from isaaclab.app import launch_simulation

    sim_cfg = sim_utils.SimulationCfg(physics=PHYSICS)
    with launch_simulation(sim_cfg, {"headless": True, "visualizer": []}):
        sim, scene, controller = launch()
        sent = np.array([0, 0, 0, 0, spot_weeder.pitch_to(0.04), 0])
        controller.move(sent)
        run(sim, scene, controller, MOVE + 1.0)
        error = controller.pose()[0] - sent
        print(
            f"[INFO]: sent pitch {math.degrees(sent[4]):.2f} degrees; error (mm, mrad): {(error * 1000).round(2)}"
        )
        assert np.abs(error).max() < 1e-3, error


if __name__ == "__main__":
    main()

"""Spot with the weeding tool on bare soil among a few leaves, its spindle at
full speed: it stands, then pitches its body down over its planted feet until
the bit is 4 cm up and just behind a leaf, where the paper stops to find the
weed again before it mills, with the tool out of the camera's way. Filmed
there from outside and through the robot's own L515.

    python demo.py OUT_DIR              # headless: writes side.png, front.png and l515.png
    python demo.py                      # opens Kit's viewer instead
    python demo.py OUT_DIR --usd FILE   # spawns FILE, a robot `spot_weeder.py` wrote

The paper poses the body with Spot's own controller; here `pose.Controller`
does, over planted feet.
"""

import argparse
import math
import pathlib

import isaaclab.sim as sim_utils
import numpy as np
import pose
import spot_weeder
import torch
from isaaclab.app import launch_simulation

STAND, HOLD = 1.0, 1.0
"""Seconds standing, and holding still after pitching down, before the
pictures."""

CLEARANCE = 0.04
"""How high over the ground the bit stops."""

ROBOT = "/World/envs/env_0/Robot"

SOIL = (0.3, 0.2, 0.12)

LEAVES = [
    (0.77, 0.0, 30),
    (0.69, 0.1, -50),
    (0.82, -0.08, 80),
    (0.66, -0.12, 10),
    (0.8, 0.12, 120),
]
"""Artificial weeds, like the paper's indoor ones: x and y on the ground, and
turn in degrees. The bit stops 5 cm behind the first, 0.72 m ahead of the
body."""

SUN = (-0.103, 0.326, 0.283, 0.896)
"""The sun's turn (x, y, z, w): 40 degrees from overhead, from ahead and to
the left."""

VIEWS = {
    "side": ((1.7, 1.3, 0.5), (0.25, 0.0, 0.3)),
    "front": ((1.6, -0.75, 1.15), (0.5, 0.0, 0.2)),
}
"""Where each outside camera stands and what it looks at."""


def design_scene(usd: pathlib.Path | None):
    """The ground, the light, the leaves and the robot."""
    # Imported here: they bring `pxr`, and Kit's own has to be the first one.
    from isaaclab.assets import AssetBaseCfg
    from isaaclab.scene import InteractiveSceneCfg
    from isaaclab.utils import instantiate

    scene = InteractiveSceneCfg(num_envs=1, env_spacing=0.0)
    scene.ground = AssetBaseCfg(
        prim_path="/World/ground", spawn=sim_utils.GroundPlaneCfg(color=SOIL)
    )
    scene.sky = AssetBaseCfg(
        prim_path="/World/sky",
        spawn=sim_utils.DomeLightCfg(intensity=800.0, color=(0.85, 0.9, 1.0)),
    )
    scene.sun = AssetBaseCfg(
        prim_path="/World/sun",
        spawn=sim_utils.DistantLightCfg(intensity=2500.0),
        init_state=AssetBaseCfg.InitialStateCfg(rot=SUN),
    )
    leaf = sim_utils.CuboidCfg(
        size=(0.04, 0.018, 0.002),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.1, 0.45, 0.05)),
    )
    for i, (x, y, turn) in enumerate(LEAVES):
        half = math.radians(turn) / 2
        rot = (0.0, 0.0, math.sin(half), math.cos(half))
        setattr(
            scene,
            f"leaf_{i}",
            AssetBaseCfg(
                prim_path=f"/World/leaves/leaf_{i}",
                spawn=leaf,
                init_state=AssetBaseCfg.InitialStateCfg(pos=(x, y, 0.001), rot=rot),
            ),
        )
    robot = spot_weeder.robot_cfg(usd)
    actuators = {"legs": pose.LEGS, spot_weeder.SPINDLE: robot.actuators[spot_weeder.SPINDLE]}
    scene.robot = robot.replace(prim_path="{ENV_REGEX_NS}/Robot", actuators=actuators)
    return instantiate(scene)


def cameras() -> dict:
    """A camera for each of `VIEWS`, and the L515 on the robot."""
    from isaaclab.sensors import Camera, CameraCfg
    from isaaclab_physx.renderers import IsaacRtxRendererCfg

    # The outside cameras are outside the robot's environment, which they see
    # only with the render partitions off.
    renderer = IsaacRtxRendererCfg(enable_scene_partitioning=False)
    out = {
        name: Camera(
            CameraCfg(
                prim_path=f"/World/cameras/{name}",
                width=1280,
                height=960,
                data_types=["rgb"],
                spawn=sim_utils.PinholeCameraCfg(focal_length=18.0, clipping_range=(0.01, 100.0)),
                renderer_cfg=renderer,
            )
        )
        for name in VIEWS
    }
    out["l515"] = Camera(
        CameraCfg(
            prim_path=f"{ROBOT}/{spot_weeder.TOOL}/camera",
            width=640,
            height=480,
            data_types=["rgb"],
            spawn=None,
            renderer_cfg=renderer,
        )
    )
    return out


def film(sim: sim_utils.SimulationContext, cameras: dict, out: pathlib.Path) -> None:
    """Saves what each camera sees of the scene as it stands into `out`."""
    from PIL import Image
    from pxr import Gf, UsdGeom

    stage = sim_utils.get_current_stage()
    for name, (eye, target) in VIEWS.items():
        # On the prim: under Newton, `Camera.set_world_poses` moves a Newton
        # site the renderer never sees. USD's look-at is the view matrix, world
        # to camera, so the prim takes its inverse.
        view = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(0, 0, 1))
        xform = UsdGeom.Xformable(stage.GetPrimAtPath(f"/World/cameras/{name}"))
        xform.ClearXformOpOrder()
        xform.AddTransformOp().Set(view.GetInverse())
    # Headless, a frame is drawn by a camera's update, after the render that
    # moves the bodies to where the physics has them; enough of them for the
    # renderer's accumulation to settle.
    for _ in range(32):
        sim.render()
        for camera in cameras.values():
            camera.update(sim.get_physics_dt(), force_recompute=True)
    out.mkdir(parents=True, exist_ok=True)
    for name, camera in cameras.items():
        rgb = camera.data.output["rgb"].torch[0, ..., :3].cpu().numpy()
        if not rgb.any():
            raise RuntimeError(f"the {name} camera rendered nothing")
        Image.fromarray(rgb).save(out / f"{name}.png")
        print(f"[INFO]: saved {out / f'{name}.png'}", flush=True)


def report(robot, doing: str) -> None:
    """Prints how the robot stands: the body's height and pitch, where the
    bit is, how far the shaft leans and how fast it turns."""
    from isaaclab.utils.math import euler_xyz_from_quat, quat_apply

    (bit,), _ = robot.find_bodies([spot_weeder.BIT])
    (spindle,), _ = robot.find_joints([spot_weeder.SPINDLE])
    _, pitch, _ = euler_xyz_from_quat(robot.data.root_quat_w.torch)
    body = robot.data.root_pos_w.torch[0]
    tip = robot.data.body_pos_w.torch[0, bit]
    up = quat_apply(
        robot.data.body_quat_w.torch[:, bit], torch.tensor([[0.0, 0.0, 1.0]], device=tip.device)
    )
    lean = math.degrees(math.acos(up[0, 2].item()))
    rpm = robot.data.joint_vel.torch[0, spindle].item() * 60 / (2 * math.pi)
    print(
        f"[INFO]: {doing}: body {body[2]:.3f} m up, pitched {math.degrees(pitch.item()):.1f} degrees; "
        f"bit at ({tip[0]:.3f}, {tip[1]:.3f}, {tip[2]:.3f}), shaft {lean:.1f} degrees off upright; "
        f"spindle at {rpm:.0f} rpm",
        flush=True,
    )


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("out", nargs="?", type=pathlib.Path, help="where the pictures go")
    parser.add_argument("--usd", type=pathlib.Path, help="a robot to spawn, not the published one")
    args = parser.parse_args()
    sim_cfg = sim_utils.SimulationCfg(dt=pose.SIM_DT, device="cuda:0", physics=pose.PHYSICS)
    if args.out:
        launcher = {"headless": True, "enable_cameras": True, "require_kit": True, "visualizer": []}
    else:
        launcher = {"visualizer": ["kit"]}
    with launch_simulation(sim_cfg, launcher):
        sim = sim_utils.SimulationContext(sim_cfg)
        scene = design_scene(args.usd)
        robot = scene["robot"]
        sensors = cameras() if args.out else {}
        sim.reset()
        # The reset leaves the legs as the USD has them, straight down through
        # the ground; the robot starts in its standing pose instead.
        robot.write_joint_position_to_sim_index(position=robot.data.default_joint_pos.torch.clone())
        spindle, _ = robot.find_joints([spot_weeder.SPINDLE])
        speed = torch.full((1, 1), spot_weeder.SPINDLE_SPEED, device=sim.device)
        robot.actuators.target_command.set_velocity_index(value=speed, joint_ids=spindle)
        controller = pose.Controller(robot, scene.env_origins)
        pose.run(sim, scene, controller, STAND, render=not args.out)
        report(robot, "standing")
        controller.move(np.array([0, 0, 0, 0, spot_weeder.pitch_to(CLEARANCE), 0]))
        pose.run(sim, scene, controller, pose.MOVE + HOLD, render=not args.out)
        report(robot, "pitched down")
        if args.out:
            film(sim, sensors, args.out)
        while sim.is_running() and not args.out:
            pose.run(sim, scene, controller, pose.SIM_DT, render=True)


if __name__ == "__main__":
    main()

"""Stills of a generated scene for the docs, rendered by Isaac Sim's RTX
renderer from a shot file.

A shot is a JSON object:

- `size`: the picture's width and height, in pixels.
- `set`: parameters changed from the script's cfg, named `fragment.field` as
  in a storyboard of the viewer. Optional.
- `camera`: the camera as a storyboard of the viewer places it, `Pose` in
  `crates/misina-lab/src/tour.rs`: `focus`, the point looked at, x and y in
  meters and then a height above the ground there; `yaw`, degrees around the
  vertical, 0 putting the camera on the focus's -y side and 90 on its +x
  side; `pitch`, degrees above the horizon; `radius`, meters from the focus;
  and `fov`, the vertical field of view in degrees, 45 if left out.

A script may add fields of its own, such as where a robot starts, and a
script of a demo that moves takes the moment of the picture as two:

- `at`: what the demo is doing, as `Moments` tells it, e.g. `{"box": 2,
  "stage": "carry"}`; the picture is taken the first time it holds every
  item. Optional: from the start.
- `warmup`: seconds of simulated time on from there.

It takes `SHOT.json [OUT.png]` and makes these calls around its own scene:

    args = arguments("Renders a still of ...")
    shot = load(args.shot, {"its_own_field", "at", "warmup"})
    configure(cfg, shot.get("set", {}))
    with launch_simulation(sim_cfg, launcher(args.out)):
        ...  # the scene
        camera = sensor(shot) if args.out else None
        sim.reset()
        moments = Moments(doing(steps), dt)  # the demo's steps, told as dicts
        moments.reach(shot)
        finish(sim, shot, ground, camera, args.out, moments)

Without OUT, Kit opens on the moment the picture shows, the simulation stopped
there and the viewport aimed as the shot. P prints the viewport's camera as
the shot's `camera`, and the moment as its `at` and `warmup`; G plays the
demo on from there and pauses it again.
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib
from collections.abc import Iterator
from typing import TYPE_CHECKING

import isaaclab.sim as sim_utils
import numpy as np

if TYPE_CHECKING:
    from isaaclab.sensors import Camera
    from pxr import Usd

    from ..usd import Ground
    from .spawn import GeneratedSceneCfg

FIELDS = {"size", "set", "camera"}
"""What every shot may hold; a script names its own fields on top."""

POSE = {"focus", "yaw", "pitch", "radius", "fov"}

SETTLE = 32
"""Frames rendered of the stopped scene before the picture is read, for the
renderer's accumulation to settle."""

CAMERA = "/World/ShotCamera"

PATIENCE = 3600.0
"""Seconds of simulated time `Moments.reach` steps a demo through before it
gives up on a shot's `at`: longer than any demo's run, for one that stalls
short of its end."""

APERTURE = 20.955
"""The horizontal aperture of Isaac Lab's pinhole camera, in the units of its
focal length."""


def arguments(description: str) -> argparse.Namespace:
    """A shot script's command line: the shot, and the picture to write."""
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("shot", type=pathlib.Path, help="the shot, a JSON file")
    parser.add_argument(
        "out",
        type=pathlib.Path,
        nargs="?",
        help="the PNG to write; without it Kit opens on the shot, to frame it by hand",
    )
    return parser.parse_args()


def load(path: pathlib.Path, fields: frozenset[str] | set[str] = frozenset()) -> dict:
    """The shot at `path`, checked for fields that neither every shot nor the
    script's own `fields` hold."""
    shot = json.loads(path.read_text())
    unknown = (set(shot) - FIELDS - set(fields)) | (set(shot["camera"]) - POSE)
    if unknown:
        raise ValueError(f"{path}: no shot field {sorted(unknown)}")
    shot["camera"].setdefault("fov", 45.0)
    return shot


class Moments:
    """A demo run a physics step at a time, each step yielding what the demo
    is doing then as a dict, such as `{"box": 2, "stage": "carry"}`, which
    a shot's `at` picks a moment by. Every step's dict has the same keys,
    and names a moment once: a stage that comes again, as a job tried
    again, tells which time it is. The steps end where the demo's run
    does, its last job done, so a moment it never comes to fails; one it
    stalls short of fails after `PATIENCE`."""

    def __init__(self, steps: Iterator[dict], dt: float):
        self.steps, self.dt = steps, dt
        self.now = next(steps)
        """What the demo is doing."""
        self.since = 0
        """Physics steps since it started doing that."""

    def step(self) -> bool:
        """Steps the demo on; False once it has ended."""
        now = next(self.steps, None)
        if now is None:
            return False
        self.since = self.since + 1 if now == self.now else 0
        self.now = now
        return True

    def reach(self, shot: dict) -> None:
        """Steps on to the shot's moment: until what the demo is doing holds
        every item of its `at`, then `warmup` seconds on."""
        at, warmup = shot.get("at", {}), shot.get("warmup", 0.0)
        if unknown := set(at) - set(self.now):
            raise ValueError(f"no moment {sorted(unknown)}: the demo tells {sorted(self.now)}")
        waited = 0
        while not at.items() <= self.now.items():
            if waited * self.dt > PATIENCE or not self.step():
                raise ValueError(f"the demo never got to {at}")
            waited += 1
        for _ in range(round(warmup / self.dt)):
            if not self.step():
                raise ValueError(f"the demo ended within {warmup} s of {at}")

    def __str__(self) -> str:
        """The moment as a shot's `at` and `warmup`."""
        return f'"at": {json.dumps(self.now)}, "warmup": {self.since * self.dt:.3f}'


def configure(cfg: GeneratedSceneCfg, changes: dict) -> None:
    """Applies a shot's `set` to `cfg`, before anything reads it."""
    for path, value in changes.items():
        fragment, field = path.split(".")
        part = getattr(cfg, fragment)
        if not hasattr(part, field):
            raise ValueError(f"no parameter {path}")
        setattr(part, field, value)


def launcher(out: pathlib.Path | None) -> dict:
    """What `launch_simulation` starts: headless, with nothing to look at the
    scene but the shot's camera, whose renderer needs Kit's; or Kit's window,
    to frame the shot in."""
    if out:
        return {"headless": True, "enable_cameras": True, "require_kit": True, "visualizer": []}
    return {"visualizer": ["kit"]}


def aim(camera: dict, ground: Ground | None) -> tuple[np.ndarray, np.ndarray]:
    """Where a shot's `camera` stands and what it looks at, in scene
    coordinates. With no `ground`, the ground is at z = 0."""
    x, y, up = camera["focus"]
    target = np.array([x, y, _height(ground, x, y) + up])
    yaw, pitch = math.radians(camera["yaw"]), math.radians(camera["pitch"])
    back = [math.cos(pitch) * math.sin(yaw), -math.cos(pitch) * math.cos(yaw), math.sin(pitch)]
    return target + camera["radius"] * np.array(back), target


def pose(eye: np.ndarray, target: np.ndarray, fov: float, ground: Ground | None) -> str:
    """The inverse of `aim`: a shot's `camera` for one at `eye` looking at
    `target`, as the JSON the viewer's P key prints."""
    back = eye - target
    radius = float(np.linalg.norm(back))
    yaw = math.degrees(math.atan2(back[0], -back[1]))
    pitch = math.degrees(math.asin(back[2] / radius))
    up = target[2] - _height(ground, target[0], target[1])
    return (
        f'"camera": {{"focus": [{target[0]:.2f}, {target[1]:.2f}, {up:.2f}],'
        f' "yaw": {yaw:.1f}, "pitch": {pitch:.1f}, "radius": {radius:.2f}, "fov": {fov:.1f}}}'
    )


def place(prim: Usd.Prim, eye: np.ndarray, target: np.ndarray) -> None:
    """Stands the camera `prim` at `eye`, looking at `target`, z up."""
    from pxr import Gf, UsdGeom

    # On the prim: under Newton, `Camera.set_world_poses` moves a Newton site
    # the renderer never sees. USD's look-at is the view matrix, world to
    # camera, so the prim takes its inverse.
    view = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(0, 0, 1))
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    xform.AddTransformOp().Set(view.GetInverse())


def lens(fov: float, size: list[int], aperture: float = APERTURE) -> float:
    """The focal length that fits `fov` degrees into the height of a picture
    `size` wide and high: a camera's vertical aperture follows the picture's
    shape."""
    width, height = size
    return aperture * height / width / (2 * math.tan(math.radians(fov) / 2))


def sensor(shot: dict) -> Camera:
    """The camera that renders the shot. Made before the simulation's first
    reset, as every sensor is."""
    # Imported here: they bring `pxr`, and Kit's copy has to be the first one.
    from isaaclab.sensors import Camera, CameraCfg
    from isaaclab_physx.renderers import IsaacRtxRendererCfg

    return Camera(
        CameraCfg(
            prim_path=CAMERA,
            update_period=0.0,
            width=shot["size"][0],
            height=shot["size"][1],
            data_types=["rgb"],
            spawn=sim_utils.PinholeCameraCfg(
                focal_length=lens(shot["camera"]["fov"], shot["size"]),
                horizontal_aperture=APERTURE,
                clipping_range=(0.02, 1000.0),
            ),
            # A robot is in its environment's render partition, which a
            # camera outside the environments sees only with partitions off.
            renderer_cfg=IsaacRtxRendererCfg(enable_scene_partitioning=False),
        )
    )


def finish(
    sim: sim_utils.SimulationContext,
    shot: dict,
    ground: Ground | None,
    camera: Camera | None,
    out: pathlib.Path | None,
    moments: Moments | None = None,
) -> None:
    """Saves what the shot's `camera` sees of the scene as it stands to `out`;
    with no camera, opens Kit's viewport on it instead, to frame by hand and,
    given the demo's `moments`, to play it on."""
    if camera is None:
        _frame(sim, shot, ground, moments)
        return
    place(sim_utils.get_current_stage().GetPrimAtPath(CAMERA), *aim(shot["camera"], ground))
    _take(sim, camera, out)
    print(f"[INFO]: saved {out}", flush=True)


def _height(ground: Ground | None, x: float, y: float) -> float:
    return 0.0 if ground is None else ground.height(x, y)


def _take(sim: sim_utils.SimulationContext, camera: Camera, out: pathlib.Path) -> None:
    """Renders the stopped scene until it settles, and saves what `camera`
    sees."""
    from PIL import Image

    # Headless, a frame is drawn by the camera's update, which first moves
    # the bodies' render transforms to where the physics has them, and only
    # once per render generation: a render, then an update, each frame.
    for _ in range(SETTLE):
        sim.render()
        camera.update(sim.get_physics_dt(), force_recompute=True)
    rgb = camera.data.output["rgb"].torch[0, ..., :3].cpu().numpy()
    if not rgb.any():
        raise RuntimeError("the camera rendered nothing")
    Image.fromarray(rgb).save(out)


def _frame(
    sim: sim_utils.SimulationContext, shot: dict, ground: Ground | None, moments: Moments | None
) -> None:
    """Holds the shot's moment in Kit's viewport, aimed and shaped as the shot,
    and prints the viewport's camera, and the moment, each time P is pressed.
    G plays `moments` on and pauses them."""
    import carb.input
    from omni.kit.viewport.utility import get_active_viewport
    from pxr import Gf, Usd, UsdGeom

    viewport = get_active_viewport()
    # Rendered at the shot's size, so the viewport frames what the picture would.
    viewport.resolution = tuple(shot["size"])
    sim.set_camera_view(*(point.tolist() for point in aim(shot["camera"], ground)))
    stage = sim_utils.get_current_stage()
    prim = stage.GetPrimAtPath(viewport.camera_path)
    camera = UsdGeom.Camera(prim)
    aperture = camera.GetHorizontalApertureAttr().Get()
    # Kit authors its viewport camera in the session layer, which outranks
    # the stage's own.
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        camera.GetFocalLengthAttr().Set(lens(shot["camera"]["fov"], shot["size"], aperture))

    playing = False

    def on_input(event) -> bool:
        nonlocal playing
        key = event.event
        if (
            event.deviceType != carb.input.DeviceType.KEYBOARD
            or key.type != carb.input.KeyboardEventType.KEY_PRESS
        ):
            return True
        if key.input.name == "G" and moments:
            playing = not playing
        elif key.input.name == "P":
            to_world = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            # Kit keeps the point the viewport orbits in the camera's own frame.
            orbited = prim.GetAttribute("omni:kit:centerOfInterest").Get() or Gf.Vec3d(0, 0, -10)
            width, height = shot["size"]
            focal = camera.GetFocalLengthAttr().Get()
            fov = math.degrees(2 * math.atan(aperture * height / width / (2 * focal)))
            eye, target = to_world.ExtractTranslation(), to_world.Transform(orbited)
            print(pose(np.array(eye), np.array(target), fov, ground), flush=True)
            if moments:
                print(moments, flush=True)
        return True

    inputs = carb.input.acquire_input_interface()
    listening = inputs.subscribe_to_input_events(on_input)
    plays = "; G plays the demo on and pauses it" if moments else ""
    print(f"[INFO]: frame the shot in the viewport; P prints its camera{plays}", flush=True)
    while sim.is_running():
        # A demo's step renders, as it does with Kit's window open.
        if playing:
            # A G pressed during the step, while it renders, pauses after it.
            stepped = moments.step()
            playing = playing and stepped
        else:
            sim.render()
    inputs.unsubscribe_to_input_events(listening)

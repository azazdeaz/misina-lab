"""Record sequences like BUTom-ST21's in a generated greenhouse: a pipe-rail
trolley drives down a path at 0.2 m/s and the two middle cameras on its mast
film the crop across the path, in RGB and depth, as PATHoBot's did at Campus
Klein-Altendorf. It runs headless.

    uv run capture.py OUT
    uv run capture.py OUT --passes 10 --set scene.seed=3 fruit.ripe_trusses=2

Each pass films one face of a row, from a random place along it, and makes a
sequence per camera. OUT takes BUTom's layout, so what reads the real dataset
reads this too:

    camera_parameters.yaml       each camera's intrinsics, as BUTom's
    rgb/<seq>/<frame_id>.png     720 x 1280, portrait
    depth/<seq>/<frame_id>.png   uint16 millimetres along the optical axis,
                                 0 where nothing is in range
    sequences.json               each sequence's camera, the row and face it
                                 films, the scene's parameters, and each
                                 frame's camera pose

A frame id is the frame's time into its pass, in nanoseconds; the two
cameras of a pass share them. A run numbers its sequences on from those
already in OUT, so runs with different `--set` build one dataset.
"""

import argparse
import json
import pathlib
import random

import isaaclab.sim as sim_utils
import numpy as np
from isaaclab.app import launch_simulation

from greenhouse_lab.isaaclab import (
    GreenhouseCfg,
    GutterCfg,
    LeafCfg,
    RailCfg,
    StemCfg,
    TrussCfg,
    WireCfg,
)
from misina_lab.isaaclab import shots

HERE = pathlib.Path(__file__).parent

GREENHOUSE_CFG = GreenhouseCfg(
    # The trolley's wheels; growers lay 550 mm too.
    rail=RailCfg(gauge=0.55),
    # BUTom's lower camera sees the substrate's top along the bottom of its
    # picture, its upper one the heads of the crop near the top of its.
    gutter=GutterCfg(height=0.95),
    wire=WireCfg(height=3.0),
    # A crop thick with leaves from its head down past the lower camera,
    # where its ripe trusses hang. The generator strips the stem below the
    # kept leaves, and always the two trusses nearest the harvest height.
    stem=StemCfg(harvest_height=1.0, leaves_kept=21, extra_stems=0.3),
    leaf=LeafCfg(length=0.5),
    truss=TrussCfg(fruit_diameter=0.05),
)

RADIATORS = (1.65, 1.95)
"""Heights, in meters, of the two heating pipes along every row's
centreline, between the two faces its stems are trained to: behind the face
the cameras film, from either path."""

ROBOT = "/World/PATHoBot"
CAMERAS = ("cam_1", "cam_2")
"""The cameras that film, prims of `pathobot.usda`: BUTom's two."""

SIZE = (720, 1280)
FPS = 15
SPEED = 0.2
"""The trolley's speed, in meters a second: about 1.1 m of row a sequence."""

FRAMES = 83
"""Frames in a sequence, as in BUTom's."""

MARGIN = 1.0
"""Meters a pass keeps from either end of a row."""

SUBFRAMES = 4
"""Renders of each frame before it is read, for the renderer's temporal
accumulation to catch up with the move."""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Records BUTom-like RGB-D sequences in a generated greenhouse."
    )
    parser.add_argument("out", type=pathlib.Path, help="the dataset's folder, made or added to")
    parser.add_argument(
        "--passes", type=int, default=2, help="passes of the trolley, each a sequence per camera"
    )
    parser.add_argument("--frames", type=int, default=FRAMES, help="frames a sequence")
    parser.add_argument(
        "--set",
        nargs="*",
        default=[],
        metavar="FRAGMENT.FIELD=VALUE",
        help="greenhouse parameters changed from the script's, values as JSON: scene.seed=3",
    )
    args = parser.parse_args()
    args.set = {key: json.loads(value) for key, value in (item.split("=", 1) for item in args.set)}
    return args


def plan(rng: random.Random, layout, frames: int) -> tuple[int, int, np.ndarray]:
    """A pass: the row it films, the face of it, -1 for the row's -y side and
    1 for its +y, and the trolley's x at each frame."""
    row, face = rng.randrange(layout.rows), rng.choice((-1, 1))
    run = SPEED * (frames - 1) / FPS
    start = rng.uniform(MARGIN, layout.row_length - MARGIN - run)
    xs = start + SPEED / FPS * np.arange(frames)
    # The cameras look to the trolley's right and it drives forward, as on
    # PATHoBot, so a row's -y face is filmed driving along -x.
    return row, face, xs if face == 1 else xs[::-1]


def millimetres(depth: np.ndarray) -> np.ndarray:
    """Depth in meters as a 16-bit PNG holds it: millimetres, 0 where nothing
    is in range."""
    mm = np.nan_to_num(depth * 1000.0, nan=0.0, posinf=0.0)
    return np.where(mm < 65535.5, np.round(mm), 0.0).astype(np.uint16)


def row_y(layout, row: int) -> float:
    """A row's gutter centreline, as the generator lays it out."""
    return (row - (layout.rows - 1) / 2) * layout.pitch


def design_scene() -> None:
    # The glasshouse's whitewashed glass: a bright, even sky, which the
    # crop's gaps show as the real pictures do, blown out.
    sky = sim_utils.DomeLightCfg(intensity=2500.0, color=(1.0, 0.98, 0.94))
    sky.func("/World/Light", sky)
    layout = GREENHOUSE_CFG.layout
    pipe = sim_utils.CylinderCfg(
        radius=0.02,
        height=layout.row_length,
        axis="X",
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.72, 0.6, 0.38)),
    )
    for row in range(layout.rows):
        for k, z in enumerate(RADIATORS):
            at = (layout.row_length / 2, row_y(layout, row), z)
            pipe.func(f"/World/Radiators/Row_{row}_{k}", pipe, translation=at)
    # Past the floor's edge a camera sees ground, not the underside of the sky.
    ground = sim_utils.CuboidCfg(
        size=(200.0, 200.0, 0.1),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.3, 0.27, 0.22)),
    )
    ground.func("/World/Ground", ground, translation=(0.0, 0.0, -0.06))
    GREENHOUSE_CFG.func("/World/Greenhouse", GREENHOUSE_CFG)
    robot = sim_utils.UsdFileCfg(usd_path=str(HERE / "pathobot.usda"))
    robot.func(ROBOT, robot)


def sensor(name: str):
    """The Isaac Lab camera reading `pathobot.usda`'s camera `name`, which
    brings its own intrinsics. Made before the simulation's first reset, as
    every sensor is."""
    # Imported here: they bring `pxr`, and Kit's copy has to be the first one.
    from isaaclab.sensors import Camera, CameraCfg
    from isaaclab_physx.renderers import IsaacRtxRendererCfg

    return Camera(
        CameraCfg(
            prim_path=f"{ROBOT}/{name}",
            spawn=None,
            update_period=0.0,
            width=SIZE[0],
            height=SIZE[1],
            data_types=["rgb", "distance_to_image_plane"],
            renderer_cfg=IsaacRtxRendererCfg(enable_scene_partitioning=False),
        )
    )


def record(sim: sim_utils.SimulationContext, cameras: list, args: argparse.Namespace) -> None:
    import yaml
    from PIL import Image
    from pxr import Gf, Usd, UsdGeom

    stage = sim_utils.get_current_stage()
    robot = UsdGeom.Xformable(stage.GetPrimAtPath(ROBOT))
    robot.ClearXformOpOrder()
    pose = robot.AddTransformOp()
    lenses = [UsdGeom.Xformable(stage.GetPrimAtPath(f"{ROBOT}/{name}")) for name in CAMERAS]

    out = args.out
    out.mkdir(parents=True, exist_ok=True)
    intrinsics = {
        name: {
            "image_width": SIZE[0],
            "image_height": SIZE[1],
            "intrinsics": camera.data.intrinsic_matrices.torch[0].cpu().double().tolist(),
        }
        for name, camera in zip(CAMERAS, cameras)
    }
    (out / "camera_parameters.yaml").write_text(yaml.safe_dump(intrinsics))
    index = out / "sequences.json"
    sequences = json.loads(index.read_text()) if index.exists() else {}
    scene = {name: getattr(GREENHOUSE_CFG, name).to_dict() for name in GREENHOUSE_CFG.FRAGMENTS}
    layout = GREENHOUSE_CFG.layout
    # Numbered on from every sequence folder there, those of a pass cut
    # short too, which the index leaves out. Seeded by the scene and that
    # count: a rerun into the same folder films new places.
    taken = len(list((out / "rgb").glob("*")))
    rng = random.Random(f"{GREENHOUSE_CFG.scene.seed}/{taken}")

    for _ in range(args.passes):
        row, face, xs = plan(rng, layout, args.frames)
        path_y = row_y(layout, row) + face * layout.pitch / 2
        # Turned to face the row, since the cameras look to the trolley's right.
        turn = Gf.Rotation(Gf.Vec3d(0, 0, 1), 0.0 if face == 1 else 180.0)
        names = [f"{taken + i:03d}" for i in range(len(CAMERAS))]
        taken += len(names)
        for name in names:
            (out / "rgb" / name).mkdir(parents=True)
            (out / "depth" / name).mkdir(parents=True)
        poses = [[] for _ in names]
        print(
            f"[INFO]: sequences {names}: row {row}, its {'+y' if face == 1 else '-y'} face",
            flush=True,
        )
        for k, x in enumerate(xs):
            pose.Set(
                Gf.Matrix4d().SetRotate(turn).SetTranslateOnly(Gf.Vec3d(float(x), path_y, 0.0))
            )
            # Headless, a frame is drawn by the cameras' update, once per render.
            for _ in range(SUBFRAMES):
                sim.render()
                for camera in cameras:
                    camera.update(0.0, force_recompute=True)
            frame = f"{round(k * 1e9 / FPS):019d}.png"
            for name, camera, lens, track in zip(names, cameras, lenses, poses):
                output = camera.data.output
                rgb = output["rgb"].torch[0, ..., :3].cpu().numpy()
                depth = output["distance_to_image_plane"].torch[0, ..., 0].cpu().numpy()
                Image.fromarray(rgb).save(out / "rgb" / name / frame)
                Image.fromarray(millimetres(depth)).save(out / "depth" / name / frame)
                # USD's matrices are row vectors and its cameras look down -z
                # with y up; OpenCV's look down +z with y down.
                usd = np.array(lens.ComputeLocalToWorldTransform(Usd.TimeCode.Default())).T
                track.append((usd @ np.diag([1.0, -1.0, -1.0, 1.0])).round(6).tolist())
        for name, camera_name, track in zip(names, CAMERAS, poses):
            sequences[name] = {
                "camera": camera_name,
                "row": row,
                "face": face,
                "world_from_camera": track,
                "scene": scene,
            }
        index.write_text(json.dumps(sequences))
    print(f"[INFO]: {len(sequences)} sequences in {out}", flush=True)


def main():
    args = parse_args()
    shots.configure(GREENHOUSE_CFG, args.set)
    sim_cfg = sim_utils.SimulationCfg()
    # Headless: nothing looks at the scene but the cameras, whose renderer
    # needs Kit's, which a camera made after launch cannot ask for.
    launcher = {"headless": True, "enable_cameras": True, "require_kit": True, "visualizer": []}
    with launch_simulation(sim_cfg, launcher):
        sim = sim_utils.SimulationContext(sim_cfg)
        design_scene()
        cameras = [sensor(name) for name in CAMERAS]
        sim.reset()
        record(sim, cameras, args)


if __name__ == "__main__":
    main()

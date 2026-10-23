"""Record a short video of the harvest demo: the harvester drives into a path
and cuts its due trusses, the first few filmed close up, in Isaac Sim's RTX
renderer by a camera this script moves. It runs headless.

    uv run video.py OUT.mp4

writes the frames to `OUT.mp4`, near-lossless, and the captions beside it as
`OUT.srt`, then fails if a take's truss was not cut. `record.sh` runs it and
encodes the two into the video to upload.

The storyboard is the constants below: the shots, the `TAKES` filmed close
up, and `PACE`, how fast each part plays. Frames are sampled on the
simulation's clock, one every so many physics steps however long those take
to compute, so the video plays in simulated time.
"""

import argparse
import dataclasses
import pathlib
import subprocess

import numpy as np

import isaaclab.sim as sim_utils
from isaaclab.app import launch_simulation

import harvest as demo

SIZE = (1920, 1080)
"""The frames, in pixels. `record.sh` scales them down to the video's size."""

FPS = 25
"""Frames a second of video: a frame is 8 physics steps of simulated time, at
the simulation's own speed."""

PATHS = [1]
"""The path harvested: Path_01, between the two rows, straight down the rail
from where the trolley starts."""

SETTLE = 1.5
"""Seconds of simulated time not filmed at the start, while the arm folds
from where it spawned."""


@dataclasses.dataclass(frozen=True)
class Shot:
    """Where the camera stands and what it looks at, relative to a point it
    frames: in meters along the path the way the trolley drives in, across it
    toward the truss cut, or the left-hand row, and up."""

    eye: tuple[float, float, float]
    look: tuple[float, float, float] = (0.0, 0.0, 0.0)
    """Where it aims, from the point."""
    lens: float = 24.0
    """Focal length, in millimetres of a 21 mm wide sensor: 24 is a 47 degree
    view across, 50 a 24 degree one."""
    f_stop: float = 0.0
    """The aperture, focused on where the camera aims; 0 keeps everything
    sharp. The renderer's depth of field is strong for the number."""
    caption: str | None = None


CLOSE = Shot(eye=(0.5, -0.9, 0.2), look=(0.0, 0.0, -0.15))
"""The cuts' close-up: from the path a little ahead of the cut, so the
cutter comes in from behind it, toward the camera."""

FALLING = Shot(eye=(0.5, -0.9, 0.2), look=(0.0, 0.0, -0.6), lens=18.0)
"""The same, looking down after the cut, at the truss falling to the floor."""

TAKES = (
    "A truss is due once its least ripe fruit is turning",
    "The cutter comes in square to the peduncle, on a straight line",
    "Cut at its stem, the truss drops whole to the floor",
)
"""The captions of the cuts filmed close up: the first the trolley makes, in
order."""

OPENING = Shot(
    eye=(-3.2, -2.8, 1.8),
    look=(-0.2, 0.0, 0.8),
    lens=20.0,
    caption="Harvesting tomato trusses with a pipe-rail robot, in Isaac Sim",
)
"""From the main path, behind the trolley and to its right, as it drives
into the path with the arm folded. Points along the path count from its
mouth, where the rows start."""

REST = Shot(
    eye=(5.0, -0.4, 2.6),
    look=(1.0, 0.0, 0.8),
    caption="The rest of the path, four times faster",
)
"""From past the far end of the rows, looking back down the path at the
trolley, while the cuts no take has are made: the lift column hides the arm
from behind."""

CLOSING = Shot(
    eye=(5.0, -0.4, 2.0),
    look=(1.5, 0.0, 0.3),
    caption="Every due truss cut and on the floor",
)
"""The same, lower: the trusses on the floor and the trolley driving out."""

PACE = {"move": 1.0, "approach": 2.0, "cut": 2.0, "retract": 2.0, "drop": 3.0, "rest": 0.25}
"""How many times slower than the simulation the video plays: a take's
stages by `Harvest.stage`, the truss dropping after its cut, and the cuts no
take has. The rest plays at the simulation's own speed."""

DROP = 0.8
"""Seconds of simulated time a take stays on after its cut, while the truss
falls."""

EASE = 1.5
"""Seconds of video the camera takes from one shot to the next."""

CUT = 1.5
"""How far the camera may travel from one shot to the next, in meters,
before it cuts to the next rather than easing there. It cuts from one take
to the next too: the arm moves on to the next truss past where the last
take's camera stands."""

CAMERA = "/World/VideoCamera"


class Rig:
    """The camera, easing from one shot to the next: eye, aim, focal length
    and aperture in one array, on a smoothstep over `EASE` seconds."""

    def __init__(self, stage):
        from pxr import UsdGeom

        self.prim = stage.GetPrimAtPath(CAMERA)
        self.camera = UsdGeom.Camera(self.prim)
        self.pose = self.start = self.goal = None
        self.eased = 1.0

    def aim(self, goal: np.ndarray, cut: bool = False) -> None:
        """Move towards `goal` by a frame, starting a new ease if it is a new
        goal -- or cutting to it, if it is `CUT` away or `cut` says so -- and
        place the camera there."""
        from pxr import Gf, UsdGeom

        if cut or self.goal is None or np.linalg.norm(goal[:3] - self.goal[:3]) > CUT:
            self.pose = self.start = self.goal = goal
        elif not np.allclose(goal, self.goal):
            self.start, self.goal, self.eased = self.pose, goal, 0.0
        self.eased = min(self.eased + 1 / (EASE * FPS), 1.0)
        s = self.eased * self.eased * (3 - 2 * self.eased)
        self.pose = self.start + s * (self.goal - self.start)
        eye, target, (lens, f_stop) = self.pose[:3], self.pose[3:6], self.pose[6:]
        # Placed on the prim: under Newton, `Camera.set_world_poses` moves a
        # Newton site the renderer never sees. USD's look-at is the view
        # matrix, world to camera, so the prim takes its inverse.
        view = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(0, 0, 1))
        xform = UsdGeom.Xformable(self.prim)
        xform.ClearXformOpOrder()
        xform.AddTransformOp().Set(view.GetInverse())
        self.camera.GetFocalLengthAttr().Set(float(lens))
        self.camera.GetFStopAttr().Set(float(f_stop))
        self.camera.GetFocusDistanceAttr().Set(float(np.linalg.norm(target - eye)))


class Film:
    """Frames piped to ffmpeg as they come, and captions timed against
    them, written out as SubRip when the film is done."""

    def __init__(self, path: pathlib.Path):
        self.path = path
        self.frames = 0
        self.captions: list[tuple[int, int, str]] = []
        w, h = SIZE
        self.ffmpeg = subprocess.Popen(
            # Near-lossless and quick to write: `record.sh` encodes the video
            # to upload from this.
            [
                *f"ffmpeg -y -loglevel error -f rawvideo -pix_fmt rgb24 -s {w}x{h} -r {FPS}".split(),
                *"-i - -c:v libx264 -preset fast -crf 10 -pix_fmt yuv420p".split(),
                str(path),
            ],
            stdin=subprocess.PIPE,
        )

    def add(self, rgb: np.ndarray, caption: str | None) -> None:
        """Append a frame, under `caption`."""
        if caption and not (self.captions and self.captions[-1][2] == caption):
            self.captions.append((self.frames, self.frames, caption))
            print(f"[INFO]: caption {caption!r} at {self.frames / FPS:.1f} s")
        if caption:
            first, _, text = self.captions[-1]
            self.captions[-1] = (first, self.frames + 1, text)
        self.ffmpeg.stdin.write(rgb.tobytes())
        self.frames += 1

    def close(self) -> None:
        self.ffmpeg.stdin.close()
        self.ffmpeg.wait()

        def stamp(frame: int) -> str:
            s, ms = divmod(round(1000 * frame / FPS), 1000)
            return f"{s // 3600:02}:{s // 60 % 60:02}:{s % 60:02},{ms:03}"

        self.path.with_suffix(".srt").write_text(
            "".join(
                f"{i}\n{stamp(first)} --> {stamp(last)}\n{text}\n\n"
                for i, (first, last, text) in enumerate(self.captions, 1)
            )
        )


def framing(shot: Shot, point: np.ndarray, side: np.ndarray) -> np.ndarray:
    """The camera's pose for `shot` of `point`, `side` the horizontal direction
    across the path it counts from: eye, aim, focal length and aperture."""
    axes = np.column_stack([[1.0, 0.0, 0.0], side, [0.0, 0.0, 1.0]])
    eye, target = point + axes @ shot.eye, point + axes @ shot.look
    return np.array([*eye, *target, shot.lens, shot.f_stop])


def main():
    parser = argparse.ArgumentParser(description="Records the harvest demo's video, headless.")
    parser.add_argument("out", type=pathlib.Path, help="the mp4 to write; captions go beside it")
    out = parser.parse_args().out.resolve()
    sim_cfg = demo.simulation_cfg()
    # Kit, headless, with nothing to look at the scene but the camera below:
    # its renderer needs Kit's, which a camera made after launch cannot ask for.
    launcher = {"headless": True, "enable_cameras": True, "require_kit": True, "visualizer": []}
    with launch_simulation(sim_cfg, launcher):
        # Imported once Kit is up: they bring `pxr`, and Kit's copy has to win.
        from isaaclab.sensors import Camera, CameraCfg
        from isaaclab_physx.renderers import IsaacRtxRendererCfg

        sim = sim_utils.SimulationContext(sim_cfg)
        machine = demo.make_machine()
        robot = demo.design_scene(machine)
        camera = Camera(
            CameraCfg(
                prim_path=CAMERA,
                update_period=0.0,
                width=SIZE[0],
                height=SIZE[1],
                data_types=["rgb"],
                spawn=sim_utils.PinholeCameraCfg(clipping_range=(0.02, 200.0)),
                # The robot is in env_0's render partition, which cameras outside env_0 cannot see.
                renderer_cfg=IsaacRtxRendererCfg(enable_scene_partitioning=False),
            )
        )
        stage = sim_utils.get_current_stage()
        y = demo.path_ys(stage, demo.GREENHOUSE_PATH)[PATHS[0]]
        entrance, left = np.array([0.0, y, 0.0]), np.array([0.0, 1.0, 0.0])
        rig, film = Rig(stage), Film(out)
        # The takes' captions, by the prim of the truss each films.
        captions: dict[str, str] = {}
        # Frames due, counted a fraction a physics step: an eighth, at the
        # simulation's own speed. `last` is the truss cut last, at `cut_at`,
        # and `shown` the one the last frame filmed.
        due, last, cut_at, shown = 0.0, None, 0.0, None
        try:
            for step, harvest in enumerate(demo.run(sim, robot, machine, PATHS, render=False)):
                now = step * demo.SIM_DT
                truss = harvest.job if isinstance(harvest.job, demo.Truss) else None
                if truss and truss.prim not in captions and len(captions) < len(TAKES):
                    captions[truss.prim] = TAKES[len(captions)]
                if harvest.cut and harvest.cut[-1] is not last:
                    last, cut_at = harvest.cut[-1], now
                filmed = None
                if last is not None and last.prim in captions and now - cut_at < DROP:
                    shot, filmed, pace = FALLING, last, "drop"
                elif (
                    truss
                    and truss.prim in captions
                    and not (last is None and len(harvest.goals) > 1)
                ):
                    # The first take starts as the arm reaches out at the
                    # end of the drive in.
                    shot, filmed, pace = CLOSE, truss, harvest.stage
                elif last is None:
                    shot, pace = OPENING, None
                elif truss or any(isinstance(job, demo.Truss) for job in harvest.jobs):
                    shot, pace = REST, "rest"
                else:
                    shot, pace = CLOSING, None
                due += PACE.get(pace, 1.0) * FPS * demo.SIM_DT
                if now < SETTLE or due < 1:
                    continue
                due -= 1
                if filmed:
                    point, side, caption = filmed.cut[:3, 3], filmed.side, captions[filmed.prim]
                else:
                    point, side, caption = entrance, left, shot.caption
                rig.aim(framing(shot, point, side), cut=filmed is not shown)
                shown = filmed
                sim.render()
                camera.update(demo.SIM_DT, force_recompute=True)
                film.add(camera.data.output["rgb"].torch[0, ..., :3].cpu().numpy(), caption)
        finally:
            film.close()
        print(f"[INFO]: {film.frames} frames, {film.frames / FPS:.1f} s, written to {out}")
        missed = captions.keys() - {truss.prim for truss in harvest.cut}
        if len(captions) < len(TAKES) or missed:
            raise RuntimeError(f"of {len(TAKES)} takes, these trusses were not cut: {missed}")


if __name__ == "__main__":
    main()

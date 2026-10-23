"""Videos of a running simulation for the docs, filmed in Isaac Sim's RTX
renderer by a camera that moves from shot to shot.

A script makes the camera before the simulation's first reset, then samples
it on the simulation's clock, a frame every so many physics steps however
long those take to compute, so the video plays in simulated time. Each frame
it aims the `Rig`, renders, and hands the picture to the `Film`, under the
frame's caption:

    camera = sensor(SIZE)
    sim.reset()
    rig, film = Rig(stage, FPS, ease=1.5, cut=1.0), Film(out, SIZE, FPS)
    try:
        for ... in the simulation's steps, every so many:
            rig.aim(np.array([*eye, *target, focal_length, f_stop]))
            sim.render()
            camera.update(dt, force_recompute=True)
            film.add(camera.data.output["rgb"].torch[0, ..., :3].cpu().numpy(), caption)
    finally:
        film.close()
"""

from __future__ import annotations

import pathlib
import subprocess
from typing import TYPE_CHECKING

import isaaclab.sim as sim_utils
import numpy as np

from .shots import place

if TYPE_CHECKING:
    from isaaclab.sensors import Camera
    from pxr import Usd

CAMERA = "/World/VideoCamera"


def sensor(size: tuple[int, int], lens: float = 24.0) -> Camera:
    """The camera that films, at `CAMERA`, `size` pixels wide and high, with
    a `lens` of that many millimetres until a `Rig` sets its own. Made before
    the simulation's first reset, as every sensor is."""
    # Imported here: they bring `pxr`, and Kit's copy has to be the first one.
    from isaaclab.sensors import Camera, CameraCfg
    from isaaclab_physx.renderers import IsaacRtxRendererCfg

    return Camera(
        CameraCfg(
            prim_path=CAMERA,
            update_period=0.0,
            width=size[0],
            height=size[1],
            data_types=["rgb"],
            spawn=sim_utils.PinholeCameraCfg(focal_length=lens, clipping_range=(0.02, 200.0)),
            # A robot is in its environment's render partition, which a
            # camera outside the environments sees only with partitions off.
            renderer_cfg=IsaacRtxRendererCfg(enable_scene_partitioning=False),
        )
    )


class Rig:
    """The camera at `CAMERA` on `stage`, easing from one shot to the next:
    eye, aim, focal length and aperture in one array, on a smoothstep over
    `ease` seconds of video at `fps` frames a second. To a shot whose eye
    stands more than `cut` meters from the last one's it cuts instead, as a
    longer move would fly through whatever stands between."""

    def __init__(self, stage: Usd.Stage, fps: int, ease: float, cut: float):
        from pxr import UsdGeom

        self.prim = stage.GetPrimAtPath(CAMERA)
        self.camera = UsdGeom.Camera(self.prim)
        self.fps, self.ease, self.cut = fps, ease, cut
        self.pose = self.start = self.goal = None
        self.eased = 1.0

    def aim(self, goal: np.ndarray, cut: bool = False) -> None:
        """Move towards `goal` by a frame, starting a new ease if it is a new
        goal -- or cutting to it, if it is too far to ease to or `cut` says
        so -- and place the camera there."""
        if cut or self.goal is None or np.linalg.norm(goal[:3] - self.goal[:3]) > self.cut:
            self.pose = self.start = self.goal = goal
        elif not np.allclose(goal, self.goal):
            self.start, self.goal, self.eased = self.pose, goal, 0.0
        self.eased = min(self.eased + 1 / (self.ease * self.fps), 1.0)
        s = self.eased * self.eased * (3 - 2 * self.eased)
        self.pose = self.start + s * (self.goal - self.start)
        eye, target, (lens, f_stop) = self.pose[:3], self.pose[3:6], self.pose[6:]
        place(self.prim, eye, target)
        self.camera.GetFocalLengthAttr().Set(float(lens))
        self.camera.GetFStopAttr().Set(float(f_stop))
        self.camera.GetFocusDistanceAttr().Set(float(np.linalg.norm(target - eye)))


class Film:
    """Frames piped to ffmpeg as they come, `size` pixels at `fps` frames a
    second, and captions timed against them, written out beside the video as
    SubRip when the film is done."""

    def __init__(self, path: pathlib.Path, size: tuple[int, int], fps: int):
        self.path, self.fps = path, fps
        self.frames = 0
        self.captions: list[tuple[int, int, str]] = []
        w, h = size
        self.ffmpeg = subprocess.Popen(
            # Near-lossless and quick to write: the video to upload is
            # encoded from this.
            [
                *f"ffmpeg -y -loglevel error -f rawvideo -pix_fmt rgb24 -s {w}x{h} -r {fps}".split(),
                *"-i - -c:v libx264 -preset fast -crf 10 -pix_fmt yuv420p".split(),
                str(path),
            ],
            stdin=subprocess.PIPE,
        )

    def add(self, rgb: np.ndarray, caption: str | None = None) -> None:
        """Append a frame, under `caption`."""
        if caption and not (self.captions and self.captions[-1][2] == caption):
            self.captions.append((self.frames, self.frames, caption))
            print(f"[INFO]: caption {caption!r} at {self.frames / self.fps:.1f} s")
        if caption:
            first, _, text = self.captions[-1]
            self.captions[-1] = (first, self.frames + 1, text)
        self.ffmpeg.stdin.write(rgb.tobytes())
        self.frames += 1

    def close(self) -> None:
        self.ffmpeg.stdin.close()
        self.ffmpeg.wait()

        def stamp(frame: int) -> str:
            s, ms = divmod(round(1000 * frame / self.fps), 1000)
            return f"{s // 3600:02}:{s // 60 % 60:02}:{s % 60:02},{ms:03}"

        self.path.with_suffix(".srt").write_text(
            "".join(
                f"{i}\n{stamp(first)} --> {stamp(last)}\n{text}\n\n"
                for i, (first, last, text) in enumerate(self.captions, 1)
            )
        )

"""Tests for a video's camera, which eases between near shots and cuts to far
ones, and for its captions.

Requires Isaac Lab; skipped entirely where it isn't installed.
"""

from __future__ import annotations

from pathlib import Path

import pytest

# Not `pytest.importorskip`: see `test_isaaclab_shots.py`.
try:
    from misina_lab.isaaclab import film
except ImportError:
    pytest.skip("Isaac Lab is not installed", allow_module_level=True)

import numpy as np  # noqa: E402
from pxr import Gf, Usd, UsdGeom  # noqa: E402


def test_the_camera_eases_to_a_near_shot_and_cuts_to_a_far_one():
    """At 4 frames a second, a 1 s ease is halfway after 2 frames and over
    after 4. Every shot looks along +y."""
    stage = Usd.Stage.CreateInMemory()
    prim = UsdGeom.Camera.Define(stage, film.CAMERA).GetPrim()
    rig = film.Rig(stage, fps=4, ease=1.0, cut=1.0)

    def frame(x: float, times: int = 1) -> np.ndarray:
        """Where the camera stands along x after `times` frames aimed at a
        shot from `x`, and the way it looks."""
        for _ in range(times):
            rig.aim(np.array([x, 0.0, 1.0, x, 5.0, 1.0, 24.0, 0.0]))
        to_world = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        look = to_world.TransformDir(Gf.Vec3d(0, 0, -1))
        return np.array([to_world.ExtractTranslation()[0], *look])

    assert np.allclose(frame(0.0), [0.0, 0, 1, 0]), "the first shot is cut to"
    assert np.allclose(frame(0.8, 2), [0.4, 0, 1, 0]), "a 0.8 m move eases"
    assert np.allclose(frame(0.8, 2), [0.8, 0, 1, 0])
    assert np.allclose(frame(2.0), [2.0, 0, 1, 0]), "a 1.2 m move is past `cut`"


def test_a_caption_runs_from_its_first_frame_to_the_end_of_its_last(tmp_path: Path):
    reel = film.Film(tmp_path / "out.mp4", (16, 16), fps=4)
    for caption in (None, "One", "One", "Two", None):
        reel.add(np.zeros((16, 16, 3), np.uint8), caption)
    reel.close()
    assert (tmp_path / "out.srt").read_text() == (
        "1\n00:00:00,250 --> 00:00:00,750\nOne\n\n2\n00:00:00,750 --> 00:00:01,000\nTwo\n\n"
    )

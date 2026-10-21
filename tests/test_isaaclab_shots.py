"""Tests for a shot's camera, which has to mean what it means to the viewer.

Requires Isaac Lab; skipped entirely where it isn't installed.
"""

from __future__ import annotations

import pytest

# Not `pytest.importorskip`: `misina_lab.isaaclab` re-raises a missing Isaac Lab
# under its own message, and pytest skips only when the error names the module
# it was asked to import.
try:
    from misina_lab.isaaclab import shots
except ImportError:
    pytest.skip("Isaac Lab is not installed", allow_module_level=True)

# Past the skip: `misina-lab` itself needs only usd-core, so numpy comes with
# Isaac Lab.
import numpy as np  # noqa: E402


class Slope:
    """Ground rising 10 cm a meter along x."""

    def height(self, x: float, y: float) -> float:
        return 0.1 * x


@pytest.mark.parametrize(("yaw", "side"), [(0, [0, -1]), (90, [1, 0])])
def test_yaw_turns_the_camera_as_the_viewer_does(yaw: float, side: list[float]):
    """0 puts the camera on the focus's -y side, 90 on its +x side."""
    camera = {"focus": [1.0, 2.0, 0.5], "yaw": yaw, "pitch": 0.0, "radius": 3.0}
    eye, target = shots.aim(camera, Slope())
    assert np.allclose(eye - target, 3 * np.array([*side, 0]))


@pytest.mark.parametrize(("ground", "under"), [(Slope(), 0.3), (None, 0.0)])
def test_a_printed_camera_is_the_viewer_s(ground: Slope | None, under: float):
    """`pose` inverts `aim`, the focus over the ground included, and prints
    what the viewer's P key prints for the same camera. With no ground, the
    ground is at z = 0."""
    camera = {"focus": [3.0, -4.5, 1.2], "yaw": -35.0, "pitch": 12.5, "radius": 8.25}
    eye, target = shots.aim(camera, ground)
    assert target[2] == pytest.approx(under + 1.2)
    assert shots.pose(eye, target, 30.0, ground) == (
        '"camera": {"focus": [3.00, -4.50, 1.20], "yaw": -35.0, "pitch": 12.5,'
        ' "radius": 8.25, "fov": 30.0}'
    )

"""Tests for a shot's camera, which has to mean what it means to the viewer.

`shot` reaches Isaac Lab, so this is skipped entirely where that isn't installed.
"""

from __future__ import annotations

import numpy as np
import pytest

shot = pytest.importorskip("shot", reason="Isaac Lab is not installed")


class Slope:
    """Ground rising 10 cm a meter along x."""

    def height(self, x: float, y: float) -> float:
        return 0.1 * x


@pytest.mark.parametrize(("yaw", "side"), [(0, [0, -1]), (90, [1, 0])])
def test_yaw_turns_the_camera_as_the_viewer_does(yaw: float, side: list[float]):
    """0 puts the camera on the focus's -y side, 90 on its +x side."""
    camera = {"focus": [1.0, 2.0, 0.5], "yaw": yaw, "pitch": 0.0, "radius": 3.0}
    eye, target = shot.aim(camera, Slope())
    assert np.allclose(eye - target, 3 * np.array([*side, 0]))


def test_a_printed_camera_is_the_viewer_s():
    """`pose` inverts `aim`, the focus over the ground included, and prints
    what the viewer's P key prints for the same camera."""
    camera = {"focus": [3.0, -4.5, 1.2], "yaw": -35.0, "pitch": 12.5, "radius": 8.25}
    eye, target = shot.aim(camera, Slope())
    assert target[2] == pytest.approx(0.3 + 1.2)
    assert shot.pose(eye, target, 30.0, Slope()) == (
        '"camera": {"focus": [3.00, -4.50, 1.20], "yaw": -35.0, "pitch": 12.5,'
        ' "radius": 8.25, "fov": 30.0}'
    )

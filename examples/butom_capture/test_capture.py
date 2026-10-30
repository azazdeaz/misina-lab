"""Checks of `capture` that need no Kit: where a pass goes, how depth is
stored, and that importing the scripts leaves `pxr` unloaded. Kit ships its own
`pxr` and dies during startup if the pip one was imported first."""

import pathlib
import random
import subprocess
import sys

import numpy as np
import pytest

pytest.importorskip("isaaclab", reason="Isaac Lab is not installed")

import capture  # noqa: E402


def test_importing_the_scripts_does_not_load_pxr():
    # A subprocess, so the check is not fooled by another test's imports.
    subprocess.run(
        [sys.executable, "-c", "import capture, shot, sys; assert 'pxr' not in sys.modules"],
        cwd=pathlib.Path(__file__).parent,
        check=True,
    )


@pytest.mark.parametrize("longest", [False, True])
@pytest.mark.parametrize("seed", range(20))
def test_a_pass_stays_on_its_row_and_drives_forward(seed, longest):
    layout = capture.GREENHOUSE_CFG.layout
    frames = capture.most_frames(layout) if longest else capture.FRAMES
    row, face, xs = capture.plan(random.Random(seed), layout, frames)
    assert 0 <= row < layout.rows
    assert capture.MARGIN <= xs.min() and xs.max() <= layout.row_length - capture.MARGIN
    # The cameras look to the trolley's right: along +x past a row's +y face,
    # along -x past its -y face.
    np.testing.assert_allclose(np.diff(xs) * face, capture.SPEED / capture.FPS)


def test_one_frame_more_than_the_most_leaves_the_margins():
    layout = capture.GREENHOUSE_CFG.layout
    run = capture.SPEED * capture.most_frames(layout) / capture.FPS
    assert run > layout.row_length - 2 * capture.MARGIN


def test_depth_is_millimetres_and_zero_out_of_range():
    depth = np.array([0.5, 1.2344, 65.0, np.inf, np.nan, 70.0])
    assert capture.millimetres(depth).tolist() == [500, 1234, 65000, 0, 0, 0]

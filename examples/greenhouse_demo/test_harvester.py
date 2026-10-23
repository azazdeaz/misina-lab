"""Tests for the robot.

`harvester` reaches Isaac Lab through `isaaclab.sim`, so this is skipped
entirely where that isn't installed. The arm comes from the Hugging Face
dataset, which the first run downloads.
"""

from __future__ import annotations

import xml.etree.ElementTree as ElementTree

import numpy as np
import pytest

harvester = pytest.importorskip("harvester", reason="Isaac Lab is not installed")

import kinematics  # noqa: E402

MACHINE = harvester.Harvester()


def test_the_arm_is_posed_where_mujoco_poses_menageries():
    """The joints read out of the MJCF put the flange where MuJoCo puts the
    model's own attachment site, at any joint positions."""
    mujoco = pytest.importorskip("mujoco")
    model = mujoco.MjModel.from_xml_path(str(harvester.ur5e_folder() / "ur5e.xml"))
    data = mujoco.MjData(model)
    site = model.site("attachment_site").id
    # MuJoCo's world is the platform the arm stands on.
    platform = np.linalg.inv(kinematics.frames(MACHINE.chain, np.zeros(9))[2])
    for q in np.random.default_rng(0).uniform(-3.0, 3.0, (20, 6)):
        data.qpos[:] = q
        mujoco.mj_forward(model, data)
        flange = (
            platform
            @ kinematics.frames(MACHINE.chain, [0, 0, 0, *q])[-1]
            @ harvester._ur5e().flange
        )
        assert flange[:3, 3] == pytest.approx(data.site_xpos[site], abs=1e-9)
        assert flange[:3, :3] == pytest.approx(data.site_xmat[site].reshape(3, 3), abs=1e-9)


def test_the_urdf_moves_the_chains_joints_in_order():
    """The importer builds the joints the solver poses, in the order it
    poses them, with the same limits."""
    built = ElementTree.fromstring(harvester.urdf(MACHINE))
    moving = [joint for joint in built.iter("joint") if joint.get("type") != "fixed"]
    assert [joint.get("name") for joint in moving] == [joint.name for joint in MACHINE.chain]
    for element, joint in zip(moving, MACHINE.chain, strict=True):
        assert element.get("type") == ("prismatic" if joint.prismatic else "revolute")
        limit = element.find("limit")
        assert (float(limit.get("lower")), float(limit.get("upper"))) == joint.limits

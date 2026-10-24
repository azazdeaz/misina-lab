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


@pytest.mark.parametrize(
    "shape",
    [
        harvester.Shape("capsule", (0.0, 0.02, 0.1), (0.04, 0.04), (np.pi / 2, 0.0, 0.0)),
        harvester.Shape("cylinder", (0.0, 0.0, 0.2), (0.038, 0.19)),
        harvester.CUTTER_HEAD,
        *harvester.blades(),
    ],
)
def test_a_shapes_spheres_hold_all_of_it(shape):
    """Every point of a shape is inside one of the spheres that cover it, so
    a pose cuRobo finds clear is clear of the shape too."""
    points = np.random.default_rng(0).uniform(-1.0, 1.0, (4000, 3))
    if shape.kind == "box":
        points *= np.array(shape.size) / 2
    else:
        radius, half = shape.size
        points[:, :2] *= radius
        points = points[np.linalg.norm(points[:, :2], axis=1) <= radius]
        points[:, 2] *= half + (radius if shape.kind == "capsule" else 0.0)
        if shape.kind == "capsule":
            off = np.abs(points[:, 2]) - half
            points = points[
                (off <= 0) | (np.hypot(np.linalg.norm(points[:, :2], axis=1), off) <= radius)
            ]
    frame = kinematics.transform(shape.xyz, shape.rpy)
    points = points @ frame[:3, :3].T + frame[:3, 3]
    centres, radii = map(np.array, zip(*shape.cover(), strict=True))
    gaps = np.linalg.norm(points[:, None] - centres[None], axis=-1) - radii[None]
    assert gaps.min(axis=1).max() <= 1e-12


def test_curobo_poses_the_cutter_where_the_chain_does():
    """cuRobo reads the URDF the simulation imports: from the platform, its
    cutter is where `Harvester.chain` puts the flange, at any arm pose."""
    pytest.importorskip("curobo")
    torch = pytest.importorskip("torch")
    if not torch.cuda.is_available():
        pytest.skip("cuRobo needs a GPU")
    from curobo.kinematics import Kinematics, KinematicsCfg
    from curobo.types import JointState

    robot = Kinematics(KinematicsCfg.from_data_dict(harvester.curobo_cfg(MACHINE)["kinematics"]))
    q = np.random.default_rng(0).uniform(-3.0, 3.0, (20, 6))
    state = JointState.from_position(
        torch.tensor(q, dtype=torch.float32, device="cuda"), joint_names=robot.joint_names
    )
    poses = robot.compute_kinematics(state).tool_poses.get_link_pose(harvester.CUTTER)
    for row, cutter in zip(q, poses.get_matrix().cpu().numpy(), strict=True):
        chain = kinematics.frames(MACHINE.chain, [0, 0, 0, *row])
        flange = np.linalg.inv(chain[2]) @ chain[-1] @ harvester._ur5e().flange
        assert cutter == pytest.approx(flange, abs=1e-4)

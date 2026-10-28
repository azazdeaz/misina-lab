"""Tests for the robot.

`harvester` reaches Isaac Lab through `isaaclab.sim`, so this is skipped
entirely where that isn't installed. The arm comes from the Hugging Face
dataset, which the first run downloads.
"""

from __future__ import annotations

import itertools
import xml.etree.ElementTree as ElementTree

import numpy as np
import pytest

harvester = pytest.importorskip("harvester", reason="Isaac Lab is not installed")

import kinematics  # noqa: E402
import shear  # noqa: E402

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
    poses them, with the same limits, and the shear's and its finger's
    after them."""
    built = ElementTree.fromstring(harvester.urdf(MACHINE))
    moving = [joint for joint in built.iter("joint") if joint.get("type") != "fixed"]
    names = [joint.get("name") for joint in moving]
    assert names == [
        *(joint.name for joint in MACHINE.chain),
        shear.SHEAR_JOINT,
        shear.FINGER_JOINT,
    ]
    for element, joint in zip(moving, MACHINE.chain, strict=False):
        assert element.get("type") == ("prismatic" if joint.prismatic else "revolute")
        limit = element.find("limit")
        assert (float(limit.get("lower")), float(limit.get("upper"))) == joint.limits


@pytest.mark.parametrize(
    "shape",
    [
        harvester.Shape("capsule", (0.0, 0.02, 0.1), (0.04, 0.04), (np.pi / 2, 0.0, 0.0)),
        harvester.Shape("cylinder", (0.0, 0.0, 0.2), (0.038, 0.19)),
        *(box for boxes in harvester.shear_shapes(MACHINE.shear).values() for box in boxes),
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


def test_the_crates_boxes_are_its_mesh():
    """Every corner of the crate's mesh, its ribs and rim included, is in one
    of the boxes it collides as, and the boxes reach no further out than the
    mesh does: a wall leaning the wrong way would leave its rim out."""
    crate = MACHINE.crate
    points = np.unique(crate.mesh().reshape(-1, 3), axis=0)
    gaps, corners = [], []
    for box in crate.boxes():
        frame = kinematics.transform(box.xyz, box.rpy)
        half = np.array(box.size) / 2
        local = (points - frame[:3, 3]) @ frame[:3, :3]
        gaps.append(np.linalg.norm(np.clip(np.abs(local) - half, 0.0, None), axis=1))
        ends = np.array(list(itertools.product(*zip(-half, half, strict=True))))
        corners.append(ends @ frame[:3, :3].T + frame[:3, 3])
    assert np.min(gaps, axis=0).max() < 0.001
    corners = np.concatenate(corners)
    assert corners.min(axis=0) == pytest.approx(points.min(axis=0), abs=0.001)
    assert corners.max(axis=0) == pytest.approx(points.max(axis=0), abs=0.001)


@pytest.mark.parametrize(("way", "clear"), [(MACHINE.way_in, True), ((0.0, 0.0, 1.0), False)])
def test_a_peduncle_comes_in_under_the_hook(way, clear):
    """The mouth coming in along `way_in`, a 7 mm peduncle passes under the
    hook's claw into the mouth and on to `AIM` without touching the hook;
    coming in along the blades, the claw curling down past `AIM` sweeps
    through it. In the shear's YZ plane, the peduncle's section runs from a
    blade's length back along the way in to `AIM`, and the hook lies above
    its edge."""
    s, radius = MACHINE.shear, 0.0035
    out, off = harvester.AIM
    aim = np.array(s.off_edge(out * s.blade, off))
    path = aim + np.linspace(0.0, s.blade, 400)[:, None] * np.array(way)[1:]
    z = np.linspace(0.0, s.blade + s.claw, 400)
    across = z[None, :] - path[:, 1:]
    top = path[:, :1] + np.sqrt(np.clip(radius**2 - across**2, 0.0, None))
    assert (top <= s.edge(z)[None, :])[np.abs(across) < radius].all() == clear


def test_curobo_poses_the_shear_where_the_chain_does():
    """cuRobo reads the URDF the simulation imports: from the platform, the
    shear's head is where `Harvester.chain` puts it, at any arm pose."""
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
    poses = robot.compute_kinematics(state).tool_poses.get_link_pose(shear.HEAD)
    for row, head in zip(q, poses.get_matrix().cpu().numpy(), strict=True):
        chain = kinematics.frames(MACHINE.chain, [0, 0, 0, *row])
        assert head == pytest.approx(
            np.linalg.inv(chain[2]) @ chain[-1] @ MACHINE.shear_frame, abs=1e-4
        )


def test_a_stems_fin_runs_from_its_front_back_to_its_rows_middle():
    """An upright stem's box is swept back, square to the stem, from its
    front to the middle of its row; a stem lowered along the gutter stays as
    it is, as does any other box."""
    planner = pytest.importorskip("planner")
    poses = np.array(
        [
            kinematics.transform((0.0, -0.2, 1.2), (0.15, 0.0, 0.0)),  # leaning out of the row
            kinematics.transform((0.0, -0.1, 0.6), (0.0, np.pi / 2, 0.0)),  # lowered, along x
            np.eye(4),
        ]
    )
    size = np.array([[0.02, 0.02, 1.0]] * 3)
    swept, grown = poses.copy(), size.copy()
    planner._fins(swept, grown, np.array([0.1, 0.1, np.nan]))
    assert swept[1:] == pytest.approx(poses[1:]) and grown[1:] == pytest.approx(size[1:])
    fin, half = swept[0], grown[0, 1] / 2
    assert fin[:3, :3] @ fin[:3, :3].T == pytest.approx(np.eye(3))
    assert np.linalg.det(fin[:3, :3]) == pytest.approx(1.0)
    assert fin[:3, 2] == pytest.approx(poses[0, :3, 2])
    front, back = fin[:3, 3] - fin[:3, 1] * half, fin[:3, 3] + fin[:3, 1] * half
    assert np.linalg.norm(front - poses[0, :3, 3]) == pytest.approx(0.01)
    assert front[1] < poses[0, 1, 3] < 0.1 < back[1]


def test_a_turned_approach_cuts_as_far_off_the_stem():
    """Every cut pose keeps the blades' plane along the stem and the hook
    up, and the fixed blade as far off the stem's axis as coming in along
    the row; turned, the shear points into the row from the path."""
    harvest = pytest.importorskip("harvest")
    truss = harvest.Truss("", np.eye(4), True, side=np.array([0.0, 1.0, 0.0]))
    truss.axis = np.array([0.3, 0.5, 1.0])
    along = np.array([0.0, -1.0, 0.0])  # at the path
    at = truss.axis + 0.016 * along + [0.0, 0.0, 0.005]
    poses = harvest.approaches(truss, at, along, MACHINE)
    turns = list(itertools.product(harvest.YAWS, harvest.TILTS))
    assert len(poses) == len(turns)
    for pose, (yaw, tilt) in zip(poses, turns, strict=True):
        x, y, z = pose[:3, :3].T
        assert pose[:3, :3] @ pose[:3, :3].T == pytest.approx(np.eye(3))
        assert x @ truss.stem == pytest.approx(0.0, abs=1e-9)
        assert x @ (pose[:3, 3] - truss.axis) == pytest.approx(0.016 - MACHINE.shear.bypass)
        assert z @ truss.side == pytest.approx(np.sin(yaw) * np.cos(tilt))
        assert y[2] > 0.0

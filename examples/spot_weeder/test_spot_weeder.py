"""Checks of the robot `spot_weeder.write_usd` writes, against the paper's
numbers. Its Spot is composed from NVIDIA's URL through Isaac Lab, so the
first run needs the network; nothing here starts Kit."""

import math
import pathlib
import subprocess
import sys

import numpy as np
import pytest

spot_weeder = pytest.importorskip("spot_weeder", reason="Isaac Lab is not installed")


def pitched(angle: float) -> np.ndarray:
    """A turn of `angle` about the y axis, nose down."""
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rolled(angle: float) -> np.ndarray:
    """A turn of `angle` about the x axis, left side up."""
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def fk(x: float, pitch: float, yaw: float, point: np.ndarray) -> np.ndarray:
    """Where `point`, in the body's frame, is in the ground frame, for a body
    shifted `x` forward and turned by `pitch` and `yaw`: the paper's Eq. 1."""
    c, s = math.cos(yaw), math.sin(yaw)
    yawed = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    return np.array([x, 0, spot_weeder.Z_NOM]) + yawed @ pitched(pitch) @ point


@pytest.fixture(scope="module")
def stage(tmp_path_factory):
    from isaaclab.utils.assets import retrieve_file_path
    from pxr import Usd

    written = spot_weeder.write_usd(tmp_path_factory.mktemp("robot"))
    cache = tmp_path_factory.mktemp("cache")
    return Usd.Stage.Open(retrieve_file_path(str(written), download_dir=str(cache)))


def pose(stage, path) -> tuple[np.ndarray, np.ndarray]:
    """The position and the rotation of the prim at `path`, with Spot
    standing at the origin as written: in its body frame."""
    from pxr import UsdGeom

    # USD's matrices take row vectors.
    matrix = np.array(UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(path))).T
    return matrix[:3, 3], matrix[:3, :3]


def test_the_bit_is_where_the_paper_puts_it(stage):
    tip, _ = pose(stage, f"/spot/{spot_weeder.BIT}")
    np.testing.assert_allclose(tip, spot_weeder.TIP, atol=1e-6)
    # Its Table 2: the body shifted 0.08 m and turned pi/10 in pitch and yaw.
    bit = fk(0.08, math.pi / 10, math.pi / 10, tip)
    np.testing.assert_allclose(bit * 1000, [715.44, 206.47, -32.53], atol=0.01)
    np.testing.assert_allclose(spot_weeder.pose_for(bit), [0.08, math.pi / 10, math.pi / 10])


@pytest.mark.parametrize("height", [-0.02, 0.0, 0.1])
def test_the_pose_puts_the_bit_on_the_point(height):
    for x, y in spot_weeder.E:
        point = (x, y, height)
        np.testing.assert_allclose(fk(*spot_weeder.pose_for(point), spot_weeder.TIP), point)


def test_the_shaft_stands_upright_with_the_bit_2_cm_in(stage):
    _, turn = pose(stage, f"/spot/{spot_weeder.BIT}")
    pitch = spot_weeder.pitch_to(-0.02)
    np.testing.assert_allclose(fk(0, pitch, 0, spot_weeder.TIP)[2], -0.02)
    up = pitched(pitch) @ turn[:, 2]
    assert math.degrees(math.acos(up[2])) < 0.5


@pytest.mark.parametrize(
    ("body", "raised"),
    [
        ((0.0, 0.0, 0.0), {}),
        ((0.0, 0.3, 0.0), {}),
        ((0.1, 0.25, 0.3), {}),
        ((-0.05, 0.27, -0.4), {}),
        ((0.05, 0.2, 0.2), {"fl": 0.05, "hr": 0.1}),
    ],
)
def test_the_stance_keeps_the_feet_where_they_stood(body, raised):
    angles = spot_weeder.stance(*body, raised)
    for name, angle in angles.items():
        low, high = spot_weeder.LIMITS[name[-2:]]
        assert type(angle) is float and low <= angle <= high
    for leg, (x, y) in spot_weeder.HIPS.items():
        out = math.copysign(spot_weeder.HIP, y)
        knee = np.array([spot_weeder.THIGH[0], 0, spot_weeder.THIGH[1]])
        shin = pitched(angles[f"{leg}_kn"]) @ [0, 0, -spot_weeder.SHIN]
        swung = pitched(angles[f"{leg}_hy"]) @ (knee + shin)
        foot = np.array([x, y, 0]) + rolled(angles[f"{leg}_hx"]) @ ([0, out, 0] + swung)
        stood = [x, y + out, spot_weeder.FOOT + raised.get(leg, 0.0)]
        np.testing.assert_allclose(fk(*body, foot), stood, atol=1e-9)


def test_spots_legs_are_as_the_stance_has_them(stage):
    from pxr import Gf, Usd, UsdGeom, UsdPhysics

    def joint(path):
        return UsdPhysics.RevoluteJoint(stage.GetPrimAtPath(path))

    def joint_at(path):
        return np.array(joint(path).GetLocalPos0Attr().Get())

    for leg, hip in spot_weeder.HIPS.items():
        np.testing.assert_allclose(joint_at(f"/spot/body/{leg}_hx"), [*hip, 0])
        position, turn = pose(stage, f"/spot/{leg}_hip")
        np.testing.assert_allclose(
            position + turn @ joint_at(f"/spot/{leg}_hip/{leg}_hy"),
            [hip[0], hip[1] + math.copysign(spot_weeder.HIP, hip[1]), 0],
        )
        knee = joint_at(f"/spot/{leg}_uleg/{leg}_kn")
        np.testing.assert_allclose(knee[[0, 2]], spot_weeder.THIGH)
        np.testing.assert_allclose(
            joint_at(f"/spot/{leg}_lleg/{leg}_ank"), [0, 0, -spot_weeder.SHIN]
        )
        # Each joint turns about the axis, and within the range, that the
        # stance takes it to.
        for name, parent, axis in (
            ("hx", "body", "X"),
            ("hy", f"{leg}_hip", "Y"),
            ("kn", f"{leg}_uleg", "Y"),
        ):
            moving = joint(f"/spot/{parent}/{leg}_{name}")
            assert moving.GetAxisAttr().Get() == axis
            assert moving.GetLocalRot0Attr().Get() == moving.GetLocalRot1Attr().Get() == Gf.Quatf(1)
            limits = moving.GetLowerLimitAttr().Get(), moving.GetUpperLimitAttr().Get()
            np.testing.assert_allclose(np.radians(limits), spot_weeder.LIMITS[name], atol=1e-6)
        feet = Usd.PrimRange(
            stage.GetPrimAtPath(f"/spot/{leg}_foot"), Usd.TraverseInstanceProxies()
        )
        (ball,) = [UsdGeom.Sphere(prim) for prim in feet if prim.IsA(UsdGeom.Sphere)]
        assert ball.GetRadiusAttr().Get() == pytest.approx(spot_weeder.FOOT)


def test_the_legs_reach_most_of_the_papers_workspace():
    """The paper's E was measured on the real robot, so a true K holds all of
    it; this one misses its outer corners, where Spot's hind legs run out of
    reach. Spot's joint limits alone give a K larger than the paper's
    0.402 m2."""
    step = 0.005
    x, y = np.meshgrid(np.arange(0.2, 1.2, step), np.arange(-0.8, 0.8, step))
    outline = spot_weeder.E
    area = abs(
        outline[:, 0] @ np.roll(outline[:, 1], 1) - outline[:, 1] @ np.roll(outline[:, 0], 1)
    )
    assert area / 2 == pytest.approx(0.165, abs=0.001)
    in_e = spot_weeder.inside(outline, x, y)
    assert in_e.sum() * step**2 == pytest.approx(0.165, abs=0.002)
    k = spot_weeder.reachable(x, y)
    assert (k & in_e).sum() / in_e.sum() > 0.9
    assert k.sum() * step**2 == pytest.approx(0.56, abs=0.01)


def test_the_tool_weighs_7_kg_centred_25_cm_ahead_of_the_front_hips(stage):
    from pxr import UsdPhysics

    mass, moment = 0.0, np.zeros(3)
    for link in (spot_weeder.TOOL, spot_weeder.BIT):
        body = UsdPhysics.MassAPI(stage.GetPrimAtPath(f"/spot/{link}"))
        position, turn = pose(stage, f"/spot/{link}")
        mass += body.GetMassAttr().Get()
        moment += body.GetMassAttr().Get() * (position + turn @ body.GetCenterOfMassAttr().Get())
    hip, _ = pose(stage, "/spot/fl_hip")
    assert mass == pytest.approx(7.0)
    assert moment[0] / mass - hip[0] == pytest.approx(0.25, abs=0.005)


def test_the_camera_looks_down_at_the_bit_from_26_cm(stage):
    from pxr import Gf, UsdGeom

    path = f"/spot/{spot_weeder.TOOL}/camera"
    position, turn = pose(stage, path)
    np.testing.assert_allclose(position - spot_weeder.TIP, [0, 0, 0.26], atol=1e-6)
    # Down the body's -z, the top of the picture forward.
    np.testing.assert_allclose(turn @ [0, 0, -1], [0, 0, -1], atol=1e-6)
    np.testing.assert_allclose(turn @ [0, 1, 0], [1, 0, 0], atol=1e-6)
    lens = UsdGeom.Camera(stage.GetPrimAtPath(path)).GetCamera()
    assert lens.GetFieldOfView(Gf.Camera.FOVHorizontal) == pytest.approx(70)
    assert lens.GetFieldOfView(Gf.Camera.FOVVertical) == pytest.approx(55)


@pytest.mark.parametrize("joint", ["body/tool_mount", f"{spot_weeder.TOOL}/{spot_weeder.SPINDLE}"])
def test_a_joint_holds_its_links_as_they_stand(stage, joint):
    """Its frame is the same place on both its links, so nothing jumps when
    the simulation starts."""
    from pxr import Gf, UsdGeom, UsdPhysics

    joint = UsdPhysics.Joint(stage.GetPrimAtPath(f"/spot/{joint}"))
    frames = []
    for body, at, rotation in (
        (joint.GetBody0Rel(), joint.GetLocalPos0Attr(), joint.GetLocalRot0Attr()),
        (joint.GetBody1Rel(), joint.GetLocalPos1Attr(), joint.GetLocalRot1Attr()),
    ):
        local = Gf.Matrix4d().SetRotate(Gf.Quatd(rotation.Get()))
        local.SetTranslateOnly(Gf.Vec3d(at.Get()))
        link = stage.GetPrimAtPath(body.GetTargets()[0])
        frames.append(np.array(local * UsdGeom.XformCache().GetLocalToWorldTransform(link)))
    np.testing.assert_allclose(frames[0], frames[1], atol=1e-6)


def test_the_legs_and_the_spindle_move(stage):
    from pxr import UsdPhysics

    moving = [
        prim.GetName()
        for prim in stage.Traverse()
        if prim.IsA(UsdPhysics.Joint) and not prim.IsA(UsdPhysics.FixedJoint)
    ]
    assert len(moving) == 13 and spot_weeder.SPINDLE in moving


def test_importing_the_scripts_does_not_load_pxr():
    """Kit ships its own `pxr` and dies during startup if the pip one was
    imported first."""
    # A subprocess, so the check is not fooled by another test's imports.
    subprocess.run(
        [
            sys.executable,
            "-c",
            "import demo, spot_weeder, sys, workspace; assert 'pxr' not in sys.modules",
        ],
        cwd=pathlib.Path(__file__).parent,
        check=True,
    )

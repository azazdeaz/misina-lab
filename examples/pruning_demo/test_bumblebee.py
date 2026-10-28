"""Tests for the robot.

`bumblebee` reaches Isaac Lab through `isaaclab.sim`, so this is skipped
entirely where that isn't installed. The generated URDF is the subject of
most of them: it is where the dimensions above end up. The Warthog is
downloaded on first use, so the first run needs the network.
"""

from __future__ import annotations

import dataclasses
import math
import pathlib
import xml.etree.ElementTree as ElementTree

import numpy as np
import pytest

bumblebee = pytest.importorskip("bumblebee", reason="Isaac Lab is not installed")

import kinematics  # noqa: E402
from shear import BLADE  # noqa: E402

MACHINE = bumblebee.Bumblebee()


def in_hand(corner, across, up) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """A rectangle in the shear's frame, in the hand's."""
    tool = bumblebee.TOOL
    return (tool @ [*corner, 1.0])[:3], tool[:3, :3] @ across, tool[:3, :3] @ up


@pytest.fixture
def built() -> ElementTree.Element:
    return ElementTree.fromstring(bumblebee.urdf(MACHINE))


def test_the_chain_is_the_urdfs_own_joints_in_order(built):
    """The solver poses the joints the importer builds, in the order it
    builds them, with the same origins, axes and limits."""
    written = {joint.get("name"): joint for joint in built.iter("joint")}
    moving = [name for name, joint in written.items() if joint.get("type") != "fixed"]

    assert moving == [
        *bumblebee.WHEEL_JOINTS,
        *(j.name for j in MACHINE.chain),
        bumblebee.SHEAR_JOINT,
    ]
    for joint in MACHINE.chain:
        element = written[joint.name]
        origin = element.find("origin")
        xyz = [float(v) for v in origin.get("xyz").split()]
        axis = [float(v) for v in element.find("axis").get("xyz").split()]
        limit = element.find("limit")
        # The arm's first joint is written off the carriage's top, which the
        # mount joint above it climbs.
        if joint.name == bumblebee.ARM_JOINTS[0]:
            xyz[2] += MACHINE.carriage
        assert xyz == pytest.approx(joint.xyz), joint.name
        assert axis == pytest.approx(joint.axis), joint.name
        assert (float(limit.get("lower")), float(limit.get("upper"))) == joint.limits, joint.name
        assert element.get("type") == ("prismatic" if joint.prismatic else "revolute")


def test_the_arm_stands_on_the_rail_and_folds_over_the_deck():
    """At home the mouth is over the deck's edge, not out over the row or in
    the ground, and the rail is where the slide joint says it is."""
    q = np.array([0.0, *bumblebee.ARM_HOME])
    mouth = (kinematics.frames(MACHINE.chain, q)[-1] @ MACHINE.tool)[:3, 3]

    front = max(x for x, _ in MACHINE.corners) + MACHINE.wheel_radius
    assert MACHINE.deck < mouth[2] < MACHINE.deck + 1.0
    assert abs(mouth[0]) < front and abs(mouth[1]) < MACHINE.track / 2 + 0.2
    assert MACHINE.chain[0].xyz == pytest.approx(MACHINE.rail)


def test_the_mouth_is_what_the_closing_blade_sweeps():
    """The tool frame's origin, where a cane is brought to, lies in the wedge
    between the edges, as far off the lines from the pivot to the hook's tip
    and to the open blade's as from the cutting blade's tip, and further
    off the hook's bowed edge: clear of the open blade, and covered by it on
    its way shut. The frame's +X is the pivot, which a cane lies along, and
    the blade's rectangle faces it."""
    shear = MACHINE.shear
    centre = MACHINE.tool[:3, 3]

    def reached(angle: float) -> bool:
        corner, across, up = in_hand(*shear.blade_plane(angle))
        offset = centre - corner
        on = np.array([[offset @ across / (across @ across), offset @ up / (up @ up)]])
        covered, _ = shear.covers(on, angle)
        return bool(((0 <= on) & (on <= 1)).all() and covered[0])

    assert not reached(shear.opening)
    assert any(reached(angle) for angle in np.linspace(0.0, shear.opening, 100))
    y, z = shear.mouth
    assert -shear.reach < y < 0 and 0 < z < shear.blade
    room = shear.blade - math.hypot(y, z)
    tip = shear.blade + shear.claw
    for chord in ([shear.edge(tip), tip], [-math.sin(shear.opening), math.cos(shear.opening)]):
        assert abs(np.cross(chord, [y, z])) / np.linalg.norm(chord) == pytest.approx(room)
    assert shear.clearance(y, z) > room
    assert MACHINE.tool[:3, 0] == pytest.approx([1.0, 0.0, 0.0])
    corner, across, up = in_hand(*shear.blade_plane(0.0))
    normal = np.cross(across, up)
    assert abs(normal @ MACHINE.tool[:3, 0]) == pytest.approx(np.linalg.norm(normal))


def test_the_head_collides(built):
    shear = MACHINE.shear
    assert any(
        size[0] == pytest.approx(shear.head)
        for _, size in _boxes(built.find("./link[@name='shear_link']"), "collision")
    )


@pytest.mark.parametrize(
    ("name", "x", "length"),
    [
        ("shear_link", 0.0, MACHINE.shear.blade + MACHINE.shear.claw),
        (BLADE, MACHINE.shear.bypass, MACHINE.shear.blade),
    ],
)
def test_the_blades_collide_where_they_are_drawn(built, name, x, length):
    """Each blade is drawn as one mesh, and collides as convex pieces end to
    end along it within where the mesh is, the edge of each on the chord of
    `Shear.edge` between its ends. Each piece is about its own middle, so
    its pose says where along the blade it is. The moving blade's pieces are
    the `Shear.slices` `Stroke` stands at a cane one by one, lying out from
    the pivot as `Shear.spans` says."""
    shear = MACHINE.shear
    link = built.find(f"./link[@name='{name}']")
    [(_, drawn)] = _meshes(link, "visual")
    low, high = drawn.min(axis=0), drawn.max(axis=0)
    assert (low[0], high[0]) == pytest.approx((x - shear.thickness / 2, x + shear.thickness / 2))
    assert (low[2], high[2]) == pytest.approx((0.0, length))
    ends = []
    for origin, corners in _meshes(link, "collision"):
        assert corners[:, 2].min() < 0 < corners[:, 2].max(), "about its own middle"
        corners = corners + origin
        assert ((low - 1e-5 <= corners) & (corners <= high + 1e-5)).all(), "where it is drawn"
        on_edge = corners[np.abs(corners[:, 1] - shear.edge(corners[:, 2])) < 1e-6]
        assert len(np.unique(on_edge[:, 2])) == 2, "its edge on the chord between its ends"
        ends.append(sorted(np.unique(np.hypot(on_edge[:, 1], on_edge[:, 2]))))
    # Read back from single precision.
    ends = np.array(sorted(ends))
    assert ends[0, 0] == pytest.approx(0.0, abs=1e-6)
    assert ends[1:, 0] == pytest.approx(ends[:-1, 1], abs=1e-6), "end to end"
    if name == BLADE:
        assert ends == pytest.approx(shear.spans, abs=1e-6)


def test_a_cane_in_the_mouth_is_pinched_where_the_edges_are_a_cane_apart():
    """`pinch` is the angle at which the moving edge, turned shut by it, is
    a radius from the cane's axis, and passes through it with no radius;
    `clearance` is how far a point is off the fixed edge; and `in_wedge`
    maps a crossing of `wedge` back to where in the shear's frame the cane
    is."""
    shear = MACHINE.shear
    out = np.linspace(0.0, shear.blade, 4001)
    edge = np.column_stack([shear.edge(out), out])

    def gap(angle: float, y: float, z: float) -> float:
        """How far the point (y, z) is from the moving edge at `angle`."""
        c, s = math.cos(angle), math.sin(angle)
        return float(np.hypot(*(edge @ np.array([[c, s], [-s, c]]) - [y, z]).T).min())

    y, z, radius = -0.005, 0.06, 0.005
    angle = shear.pinch(y, z, radius)
    assert 0 < angle < shear.opening
    assert gap(angle, y, z) == pytest.approx(radius, abs=2e-4)
    assert gap(shear.pinch(y, z, 0.0), y, z) == pytest.approx(0.0, abs=1e-4), "the axis itself"
    assert shear.clearance(shear.edge(0.05), 0.05) == pytest.approx(0.0, abs=1e-6)
    front, behind = shear.edge(0.05) - 0.003, shear.edge(0.05) + 0.003
    assert shear.clearance(front, 0.05) > 0 > shear.clearance(behind, 0.05), "the mouth's side"
    corner, across, up = in_hand(*shear.wedge())
    point = (bumblebee.TOOL @ [shear.bypass, *shear.mouth, 1.0])[:3]
    on = np.array(
        [[(point - corner) @ across / (across @ across), (point - corner) @ up / (up @ up)]]
    )
    assert np.cross(across, up) @ (point - corner) == pytest.approx(0.0), "in the plate's plane"
    assert np.ravel(shear.in_wedge(on)) == pytest.approx(shear.mouth)


def _meshes(link: ElementTree.Element, tag: str) -> list[tuple[np.ndarray, np.ndarray]]:
    """Each mesh under `link`'s `tag` elements, as its origin and the corners
    of its triangles about that origin, shape (n, 3)."""
    meshes = []
    for element in link.findall(tag):
        mesh = element.find("geometry/mesh")
        if mesh is not None:
            origin = element.find("origin")
            xyz = [0.0] * 3 if origin is None else [float(v) for v in origin.get("xyz").split()]
            triangles = np.fromfile(
                mesh.get("filename"),
                dtype=[("normal", "<f4", 3), ("corners", "<f4", (3, 3)), ("attribute", "<u2")],
                offset=84,
            )
            meshes.append((np.array(xyz), triangles["corners"].reshape(-1, 3).astype(float)))
    return meshes


def _boxes(link: ElementTree.Element, tag: str) -> list[tuple[list[float], list[float]]]:
    """Each box under `link`'s `tag` elements, as its centre and its size."""
    boxes = []
    for element in link.findall(tag):
        box = element.find("geometry/box")
        if box is not None:
            centre = [float(v) for v in element.find("origin").get("xyz").split()]
            boxes.append((centre, [float(v) for v in box.get("size").split()]))
    return boxes


def test_every_part_has_a_positive_size(built):
    sizes = [float(value) for box in built.iter("box") for value in box.get("size").split()] + [
        float(cylinder.get(attribute))
        for cylinder in built.iter("cylinder")
        for attribute in ("radius", "length")
    ]
    assert sizes and all(size > 0 for size in sizes)


def test_the_warthog_stands_on_its_wheels(built):
    """Each wheel turns about the machine's Y at its corner with its bottom
    on the ground under `base_link`, and every mesh the robot is drawn with
    is found."""
    for joint, (x, y) in zip(bumblebee.WHEEL_JOINTS, MACHINE.corners, strict=True):
        element = built.find(f"./joint[@name='{joint}']")
        centre = bumblebee._placement(built, element.find("child").get("link"))[:3, 3]
        assert element.find("axis").get("xyz") == "0 1 0"
        assert centre == pytest.approx([x, y, MACHINE.wheel_radius])
    assert all(pathlib.Path(mesh.get("filename")).is_file() for mesh in built.iter("mesh"))


@pytest.mark.parametrize("impossible", [{"travel": 3.0}, {"deck": 0.8}])
def test_a_machine_that_cannot_be_built_is_refused(impossible):
    with pytest.raises(ValueError):
        dataclasses.replace(MACHINE, **impossible)

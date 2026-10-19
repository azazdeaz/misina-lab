"""Tests for cutting a rod while it is simulated, and for drawing its
colliders.

Requires Isaac Lab; skipped entirely where it isn't installed. Runs a bare
Newton rod -- no Kit, no coupler -- stepped the way Isaac Lab steps it: the
queued model changes first, then the contacts, then the solver.
"""

from __future__ import annotations

import pytest

try:
    from misina_lab.isaaclab import colliders, cutting
except ImportError:
    pytest.skip("Isaac Lab is not installed", allow_module_level=True)

# Past the skip: `vinery-lab` itself needs only usd-core, so numpy comes with
# Isaac Lab or not at all.
import numpy as np  # noqa: E402
import warp as wp  # noqa: E402
from isaaclab_newton.physics import NewtonManager  # noqa: E402
from newton import BodyFlags, CollisionPipeline, ModelBuilder  # noqa: E402
from newton.solvers import SolverVBD  # noqa: E402
from pxr import Usd, UsdGeom, UsdShade  # noqa: E402

SEGMENT = 0.1
DT = 1e-3
GRAVITY = 9.81


class Rig:
    """A straight rod of four 10 cm segments along x, a meter up -- or up z
    from a meter, `upright` -- its first body held still the way the importer
    bolts a shoot to the wood. A ground, if `ground` is given, that many
    meters up. Its segments share the default group, so they collide, but for
    each pair a joint ties."""

    def __init__(self, monkeypatch, ground: float | None = None, upright: bool = False):
        builder = ModelBuilder(gravity=(0.0, 0.0, -GRAVITY))
        if ground is not None:
            builder.add_ground_plane(height=ground)
        along = wp.vec3(0.0, 0.0, 1.0) if upright else wp.vec3(1.0, 0.0, 0.0)
        positions = [wp.vec3(0.0, 0.0, 1.0) + along * (SEGMENT * i) for i in range(5)]
        self.bodies, self.joints = builder.add_rod(
            positions,
            radius=0.01,
            bend_stiffness=100.0,
            label="/rod",
            body_frame_origin="com",
        )
        builder.body_mass[self.bodies[0]] = 0.0
        builder.body_inertia[self.bodies[0]] = wp.mat33(0.0)
        builder.body_flags[self.bodies[0]] = int(BodyFlags.KINEMATIC)
        builder.color()
        self.model = builder.finalize()
        self.solver = SolverVBD(self.model, iterations=10)
        self.state, self.next = self.model.state(), self.model.state()
        self.control = self.model.control()
        self.collision = CollisionPipeline(self.model)
        self.contacts = self.collision.contacts()
        monkeypatch.setattr(NewtonManager, "_model", self.model)
        monkeypatch.setattr(NewtonManager, "_solver", self.solver)
        monkeypatch.setattr(NewtonManager, "_state_0", self.state)
        monkeypatch.setattr(NewtonManager, "_model_changes", set())
        self.shears = cutting.Shears(Usd.Stage.CreateInMemory())

    def step(self, seconds: float) -> None:
        for _ in range(round(seconds / DT)):
            for change in NewtonManager._model_changes:
                self.solver.notify_model_changed(change)
            NewtonManager._model_changes = set()
            self.collision.collide(self.state, self.contacts)
            self.solver.step(self.state, self.next, self.control, self.contacts, DT)
            self.state, self.next = self.next, self.state
            NewtonManager._state_0 = self.state

    def height(self) -> np.ndarray:
        return self.state.body_q.numpy()[self.bodies, 2]

    def spans(self) -> np.ndarray:
        """Each capsule's axis along x, as (start, end) per body."""
        return np.stack(self.shears.capsules(), axis=1)[:, :, 0]


@pytest.fixture
def rig(monkeypatch) -> Rig:
    return Rig(monkeypatch)


def plane_at(x: float) -> tuple:
    """A knife across the rod at `x`: a square meter in the yz plane."""
    return (x, -0.5, 0.5), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)


def test_a_cut_between_joints_moves_the_joint_to_the_cut(rig):
    """The segment the knife crosses ends at the knife, and the next one starts
    there; the mass follows the length, so the rod weighs what it did."""
    mass = rig.model.body_mass.numpy().sum()

    assert rig.shears.cut_through(*plane_at(0.14)) == 1

    spans = rig.spans()
    assert spans[1, 1] == pytest.approx(0.14, abs=1e-5)
    assert spans[2, 0] == pytest.approx(0.14, abs=1e-5)
    assert spans[2, 1] == pytest.approx(0.3, abs=1e-5)
    assert rig.model.body_mass.numpy().sum() == pytest.approx(mass)
    assert rig.shears.loose.tolist() == [False, False, True, True]
    # Only the knife's segment and the one after it were touched.
    assert spans[[0, 3]].ravel() == pytest.approx([0.0, 0.1, 0.3, 0.4], abs=1e-5)


def test_a_crossing_is_reported_where_it_is_and_a_collider_moved_where_it_is_put(rig):
    """What `cut_through` cuts at, without the cut; and a body's collision
    shapes -- a rod segment has one -- moved in the body's frame by the same
    in-place write."""
    hits, at, on = rig.shears.crossing(*plane_at(0.14))
    assert hits.tolist() == [1] and at == pytest.approx([0.4], abs=1e-5)
    assert on.ravel() == pytest.approx([0.5, 0.5], abs=1e-5)
    assert rig.shears.loose.tolist() == [False] * 4, "nothing cut"

    shapes, rest = rig.shears.colliders(rig.bodies[1])
    assert shapes.tolist() == [rig.shears._shape[1]]
    assert rest[0] == pytest.approx([0.0, 0.0, rig.shears._center[1], 0.0, 0.0, 0.0, 1.0])
    pose = np.array([0.0, 0.02, 0.0, 0.0, 0.0, 0.0, 1.0])
    rig.shears.place(shapes[0], pose)
    assert rig.model.shape_transform.numpy()[shapes[0]] == pytest.approx(pose)


def test_the_piece_falls_and_the_stub_stays(rig):
    before = rig.height()
    rig.shears.cut_through(*plane_at(0.14))
    rig.step(0.5)

    drop = before - rig.height()
    assert drop[2:] == pytest.approx(GRAVITY * 0.5**2 / 2, rel=0.02)
    assert abs(drop[1]) < 0.01
    # Cut free, the piece passes back through the knife without being cut again.
    assert rig.shears.cut_through(*plane_at(0.14)) == 0


@pytest.mark.parametrize(
    ("body", "at", "loose"),
    [
        (1, 0.05, [False, True, True, True]),  # a splinter from its root joint
        (1, 0.95, [False, False, True, True]),  # a splinter from its tip joint
        (3, 0.5, [False, False, False, True]),  # the last segment, which goes whole
        (3, 0.95, [False] * 4),  # a splinter off the rod's end, which is no cut
        (0, 0.0, [False, True, True, True]),  # the first body, which hangs from nothing
    ],
)
def test_a_cut_next_to_a_joint_is_made_at_it(rig, body, at, loose):
    """Nothing is resized: a body that small would have no mass to divide by."""
    scale = rig.model.shape_scale.numpy()
    assert rig.shears.cut(rig.bodies[body], at) == any(loose)

    assert rig.shears.loose.tolist() == loose
    assert (rig.model.shape_scale.numpy() == scale).all()


def test_a_cut_piece_lands_on_the_ground_and_lies_there(monkeypatch):
    """Nothing lays it down: it collides with the ground like any body."""
    rig = Rig(monkeypatch, ground=0.8)
    rig.shears.cut(rig.bodies[1], 0.5)
    rig.step(1.0)

    start, end = rig.shears.capsules()
    assert np.concatenate([start[2:, 2], end[2:, 2]]) == pytest.approx(0.8 + 0.01, abs=2e-3)
    assert np.abs(rig.state.body_qd.numpy()[rig.bodies[2:]]).max() < 0.01


def test_a_cut_piece_catches_on_the_rod_it_was_cut_from(monkeypatch):
    """A piece dropping straight down its own rod passes the shortened stub
    segment, whose pair the joint filters, and stops on the segment below it,
    cap on cap -- the way a piece catches on its vine instead of falling
    through it. Newton's rod-on-rod contact is what this pins."""
    rig = Rig(monkeypatch, upright=True)
    rig.shears.cut(rig.bodies[2], 0.5)
    rig.step(1.0)

    start, end = rig.shears.capsules()
    assert start[3, 2] == pytest.approx(end[1, 2] + 2 * 0.01, abs=3e-3)


def test_every_collider_is_drawn_where_newton_has_it(monkeypatch):
    """Each segment's capsule is drawn under the segment's prim, where it is
    in the segment's frame and as long, and the ground under the drawing's
    own scope; a cut redraws the two segments it reshaped once the drawing
    looks again, shown though the cut hid what hangs off the segment past
    it."""
    rig = Rig(monkeypatch, ground=0.0)
    stage = rig.shears._stage
    for label in rig.model.body_label:
        UsdGeom.Xform.Define(stage, label)
    view = colliders.ColliderView(stage)

    def drawn() -> list[tuple[float, float]]:
        """Each segment's drawn capsule, as its centre along the segment and
        its length between the round ends."""
        capsules = [
            next(iter(stage.GetPrimAtPath(rig.model.body_label[body]).GetChildren()))
            for body in rig.bodies
        ]
        return [
            (
                prim.GetAttribute("xformOp:translate").Get()[2],
                UsdGeom.Capsule(prim).GetHeightAttr().Get(),
            )
            for prim in capsules
        ]

    def newton() -> list[tuple[float, float]]:
        return list(zip(rig.shears._center, 2 * rig.shears._half, strict=True))

    assert view.missing == 0
    assert drawn() == pytest.approx(newton())
    capsule = stage.GetPrimAtPath(rig.model.body_label[rig.bodies[0]]).GetChildren()[0]
    assert UsdShade.MaterialBindingAPI(capsule).GetDirectBinding().GetMaterialPath() == (
        f"{colliders.ROOT}/Looks/rod"
    )
    [ground] = [
        prim for prim in stage.GetPrimAtPath(colliders.ROOT).GetChildren() if prim.IsA(UsdGeom.Mesh)
    ]
    assert UsdShade.MaterialBindingAPI(ground).GetDirectBinding().GetMaterialPath() == (
        f"{colliders.ROOT}/Looks/scene"
    )

    before = drawn()
    rig.shears.cut_through(*plane_at(0.14))
    assert drawn() == before, "as it was, until the drawing looks"
    view.update()
    assert drawn() == pytest.approx(newton()) and drawn() != pytest.approx(before)
    assert all(
        UsdGeom.Imageable(capsule).ComputeVisibility() == UsdGeom.Tokens.inherited
        for body in rig.bodies
        for capsule in stage.GetPrimAtPath(rig.model.body_label[body]).GetChildren()
    ), "every capsule shown"

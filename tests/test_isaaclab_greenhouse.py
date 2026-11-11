"""Tests for the physics a greenhouse runs under: its bent trusses.

Requires Isaac Lab, and the greenhouse's extension; skipped where either is
missing. What is pinned here is what the cfg cannot show: that Newton's rod
importer takes a bent truss as a rod held at the stem, carrying the weight of
its fruit, its every body where the prim riding it was authored, and that the
truss's skin, posed at those bodies, starts where it was built.
"""

from __future__ import annotations

import json
import re
from types import SimpleNamespace

import pytest

# Not `pytest.importorskip`: `greenhouse_lab.isaaclab` re-raises a missing
# Isaac Lab under its own message, and pytest skips only when the error names
# the module it was asked to import.
try:
    import greenhouse_lab
    from greenhouse_lab.isaaclab import GreenhouseCfg, StemCfg
except ImportError:
    pytest.skip("Isaac Lab or greenhouse_lab is not installed", allow_module_level=True)

import numpy as np  # noqa: E402
import warp as wp  # noqa: E402
from isaaclab_newton.physics import NewtonManager  # noqa: E402
from newton import ModelBuilder  # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom, UsdSkel, Vt  # noqa: E402

from misina_lab.isaaclab import cutting, rods, skins  # noqa: E402
from misina_lab.usd.build import ALONG, ANIMATION, SKELETON, build_usd  # noqa: E402

BELOW = 1.8
"""A height that bends the two lowest trusses on every stem at the defaults."""


def test_a_greenhouse_bends_trusses_only_below_its_height():
    """Its static fallback is the same greenhouse with nothing below the
    height."""
    bent = GreenhouseCfg(stem=StemCfg(flexible_below=BELOW))
    assert GreenhouseCfg().without_rods() is None
    assert bent.without_rods().stem.flexible_below == 0.0
    assert bent.stem.flexible_below == BELOW, "the caller's cfg is left as it was"


@pytest.fixture(scope="module")
def greenhouse(tmp_path_factory):
    """A short row whose lowest trusses bend, skinned: its document, the
    stage built from it, and Newton's builder holding that stage."""
    params = greenhouse_lab.GreenhouseParams()
    layout, stem = params.layout, params.stem
    (layout.rows, layout.row_length), stem.flexible_below, stem.skinned = (1, 2.0), BELOW, True
    params.layout, params.stem = layout, stem
    doc = json.loads(params.generate_scene_json())
    path = tmp_path_factory.mktemp("greenhouse") / "greenhouse.usda"
    build_usd(doc, str(path))
    builder = ModelBuilder()
    builder.add_usd(str(path))
    return doc, path, builder


def test_a_rods_bodies_start_where_their_prims_were_authored(greenhouse):
    """Every segment's body starts at the prim the importer drives with it,
    turned as that prim is, though every truss is turned about its stem: so
    what hangs off the prim -- a fruit, its collider -- stays where it was
    put. Left to frame the curve itself, the importer frames it in the world
    and turns each body about its segment off its prim, by as much as half a
    turn."""
    _, path, builder = greenhouse
    stage = Usd.Stage.Open(str(path))
    cache = UsdGeom.XformCache()
    checked = 0
    for body, label in enumerate(builder.body_label):
        if "_edge_body_" not in label:
            continue
        world = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(label))
        turn = world.ExtractRotationQuat()
        prim = np.array([*turn.GetImaginary(), turn.GetReal()])
        pose = np.array(builder.body_q[body], dtype=float)
        assert np.allclose(pose[:3], world.ExtractTranslation(), atol=1e-5), label
        assert abs(prim @ pose[3:]) > np.cos(1e-3 / 2), label
        checked += 1
    assert checked > 20


def test_a_bent_truss_is_a_rod_held_at_the_stem_carrying_its_fruit(greenhouse):
    """Its first body is static, the way the importer bolts a rod down, and the
    rest weigh the fruit's loads and a few grams of rachis on top."""
    doc, _, builder = greenhouse

    # Every curve's loads, by the name of the truss it stands in for.
    loads = {}

    def walk(node, at):
        if node.get("cable"):
            loads[at] = sum(node["cable"].get("loads", ()))
        for child in node.get("children", ()):
            walk(child, f"{at}/{child['name']}")

    walk(doc["root"], "")
    bodies = {}
    for body, label in enumerate(builder.body_label):
        found = re.search(r"/Rows/(.*)_edge_body_(\d+)$", label)
        if found:
            bodies.setdefault(found[1], []).append((int(found[2]), builder.body_mass[body]))
    loads = {re.search(r"/Rows/(.*)$", at)[1]: load for at, load in loads.items()}

    assert loads and bodies.keys() == loads.keys()
    for truss, chain in bodies.items():
        masses = [mass for _, mass in sorted(chain)]
        assert masses[0] == 0.0 and min(masses[1:]) > 0.0, truss
        assert 0.0 < sum(masses) - loads[truss] < 0.02, f"{truss}: {masses} for {loads[truss]} kg"

    # Every fruit's collider is imported static. Seated, one under a segment
    # rides that segment's body and one on a rigid truss stays where it is,
    # and a rod body's mass is as it was: the fruit weigh as the loads.
    fruit = {
        shape: label
        for shape, label in enumerate(builder.shape_label)
        if re.search(r"/Fruit_\d+/Collision$", label)
    }
    assert fruit and all(builder.shape_body[shape] == -1 for shape in fruit)
    was = list(builder.body_mass)
    riding = [shape for shape, label in fruit.items() if "_edge_body_" in label]
    assert rods.carry_riders(builder) == len(riding) > 0
    assert list(builder.body_mass) == was
    for shape, label in fruit.items():
        body = builder.shape_body[shape]
        if shape in riding:
            assert builder.body_label[body] == label.rsplit("/", 2)[0], label
        else:
            assert body == -1, label


def skin_roots(stage: Usd.Stage) -> list[Usd.Prim]:
    return [prim for prim in stage.Traverse() if prim.IsA(UsdSkel.Root)]


def drawn(root: Usd.Prim, pose) -> np.ndarray:
    """Where USD's own skinning puts the points of the skin under `root`, in
    its skeleton's space, with each joint at `pose` of its segment's path:
    the world pose of the body that segment's prim is driven by."""
    organ = str(root.GetParent().GetPath())
    _, segments, frame, size = skins.rig(root.GetStage(), organ)
    local = skins.seen_from(frame, np.array([pose(each) for each in segments]), size)
    animation = UsdSkel.Animation(root.GetChild(ANIMATION))
    animation.GetTranslationsAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*p) for p in local[:, :3]]))
    animation.GetRotationsAttr().Set(
        Vt.QuatfArray([Gf.Quatf(w, x, y, z) for x, y, z, w in local[:, 3:]])
    )
    cache = UsdSkel.Cache()
    cache.Populate(UsdSkel.Root(root), Usd.PrimDefaultPredicate)
    (binding,) = cache.ComputeSkelBindings(UsdSkel.Root(root), Usd.PrimDefaultPredicate)
    joints = cache.GetSkelQuery(binding.GetSkeleton()).ComputeSkinningTransforms(
        Usd.TimeCode.Default()
    )
    (target,) = binding.GetSkinningTargets()
    skinned = Vt.Vec3fArray(UsdGeom.Mesh(target.GetPrim()).GetPointsAttr().Get())
    assert target.ComputeSkinnedPoints(joints, skinned, Usd.TimeCode.Default())
    return np.array(skinned)


def skin_mesh(root: Usd.Prim) -> Usd.Prim:
    return next(prim for prim in Usd.PrimRange(root) if prim.IsA(UsdGeom.Mesh))


def test_a_skeleton_is_framed_at_a_uniform_scale_only():
    """A turned skeleton at one scale is framed with it; one turned under an
    uneven scale is not, even where its axes come out as long as each other."""
    turn = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(1, 1, 1), Gf.Vec3d(1, 0, 0)))
    assert skins.frame_of(turn * Gf.Matrix4d().SetScale(2.0))[1] == pytest.approx(2.0)
    sheared = turn * Gf.Matrix4d().SetScale(Gf.Vec3d(2, 1, 1))
    assert [sheared.GetRow3(axis).GetLength() for axis in range(3)] == pytest.approx([2**0.5] * 3)
    assert skins.frame_of(sheared)[1] is None


@pytest.mark.parametrize("scale", [1.0, 2.0])
def test_a_skin_posed_at_its_bodies_starts_where_it_was_built(greenhouse, tmp_path, scale):
    """Each joint of a truss's skin posed at its segment's body, where Newton
    starts it, leaves the skin where it was built, on whatever rides the
    segments: the skin's binds are the frames the bodies are built in. In a
    scene spawned scaled, the scale is divided out of each joint's position."""
    _, path, builder = greenhouse
    if scale != 1.0:
        # A copy apart from the fixture's layer, which the other tests read.
        layer = Sdf.Layer.OpenAsAnonymous(str(path))
        copy = Usd.Stage.Open(layer)
        UsdGeom.Xformable(copy.GetDefaultPrim()).AddScaleOp().Set(Gf.Vec3f(scale, scale, scale))
        path = tmp_path / "scaled.usda"
        layer.Export(str(path))
        builder = ModelBuilder()
        builder.add_usd(str(path))
    stage = Usd.Stage.Open(str(path))
    index = {label: body for body, label in enumerate(builder.body_label)}
    body_q = np.array(builder.body_q, dtype=float)
    worst, posed = 0.0, 0
    for root in skin_roots(stage):
        built = np.array(UsdGeom.Mesh(skin_mesh(root)).GetPointsAttr().Get())
        now = drawn(root, lambda segment: body_q[index[segment]])
        worst = max(worst, float(np.abs(now - built).max()))
        posed += 1
    assert posed > 5 and worst < 1e-4, f"{posed} skins, {worst} m off"


@pytest.fixture
def shears(greenhouse, monkeypatch):
    """`Shears` over the greenhouse's model, as the running simulation has
    them, on a copy of its stage, with the model's state to set poses in."""
    _, path, builder = greenhouse
    model = builder.finalize()
    state = model.state()
    monkeypatch.setattr(NewtonManager, "backend", SimpleNamespace(model=model, state_0=state))
    monkeypatch.setattr(NewtonManager, "_solver", None)
    monkeypatch.setattr(NewtonManager, "_model_changes", set())
    stage = Usd.Stage.Open(Sdf.Layer.OpenAsAnonymous(str(path)))
    return cutting.Shears(stage), stage, state


@pytest.mark.parametrize(
    ("at", "cut"),
    [(0.5, 2.5), (0.05, 2.0), (0.95, 3.0)],
    ids=["along a segment", "at the joint before", "at the joint after"],
)
def test_a_cut_splits_a_skin_where_it_splits_the_rod_and_leaves_it_as_drawn(shears, at, cut):
    """Past a cut on the third segment of a bent truss, made `cut` segments
    along the rod, the skin's points follow only the bodies past it and the
    rest only those before it. No face spans the cut: the two parts end on
    it, where the stub's capsule ends, each closed by a cap facing the other,
    of points of its own whose normals face that way too. And but for those
    ends, moved onto the cut, every point is drawn where it was."""
    shears, stage, state = shears
    root = skin_roots(stage)[0]
    mesh = skin_mesh(root)
    organ = str(root.GetParent().GetPath())
    bodies = [shears.labels.index(each) for each in skins.rig(stage, organ)[1]]
    # Each body turned at the joint it hangs from, and with the bodies before
    # it: bent, as no cut finds a rod at rest.
    body_q = state.body_q.numpy()
    joints, bend = shears.capsules()[0], wp.transform_identity()
    for body in bodies[1:]:
        joint = wp.vec3(*joints[list(shears.bodies).index(body)])
        turn = wp.transform(joint, wp.quat_from_axis_angle(wp.vec3(1.0, 0.0, 0.0), 0.08))
        bend = bend * turn * wp.transform(-joint, wp.quat_identity())
        body_q[body] = np.array(bend * wp.transform(*body_q[body]))
    state.body_q.assign(body_q)

    def read():
        along = np.array(UsdGeom.PrimvarsAPI(mesh).GetPrimvar(ALONG).Get())
        faces = np.array(UsdGeom.Mesh(mesh).GetFaceVertexIndicesAttr().Get()).reshape(-1, 3)
        return along, faces, drawn(root, lambda segment: body_q[shears.labels.index(segment)])

    along, faces, was = read()
    assert shears.cut(bodies[2], at)
    moved, kept, now = read()
    # The points past the first `count` are the caps'.
    count = len(along)
    ends, before = moved[:count] != along, along < cut
    stub = int(np.ceil(cut)) - 1
    bound = np.array(UsdSkel.BindingAPI(mesh).GetJointIndicesPrimvar().Get()).reshape(-1, 3)
    assert bound[:count][before].max() == stub and bound[:count][~before].min() == stub + 1
    side = bound.max(axis=1) <= stub
    assert (side[kept].all(axis=1) | ~side[kept].any(axis=1)).all()
    assert np.abs(now[:count] - was)[~ends].max() < 1e-5, "a point off the cut moved"

    # The two parts end together, where the stub's capsule now does.
    world = np.array(UsdGeom.XformCache().GetLocalToWorldTransform(root.GetChild(SKELETON)))
    end = shears.capsules()[1][list(shears.bodies).index(bodies[stub])]
    for part in (ends & before, ends & ~before):
        assert part.sum() > 2
        middle = np.append(now[:count][part].mean(axis=0), 1.0) @ world
        assert np.linalg.norm(middle[:3] - end) < 2e-3

    # A cap is the faces between its own points only, and faces the cut, as
    # their normals, at rest, face out of it.
    caps = kept[(kept >= count).all(axis=1)]
    gap = np.linalg.norm(now[count:, None] - now[:count][ends], axis=2).min(axis=1)
    assert gap.max() < 1e-6, "a cap's point is not drawn on an end's"
    corners = now[caps]
    out = np.cross(corners[:, 1] - corners[:, 0], corners[:, 2] - corners[:, 0])
    onward = now[~side].mean(axis=0) - now[side].mean(axis=0)
    assert len(caps) > 2 and ((out @ onward > 0) == side[caps[:, 0]]).all()
    points = np.array(UsdGeom.Mesh(mesh).GetPointsAttr().Get())
    extent = np.array(UsdGeom.Mesh(mesh).GetExtentAttr().Get())
    assert np.allclose(extent, [points.min(axis=0), points.max(axis=0)])
    rest = points[caps]
    normals = np.array(UsdGeom.Mesh(mesh).GetNormalsAttr().Get())[caps]
    for cap in (side[caps[:, 0]], ~side[caps[:, 0]]):
        face = np.cross(rest[cap, 1] - rest[cap, 0], rest[cap, 2] - rest[cap, 0]).sum(axis=0)
        assert (normals[cap] @ face / np.linalg.norm(face) > 0.999).all()

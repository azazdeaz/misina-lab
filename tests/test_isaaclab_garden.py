"""Tests for the physics a garden runs under: its pole beans' vines.

Requires Isaac Lab, and the garden's extension; skipped where either is
missing. What is pinned here is what the cfg cannot show: that Newton's rod
importer takes a flexible vine as a rod held at the soil, carrying the weight
of its leaves and pods, its every body where the prim riding it was authored.
"""

from __future__ import annotations

import json
import re

import pytest

# Not `pytest.importorskip`: `garden_lab.isaaclab` re-raises a missing Isaac
# Lab under its own message, and pytest skips only when the error names the
# module it was asked to import.
try:
    import garden_lab
    from garden_lab.isaaclab import BedCfg, GardenCfg, PhysicsCfg, PoleBeanCfg
except ImportError:
    pytest.skip("Isaac Lab or garden_lab is not installed", allow_module_level=True)

import numpy as np  # noqa: E402
from newton import CollisionPipeline, JointType, ModelBuilder  # noqa: E402
from newton.solvers import SolverVBD  # noqa: E402
from pxr import Usd, UsdGeom  # noqa: E402

from misina_lab.isaaclab import rods  # noqa: E402
from misina_lab.isaaclab.spawn import to_params  # noqa: E402
from misina_lab.usd.build import build_usd  # noqa: E402

DT = 1.0 / 800.0


def cables(doc) -> int:
    """How many rods the document authors."""
    count = 0
    nodes = [doc["root"]]
    while nodes:
        node = nodes.pop()
        count += bool(node.get("cable"))
        nodes.extend(node.get("children", ()))
    return count


def test_a_garden_bends_vines_only_with_flexible_stems_and_a_climber():
    """Its static fallback is the same garden with the stems rigid, and a
    garden whose one climber is left out, confined to a bed it has not got or
    not yet sown authors no rod however its stems are; a bed of them does."""
    flexible = PhysicsCfg(flexible_stems=True)
    bent = GardenCfg(physics=flexible)
    assert GardenCfg().without_rods() is None
    assert bent.without_rods().physics.flexible_stems is False
    assert bent.physics.flexible_stems, "the caller's cfg is left as it was"
    for bean in (PoleBeanCfg(share=0.0), PoleBeanCfg(bed=7), PoleBeanCfg(sown_week=9.0)):
        assert GardenCfg(physics=flexible, pole_bean=bean).without_rods() is None, bean
    one_bed = GardenCfg(physics=flexible, bed=BedCfg(columns=1, rows=1))
    assert one_bed.without_rods() is not None
    assert cables(json.loads(to_params(one_bed).generate_scene_json())) > 0


def beans(
    segment: float = 0.05, fruit_collides: bool = True, leaves_collide: bool = False
) -> garden_lab.GardenParams:
    """One square of pole beans grown to harvest, their vines flexible."""
    params = garden_lab.GardenParams()
    bed, physics, planting, scene = params.bed, params.physics, params.planting, params.scene
    bed.columns, bed.rows, bed.length, bed.width = 1, 1, 0.5, 0.5
    physics.flexible_stems, physics.segment = True, segment
    physics.fruit_collides, physics.leaves_collide = fruit_collides, leaves_collide
    planting.misses, planting.jitter = 0.0, 0.0
    scene.week = 10.0
    params.bed, params.physics, params.planting, params.scene = bed, physics, planting, scene
    for name in ("lettuce", "kale", "carrot", "bush_bean", "zucchini"):
        crop = getattr(params, name)
        crop.share = 0.0
        setattr(params, name, crop)
    pole_bean = params.pole_bean
    pole_bean.share = 1.0
    params.pole_bean = pole_bean
    return params


@pytest.fixture(scope="module")
def garden(tmp_path_factory):
    """One square of pole beans grown to harvest, their vines flexible and
    their pods colliding: its document, the stage built from it, and Newton's
    builder holding that stage."""
    doc = json.loads(beans().generate_scene_json())
    path = tmp_path_factory.mktemp("garden") / "garden.usda"
    build_usd(doc, str(path))
    builder = ModelBuilder()
    builder.add_usd(str(path))
    return doc, path, builder


def test_a_vines_bodies_start_where_their_prims_were_authored(garden):
    """Every segment's body starts at the prim the importer drives with it,
    turned as that prim is, though every vine winds round its string: so what
    hangs off the prim, a leaf, a pod, its collider, stays where it was put."""
    _, path, builder = garden
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
    assert checked > 100


def test_a_vine_is_a_rod_held_at_the_soil_carrying_its_leaves_and_pods(garden):
    """Its first body is static, the way the importer bolts a rod down, and
    the rest weigh the leaves' and pods' loads and a few tens of grams of vine on top;
    its string stays a static shape beside it."""
    doc, _, builder = garden

    # Every curve's loads, by the name of the rod it stands in for.
    loads = {}

    def walk(node, at):
        if node.get("cable"):
            loads[at] = sum(node["cable"].get("loads", ()))
        for child in node.get("children", ()):
            walk(child, f"{at}/{child['name']}")

    walk(doc["root"], "")
    bodies = {}
    for body, label in enumerate(builder.body_label):
        found = re.search(r"/Beds/(.*)_edge_body_(\d+)$", label)
        if found:
            bodies.setdefault(found[1], []).append((int(found[2]), builder.body_mass[body]))
    loads = {re.search(r"/Beds/(.*)$", at)[1]: load for at, load in loads.items()}

    assert len(loads) == 4 and bodies.keys() == loads.keys()
    for vine, chain in bodies.items():
        masses = [mass for _, mass in sorted(chain)]
        assert masses[0] == 0.0 and min(masses[1:]) > 0.0, vine
        assert 0.0 < sum(masses) - loads[vine] < 0.1, f"{vine}: {masses} for {loads[vine]} kg"

    # Every pod's collider is imported static and rides a segment once seated,
    # with the rod bodies' masses as they were: the pods weigh as the loads.
    # The string's capsule hangs under no segment, so it stays where it is.
    pods = {
        shape: label
        for shape, label in enumerate(builder.shape_label)
        if re.search(r"/Fruit_\d+/Collision$", label)
    }
    strings = [shape for shape, label in enumerate(builder.shape_label) if "/String/" in label]
    assert pods and strings and all(builder.shape_body[shape] == -1 for shape in pods)
    was = list(builder.body_mass)
    assert rods.carry_riders(builder) == len(pods)
    assert list(builder.body_mass) == was
    for shape, label in pods.items():
        assert builder.body_label[builder.shape_body[shape]] == label.rsplit("/", 2)[0], label
    assert all(builder.shape_body[shape] == -1 for shape in strings)


def tuned(path) -> tuple[ModelBuilder, list[int], int]:
    """The stage's builder as `rods.tune_rods` leaves it, outside Isaac Lab,
    with the rod bodies in order and how many wood pairs the tuning let
    through."""
    builder = ModelBuilder()
    builder.add_usd(str(path))
    let = rods.tune_builder(builder, GardenCfg.PLANT)
    bodies = {
        end
        for joint, kind in enumerate(builder.joint_type)
        if kind == JointType.ROD
        for end in (builder.joint_parent[joint], builder.joint_child[joint])
    }
    builder.color()
    return builder, sorted(bodies), let


@pytest.mark.parametrize("segment", [0.02, 0.04, 0.06])
def test_a_vine_clears_its_string_at_every_segment_length(tmp_path, segment):
    """A rod's segments run straight between their points, so the longer they
    are the deeper they cut inside the vine's turns; up to the longest the
    scene allows, none is authored into the string, which the tuning would
    let it through for good. Without pod colliders, nothing else can be."""
    path = tmp_path / "vines.usda"
    build_usd(json.loads(beans(segment, fruit_collides=False).generate_scene_json()), str(path))
    assert tuned(path)[2] == 0


def settled(builder: ModelBuilder, bodies: list[int], seconds: float):
    """The rod bodies' positions as authored, half a second before the end
    of `seconds` under the solver that steps rods, and at the end."""
    model = builder.finalize()
    rods.reserve_contact_slots(model)
    solver = SolverVBD(model, iterations=10, rigid_compliant_alm=False)
    state, nxt = model.state(), model.state()
    control, collision = model.control(), CollisionPipeline(model)
    contacts = collision.contacts()
    rest = state.body_q.numpy()[bodies, :3]
    before = rest
    for step in range(round(seconds / DT)):
        collision.collide(state, contacts)
        solver.step(state, nxt, control, contacts, DT)
        state, nxt = nxt, state
        if step == round((seconds - 0.5) / DT):
            before = state.body_q.numpy()[bodies, :3]
    return rest, before, state.body_q.numpy()[bodies, :3]


def test_a_vine_holds_on_its_string_under_vbd(garden):
    """Settled for four seconds under the solver that steps rods, a vine
    loaded with its leaves and pods hangs on its string by its turns: no body
    leaves where it was authored by more than a few centimetres, and it comes
    to rest."""
    _, path, _ = garden
    builder, bodies, _ = tuned(path)
    rest, before, final = settled(builder, bodies, 4.0)
    assert np.isfinite(final).all()
    moved = np.linalg.norm(final - rest, axis=1)
    assert moved.max() < 0.06, f"a body moved {moved.max():.3f} m"
    speed = np.linalg.norm(final - before, axis=1).max() / 0.5
    assert speed < 0.01, f"still moving at {speed:.3f} m/s"


def test_a_vine_settles_with_its_leaves_boxes_riding_it(tmp_path):
    """With the leaves colliding, every leaf's box is a `Cube` the importer
    takes as a box, seated on the segment its leaf rides, and a vine alone in
    its bed, its boxes clear of its string and of the trellis, settles as it
    does without them: no body leaves where it was authored by more than a
    few centimetres. Neighbouring plants' leaves authored through each other
    push apart, which is why this vine is alone."""
    from newton import GeoType

    params = beans(fruit_collides=False, leaves_collide=True)
    bed, pole_bean = params.bed, params.pole_bean
    bed.length, pole_bean.spacing, pole_bean.height = 1.2, 0.9, 1.5
    params.bed, params.pole_bean = bed, pole_bean
    path = tmp_path / "leafy.usda"
    build_usd(json.loads(params.generate_scene_json()), str(path))
    builder, bodies, _ = tuned(path)
    boxes = [
        shape
        for shape, label in enumerate(builder.shape_label)
        if re.search(r"/Leaf_\d+/Collision$", label)
    ]
    assert (
        len({label.rsplit("/", 1)[0] for label in builder.body_label if "_edge_body_" in label})
        == 1
    )
    assert boxes and all(builder.shape_type[shape] == GeoType.BOX for shape in boxes)
    assert all(builder.shape_body[shape] >= 0 for shape in boxes), "a box left unseated"
    rest, _, final = settled(builder, bodies, 2.0)
    assert np.isfinite(final).all()
    moved = np.linalg.norm(final - rest, axis=1)
    assert moved.max() < 0.06, f"a body moved {moved.max():.3f} m"


def test_a_rod_shape_is_let_through_its_plants_box_by_the_sphere_round_it():
    """A static box of a rod's own plant is let through when the sphere round
    it reaches the rod's shape, whatever the box's extents; one out of reach
    is not, nor another plant's."""
    import warp as wp
    from newton import ModelBuilder

    builder = ModelBuilder()
    own = ModelBuilder.ShapeConfig(collision_group=rods.PLANT_GROUPS)
    other = ModelBuilder.ShapeConfig(collision_group=rods.PLANT_GROUPS + 1)
    # A segment up Z, its capsule reaching 2.9 cm, and three thin boxes whose
    # spheres reach 4.4 cm: one just past the tip, one 30 cm off, and one of
    # another plant under the foot.
    body = builder.add_body(xform=wp.transform((0.0, 0.0, 0.0), wp.quat_identity()))
    builder.add_shape_capsule(body, radius=0.004, half_height=0.025, cfg=own)
    for z, x, cfg in ((0.06, 0.0, own), (0.0, 0.3, own), (-0.06, 0.0, other)):
        xform = wp.transform((x, 0.0, z), wp.quat_identity())
        builder.add_shape_box(-1, xform=xform, hx=0.03, hy=0.03, hz=0.01, cfg=cfg)
    assert rods.let_rods_out_of_their_wood(builder) == 1
    assert list(builder.shape_collision_filter_pairs) == [(0, 1)]

"""Tests for the physics a vineyard runs under.

Requires Isaac Lab; skipped entirely where it isn't installed. What is pinned
here is the agreement with two things this module cannot see: the prim name
the Rust generator gives a flexible organ (`CABLE` in `crates/misina-lab/src/scene/mod.rs`), and
the body labels Newton's rod importer derives from it -- and, for the spawner,
that the scene it spawns is one the backend in force can step.
"""

from __future__ import annotations

import re
import types

import pytest

# Not `pytest.importorskip`: `vinery_lab.isaaclab` re-raises a missing Isaac Lab
# under its own message, and pytest skips only when the error names the module
# it was asked to import.
try:
    from misina_lab.isaaclab import rods, spawn
    from vinery_lab.isaaclab import ShootCfg, VineyardCfg, physics, vineyard
except ImportError:
    pytest.skip("Isaac Lab is not installed", allow_module_level=True)

from isaaclab.physics import PhysicsEvent  # noqa: E402
from isaaclab.sim import SimulationContext  # noqa: E402
from isaaclab_newton.physics import (  # noqa: E402
    MJWarpSolverCfg,
    NewtonCfg,
    NewtonManager,
    VBDSolverCfg,
)
from isaaclab_physx.physics import PhysxCfg  # noqa: E402
from newton import GeoType, JointType, ShapeFlags  # noqa: E402

VINEYARD = "/World/Vineyard"
ROBOT = "/World/envs/env_0/Robot"
FLEXIBLE = VineyardCfg(shoot=ShootCfg(stray=0.05))
"""A vineyard with flexible shoots, as far as a cfg can promise any."""


@pytest.fixture
def cfg() -> object:
    return physics.make_physics_cfg_newton(FLEXIBLE, ROBOT, [f"{ROBOT}/.*FOOT"])


@pytest.fixture
def entries(cfg) -> dict:
    return {entry.name: entry for entry in cfg.solver_cfg.entries}


def test_the_cable_selector_matches_the_bodies_newton_builds(entries):
    """A rod's capsules have no prim of their own: the importer labels them
    after the curve's path, and this regex is the only handle on them. Selecting
    nothing is an error at build time, so this fails loudly -- but only once a
    simulator is running, which is late."""
    (selector,) = entries["rods"].bodies
    label = f"{VINEYARD}/Planting/Row_00/Vine_007/Shoot_03_1/{physics.CABLE}_edge_body_5"

    # The coupler wraps a selector this way before matching it in full.
    assert re.fullmatch(f"(?:{selector})(?:/.*)?", label)
    # The curve prim itself is not a body, and the vineyard's meshes are not rods.
    assert not re.fullmatch(selector, label.removesuffix("_edge_body_5"))
    # No vineyard path in the selector: the label alone keeps the robot out.
    assert not re.fullmatch(f"(?:{selector})(?:/.*)?", f"{ROBOT}/LF_FOOT")


def test_the_robot_owns_the_static_scene_and_the_shoots_see_it(entries):
    """A static shape belongs to one entry at most: on the wrong one the robot
    walks through the terrain. An entry that lists any shape stops seeing the
    static ones it does not own, so the shoots list none, not even their own
    -- or a cut piece falls through the ground."""
    assert entries["rigid"].include_static_shapes
    assert not entries["rods"].include_static_shapes
    assert not entries["rods"].include_body_shapes
    assert not entries["rods"].shape_label_patterns


def test_only_the_named_robot_bodies_can_bend_a_shoot(cfg):
    """The proxy mapping is the whole of the coupling: a body outside it passes
    through a cane without touching it."""
    (proxy,) = cfg.solver_cfg.proxies
    assert (proxy.source, proxy.destination) == ("rigid", "rods")
    assert proxy.bodies == [f"{ROBOT}/.*FOOT"]


@pytest.mark.parametrize(
    "shoot", [ShootCfg(), ShootCfg(stray=0.05, flexible=False)], ids=["held", "static strays"]
)
def test_a_vineyard_without_flexible_shoots_runs_on_mjwarp_alone(shoot):
    """A scene with no rod in it has nothing for a second entry to own -- the
    coupler refuses an entry that matches no body. The robot still wants its
    substeps: `NewtonCfg` defaults to one, where the quadruped falls over."""
    cfg = physics.make_physics_cfg_newton(VineyardCfg(shoot=shoot), ROBOT, [ROBOT])
    assert isinstance(cfg.solver_cfg, MJWarpSolverCfg)
    assert cfg.num_substeps == physics.SUBSTEPS


@pytest.mark.parametrize(
    ("backend", "steps"),
    [
        (NewtonCfg(), False),
        (PhysxCfg(), False),
        (NewtonCfg(solver_cfg=VBDSolverCfg()), True),
        (physics.make_physics_cfg_newton(FLEXIBLE, ROBOT, [ROBOT]), True),
    ],
    ids=["mjwarp", "physx", "vbd", "coupled"],
)
def test_only_a_vbd_solver_steps_a_rod(backend, steps):
    assert physics.steps_rods(backend) is steps


@pytest.fixture
def running(monkeypatch) -> types.SimpleNamespace:
    """A simulation running on the backend the test sets -- none while it is
    unset -- and a record of whether the rods were tuned for it."""
    state = types.SimpleNamespace(physics=None, tuned=False)
    monkeypatch.setattr(
        SimulationContext,
        "instance",
        lambda: types.SimpleNamespace(cfg=state) if state.physics is not None else None,
    )
    monkeypatch.setattr(spawn, "tune_rods", lambda plant: setattr(state, "tuned", True))
    return state


@pytest.mark.parametrize(
    ("backend", "flexible", "tuned"),
    [
        (None, True, False),
        (NewtonCfg(), False, False),
        (PhysxCfg(), False, False),
        (NewtonCfg(solver_cfg=VBDSolverCfg()), True, True),
    ],
    ids=["no sim", "mjwarp", "physx", "vbd"],
)
def test_the_spawner_keeps_the_scene_to_what_the_backend_can_step(
    running, backend, flexible, tuned
):
    """The physics config built for a vineyard is provisional -- a `--physics`
    override or a preset can replace it before the scene is built -- and the
    spawner is the one place that sees both. MJWarp cannot build a model that
    holds a rod, so under a backend with no rod solver the strays go static;
    under one with, they are tuned."""
    running.physics = backend
    spawned = vineyard.for_backend(FLEXIBLE)
    assert spawned.shoot.flexible is flexible
    assert running.tuned is tuned
    assert FLEXIBLE.shoot.flexible, "the caller's cfg is left as it was"


def test_a_vineyard_with_nothing_to_bend_is_spawned_as_it_is(running):
    running.physics = NewtonCfg(solver_cfg=VBDSolverCfg())
    held = VineyardCfg()
    assert vineyard.for_backend(held) is held
    assert not running.tuned


VINE_0 = f"{VINEYARD}/Planting/Row_00/{physics.VINE}_000"


@pytest.fixture
def builder(monkeypatch) -> object:
    """A model builder holding one rod joint between two of something else.

    Four slots per rod, starting at the joint's own DoF: a walk that reads the
    start index or the slot count wrong writes into a neighbouring joint
    instead, and nothing about that fails loudly.

    Bodies 0 and 1 are the rod's, carrying shapes 0 and 1; bodies 2 and 3 stand
    for the robot, carrying shapes 2 and 3; shapes 4 to 8 are the static scene,
    which is body -1 because a static shape has no body: the ground, a post,
    the trunk of the rod's vine, the trunk of another, and a collider authored
    under the rod's second segment, 20 cm past the segment's body along x. The
    other two joints are there to be walked past.

    Every shape is a capsule 2 cm across and 10 cm long but the ground, a
    plane, and the rider, a sphere. The trunk of the rod's vine stands on
    the origin, where the rod's first segment is held inside it; the second
    segment, a metre off, is clear of it.
    """
    import warp as wp

    builder = types.SimpleNamespace(
        joint_type=[JointType.D6, JointType.ROD, JointType.REVOLUTE],
        joint_qd_start=[0, 6, 10],
        joint_target_ke=[1.0] * 6 + [200.0, 100.0, 4.0, 2.0] + [1.0],
        joint_target_kd=[0.0] * 11,
        joint_parent=[3, 0, 2],
        joint_child=[2, 1, 3],
        body_count=4,
        body_label=[
            f"{VINE_0}/Shoot_00_0/{physics.CABLE}_edge_body_0",
            f"{VINE_0}/Shoot_00_0/{physics.CABLE}_edge_body_1",
            f"{ROBOT}/base",
            f"{ROBOT}/arm",
        ],
        body_shapes={-1: [4, 5, 6, 7, 8], 0: [0], 1: [1], 2: [2], 3: [3]},
        # The rod's second body a metre up, turned a quarter turn about z.
        body_q=[wp.transform_identity()]
        + [
            wp.transform(
                (1.0, 0.0, 1.0), wp.quat_from_axis_angle(wp.vec3(0.0, 0.0, 1.0), 1.5707964)
            )
        ]
        + [wp.transform_identity()] * 2,
        shape_label=[
            f"{VINE_0}/Shoot_00_0/{physics.CABLE}_edge_capsule_0",
            f"{VINE_0}/Shoot_00_0/{physics.CABLE}_edge_capsule_1",
            f"{ROBOT}/base/collision",
            f"{ROBOT}/arm/collision",
            f"{VINEYARD}/Terrain/Geom",
            f"{VINEYARD}/Planting/Row_00/Pole_000/Collision",
            f"{VINE_0}/Collision",
            f"{VINEYARD}/Planting/Row_00/{physics.VINE}_001/Collision",
            f"{VINE_0}/Shoot_00_0/{physics.CABLE}_edge_body_1/Leaf_03/Collision",
        ],
        shape_body=[0, 1, 2, 3, -1, -1, -1, -1, -1],
        shape_transform=[wp.transform_identity()] * 8
        + [wp.transform((1.2, 0.0, 1.0), wp.quat_identity())],
        shape_flags=[int(ShapeFlags.COLLIDE_SHAPES)] * 9,
        shape_type=[GeoType.CAPSULE] * 4
        + [GeoType.PLANE]
        + [GeoType.CAPSULE] * 3
        + [GeoType.SPHERE],
        shape_scale=[(0.01, 0.05, 0.0)] * 9,
        shape_margin=[0.0] * 9,
        shape_collision_group=[1] * 9,
        shape_collision_filter_pairs=[],
    )
    builder.add_shape_collision_filter_pair = lambda a, b: (
        builder.shape_collision_filter_pairs.append((min(a, b), max(a, b)))
    )
    # The simulation the builder is fetched from, by its physics config.
    sim = types.SimpleNamespace(
        cfg=types.SimpleNamespace(physics=NewtonCfg()),
        get_or_create_backend=lambda _cfg: builder,
    )
    monkeypatch.setattr(SimulationContext, "instance", lambda: sim)
    # `tune_shoots` registers a callback per call, and nothing here deregisters.
    NewtonManager.clear_callbacks()
    return builder


def test_a_rods_linear_slots_are_softened_and_its_angular_ones_kept(builder):
    """Stretch and shear are what the solver converges worst on. Bend and
    twist are what a cane stands up with, at the stiffness its taper gave
    each joint."""
    physics.tune_shoots(damping=0.0, stretch=0.5)
    NewtonManager.dispatch_event(PhysicsEvent.MODEL_INIT)

    assert builder.joint_target_ke == pytest.approx([1.0] * 6 + [100.0, 50.0, 4.0, 2.0] + [1.0])


def test_only_a_rods_angular_slots_take_damping_from_their_own_stiffness(builder):
    """A fraction of each slot's own stiffness is a time constant, so bend and
    twist settle alike. Stretch and shear are left undamped: VBD adds damping
    to a slot's stiffness every sweep, and there it keeps the canes swinging."""
    physics.tune_shoots(damping=0.1, stretch=1.0)
    NewtonManager.dispatch_event(PhysicsEvent.MODEL_INIT)

    assert builder.joint_target_kd == pytest.approx([0.0] * 8 + [0.4, 0.2] + [0.0])


def test_tune_shoots_registers_once_while_its_registration_stands(builder):
    """The spawner and a script may both call it. A second callback would
    multiply the joints over again, and nothing would report it."""
    first = physics.tune_shoots(damping=0.0, stretch=0.5)
    assert physics.tune_shoots(damping=0.0, stretch=0.5) is first
    NewtonManager.dispatch_event(PhysicsEvent.MODEL_INIT)
    assert builder.joint_target_ke[6] == pytest.approx(100.0)

    # Dropped by the manager when the simulation stops; the next call stands anew.
    NewtonManager.clear_callbacks()
    assert physics.tune_shoots() is not first


def test_a_vine_and_its_wood_share_a_group_and_the_rest_meets_every_rod(builder):
    """Two vines get two groups, so their segments never pair. The ground and
    the post are negative, so every rod lands on them. The robot's bodies are
    negative and distinct, so they pair with each other as the default group
    had them, and with every rod. A walk that reached the robot's bodies --
    following the wrong joints, or a rod joint's own DoF index instead of its
    bodies -- would put them in a vine's group, and nothing would report it."""
    physics.tune_shoots()
    NewtonManager.dispatch_event(PhysicsEvent.MODEL_INIT)

    vine, other, scene = physics.VINE_GROUPS, physics.VINE_GROUPS + 1, physics.SCENE_GROUP
    groups = builder.shape_collision_group
    assert groups[:2] == [vine, vine]
    assert groups[4:8] == [scene, scene, vine, other]
    assert all(group < 0 and group != scene for group in groups[2:4])
    assert groups[2] != groups[3]


def test_a_collider_under_a_segment_rides_the_segments_body_and_passes_its_rod(builder):
    """The importer left it static where it was authored. Seated, it sits on
    the segment's body in that body's frame, takes the vine's group with the
    rod, and is filtered against the rod's own capsules, which it was
    authored to clear by little."""
    physics.tune_shoots()
    NewtonManager.dispatch_event(PhysicsEvent.MODEL_INIT)

    assert builder.shape_body[8] == 1
    assert builder.body_shapes[1] == [1, 8] and 8 not in builder.body_shapes[-1]
    # 20 cm along x in the world is 20 cm along -y to a body turned a quarter
    # turn about z.
    assert list(builder.shape_transform[8])[:3] == pytest.approx([0.0, -0.2, 0.0], abs=1e-6)
    # And the held segment, authored inside its vine's trunk at the origin,
    # passes through that.
    assert set(builder.shape_collision_filter_pairs) == {(0, 8), (1, 8), (0, 6)}
    assert builder.shape_collision_group[8] == physics.VINE_GROUPS


@pytest.mark.parametrize("margin", [0.0, 0.003])
def test_a_rod_passes_through_its_own_wood_only_where_it_starts_inside_it(builder, margin):
    """The vine's trunk lies level across the top of the rod's first segment,
    5 mm clear of it. Grown by a 3 mm margin each, the two overlap, and a pair
    authored overlapping would shove the rod off its rest shape at the first
    step, so the segment is let through the trunk; clear, it still meets it.
    The other vine's trunk and the post overlap the segment too, but the one
    never pairs with it and the other is the scene's; the riders' pairs are
    `carry_riders`'."""
    import warp as wp

    across = wp.quat_from_axis_angle(wp.vec3(1.0, 0.0, 0.0), 1.5707964)
    builder.shape_transform[6] = wp.transform((0.0, 0.0, 0.075), across)
    builder.shape_margin[0] = builder.shape_margin[6] = margin
    physics.tune_shoots()
    NewtonManager.dispatch_event(PhysicsEvent.MODEL_INIT)

    let = {(0, 6)} if margin else set()
    assert set(builder.shape_collision_filter_pairs) == {(0, 8), (1, 8)} | let


@pytest.mark.parametrize(
    ("p", "q", "s", "t", "near"),
    [
        ((0, 0, 0), (0, 0, 1), (1, 0, 0), (1, 0, 1), 1.0),  # side by side
        ((-1, 0, 0), (1, 0, 0), (0, -1, 0.2), (0, 1, 0.2), 0.2),  # crossing
        ((0, 0, 0), (0, 0, 1), (0, 0, 1.5), (0, 0, 3), 0.5),  # end to end
        ((0, 0, 0), (1, 0, 0), (2, 1, 0), (2, 2, 1), 2**0.5),  # skew, end to end
        ((0.5, 0.3, 0), (0.5, 0.3, 0), (0, 0, 0), (1, 0, 0), 0.3),  # a point
        ((0, 0, 0), (0, 0, 0), (0, 3, 4), (0, 3, 4), 5.0),  # two points
    ],
)
def test_the_least_distance_between_two_segments(p, q, s, t, near):
    """Wherever the nearest pair of points lies: inside both segments, at an
    end of one or of both, or on a segment shrunk to a point."""
    import numpy as np

    got = rods._distances(
        np.array(p, float), np.array(q, float), np.array([s], float), np.array([t], float)
    )
    assert got[0] == pytest.approx(near, abs=1e-12)


def test_the_groups_pair_what_they_are_meant_to():
    """Pinned against Newton's own test rather than restated, because the
    convention is a bare integer sign with nothing to make a change in it
    fail: were two positive groups to start meeting, the scene would still run
    and quietly cost N(N-1)/2 candidate pairs again."""
    from newton import ModelBuilder

    collides = ModelBuilder()._test_group_pair
    vine, other, scene = physics.VINE_GROUPS, physics.VINE_GROUPS + 1, physics.SCENE_GROUP
    base, arm = -2, -3  # two of the robot's bodies

    assert collides(vine, vine) and not collides(vine, other)
    assert collides(vine, scene) and collides(vine, base) and collides(base, scene)
    assert collides(base, arm) and not collides(base, base)

"""Newton physics for a generated scene's rods.

Nothing in a generated scene is a rigid body except a flexible organ, imported
as a **rod**: one capsule rigid body per segment, joined by spring joints and
clamped to the wood it grew from. Only Newton's VBD solver steps one, and a
robot needs MuJoCo, so a scene with any runs the two side by side as named
entries of a coupled solver with the robot's own bodies handed across as
proxies. A scene without runs plain MJWarp, which cannot build a model that
holds a rod at all.

`make_physics_cfg_newton` picks between the two from the scene's cfg. The
choice is provisional -- Isaac Lab's `--physics` override, a Hydra preset or a
plain edit can each replace it before the scene is built -- so
`spawn.spawn_generated` checks the cfg against the backend in force when it
runs: the rods are spawned static where nothing steps one, and tuned where
something does.

Everything a caller cannot know -- what the cable prims are called, what the
rod bodies end up labelled, which entry has to own the ground -- is decided
here. What a caller *does* know, the robot and the parts of it a rod may touch,
are the arguments. The numbers below were measured on rods a metre or so long,
tapering from a centimetre or so across to half that, and hold for a rod of
that build.
"""

from __future__ import annotations

import re
from collections.abc import Sequence
from typing import TYPE_CHECKING, Literal

import numpy as np
from isaaclab.physics import PhysicsCfg, PhysicsEvent
from isaaclab_contrib.coupling import (
    CouplerEntryCfg,
    CouplerProxyCfg,
    CouplerProxyMappingCfg,
)
from isaaclab_newton.physics import (
    MJWarpSolverCfg,
    NewtonCfg,
    NewtonCollisionPipelineCfg,
    VBDSolverCfg,
)

if TYPE_CHECKING:
    from isaaclab.physics.physics_manager import CallbackHandle

    from .spawn import GeneratedSceneCfg

CABLE = "Cable"
"""The prim name every flexible organ's curve takes.

The generator picks it -- `CABLE` in `crates/misina-lab/src/scene/mod.rs` --
and it survives into the body labels below, so the two have to be changed
together."""

_ROD_BODY_SUFFIX = r"_edge_body_\d+"
"""What Newton's rod importer appends to the curve's path to label each body it
builds, and what `spawn_rod` names the prim it authors for that body: the one
Isaac Lab moves with the body, and everything under it with it."""

ROD_SUBSTEPS = 1
"""Substeps the rods take inside each physics substep.

One. More settles a rod worse, not better: VBD adds a joint's damping to its
stiffness as `kd / dt` every sweep, so the shorter the step the more that term
swamps the rest and the less ten sweeps converge. At two, a rod shoved at the
robot's cruising speed takes several times as long to come within a centimetre
of rest, and some end centimetres off where they started.
"""


ROD_STRETCH = 0.01
"""What a rod joint's stretch and shear stiffness is multiplied by.

Not a preference. VBD solves a rod chain by Gauss-Seidel sweeps, one body at a
time against its neighbours, and a joint far stiffer than a segment's inertia
over one step (m / dt^2) is what those converge worst on. The cable material
makes stretch and shear several hundred times that, and ten sweeps spent on
them leave the bend barely felt: at the authored values a 1.4 m rod folds
over under its own weight and never comes to rest. Compliant ALM converges it
no better. A hundredth brings them to a few times the inertia, which holds a
1.4 m rod to about a millimetre of stretch. A tenth already leaves it sagging
half again as far and never quite still; a thousandth lets it stretch a
centimetre.

Bend and twist keep the material's stiffness. Each joint's comes from the
diameter at its point, so a rod tapered as its organ is drawn stands up on
its thick base: a 1.4 m rod sags about 9 cm out of its drawn pose and swings
at about 1.2 Hz.
"""

ROD_DAMPING = 0.1
"""Damping a rod joint's bend and twist take as a fraction of their own
stiffness, in seconds.

Newton builds a rod joint undamped unless its material authors a damping, and
that is one value for the whole curve, while a tapered rod's joints differ in
stiffness twentyfold from base to tip. Damped here in proportion to its own
stiffness instead, every joint settles alike. This much is a heavily damped
swing, the way a leafy stem moves in air: pushed aside and let go, a rod
swings back past its rest pose once, by about a sixth of the push, and is
within a centimetre of rest in about a second.

Bend and twist only. VBD adds damping to a slot's stiffness as `kd / dt` every
sweep, so on stretch and shear it would make the slots that already converge
worst stiffer still; damped there, rods fold over and pump themselves into a
swing that never stops.
"""

BODY_CONTACTS = 1024
"""How many contacts the rods' solver keeps for each body, where Newton's
default is 64.

VBD files every contact under each body it touches, drops a body's contacts
past this many and prints a warning every substep it does. A rod segment
touches a handful. A robot body handed to the rods as a proxy touches every
rod shape against it at once, up to six points each against a box: a crate
holding nineteen cut pieces of rod reaches about 500, a shear closed on a rod
about 180. Each slot costs four bytes per body, and a body's solve visits
only the contacts it has -- on Newton's four threads, unless
`widen_contact_threads` gives it `CONTACT_THREADS`."""

CONTACT_THREADS = 32
"""How many GPU threads VBD solves one body's contacts with, where Newton
uses 4; `widen_contact_threads` sets it.

A body's contacts are shared out among its threads, and every other body of
its colour waits for the busiest, so a crate holding ~480 contacts on four
threads made a whole run 47% slower than the 64-contact cap had. 32 brings
the busiest body's share back to what 64 contacts on four threads was
(480 / 32 = 15, against 16), and the run to 2.6% over it. Any count is
correct: the kernels use it only as `tid // n`, `tid % n`, a strided loop
`i += n` and a launch size `rows * n` behind a body guard. The 2.6% left is
the five atomic adds each thread makes at the end of the per-body contact
kernel, whether it had a contact or not."""

PLANT_GROUPS = 2
"""The first collision group a plant takes; each plant gets the next one up.

Not a preference. Newton appends one candidate pair per colliding shape pair
while `Model.finalize` builds `shape_contact_pairs`, and sizes every broad- and
narrow-phase buffer from that list, so N rod segments that may all meet cost
N(N-1)/2 pairs before a step is taken: ten thousand segments is 55 million
pairs and 4 GiB. Filtering happens as the list is built, not inside a kernel,
so a group that declines a pair never allocates it. That is the explicit
broad phase, the default; the NxN and SAP ones `make_physics_cfg_newton` can
pick instead read the groups in a kernel every step, with a pair buffer sized
for every pair of shapes.

A group is one signed integer per shape. A positive group meets itself and
every negative one; a negative group meets everything but its own; zero meets
nothing. That is the whole algebra, and it is enough: a plant's rods and its
wood share a positive group, so a cut piece catches on its own plant's stub,
its other rods and its wood -- but a wood collider a segment of it was
authored inside, which `let_rods_out_of_their_wood` lets it through for good
-- and segments of different plants never pair. The
ground, the trellis and the robot's bodies are negative, so every rod meets
them. The robot takes a group per body, which pairs its parts exactly as the
default group did.

Measured on a row of twelve plants, each a trunk and two arms of wood, with 953
rod segments among them: 62,058 contact pairs against 19,364 with no rod
meeting a rod, at the same step time. Every rod meeting every other would be
454,000.
"""

SCENE_GROUP = -1
"""Collision group of the ground and of every static shape that is not a
plant's wood: negative, so every rod and the robot meet it, at one pair per rod
segment each. A post costs that much, a wire would too. A large scene, 9,800
rod segments among some eighty posts, pays 900,000 pairs and 8% of its step
time for them."""


_tuning: CallbackHandle | None = None
"""The registration `tune_rods` made, while it stands."""

_slots: ContactSlots | None = None
"""The rows `reserve_contact_slots` added to the model built last."""


class ContactSlots:
    """A spare row of `Model.shape_contact_pairs` for every rod joint.

    Newton lists every pair of shapes that may ever touch when the model is
    finalized, leaves out the two either side of each joint, and its broad
    phase reads that list, and only that list, for the life of the model.
    When a cut turns a rod joint off, the two segments either side of it are
    the two faces of the cut, with no row between them, and the piece would
    fall through the stub. So the list gets one more row per rod joint while
    it can still grow -- the model finalized, nothing yet holding a pointer
    into it -- parked on a pair of static shapes, which the broad phase never
    tests, and `Shears` writes the joint's own pair into the row when the
    joint goes.

    A coupled solver's entry clones the list and compacts it to the shapes it
    sees, keeping the order, so the rows are the last `count` of every copy.

    Only the explicit broad phase reads the list. NxN and SAP read the
    collision groups and the joints' filter pairs every step, so under them
    `tune_rods` reserves no rows (`reads_contact_pairs`), the two faces of a
    cut pass through each other, and a cut regroups the piece instead; see
    `make_physics_cfg_newton`.
    """

    def __init__(self, model):
        import warp as wp
        from newton import JointType

        self.model = model
        joints = np.flatnonzero(model.joint_type.numpy() == int(JointType.ROD))
        self.row = {int(joint): k for k, joint in enumerate(joints)}
        """Each rod joint's row, counted from the first of the rows."""
        self.count = len(joints)
        self.parked = _parking(model)
        """The pair every row holds until its joint goes."""
        pairs = model.shape_contact_pairs.numpy().reshape(-1, 2)
        extended = np.concatenate([pairs, np.tile(self.parked, (self.count, 1))])
        model.shape_contact_pairs = wp.array(
            extended.astype(np.int32), dtype=wp.vec2i, device=model.device
        )
        model.shape_contact_pair_count = len(extended)


def _parking(model) -> np.ndarray:
    """Two static shapes that collide, in the list's (low, high) order, the
    ground left out.

    Every broad phase drops a pair of two static shapes before testing it,
    wherever they are, so a row parked on one is never a contact. They have
    to collide: a coupled solver's entry compacts its copy of the list to
    the shapes it collides, and a row naming another would drop out of it.
    """
    from newton import GeoType, ShapeFlags

    static = (
        (model.shape_body.numpy() < 0)
        & (model.shape_flags.numpy() & int(ShapeFlags.COLLIDE_SHAPES) != 0)
        & ~np.isin(model.shape_type.numpy(), [int(GeoType.PLANE), int(GeoType.HFIELD)])
    )
    shapes = np.flatnonzero(static)[:2]
    if len(shapes) < 2:
        raise ValueError(
            "no two static shapes to park a contact row on: a scene with rods needs two"
        )
    return shapes.astype(np.int32)


def reserve_contact_slots(model) -> ContactSlots:
    """Add the rows to `model`, just finalized, and keep them for `Shears`.

    Before anything takes the model's contact-pair list -- a collision
    pipeline, a coupled solver's entries -- since the list is replaced by a
    longer one. `tune_rods` does this on `PHYSICS_READY`, which Isaac Lab
    fires in that window, under a broad phase that reads the rows
    (`reads_contact_pairs`); a model built by hand calls it itself.
    """
    global _slots
    _slots = ContactSlots(model)
    return _slots


def contact_slots(model) -> ContactSlots | None:
    """The rows reserved on `model`, or None where none were: under a broad
    phase that would not read them, or a model built by hand without
    `reserve_contact_slots`. `Shears` then opens no row at a cut."""
    if _slots is None or _slots.model is not model:
        return None
    return _slots


def tune_rods(
    plant: str,
    damping: float = ROD_DAMPING,
    stretch: float = ROD_STRETCH,
    lone_damping: float | None = None,
) -> CallbackHandle:
    """Soften every rod joint's stretch and shear, damp its bend and twist, and
    group what collides.

    `plant` is the prim name every plant takes, its index appended -- `Plant`
    for `Plant_007`, the cfg's `PLANT`. A rod's body label and a wood
    collider's shape label both pass through the plant's prim, which is what
    puts a plant's rods and its wood in one collision group. `lone_damping`
    damps a rod under no plant's prim in place of `damping`, as the cfg's
    `LONE_ROD_DAMPING` asks.

    `spawn.spawn_generated` calls this when it spawns rods under a solver that
    steps them, so a script calls it only to change the numbers: from inside the
    running app and **before the first `SimulationContext.reset()`**. That is
    what builds the model, and both the stiffnesses and the collision groups
    are read out of the builder when it is finalized and never looked at
    again. The only window to write them is the builder's -- after the
    importer has filled it, before it is finalized -- and Isaac Lab dispatches
    `MODEL_INIT` in exactly that window.

    First seats every collider authored under a segment's prim on the
    segment's body, with `carry_riders`, so it takes the plant's group with
    the rod, and once the groups are set lets each rod through the wood it
    is authored inside, with `let_rods_out_of_their_wood`. Also reserves the
    `ContactSlots` rows on `PHYSICS_READY` when the broad phase in force
    reads them, the explicit one (`reads_contact_pairs`), so a cut can make
    the two faces of a cut meet.

    Registers once. While the registration stands -- the manager drops it when
    the simulation stops -- a further call returns it unchanged, rather than
    multiplying the joints a second time.

    Without the tuning a rod folds over under its own weight and never comes
    to rest; see `ROD_STRETCH` and `ROD_DAMPING`. Without the grouping the
    scene pays N(N-1)/2 candidate pairs for collisions it never solves; see
    `PLANT_GROUPS`.

    A rod meets its own plant's rods and wood (all but the wood colliders it
    is authored inside), the ground, the posts and the robot, and passes
    through every other plant. A rod under no plant's prim is a plant of its
    own: it meets the ground, the posts and the robot, and passes through
    every plant and every other rod.
    """
    global _tuning
    # Imported here and not at module scope: `newton` brings `pxr` with it, and
    # Kit's own `pxr` wins the import only if nothing loaded the pip one first.
    from isaaclab.sim import SimulationContext
    from isaaclab_newton.physics import NewtonBuilderCfg, NewtonManager

    if _tuning is not None and _tuning.id in NewtonManager._callbacks:
        return _tuning

    def tune(_payload) -> None:
        sim = SimulationContext.instance()
        builder = sim.get_or_create_backend(NewtonBuilderCfg(physics_cfg=sim.cfg.physics))
        tune_builder(builder, plant, damping, stretch, lone_damping)

    def reserve(_payload) -> None:
        # With no simulation running, as when a test dispatches the event by
        # hand, Newton's default broad phase, the explicit one, is assumed.
        sim = SimulationContext.instance()
        if reads_contact_pairs(sim.cfg.physics if sim is not None else None):
            reserve_contact_slots(NewtonManager.get_model())

    _tuning = NewtonManager.register_callback(tune, PhysicsEvent.MODEL_INIT)
    NewtonManager.register_callback(reserve, PhysicsEvent.PHYSICS_READY)
    return _tuning


def tune_builder(
    builder,
    plant: str,
    damping: float = ROD_DAMPING,
    stretch: float = ROD_STRETCH,
    lone_damping: float | None = None,
) -> int:
    """What `tune_rods` does to the builder once the importer has filled it:
    softens every rod joint's stretch and shear by `stretch`, damps its bend
    and twist by `damping` times their stiffness (`lone_damping` for a rod
    under no plant's prim, when given), seats the riders, groups each plant's
    rods and wood under its own collision group, a rod under no plant as a
    plant of its own and every other shape under no plant into the scene's,
    and lets each rod out of the wood it is authored inside. Returns how many
    pairs that let through.

    For a builder outside Isaac Lab, a test or a rig: `tune_rods` is the same
    call inside the app's `MODEL_INIT` window.
    """
    from newton import JointType

    # Matches a label up to and including its plant prim.
    plant_path = re.compile(rf"(.*/{re.escape(plant)}_[^/]*)/")
    rods: set[int] = set()
    for joint, kind in enumerate(builder.joint_type):
        if kind != JointType.ROD:
            continue
        # Both ends, so a chain's first body is counted too: it is the
        # first rod joint's parent and no rod joint's child.
        rods.update((builder.joint_parent[joint], builder.joint_child[joint]))
        # A rod's four slots, in the builder's order: stretch, shear, bend,
        # twist.
        dof = builder.joint_qd_start[joint]
        for slot in (dof, dof + 1):
            builder.joint_target_ke[slot] *= stretch
        lone = not plant_path.match(builder.body_label[builder.joint_child[joint]])
        fraction = lone_damping if lone and lone_damping is not None else damping
        for slot in (dof + 2, dof + 3):
            builder.joint_target_kd[slot] = fraction * builder.joint_target_ke[slot]

    carry_riders(builder)

    # Groups: a plant's for its rods and wood, allotted as they turn up,
    # a rod under no plant counting as a plant of its own; `SCENE_GROUP`
    # for any other shape under no plant; one of its own for every other
    # body. Group 0 is a site, which collides with nothing, and stays.
    plants: dict[str, int] = {}

    def plant_group(label: str, alone: str | None = None) -> int:
        owner = plant_path.match(label)
        key = owner.group(1) if owner is not None else alone
        if key is None:
            return SCENE_GROUP
        return plants.setdefault(key, PLANT_GROUPS + len(plants))

    def regroup(shapes: Sequence[int], group: int) -> None:
        for shape in shapes:
            if builder.shape_collision_group[shape] != 0:
                builder.shape_collision_group[shape] = group

    # `body_shapes` is keyed by body index, and a static shape has none,
    # so -1 is the whole of the static scene.
    for body in range(builder.body_count):
        label = builder.body_label[body]
        if body in rods:
            # Keyed alone by its curve, the label short of the suffix.
            group = plant_group(label, alone=re.sub(rf"{_ROD_BODY_SUFFIX}$", "", label))
        else:
            group = -(2 + body)
        regroup(builder.body_shapes.get(body, ()), group)
    for shape in builder.body_shapes[-1]:
        regroup((shape,), plant_group(builder.shape_label[shape]))

    return let_rods_out_of_their_wood(builder)


def carry_riders(builder) -> int:
    """Seat every collider authored under a rod segment's prim on the body that
    drives the prim, in that body's frame, and let it pass through the rest of
    its own rod. Returns how many were seated.

    The importer reads a collision prim as a static shape unless a rigid-body
    prim stands over it, and a segment's prim is not one: the rod importer
    builds the body and labels it with the prim's path, which is all that
    ties the two. So a collider under one -- round something hanging off the
    rod -- would stand where it was authored while the prim moved on with the
    body. Seated on the body it moves with the segment and falls with a cut
    piece.

    Its own rod lets it through. It was authored to clear the rod's rest
    shape by little, and a pair resting that close to a neighbouring segment
    would shove the rod about; what it meets is the rest of the scene.

    Call it on a builder the importer has filled and nothing has finalized.
    `tune_rods` does, before it sets the collision groups, so a seated shape
    takes its plant's group with the body it sits on.
    """
    import warp as wp
    from newton import ShapeFlags

    def collides(shape: int) -> bool:
        return bool(builder.shape_flags[shape] & ShapeFlags.COLLIDE_SHAPES)

    body_at = {label: body for body, label in enumerate(builder.body_label)}
    rods: dict[str, list[int]] = {}
    for label, body in body_at.items():
        if found := re.fullmatch(rf"(.*){_ROD_BODY_SUFFIX}", label):
            rods.setdefault(found[1], []).append(body)

    seated = 0
    # The importer's visual shapes stay where they are: nothing steps them.
    for shape in [each for each in builder.body_shapes[-1] if collides(each)]:
        found = re.match(rf"((.*){_ROD_BODY_SUFFIX})/", builder.shape_label[shape])
        if found is None or (body := body_at.get(found[1])) is None:
            continue
        for other in [each for rod in rods[found[2]] for each in builder.body_shapes[rod]]:
            if collides(other):
                builder.add_shape_collision_filter_pair(shape, other)
        # A static shape's transform is in the world; a body's is in its own.
        world = builder.shape_transform[shape]
        builder.shape_transform[shape] = wp.transform_inverse(builder.body_q[body]) * world
        builder.shape_body[shape] = body
        builder.body_shapes[-1].remove(shape)
        builder.body_shapes[body].append(shape)
        seated += 1
    return seated


def let_rods_out_of_their_wood(builder) -> int:
    """Let every shape on a rod's body, a segment's own or a rider seated on
    it, through each static shape of its own plant that it overlaps as
    authored, its wood or its support. Returns how many pairs were let
    through.

    A rod is held where it leaves the wood it grows from, and a wood collider
    is a proxy, straighter and often fatter than the wood it stands for, so
    the segments next to the held one can start inside it. A pair authored
    overlapping would shove the rod off its rest shape the moment the
    simulation starts, at the joint a flush cut is made at.

    A pair let through stays so for as long as the model stands, a piece cut
    free included: such a segment passes through the whole of that shape,
    however long. Every other pair of the rod's shapes and its wood still
    meets; what rides the rod passes its own rod by `carry_riders`.

    Capsules and spheres meet by their axes; any other shape, a box, by the
    sphere round it, so a pair let through on its account need not touch.
    Call it on a builder whose collision groups are set, as `tune_rods` does:
    a plant's group is what says whose wood a rod's shape may be let through.
    """
    from newton import GeoType, ShapeFlags

    def own(shape: int) -> bool:
        return (
            bool(builder.shape_flags[shape] & ShapeFlags.COLLIDE_SHAPES)
            and builder.shape_collision_group[shape] >= PLANT_GROUPS
        )

    def axis(shape: int) -> tuple[np.ndarray, np.ndarray, float]:
        """The shape's axis, its two ends in the world, and its radius out to
        where it touches: a capsule's and a sphere's own, any other's the
        sphere round it."""
        pose = builder.shape_transform[shape]
        if (body := builder.shape_body[shape]) >= 0:
            pose = builder.body_q[body] * pose
        pose = np.array(pose, dtype=float)
        x, y, z, w = pose[3:]
        up = np.array([2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y)])
        scale = np.array(builder.shape_scale[shape], dtype=float)
        kind = builder.shape_type[shape]
        reach = up * (scale[1] if kind == GeoType.CAPSULE else 0.0)
        round_ = kind in (GeoType.CAPSULE, GeoType.SPHERE)
        radius = scale[0] if round_ else float(np.linalg.norm(scale))
        return pose[:3] - reach, pose[:3] + reach, radius + builder.shape_margin[shape]

    woods: dict[int, list[int]] = {}
    for shape in filter(own, builder.body_shapes[-1]):
        woods.setdefault(builder.shape_collision_group[shape], []).append(shape)
    ends = {shape: axis(shape) for group in woods.values() for shape in group}
    let = 0
    for body, shapes in builder.body_shapes.items():
        for shape in shapes if body >= 0 else ():
            wood = woods.get(builder.shape_collision_group[shape]) if own(shape) else None
            if not wood:
                continue
            start, end, radius = axis(shape)
            starts, stops, radii = (np.array(column) for column in zip(*(ends[w] for w in wood)))
            gaps = _distances(start, end, starts, stops) - radius - radii
            for other in np.array(wood)[gaps < 0.0]:
                builder.add_shape_collision_filter_pair(shape, int(other))
                let += 1
    return let


def _distances(p: np.ndarray, q: np.ndarray, s: np.ndarray, t: np.ndarray) -> np.ndarray:
    """The least distance from the segment `p`-`q` to each segment
    `s[i]`-`t[i]`: the closest points of the two lines, each clamped to its
    segment in turn (Ericson, *Real-Time Collision Detection*, 5.1.9)."""
    d1, d2, r = q - p, t - s, p - s
    a, e = d1 @ d1, np.einsum("ij,ij->i", d2, d2)
    b, c, f = d2 @ d1, r @ d1, np.einsum("ij,ij->i", d2, r)
    den = a * e - b * b
    u = np.clip(np.divide(b * f - c * e, den, out=np.zeros_like(den), where=den > 1e-12), 0, 1)
    v = np.clip((b * u + f) / np.maximum(e, 1e-12), 0, 1)
    u = np.clip((b * v - c) / max(a, 1e-12), 0, 1)
    return np.linalg.norm(r + np.outer(u, d1) - d2 * v[:, None], axis=1)


def steps_rods(physics: PhysicsCfg | None) -> bool:
    """Whether a physics config steps a rod: it has a VBD solver, on its own or
    as an entry of a coupled one. PhysX ignores a rod's curve schema, and
    MJWarp cannot build a model that holds one."""
    solver = getattr(physics, "solver_cfg", None)
    entries = getattr(solver, "entries", None)
    solvers = [entry.solver_cfg for entry in entries] if entries is not None else [solver]
    return any(isinstance(each, VBDSolverCfg) for each in solvers)


def reads_contact_pairs(physics: PhysicsCfg | None) -> bool:
    """Whether the broad phase that finds the rods' contacts under `physics`
    is the explicit one, which reads the model's contact-pair list and nothing
    else: the rows `ContactSlots` adds are for it alone. That is the pipeline
    of each proxy mapping into a VBD entry of a coupled solver
    (`make_physics_cfg_newton`), or `collision_cfg` under a VBD solver on its
    own. Either left unset is Newton's default, explicit, and so is a pipeline
    given as a callable, which names no broad phase."""
    solver = getattr(physics, "solver_cfg", None)
    entries = getattr(solver, "entries", None)
    if entries is None:
        pipelines = [getattr(physics, "collision_cfg", None)]
    else:
        rods = {entry.name for entry in entries if isinstance(entry.solver_cfg, VBDSolverCfg)}
        pipelines = [m.collision_pipeline for m in solver.proxies if m.destination in rods]
    return not pipelines or any(
        getattr(pipeline, "broad_phase", "explicit") == "explicit" for pipeline in pipelines
    )


def widen_contact_threads() -> None:
    """Solve each body's contacts with `CONTACT_THREADS` threads in VBD.

    Call it once Newton can be imported -- under Isaac Lab, after the app is
    launched -- and before the simulation is stepped. Warp reads the thread
    count into a kernel when it first builds the kernel's module, while the
    launch sizes read it at every launch, so a call after the build would
    leave the kernels on four threads and the launches sized for 32. It
    asserts that no such module is built yet.
    """
    # TODO: drop once Newton takes the body-body contact thread count as a
    # SolverVBD parameter; through 1.8.0rc1 it is the module constant
    # `_NUM_CONTACT_THREADS_PER_BODY` read at codegen, while the body-particle
    # path took a runtime count there.
    import warp as wp
    from newton._src.solvers.vbd import rigid_vbd_kernels, solver_vbd, vbd_coupling_kernels

    # Each holds its own binding of the constant.
    modules = (rigid_vbd_kernels, solver_vbd, vbd_coupling_kernels)
    built = [module.__name__ for module in modules if wp.get_module(module.__name__).execs]
    assert not built, f"{built} already built: call widen_contact_threads before the first step"
    for module in modules:
        module._NUM_CONTACT_THREADS_PER_BODY = wp.constant(CONTACT_THREADS)


def make_physics_cfg_newton(
    cfg: GeneratedSceneCfg,
    robot: str,
    contact_bodies: Sequence[str],
    substeps: int,
    broad_phase: Literal["explicit", "nxn", "sap"] = "explicit",
) -> NewtonCfg:
    """Newton for a generated scene: plain MJWarp, or -- for a scene with rods
    -- MuJoCo for the robot coupled with VBD for the rods.

    Args:
        cfg: The scene to be spawned. Only whether it authors a rod is read,
            through `cfg.without_rods()`.
        robot: Prim path of the articulation MuJoCo steps, as a regex.
        contact_bodies: Robot bodies a rod may touch, as full-label regexes.
            These are handed to the VBD half as proxies, and **nothing outside
            this list can bend a rod** -- a leg that is not named passes
            straight through one.
        substeps: Physics substeps per simulation step, under either config.
            The rod tuning above assumes the 1.25 ms step four gives with
            `ROD_SUBSTEPS`; at one substep a rod sags about twice as far.
        broad_phase: How the rods' contacts are found. `"explicit"` tests
            the shape pairs `Model.finalize` listed from the collision
            groups, and nothing else, for the life of the model: a cut piece
            meets its own plant and passes through every other
            (`PLANT_GROUPS`), and `ContactSlots` lets the two faces of a cut
            meet. `"nxn"` and `"sap"` read the groups every step instead,
            with a pair buffer sized for every pair of shapes: a cut gives
            its piece groups of its own, so it meets every plant and every
            other piece, all but the face across the cut, which the joint's
            filter pair keeps out for good. No `ContactSlots` rows are
            reserved, so the scene needs no static shapes to park them on.

    Returns:
        A physics config to hand to `SimulationCfg(physics=...)`.
    """
    if cfg.without_rods() is None:
        return NewtonCfg(num_substeps=substeps)
    return NewtonCfg(
        solver_cfg=CouplerProxyCfg(
            entries=[
                CouplerEntryCfg(
                    name="rigid",
                    solver_cfg=MJWarpSolverCfg(),
                    bodies=[robot],
                    # The ground and the trellis. A static shape belongs to
                    # one entry at most, and the robot walking on the terrain
                    # is the one that cannot do without it.
                    include_static_shapes=True,
                ),
                CouplerEntryCfg(
                    name="rods",
                    # The legacy AVBD path, named so Newton's warning about the
                    # default stays quiet. Compliant ALM settles a rod alike on
                    # the numbers above, costs a little more, and under contact
                    # two runs of one scene drift apart where these repeat.
                    solver_cfg=VBDSolverCfg(
                        rigid_compliant_alm=False, rigid_body_contact_buffer_size=BODY_CONTACTS
                    ),
                    # Under any path: only the rod importer labels a body so.
                    bodies=[rf".*/{CABLE}{_ROD_BODY_SUFFIX}"],
                    # An entry that lists no shape still sees its own bodies'
                    # capsules, and the static shapes another entry owns: the
                    # ground, for a cut piece to land on.
                    include_body_shapes=False,
                    substeps=ROD_SUBSTEPS,
                ),
            ],
            proxies=[
                CouplerProxyMappingCfg(
                    source="rigid",
                    destination="rods",
                    bodies=list(contact_bodies),
                    # Refresh the proxies' contacts every pass: a walking robot
                    # moves a rod's width within one step.
                    collide_interval=1,
                    # The pipeline on the rods' view, which finds every
                    # contact a rod makes: with a rod, a proxy or a static
                    # shape.
                    collision_pipeline=NewtonCollisionPipelineCfg(broad_phase=broad_phase),
                )
            ],
        ),
        num_substeps=substeps,
    )

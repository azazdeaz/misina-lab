"""Rod skins, posed from the running simulation before each frame is drawn.

A rod drawn by a skin carries a `SkelRoot` beside its segment prims (see
`misina_lab.usd.build`): a `Skeleton` with a joint per segment, named after
the segment's prim, and the `SkelAnimation` posing it. Newton drives the
segment prims, and whatever rides them, itself; nothing poses the skeleton,
so a skin keeps its rest shape until `drive_skins` is called. From then on,
whenever a renderer is about to draw -- Kit's viewport and Isaac RTX's
cameras both copy the bodies' poses into Fabric first, through
`FabricBackend.update_transforms` -- every skin whose bodies moved since it
was last posed is posed at them. A step nothing draws poses nothing.

The poses go to Fabric, the copy of the stage Kit renders from, through
`usdrt`: written to USD, each would recompose the stage. Kit re-skins a mesh
only in a frame its skeleton was posed in, while its timeline stands still,
as it does under Newton, so a skin that did not move is not written.

A cut splits a skin where it splits the rod, and leaves it where it was
drawn: see `split`, which `Shears` calls.
"""

from __future__ import annotations

import re
from collections.abc import Callable

import numpy as np

SKIN = "Skin"
"""The prim a rod's skin is drawn under, beside the rod's segment prims:
`SKIN` in `crates/misina-lab/src/scene/mod.rs`."""

MOVED = 1e-5
"""How far a joint moves, in meters or in quaternion components, before its
skin is posed again: about 2e-5 rad, far under a pixel."""

_posed: Skins | None = None


def drive_skins() -> None:
    """Pose every rod skin from the Newton model before each frame is drawn,
    from now on. `spawn.spawn_generated` calls this when it spawns rods under
    a solver that steps them. Without Kit nothing is drawn, so it does
    nothing; called again, it does nothing either."""
    from isaaclab.utils.version import has_kit

    if not has_kit():
        return
    from isaaclab_physx.renderers.fabric import FabricBackend

    update = FabricBackend.update_transforms
    if getattr(update, "poses_skins", False):
        return

    def update_transforms(backend, provider) -> None:
        update(backend, provider)
        _pose(backend.stage)

    update_transforms.poses_skins = True
    FabricBackend.update_transforms = update_transforms


def _pose(fabric) -> None:
    """Pose the skins of the model in force, indexing them the first time."""
    global _posed
    from isaaclab_newton.physics import NewtonManager

    model = NewtonManager.get_model()
    if model is None:
        return
    if _posed is None or _posed.model is not model:
        _posed = Skins(model, fabric)
    if len(_posed):
        _posed.pose(NewtonManager.get_state_0().body_q.numpy())


class Skins:
    """Every rod skin on the current stage, the bodies its joints follow in
    `model`, and the Fabric attributes its pose is written to on `fabric`, a
    `usdrt` stage."""

    def __init__(self, model, fabric):
        import isaaclab.sim as sim_utils
        import usdrt
        from pxr import UsdGeom

        from ..usd.build import ANIMATION
        from .rods import _ROD_BODY_SUFFIX, CABLE

        self.model = model
        stage = sim_utils.get_current_stage()
        cache = UsdGeom.XformCache()
        index = {label: body for body, label in enumerate(model.body_label)}
        organs = sorted(
            {
                found[1]
                for label in model.body_label
                if (found := re.fullmatch(rf"(.*)/{CABLE}{_ROD_BODY_SUFFIX}", label))
            }
        )
        bodies, starts, frames, sizes, self._poses = [], [], [], [], []
        for organ in organs:
            if (bound := rig(stage, organ, cache)) is None:
                continue
            _, segments, frame, size = bound
            joints = [index[segment] for segment in segments]
            animation = fabric.GetPrimAtPath(f"{organ}/{SKIN}/{ANIMATION}")
            starts.append(len(bodies))
            bodies += joints
            frames += [frame] * len(joints)
            sizes += [size] * len(joints)
            self._poses.append(
                (animation.GetAttribute("translations"), animation.GetAttribute("rotations"))
            )
        self._bodies = np.array(bodies, dtype=int)
        self._starts = np.array(starts, dtype=int)
        self._frames = np.array(frames, dtype=float).reshape(-1, 7)
        self._sizes = np.array(sizes, dtype=float)[:, None]
        self._last = np.full((len(bodies), 7), np.nan)
        self._vt = usdrt.Vt

    def __len__(self) -> int:
        return len(self._poses)

    def forget(self) -> None:
        """Have every skin posed at the next `pose`, moved or not."""
        self._last[:] = np.nan

    def pose(self, body_q: np.ndarray) -> None:
        """Pose every skin whose bodies moved, at `body_q`, the model's body
        poses in the world: each a position and an xyzw quaternion."""
        local = seen_from(self._frames, body_q[self._bodies], self._sizes)
        moved = np.maximum.reduceat(np.abs(local - self._last).max(axis=1), self._starts)
        # A skin never posed has NaN for its last pose, which moved too.
        moved = np.flatnonzero(~(moved <= MOVED))
        ends = np.append(self._starts[1:], len(local))
        local32 = local.astype(np.float32)
        for skin in moved:
            joints = slice(self._starts[skin], ends[skin])
            translations, rotations = self._poses[skin]
            translations.Set(self._vt.Vec3fArray(np.ascontiguousarray(local32[joints, :3])))
            # `usdrt` reads a quaternion's four floats in xyzw order, Newton's.
            rotations.Set(self._vt.QuatfArray(np.ascontiguousarray(local32[joints, 3:])))
            self._last[joints] = local[joints]


def split(stage, segment: str, at: float, pose: Callable[[str], np.ndarray] | None = None) -> bool:
    """Split the skin of the rod whose segment prim is at `segment` on
    `stage`, the last segment before a cut, which leaves it `at` of the way
    along. Every point before the cut is bound to that segment's joint or
    ones before it, every point past it to the next one's or ones after it.
    The points either side of the cut, a ring each, move onto it along the
    edges that cross it; each ring is closed by a cap there, and the faces
    that spanned the cut go. So the two parts end where the rod was cut.

    `pose` gives the world pose of a segment's body by the segment's prim
    path, as a position and an xyzw quaternion. Every point moved or bound
    anew is then rebased, its place at rest rewritten, so that those poses
    leave the skin where it was drawn: bound to other joints only, a point
    of a bent rod would jump to where they carry it. Without `pose` the rod
    is split as it is at rest.

    Authored in USD, as a cut is rare, and every skin is posed anew at the
    next draw, since Kit skins a mesh again only in a frame its skeleton is
    posed in. Returns whether the rod has a skin."""
    from pxr import Usd, UsdGeom, UsdSkel, Vt

    from ..usd.build import ALONG

    organ = segment.rpartition("/")[0]
    if (bound := rig(stage, organ)) is None:
        return False
    skeleton, segments, frame, size = bound
    joint = segments.index(segment)
    # What takes a point bound to each joint from its place at rest to where
    # it is drawn, in the skeleton's space.
    skinning = np.tile(np.eye(4), (len(segments), 1, 1))
    if pose is not None:
        posed = seen_from(frame, np.array([pose(each) for each in segments]), size)
        # USD's matrices turn a row vector; these turn a column.
        binds = np.array(skeleton.GetBindTransformsAttr().Get()).transpose(0, 2, 1)
        skinning = _matrices(posed) @ np.linalg.inv(binds)
    for prim in Usd.PrimRange(stage.GetPrimAtPath(f"{organ}/{SKIN}")):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        mesh, binding = UsdGeom.Mesh(prim), UsdSkel.BindingAPI(prim)
        along, indices = (
            UsdGeom.PrimvarsAPI(prim).GetPrimvar(ALONG),
            binding.GetJointIndicesPrimvar(),
        )
        points = np.array(mesh.GetPointsAttr().Get(), dtype=float)
        points, where, joints, faces, copied, facing = _split_mesh(
            points,
            np.array(along.Get(), dtype=np.float32),
            np.array(indices.Get()).reshape(len(points), -1),
            np.array(binding.GetJointWeightsPrimvar().Get()).reshape(len(points), -1),
            np.array(mesh.GetFaceVertexIndicesAttr().Get()).reshape(-1, 3),
            skinning,
            joint,
            at,
        )
        along.Set(Vt.FloatArray.FromNumpy(where))
        indices.Set(Vt.IntArray.FromNumpy(joints.astype(np.int32).ravel()))
        # Each point the caps copy, with all it has per point: these, its
        # weights, any colour or texture coordinates.
        source = np.r_[np.arange(len(points)), copied]
        mesh.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(points[source].astype(np.float32)))
        for primvar in UsdGeom.PrimvarsAPI(prim).GetPrimvars():
            if primvar.GetInterpolation() == UsdGeom.Tokens.vertex:
                value = primvar.Get()
                array = np.array(value)
                each = array.reshape(len(points), -1, *array.shape[1:])[source]
                primvar.Set(type(value).FromNumpy(each.reshape(-1, *array.shape[1:])))
        # But for their normals, which face out of the cut, not the tube's way.
        if normals := mesh.GetNormalsAttr().Get():
            normals = np.r_[np.array(normals), facing].astype(np.float32)
            mesh.GetNormalsAttr().Set(Vt.Vec3fArray.FromNumpy(normals))
        mesh.GetFaceVertexIndicesAttr().Set(Vt.IntArray.FromNumpy(faces.astype(np.int32).ravel()))
        mesh.GetFaceVertexCountsAttr().Set(Vt.IntArray([3] * len(faces)))
    if _posed is not None:
        _posed.forget()
    return True


def _split_mesh(points, along, joints, weights, faces, skinning, joint: int, at: float):
    """A skin's mesh split `at` of the way along the segment of `joint`, as
    `split` describes. Takes and returns its `points` at rest, where each is
    `along` the rod, the `joints` each is bound to, and its triangles,
    `faces`. `weights` are the points' own over their joints, and `skinning`
    is, per joint of the rod, the 4x4 that takes a point from its place at
    rest to where the joint carries it. Also returns the points the caps
    copy, in the order the copies are numbered on from the last point, and
    each copy's normal at rest."""
    # As exact as `along` is, so that a point put on the cut is on a side.
    cut = np.float32(joint + at)
    before = along < cut

    def blend(joints):
        return np.einsum("vi,vijk->vjk", weights, skinning[joints])

    drawn = np.einsum("vjk,vk->vj", blend(joints), np.c_[points, np.ones(len(points))])
    bound = np.where(before[:, None], np.minimum(joints, joint), np.maximum(joints, joint + 1))

    side = before[faces]
    across = side.any(axis=1) & ~side.all(axis=1)
    spanning = faces[across]
    # Every edge across the cut, from each of its ends, by how far along the
    # rod it gets for its length: a point with several takes the last
    # written, the one running most nearly along the rod, not a face's
    # diagonal or an edge round the rod.
    ways = ((0, 1), (1, 2), (2, 0), (1, 0), (2, 1), (0, 2))
    edges = np.concatenate([spanning[:, way] for way in ways])
    edges = edges[before[edges[:, 0]] != before[edges[:, 1]]]
    near, far = edges.T
    steep = np.abs(along[far] - along[near]) / np.linalg.norm(points[far] - points[near], axis=1)
    near, far = edges[np.argsort(steep)].T
    other = np.arange(len(points))
    other[near] = far
    ends = np.unique(near)
    reach = (cut - along[ends]) / (along[other[ends]] - along[ends])
    drawn[ends] += reach[:, None] * (drawn[other[ends]] - drawn[ends])
    along = along.copy()
    along[ends] = np.where(before[ends], np.nextafter(cut, np.float32(-np.inf)), cut)

    # ponytail: a point's normal keeps its direction at rest, which the new
    # joints turn by the rod's bend at the cut more or less than the old.
    # Rebase the normals like the points if the shading there ever shows it.
    moved = (bound != joints).any(axis=1)
    moved[ends] = True
    points = points.copy()
    points[moved] = np.linalg.solve(blend(bound)[moved], drawn[moved][..., None])[:, :3, 0]

    # Each ring is closed by a fan from one of its points over its edges,
    # each wound as the spanning face it was an edge of, so a cap faces out.
    # The fan is over copies of the ring's points, numbered on from the
    # last, whose normals face out of the cut as the cap does: shared, the
    # points' own normals, square to the rod, would shade the cap as the tube.
    caps, copied, facing = [faces[~across]], np.empty(0, int), np.empty((0, 3))
    turned = np.concatenate([np.roll(spanning, -turn, axis=1) for turn in range(3)])
    for first in (True, False):
        rim = turned[(before[turned[:, :2]] == first).all(axis=1), :2]
        if len(rim):
            hub = rim.min()
            fan = np.c_[np.full(len(rim), hub), rim][(rim != hub).all(axis=1)]
            corner = points[fan]
            out = np.cross(corner[:, 1] - corner[:, 0], corner[:, 2] - corner[:, 0]).sum(axis=0)
            ring, fan = np.unique(fan, return_inverse=True)
            caps.append(len(points) + len(copied) + fan.reshape(-1, 3))
            copied = np.r_[copied, ring]
            facing = np.r_[facing, np.tile(out / np.linalg.norm(out), (len(ring), 1))]
    return points, along, bound, np.concatenate(caps), copied, facing


def rig(stage, organ: str, cache=None):
    """The skeleton the skin of the rod at `organ` on `stage` is bound to,
    the prim paths of the segments its joints follow, which are their
    bodies' labels, in joint order, and the skeleton's frame and uniform
    scale in the world: see `frame_of`. None where the rod has no skin.
    `cache` is a `UsdGeom.XformCache` to share between rods."""
    from pxr import UsdGeom, UsdSkel

    from ..usd.build import SKELETON

    skeleton = UsdSkel.Skeleton(stage.GetPrimAtPath(f"{organ}/{SKIN}/{SKELETON}"))
    if not skeleton:
        return None
    # The skeleton's space is its organ's, which nothing moves.
    world = (cache or UsdGeom.XformCache()).GetLocalToWorldTransform(skeleton.GetPrim())
    frame, size = frame_of(world)
    if size is None:
        raise ValueError(f"{organ} is scaled unevenly or sheared; its skin would not fit")
    return skeleton, [f"{organ}/{name}" for name in skeleton.GetJointsAttr().Get()], frame, size


def frame_of(world) -> tuple[np.ndarray, float | None]:
    """A skeleton's local-to-world matrix `world` as a frame, its position
    and unit xyzw quaternion, and the uniform scale it is drawn at: None
    where `world` scales unevenly or shears, which no one scale undoes."""
    turn = world.RemoveScaleShear().ExtractRotationQuat()
    frame = np.array([*world.ExtractTranslation(), *turn.GetImaginary(), turn.GetReal()])
    # A turn at one scale keeps its axes square to each other and as long.
    axes = np.array([world.GetRow3(axis) for axis in range(3)])
    square = axes @ axes.T
    size = float(np.sqrt(np.trace(square) / 3))
    return frame, size if np.allclose(square, size**2 * np.eye(3), atol=1e-4 * size**2) else None


def seen_from(frames: np.ndarray, poses: np.ndarray, sizes=1.0) -> np.ndarray:
    """`poses` as seen from `frames` drawn at `sizes`, a uniform scale: each
    a position and a unit xyzw quaternion in the world, `frames` and `sizes`
    row by row with `poses` or one for all of them. A joint's pose in its
    skeleton's space is its body's seen from the skeleton's frame, its
    position divided by the skeleton's scale."""
    away = frames[..., 3:] * np.array([-1.0, -1.0, -1.0, 1.0])
    near = _rotate(away, poses[:, :3] - frames[..., :3]) / sizes
    return np.concatenate([near, _multiply(away, poses[:, 3:])], axis=1)


def _matrices(poses: np.ndarray) -> np.ndarray:
    """`poses`, each a position and a unit xyzw quaternion, as 4x4 matrices
    that turn a column vector."""
    matrices = np.tile(np.eye(4), (len(poses), 1, 1))
    for axis in range(3):
        matrices[:, :3, axis] = _rotate(poses[:, 3:], np.eye(3)[axis])
    matrices[:, :3, 3] = poses[:, :3]
    return matrices


def _multiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """The xyzw quaternion products `a b`, row by row."""
    (ax, ay, az, aw), (bx, by, bz, bw) = a.T, b.T
    return np.stack(
        [
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        ],
        axis=1,
    )


def _rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """`v` turned by the unit xyzw quaternions `q`, row by row."""
    t = 2.0 * np.cross(q[..., :3], v)
    return v + q[..., 3:] * t + np.cross(q[..., :3], t)

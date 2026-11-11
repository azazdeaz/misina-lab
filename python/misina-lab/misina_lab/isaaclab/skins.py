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

A cut splits a skin where it splits the rod: see `split`, which `Shears`
calls.
"""

from __future__ import annotations

import re

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
        from pxr import UsdGeom, UsdSkel

        from ..usd.build import ANIMATION, SKELETON
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
        bodies, starts, frames, self._poses = [], [], [], []
        for organ in organs:
            skeleton = UsdSkel.Skeleton(stage.GetPrimAtPath(f"{organ}/{SKIN}/{SKELETON}"))
            if not skeleton:
                continue
            joints = [index[f"{organ}/{name}"] for name in skeleton.GetJointsAttr().Get()]
            # The skeleton's space is its organ's, which nothing moves.
            world = cache.GetLocalToWorldTransform(skeleton.GetPrim())
            scale = [world.GetRow3(axis).GetLength() for axis in range(3)]
            if not np.allclose(scale, 1.0, atol=1e-4):
                raise ValueError(f"{organ} is scaled by {scale}; its skin would not fit its rod")
            turn = world.ExtractRotationQuat()
            frame = [*world.ExtractTranslation(), *turn.GetImaginary(), turn.GetReal()]
            animation = fabric.GetPrimAtPath(f"{organ}/{SKIN}/{ANIMATION}")
            starts.append(len(bodies))
            bodies += joints
            frames += [frame] * len(joints)
            self._poses.append(
                (animation.GetAttribute("translations"), animation.GetAttribute("rotations"))
            )
        self._bodies = np.array(bodies, dtype=int)
        self._starts = np.array(starts, dtype=int)
        self._frames = np.array(frames, dtype=float).reshape(-1, 7)
        self._last = np.full((len(bodies), 7), np.nan)
        self._vt = usdrt.Vt

    def __len__(self) -> int:
        return len(self._poses)

    def pose(self, body_q: np.ndarray) -> int:
        """Pose every skin whose bodies moved, at `body_q`, the model's body
        poses in the world: each a position and an xyzw quaternion. Returns
        how many it posed."""
        if not len(self):
            return 0
        local = seen_from(self._frames, body_q[self._bodies])
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
        return len(moved)


def split(stage, segment: str, at: float) -> bool:
    """Split the skin of the rod whose segment prim is at `segment` on
    `stage`, the last segment before a cut, which leaves it `at` of the way
    along: every point before the cut is bound to that segment's joint or
    ones before it, every point past it to the next one's or ones after it,
    and every face across the cut is dropped. Authored in USD, as a cut is
    rare. Returns whether the rod has a skin."""
    from pxr import Usd, UsdGeom, UsdSkel, Vt

    from ..usd.build import ALONG

    root = stage.GetPrimAtPath(f"{segment.rpartition('/')[0]}/{SKIN}")
    if not root.IsValid():
        return False
    joint = int(segment.rsplit("_", 1)[1])
    cut = joint + at
    for prim in Usd.PrimRange(root):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        mesh = UsdGeom.Mesh(prim)
        before = np.array(UsdGeom.PrimvarsAPI(prim).GetPrimvar(ALONG).Get()) < cut
        indices = UsdSkel.BindingAPI(prim).GetJointIndicesPrimvar()
        joints = np.array(indices.Get()).reshape(len(before), -1)
        joints = np.where(before[:, None], np.minimum(joints, joint), np.maximum(joints, joint + 1))
        indices.Set(Vt.IntArray.FromNumpy(joints.astype(np.int32).ravel()))
        faces = np.array(mesh.GetFaceVertexIndicesAttr().Get()).reshape(-1, 3)
        side = before[faces]
        faces = faces[side.all(axis=1) | ~side.any(axis=1)]
        mesh.GetFaceVertexIndicesAttr().Set(Vt.IntArray.FromNumpy(faces.astype(np.int32).ravel()))
        mesh.GetFaceVertexCountsAttr().Set(Vt.IntArray([3] * len(faces)))
    return True


def seen_from(frames: np.ndarray, poses: np.ndarray) -> np.ndarray:
    """`poses` as seen from `frames`: each a position and a unit xyzw
    quaternion in the world, `frames` row by row with `poses` or one for all
    of them. A joint's pose in its skeleton's space is its body's seen from
    the skeleton's frame."""
    away = frames[..., 3:] * np.array([-1.0, -1.0, -1.0, 1.0])
    return np.concatenate(
        [_rotate(away, poses[:, :3] - frames[..., :3]), _multiply(away, poses[:, 3:])], axis=1
    )


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

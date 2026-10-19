"""Every collider Newton steps, drawn in Kit's viewport.

Kit draws a PhysX scene's colliders on request, from the physics schema on the
stage; a Newton model's it knows nothing of. `ColliderView` draws each
collision shape of the running model as a translucent primitive of the same
shape and size, a hair proud of it. A shape on a body is drawn under that
body's prim, so the
drawing moves with the body the way the body's own meshes do, at no cost per
frame; a shape on no body is drawn in place, under one scope of its own. The
colour says what a shape is on: the static scene, a rod, or any other body.

Newton's own copy is what is drawn, so a shape changed while the simulation
runs is drawn changed once `update` has looked: a rod cut, its segment
shortened and the next one stretched, or a collider moved off its body's
meshes, the way a blade's slice stands at a rod the blade closes through.
"""

from __future__ import annotations

import numpy as np

ROOT = "/World/Colliders"
"""Where the drawing keeps the static scene's shapes and its materials."""

LOOKS = {
    "scene": (0.2, 0.5, 1.0, 0.2),
    "rod": (0.2, 0.9, 0.3, 0.35),
    "body": (1.0, 0.1, 0.8, 0.35),
}
"""The linear RGB and opacity of each kind of shape: the static scene's, a
rod's, and any other body's -- a robot's. Opaque enough to see, and thin
enough to see the meshes they collide for through them; the static scene's
thinnest, since a ground fills the picture."""

SKIN = 0.0005
"""How far proud of its collider a shape is drawn, in meters: a drawing
exactly on a mesh it coincides with -- a rod's tube, a robot's plate --
flickers in and out of it."""


class ColliderView:
    """Draws every collision shape of the running Newton model on `stage`,
    the running one by default.

    Build one after the first `SimulationContext.reset()`, once the model
    exists, and call `update` as the simulation runs to redraw what has
    moved or changed size in its body's frame since the last call. A body
    whose label names no prim on the stage cannot carry a drawing, so its
    shapes are left out; `missing` counts them.
    """

    def __init__(self, stage=None, root: str = ROOT):
        # Imported here, as in `cutting`: `newton` brings `pxr` with it.
        import isaaclab.sim as sim_utils
        from isaaclab_newton.physics import NewtonManager
        from newton import JointType, ShapeFlags
        from pxr import Sdf, UsdGeom, UsdShade

        self._stage = stage if stage is not None else sim_utils.get_current_stage()
        model = self._model = NewtonManager._model
        if model is None:
            raise RuntimeError(
                "no Newton model to draw: build ColliderView after the first reset, under Newton"
            )
        rods = model.joint_type.numpy() == int(JointType.ROD)
        rod_bodies = set(model.joint_parent.numpy()[rods]) | set(model.joint_child.numpy()[rods])
        colliding = (model.shape_flags.numpy() & int(ShapeFlags.COLLIDE_SHAPES)) != 0
        bodies, types = model.shape_body.numpy(), model.shape_type.numpy()
        self._transform = model.shape_transform.numpy()
        self._scale = model.shape_scale.numpy()

        self._ops: dict[int, tuple] = {}
        """Each drawn shape's primitive, its Newton type, and its translate,
        orient and scale ops."""
        self.missing = 0
        """Shapes left out, their body having no prim on the stage."""
        UsdGeom.Scope.Define(self._stage, root)
        looks = {
            kind: _material(self._stage, f"{root}/Looks/{kind}", *look)
            for kind, look in LOOKS.items()
        }
        for shape in np.flatnonzero(colliding):
            body = int(bodies[shape])
            parent = root if body < 0 else model.body_label[body]
            if not self._stage.GetPrimAtPath(parent).IsValid():
                self.missing += 1
                continue
            kind = "scene" if body < 0 else "rod" if body in rod_bodies else "body"
            drawn = self._author(f"{parent}/Collider_{shape}", int(types[shape]), shape)
            UsdGeom.PrimvarsAPI(drawn).CreatePrimvar(
                "doNotCastShadows", Sdf.ValueTypeNames.Bool
            ).Set(True)
            UsdShade.MaterialBindingAPI.Apply(drawn.GetPrim()).Bind(looks[kind])
            ops = (drawn.AddTranslateOp(), drawn.AddOrientOp(), drawn.AddScaleOp())
            self._ops[int(shape)] = (drawn, int(types[shape]), *ops)
            self._place(int(shape))

    def update(self) -> None:
        """Redraw every shape that has moved in its body's frame, or changed
        size, since the last look."""
        transform = self._model.shape_transform.numpy()
        scale = self._model.shape_scale.numpy()
        changed = (transform != self._transform).any(axis=1) | (scale != self._scale).any(axis=1)
        self._transform, self._scale = transform, scale
        for shape in np.flatnonzero(changed):
            if int(shape) in self._ops:
                self._place(int(shape))

    def _author(self, path: str, kind: int, shape: int):
        """The primitive at `path` drawing shape `shape`, of Newton type
        `kind`. Its size is set by `_place`; a mesh's points are its own, and
        only scaled."""
        from newton import GeoType
        from pxr import UsdGeom, Vt

        primitives = {
            GeoType.BOX: UsdGeom.Cube,
            GeoType.SPHERE: UsdGeom.Sphere,
            GeoType.ELLIPSOID: UsdGeom.Sphere,
            GeoType.CAPSULE: UsdGeom.Capsule,
            GeoType.CYLINDER: UsdGeom.Cylinder,
            GeoType.CONE: UsdGeom.Cone,
        }
        if kind in primitives:
            return primitives[kind].Define(self._stage, path)
        vertices, indices = _mesh(kind, self._scale[shape], self._model.shape_source[shape])
        mesh = UsdGeom.Mesh.Define(self._stage, path)
        mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(vertices.astype(np.float32)))
        mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(indices.astype(np.int32)))
        mesh.CreateFaceVertexCountsAttr(
            Vt.IntArray.FromNumpy(np.full(len(indices) // 3, 3, np.int32))
        )
        return mesh

    def _place(self, shape: int) -> None:
        """Set shape `shape`'s drawing to where Newton has it, at its size."""
        from newton import GeoType
        from pxr import Gf, UsdGeom

        drawn, kind, translate, orient, scale = self._ops[shape]
        size = self._scale[shape].astype(float)
        stretch = (1.0, 1.0, 1.0)
        if kind in (GeoType.CAPSULE, GeoType.CYLINDER, GeoType.CONE):
            # Round ends have to keep their radius, so these are sized, not
            # scaled; Newton's second number is half the length along z, and
            # a capsule's is the length between its round ends.
            drawn.CreateAxisAttr("Z")
            drawn.CreateRadiusAttr(size[0] + SKIN)
            drawn.CreateHeightAttr(2 * (size[1] + (SKIN if kind != GeoType.CAPSULE else 0.0)))
        elif kind == GeoType.SPHERE:
            drawn.CreateRadiusAttr(size[0] + SKIN)
        elif kind == GeoType.BOX:
            # A cube two across, scaled by the half extents.
            drawn.CreateSizeAttr(2.0)
            stretch = size + SKIN
        elif kind == GeoType.ELLIPSOID:
            drawn.CreateRadiusAttr(1.0)
            stretch = size + SKIN
        elif kind in (GeoType.MESH, GeoType.CONVEX_MESH):
            stretch = size
        x, y, z, w = self._transform[shape, 3:].astype(float)
        translate.Set(Gf.Vec3d(*self._transform[shape, :3].astype(float)))
        orient.Set(Gf.Quatf(w, x, y, z))
        scale.Set(Gf.Vec3f(*stretch))
        # A cut hides whatever hangs off its segment past it, and the drawing
        # of the segment's own shape, placed at its old middle, can be among
        # that.
        if drawn.GetVisibilityAttr().Get() == UsdGeom.Tokens.invisible:
            drawn.GetVisibilityAttr().Set(UsdGeom.Tokens.inherited)


def _mesh(kind: int, scale: np.ndarray, source) -> tuple[np.ndarray, np.ndarray]:
    """The vertices and flat triangle indices drawing a shape that is no
    primitive: a mesh's own, unscaled, and a height field or a plane meshed
    the way Newton's viewer meshes them."""
    from newton import GeoType, Mesh

    if kind == GeoType.HFIELD:
        # Rows run along y and columns along x; the mesh wants x first.
        heights = source.min_z + source.data * (source.max_z - source.min_z)
        source = Mesh.create_heightfield(
            heightfield=heights.T,
            extent_x=2 * source.hx,
            extent_y=2 * source.hy,
            ground_z=source.min_z,
            compute_inertia=False,
        )
    elif kind == GeoType.PLANE:
        # A zero extent is an infinite plane, drawn a kilometre across.
        width, length = (float(v) if v > 0 else 1000.0 for v in scale[:2])
        source = Mesh.create_plane(width, length, compute_inertia=False)
    return np.asarray(source.vertices), np.asarray(source.indices).ravel()


def _material(stage, path: str, r: float, g: float, b: float, opacity: float):
    """A translucent surface of colour (`r`, `g`, `b`), glowing faintly so it
    shows in shadow too."""
    from pxr import Gf, Sdf, UsdShade

    colour = Gf.Vec3f(r, g, b)
    material = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, f"{path}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(colour)
    shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(colour * 0.25)
    shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material

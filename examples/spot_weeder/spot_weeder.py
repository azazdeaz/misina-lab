"""Boston Dynamics Spot with the mechanical weeding tool of Beumer et al.,
"Mechanical Precision Weeding with a Quadruped Robot" (arXiv 2609.20048).

The tool is bolted to the rails on Spot's back: a tray along the body carrying
the electronics, and at its front a head with the stepper motor on top and
the spindle under it, a bearing tube with a steel shaft turning in it and a
22 mm milling bit at the shaft's end. The shaft leans forward, bit first, so that it stands
upright when Spot pitches its body down to mill. The tool has no joint to aim
with: Spot shifts, pitches and yaws its body over planted feet to put the bit
on a weed. A RealSense L515 hangs in front of the shaft on an arm, looking
down at the bit.

`write_usd` writes the robot as `spot_weeder.usda`. Spot in it is NVIDIA's,
from Isaac Sim 6.2's asset server, referenced by its URL: NVIDIA's licence
lets nobody else host a copy. Isaac Lab downloads it when it spawns the
robot. On top of it the file adds the tool's two links, each with its
visuals and its colliders:

- `tool`, fixed to Spot's `body` by `body/tool_mount`. Its frame is the
  body's, and it holds the L515's camera, `tool/camera`.
- `bit`, the shaft and the bit, turning on `tool/spindle`, a revolute joint
  along the shaft with a velocity drive. Its origin is the end effector, the
  bottom of the bit, and its z axis runs up the shaft.

Every position here is in Spot's body frame: x forward, z up, the origin
between the four hips. The numbers marked as the paper's are from its text
and tables; the rest of the tool's shape is read off its photos and its
drawing of the robot, which give no dimensions.

The example spawns the copy published in `ASSETS`. To write it again:

    python spot_weeder.py OUT_DIR
"""

from __future__ import annotations

import functools
import math
import pathlib
import sys

import numpy as np
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg
from isaaclab_assets.robots.spot import SPOT_CFG

ASSETS = "azazdeaz/misina-lab-sim-assets"
"""The Hugging Face dataset the robot is published in. `huggingface_hub`
fetches it from there on first use and caches it."""

ASSETS_REVISION = "226040398cd51bcd9aa4ff9c5fcf88163496e8ba"
"""The dataset commit the example is pinned to."""

FOLDER = "spot_weeder"
"""The robot's folder in `ASSETS`."""

FILE = "spot_weeder.usda"

SPOT_URL = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com"
    "/Assets/Isaac/6.2/Isaac/Robots/BostonDynamics/spot/spot.usd"
)
"""NVIDIA's Spot, the one Isaac Lab's `SPOT_CFG` spawns."""

TOOL, BIT, SPINDLE = "tool", "bit", "spindle"
"""The tool's two links, and the joint the bit turns on."""

TIP = np.array([0.80, 0.0, -0.30])
"""The end effector, the bottom of the bit: the paper's x_arm, y_arm and
z_arm, which its Table 2 gives through its Eq. 1. A body shifted 0.08 m and
turned pi/10 in pitch and yaw puts the bit at (715.44, 206.47, -32.53) mm."""

Z_NOM = 0.5
"""The body's height over the ground when Spot stands: the paper's z_nom."""

TILT = math.radians(17.0)
"""How far the shaft leans forward from the body's z axis. It stands upright
at a body pitch of 17 degrees: about where the paper mills, with the bit 2 cm
into the ground."""

MASS = 7.0
"""The tool's mass, the paper's "approximately 7 kg"."""

CENTER_OF_MASS = np.array([0.548, 0.0, 0.10])
"""The tool's centre of mass, at the height of the tray and 25 cm ahead of the
front hips: one reading of the paper's "The weight is centered approximately
25 cm from the base of the tool extending in front of Spot's front legs"."""

LENS = 0.26
"""How high the L515's lens is above the bit: the paper's "slightly more than
25 cm", the camera's nearest depth, on an arm down from the head as in its
indoor photos. Its field photo has the camera higher, level with the tray."""

SPINDLE_SPEED = 7000 * 2 * math.pi / 60
"""The spindle's top speed, the paper's 7000 rpm, in rad/s."""

SPINDLE_TORQUE = 0.05
"""The most the spindle's NEMA 17 stepper turns it with, in N m: an estimate
of what such a motor keeps of its ~0.4 N m holding torque near its top
speed."""

SPINDLE_DAMPING = SPINDLE_TORQUE / (0.1 * SPINDLE_SPEED)
"""The spindle drive's gain, in N m s/rad: its full torque at 10% under the
target speed."""

HIPS = {
    "fl": (0.29785, 0.165945),
    "fr": (0.29785, -0.165945),
    "hl": (-0.29785, 0.165945),
    "hr": (-0.29785, -0.165945),
}
"""Where each of Spot's legs pitches, its `*_hy` axis, in the body's xy
plane, with the hip's roll at zero."""

THIGH = np.array([0.025, -0.3205])
"""Spot's knee from its hip's pitch axis, (x, z) in the upper leg."""

SHIN = 0.3365
"""Spot's foot from its knee, straight down the lower leg."""

FOOT = 0.035
"""The radius of Spot's foot."""


def stance(pitch: float = 0.0) -> dict[str, float]:
    """Spot's joint angles for its body `Z_NOM` over feet planted below its
    hips, pitched `pitch` radians nose down about its centre: the paper's
    body pose command, with no shift and no yaw."""
    c, s = math.cos(pitch), math.sin(pitch)
    down = FOOT - Z_NOM
    # How far the knee sits off the upper leg's axis, as an angle at the hip.
    offset = math.atan2(THIGH[0], -THIGH[1])
    joints = {}
    for leg, (x, y) in HIPS.items():
        # The foot from the hip's pitch axis, (x, z) in the pitched body.
        foot = np.array([c * x - s * down - x, s * x + c * down])
        bend = (foot @ foot - THIGH @ THIGH - SHIN**2) / (2 * np.linalg.norm(THIGH) * SHIN)
        knee = -math.acos(bend) - offset
        # Spot's leg as the knee bends it, then swung about the hip onto the foot.
        reach = THIGH + SHIN * np.array([-math.sin(knee), -math.cos(knee)])
        hip = math.atan2(reach[1], reach[0]) - math.atan2(foot[1], foot[0])
        joints |= {f"{leg}_hx": 0.0, f"{leg}_hy": hip, f"{leg}_kn": knee}
    return joints


def pitch_to(height: float) -> float:
    """The body pitch over planted feet that puts the bit `height` over the
    ground, with no shift and no yaw: the paper's Eq. 2."""
    x, _, z = TIP
    return math.acos((height - Z_NOM) / math.hypot(x, z)) - math.atan2(x, z)


@functools.cache
def asset() -> pathlib.Path:
    """`FILE` from `ASSETS`, downloaded the first time."""
    from huggingface_hub import snapshot_download

    root = snapshot_download(
        ASSETS, repo_type="dataset", revision=ASSETS_REVISION, allow_patterns=f"{FOLDER}/*"
    )
    return pathlib.Path(root) / FOLDER / FILE


def robot_cfg(usd: pathlib.Path | None = None) -> ArticulationCfg:
    """Isaac Lab's `SPOT_CFG` on the weeder in `usd`, by default the published
    one: Spot's own actuators on the legs, and a velocity drive on the
    spindle. It starts standing in `stance`, its feet on the ground."""
    return SPOT_CFG.replace(
        spawn=SPOT_CFG.spawn.replace(usd_path=str(usd or asset())),
        init_state=SPOT_CFG.init_state.replace(pos=(0.0, 0.0, Z_NOM), joint_pos=stance()),
        actuators={
            **SPOT_CFG.actuators,
            SPINDLE: ImplicitActuatorCfg(
                joint_names_expr=[SPINDLE],
                stiffness=0.0,
                damping=SPINDLE_DAMPING,
                joint_effort_limit=SPINDLE_TORQUE,
                joint_velocity_limit=SPINDLE_SPEED,
            ),
        },
    )


def write_usd(folder: pathlib.Path) -> pathlib.Path:
    """Writes the robot, `FILE`, into `folder`, and returns its path."""
    # Imported here: the demo imports this module before Kit starts, and Kit's
    # own `pxr` has to be the first one loaded.
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    robot = UsdGeom.Xform.Define(stage, "/spot").GetPrim()
    stage.SetDefaultPrim(robot)
    UsdGeom.Scope.Define(stage, "/spot/Looks")
    upright = Gf.Rotation()
    # Turns the z axis up the shaft.
    lean = Gf.Rotation(Gf.Vec3d(0, 1, 0), -math.degrees(TILT))

    def place(prim, at, turn=upright, scale=None):
        xform = UsdGeom.Xformable(prim)
        xform.AddTranslateOp().Set(Gf.Vec3d(*map(float, at)))
        xform.AddOrientOp().Set(Gf.Quatf(turn.GetQuat()))
        if scale is not None:
            xform.AddScaleOp().Set(Gf.Vec3f(*map(float, scale)))

    def material(name, color, metallic=0.0, roughness=0.5):
        look = UsdShade.Material.Define(stage, f"/spot/Looks/{name}")
        shader = UsdShade.Shader.Define(stage, look.GetPath().AppendChild("shader"))
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(color)
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
        look.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        return look

    blue = material("anodized_blue", (0.01, 0.1, 0.6), metallic=0.5, roughness=0.35)
    black = material("black", (0.02, 0.02, 0.02), roughness=0.4)
    steel = material("steel", (0.6, 0.6, 0.62), metallic=1.0, roughness=0.25)
    red = material("red", (0.6, 0.02, 0.02), roughness=0.4)
    green = material("green_pla", (0.21, 0.89, 0.03), roughness=0.6)
    white = material("white", (0.8, 0.8, 0.8))
    cyan = material("cyan", (0.0, 0.45, 0.6))

    def finish(gprim, look):
        """Paints a visual with `look`; with none, makes `gprim` a collider,
        which renders only as a guide."""
        if look is None:
            UsdPhysics.CollisionAPI.Apply(gprim.GetPrim())
            gprim.CreatePurposeAttr(UsdGeom.Tokens.guide)
        else:
            UsdShade.MaterialBindingAPI.Apply(gprim.GetPrim()).Bind(look)

    def box(path, at, size, look=None, turn=upright):
        cube = UsdGeom.Cube.Define(stage, path)
        cube.CreateSizeAttr(1.0)
        cube.CreateExtentAttr([(-0.5, -0.5, -0.5), (0.5, 0.5, 0.5)])
        place(cube, at, turn, size)
        finish(cube, look)

    def cylinder(path, at, radius, height, look=None):
        """A cylinder along its parent's z axis."""
        shape = UsdGeom.Cylinder.Define(stage, path)
        shape.CreateRadiusAttr(radius)
        shape.CreateHeightAttr(height)
        shape.CreateExtentAttr([(-radius, -radius, -height / 2), (radius, radius, height / 2)])
        place(shape, at)
        finish(shape, look)

    def mesh(path, points, faces, look):
        shape = UsdGeom.Mesh.Define(stage, path)
        shape.CreatePointsAttr([Gf.Vec3f(*map(float, p)) for p in points])
        shape.CreateFaceVertexCountsAttr([len(face) for face in faces])
        shape.CreateFaceVertexIndicesAttr([int(i) for face in faces for i in face])
        shape.CreateExtentAttr([Gf.Vec3f(*points.min(0)), Gf.Vec3f(*points.max(0))])
        # Machined parts: flat facets, not a subdivision surface.
        shape.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        finish(shape, look)

    def prism(path, outline, depth, look):
        """The polygon `outline` swept along the vector `depth`. The outline
        runs anticlockwise seen from the far end of `depth`."""
        outline = np.asarray(outline, dtype=float)
        n = len(outline)
        sides = [[i, (i + 1) % n, n + (i + 1) % n, n + i] for i in range(n)]
        faces = [list(range(n))[::-1], list(range(n, 2 * n)), *sides]
        mesh(path, np.vstack([outline, outline + depth]), faces, look)

    def lathe(path, profile, look, segments=24):
        """The (radius, height) points of `profile`, bottom to top, turned
        about the z axis."""
        turn = np.linspace(0, 2 * np.pi, segments, endpoint=False)
        ring = np.stack([np.cos(turn), np.sin(turn), np.zeros(segments)], axis=1)
        points = np.vstack([ring * r + (0, 0, z) for r, z in profile])
        faces = [
            [i * segments + j, i * segments + k, (i + 1) * segments + k, (i + 1) * segments + j]
            for i in range(len(profile) - 1)
            for j, k in ((j, (j + 1) % segments) for j in range(segments))
        ]
        mesh(path, points, faces, look)

    def link(name, at, turn, mass, center, inertia):
        """A rigid body under the robot, its mass at `center` in its own frame,
        with an Xform for its visuals and one for its colliders."""
        prim = UsdGeom.Xform.Define(stage, f"/spot/{name}").GetPrim()
        place(prim, at, turn)
        for group in ("visuals", "collisions"):
            UsdGeom.Xform.Define(stage, f"/spot/{name}/{group}")
        UsdPhysics.RigidBodyAPI.Apply(prim)
        body = UsdPhysics.MassAPI.Apply(prim)
        body.CreateMassAttr(mass)
        body.CreateCenterOfMassAttr(Gf.Vec3f(*map(float, center)))
        body.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertia))
        body.CreatePrincipalAxesAttr(Gf.Quatf(1.0))
        return prim.GetPath()

    def shaft(path):
        """An Xform at the bit, its z axis up the shaft."""
        place(UsdGeom.Xform.Define(stage, path), TIP, lean)

    # The bit's link: the steel shaft tapering down to the bit, a cylinder
    # 0.25 kg in all.
    bit_mass, bit_length, bit_radius = 0.25, 0.17, 0.008
    bit_center = np.array([0.0, 0.0, 0.08])
    across = bit_mass * (3 * bit_radius**2 + bit_length**2) / 12
    bit = link(BIT, TIP, lean, bit_mass, bit_center, (across, across, bit_mass * bit_radius**2 / 2))
    cylinder(f"{bit}/visuals/bit", (0, 0, 0.01), 0.011, 0.02, red)
    lathe(f"{bit}/visuals/shaft", [(0.0045, 0.02), (0.009, 0.12), (0.009, 0.17)], steel)
    cylinder(f"{bit}/collisions/bit", (0, 0, 0.01), 0.011, 0.02)
    cylinder(f"{bit}/collisions/shaft", (0, 0, 0.095), 0.007, 0.15)

    # The tool's link takes the rest of the paper's mass, centred so that the
    # two links' centre is the paper's. Its inertia is the estimate of a
    # tray-long body heavy at its front end.
    shaft_up = np.array(lean.TransformDir(Gf.Vec3d(0, 0, 1)))
    rest = MASS - bit_mass
    center = (MASS * CENTER_OF_MASS - bit_mass * (TIP + bit_center[2] * shaft_up)) / rest
    tool = link(TOOL, (0, 0, 0), upright, rest, center, (0.05, 0.38, 0.37))
    seen, felt = f"{tool}/visuals", f"{tool}/collisions"

    # The tray: a channel tapering forward, standing on the body's top at
    # z = 0.095 from its middle to past its front, as Fig. 5a and the indoor
    # photos have it.
    rear, front, half_rear, half_front = 0.0, 0.64, 0.07, 0.035
    floor, top, wall = 0.095, 0.145, 0.004
    right = [
        (rear, -half_rear),
        (front, -half_front),
        (front, wall - half_front),
        (rear, wall - half_rear),
    ]

    def level(outline):
        return [(x, y, floor) for x, y in outline]

    footprint = [(rear, -half_rear), (front, -half_front), (front, half_front), (rear, half_rear)]
    prism(f"{seen}/tray_floor", level(footprint), (0, 0, 0.006), blue)
    prism(f"{seen}/tray_right", level(right), (0, 0, top - floor), blue)
    left = [(x, -y) for x, y in reversed(right)]
    prism(f"{seen}/tray_left", level(left), (0, 0, top - floor), blue)
    back = [
        (rear, -half_rear),
        (rear + wall, -half_rear),
        (rear + wall, half_rear),
        (rear, half_rear),
    ]
    prism(f"{seen}/tray_back", level(back), (0, 0, top - floor), blue)
    box(
        f"{felt}/tray",
        ((rear + front) / 2, 0, (floor + top) / 2),
        (front - rear, 2 * half_rear, top - floor),
    )

    # The electronics in the tray, as the paper ran all its tests: the DC-DC
    # converter feeding the motor controller 48 V from Spot, the Raspberry Pi
    # 4B that talks to the controller and the camera, the battery powering
    # it, and the Nanotec C5-E motor controller.
    inside = floor + 0.006
    box(f"{seen}/converter", (0.08, 0, inside + 0.015), (0.08, 0.06, 0.03), white)
    box(f"{seen}/raspberry_pi", (0.17, 0, inside + 0.0125), (0.095, 0.065, 0.025), black)
    box(f"{seen}/battery", (0.33, 0, inside + 0.0125), (0.15, 0.07, 0.025), cyan)
    box(f"{seen}/controller", (0.5, 0, inside + 0.0125), (0.105, 0.065, 0.025), steel)

    # The head at the tray's front end. Its top is square to the shaft, the
    # motor's seat, at 0.5 m up the shaft from the bit.
    seat = 0.5
    seat_x, _, seat_z = TIP + seat * shaft_up

    def roof(x):
        return seat_z + math.tan(TILT) * (x - seat_x)

    head_back, head_front, head_bottom, head_half = 0.62, 0.71, 0.085, 0.035
    side = [
        (head_back, -head_half, head_bottom),
        (head_back, -head_half, roof(head_back)),
        (head_front, -head_half, roof(head_front)),
        (head_front, -head_half, head_bottom),
    ]
    prism(f"{seen}/head", side, (0, 2 * head_half, 0), blue)
    box(
        f"{felt}/head",
        ((head_back + head_front) / 2, 0, (head_bottom + roof(head_front)) / 2),
        (head_front - head_back, 2 * head_half, roof(head_front) - head_bottom),
    )

    # Along the shaft, from the bit up: the bearing tube, the coupling into
    # the head, and the NEMA 17 motor on the head's top.
    shaft(f"{seen}/shaft")
    cylinder(f"{seen}/shaft/tube", (0, 0, 0.265), 0.016, 0.19, black)
    cylinder(f"{seen}/shaft/coupling", (0, 0, 0.3825), 0.012, 0.045, steel)
    box(f"{seen}/shaft/motor", (0, 0, seat + 0.024), (0.0423, 0.0423, 0.048), black)
    shaft(f"{felt}/shaft")
    cylinder(f"{felt}/shaft/tube", (0, 0, 0.2875), 0.016, 0.235)

    # The L515, a puck 61 mm across and 26 mm thick, its lens face down,
    # straight above the bit, on a printed arm from the head's front.
    lens = TIP + (0, 0, LENS)
    puck = lens + (0, 0, 0.013)
    cylinder(f"{seen}/l515", puck, 0.0305, 0.026, white)
    cylinder(f"{felt}/l515", puck, 0.0305, 0.026)
    start, end = np.array([head_front, 0, 0.13]), lens + (0, 0, 0.026)
    reach = Gf.Rotation(Gf.Vec3d(0, 0, 1), Gf.Vec3d(*(end - start)))
    length = float(np.linalg.norm(end - start))
    box(f"{seen}/camera_arm", (start + end) / 2, (0.012, 0.02, length), green, reach)

    # The L515's camera, at its lens, looking down the body's -z with the top
    # of its picture forward, so the shaft comes in from the bottom edge. Its
    # field is the depth camera's, 70 x 55 degrees.
    camera = UsdGeom.Camera.Define(stage, f"{tool}/camera")
    place(camera, lens, Gf.Rotation(Gf.Vec3d(0, 0, 1), -90))
    camera.CreateFocalLengthAttr(10.0)
    camera.CreateHorizontalApertureAttr(20 * math.tan(math.radians(35)))
    camera.CreateVerticalApertureAttr(20 * math.tan(math.radians(27.5)))
    camera.CreateClippingRangeAttr(Gf.Vec2f(0.01, 100.0))

    mount = UsdPhysics.FixedJoint.Define(stage, "/spot/body/tool_mount")
    mount.CreateBody0Rel().SetTargets(["/spot/body"])
    mount.CreateBody1Rel().SetTargets([tool])
    spindle = UsdPhysics.RevoluteJoint.Define(stage, f"{tool}/{SPINDLE}")
    spindle.CreateBody0Rel().SetTargets([tool])
    spindle.CreateBody1Rel().SetTargets([bit])
    spindle.CreateAxisAttr(UsdPhysics.Tokens.z)
    spindle.CreateLocalPos0Attr(Gf.Vec3f(*TIP))
    spindle.CreateLocalRot0Attr(Gf.Quatf(lean.GetQuat()))
    drive = UsdPhysics.DriveAPI.Apply(spindle.GetPrim(), UsdPhysics.Tokens.angular)
    drive.CreateTypeAttr(UsdPhysics.Tokens.force)
    drive.CreateStiffnessAttr(0.0)
    # USD's angular drives work in degrees.
    drive.CreateDampingAttr(SPINDLE_DAMPING * math.pi / 180)
    drive.CreateMaxForceAttr(SPINDLE_TORQUE)

    # Isaac Sim's robot schema lists a robot's links and joints, each with
    # its API applied; the tool's go after Spot's.
    for name, api, targets in (
        ("robotLinks", "IsaacLinkAPI", [tool, bit]),
        ("robotJoints", "IsaacJointAPI", [spindle.GetPath()]),
    ):
        listed = robot.CreateRelationship(f"isaac:physics:{name}", custom=False)
        for target in targets:
            listed.AddTarget(target, Usd.ListPositionBackOfAppendList)
            stage.GetPrimAtPath(target).AddAppliedSchema(api)

    # Spot goes in last, onto a copy no stage composes: usd-core cannot open
    # a URL, and only Isaac Lab's resolver fetches it.
    layer = Sdf.Layer.CreateAnonymous(".usda")
    layer.TransferContent(stage.GetRootLayer())
    layer.documentation = (
        "Boston Dynamics Spot with the weeding tool of Beumer et al., arXiv 2609.20048, "
        "written by examples/spot_weeder/spot_weeder.py of github.com/azazdeaz/misina-lab"
    )
    layer.GetPrimAtPath("/spot/body").specifier = Sdf.SpecifierOver
    layer.GetPrimAtPath("/spot").referenceList.Prepend(Sdf.Reference(SPOT_URL))
    folder.mkdir(parents=True, exist_ok=True)
    path = folder / FILE
    layer.Export(str(path))
    return path


if __name__ == "__main__":
    print(write_usd(pathlib.Path(sys.argv[1])))

"""Bend and cut the greenhouse's lowest trusses in Isaac Sim.

One short row, its trusses below `BELOW` rods that Newton's VBD bends: they
settle under their fruit's weight, a capsule standing on the floor sweeps
down each side of the row through them, and once they are back at rest every
one is cut at its first joint, by its stem, and drops to the floor.

    uv run bend.py              # in Kit's window, running on once it is done
    uv run bend.py --viz none   # without one, printing what happened

The capsules stand in for a robot: kinematic bodies whose pose is written
every step. Nothing else in the scene moves.
"""

import argparse
import re

import isaaclab.sim as sim_utils
import numpy as np
import warp as wp
from isaaclab.app import add_launcher_args, launch_simulation
from isaaclab.assets import AssetBaseCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim.schemas import UsdPhysicsCollisionCfg, UsdPhysicsRigidBodyCfg
from isaaclab.utils import instantiate
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
from isaaclab_newton.physics import NewtonCfg, VBDSolverCfg

from greenhouse_lab.isaaclab import GreenhouseCfg, LayoutCfg, StemCfg
from misina_lab.isaaclab.cutting import Shears

BELOW = 1.8
"""Height, in meters, below which a truss's node puts it on a rod."""

GREENHOUSE_CFG = GreenhouseCfg(
    layout=LayoutCfg(rows=1, row_length=3.0),
    stem=StemCfg(flexible_below=BELOW),
)

# Four substeps of the 1.25 ms `rods.tune_rods`' numbers were measured at.
SIM_DT = 0.005
SUBSTEPS = 4

PUSHER = "/World/Pusher_{}"
RADIUS = 0.04
SPEED = 0.3
"""How fast a capsule sweeps along the row, in m/s."""
OVERRUN = 0.3
"""How far past the first and last truss a sweep starts and ends, in meters."""

SETTLE = 2.0
RECOVER = 2.0
FALL = 3.0
"""Seconds held still: before the sweep, after it, and after the cut."""
FELL = 0.2
"""How far, in meters, every body of a cut truss has dropped once it fell."""

# A rod body's label ends in its segment's prim; segment 0 is the held one.
SEGMENT = re.compile(r"(.*)/Cable_edge_body_(\d+)$")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Bends and cuts a greenhouse's lowest trusses.")
    add_launcher_args(parser)
    parser.set_defaults(visualizer=["kit"])
    return parser.parse_args()


def design_scene() -> None:
    """The greenhouse, a sky, and a capsule for each side of the row.

    Declared as a scene, since Newton steps only what a scene declares.
    """
    scene = InteractiveSceneCfg(num_envs=1, env_spacing=0.0)
    scene.light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(
            intensity=750.0,
            texture_file=f"{ISAAC_NUCLEUS_DIR}/Materials/Textures/Skies/PolyHaven/kloofendal_43d_clear_puresky_4k.hdr",
        ),
    )
    scene.greenhouse = AssetBaseCfg(prim_path="/World/Greenhouse", spawn=GREENHOUSE_CFG)
    # Standing on the floor and as tall as the band that bends. They wait
    # off the row's end until the rods are there to find the lanes by.
    capsule = sim_utils.CapsuleCfg(
        radius=RADIUS,
        height=BELOW - 2 * RADIUS,
        rigid_props=UsdPhysicsRigidBodyCfg(kinematic_enabled=True),
        collision_props=UsdPhysicsCollisionCfg(collision_enabled=True),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.8, 0.25, 0.2)),
    )
    for side in range(2):
        pusher = AssetBaseCfg(
            prim_path=PUSHER.format(side),
            spawn=capsule,
            init_state=AssetBaseCfg.InitialStateCfg(pos=(-10.0, 2.0 * side, BELOW / 2)),
        )
        setattr(scene, f"pusher_{side}", pusher)
    instantiate(scene)


def trusses(labels: list[str]) -> dict[str, list[int]]:
    """Every rod's bodies by model index, from the stem out, keyed by the
    truss's prim."""
    chains: dict[str, list[tuple[int, int]]] = {}
    for body, label in enumerate(labels):
        if match := SEGMENT.match(label):
            chains.setdefault(match[1], []).append((int(match[2]), body))
    return {truss: [body for _, body in sorted(chain)] for truss, chain in chains.items()}


def main():
    args = parse_args()
    sim_cfg = sim_utils.SimulationCfg(
        dt=SIM_DT,
        physics=NewtonCfg(
            solver_cfg=VBDSolverCfg(rigid_compliant_alm=False), num_substeps=SUBSTEPS
        ),
    )
    with launch_simulation(sim_cfg, args):
        from isaaclab_newton.physics import NewtonManager

        sim = sim_utils.SimulationContext(sim_cfg)
        design_scene()
        sim.reset()

        shears = Shears()
        chains = trusses(shears.labels)
        moving = [body for chain in chains.values() for body in chain[1:]]

        def positions() -> np.ndarray:
            return NewtonManager.get_state_0().body_q.numpy()[:, :3]

        # The lanes the trusses hang in, one either side of the stems.
        rest = positions()
        stems = np.median([rest[chain[0], 1] for chain in chains.values()])
        hanging = rest[moving]
        lanes = [
            np.median(hanging[side, 1]) for side in (hanging[:, 1] > stems, hanging[:, 1] <= stems)
        ]
        start, end = hanging[:, 0].min() - OVERRUN, hanging[:, 0].max() + OVERRUN
        pushers = [shears.labels.index(PUSHER.format(side)) for side in range(2)]

        def place(x: float, speed: float) -> None:
            """Write both capsules' pose and velocity into every state a
            substep reads; VBD never moves a kinematic body itself."""
            for body, y in zip(pushers, lanes, strict=True):
                pose = wp.transform((x, y, BELOW / 2), wp.quat_identity())
                # Linear, then angular.
                velocity = wp.spatial_vector(speed, 0.0, 0.0, 0.0, 0.0, 0.0)
                for state in (NewtonManager.get_state_0(), NewtonManager.get_state_1()):
                    state.body_q[body : body + 1].fill_(pose)
                    state.body_qd[body : body + 1].fill_(velocity)

        def run(seconds: float, x=None, speed: float = 0.0) -> np.ndarray:
            """Step for `seconds`, the capsules at `x` of the time since this
            started, and return the moving bodies' farthest reach from where
            they started it."""
            origin, reach = positions()[moving], np.zeros(len(moving))
            for step in range(round(seconds / SIM_DT)):
                if not sim.is_running():
                    break
                if x is not None:
                    place(x(step * SIM_DT), speed)
                sim.step()
                reach = np.maximum(reach, np.linalg.norm(positions()[moving] - origin, axis=1))
            return reach

        middle = (start + end) / 2
        sim.set_camera_view(eye=[middle - 1.2, lanes[0] + 2.0, 1.6], target=[middle, stems, 1.0])
        place(start, 0.0)
        print(f"[INFO]: {len(chains)} trusses bend, {len(moving)} bodies", flush=True)
        run(SETTLE)
        settled = positions()[moving]
        pushed = run((end - start) / SPEED, lambda t: start + SPEED * t, SPEED)
        run(RECOVER, lambda _: end)
        back = np.linalg.norm(positions()[moving] - settled, axis=1)
        print(
            f"[INFO]: swept: {np.count_nonzero(pushed > 0.01)} of {len(moving)} bodies pushed "
            f"over 1 cm, up to {pushed.max() * 100:.1f} cm; back within {back.max() * 1000:.1f} mm",
            flush=True,
        )
        # Flush at the stem: the first joint, which `Cut` sits by.
        cuts = sum(shears.cut(chain[1], 0.0) for chain in chains.values())
        run(FALL, lambda _: end)
        drop = rest[:, 2] - positions()[:, 2]
        fell = sum(drop[chain[1:]].min() > FELL for chain in chains.values())
        print(f"[INFO]: cut {cuts} of {len(chains)}; {fell} fell", flush=True)
        while sim.is_running() and sim.has_gui:
            sim.step()


if __name__ == "__main__":
    main()

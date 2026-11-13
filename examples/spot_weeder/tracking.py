"""Prints how closely `pose.Controller` holds the body poses it is sent,
beside the paper's Table 1, which measured Spot's own controller over 75
poses, and how far the feet creep over the run. With `workspace`, it prints
the area where the controller holds the bit over the ground: our E, beside
the paper's E and K.

    python tracking.py              # on flat ground
    python tracking.py soft         # on the soft ground; `raised` stands two feet on blocks
    python tracking.py workspace    # our E, on flat ground

Two robots are sent the same 75 random poses, each putting the bit over a
random point of E inside K, 2 to 10 cm over the ground, where the robot
keeps its balance (`pose.Controller.balanced`). One is sent them as they
are, the other with `pose.SpotLike`'s errors. After each move, both make
a small correction, 2 cm further the way the bit went, as the paper's
pipeline corrects after it finds the weed again. Errors are measured after
each move, once the body has held still for `HOLD`.
"""

import sys

import isaaclab.sim as sim_utils
import numpy as np
import pose
import spot_weeder
from isaaclab.app import launch_simulation
from spot_weeder import E, inside, reachable

POSES = 75

HOLD = 1.0
"""Seconds the body holds still after a move before it is measured: the
paper's pipeline stands still about 2 s for each detection (its Table 5)."""

NUDGE = 0.02
"""How far, at the bit, the correction after each move goes."""

HELD = 0.005
"""How close a held body is to its pose: 5 mm and 5 mrad, which keep the
bit within about 1 cm of its point, the paper's 11 mm, half the bit."""

DOFS = ["x, mm", "y, mm", "z, mm", "roll, mrad", "pitch, mrad", "yaw, mrad"]


def body_pose(point) -> np.ndarray:
    """The pose that puts the bit on `point`, by the paper's Eqs. 2-4."""
    x, pitch, yaw = spot_weeder.pose_for(point)
    return np.array([x, 0.0, 0.0, 0.0, pitch, yaw])


def sequence(rng: np.random.Generator, balanced) -> list[tuple[np.ndarray, np.ndarray]]:
    """`POSES` pairs of random points in E and K, 2 to 10 cm over the ground,
    where `balanced(point)`: each a point and its correction, `NUDGE` further
    the way the bit went from the last correction, starting from where it
    stands at rest."""
    low, high = (*E.min(axis=0), 0.02), (*E.max(axis=0), 0.1)
    last = spot_weeder.fk()
    pairs = []
    while len(pairs) < POSES:
        point = rng.uniform(low, high)
        way = (point - last) * (1, 1, 0)
        nudged = point + NUDGE * way / np.linalg.norm(way)
        if all(inside(E, *p[:2]) and reachable(*p) and balanced(p) for p in (point, nudged)):
            pairs.append((point, nudged))
            last = nudged
    return pairs


def launch(ground: str, num_envs: int):
    """The simulation and the scene, standing still, and a controller for it."""
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=pose.SIM_DT, physics=pose.PHYSICS))
    scene = pose.design_scene(ground, num_envs)
    robot = scene["robot"]
    sim.reset()
    robot.write_joint_position_to_sim_index(position=robot.data.default_joint_pos.torch.clone())
    controller = pose.Controller(
        robot, scene.env_origins, pose.RAISED if ground == "raised" else None
    )
    pose.run(sim, scene, controller, HOLD)
    return sim, scene, controller


def track(ground: str) -> None:
    """Table 1 on `ground`, with the corrections and the feet's creep."""
    sim, scene, controller = launch(ground, 2)
    robot = scene["robot"]
    feet, names = robot.find_bodies(".*_foot")
    planted = robot.data.body_pos_w.torch[:, feet].cpu().numpy()
    spot_like = pose.SpotLike()

    def balanced(point) -> bool:
        sent = body_pose(point)
        return controller.balanced(np.stack([sent, sent + spot_like(sent)])).all()

    moved, nudged = [], []
    for point, further in sequence(np.random.default_rng(0), balanced):
        for target, seconds, errors in ((point, pose.MOVE, moved), (further, 1.0, nudged)):
            sent = body_pose(target)
            controller.move(np.stack([sent, sent + spot_like(sent)]), seconds)
            pose.run(sim, scene, controller, seconds + HOLD)
            errors.append(controller.pose() - sent)
    # Back where it started: Spot's ball feet roll as its shins tilt, so they
    # are compared in the same pose.
    controller.move(np.zeros(6))
    pose.run(sim, scene, controller, pose.MOVE + HOLD)
    creep = (robot.data.body_pos_w.torch[:, feet].cpu().numpy() - planted) * 1000
    moved, nudged = np.array(moved) * 1000, np.array(nudged) * 1000

    paper = np.stack([pose.SpotLike.MEAN, pose.SpotLike.STD], axis=-1) * 1000
    print(f"\nTracking error over {POSES} poses on {ground} ground, mean ± standard deviation:\n")
    print(f"{'':13}{'ours':>17}{'ours, Spot-like':>21}{'paper (Table 1)':>21}")
    for i, dof in enumerate(DOFS):
        cells = [f"{m[:, i].mean():.2f} ± {m[:, i].std():.2f}" for m in moved.transpose(1, 0, 2)]
        cells.append(f"{paper[i, 0]:.2f} ± {paper[i, 1]:.2f}")
        print(f"{dof:13}{cells[0]:>17}{cells[1]:>21}{cells[2]:>21}")
    change = np.sqrt(((nudged - moved) ** 2).mean(axis=0))[1]
    spread = moved.std(axis=0)[1]
    print(f"\nSpot-like errors from a move to its {NUDGE * 100:.0f} cm correction:\n")
    print(f"{'':13}{'spread':>10}{'change':>10}")
    for i, dof in enumerate(DOFS):
        print(f"{dof:13}{spread[i]:>10.2f}{change[i]:>10.2f}")
    print(
        "\nWhere the feet stand after the run, back in the pose they started in, from where they\nstood, x and y in mm:\n"
    )
    for env, label in enumerate(("ours", "ours, Spot-like")):
        cells = [
            f"{name[:2]} ({dx:+.1f}, {dy:+.1f})" for name, (dx, dy, _) in zip(names, creep[env])
        ]
        print(f"{label:17}" + "   ".join(cells))


def workspace(step: float = 0.02, height: float = 0.01) -> None:
    """Our E: where the controller holds the body within `HELD` of the pose
    that puts the bit on a point `height` over flat ground, on a grid of
    `step`, one robot per point."""
    x, y = np.meshgrid(np.arange(0.2, 1.2, step), np.arange(-0.8, 0.8, step))
    k = reachable(x, y, height)
    points = np.stack([x[k], y[k], np.full(k.sum(), height)], axis=-1)
    sim, scene, controller = launch("flat", len(points))
    sent = np.stack([body_pose(p) for p in points])
    controller.move(sent)
    pose.run(sim, scene, controller, pose.MOVE + HOLD)
    held = np.abs(controller.pose() - sent).max(axis=1) < HELD
    in_e = inside(E, points[:, 0], points[:, 1])
    area = step**2
    print(f"\nWhere the bit is held on a point {height * 100:.0f} cm up, m2:\n")
    # The paper's K and E are at the ground; the bit is held 1 cm over it
    # here, clear of the ground's contact.
    up = f"{height * 100:.0f} cm up"
    print(f"{'K, ' + up:18}{k.sum() * area:.3f}")
    print(f"{'K at the ground':18}{reachable(x, y).sum() * area:.3f}   (the paper's 0.402)")
    print(f"{'E, ' + up:18}{held.sum() * area:.3f}   (the paper's 0.165, at the ground)")
    print(f"the paper's E held here: {(held & in_e).sum() / in_e.sum():.0%} of its part inside K")
    # The map: forward to the right, the robot's left up.
    grid = np.full(x.shape, " ")
    grid[k] = np.where(held, np.where(in_e, "#", "+"), np.where(in_e, "x", "."))
    print(
        f"\nx from {x[0, 0]:.2f} m to the right, y from {y[-1, 0]:.2f} m down, in {step * 100:.0f} cm squares:"
    )
    print("# held, in the paper's E   + held   x missed, in the paper's E   . missed\n")
    print("\n".join("".join(row).rstrip() for row in grid[::-1]).strip("\n"))


def main():
    what = sys.argv[1] if len(sys.argv) > 1 else "flat"
    if what not in (*pose.GROUNDS, "workspace"):
        sys.exit(__doc__)
    with launch_simulation(
        sim_utils.SimulationCfg(physics=pose.PHYSICS), {"headless": True, "visualizer": []}
    ):
        workspace() if what == "workspace" else track(what)


if __name__ == "__main__":
    main()

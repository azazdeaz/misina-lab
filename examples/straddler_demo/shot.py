"""Render a still of the demo for the docs, from a shot file: the straddling
robot driving down a row, in Isaac Sim's RTX renderer.

    uv run shot.py SHOT.json OUT.png    # render it, headless
    uv run shot.py SHOT.json            # open it in Kit, to frame it by hand

`docs/shots/render.sh` runs the first for every shot in
`docs/shots/straddler_demo`. A shot is one of `misina_lab.isaaclab.shots`,
whose docstring is its reference, its `set` changing the demo's
`VINEYARD_CFG`, with fields of its own: `at` and `warmup`, the moment of the
picture (`shots.Moments`). What the robot is doing is `waypoint`, the index
of the waypoint it drives to, `route.STRIDE` apart along the route from its
start, out in the headland before row `ROW`; the last leg, back to that
start, is the route's length.
"""

import isaaclab.sim as sim_utils
from isaaclab.app import launch_simulation

from misina_lab.isaaclab import shots

import main as demo
from driver import SIM_DT
from straddler import Straddler

ROW = 3
"""The row the robot sets off down: the middle of the vineyard's seven, so
there are vines either side of it."""


def doing(steps, driver):
    """What the robot is doing at each step of the demo's `steps`: the
    waypoint `driver` drives to, or the route's length on the last leg, back
    to the first. It ends once the route is driven round, every row driven,
    and the driver moves on from the first waypoint again."""
    length, left, back = len(driver.waypoints), False, False
    for _ in steps:
        left = left or driver.index > 0
        back = back or (left and driver.index == 0)
        if back and driver.index > 0:
            return
        yield {"waypoint": length if back else driver.index}


def main():
    args = shots.arguments("Renders a still of the straddler demo from a shot file.")
    shot = shots.load(args.shot, {"at", "warmup"})
    vineyard = demo.VINEYARD_CFG
    shots.configure(vineyard, shot.get("set", {}))
    sim_cfg = demo.simulation_cfg(vineyard, "cuda:0")
    with launch_simulation(sim_cfg, shots.launcher(args.out)):
        sim = sim_utils.SimulationContext(sim_cfg)
        machine = Straddler.for_vineyard(vineyard)
        robot = demo.design_scene(vineyard, machine)
        camera = shots.sensor(shot) if args.out else None
        driver, _ = demo.start(sim, robot, machine, vineyard, ROW)
        steps = demo.run_simulator(sim, robot, driver, render=camera is None)
        moments = shots.Moments(doing(steps, driver), SIM_DT)
        moments.reach(shot)
        shots.finish(sim, shot, driver.ground, camera, args.out, moments)


if __name__ == "__main__":
    main()

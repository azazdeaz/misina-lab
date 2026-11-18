"""Render a still of the demo for the docs, from a shot file: the robot
pruning a dormant row, in Isaac Sim's RTX renderer.

    uv run shot.py SHOT.json OUT.png    # render it, headless
    uv run shot.py SHOT.json            # open it in Kit, to frame it by hand

`docs/shots/render.sh` runs the first for every shot in
`docs/shots/pruning_demo`. A shot is one of `misina_lab.isaaclab.shots`,
whose docstring is its reference, its `set` changing the demo's
`VINEYARD_CFG`, with fields of its own: `at` and `warmup`, the moment of the
picture (`shots.Moments`). What the robot is doing is `cut`, the cut it
works, counted from 1 over the run, and `stage`, `Pruning.stage_name`, or
`drive` between vines: the third cut's blade closing is `{"cut": 3,
"stage": "close"}`.
"""

import isaaclab.sim as sim_utils
from isaaclab.app import launch_simulation

from misina_lab.isaaclab import shots

import main as demo
from bumblebee import Bumblebee
from driver import SIM_DT

ROW = 1
"""The row pruned: the middle one, as the demo prunes by default."""


def doing(steps):
    """What the robot is doing at each step of the demo's `steps`: the cut it
    works, counted from 1, and the stage of that cut."""
    cut, working = 0, None
    for pruning in steps:
        if pruning and pruning.cut is not None and pruning.cut is not working:
            cut, working = cut + 1, pruning.cut
        yield {"cut": cut, "stage": pruning.stage_name if pruning else "drive"}


def main():
    args = shots.arguments("Renders a still of the pruning demo from a shot file.")
    shot = shots.load(args.shot, {"at", "warmup"})
    shots.configure(demo.VINEYARD_CFG, shot.get("set", {}))
    sim_cfg = demo.simulation_cfg("cuda:0")
    with launch_simulation(sim_cfg, shots.launcher(args.out)):
        sim = sim_utils.SimulationContext(sim_cfg)
        machine = Bumblebee()
        robot = demo.design_scene(machine)
        camera = shots.sensor(shot) if args.out else None
        driver, vines, to_row, shears = demo.start(sim, robot, machine, ROW)
        steps = demo.run_simulator(
            sim, robot, machine, driver, vines, to_row, shears, render=camera is None
        )
        moments = shots.Moments(doing(steps), SIM_DT)
        moments.reach(shot)
        shots.finish(sim, shot, driver.ground, camera, args.out, moments)


if __name__ == "__main__":
    main()

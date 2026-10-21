"""Render a still of the demo for the docs, from a shot file: the quadruped
walking the vineyard, in Isaac Sim's RTX renderer.

    uv run shot.py SHOT.json OUT.png    # render it, headless
    uv run shot.py SHOT.json            # open it in Kit, to frame it by hand

`docs/shots/render.sh` runs the first for every shot in
`docs/shots/isaaclab_demo`. A shot is one of `misina_lab.isaaclab.shots`,
whose docstring is its reference, its `set` changing the demo's
`VINEYARD_CFG`, with two fields of its own:

- `waypoint`: the waypoint of the alley route the robot is placed on.
- `warmup`: seconds the robot walks on from there before the picture.
"""

import isaaclab.sim as sim_utils
from isaaclab.app import launch_simulation

from misina_lab.isaaclab import shots
from vinery_lab.isaaclab.newton_patches import fix_heightfield_offsets
from vinery_lab.isaaclab.vineyard import resolve_usd_path

import main as demo
from driver import Driver
from route import alley_route


def main():
    args = shots.arguments("Renders a still of the demo from a shot file.")
    shot = shots.load(args.shot, {"waypoint", "warmup"})
    shots.configure(demo.VINEYARD_CFG, shot.get("set", {}))
    sim_cfg = demo.simulation_cfg("cuda:0")
    with launch_simulation(sim_cfg, shots.launcher(args.out)):
        # Imported once Kit is up: they bring `pxr`, and Kit's copy has to win.
        from pxr import Usd

        from vinery_lab.usd import Ground

        sim = sim_utils.SimulationContext(sim_cfg)
        route = alley_route(demo.VINEYARD_CFG)
        ground = Ground(Usd.Stage.Open(resolve_usd_path(demo.VINEYARD_CFG)))
        robot = demo.design_scene()
        camera = shots.sensor(shot) if args.out else None
        sim.reset()
        fix_heightfield_offsets()
        driver = Driver(route, robot)
        driver.index = shot["waypoint"]
        driver.place()
        demo.run_simulator(sim, robot, driver, shot["warmup"], render=camera is None)
        x, y, _ = robot.data.root_pos_w.torch[0].tolist()
        print(f"[INFO]: after the warmup the robot is at ({x:.2f}, {y:.2f})", flush=True)
        shots.finish(sim, shot, ground, camera, args.out)


if __name__ == "__main__":
    main()

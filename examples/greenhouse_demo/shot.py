"""Render a still of a generated greenhouse for the docs, from a shot file:
the greenhouse alone, on bare ground under a sky, or the harvester at work in
it, in Isaac Sim's RTX renderer.

    uv run shot.py SHOT.json OUT.png    # render it, headless
    uv run shot.py SHOT.json            # open it in Kit, to frame it by hand

`docs/shots/render.sh` runs the first for every shot in
`docs/shots/greenhouse_demo`. A shot is one of `misina_lab.isaaclab.shots`,
whose docstring is its reference, its `set` changing a default
`GreenhouseCfg`, with two fields of its own:

- `harvest`: the paths the harvester works, as `harvest.py --paths` takes
  them. With it the scene is `harvest.py`'s, and `set` changes its cfg.
- `warmup`: seconds of simulated time it works before the picture.
"""

import isaaclab.sim as sim_utils
from isaaclab.app import launch_simulation
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

from greenhouse_lab.isaaclab import GreenhouseCfg
from misina_lab.isaaclab import shots

import harvest

GREENHOUSE_CFG = GreenhouseCfg()


def main():
    args = shots.arguments("Renders a still of a generated greenhouse from a shot file.")
    shot = shots.load(args.shot, {"harvest", "warmup"})
    if "harvest" in shot:
        harvesting(shot, args.out)
        return
    shots.configure(GREENHOUSE_CFG, shot.get("set", {}))
    # Nothing moves, so any physics will do: Isaac Lab's default.
    sim_cfg = sim_utils.SimulationCfg()
    with launch_simulation(sim_cfg, shots.launcher(args.out)):
        sim = sim_utils.SimulationContext(sim_cfg)
        # The sky lights the scene; the building around it is left out, as
        # the generator leaves it.
        sky = sim_utils.DomeLightCfg(
            intensity=750.0,
            texture_file=f"{ISAAC_NUCLEUS_DIR}/Materials/Textures/Skies/PolyHaven/kloofendal_43d_clear_puresky_4k.hdr",
        )
        sky.func("/World/Light", sky)
        # Bare ground around the floor, its top a centimeter under the
        # floor's: past the floor's edge a camera sees ground, not the
        # underside of the sky.
        ground = sim_utils.CuboidCfg(
            size=(200.0, 200.0, 0.1),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.3, 0.27, 0.22)),
        )
        ground.func("/World/Ground", ground, translation=(0.0, 0.0, -0.06))
        GREENHOUSE_CFG.func("/World/Greenhouse", GREENHOUSE_CFG)
        camera = shots.sensor(shot) if args.out else None
        sim.reset()
        # The floor's top is z = 0, with no terrain to read a height off.
        shots.finish(sim, shot, None, camera, args.out)


def harvesting(shot: dict, out) -> None:
    """The still of a shot with `harvest`: `harvest.py`'s scene, `warmup`
    seconds into the harvest."""
    shots.configure(harvest.GREENHOUSE_CFG, shot.get("set", {}))
    sim_cfg = harvest.simulation_cfg()
    with launch_simulation(sim_cfg, shots.launcher(out)):
        sim = sim_utils.SimulationContext(sim_cfg)
        machine = harvest.make_machine()
        robot = harvest.design_scene(machine)
        camera = shots.sensor(shot) if out else None
        run = harvest.run(sim, robot, machine, shot["harvest"], render=camera is None)
        for step, _ in enumerate(run):
            if step * harvest.SIM_DT >= shot["warmup"]:
                break
        shots.finish(sim, shot, None, camera, out)


if __name__ == "__main__":
    main()

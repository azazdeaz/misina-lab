"""Render a still of a generated greenhouse for the docs, from a shot file:
the greenhouse alone, on bare ground under a sky, or the harvester at work in
it, in Isaac Sim's RTX renderer.

    uv run shot.py SHOT.json OUT.png    # render it, headless
    uv run shot.py SHOT.json            # open it in Kit, to frame it by hand

`docs/shots/render.sh` runs the first for every shot in
`docs/shots/greenhouse_demo`. A shot is one of `misina_lab.isaaclab.shots`,
whose docstring is its reference, its `set` changing a default
`GreenhouseCfg`, with fields of its own:

- `harvest`: the paths the harvester works, as `harvest.py --paths` takes
  them. With it the scene is `harvest.py`'s, and `set` changes its cfg.
- `at` and `warmup`, with `harvest`: the moment of the picture
  (`shots.Moments`). What the harvester is doing is `truss`, the truss it
  works, counted from 1 over the run; `try`, `Harvest.tries`, the attempt
  at its cut, from 1 once the first is planned; `severed`, whether that
  truss is cut; and `stage`, `Harvest.stage`: the second truss backed out
  once cut is `{"truss": 2, "severed": true, "stage": "outward"}`.
"""

import isaaclab.sim as sim_utils
from isaaclab.app import launch_simulation
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

from greenhouse_lab.isaaclab import GreenhouseCfg
from misina_lab.isaaclab import rods, shots

import harvest

GREENHOUSE_CFG = GreenhouseCfg()


def main():
    args = shots.arguments("Renders a still of a generated greenhouse from a shot file.")
    shot = shots.load(args.shot, {"harvest", "at", "warmup"})
    if "harvest" not in shot and shot.keys() & {"at", "warmup"}:
        raise ValueError(f"{args.shot}: `at` and `warmup` pick a moment of `harvest`")
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
    """The still of a shot with `harvest`: `harvest.py`'s scene, at the
    shot's moment of the harvest."""
    shots.configure(harvest.GREENHOUSE_CFG, shot.get("set", {}))
    sim_cfg = harvest.simulation_cfg()
    with launch_simulation(sim_cfg, shots.launcher(out)):
        rods.widen_contact_threads()
        sim = sim_utils.SimulationContext(sim_cfg)
        machine = harvest.make_machine()
        robot = harvest.design_scene(machine)
        camera = shots.sensor(shot) if out else None
        run = harvest.run(sim, robot, machine, shot["harvest"], render=camera is None)
        moments = shots.Moments(doing(run), harvest.SIM_DT)
        moments.reach(shot)
        shots.finish(sim, shot, None, camera, out, moments)


def doing(run):
    """What the harvester is doing at each step of `run`: the truss it works,
    counted from 1, the attempt at its cut, whether it is cut, and the
    stage of the job. It ends
    `harvest.FALL` seconds after the last job, as `run` does without a
    window."""
    truss, job, after = 0, None, 0
    for state in run:
        after = after + 1 if state.done else 0
        if after > round(harvest.FALL / harvest.SIM_DT):
            return
        if state.job is not job:
            job = state.job
            truss += isinstance(job, harvest.Truss)
        yield {
            "truss": truss,
            "try": state.tries,
            "severed": state.severed,
            "stage": state.stage,
        }


if __name__ == "__main__":
    main()

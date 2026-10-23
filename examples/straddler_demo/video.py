"""Record a short video of the demo: the robot driving down a row, filmed in
Isaac Sim's RTX renderer from behind and above it. It runs headless.

    uv run video.py OUT.mp4

writes the frames to `OUT.mp4`, near-lossless; `record.sh` runs it and
encodes that into the video to upload. Frames are sampled on the
simulation's clock, one every so many physics steps however long those take
to compute, so the video plays in simulated time.
"""

import argparse
import dataclasses
import pathlib

import numpy as np

import isaaclab.sim as sim_utils
from isaaclab.app import launch_simulation

from misina_lab.isaaclab.film import CAMERA, Film, sensor
from misina_lab.isaaclab.shots import place
from vinery_lab.isaaclab import ShootCfg

import main as demo
from driver import SIM_DT
from straddler import Straddler

SIZE = (1920, 1080)
"""The frames, in pixels. `record.sh` scales them down to the video's size,
which smooths the thin shoots."""

FPS = 25
"""Frames a second of video: a frame is 8 physics steps of simulated time."""

SETTLE = 6.0
"""Seconds of simulated time not filmed at the start, while the robot drops
onto its wheels and drives out of the headland into the row."""

LENGTH = 10.0
"""Seconds of video."""

ROW = 3
"""The row the robot sets off down: the middle of the vineyard's seven, so
there are vines either side of it."""

EYE = (-6.0, 4.0)
"""Where the camera stands, from where the robot stands once `SETTLE` is
over: meters along the way it drives, and up. Behind it and over the vines,
so it drives away down the row."""

LOOK = 10.0
"""How far ahead of the robot the camera looks, along the ground: far enough
that the horizon is in the picture."""

LENS = 18.15
"""Focal length, in millimetres of a 21 mm wide sensor: the 60 degree view
Kit's viewport starts with."""

VINEYARD_CFG = dataclasses.replace(demo.VINEYARD_CFG, shoot=ShootCfg(stray=0.35))
"""The demo's vineyard with a third of its shoots strayed out of the canopy,
so the frame is seen pushing through them all the way down the row."""


def main():
    parser = argparse.ArgumentParser(description="Records the straddler demo's video, headless.")
    parser.add_argument("out", type=pathlib.Path, help="the mp4 to write")
    out = parser.parse_args().out.resolve()
    vineyard = VINEYARD_CFG
    sim_cfg = demo.simulation_cfg(vineyard, "cuda:0")
    # Kit, headless, with nothing to look at the scene but the camera below:
    # its renderer needs Kit's, which a camera made after launch cannot ask for.
    launcher = {"headless": True, "enable_cameras": True, "require_kit": True, "visualizer": []}
    with launch_simulation(sim_cfg, launcher):
        sim = sim_utils.SimulationContext(sim_cfg)
        machine = Straddler.for_vineyard(vineyard)
        robot = demo.design_scene(vineyard, machine)
        camera = sensor(SIZE, LENS)
        driver, route = demo.start(sim, robot, machine, vineyard, ROW)
        steps = demo.run_simulator(sim, robot, driver, render=False)
        for _ in range(round(SETTLE / SIM_DT)):
            next(steps)

        ahead = route[1] - route[0]
        ahead[2] = 0.0
        ahead /= np.linalg.norm(ahead)
        # `base_link`'s origin is the ground under the wheels.
        at = robot.data.root_pos_w.torch[0].cpu().numpy().astype(float)
        eye, target = at + EYE[0] * ahead + [0.0, 0.0, EYE[1]], at + LOOK * ahead
        place(sim_utils.get_current_stage().GetPrimAtPath(CAMERA), eye, target)

        film, every = Film(out, SIZE, FPS), round(1 / (FPS * SIM_DT))
        try:
            for step, _ in enumerate(steps):
                if step % every:
                    continue
                sim.render()
                camera.update(SIM_DT, force_recompute=True)
                film.add(camera.data.output["rgb"].torch[0, ..., :3].cpu().numpy())
                if film.frames >= LENGTH * FPS:
                    break
        finally:
            film.close()
        print(f"[INFO]: {film.frames} frames, {film.frames / FPS:.1f} s, written to {out}")


if __name__ == "__main__":
    main()

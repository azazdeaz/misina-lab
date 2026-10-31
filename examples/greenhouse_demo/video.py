"""Record a short video of the harvest demo: the harvester drives into a path
and cuts its due trusses, the first few filmed close up and the rest by a
camera riding along with the trolley, in Isaac Sim's RTX renderer by a
camera this script moves. It runs headless.

    uv run video.py OUT.mp4

writes the frames to `OUT.mp4`, near-lossless, and the captions beside it as
`OUT.srt`, then fails if a take's truss was not cut. `record.sh` runs it and
encodes the two into the video to upload.

The storyboard is the constants below: the shots, the `TAKES` filmed close
up, and `PACE`, how fast each part plays. Frames are sampled on the
simulation's clock, one every so many physics steps however long those take
to compute, so the video plays in simulated time.
"""

import argparse
import dataclasses
import pathlib

import numpy as np

import isaaclab.sim as sim_utils
from isaaclab.app import launch_simulation

import harvest as demo
from misina_lab.isaaclab.film import Film, Rig, sensor

SIZE = (1920, 1080)
"""The frames, in pixels. `record.sh` scales them down to the video's size."""

FPS = 25
"""Frames a second of video: a frame is 8 physics steps of simulated time, at
the simulation's own speed."""

PATHS = [1]
"""The path harvested: Path_01, between the two rows, straight down the rail
from where the trolley starts."""

SETTLE = 1.5
"""Seconds of simulated time not filmed at the start, while the trusses
settle under their fruit."""


@dataclasses.dataclass(frozen=True)
class Shot:
    """Where the camera stands and what it looks at, relative to a point it
    frames: in meters along the path the way the trolley drives in, across it
    toward the truss cut, or the left-hand row, and up."""

    eye: tuple[float, float, float]
    look: tuple[float, float, float] = (0.0, 0.0, 0.0)
    """Where it aims, from the point."""
    lens: float = 24.0
    """Focal length, in millimetres of a 21 mm wide sensor: 24 is a 47 degree
    view across, 50 a 24 degree one."""
    f_stop: float = 0.0
    """The aperture, focused on where the camera aims; 0 keeps everything
    sharp. The renderer's depth of field is strong for the number."""
    caption: str | None = None


CLOSE = Shot(eye=(0.25, -0.95, 0.35), look=(0.0, 0.0, -0.08), lens=20.0)
"""The cuts' close-up: from the path a little ahead of the cut and above
it, so the shear comes in from behind it, toward the camera, and the truss
hangs below it in frame. Half a meter ahead, the lift column stands between
it and a cut the arm reaches back for."""

CRATE = Shot(eye=(0.0, -0.45, 0.8), look=(0.0, 0.0, 0.1), lens=18.0)
"""The crate, from above the path across from the side the truss is cut on,
as the arm carries the truss to it and lets it go: the point it frames is
the crate's middle, at its underside."""

TAKES = (
    "A truss is due once its least ripe fruit is turning",
    "The shear comes in along the stem, on a straight line",
    "Cut and held by its stem, the truss is carried to the crate",
)
"""The captions of the cuts filmed close up: the first the trolley makes, in
order."""

OPENING = Shot(
    eye=(-3.2, -2.8, 1.8),
    look=(-0.2, 0.0, 0.8),
    lens=20.0,
    caption="Harvesting tomato trusses with a pipe-rail robot, in Isaac Sim",
)
"""From the main path, behind the trolley and to its right, as it drives
into the path with the arm folded. Points along the path count from its
mouth, where the rows start."""

REST = Shot(
    eye=(1.6, -0.25, 0.7),
    look=(0.0, 0.5, 0.25),
    lens=20.0,
    caption="The rest of the path, four times faster",
)
"""Riding along with the trolley while the cuts no take has are made, and
as the arm swings out to a later take's truss, under its caption: down
the path ahead of the arm, a little across from the row it cuts from and
above the trusses, looking at that row beside the arm. The point it frames
is the platform's top under the arm's base, on the path's centre line: the
lift holds it as far under every cut, so each is framed alike. From behind,
the lift column would hide the arm."""

CLOSING = Shot(
    eye=(1.4, -0.3, 1.2),
    look=(0.0, 0.0, 0.0),
    lens=20.0,
    caption="The path harvested into the crate",
)
"""Riding along with the trolley as it drives out, from above the path
ahead of it, looking down into the crate: the point it frames is the
crate's middle, at its underside."""

LAG = 0.4
"""Seconds of video a shot riding along with the trolley lags it by,
smoothing its runs and its turn from one row to the other."""

PACE = {
    "inward": 2.0,
    "close": 2.0,
    "outward": 1.5,
    "pull": 1.5,
    "open": 3.0,
    "drop": 3.0,
    "rest": 0.25,
}
"""How many times slower than the simulation the video plays: a take's
stages by `Harvest.stage`, the truss settling in the crate once let go, and
the cuts no take has. The rest plays at the simulation's own speed."""

DROP = 0.8
"""Seconds of simulated time a take stays on the crate once the harvest
moves on from its truss, while the truss settles in it."""

EASE = 1.5
"""Seconds of video the camera takes from one shot to the next."""

CUT = 1.5
"""How far the camera may travel from one shot to the next, in meters,
before it cuts to the next rather than easing there. It cuts from one take
to the next too: the arm moves on to the next truss past where the last
take's camera stands."""


def framing(shot: Shot, point: np.ndarray, side: np.ndarray, ahead: float = 1.0) -> np.ndarray:
    """The camera's pose for `shot` of `point`, `side` the horizontal direction
    across the path it counts from and `ahead` the way along the row, +1 or
    -1: eye, aim, focal length and aperture."""
    axes = np.column_stack([[ahead, 0.0, 0.0], side, [0.0, 0.0, 1.0]])
    eye, target = point + axes @ shot.eye, point + axes @ shot.look
    return np.array([*eye, *target, shot.lens, shot.f_stop])


def main():
    parser = argparse.ArgumentParser(description="Records the harvest demo's video, headless.")
    parser.add_argument("out", type=pathlib.Path, help="the mp4 to write; captions go beside it")
    out = parser.parse_args().out.resolve()
    sim_cfg = demo.simulation_cfg()
    # Kit, headless, with nothing to look at the scene but the camera below:
    # its renderer needs Kit's, which a camera made after launch cannot ask for.
    launcher = {"headless": True, "enable_cameras": True, "require_kit": True, "visualizer": []}
    with launch_simulation(sim_cfg, launcher):
        sim = sim_utils.SimulationContext(sim_cfg)
        machine = demo.make_machine()
        robot = demo.design_scene(machine)
        camera = sensor(SIZE)
        stage = sim_utils.get_current_stage()
        y = demo.path_ys(stage, demo.GREENHOUSE_PATH)[PATHS[0]]
        entrance, left = np.array([0.0, y, 0.0]), np.array([0.0, 1.0, 0.0])
        rig, film = Rig(stage, FPS, EASE, CUT), Film(out, SIZE, FPS)
        # The takes' captions, by the prim of the truss each films, the
        # trusses a cut was planned for and those swung round to the crate.
        captions: dict[str, str] = {}
        planned: set[str] = set()
        swung: set[str] = set()
        # Frames due, counted a fraction a physics step: an eighth, at the
        # simulation's own speed. `last` is the truss cut last, let go at
        # `left_at` once the harvest moves on from it, and `shown` the one
        # the last frame filmed. `ridden` is where the last frame riding
        # along stood, `row` the side it was across from, and a frame riding
        # along closes `share` of the gap to its shot.
        due, last, left_at, shown = 0.0, None, None, None
        ridden, row = None, left
        share = 1.0 - np.exp(-1.0 / (LAG * FPS))
        try:
            for step, harvest in enumerate(demo.run(sim, robot, machine, PATHS, render=False)):
                now = step * demo.SIM_DT
                truss = harvest.job if isinstance(harvest.job, demo.Truss) else None
                if truss and harvest.stage == "approach":
                    planned.add(truss.prim)
                # A take's truss out of reach passes its caption on.
                current = truss.prim if truss else None
                for prim in [p for p in captions if p not in planned and p != current]:
                    del captions[prim]
                if truss and truss.prim not in captions and len(captions) < len(TAKES):
                    captions[truss.prim] = TAKES[len(captions)]
                if harvest.cut and harvest.cut[-1] is not last:
                    last, left_at = harvest.cut[-1], None
                if last is not None and left_at is None and harvest.job is not last:
                    left_at = now
                if truss and harvest.stage == "carry":
                    swung.add(truss.prim)
                filmed = None
                gone = left_at is not None and now - left_at < DROP
                carried = harvest.severed and harvest.stage in ("carry", "open", "tip")
                if truss and truss.prim in captions and truss.prim in swung and carried:
                    shot, filmed, pace = CRATE, truss, harvest.stage
                elif last is not None and last.prim in captions and last.prim in swung and gone:
                    shot, filmed, pace = CRATE, last, "drop"
                elif (
                    truss
                    and truss.prim in captions
                    and last is not None
                    and harvest.stage in ("fold", "run", "approach")
                ):
                    # A later take rides along as the arm swings out to its
                    # truss, past where the close-up stands.
                    shot, pace = REST, harvest.stage
                elif (
                    truss
                    and truss.prim in captions
                    and not (last is None and harvest.stage in ("fold", "run"))
                ):
                    # The first take starts as the arm reaches out at the
                    # end of the drive in.
                    shot, filmed, pace = CLOSE, truss, harvest.stage
                elif last is None:
                    shot, pace = OPENING, None
                elif truss or any(isinstance(job, demo.Truss) for job in harvest.jobs):
                    shot, pace = REST, "rest"
                else:
                    shot, pace = CLOSING, None
                due += PACE.get(pace, 1.0) * FPS * demo.SIM_DT
                if now < SETTLE or due < 1:
                    continue
                due -= 1
                riding = shot is REST or shot is CLOSING
                if filmed:
                    # Ahead is the way the shear comes in.
                    way = demo.heading(filmed.cut[:3, 0], filmed.side)
                    point, side, caption = filmed.cut[:3, 3], filmed.side, captions[filmed.prim]
                    if shot is CRATE:
                        point = (harvest.platform() @ machine.crate_frame)[:3, 3]
                    pose = framing(shot, point, side, float(np.sign(way[0])))
                elif riding:
                    # Along with the trolley, across from the row it last cut from.
                    row = truss.side if truss else row
                    platform = harvest.platform()
                    point = np.array([platform[0, 3], y, platform[2, 3]])
                    if shot is CLOSING:
                        point = (platform @ machine.crate_frame)[:3, 3]
                    goal = framing(shot, point, row)
                    caption = captions.get(truss.prim, shot.caption) if truss else shot.caption
                    pose = goal if ridden is None else ridden + share * (goal - ridden)
                else:
                    point, side, caption = entrance, left, shot.caption
                    pose = framing(shot, point, side)
                # A shot riding along eases itself: the rig cuts to each frame of it.
                rig.aim(pose, cut=riding or filmed is not shown)
                shown, ridden = filmed, pose if riding else None
                sim.render()
                camera.update(demo.SIM_DT, force_recompute=True)
                film.add(camera.data.output["rgb"].torch[0, ..., :3].cpu().numpy(), caption)
        finally:
            film.close()
        print(f"[INFO]: {film.frames} frames, {film.frames / FPS:.1f} s, written to {out}")
        missed = captions.keys() - {truss.prim for truss in harvest.cut}
        if len(captions) < len(TAKES) or missed:
            raise RuntimeError(f"of {len(TAKES)} takes, these trusses were not cut: {missed}")


if __name__ == "__main__":
    main()

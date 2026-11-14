"""Weeds `SCENES` scenes of `weeding.WEEDS` weeds each, a robot to a scene,
and prints what became of the weeds beside the paper's Table 4, and how long
a cycle takes beside its Table 5.

    python experiments.py           # on flat ground
    python experiments.py soft      # on the soft ground; `raised` stands two feet on blocks

Each scene is weeded in three ways, on the same weeds: as the controller
tracks; with the errors of Spot's own controller (`pose.SpotLike`) and the
detection's `weeding.NOISE`; and, on flat ground, with the tool mounted 3 cm
forward of where the pipeline takes it to be, the paper's §4.1 check. The
paper's Table 4 comes from its soft ground, a mattress.
"""

import sys

import isaaclab.sim as sim_utils
import numpy as np
import pose
import weeding
from isaaclab.app import launch_simulation

SCENES = 20
"""Scenes weeded in each way: 100 weeds, like the paper's 88."""

OFFSET = 0.03
"""How far forward the tool is mounted in the §4.1 check, in metres."""

LOCOMOTION = 3.151
"""The paper's walk to the next patch, in seconds. No robot walks here: each
scene is a patch of its own."""

FOUND = 88
"""The weeds the paper's Table 4 counts: the ones it found."""

ROWS = (
    ("Removed", (weeding.REMOVED,), "72 (82%)"),
    ("Not removed", (weeding.OUTSIDE, weeding.FAR, weeding.LOST, weeding.MISSED), "16 (18%)"),
    ("  Poor tracking", (weeding.MISSED,), "12 (14%)"),
    ("  No second detection", (weeding.LOST,), "0 (0%)"),
    ("  Outside E", (weeding.OUTSIDE,), "4 (4%)"),
    ("  Out of reach", (weeding.FAR,), ""),
)
"""Table 4's rows: the outcomes each one counts, and the paper's cell as it
prints it. The paper's robot reaches all of E, which is where it reached."""


def report(ground: str, ways: list[str], pipeline: weeding.Pipeline) -> None:
    """Prints Tables 4 and 5, and how close the bit came to the weeds."""
    status = pipeline.status.reshape(len(ways), -1)
    found = (status >= weeding.OUTSIDE).sum(axis=1)
    width = 18
    print(f"\nWeeds of {SCENES} scenes on {ground} ground, of the ones found:\n")
    print(f"{'':24}" + "".join(f"{way:>{width}}" for way in [*ways, "paper (Table 4)"]))
    for label, outcomes, paper in ROWS:
        counts = np.isin(status, outcomes).sum(axis=1)
        cells = [f"{count} ({count / total:.0%})" for count, total in zip(counts, found)]
        cells.append(paper)
        print(f"{label:24}" + "".join(f"{cell:>{width}}" for cell in cells))
    print(f"{'Found':24}" + "".join(f"{total:>{width}}" for total in [*found, FOUND]))
    unseen = (status == weeding.UNSEEN).sum(axis=1)
    print(f"{'Not found':24}" + "".join(f"{count:>{width}}" for count in unseen))
    fell = pipeline.fell.reshape(len(ways), SCENES).sum(axis=1)
    print(f"{'Robots off their feet':24}" + "".join(f"{count:>{width}}" for count in fell))

    scan = weeding.REACH + weeding.DETECT
    movement = weeding.REACH + weeding.CORRECT
    per_weed = movement + weeding.DETECT + weeding.MILL
    print("\nSeconds for each part of a cycle:\n")
    print(f"{'':32}{'ours':>8}{'paper (Table 5)':>{width}}")
    for label, ours, paper in (
        ("Removal planner (2 a cycle)", scan, 2.384),
        ("Weed removal (1 a weed)", per_weed, 7.789),
        ("  Movement", movement, 4.795),
        ("  Detection", weeding.DETECT, 1.994),
        ("  Removal", weeding.MILL, 1.0),
        ("Locomotion (1 a cycle)", LOCOMOTION, 3.151),
    ):
        print(f"{label:32}{ours:>8.3f}{paper:>{width}.3f}")
    print("\nOur removal planner's time is the move to the scan and the detection there.")
    print(f"Our cycle also stands up again before the walk: {weeding.REACH:.1f} s.")
    aimed = np.array([len(points) for points in pipeline.aimed]).reshape(len(ways), SCENES)
    for way, counts in zip(ways, aimed):
        cycle = 2 * scan + counts.mean() * per_weed + weeding.REACH + LOCOMOTION
        print(f"A cycle, {way}: {cycle:.1f} s, for the {counts.mean():.1f} weeds it aims at")
    print("A cycle in the paper: 47 s, for 5 weeds")

    print("\nHow close the bit's centre came, in the ground, to the weed it aimed at, mm:\n")
    for k, way in enumerate(ways):
        closest = pipeline.closest[k * SCENES : (k + 1) * SCENES]
        mm = np.concatenate([np.array(each, dtype=float) for each in closest]) * 1000
        reached = mm[np.isfinite(mm)]
        print(
            f"{way:16} mean {reached.mean():5.1f}, worst {reached.max():5.1f}"
            f"   ({(~np.isfinite(mm)).sum()} of {len(mm)} kept out of the ground)"
        )
    if ground == "flat":
        print("\nThe paper's §4.1: under 3 mm with the tool 3 cm off.")


def main():
    ground = sys.argv[1] if len(sys.argv) > 1 else "flat"
    if ground not in pose.GROUNDS:
        sys.exit(__doc__)
    ways = ["ideal", "Spot-like"] + (["tool 3 cm off"] if ground == "flat" else [])
    with launch_simulation(
        sim_utils.SimulationCfg(physics=pose.PHYSICS), {"headless": True, "visualizer": []}
    ):
        sim, scene, controller = pose.launch(ground, SCENES * len(ways))
        scenes = [weeding.scatter(np.random.default_rng(seed)) for seed in range(SCENES)]
        way = np.repeat(np.arange(len(ways)), SCENES)
        pipeline = weeding.Pipeline(
            sim,
            scene,
            controller,
            np.array(scenes * len(ways)),
            spot_like=[pose.SpotLike(seed) if w == 1 else None for seed, w in enumerate(way)],
            offset=np.where((way == 2)[:, None], [OFFSET, 0.0, 0.0], 0.0),
            noise=np.where(way == 1, weeding.NOISE, 0.0),
            raised=pose.RAISED if ground == "raised" else None,
        )
        pipeline.run()
        report(ground, ways, pipeline)


if __name__ == "__main__":
    main()

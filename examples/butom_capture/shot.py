"""Render the picture that sets this example's frames beside BUTom-ST21's:
BUTom's along the top, the trolley's cameras' under them, column by column.

    BUTOM=DIR uv run shot.py SHOT.json OUT.png

`docs/shots/render.sh` runs it for every shot in `docs/shots/butom_capture`.
DIR holds BUTom's RGB sequences, a folder each, as its `rgb.tar.gz` unpacks
them. A shot is a JSON object:

- `set`: greenhouse parameters changed from `capture.py`'s, as its `--set`
  takes them. Optional.
- `places`: where the trolley stands for each pair of columns, `row`, `face`
  and `x` as `capture.plan` gives a pass's, and `butom`, the two BUTom frames
  above its `cam_1`'s and `cam_2`'s, each `<seq>/<frame_id>.png` in DIR.
- `scale`: the frames' size in the picture, from their 720 x 1280.
"""

import argparse
import json
import os
import pathlib

import capture
import isaaclab.sim as sim_utils
from isaaclab.app import launch_simulation
from PIL import Image

from misina_lab.isaaclab import shots

GAP = 4
"""Pixels of white between the frames."""


def sheet(top: list[Image.Image], bottom: list[Image.Image], scale: float) -> Image.Image:
    """`top` and `bottom` in two rows, each frame scaled by `scale`."""
    w, h = (round(n * scale) for n in capture.SIZE)
    picture = Image.new("RGB", (len(top) * (w + GAP) - GAP, 2 * h + GAP), "white")
    for i, column in enumerate(zip(top, bottom, strict=True)):
        for j, frame in enumerate(column):
            frame = frame.convert("RGB").resize((w, h), Image.Resampling.LANCZOS)
            picture.paste(frame, (i * (w + GAP), j * (h + GAP)))
    return picture


def main():
    parser = argparse.ArgumentParser(
        description="Renders the picture of this example's frames beside BUTom-ST21's."
    )
    parser.add_argument("shot", type=pathlib.Path, help="the shot, a JSON file")
    parser.add_argument("out", type=pathlib.Path, help="the PNG to write")
    args = parser.parse_args()
    shot = json.loads(args.shot.read_text())
    if "BUTOM" not in os.environ:
        raise SystemExit("BUTOM names no folder: set it to BUTom-ST21's RGB sequences'")
    butom = [
        Image.open(pathlib.Path(os.environ["BUTOM"]) / frame)
        for place in shot["places"]
        for frame in place["butom"]
    ]
    shots.configure(capture.GREENHOUSE_CFG, shot.get("set", {}))
    sim_cfg = sim_utils.SimulationCfg()
    with launch_simulation(sim_cfg, shots.launcher(args.out)):
        sim = sim_utils.SimulationContext(sim_cfg)
        capture.design_scene()
        cameras = [capture.sensor(name) for name in capture.CAMERAS]
        sim.reset()
        pose = capture.trolley()
        rendered = []
        for place in shot["places"]:
            layout = capture.GREENHOUSE_CFG.layout
            capture.stand(pose, layout, place["row"], place["face"], place["x"])
            # The trolley jumped, so the renderer's accumulation starts over.
            for _ in range(shots.SETTLE):
                sim.render()
                for camera in cameras:
                    camera.update(0.0, force_recompute=True)
            for camera in cameras:
                rendered.append(
                    Image.fromarray(camera.data.output["rgb"].torch[0, ..., :3].cpu().numpy())
                )
        sheet(butom, rendered, shot["scale"]).save(args.out)
        print(f"[INFO]: saved {args.out}", flush=True)


if __name__ == "__main__":
    main()

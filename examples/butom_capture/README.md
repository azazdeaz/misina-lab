# BUTom capture

RGB-D sequences like those of
[BUTom-ST21](https://doi.org/10.60507/FK2/TTPCNV), filmed in a
[generated greenhouse](../../crates/greenhouse-lab/README.md): a pipe-rail
trolley laid out like the University of Bonn's PATHoBot drives down a path
at 0.2 m/s, and the two middle cameras on its mast film the crop across the
path, as the real ones did at Campus Klein-Altendorf in 2021. The frames
come out in BUTom's own layout, each with its depth and camera pose, so
tooling written for the real dataset reads them too.

**Contents:** [Requirements](#requirements) · [Running it](#running-it) ·
[What it writes](#what-it-writes) · [How close it is](#how-close-it-is) ·
[Experiments](#experiments) · [Layout](#layout)

## Requirements
 - [uv](https://docs.astral.sh/uv/getting-started/installation/) installed
 - [Isaac Sim 6.2](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/installation/requirements.html) compatible hardware: an RTX GPU, which renders the cameras
 - [Rust](https://www.rust-lang.org/tools/install) installed (this example builds the scene generator from source)

## Running it

```bash
uv run capture.py out
```

generates the greenhouse, caches it, and films two passes of the trolley
into `out/`, headless: about 45 s a pass on an RTX 5070 Ti once the
greenhouse is cached, and about 110 MB a sequence. `--passes` sets how many, `--frames` how long each is, at most
as many as fit a row between its 1 m margins, and `--set`
changes the greenhouse's [parameters](../../crates/greenhouse-lab/docs/parameters.md)
from the script's, each value as JSON:

```bash
uv run capture.py out --passes 10 --set scene.seed=3 scene.week=20 fruit.ripe_trusses=2
```

A run films one greenhouse. Run it again into the same folder with other
parameters, and its sequences number on from the ones there.

## What it writes

```text
out/
  camera_parameters.yaml       each camera's intrinsics, in BUTom's format
  rgb/<seq>/<frame_id>.png     720 x 1280, portrait
  depth/<seq>/<frame_id>.png   uint16 millimetres along the optical axis, 0 where nothing is in range
  sequences.json               per sequence: its camera, the row and face it films,
                               every greenhouse parameter, and each frame's camera pose
```

Each pass makes a sequence per camera, `cam_1` the lower one and `cam_2`
the upper, as in BUTom. A frame id is the frame's time into its pass in
nanoseconds, 15 a second; the two sequences of a pass share them. A pose is
the 4x4 world-from-camera matrix in OpenCV's convention, x right, y down, z
forward, in the greenhouse's frame: z up from the floor's top, x along the
rows from the main path.

## How close it is

![BUTom-ST21's frames above this example's](https://raw.githubusercontent.com/azazdeaz/misina-lab-docs-assets/main/butom_capture/compare.webp)

BUTom-ST21's frames along the top, this example's under them: on the left a
row inside the house, on the right an outer row, with the glass behind it,
here the white wall; each the lower camera's and then the upper's. BUTom's
frames are the University of Bonn's, under
[CC BY 4.0](https://creativecommons.org/licenses/by/4.0/). `shot.py`
renders the picture again, with the dataset's RGB sequences at hand.

| | BUTom-ST21 | Here | From |
| --- | --- | --- | --- |
| Cameras | RealSense D435i, portrait, 720 x 1280 at 15 Hz | the same | BUTom's readme |
| Intrinsics | `camera_parameters.yaml` | the same, principal point included | the dataset |
| Camera heights | ~1.6 and ~2.3 m above the floor | 1.63 and 2.30 m | PATHoBot's drawing |
| Camera to the stems | ~1 m | ~1 m | the drawing, and the frames' motion at 0.2 m/s |
| Speed | 0.2 m/s, 83 frames a sequence | the same | the PATHoBot paper |
| Looking | level, across the path, to the trolley's right | the same | the frames |
| Crop | about 3 m tall, leafy down to its ripe trusses | about 3 m tall, leafy but for the stem its lowest two trusses hang on | the frames |

The heights and offsets are read off PATHoBot's dimensioned drawing
([arXiv 2010.16272](https://arxiv.org/abs/2010.16272), Fig. 3), so expect
about 10% on each. What the greenhouse differs in:

- **The crop.** BUTom's plants stand in black trays of coir; the
  generator's stand in stone wool on a steel gutter, with lowered stem along
  it. Its trusses hang as longer strings of fruit, and its stems are thicker.
- **The building.** Rows 1 and 6 of the real house have the glass wall and
  its heating pipes behind them; here a plain white wall a row's pitch past
  each outer row stands in for the glass, a bright, even sky lights the
  crop, and two pipes along every row stand in for the heating.
- **The sensor.** The depth is exact, where a D435i's is noisy and has holes,
  and the colour has no motion blur, noise or auto exposure.
- **BUTom's depth unit.** The real depth PNGs are taken to be millimetres,
  the RealSense default; the dataset does not say.

## Experiments

### Cosmos3-Nano video transfer: the frames as photos

https://github.com/user-attachments/assets/d5f10bc4-ba70-478c-bb92-eb53b7435b9b

[Cosmos3-Nano](https://huggingface.co/nvidia/Cosmos3-Nano), NVIDIA's world
model, redraws 12 s of the lower camera's pass as a video in its
[video transfer](https://github.com/NVIDIA/cosmos-framework/tree/main/.agents/skills/cookbooks/cosmos3/generator/transfer)
mode, guided by three hints rendered with it: depth, a segmentation painted
in each class's mean colour in the render, and edges. It runs in FP8 on a
16 GB RTX 5070 Ti, in about 40 minutes for the 181 frames at 480 x 848. Above, the render and
the result; below, the hints.

The fruit keeps its ripeness, the stems and leaves stay where they are, and
the two chunks it is drawn in meet without a seam. Leaves near the camera
come out pale, stems grow thorn-like stubs, and the unripe fruit high in the
picture drifts toward cream. Where the depth is empty it draws a black void;
the walls keep the depth filled.

**Next: fine-tuning on BUTom-ST21.** Trained on BUTom's frames, with hints
made from them, its depth and its frames' edges and blur, a model would draw
that house's crop, light and lenses from this example's hints, its labels
still exact. NVIDIA documents the post-training of
[Cosmos-Transfer2.5-2B](https://github.com/nvidia-cosmos/cosmos-transfer2.5)
on these hints: about 2000 iterations, under an hour on eight rented 80 GB
A100s or H100s, its clips 77 frames of BUTom's 83. Rendering with it needs
an 80 GB GPU too. Cosmos3-Nano's own takes eight H200s and has no LoRA
recipe yet.

## Layout

| file | what it owns |
| --- | --- |
| `pathobot.usda` | the trolley: its chassis, wheels and mast, and the two cameras with BUTom's intrinsics |
| `capture.py` | the greenhouse's parameters, the passes, and writing the dataset |
| `shot.py` | the picture in [How close it is](#how-close-it-is): the trolley's cameras at a shot's places, under BUTom's frames |
| `test_capture.py` | where a pass goes, how depth is stored, and the scripts' imports, without Kit |

[`examples/isaaclab_demo/DEVELOPMENT.md`](../isaaclab_demo/DEVELOPMENT.md)
covers the pinned Isaac Lab release, which this example pins too.

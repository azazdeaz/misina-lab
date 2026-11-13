# Spot weeder

Boston Dynamics Spot with the mechanical weeding tool of Beumer et al.,
[Mechanical Precision Weeding with a Quadruped Robot](https://arxiv.org/abs/2609.20048)
(TU Eindhoven, 2026), as a USD robot for Isaac Lab. The tool rides on Spot's
back and mills weeds out with a 22 mm bit on a spinning shaft. It has no
joint to aim with: Spot shifts, pitches and yaws its body over planted feet
to put the bit on a weed.

**Contents:** [Requirements](#requirements) · [Running it](#running-it) ·
[The robot](#the-robot) · [Where it comes from](#where-it-comes-from) ·
[Using it](#using-it) · [What it does not do](#what-it-does-not-do) ·
[Layout](#layout)

## Requirements

- [uv](https://docs.astral.sh/uv/getting-started/installation/) installed
- [Isaac Sim 6.2](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/installation/requirements.html)
  compatible hardware: an RTX GPU, which renders the cameras
- the network on the first run, which downloads the robot and NVIDIA's Spot

## Running it

```bash
uv run demo.py out
```

stands Spot on bare soil among a few artificial weeds, its spindle at
7000 rpm, then pitches the body over the planted feet until the bit is 4 cm
up and just behind a leaf, where the paper stops to find the weed again
before it mills, with the tool out of the camera's way. It saves three
pictures into `out/`: `side.png` and `front.png` from outside, and
`l515.png` through the robot's own camera. Without `out`, Kit's viewer
opens instead. At the end of each phase it prints how the robot stands:

```text
[INFO]: standing: body 0.506 m up, pitched 0.7 degrees; bit at (0.794, -0.000, 0.196), shaft 16.3 degrees off upright; spindle at 7000 rpm
[INFO]: pitched down: body 0.506 m up, pitched 13.2 degrees; bit at (0.712, -0.000, 0.032), shaft 3.8 degrees off upright; spindle at 7000 rpm
```

The body stands 6 mm over the paper's 0.5 m, on the contact margins of
Isaac Lab's physics settings for Spot, and pitches 13.2 degrees for the 12.0
it is sent to, as the front legs give under the tool.

```bash
uv run pytest
```

checks the robot against the paper's numbers.

## The robot

`spot_weeder.usda` is NVIDIA's Spot with the tool's two links on top:

| Link | Its joint | What it holds |
| --- | --- | --- |
| `tool` | `body/tool_mount`, fixing it to Spot's `body` | the tray on Spot's back, with the Raspberry Pi 4B and its battery, the motor controller and its DC-DC converter in it, as the paper ran all its tests; the head at the tray's front, with the NEMA 17 stepper on it; the bearing tube; the RealSense L515 on its arm, and the L515's camera, `tool/camera` |
| `bit` | `tool/spindle`, revolute along the shaft, with a velocity drive | the steel shaft and the 22 mm bit. Its origin is the end effector, the bottom of the bit, and its z axis runs up the shaft. |

The tool's frame is Spot's body frame, where the paper measures from: x
forward, z up, the origin between the four hips. Spot keeps its 12 leg
joints.

The paper gives these, and the robot has them:

| | Value | In the paper |
| --- | --- | --- |
| End effector, the bottom of the bit | (0.80, 0, -0.30) m in the body frame | Table 2, through Eq. 1 |
| Bit | 22 mm across | Section 2.2 |
| Spindle | up to 7000 rpm | Section 2.2 |
| Tool | 7 kg | Section 2.2 |
| L515 | 26 cm over the bit, looking down at it | Section 2.3: "slightly more than 25 cm", its nearest depth |
| Body when standing | 0.5 m over the ground | z_nom in Eq. 1 |

The tool's centre of mass is 25 cm ahead of the front legs, one reading of
the paper's "The weight is centered approximately 25 cm from the base of the
tool extending in front of Spot's front legs". The rest of the tool's shape
is read off the paper's photos and its drawing of the robot in Fig. 5a,
which give no dimensions: the tray from Spot's middle to 0.27 m past its
front, the shaft leaning 17 degrees forward so that it stands upright with
the bit 2 cm into the ground, the L515's arm down from the head as in the
indoor photos, the parts' sizes, the inertia and the spindle's torque.
[`spot_weeder.py`](spot_weeder.py) says which number is which.

## Where it comes from

The robot is published in
[misina-lab-sim-assets](https://huggingface.co/datasets/azazdeaz/misina-lab-sim-assets)
on Hugging Face, in `spot_weeder/`, and `ASSETS_REVISION` in `spot_weeder.py`
pins the dataset's commit. Spot itself is not in the dataset. The USD
references NVIDIA's Spot on Isaac Sim 6.2's asset server by its URL, the one
Isaac Lab's `SPOT_CFG` spawns, because NVIDIA's licence for its assets lets
nobody else host a copy. Isaac Lab downloads and caches it when it spawns
the robot.

`spot_weeder.py` writes the robot:

```bash
uv run spot_weeder.py out
uv run demo.py pictures --usd out/spot_weeder.usda
```

The second line tries it before it is published. To publish it, upload
`out/` as the dataset's `spot_weeder/` with `huggingface_hub`'s
`HfApi.upload_folder`, and set `ASSETS_REVISION` to the commit it returns.

## Using it

```python
from spot_weeder import robot_cfg

scene.robot = robot_cfg().replace(prim_path="{ENV_REGEX_NS}/Robot")
```

spawns the robot standing, with `stance()` as its default joint positions.
A reset of the simulation leaves the legs as the USD has them, straight
down through the ground, so a script writes the defaults to the joints after
it, as `demo.py` does. The legs have Isaac Lab's actuators for Spot, the ones
its locomotion policies are trained with, and the spindle a velocity drive:
its velocity target spins it. The legs' gains fold under the tool's weight
when they hold a pose, so the demo puts stiffer drives on them.

`stance(pitch)` gives the leg joint angles that pitch the body about its
centre over feet planted below the hips: the paper's body pose command with
no shift and no yaw. `pitch_to(height)` is the pitch that brings the bit to
`height` over the ground, the paper's Eq. 2.

## What it does not do

- Mill. The ground is rigid, so the bit cannot go into it, and nothing is
  cut. With the 1 cm contact margin Isaac Lab's settings for Spot give every
  shape, the ground already touches the bit 2 cm above it and brakes the
  spindle, which is why the demo stops the bit at 4 cm.
- Find weeds. The L515's camera renders, but nothing looks for a weed in its
  pictures.
- Shift or yaw the body. `stance` only pitches it.
- Balance like Spot. The paper poses the body with Spot's own controller,
  which takes the tool's weight into account; here stiff joint drives hold
  the pose.

## Layout

| File | What it is |
| --- | --- |
| [spot_weeder.py](spot_weeder.py) | the robot: the paper's numbers, `write_usd`, `robot_cfg`, `stance` and `pitch_to` |
| [demo.py](demo.py) | the demo above |
| [test_spot_weeder.py](test_spot_weeder.py) | the robot checked against the paper's numbers |

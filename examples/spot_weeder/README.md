# Spot weeder

Boston Dynamics Spot with the mechanical weeding tool of Beumer et al.,
[Mechanical Precision Weeding with a Quadruped Robot](https://arxiv.org/abs/2609.20048)
(TU Eindhoven, 2026), as a USD robot for Isaac Lab. The tool rides on Spot's
back and mills weeds out with a 22 mm bit on a spinning shaft. It has no
joint to aim with: Spot shifts, pitches and yaws its body over planted feet
to put the bit on a weed.

**Contents:** [Requirements](#requirements) · [Running it](#running-it) ·
[The robot](#the-robot) · [Where it comes from](#where-it-comes-from) ·
[Using it](#using-it) · [Workspace](#workspace) ·
[Holding a pose](#holding-a-pose) · [Weeding](#weeding) ·
[What it does not do](#what-it-does-not-do) · [Layout](#layout)

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
7000 rpm, then pitches the body over the planted feet to stop the bit 4 cm
up and just behind a leaf, where the paper stops to find the weed again
before it mills, with the tool out of the camera's way. It saves three
pictures into `out/`: `side.png` and `front.png` from outside, and
`l515.png` through the robot's own camera. Without `out`, Kit's viewer
opens instead. At the end of each phase it prints how the robot stands:

```text
[INFO]: standing: body 0.500 m up, pitched 0.1 degrees; bit at (0.801, -0.000, 0.199), shaft 16.9 degrees off upright; spindle at 7000 rpm
[INFO]: pitched down: body 0.500 m up, pitched 12.0 degrees; bit at (0.720, 0.000, 0.040), shaft 5.0 degrees off upright; spindle at 7000 rpm
```

The body is where it is sent to the millimetre: the pose controller of
[Holding a pose](#holding-a-pose) moves it there and holds it.

```bash
uv run pose.py
```

checks that the controller holds a pose, without Kit, in under a minute.

```bash
uv run tracking.py
```

prints how closely the controller holds the poses it is sent, beside the
paper's Table 1, and `uv run tracking.py workspace` the area where it holds
them; see [Holding a pose](#holding-a-pose). They run without Kit too.

```bash
uv run weeding.py
```

weeds a patch of ground the way the paper does, without Kit, and says what
became of each weed.

```bash
uv run experiments.py
```

weeds 20 patches in each of three ways and prints what became of the weeds
beside the paper's Table 4, and how long it took beside its Table 5, on flat
ground; `soft` and `raised` take the other grounds. See [Weeding](#weeding).

```bash
uv run workspace.py
```

prints where the bit can reach the ground beside the paper's numbers; see
[Workspace](#workspace).

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
when they hold a pose, so `pose.design_scene` puts stiffer drives on them,
`pose.LEGS`, for the controller of [Holding a pose](#holding-a-pose).

The bit is aimed with the body, as in the paper. Points are in the ground
frame: the body's frame with the robot standing at rest, dropped onto the
ground.

```python
from spot_weeder import pose_for, reachable, stance

point = (0.85, 0.2, -0.02)  # 2 cm into the ground, ahead and to the left
if reachable(*point):
    joints = stance(*pose_for(point))
```

- `pose_for(point)` is the paper's Eqs. 2–4: the body's shift, pitch and yaw
  that put the bit on `point`, here 0.20 m, 16.9 degrees and 17.2 degrees.
- `stance(x, pitch, yaw)` gives the leg joint angles that hold the body in
  that pose, `Z_NOM` over the ground. The feet stay planted below the hips,
  and each leg's three joints are solved as in the paper's Appendix A.
  `raised` stands feet on blocks, and `y`, `z` and `roll` make it a full
  body pose.
- `fk(x, pitch, yaw)` is Eq. 1: where the bit is for a body pose. It takes
  `y`, `z` and `roll` as `stance` does.
- `reachable` is the paper's workspace K, which `LIMITS` bounds.
- `E` is the paper's workspace E, as an outline. `inside(E, x, y)` tests
  points against it, as the paper's Eq. 5 keeps the weeds inside E.
- `pitch_to(height)` is Eq. 2 alone.

## Workspace

The paper's two workspaces at ground level (its Fig. 5b) are where the bit
can reach the ground over planted feet:

- K, the area every leg joint allows within its range: 0.402 m².
- E, the area Spot's own controller allowed on the real robot, measured on
  flat ground: 0.165 m². `E` is its outline, traced from the paper's PDF.

`workspace.py` counts K for this robot, from the joint limits and the leg
lengths in NVIDIA's USD, with the body `Z_NOM` over the ground:

| Feet raised on blocks | K here | K in the paper's text | K in the paper's figure |
| --- | --- | --- | --- |
| none | 0.562 m² | 0.402 m² | 0.402 m² |
| FL 5 cm | 0.540 m² | 0.420 m² | 0.452 m² |
| FL 5 cm, FR 10 cm | 0.491 m² | 0.455 m² | 0.444 m² |
| FL 5 cm, HL 10 cm | 0.580 m² | 0.459 m² | 0.568 m² |
| FL 5 cm, HR 10 cm | 0.588 m² | 0.557 m² | 0.459 m² |

- Here K is larger than the paper's. It reaches further back and less far to
  the sides. The paper doesn't say where its feet stand or which joint
  ranges it took.
- 93% of E is inside K here. The rest is E's two outer corners, beyond 30
  degrees of yaw, where the hind legs run out of reach with the feet below
  the hips. The real Spot reached them, so its feet stand elsewhere.
- With feet raised, the paper's text (its Table 3) and the outlines it draws
  (its Appendix B) disagree on K, though they agree on flat ground.

## Holding a pose

The paper sends Spot only the body pose it wants, and Spot's own controller
finds its way there and holds it. `pose.Controller` does the same in Newton,
over planted feet:

```python
import isaaclab.sim as sim_utils
import pose
from isaaclab.app import launch_simulation
from spot_weeder import pose_for

sim_cfg = sim_utils.SimulationCfg(physics=pose.PHYSICS)
with launch_simulation(sim_cfg, {"headless": True, "visualizer": []}):
    sim, scene, controller = pose.launch("soft")  # or "flat", or "raised"
    x, pitch, yaw = pose_for((0.8, 0.1, 0.04))
    controller.move([x, 0, 0, 0, pitch, yaw])  # x, y, z, roll, pitch, yaw
    pose.run(sim, scene, controller, pose.MOVE + 1.0)
    print(controller.pose())
```

- It moves the body along a smooth path in pose space, 4.8 s by default,
  the paper's movement time per weed (its Table 5), and solves `stance`
  for the legs at every step.
- An integrator on the body's measured pose takes out the sag of the leg
  drives under the tool: on the drives alone the body pitches 13.2 degrees
  for the 12 it is sent to.
- The legs reach the feet as deep as they sink into the ground. Legs that
  stop short push the feet against each other, and MuJoCo's friction gives
  way under a lasting push.
- `balanced(pose)` says whether a pose keeps the robot's centre of mass
  6 cm inside its feet. The controller holds poses up to the edge of the
  feet; the 6 cm leaves room for `SpotLike`'s errors.

`pose.PHYSICS` is Isaac Lab's Newton settings for Spot, with a contact
margin of 2 mm on every shape for their 1 cm: the feet stand a few
millimetres into the ground instead of 1 to 2 cm over it. The ground doesn't
touch the bit or its shaft at all, so the bit goes into it as into soil:
`pose.let_the_bit_in` keeps them apart in Newton's model, where Isaac Lab
drops a filtered pair authored in USD.

The grounds are the paper's (its Fig. 7):

| Ground | What it is |
| --- | --- |
| `flat` | Isaac Lab's ground plane. The front feet stand 4 mm into it. |
| `soft` | the plane with a softer contact, like the paper's mattress: the front feet stand 27 mm into it, the hind feet 14 mm |
| `raised` | blocks under two feet, the paper's Table 3 case 4: the front left 5 cm up, the hind right 10 cm |

`tracking.py` sends two robots the same 75 random poses, the bit over
random points of E inside K, 2 to 10 cm over the ground, where the robot
keeps its balance. One gets the poses as they are and holds them to a
hundredth of a millimetre and a hundredth of a milliradian on the flat and
the raised ground, and to a tenth on the soft one. The other gets them with
`pose.SpotLike`'s errors added, Spot's own as the paper measured them. On
flat ground, in mm and mrad:

| | Ours | Ours, Spot-like | The paper's Table 1 |
| --- | --- | --- | --- |
| x | 0.00 ± 0.01 | 1.61 ± 5.18 | 1.11 ± 6.33 |
| y | 0.00 ± 0.00 | −6.76 ± 19.42 | 4.79 ± 27.88 |
| z | 0.00 ± 0.00 | 0.00 ± 1.88 | 0.33 ± 2.04 |
| roll | 0.00 ± 0.01 | 0.73 ± 3.21 | 0.84 ± 4.23 |
| pitch | 0.00 ± 0.01 | 2.82 ± 7.31 | 3.34 ± 8.14 |
| yaw | 0.00 ± 0.01 | −1.68 ± 2.20 | −1.78 ± 2.23 |

- `SpotLike`'s errors are a smooth random function of where the bit goes,
  scaled to Table 1's mean and spread over E. After each move both robots
  make a 2 cm correction, as the paper's pipeline does once it finds the
  weed again, and the y error changes by 4 mm where it spreads 19 mm: the
  error repeats, as the paper found beside its Table 2, so one correction
  takes most of it out.
- How fast the errors change across the ground, 40 cm at the bit, is fitted
  to the paper's Table 4: see [Weeding](#weeding). The 75 poses sample so
  smooth a function coarsely, so the means stray from the paper's by up to
  half a standard deviation.
- Spot's ball feet roll as its shins tilt: a yaw of 0.4 rad moves the
  centre of a front foot 11 mm, and back again with the yaw. Over a run,
  all four feet end up about 1 cm to the left on flat ground and on blocks;
  on the soft ground the front feet 23 mm and the hind feet 9 mm.

`tracking.py workspace` sends a robot to each point of a 2 cm grid over K,
the bit 1 cm over flat ground, and counts where the body is held within
5 mm and 5 mrad: our E. It is 0.470 m², 80% of K here (0.585 m² at that
height) and 2.9 times the paper's 0.165 m². It holds 98% of the paper's E,
all but a few points at its outer front corners. The paper expects
joint-level control to bring E close to K (its §6.2), and here it does.

- Its front edge is where the robot's centre of mass reaches its front
  feet: the bit 0.92 m ahead at y = 0, where the paper's E ends at 0.93.
  Two points past it count as held only because the feet slid 6 to 8 cm
  forward under the tipping robot.
- Its back edge, 8 cm inside K at y = 0, is where the legs give 5 cm under
  the body shifted back, and the knees run out of range taking that out.

## Weeding

`weeding.Pipeline` weeds a patch of ground as the paper does (its §3.2–3.4),
with the weeds' positions read from the scene instead of from the L515's
pictures:

```python
import isaaclab.sim as sim_utils
import numpy as np
import pose
import weeding
from isaaclab.app import launch_simulation

sim_cfg = sim_utils.SimulationCfg(physics=pose.PHYSICS)
with launch_simulation(sim_cfg, {"headless": True, "visualizer": []}):
    sim, scene, controller = pose.launch("soft")
    weeds = weeding.scatter(np.random.default_rng(0))[None]  # 5 weeds in E, for one robot
    pipeline = weeding.Pipeline(sim, scene, controller, weeds, spot_like=[pose.SpotLike()])
    pipeline.run()
    print([weeding.OUTCOMES[status] for status in pipeline.status[0]])
```

1. The robot scans twice, yawed 10 degrees right and then left: its camera,
   26 cm over the bit and looking down, sees part of E at a time. Standing
   still for each scan, it takes its feet where they stand for planted
   (`Controller.replant`).
2. After each scan the planner keeps the weeds found in E (the paper's
   Eq. 5) that the bit can reach (K) and the robot can lean over: its
   centre of mass stays 4 cm inside its feet (`weeding.LEAN`). It sorts
   them right to left.
3. For each weed it moves the bit 3 cm behind and 4 cm above it, where the
   tool doesn't hide it, and finds the weed again standing still. One
   correction (Eq. 6) sends the bit 2 cm into the ground under the weed as
   found again, and it mills for a second. A robot done with its weeds
   waits where it scanned.

`weeding.detect` takes each weed in the camera's 70 × 55 degree view from
where the camera really is, and places it through the camera's nominal
mounting from the pose the pipeline sent: all the pipeline knows of where
the body is. A weed counts as milled out when the bit's centre comes within
11 mm of its stem, half the bit, in the ground.

`experiments.py` weeds 20 patches of 5 weeds in E, a robot to a patch, in
three ways on the same weeds: as the controller tracks; with `SpotLike`'s
errors and 3 mm of detection noise; and, on flat ground, with the tool
mounted 3 cm forward of where the pipeline takes it to be. On the soft
ground, where the paper measured its Table 4, of the weeds found:

| | Ours | Ours, Spot-like | The paper's Table 4 |
| --- | --- | --- | --- |
| Removed | 45 (45%) | 39 (40%) | 72 (82%) |
| Not removed | 55 (55%) | 59 (60%) | 16 (18%) |
| Poor tracking | 0 | 7 (7%) | 12 (14%) |
| No second detection | 0 | 0 | 0 |
| Outside E | 0 | 7 (7%) | 4 (4%) |
| Out of reach | 55 (55%) | 45 (46%) | |
| Found | 100 | 98 | 88 |

Flat and raised ground come out within 2 points of these.

- Of the weeds it aims at, the Spot-like robot removes 85%, the paper's
  86%: by construction, as `SpotLike`'s 40 cm is fitted to it. At 10 cm,
  half of them were missed.
- Half the weeds found are out of reach. A plunge at the front of E would
  put the robot's centre of mass past its front feet. On flat ground with
  no margin, 11 of 20 robots tipped over their front feet while they
  milled, as tracked. A 2 cm margin keeps them up, and Spot's errors take
  2 cm more.
- The paper's Spot weeded all of E, and the tool's centre of mass here is
  one reading of the paper's sentence about it (see [The robot](#the-robot)).
  With it 25 cm from the tray's back end instead, about 30% of the weeds
  are out of reach and 62% come out as tracked, but 5 to 8 of 20 Spot-like
  robots fall.
- On the soft ground the feet slide a little under the yawing moves. Legs
  solved for where they stood push them on until the robot rolls over: 7
  of 20 did before `replant`, none after.
- With the tool 3 cm forward, the bit lands 0.1 mm from the weed on
  average and 0.3 mm at worst, where the paper's §4.1 has under 3 mm: the
  camera rides on the tool, so finding the weed again takes the offset out.
- Of 20 Spot-like robots, 1 to 3 lose their footing.

For the paper's Table 5, the moves take its 4.8 s of movement per weed,
3.8 s to the weed and 1 s to correct; detection stands still its 2 s and
the bit mills its 1 s. A weed takes the paper's 7.8 s by construction. Our
planner's time is the move to the scan and the detection there, 5.8 s to
the paper's 2.4 s. A patch takes 33 s for the 2.3 weeds aimed at, where the
paper takes 47 s for 5; no robot walks, so the walk is the paper's 3.2 s.

## What it does not do

- Mill. The bit goes into the ground, but nothing is cut: a weed counts as
  milled out when the bit's centre comes within 11 mm of its stem, in the
  ground.
- Find weeds in pictures. The pipeline reads the weeds' positions from the
  scene, as the L515 would measure them. The camera renders, but nothing
  looks for a weed in its pictures.
- Walk. The robot weeds the patch it stands at; in the paper, Spot walks
  20 cm on to the next one after each.
- Estimate its pose. The controller reads the body's pose from the
  simulation, where Spot estimates its own from its joints and its IMU.

## Layout

| File | What it is |
| --- | --- |
| [spot_weeder.py](spot_weeder.py) | the robot: the paper's numbers, `write_usd`, `robot_cfg`, and the aiming: `pose_for`, `fk`, `stance`, `reachable` and `E` |
| [pose.py](pose.py) | the body pose in Newton: `Controller`, `SpotLike`, the grounds, `PHYSICS` and `launch`; run, it checks that a pose is held |
| [demo.py](demo.py) | the demo above |
| [tracking.py](tracking.py) | the controller's tracking error and its E, beside the paper's |
| [weeding.py](weeding.py) | the paper's weeding pipeline, with the weeds read from the scene: `Pipeline`, and the detection, `detect`; run, it weeds a patch |
| [experiments.py](experiments.py) | the pipeline over many patches, beside the paper's Tables 4 and 5 |
| [workspace.py](workspace.py) | the workspace at ground level, beside the paper's |
| [test_spot_weeder.py](test_spot_weeder.py) | the robot checked against the paper's numbers |

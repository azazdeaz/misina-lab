# Pictures and videos

The framework's Isaac Lab package, `misina_lab.isaaclab`, has two modules
that render a generated scene, the scene a [generator](generators.md)
builds from its parameters, in Isaac Sim's RTX renderer. `shots` renders
one picture, a still, and `film` records a video of the simulation while it
runs. With both, a script of your own builds the scene and the simulation,
and the module makes the camera and saves the frames.

## A still of the scene

`shots` renders its picture from a **shot file**: a JSON file with the
picture's size, the parameters to change, and the camera. Without an output
path, it opens the viewport of Kit, Isaac Sim's application framework, on
the scene instead, so you can frame the shot by hand.

The camera is given as in the viewer's storyboards. The viewer is the window
that draws the scene, with a control for every parameter
([generators.md](generators.md#what-the-framework-builds-from-it)). A
storyboard is a JSON file that the viewer plays back to record videos and
stills of itself. The camera is the point looked at, a yaw and a pitch in
degrees, a distance in meters and a field of view in degrees (`Pose` in
[tour.rs](../src/tour.rs)). Press `P` in Kit's viewport, or in the viewer,
to print the current camera in that form. A view framed in one app then
gives the same view in the other. The module's docstring lists every field
of a shot. A script builds its own scene between the module's calls. Below,
`BOXES_CFG` is the cfg of the README's example scene, a row of boxes: an
instance of its generator's Isaac Lab config class
([isaac-lab.md](isaac-lab.md)). `sim_cfg` is the script's own
`SimulationCfg`:

```python
import isaaclab.sim as sim_utils
from isaaclab.app import launch_simulation
from misina_lab.isaaclab import shots

args = shots.arguments("Renders a still of the boxes.")
shot = shots.load(args.shot)
shots.configure(BOXES_CFG, shot.get("set", {}))
with launch_simulation(sim_cfg, shots.launcher(args.out)):
    sim = sim_utils.SimulationContext(sim_cfg)
    BOXES_CFG.func("/World/Boxes", BOXES_CFG)
    camera = shots.sensor(shot) if args.out else None
    sim.reset()
    shots.finish(sim, shot, None, camera, args.out)
```

`arguments` reads the command line: the shot file, and the PNG to write if
there is one. `load` reads the shot and rejects a field it does not know. A
script that reads fields of its own passes their names as a second argument.
`configure` applies the shot's parameter changes to the cfg. `launcher`
gives the launch options: headless with cameras for a picture, or Kit's
window to frame one in. `sensor` makes the camera, before the first reset as
every Isaac Lab sensor is. `finish` places the camera and saves the picture,
or, with no camera, opens the viewport. Its third argument is the scene's
ground, a `misina_lab.usd.Ground`
([export.md](export.md#reading-the-ground-back)). The height of the point
looked at is measured from that ground; `None` puts the ground at height 0.

A demo that moves takes its picture at a chosen moment. The script wraps
the demo in `shots.Moments(steps, dt)`. `steps` is an iterator of the
script's own: each time it is advanced it runs one physics step, and yields
a dict that says what the demo is doing then, such as
`{"box": 2, "stage": "carry"}`. The shot names the moment with two more
fields: `at`, a dict of the same kind, and `warmup`, seconds of simulated
time after that. `moments.reach(shot)` steps the demo until what it is
doing holds every item of `at`, and then for `warmup` seconds more. The
script names both fields in its call to `load`,
`shots.load(args.shot, {"at", "warmup"})`, and passes the `Moments` to
`finish` as a sixth argument.

## A video of the scene

`misina_lab.isaaclab.film` films the running simulation in the same
renderer. The script takes a frame every so many physics steps, so the
video plays in simulated time, however long the steps take to compute.
`film.sensor` makes the camera, as `shots.sensor` does for a still. `Rig`
moves the camera from one shot to the next, starting and ending each move
slowly. If the next camera position is more than `cut` meters away, it
jumps there instead. `develop` renders each frame four times, so that
moving edges come out clean and do not flicker, and returns the picture.
`Film` sends the frames to ffmpeg, and writes their captions beside the
video as a SubRip (`.srt`) subtitle file. The module's docstring has the
loop a script runs.

## Where it lives

- [`misina_lab/isaaclab/shots.py`](../../../python/misina-lab/misina_lab/isaaclab/shots.py): the shot file, `arguments`, `load`, `configure`, `launcher`, `sensor`, `finish` and `Moments`
- [`misina_lab/isaaclab/film.py`](../../../python/misina-lab/misina_lab/isaaclab/film.py): `sensor`, `Rig`, `develop` and `Film`
- [`src/tour.rs`](../src/tour.rs): `Pose`, the camera as a shot and a storyboard give it

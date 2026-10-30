---
name: docs-shots
description: Renders the docs' still pictures from their shot files in docs/shots with docs/shots/render.sh, publishes them to the assets repository with docs/shots/publish.sh, and adds or reframes a shot, with the camera placed by hand in the viewer or in Isaac Sim. Use after a change to how the scene, the viewer or a demo looks, or when asked to update, add, reframe or publish a picture in the docs. Not for the videos, which are readme-video and pruning-video.
---

# Docs pictures

The pictures in the docs are rendered, never made by hand. Each comes from a
shot: a JSON file under `docs/shots/` holding everything the picture needs,
the scene's parameters, the camera and the size, and for Isaac Lab how long
the robot walks first. `docs/shots/A/B.json` is published as `A/B.webp` in
[azazdeaz/misina-lab-docs-assets](https://github.com/azazdeaz/misina-lab-docs-assets),
and a page links it as

    https://raw.githubusercontent.com/azazdeaz/misina-lab-docs-assets/main/A/B.webp

An absolute URL: GitHub resolves no relative image path into another
repository. `tests/test_docs.py` fails on a link to a picture no shot renders.

| Shot | Rendered by | Shown at |
| --- | --- | --- |
| `docs/shots/vinery-lab/editor.json` | the vineyard's viewer | the top of `crates/vinery-lab/README.md` |
| `docs/shots/isaaclab_demo/alley.json` | `examples/isaaclab_demo/shot.py` | the top of `README.md` and of `crates/vinery-lab/README.md` |
| `docs/shots/greenhouse-lab/editor.json` | the greenhouse's viewer | the top of `crates/greenhouse-lab/README.md` |
| `docs/shots/greenhouse_demo/rows.json` | `examples/greenhouse_demo/shot.py` | the top of `crates/greenhouse-lab/README.md`, and its section of `README.md` |
| `docs/shots/greenhouse_demo/harvest.json` | `examples/greenhouse_demo/shot.py` | the top of `examples/greenhouse_demo/README.md` |
| `docs/shots/butom_capture/compare.json` | `examples/butom_capture/shot.py`, beside BUTom-ST21's frames | "How close it is" in `examples/butom_capture/README.md` |

## Render them

```bash
docs/shots/render.sh                                     # every shot, into an emptied folder
docs/shots/render.sh docs/shots/vinery-lab/editor.json   # one
```

Each picture lands in `target/docs-assets/`, laid out as the assets
repository is, as WebP. The folder a shot is in names what renders it: an
example with a `shot.py` renders it in Isaac Sim, headless, in a few minutes
on an RTX GPU; any other folder is a crate whose viewer renders it, in a
window that opens on the desktop at the shot's size for about ten seconds.
`butom_capture`'s sets its frames beside the real dataset's, so it needs
`BUTOM` set to the folder BUTom-ST21's `rgb.tar.gz` unpacks its sequences
into. Look at every picture before publishing.

## Publish them

```bash
docs/shots/publish.sh
```

It force-pushes `target/docs-assets` as the one commit on `main` of the assets
repository, which so holds the current set and nothing older; pages show it
within about five minutes. It refuses while the folder lacks a picture this
checkout's Markdown links, since publishing would take that picture down: render
every shot first. A picture changes at once on every page that links its
path, `main`'s README and the docs of older versions alike, so publish when
the change that needs it is about to merge. Publishing replaces what is there
and needs push access to the assets repository: run it only when asked.

## The shot files

**A viewer shot** is a storyboard with no steps, which the viewer plays and
saves as a still: `{PACKAGE}_TOUR` and `{PACKAGE}_SHOT` in
[tour.rs](../../../crates/misina-lab/src/tour.rs). It has `size`, `scale`,
`warmup`, the wall-clock seconds the scene gets to build before the picture,
and `start`, with the `camera`, the parameters to `set` and a `note`.
`cargo test` checks it against the params.

**An Isaac Lab shot** is read by the example's `shot.py` through
`misina_lab.isaaclab.shots`, whose docstring is its reference: `size`;
`set`, parameters changed from the example's own; and `camera`. An example
may add fields of its own, which its `shot.py` names: the quadruped demo's
`waypoint`, where on its route the robot starts, and `warmup`, the simulated
seconds it walks before the picture; the greenhouse's `harvest`, the paths
the harvester works, and `warmup`, the seconds it works first.

**The camera** is the same in both, in the scene's coordinates, Z up and in
meters: `focus`, x and y and then a height above the ground there; `yaw`, 0
looking from −y and 90 from +x; `pitch`, above the horizon; `radius`, the
distance; `fov`, the vertical field of view in degrees. A camera framed in the
viewer frames the same view in Isaac Sim of a scene with the same parameters.

## Frame a shot by hand

The quickest way to a camera is to place it in the running app and have the
app print it.

**In the viewer**, play the shot as a preview:

```bash
VINERY_LAB_TOUR=docs/shots/vinery-lab/editor.json cargo run --release
```

The window opens at the shot's size, on its parameters and camera, and takes
the pointer once the warmup is over. Orbit to the view and press `P`: the log
prints the camera as a line to paste over the shot's `camera`, and the
clipboard holds it too. `P` works in any viewer session.

**In Isaac Sim**, open the Isaac Lab shot without an output file:

```bash
cd examples/isaaclab_demo && uv run shot.py ../../docs/shots/isaaclab_demo/alley.json
```

Kit's window opens on the moment of the picture: the robot stopped where it
will be, the viewport at the shot's size and camera. Navigate the viewport and
press `P`: the terminal prints the camera. The log's
`after the warmup the robot is at (x, y)` says where to aim. To move the robot
instead, change `waypoint`, a meter apart down one alley and back up the next,
or `warmup`. The greenhouse's opens the same way from `examples/greenhouse_demo`;
to move the harvester, change `warmup`.

Then render the shot and look at it.

## Add a shot

1. Copy a shot rendered by the same thing to `docs/shots/<renderer>/<name>.json`.
2. Frame it as above, render it and look at it.
3. Link `https://raw.githubusercontent.com/azazdeaz/misina-lab-docs-assets/main/<renderer>/<name>.webp`
   from the page.
4. Publish, once asked.

A shot of another example needs a `shot.py` there that takes `SHOT.json
OUT.png` and is built on `misina_lab.isaaclab.shots`;
`examples/greenhouse_demo/shot.py` is the shortest to start from.

## Videos stay GitHub uploads

The README videos are not in the assets repository: github.com plays only
video GitHub hosts. Its Markdown drops a `<video>` tag, shows a bare URL of
any other host as a link, and its pages load media from GitHub's own hosts
only. `docs/video/publish.sh` uploads a video; see the readme-video and
pruning-video skills.

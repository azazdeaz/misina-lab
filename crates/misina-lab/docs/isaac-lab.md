# Isaac Lab

A generated scene goes into Isaac Lab as an ordinary scene asset, like a
robot or a USD file. Each [generator](generators.md), the code that
produces one kind of scene, comes with a **cfg class**, an Isaac Lab
[config class](https://isaac-sim.github.io/IsaacLab/main/source/api/lab/isaaclab.utils.html#module-isaaclab.utils.configclass).
The first time the cfg class is used, it builds the scene and saves it as a
USD file for reuse. Then it spawns that file: Isaac Lab adds it to the USD
stage, the scene the simulation runs. You construct the cfg class with the
parameters you want and declare it in your
[scene config](https://isaac-sim.github.io/IsaacLab/main/source/tutorials/02_scene/create_scene.html).
The base class and every helper on this page are in `misina_lab.isaaclab`,
the framework's Isaac Lab package.

The generator's package holds the cfg class, below the generated classes
of its fragments, such as `BoxCfg`:

```python
from typing import ClassVar

import boxlab
from isaaclab.utils.configclass import configclass
from misina_lab.isaaclab import GeneratedSceneCfg


@configclass
class BoxesCfg(GeneratedSceneCfg):
    PARAMS: ClassVar[type] = boxlab.BoxesParams

    # >>> generated: aggregate
    boxes: BoxCfg = BoxCfg()
    FRAGMENTS: ClassVar[tuple[str, ...]] = ("boxes",)
    # <<< generated: aggregate
```

Your scene config uses it:

```python
from boxlab.isaaclab import BoxCfg, BoxesCfg
from isaaclab.assets import AssetBaseCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils.configclass import configclass

BOXES_CFG = BoxesCfg(boxes=BoxCfg(count=5))


@configclass
class MySceneCfg(InteractiveSceneCfg):
    boxes = AssetBaseCfg(prim_path="{ENV_REGEX_NS}/Boxes", spawn=BOXES_CFG)
```

The cfg class extends `GeneratedSceneCfg`. It has one field per
[fragment](generators.md#the-params-aggregate). A fragment is the struct
of parameters of one [element](elements.md), the code that builds one kind
of thing in the scene: `BoxParams` holds the parameters of the boxes. Each
field of the cfg class holds an instance of a Python class with the same
fields as its Rust fragment: `BoxCfg` for `BoxParams`.

A code generator writes the fragment classes, and the lines between the
`# >>> generated` and `# <<< generated` comments, from the Rust structs
([editing-parameters.md](editing-parameters.md)). Do not edit them by
hand, since the next regeneration overwrites them. You write the rest of
the class. The viewer, a window on the scene with a control for every
parameter, has a **Copy Isaac Lab cfg** button. It copies a
[snippet](generators.md#what-the-framework-builds-from-it): Python code
that constructs this class with the viewer's current settings.

On first use, the spawner (`spawn_generated`, the function the cfg class
spawns with) builds the scene from the fragments. It writes the scene to a
USD file in [the cache](#the-cache), and spawns that file through Isaac
Lab's own
[USD-file spawner](https://isaac-sim.github.io/IsaacLab/main/source/api/lab/isaaclab.sim.spawners.html#isaaclab.sim.spawners.from_files.UsdFileCfg).
So everything Isaac Lab's `FileCfg` offers works here too: `scale`,
`semantic_tags`, `rigid_props`, `collision_props`, visual materials. The
`prim_path` above, where the scene goes on the stage, uses
`{ENV_REGEX_NS}`. That pattern matches every environment, Isaac Lab's
parallel copies of the task. So the scene is built once and copied into
each environment.

Declare the scene in a scene config with
[`AssetBaseCfg`](https://isaac-sim.github.io/IsaacLab/main/source/api/lab/isaaclab.assets.html#isaaclab.assets.AssetBaseCfg),
and do not spawn it onto the stage yourself. Isaac Lab can run its physics
on [Newton](https://newton-physics.github.io/newton/latest/), a GPU physics
engine, and the framework's physics config
([rods.md](rods.md#simulating-rods)) is a Newton one. Under Newton, Isaac
Lab simulates only the assets that a scene config declares. A scene spawned
onto the stage directly is rendered, but nothing collides with it.

## The contract

The upper-case attributes in the table belong to the generator. Its
subclass sets them on the class, not on an instance: `copy()` and
`replace()` rebuild a cfg from its fields only, so they drop a value set
on an instance.

| Attribute | What it says |
| --- | --- |
| `PARAMS` | the Python class of the [aggregate](generators.md#the-params-aggregate), the struct that contains every fragment: `boxlab.BoxesParams`. The framework finds the rest from it. Each fragment's class comes from a default `PARAMS()`. The compiled extension module, `boxlab._core`, comes from `PARAMS.__module__`. The package, `boxlab`, comes from that module's name |
| `FRAGMENTS` | the names of the fragment fields, in the order the aggregate takes them. These fields are turned into params, and the cache key is made from them. The code generator writes this line |
| `PLANT` | the name every plant's prim takes before its index: `Plant` for `Plant_007`. Required for a scene with **rods**, flexible organs that bend in the simulation ([rods.md](rods.md)). The rods and wood under one plant's prim form one collision group ([rods.md](rods.md#the-tuning)) |
| `ROD_STRETCH`, `ROD_DAMPING` | the rod tuning ([rods.md](rods.md#the-tuning)). The defaults are `rods.ROD_STRETCH` and `rods.ROD_DAMPING` |
| `LONE_ROD_DAMPING` | the damping of a rod under no plant's prim, used in place of `ROD_DAMPING`. `None` damps it like any other rod |
| `without_rods()` | this cfg with nothing flexible in its scene, or `None` if its scene has no rod with these parameters. The default returns `None`; a generator whose scene can have rods overrides it |
| `cache_dir` | where the USD files go. Unset, it is `$<PACKAGE>_CACHE_DIR`, else `$XDG_CACHE_HOME/<package>/scenes`, else `~/.cache/<package>/scenes`: `BOXLAB_CACHE_DIR` and `~/.cache/boxlab/scenes` for `boxlab` |
| `force_regenerate` | build the scene again even when the cache has it, for working on the generator itself |

The fragment fields are plain `@configclass` dataclasses, not the Rust
classes, so `to_dict()`, `replace()` and Isaac Lab's YAML round trips work
on them. `to_params` turns them into the Rust classes when the scene is
built.

Two rules of `configclass` apply to the class you write. Every annotated
attribute needs a value, so the base class gives `PARAMS` the placeholder
`MISSING`, and your subclass replaces it. And a class attribute must not
hold a dataclass *class*, even inside a tuple, because `configclass` takes
such a class for an instance. So `FRAGMENTS` holds the fields' names as
strings.

## The cache

The same parameters give the same file, so a scene is built once and read
from disk after that. The file is `boxes_<key>.usd`. The key is a hash
over each fragment's `to_dict()` and over what identifies the generator:
the extension's version, the size and modification time of the
extension's file and of the USD builder's (`misina_lab.usd.build`), and the
OpenUSD version. So a rebuilt extension gets new files. The rest of the
cfg, such as `rigid_props` or `scale`, is applied to the prim after
spawning and does not change the USD, so it is not part of the key.
Nothing deletes an old file; clear the directory by hand.

Several processes can share a cache directory, such as the ranks of a
distributed training run. They use a lock file, so only one of them builds
a scene at a time. A scene is written under a temporary name and then
renamed, so no process reads a half-written file.

## Seeing the colliders

Kit, Isaac Sim's application framework, can draw a PhysX scene's colliders,
but not a Newton model's. `ColliderView` draws them: every collision shape
of the running model, as a see-through shape of the same kind and size. The
static scene is blue. A rod, a part of the scene that bends
([rods.md](rods.md)), is green, and any other body, such as a robot's, is
magenta.

```python
from misina_lab.isaaclab import ColliderView

colliders = ColliderView()  # after the first reset, which builds the model
while simulation_app.is_running():
    sim.step()
    colliders.update()
```

Each shape is drawn as a child of its body's prim, so it moves with the
body at no cost per frame. `update()` redraws only the shapes whose pose
relative to their body, or whose size, changed since the last call, such as
the two rod segments at a cut ([cutting.md](cutting.md)). A shape whose
body has no prim on the stage has nowhere to be drawn, so it is skipped,
and `ColliderView.missing` counts such shapes.

## Rods, cutting and pictures

`misina_lab.isaaclab` holds more than the cfg base. Each of these has a
page of its own:

- **Rods.** A scene can hold rods, flexible organs such as long stems.
  `make_physics_cfg_newton` builds the physics config that lets them bend,
  and the spawner tunes them when it spawns the scene
  ([rods.md](rods.md)).
- **Cutting.** `Shears` cuts a rod at any point while the simulation runs
  ([cutting.md](cutting.md)).
- **Pictures and videos.** `shots` renders a still of a scene, and `film`
  records a video of the running simulation ([pictures.md](pictures.md)).

## Where it lives

- [`misina_lab/isaaclab/spawn.py`](../../../python/misina-lab/misina_lab/isaaclab/spawn.py): `GeneratedSceneCfg`, `spawn_generated`, `resolve_usd_path`, `to_params`, and the cache key
- [`misina_lab/isaaclab/colliders.py`](../../../python/misina-lab/misina_lab/isaaclab/colliders.py): `ColliderView`

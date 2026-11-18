# Generators

> [!WARNING]
> **Under development.** This page may not reflect the current state of
> the source. We're working on it.

A **generator** is the code that produces one kind of scene, such as a row
of boxes. You write it as one Rust type that implements the `Generator`
trait. The trait asks for four things: the scene's name, the Python package
it is published in, its parameters, and the code that builds it.

From those four, misina-lab (the framework) makes the tools around the
scene: a viewer, a window in which every parameter is a slider; a Python
package that writes the scene as a USD file; and an Isaac Lab config class
that spawns it.
[What the framework builds from it](#what-the-framework-builds-from-it)
lists them all.

```rust
pub struct Boxes;

impl Generator for Boxes {
    type Params = BoxesParams;
    const NAME: &'static str = "Boxes";
    const PACKAGE: &'static str = "boxlab";
    fn plugin(app: &mut App) { /* the elements' plugins */ }
}
```

| Item | What it decides |
| --- | --- |
| `Params` | one struct holding every parameter of the scene, [the params aggregate](#the-params-aggregate); declared with the `generator!` macro |
| `NAME` | the name of the scene's root prim, and the stem of the Python names: `BoxesParams`, `BoxesCfg`, `BOXES_CFG`, `boxes_cfg.py` |
| `PACKAGE` | the name of the scene's Python package. The package holds `_core`, the compiled Rust extension module, and `boxlab.isaaclab`, its Isaac Lab side. The name is also the prefix of the environment variables, `BOXLAB_PERF`, and of the custom USD attributes, `boxlab:` |
| `plugin` | adds every [element](elements.md), the code that builds one kind of thing in the scene, as a Bevy plugin; the elements run in the order of the generator's [stage sets](#stage-sets) |

## The params aggregate

Each [element](elements.md) has a struct of parameters of its own, called a
**fragment**: `BoxParams` for the boxes. The **aggregate** is the struct
that contains every fragment, one field each, so it holds every parameter
of the scene: `BoxesParams` here.

The aggregate is what gets passed around. Python code constructs it, the
viewer's panel of sliders edits it, and a **headless build** takes it. A
headless build makes the scene once without opening a window; it is how
Python and the tests build a scene.

The `generator!` macro declares the aggregate, from a list with one line per
fragment:

```rust
misina_lab::generator! {
    /// Every knob of the box row.
    pub struct BoxesParams as PyBoxesParams("BoxesParams") for Boxes {
        pub scene: SceneParams,
        pub terrain: misina_lab::terrain::TerrainParams as core,
        #[reflect(@Label("The boxes"))]
        pub boxes: BoxParams,
    }
}
```

The macro writes the struct and its `Params` implementation, which has two
methods: `apply` copies the aggregate into the running scene, and `read`
copies it back out. With the generator's `python` feature on, the macro
also writes [the Python module](#the-python-module). A `@Label` on a field
is the title of that fragment's section in the panel. A fragment that the
framework itself provides, here `TerrainParams`, is marked `as core`. The
framework has already written its Python constructor, because PyO3, the
library that binds Rust to Python, lets only the crate that defines a type
add methods to it.

`SceneParams` is the generator's own struct, for whatever applies to the
whole scene. The framework asks nothing of it; by convention it holds the
seed every element draws its random numbers from
([randomness.md](randomness.md)).

In the running scene each fragment is stored on its own, so an edit
rebuilds only what depends on it: changing one fragment rebuilds the
elements that read it and the elements built on top of those, and the rest
of the scene stays as it was.

> [!NOTE]
> **In Bevy terms.** Each fragment is a `Resource`, and an element's systems
> run only when a fragment they read has changed:
> `run_if(resource_changed::<BoxParams>)`. `Params::apply` writes each
> fragment with `set_if_neq`, which marks a resource as changed only when
> the new value differs.

## Stage sets

A scene is built in layers, each on what the one before it placed: for
example the ground first, then whatever stands on it, then what grows on
those. A generator lists its **stages** in build order, and each element
runs in one of them. The stages run one after another, so a layer is
complete before the next one starts and can read what it placed
([elements.md](elements.md#the-pipeline)). Elements that do not read each
other's output can share a stage.

> [!NOTE]
> **In Bevy terms.** A stage is a variant of a `SystemSet` enum. The
> generator chains its sets inside `Build`, the framework's set in
> `PreUpdate`, where everything that builds the scene runs. The panel
> applies an edit before `Build` runs.

## What the framework builds from it

Each of these is a function that takes the generator's type as `G`:

| What | Entry point |
| --- | --- |
| the viewer: a window on the scene with the parameter panel, a footer of scene statistics and a key that saves the scene | `viewer::app::<G>()` |
| one headless build, which returns the scene document, the JSON that Python turns into USD ([export.md](export.md)) | `generate::scene::<G>(&params)` |
| the parameter panel on its own | `ui::plugin::<G>` |
| the **snippet**: Python code that constructs the Isaac Lab config with the current settings, listing only the parameters moved off their defaults. The viewer's **Copy Isaac Lab cfg** button puts it on the clipboard | `snippet::cfg::<G>(&params)` |
| the generated files: the Python type stub, the Isaac Lab cfg classes, the re-exports and the parameters page ([editing-parameters.md](editing-parameters.md)) | `codegen::targets::<G>(docs)`, and `codegen::stale` for the test |
| a built scene to assert against in a test | `testing::grown::<G>(params)` |

A generator's `main` function calls `viewer::app::<G>()`, adds any plugins
that only the viewer needs, and runs the app. The camera starts where
`viewer::Framing` says, which frames a scene of about 80 by 50 m unless
`main` inserts a `Framing` of its own. To start from params other than the
defaults, pass them to `viewer::start_from`. If you set them any other
way, the defaults overwrite them: the panel keeps its own copy of the
params and writes that copy into the scene shortly after start-up.
`start_from` sets both the panel's copy and the scene.

## The Python module

With its `python` feature on, a generator's crate also builds as a Python
extension module, `_core`, and the macro writes what is in it:

- a class per fragment, constructed with keyword arguments:
  `BoxParams(count=5)`;
- the aggregate's class, `BoxesParams`, with one fragment per field, so
  `params.boxes.count = 5` changes the scene that gets built;
- two methods on the aggregate. `write_usd(path)` builds the scene once and
  writes it as a USD file. `generate_scene_json()` returns the scene
  document instead, for a caller that wants to cache or compare it.

A field with a fixed list of allowed names, declared with `@Choices`, is
checked when one of those two methods is called, not when it is set: a name
that is not on the list raises `ValueError` then.

`write_usd` hands the document to `misina_lab.usd.build_usd`, in the
framework's Python package, so `usd-core` is a dependency of the Python
package and not of the Rust crate. Editors and type checkers read the
classes' signatures from `_core.pyi`, a stub file generated from the same
structs ([editing-parameters.md](editing-parameters.md)).

The generator's `lib.rs` holds the `Generator` impl and, under the `python`
feature, the `#[pymodule] fn _core`. It calls `module`, a function the macro
wrote, which registers the classes.

> [!NOTE]
> **In PyO3 terms.** The aggregate holds each fragment as a `Py<T>`, and
> fragments are `skip_from_py_object`, so Python always gets the same object
> and never a copy: with a plain field, `params.boxes.count = 5` would
> change a copy and the scene would keep the old value. Both methods release
> the GIL while Rust builds the scene. `module` sets each class's
> `__module__` to the extension module's name, which the Isaac Lab spawner
> reads to find the extension the params came from
> ([isaac-lab.md](isaac-lab.md)).

## What a generator's tests run

- `testing::check_params::<P>()` panics on the first field that does not
  declare what the panel, the snippet and the parameters page need: a doc
  comment, a slider or a choice list, a default in range.
- `testing::nudged::<P>()` is a set of params with every field moved off its
  default, for a test that `apply` and `read` return what was put in.
- A test over `codegen::stale::<G>(root, docs)` fails while a generated file
  is out of date with the structs; `regen_params`, an ignored test beside
  it, rewrites the files
  ([editing-parameters.md](editing-parameters.md)).

## Where it lives

- [`src/lib.rs`](../src/lib.rs): `Generator`, `Params`, `Build`
- [`src/params.rs`](../src/params.rs): `generator!`, `fragment_python!`, and the walk over a params struct
- [`src/python.rs`](../src/python.rs): the keyword constructor, the choice check, the two entry points
- [`src/generate.rs`](../src/generate.rs): the headless app
- [`src/testing.rs`](../src/testing.rs): what a generator's tests get, and the `Boxes` fixture this crate tests itself with

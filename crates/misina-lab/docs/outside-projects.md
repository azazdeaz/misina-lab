# A generator in its own repository

A **generator** is the code that produces one kind of scene
([generators.md](generators.md)). This page sets one up in a repository of
its own, outside misina-lab's. The repository holds one Rust crate, and
that crate builds three things: the generator, its viewer, and its Python
extension module. The viewer is an app that shows the scene in a window,
with a panel that has a control for every parameter: a slider, a dropdown
or a checkbox. The extension module is the compiled Rust that Python
imports to build the scene.

The generator depends on both halves of misina-lab: the Rust crate from
crates.io and the Python package from PyPI. Both must come from the same
misina-lab release ([One misina-lab release](#one-misina-lab-release)).
Write the generator's own Rust code as the
[README's smallest generator](../README.md#the-smallest-generator) shows.
This page covers the other files the repository needs. Below, `0.1.0`
stands for the release the generator pins.

## The layout

The README's example generator builds a row of boxes. Here it is laid out
as a project of its own: the crate is `boxlab`, and the generator's type is
`Boxes`.

```text
boxlab/
├── AGENTS.md
├── Cargo.toml
├── pyproject.toml
├── docs/parameters.md          generated
├── src/
│   ├── lib.rs                  the generator, `_core`, the generated-files tests
│   └── main.rs                 misina_lab::viewer::app::<boxlab::Boxes>().run()
└── python/boxlab/
    ├── __init__.py             re-exports `_core`
    ├── _core.pyi               the stub for `_core`
    ├── py.typed
    └── isaaclab/
        ├── __init__.py         re-exports the cfg classes
        └── boxes_cfg.py        `BoxesCfg`, over `GeneratedSceneCfg`
```

The generator's type implements misina-lab's `Generator` trait
([generators.md](generators.md)). Two constants of that implementation set
two of these names. `PACKAGE`, the name of the Python package, is the
directory under `python/`. `NAME`, the scene's name, gives
the cfg file its name in lower case: `Boxes` gives `boxes_cfg.py`.

## Cargo.toml

```toml
[package]
name = "boxlab"
version = "0.1.0"
edition = "2024"

[lib]
# `cdylib` builds the shared library that Python imports; `rlib` builds the
# Rust library that the viewer binary and `cargo test` use.
crate-type = ["cdylib", "rlib"]

[dependencies]
misina-lab = "=0.1.0"
bevy = "0.20.1"
pyo3 = { version = "0.29", features = ["abi3-py312"], optional = true }

[features]
python = ["dep:pyo3", "misina-lab/python"]
extension-module = ["python", "pyo3/extension-module"]

# Copied from the root Cargo.toml of misina-lab's repository at tag v0.1.0.
[profile.dev]
opt-level = 1

[profile.dev.package."*"]
opt-level = 3
```

- **`misina-lab`** is pinned to exactly one version, `=0.1.0`.
  [One misina-lab release](#one-misina-lab-release) gives the reason.
- **`[profile]`**: copy these lines from misina-lab's
  [root `Cargo.toml`](https://github.com/azazdeaz/misina-lab/blob/v0.1.0/Cargo.toml) at the
  release's tag. Cargo reads profile settings only from the manifest at
  the root of a build, so misina-lab's own are not used in yours. They
  compile the dependencies with optimization, so a debug build of the
  viewer runs at a usable frame rate.
- **`bevy`** is the version misina-lab uses. [Bevy](https://bevy.org) is
  the game engine that misina-lab builds the scene in. List no Bevy
  features on this line. Cargo compiles Bevy once for the whole build, with
  every feature that any crate asks for, so the features misina-lab needs
  are already on.
- **`pyo3`** has the same minor version as misina-lab's, 0.29.
  [PyO3](https://pyo3.rs/) is the library that binds Rust to Python. A
  build can contain only one copy of it: `pyo3-ffi` declares
  `links = "python"`, and Cargo allows only one package per `links` value.
  With another minor version, Cargo cannot resolve the dependencies, even
  with the `python` feature off.
- **The feature must be called `python`.** The `generator!` macro, which
  declares the parameters, expands inside your crate. It wraps its Python
  code in `#[cfg(feature = "python")]`, and Cargo checks that condition
  against your crate's features, not misina-lab's.
- **`extension-module`** tells PyO3 not to link libpython, the Python
  runtime library, because the Python that loads the module provides it. A
  binary built this way cannot start on its own, so `cargo run` and
  `cargo test` fail with this feature on. Only maturin turns it on.
  [maturin](https://www.maturin.rs/) is the tool that builds a Rust crate
  into a Python package ([pyproject.toml](#pyprojecttoml)).

To build the viewer for the browser as well, as WebAssembly, also copy the
`[profile.wasm-release]` table from misina-lab's root `Cargo.toml`. You add
nothing else: misina-lab's own `Cargo.toml` turns on what a `wasm32` build
needs, Bevy's WebGPU backend and the `wasm_js` feature of `getrandom`.

## lib.rs

Beside the generator, the crate root holds two more things: the extension
module's entry point, and three tests. The params structs are the Rust
structs that declare the scene's parameters. The first test fails on a
parameter that does not declare what the panel and the generated files
need, such as a doc comment. The second fails while a generated file
differs from what the structs give, and the third rewrites the files
([generators.md](generators.md#what-a-generators-tests-run)):

```rust
/// `boxlab._core`. PyO3 names the init symbol after this function, so it
/// matches the last component of `module-name` in pyproject.toml.
#[cfg(feature = "python")]
#[pyo3::pymodule]
fn _core(m: &pyo3::Bound<'_, pyo3::types::PyModule>) -> pyo3::PyResult<()> {
    module(m) // written by `generator!`
}

#[cfg(test)]
mod tests {
    use std::path::{Path, PathBuf};

    use misina_lab::codegen;
    use misina_lab::testing::check_params;

    /// Every field has a doc comment, a slider or a choice list, and a
    /// default inside it.
    #[test]
    fn every_field_declares_what_the_panel_and_the_docs_need() {
        check_params::<super::BoxesParams>();
    }

    fn stale() -> Vec<(PathBuf, String)> {
        codegen::stale::<super::Boxes>(Path::new(env!("CARGO_MANIFEST_DIR")), "docs/parameters.md")
    }

    #[test]
    fn generated_python_and_docs_are_fresh() {
        let stale: Vec<PathBuf> = stale().into_iter().map(|(path, _)| path).collect();
        assert!(
            stale.is_empty(),
            "stale: {stale:?} — run `cargo test regen_params -- --ignored`"
        );
    }

    /// Rewrites the generated files. A dev tool: it writes files and asserts nothing.
    #[test]
    #[ignore]
    fn regen_params() {
        for (path, fresh) in stale() {
            std::fs::write(path, fresh).unwrap();
        }
    }
}
```

`module` is a function the `generator!` macro writes. It adds the Python
classes of the parameters, and `__version__`, to the extension module.
`codegen::stale` renders every generated file from the params structs as
they are now, and returns each file whose text differs, with its new text.
So the second test fails while a generated file is out of date, and
`regen_params` writes the new text out.

## The generated files

Five files are generated from the params structs: the parameters page, the
type stub `_core.pyi` that editors and type checkers read, the cfg class
file, and the two `__init__.py` files that re-export their classes. The
cfg class is the Isaac Lab config class of the scene. On first use it
builds the scene and saves it as a USD file for later runs. Then it spawns
the scene in the simulation ([isaac-lab.md](isaac-lab.md)).

`regen_params` overwrites the whole parameters page. In the four Python
files it rewrites only marked regions, from a line
`# >>> generated: <region>` to a line `# <<< generated: <region>`.
Everything outside the markers is written by hand and left alone. So when
you start a project, create each Python file yourself, with an empty pair
of markers for each region. A missing file or a missing pair fails the
test, and the message names the file and the region. Create the `docs/`
folder too: `regen_params` writes the parameters page into it, and does
not create the folder.

| File | Regions |
| --- | --- |
| `python/boxlab/__init__.py` | `exports` |
| `python/boxlab/_core.pyi` | `fragments`; `aggregate` inside `class BoxesParams` |
| `python/boxlab/isaaclab/__init__.py` | `imports`; `all` inside `__all__ = [...]` |
| `python/boxlab/isaaclab/boxes_cfg.py` | `fragments`; `aggregate` inside `class BoxesCfg` |

Each `fragments` region gets one class per fragment. A fragment is an
element's own struct of parameters, and an [element](elements.md) is the
code that builds one kind of thing in the scene, such as the boxes. Each
`aggregate` region gets the fields of the aggregate, the one struct that
contains every fragment
([generators.md](generators.md#the-params-aggregate)). Indent a pair to
where the generated code goes:

```python
@configclass
class BoxesCfg(GeneratedSceneCfg):
    PARAMS: ClassVar[type] = boxlab.BoxesParams

    # >>> generated: aggregate
    # <<< generated: aggregate
```

`GeneratedSceneCfg`, in `misina_lab.isaaclab.spawn`, is the framework's
base for every cfg class. Your subclass sets `PARAMS` to the aggregate's
Python class. That line is outside the markers, so you write it by hand.

These parts are also written by hand:

- the docstrings, and the imports the generated code uses;
- in the stub, `__version__: str` and the aggregate's methods: `__repr__`,
  `generate_scene_json` and `write_usd`
  ([generators.md](generators.md#the-python-module));
- at the top of `isaaclab/__init__.py`, an `import isaaclab` that raises a
  clear `ImportError` when Isaac Lab is not installed.

## pyproject.toml

```toml
[build-system]
requires = ["maturin>=1.9,<2"]
build-backend = "maturin"

[project]
name = "boxlab"
requires-python = ">=3.12"
dynamic = ["version"]
dependencies = ["misina-lab==0.1.0"]

[tool.maturin]
features = ["extension-module"]
module-name = "boxlab._core"
python-source = "python"
```

maturin builds with the `extension-module` feature, puts the compiled
module at `boxlab._core`, and takes the hand-written Python from
`python/`. It also takes the package's version from `Cargo.toml`.

`uv build` writes the wheel and the sdist (the source archive) to `dist/`.
`uv sync` installs the project into `.venv`. It does not rebuild the
extension when the Rust code changes. Run
`uv sync --reinstall-package boxlab` to rebuild it.

## One misina-lab release

The extension writes the scene document, the JSON description of the
built scene ([export.md](export.md)). `misina_lab.usd`, in the Python
package, turns that document into USD, the scene format that Isaac Sim
and Isaac Lab load. Each document carries a format number, and
`misina_lab.usd` refuses a document whose number differs from its own. So
both halves must come from one release: `misina-lab = "=X.Y.Z"` in
`Cargo.toml` and `misina-lab==X.Y.Z` in `pyproject.toml`.

Each misina-lab release puts the crate on crates.io and the Python package
on PyPI from one commit, tagged `vX.Y.Z`. The Rust pin needs the `=`.
Cargo's default requirement, `"X.Y"`, accepts any later compatible
release, so `cargo update` could upgrade the crate to a later release
while the Python package stays on the old one.

To move to a new misina-lab release, change all of this in one commit:

1. Both pins.
2. The `bevy` and `pyo3` versions and the `[profile]` lines, as the new
   release has them.
3. Every file `cargo test` then reports stale, rewritten by `regen_params`.

### Between releases

To use a change that is not released yet, pin both halves to one commit,
from git:

```toml
# Cargo.toml
misina-lab = { git = "https://github.com/azazdeaz/misina-lab", rev = "<commit>" }
```

```toml
# pyproject.toml
[tool.uv.sources]
misina-lab = { git = "https://github.com/azazdeaz/misina-lab", rev = "<commit>", subdirectory = "python/misina-lab" }
```

The Python package now has two pins, and each holds in a different place:

- In the project's own `.venv`, [uv](https://docs.astral.sh/uv/) installs
  misina-lab from the `[tool.uv.sources]` entry, at `<commit>`. It ignores
  the version bound `misina-lab==X.Y.Z` while the entry is there.
- A wheel built from the project records only the version bound, not the
  entry. Whoever installs the wheel gets misina-lab by the bound.

Nothing checks that the two agree, so set the bound to the version that
misina-lab's `pyproject.toml` has at `<commit>`. Install a wheel built
this way only together with misina-lab built from the same commit.
Without that, the installer downloads release `X.Y.Z` from PyPI, if it
exists. If the document format changed after that release, its
`misina_lab.usd` refuses the documents your extension writes. A wheel that
you publish for others pins a release, not a commit.

## AGENTS.md

`AGENTS.md` is the project's map for coding agents: its commands and the
rules that cause failures. If you use Claude Code, add a `CLAUDE.md` next
to it that holds the single line `@AGENTS.md`. Claude Code then loads
`AGENTS.md` as well. The map points to misina-lab's docs, which ship
inside the crate at the pinned version.

```markdown
# Boxlab

A row of boxes, built on misina-lab.

## Commands

    cargo run --release                     # the viewer
    cargo test                              # fails while a generated file is stale
    cargo test regen_params -- --ignored    # rewrites the generated files
    cargo clippy --all-targets --features python -- -D warnings
    uv build                                # the wheel and the sdist, into dist/

## The framework

misina-lab is pinned to one release in Cargo.toml and in pyproject.toml; the
two change together. Its docs ship inside the crate. To find them, run
`cargo metadata --format-version 1` and take the `manifest_path` of the
`misina-lab` package. Read README.md in that folder first, then docs/.

## Rules that bite

- A parameter is declared once, on its Rust params struct. All of
  docs/parameters.md is generated from it. So are the stub, the cfg
  classes and the re-exports, between `# >>> generated` markers. Generated
  text is never edited by hand.
- `extension-module` is for maturin only; it breaks `cargo run` and `cargo test`.
- The scene is authored Z-up, in meters.
```

## Where it lives

- [`src/params.rs`](../src/params.rs): `generator!`, and the `module` function it writes
- [`src/codegen.rs`](../src/codegen.rs): `targets`, the files and their regions; `stale`, what both tests call
- [`Cargo.toml`](../Cargo.toml): the `pyo3` version a generator matches
- [the root `Cargo.toml`](https://github.com/azazdeaz/misina-lab/blob/v0.1.0/Cargo.toml): the `bevy` version, and the `[profile.dev]` lines a generator copies
- [`python/misina-lab/pyproject.toml`](https://github.com/azazdeaz/misina-lab/blob/v0.1.0/python/misina-lab/pyproject.toml): the version the Python pin names
- [`python/misina-lab/misina_lab/isaaclab/spawn.py`](../../../python/misina-lab/misina_lab/isaaclab/spawn.py): `GeneratedSceneCfg`, the base of the cfg class

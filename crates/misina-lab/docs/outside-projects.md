# A generator in its own repository

A generator outside this repository depends on both halves of misina-lab:
the Rust crate by git, and the Python package by version. Its repository
holds one crate that is the generator, its viewer and its Python extension
module at once. This page is what that crate needs beyond the generator's
code, which is written as the [README](../README.md) shows. Below,
`<commit>` stands for the commit a generator pins.

## The layout

The row of boxes, as its own project:

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

`PACKAGE` names the directory under `python/`, and `NAME` the cfg file
([generators.md](generators.md)).

## Cargo.toml

```toml
[package]
name = "boxlab"
version = "0.1.0"
edition = "2024"

[lib]
# `cdylib` is the extension module Python imports; `rlib` is what the viewer
# binary and `cargo test` link against.
crate-type = ["cdylib", "rlib"]

[dependencies]
misina-lab = { git = "https://github.com/azazdeaz/misina-lab", rev = "<commit>" }
bevy = "0.20.0"
pyo3 = { version = "0.29", features = ["abi3-py312"], optional = true }

[features]
python = ["dep:pyo3", "misina-lab/python"]
extension-module = ["python", "pyo3/extension-module"]

# Copied from the root Cargo.toml of misina-lab's repository at <commit>.
[profile.dev]
opt-level = 1

[profile.dev.package."*"]
opt-level = 3
```

- **`[profile]`** is read only from the root manifest of a build, so it is
  copied from the [root `Cargo.toml`](../../../Cargo.toml) at `<commit>`. It
  optimizes the dependencies, so a debug viewer runs at a usable frame rate.
- **`bevy`** is misina-lab's version. Its features are not copied: Cargo
  turns on the ones misina-lab asks for on the one Bevy in the build.
- **`pyo3`** is misina-lab's minor version. `pyo3-ffi` declares
  `links = "python"`, and a build holds one package per `links` value, so
  another minor fails to resolve even with the `python` feature off.
- **The feature is called `python`**, since `generator!` expands its Python
  side under `#[cfg(feature = "python")]`, evaluated in the calling crate.
- **`extension-module`** unlinks libpython, which breaks `cargo run` and
  `cargo test`, so only maturin turns it on.

A web build needs only the `[profile.wasm-release]` table, copied the same
way. misina-lab turns on Bevy's WebGPU backend and `getrandom`'s `wasm_js`
for `wasm32` itself.

## lib.rs

Beside the generator, the crate root holds the extension module and the
pair of tests that keeps the generated files in step with the structs
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

## The generated files

`regen_params` writes the parameters page whole and fills marked regions in
the Python files. Everything outside the markers is hand-written, so a new
project starts each file with empty marker pairs; a missing pair fails the
test with the file and the region named.

| File | Regions |
| --- | --- |
| `python/boxlab/__init__.py` | `exports` |
| `python/boxlab/_core.pyi` | `fragments`; `aggregate` inside `class BoxesParams` |
| `python/boxlab/isaaclab/__init__.py` | `imports`; `all` inside `__all__ = [...]` |
| `python/boxlab/isaaclab/boxes_cfg.py` | `fragments`; `aggregate` inside `class BoxesCfg` |

A pair is indented to where the generated code goes:

```python
@configclass
class BoxesCfg(GeneratedSceneCfg):
    PARAMS: ClassVar[type] = boxlab.BoxesParams

    # >>> generated: aggregate
    # <<< generated: aggregate
```

The hand-written rest is the docstrings, the imports the generated code
uses, `__version__: str` and the aggregate's methods in the stub
(`__repr__`, `generate_scene_json`, `write_usd`), and the `isaaclab`
package's guard against a missing Isaac Lab
([generators.md](generators.md#the-python-module)).

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

[tool.uv.sources]
misina-lab = { git = "https://github.com/azazdeaz/misina-lab", rev = "<commit>", subdirectory = "python/misina-lab" }

[tool.maturin]
features = ["extension-module"]
module-name = "boxlab._core"
python-source = "python"
```

maturin takes the version from `Cargo.toml`. `uv build` writes the wheel
and the sdist to `dist/`. `uv sync` installs the project into `.venv` and
keeps that build however the Rust changes; `uv sync --reinstall-package
boxlab` rebuilds it.

## One misina-lab release

The extension writes a scene document and `misina_lab.usd` reads it, so both
halves are pinned to one misina-lab: the `rev` in `Cargo.toml`, and the
`rev` and the version in `pyproject.toml`. The two Python pins hold in
different places. uv installs from the `[tool.uv.sources]` entry and ignores
the version bound while the entry is there, so the project's `.venv` is held
by the `rev`. The wheel's metadata carries the bound and not the entry, so
an installer of the wheel is held by `misina-lab==0.1.0`. Nothing compares
the two: keep the bound at the version misina-lab's `pyproject.toml` has at
`<commit>`.

A release tagged `vX.Y.Z` has both halves at one commit and the Python
package on PyPI as `X.Y.Z`. Pin it with `tag = "vX.Y.Z"` in place of `rev`
in `Cargo.toml` and `misina-lab==X.Y.Z` in `pyproject.toml`, with no
`[tool.uv.sources]` entry. Until a version is on PyPI, a wheel that names it
installs only beside misina-lab installed from git.

Moving to a new misina-lab is one commit: both pins; the `bevy` and `pyo3`
versions and the `[profile]` lines as the new commit has them; and
whatever `cargo test` then reports stale, rewritten by `regen_params`.

## AGENTS.md

The project's map points at the framework's docs at the pinned commit; a
`CLAUDE.md` holding `@AGENTS.md` beside it loads the map for Claude Code.

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

misina-lab is pinned to one commit in Cargo.toml and in pyproject.toml; the
two change together. Its docs are in Cargo's checkout of that commit, beside
the `manifest_path` that `cargo metadata --format-version 1` gives the
`misina-lab` package: README.md first, then docs/.

## Rules that bite

- A parameter is declared once, on its Rust params struct. The stub, the cfg
  classes and docs/parameters.md are generated from it, between
  `# >>> generated` markers that are never edited by hand.
- `extension-module` is for maturin only; it breaks `cargo run` and `cargo test`.
- The scene is authored Z-up, in meters.
```

## Where it lives

- [`src/params.rs`](../src/params.rs): `generator!`, and the `module` function it writes
- [`src/codegen.rs`](../src/codegen.rs): `targets`, the files and their regions; `stale`, what both tests call
- [`Cargo.toml`](../Cargo.toml): the `pyo3` version a generator matches
- [the root `Cargo.toml`](../../../Cargo.toml): the `bevy` version, and the `[profile.dev]` lines a generator copies
- [`python/misina-lab/pyproject.toml`](../../../python/misina-lab/pyproject.toml): the version the Python pin names

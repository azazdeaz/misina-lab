# Misina Lab

A framework for procedural scene generators for robotics simulation, mainly
Isaac Lab, and two reference generators built on it: `vinery-lab`, a
vineyard, and `greenhouse-lab`, the rows of a tomato greenhouse. A Bevy app
builds the scene as ordinary meshes and transforms, exports it as a plain
JSON scene document, and Python turns that into a USD stage. Each
generator's crate ships as an interactive viewer and as a Python extension
module.

[README.md](README.md) is the user-facing front page: what the framework
gives a generator, then the reference generators and their examples. Each
one's own README, [vinery-lab](crates/vinery-lab/README.md) and
[greenhouse-lab](crates/greenhouse-lab/README.md), covers what it builds and
spawning its cfg. This file is the map for working on the repo itself.

## Commands

```bash
cargo run --release                                          # viewer / parameter editor
cargo run --release -p greenhouse-lab                        # the greenhouse's
cargo fmt --all
cargo clippy --workspace --all-targets --features vinery-lab/python,greenhouse-lab/python -- -D warnings
cargo test --workspace
uv run ruff check . && uv run ruff format --check .
uv run mypy
uv run pytest tests -q
```

Everything below the first two lines is what `.github/workflows/ci.yml` runs on
every push and pull request. Isaac Lab is not installed there, so the tests
that need it skip themselves; run those from a demo venv — see
[DEVELOPMENT.md](DEVELOPMENT.md).

## Where things are

### Rust

Three crates in one Cargo workspace: `crates/misina-lab` is the framework —
everything that knows nothing about vines — and `crates/vinery-lab` and
`crates/greenhouse-lab` are the generators built on it. The seam is
`misina_lab::Generator`: one marker type per generator, which every framework
surface is keyed on. The framework has a map of its own,
[crates/misina-lab/AGENTS.md](crates/misina-lab/AGENTS.md), and its docs
explain how a generator is built. The root `Cargo.toml`'s `default-members`
leaves the greenhouse out of a bare `cargo run` or `cargo test`, so name it
with `-p greenhouse-lab` or reach it with `--workspace`. What follows is the
vineyard, and then the greenhouse.

| Path | What it is |
| --- | --- |
| [vinery-lab/src/lib.rs](crates/vinery-lab/src/lib.rs) | crate root: the `Vineyard` generator and the `_core` Python module |
| [vinery-lab/src/main.rs](crates/vinery-lab/src/main.rs) | the `vinery-lab` binary: the viewer plus the layout gizmos and the per-layer timing |
| [vinery-lab/src/elements/](crates/vinery-lab/src/elements/) | one module per vineyard thing; `mod.rs` holds the pipeline contract, the ground and layout wiring, and the `generator!` call |
| [vinery-lab/src/elements/util/](crates/vinery-lab/src/elements/util/) | the blade outline built from a leaf's veins, the named colours and responses, layout solver, planting walk — everything under `elements/` that is not an element |
| [vinery-lab/src/perf.rs](crates/vinery-lab/src/perf.rs) | the per-layer marks and the bench |
| [greenhouse-lab/src/lib.rs](crates/greenhouse-lab/src/lib.rs) | crate root: the `Greenhouse` generator and its `_core` Python module |
| [greenhouse-lab/src/elements/](crates/greenhouse-lab/src/elements/) | the layout, the floor, one module per fixture and one per layer of the crop, and `leaflet.rs`, a leaflet's outline built from its veins; `mod.rs` holds the stage sets and the `generator!` call |

### Python

Three packages in one uv workspace, mirroring the crates: `python/misina-lab`
is the framework's Python side, pure Python over `usd-core` and mapped in the
framework's `AGENTS.md`; `crates/vinery-lab/python/vinery_lab` is the
vineyard's, a maturin package holding the compiled `_core` and the generated
cfg classes; `crates/greenhouse-lab/python/greenhouse_lab` is the
greenhouse's, laid out the same way.

| Path | What it is |
| --- | --- |
| [vinery_lab/isaaclab/vineyard_cfg.py](crates/vinery-lab/python/vinery_lab/isaaclab/vineyard_cfg.py) | `@configclass` fragments mirroring the Rust params (generated), and `VineyardCfg` over them |
| [vinery_lab/isaaclab/physics.py](crates/vinery-lab/python/vinery_lab/isaaclab/physics.py) | the vineyard's names and numbers over the framework's `rods`: which prim a vine is, the demos' substeps |
| [vinery_lab/isaaclab/vineyard.py](crates/vinery-lab/python/vinery_lab/isaaclab/vineyard.py) | the spawner under the vineyard's names, for the demos |
| [vinery_lab/isaaclab/newton_patches.py](crates/vinery-lab/python/vinery_lab/isaaclab/newton_patches.py) | workarounds for the pinned Newton, applied by the demos; not imported by the package |
| [vinery_lab/_core.pyi](crates/vinery-lab/python/vinery_lab/_core.pyi) | typed signatures for the compiled extension (generated) |

Only the `isaaclab` subpackages import Isaac Lab, so plain `import vinery_lab`,
`import greenhouse_lab` and `import misina_lab` work without it.

### Everything else

| Path | What it is |
| --- | --- |
| [tests/](tests/) | Python tests: the params, the USD build, the Isaac Lab cfg and physics |
| [examples/isaaclab_demo/](examples/isaaclab_demo/) | a quadruped walking the alleys; its own uv project |
| [examples/straddler_demo/](examples/straddler_demo/) | a straddling robot driving every row, `--trim` to hedge it, and `record.sh`, which films it; its own uv project |
| [examples/pruning_demo/](examples/pruning_demo/) | a Bumblebee-like arm pruning a dormant row to two-bud spurs, and `record.sh`, which films it; its own uv project |
| [examples/greenhouse_demo/](examples/greenhouse_demo/) | the greenhouse in Isaac Sim, which its docs picture is rendered from, and a pipe-rail robot harvesting it with a cut-and-hold shear, its arm planned by cuRobo; its own uv project |
| [examples/butom_capture/](examples/butom_capture/) | RGB-D sequences like BUTom-ST21's, filmed by a PATHoBot-like trolley whose cameras carry that dataset's calibration, written in its layout; its own uv project |
| [web/index.html](web/index.html) | host page for the wasm playground |
| [web/stage_docs.py](web/stage_docs.py) | copies the docs site's pages to where Zensical builds them from, pointing links to anything else at GitHub |
| [zensical.toml](zensical.toml) | the docs site; its nav lists every page on it |
| [docs/video/](docs/video/) | the README video: `record.sh` and the storyboard it plays, `encode.sh`, which every video is encoded with, and `publish.sh`, which uploads any of them to GitHub, the only host github.com plays video from, and points its README at it |
| [docs/shots/](docs/shots/) | the docs' pictures: a shot file per picture, in a folder named after what renders it; `render.sh`, which renders them into `target/docs-assets`, and `publish.sh`, which force-pushes that folder to the assets repository the docs link |

## Docs

- [DEVELOPMENT.md](DEVELOPMENT.md) — building, the checks, releasing, running the viewer, the web build, the docs site
- [crates/vinery-lab/README.md](crates/vinery-lab/README.md) — the vineyard generator: its features, the Isaac Lab workflow and the examples
- [crates/vinery-lab/docs/parameters.md](crates/vinery-lab/docs/parameters.md) — every parameter, with its default and range (generated)
- [crates/greenhouse-lab/README.md](crates/greenhouse-lab/README.md) — the greenhouse generator: what it builds, and its parameters page
- [.claude/skills/readme-video/SKILL.md](.claude/skills/readme-video/SKILL.md) — re-recording the README video and changing what it shows; plain Markdown, for any agent
- [.claude/skills/pruning-video/SKILL.md](.claude/skills/pruning-video/SKILL.md) — the same for the pruning demo's video, filmed in Isaac Sim
- [.claude/skills/docs-shots/SKILL.md](.claude/skills/docs-shots/SKILL.md) — rendering the docs' pictures from their shots, placing a shot's camera by hand, and publishing them
- [examples/isaaclab_demo/DEVELOPMENT.md](examples/isaaclab_demo/DEVELOPMENT.md) — the pinned Isaac Lab release and how to bump it
- [crates/misina-lab/docs/](crates/misina-lab/docs/) — the framework's: how a scene is built and exported, quantization, randomness, Isaac Lab, and how to add or change a parameter
- [crates/misina-lab/docs/AGENTS.md](crates/misina-lab/docs/AGENTS.md) — how a page is written and kept current

## Rules that bite

- **A parameter is declared once, on its Rust params struct.** The panel, the
  snippet, the Python class, the stub and `docs/parameters.md` all derive from
  it. [editing-parameters.md](crates/misina-lab/docs/editing-parameters.md)
  is the how-to.
- **Generated files are not hand-edited.** `cargo test` fails while
  `_core.pyi`, `isaaclab/vineyard_cfg.py` (both under `crates/vinery-lab/python/vinery_lab/`) or
  `crates/vinery-lab/docs/parameters.md` are stale; `cargo test regen_params -- --ignored`
  rewrites them. The greenhouse's are the same files under
  `crates/greenhouse-lab/`, rewritten with `-p greenhouse-lab`.
- **A user-facing change adds its line under Unreleased in
  [CHANGELOG.md](CHANGELOG.md)**, in the same change, marked the way the
  changelog's header says when it bumps the scene document format or changes
  a cfg class.
- **The `python` feature is off by default**, so a plain `cargo clippy` never
  sees the PyO3 side. Pass `--features vinery-lab/python,greenhouse-lab/python`,
  as CI does.
  `extension-module` is a different feature: it unlinks libpython, so it is
  for maturin builds only and breaks `cargo run` and `cargo test`.
- **Rust tests are inline `#[cfg(test)]` modules** beside the code they cover.
  The top-level `tests/` is Python only, and `test_docs.py` there fails on a
  Markdown link that resolves to nothing or points at a line number, and on
  a docs page missing from the site's nav in `zensical.toml`.
- **The scene is authored Z-up, in meters** (REP-103). Bevy renders Y-up; the
  correction is a single parent entity, above where the export walk starts.
- **Comments and docs are for someone who has only the code in front of
  them.** A note states a fact about the code or a rule to follow, not how it
  was found, and not the answer to a question only ever asked in a chat. If a
  detail doesn't change what the next reader does, cut it.

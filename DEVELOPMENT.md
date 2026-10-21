# Development

## Building the Python extension

No need to install `maturin` yourself — uv fetches it automatically as a
PEP 517 build backend.

The repo is a uv workspace of three packages. `python/misina-lab` is plain
Python: the USD builder and the Isaac Lab spawner every generator shares.
`crates/vinery-lab` is a maturin project in its *mixed* layout: hand-written
Python lives in `crates/vinery-lab/python/vinery_lab/`, and the compiled Rust
extension is built into it as the `_core` submodule, which `__init__.py`
re-exports. Only the `isaaclab` subpackages import Isaac Lab, so plain
`import vinery_lab` stays usable without it. `crates/greenhouse-lab` is laid
out the same way, as `greenhouse_lab`; `--reinstall-package greenhouse-lab`
rebuilds it.

How the binding surface is shaped — why fragments are `Py<T>`, what
`write_usd` releases the GIL around — is in the framework's
[generators.md](crates/misina-lab/docs/generators.md#the-python-module).

### Iterating on the wrapper itself

    uv sync                                  # at the repo root; builds the extension
    uv sync --reinstall-package vinery-lab    # after a Rust change

### Consuming it as a dependency (e.g. examples/isaaclab_demo)

    cd examples/isaaclab_demo
    uv sync
    uv run python main.py

If you only changed Rust source (not pyproject.toml), force a rebuild:

    uv sync --reinstall-package vinery-lab

### Build the distributions

    CARGO_TARGET_DIR=$PWD/target uv build --all-packages

An sdist and a wheel for each package, into `dist/`, as the release workflow
builds them. Each wheel is built from its sdist in a scratch directory; the
target directory lets them reuse this checkout's compiled dependencies instead
of building Bevy once per generator.

## Checks

`.github/workflows/ci.yml` runs all of these on every push and pull request:

    cargo fmt --all -- --check
    cargo clippy --workspace --all-targets --features vinery-lab/python,greenhouse-lab/python -- -D warnings
    cargo test --workspace
    ruff check . && ruff format --check .
    mypy
    pytest tests

Each generator's `python` feature is what puts the PyO3 side — `crates/misina-lab/src/python.rs`
and what the `generator!` macro emits — in front of clippy. It is out of the
default feature set, so a plain `cargo clippy` never sees it.

`cargo test` also fails while a generator's Python stub, its Isaac Lab cfg
classes or its `docs/parameters.md` are stale against the Rust params
structs they are generated from. `cargo test regen_params -- --ignored`
rewrites them — see
[editing-parameters.md](crates/misina-lab/docs/editing-parameters.md).

Isaac Lab is not installed on the runner, so `tests/test_isaaclab_cfg.py` skips
itself there. It runs locally, from the demo venv:

    examples/isaaclab_demo/.venv/bin/python -m pytest tests/ -q -s < /dev/null

Capture has to be off: pytest's stdin capture breaks Kit's kernel bootstrap.
The demo's own tests live beside its code and run the same way:

    examples/isaaclab_demo/.venv/bin/python -m pytest examples/isaaclab_demo -q -s < /dev/null

Formatting and the fast lints are also available as commit hooks:

    uvx pre-commit install

Clippy and the test suites stay out of them — a Bevy rebuild is too slow to sit
in front of a commit.

## Releasing

`misina-lab`, `vinery-lab` and `greenhouse-lab` release to PyPI together, under
the workspace version, `[workspace.package]` in the root `Cargo.toml`. The
generators' wheels take it from their crates. Two other places spell it out:
`version` in `python/misina-lab/pyproject.toml`, and the `misina-lab==` pin in
each generator's `pyproject.toml`. `tests/test_versions.py` fails until all
four agree.

1. Bump the version in all four places and merge.
2. Tag the merge commit `v<version>` and push the tag.

`.github/workflows/release.yml` then builds an sdist and a Linux x86_64 wheel
for each package and publishes them. A manual run of it from a branch builds
them without publishing.

Publishing goes through PyPI's trusted publishing, so no token is stored. Each
project's publisher on PyPI names the `azazdeaz/misina-lab` repository, the
`release.yml` workflow and the `pypi` environment. A project not on PyPI yet
takes a pending publisher, which its first upload turns into the project.
Renaming the repository means updating all three publishers.

## Viewer (interactive)

    cargo run --release                         # the vineyard
    cargo run --release -p greenhouse-lab       # the greenhouse

The panel is built from the params structs: one section per fragment of
`VineyardParams`, one control per field, with the caption, range and tooltip
read off the field's declaration — see `crates/misina-lab/src/ui.rs` and
[editing-parameters.md](crates/misina-lab/docs/editing-parameters.md). Sliders write a
staged copy of the params; a value reaches the live resources once it has held
still for 150 ms, and re-runs the layers below it. Dragging one is a single
rebuild rather than one per frame.

Clicking a slider without dragging it turns it into a text field, as Blender's
sliders do: the number starts selected, `Enter` or clicking away applies it,
`Escape` drops it. A typed value is clamped to the slider's range.

The footer's **Colliders** box draws what a physics engine collides with
over the scene: in magenta the capsules nothing moves, the posts' and the
wood's proxies and each rod's held first segment; in yellow every other rod
segment, at the thickness it is simulated at rather than the width it is
drawn at; in cyan the meshes that collide as themselves, the ground among
them. See `crates/misina-lab/src/colliders.rs`.

Press `S` to write the scene out as `scene.json`, and build it with:

    python -m misina_lab.usd scene.json scene.usd

Press `P` to log the camera as a storyboard's `camera`, and copy it: how a
view found by hand goes into a storyboard or a [docs picture](#docs-pictures).

`VINERY_LAB_PERF=1 cargo run` logs a per-layer breakdown on any frame that
rebuilt something — the marks are `crates/vinery-lab/src/perf.rs`, the report
`crates/misina-lab/src/perf.rs`.

`VINERY_LAB_RECORD=demo.mp4 cargo run --release` records the window for the
whole run. One captured frame becomes one video frame, so the stall a rebuild
causes costs a frame rather than the freeze a screen recorder would keep —
which is the point of it, for demo videos. Frames are piped to `ffmpeg`, which
has to be on `PATH`; the extension picks the container. `VINERY_LAB_RECORD_FPS`
sets the rate, 30 by default, and is also how fast the window is sampled. See
`crates/misina-lab/src/record.rs`.

`VINERY_LAB_TOUR=docs/video/tour.json cargo run --release` plays a storyboard
of camera moves and panel edits, then hands the window back to the pointer.
With `VINERY_LAB_RECORD` set too its clock advances one video frame per
rendered frame, so the recording is smooth however long the rebuilds take, and
the app quits at the end; `docs/video/record.sh` records the README video that
way. `VINERY_LAB_SHOT=still.png` instead saves the window as the storyboard
ends, and quits. See `crates/misina-lab/src/tour.rs`, and
[the skill](.claude/skills/readme-video/SKILL.md) for changing the video.

The pruning demo's video is filmed in Isaac Sim instead, headless:
`examples/pruning_demo/record.sh` runs the demo's `video.py`, which samples
an RTX camera on the simulation's clock. Both scripts encode what they record
with `docs/video/encode.sh`. See
[its skill](.claude/skills/pruning-video/SKILL.md) for changing it.
`docs/video/publish.sh` uploads either video to the branch's pull request
and puts the link GitHub gives it in its README. github.com plays only video
GitHub hosts, so the videos stay uploads; the pictures are published apart,
see [Docs pictures](#docs-pictures).

### Web build

The same viewer, compiled to wasm, is the playground at
[azazdeaz.github.io/misina-lab/playground/vinery](https://azazdeaz.github.io/misina-lab/playground/vinery/).
The manually triggered `.github/workflows/playground.yml` builds it and
attaches the bundle to the `playground` release, which the site is deployed
from; see [Docs site](#docs-site). To reproduce the build locally:

    rustup target add wasm32-unknown-unknown
    cargo install wasm-bindgen-cli --version 0.2.127   # must match Cargo.lock
    cargo build --profile wasm-release --target wasm32-unknown-unknown --bin vinery-lab
    mkdir -p site/playground/vinery && cp web/index.html site/playground/vinery/
    wasm-bindgen --target web --no-typescript --out-dir site/playground/vinery \
      target/wasm32-unknown-unknown/wasm-release/vinery-lab.wasm
    python3 -m http.server -d site 8000      # localhost:8000/playground/vinery/

A docs build empties `site/` first, so build the playground after it. Serve
it rather than opening the file: the module is fetched, and `localhost`
is a secure context, which both WebGPU and the clipboard require.

Two things differ from the native build. The renderer is WebGPU — the `webgpu`
feature in `crates/vinery-lab/Cargo.toml` is target-gated, and WebGL2 would lose compute shaders,
which this many meshes need. And the `S` key is compiled out, since there is no
filesystem to write `scene.json` to; **Copy Isaac Lab cfg** is the whole export
path on the web.

## Docs pictures

The pictures in the docs are rendered from shots, JSON files under
`docs/shots/` that hold the scene's parameters, the camera, the size, and for
Isaac Lab how long the robot walks first:

    docs/shots/render.sh     # every shot, into target/docs-assets
    docs/shots/publish.sh    # force-pushes that folder to the assets repository

`docs/shots/A/B.json` becomes `A/B.webp` in
[azazdeaz/misina-lab-docs-assets](https://github.com/azazdeaz/misina-lab-docs-assets),
which holds only the current set, and a page links it as
`https://raw.githubusercontent.com/azazdeaz/misina-lab-docs-assets/main/A/B.webp`.
A viewer shot is a storyboard with no steps; an Isaac Lab shot is read by the
example's `shot.py`. Both take the camera in a storyboard's terms, and both
apps print their camera on `P`, so a view placed by hand goes straight into a
shot. [The skill](.claude/skills/docs-shots/SKILL.md) is the how-to.

## Docs site

[azazdeaz.github.io/misina-lab](https://azazdeaz.github.io/misina-lab/) is
built with [Zensical](https://zensical.org) from the pages where they live in
the repository. The nav in `zensical.toml` lists every page on the site by
its path, and `web/stage_docs.py` copies those pages to `target/docs` under
the same paths: a link between two pages works as written, and every other
link, to source or to a page left off the site, is pointed at GitHub at the
commit being built. A line holding only a `github.com/user-attachments`
URL, which GitHub plays as a video, becomes a `<video>` tag in the staged copy.
`tests/test_docs.py` fails while a page of a docs folder
is missing from the nav.

    python3 web/stage_docs.py
    uv run --only-group docs zensical serve    # localhost:8000

The server rebuilds when the staged copy changes, so re-run the script after
editing a page. `zensical build` writes the site to `site/` instead.

`.github/workflows/pages.yml` builds the site on every push to main and after
every successful playground build, unpacks each bundle of the `playground`
release under `/playground/`, and deploys the lot. A Pages deploy replaces the
whole site, so it is the only workflow that deploys. It fails while that
release does not exist, so the Playground workflow has to have run once.

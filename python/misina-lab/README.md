# misina-lab

The Python half of [misina-lab](https://github.com/azazdeaz/misina-lab), a
framework for procedural scene generators for robotics simulation. A
generator is written in Rust on the `misina-lab` crate: it builds a scene in
[Bevy](https://bevy.org) and writes it out as a JSON scene document. This
package turns that document into something a simulator loads.

- `misina_lab.usd` builds the document into a USD stage, Z-up and in meters:
  a material per part, static colliders, and the ground as a height field.
- `misina_lab.isaaclab` spawns a generator's scene in
  [Isaac Lab](https://isaac-sim.github.io/IsaacLab/) 3.0.0. It generates the
  scene on first use, caches it as a USD file, and spawns it once however
  many environments there are. It also has flexible rods on Newton, cutting
  them while the simulation runs, a view of every collider, and tools that
  render stills and film videos.

## Install

```bash
pip install misina-lab
```

It needs Python 3.12 or later and brings `usd-core` with it.
`misina_lab.isaaclab` needs Isaac Lab installed beside it; `import
misina_lab` and `misina_lab.usd` work without it.

A project usually gets this package through a generator's Python package,
which pins it exactly, `misina-lab==X.Y.Z`, at the version of the
`misina-lab` crate the generator is built with. The builder reads only the
scene document format of its own release, so the two versions always match.

A saved scene document builds from the command line:

```bash
python -m misina_lab.usd scene.json scene.usd
```

## Documentation

- [The docs site](https://azazdeaz.github.io/misina-lab/): the framework,
  the reference generators built on it, and their examples.
- The framework's pages on this package: how a scene is
  [exported](https://azazdeaz.github.io/misina-lab/crates/misina-lab/docs/export/)
  and spawned in
  [Isaac Lab](https://azazdeaz.github.io/misina-lab/crates/misina-lab/docs/isaac-lab/),
  and [rods](https://azazdeaz.github.io/misina-lab/crates/misina-lab/docs/rods/).
- The Rust crate on [crates.io](https://crates.io/crates/misina-lab) and
  [docs.rs](https://docs.rs/misina-lab).
- The [changelog](https://github.com/azazdeaz/misina-lab/blob/main/CHANGELOG.md).

## Licence

MIT or Apache-2.0, at your option.

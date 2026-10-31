//! `garden-lab` — raised vegetable beds, for robotics simulation.
//!
//! The scene is a garden of raised beds on a flat floor: the beds' timber
//! frames and the soil in them, and, as the crate grows, the crops planted
//! in them. The parts are [`elements`]; everything around them — the panel,
//! the export, the generated Python and docs — is [`misina_lab`], keyed on
//! [`Garden`].

pub mod elements;
pub mod grow;
pub mod shape;
pub mod species;

use bevy::prelude::*;
use misina_lab::Generator;

/// The generator, as `misina_lab` sees it.
pub struct Garden;

impl Generator for Garden {
    type Params = elements::GardenParams;
    const NAME: &'static str = "Garden";
    const PACKAGE: &'static str = "garden_lab";

    fn plugin(app: &mut App) {
        elements::plugin(app);
    }
}

/// The extension module, `garden_lab._core`. PyO3 names the module's init
/// symbol after this function, so it has to match the last component of
/// `module-name` in `pyproject.toml`. What it holds is written by the
/// `generator!` call in [`elements`].
#[cfg(feature = "python")]
#[pyo3::pymodule]
fn _core(m: &pyo3::Bound<'_, pyo3::types::PyModule>) -> pyo3::PyResult<()> {
    elements::module(m)
}

#[cfg(test)]
mod tests {
    use std::path::PathBuf;

    use super::*;
    use misina_lab::codegen;

    /// The parameters page, relative to this crate; the Python targets are
    /// under `python/garden_lab/` and need no naming.
    const DOCS: &str = "docs/parameters.md";

    fn root() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
    }

    /// Every generated region reads as the structs say now.
    #[test]
    fn generated_python_and_docs_are_fresh() {
        let stale: Vec<PathBuf> = codegen::stale::<Garden>(&root(), DOCS)
            .into_iter()
            .map(|(path, _)| path)
            .collect();
        assert!(
            stale.is_empty(),
            "stale: {stale:?} — run `cargo test -p garden-lab regen_params -- --ignored`"
        );
    }

    /// Rewrites the generated files from the structs. A dev tool rather than
    /// a test: it writes files and asserts nothing.
    #[test]
    #[ignore]
    fn regen_params() {
        for (path, fresh) in codegen::stale::<Garden>(&root(), DOCS) {
            std::fs::write(path, fresh).unwrap();
        }
    }
}

//! `greenhouse-lab` — the rows of a Western European high-wire tomato
//! greenhouse, for robotics simulation.
//!
//! The scene is what a robot touches or sees up close in a row: the floor,
//! the pipe rails, the gutters with their slabs and blocks, the crop wires,
//! and the crop's stems, leaves, trusses and fruit. The building around them comes from an HDR or a Gaussian splat and
//! is not generated. The parts are [`elements`]; everything around them — the
//! panel, the export, the generated Python and docs — is [`misina_lab`],
//! keyed on [`Greenhouse`].

pub mod elements;

use bevy::prelude::*;
use misina_lab::Generator;

/// The generator, as `misina_lab` sees it.
pub struct Greenhouse;

impl Generator for Greenhouse {
    type Params = elements::GreenhouseParams;
    const NAME: &'static str = "Greenhouse";
    const PACKAGE: &'static str = "greenhouse_lab";

    fn plugin(app: &mut App) {
        elements::plugin(app);
    }
}

/// The extension module, `greenhouse_lab._core`. PyO3 names the module's init
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
    /// under `python/greenhouse_lab/` and need no naming.
    const DOCS: &str = "docs/parameters.md";

    fn root() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
    }

    /// Every generated region reads as the structs say now.
    #[test]
    fn generated_python_and_docs_are_fresh() {
        let stale: Vec<PathBuf> = codegen::stale::<Greenhouse>(&root(), DOCS)
            .into_iter()
            .map(|(path, _)| path)
            .collect();
        assert!(
            stale.is_empty(),
            "stale: {stale:?} — run `cargo test regen_params -- --ignored`"
        );
    }

    /// Rewrites the generated files from the structs. A dev tool rather than
    /// a test: it writes files and asserts nothing.
    #[test]
    #[ignore]
    fn regen_params() {
        for (path, fresh) in codegen::stale::<Greenhouse>(&root(), DOCS) {
            std::fs::write(path, fresh).unwrap();
        }
    }
}

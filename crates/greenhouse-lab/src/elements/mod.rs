//! Elements — the parts of a greenhouse row, one module each.
//!
//! [`layout`] solves where the rows and paths go and spawns a prim for each,
//! the fixtures fill those prims, and the crop grows on them: [`stem`] roots
//! a plant on every block and hangs leaves and trusses off its stems, [`leaf`]
//! builds the leaves from [`leaflet`] outlines, [`truss`] hangs fruit off the
//! trusses, and [`fruit`] shapes and colours them. Every element
//! holds its params and a build system that replaces everything it spawned
//! whenever its params or the layout change. The panel, the config snippet and the Python classes
//! are read off the params structs — see
//! `crates/misina-lab/docs/editing-parameters.md`.
//!
//! Rows run along +X from the row head at x = 0, the concrete main path lies
//! at negative x, and the top of the floor is z = 0.

pub mod floor;
pub mod fruit;
pub mod gutter;
pub mod layout;
pub mod leaf;
pub mod leaflet;
pub mod rail;
pub mod stem;
pub mod truss;
pub mod wire;

use bevy::prelude::*;
use misina_lab::Build;
use misina_lab::geometry::mesh::{MeshData, box_mesh};
use misina_lab::palette::Response;
use misina_lab::params::{Label, Slider};

/// Build order: the layout spawns the row and path prims the fixtures fill,
/// and each layer of the crop builds the configs the one above it authored.
#[derive(SystemSet, Debug, Clone, PartialEq, Eq, Hash)]
pub enum Grow {
    /// The rows and the paths.
    Layout,
    /// The floor, the rails, the gutters and the wires.
    Fixtures,
    Stems,
    Leaves,
    Trusses,
    Fruit,
}

pub fn plugin(app: &mut App) {
    app.init_resource::<SceneParams>()
        .configure_sets(
            PreUpdate,
            (
                Grow::Layout,
                Grow::Fixtures,
                Grow::Stems,
                Grow::Leaves,
                Grow::Trusses,
                Grow::Fruit,
            )
                .chain()
                .in_set(Build),
        )
        .add_plugins((
            layout::plugin,
            floor::plugin,
            rail::plugin,
            gutter::plugin,
            wire::plugin,
            stem::plugin,
            leaf::plugin,
            truss::plugin,
            fruit::plugin,
        ));
}

/// Scene-wide parameters: the seed, and how far into the season the crop is.
#[derive(Resource, Reflect, Clone, Debug, PartialEq)]
#[cfg_attr(
    feature = "python",
    pyo3::pyclass(get_all, set_all, skip_from_py_object)
)]
pub struct SceneParams {
    /// The one seed the whole scene is generated from. Each element salts it
    /// with a constant of its own, so a new seed is a different greenhouse and
    /// an edit to one element never re-rolls another.
    #[reflect(@Slider { min: 0.0, max: 64.0, step: 1.0 })]
    pub seed: u64,
    /// Weeks since the crop was planted. A stem grows about a truss a week up
    /// its twine, reaches the wire after about nine weeks at the defaults,
    /// when its first trusses ripen, and is lowered along the gutter from
    /// then on.
    ///
    /// Scene-wide rather than the stems', because it is one date that every
    /// layer of the crop reads.
    #[reflect(@Slider { min: 0.0, max: 40.0, step: 1.0 })]
    pub week: f32,
}

impl Default for SceneParams {
    fn default() -> Self {
        Self {
            seed: 0,
            week: 16.0,
        }
    }
}

misina_lab::generator! {
    /// Every element's params as one value: what a Python call, a headless
    /// build and the viewer panel's staged copy hold.
    pub struct GreenhouseParams as PyGreenhouseParams("GreenhouseParams") for crate::Greenhouse {
        pub scene: SceneParams,
        #[reflect(@Label("Rows"))]
        pub layout: layout::LayoutParams,
        #[reflect(@Label("Pipe rail"))]
        pub rail: rail::RailParams,
        pub gutter: gutter::GutterParams,
        #[reflect(@Label("Crop wires"))]
        pub wire: wire::WireParams,
        #[reflect(@Label("Stems"))]
        pub stem: stem::StemParams,
        #[reflect(@Label("Leaves"))]
        pub leaf: leaf::LeafParams,
        #[reflect(@Label("Trusses"))]
        pub truss: truss::TrussParams,
        pub fruit: fruit::FruitParams,
    }
}

/// Galvanized steel: the rails, the gutters, their supports and the wires.
pub const STEEL: Response = Response {
    roughness: 0.45,
    reflectance: 0.5,
};

/// A box spanning `min` to `max`.
pub fn cuboid(min: Vec3, max: Vec3) -> MeshData {
    box_mesh(1.0).transformed(&Transform::from_translation((min + max) / 2.0).with_scale(max - min))
}

/// `v` in nalgebra's terms, which the geometry kernels take.
pub fn vector(v: Vec3) -> nalgebra::Vector3<f64> {
    nalgebra::Vector3::new(v.x as f64, v.y as f64, v.z as f64)
}

/// `t` eased in and out over `0..1`.
pub fn smooth(t: f32) -> f32 {
    let t = t.clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

#[cfg(test)]
pub(crate) fn grown(params: GreenhouseParams) -> App {
    misina_lab::testing::grown::<crate::Greenhouse>(params)
}

/// The default params on a short row or two, for a test that only needs a
/// few plants.
#[cfg(test)]
pub(crate) fn small() -> GreenhouseParams {
    let mut params = GreenhouseParams::default();
    (params.layout.rows, params.layout.row_length) = (2, 6.0);
    params
}

/// Where `entity` is in the scene: its transform composed with its
/// ancestors' up to the scene root. A headless build propagates none.
#[cfg(test)]
pub(crate) fn in_scene(world: &World, mut entity: Entity) -> Transform {
    let root = world.resource::<misina_lab::scene::PrimRoot>().0;
    let mut placed = Transform::IDENTITY;
    while entity != root {
        let at = world.entity(entity);
        placed = at.get::<Transform>().copied().unwrap_or_default() * placed;
        entity = at.get::<ChildOf>().unwrap().parent();
    }
    placed
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeSet;

    use misina_lab::Params;
    use misina_lab::scene::doc::Node;
    use misina_lab::scene::export::{scene_doc, scene_json};
    use misina_lab::testing::{check_params, nudged};

    use super::*;

    /// What the panel, the snippet and the generated docs need of every field
    /// — see `docs/editing-parameters.md`.
    #[test]
    fn every_field_declares_what_the_panel_and_the_docs_need() {
        check_params::<GreenhouseParams>();
    }

    /// `apply` reaches every fragment: a set with every field moved lands in a
    /// bare world and reads back out whole.
    #[test]
    fn apply_and_read_round_trip_every_fragment() {
        let params = nudged::<GreenhouseParams>();
        let mut world = World::new();
        params.apply(&mut world);
        assert_eq!(GreenhouseParams::read(&world), params);
    }

    /// Every prim of the document, with its path.
    fn walk<'a>(node: &'a Node, at: &str, into: &mut Vec<(String, &'a Node)>) {
        let path = format!("{at}/{}", node.name);
        for child in &node.children {
            walk(child, &path, into);
        }
        into.push((path, node));
    }

    /// Three things a USD consumer assumes and never checks: every reference
    /// names a part, every part is drawn by some prim, and no two prims share
    /// a path, which a config keyed on one would be silently repointed by. At
    /// the defaults, and with the lowest trusses bent.
    #[test]
    fn every_reference_resolves_and_every_path_is_unique() {
        let mut bent = GreenhouseParams::default();
        bent.stem.flexible_below = 1.8;
        for params in [GreenhouseParams::default(), bent] {
            let doc = scene_doc(grown(params).world_mut()).unwrap();
            let mut prims = Vec::new();
            walk(&doc.root, "", &mut prims);

            let paths: BTreeSet<&str> = prims.iter().map(|(path, _)| path.as_str()).collect();
            assert_eq!(paths.len(), prims.len(), "a prim path repeats");
            let drawn: BTreeSet<&str> = prims
                .iter()
                .filter_map(|(_, node)| node.reference.as_deref())
                .collect();
            let parts: BTreeSet<&str> = doc.parts.iter().map(|part| part.name.as_str()).collect();
            assert_eq!(drawn, parts);
        }
    }

    /// An edit to a standing scene lands exactly where generating from scratch
    /// would: no element may leave a prim of its last build behind. The two
    /// sides are separate runs, so this is also the check that a scene comes
    /// out byte for byte the same every time.
    #[test]
    fn an_edit_lands_where_generating_would_have() {
        let edited = |edit: fn(&mut GreenhouseParams)| {
            let mut params = small();
            edit(&mut params);
            params
        };
        let json = |app: &mut App| scene_json(app.world_mut()).unwrap();
        for (what, want) in [
            ("rows", edited(|p| p.layout.rows = 3)),
            ("mount", edited(|p| p.gutter.mount = "standing".into())),
            ("unevenness", edited(|p| p.rail.unevenness = 1.5)),
            ("wire height", edited(|p| p.wire.height = 4.0)),
            ("week", edited(|p| p.scene.week = 4.0)),
            ("no trusses yet", edited(|p| p.scene.week = 0.0)),
            ("leaves kept", edited(|p| p.stem.leaves_kept = 12)),
            ("leaf length", edited(|p| p.leaf.length = 0.5)),
            ("leaf detail", edited(|p| p.leaf.detail = 0.5)),
            ("stem detail", edited(|p| p.stem.detail = 2.0)),
            ("peduncle", edited(|p| p.truss.peduncle_length = 0.1)),
            ("bent trusses", edited(|p| p.stem.flexible_below = 1.8)),
            ("ripeness", edited(|p| p.fruit.ripe_trusses = 2.5)),
            ("seed", edited(|p| p.scene.seed = 7)),
        ] {
            let mut app = grown(small());
            want.apply(app.world_mut());
            app.update();
            assert!(json(&mut app) == json(&mut grown(want)), "{what} drifted");
        }
    }
}

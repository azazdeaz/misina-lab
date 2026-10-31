//! Elements — the parts of a garden, one module each.
//!
//! [`layout`] solves where the beds go and spawns a prim for each; the
//! fixtures fill them, [`floor`] the ground the garden stands on and [`bed`]
//! the frame and the soil of every bed; [`planting`] puts a plant of some
//! species in every planting place; and the crop grows on them, a system
//! per species in each layer: [`plant`] runs a species' recipe into organs,
//! [`leaf`] meshes its leaves and [`fruit`] its fruit and roots. Every
//! element holds its params and a
//! build system that replaces everything it spawned whenever its params or
//! what it builds on change. The panel, the config snippet and the Python
//! classes are read off the params structs — see
//! `crates/misina-lab/docs/editing-parameters.md`.
//!
//! +Y is north. Beds run east–west along X and are numbered across Y from the
//! south row, the garden is centred on the origin, and the top of the floor
//! is z = 0.

pub mod bed;
pub mod floor;
pub mod fruit;
pub mod layout;
pub mod leaf;
pub mod plant;
pub mod planting;

use bevy::prelude::*;
use misina_lab::Build;
use misina_lab::geometry::mesh::{MeshData, box_mesh};
use misina_lab::params::{Label, Slider};

use crate::species::{self, Commons, carrot, kale, lettuce};

/// Build order: the species publish their common knobs, the layout spawns
/// the bed prims the fixtures fill, planting fills the beds, and each layer
/// of the crop builds the configs the one above it authored.
#[derive(SystemSet, Debug, Clone, PartialEq, Eq, Hash)]
pub enum Grow {
    /// Every species' common knobs into [`Commons`].
    Species,
    /// Where the beds go, and how far the floor reaches.
    Layout,
    /// The floor, and the frame and the soil of every bed.
    Fixtures,
    /// A plant in every planting place.
    Planting,
    /// Every plant's organs, from its species' recipe.
    Plants,
    /// Every species' leaves, meshed and shared.
    Leaves,
    /// Every species' fruit and roots, meshed and shared.
    Fruit,
}

pub fn plugin(app: &mut App) {
    app.init_resource::<SceneParams>()
        .init_resource::<PhysicsParams>()
        .init_resource::<Commons>()
        .configure_sets(
            PreUpdate,
            (
                Grow::Species,
                Grow::Layout,
                Grow::Fixtures,
                Grow::Planting,
                Grow::Plants,
                Grow::Leaves,
                Grow::Fruit,
            )
                .chain()
                .in_set(Build),
        )
        .add_plugins((layout::plugin, floor::plugin, bed::plugin, planting::plugin));
    species::species::<lettuce::Lettuce>(app);
    species::species::<kale::Kale>(app);
    species::species::<carrot::Carrot>(app);
}

/// Scene-wide parameters: the seed, and how far into the season the garden is.
#[derive(Resource, Reflect, Clone, Debug, PartialEq)]
#[cfg_attr(
    feature = "python",
    pyo3::pyclass(get_all, set_all, skip_from_py_object)
)]
pub struct SceneParams {
    /// The one seed the whole scene is generated from. Each element salts it
    /// with a constant of its own, so a new seed is a different garden and an
    /// edit to one element never re-rolls another.
    #[reflect(@Slider { min: 0.0, max: 64.0, step: 1.0 })]
    pub seed: u64,
    /// The week on the garden's calendar. Each crop grows from its own
    /// sowing week toward its harvest, so one garden holds seedlings beside
    /// a bed ready to pick.
    #[reflect(@Slider { min: 0.0, max: 20.0, step: 1.0 })]
    pub week: f32,
}

impl Default for SceneParams {
    fn default() -> Self {
        Self { seed: 0, week: 8.0 }
    }
}

/// What a robot bumps into, besides the floor and the beds. Scene-wide,
/// since a demo wants one switch.
#[derive(Resource, Reflect, Clone, Debug, PartialEq)]
#[cfg_attr(
    feature = "python",
    pyo3::pyclass(get_all, set_all, skip_from_py_object)
)]
pub struct PhysicsParams {
    /// Whether every stem collides, as a chain of capsules along it.
    pub stems_collide: bool,
    /// Whether every fruit and root collides, as a capsule each.
    pub fruit_collides: bool,
}

impl Default for PhysicsParams {
    fn default() -> Self {
        Self {
            stems_collide: true,
            fruit_collides: false,
        }
    }
}

misina_lab::generator! {
    /// Every element's params as one value: what a Python call, a headless
    /// build and the viewer panel's staged copy hold.
    pub struct GardenParams as PyGardenParams("GardenParams") for crate::Garden {
        pub scene: SceneParams,
        pub physics: PhysicsParams,
        #[reflect(@Label("Beds"))]
        pub bed: layout::BedParams,
        pub planting: planting::PlantingParams,
        pub lettuce: lettuce::LettuceParams,
        pub kale: kale::KaleParams,
        pub carrot: carrot::CarrotParams,
    }
}

/// A box spanning `min` to `max`.
pub fn cuboid(min: Vec3, max: Vec3) -> MeshData {
    box_mesh(1.0).transformed(&Transform::from_translation((min + max) / 2.0).with_scale(max - min))
}

#[cfg(test)]
pub(crate) fn grown(params: GardenParams) -> App {
    misina_lab::testing::grown::<crate::Garden>(params)
}

/// The default params on one bed, for a test that only needs a few plants.
#[cfg(test)]
pub(crate) fn small() -> GardenParams {
    let mut params = GardenParams::default();
    (params.bed.columns, params.bed.rows) = (1, 1);
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
    use misina_lab::scene::COLLISION;
    use misina_lab::scene::doc::Node;
    use misina_lab::scene::export::{scene_doc, scene_json};
    use misina_lab::testing::{check_params, named_children, nudged, organs, prim};

    use super::fruit::FruitConfig;
    use super::leaf::LeafConfig;
    use super::planting::{PlantConfig, SQUARE};
    use super::*;

    /// What the panel, the snippet and the generated docs need of every field
    /// — see `docs/editing-parameters.md`.
    #[test]
    fn every_field_declares_what_the_panel_and_the_docs_need() {
        check_params::<GardenParams>();
    }

    /// `apply` reaches every fragment: a set with every field moved lands in a
    /// bare world and reads back out whole.
    #[test]
    fn apply_and_read_round_trip_every_fragment() {
        let params = nudged::<GardenParams>();
        let mut world = World::new();
        params.apply(&mut world);
        assert_eq!(GardenParams::read(&world), params);
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
    /// a path, which a config keyed on one would be silently repointed by.
    #[test]
    fn every_reference_resolves_and_every_path_is_unique() {
        let doc = scene_doc(grown(small()).world_mut()).unwrap();
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

    /// An edit to a standing scene lands exactly where generating from scratch
    /// would: no element may leave a prim of its last build behind. The two
    /// sides are separate runs, so this is also the check that a scene comes
    /// out byte for byte the same every time.
    #[test]
    fn an_edit_lands_where_generating_would_have() {
        let edited = |edit: fn(&mut GardenParams)| {
            let mut params = small();
            edit(&mut params);
            params
        };
        let json = |app: &mut App| scene_json(app.world_mut()).unwrap();
        for (what, want) in [
            ("columns", edited(|p| p.bed.columns = 2)),
            ("path", edited(|p| p.bed.path = 0.5)),
            ("height", edited(|p| p.bed.height = 0.6)),
            ("width", edited(|p| p.bed.width = 1.5)),
            ("week", edited(|p| p.scene.week = 3.0)),
            ("seed", edited(|p| p.scene.seed = 7)),
            ("jitter", edited(|p| p.planting.jitter = 0.0)),
            ("misses", edited(|p| p.planting.misses = 0.3)),
            ("leaves", edited(|p| p.lettuce.leaves = 20)),
            ("ruffle", edited(|p| p.lettuce.ruffle = 0.0)),
            ("heading", edited(|p| p.lettuce.heading = true)),
            ("tint", edited(|p| p.lettuce.tint = "red".into())),
            ("spacing", edited(|p| p.lettuce.spacing = 0.3)),
            ("no lettuce", edited(|p| p.lettuce.share = 0.0)),
            ("sown later", edited(|p| p.lettuce.sown_week = 4.0)),
            ("variations", edited(|p| p.lettuce.variations = 4)),
            ("leaf length", edited(|p| p.lettuce.leaf_length = 0.25)),
            ("one bed only", edited(|p| p.lettuce.bed = 1)),
            ("rows", edited(|p| p.bed.rows = 2)),
            ("rim", edited(|p| p.bed.rim = 0.0)),
            (
                "no stem colliders",
                edited(|p| p.physics.stems_collide = false),
            ),
            ("fruit collide", edited(|p| p.physics.fruit_collides = true)),
            ("kale height", edited(|p| p.kale.height = 0.9)),
            ("kale stripped", edited(|p| p.kale.leaves_kept = 6)),
            ("no kale", edited(|p| p.kale.share = 0.0)),
            ("carrot leaves", edited(|p| p.carrot.leaves = 12)),
            ("carrot root", edited(|p| p.carrot.root_length = 0.1)),
            ("carrots later", edited(|p| p.carrot.sown_week = 6.0)),
        ] {
            let mut app = grown(small());
            want.apply(app.world_mut());
            app.update();
            assert!(json(&mut app) == json(&mut grown(want)), "{what} drifted");
        }
    }

    /// The default params on one bed of lettuce alone.
    fn lettuce_bed() -> GardenParams {
        let mut params = small();
        (params.kale.share, params.carrot.share) = (0.0, 0.0);
        params
    }

    /// Four lettuces to every whole 30 cm square of a bed's soil, each on the
    /// soil surface inside the frame, when none is missing.
    #[test]
    fn a_bed_holds_four_lettuces_to_a_square() {
        let mut params = lettuce_bed();
        params.planting.misses = 0.0;
        let (bed, rim) = (params.bed.clone(), params.bed.rim);
        let mut app = grown(params);
        let bed_prim = prim(app.world_mut(), &["Beds", "Bed_00"]).unwrap();
        let plants: Vec<Entity> = named_children(app.world_mut(), bed_prim)
            .into_iter()
            .filter(|(name, _)| name.starts_with("Plant_"))
            .map(|(_, plant)| plant)
            .collect();
        let inner = Vec2::new(bed.length, bed.width) - Vec2::splat(2.0 * layout::BOARD);
        let squares = (inner / SQUARE).floor();
        assert_eq!(plants.len(), (squares.x * squares.y) as usize * 4);
        for plant in plants {
            let at = app
                .world()
                .entity(plant)
                .get::<Transform>()
                .unwrap()
                .translation;
            assert!(
                at.x.abs() < inner.x / 2.0 && at.y.abs() < inner.y / 2.0,
                "{at}"
            );
            assert!((at.z - (bed.height - rim)).abs() < 1e-6, "{at}");
        }
    }

    /// Every week builds every species, with every leaf and every fruit
    /// anchored at or above the soil, more leaves the older the plant, and a
    /// plant grown by a week keeping the leaves it had: the same azimuth and
    /// the same stream at every rank.
    #[test]
    fn a_plant_keeps_its_leaves_as_it_grows() {
        let mut params = small();
        params.planting.misses = 0.0;
        let soil_z = params.bed.height - params.bed.rim;
        let mut counts = Vec::new();
        let mut first: Vec<Vec<(f32, u64)>> = Vec::new();
        for week in [0.0, 4.0, 5.0, 8.0, 14.0] {
            params.scene.week = week;
            let mut app = grown(params.clone());
            let leaves = organs::<LeafConfig>(app.world_mut());
            assert!(!leaves.is_empty(), "week {week}");
            assert!(
                !organs::<FruitConfig>(app.world_mut()).is_empty(),
                "week {week}"
            );
            let world = app.world_mut();
            let mut query = world
                .query_filtered::<(Entity, &Name), Or<(With<LeafConfig>, With<FruitConfig>)>>();
            let named: Vec<(Entity, String)> = query
                .iter(world)
                .map(|(organ, name)| (organ, name.to_string()))
                .collect();
            for (organ, name) in named {
                let at = in_scene(world, organ).translation;
                assert!(at.z >= soil_z - 1e-5, "week {week}: {name} at {at}");
            }
            let plants = organs::<PlantConfig>(app.world_mut());
            counts.push(leaves.len() / plants.len());
            first.push(
                leaves
                    .iter()
                    .filter(|leaf| {
                        leaf.path.ends_with("Plant_000") && leaf.name.starts_with("Leaf_")
                    })
                    .map(|leaf| {
                        (
                            (leaf.transform.rotation * Vec3::X).xy().to_angle(),
                            leaf.config.seed,
                        )
                    })
                    .collect(),
            );
        }
        assert!(
            counts.windows(2).all(|pair| pair[0] <= pair[1]),
            "{counts:?}"
        );
        assert!(counts[0] < counts[3], "{counts:?}");
        let (at_four, at_five) = (&first[1], &first[2]);
        assert!(at_four.len() <= at_five.len());
        for (rank, (had, has)) in at_four.iter().zip(at_five).enumerate() {
            assert!(
                (had.0 - has.0).abs() < 1e-4 && had.1 == has.1,
                "rank {rank}: {had:?} became {has:?}"
            );
        }
    }

    /// With the stems colliding a kale carries a capsule chain and a carrot's
    /// root none; with the fruit colliding too the root carries a capsule the
    /// root's whole length; with neither switch, nothing on a plant collides.
    #[test]
    fn colliders_follow_the_physics_switches() {
        let capsules = |stems: bool, fruit: bool| -> (usize, usize) {
            let mut params = small();
            (params.physics.stems_collide, params.physics.fruit_collides) = (stems, fruit);
            let mut app = grown(params);
            let world = app.world_mut();
            let mut query = world.query::<(&Name, &misina_lab::scene::Collider, &ChildOf)>();
            let (mut on_stems, mut on_fruit) = (0, 0);
            for (name, collider, parent) in query.iter(world) {
                let parent = world.entity(parent.parent());
                let holder = parent.get::<Name>().unwrap().as_str();
                if name.as_str() == COLLISION && holder.starts_with("Fruit_") {
                    let root = parent.get::<FruitConfig>().unwrap();
                    let span = collider.0.height + 2.0 * collider.0.radius;
                    assert!(
                        (span - root.length).abs() < 1e-5,
                        "{span} m over a {} m root",
                        root.length
                    );
                    on_fruit += 1;
                } else if name.starts_with(COLLISION) && holder.starts_with("Plant_") {
                    on_stems += 1;
                }
            }
            (on_stems, on_fruit)
        };
        assert_eq!(capsules(false, false), (0, 0));
        let (stems, fruit) = capsules(true, false);
        assert!(
            stems > 0 && fruit == 0,
            "{stems} stem capsules, {fruit} on fruit"
        );
        let (stems_too, fruit) = capsules(true, true);
        assert!(
            stems_too == stems && fruit > 0,
            "{stems_too} stem capsules, {fruit} on fruit"
        );
    }
}

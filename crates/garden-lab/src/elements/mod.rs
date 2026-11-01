//! Elements — the parts of a garden, one module each.
//!
//! [`layout`] solves where the beds go and spawns a prim for each; the
//! fixtures fill them, [`floor`] the ground the garden stands on and [`bed`]
//! the frame and the soil of every bed; [`planting`] puts a plant of some
//! species in every planting place and [`weed`] the plants nobody sowed in
//! the beds and on the paths; and the crop grows on them, a system
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
pub mod weed;

use bevy::prelude::*;
use misina_lab::Build;
use misina_lab::geometry::mesh::{MeshData, box_mesh};
use misina_lab::params::{Label, Slider};

use crate::species::{self, Commons, bush_bean, carrot, kale, lettuce, pole_bean, zucchini};

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
    /// A plant in every planting place, and the weeds.
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
        .add_plugins((
            layout::plugin,
            floor::plugin,
            bed::plugin,
            planting::plugin,
            weed::plugin,
        ));
    species::species::<lettuce::Lettuce>(app);
    species::species::<kale::Kale>(app);
    species::species::<carrot::Carrot>(app);
    species::species::<bush_bean::BushBean>(app);
    species::species::<pole_bean::PoleBean>(app);
    species::species::<zucchini::Zucchini>(app);
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

/// What a robot bumps into, besides the floor and the beds, and what bends.
/// Scene-wide, since a demo wants one switch.
#[derive(Resource, Reflect, Clone, Debug, PartialEq)]
#[cfg_attr(
    feature = "python",
    pyo3::pyclass(get_all, set_all, skip_from_py_object)
)]
pub struct PhysicsParams {
    /// Whether every stem collides, as a chain of capsules along it. A rod
    /// collides as itself, whatever this says.
    pub stems_collide: bool,
    /// Whether every fruit and root collides, as a capsule each.
    pub fruit_collides: bool,
    /// Whether every leaf collides, as a box round its blade, fitted to the
    /// blade's extent in the plane it lies flattest in; the petiole does not.
    /// A leaf riding a bending stem carries its box with it, and two plants
    /// authored with their leaves through each other push apart when the
    /// solver starts.
    pub leaves_collide: bool,
    /// Whether the stems that bend, a pole bean's vine, are rods the solver
    /// moves, their leaves and pods riding them; off, they are wood like
    /// every other stem.
    pub flexible_stems: bool,
    /// Length of a rod's segments, in meters: shorter bends smoother and
    /// costs a body per segment. No longer than 6 cm, the most a pole bean's
    /// vine can be sampled at and still clear its string.
    #[reflect(@Slider { min: 0.02, max: 0.06, step: 0.005 })]
    pub segment: f32,
}

impl Default for PhysicsParams {
    fn default() -> Self {
        Self {
            stems_collide: true,
            fruit_collides: false,
            leaves_collide: false,
            flexible_stems: false,
            segment: 0.05,
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
        pub weeds: weed::WeedParams,
        pub lettuce: lettuce::LettuceParams,
        pub kale: kale::KaleParams,
        pub carrot: carrot::CarrotParams,
        #[reflect(@Label("Bush bean"))]
        pub bush_bean: bush_bean::BushBeanParams,
        #[reflect(@Label("Pole bean"))]
        pub pole_bean: pole_bean::PoleBeanParams,
        pub zucchini: zucchini::ZucchiniParams,
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

/// [`small`] with no crop dealt a square: a test turns on the crop it looks
/// at.
#[cfg(test)]
pub(crate) fn bare() -> GardenParams {
    let mut params = small();
    for share in [
        &mut params.lettuce.share,
        &mut params.kale.share,
        &mut params.carrot.share,
        &mut params.bush_bean.share,
        &mut params.pole_bean.share,
        &mut params.zucchini.share,
    ] {
        *share = 0.0;
    }
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
    use std::collections::{BTreeMap, BTreeSet};

    use misina_lab::Params;
    use misina_lab::scene::doc::Node;
    use misina_lab::scene::export::{scene_doc, scene_json};
    use misina_lab::scene::{COLLISION, Cable, Collider};
    use misina_lab::testing::{check_params, named_children, nudged, organs, prim};

    use super::fruit::{FruitConfig, Kind};
    use super::leaf::{LeafConfig, Template};
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
        for flexible in [false, true] {
            let mut params = small();
            params.physics.flexible_stems = flexible;
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
            (
                "leaves collide",
                edited(|p| p.physics.leaves_collide = true),
            ),
            ("kale height", edited(|p| p.kale.height = 0.9)),
            ("kale stripped", edited(|p| p.kale.leaves_kept = 6)),
            ("no kale", edited(|p| p.kale.share = 0.0)),
            ("carrot leaves", edited(|p| p.carrot.leaves = 12)),
            ("carrot root", edited(|p| p.carrot.root_length = 0.1)),
            ("carrots later", edited(|p| p.carrot.sown_week = 6.0)),
            ("trellis", edited(|p| p.bed.trellis = 1.5)),
            ("bush bean branches", edited(|p| p.bush_bean.branches = 1)),
            ("pod length", edited(|p| p.bush_bean.pod_length = 0.2)),
            ("pole bean height", edited(|p| p.pole_bean.height = 1.5)),
            ("no pole beans", edited(|p| p.pole_bean.share = 0.0)),
            ("zucchini leaf", edited(|p| p.zucchini.leaf_length = 0.9)),
            ("zucchini closer", edited(|p| p.zucchini.spacing = 0.6)),
            ("no zucchini", edited(|p| p.zucchini.share = 0.0)),
            ("weeds", edited(|p| p.weeds.pressure = 12.0)),
            (
                "no weeds",
                edited(|p| (p.weeds.pressure, p.weeds.path_pressure) = (0.0, 0.0)),
            ),
            ("weed budget", edited(|p| p.weeds.variations = 3)),
            (
                "flexible stems",
                edited(|p| p.physics.flexible_stems = true),
            ),
            (
                "segment",
                edited(|p| (p.physics.flexible_stems, p.physics.segment) = (true, 0.08)),
            ),
        ] {
            let mut app = grown(small());
            want.apply(app.world_mut());
            app.update();
            assert!(json(&mut app) == json(&mut grown(want)), "{what} drifted");
        }
        // And from a garden whose stems already bend: the rods come down, or
        // are resampled, where generating would have put them.
        let flexible = || {
            let mut params = small();
            params.physics.flexible_stems = true;
            params
        };
        let bent = |edit: fn(&mut GardenParams)| {
            let mut params = flexible();
            edit(&mut params);
            params
        };
        for (what, want) in [
            (
                "stems rigid again",
                bent(|p| p.physics.flexible_stems = false),
            ),
            ("segment while flexible", bent(|p| p.physics.segment = 0.03)),
            (
                "vine height while flexible",
                bent(|p| p.pole_bean.height = 1.6),
            ),
        ] {
            let mut app = grown(flexible());
            want.apply(app.world_mut());
            app.update();
            assert!(json(&mut app) == json(&mut grown(want)), "{what} drifted");
        }
    }

    /// The default params on one bed of lettuce alone.
    fn lettuce_bed() -> GardenParams {
        let mut params = bare();
        params.lettuce.share = 1.0;
        params
    }

    /// A bed of zucchini alone: at 0.9 m spacing a plant every three squares
    /// along the middle row, each a stem under lobed leaves and, at harvest,
    /// berries lying out from the stem clear of the soil.
    #[test]
    fn a_zucchini_bed_holds_a_plant_every_three_squares() {
        let mut params = bare();
        params.zucchini.share = 1.0;
        (params.planting.jitter, params.planting.misses) = (0.0, 0.0);
        let mut app = grown(params);
        let plants = organs::<PlantConfig>(app.world_mut());
        let xs: Vec<f32> = plants.iter().map(|p| p.position().x).collect();
        assert_eq!(xs.len(), 3, "{xs:?}");
        assert!(
            xs.windows(2)
                .all(|w| (w[1] - w[0] - 3.0 * SQUARE).abs() < 1e-4),
            "{xs:?}"
        );
        assert!(plants.iter().all(|p| p.position().y.abs() < 1e-5));
        let leaves = organs::<LeafConfig>(app.world_mut());
        assert!(
            leaves.len() >= 3 * 8 && leaves.iter().all(|l| l.config.template == Template::Lobed),
            "{}",
            leaves.len()
        );
        let fruit = organs::<FruitConfig>(app.world_mut());
        assert!(fruit.len() >= 3, "{}", fruit.len());
        for f in &fruit {
            assert_eq!(f.config.kind, Kind::Berry);
            // In the plant's frame, the soil at z = 0: the fruit hangs down
            // its own -Z from its stalk end, and neither end lies in the soil.
            let radius = f.config.diameter / 2.0;
            let far = f.position() + f.transform.rotation * -Vec3::Z * f.config.length;
            assert!(
                f.position().z > 0.8 * radius && far.z > radius,
                "{} lies in the soil: {} to {far}",
                f.path,
                f.position()
            );
        }
    }

    /// Four lettuces to every whole 30 cm square of a bed's soil, each on the
    /// soil surface inside the frame and numbered by lattice place, when none
    /// is missing.
    #[test]
    fn a_bed_holds_four_lettuces_to_a_square() {
        let mut params = lettuce_bed();
        params.planting.misses = 0.0;
        let (bed, rim) = (params.bed.clone(), params.bed.rim);
        let mut app = grown(params);
        let bed_prim = prim(app.world_mut(), &["Beds", "Bed_00"]).unwrap();
        let plants: Vec<(String, Entity)> = named_children(app.world_mut(), bed_prim)
            .into_iter()
            .filter(|(name, _)| name.starts_with("Plant_"))
            .collect();
        let inner = Vec2::new(bed.length, bed.width) - Vec2::splat(2.0 * layout::BOARD);
        let squares = (inner / SQUARE).floor();
        // Two to a side, numbered by row and column of the four-by-four
        // lattice, so the places a denser crop would fill stay unused.
        let mut names: Vec<&str> = plants.iter().map(|(name, _)| name.as_str()).collect();
        names.sort_unstable();
        let expected: Vec<String> = (0..(squares.x * squares.y) as u64)
            .flat_map(|cell| [0, 1, 4, 5].map(|place| format!("Plant_{:03}", cell * 16 + place)))
            .collect();
        assert_eq!(names, expected);
        for (_, plant) in plants {
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
    /// the same stream under every name.
    #[test]
    fn a_plant_keeps_its_leaves_as_it_grows() {
        let mut params = small();
        params.planting.misses = 0.0;
        let soil_z = params.bed.height - params.bed.rim;
        let mut counts = Vec::new();
        let mut first: Vec<BTreeMap<String, (f32, u64)>> = Vec::new();
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
                        leaf.path.contains("/Plant_000/") && leaf.name.starts_with("Leaf_")
                    })
                    .map(|leaf| {
                        (
                            leaf.name.clone(),
                            (
                                (leaf.transform.rotation * Vec3::X).xy().to_angle(),
                                leaf.config.seed,
                            ),
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
        assert!(!at_four.is_empty() && at_four.len() <= at_five.len());
        for (name, had) in at_four {
            let has = at_five
                .get(name)
                .unwrap_or_else(|| panic!("{name} was dropped"));
            assert!(
                (had.0 - has.0).abs() < 1e-4 && had.1 == has.1,
                "{name}: {had:?} became {has:?}"
            );
        }
    }

    /// One square of pole beans with the stems flexible: a plant's vine is a
    /// `Rod_00` holding the cable and a prim per segment, the points past the
    /// held pair evenly spaced, every leaf and pod of the plant riding a
    /// segment with its mass in the cable's loads, and no capsule chain; with
    /// the stems rigid the same plant is wood with a chain and no rod.
    #[test]
    fn a_pole_bean_vine_bends_into_a_rod_carrying_its_organs() {
        let beans = |flexible: bool| {
            let mut params = bare();
            params.pole_bean.share = 1.0;
            (params.bed.length, params.bed.width) = (0.5, 0.5);
            (params.planting.misses, params.scene.week) = (0.0, 10.0);
            params.physics.flexible_stems = flexible;
            grown(params)
        };
        let mut app = beans(true);
        let plant = prim(app.world_mut(), &["Beds", "Bed_00", "Plant_000"]).unwrap();
        let children = named_children(app.world_mut(), plant);
        let names: Vec<&str> = children.iter().map(|(name, _)| name.as_str()).collect();
        assert!(
            names.contains(&"Rod_00") && names.contains(&"String"),
            "{names:?}"
        );
        // The string runs from the plant's foot to under the rail, straight
        // north of it.
        let string = children
            .iter()
            .find(|(name, _)| name == "String")
            .map(|(_, entity)| *entity)
            .unwrap();
        let turn = app
            .world()
            .entity(string)
            .get::<Transform>()
            .unwrap()
            .rotation;
        let span = named_children(app.world_mut(), string)
            .into_iter()
            .find(|(name, _)| name == COLLISION)
            .map(|(_, capsule)| {
                let Collider::Capsule(capsule) =
                    app.world().entity(capsule).get::<Collider>().unwrap()
                else {
                    panic!("the string's collider is no capsule")
                };
                capsule.height + 2.0 * capsule.radius
            })
            .unwrap();
        let foot = app
            .world()
            .entity(plant)
            .get::<Transform>()
            .unwrap()
            .translation;
        let top = foot + turn * Vec3::Z * span;
        let defaults = GardenParams::default().bed;
        let rail_z = defaults.height - defaults.rim + defaults.trellis;
        let under_rail = Vec3::new(
            foot.x,
            bed::trellis_y(Vec2::splat(0.5)),
            rail_z - bed::TRELLIS_TIMBER / 2.0,
        );
        assert!(
            top.distance(under_rail) < 1e-4,
            "{top} is not under the rail at {under_rail}"
        );
        assert!(
            !names.iter().any(|name| name.starts_with(COLLISION)
                || name.starts_with("Leaf_")
                || name.starts_with("Fruit_")),
            "{names:?}"
        );
        let rod = prim(app.world_mut(), &["Beds", "Bed_00", "Plant_000", "Rod_00"]).unwrap();
        let parts = named_children(app.world_mut(), rod);
        let cable = parts
            .iter()
            .find(|(name, _)| name == "Cable")
            .map(|(_, entity)| {
                app.world()
                    .entity(*entity)
                    .get::<Cable>()
                    .unwrap()
                    .0
                    .clone()
            })
            .unwrap();
        let segments: Vec<Entity> = parts
            .iter()
            .filter(|(name, _)| name.starts_with("Cable_edge_body_"))
            .map(|(_, entity)| *entity)
            .collect();
        assert_eq!(segments.len(), cable.points.len() - 1);
        let steps: Vec<f32> = cable.points[1..]
            .windows(2)
            .map(|pair| Vec3::from(pair[0]).distance(pair[1].into()))
            .collect();
        assert!(
            steps.len() > 10 && steps.iter().all(|step| (step - steps[0]).abs() < 1e-4),
            "{steps:?}"
        );
        let (mut riders, mut mass) = (0, 0.0);
        for segment in segments {
            for (name, entity) in named_children(app.world_mut(), segment) {
                let organ = app.world().entity(entity);
                if name.starts_with("Leaf_") {
                    mass += organ.get::<LeafConfig>().unwrap().mass();
                } else if name.starts_with("Fruit_") {
                    mass += organ.get::<FruitConfig>().unwrap().mass();
                } else {
                    continue;
                }
                riders += 1;
            }
        }
        let loads: f32 = cable.loads.iter().sum();
        assert!(
            riders > 10 && (loads - mass).abs() < 1e-4,
            "{riders} riders, {loads} kg of {mass}"
        );

        let mut app = beans(false);
        let plant = prim(app.world_mut(), &["Beds", "Bed_00", "Plant_000"]).unwrap();
        let names: Vec<String> = named_children(app.world_mut(), plant)
            .into_iter()
            .map(|(name, _)| name)
            .collect();
        assert!(
            names.iter().any(|name| name == "Wood")
                && names.iter().any(|name| name.starts_with(COLLISION))
                && names.iter().any(|name| name.starts_with("Leaf_"))
                && !names.iter().any(|name| name.starts_with("Rod_")),
            "{names:?}"
        );
    }

    /// With the stems colliding a kale carries a capsule chain and a carrot's
    /// root none; with the fruit colliding too the root carries a capsule the
    /// root's whole length; with neither switch, nothing on a plant collides.
    #[test]
    fn colliders_follow_the_physics_switches() {
        let proxies = |stems: bool, fruit: bool, leaves: bool| -> [usize; 3] {
            let mut params = small();
            params.physics.stems_collide = stems;
            (params.physics.fruit_collides, params.physics.leaves_collide) = (fruit, leaves);
            let mut app = grown(params);
            let world = app.world_mut();
            let mut query = world.query::<(&Name, &Collider, &ChildOf)>();
            let mut counts = [0; 3];
            for (name, collider, parent) in query.iter(world) {
                let parent = world.entity(parent.parent());
                let holder = parent.get::<Name>().unwrap().as_str();
                match (collider, holder) {
                    (Collider::Capsule(capsule), _) if holder.starts_with("Fruit_") => {
                        let fruit = parent.get::<FruitConfig>().unwrap();
                        let span = capsule.height + 2.0 * capsule.radius;
                        assert!(
                            (span - fruit.length).abs() < 1e-5,
                            "{span} m over a {} m fruit",
                            fruit.length
                        );
                        counts[1] += 1;
                    }
                    (Collider::Capsule(_), _) if holder.starts_with("Plant_") => {
                        assert!(name.starts_with(COLLISION));
                        counts[0] += 1;
                    }
                    // A box under the leaf's Xform, beside the Blade that
                    // draws it.
                    (Collider::Box, _) if holder.starts_with("Leaf_") => {
                        assert_eq!(name.as_str(), COLLISION);
                        assert!(parent.get::<LeafConfig>().is_some());
                        let blade = parent.get::<Children>().unwrap().iter().any(|child| {
                            world.entity(child).get::<Name>().unwrap().as_str() == "Blade"
                        });
                        assert!(blade, "a leaf with a box but no Blade");
                        counts[2] += 1;
                    }
                    // The strings' and the fixtures' own.
                    _ => {}
                }
            }
            counts
        };
        assert_eq!(proxies(false, false, false), [0, 0, 0]);
        let [stems, fruit, leaves] = proxies(true, false, false);
        assert!(
            stems > 0 && fruit == 0 && leaves == 0,
            "{stems} stem capsules, {fruit} on fruit, {leaves} leaf boxes"
        );
        let [stems_too, fruit, leaves] = proxies(true, true, true);
        assert!(
            stems_too == stems && fruit > 0 && leaves > 0,
            "{stems_too} stem capsules, {fruit} on fruit, {leaves} leaf boxes"
        );
    }
}

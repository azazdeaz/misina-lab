//! Weeds — the plants that come up where nothing was sown, in the beds'
//! soil and on the paths.
//!
//! The vineyard's weed element over the garden's zones: a short table of
//! growth habits, a tuft, a mat, a rosette, a bolter and a sprawling
//! broadleaf, each standing in for the species commonest in it, on a fixed
//! slot grid over every bed's soil and over the floor off the beds, at a
//! pressure for each. Every plant is one mesh, its leaves and stems built
//! and merged here, and two plants of one species at about the same stage
//! and size share one. Weeds germinate all season, so a plant is at any
//! stage whatever the scene's week, and the week moves none of them.
//!
//! Prims: `/Weeds/Bed_00/Weed_0017` stands in the soil of `Bed_00`,
//! `/Weeds/Paths/Weed_0042` on the floor. A plant is named by its slot on
//! the grid, so a pressure edit thins the weeds rather than renumbering
//! them. Every weed carries `species` and `crop`, false.

use std::f64::consts::TAU;

// Named over Bevy's own `Context`, which the prelude brings in.
use anyhow::Context as ContextExt;
use bevy::prelude::*;
use misina_lab::geometry::mesh::{MeshData, merge_meshes};
use misina_lab::geometry::par_map;
use misina_lab::geometry::scatter::{Band, jittered_grid};
use misina_lab::geometry::shapes::{self, bent, lying, standing};
use misina_lab::geometry::strand::{Bark, Strand, strand_mesh};
use misina_lab::palette::{COLOR_STREAM, Response, shade, srgb};
use misina_lab::params::{Label, Slider};
use misina_lab::quantize::{Metric, farthest_first};
use misina_lab::rng::{Rng, salt};
use misina_lab::scene::{
    Geometry, Library, Order, PrimRoot, Surface, UsdAttributes, UsdType, configs_changed, placed,
};
use nalgebra::Point3;

use super::layout::{BOARD, GardenLayout};
use super::{Grow, SceneParams};

/// The mesh-library prefix the plants are registered under, and the prim
/// this element owns under the scene root.
const PART: &str = "Weed";
const WEEDS: &str = "Weeds";

/// Marks the subtree, so a rebuild can drop the last one.
#[derive(Component)]
struct Weeds;

/// The slot grid a weed may stand on, in meters: about twenty-five slots to
/// the square metre, which caps a pressure near that; and how far a plant
/// sits off its slot's centre, in slots: half, anywhere in the cell and
/// never on top of a neighbour.
const SLOT: f32 = 0.2;
const SLOT_JITTER: f32 = 0.5;

/// How far along its season a plant may be, and the spread of plant sizes
/// as a multiplier on a species' typical size.
const STAGE: (f64, f64) = (0.15, 1.0);
const VIGOUR: (f64, f64) = (0.6, 1.4);

/// Sides around a stem, and rings along it per meter. Stems are millimetres
/// across; nothing finer would show.
const SIDES: usize = 5;
const RINGS_PER_METER: f64 = 12.0;

/// The streams the beds and the paths are planted from, the beds' salted
/// per bed; the stream a plant's mesh is drawn from, salted per
/// representative; and the salt splitting the weeds' shades off every other
/// stream.
const BED_STREAM: u64 = 0x4C8A_2F17_D93B_E605;
const PATH_STREAM: u64 = 0xB16E_9D40_73A2_5CF9;
const PLANT_STREAM: u64 = 0xE73D_0B58_A14C_962F;
const WEED_SHADE: u64 = 0x5A1F_C7E3_08D6_B294;

/// How far apart two plants of different species are. Categorical, and
/// larger than any stage or size can reach, so a budget of five keeps one
/// mesh per species before it buys a second of any.
const SPECIES_APART: f32 = 1.0;

/// Weed foliage, and how it answers light.
const WEED: (u32, Response) = (
    0x4E7A31,
    Response {
        roughness: 0.5,
        reflectance: 0.55,
    },
);

// ─── Species ────────────────────────────────────────────────────────

/// A growth habit: the shape a plant is built as. Each stands in for the
/// species commonest in it, and the mesh builders are one per habit.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Habit {
    /// A tussock of soft blades from one crown.
    Tuft,
    /// Stems running out along the ground, small leaves in pairs.
    Mat,
    /// A flat whorl of leaves from a taproot.
    Rosette,
    /// A rosette that, once bolted, sends up one leafy wand.
    Bolter,
    /// Stems that run out from a woody crown and turn up, with round leaves
    /// off them; hollow in the middle.
    Broadleaf,
}

impl Habit {
    /// The habits a `tall` share buys: the ones a tool meets above the crop.
    pub fn tall(self) -> bool {
        matches!(self, Habit::Bolter | Habit::Broadleaf)
    }
}

/// One species: its habit, and its share of the plants of its height.
#[derive(Clone, Copy, Debug)]
pub struct Species {
    pub name: &'static str,
    pub habit: Habit,
    /// Its weight against the other species of its height, tall or low.
    pub weight: f32,
}

/// The species the garden's weeds are built from, one per habit. Indexed by
/// [`WeedConfig::species`].
pub const SPECIES: &[Species] = &[
    Species {
        name: "Poa annua",
        habit: Habit::Tuft,
        weight: 0.4,
    },
    Species {
        name: "Stellaria media",
        habit: Habit::Mat,
        weight: 0.35,
    },
    Species {
        name: "Taraxacum officinale",
        habit: Habit::Rosette,
        weight: 0.25,
    },
    Species {
        name: "Erigeron canadensis",
        habit: Habit::Bolter,
        weight: 0.5,
    },
    Species {
        name: "Malva sylvestris",
        habit: Habit::Broadleaf,
        weight: 0.5,
    },
];

/// Each species' share of the plants, the tall habits together holding
/// `tall` of the total and the low ones the rest.
fn shares(tall: f32) -> Vec<f32> {
    let sum = |tall_ones: bool| -> f32 {
        SPECIES
            .iter()
            .filter(|species| species.habit.tall() == tall_ones)
            .map(|species| species.weight)
            .sum()
    };
    let (tall_sum, low_sum) = (sum(true), sum(false));
    SPECIES
        .iter()
        .map(|species| {
            if species.habit.tall() {
                species.weight / tall_sum * tall
            } else {
                species.weight / low_sum * (1.0 - tall)
            }
        })
        .collect()
}

/// The species a unit draw lands on, walking the shares.
fn pick(shares: &[f32], draw: f64) -> usize {
    let mut cumulative = 0.0;
    for (index, share) in shares.iter().enumerate() {
        cumulative += *share as f64;
        if draw < cumulative {
            return index;
        }
    }
    shares.len().saturating_sub(1)
}

// ─── Config ─────────────────────────────────────────────────────────

/// One plant's shape.
#[derive(Component, Clone, Copy, Debug, PartialEq)]
pub struct WeedConfig {
    /// Index into [`SPECIES`].
    pub species: u32,
    /// How far along its season the plant is, `0..=1`: size for every
    /// habit, and for a bolter whether it has bolted.
    pub stage: f32,
    /// Size, as a multiplier on the species' typical one.
    pub vigour: f32,
    /// Triangles a leaf's fill is cut into.
    pub detail: u32,
}

impl WeedConfig {
    /// Clamped here rather than in the builder, so the config a metric
    /// compares is the plant that actually gets built.
    pub fn new(species: usize, stage: f32, vigour: f32, detail: u32) -> Self {
        Self {
            species: (species % SPECIES.len()) as u32,
            stage: stage.clamp(0.05, 1.0),
            vigour: vigour.max(0.1),
            detail: detail.max(1),
        }
    }
}

/// Two plants share a mesh when they are one species at about the same
/// stage and size.
pub struct WeedMetric;

impl Metric<WeedConfig> for WeedMetric {
    fn distance(&self, a: &WeedConfig, b: &WeedConfig) -> f32 {
        let species = if a.species == b.species {
            0.0
        } else {
            SPECIES_APART
        };
        let stage = (a.stage - b.stage) * 0.5;
        let vigour = (a.vigour / b.vigour).ln().abs() * 0.3;
        let detail = (a.detail as f32 - b.detail as f32) * 0.001;
        (species * species + stage * stage + vigour * vigour + detail * detail).sqrt()
    }
}

// ─── Params ─────────────────────────────────────────────────────────

/// The plants that come up where nothing was sown: in the beds' soil and
/// on the paths, so many to the square metre of each. Every plant is one
/// mesh: `variations` is the budget of distinct ones and `detail` the
/// triangles a leaf is cut into.
#[derive(Resource, Reflect, Clone, Debug, PartialEq)]
#[cfg_attr(
    feature = "python",
    pyo3::pyclass(get_all, set_all, skip_from_py_object)
)]
pub struct WeedParams {
    /// Plants per square metre of bed soil. Zero is a weeded bed; the
    /// ceiling is one plant per slot of the 20 cm grid, about twenty-five.
    #[reflect(@Slider { min: 0.0, max: 25.0, step: 0.5 })]
    pub pressure: f32,
    /// Plants per square metre of path, the floor off the beds.
    #[reflect(@Slider { min: 0.0, max: 25.0, step: 0.5 }, @Label("Path weeds"))]
    pub path_pressure: f32,
    /// The share of plants that are tall, bolters and broadleaves, against
    /// low tufts, mats and rosettes.
    #[reflect(@Slider { min: 0.0, max: 1.0, step: 0.05 }, @Label("Tall share"))]
    pub tall: f32,
    /// How many distinct plant meshes the scene may hold. A budget, not a
    /// count: the plants are clustered by species, stage and size and this
    /// is how many representatives the clustering may keep.
    #[reflect(@Slider { min: 1.0, max: 64.0, step: 1.0 })]
    pub variations: u32,
    /// Triangles a leaf is cut into.
    #[reflect(@Slider { min: 4.0, max: 64.0, step: 2.0 })]
    pub detail: u32,
}

impl Default for WeedParams {
    fn default() -> Self {
        Self {
            pressure: 3.0,
            path_pressure: 1.0,
            tall: 0.3,
            variations: 24,
            detail: 16,
        }
    }
}

pub fn plugin(app: &mut App) {
    app.init_resource::<WeedParams>().add_systems(
        PreUpdate,
        (
            author.run_if(changed),
            // Gated on the inputs as well as on the configs, so a change that
            // leaves no plants at all still clears the library.
            build.run_if(configs_changed::<WeedConfig>.or_eager(changed)),
        )
            .chain()
            .in_set(Grow::Planting),
    );
}

/// Run condition: something this element authors from has changed: its
/// params, the layout, or the scene's seed. The scene's week is none of it,
/// so a week scrub leaves the weeds standing.
fn changed(
    params: Res<WeedParams>,
    layout: Res<GardenLayout>,
    scene: Res<SceneParams>,
    mut seed: Local<Option<u64>>,
) -> bool {
    let reseeded = *seed != Some(scene.seed);
    *seed = Some(scene.seed);
    params.is_changed() || layout.is_changed() || reseeded
}

// ─── Authoring ──────────────────────────────────────────────────────

/// Places a plant config on every kept slot of every bed's soil and of the
/// floor off the beds.
fn author(
    mut commands: Commands,
    scene: Res<SceneParams>,
    params: Res<WeedParams>,
    layout: Res<GardenLayout>,
    root: Res<PrimRoot>,
) {
    commands.despawn_all::<With<Weeds>>();
    let weeds = commands
        .spawn((
            Weeds,
            Name::new(WEEDS),
            UsdType("Scope"),
            Transform::IDENTITY,
            Visibility::default(),
            ChildOf(root.0),
        ))
        .id();

    let shares = shares(params.tall.clamp(0.0, 1.0));
    let mut planter = Planter {
        commands: &mut commands,
        weeds,
        shares: &shares,
        detail: params.detail,
        order: 0,
    };
    let inner = layout.bed - Vec2::splat(2.0 * BOARD);
    for (index, centre) in layout.beds.iter().enumerate() {
        let mut rng = Rng::new(scene.seed ^ BED_STREAM ^ salt(index as u64));
        let soil = Band {
            start: *centre - Vec2::X * inner.x / 2.0,
            end: *centre + Vec2::X * inner.x / 2.0,
            half_width: inner.y / 2.0,
        };
        planter.plant(
            format!("Bed_{index:02}"),
            &soil,
            layout.soil_z,
            params.pressure,
            |_| true,
            &mut rng,
        );
    }
    // The paths: the floor less every bed's footprint.
    let floor = Band {
        start: -Vec2::X * layout.floor.x,
        end: Vec2::X * layout.floor.x,
        half_width: layout.floor.y,
    };
    let off_the_beds = |at: Vec2| {
        layout
            .beds
            .iter()
            .all(|centre| (at - *centre).abs().cmpgt(layout.bed / 2.0).any())
    };
    let mut rng = Rng::new(scene.seed ^ PATH_STREAM);
    planter.plant(
        "Paths".to_string(),
        &floor,
        0.0,
        params.path_pressure,
        off_the_beds,
        &mut rng,
    );

    if planter.order == 0 {
        commands.entity(weeds).despawn();
    }
}

/// What every zone is planted with, under the `Weeds` prim.
struct Planter<'a, 'w, 's> {
    commands: &'a mut Commands<'w, 's>,
    weeds: Entity,
    shares: &'a [f32],
    detail: u32,
    order: u64,
}

impl Planter<'_, '_, '_> {
    /// Plants `band` at `pressure` plants per square metre, standing at
    /// height `z`, under a `Scope` named `name`, spawned only if a plant
    /// survives; a slot `allowed` refuses is skipped.
    ///
    /// Five draws per slot, after the grid's own two: whether it is kept,
    /// which species, its vigour, its stage, its yaw. All five are taken for
    /// every slot, so thinning the pressure leaves the survivors as they were.
    fn plant(
        &mut self,
        name: String,
        band: &Band,
        z: f32,
        pressure: f32,
        allowed: impl Fn(Vec2) -> bool,
        rng: &mut Rng,
    ) {
        let slots = jittered_grid(band, SLOT, SLOT_JITTER, rng);
        // The grid stretches its cells to fill the band, so the keep rate
        // is the pressure over the cells' real density.
        let area = band.length() * 2.0 * band.half_width;
        let keep = (pressure * area / slots.len() as f32).clamp(0.0, 1.0) as f64;
        let mut plants = Vec::new();
        for slot in slots {
            let kept = rng.unit() < keep;
            let species = pick(self.shares, rng.unit());
            let vigour = rng.range(VIGOUR.0, VIGOUR.1);
            let stage = rng.range(STAGE.0, STAGE.1);
            let yaw = rng.unit() * TAU;
            if !kept || !allowed(slot.position) {
                continue;
            }
            plants.push((
                slot.index,
                slot.position.extend(z),
                yaw as f32,
                WeedConfig::new(species, stage as f32, vigour as f32, self.detail),
            ));
        }
        if plants.is_empty() {
            return;
        }

        let group = self
            .commands
            .spawn((
                Name::new(name),
                UsdType("Scope"),
                Transform::IDENTITY,
                Visibility::default(),
                ChildOf(self.weeds),
            ))
            .id();
        for (slot, position, yaw, config) in plants {
            self.order += 1;
            self.commands.spawn((
                Name::new(format!("Weed_{slot:04}")),
                placed(position, yaw, Vec2::ZERO, 1.0),
                Visibility::default(),
                config,
                Order(self.order),
                UsdAttributes::default()
                    .with("species", SPECIES[config.species as usize].name)
                    .with("crop", false),
                ChildOf(group),
            ));
        }
    }
}

// ─── Building ───────────────────────────────────────────────────────

/// Builds one mesh per distinct plant and gives it to every plant that drew
/// it. A plant has nothing hanging off it, so it carries its geometry
/// directly.
fn build(
    mut commands: Commands,
    mut library: Library,
    params: Res<WeedParams>,
    plants: Query<(Entity, &Order, &WeedConfig)>,
) -> Result {
    library.clear(PART);

    let mut grown: Vec<(Order, Entity, WeedConfig)> = plants
        .iter()
        .map(|(entity, order, config)| (*order, entity, *config))
        .collect();
    grown.sort_by_key(|(order, ..)| *order);

    let configs: Vec<WeedConfig> = grown.iter().map(|(_, _, config)| *config).collect();
    let book = farthest_first(
        &configs,
        params.variations.max(1) as usize,
        0.0,
        &WeedMetric,
    );

    let meshes = par_map(&book.representatives, |index, config| {
        plant_mesh(config, index as u64)
            .map(|mesh| mesh.to_mesh())
            .with_context(|| {
                format!(
                    "weed {} could not be built",
                    SPECIES[config.species as usize].name
                )
            })
    });

    let mut geometry: Vec<Geometry> = Vec::with_capacity(book.len());
    for (index, mesh) in meshes.into_iter().enumerate() {
        geometry.push(library.part(PART, index, mesh?, surface(index as u64)));
    }
    for ((_, entity, _), drew) in grown.iter().zip(&book.assignment) {
        commands
            .entity(*entity)
            .insert(geometry[*drew as usize].clone());
    }
    Ok(())
}

/// One plant in its own frame: the root collar on the origin, growing up
/// `+Z`.
fn plant_mesh(config: &WeedConfig, index: u64) -> anyhow::Result<MeshData> {
    let species = &SPECIES[config.species as usize % SPECIES.len()];
    let mut rng = Rng::new(PLANT_STREAM ^ salt(index));
    let (size, stage, triangles) = (
        config.vigour as f64,
        config.stage as f64,
        config.detail as f64,
    );
    match species.habit {
        Habit::Tuft => tuft(size, stage, triangles, &mut rng),
        Habit::Mat => mat(size, stage, triangles, &mut rng),
        Habit::Rosette => rosette(size, stage, triangles, &mut rng),
        Habit::Bolter => bolter(size, stage, triangles, &mut rng),
        Habit::Broadleaf => broadleaf(size, stage, triangles, &mut rng),
    }
}

/// A stem: a strand skinned at [`SIDES`] and [`RINGS_PER_METER`] with no
/// bark.
fn stem(points: Vec<Point3<f64>>, radii: Vec<f64>) -> anyhow::Result<MeshData> {
    strand_mesh(&Strand::new(
        points,
        radii,
        SIDES,
        RINGS_PER_METER,
        Bark::none(),
    ))
}

/// The point `t` of the way along a polyline, by segment.
fn along(points: &[Point3<f64>], t: f64) -> Vec3 {
    let segments = points.len().saturating_sub(1).max(1);
    let s = (t * segments as f64).clamp(0.0, segments as f64);
    let i = (s.floor() as usize).min(segments - 1);
    let f = s - i as f64;
    let (a, b) = (points[i], points[(i + 1).min(points.len() - 1)]);
    Vec3::new(
        (a.x + (b.x - a.x) * f) as f32,
        (a.y + (b.y - a.y) * f) as f32,
        (a.z + (b.z - a.z) * f) as f32,
    )
}

/// A leaf standing at `translation`, turned to `yaw` and lifted `pitch`.
fn leaf_at(translation: Vec3, yaw: f64, pitch: f64, scale: f64) -> Transform {
    Transform {
        translation,
        rotation: lying(yaw, pitch),
        scale: Vec3::splat(scale as f32),
    }
}

/// *Poa annua*: a tussock of soft blades from one crown, a hand high.
///
/// Draws: per variant its length and droop; per blade which variant, its
/// yaw and its lean.
fn tuft(size: f64, stage: f64, triangles: f64, rng: &mut Rng) -> anyhow::Result<MeshData> {
    let reach = 0.10 * size * (0.4 + 0.6 * stage);
    let variants: Vec<MeshData> = (0..3)
        .map(|_| {
            let length = reach * rng.range(0.5, 1.0);
            let droop = rng.range(0.4, 1.3);
            shapes::blade_mesh(length, 0.003, droop, 300.0, triangles)
        })
        .collect::<anyhow::Result<_>>()?;
    let parts: Vec<MeshData> = (0..30)
        .map(|_| {
            let variant = (rng.unit() * variants.len() as f64) as usize;
            let yaw = rng.unit() * TAU;
            let lean = rng.range(0.1, 1.0);
            variants[variant].transformed(&Transform {
                rotation: standing(yaw, lean),
                ..default()
            })
        })
        .collect();
    Ok(merge_meshes(&parts))
}

/// *Stellaria media*: stems running out along the ground from one root,
/// small ovate leaves in opposite pairs along them.
///
/// Draws: per stem its heading, its sway and the lift of its tip; per leaf
/// its turn off the stem and its pitch.
fn mat(size: f64, stage: f64, triangles: f64, rng: &mut Rng) -> anyhow::Result<MeshData> {
    let leaf = bent(
        shapes::leaf(0.015 * size, 0.008 * size, 0.4),
        0.3,
        0.0,
        triangles,
    )?;
    let run = 0.18 * size * (0.3 + 0.7 * stage);
    let stems = 4;
    let mut parts = Vec::new();
    for s in 0..stems {
        let heading = (s as f64 + rng.range(-0.3, 0.3)) / stems as f64 * TAU;
        let sway = rng.range(-0.15, 0.15);
        let lift = rng.range(0.01, 0.03);
        let points = vec![
            Point3::new(0.0, 0.0, 0.005),
            Point3::new(run * 0.5, run * sway, 0.01),
            Point3::new(run, 0.0, lift),
        ];
        let frame = Transform::from_rotation(Quat::from_rotation_z(heading as f32));
        parts.push(stem(points.clone(), vec![0.001, 0.0009, 0.0006])?.transformed(&frame));

        let nodes = ((run / 0.025) as usize).max(1);
        for n in 1..=nodes {
            let at = frame.transform_point(along(&points, n as f64 / nodes as f64));
            for side in [-1.0f64, 1.0] {
                let turn = heading + side * rng.range(0.9, 1.5);
                let pitch = rng.range(0.1, 0.4);
                parts.push(leaf.transformed(&leaf_at(at, turn, pitch, 1.0)));
            }
        }
    }
    Ok(merge_meshes(&parts))
}

/// *Taraxacum officinale*: a flat whorl of toothed leaves from a taproot's
/// crown, nothing standing up.
///
/// Draws: per leaf its length, its heading, its pitch and its droop.
fn rosette(size: f64, stage: f64, triangles: f64, rng: &mut Rng) -> anyhow::Result<MeshData> {
    let leaves = 10;
    let reach = 0.15 * size * (0.4 + 0.6 * stage);
    let mut parts = Vec::with_capacity(leaves);
    for i in 0..leaves {
        let length = reach * rng.range(0.6, 1.0);
        let heading = (i as f64 + rng.range(-0.3, 0.3)) / leaves as f64 * TAU;
        let pitch = rng.range(0.15, 0.6);
        let droop = rng.range(0.3, 0.8);
        let leaf = bent(
            shapes::toothed(length, length * 0.28, 0.6, 5),
            droop,
            0.0,
            triangles,
        )?;
        parts.push(leaf.transformed(&leaf_at(Vec3::Z * 0.005, heading, pitch, 1.0)));
    }
    Ok(merge_meshes(&parts))
}

/// *Erigeron canadensis*: a basal rosette that, once bolted, sends up one
/// wand clothed spirally in narrow leaves, to over a metre. The rosette
/// withers as the wand grows.
///
/// Draws: per basal leaf its length, its heading and its pitch; then the
/// wand's sway; per wand leaf its turn and its pitch.
fn bolter(size: f64, stage: f64, triangles: f64, rng: &mut Rng) -> anyhow::Result<MeshData> {
    let mut parts = Vec::new();
    let basal = 8;
    for i in 0..basal {
        let length = 0.06 * size * rng.range(0.6, 1.0) * (1.0 - 0.5 * stage);
        let heading = (i as f64 + rng.range(-0.3, 0.3)) / basal as f64 * TAU;
        let pitch = rng.range(0.2, 0.6);
        let leaf = bent(
            shapes::leaf(length, length * 0.25, 0.45),
            0.4,
            0.0,
            triangles,
        )?;
        parts.push(leaf.transformed(&leaf_at(Vec3::Z * 0.005, heading, pitch, 1.0)));
    }

    let bolted = ((stage - 0.3) / 0.7).max(0.0);
    if bolted > 0.0 {
        let height = 1.2 * size * bolted;
        let sway = rng.range(0.0, TAU);
        let bow = 0.02 * height;
        let points = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(bow * sway.cos(), bow * sway.sin(), height / 2.0),
            Point3::new(2.0 * bow * sway.cos(), 2.0 * bow * sway.sin(), height),
        ];
        parts.push(stem(
            points.clone(),
            vec![0.004 * size, 0.003 * size, 0.0015 * size],
        )?);

        let leaf = bent(
            shapes::leaf(0.04 * size, 0.008 * size, 0.4),
            0.3,
            0.0,
            triangles,
        )?;
        let nodes = ((height / 0.03) as usize).clamp(1, 40);
        // The golden angle: no two leaves up the wand line up.
        const GOLDEN: f64 = 2.399_963;
        for n in 0..nodes {
            let t = (n as f64 + 0.5) / nodes as f64;
            let turn = n as f64 * GOLDEN + rng.range(-0.2, 0.2);
            let pitch = rng.range(0.8, 1.2);
            let shrink = 1.0 - 0.6 * t;
            parts.push(leaf.transformed(&leaf_at(along(&points, t), turn, pitch, shrink)));
        }
    }
    Ok(merge_meshes(&parts))
}

/// *Malva sylvestris*: stems that run out from a woody crown and then turn
/// up, leaving the middle hollow, with round leaves off them. The shape a
/// weeder cannot reach the middle of.
///
/// Draws: per stem its heading and its sway; per leaf its turn and pitch.
fn broadleaf(size: f64, stage: f64, triangles: f64, rng: &mut Rng) -> anyhow::Result<MeshData> {
    let grown = 0.3 + 0.7 * stage;
    let height = 0.5 * size * grown;
    let reach = 0.3 * size * grown;
    let leaf = bent(
        shapes::leaf(0.06 * size, 0.06 * size, 0.5),
        0.3,
        0.0,
        triangles,
    )?;
    let stems = 4;
    let mut parts = Vec::new();
    for s in 0..stems {
        let heading = (s as f64 + rng.range(-0.3, 0.3)) / stems as f64 * TAU;
        let sway = rng.range(-0.2, 0.2);
        let points = vec![
            Point3::new(0.0, 0.0, 0.02),
            Point3::new(reach * 0.5, reach * sway, 0.05),
            Point3::new(reach, reach * sway * 0.5, height * 0.4),
            Point3::new(reach * 1.1, 0.0, height),
        ];
        let frame = Transform::from_rotation(Quat::from_rotation_z(heading as f32));
        let radii = [0.004, 0.0035, 0.003, 0.002].map(|r| r * size).to_vec();
        parts.push(stem(points.clone(), radii)?.transformed(&frame));

        let nodes = 4;
        for n in 1..=nodes {
            let at = frame.transform_point(along(&points, n as f64 / nodes as f64));
            let turn = heading + rng.range(-1.2, 1.2);
            let pitch = rng.range(0.1, 0.5);
            parts.push(leaf.transformed(&leaf_at(at, turn, pitch, 1.0)));
        }
    }
    Ok(merge_meshes(&parts))
}

/// A plant's foliage, shaded per representative.
fn surface(index: u64) -> Surface {
    let (color, response) = WEED;
    response.blade(shade(
        srgb(color),
        &mut Rng::new(COLOR_STREAM ^ WEED_SHADE ^ salt(index)),
    ))
}

#[cfg(test)]
mod tests {
    use misina_lab::scene::Prototypes;
    use misina_lab::testing::{bounds, organs, prim};

    use super::*;
    use crate::elements::{GardenParams, bare, grown};

    /// One bare bed, weedy.
    fn weedy() -> GardenParams {
        let mut params = bare();
        (params.weeds.pressure, params.weeds.path_pressure) = (8.0, 2.0);
        params
    }

    fn part_count(app: &App) -> usize {
        app.world()
            .resource::<Prototypes>()
            .iter()
            .filter(|(name, _)| name.starts_with("Weed_"))
            .count()
    }

    /// Every weed stands upright, on the soil inside its bed's frame and
    /// filed under the bed, or on the floor off every bed's footprint and
    /// filed under the paths; and each pressure is plants per square metre of
    /// its zone.
    #[test]
    fn weeds_stand_on_the_soil_in_the_beds_and_on_the_floor_between() {
        let mut app = grown(weedy());
        let layout = app.world().resource::<GardenLayout>().clone();
        let inner = layout.bed - Vec2::splat(2.0 * BOARD);
        let (mut in_beds, mut on_paths) = (0, 0);
        for weed in organs::<WeedConfig>(app.world_mut()) {
            let at = weed.position();
            assert!(weed.up().z > 0.999, "{} leans", weed.path);
            let bed = layout
                .beds
                .iter()
                .position(|centre| (at.truncate() - *centre).abs().cmplt(inner / 2.0).all());
            match bed {
                Some(index) => {
                    assert!((at.z - layout.soil_z).abs() < 1e-5, "{} floats", weed.path);
                    assert!(weed.path.starts_with(&format!("Weeds/Bed_{index:02}/")));
                    in_beds += 1;
                }
                None => {
                    assert!(at.z.abs() < 1e-6, "{} floats", weed.path);
                    assert!(weed.path.starts_with("Weeds/Paths/"), "{}", weed.path);
                    let off = layout.beds.iter().all(|centre| {
                        (at.truncate() - *centre)
                            .abs()
                            .cmpgt(layout.bed / 2.0)
                            .any()
                    });
                    assert!(off, "{} stands on a frame", weed.path);
                    on_paths += 1;
                }
            }
        }
        let bed_area = inner.x * inner.y;
        let path_area = 4.0 * layout.floor.x * layout.floor.y - layout.bed.x * layout.bed.y;
        let about = |got: usize, want: f32| (got as f32 - want).abs() < 0.25 * want + 2.0;
        assert!(about(in_beds, 8.0 * bed_area), "{in_beds} in {bed_area} m²");
        assert!(
            about(on_paths, 2.0 * path_area),
            "{on_paths} on {path_area} m²"
        );
    }

    #[test]
    fn no_pressure_grows_nothing_and_the_budget_caps_the_meshes() {
        let mut params = weedy();
        (params.weeds.pressure, params.weeds.path_pressure) = (0.0, 0.0);
        let mut app = grown(params);
        assert!(prim(app.world_mut(), &[WEEDS]).is_none());
        assert_eq!(part_count(&app), 0);

        let mut params = weedy();
        params.weeds.variations = 4;
        assert!(part_count(&grown(params)) <= 4);
    }

    /// The shares sum to one with the tall habits holding their share, and
    /// a tall share of zero grows no tall weed.
    #[test]
    fn the_tall_share_picks_the_tall_habits() {
        let s = shares(0.3);
        assert!((s.iter().sum::<f32>() - 1.0).abs() < 1e-5);
        let tall: f32 = SPECIES
            .iter()
            .zip(&s)
            .filter(|(species, _)| species.habit.tall())
            .map(|(_, share)| share)
            .sum();
        assert!((tall - 0.3).abs() < 1e-5);

        let mut params = weedy();
        params.weeds.tall = 0.0;
        let weeds = organs::<WeedConfig>(grown(params).world_mut());
        assert!(!weeds.is_empty());
        for weed in weeds {
            let species = &SPECIES[weed.config.species as usize];
            assert!(
                !species.habit.tall(),
                "{} grew with no tall share",
                species.name
            );
        }
    }

    /// Every habit builds, stands on its own ground and stays about the size
    /// its species is; a bolted bolter stands tall where its rosette lay low.
    #[test]
    fn every_species_builds_a_grounded_plant() {
        for (species, expected) in SPECIES.iter().enumerate() {
            let mesh = plant_mesh(&WeedConfig::new(species, 0.7, 1.0, 16), 3)
                .unwrap_or_else(|e| panic!("{}: {e:#}", expected.name));
            assert!(!mesh.face_vertex_counts.is_empty());
            let (lo, hi) = bounds(&mesh, 2);
            assert!(lo > -0.02, "{} digs to {lo}", expected.name);
            assert!(hi > 0.02 && hi < 1.5, "{} reaches {hi}", expected.name);
        }
        let bolter = SPECIES
            .iter()
            .position(|s| s.habit == Habit::Bolter)
            .unwrap();
        let low = plant_mesh(&WeedConfig::new(bolter, 0.1, 1.0, 16), 0).unwrap();
        let high = plant_mesh(&WeedConfig::new(bolter, 1.0, 1.0, 16), 0).unwrap();
        assert!(bounds(&low, 2).1 < 0.1 && bounds(&high, 2).1 > 0.8);
    }
}

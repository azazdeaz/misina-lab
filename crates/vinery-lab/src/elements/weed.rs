//! Weed element — the plants that come up where nothing was sown.
//!
//! Under the vines is a strip nothing drives over and, over most of Europe,
//! nothing grows on purpose: it is sprayed, hoed, mown or left alone, and
//! what comes up is decided by which. Out in the alley the sward keeps most
//! weeds down, and the few that escape are the tall ones a machine meets.
//! This element places individual plants in both zones from a short table
//! of growth habits — a tuft, a mat, a rosette, a bolter, a sprawling
//! broadleaf — each standing in for the species commonest in that habit, and
//! lets the strip's regime and the season pick between them.
//!
//! # One mesh per plant
//!
//! A plant is one geometry prim: its leaves and stems are built and merged
//! here, and the plant carries the part directly, as a leaf does. Two plants
//! of one species at about the same stage and size share a mesh. A plant
//! whose stems bend is the exception, below.
//!
//! # Bending stems
//!
//! With [`WeedParams::flexible`] on, every stem at least [`LONG_STEM`] long —
//! a bolter's wand, a broadleaf's runners — is a rod a solver bends, and its
//! plant holds its parts instead of being one:
//!
//! ```text
//! Weed_0017               the placed plant, carrying its WeedConfig
//!   Crown                 -> parts/Weed_<rep>: everything that stays rigid
//!   Stalk_0               one bending stem
//!     Skin                with `skinned`, what draws it, in place of each Stem
//!     Cable               its centerline, simulated, never drawn
//!     Cable_edge_body_0   one rod segment, its transform driven by the solver
//!       Stem              the tube this segment is drawn with
//!       Leaves            the leaves along this segment, in its frame
//!     Cable_edge_body_1   ...
//! ```
//!
//! A rod's prims are named after its curve, so each bending stem gets a
//! `Stalk_n` of its own. A plant with no stem that long stays one geometry
//! prim.
//!
//! # The layer
//!
//! Layout-driven, like [`cover`](super::cover): [`author`] places a
//! [`WeedConfig`] on every kept slot of every strip and alley band, and
//! [`build`] turns the distinct configs into plants. No `reauthor`: a params
//! edit re-authors the layer.
//!
//! Prims: `/Vineyard/Weeds/Row_003/Weed_0017` stands in the under-vine strip
//! of `Row_003`, `/Vineyard/Weeds/Alley_003/Weed_0004` in the alley between
//! rows 3 and 4. A plant is named by its slot on a fixed grid, so a pressure
//! edit thins the weeds rather than renumbering them.

use std::f64::consts::TAU;

// Named over the prelude's `ContextExt`, Bevy's own `context`, which would
// make every call ambiguous.
use anyhow::Context as ContextExt;
use bevy::prelude::*;
use nalgebra::Point3;

use super::terrain::Ground;
use super::util::parcel::{Band, VineyardLayout};
use super::util::{color, material, par_map};
use super::{Grow, Rng, SceneParams, salt};
use misina_lab::geometry::mesh::{MeshData, merge_meshes};
use misina_lab::geometry::scatter::jittered_grid;
use misina_lab::geometry::shapes::{self, bent, lying, standing};
use misina_lab::geometry::strand::{Bark, Strand, strand_mesh, strand_mesh_ringed};
use misina_lab::params::{Choices, Label, Slider};
use misina_lab::quantize::{Metric, farthest_first};
use misina_lab::scene::{
    Geometry, Library, Order, PrimRoot, Surface, UsdType, cable, configs_changed, placed, rebase,
    segment_frames, segment_tubes, skin_mesh, spawn_rod,
};

/// The mesh-library prefix the plants are registered under: a whole plant, or
/// the crown of one with bending stems.
pub const PART: &str = "Weed";

/// The mesh-library prefixes of a bending stem's per-segment tubes, or its
/// skin, and of the leaves riding each segment. Numbered across the layer,
/// since how many a representative needs depends on how long its stems came
/// out.
pub const STALK_TUBE: &str = "WeedStalk";
pub const STALK_LEAVES: &str = "WeedLeaves";

/// The prim a plant with bending stems keeps its rigid parts under.
pub const CROWN: &str = "Crown";

/// The prim each bending stem's rod is spawned under, numbered.
pub const STALK: &str = "Stalk";

/// The prim the leaves riding a rod segment take, beside its tube.
pub const LEAVES: &str = "Leaves";

/// The prim this element owns under the scene root.
pub const WEEDS: &str = "Weeds";

/// Marks the subtree, so a rebuild can drop the last one.
#[derive(Component)]
pub struct Weeds;

/// The slot grid a weed may stand on, in meters: twenty-five slots to the
/// square metre, which caps `pressure` at that.
const SLOT: f32 = 0.2;

/// How far a plant sits off its slot's centre, in slots. Half: anywhere in
/// the cell, and never on top of a neighbour.
const SLOT_JITTER: f32 = 0.5;

/// How far a plant's stage strays from the season, either way.
const STAGE_JITTER: f64 = 0.25;

/// The spread of plant sizes, as a multiplier on a species' typical size.
const VIGOUR: std::ops::Range<f64> = 0.6..1.4;

/// Sides around a stem, and rings along it per meter. Stems are millimetres
/// across; nothing finer would show.
const SIDES: usize = 5;
const RINGS_PER_METER: f64 = 12.0;

/// Length of one segment of a bending stem, in meters. A segment is a capsule
/// body and a joint, so this is the whole cost of a bending stem: a metre-tall
/// wand is ten of each.
const ROD_SEGMENT: f64 = 0.1;

/// The shortest stem that bends, in meters: three segments, the held one and
/// two to bend with. Only a bolter's wand and a broadleaf's runners grow this
/// long.
const LONG_STEM: f64 = 3.0 * ROD_SEGMENT;

/// The stream the strips are planted from, salted per row.
const STRIP_STREAM: u64 = 0x4C8A_2F17_D93B_E605;

/// The stream the alleys are planted from, salted per alley.
const ALLEY_STREAM: u64 = 0xB16E_9D40_73A2_5CF9;

/// The stream a plant's mesh is drawn from, salted per representative.
const PLANT_STREAM: u64 = 0xE73D_0B58_A14C_962F;

/// Salt splitting the weeds' shades off every other stream.
const WEED_SHADE: u64 = 0x5A1F_C7E3_08D6_B294;

/// How far apart two plants of different species are. Categorical, and
/// larger than any stage or size can reach, so a budget of five keeps one
/// mesh per species before it buys a second of any.
const SPECIES_APART: f32 = 1.0;

// ─── Strip ──────────────────────────────────────────────────────────

/// What is done to the under-vine strip — which is what decides what grows
/// there.
///
/// Named by string in [`WeedParams::strip`]; this is the parsed form.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Strip {
    /// Sprayed. What survives escapes by its cycle or tolerates the spray:
    /// small annual grasses, and the tall bolters glyphosate no longer kills.
    Herbicide,
    /// Hoed. Ruderal annuals that come back from seed between passes.
    Tilled,
    /// Mown. Perennials that duck the blade: rosettes and tufts.
    #[default]
    Mown,
    /// Left alone: everything, and the tall ones tallest.
    Untouched,
}

impl Strip {
    pub const ALL: [Strip; 4] = [
        Strip::Herbicide,
        Strip::Tilled,
        Strip::Mown,
        Strip::Untouched,
    ];

    /// The names, in [`ALL`](Self::ALL) order — what a config says.
    pub const NAMES: [&'static str; 4] = ["herbicide", "tilled", "mown", "untouched"];

    pub fn name(self) -> &'static str {
        Self::NAMES[self as usize]
    }

    pub fn parse(name: &str) -> Option<Strip> {
        Self::ALL.into_iter().find(|strip| strip.name() == name)
    }
}

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
    /// A rosette that, once bolted, sends up one leafy wand — the weed that
    /// reaches the fruiting wire.
    Bolter,
    /// Stems that run out from a woody crown and turn up, with round leaves
    /// off them; hollow in the middle.
    Broadleaf,
}

/// One species: its habit, and how much of the strip it holds under each
/// regime.
#[derive(Clone, Copy, Debug)]
pub struct Species {
    pub name: &'static str,
    pub habit: Habit,
    /// Relative share under each [`Strip`], in [`Strip::ALL`] order.
    pub weights: [f32; 4],
    /// `-1.0` for a plant of the spring flush, `1.0` for one of the summer's;
    /// how its share tilts with [`SceneParams::season`].
    pub summer: f32,
}

/// The species the floor is built from, one per habit. Indexed by
/// [`WeedConfig::species`].
pub const SPECIES: &[Species] = &[
    Species {
        name: "Poa annua",
        habit: Habit::Tuft,
        weights: [0.5, 0.3, 0.3, 0.15],
        summer: 0.0,
    },
    Species {
        name: "Stellaria media",
        habit: Habit::Mat,
        weights: [0.1, 0.4, 0.1, 0.2],
        summer: -1.0,
    },
    Species {
        name: "Taraxacum officinale",
        habit: Habit::Rosette,
        weights: [0.05, 0.1, 0.5, 0.25],
        summer: 0.0,
    },
    Species {
        name: "Erigeron canadensis",
        habit: Habit::Bolter,
        weights: [0.3, 0.1, 0.05, 0.25],
        summer: 1.0,
    },
    Species {
        name: "Malva sylvestris",
        habit: Habit::Broadleaf,
        weights: [0.05, 0.1, 0.05, 0.15],
        summer: 0.5,
    },
];

/// Each species' share of the plants under `strip` at `season`, with the
/// tall habits — bolters and broadleaves — together holding `tall` of the
/// total and the low ones the rest.
fn shares(strip: Strip, season: f32, tall: f32) -> Vec<f32> {
    let is_tall = |species: &Species| matches!(species.habit, Habit::Bolter | Habit::Broadleaf);
    let raw: Vec<f32> = SPECIES
        .iter()
        .map(|species| {
            let tilt = 1.0 + species.summer * (2.0 * season - 1.0) * 0.5;
            species.weights[strip as usize] * tilt.max(0.05)
        })
        .collect();
    let sum = |tall_ones: bool| -> f32 {
        SPECIES
            .iter()
            .zip(&raw)
            .filter(|(species, _)| is_tall(species) == tall_ones)
            .map(|(_, weight)| weight)
            .sum()
    };
    let (tall_sum, low_sum) = (sum(true), sum(false));
    SPECIES
        .iter()
        .zip(&raw)
        .map(|(species, weight)| {
            let (group, share) = if is_tall(species) {
                (tall_sum, tall)
            } else {
                (low_sum, 1.0 - tall)
            };
            if group > 0.0 {
                weight / group * share
            } else {
                0.0
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

/// The plants that come up where nothing was sown: in the under-vine strip,
/// and as escapes in the alley.
///
/// `strip` is what is done to the under-vine strip, and it picks the species;
/// the scene's `season` tilts the mix between the spring and summer flushes
/// and decides whether a bolter has bolted. Every plant is one mesh, bar the
/// long stems `flexible` exports as deformable curves a physics engine bends:
/// `variations` is the budget of distinct meshes and `detail` the triangles a
/// leaf is cut into.
#[derive(Resource, Reflect, Clone, Debug, PartialEq)]
#[cfg_attr(
    feature = "python",
    pyo3::pyclass(get_all, set_all, skip_from_py_object)
)]
pub struct WeedParams {
    /// What is done to the under-vine strip, by name, which picks the
    /// species: `herbicide` leaves the annual grasses and tall bolters a spray
    /// does not kill, `tilled` the annuals that come back from seed, `mown`
    /// the rosettes and tufts that duck the blade, `untouched` everything.
    ///
    /// A name that is none of [`Strip::NAMES`] is read as the default with a
    /// warning; Python rejects it at the boundary instead.
    #[reflect(@Choices(&Strip::NAMES))]
    pub strip: String,
    /// How far the under-vine strip reaches either side of the trunks, in
    /// meters.
    #[reflect(@Slider { min: 0.1, max: 0.6, step: 0.05 })]
    pub strip_width: f32,
    /// Plants per square metre in the strip. Zero is a clean strip; the
    /// ceiling is one plant per slot, twenty-five.
    #[reflect(@Slider { min: 0.0, max: 25.0, step: 0.5 })]
    pub pressure: f32,
    /// Plants per square metre in the alley, between the strips — the
    /// escapes.
    #[reflect(@Slider { min: 0.0, max: 5.0, step: 0.1 }, @Label("Alley weeds"))]
    pub alley_pressure: f32,
    /// The share of plants that are tall — bolters and broadleaves — as
    /// against low tufts, mats and rosettes.
    #[reflect(@Slider { min: 0.0, max: 1.0, step: 0.05 }, @Label("Tall share"))]
    pub tall: f32,
    /// Whether a stem at least 30 cm long — a bolter's wand, a broadleaf's
    /// runners — is exported as a deformable curve a solver bends, with its
    /// leaves riding it. Each is a chain of rigid bodies, ten to the metre:
    /// the default parcel at the defaults bends about 2,500 stems, 12,000
    /// bodies.
    ///
    /// See [`LONG_STEM`] and the module docs for the prims. A stem authored
    /// into a post, or into ground rising faster than it does, is pushed
    /// clear as the simulation starts.
    #[reflect(@Label("Bendable stems"))]
    pub flexible: bool,
    /// Whether a stem that bends is drawn as one skin bending with it, a
    /// mesh bound to every segment of its rod, rather than as a tube on each
    /// segment, which kinks at every joint. A skin bends only where a
    /// consumer poses it from the rod's bodies, as `misina_lab.isaaclab`
    /// does under Newton; anywhere else it keeps the shape it was built in.
    #[reflect(@Label("Skinned stems"))]
    pub skinned: bool,
    /// How many distinct plant meshes the scene may hold. A budget, not a
    /// count.
    ///
    /// See [`WeedMetric`] for what it is spent on.
    #[reflect(@Slider { min: 1.0, max: 64.0, step: 1.0 })]
    pub variations: u32,
    /// Triangles a leaf is cut into.
    #[reflect(@Slider { min: 4.0, max: 64.0, step: 2.0 })]
    pub detail: u32,
}

impl Default for WeedParams {
    fn default() -> Self {
        Self {
            strip: Strip::default().name().to_string(),
            strip_width: 0.3,
            pressure: 6.0,
            alley_pressure: 0.5,
            tall: 0.3,
            flexible: false,
            skinned: false,
            variations: 24,
            detail: 16,
        }
    }
}

impl WeedParams {
    /// The parsed strip regime. An unknown name comes back as the default
    /// rather than as an error: a build system is no place to fail, and the
    /// Python boundary has already rejected it — see `python.rs`.
    pub fn strip(&self) -> Strip {
        Strip::parse(&self.strip).unwrap_or_else(|| {
            warn!(
                "weed strip {:?} is not one of {:?}; using {:?}",
                self.strip,
                Strip::NAMES,
                Strip::default().name()
            );
            Strip::default()
        })
    }
}

pub fn plugin(app: &mut App) {
    app.init_resource::<WeedParams>().add_systems(
        PreUpdate,
        (
            author.run_if(regime_changed),
            // Gated on the regime as well as on the configs, so a change that
            // leaves no plants at all still clears the library.
            build.run_if(configs_changed::<WeedConfig>.or_eager(regime_changed)),
        )
            .chain()
            .in_set(Grow::Scatter),
    );
}

/// Run condition: something this layer authors from has changed.
fn regime_changed(
    params: Res<WeedParams>,
    layout: Res<VineyardLayout>,
    scene: Res<SceneParams>,
) -> bool {
    params.is_changed() || layout.is_changed() || scene.is_changed()
}

// ─── Authoring ──────────────────────────────────────────────────────

/// Places a plant config on every kept slot of every strip and alley.
pub(crate) fn author(
    mut commands: Commands,
    scene: Res<SceneParams>,
    params: Res<WeedParams>,
    layout: Res<VineyardLayout>,
    ground: Res<Ground>,
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

    let shares = shares(params.strip(), scene.season, params.tall.clamp(0.0, 1.0));
    let mut planter = Planter {
        commands: &mut commands,
        shares: &shares,
        detail: params.detail,
        season: scene.season,
        bounds: layout.bounds,
        ground: &ground,
        order: 0,
    };

    let strip_width = params.strip_width.max(0.0);
    for (index, row) in layout.rows.iter().enumerate() {
        let mut rng = Rng::new(scene.seed ^ STRIP_STREAM ^ salt(index as u64));
        planter.plant(
            weeds,
            format!("Row_{index:03}"),
            &row.band(strip_width),
            params.pressure,
            &mut rng,
        );
    }
    for alley in layout.alleys() {
        // The alley less the strips either side of it.
        let half_width = alley.band.half_width - strip_width;
        if half_width <= 0.0 {
            continue;
        }
        let mut rng = Rng::new(scene.seed ^ ALLEY_STREAM ^ salt(alley.index as u64));
        planter.plant(
            weeds,
            format!("Alley_{:03}", alley.index),
            &Band {
                half_width,
                ..alley.band
            },
            params.alley_pressure,
            &mut rng,
        );
    }

    if planter.order == 0 {
        commands.entity(weeds).despawn();
    }
}

/// What every band is planted with.
struct Planter<'a, 'w, 's> {
    commands: &'a mut Commands<'w, 's>,
    shares: &'a [f32],
    detail: u32,
    season: f32,
    bounds: Rect,
    ground: &'a Ground,
    order: u64,
}

impl Planter<'_, '_, '_> {
    /// Plants `band` at `pressure`, under a `Scope` named `name` — spawned
    /// only if a plant survives.
    ///
    /// Five draws per slot, after the grid's own two: whether it is kept,
    /// which species, its vigour, its stage, its yaw. All five are taken for
    /// every slot, so thinning the pressure leaves the survivors as they were.
    fn plant(&mut self, parent: Entity, name: String, band: &Band, pressure: f32, rng: &mut Rng) {
        let keep = (pressure * SLOT * SLOT).clamp(0.0, 1.0) as f64;
        let mut plants = Vec::new();
        for slot in jittered_grid(band, SLOT, SLOT_JITTER, rng) {
            let kept = rng.unit() < keep;
            let species = pick(self.shares, rng.unit());
            let vigour = rng.range(VIGOUR.start, VIGOUR.end);
            let stage = self.season as f64 + rng.range(-STAGE_JITTER, STAGE_JITTER);
            let yaw = rng.unit() * TAU;
            if !kept || !self.bounds.contains(slot.position) {
                continue;
            }
            plants.push((
                slot.index,
                self.ground.lift(slot.position),
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
                ChildOf(parent),
            ))
            .id();
        for (slot, position, yaw, config) in plants {
            self.order += 1;
            self.commands.spawn((
                Name::new(format!("Weed_{slot:04}")),
                // Upright: a plant grows against gravity whatever the slope.
                placed(position, yaw, Vec2::ZERO, 1.0),
                Visibility::default(),
                config,
                Order(self.order),
                ChildOf(group),
            ));
        }
    }
}

// ─── Building ───────────────────────────────────────────────────────

/// Builds one mesh per distinct plant and gives it to every plant that drew
/// it. A plant whose stems all stay rigid has nothing hanging off it, so it
/// carries its geometry directly; one with bending stems holds its crown and
/// a rod per bending stem, as the module docs draw it.
pub(crate) fn build(
    mut commands: Commands,
    mut library: Library,
    params: Res<WeedParams>,
    plants: Query<(Entity, &Order, &WeedConfig)>,
) -> Result<()> {
    library.clear(PART);
    library.clear(STALK_TUBE);
    library.clear(STALK_LEAVES);

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

    let (flexible, skinned) = (params.flexible, params.skinned);
    let built = par_map(&book.representatives, |index, config| {
        grow(config, index as u64)
            .and_then(|plant| plant.build(flexible, skinned))
            .with_context(|| {
                format!(
                    "weed {} could not be built",
                    SPECIES[config.species as usize].name
                )
            })
    });

    // Tubes and leaves are numbered across the layer.
    let (mut tubes, mut leaves) = (0, 0);
    let mut registered: Vec<(Option<Geometry>, Vec<Rod<Geometry>>, Surface)> =
        Vec::with_capacity(book.len());
    for (index, built) in built.into_iter().enumerate() {
        let Built { rigid, rods } = built?;
        let skin = surface(index as u64);
        let rigid = rigid.map(|mesh| library.part(PART, index, mesh.to_mesh(), skin));
        let mut register = |prefix: &str, count: &mut usize, mesh: Mesh| {
            *count += 1;
            library.part(prefix, *count - 1, mesh, skin)
        };
        let rods = rods
            .into_iter()
            .map(|rod| Rod {
                tubes: rod
                    .tubes
                    .into_iter()
                    .map(|tube| register(STALK_TUBE, &mut tubes, tube))
                    .collect(),
                skin: rod.skin.map(|mesh| register(STALK_TUBE, &mut tubes, mesh)),
                leaves: rod
                    .leaves
                    .into_iter()
                    .map(|mesh| mesh.map(|mesh| register(STALK_LEAVES, &mut leaves, mesh)))
                    .collect(),
                points: rod.points,
                widths: rod.widths,
                frames: rod.frames,
            })
            .collect();
        registered.push((rigid, rods, skin));
    }

    for ((_, entity, _), drew) in grown.iter().zip(&book.assignment) {
        let (rigid, rods, skin) = &registered[*drew as usize];
        if rods.is_empty() {
            if let Some(rigid) = rigid {
                commands.entity(*entity).insert(rigid.clone());
            }
            continue;
        }
        if let Some(rigid) = rigid {
            commands
                .entity(*entity)
                .with_child((Name::new(CROWN), rigid.clone()));
        }
        for (k, rod) in rods.iter().enumerate() {
            let mut stalk = commands.spawn((
                Name::new(format!("{STALK}_{k}")),
                Transform::IDENTITY,
                Visibility::default(),
                ChildOf(*entity),
            ));
            let curve = cable(rod.points.clone(), rod.widths.clone(), Vec::new(), *skin);
            let segments = spawn_rod(
                &mut stalk,
                curve,
                &rod.frames,
                &rod.tubes,
                rod.skin.as_ref(),
            );
            for (segment, leaves) in segments.iter().zip(&rod.leaves) {
                if let Some(leaves) = leaves {
                    commands
                        .entity(*segment)
                        .with_child((Name::new(LEAVES), leaves.clone()));
                }
            }
        }
    }
    Ok(())
}

// ─── Layout ─────────────────────────────────────────────────────────

/// A plant as its habit lays it out, in its own frame: the parts that stand
/// rigid however it is built, and its stems, each with the leaves along it.
#[derive(Default)]
struct Plant {
    parts: Vec<MeshData>,
    stems: Vec<Stem>,
}

/// A plant built from its layout: one mesh for whatever stays rigid, none if
/// everything hangs on a rod, and its rods.
struct Built {
    rigid: Option<MeshData>,
    rods: Vec<Rod<Mesh>>,
}

/// A bending stem: the curve a solver bends, and per segment its frame, its
/// tube and the leaves it carries, both in that frame. A skinned stem has no
/// tubes but its skin, in the plant's frame.
struct Rod<M> {
    points: Vec<[f32; 3]>,
    widths: Vec<f32>,
    frames: Vec<Transform>,
    tubes: Vec<M>,
    skin: Option<M>,
    /// `None` on a segment that carries no leaf.
    leaves: Vec<Option<M>>,
}

impl Plant {
    /// One mesh for whatever stays rigid, and a rod for every stem at least
    /// [`LONG_STEM`] long if `flexible`, drawn by a skin if `skinned`.
    fn build(self, flexible: bool, skinned: bool) -> anyhow::Result<Built> {
        let mut rigid = self.parts;
        let mut rods = Vec::new();
        for stem in self.stems {
            if flexible && stem.length() >= LONG_STEM {
                rods.push(stem.rod(skinned)?);
            } else {
                rigid.push(stem.mesh()?);
                rigid.extend(
                    stem.leaves
                        .iter()
                        .map(|(placement, leaf)| leaf.transformed(placement)),
                );
            }
        }
        Ok(Built {
            rigid: (!rigid.is_empty()).then(|| merge_meshes(&rigid)),
            rods,
        })
    }
}

/// One stem: a strand through `points`, `radii` across at them, laid into
/// the plant by `frame`, and the leaves hung along it. The points must not be
/// collinear — a straight polyline has no curvature for the tube's frames to
/// follow — so every habit bows its stems by at least a millimetre.
struct Stem {
    points: Vec<Point3<f64>>,
    radii: Vec<f64>,
    frame: Transform,
    /// Each leaf's placement in the plant's frame, whose origin is where it
    /// attaches, and its mesh.
    leaves: Vec<(Transform, MeshData)>,
}

impl Stem {
    fn new(points: Vec<Point3<f64>>, radii: Vec<f64>, frame: Transform) -> Self {
        Self {
            points,
            radii,
            frame,
            leaves: Vec::new(),
        }
    }

    /// The point `t` of the way along the stem, by control segment, in the
    /// plant's frame.
    fn at(&self, t: f64) -> Vec3 {
        self.frame.transform_point(along(&self.points, t))
    }

    /// Hangs `leaf` at `placement`, in the plant's frame.
    fn hang(&mut self, leaf: &MeshData, placement: Transform) {
        self.leaves.push((placement, leaf.clone()));
    }

    /// Along the control points, in meters.
    fn length(&self) -> f64 {
        self.points
            .windows(2)
            .map(|pair| (pair[1] - pair[0]).norm())
            .sum()
    }

    /// The stem's strand, skinned at [`SIDES`] and [`RINGS_PER_METER`] with
    /// no bark, in the stem's own frame.
    fn strand(&self) -> Strand {
        Strand::new(
            self.points.clone(),
            self.radii.clone(),
            SIDES,
            RINGS_PER_METER,
            Bark::none(),
        )
    }

    /// The stem as one rigid tube.
    fn mesh(&self) -> anyhow::Result<MeshData> {
        Ok(strand_mesh(&self.strand())?.transformed(&self.frame))
    }

    /// The stem as a rod: the centerline its tube is skinned along, cut into
    /// even segments as near [`ROD_SEGMENT`] long as fit, with the taper at
    /// each cut. The first segment is the held one, so the stem is held at
    /// its base. Each leaf rides the segment nearest where it attaches.
    /// `skinned`, the rod is drawn by the stem itself, skinned to it.
    fn rod(self, skinned: bool) -> anyhow::Result<Rod<Mesh>> {
        let count = (self.length() / ROD_SEGMENT).round() as usize;
        let (points, widths): (Vec<[f32; 3]>, Vec<f32>) = self
            .strand()
            .centerline(count)?
            .into_iter()
            .map(|(p, radius)| {
                let p = Vec3::new(p.x as f32, p.y as f32, p.z as f32);
                (
                    self.frame.transform_point(p).to_array(),
                    2.0 * radius as f32,
                )
            })
            .unzip();

        // How far `at` lies from segment `k`.
        let off = |at: Vec3, k: usize| {
            let (a, b) = (Vec3::from(points[k]), Vec3::from(points[k + 1]));
            let t = ((at - a).dot(b - a) / (b - a).length_squared()).clamp(0.0, 1.0);
            at.distance(a.lerp(b, t))
        };
        let frames = segment_frames(&points);
        let mut carried = vec![Vec::new(); frames.len()];
        for (placement, leaf) in &self.leaves {
            let at = placement.translation;
            let k = (0..frames.len())
                .min_by(|a, b| off(at, *a).total_cmp(&off(at, *b)))
                .expect("a rod has segments");
            carried[k].push(leaf.transformed(&rebase(&frames[k], placement)));
        }
        let (tubes, skin) = if skinned {
            let mesh = strand_mesh_ringed(&self.strand(), ROD_SEGMENT / 4.0)?;
            let skin = skin_mesh(&mesh.transformed(&self.frame), &points);
            (Vec::new(), Some(skin))
        } else {
            let tubes = segment_tubes(&points, &widths, SIDES);
            (tubes.iter().map(MeshData::to_mesh).collect(), None)
        };
        Ok(Rod {
            tubes,
            skin,
            leaves: carried
                .iter()
                .map(|leaves| (!leaves.is_empty()).then(|| merge_meshes(leaves).to_mesh()))
                .collect(),
            points,
            widths,
            frames,
        })
    }
}

/// One plant laid out in its own frame: the root collar on the origin,
/// growing up `+Z`.
fn grow(config: &WeedConfig, index: u64) -> anyhow::Result<Plant> {
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
fn tuft(size: f64, stage: f64, triangles: f64, rng: &mut Rng) -> anyhow::Result<Plant> {
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
    Ok(Plant { parts, ..default() })
}

/// *Stellaria media*: stems running out along the ground from one root,
/// small ovate leaves in opposite pairs along them.
///
/// Draws: per stem its heading, its sway and the lift of its tip; per leaf
/// its turn off the stem and its pitch.
fn mat(size: f64, stage: f64, triangles: f64, rng: &mut Rng) -> anyhow::Result<Plant> {
    let leaf = bent(
        shapes::leaf(0.015 * size, 0.008 * size, 0.4),
        0.3,
        0.0,
        triangles,
    )?;
    let run = 0.18 * size * (0.3 + 0.7 * stage);
    let stems = 4;
    let mut plant = Plant::default();
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
        let mut stem = Stem::new(points, vec![0.001, 0.0009, 0.0006], frame);

        let nodes = ((run / 0.025) as usize).max(1);
        for n in 1..=nodes {
            let at = stem.at(n as f64 / nodes as f64);
            for side in [-1.0f64, 1.0] {
                let turn = heading + side * rng.range(0.9, 1.5);
                let pitch = rng.range(0.1, 0.4);
                stem.hang(&leaf, leaf_at(at, turn, pitch, 1.0));
            }
        }
        plant.stems.push(stem);
    }
    Ok(plant)
}

/// *Taraxacum officinale*: a flat whorl of toothed leaves from a taproot's
/// crown, nothing standing up.
///
/// Draws: per leaf its length, its heading, its pitch and its droop.
fn rosette(size: f64, stage: f64, triangles: f64, rng: &mut Rng) -> anyhow::Result<Plant> {
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
    Ok(Plant { parts, ..default() })
}

/// *Erigeron canadensis*: a basal rosette that, once bolted, sends up one
/// wand clothed spirally in narrow leaves — to over a metre, into the
/// fruiting zone. The rosette withers as the wand grows.
///
/// Draws: per basal leaf its length, its heading and its pitch; then the
/// wand's sway; per wand leaf its turn and its pitch.
fn bolter(size: f64, stage: f64, triangles: f64, rng: &mut Rng) -> anyhow::Result<Plant> {
    let mut plant = Plant::default();
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
        plant
            .parts
            .push(leaf.transformed(&leaf_at(Vec3::Z * 0.005, heading, pitch, 1.0)));
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
        let radii = vec![0.004 * size, 0.003 * size, 0.0015 * size];
        let mut wand = Stem::new(points, radii, Transform::IDENTITY);

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
            wand.hang(&leaf, leaf_at(wand.at(t), turn, pitch, shrink));
        }
        plant.stems.push(wand);
    }
    Ok(plant)
}

/// *Malva sylvestris*: stems that run out from a woody crown and then turn
/// up, leaving the middle hollow, with round leaves off them. The shape a
/// weeder cannot reach the middle of.
///
/// Draws: per stem its heading and its sway; per leaf its turn and pitch.
fn broadleaf(size: f64, stage: f64, triangles: f64, rng: &mut Rng) -> anyhow::Result<Plant> {
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
    let mut plant = Plant::default();
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
        let mut stem = Stem::new(points, radii, frame);

        let nodes = 4;
        for n in 1..=nodes {
            let at = stem.at(n as f64 / nodes as f64);
            let turn = heading + rng.range(-1.2, 1.2);
            let pitch = rng.range(0.1, 0.5);
            stem.hang(&leaf, leaf_at(at, turn, pitch, 1.0));
        }
        plant.stems.push(stem);
    }
    Ok(plant)
}

/// A plant's foliage, shaded per representative.
fn surface(index: u64) -> Surface {
    material::FOLIAGE.blade(color::shade(
        color::srgb(color::WEED),
        &mut Rng::new(color::COLOR_STREAM ^ WEED_SHADE ^ salt(index)),
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::elements::VineyardParams;
    use crate::elements::util::testing::{
        bounds, grown, named_children, organs, prim, prim_path, skin_span,
    };
    use bevy::mesh::VertexAttributeValues;
    use misina_lab::geometry::mesh::box_mesh;
    use misina_lab::scene::{Cable, Prototypes, STEM, UsdReference};

    /// A parcel small enough to grow in a test and wide enough for three rows.
    fn small() -> VineyardParams {
        let mut params = VineyardParams::default();
        params.terrain.length = 30.0;
        params.terrain.width = 20.0;
        params
    }

    /// The plant `config` grows with nothing bending: one mesh.
    fn whole(config: WeedConfig, index: u64) -> MeshData {
        let built = grow(&config, index)
            .and_then(|plant| plant.build(false, false))
            .unwrap_or_else(|e| panic!("{e:#}"));
        assert!(built.rods.is_empty());
        built.rigid.expect("every plant grows something")
    }

    fn part_count(app: &App) -> usize {
        app.world()
            .resource::<Prototypes>()
            .iter()
            .filter(|(name, _)| name.starts_with("Weed_"))
            .count()
    }

    #[test]
    fn plants_stand_upright_on_the_ground_in_a_strip_or_an_alley() {
        let mut app = grown(small());
        let ground = app.world().resource::<Ground>().clone();
        let layout = app.world().resource::<VineyardLayout>().clone();
        let plants = organs::<WeedConfig>(app.world_mut());
        assert!(plants.len() > 50, "got {}", plants.len());

        let strips: Vec<Band> = layout.rows.iter().map(|row| row.band(0.3)).collect();
        let alleys: Vec<Band> = layout.alleys().iter().map(|alley| alley.band).collect();
        for plant in &plants {
            let p = plant.position();
            assert!(
                (p.z - ground.height(p.x, p.y)).abs() < 1e-3,
                "{} floats",
                plant.path
            );
            assert!(plant.up().z > 0.999, "{} leans", plant.path);
            let in_strip = strips.iter().any(|b| b.contains(p.truncate()));
            let in_alley = alleys.iter().any(|b| b.contains(p.truncate()));
            assert!(in_strip || in_alley, "{} is nowhere", plant.path);
            assert_eq!(
                plant.path.starts_with("Weeds/Row_"),
                in_strip,
                "{} is filed under the wrong zone",
                plant.path
            );
        }
    }

    #[test]
    fn the_pressure_is_plants_per_square_metre_of_strip() {
        let mut params = small();
        params.weed.alley_pressure = 0.0;
        params.weed.pressure = 6.0;
        let mut app = grown(params);
        let layout = app.world().resource::<VineyardLayout>().clone();
        let area: f32 = layout
            .rows
            .iter()
            .map(|row| row.band(0.3).length() * 0.6)
            .sum();
        let expected = 6.0 * area;
        let got = organs::<WeedConfig>(app.world_mut()).len() as f32;
        assert!(
            (got - expected).abs() < expected * 0.2,
            "expected about {expected} plants, got {got}"
        );
    }

    #[test]
    fn no_pressure_grows_nothing() {
        let mut params = small();
        params.weed.pressure = 0.0;
        params.weed.alley_pressure = 0.0;
        let mut app = grown(params);
        assert!(prim(app.world_mut(), &["Weeds"]).is_none());
        assert_eq!(part_count(&app), 0);
    }

    #[test]
    fn an_unknown_strip_falls_back_without_panicking() {
        let mut params = small();
        params.weed.strip = "napalm".into();
        let mut app = grown(params);
        assert!(!organs::<WeedConfig>(app.world_mut()).is_empty());
    }

    fn histogram(params: VineyardParams) -> Vec<usize> {
        let mut counts = vec![0; SPECIES.len()];
        for plant in organs::<WeedConfig>(grown(params).world_mut()) {
            counts[plant.config.species as usize] += 1;
        }
        counts
    }

    /// The regime picks the flora: a sprayed strip and one left alone do not
    /// grow the same things, and no share can be turned on without the tall
    /// ones following it.
    #[test]
    fn the_regime_and_the_tall_share_pick_the_species() {
        let mut sprayed = small();
        sprayed.weed.strip = "herbicide".into();
        let mut left = small();
        left.weed.strip = "untouched".into();
        assert_ne!(histogram(sprayed), histogram(left));

        let mut low = small();
        low.weed.tall = 0.0;
        let counts = histogram(low);
        for (species, count) in SPECIES.iter().zip(&counts) {
            if matches!(species.habit, Habit::Bolter | Habit::Broadleaf) {
                assert_eq!(*count, 0, "{} grew with no tall share", species.name);
            }
        }
        assert!(counts.iter().sum::<usize>() > 0);
    }

    #[test]
    fn shares_sum_to_one_and_honour_the_tall_share() {
        for strip in Strip::ALL {
            for season in [0.0, 0.5, 1.0] {
                let s = shares(strip, season, 0.3);
                assert!(
                    (s.iter().sum::<f32>() - 1.0).abs() < 1e-5,
                    "{strip:?} {season}"
                );
                let tall: f32 = SPECIES
                    .iter()
                    .zip(&s)
                    .filter(|(sp, _)| matches!(sp.habit, Habit::Bolter | Habit::Broadleaf))
                    .map(|(_, w)| w)
                    .sum();
                assert!((tall - 0.3).abs() < 1e-5);
            }
        }
    }

    #[test]
    fn the_budget_caps_the_plant_meshes() {
        let mut params = small();
        params.weed.variations = 4;
        assert!(part_count(&grown(params)) <= 4);
    }

    #[test]
    fn a_bolted_bolter_stands_taller_than_its_rosette() {
        let species = SPECIES
            .iter()
            .position(|s| s.habit == Habit::Bolter)
            .unwrap();
        let low = whole(WeedConfig::new(species, 0.1, 1.0, 16), 0);
        let high = whole(WeedConfig::new(species, 1.0, 1.0, 16), 0);
        let (_, low_top) = bounds(&low, 2);
        let (_, high_top) = bounds(&high, 2);
        assert!(low_top < 0.1, "a rosette: {low_top}");
        assert!(high_top > 0.8, "a wand: {high_top}");
    }

    /// Every habit builds, stands on its own ground and stays about the size
    /// its species is.
    #[test]
    fn every_species_builds_a_grounded_plant() {
        for (species, expected) in SPECIES.iter().enumerate() {
            let mesh = whole(WeedConfig::new(species, 0.7, 1.0, 16), 3);
            assert!(!mesh.face_vertex_counts.is_empty());
            let (lo, hi) = bounds(&mesh, 2);
            assert!(lo > -0.02, "{} digs to {lo}", expected.name);
            assert!(hi > 0.02 && hi < 1.5, "{} reaches {hi}", expected.name);
        }
    }

    /// The stems of `species` long enough to bend, at `stage` and its typical
    /// size.
    fn long_stems(species: usize, stage: f32) -> Vec<Stem> {
        let plant = grow(&WeedConfig::new(species, stage, 1.0, 16), 0).unwrap();
        plant
            .stems
            .into_iter()
            .filter(|stem| stem.length() >= LONG_STEM)
            .collect()
    }

    /// A bolter's wand and a broadleaf's four runners bend once grown; a
    /// mat's runners and a wand just bolted stay in the mesh.
    #[test]
    fn only_a_long_stem_bends() {
        for (species, expected) in SPECIES.iter().enumerate() {
            let long = match expected.habit {
                Habit::Bolter => 1,
                Habit::Broadleaf => 4,
                _ => 0,
            };
            assert_eq!(long_stems(species, 1.0).len(), long, "{}", expected.name);
        }
        let bolter = SPECIES
            .iter()
            .position(|s| s.habit == Habit::Bolter)
            .unwrap();
        // A wand of 17 cm.
        assert!(long_stems(bolter, 0.4).is_empty());
    }

    /// A rod starts where its stem does, held there by its first segment, and
    /// runs along it in even segments about [`ROD_SEGMENT`] long, three at
    /// least, tapering as the stem does.
    #[test]
    fn a_rod_is_its_stem_cut_into_even_segments() {
        let stems: Vec<Stem> = (0..SPECIES.len())
            .flat_map(|species| long_stems(species, 1.0))
            .collect();
        assert!(!stems.is_empty());
        for stem in stems {
            let (base, radii) = (stem.at(0.0), stem.radii.clone());
            let rod = stem.rod(false).unwrap();
            let spans: Vec<f32> = rod
                .points
                .windows(2)
                .map(|pair| Vec3::from(pair[0]).distance(Vec3::from(pair[1])))
                .collect();
            assert!(spans.len() >= 3, "{} segments", spans.len());
            assert!(Vec3::from(rod.points[0]).abs_diff_eq(base, 1e-6));
            let longest = spans.iter().copied().fold(0.0, f32::max);
            for span in &spans {
                assert!(*span > 0.95 * longest, "{span} against {longest}");
                assert!((span - ROD_SEGMENT as f32).abs() < 0.2 * ROD_SEGMENT as f32);
            }
            assert!((rod.widths[0] - 2.0 * radii[0] as f32).abs() < 1e-6);
            let tip = 2.0 * radii[radii.len() - 1] as f32;
            assert!((rod.widths[rod.widths.len() - 1] - tip).abs() < 1e-6);
            assert!(rod.widths.windows(2).all(|pair| pair[1] <= pair[0]));
        }
    }

    /// A leaf rides the segment it attaches to, placed in that segment's
    /// frame so that the frame — which the solver moves — puts it back where
    /// it was hung.
    #[test]
    fn a_leaf_rides_the_segment_it_attaches_to() {
        let leaf = box_mesh(0.01);
        for t in [0.0, 0.3, 0.55, 1.0] {
            let mut stem = Stem::new(
                vec![
                    Point3::new(0.0, 0.0, 0.0),
                    Point3::new(0.01, 0.0, 0.4),
                    Point3::new(0.02, 0.0, 0.8),
                ],
                vec![0.004, 0.003, 0.002],
                Transform::from_rotation(Quat::from_rotation_z(0.7)),
            );
            let at = stem.at(t);
            let placement = leaf_at(at, 1.0, 0.5, 0.8);
            stem.hang(&leaf, placement);
            let rod = stem.rod(false).unwrap();

            let carriers: Vec<usize> = (0..rod.leaves.len())
                .filter(|k| rod.leaves[*k].is_some())
                .collect();
            let [k] = carriers[..] else {
                panic!("the leaf hung at {t} rides {carriers:?}");
            };
            let (from, to) = (rod.points[k][2], rod.points[k + 1][2]);
            assert!(
                from - 1e-3 <= at.z && at.z <= to + 1e-3,
                "the leaf at {} rides {from}..{to}",
                at.z
            );
            let back = rod.leaves[k].clone().unwrap().transformed_by(rod.frames[k]);
            let Some(VertexAttributeValues::Float32x3(back)) =
                back.attribute(Mesh::ATTRIBUTE_POSITION)
            else {
                panic!("a leaf has points");
            };
            let hung = leaf.transformed(&placement);
            for (a, b) in back.iter().zip(&hung.points) {
                assert!(Vec3::from(*a).abs_diff_eq(Vec3::from(*b), 1e-5));
            }
        }
    }

    /// With bending stems on, a plant with a long stem holds a rod per such
    /// stem, its leaves riding the segments, and keeps the rest of itself as
    /// its crown; one without stays a single mesh. Off, as it is by default,
    /// no weed carries a curve.
    #[test]
    fn a_bending_weed_hangs_its_long_stems_on_rods() {
        let curves = |app: &mut App| -> usize {
            let mut query = app.world_mut().query_filtered::<Entity, With<Cable>>();
            let found: Vec<Entity> = query.iter(app.world()).collect();
            found
                .into_iter()
                .filter(|curve| prim_path(app.world(), *curve).starts_with("Weeds/"))
                .count()
        };
        let mut params = small();
        params.weed.tall = 1.0;
        assert_eq!(curves(&mut grown(params.clone())), 0);

        params.weed.flexible = true;
        let mut app = grown(params);
        let mut rods = 0;
        for plant in organs::<WeedConfig>(app.world_mut()) {
            let path: Vec<&str> = plant.path.split('/').collect();
            let entity = prim(app.world_mut(), &path).unwrap();
            let children = named_children(app.world_mut(), entity);
            let stalks: Vec<Entity> = children
                .iter()
                .filter(|(name, _)| name.starts_with(STALK))
                .map(|(_, stalk)| *stalk)
                .collect();
            let whole = app.world().entity(entity).contains::<UsdReference>();
            assert_eq!(whole, stalks.is_empty(), "{}", plant.path);
            if !whole && SPECIES[plant.config.species as usize].habit == Habit::Bolter {
                assert!(
                    children.iter().any(|(name, _)| name == CROWN),
                    "{}: a bolter's rosette stays rigid",
                    plant.path
                );
            }

            for stalk in stalks {
                rods += 1;
                let segments = named_children(app.world_mut(), stalk);
                let leafy = segments.iter().any(|(_, segment)| {
                    named_children(app.world_mut(), *segment)
                        .iter()
                        .any(|(name, _)| name == LEAVES)
                });
                assert!(leafy, "{}: no leaves ride its rod", plant.path);
            }
        }
        assert_eq!(curves(&mut app), rods);
        assert!(rods > 20, "only {rods} rods");
    }

    /// Skinned, every bending stem is drawn by one skin bound from its base
    /// to its tip, and its segments carry no tube.
    #[test]
    fn a_skinned_weed_draws_each_rod_with_a_skin() {
        let mut params = small();
        (params.weed.tall, params.weed.flexible, params.weed.skinned) = (1.0, true, true);
        let mut app = grown(params);
        let mut skins = 0;
        for plant in organs::<WeedConfig>(app.world_mut()) {
            let path: Vec<&str> = plant.path.split('/').collect();
            let entity = prim(app.world_mut(), &path).unwrap();
            for (name, stalk) in named_children(app.world_mut(), entity) {
                if !name.starts_with(STALK) {
                    continue;
                }
                let segments: Vec<Entity> = named_children(app.world_mut(), stalk)
                    .into_iter()
                    .filter(|(name, _)| name.contains("_edge_body_"))
                    .map(|(_, segment)| segment)
                    .collect();
                let (reach, bare) = skin_span(app.world_mut(), stalk).expect("a skin");
                assert!(
                    (reach - segments.len() as f32).abs() < 1e-3 && bare < 0.5,
                    "{}/{name}: bound to {reach} of {} segments, {bare} of one bare",
                    plant.path,
                    segments.len()
                );
                for segment in segments {
                    let drawn = named_children(app.world_mut(), segment);
                    assert!(drawn.iter().all(|(name, _)| name != STEM), "{}", plant.path);
                }
                skins += 1;
            }
        }
        assert!(skins > 20, "only {skins} skins");
    }

    #[test]
    fn the_weeds_are_deterministic() {
        let a = organs::<WeedConfig>(grown(small()).world_mut());
        let b = organs::<WeedConfig>(grown(small()).world_mut());
        assert_eq!(a.len(), b.len());
        for (x, y) in a.iter().zip(&b) {
            assert_eq!(
                (&x.path, x.transform, x.config),
                (&y.path, y.transform, y.config)
            );
        }
    }
}

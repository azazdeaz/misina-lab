//! Truss — a truss of tomatoes: the peduncle off the stem, the rachis the
//! fruit hang along, and a pedicel to every fruit.
//!
//! A truss is built in its own frame, on the stem's axis at the node it grew
//! from, facing +X with +Z up. The peduncle leaves the stem `peduncle_angle`
//! off vertical and bends over under the fruit's weight into the rachis,
//! which hangs [`RACHIS_TILT`] off straight down, out from the stem, at its
//! first fruit. The rachis is a chain of internodes, one to a fruit: a
//! pedicel leaves at every node, to alternate sides, and the rachis kinks
//! away from it there, so it zigzags, and hangs closer to straight down node
//! by node under the weight past it. A pedicel arches out and over and comes
//! straight down onto its fruit, which hangs off its tip a little off
//! vertical, bending at the knuckle just below:
//!
//! ```text
//!         Bend
//!     ╭────╮
//!  ───╯     ╲        a pedicel to every fruit, off alternate sides
//!  Cut       ╲ ●
//!          ●  ╱
//!            ╲  ●
//!          ●  ╱
//!             ╵
//! ```
//!
//! A branched truss, the cherry kind, forks into two rachises at the first
//! fruit and shares its fruit between them.
//!
//! Peduncle, rachis and pedicels are one mesh, quantized like any organ, to a
//! budget of `variations`. The representative a truss drew decides where its
//! `Cut` and `Bend` and its fruit go, and how its rachis kinks; which fruit
//! are missing, how big each one is and how it hangs are drawn per truss.
//!
//! ```text
//! Truss_04            Xform, a TrussConfig
//!   Peduncle          -> parts/Truss_k
//!   Cut               Xform at the peduncle's base, +X along the peduncle
//!   Bend              Xform where the peduncle bends most, +X along it
//!   Fruit_00 …        a FruitConfig, hung from a pedicel's tip
//! ```
//!
//! `Cut` and `Bend` are where the two kinds of harvesting gripper take a
//! truss: one cuts flush at the stem, the other clamps the bend and cuts
//! beside it. Both are frames, since the approach matters as much as the
//! point.
//!
//! A truss whose node is below
//! [`StemParams::flexible_below`](super::stem::StemParams::flexible_below)
//! is a rod instead. Its main axis, the peduncle and the first rachis, is a
//! [`cable`] a solver bends and a robot can cut, and everything else rides the
//! segment of it that carries it, in that segment's frame: the pedicels, the
//! fruit, a branched truss's second rachis on the segment it forks from, and
//! `Bend` on the one at the middle of the bend. The fruit's mass is the rod's
//! load, so it hangs, and falls once cut, with their weight. `Cut` stays on
//! the truss, where the rod's held segment ends:
//!
//! ```text
//! Truss_04              Xform, a TrussConfig
//!   Cable               the main axis, simulated; `guide`, so never drawn
//!   Cable_edge_body_0   the held segment, from the stem's axis out past it
//!     Stem              -> parts/TrussTube_n
//!   Cable_edge_body_1   a segment, its transform driven by the solver
//!     Stem
//!     Pedicels          -> parts/Pedicels_n, those leaving the axis here
//!     Fruit_00          rebased into the segment's frame
//!   …
//!   Cut
//! ```

use std::f32::consts::{FRAC_PI_2, PI, TAU};
use std::iter::once;

use bevy::prelude::*;
use misina_lab::geometry::mesh::{MeshData, merge_meshes};
use misina_lab::geometry::par_map;
use misina_lab::geometry::strand::{Bark, Bulge, Collar, Strand, sides_within, strand_mesh};
use misina_lab::palette::{COLOR_STREAM, Response, shade, srgb};
use misina_lab::params::{Label, Slider};
use misina_lab::quantize::{Metric, farthest_first};
use misina_lab::rng::{Rng, salt};
use misina_lab::scene::{
    Library, Order, cable, configs_changed, rebase, segment_frames, segment_of, segment_tubes,
    spawn_rod,
};
use nalgebra::Point3;

use super::fruit::{FruitConfig, FruitParams, Hung};
use super::stem::{MIN_DETAIL, StemParams};
use super::{Grow, SceneParams, vector};

/// The mesh-library prefix the trusses register under.
pub const PART: &str = "Truss";

/// The mesh-library prefixes a bent truss's parts register under: the tube
/// on each segment of its rod, and the pedicels a segment carries. Layers of
/// their own, because a representative has one of each per segment where it
/// has one mesh, and [`Library::clear`] drops a whole prefix at a time.
const TUBE: &str = "TrussTube";
const PEDICELS: &str = "Pedicels";

/// The held segment of a bent truss's rod, from the stem's axis out along the
/// peduncle, in meters: past the surface of the thickest stem, so a cut flush
/// at `Cut` lands within a splinter of the first joint (`SPLINTER` in
/// `misina_lab/isaaclab/cutting.py`) on the thinnest.
const HELD: f32 = 0.0135;

/// Length of one of the rod's moving segments, in meters: about ten to a
/// medium truss, so the bend turns about 45° at a joint.
const SEGMENT: f32 = 0.025;

/// Salts splitting a truss's shape draws, its rachis's kinks, its fruit draws
/// and each fruit's own shape off the scene seed.
const SHAPE_STREAM: u64 = 0x94D0_49BB_1331_11EB;
const NODE_STREAM: u64 = 0x5B3E_7A91_C04D_2F68;
const FRUIT_STREAM: u64 = 0xD6E8_FEB8_6659_FD93;
const FRUIT_SHAPE_STREAM: u64 = 0xA076_1D64_78BD_642F;

/// How far a truss's peduncle and fruit vary about the params, as fractions.
const LENGTH_JITTER: f64 = 0.2;
const RADIUS_JITTER: f64 = 0.15;
const ANGLE_JITTER: f64 = 0.15;
const COUNT_JITTER: f64 = 0.2;
const DIAMETER_JITTER: f64 = 0.08;

/// How far one fruit's diameter varies about its truss's, as a fraction, and
/// how much smaller the fruit at a truss's tip is than the one at its base.
const FRUIT_JITTER: f64 = 0.05;
const TIP_SHRINK: f32 = 0.15;

/// How far the rachis hangs off straight down, out from the stem, at its
/// first fruit, in radians; and how much closer to straight down it hangs
/// past each node, under the weight past it, never past straight down.
const RACHIS_TILT: f32 = 0.6;
const SAG: f32 = 0.1;

/// How far the rachis kinks at a node, in radians, away from the pedicel
/// that leaves there. Pedicels leave to alternate sides, so it zigzags.
const KINK: f32 = 0.6;

/// How far a node's sag and kink vary about those, as a fraction.
const NODE_JITTER: f64 = 0.5;

/// Rachis between neighbouring fruit, in fruit diameters. Neighbours hang off
/// alternate sides, so the fruit down one side are 1.1 diameters apart.
const FRUIT_PITCH: f32 = 0.55;

/// Rachis left past the last fruit, in fruit diameters.
const RACHIS_TIP: f32 = 0.3;

/// Where a pedicel's tip sits off the rachis, in fruit radii: out to the
/// side, and down. Far enough that its fruit clears the rachis. How far the
/// pedicel arches up on its way there, and how far above its tip it is
/// already heading straight down onto its fruit, in fruit radii, so the
/// fruit's stub carries straight on from it.
const PEDICEL: Vec2 = Vec2::new(0.9, 0.3);
const ARCH: f32 = 0.3;
const DROP: f32 = 0.25;

/// How far a fruit hangs off vertical, in radians, at most.
const TILT: f64 = 0.15;

/// The share of a peduncle that runs straight out of the stem before it
/// bends.
const STRAIGHT: f32 = 0.3;

/// Control points round the bend.
const BEND_POINTS: usize = 6;

/// Least the bend turns, in radians, so a peduncle already pointing down
/// still has a bend to clamp.
const MIN_TURN: f32 = 0.1;

/// A branched truss's two rachises, either side of where a simple one would
/// hang, in radians.
const BRANCH: f32 = 0.45;

/// The rachis's radius at its tip, as a fraction of the peduncle's.
const RACHIS_TAPER: f32 = 0.6;

/// A pedicel's radius as a fraction of its fruit's full-grown diameter, and
/// the least it is, in meters: about 3.5 mm across under a 60 mm fruit.
const PEDICEL_RADIUS: f32 = 0.028;
const MIN_PEDICEL: f32 = 0.0004;

/// The radius of the pedicel to a fruit `diameter` across when full grown,
/// which the fruit's stub carries on at.
pub fn pedicel_radius(diameter: f32) -> f32 {
    (PEDICEL_RADIUS * diameter).max(MIN_PEDICEL)
}

/// Floors on what Python may set, so that every truss still builds.
const MIN_LENGTH: f32 = 0.01;
const MIN_RADIUS: f32 = 0.0005;
const MIN_DIAMETER: f32 = 0.005;
const ANGLE: (f32, f32) = (0.05, PI - RACHIS_TILT - MIN_TURN);

/// Vertices round a bent truss's segment tubes, and the fewest round any of
/// its stems.
const SIDES: usize = 6;
const MIN_SIDES: usize = 5;

/// How far a truss's stems may stray from their shape at a stem `detail` of
/// 1, in meters: four times closer than a stem, a truss being about that
/// much thinner, and its meshes few.
const TOLERANCE: f64 = 0.00015;

/// How much a stalk swells where it leaves what it grows from, as a fraction
/// of its radius, and over how far, in radii of what it leaves: a peduncle
/// out of the stem, and a pedicel out of the rachis. How much the rachis
/// swells at a node, and over how many of its own radii.
const FLARE: (f64, f64) = (0.6, 1.5);
const KNOT: (f64, f64) = (0.25, 1.5);

/// A green peduncle.
const COLOR: u32 = 0x6A8A34;
const RESPONSE: Response = Response {
    roughness: 0.7,
    reflectance: 0.35,
};

/// One truss's shape, as the stem that grew it specified.
#[derive(Component, Clone, Copy, Debug, PartialEq)]
pub struct TrussConfig {
    /// Stem to the first fruit, in meters.
    pub peduncle_length: f32,
    pub peduncle_radius: f32,
    /// Between the stem and the peduncle where it leaves, in radians.
    pub peduncle_angle: f32,
    pub fruits: u32,
    pub fruit_diameter: f32,
    pub branched: bool,
    /// The stream its rachis's kinks are drawn from. [`TrussMetric`] does not
    /// read it: two trusses near in shape share a mesh, kinked as the
    /// representative's were.
    pub seed: u64,
    /// What it takes from its stem, kept so the params can be re-applied to a
    /// truss already hanging.
    pub node: Node,
}

/// What a truss takes from the node of the stem it hangs on. Placement and
/// ripening, not shape: [`TrussMetric`] reads none of it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Node {
    /// The stem's radius at the node; the `Cut` sits that far out along the
    /// peduncle.
    pub stem_radius: f32,
    /// How many weeks this one is behind its stem's ripening front, a truss
    /// a week, which its fruit's ripeness is read off.
    pub rank: f32,
    /// How far its fruit have come toward full size, 0 just set to 1: the
    /// trusses nearest the head set theirs last.
    pub growth: f32,
    /// Whether it is a rod a solver bends rather than a static mesh: its node
    /// is below [`StemParams::flexible_below`](super::stem::StemParams::flexible_below).
    pub flexible: bool,
}

impl TrussConfig {
    /// The truss these params call for at the `order`-th node a stem hung one
    /// on, its jitter drawn off a stream of its own.
    ///
    /// The *draw order* is part of this element's output: the peduncle's
    /// length, radius and angle, then the fruit count and diameter.
    pub fn new(params: &TrussParams, seed: u64, order: u64, node: Node) -> Self {
        let mut rng = Rng::new(seed ^ SHAPE_STREAM ^ salt(order));
        let mut about = |jitter: f64| rng.range(1.0 - jitter, 1.0 + jitter) as f32;
        let length = params.peduncle_length * about(LENGTH_JITTER);
        let radius = params.peduncle_diameter / 2.0 * about(RADIUS_JITTER);
        let angle = params.peduncle_angle.to_radians() * about(ANGLE_JITTER);
        let fruits = (params.fruits as f32 * about(COUNT_JITTER)).round();
        let diameter = params.fruit_diameter * about(DIAMETER_JITTER);
        Self {
            peduncle_length: length.max(MIN_LENGTH),
            peduncle_radius: radius.max(MIN_RADIUS),
            peduncle_angle: angle.clamp(ANGLE.0, ANGLE.1),
            fruits: fruits.max(1.0) as u32,
            fruit_diameter: diameter.max(MIN_DIAMETER),
            branched: params.branched,
            seed: seed ^ NODE_STREAM ^ salt(order),
            node,
        }
    }
}

/// Two trusses share a mesh when they are close in every dimension that
/// shows, weighted as roughly how far apart each moves a fruit.
pub struct TrussMetric;

impl Metric<TrussConfig> for TrussMetric {
    fn distance(&self, a: &TrussConfig, b: &TrussConfig) -> f32 {
        // A forked truss and a simple one are different shapes, not near
        // sizes of one.
        if a.branched != b.branched {
            return 1.0;
        }
        [
            a.peduncle_length - b.peduncle_length,
            (a.peduncle_radius - b.peduncle_radius) * 8.0,
            // A radian swings the rachis's root by a peduncle's length.
            (a.peduncle_angle - b.peduncle_angle) * a.peduncle_length.max(b.peduncle_length),
            // A fruit more is a pedicel more, half a fruit off the last.
            (a.fruits as f32 - b.fruits as f32) * a.fruit_diameter.max(b.fruit_diameter) / 2.0,
            // The rachis is spaced by the fruit, so its tip moves by the
            // difference times the fruit along it.
            (a.fruit_diameter - b.fruit_diameter) * FRUIT_PITCH * a.fruits.max(b.fruits) as f32,
        ]
        .iter()
        .map(|d| d * d)
        .sum::<f32>()
        .sqrt()
    }
}

/// The trusses: a peduncle off the stem, bending over into a rachis with the
/// fruit hung along it.
///
/// The peduncle numbers default to the means measured on medium trusses in a
/// high-wire house, and every truss draws its own about them. A medium truss
/// tomato is about 6 fruit of 60 mm; a cherry truss about 15 of 25 mm,
/// branched.
#[derive(Resource, Reflect, Clone, Debug, PartialEq)]
#[cfg_attr(
    feature = "python",
    pyo3::pyclass(get_all, set_all, skip_from_py_object)
)]
pub struct TrussParams {
    /// Fruit on a truss. Each truss draws a few more or fewer.
    #[reflect(@Slider { min: 1.0, max: 24.0, step: 1.0 })]
    pub fruits: u32,
    /// Diameter of a fruit at the base of a truss, in meters; the fruit
    /// toward its tip come out a little smaller.
    #[reflect(@Slider { min: 0.02, max: 0.08, step: 0.002 })]
    pub fruit_diameter: f32,
    /// Whether the rachis forks in two at the first fruit, as a cherry truss
    /// does, rather than hanging as one.
    pub branched: bool,
    /// Length of the peduncle from the stem to the first fruit, in meters.
    #[reflect(@Slider { min: 0.03, max: 0.14, step: 0.001 })]
    pub peduncle_length: f32,
    /// Diameter of the peduncle, in meters.
    #[reflect(@Slider { min: 0.004, max: 0.012, step: 0.0005 })]
    pub peduncle_diameter: f32,
    /// Angle between the stem and the peduncle where it leaves, in degrees:
    /// 90 leaves level, less leaves rising.
    #[reflect(@Slider { min: 5.0, max: 139.0, step: 1.0 })]
    pub peduncle_angle: f32,
    /// Chance that a fruit is missing from its pedicel.
    #[reflect(@Slider { min: 0.0, max: 0.3, step: 0.01 })]
    pub missing_fruit: f32,
    /// How many distinct truss meshes the scene may hold. A budget, not a
    /// count: the trusses are clustered and this is how many representatives
    /// the clustering may keep.
    #[reflect(@Slider { min: 1.0, max: 64.0, step: 1.0 }, @Label("Truss variations"))]
    pub variations: u32,
}

impl Default for TrussParams {
    fn default() -> Self {
        Self {
            fruits: 6,
            fruit_diameter: 0.06,
            branched: false,
            peduncle_length: 0.079,
            peduncle_diameter: 0.007,
            peduncle_angle: 70.0,
            missing_fruit: 0.05,
            variations: 16,
        }
    }
}

pub fn plugin(app: &mut App) {
    // `SceneParams` is not gated on although the configs read the seed:
    // `stem::author` gates on it and hangs fresh trusses when it moves.
    app.init_resource::<TrussParams>().add_systems(
        PreUpdate,
        (
            reauthor.run_if(resource_changed::<TrussParams>),
            build.run_if(
                configs_changed::<TrussConfig>
                    // `variations` and `missing_fruit` reach no config.
                    .or_eager(resource_changed::<TrussParams>),
            ),
        )
            .chain()
            .in_set(Grow::Trusses),
    );
}

/// Re-applies the params to every truss already hanging, in place, from the
/// same draws: they are keyed on its `Order`.
fn reauthor(
    params: Res<TrussParams>,
    scene: Res<SceneParams>,
    mut trusses: Query<(&Order, &mut TrussConfig)>,
) {
    for (order, mut config) in &mut trusses {
        let next = TrussConfig::new(&params, scene.seed, order.0, config.node);
        config.set_if_neq(next);
    }
}

/// The yaw of each rachis off where a simple one would hang: one, or a
/// branched truss's two.
fn arms(config: &TrussConfig) -> &'static [f32] {
    if config.branched {
        &[BRANCH, -BRANCH]
    } else {
        &[0.0]
    }
}

/// One internode of a rachis: the node it leaves, which way it runs, and the
/// side the node's pedicel leaves to.
#[derive(Clone, Copy, Debug)]
struct Internode {
    from: Vec3,
    along: Vec3,
    side: Vec3,
}

/// `by` of the way either side of one.
fn jitter(rng: &mut Rng, by: f64) -> f32 {
    rng.range(1.0 - by, 1.0 + by) as f32
}

/// A rachis from `base` hanging `mean`, node by node: `nodes` of them a
/// `pitch` apart, the pedicel at each leaving to alternate sides, `first` the
/// first. At every node the rachis hangs [`SAG`] closer to straight down, and
/// each internode runs half of [`KINK`] off that, away from its node's
/// pedicel, so the rachis turns the whole kink at the node after.
///
/// Draws, per node: how far the sag varies, then how far the kink does.
fn rachis(
    base: Vec3,
    mut mean: Vec3,
    pitch: f32,
    nodes: usize,
    first: f32,
    rng: &mut Rng,
) -> Vec<Internode> {
    // Across the rachis, level. The mean only ever pitches about it, so it
    // holds down the whole rachis.
    let side = Vec3::Z.cross(mean).try_normalize().unwrap_or(Vec3::Y);
    let mut from = base;
    (0..nodes)
        .map(|i| {
            let upright = (-mean.z).clamp(-1.0, 1.0).acos();
            let sag = (SAG * jitter(rng, NODE_JITTER)).min(upright);
            mean = Quat::from_axis_angle(side, sag) * mean;
            let flip = if i % 2 == 0 { first } else { -first };
            let (sin, cos) = (KINK * jitter(rng, NODE_JITTER) / 2.0).sin_cos();
            let run = Internode {
                from,
                along: mean * cos - side * (flip * sin),
                side: side * flip,
            };
            from += run.along * pitch;
            run
        })
        .collect()
}

/// A truss's main axis by arc length: out of the stem's axis along the
/// peduncle, round its bend, and down the first rachis to its tip. The
/// peduncle and the first rachis are drawn along it, and a bent truss's rod is
/// cut from it.
struct Axis {
    /// Off vertical where it leaves the stem, and how much further the bend
    /// turns it, in radians.
    angle: f32,
    turn: f32,
    /// The straight run out of the stem, and the bend's radius, in meters.
    straight: f32,
    bend: f32,
    /// The first rachis's internodes, from the bend's end to the last node,
    /// how far apart its nodes are, and how far past the last the tip runs.
    rachis: Vec<Internode>,
    pitch: f32,
    tip: f32,
    /// The peduncle's radius, which the rachis tapers from.
    radius: f32,
}

impl Axis {
    /// Draws the first rachis's kinks.
    fn new(config: &TrussConfig, rng: &mut Rng) -> Self {
        let angle = config.peduncle_angle;
        let turn = (PI - RACHIS_TILT - angle).max(MIN_TURN);
        let arms = arms(config);
        let per_arm = (config.fruits as usize).div_ceil(arms.len());
        let pitch = config.fruit_diameter * FRUIT_PITCH;
        let mut axis = Self {
            angle,
            turn,
            straight: config.peduncle_length * STRAIGHT,
            bend: config.peduncle_length * (1.0 - STRAIGHT) / turn,
            rachis: Vec::new(),
            pitch,
            tip: RACHIS_TIP * pitch,
            radius: config.peduncle_radius,
        };
        let hang = Quat::from_rotation_z(arms[0]) * heading(angle + turn);
        axis.rachis = rachis(axis.at(axis.fork()), hang, pitch, per_arm, 1.0, rng);
        axis
    }

    /// Where the bend ends and the rachis starts, at the first fruit; where a
    /// branched truss forks.
    fn fork(&self) -> f32 {
        self.straight + self.bend * self.turn
    }

    /// The first rachis's length, from the fork to its tip.
    fn reach(&self) -> f32 {
        (self.rachis.len() - 1) as f32 * self.pitch + self.tip
    }

    fn length(&self) -> f32 {
        self.fork() + self.reach()
    }

    /// The point `s` along.
    fn at(&self, s: f32) -> Vec3 {
        if s <= self.straight {
            return heading(self.angle) * s;
        }
        let fork = self.fork();
        if s < fork || self.rachis.is_empty() {
            // Round a circle from where the straight run ends.
            let a = self.angle + ((s - self.straight) / self.bend).min(self.turn);
            return heading(self.angle) * self.straight
                + self.bend
                    * Vec3::new(self.angle.cos() - a.cos(), 0.0, a.sin() - self.angle.sin());
        }
        // Down the internode `s` falls in; the last runs on to the tip.
        let i = (((s - fork) / self.pitch) as usize).min(self.rachis.len() - 1);
        let run = &self.rachis[i];
        run.from + run.along * (s - fork - i as f32 * self.pitch)
    }

    /// The radius `s` along: the peduncle's, tapering down the rachis to
    /// [`RACHIS_TAPER`] of it at the tip.
    fn radius_at(&self, s: f32) -> f32 {
        let down = ((s - self.fork()) / self.reach()).clamp(0.0, 1.0);
        self.radius * (1.0 - (1.0 - RACHIS_TAPER) * down)
    }
}

/// A bent truss's rod: what its curve is authored from and what rides each
/// of its segments. `M` is a mesh while it is built and the part it was
/// registered as once it is.
struct Rod<M> {
    /// Its points, held end first, how far along the axis each sits, and the
    /// diameter at each.
    points: Vec<[f32; 3]>,
    stations: Vec<f32>,
    widths: Vec<f32>,
    /// Each segment's rest frame, its tube, and the pedicels it carries, in
    /// that frame: on a branched truss's fork segment, the second rachis too.
    frames: Vec<Transform>,
    tubes: Vec<M>,
    pedicels: Vec<Option<M>>,
    /// Where along the axis each fruit's pedicel leaves it, from the truss's
    /// base to its tip, and where the `Bend` sits.
    holds: Vec<f32>,
    bend: f32,
}

impl Rod<Mesh> {
    /// The rod cut from `axis`, carrying each of `riders` on the segment at
    /// its station. Its points are evenly spaced past the held segment.
    fn new(axis: &Axis, riders: &[(f32, Strand)], holds: Vec<f32>, bend: f32) -> Result<Self> {
        let length = axis.length();
        let held = HELD.min(length / 2.0);
        let count = ((length - held) / SEGMENT).round().max(2.0) as usize;
        let stations: Vec<f32> = once(0.0)
            .chain((0..=count).map(|k| held + (length - held) * k as f32 / count as f32))
            .collect();
        let points: Vec<[f32; 3]> = stations.iter().map(|s| axis.at(*s).to_array()).collect();
        let widths: Vec<f32> = stations.iter().map(|s| 2.0 * axis.radius_at(*s)).collect();
        let frames = segment_frames(&points);
        let mut carried = vec![Vec::new(); frames.len()];
        for (at, rider) in riders {
            let k = segment_of(&stations, *at);
            let into = rebase(&frames[k], &Transform::IDENTITY);
            carried[k].push(strand_mesh(rider)?.transformed(&into));
        }
        Ok(Self {
            tubes: segment_tubes(&points, &widths, SIDES)
                .iter()
                .map(MeshData::to_mesh)
                .collect(),
            pedicels: carried
                .iter()
                .map(|meshes| (!meshes.is_empty()).then(|| merge_meshes(meshes).to_mesh()))
                .collect(),
            points,
            stations,
            widths,
            frames,
            holds,
            bend,
        })
    }
}

/// One representative truss: what its instances are drawn with, and the
/// frames and fruit they hang off it.
struct Built<M> {
    /// The whole truss as one mesh, for one that stays rigid, and its rod,
    /// for one that bends; each only if some truss that drew it needs it.
    mesh: Option<M>,
    rod: Option<Rod<M>>,
    /// The peduncle's direction leaving the stem, which `Cut` faces.
    leaves: Vec3,
    bend: Transform,
    /// Every pedicel's tip, from the truss's base to its tip.
    tips: Vec<Vec3>,
}

/// The direction `angle` off +Z toward +X.
fn heading(angle: f32) -> Vec3 {
    Vec3::new(angle.sin(), 0.0, angle.cos())
}

/// A strand through `points`, swollen at `bulges` and collared at `collars`,
/// meshed to `tolerance`.
fn strand(
    points: &[Vec3],
    radii: &[f32],
    tolerance: f64,
    bulges: Vec<Bulge>,
    collars: Vec<Collar>,
) -> Strand {
    let thickest = radii.iter().copied().fold(0.0, f32::max) as f64;
    let sides = sides_within(thickest, tolerance).max(MIN_SIDES);
    let points = points
        .iter()
        .map(|p| Point3::new(p.x as f64, p.y as f64, p.z as f64))
        .collect();
    let radii = radii.iter().map(|r| *r as f64).collect();
    Strand {
        bulges,
        collars,
        tolerance: Some(tolerance),
        // Rings per meter go unread: the tolerance places the rings.
        ..Strand::new(points, radii, sides, 0.0, Bark::none())
    }
}

/// A stalk's flare where it leaves something `around` thick.
fn flare(around: f32) -> Bulge {
    Bulge {
        at: 0.0,
        width: FLARE.1 * around as f64,
        height: FLARE.0,
    }
}

/// Builds one truss in its own frame, its stems meshed to `tolerance`: as one
/// mesh if `whole`, and as a rod if `bent`.
///
/// Draws, off the config's own stream: the first rachis's kinks, then a
/// branched truss's second rachis's.
fn build_truss(
    config: &TrussConfig,
    tolerance: f64,
    whole: bool,
    bent: bool,
) -> Result<Built<Mesh>> {
    let mut rng = Rng::new(config.seed);
    let axis = Axis::new(config, &mut rng);
    let (straight, fork) = (axis.straight, axis.fork());
    // Out of the stem along `angle`, then round the bend until it heads `turn`
    // further over.
    let mut peduncle = vec![axis.at(0.0), axis.at(straight / 2.0), axis.at(straight)];
    peduncle.extend(
        (1..=BEND_POINTS)
            .map(|i| axis.at(straight + (fork - straight) * i as f32 / BEND_POINTS as f32)),
    );
    let (base, hang) = (axis.at(fork), heading(axis.angle + axis.turn));

    let arms = arms(config);
    let (pitch, fruit) = (axis.pitch, config.fruit_diameter / 2.0);
    let (r, thin) = (
        config.peduncle_radius,
        pedicel_radius(config.fruit_diameter),
    );

    // The first rachis carries on from the peduncle; everything else hangs off
    // the axis they make, where it leaves it.
    let mut main = None;
    let mut riders = Vec::new();
    let mut tips = vec![Vec3::ZERO; config.fruits as usize];
    let mut holds = vec![fork; config.fruits as usize];
    for (arm, yaw) in arms.iter().enumerate() {
        // Fruit alternate between the arms, then between the sides of each.
        let fruits: Vec<usize> = (arm..config.fruits as usize).step_by(arms.len()).collect();
        let runs = if arm == 0 {
            axis.rachis.clone()
        } else {
            let hang = Quat::from_rotation_z(*yaw) * hang;
            rachis(base, hang, pitch, fruits.len(), -1.0, &mut rng)
        };
        // A second rachis starts a little back inside the first. Node by node
        // to the tip; the first node is the point before it, which the strand
        // merges.
        let mut points = if arm == 0 {
            peduncle.clone()
        } else {
            vec![base - hang * pitch * 0.5, base]
        };
        let mut radii = vec![r; points.len()];
        let nodes = points.len();
        for (i, run) in runs.iter().enumerate() {
            points.push(run.from);
            radii.push(axis.radius_at(fork + i as f32 * pitch));
        }
        if let Some(last) = runs.last() {
            points.push(last.from + last.along * axis.tip);
            radii.push(axis.radius_at(axis.length()));
        }
        // How far along its points each is.
        let along: Vec<f32> = once(0.0)
            .chain(points.windows(2).scan(0.0, |length, w| {
                *length += w[0].distance(w[1]);
                Some(*length)
            }))
            .collect();

        // The peduncle flares out of the stem; the rachis swells at every
        // node, round its pedicel, which flares out of it.
        let mut bulges = Vec::new();
        if arm == 0 {
            bulges.push(flare(config.node.stem_radius));
        }
        let (mut collars, mut pedicels) = (Vec::new(), Vec::new());
        for (i, j) in fruits.into_iter().enumerate() {
            let run = runs[i];
            let tip = run.from + run.side * (PEDICEL.x * fruit) - Vec3::Z * (PEDICEL.y * fruit);
            // Up and over, then straight down onto the fruit.
            let mid = run.from.lerp(tip, 0.5) + Vec3::Z * (ARCH * fruit);
            let over = tip + Vec3::Z * (DROP * fruit);
            // A second rachis's pedicels ride with it, from the fork.
            if arm == 0 {
                holds[j] = fork + i as f32 * pitch;
            }
            let (at, around) = (along[nodes + i] as f64, radii[nodes + i]);
            bulges.push(Bulge {
                at,
                width: KNOT.1 * around as f64,
                height: KNOT.0,
            });
            collars.push(Collar {
                at,
                toward: vector(mid - run.from),
                radius: thin as f64,
            });
            let pedicel = [run.from, mid, over, tip];
            let flared = vec![flare(around)];
            pedicels.push((
                holds[j],
                strand(&pedicel, &[thin; 4], tolerance, flared, Vec::new()),
            ));
            tips[j] = tip;
        }
        let stem = strand(&points, &radii, tolerance, bulges, collars);
        if arm == 0 {
            main = Some(stem);
        } else {
            riders.push((fork, stem));
        }
        riders.extend(pedicels);
    }

    let main = main.expect("a truss has a first rachis");
    let mesh = if whole {
        let meshes: Vec<MeshData> = once(&main)
            .chain(riders.iter().map(|(_, rider)| rider))
            .map(strand_mesh)
            .collect::<Result<_, _>>()?;
        Some(merge_meshes(&meshes).to_mesh())
    } else {
        None
    };
    let middle = straight + (fork - straight) / 2.0;
    let rod = if bent {
        Some(Rod::new(&axis, &riders, holds, middle)?)
    } else {
        None
    };
    let turned = axis.angle + axis.turn / 2.0;
    Ok(Built {
        mesh,
        rod,
        leaves: heading(axis.angle),
        bend: Transform::from_translation(axis.at(middle))
            .with_rotation(Quat::from_rotation_y(turned - FRAC_PI_2)),
        tips,
    })
}

/// The rotation turning +X onto `direction`, which lies in the XZ plane.
fn facing(direction: Vec3) -> Quat {
    Quat::from_rotation_y(-direction.z.atan2(direction.x))
}

/// `fruit`, a station and a mass for each, as loads on the points of a rod
/// whose points sit at `stations`: each fruit's mass split between the two
/// points either side of where its pedicel leaves the rod, the nearer taking
/// more.
fn loads(stations: &[f32], fruit: impl Iterator<Item = (f32, f32)>) -> Vec<f32> {
    let mut loads = vec![0.0; stations.len()];
    for (at, mass) in fruit {
        let k = segment_of(stations, at);
        let t = ((at - stations[k]) / (stations[k + 1] - stations[k])).clamp(0.0, 1.0);
        loads[k] += mass * (1.0 - t);
        loads[k + 1] += mass * t;
    }
    loads
}

/// Builds the meshes for every distinct truss, and hangs its frames and its
/// fruit off every truss: on the truss itself, or on a bent one's rod, each
/// on the segment that carries it.
///
/// Each truss's fruit draw from a stream keyed on its [`Order`]: four draws
/// per pedicel, whether its fruit is missing, how big it is, how it is
/// turned about its axis and how far it hangs off vertical, made for every
/// pedicel so a higher `missing_fruit` drops fruit without resizing the rest.
/// A fruit's own shape is drawn off a stream keyed on its truss and its
/// pedicel.
pub(crate) fn build(
    mut commands: Commands,
    mut library: Library,
    scene: Res<SceneParams>,
    params: Res<TrussParams>,
    stem: Res<StemParams>,
    fruit_params: Res<FruitParams>,
    trusses: Query<(Entity, &Order, &TrussConfig)>,
) -> Result<()> {
    for prefix in [PART, TUBE, PEDICELS] {
        library.clear(prefix);
    }

    let mut hung: Vec<(Order, Entity, TrussConfig)> = trusses
        .iter()
        .map(|(entity, order, config)| (*order, entity, *config))
        .collect();
    hung.sort_by_key(|(order, ..)| *order);

    let configs: Vec<TrussConfig> = hung.iter().map(|(_, _, config)| *config).collect();
    let book = farthest_first(
        &configs,
        params.variations.max(1) as usize,
        0.0,
        &TrussMetric,
    );
    // Which ways each representative is drawn: whole for the trusses that drew
    // it and stay rigid, as a rod for the ones that bend.
    let mut ways = vec![(false, false); book.len()];
    for (config, drew) in configs.iter().zip(&book.assignment) {
        let way = &mut ways[*drew as usize];
        if config.node.flexible {
            way.1 = true;
        } else {
            way.0 = true;
        }
    }
    let tolerance = TOLERANCE / stem.detail.max(MIN_DETAIL) as f64;
    let shapes = par_map(&book.representatives, |index, config| {
        build_truss(config, tolerance, ways[index].0, ways[index].1)
    });

    // Tubes and pedicels are numbered across the whole layer, because how many
    // a representative needs depends on how long its rod came out.
    let (mut tubes, mut pedicels) = (0, 0);
    let mut built = Vec::with_capacity(book.len());
    for (index, shape) in shapes.into_iter().enumerate() {
        let shape = shape?;
        let surface = RESPONSE.surface(shade(
            srgb(COLOR),
            &mut Rng::new(scene.seed ^ COLOR_STREAM ^ salt(index as u64)),
        ));
        let mut register = |prefix: &str, count: &mut usize, mesh: Mesh| {
            *count += 1;
            library.part(prefix, *count - 1, mesh, surface)
        };
        let rod = shape.rod.map(|rod| Rod {
            tubes: rod
                .tubes
                .into_iter()
                .map(|tube| register(TUBE, &mut tubes, tube))
                .collect(),
            pedicels: rod
                .pedicels
                .into_iter()
                .map(|mesh| mesh.map(|mesh| register(PEDICELS, &mut pedicels, mesh)))
                .collect(),
            points: rod.points,
            stations: rod.stations,
            widths: rod.widths,
            frames: rod.frames,
            holds: rod.holds,
            bend: rod.bend,
        });
        let mesh = shape
            .mesh
            .map(|mesh| library.part(PART, index, mesh, surface));
        built.push((
            Built {
                mesh,
                rod,
                leaves: shape.leaves,
                bend: shape.bend,
                tips: shape.tips,
            },
            surface,
        ));
    }

    let mut fruit_order = 0u64;
    for ((order, entity, config), drew) in hung.iter().zip(&book.assignment) {
        let (built, surface) = &built[*drew as usize];

        let mut rng = Rng::new(scene.seed ^ FRUIT_STREAM ^ salt(order.0));
        let last = (built.tips.len() - 1).max(1) as f32;
        let mut fruit = Vec::new();
        for (j, tip) in built.tips.iter().enumerate() {
            let missing = rng.unit() < params.missing_fruit as f64;
            let size = rng.range(1.0 - FRUIT_JITTER, 1.0 + FRUIT_JITTER) as f32;
            let spin = rng.unit() as f32 * TAU;
            let tilt = rng.range(0.0, TILT) as f32;
            if missing {
                continue;
            }
            let along = j as f32 / last;
            let config = FruitConfig::new(
                &fruit_params,
                // A truss holds fewer than 256 fruit.
                scene.seed ^ FRUIT_SHAPE_STREAM ^ salt(order.0 << 8 | j as u64),
                config.fruit_diameter * (1.0 - TIP_SHRINK * along) * size,
                Hung {
                    rank: config.node.rank,
                    along,
                    growth: config.node.growth,
                },
            );
            // Tilted first, so the spin turns the tilt any way round.
            let placement = Transform::from_translation(*tip)
                .with_rotation(Quat::from_rotation_z(spin) * Quat::from_rotation_x(tilt));
            fruit.push((j, placement, config));
        }

        let mut truss = commands.entity(*entity);
        // The layer owns everything below a truss, and a rebuild may hang a
        // different number of fruit than the last one did.
        truss.despawn_children();
        let rod = config.node.flexible.then(|| {
            built
                .rod
                .as_ref()
                .expect("built as a rod for a truss that bends")
        });
        let segments = match rod {
            Some(rod) => {
                let loads = loads(
                    &rod.stations,
                    fruit
                        .iter()
                        .map(|(j, _, fruit)| (rod.holds[*j], fruit.mass())),
                );
                let curve = cable(rod.points.clone(), rod.widths.clone(), loads, *surface);
                spawn_rod(&mut truss, curve, &rod.frames, &rod.tubes)
            }
            None => {
                let mesh = built
                    .mesh
                    .clone()
                    .expect("built whole for a truss that stays rigid");
                truss.with_child((Name::new("Peduncle"), mesh));
                Vec::new()
            }
        };
        truss.with_child((
            Name::new("Cut"),
            Transform::from_translation(built.leaves * config.node.stem_radius)
                .with_rotation(facing(built.leaves)),
            Visibility::default(),
        ));

        // What hangs `at` along the axis, and where it is placed under what
        // carries it: the truss, or the rod segment there in its own frame.
        let host = |at: f32, placement: &Transform| match rod {
            Some(rod) => {
                let k = segment_of(&rod.stations, at);
                (segments[k], rebase(&rod.frames[k], placement))
            }
            None => (*entity, *placement),
        };
        if let Some(rod) = rod {
            for (segment, mesh) in segments.iter().zip(&rod.pedicels) {
                if let Some(mesh) = mesh {
                    commands
                        .entity(*segment)
                        .with_child((Name::new(PEDICELS), mesh.clone()));
                }
            }
        }
        let (on, bend) = host(rod.map_or(0.0, |rod| rod.bend), &built.bend);
        commands
            .entity(on)
            .with_child((Name::new("Bend"), bend, Visibility::default()));
        for (j, placement, fruit) in fruit {
            fruit_order += 1;
            let (on, placement) = host(rod.map_or(0.0, |rod| rod.holds[j]), &placement);
            commands.entity(on).with_child((
                Name::new(format!("Fruit_{j:02}")),
                placement,
                Visibility::default(),
                fruit,
                Order(fruit_order),
            ));
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeMap;

    use misina_lab::scene::{CABLE, Cable, STEM, segment_name};
    use misina_lab::testing::{named_children, prim_path};

    use super::*;
    use crate::elements::{grown, in_scene, small};

    /// What hangs off each truss, its `Bend` and its fruit, keyed by the
    /// truss's path and its own name, and where it lands in the scene, however
    /// deep under the truss it hangs.
    fn riders(world: &mut World) -> BTreeMap<(String, String), Transform> {
        let named: Vec<(Entity, String)> = world
            .query::<(Entity, &Name)>()
            .iter(world)
            .map(|(entity, name)| (entity, name.to_string()))
            .filter(|(_, name)| name == "Bend" || name.starts_with("Fruit_"))
            .collect();
        let mut riders = BTreeMap::new();
        for (entity, name) in named {
            let mut truss = entity;
            while !world.entity(truss).contains::<TrussConfig>() {
                truss = world.entity(truss).get::<ChildOf>().unwrap().parent();
            }
            riders.insert((prim_path(world, truss), name), in_scene(world, entity));
        }
        riders
    }

    /// A truss whose node is below `flexible_below` is a rod, and the rest are
    /// meshes. The rod is held from the stem's axis to within a splinter past
    /// `Cut`, its moving segments are even and taper toward the tip, a tube
    /// rides each, and its fruit's weight hangs off its moving points. Whatever
    /// hangs off a truss lands where it does on a rigid one: a simple truss
    /// and a branched one.
    #[test]
    fn a_truss_below_the_bend_height_is_a_rod_its_fruit_ride() {
        const BELOW: f32 = 1.8;
        for branched in [false, true] {
            let mut params = small();
            params.truss.branched = branched;
            let rigid = riders(grown(params.clone()).world_mut());
            params.stem.flexible_below = BELOW;
            let mut app = grown(params);
            let world = app.world_mut();

            let bent = riders(world);
            assert_eq!(bent.len(), rigid.len());
            for ((key, at), (_, was)) in bent.iter().zip(&rigid) {
                // By the axes it turns, as `angle_between` is all rounding
                // this close to none.
                let turned = [Vec3::X, Vec3::Z]
                    .map(|axis| (at.rotation * axis).distance(was.rotation * axis))
                    .into_iter()
                    .fold(0.0, f32::max);
                assert!(
                    at.translation.distance(was.translation) < 1e-5 && turned < 1e-5,
                    "branched {branched}: {key:?} moved from {was:?} to {at:?}"
                );
            }

            let trusses: Vec<(Entity, TrussConfig)> = world
                .query::<(Entity, &TrussConfig)>()
                .iter(world)
                .map(|(entity, config)| (entity, *config))
                .collect();
            let mut rods = 0;
            for (truss, config) in &trusses {
                let path = prim_path(world, *truss);
                let node = in_scene(world, *truss).translation.z;
                assert_eq!(config.node.flexible, node < BELOW, "{path} at {node}");
                let children: BTreeMap<String, Entity> =
                    named_children(world, *truss).into_iter().collect();
                assert!(children.contains_key("Cut"), "{path}");
                assert_eq!(
                    children.contains_key("Peduncle"),
                    !config.node.flexible,
                    "{path}: drawn as a mesh"
                );
                if !config.node.flexible {
                    continue;
                }
                rods += 1;

                let rod = world
                    .entity(children[CABLE])
                    .get::<Cable>()
                    .unwrap()
                    .0
                    .clone();
                let points: Vec<Vec3> = rod.points.iter().map(|p| Vec3::from(*p)).collect();
                assert_eq!(points[0], Vec3::ZERO, "{path}: held off the stem's axis");
                let past = points[1].length() - config.node.stem_radius;
                assert!(
                    (0.0..0.01).contains(&past),
                    "{path}: held {past} m past `Cut`"
                );
                // Even along the axis, so as long as each other where it runs
                // straight; a chord across the bend is shorter than its arc.
                let spans: Vec<f32> = points.windows(2).map(|w| w[0].distance(w[1])).collect();
                let even = spans[1..].iter().copied().fold(0.0, f32::max);
                assert!(
                    spans[1..].iter().all(|span| *span > 0.75 * even),
                    "{path}: spans {spans:?}"
                );
                assert!(
                    rod.widths.windows(2).all(|w| w[1] <= w[0]),
                    "{path}: widths {:?}",
                    rod.widths
                );
                for index in 0..spans.len() {
                    let segment = children[&segment_name(index)];
                    let carried = named_children(world, segment);
                    assert!(
                        carried.iter().any(|(name, _)| name == STEM),
                        "{path}: segment {index}"
                    );
                }

                // The weight of the fruit under this truss, wherever they hang.
                let mut weight = 0.0;
                let mut below = vec![*truss];
                while let Some(at) = below.pop() {
                    if let Some(fruit) = world.entity(at).get::<FruitConfig>() {
                        weight += fruit.mass();
                    }
                    below.extend(world.entity(at).get::<Children>().into_iter().flatten());
                }
                let loads = &rod.loads;
                assert_eq!(loads.len(), points.len(), "{path}");
                assert!(
                    loads[0] == 0.0 && loads[1] == 0.0,
                    "{path}: a load on the held segment"
                );
                let carried: f32 = loads.iter().sum();
                assert!(
                    weight > 0.0 && (carried / weight - 1.0).abs() < 1e-4,
                    "{path}: carries {carried} kg of {weight}"
                );
            }
            assert!(
                rods > 0 && rods < trusses.len(),
                "branched {branched}: {rods} of {} bent",
                trusses.len()
            );
        }
    }

    /// Down a rachis every internode runs off its node's pedicel, the next
    /// one back the other way, a pitch apart; and the rachis hangs closer to
    /// straight down as it goes, never past it, nor ever up.
    #[test]
    fn a_rachis_zigzags_away_from_its_pedicels_and_hangs_closer_to_vertical() {
        let mut rng = Rng::new(7);
        let runs = rachis(
            Vec3::ZERO,
            heading(PI - RACHIS_TILT),
            0.033,
            12,
            1.0,
            &mut rng,
        );
        assert_eq!(runs.len(), 12);
        for (i, run) in runs.iter().enumerate() {
            assert!(
                (run.along.length() - 1.0).abs() < 1e-5 && run.along.z < 0.0,
                "{i}: {run:?}"
            );
            assert!(
                run.along.dot(run.side) < -0.04,
                "{i}: toward its pedicel: {run:?}"
            );
            assert!(
                run.side.dot(Vec3::Y) * if i % 2 == 0 { 1.0 } else { -1.0 } > 0.0,
                "{i}"
            );
        }
        for pair in runs.windows(2) {
            assert!((pair[1].from.distance(pair[0].from) - 0.033).abs() < 1e-5);
        }
        // Off straight down: by the end, only its own half kink.
        let tilt = |run: &Internode| (-run.along.z).acos();
        let (first, last) = (tilt(&runs[0]), tilt(&runs[11]));
        let kink = KINK * (1.0 + NODE_JITTER as f32) / 2.0;
        assert!(
            last < first - 0.2 && last <= kink + 1e-5,
            "{first} to {last}"
        );
    }

    /// A fruit's mass goes to the two points either side of where it hangs,
    /// the nearer taking more, and one past the last point to the last.
    #[test]
    fn a_fruits_mass_is_shared_by_the_points_either_side() {
        let shared = loads(
            &[0.0, 0.01, 0.03, 0.05],
            [(0.025, 0.2), (0.06, 0.1)].into_iter(),
        );
        for (got, want) in shared.iter().zip([0.0, 0.05, 0.15, 0.1]) {
            assert!((got - want).abs() < 1e-6, "{shared:?}");
        }
    }
}

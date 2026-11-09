//! Truss — a truss of tomatoes: the peduncle off the stem, the rachis the
//! fruit hang under, and a pedicel to every fruit.
//!
//! A truss is built in its own frame, on the stem's axis at the node it grew
//! from, facing +X with +Z up. It grows carrying nothing: the peduncle
//! leaves the stem `peduncle_angle` off vertical and bends round to level
//! into the rachis, which runs on out from the stem, a chain of internodes,
//! one to a fruit. Then its fruit's weight bends it down against its
//! `stiffness` ([`sag`]), each joint by the weight past it, the rachis more
//! than the peduncle, being thinner, so it hangs nearer straight down node
//! by node. A truss in flower stands out from the stem, and one sags as its
//! fruit swell until it hangs from its bend. A pedicel leaves at every node but
//! the last, off the rachis's underside, the inside of its bend, down one
//! flank and then the other, `pedicel_angle` below its sides; the rachis
//! kinks away from it there, so it zigzags. The last fruit has no pedicel of
//! its own: the rachis's last internode is its pedicel out to the knuckle,
//! at the last node, one stalk with the rachis.
//! Every pedicel heads out the way it leaves to its knuckle, where its fruit
//! bends it down, and runs on into the fruit's calyx. It leans the fruit out
//! the way it left, so the fruit hangs with its top toward the rachis. Each
//! pedicel draws its own reach, swing and lean. That is where each fruit
//! would hang alone; a truss's fruit crowd each other, so they are then
//! packed ([`pack`]) until none lies in another, in the stems or on another's
//! pedicel, each pedicel swinging round its node and bending at its knuckle,
//! and each fruit turned to face its knuckle. The last fruit, held at the
//! rachis's end, is pushed out past it and faces back along it:
//!
//! ```text
//!         Bend
//!     ╭────╮
//!  ───╯     ╲         a pedicel off the rachis's underside at every node,
//!  Cut    ╭──╲        down one flank and then the other, bent down at its
//!        ╱●   ╲       knuckle and on into its fruit's calyx
//!           ╭──╲
//!          ╱●   ╲
//!                ╲    the rachis ends in the last fruit, its last
//!                 ╲   internode that fruit's pedicel
//!                 ●
//! ```
//!
//! Its stalks grow with its flowers, each pedicel and the internode of
//! rachis leading to it with the flower at its end: a bud's are short,
//! `stalks_in_flower` of their full length once it opens, and full length
//! [`STALK_GROWING`] weeks on, well before its fruit is full size. So a truss
//! in flower is a short one, its buds crowded at its tip, and its rachis
//! lengthens as they open. The peduncle and the rachis thicken as the first
//! fruit's stalks grow.
//!
//! A branched truss, the cherry kind, forks into two rachises at the first
//! fruit and shares its fruit between them.
//!
//! Peduncle, rachis and pedicels are one mesh, quantized like any organ, to a
//! budget of `variations`, a truss's sag and the size of its fruit among what
//! tells two apart. The representative a truss drew decides where its `Cut`
//! and `Bend` and its fruit go, how far it sags, and how its rachis kinks and
//! its pedicels lie; which fruit are missing, how big each one is and how it
//! is turned about its axis are drawn per truss. Each of a representative's
//! fruit is packed as one ball holding every ball it fills on any truss that
//! drew it, so none of those trusses' fruit lie in each other, and a truss
//! of smaller fruit hangs them a little apart.
//!
//! ```text
//! Truss_04            Xform, a TrussConfig
//!   Peduncle          -> parts/Truss_k
//!   Cut               Xform at the peduncle's base, +X along the peduncle
//!   Bend              Xform halfway round the peduncle's bend, +X along it
//!   Fruit_00 …        a FruitConfig, its calyx on a pedicel's end, +Z up it
//! ```
//!
//! `Cut` and `Bend` are where the two kinds of harvesting gripper take a
//! truss: one cuts flush at the stem, the other clamps the bend and cuts
//! beside it. Both are frames, since the approach matters as much as the
//! point.
//!
//! A truss whose node is below
//! [`StemParams::flexible_below`](super::stem::StemParams::flexible_below)
//! is a rod instead. Its main axis, the peduncle and the first rachis down to
//! its last fruit's calyx, is a
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
use misina_lab::geometry::bending::{rigidity, sag};
use misina_lab::geometry::mesh::{MeshData, merge_meshes};
use misina_lab::geometry::packing::{Bob, pack};
use misina_lab::geometry::par_map;
use misina_lab::geometry::strand::{Bark, Bulge, Collar, Strand, sides_within, strand_mesh};
use misina_lab::palette::{COLOR_STREAM, Response, shade, srgb};
use misina_lab::params::{Label, Slider};
use misina_lab::quantize::{Metric, farthest_first};
use misina_lab::rng::{Rng, salt};
use misina_lab::scene::{
    Library, Order, cable, configs_changed, rebase, segment_frames, segment_of, spawn_rod,
};
use nalgebra::Point3;

use super::fruit::{FruitConfig, FruitParams, Hung};
use super::stem::{MIN_DETAIL, StemParams};
use super::{Grow, SceneParams, smooth, vector};

/// The mesh-library prefix the trusses register under.
pub const PART: &str = "Truss";

/// The mesh-library prefixes a bent truss's parts register under: the tube
/// on each segment of its rod, and the pedicels a segment carries. Layers of
/// their own, because a representative has one of each per segment where it
/// has one mesh, and [`Library::clear`] drops a whole prefix at a time.
const TUBE: &str = "TrussTube";
const PEDICELS: &str = "Pedicels";

/// The held segment of a bent truss's rod, from the stem's axis out along the
/// peduncle, in meters: past the surface of the thickest stem a
/// `head_diameter` of up to 14 mm grows, and short enough that a cut flush at
/// `Cut` lands within a splinter of the first joint (`SPLINTER` in
/// `misina_lab/isaaclab/cutting.py`) on the thinnest a 6 mm head grows from
/// `HEAD_REACH` down. Nearer the head of one under 8 mm it lands a millimeter
/// or so further out, and is made at that joint all the same.
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

/// How far the rachis kinks at a node, in radians, away from the pedicel
/// that leaves there. Pedicels leave to alternate sides, so it zigzags.
const KINK: f32 = 0.6;

/// How far a node's kink varies about that, as a fraction.
const NODE_JITTER: f64 = 0.5;

/// How far along a pedicel its knuckle is, from the rachis, as a share of
/// its length: the joint is in its distal third.
const KNUCKLE_AT: f32 = 0.65;

/// How far the stem a truss hangs from runs either way of its node, in
/// meters, as its fruit are packed clear of it: past where any truss's fruit
/// hang.
const STEM_SPAN: f32 = 2.0;

/// How far a pedicel leans its fruit out from under its knuckle, the way it
/// left, as a slope per unit of the pedicel's heading off vertical: its top
/// then faces back toward the rachis.
const LEAN: f32 = 0.35;

/// How far a pedicel's reach and lean vary, as fractions, and how far it
/// swings off its flank toward either end of the rachis, in radians, at
/// most.
const REACH_JITTER: f64 = 0.15;
const LEAN_JITTER: f64 = 0.3;
const SWING: f64 = 0.1;

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
/// `grown` of the way there.
fn pedicel_radius(diameter: f32, grown: f32) -> f32 {
    (PEDICEL_RADIUS * diameter * grown).max(MIN_PEDICEL)
}

/// Weeks over which a bud's stalks grow from nothing to their length in
/// flower, the youngest bud on a truss just showing at the head opening
/// about one and a half weeks later; and the weeks after its flower opens
/// over which they grow on to full length, as a pedicel does mostly over the
/// first two weeks (est.).
const BUDDING: f32 = 2.0;
const STALK_GROWING: f32 = 2.5;

/// How far a stalk has grown, as a share of its full length, `opened` weeks
/// after the flower at its end opened, `flowering` of it when it did: on
/// from nothing in bud, then eased in and out to full length.
fn grown(opened: f32, flowering: f32) -> f32 {
    let share = if opened < 0.0 {
        flowering * (1.0 + opened / BUDDING)
    } else {
        flowering + (1.0 - flowering) * smooth(opened / STALK_GROWING)
    };
    share.max(MIN_STALK)
}

/// How far each of a truss's stalks has grown, from its base to its tip:
/// the pedicel to each fruit, and the internode of rachis leading to it.
fn stalks(config: &TrussConfig, params: &TrussParams) -> Vec<f32> {
    let flowering = params.stalks_in_flower.clamp(MIN_STALK, 1.0);
    let last = (config.fruits - 1).max(1) as f32;
    (0..config.fruits)
        .map(|j| {
            let hung = Hung {
                rank: config.node.rank,
                along: j as f32 / last,
                age: config.node.age,
            };
            grown(hung.opened(), flowering)
        })
        .collect()
}

/// Floors on what Python may set, so that every truss still builds; the
/// stiffness in MPa, a stalk's share of its full length, and a pedicel's and
/// an internode's full length.
const MIN_LENGTH: f32 = 0.01;
const MIN_RADIUS: f32 = 0.0005;
const MIN_DIAMETER: f32 = 0.005;
const ANGLE: (f32, f32) = (0.05, PI - MIN_TURN);
const MIN_STIFFNESS: f32 = 1.0;
const MIN_STALK: f32 = 0.05;
const MIN_STALK_LENGTH: f32 = 0.002;

/// The params' `stiffness`, in pascals.
fn modulus(params: &TrussParams) -> f32 {
    params.stiffness.max(MIN_STIFFNESS) * 1e6
}

/// The fewest vertices round any of a truss's stems.
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

/// How much a pedicel swells at its knuckle, the joint a jointed fruit
/// breaks off at, and where it widens into the calyx at its end, as fractions
/// of its radius, and over how many of its radii.
const KNUCKLE: (f64, f64) = (0.5, 1.0);
const RECEPTACLE: (f64, f64) = (1.0, 0.7);

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
    /// A pedicel's length under a grown fruit, and the rachis between two
    /// pedicels, in meters.
    pub pedicel: f32,
    pub internode: f32,
    /// The stream its rachis's kinks are drawn from. [`TrussMetric`] does not
    /// read it: two trusses near in shape share a mesh, kinked as the
    /// representative's were.
    pub seed: u64,
    /// What it takes from its stem, kept so the params can be re-applied to a
    /// truss already hanging.
    pub node: Node,
}

/// What a truss takes from the node of the stem it hangs on: placement and
/// ripening. [`TrussMetric`] reads none of it, only what its age and rank
/// make of the truss: how far its stalks have grown, how big its fruit are
/// and how far their weight sags it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Node {
    /// The stem's radius at the node; the `Cut` sits that far out along the
    /// peduncle.
    pub stem_radius: f32,
    /// How many weeks this one is behind its stem's ripening front, a truss
    /// a week, which its fruit's ripeness is read off.
    pub rank: f32,
    /// Weeks since it showed at the head, a truss a week: its flowers open
    /// and set, and its fruit grow, by it.
    pub age: f32,
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
            pedicel: params.pedicel_length.max(MIN_STALK_LENGTH),
            internode: params.rachis_internode.max(MIN_STALK_LENGTH),
            seed: seed ^ NODE_STREAM ^ salt(order),
            node,
        }
    }
}

/// A truss as it is clustered: its config, its axis, grown and bent, and
/// the mean radius of the balls its fruit fill, which [`TrussMetric`] reads,
/// and the mass of each of its fruit in kilograms, none where one is
/// missing, which a representative is built bent under.
#[derive(Clone, Debug)]
struct Laden {
    config: TrussConfig,
    loads: Vec<f32>,
    axis: Axis,
    size: f32,
}

/// Two trusses share a mesh when they are close in every dimension that
/// shows, weighted as roughly how far apart each moves a fruit.
struct TrussMetric;

impl Metric<Laden> for TrussMetric {
    fn distance(&self, a: &Laden, b: &Laden) -> f32 {
        let (x, y, size) = (&a.axis, &b.axis, a.size - b.size);
        let (a, b) = (&a.config, &b.config);
        // A forked truss and a simple one are different shapes, not near
        // sizes of one.
        if a.branched != b.branched {
            return 1.0;
        }
        let mean = |stalks: &[f32]| stalks.iter().sum::<f32>() / stalks.len() as f32;
        [
            x.droop.distance(y.droop),
            a.peduncle_length - b.peduncle_length,
            (x.radius - y.radius) * 8.0,
            // A radian swings the rachis's root by a peduncle's length.
            (a.peduncle_angle - b.peduncle_angle) * a.peduncle_length.max(b.peduncle_length),
            // A fruit more is a pedicel more, half a fruit off the last.
            (a.fruits as f32 - b.fruits as f32) * a.fruit_diameter.max(b.fruit_diameter) / 2.0,
            // The rachis's tip moves by how much longer it grew.
            x.reach() - y.reach(),
            // A pedicel's fruit moves by how much longer it grew.
            (mean(&x.stalks) - mean(&y.stalks)) * a.pedicel.max(b.pedicel),
            // Packed fruit move apart by as much as they grow across.
            size * 2.0,
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
/// high-wire house, and every truss draws its own about them; the rachis's
/// internode to what those trusses' measured lengths leave for it (est.),
/// and the pedicel to a genebank's measurements. A medium truss tomato is
/// about 6 fruit of 60 mm; a cherry truss about 15 of 25 mm, branched.
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
    /// How stiff the peduncle and rachis are, as a Young's modulus in MPa.
    /// A truss grows standing out from the stem, and its fruit's weight bends
    /// it down against this: one in flower barely bends, and one sags further
    /// as its fruit swell. Below green wood's, as a stalk under a load for
    /// weeks gives way to it more than it would at once.
    #[reflect(@Slider { min: 20.0, max: 1000.0, step: 10.0 })]
    pub stiffness: f32,
    /// Length of the rachis between two pedicels under grown fruit, in
    /// meters: short, a truss's fruit crowd into a bunch, and long, they hang
    /// apart down the rachis.
    #[reflect(@Slider { min: 0.005, max: 0.05, step: 0.001 })]
    pub rachis_internode: f32,
    /// Length of a pedicel under a grown fruit, from the rachis to the
    /// fruit's calyx, in meters. A small-fruited truss's are about as long as
    /// a large-fruited one's, so its fruit hang further out for their size.
    #[reflect(@Slider { min: 0.01, max: 0.06, step: 0.001 })]
    pub pedicel_length: f32,
    /// How far under the rachis its pedicels leave it, in degrees below its
    /// sides: 0 leaves them level out to the sides, and more hangs the fruit
    /// further under it. At 45 the two ranks, down alternate flanks, meet at
    /// a right angle, as they do on a tomato truss.
    #[reflect(@Slider { min: 0.0, max: 50.0, step: 1.0 })]
    pub pedicel_angle: f32,
    /// Whether a pedicel has a knuckle, the joint a fruit breaks off at, as
    /// most truss varieties do.
    #[reflect(@Label("Jointed pedicels"))]
    pub jointed: bool,
    /// How long a pedicel, and the internode of rachis leading to it, are
    /// when its flower opens, as a share of their length under the grown
    /// fruit: about a third on a large-fruited truss, whose stalks grow to
    /// about three times their length in flower, and nearer 1 on a
    /// small-fruited one, whose barely grow on.
    #[reflect(@Slider { min: 0.1, max: 1.0, step: 0.05 })]
    pub stalks_in_flower: f32,
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
            stiffness: 70.0,
            rachis_internode: 0.014,
            pedicel_length: 0.031,
            pedicel_angle: 45.0,
            jointed: true,
            stalks_in_flower: 0.35,
            missing_fruit: 0.05,
            variations: 24,
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
                    // `pedicel_angle`, `jointed`, `stalks_in_flower`,
                    // `stiffness`, `variations` and `missing_fruit` reach no
                    // config.
                    .or_eager(resource_changed::<TrussParams>)
                    // The fruit's params set their weight, which bends a
                    // truss, and their size, which its fruit are packed at.
                    .or_eager(resource_changed::<FruitParams>),
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

/// One internode of a rachis: the node it leaves, which way it runs, the
/// side the node's pedicel leaves to, and its back there, the outside of its
/// bend, square to both: facing up where the rachis runs out level, and out
/// from the stem where it hangs plumb.
#[derive(Clone, Copy, Debug)]
struct Internode {
    from: Vec3,
    along: Vec3,
    side: Vec3,
    up: Vec3,
}

/// `by` of the way either side of one.
fn jitter(rng: &mut Rng, by: f64) -> f32 {
    rng.range(1.0 - by, 1.0 + by) as f32
}

/// A rachis from `base` heading `mean`, as it grows before its fruit weigh
/// it down, node by node: one more than `steps`, how long each internode
/// is, the pedicel at each leaving to alternate sides, `first` the first.
/// Each internode runs half of [`KINK`] off `mean`, away from its node's
/// pedicel, so the rachis turns the whole kink at the node after.
///
/// Draws, per node: how far the kink varies.
fn rachis(base: Vec3, mean: Vec3, steps: &[f32], first: f32, rng: &mut Rng) -> Vec<Internode> {
    // Across the rachis, level.
    let side = Vec3::Z.cross(mean).try_normalize().unwrap_or(Vec3::Y);
    let mut from = base;
    (0..=steps.len())
        .map(|i| {
            let flip = if i % 2 == 0 { first } else { -first };
            let (sin, cos) = (KINK * jitter(rng, NODE_JITTER) / 2.0).sin_cos();
            let run = Internode {
                from,
                along: mean * cos - side * (flip * sin),
                side: side * flip,
                up: mean.cross(side),
            };
            from += run.along * steps.get(i).copied().unwrap_or(0.0);
            run
        })
        .collect()
}

/// `lead`, the stalk a rachis grows on from, and the rachis's `runs`, the
/// first starting where `lead` ends, bent under `loads` against `rigidity`,
/// each one to a point, the lead's and then the nodes: where the lead's
/// points settle, and the runs moved and turned with their nodes.
fn sag_rachis(
    lead: &[Vec3],
    runs: &[Internode],
    rigidity: &[f32],
    loads: &[f32],
) -> (Vec<Vec3>, Vec<Internode>) {
    let points: Vec<Vec3> = lead
        .iter()
        .copied()
        .chain(runs.iter().map(|run| run.from))
        .collect();
    let (points, turns) = sag(&points, rigidity, loads);
    let nodes = points[lead.len()..].iter().zip(&turns[lead.len()..]);
    let runs = runs
        .iter()
        .zip(nodes)
        .map(|(run, (from, turn))| Internode {
            from: *from,
            along: *turn * run.along,
            side: *turn * run.side,
            up: *turn * run.up,
        })
        .collect();
    (points[..lead.len()].to_vec(), runs)
}

/// A truss's main axis by arc length: out of the stem's axis along the
/// peduncle, round its bend, down the first rachis to its last node, the
/// last fruit's knuckle, and on from there to that fruit's calyx, bent under
/// its fruit's weight. The peduncle and the first rachis are drawn along it,
/// and a bent truss's rod is cut from it.
#[derive(Clone, Debug)]
struct Axis {
    /// Its points, and how far along it each one is: the peduncle's, `lead`
    /// of them out of the stem and round its bend, then the first rachis's
    /// nodes from the fork, then the points the last fruit's pedicel runs
    /// through from its knuckle, at the last node, on to the calyx, once
    /// [`build_truss`] draws it.
    points: Vec<Vec3>,
    stations: Vec<f32>,
    lead: usize,
    /// Off vertical where it leaves the stem, which its straight run out of
    /// the stem keeps, and that run's length, in meters.
    angle: f32,
    straight: f32,
    /// How far along the fork is, where the bend ends and the rachis starts,
    /// at the first fruit.
    fork: f32,
    /// Each rachis's internodes, from the fork to its last node: the first's,
    /// and a branched truss's second's. How far apart their nodes are full
    /// grown, and how far each fruit's stalks have grown, from the truss's
    /// base to its tip.
    rachises: Vec<Vec<Internode>>,
    pitch: f32,
    stalks: Vec<f32>,
    /// The peduncle's radius, which the rachis tapers from, its cross-section
    /// grown in step with the first fruit's stalks (est.), and the last
    /// pedicel's, no thicker than the rachis it carries on.
    radius: f32,
    thin: f32,
    /// How far the fruit's weight moved the first rachis's last node.
    droop: Vec3,
}

impl Axis {
    /// The axis of a truss carrying `loads`, the mass of each of its fruit in
    /// kilograms, none where one is missing, its `stalks` as far grown as
    /// [`stalks`] has them, bent against a Young's modulus of `modulus`
    /// pascals.
    ///
    /// Draws the first rachis's kinks, then a branched truss's second's.
    fn new(
        config: &TrussConfig,
        loads: &[f32],
        stalks: &[f32],
        modulus: f32,
        rng: &mut Rng,
    ) -> Self {
        let angle = config.peduncle_angle;
        // Carrying nothing, the peduncle bends round to level, or by the
        // least turn if it leaves at or below level.
        let turn = (FRAC_PI_2 - angle).max(MIN_TURN);
        let straight = config.peduncle_length * STRAIGHT;
        let bend = config.peduncle_length * (1.0 - STRAIGHT) / turn;
        let round = |a: f32| {
            heading(angle) * straight
                + bend * Vec3::new(angle.cos() - a.cos(), 0.0, a.sin() - angle.sin())
        };
        // Out of the stem along `angle`, holding straight, then round a
        // circle to the fork.
        let mut points: Vec<Vec3> = [0.0, 0.5, 1.0]
            .map(|t| heading(angle) * straight * t)
            .to_vec();
        let held = points.len();
        points
            .extend((1..BEND_POINTS).map(|i| round(angle + turn * i as f32 / BEND_POINTS as f32)));
        let lead = points.len();

        let arms = arms(config);
        let pitch = config.internode;
        // Each internode of rachis `arm` grows with the fruit it leads to.
        let steps = |arm: usize| -> Vec<f32> {
            (stalks.iter().skip(arm + arms.len()).step_by(arms.len()))
                .map(|grown| pitch * grown)
                .collect()
        };
        let rest = heading(angle + turn);
        let first = rachis(
            round(angle + turn),
            Quat::from_rotation_z(arms[0]) * rest,
            &steps(0),
            1.0,
            rng,
        );
        let tip = first[first.len() - 1].from;
        points.extend(first.iter().map(|run| run.from));
        let radius = config.peduncle_radius * stalks[0].sqrt();
        // The first rachis's last fruit.
        let last = (config.fruits as usize - 1) / arms.len() * arms.len();
        let mut axis = Self {
            stations: chords(&points),
            points,
            lead,
            angle,
            straight,
            fork: 0.0,
            rachises: vec![first],
            pitch,
            stalks: stalks.to_vec(),
            radius,
            thin: pedicel_radius(config.fruit_diameter, stalks[last]).min(radius * RACHIS_TAPER),
            droop: Vec3::ZERO,
        };
        axis.fork = axis.stations[lead];

        // The straight run holds, and past it the axis bends as thick as it
        // is. The first rachis's fruit weigh on its nodes, and a branched
        // truss's second rachis's on the fork as well as on its own nodes.
        let stiff = |s: f32| rigidity(modulus, axis.radius_at(s));
        let bends: Vec<f32> = (axis.stations.iter().enumerate())
            .map(|(k, s)| if k < held { f32::INFINITY } else { stiff(*s) })
            .collect();
        let second: Vec<f32> = loads.iter().skip(1).step_by(arms.len()).copied().collect();
        let second_steps = steps(1);
        let second_nodes = once(0.0).chain(second_steps.iter().scan(0.0, |along, step| {
            *along += step;
            Some(*along)
        }));
        let second_bends: Vec<f32> = once(f32::INFINITY)
            .chain(second_nodes.map(|along| stiff(axis.fork + along)))
            .collect();
        let grown = axis.entering();
        let mut weights = vec![0.0; axis.points.len()];
        for (j, mass) in loads.iter().enumerate() {
            let node = if j % arms.len() == 0 {
                j / arms.len()
            } else {
                0
            };
            weights[lead + node] += mass;
        }
        let (peduncle, first) =
            sag_rachis(&axis.points[..lead], &axis.rachises[0], &bends, &weights);
        axis.points = peduncle
            .into_iter()
            .chain(first.iter().map(|run| run.from))
            .collect();
        axis.droop = first[first.len() - 1].from - tip;
        axis.rachises[0] = first;

        // A second rachis grows on from where the peduncle ends, turned as
        // the peduncle's end turned, from inside the first.
        if let Some(yaw) = arms.get(1)
            && !second.is_empty()
        {
            let (fork, into) = (axis.points[lead], axis.entering());
            let hang = Quat::from_rotation_arc(grown, into) * Quat::from_rotation_z(*yaw) * rest;
            let runs = rachis(fork, hang, &second_steps, -1.0, rng);
            let weights: Vec<f32> = once(0.0).chain(second).collect();
            let back = [axis.behind()];
            axis.rachises
                .push(sag_rachis(&back, &runs, &second_bends, &weights).1);
        }
        axis
    }

    /// Which way the peduncle runs into the fork.
    fn entering(&self) -> Vec3 {
        (self.points[self.lead] - self.points[self.lead - 1]).normalize()
    }

    /// Where a branched truss's second rachis starts, inside the first: half
    /// an internode back from the fork, as long as its first fruit's.
    fn behind(&self) -> Vec3 {
        self.points[self.lead] - self.entering() * self.pitch * self.stalks[1] / 2.0
    }

    /// How far along each node of rachis `arm` is, from the fork.
    fn nodes(&self, arm: usize) -> Vec<f32> {
        let points: Vec<Vec3> = self.rachises[arm].iter().map(|run| run.from).collect();
        chords(&points).into_iter().map(|s| self.fork + s).collect()
    }

    /// The first rachis's length, from the fork to its last node: none on a
    /// truss of one fruit.
    fn reach(&self) -> f32 {
        self.stations[self.lead + self.rachises[0].len() - 1] - self.fork
    }

    fn length(&self) -> f32 {
        self.stations[self.stations.len() - 1]
    }

    /// Carries the axis on through `points`.
    fn extend(&mut self, points: &[Vec3]) {
        for point in points {
            let last = self.points[self.points.len() - 1];
            self.stations.push(self.length() + last.distance(*point));
            self.points.push(*point);
        }
    }

    /// The point `s` along.
    fn at(&self, s: f32) -> Vec3 {
        let k = segment_of(&self.stations, s);
        let (from, to) = (self.stations[k], self.stations[k + 1]);
        self.points[k].lerp(
            self.points[k + 1],
            ((s - from) / (to - from)).clamp(0.0, 1.0),
        )
    }

    /// The radius `s` along: the peduncle's, tapering down the rachis to
    /// [`RACHIS_TAPER`] of it at the last node, and the last pedicel's past
    /// it.
    fn radius_at(&self, s: f32) -> f32 {
        let down = (s - self.fork) / self.reach().max(f32::EPSILON);
        if down > 1.0 + f32::EPSILON {
            return self.thin;
        }
        self.radius * (1.0 - (1.0 - RACHIS_TAPER) * down.clamp(0.0, 1.0))
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
    /// its station, its tubes meshed to `tolerance` and swollen where `main`,
    /// the strand along the axis, is. Its points are evenly spaced past the
    /// held segment.
    fn new(
        axis: &Axis,
        main: &Strand,
        riders: &[(f32, Strand)],
        tolerance: f64,
        holds: Vec<f32>,
        bend: f32,
    ) -> Result<Self> {
        let length = axis.length();
        let held = HELD.min(length / 2.0);
        let count = ((length - held) / SEGMENT).round().max(2.0) as usize;
        let stations: Vec<f32> = once(0.0)
            .chain((0..=count).map(|k| held + (length - held) * k as f32 / count as f32))
            .collect();
        let points: Vec<[f32; 3]> = stations.iter().map(|s| axis.at(*s).to_array()).collect();
        let widths: Vec<f32> = stations.iter().map(|s| 2.0 * axis.radius_at(*s)).collect();
        let frames = segment_frames(&points);
        // Each tube runs straight along its segment and overruns both ends by
        // its radius there, as a `segment_tubes` tube does, and swells where
        // `main` does. `main`'s swellings sit at their distance along its
        // points, under a millimeter short of their station past the bend.
        let tubes = frames
            .iter()
            .enumerate()
            .map(|(k, frame)| {
                let (from, to) = (Vec3::from(points[k]), Vec3::from(points[k + 1]));
                let (base, top) = (widths[k] / 2.0, widths[k + 1] / 2.0);
                let run = (to - from).normalize();
                let mut tube = strand(
                    &[from - run * base, to + run * top],
                    &[base, top],
                    tolerance,
                    main.bulges.clone(),
                    main.collars.clone(),
                );
                // Measured from where the tube starts instead.
                let start = (stations[k] - base) as f64;
                tube.bulges.iter_mut().for_each(|b| b.at -= start);
                tube.collars.iter_mut().for_each(|c| c.at -= start);
                let into = rebase(frame, &Transform::IDENTITY);
                Ok(strand_mesh(&tube)?.transformed(&into).to_mesh())
            })
            .collect::<Result<_>>()?;
        let mut carried = vec![Vec::new(); frames.len()];
        for (at, rider) in riders {
            let k = segment_of(&stations, *at);
            let into = rebase(&frames[k], &Transform::IDENTITY);
            carried[k].push(strand_mesh(rider)?.transformed(&into));
        }
        Ok(Self {
            tubes,
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
    /// Where every fruit hangs, from the truss's base to its tip: its calyx on
    /// its pedicel's end, +Z back up the pedicel.
    fruit: Vec<Transform>,
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

/// How far along `points` each one is.
fn chords(points: &[Vec3]) -> Vec<f32> {
    once(0.0)
        .chain(points.windows(2).scan(0.0, |length, w| {
            *length += w[0].distance(w[1]);
            Some(*length)
        }))
        .collect()
}

/// A stalk's flare where it leaves something `around` thick.
fn flare(around: f32) -> Bulge {
    Bulge {
        at: 0.0,
        width: FLARE.1 * around as f64,
        height: FLARE.0,
    }
}

/// Where a pedicel `length` long from `run`'s node, heading `out`, has its
/// knuckle and its calyx: out to the knuckle, then on as [`calyx`] bends it.
///
/// Draws: how far it swings off `out` and how far it reaches, then the
/// calyx's.
fn pedicel(run: &Internode, out: Vec3, length: f32, rng: &mut Rng) -> (Vec3, Vec3) {
    let out = Quat::from_axis_angle(run.up, rng.range(-SWING, SWING) as f32) * out;
    let knee = run.from + out * (KNUCKLE_AT * length * jitter(rng, REACH_JITTER));
    (knee, calyx(knee, out, (1.0 - KNUCKLE_AT) * length, rng))
}

/// Where a calyx hangs `distal` off a pedicel's knuckle `knee`, the pedicel
/// heading `out` into it: bent down there by the fruit and leaned out the
/// way it headed.
///
/// Draws how far it leans.
fn calyx(knee: Vec3, out: Vec3, distal: f32, rng: &mut Rng) -> Vec3 {
    let lean = LEAN * jitter(rng, LEAN_JITTER);
    let down = (Vec3::new(out.x, out.y, 0.0) * lean - Vec3::Z).normalize();
    knee + down * distal
}

/// The ball each of a truss's fruit fills now, from its base to its tip, as
/// [`FruitConfig::ball`] has it at the size its truss gives it, before its
/// own jitter: the least a representative's fruit are packed at, so that a
/// pedicel no truss drawing it hangs a fruit on is packed all the same.
fn balls(config: &TrussConfig, fruit: &FruitParams) -> Vec<(f32, f32)> {
    let last = (config.fruits - 1).max(1) as f32;
    (0..config.fruits)
        .map(|j| {
            let along = j as f32 / last;
            let hung = Hung {
                rank: config.node.rank,
                along,
                age: config.node.age,
            };
            let diameter = config.fruit_diameter * (1.0 - TIP_SHRINK * along);
            FruitConfig::new(fruit, 0, diameter, hung).ball()
        })
        .collect()
}

/// The least ball holding two that hang on one line, each as its radius and
/// how far down the line its middle is: from whichever reaches further up
/// the line to whichever reaches further down it.
fn holding(a: (f32, f32), b: (f32, f32)) -> (f32, f32) {
    let up = (a.1 - a.0).min(b.1 - b.0);
    let down = (a.1 + a.0).max(b.1 + b.0);
    ((down - up) / 2.0, (down + up) / 2.0)
}

/// Where each of a truss's fruit, from its base to its tip, would hang
/// alone off `axis`, filling its ball in `balls`, its stalks as far grown as
/// `grown` has them: every pedicel but a rachis's last down its flank under
/// the rachis, as [`pedicel`] lays it, and the last off the rachis's last
/// node, its knuckle, the rachis's last internode its pedicel that far.
///
/// Draws, down the first rachis and then a branched truss's second, how
/// each pedicel lies, the last one's lean only.
fn lay(
    config: &TrussConfig,
    axis: &Axis,
    params: &TrussParams,
    balls: &[(f32, f32)],
    grown: &[f32],
    rng: &mut Rng,
) -> Vec<Bob> {
    let (count, arms) = (config.fruits as usize, arms(config).len());
    let (under, level) = params.pedicel_angle.to_radians().sin_cos();
    let mut laid = vec![None; count];
    for (arm, runs) in axis.rachises.iter().enumerate() {
        // Fruit alternate between the arms, then between the flanks of each.
        let fruits: Vec<usize> = (arm..count).step_by(arms).collect();
        let last = fruits.len() - 1;
        for (i, (run, j)) in runs.iter().zip(fruits).enumerate() {
            let length = config.pedicel * grown[j];
            let (knee, calyx) = if i < last {
                pedicel(run, run.side * level - run.up * under, length, rng)
            } else {
                let toward = runs[last.saturating_sub(1)].along;
                let distal = (1.0 - KNUCKLE_AT) * length;
                (run.from, calyx(run.from, toward, distal, rng))
            };
            laid[j] = Some(Bob {
                node: run.from,
                joint: knee,
                end: calyx,
                radius: balls[j].0,
                inset: balls[j].1,
                stalk: pedicel_radius(config.fruit_diameter, grown[j]),
            });
        }
    }
    laid.into_iter()
        .map(|bob| bob.expect("every fruit hangs off a rachis"))
        .collect()
}

/// The stems a truss's fruit are packed clear of, as segments and their
/// radii: the peduncle and the first rachis, a branched truss's second, and
/// the stem it hangs from, `node`'s radius, plumb through the node, as it
/// grows where trusses hang.
fn stems(axis: &Axis, node: &Node) -> Vec<(Vec3, Vec3, f32)> {
    let mut stems: Vec<(Vec3, Vec3, f32)> = (axis.points.windows(2).zip(&axis.stations))
        .map(|(w, s)| (w[0], w[1], axis.radius_at(*s)))
        .collect();
    // A branched truss's second rachis, as thick as it is drawn: from the
    // peduncle's radius inside the first, tapering as the first does.
    if let Some(second) = axis.rachises.get(1) {
        let points: Vec<Vec3> = once(axis.behind())
            .chain(second.iter().map(|run| run.from))
            .collect();
        let radii = once(axis.radius).chain(axis.nodes(1).into_iter().map(|s| axis.radius_at(s)));
        stems.extend(points.windows(2).zip(radii).map(|(w, r)| (w[0], w[1], r)));
    }
    stems.push((
        Vec3::Z * STEM_SPAN,
        Vec3::NEG_Z * STEM_SPAN,
        node.stem_radius,
    ));
    stems
}

/// Builds one truss in its own frame, bent under `loads`, the mass of each of
/// its fruit, its stalks as far grown as its age has them, its fruit laid
/// out as the params lay them and packed as the `balls` they fill, and its
/// stems meshed to `tolerance`: as one mesh if `whole`, and as a rod if
/// `bent`.
///
/// Draws, off the config's own stream: its rachises' kinks, as [`Axis::new`]
/// does, then how its pedicels lie, as [`lay`] does.
fn build_truss(
    config: &TrussConfig,
    loads: &[f32],
    params: &TrussParams,
    balls: &[(f32, f32)],
    tolerance: f64,
    whole: bool,
    bent: bool,
) -> Result<Built<Mesh>> {
    let mut rng = Rng::new(config.seed);
    let grown = stalks(config, params);
    let mut axis = Axis::new(config, loads, &grown, modulus(params), &mut rng);
    let (straight, fork) = (axis.straight, axis.fork);
    let peduncle = axis.points[..axis.lead].to_vec();
    let mut packed = lay(config, &axis, params, balls, &grown, &mut rng);
    pack(&mut packed, &stems(&axis, &config.node));

    let arms = arms(config);
    let count = config.fruits as usize;
    // The points a pedicel runs through, cutting the corner at its knuckle
    // through the middle of each run.
    let through = |bob: &Bob| {
        [
            bob.node,
            bob.node.lerp(bob.joint, 0.5),
            bob.joint.lerp(bob.end, 0.5),
            bob.end,
        ]
    };
    // A pedicel `thin` in radius swells at its knuckle, `knuckle` along it,
    // and into the calyx at its end, `end` along.
    let swellings = |thin: f32, knuckle: f32, end: f32| {
        let knuckle = Bulge {
            at: knuckle as f64,
            width: KNUCKLE.1 * thin as f64,
            height: KNUCKLE.0,
        };
        let receptacle = Bulge {
            at: end as f64,
            width: RECEPTACLE.1 * thin as f64,
            height: RECEPTACLE.0,
        };
        params
            .jointed
            .then_some(knuckle)
            .into_iter()
            .chain(once(receptacle))
    };

    // The first rachis carries on from the peduncle; everything else hangs off
    // the axis they make, where it leaves it.
    let mut main = None;
    let mut riders = Vec::new();
    let mut hangs = vec![Transform::IDENTITY; count];
    let mut holds = vec![fork; count];
    for (arm, runs) in axis.rachises.clone().into_iter().enumerate() {
        // A rachis ends in its last fruit, so each one grown has one.
        let fruits: Vec<usize> = (arm..count).step_by(arms.len()).collect();
        let last = fruits.len() - 1;
        // On from the peduncle, or from inside the first rachis; node by
        // node to the last.
        let mut points = if arm == 0 {
            peduncle.clone()
        } else {
            vec![axis.behind()]
        };
        let mut radii = vec![axis.radius; points.len()];
        let nodes = points.len();
        let stations = axis.nodes(arm);
        for (run, at) in runs.iter().zip(&stations) {
            points.push(run.from);
            radii.push(axis.radius_at(*at));
        }
        // The rachis's strand runs on from its last node into its last
        // fruit's calyx.
        let stalks: Vec<[Vec3; 4]> = fruits[..last]
            .iter()
            .map(|j| through(&packed[*j]))
            .collect();
        let end = through(&packed[fruits[last]]);
        let tail = [end[0], end[2], end[3]];
        let tip = packed[fruits[last]].stalk;
        points.extend(&tail[1..]);
        radii.extend([tip.min(radii[radii.len() - 1]); 2]);
        if arm == 0 {
            axis.extend(&tail[1..]);
        }
        let along = chords(&points);

        // The peduncle flares out of the stem; the rachis swells at every
        // node, round its pedicel, which flares out of it.
        let mut bulges = Vec::new();
        if arm == 0 {
            bulges.push(flare(config.node.stem_radius));
        }
        let (mut collars, mut pedicels) = (Vec::new(), Vec::new());
        for (i, j) in fruits.into_iter().enumerate() {
            let (at, around) = (along[nodes + i], radii[nodes + i]);
            let thin = packed[j].stalk;
            // Its calyx, and the point before it on the way in.
            let (calyx, before) = if i < last {
                let stalk = &stalks[i];
                bulges.push(Bulge {
                    at: at as f64,
                    width: KNOT.1 * around as f64,
                    height: KNOT.0,
                });
                // A second rachis's pedicels ride with it, from the fork.
                if arm == 0 {
                    holds[j] = stations[i];
                }
                collars.push(Collar {
                    at: at as f64,
                    toward: vector(stalk[1] - stalk[0]),
                    radius: thin as f64,
                });
                let length = chords(stalk);
                let swollen = once(flare(around))
                    .chain(swellings(thin, (length[1] + length[2]) / 2.0, length[3]))
                    .collect();
                pedicels.push((
                    holds[j],
                    strand(stalk, &[thin; 4], tolerance, swollen, Vec::new()),
                ));
                (stalk[3], stalk[2])
            } else {
                if arm == 0 {
                    holds[j] = axis.length();
                }
                bulges.extend(swellings(thin, at, along[along.len() - 1]));
                (tail[2], tail[1])
            };
            hangs[j] = Transform::from_translation(calyx).with_rotation(Quat::from_rotation_arc(
                Vec3::Z,
                (before - calyx).normalize(),
            ));
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
        Some(Rod::new(&axis, &main, &riders, tolerance, holds, middle)?)
    } else {
        None
    };
    // Along the axis there, from a millimeter either side.
    let toward = axis.at(middle + 0.001) - axis.at(middle - 0.001);
    Ok(Built {
        mesh,
        rod,
        leaves: heading(axis.angle),
        bend: Transform::from_translation(axis.at(middle))
            .with_rotation(facing(toward.normalize())),
        fruit: hangs,
    })
}

/// The least rotation turning +X onto `direction`: about +Y alone for one in
/// the XZ plane.
fn facing(direction: Vec3) -> Quat {
    Quat::from_rotation_arc(Vec3::X, direction)
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
/// Each truss's fruit draw from a stream keyed on its [`Order`]: three draws
/// per pedicel, whether its fruit is missing, how big it is and how it is
/// turned about its axis, made for every pedicel so a higher `missing_fruit`
/// drops fruit without resizing the rest.
/// A fruit's own shape is drawn off a stream keyed on its truss and its
/// pedicel. A truss's fruit are drawn before its shape is chosen, as their
/// weight bends it.
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

    // A truss's fruit on `pedicels` of them, each with how it is turned about
    // its axis. Drawn per pedicel, so the fruit on the pedicels two counts
    // share are drawn alike.
    let hang_fruit = |order: Order, config: &TrussConfig, pedicels: usize| {
        let mut rng = Rng::new(scene.seed ^ FRUIT_STREAM ^ salt(order.0));
        let last = (pedicels - 1).max(1) as f32;
        (0..pedicels)
            .filter_map(|j| {
                let missing = rng.unit() < params.missing_fruit as f64;
                let size = rng.range(1.0 - FRUIT_JITTER, 1.0 + FRUIT_JITTER) as f32;
                let spin = rng.unit() as f32 * TAU;
                let along = j as f32 / last;
                let fruit = || {
                    FruitConfig::new(
                        &fruit_params,
                        // A truss holds fewer than 256 fruit.
                        scene.seed ^ FRUIT_SHAPE_STREAM ^ salt(order.0 << 8 | j as u64),
                        config.fruit_diameter * (1.0 - TIP_SHRINK * along) * size,
                        Hung {
                            rank: config.node.rank,
                            along,
                            age: config.node.age,
                        },
                    )
                };
                (!missing).then(|| (j, spin, fruit()))
            })
            .collect::<Vec<_>>()
    };
    // What each truss's own fruit weigh, which its shape and its share of the
    // budget are read off with how far its stalks have grown; it hangs them
    // on the pedicels of the representative it draws.
    let modulus = modulus(&params);
    let laden = par_map(&hung, |_, (order, _, config)| {
        let mut loads = vec![0.0; config.fruits as usize];
        let fruit = hang_fruit(*order, config, config.fruits as usize);
        for (j, _, fruit) in &fruit {
            loads[*j] = fruit.mass();
        }
        let grown = stalks(config, &params);
        let axis = Axis::new(config, &loads, &grown, modulus, &mut Rng::new(config.seed));
        let size = fruit.iter().map(|(.., fruit)| fruit.ball().0).sum::<f32>();
        Laden {
            config: *config,
            loads,
            axis,
            size: size / fruit.len().max(1) as f32,
        }
    });
    let book = farthest_first(&laden, params.variations.max(1) as usize, 0.0, &TrussMetric);
    // Which ways each representative is drawn: whole for the trusses that drew
    // it and stay rigid, as a rod for the ones that bend.
    let mut ways = vec![(false, false); book.len()];
    for ((_, _, config), drew) in hung.iter().zip(&book.assignment) {
        let way = &mut ways[*drew as usize];
        if config.node.flexible {
            way.1 = true;
        } else {
            way.0 = true;
        }
    }
    // Each truss's own fruit, on the pedicels of the representative it drew;
    // and, for each of a representative's pedicels, one ball holding every
    // ball its fruit fills on any truss that drew it, which the
    // representative is packed at, so that no truss's own fruit run into
    // each other. The largest radius at the furthest middle would not do: an
    // open flower's ball is wider than a bud's, but its middle is nearer its
    // pedicel.
    let drawn = par_map(&hung, |t, (order, _, config)| {
        let drew = &book.representatives[book.assignment[t] as usize];
        hang_fruit(*order, config, drew.config.fruits as usize)
    });
    let mut held: Vec<Vec<(f32, f32)>> = (book.representatives.iter())
        .map(|laden| balls(&laden.config, &fruit_params))
        .collect();
    for (fruit, drew) in drawn.iter().zip(&book.assignment) {
        for (j, _, fruit) in fruit {
            let ball = &mut held[*drew as usize][*j];
            *ball = holding(*ball, fruit.ball());
        }
    }
    let tolerance = TOLERANCE / stem.detail.max(MIN_DETAIL) as f64;
    let shapes = par_map(&book.representatives, |index, laden| {
        let (whole, bent) = ways[index];
        let (config, balls) = (&laden.config, &held[index]);
        build_truss(config, &laden.loads, &params, balls, tolerance, whole, bent)
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
                fruit: shape.fruit,
            },
            surface,
        ));
    }

    let mut fruit_order = 0u64;
    for (((_, entity, config), drew), fruit) in hung.iter().zip(&book.assignment).zip(drawn) {
        let (built, surface) = &built[*drew as usize];
        let fruit: Vec<(usize, Transform, FruitConfig)> = fruit
            .into_iter()
            .map(|(j, spin, fruit)| {
                let hang = built.fruit[j];
                let placement = hang.with_rotation(hang.rotation * Quat::from_rotation_z(spin));
                (j, placement, fruit)
            })
            .collect();

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

    /// A bent truss's tubes swell as a rigid truss's stems do: the held one
    /// flares out of the stem, and the one holding each pedicel swells round
    /// it.
    #[test]
    fn a_bent_truss_swells_where_a_rigid_one_does() {
        let node = Node {
            flexible: true,
            ..NODE
        };
        let params = TrussParams::default();
        let config = TrussConfig::new(&params, 0, 0, node);
        let rod = build_truss(
            &config,
            &laden(&config, 1.0),
            &params,
            &balls(&config, &FruitParams::default()),
            TOLERANCE,
            false,
            true,
        )
        .unwrap()
        .rod
        .unwrap();
        // How far tube `k` reaches off its segment's axis, as a fraction of
        // the least radius it tapers between.
        let swell = |k: usize| {
            let points = rod.tubes[k]
                .attribute(Mesh::ATTRIBUTE_POSITION)
                .and_then(|a| a.as_float3())
                .unwrap();
            let widest = points.iter().map(|p| p[0].hypot(p[1])).fold(0.0, f32::max);
            widest / (rod.widths[k].min(rod.widths[k + 1]) / 2.0) - 1.0
        };
        assert!(
            swell(0) > FLARE.0 as f32 / 2.0,
            "the held tube swells {}",
            swell(0)
        );
        for hold in &rod.holds {
            let k = segment_of(&rod.stations, *hold);
            assert!(
                swell(k) > KNOT.0 as f32 / 2.0,
                "tube {k} swells {}",
                swell(k)
            );
        }
    }

    /// A node a truss hangs from rigid, its fruit full grown.
    const NODE: Node = Node {
        stem_radius: 0.006,
        rank: 0.0,
        age: 8.0,
        flexible: false,
    };

    /// The mass of each fruit on a truss with all of them, `share` of a
    /// full-grown fruit's.
    fn laden(config: &TrussConfig, share: f32) -> Vec<f32> {
        let hung = Hung {
            rank: 0.0,
            along: 0.0,
            age: 8.0,
        };
        let full = FruitConfig::new(&FruitParams::default(), 0, config.fruit_diameter, hung);
        vec![full.mass() * share; config.fruits as usize]
    }

    /// Down a rachis every internode runs off its node's pedicel, the next
    /// one back the other way, a pitch apart, about the way it grows.
    #[test]
    fn a_rachis_zigzags_away_from_its_pedicels() {
        let mut rng = Rng::new(7);
        let mean = heading(FRAC_PI_2);
        let runs = rachis(Vec3::ZERO, mean, &[0.033; 11], 1.0, &mut rng);
        assert_eq!(runs.len(), 12);
        let kink = KINK * (1.0 + NODE_JITTER as f32) / 2.0;
        for (i, run) in runs.iter().enumerate() {
            assert!(
                (run.along.length() - 1.0).abs() < 1e-5
                    && run.along.angle_between(mean) <= kink + 1e-5,
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
    }

    /// Carrying nothing, a truss stands out from the stem, its rachis level;
    /// the heavier its fruit the lower it hangs, its rachis nearer straight
    /// down, and full grown it hangs from its bend, its tip further below the
    /// fork than half the rachis's length. Its nodes stay a pitch apart
    /// however far it bends.
    #[test]
    fn a_truss_stands_out_in_flower_and_sags_as_its_fruit_swell() {
        let params = TrussParams::default();
        for order in 0..20 {
            let config = TrussConfig::new(&params, 0, order, NODE);
            let axes = [0.0, 0.05, 0.3, 1.0].map(|share| {
                let (loads, rng) = (laden(&config, share), &mut Rng::new(config.seed));
                Axis::new(
                    &config,
                    &loads,
                    &stalks(&config, &params),
                    modulus(&params),
                    rng,
                )
            });
            // How far the rachis's first internode hangs off straight down,
            // how high its tip is, and how far below the fork.
            let hang = |axis: &Axis| {
                let runs = &axis.rachises[0];
                let tip = runs[runs.len() - 1].from.z;
                let tilt = runs[0].along.angle_between(Vec3::NEG_Z);
                (tilt, tip, axis.points[axis.lead].z - tip)
            };
            let hangs = axes.each_ref().map(hang);
            let (level, standing, _) = hangs[0];
            assert!(
                (level - FRAC_PI_2).abs() < KINK && standing > 0.0,
                "{order}: in flower {:?}",
                hangs[0]
            );
            for (lighter, heavier) in hangs.iter().zip(&hangs[1..]) {
                assert!(
                    heavier.0 < lighter.0 && heavier.1 < lighter.1,
                    "{order}: {hangs:?}"
                );
            }
            let (first, _, below) = hangs[3];
            assert!(
                first < 1.0 && below > axes[3].reach() / 2.0,
                "{order}: full grown {:?}",
                hangs[3]
            );
            for runs in axes.iter().flat_map(|axis| &axis.rachises) {
                for pair in runs.windows(2) {
                    let pitch = config.internode;
                    assert!((pair[1].from.distance(pair[0].from) - pitch).abs() < 1e-5);
                }
            }
        }
    }

    /// A truss's stalks grow with its flowers: each pedicel, laid out before
    /// its fruit are packed, and the internode leading to it, is
    /// `stalks_in_flower` of its full length when its flower opens, shorter
    /// the younger its bud, and full length while its fruit is still growing.
    /// The peduncle thickens with them. A simple truss and a branched one.
    #[test]
    fn a_trusss_stalks_grow_with_its_flowers() {
        // The age at which the flower `along` its truss opens.
        let opens = |along: f32| {
            -Hung {
                rank: 0.0,
                along,
                age: 0.0,
            }
            .opened()
        };
        let full = opens(1.0) + STALK_GROWING;
        let ages = [0.0, opens(0.0), opens(0.5), opens(1.0), 2.0, full, 8.0];
        for branched in [false, true] {
            let params = TrussParams {
                branched,
                ..TrussParams::default()
            };
            let arms = arms(&TrussConfig::new(&params, 0, 3, NODE));
            // How far each fruit's calyx hangs from its node, and how long
            // the internode leading to that node is, at `age`; and the
            // peduncle's radius.
            let grow = |age: f32| {
                let config = TrussConfig::new(&params, 0, 3, Node { age, ..NODE });
                let loads = vec![0.0; config.fruits as usize];
                let grown = stalks(&config, &params);
                let rng = &mut Rng::new(config.seed);
                let axis = Axis::new(&config, &loads, &grown, modulus(&params), rng);
                let balls = balls(&config, &FruitParams::default());
                let laid = lay(&config, &axis, &params, &balls, &grown, rng);
                let node = |j: usize| axis.rachises[j % arms.len()][j / arms.len()].from;
                let spans: Vec<[f32; 2]> = (laid.iter().enumerate())
                    .map(|(j, bob)| {
                        let leading = j
                            .checked_sub(arms.len())
                            .map_or(0.0, |k| node(j).distance(node(k)));
                        [bob.end.distance(bob.node), leading]
                    })
                    .collect();
                (spans, axis.radius)
            };
            let (grown, thickest) = grow(8.0);
            let last = grown.len() - 1;
            let (mut was, mut thinner) = (vec![0.0; grown.len()], 0.0);
            for (k, age) in ages.into_iter().enumerate() {
                let (spans, radius) = grow(age);
                let shares: Vec<f32> = (spans.iter().zip(&grown))
                    .map(|([span, leading], [full, full_leading])| {
                        let share = span / full;
                        assert!(
                            *full_leading == 0.0 || (leading / full_leading - share).abs() < 1e-4,
                            "branched {branched}, age {age}: {spans:?}"
                        );
                        share
                    })
                    .collect();
                for (share, before) in shares.iter().zip(&was) {
                    assert!(
                        *share >= before - 1e-5 && *share < 1.0 + 1e-4,
                        "branched {branched}, age {age}: {shares:?} after {was:?}"
                    );
                }
                assert!(radius >= thinner && radius <= thickest + 1e-7);
                let flower = |share: f32| (share - params.stalks_in_flower).abs() < 1e-4;
                match k {
                    // The first flower open, the rest in bud behind it.
                    1 => assert!(
                        flower(shares[0]) && shares.windows(2).all(|w| w[1] < w[0]),
                        "branched {branched}: {shares:?}"
                    ),
                    3 => assert!(flower(shares[last]), "branched {branched}: {shares:?}"),
                    5 => {
                        let hung = Hung {
                            rank: 10.0,
                            along: 1.0,
                            age,
                        };
                        let fruit = FruitConfig::new(&FruitParams::default(), 0, 0.06, hung);
                        assert!(
                            shares.iter().all(|share| (share - 1.0).abs() < 1e-4)
                                && fruit.across() < 0.9 * fruit.diameter,
                            "branched {branched}: {shares:?}, fruit {}",
                            fruit.across()
                        );
                    }
                    _ => {}
                }
                (was, thinner) = (shares, radius);
            }
        }
    }

    /// Every pedicel but a rachis's last leaves it under its sides, down its
    /// own flank; the last hangs on past the rachis's last node, its
    /// knuckle, where a bent truss's rod ends at the fruit's calyx. Packed,
    /// no fruit lies more than a little in another, no stalk runs through
    /// one, and none turns its top, toward its knuckle, far past sideways:
    /// full grown, and still growing on a truss that sags less; on a simple
    /// truss and a branched cherry truss.
    #[test]
    fn a_truss_hangs_its_fruit_under_its_rachis_and_ends_in_the_last() {
        let cherry = TrussParams {
            fruits: 15,
            fruit_diameter: 0.025,
            branched: true,
            ..TrussParams::default()
        };
        for params in [TrussParams::default(), cherry] {
            for (order, (age, share)) in (0..100).zip([(8.0, 1.0), (3.5, 0.2)].into_iter().cycle())
            {
                let node = Node {
                    age,
                    flexible: true,
                    ..NODE
                };
                let config = TrussConfig::new(&params, 0, order, node);
                let (loads, balls) = (
                    laden(&config, share),
                    balls(&config, &FruitParams::default()),
                );
                let built =
                    build_truss(&config, &loads, &params, &balls, TOLERANCE, true, order < 3)
                        .unwrap();
                let grown = stalks(&config, &params);
                let rng = &mut Rng::new(config.seed);
                let rachises = Axis::new(&config, &loads, &grown, modulus(&params), rng).rachises;
                let mesh = built.mesh.unwrap();
                let points = mesh
                    .attribute(Mesh::ATTRIBUTE_POSITION)
                    .and_then(|a| a.as_float3())
                    .unwrap();
                let arms = rachises.len();
                for (j, (hang, (radius, inset))) in built.fruit.iter().zip(&balls).enumerate() {
                    let runs = &rachises[j % arms];
                    let (i, run) = (j / arms, &runs[j / arms]);
                    let off = hang.translation - run.from;
                    let (under, flank, on) =
                        (-off.dot(run.up), off.dot(run.side), off.dot(run.along));
                    assert!(
                        if i + 1 < runs.len() {
                            under > 0.0 && flank > 0.0
                        } else {
                            on > 0.0
                        },
                        "{order}: fruit {j} hangs {off} off its node"
                    );
                    let top = hang.rotation * Vec3::Z;
                    assert!(top.z > -0.25, "{order}: fruit {j} faces {top}");
                    let middle = hang.transform_point(-Vec3::Z * *inset);
                    for (k, (other, (by, sunk))) in
                        built.fruit.iter().zip(&balls).enumerate().take(j)
                    {
                        let apart = middle.distance(other.transform_point(-Vec3::Z * *sunk));
                        let into = (radius + by - apart) / (2.0 * radius.min(*by));
                        assert!(into < 0.08, "{order}: fruit {j} lies {into} into {k}");
                    }
                    // No stem inside its body, but its own pedicel coming in
                    // at its top.
                    let thin = pedicel_radius(config.fruit_diameter, grown[j]);
                    let inside = |p: &[f32; 3]| {
                        let local = hang.rotation.inverse() * (Vec3::from(*p) - middle);
                        let (across, up) = (local.truncate().length(), local.z);
                        let entering =
                            across < (0.35 * radius).max(2.5 * thin) && up > 0.4 * radius;
                        (across / radius).powi(2) + (up / (0.9 * radius)).powi(2) < 0.9 && !entering
                    };
                    assert!(!points.iter().any(inside), "{order}: fruit {j} is pierced");
                }
                if let Some(rod) = built.rod {
                    let last = (config.fruits as usize - 1) / arms * arms;
                    let past = rod.stations[rod.stations.len() - 1] - rod.holds[last];
                    let end = Vec3::from(rod.points[rod.points.len() - 1]);
                    let short = end.distance(built.fruit[last].translation);
                    assert!(
                        past.abs() < 1e-6 && short < 1e-6,
                        "{order}: the rod runs {past} m on, {short} m off its last fruit"
                    );
                }
            }
        }
    }

    /// However many trusses share a mesh, none's own fruit lie more than a
    /// little in each other, as each of a representative's fruit is packed
    /// as one ball holding every ball it fills on any truss that drew it;
    /// flowers, more crowded than their pedicels can part, lie further into
    /// each other.
    #[test]
    fn no_trusss_own_fruit_lie_in_each_other() {
        let mut app = grown(small());
        let world = app.world_mut();
        let trusses: Vec<Entity> = world
            .query_filtered::<Entity, With<TrussConfig>>()
            .iter(world)
            .collect();
        for truss in &trusses {
            // The ball each of its fruit fills, wherever under it it hangs.
            let mut balls = Vec::new();
            let mut below = vec![*truss];
            while let Some(at) = below.pop() {
                if let Some(fruit) = world.entity(at).get::<FruitConfig>() {
                    let (radius, inset) = fruit.ball();
                    let middle = in_scene(world, at).transform_point(Vec3::NEG_Z * inset);
                    balls.push((middle, radius, fruit.in_flower()));
                }
                below.extend(world.entity(at).get::<Children>().into_iter().flatten());
            }
            for (i, (a, ra, flower_a)) in balls.iter().enumerate() {
                for (b, rb, flower_b) in &balls[..i] {
                    let into = (ra + rb - a.distance(*b)) / (2.0 * ra.min(*rb));
                    let most = if *flower_a || *flower_b { 0.3 } else { 0.08 };
                    assert!(
                        into < most,
                        "{}: two lie {into} into each other",
                        prim_path(world, *truss)
                    );
                }
            }
        }
    }

    /// The ball holding two that hang on one line holds both, wherever along
    /// the line each is, and is no bigger than it needs be: an open flower's
    /// holds a bud's of its size, so the ball holding the two is the open
    /// flower's.
    #[test]
    fn a_held_ball_holds_both() {
        let holds = |(radius, inset): (f32, f32), ball: (f32, f32)| {
            (inset - ball.1).abs() + ball.0 < radius + 1e-6
        };
        let (bud, open) = ((0.5, 0.5), (0.85, 0.25));
        let fruit = [(0.03, 0.024), (0.028, 0.027)];
        let apart = [(0.01, 0.0), (0.01, 0.05)];
        for [a, b] in [[bud, open], fruit, apart] {
            let held = holding(a, b);
            assert!(
                holds(held, a) && holds(held, b),
                "{a:?} and {b:?} in {held:?}"
            );
        }
        let held = holding(bud, open);
        assert!(
            (held.0 - open.0).abs() < 1e-6 && (held.1 - open.1).abs() < 1e-6,
            "{held:?}"
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

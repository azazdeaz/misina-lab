//! Leaf — a compound tomato leaf: a petiole off the stem, a rachis that
//! arches out and droops under its leaflets, and the leaflets.
//!
//! A tomato leaf is pinnately compound. The rachis ends in a terminal
//! leaflet and carries three or four pairs of near-opposite laterals, the
//! largest nearest the tip, with small intercalary leaflets in the gaps; a
//! lateral may carry a pair of secondary leaflets on its stalk. Every leaflet
//! is a [`Leaflet`] outline filled with its veins kept as edges, bent into
//! relief along them — see [`sheet`] — angled off the rachis toward the tip
//! and hung a little below it:
//!
//! ```text
//!              lateral pairs        an intercalary leaflet
//!            ╲   ╱      ╲   ╱      ╲ ╱  ╱
//!   ──────────┴──────────┴──────────┴──·───▶   the terminal leaflet
//!   petiole  ╱   ╲      ╱   ╲      ╱ ╲
//!   ↑
//!   the stem's axis
//! ```
//!
//! A leaf is built in its own frame, on the stem's axis at its node, facing
//! +X with +Z up. The petiole leaves `angle` off vertical and the rachis
//! turns `droop` further down by the tip, the turn gathering toward the tip,
//! where the rachis is thinnest.
//!
//! A stem hangs a leaf at each of its top nodes, `rank` counting down from
//! the head: small and upright there, its leaflets cut narrow and still
//! folded shut from the bud, full grown and opened out a couple of trusses
//! down, and drooping more the older it is. The leaves below have been
//! stripped.
//!
//! Rachis and leaflets are one mesh, quantized like any organ to a budget of
//! `variations`; which leaflets a leaf carries and how each one is cut are
//! the representative's. Leaves are visual only: they have no collider.
//!
//! ```text
//! Stem_0
//!   Leaf_00 …         -> parts/Leaf_k, the lowest still on the stem first
//! ```

use std::ops::Range;

use bevy::math::DVec2;
use bevy::prelude::*;
use misina_lab::geometry::mesh::{MeshData, bend, merge_meshes};
use misina_lab::geometry::outline::{creased_mesh, spacing};
use misina_lab::geometry::par_map;
use misina_lab::geometry::shapes::Ruffle;
use misina_lab::geometry::strand::{Bark, Bulge, Strand, strand_mesh};
use misina_lab::palette::{COLOR_STREAM, Response, shade, srgb};
use misina_lab::params::{Label, Slider};
use misina_lab::quantize::{Metric, farthest_first};
use misina_lab::rng::{Rng, salt};
use misina_lab::scene::{Library, Order, configs_changed};
use nalgebra::Point3;

use super::leaflet::Leaflet;
use super::{Grow, SceneParams, smooth};

/// The mesh-library prefix the leaves register under.
pub const PART: &str = "Leaf";

/// Salt splitting a leaf's mesh draws off the stream its config draws from.
const LEAFLET_STREAM: u64 = 0x2545_F491_4F6C_DD1D;

/// Nodes below the head a leaf takes to grow to full size, and to settle
/// from upright to its full angle and droop.
const GROWING: f32 = 8.0;
const SETTLING: f32 = 12.0;

/// The leaf at the head, as a fraction of a full-grown one's length and of
/// an old one's droop.
const YOUNG_LENGTH: f32 = 0.2;
const YOUNG_DROOP: f32 = 0.2;

/// The petiole's angle off the stem, in degrees, at the head and on an old
/// leaf.
const ANGLE: (f32, f32) = (30.0, 65.0);

/// How far a leaf's length, angle and droop vary about what its node calls
/// for, as fractions.
const LENGTH_JITTER: f64 = 0.1;
const ANGLE_JITTER: f64 = 0.15;
const DROOP_JITTER: f64 = 0.25;

/// The fewest and the most lateral pairs a leaf has, besides its terminal
/// leaflet.
const PAIRS: (u32, u32) = (3, 4);

/// Lengths along a leaf, as fractions of it: the petiole, from the stem to
/// the first pair; the terminal leaflet; a lateral of the top pair; and an
/// intercalary leaflet.
const PETIOLE: f32 = 0.16;
const TERMINAL: f32 = 0.28;
const LATERAL: f32 = 0.36;
const INTERCALARY: f32 = 0.08;

/// A basal lateral, as a fraction of a top one: the laterals grow toward the
/// tip.
const BASAL: f32 = 0.6;

/// How far the petiole and a leaflet vary in length, as fractions.
const SIZE_JITTER: f64 = 0.15;

/// A lateral's angle off the rachis toward the tip, in degrees, on the basal
/// pair and on the top one; an intercalary leaflet's; and how far either
/// strays.
const SPREAD: (f32, f32) = (75.0, 50.0);
const INTERCALARY_SPREAD: f32 = 80.0;
const SPREAD_JITTER: f64 = 10.0;

/// How far apart along the rachis a pair's two laterals sit, at most, as a
/// fraction of the gap between two pairs.
const OPPOSITE: f64 = 0.2;

/// How far a leaflet hangs below the leaf's plane, in degrees, and droops
/// toward its own tip, in radians.
const HANG: (f64, f64) = (5.0, 25.0);
const DROOP: (f64, f64) = (0.2, 0.6);

/// How far a grown leaflet bends out of flat at a [`LeafParams::curl`] of 1:
/// how high it puckers between two veins, as a fraction of the gap between
/// them, and the steepest its margin's ruffle gets. Neither lifts the blade
/// off the midrib's plane by more than half its distance from the midrib.
const PLEAT: f64 = 0.25;
const RUFFLE: f64 = 0.6;

/// A leaflet just unfolded, past what a grown one does: how far its halves
/// fold up along the midrib, in radians each side, and how far it puckers
/// between its veins. In the bud a leaflet lies folded in half along its
/// midrib, and it opens out as the leaf grows, all the way by the time the
/// leaf is [`OPEN`] grown. The fold stops at [`SHUT`], where the two halves
/// meet, whatever the params add to it.
const YOUNG_FOLD: f64 = 1.4;
const YOUNG_PLEAT: f64 = 0.25;
const OPEN: f64 = 0.5;
const SHUT: f64 = 1.5;

/// Where a leaflet sits in the bud, in degrees: this far off the rachis,
/// held up along it, and this far above the leaf's plane. It spreads out
/// and hangs down as the leaf grows, open with the blade's fold.
const BUD_SPREAD: f32 = 15.0;
const BUD_LIFT: f32 = 60.0;

/// How far a grown leaflet twists about its midrib from its base to its tip,
/// in radians per unit of [`Leaflet::lopsided`].
const TWIST: f64 = 2.0;

/// How far off the midrib the pucker and the ruffle reach their full height,
/// as a fraction of the leaflet's length: the midrib is stiff, and it is
/// where the blade folds.
const CLEAR: f64 = 0.15;

/// Sinusoids summed into the ruffle, and the wavelengths they are drawn from
/// as fractions of the leaflet's length, the shortest only where the mesh
/// resolves it: a wave is at least [`RESOLVE`] triangles long.
const RUFFLE_WAVES: usize = 3;
const RUFFLE_WAVELENGTH: Range<f64> = 0.25..1.4;
const RESOLVE: f64 = 4.0;

/// The side of a triangle across a leaflet's blade at a `detail` of 1, in
/// meters. The pucker and the ruffle bend the blade's middle, which has no
/// vertices but these; a small leaflet gets few of them, as the margin
/// tolerance leaves it few points.
const MESH: f64 = 0.012;

/// The chance of an intercalary leaflet on each side of each gap, and of a
/// pair of secondary leaflets on a lateral.
const INTERCALARY_CHANCE: f64 = 0.8;
const SECONDARY_CHANCE: f64 = 0.5;

/// A lateral with secondary leaflets: its stalk, longer than a plain one's,
/// where along the stalk the pair sits, and how long each of them is, all as
/// fractions of the lateral's length; and their angle off it, in degrees.
const SECONDARY_STALK: f64 = 0.3;
const SECONDARY_AT: f32 = 0.15;
const SECONDARY: f32 = 0.3;
const SECONDARY_SPREAD: f32 = 60.0;

/// The petiole's radius where it leaves the stem, as a fraction of the leaf's
/// length, and the rachis's at the terminal leaflet, as a fraction of that.
const RADIUS: f32 = 0.008;
const TAPER: f32 = 0.3;

/// How much the petiole flares where it leaves the stem, as a fraction of
/// its radius, and over how far, in its radii, out past the stem's surface.
const FLARE: (f64, f64) = (0.6, 3.0);

/// The power the rachis's turn grows by along it: over 1 keeps the petiole
/// straight and bends the far end most.
const CURVE: f32 = 1.8;

/// Steps along the midline, vertices round the rachis, and rings per meter
/// along it at a `detail` of 1.
const STATIONS: usize = 24;
const SIDES: usize = 5;
const RINGS: f64 = 40.0;

/// How far a leaflet's margin may stray from its curves at a `detail` of 1,
/// in meters: about a pixel to a camera half a meter away.
pub const TOLERANCE: f64 = 0.0006;

/// Floors on what Python may set, so that every leaf still builds.
const MIN_LENGTH: f32 = 0.02;
const MIN_DETAIL: f32 = 0.01;

/// A tomato leaf's green, matte from its hairs.
const COLOR: u32 = 0x41672A;
const RESPONSE: Response = Response {
    roughness: 0.65,
    reflectance: 0.4,
};

/// One leaf's shape, as the stem that grew it specified.
#[derive(Component, Clone, Copy, Debug, PartialEq)]
pub struct LeafConfig {
    /// From the stem's axis to the tip of the terminal leaflet, in meters.
    pub length: f32,
    /// Between the stem and the petiole where it leaves, in radians.
    pub angle: f32,
    /// How far the rachis turns down from the stem to its tip, in radians.
    pub droop: f32,
    /// Pairs of lateral leaflets.
    pub pairs: u32,
    /// How far the leaf has grown, 0 at the head to 1 full size: what shape
    /// its leaflets are cut to and how far they are still folded from the
    /// bud. Follows `rank`.
    pub grown: f32,
    /// The stream its own draws and its leaflets' come from.
    pub seed: u64,
    /// Nodes between it and its stem's head, 0 for the youngest. Kept, with
    /// `seed`, so the params can be re-applied to a leaf already hanging;
    /// [`LeafMetric`] reads neither.
    pub rank: u32,
}

impl LeafConfig {
    /// The leaf these params call for `rank` nodes below the head, its own
    /// variation drawn off `seed`.
    ///
    /// The *draw order* is part of this element's output: the length, the
    /// angle and the droop, then the lateral pairs.
    pub fn new(params: &LeafParams, seed: u64, rank: u32) -> Self {
        let mut rng = Rng::new(seed);
        let mut about = |jitter: f64| rng.range(1.0 - jitter, 1.0 + jitter) as f32;
        let grown = smooth(rank as f32 / GROWING);
        let settled = smooth(rank as f32 / SETTLING);
        let length =
            params.length * (YOUNG_LENGTH + (1.0 - YOUNG_LENGTH) * grown) * about(LENGTH_JITTER);
        let angle = (ANGLE.0 + (ANGLE.1 - ANGLE.0) * settled).to_radians() * about(ANGLE_JITTER);
        let droop = params.droop.to_radians()
            * (YOUNG_DROOP + (1.0 - YOUNG_DROOP) * settled)
            * about(DROOP_JITTER);
        let pairs = PAIRS.0 + (rng.unit() * (PAIRS.1 - PAIRS.0 + 1) as f64) as u32;
        Self {
            length: length.max(MIN_LENGTH),
            angle,
            droop: droop.max(0.0),
            pairs,
            grown,
            seed,
            rank,
        }
    }
}

/// Two leaves share a mesh when they are close in every dimension that
/// shows, weighted as roughly how far apart each moves the leaf's tip.
pub struct LeafMetric;

impl Metric<LeafConfig> for LeafMetric {
    fn distance(&self, a: &LeafConfig, b: &LeafConfig) -> f32 {
        let reach = a.length.max(b.length);
        [
            a.length - b.length,
            (a.angle - b.angle) * reach,
            // The droop bends the far half of the leaf.
            (a.droop - b.droop) * reach / 2.0,
            // A pair more is two leaflets more, each a third of the leaf.
            (a.pairs as f32 - b.pairs as f32) * reach / 3.0,
            // The bud's fold closes every leaflet up by about half its width.
            (a.grown - b.grown) * reach / 2.0,
        ]
        .iter()
        .map(|d| d * d)
        .sum::<f32>()
        .sqrt()
    }
}

/// The leaves: a compound leaf at each of a stem's top nodes, as many as the
/// stem keeps.
///
/// A leaf is a petiole and a rachis with a terminal leaflet, three or four
/// pairs of laterals and small leaflets between the pairs, every leaflet cut
/// from its veins, folded along its midrib, puckered between its veins and
/// ruffled at its margin. The leaves near the head are still growing, small
/// and upright, their leaflets narrow and still folded shut from the bud; a
/// couple of trusses down they are full grown and open, and the older they
/// are the more they droop. Every leaf draws its own size, angle, droop and
/// leaflet count about these. Leaves are visual only, with no collider.
#[derive(Resource, Reflect, Clone, Debug, PartialEq)]
#[cfg_attr(
    feature = "python",
    pyo3::pyclass(get_all, set_all, skip_from_py_object)
)]
pub struct LeafParams {
    /// Length of a full-grown leaf, from the stem to the tip of its terminal
    /// leaflet, in meters. Each leaf draws a little more or less.
    #[reflect(@Slider { min: 0.3, max: 0.6, step: 0.01 })]
    pub length: f32,
    /// How far an old leaf's rachis bends down from where it leaves the stem
    /// to its tip, in degrees. The young leaves near the head droop less.
    #[reflect(@Slider { min: 0.0, max: 150.0, step: 1.0 })]
    pub droop: f32,
    /// How deeply the leaflets lobe: 0 leaves every leaflet entire, as on a
    /// potato-leaf cultivar, 1 cuts a lateral's basal sinuses nearly to its
    /// midrib. The terminal leaflet lobes a little less, and the small
    /// leaflets between the pairs hardly at all.
    #[reflect(@Slider { min: 0.0, max: 1.0, step: 0.05 })]
    pub lobing: f32,
    /// How far a grown leaflet's blade bends out of the flat shape it was cut
    /// as: puckered up between its veins and ruffled along its margin, both
    /// scaled together. 0 leaves it smooth but for its fold and its droop.
    ///
    /// What it buys is shading: a flat blade takes one light across its
    /// whole face, a puckered one is a facet of a different slope between
    /// every two veins.
    #[reflect(@Slider { min: 0.0, max: 2.0, step: 0.05 })]
    pub curl: f32,
    /// How far a grown leaflet folds up along its midrib, in radians each
    /// side: 0 holds it flat across, more folds it into a V. A leaflet at the
    /// head is folded far harder whatever this is, its halves still closed
    /// up toward each other as they lay in the bud, and opens out as the
    /// leaf grows.
    #[reflect(@Slider { min: -0.5, max: 1.0, step: 0.05 })]
    pub fold: f32,
    /// How lopsided a lateral leaflet is: 0 for halves that are mirror
    /// images, 1 for one side much the wider, its base oblique and the blade
    /// twisting about its midrib toward the tip. The small leaflets between
    /// the pairs lean a little, the terminal leaflet never.
    #[reflect(@Slider { min: 0.0, max: 1.0, step: 0.05 })]
    pub asymmetry: f32,
    /// How many distinct leaf meshes the scene may hold. A budget, not a
    /// count: the leaves are clustered and this is how many representatives
    /// the clustering may keep.
    #[reflect(@Slider { min: 1.0, max: 128.0, step: 1.0 }, @Label("Leaf variations"))]
    pub variations: u32,
    /// How finely a leaf is meshed, as a multiplier: at 1 a leaflet's margin
    /// keeps within 0.6 mm of its curves, about a pixel to a camera half a
    /// meter away, its blade is cut into triangles about 12 mm on a side,
    /// and a full-grown leaf comes to about 1500 triangles; about 500 at 0.1
    /// and 4500 at 4. Coarser drops the finest teeth first, then rounds off
    /// the sinuses, and keeps the lobes; the pucker and the ruffle soften as
    /// the triangles across the blade grow, and the folds are on the veins
    /// and sharp at any detail.
    #[reflect(@Slider { min: 0.1, max: 4.0, step: 0.05 }, @Label("Leaf detail"))]
    pub detail: f32,
}

impl Default for LeafParams {
    fn default() -> Self {
        Self {
            length: 0.45,
            droop: 90.0,
            lobing: 0.85,
            curl: 1.0,
            fold: 0.35,
            asymmetry: 0.5,
            variations: 32,
            detail: 1.0,
        }
    }
}

pub fn plugin(app: &mut App) {
    // `SceneParams` is not gated on although the colours read the seed:
    // `stem::author` gates on it and hangs fresh leaves when it moves.
    app.init_resource::<LeafParams>().add_systems(
        PreUpdate,
        (
            reauthor.run_if(resource_changed::<LeafParams>),
            build.run_if(
                configs_changed::<LeafConfig>
                    // `variations` and `detail` reach no config.
                    .or_eager(resource_changed::<LeafParams>),
            ),
        )
            .chain()
            .in_set(Grow::Leaves),
    );
}

/// Re-applies the params to every leaf already hanging, in place, from the
/// same draws.
fn reauthor(params: Res<LeafParams>, mut leaves: Query<&mut LeafConfig>) {
    for mut config in &mut leaves {
        let next = LeafConfig::new(&params, config.seed, config.rank);
        config.set_if_neq(next);
    }
}

// ─── The mesh ───────────────────────────────────────────────────────

/// The leaf's midline, from the stem's axis to the tip of the terminal
/// leaflet, in even steps.
struct Midline {
    points: Vec<Vec3>,
    length: f32,
    angle: f32,
    droop: f32,
}

impl Midline {
    fn new(config: &LeafConfig) -> Self {
        let mut midline = Self {
            points: vec![Vec3::ZERO],
            length: config.length,
            angle: config.angle,
            droop: config.droop,
        };
        let step = config.length / STATIONS as f32;
        for i in 0..STATIONS {
            let next = midline.points[i] + midline.along((i as f32 + 0.5) * step) * step;
            midline.points.push(next);
        }
        midline
    }

    /// The direction `s` along: `angle` off vertical at the stem, turning
    /// `droop` further down by the tip.
    fn along(&self, s: f32) -> Vec3 {
        let t = (s / self.length).clamp(0.0, 1.0);
        let heading = self.angle + self.droop * t.powf(CURVE);
        Vec3::new(heading.sin(), 0.0, heading.cos())
    }

    /// The point `s` along.
    fn at(&self, s: f32) -> Vec3 {
        let x = (s / self.length * STATIONS as f32).clamp(0.0, STATIONS as f32);
        let i = (x as usize).min(STATIONS - 1);
        self.points[i].lerp(self.points[i + 1], x - i as f32)
    }
}

/// The frame a leaflet hangs in at `at` off a midrib running `along`, in the
/// XZ plane: turned `spread` toward `side` (1 for +Y, -1 for -Y) and hung
/// `hang` below the midrib's plane; +X along the leaflet, +Z off its upper
/// face.
fn off_midrib(at: Vec3, along: Vec3, side: f32, spread: f32, hang: f32) -> Transform {
    let up = along.cross(Vec3::Y);
    let out = along * spread.cos() + Vec3::Y * side * spread.sin();
    let x = out * hang.cos() - up * hang.sin();
    let z = up * hang.cos() + out * hang.sin();
    Transform::from_translation(at).with_rotation(Quat::from_mat3(&Mat3::from_cols(
        x,
        z.cross(x),
        z,
    )))
}

/// How far a leaf's leaflets bend out of flat, from the params and how far
/// the leaf has grown: what is left of the bud's fold and pucker, and the
/// grown blade's own pucker, ruffle and twist.
struct Relief {
    /// The fold on the midrib, in radians each side.
    fold: f64,
    /// The twist about it from base to tip, in radians per unit of
    /// [`Leaflet::lopsided`].
    twist: f64,
    /// The pucker between two veins, as a fraction of the gap between them.
    pleat: f64,
    /// The steepest the margin's ruffle gets.
    ruffle: f64,
}

/// What is left of the bud in a leaf `grown` far: 1 at the head, 0 once the
/// leaf is [`OPEN`] grown.
fn bud(grown: f64) -> f64 {
    (1.0 - grown / OPEN).max(0.0).powi(2)
}

impl Relief {
    fn new(params: &LeafParams, grown: f64) -> Self {
        let curl = params.curl.max(0.0) as f64;
        let bud = bud(grown);
        Self {
            fold: (params.fold as f64 + YOUNG_FOLD * bud).min(SHUT),
            twist: TWIST * grown,
            pleat: PLEAT * curl + YOUNG_PLEAT * bud,
            ruffle: RUFFLE * curl * grown,
        }
    }
}

/// A leaflet of `shape`, `length` long, in its own frame: its outline filled
/// with its veins kept as edges and bent into `relief` along them, drooping
/// `droop` radians by its tip. Meshed as finely as `detail` says, see
/// [`LeafParams::detail`].
///
/// The blade bends in four layers, the petiolule staying flat but for the
/// droop:
///
/// * a **pucker** between every two veins, the blade standing up off each
///   to a crest between them, every vein a valley as it was a fold in the
///   bud: hard on a leaflet just unfolded, faint on a grown one.
/// * a **ruffle** deepening toward the margin.
/// * a **fold** on the midrib, either half turned up about it, the bud's
///   fold opening as the leaf grows to [`LeafParams::fold`], and twisting
///   toward the tip on a lopsided blade. The pucker and the ruffle flatten
///   as the fold shuts, and reach at most halfway to the midrib's plane, so
///   neither half pokes through the other.
/// * the **droop**, the whole sheet bent down along its length.
///
/// The midrib and the veins are edges of the mesh, so the fold creases on
/// the midrib and the pucker on the veins; both are kept off the midrib so
/// the fold there stays clean. Draws, after the outline's, the ruffle's
/// waves.
fn sheet(
    shape: &Leaflet,
    length: f32,
    droop: f64,
    relief: &Relief,
    detail: f64,
    rng: &mut Rng,
) -> Result<MeshData> {
    let length = length as f64;
    let (outline, veins) = shape.outline(length, TOLERANCE / detail, rng);
    let triangle = MESH * MESH / detail;
    let mut mesh = creased_mesh(&outline, &veins.creases(spacing(triangle)), triangle)?;
    let shortest = (RUFFLE_WAVELENGTH.start * length).max(RESOLVE * spacing(triangle));
    let longest = (RUFFLE_WAVELENGTH.end * length).max(shortest);
    let waves = Ruffle::draw(rng, RUFFLE_WAVES, shortest..longest, relief.ruffle);
    let twist = relief.twist * shape.lopsided;

    for point in &mut mesh.points {
        let (x, y) = (point[0] as f64, point[1] as f64);
        let out = y.abs();
        let mut z = 0.0;
        if x >= veins.stalk {
            let off = (out / (CLEAR * length)).min(1.0);
            let margin = (out / veins.half(x, y)).min(1.0).powi(3) * off;
            z = margin * waves.at(x, y) + relief.pleat * off * veins.ridge(DVec2::new(x, y));
        }
        // Either half turned up about the midrib, no further than SHUT,
        // carrying its pucker and ruffle with it: flattened as the halves
        // come to face each other, and never more than halfway to the
        // midrib's plane, so the two halves' bumps cannot meet.
        let half = if y >= 0.0 { 1.0 } else { -1.0 };
        let turn = (relief.fold + half * twist * x / length).clamp(-SHUT, SHUT);
        let (sin, cos) = turn.sin_cos();
        let z = z.clamp(-out / 2.0, out / 2.0) * cos;
        let (y, z) = (half * (out * cos - z * sin), out * sin + z * cos);
        let (x, z) = if droop == 0.0 {
            (x, z)
        } else {
            bend(x, z, droop / length)
        };
        *point = [x as f32, y as f32, z as f32];
    }
    Ok(mesh)
}

/// One leaf in its own frame: the rachis, and every leaflet hung off it,
/// cut and bent as `params` say and meshed as finely as they ask.
///
/// Draws, off the leaf's leaflet stream: the petiole's length; per pair how
/// far apart its laterals sit, and per lateral, left before right, its size,
/// spread, hang and droop, whether it carries secondary leaflets, and its
/// sheet and theirs; then per gap, left before right, whether it holds an
/// intercalary leaflet and, if so, its place, size, spread, hang, droop and
/// sheet; and last the terminal leaflet's sheet.
fn leaf_mesh(config: &LeafConfig, params: &LeafParams) -> Result<MeshData> {
    let detail = params.detail.max(MIN_DETAIL) as f64;
    let grown = config.grown.clamp(0.0, 1.0) as f64;
    let relief = Relief::new(params, grown);
    // Folded up along the rachis in the bud, a leaflet spreads out and
    // hangs down as the leaf opens.
    let bud = bud(grown) as f32;
    let open = |spread: f32| spread + (BUD_SPREAD.to_radians() - spread) * bud;
    let lift = |hang: f32| hang - (BUD_LIFT.to_radians() + hang) * bud;
    // A leaflet as the params cut it, lopsided toward `side`.
    let cut = |shape: &Leaflet, side: f64| Leaflet {
        lobing: shape.lobing * params.lobing.clamp(0.0, 1.0) as f64,
        lopsided: side * shape.lopsided * params.asymmetry.clamp(0.0, 1.0) as f64,
        grown,
        ..*shape
    };
    let mut rng = Rng::new(config.seed ^ LEAFLET_STREAM);
    let midline = Midline::new(config);
    let length = config.length;
    let jitter = |rng: &mut Rng| rng.range(1.0 - SIZE_JITTER, 1.0 + SIZE_JITTER) as f32;
    let petiole = PETIOLE * length * jitter(&mut rng);
    let top = length * (1.0 - TERMINAL);
    let pairs = config.pairs.max(1);
    let gap = (top - petiole) / pairs as f32;

    let mut sheets = Vec::new();
    for j in 0..pairs {
        let f = j as f32 / (pairs - 1).max(1) as f32;
        let apart = rng.range(-OPPOSITE, OPPOSITE) as f32 * gap / 2.0;
        for side in [1.0, -1.0] {
            let size = LATERAL * length * (BASAL + (1.0 - BASAL) * f) * jitter(&mut rng);
            let spread = SPREAD.0 + (SPREAD.1 - SPREAD.0) * f;
            let spread = (spread + rng.range(-SPREAD_JITTER, SPREAD_JITTER) as f32).to_radians();
            let hang = rng.range(HANG.0, HANG.1).to_radians() as f32;
            let droop = rng.range(DROOP.0, DROOP.1);
            let secondary = rng.unit() < SECONDARY_CHANCE;
            // Lopsided toward the leaf's base, on either side.
            let shape = Leaflet {
                stalk: if secondary {
                    SECONDARY_STALK
                } else {
                    Leaflet::LATERAL.stalk
                },
                ..cut(&Leaflet::LATERAL, side as f64)
            };
            let s = petiole + gap * j as f32 + side * apart;
            let frame = off_midrib(
                midline.at(s),
                midline.along(s),
                side,
                open(spread),
                lift(hang),
            );
            let leaflet = sheet(&shape, size, droop, &relief, detail, &mut rng)?;
            sheets.push(leaflet.transformed(&frame));
            if secondary {
                for side in [1.0, -1.0] {
                    let on = off_midrib(
                        Vec3::X * SECONDARY_AT * size,
                        Vec3::X,
                        side,
                        open(SECONDARY_SPREAD.to_radians()),
                        lift(hang),
                    );
                    let leaflet = sheet(
                        &cut(&Leaflet::INTERCALARY, side as f64),
                        SECONDARY * size,
                        droop,
                        &relief,
                        detail,
                        &mut rng,
                    )?;
                    sheets.push(leaflet.transformed(&(frame * on)));
                }
            }
        }
    }

    // In the gaps above each pair, the last one under the terminal leaflet.
    for j in 0..pairs {
        for side in [1.0, -1.0] {
            if rng.unit() >= INTERCALARY_CHANCE {
                continue;
            }
            let s = petiole + gap * (j as f32 + rng.range(0.3, 0.7) as f32);
            let size = INTERCALARY * length * jitter(&mut rng);
            let spread = INTERCALARY_SPREAD + rng.range(-SPREAD_JITTER, SPREAD_JITTER) as f32;
            let hang = rng.range(HANG.0, HANG.1).to_radians() as f32;
            let droop = rng.range(DROOP.0, DROOP.1);
            let frame = off_midrib(
                midline.at(s),
                midline.along(s),
                side,
                open(spread.to_radians()),
                lift(hang),
            );
            let shape = cut(&Leaflet::INTERCALARY, side as f64);
            let leaflet = sheet(&shape, size, droop, &relief, detail, &mut rng)?;
            sheets.push(leaflet.transformed(&frame));
        }
    }

    // Carrying on the midline's turn to the tip.
    let (along, tip) = (midline.along(top), midline.along(length));
    let droop = along.angle_between(tip) as f64;
    let frame = off_midrib(midline.at(top), along, 0.0, 0.0, 0.0);
    let shape = cut(&Leaflet::TERMINAL, 1.0);
    let terminal = sheet(&shape, length - top, droop, &relief, detail, &mut rng)?;
    sheets.push(terminal.transformed(&frame));

    // The rachis, from the stem's axis into the terminal leaflet's stalk.
    let reach = top + (length - top) * Leaflet::TERMINAL.stalk as f32;
    let n = (reach / length * STATIONS as f32).ceil() as usize;
    let (points, radii) = (0..=n)
        .map(|i| {
            let f = i as f32 / n as f32;
            let p = midline.at(reach * f);
            let r = RADIUS * length * (1.0 - (1.0 - TAPER) * f);
            (Point3::new(p.x as f64, p.y as f64, p.z as f64), r as f64)
        })
        .unzip();
    let flare = Bulge {
        at: 0.0,
        width: FLARE.1 * (RADIUS * length) as f64,
        height: FLARE.0,
    };
    sheets.push(strand_mesh(&Strand {
        bulges: vec![flare],
        ..Strand::new(points, radii, SIDES, RINGS * detail, Bark::none())
    })?);
    Ok(merge_meshes(&sheets))
}

/// Builds one mesh per distinct leaf and gives it to every leaf that drew it.
fn build(
    mut commands: Commands,
    mut library: Library,
    scene: Res<SceneParams>,
    params: Res<LeafParams>,
    leaves: Query<(Entity, &Order, &LeafConfig)>,
) -> Result<()> {
    library.clear(PART);

    let mut hung: Vec<(Order, Entity, LeafConfig)> = leaves
        .iter()
        .map(|(entity, order, config)| (*order, entity, *config))
        .collect();
    hung.sort_by_key(|(order, ..)| *order);

    let configs: Vec<LeafConfig> = hung.iter().map(|(_, _, config)| *config).collect();
    let book = farthest_first(
        &configs,
        params.variations.max(1) as usize,
        0.0,
        &LeafMetric,
    );
    let meshes = par_map(&book.representatives, |_, config| {
        leaf_mesh(config, &params)
    });
    let mut geometry = Vec::with_capacity(meshes.len());
    for (index, mesh) in meshes.into_iter().enumerate() {
        let color = shade(
            srgb(COLOR),
            &mut Rng::new(scene.seed ^ COLOR_STREAM ^ salt(index as u64)),
        );
        geometry.push(library.part(PART, index, mesh?.to_mesh(), RESPONSE.blade(color)));
    }

    // A leaf has nothing hanging off it, so it is the geometry prim itself.
    for ((_, entity, _), drew) in hung.iter().zip(&book.assignment) {
        commands
            .entity(*entity)
            .insert(geometry[*drew as usize].clone());
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every leaf the defaults call for builds at the coarsest, the default
    /// and the finest detail the panel offers, hangs from the stem's axis by
    /// its flared petiole and reaches about its length; it costs fewer
    /// triangles the coarser it is, and stays inside a budget at the default;
    /// and a young leaf at the head is shorter, more upright and less
    /// drooping than an old one, on average over the seeds.
    #[test]
    fn a_leaf_hangs_from_its_node_and_settles_as_it_ages() {
        let params = LeafParams::default();
        let details = [0.1, params.detail, 4.0];
        let mut triangles = [0; 3];
        let (mut young, mut old) = ([0.0; 4], [0.0; 4]);
        for seed in 0..24 {
            for rank in [0, 4, 8, 20] {
                let config = LeafConfig::new(&params, seed, rank);
                for (sum, at) in [(&mut young, 0), (&mut old, 20)] {
                    if rank == at {
                        sum[0] += config.length;
                        sum[1] += config.angle;
                        sum[2] += config.droop;
                    }
                }
                for (k, detail) in details.into_iter().enumerate() {
                    let mesh = leaf_mesh(
                        &config,
                        &LeafParams {
                            detail,
                            ..params.clone()
                        },
                    )
                    .unwrap();
                    let (near, far) =
                        mesh.points
                            .iter()
                            .fold((f32::MAX, 0.0f32), |(near, far), p| {
                                let d = Vec3::from(*p).length();
                                (near.min(d), far.max(d))
                            });
                    let at = format!("seed {seed}, rank {rank}, detail {detail}");
                    let flared = RADIUS * config.length * (1.0 + FLARE.0 as f32);
                    assert!(near <= flared + 1e-5, "{at}: {near} m off the axis");
                    assert!(
                        (0.75..=1.0).contains(&(far / config.length)),
                        "{at}: reaches {far} m of {}",
                        config.length
                    );
                    let count = mesh.to_mesh().indices().unwrap().len() / 3;
                    if detail == params.detail {
                        assert!(count < 3000, "{at}: {count} triangles");
                        let wide = mesh.points.iter().fold(0.0f32, |m, p| m.max(p[1].abs()));
                        for (sum, at) in [(&mut young, 0), (&mut old, 20)] {
                            if rank == at {
                                sum[3] += wide / config.length;
                            }
                        }
                    }
                    triangles[k] += count;
                }
            }
        }
        assert!(
            triangles[0] < triangles[1] && triangles[1] < triangles[2],
            "triangles at {details:?}: {triangles:?}"
        );
        assert!(
            (0..4).all(|i| young[i] < old[i]),
            "length, angle, droop and width: young {young:?}, old {old:?}"
        );
    }

    /// The relief, on one lateral leaflet held level: flat with nothing
    /// asked of it; puckered up between its veins, never down, and level on
    /// the midrib with the pucker alone; its halves turned up by the fold;
    /// folded harder, and puckered, at the head than once grown; and at the
    /// params' extremes, whatever the preset or the age, nothing crossing
    /// from one half into the other.
    #[test]
    fn a_leaflet_puckers_folds_and_opens_as_it_grows() {
        let params = LeafParams::default();
        let sheet = |shape: &Leaflet, relief: &Relief| {
            sheet(shape, 0.15, 0.0, relief, 1.0, &mut Rng::new(3)).unwrap()
        };
        let none = Relief {
            fold: 0.0,
            twist: 0.0,
            pleat: 0.0,
            ruffle: 0.0,
        };
        let height = |mesh: &MeshData| mesh.points.iter().fold(0.0f32, |m, p| m.max(p[2]));

        let flat = sheet(&Leaflet::LATERAL, &none);
        assert!(flat.points.iter().all(|p| p[2].abs() < 1e-6), "flat");

        let puckered = sheet(&Leaflet::LATERAL, &Relief { pleat: 0.3, ..none });
        assert!(puckered.points.iter().all(|p| p[2] > -1e-6), "puckers up");
        assert!(
            height(&puckered) > 0.002,
            "puckers: {} m",
            height(&puckered)
        );
        assert!(
            puckered
                .points
                .iter()
                .filter(|p| p[1].abs() < 1e-6)
                .all(|p| p[2].abs() < 1e-6),
            "level on the midrib"
        );

        let folded = sheet(&Leaflet::LATERAL, &Relief { fold: 0.5, ..none });
        assert!(
            folded
                .points
                .iter()
                .all(|p| (p[2] - p[1].abs() * 0.5f32.tan()).abs() < 1e-5),
            "either half turned up by the fold"
        );

        // Every preset, from the bud to grown, at the params' extremes with
        // all four layers on: a vertex comes at most halfway to the midrib's
        // plane, so nothing crosses into the other half.
        for fold in [-0.5, 1.0] {
            let extreme = LeafParams {
                curl: 2.0,
                fold,
                asymmetry: 1.0,
                ..params.clone()
            };
            for preset in [Leaflet::TERMINAL, Leaflet::LATERAL, Leaflet::INTERCALARY] {
                for grown in [0.0, 0.15, 0.25, 0.5, 1.0] {
                    let shape = Leaflet { grown, ..preset };
                    let relief = Relief::new(&extreme, grown);
                    let bare = sheet(
                        &shape,
                        &Relief {
                            pleat: 0.0,
                            ruffle: 0.0,
                            ..relief
                        },
                    );
                    let bent = sheet(&shape, &relief);
                    // Of its distance from the midrib's plane, what a vertex keeps.
                    let kept = bare
                        .points
                        .iter()
                        .zip(&bent.points)
                        .filter(|(p, _)| p[1].abs() > 1e-6)
                        .map(|(p, q)| q[1] / p[1])
                        .fold(f32::MAX, f32::min);
                    assert!(
                        kept > 0.49,
                        "{} lobes, fold {fold}, grown {grown}: a vertex keeps only {kept}",
                        preset.lobes
                    );
                }
            }
        }

        let (head, grown) = (Relief::new(&params, 0.0), Relief::new(&params, 1.0));
        assert!(head.fold > grown.fold && head.pleat > grown.pleat && head.ruffle == 0.0);
        assert!(head.fold <= SHUT, "the halves meet and go no further");
        assert!(grown.fold == params.fold as f64 && grown.pleat == PLEAT);
    }
}

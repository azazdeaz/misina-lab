//! Leaf element — one blade of the canopy.
//!
//! A grapevine leaf is a broad palmate blade on a stalk: five lobes around a
//! sinus where the stalk meets it, and a toothed margin the whole way round.
//! That outline is what makes a vineyard read as a vineyard from any distance
//! at which the wood has stopped being visible. It is built from the leaf's
//! veins by [`Blade`], at the shape of whatever stage of growth the leaf has
//! reached, and filled with triangles here, the midrib and the main veins
//! kept as lines of edges through the fill so the blade can fold on them.
//!
//! The outline is complete with the **petiole**, the stalk a leaf hangs by,
//! so a leaf is one piece from the stalk's free end to the blade's apex.
//!
//! # Local frame
//!
//! A leaf is built **about the XY plane**, with the petiole's free end — the
//! point at which it joins a shoot — on the **origin**, and the leaf running
//! along **+X**, front face toward +Z:
//!
//! ```text
//!   +Y                       ╭──╮  ╭──╮
//!    ↑                      ╱    ╲╱    ╲
//!    ·──────────────────────┤  the blade ├──→ +X
//!    ↑     the petiole      ╲    ╱╲    ╱
//!  origin                    ╰──╯  ╰──╯
//! ```
//!
//! Same convention a [`shoot`](super::shoot) leaves its bud with, and for the
//! same reason: whatever ends up placing leaves picks a point and a direction,
//! and everything else follows without the placer knowing how a leaf was
//! cut. The outline is flat and the blade is not: [`curl`] bends the filled
//! sheet along the veins the outline was built from, folded on its midrib,
//! pleated on its main veins, arching down off the petiole's top and
//! ruffled at its margin, then turns the whole blade down off the petiole at
//! the junction — see [`LeafConfig::kink`]. The petiole stays straight along
//! +X.
//!
//! # The layer
//!
//! The last one: a leaf has nothing hanging off it, so this element builds
//! meshes and expands into nothing. [`shoot`](super::shoot) authors a
//! [`LeafConfig`] on every node it offers, saying how far the leaf there has
//! grown and which way it leans, and [`build`] turns the distinct configs
//! into blades — each one built at the area [`LeafParams::area`] asks of a
//! full-grown leaf, whatever its stage.
//!
//! With no children, a leaf is a geometry prim in its own right rather than an
//! `Xform` over one, so it carries its [`Geometry`] directly. At six figures of
//! them that is half the prims in the scene. Each also carries, for ground
//! truth, its `area`, the area of its blade as placed in m², petiole left
//! out, and its `maturity`.
//!
//! Nothing here decides where a leaf hangs or how big it ends up; that belongs
//! to the shoot it hangs from, and the fixed area is what lets that shoot
//! express size as a scale. What a leaf's age changes *here* is everything
//! but its size: a leaf just unfolded is narrower and coarser toothed than
//! one full grown, with little blade yet at its base, folded up along its
//! veins, held in line with its petiole, and a lighter yellow-green.

use std::f64::consts::{PI, TAU};

// Named over the prelude's `ContextExt`, Bevy's own `context`, which would
// make every call ambiguous.
use anyhow::Context as ContextExt;
use bevy::math::DVec2;
use bevy::prelude::*;

use super::util::blade::{Blade, Veins};
use super::util::{color, material, par_map};
use super::{Grow, Rng};
use misina_lab::geometry::mesh::{MeshData, bend};
use misina_lab::geometry::outline::{creased_mesh, spacing};
use misina_lab::params::Slider;
use misina_lab::quantize::{Metric, farthest_first};
use misina_lab::scene::{Geometry, Library, Order, Surface, UsdAttributes, configs_changed};

/// The mesh-library prefix this element registers its blades under.
pub const PART: &str = "Leaf";

/// How far apart two blades are per unit between them of each thing a mesh
/// is cut from: the growth stage, which reshapes the whole outline; how
/// deeply it lobes; how far it [`curl`]s; and how lopsided it is. In that
/// order of weight, so a budget goes to telling a young leaf from a grown
/// one before it goes to two curls of one shape — see [`LeafMetric`].
const MATURITY_APART: f32 = 1.0;
const LOBING_APART: f32 = 0.5;
const CURL_APART: f32 = 0.1;
const ASYMMETRY_APART: f32 = 0.15;

/// One blade's shape, as the shoot that hangs it specified.
///
/// No size: every blade is built at the area of a full-grown leaf, so how
/// big a leaf ends up is the scale it is placed at and never a fact about
/// the mesh.
#[derive(Component, Clone, Copy, Debug, PartialEq)]
pub struct LeafConfig {
    /// How far the leaf has grown, from 0 just unfolded to 1 full size: what
    /// shape its blade is cut to, how far it folds and how light its green
    /// is. Not its size, which is the scale the shoot places it at.
    pub maturity: f32,
    /// How deeply the blade lobes, 0 entire to 1 deeply cut — see
    /// [`Blade::lobing`].
    pub lobing: f32,
    pub detail: u32,
    /// How far this blade bends out of the flat outline — see [`curl`].
    /// Signed: a negative one cups, its lobes lifting, where a positive one
    /// droops.
    pub curl: f32,
    /// How far the blade folds up along its midrib once grown — see
    /// [`LeafParams::fold`].
    pub fold: f32,
    /// How lopsided the blade is and which way, from -1 to 1 — see
    /// [`Blade::asymmetry`].
    pub asymmetry: f32,
    /// How far a grown blade bends down off its petiole, in radians — see
    /// [`LeafParams::blade_angle`].
    pub blade_angle: f32,
    /// The area of a full-grown leaf, in m², which every blade is built at.
    pub area: f32,
    /// The unit draws [`lobing`](Self::lobing), [`curl`](Self::curl) and
    /// [`asymmetry`](Self::asymmetry) were spread from, kept so the params
    /// can be re-applied to a leaf already hanging; `lean` is signed.
    ///
    /// `f64` rather than `f32` to match the stream they came out of: rounding
    /// one re-cuts a blade already hanging to a slightly different shape, and
    /// the exported bytes with it.
    ///
    /// Not shape: the fields above are what a mesh is built from, and
    /// [`LeafMetric`] reads these no more than a mesh does.
    pub shape: f64,
    pub draw: f64,
    pub lean: f64,
}

/// How far a leaf's own curl strays from what [`LeafParams::curl`] asks for.
///
/// Lopsided and signed. Most blades curl the way [`curl`] describes and by
/// about what was asked for, a few come out nearly flat, and a few bend back
/// the other way — which happens on a real vine wherever a leaf has rolled its
/// margins up to the light rather than shed off them.
const CURL_SPREAD: std::ops::Range<f64> = -0.4..1.3;

/// How far a leaf's own lobing strays either side of what
/// [`LeafParams::lobing`] asks for. The leaves of one vine are not all cut
/// alike.
const LOBING_SPREAD: f64 = 0.2;

impl LeafConfig {
    /// The blade these params call for on a leaf `maturity` of the way
    /// grown, cut by `shape`, curled by `draw` and leaning by `lean` — unit
    /// draws, the first two spread through [`LOBING_SPREAD`] and
    /// [`CURL_SPREAD`], and the last signed, from -1 to 1, scaled by
    /// [`LeafParams::asymmetry`].
    ///
    /// The draws are the caller's because the shape and the curl are *per
    /// leaf*: a canopy of one blade repeated reads as a printed pattern,
    /// however good the blade is. Which way a leaf leans is the caller's in
    /// particular, because it follows the side of the shoot the leaf is on.
    pub fn new(params: &LeafParams, maturity: f32, shape: f64, draw: f64, lean: f64) -> Self {
        let (shape, draw) = (shape.clamp(0.0, 1.0), draw.clamp(0.0, 1.0));
        let lean = lean.clamp(-1.0, 1.0);
        let lobing = params.lobing as f64 + LOBING_SPREAD * (2.0 * shape - 1.0);
        let spread = CURL_SPREAD.start + draw * (CURL_SPREAD.end - CURL_SPREAD.start);
        Self {
            maturity: maturity.clamp(0.0, 1.0),
            lobing: lobing.clamp(0.0, 1.0) as f32,
            detail: params.detail.max(1),
            curl: params.curl.max(0.0) * spread as f32,
            fold: params.fold,
            asymmetry: params.asymmetry.clamp(0.0, 1.0) * lean as f32,
            blade_angle: params.blade_angle,
            area: params.area.max(1e-6),
            shape,
            draw,
            lean,
        }
    }

    /// How far the blade is turned down off its petiole at the junction, in
    /// radians: all of [`blade_angle`](Self::blade_angle) on a grown leaf,
    /// and less the younger it is, a leaf just unfolded holding its blade
    /// out in line with the stalk. Whatever places a leaf turns its petiole
    /// up by this to hang the blade at the pitch it means.
    pub fn kink(&self) -> f32 {
        self.blade_angle * self.maturity
    }
}

/// Two blades share a mesh when they are at about the same stage of growth,
/// lobed about as deeply, cut at about the same resolution, curled about as
/// far and leaning about as far the same way.
pub struct LeafMetric;

impl Metric<LeafConfig> for LeafMetric {
    fn distance(&self, a: &LeafConfig, b: &LeafConfig) -> f32 {
        let maturity = (a.maturity - b.maturity) * MATURITY_APART;
        let lobing = (a.lobing - b.lobing) * LOBING_APART;
        // Weighted to stay well under the other axes across the whole range
        // `detail` can be set to, so resolution never outbids shape.
        let detail = (a.detail as f32 - b.detail as f32) * 0.001;
        // Weighted so that the whole of [`CURL_SPREAD`] at the default
        // [`LeafParams::curl`] comes to a fifth of the span of the growth
        // stages: a leaf takes a blade cut for its age, curled as close as
        // the budget got, before one of another age curled closer.
        let curl = (a.curl - b.curl) * CURL_APART;
        // Weighted under the curl, and signed, so the budget keeps blades
        // leaning either way for the two sides of a shoot.
        let asymmetry = (a.asymmetry - b.asymmetry) * ASYMMETRY_APART;
        (maturity * maturity
            + lobing * lobing
            + detail * detail
            + curl * curl
            + asymmetry * asymmetry)
            .sqrt()
    }
}

// ─── Params ─────────────────────────────────────────────────────────

/// One blade of the canopy.
///
/// A blade is built from its veins, five lobes with a toothed margin, cut
/// fresh for every mesh the budget keeps; `lobing` sets how deeply. Every
/// blade is built at `area`, a full-grown leaf's, and a leaf's size comes
/// from the scale it is placed at. Where leaves hang, how big each one ends
/// up and how far it has grown is the shoot's; a leaf still growing near a
/// shoot's tip is cut narrower, open at its base, folded and lighter green
/// than it will be.
#[derive(Resource, Reflect, Clone, Debug, PartialEq)]
#[cfg_attr(
    feature = "python",
    pyo3::pyclass(get_all, set_all, skip_from_py_object)
)]
pub struct LeafParams {
    /// How deeply a full-grown blade lobes: 0 is an entire, heart-shaped
    /// leaf, 1 is cut deep into five lobes. Each leaf draws a little more or
    /// less.
    #[reflect(@Slider { min: 0.0, max: 1.0, step: 0.05 })]
    pub lobing: f32,
    /// How far a grown blade bends out of the flat shape it was cut as: its
    /// veins arching down off the petiole's top, the blade standing up a
    /// little between them, and its margin ruffled, all scaled together. The
    /// middle of a spread, with every leaf drawing its own share of it, and
    /// some cupping instead, their lobes lifting. Zero leaves a grown blade
    /// flat but for its fold.
    ///
    /// A multiplier on the curl at [`curl`]'s own constants, past the
    /// [`fold`](Self::fold), which is the cultivar's. Each leaf draws
    /// its own share of it through [`CURL_SPREAD`], and how many of those the
    /// canopy can hold is [`variations`](Self::variations).
    ///
    /// What it buys is shading. A flat blade takes one light across its whole
    /// face however it is turned, and a canopy of them reads as a green wall;
    /// a curled one is a facet of a different slope between every two veins.
    #[reflect(@Slider { min: 0.0, max: 2.0, step: 0.05 })]
    pub curl: f32,
    /// How far a grown blade folds up along its midrib, in radians each side:
    /// 0 holds it flat across, more folds it into a V and less bows it over
    /// the midrib. Cultivars differ: most are flat, some fold.
    ///
    /// A leaf just unfolded is folded far harder whatever this is, its halves
    /// still closed up toward each other as they lay in the bud, and opens
    /// out as it grows.
    #[reflect(@Slider { min: -0.5, max: 0.8, step: 0.05 })]
    pub fold: f32,
    /// How lopsided a blade may be: 0 for halves that are mirror images but
    /// for small differences, 1 for one half much the larger and the midrib
    /// curving away from it toward the apex. Each leaf draws its own share,
    /// and leans the other way from the leaf below it, as on a vine.
    ///
    /// At 1 a blade may lean as far as [`Blade::asymmetry`] goes, and twist
    /// up to [`TWIST`] about its midrib toward the apex besides.
    #[reflect(@Slider { min: 0.0, max: 1.0, step: 0.05 })]
    pub asymmetry: f32,
    /// How far a grown blade bends down off its petiole where they meet, in
    /// radians. A leaf just unfolded holds its blade in line with the stalk.
    ///
    /// The shoot raises each petiole by as much, so the blade still hangs at
    /// the shoot's `leaf_droop`: this splits a leaf's posture between a
    /// petiole held up and out and a blade hanging off its end.
    #[reflect(@Slider { min: 0.0, max: 1.5, step: 0.05 })]
    pub blade_angle: f32,
    /// The area of a full-grown leaf's blade and petiole, in m². A *Vitis
    /// vinifera* leaf runs 100 to 200 cm², by cultivar and vigour.
    ///
    /// Every blade is built at it, whatever its stage, and the shoot places
    /// each at the scale of how far it has grown, so a scale is the size of a
    /// full-grown leaf whatever shape the blade was cut to.
    #[reflect(@Slider { min: 0.005, max: 0.04, step: 0.001 })]
    pub area: f32,
    /// How many distinct blade meshes the scene may hold. A budget, not a
    /// count: the leaves are clustered by growth stage, lobing, curl and
    /// lean, and this is how many representatives the clustering may keep.
    #[reflect(@Slider { min: 1.0, max: 100.0, step: 1.0 })]
    pub variations: u32,
    /// How finely a blade is meshed, as the number of triangles its area is
    /// cut into. The margin follows it: coarser drops the teeth first, then
    /// rounds off the sinuses, and keeps the lobes.
    ///
    /// A floor rather than an exact count: the margin is honoured as cut, and
    /// tiling that alone costs about one triangle per point of the outline
    /// before this is consulted at all. What it buys past that is vertices
    /// across the *middle* of the blade, which has none of its own and is
    /// what [`curl`] bends. The midrib and main veins are kept as edges as
    /// fine as the rest, which costs about a tenth more again, and the
    /// default lands a blade at roughly 250 triangles.
    ///
    /// It is also how fine the ruffle runs: no wave is shorter than
    /// [`RESOLVE`] triangles, so a coarse blade ruffles in long, gentle waves
    /// rather than in noise, and a fine one in shorter waves down to the
    /// shortest of [`RUFFLE_WAVELENGTH`]. The folds are on edges and sharp at
    /// any detail.
    #[reflect(@Slider { min: 8.0, max: 400.0, step: 8.0 })]
    pub detail: u32,
}

impl Default for LeafParams {
    fn default() -> Self {
        Self {
            lobing: 0.5,
            curl: 1.0,
            fold: 0.15,
            asymmetry: 0.5,
            blade_angle: 0.8,
            area: 0.015,
            // Enough that a run of leaves along one shoot never repeats, and
            // forty blades is nothing to hold.
            variations: 40,
            detail: 120,
        }
    }
}

pub fn plugin(app: &mut App) {
    app.init_resource::<LeafParams>().add_systems(
        PreUpdate,
        (
            reauthor.run_if(resource_changed::<LeafParams>),
            build.run_if(configs_changed::<LeafConfig>.or_eager(resource_changed::<LeafParams>)),
        )
            .chain()
            .in_set(Grow::Scatter),
    );
}

/// Re-cuts every blade already hanging, in place.
///
/// [`shoot`](super::shoot) authors these configs, but re-running that layer to
/// change two numbers on them would despawn and respawn the whole canopy — six
/// figures of entities for an edit that moves no leaf. Each blade keeps the
/// draws it was spread from so the params can be applied again without it.
fn reauthor(params: Res<LeafParams>, mut leaves: Query<&mut LeafConfig>) {
    for mut config in &mut leaves {
        let next = LeafConfig::new(
            &params,
            config.maturity,
            config.shape,
            config.draw,
            config.lean,
        );
        config.set_if_neq(next);
    }
}

// ─── Shape ──────────────────────────────────────────────────────────

/// The stream a blade's margin is cut from, salted with its growth stage,
/// its lobing and its lean, so every mesh the budget keeps is cut to a margin
/// of its own; not with its curl, so turning the curl up bends a blade
/// without re-cutting it. The mesh stays a function of the config — equal configs, equal blade —
/// which is all the library asks of it.
const SHAPE_STREAM: u64 = 0x9E37_79B9_7F4A_7C15;

/// The stream a blade's ruffle is drawn from, so that turning the curl up does
/// not re-roll the colours — see [`Rng`].
///
/// Salted with the blade's *curl* as well as its shape, so that two leaves
/// that curl by different amounts also ruffle differently rather than being
/// one wave pattern at two depths.
const CURL_STREAM: u64 = 0x4F1B_A6D0_37C2_9E85;

/// How ragged a blade's margin is — see [`Blade::jitter`].
const JITTER: f64 = 0.6;

/// How far the margin may stray from the curves it is cut from, as a
/// fraction of the side of an interior triangle at [`LeafParams::detail`]:
/// one knob meshes the margin and the middle alike.
const MARGIN: f64 = 0.1;

/// How far a grown blade curls at [`LeafParams::curl`] = 1, in units of its
/// midrib, so one setting reads the same on a long blade and a round one:
/// how far a vein turns arching down over a midrib's length, in radians;
/// the slope the blade stands up at off each vein either side; and how
/// steep the ruffle gets.
const DROOP: f64 = 0.6;
const PLEAT: f64 = 0.1;
const RUFFLE: f64 = 0.6;

/// A leaf just unfolded, past what a grown one does: how far its halves fold
/// up along the midrib, in radians each side, and the slope it stands up at
/// between its veins. In the bud a blade lies folded in half with its upper
/// faces together and every main vein a valley, and it opens out as it
/// expands, all the way by the time it is [`OPEN`] grown: the top three or
/// four leaves of a shoot.
const YOUNG_FOLD: f64 = 1.4;
const YOUNG_PLEAT: f64 = 0.45;
const OPEN: f64 = 0.5;

/// How far a grown blade as lopsided as [`LeafParams::asymmetry`] lets it
/// be twists about its midrib, in radians from the junction to the apex. A
/// younger one twists less, still held in its fold.
const TWIST: f64 = 0.5;

/// How far off the midrib the ruffle reaches its full depth, as a fraction
/// of the midrib: the midrib is stiff, and it is where the blade folds.
const CLEAR: f64 = 0.15;

/// Sinusoids summed into the ruffle, and the wavelengths they are drawn from
/// as fractions of the midrib, the shortest only where the mesh resolves it:
/// a wave is at least [`RESOLVE`] triangles long.
///
/// Three waves read as irregular at the distance a leaf is seen from.
const RUFFLE_WAVES: usize = 3;
const RUFFLE_WAVELENGTH: std::ops::Range<f64> = 0.25..1.4;
const RESOLVE: f64 = 4.0;

/// The stream a blade's margin and its green are drawn from.
fn seed(config: &LeafConfig) -> u64 {
    SHAPE_STREAM
        ^ config.maturity.to_bits() as u64
        ^ ((config.lobing.to_bits() as u64) << 32)
        ^ (config.asymmetry.to_bits() as u64).rotate_left(16)
}

/// One blade, in its own local frame, at the config's area, and how much of
/// that area is blade rather than petiole.
fn cut(config: &LeafConfig) -> anyhow::Result<(MeshData, f64)> {
    let area = config.area as f64;
    let (mut mesh, veins) = flat(config)?;
    curl(&mut mesh, &veins, config);

    // A curled blade is within a couple of percent of the area it was cut at:
    // the ruffle and the pleats stretch the sheet, and the droop packs the
    // blade between two veins in toward the margin. Scaling holds the area
    // exactly through all of it — as the area of the *surface*, which is the
    // leaf's own measure of itself, rather than of the shadow it casts. About
    // the origin, so the attachment stays on it.
    let scale = (area / mesh.surface_area()).sqrt();
    for point in &mut mesh.points {
        *point = point.map(|c| c * scale as f32);
    }
    // The petiole is the strip from the origin to the junction, and no part
    // of the curl touches it.
    let petiole = veins.junction.x * veins.stalk * scale * scale;
    Ok((mesh, area - petiole))
}

/// One blade's outline, filled flat at the config's area with its veins kept
/// as edges, and the veins it was built from.
fn flat(config: &LeafConfig) -> anyhow::Result<(MeshData, Veins)> {
    let area = config.area as f64;
    let triangle = area / config.detail as f64;
    let blade = Blade {
        lobing: config.lobing as f64,
        maturity: config.maturity as f64,
        jitter: JITTER,
        asymmetry: config.asymmetry as f64,
    };
    let (outline, veins) =
        blade.outline(area, MARGIN * triangle.sqrt(), &mut Rng::new(seed(config)));
    Ok((creased_mesh(&outline, &veins.creases(), triangle)?, veins))
}

/// Bends a flat blade into a curled one along its `veins`, in place.
///
/// The petiole is left as it is, straight along +X with its root on the
/// origin. The blade, in five layers:
///
/// * a **ruffle**, deepening toward the margin. A real edge buckles because
///   it grows more material than the middle can hold flat, so a blade still
///   folded from the bud has hardly any.
/// * **pleats** along the main veins, the blade standing up off every one
///   to a crest between it and the next: a cone over the junction, each
///   vein a valley in it as it was a fold in the bud. Hard on a leaf just
///   unfolded, faint on a grown one.
/// * a **fold** on the midrib, either half turned up about it: the bud's
///   fold, opening as the leaf grows, to the cultivar's
///   [`LeafParams::fold`] once grown, and twisting toward the apex on a
///   lopsided blade.
/// * a **droop**, every vein arching down off the junction where the
///   petiole leaves it straight. An arc anchored there keeps the *tangent*
///   as well as the point, so the blade leaves the petiole level and curves
///   away further out. Its weight is what bends it, so a young blade barely
///   droops.
/// * the **kink**, the whole blade turned down off the petiole about the
///   junction — see [`LeafConfig::kink`].
///
/// The midrib and the main veins are edges of the mesh, so the fold and the
/// pleats crease on them; the ruffle is kept off the midrib so the fold
/// there stays clean. The droop and the kink are *bends* rather than
/// displacements — see [`bend`] — and the ruffle is a sum of sinusoids
/// rather than sampled noise: the blade is twenty centimeters of domain with
/// three wavelengths on it, which is a handful of sines and not worth a
/// lattice.
fn curl(mesh: &mut MeshData, veins: &Veins, config: &LeafConfig) {
    let (amount, grown) = (config.curl as f64, config.maturity.clamp(0.0, 1.0) as f64);
    let midrib = veins.midrib;
    if midrib <= 0.0 {
        return;
    }
    // What is left of the bud's fold.
    let bud = (1.0 - grown / OPEN).max(0.0).powi(2);
    let fold = config.fold as f64 + YOUNG_FOLD * bud;
    let pleat = PLEAT * amount + YOUNG_PLEAT * bud;
    let droop = DROOP * amount * grown / midrib;
    let (sin_kink, cos_kink) = (config.kink() as f64).sin_cos();

    let mut rng = Rng::new(CURL_STREAM ^ seed(config) ^ config.curl.to_bits() as u64);
    // As far as the blade is lopsided and grown, and either way.
    let twist =
        TWIST * grown * config.asymmetry.abs() as f64 * if rng.unit() < 0.5 { -1.0 } else { 1.0 };
    // A heading, a wavenumber, a phase and a depth per wave, each as deep as
    // it is long so every one is as steep.
    let side = spacing(config.area as f64 / config.detail.max(1) as f64);
    let shortest = RUFFLE_WAVELENGTH.start.max(RESOLVE * side / midrib);
    let longest = RUFFLE_WAVELENGTH.end.max(shortest);
    let waves: Vec<[f64; 4]> = (0..RUFFLE_WAVES)
        .map(|_| {
            let heading = rng.range(0.0, PI);
            let k = TAU / (rng.range(shortest, longest) * midrib);
            let depth = RUFFLE * amount * grown / (RUFFLE_WAVES as f64 * k);
            [
                k * heading.cos(),
                k * heading.sin(),
                rng.range(0.0, TAU),
                depth,
            ]
        })
        .collect();

    for point in &mut mesh.points {
        let flat = DVec2::new(point[0] as f64, point[1] as f64);
        let at = veins.bearing(flat);
        if !at.blade {
            continue;
        }
        let across = veins.across(flat);

        let margin = (at.radius / at.reach).min(1.0).powi(3)
            * (across.offset.abs() / (CLEAR * midrib)).min(1.0);
        let ruffle: f64 = waves
            .iter()
            .map(|[kx, ky, phase, depth]| depth * (kx * flat.x + ky * flat.y + phase).sin())
            .sum();
        let z = margin * ruffle + pleat * at.radius * at.ridge;

        // Either half turned up about the midrib, carrying its pleats and
        // ruffle with it.
        let half = if across.offset >= 0.0 { 1.0 } else { -1.0 };
        let turn = fold + half * twist * (across.along / midrib).clamp(0.0, 1.0);
        let (sin, cos) = turn.sin_cos();
        let offset = across.offset * cos - half * z * sin;
        let z = across.offset.abs() * sin + z * cos;
        let u = across.foot + across.normal * offset - veins.junction;

        // The straight line out from the junction to the point, as the fold
        // and the pleats have tilted it, bent into an arc of the same length:
        // a vein keeps its length as it droops, and the blade between two
        // of them packs in toward the margin.
        let (out, length) = (u.length(), u.length().hypot(z));
        let (u, z) = if droop == 0.0 || out == 0.0 {
            (u, z)
        } else {
            let (along, down) = bend(length, 0.0, droop);
            let (sin, cos) = (z / length, out / length);
            (
                u * ((along * cos - down * sin) / out),
                along * sin + down * cos,
            )
        };

        // Turned down about the junction, tip first.
        let (x, z) = (u.x * cos_kink + z * sin_kink, z * cos_kink - u.x * sin_kink);
        *point = [
            (veins.junction.x + x) as f32,
            (veins.junction.y + u.y) as f32,
            z as f32,
        ];
    }
}

// ─── Building ───────────────────────────────────────────────────────

/// Builds one mesh per distinct blade and gives it to every leaf that drew it.
///
/// The end of the pipeline: a leaf has no children, so it carries its geometry
/// directly and expands into nothing.
pub(crate) fn build(
    mut commands: Commands,
    mut library: Library,
    params: Res<LeafParams>,
    leaves: Query<(Entity, &Order, &LeafConfig, &Transform)>,
) -> Result<()> {
    library.clear(PART);

    let mut hung: Vec<(Order, Entity, LeafConfig, f32)> = leaves
        .iter()
        .map(|(entity, order, config, placed)| (*order, entity, *config, placed.scale.x))
        .collect();
    hung.sort_by_key(|(order, ..)| *order);

    let configs: Vec<LeafConfig> = hung.iter().map(|(_, _, config, _)| *config).collect();
    let book = farthest_first(
        &configs,
        params.variations.max(1) as usize,
        0.0,
        &LeafMetric,
    );

    // The representatives are independent of each other, so they are cut in
    // parallel and registered serially: `Assets<Mesh>` takes one writer.
    let blades = par_map(&book.representatives, |_, config| {
        cut(config)
            .map(|(mesh, lamina)| (mesh.to_mesh(), lamina))
            .with_context(|| format!("a leaf could not be built from {config:?}"))
    });

    let mut geometry: Vec<(Geometry, f64)> = Vec::with_capacity(book.len());
    for (index, (config, blade)) in book.representatives.iter().zip(blades).enumerate() {
        let (mesh, lamina) = blade?;
        geometry.push((library.part(PART, index, mesh, surface(config)), lamina));
    }

    // The area each leaf's blade covers as placed, its parents unscaled.
    for ((_, entity, config, scale), drew) in hung.iter().zip(&book.assignment) {
        let (part, lamina) = &geometry[*drew as usize];
        commands.entity(*entity).insert((
            part.clone(),
            UsdAttributes::default()
                .with("area", (lamina * (*scale as f64).powi(2)) as f32)
                .with("maturity", config.maturity),
        ));
    }
    Ok(())
}

/// A blade's green: the yellow-green flush of a leaf just unfolded, darkening
/// to a grown leaf's as it grows, shaded off the stream its margin was cut
/// from rather than off which representative it happens to be, so one config
/// is one green however the budget is spent.
fn surface(config: &LeafConfig) -> Surface {
    let young = 1.0 - config.maturity.clamp(0.0, 1.0);
    let base = color::mix(
        color::srgb(color::LEAF),
        color::srgb(color::YOUNG_LEAF),
        young,
    );
    material::FOLIAGE.blade(color::shade(
        base,
        &mut Rng::new(color::COLOR_STREAM ^ seed(config)),
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::elements::VineyardParams;
    use crate::elements::util::testing::{self, bounds, face_normal, faces, organs};
    use misina_lab::scene::Prototypes;

    fn params() -> LeafParams {
        LeafParams::default()
    }

    fn blade_mesh(config: &LeafConfig) -> anyhow::Result<MeshData> {
        cut(config).map(|(mesh, _)| mesh)
    }

    /// The growth stages a shoot hangs, from the youngest leaf at its tip to
    /// a grown one.
    const STAGES: [f32; 4] = [0.15, 0.4, 0.7, 1.0];

    /// Every growth stage at three lobings, curled and leaning the way a
    /// canopy would — a fresh draw per blade.
    fn configs(params: &LeafParams) -> Vec<LeafConfig> {
        let mut rng = Rng::new(0);
        STAGES
            .iter()
            .flat_map(|&maturity| (0..3).map(move |i| (maturity, i as f64 / 2.0)))
            .map(|(maturity, shape)| {
                let (draw, lean) = (rng.unit(), rng.range(-1.0, 1.0));
                LeafConfig::new(params, maturity, shape, draw, lean)
            })
            .collect()
    }

    /// The same, held out in line with the petiole: the kink turns a blade
    /// as a whole, and the tests of how it bends in itself are clearer
    /// without it.
    fn unkinked(params: &LeafParams) -> Vec<LeafConfig> {
        configs(params)
            .into_iter()
            .map(|config| LeafConfig {
                blade_angle: 0.0,
                ..config
            })
            .collect()
    }

    fn blades(params: &LeafParams) -> Vec<MeshData> {
        configs(params)
            .iter()
            .map(|config| blade_mesh(config).unwrap_or_else(|e| panic!("{config:?}: {e:#}")))
            .collect()
    }

    /// Every growth stage at one exact curl and fold, symmetric and held in
    /// line with the petiole, for the tests that are about the curl itself
    /// rather than about what a canopy draws.
    fn curled(curl: f32, fold: f32) -> Vec<MeshData> {
        STAGES
            .iter()
            .map(|&maturity| {
                let config = LeafConfig {
                    curl,
                    fold,
                    blade_angle: 0.0,
                    ..LeafConfig::new(&params(), maturity, 0.5, 0.0, 0.0)
                };
                blade_mesh(&config).unwrap_or_else(|e| panic!("{config:?}: {e:#}"))
            })
            .collect()
    }

    /// The hardest a canopy can curl a blade, at the default params.
    fn hardest() -> f32 {
        params().curl * CURL_SPREAD.end as f32
    }

    /// Every blade a canopy asks for survives the whole path from veins to
    /// mesh: a margin that crossed itself would surface in the viewer as an
    /// author system failing mid-frame.
    #[test]
    fn every_growth_stage_builds() {
        for mesh in blades(&params()) {
            assert!(!mesh.face_vertex_counts.is_empty());
            assert!(mesh.points.iter().flatten().all(|c| c.is_finite()));
        }
    }

    /// The placement contract: hanging from the origin, running +X, and
    /// curling rather than crumpling. A leaf that drifted off any of them
    /// would hang off its shoot at an angle nobody asked for.
    #[test]
    fn a_blade_hangs_from_the_origin_and_runs_along_x() {
        for (stage, mesh) in STAGES.iter().zip(&curled(hardest(), params().fold)) {
            let (x0, x1) = bounds(mesh, 0);
            let (z0, z1) = bounds(mesh, 2);
            assert!(
                z1 - z0 < x1 * 0.5,
                "at {stage}, curls gently: {z0}..{z1} over a reach of {x1}"
            );

            assert_eq!(x0, 0.0, "at {stage}, starts at the petiole");
            assert!(x1 > 0.0, "at {stage}, reaches forward, got {x1}");

            // And straddles the axis rather than sitting to one side of it.
            let (y0, y1) = bounds(mesh, 1);
            assert!(y0 < 0.0 && y1 > 0.0, "at {stage}: {y0}..{y1}");
        }
    }

    /// Greatest distance from the leaf's axis, among the points lying in
    /// `band` of the way along it.
    fn half_width(mesh: &MeshData, band: std::ops::Range<f32>) -> f32 {
        let reach = bounds(mesh, 0).1;
        mesh.points
            .iter()
            .filter(|p| band.contains(&(p[0] / reach)))
            .fold(0.0, |m: f32, p| m.max(p[1].abs()))
    }

    /// *Which end* of a leaf sits on the origin. The outline is cut complete
    /// with its petiole, so one end of a leaf is a few millimeters across and
    /// the other is the width of a hand.
    ///
    /// Anchoring the wrong one puts every leaf on backwards — apex buried in
    /// the shoot, stalk waving in the air — and leaves a mesh that still runs
    /// along +X, still curls the way a leaf does and still covers the area
    /// asked for. No other test here can tell the difference.
    #[test]
    fn a_leaf_hangs_by_its_petiole_rather_than_by_its_tip() {
        for (config, mesh) in configs(&params()).iter().zip(&blades(&params())) {
            let stalk = half_width(mesh, 0.0..0.05);
            let blade = half_width(mesh, 0.5..0.7);
            assert!(
                stalk * 8.0 < blade,
                "{config:?}: {stalk:.4} m across at the origin, {blade:.4} m past mid-length"
            );
        }
    }

    /// The one thing every blade is asked to have in common, and the contract
    /// every later scale rests on: a prototype covers the full-grown area,
    /// whatever its stage or shape. Placing a leaf at half size has to mean half size no
    /// matter what it was cut to, and it only does if they all agree here to
    /// far better than the eye could tell them apart.
    #[test]
    fn every_variation_covers_the_same_area() {
        let areas: Vec<f32> = blades(&params())
            .iter()
            .map(|mesh| mesh.surface_area() as f32)
            .collect();
        let (lo, hi) = areas
            .iter()
            .fold((f32::MAX, f32::MIN), |(lo, hi), a| (lo.min(*a), hi.max(*a)));

        assert!(
            (hi - lo) / lo < 1e-3,
            "they agree with each other: {areas:?}"
        );
        let area = params().area;
        assert!(
            (lo - area).abs() / area < 1e-3,
            "and on {area} m²: {areas:?}"
        );
    }

    /// Greatest width of a blade, in units of the side of a square of its
    /// area: the proportion the fixed area leaves free.
    fn across(mesh: &MeshData) -> f32 {
        let (y0, y1) = bounds(mesh, 1);
        (y1 - y0) / params().area.sqrt()
    }

    /// Matched by area, not by bounding box: an entire blade and a deeply
    /// lobed one are different shapes, so their extents must *not* agree, or
    /// the lobing would be a plain scale.
    #[test]
    fn matching_the_area_leaves_the_shapes_different() {
        let widths: Vec<f32> = [0.0, 1.0]
            .map(|shape| {
                let config = LeafConfig {
                    curl: 0.0,
                    ..LeafConfig::new(&params(), 1.0, shape, 0.0, 0.0)
                };
                across(&blade_mesh(&config).unwrap())
            })
            .to_vec();
        assert!(
            widths[1] > widths[0] * 1.1,
            "same area, different proportions: {widths:?}"
        );
    }

    /// What a leaf's age changes besides its size: the youngest leaf a shoot
    /// hangs is narrower than a grown one at the same area, folds harder at
    /// the same curl, holds its blade in line with its petiole, and is a
    /// lighter green.
    #[test]
    fn a_young_leaf_is_cut_folded_and_shaded_for_its_age() {
        let config = |maturity| LeafConfig::new(&params(), maturity, 0.5, 0.75, 0.0);
        let (young, grown) = (config(STAGES[0]), config(1.0));
        let (a, b) = (blade_mesh(&young).unwrap(), blade_mesh(&grown).unwrap());

        assert!(
            across(&a) < across(&b) * 0.97,
            "narrower: {} across against {}",
            across(&a),
            across(&b)
        );

        // One flat sheet curled for either age, held in line with the
        // petiole, so only the fold separates the two.
        let (sheet, veins) = flat(&grown).unwrap();
        let fold = |maturity: f32| {
            let mut mesh = sheet.clone();
            let config = LeafConfig {
                maturity,
                blade_angle: 0.0,
                ..grown
            };
            curl(&mut mesh, &veins, &config);
            mesh.points.iter().map(|p| p[2].abs()).sum::<f32>() / mesh.points.len() as f32
        };
        assert!(
            fold(young.maturity) > fold(1.0) * 1.1,
            "folds harder: {} m out of the plane on average against {}",
            fold(young.maturity),
            fold(1.0)
        );

        assert!(
            young.kink() < grown.kink() * 0.2,
            "in line with its petiole: {} against {}",
            young.kink(),
            grown.kink()
        );

        let light = |surface: Surface| surface.color.iter().sum::<f32>();
        assert!(
            light(surface(&young)) > light(surface(&grown)) * 1.2,
            "lighter: {:?} against {:?}",
            surface(&young).color,
            surface(&grown).color
        );
    }

    /// A blade is cut from above, so its faces have to point up. USD reads
    /// counter-clockwise as front-facing by default, and the wrong winding
    /// would light the whole canopy from the wrong side.
    ///
    /// It is also the bound on the curl: [`curl`] tips every face away from
    /// +Z, and one that fell past horizontal would mean the sheet had folded
    /// over on itself rather than bent. Without the kink, which turns the
    /// blade as a whole and can fold nothing over.
    #[test]
    fn every_face_points_up() {
        for config in unkinked(&params()) {
            let mesh = &blade_mesh(&config).unwrap();
            assert!(
                faces(mesh).all(|face| face_normal(mesh, face).z > 0.0),
                "{config:?} winds counter-clockwise seen from +Z"
            );
        }
    }

    /// A grown blade hangs off its petiole at the angle asked for, turned
    /// down about the junction, and the petiole stays where the outline put
    /// it, straight along +X.
    #[test]
    fn a_grown_blade_bends_down_off_its_petiole() {
        let config = LeafConfig {
            curl: 0.0,
            fold: 0.0,
            ..LeafConfig::new(&params(), 1.0, 0.5, 0.5, 0.0)
        };
        let (sheet, veins) = flat(&config).unwrap();
        let mesh = blade_mesh(&config).unwrap();
        let scale = mesh.points[0][1] / sheet.points[0][1];
        let junction = veins.junction.x as f32 * scale;
        for (before, after) in sheet.points.iter().zip(&mesh.points) {
            if !veins
                .bearing(DVec2::new(before[0] as f64, before[1] as f64))
                .blade
            {
                assert_eq!(after[2], 0.0, "the petiole lies straight: {after:?}");
            }
        }
        let apex = mesh
            .points
            .iter()
            .max_by(|a, b| a[0].total_cmp(&b[0]))
            .unwrap();
        let pitch = (-apex[2]).atan2(apex[0] - junction);
        assert!(
            (pitch - config.kink()).abs() < 0.05,
            "the apex hangs {pitch} rad below the petiole's line, asked {}",
            config.kink()
        );
    }

    /// Every mesh the budget keeps is cut to a margin of its own: two configs
    /// are two shapes, never one blade at two curls.
    #[test]
    fn two_configs_are_two_shapes() {
        let blades = blades(&params());
        for (i, a) in blades.iter().enumerate() {
            for (j, b) in blades.iter().enumerate().skip(i + 1) {
                assert_ne!(a.points, b.points, "blades {i} and {j} are the same shape");
            }
        }
    }

    /// The curl knob, at both ends: nothing on a grown blade at zero, with
    /// no fold either, and a blade well out of the plane it was cut in at
    /// the default.
    #[test]
    fn the_curl_knob_takes_a_blade_out_of_the_drawing_plane() {
        let flat = &curled(0.0, 0.0)[3];
        assert_eq!(bounds(flat, 2), (0.0, 0.0), "at zero, the flat shape");

        let bent = &curled(params().curl, 0.0)[3];
        let (z0, z1) = bounds(bent, 2);
        let reach = bounds(bent, 0).1;
        assert!(
            z1 - z0 > reach * 0.1,
            "a curled blade stands out of the plane: {z0}..{z1} over {reach}"
        );
    }

    /// The attachment holds still. A curl that moved the petiole's root would
    /// leave every leaf in the canopy hanging off its node.
    ///
    /// Asserted as displacement *in proportion to* the distance from the root,
    /// which is the stronger claim and the one the layers are built to make:
    /// each is zero at the origin and grows outward, so nothing near the root
    /// moves at all. A curl anchored by a fade-out term instead would pass a
    /// test on the root vertex alone and still kink the petiole beside it.
    #[test]
    fn the_curl_moves_a_point_by_a_fraction_of_its_distance_from_the_root() {
        // The sheet the blade was curled from, so the fill is the same
        // vertices in the same order and only the curl separates the two.
        for &maturity in &STAGES {
            let config = LeafConfig {
                curl: hardest(),
                blade_angle: 0.0,
                ..LeafConfig::new(&params(), maturity, 0.5, 0.0, 1.0)
            };
            let (sheet, _) = flat(&config).unwrap();
            let bent = blade_mesh(&config).unwrap();
            for (before, after) in sheet.points.iter().zip(&bent.points) {
                let before = Vec3::from(*before);
                let moved = (Vec3::from(*after) - before).length();
                // A young blade's halves swing most of the way up about the
                // midrib, but nothing moves faster than its distance out.
                assert!(
                    moved <= before.length() * 0.8,
                    "at {maturity}: {before:?} moved {moved} m"
                );
            }
        }
    }

    /// The curl *bends* the sheet rather than displacing it, and this is what
    /// that buys: a blade comes out of it within a couple of percent of the
    /// size it went in at, so the rescale back to the full-grown area is a
    /// correction rather than a rescue.
    ///
    /// Displacing points in z and leaving x and y where they were — the
    /// obvious way to write this, and how a noise pass usually starts — would
    /// stretch the sheet by tens of percent at this amplitude, and every blade
    /// by a different amount, which is what the bound here catches.
    #[test]
    fn bending_a_blade_barely_stretches_it() {
        let area = params().area as f64;
        for config in configs(&params()) {
            // Both ends of the spread, so a leaf that curls the other way is
            // held to it too.
            for amount in [CURL_SPREAD.start, CURL_SPREAD.end] {
                let (mut mesh, veins) = flat(&config).unwrap();
                let curl_by = (amount * params().curl as f64) as f32;
                curl(
                    &mut mesh,
                    &veins,
                    &LeafConfig {
                        curl: curl_by,
                        ..config
                    },
                );

                let stretch = mesh.surface_area() / area - 1.0;
                assert!(
                    stretch.abs() < 0.05,
                    "{config:?} stretched {:.1}% at a curl of {amount}",
                    stretch * 100.0
                );
            }
        }
    }

    #[test]
    fn blades_are_reproducible() {
        assert_eq!(blades(&params())[0].points, blades(&params())[0].points);
    }

    /// `detail` has to reach the middle of the blade, the margin and the
    /// midrib alike: a finer blade costs more triangles, keeps more of its
    /// teeth and has more points to bend its midrib at.
    #[test]
    fn more_detail_spends_more_triangles() {
        // Triangles, and points on the midrib past the junction.
        let spent = |config: LeafConfig| {
            let (mesh, veins) = flat(&config).unwrap();
            let midrib = mesh.points.iter().filter(|p| {
                let across = veins.across(DVec2::new(p[0] as f64, p[1] as f64));
                across.along > 0.0 && across.offset.abs() < 5e-6
            });
            (mesh.face_vertex_counts.len(), midrib.count())
        };
        for config in configs(&params()) {
            let coarse = spent(LeafConfig {
                detail: 8,
                ..config
            });
            let fine = spent(LeafConfig {
                detail: 400,
                ..config
            });
            assert!(
                fine.0 > 2 * coarse.0 && fine.1 > 2 * coarse.1,
                "{config:?}: triangles and midrib points at detail 400, {fine:?}, vs at 8, {coarse:?}"
            );
        }
    }

    /// `detail` divides the blade's area, and the viewer's slider reaches its
    /// bottom stop — a leaf asked for no subdivision at all must clamp rather
    /// than divide by zero.
    #[test]
    fn a_blade_with_no_subdivision_asked_for_still_builds() {
        let mesh = blade_mesh(&LeafConfig::new(
            &LeafParams {
                detail: 0,
                ..params()
            },
            1.0,
            0.5,
            0.5,
            0.0,
        ))
        .expect("a leaf at the slider's bottom stop still builds");
        assert!(mesh.points.iter().flatten().all(|c| c.is_finite()));
        assert!(!mesh.face_vertex_counts.is_empty());
    }

    /// The budget goes to growth stages before it goes to curls or leans: a
    /// few young leaves among a shoot's worth of grown ones each keep a mesh
    /// of their own, however the grown ones curl and lean.
    ///
    /// The failure this catches is a metric that lets the curl outbid the
    /// stage — the handful of young leaves at every tip would then be drawn
    /// as grown blades scaled down, which is exactly what they are not.
    #[test]
    fn a_young_leaf_keeps_a_mesh_of_its_own_under_a_tight_budget() {
        let params = params();
        let mut rng = Rng::new(3);
        let mut canopy: Vec<LeafConfig> = (0..200)
            .map(|_| {
                let (draw, lean) = (rng.unit(), rng.range(-1.0, 1.0));
                LeafConfig::new(&params, 1.0, 0.5, draw, lean)
            })
            .collect();
        canopy.extend(
            STAGES[..3]
                .iter()
                .map(|&m| LeafConfig::new(&params, m, 0.5, 0.5, 0.0)),
        );

        let book = farthest_first(&canopy, 4, 0.0, &LeafMetric);
        let mut kept: Vec<f32> = book.representatives.iter().map(|c| c.maturity).collect();
        kept.sort_by(f32::total_cmp);
        assert_eq!(kept, STAGES, "every stage kept, got {kept:?}");
    }

    /// End to end: every leaf the shoots hung comes out carrying a blade, and
    /// the whole canopy is drawn from at most the budget's worth of meshes.
    #[test]
    fn every_hung_leaf_draws_a_blade_from_the_library() {
        let app = testing::grown(VineyardParams::default());
        let blades: Vec<(&String, &misina_lab::scene::Part)> = app
            .world()
            .resource::<Prototypes>()
            .iter()
            .filter(|(name, _)| name.starts_with(&format!("{PART}_")))
            .collect();

        assert_eq!(
            blades.len(),
            LeafParams::default().variations as usize,
            "the default budget is spent"
        );
        assert!(
            blades.iter().all(|(_, part)| part.double_sided),
            "a blade is lit from underneath too"
        );

        // One green per blade: the shade is drawn off the config, and no two
        // representatives share one.
        let mut greens: Vec<String> = blades
            .iter()
            .map(|(_, part)| format!("{:?}", part.color))
            .collect();
        greens.sort();
        greens.dedup();
        assert_eq!(greens.len(), blades.len(), "got {greens:?}");

        let mut app = app;
        let leaves = organs::<LeafConfig>(app.world_mut());
        assert!(
            leaves.len() > 1000,
            "the fixture hung leaves, got {}",
            leaves.len()
        );
        let stages: Vec<f32> = leaves.iter().map(|l| l.config.maturity).collect();
        assert!(
            stages.iter().any(|m| *m < 0.5) && stages.contains(&1.0),
            "and the shoots hung leaves still growing as well as grown ones"
        );
    }

    /// Every leaf is labelled for ground truth with its stage and with the
    /// area of the blade it is drawn with, as placed: the full-grown area at
    /// its scale, less its petiole.
    #[test]
    fn every_leaf_is_labelled_with_its_area_and_stage() {
        use misina_lab::scene::doc::AttributeValue;
        let mut app = testing::grown(VineyardParams::default());
        let area = LeafParams::default().area;
        let mut query = app
            .world_mut()
            .query::<(&LeafConfig, &Transform, &UsdAttributes)>();
        let mut labelled = 0;
        for (config, placed, labels) in query.iter(app.world()) {
            let Some(AttributeValue::Float(blade)) = labels.0.get("area") else {
                panic!("a leaf without its area: {labels:?}");
            };
            let leaf = area * placed.scale.x.powi(2);
            assert!(
                *blade < leaf && *blade > leaf * 0.95,
                "the blade's share of a leaf as placed: {blade} of {leaf} m²"
            );
            assert_eq!(
                labels.0.get("maturity"),
                Some(&AttributeValue::Float(config.maturity))
            );
            labelled += 1;
        }
        assert!(labelled > 1000, "the fixture hung leaves, got {labelled}");
    }

    /// The canopy curls every leaf for itself. One curl repeated over a
    /// hundred thousand blades is the failure this catches, and it looks
    /// perfectly good in isolation — every leaf right, the canopy printed.
    #[test]
    fn every_leaf_draws_a_curl_of_its_own() {
        let mut app = testing::grown(VineyardParams::default());
        let curls: Vec<f32> = organs::<LeafConfig>(app.world_mut())
            .iter()
            .map(|leaf| leaf.config.curl)
            .collect();

        let asked = LeafParams::default().curl;
        assert!(
            curls.iter().any(|c| *c < 0.0),
            "some blades curl back the other way"
        );
        assert!(
            curls.iter().any(|c| *c > asked),
            "and some past what the params asked for"
        );

        // Spread across the range rather than clustered at one end of it.
        let mean = curls.iter().sum::<f32>() / curls.len() as f32;
        let middle = asked * (CURL_SPREAD.start + CURL_SPREAD.end) as f32 / 2.0;
        assert!(
            (mean - middle).abs() < asked * 0.05,
            "a canopy averages the middle of the spread: {mean} against {middle}"
        );
    }

    /// A leaf has no children, so it is the geometry prim itself rather than
    /// an `Xform` over one — which is what halves the prim count of a scene
    /// that is mostly leaves.
    #[test]
    fn a_leaf_carries_its_geometry_rather_than_wrapping_it() {
        let mut app = testing::grown(VineyardParams::default());
        let mut query = app
            .world_mut()
            .query_filtered::<(Entity, Option<&Children>), With<LeafConfig>>();
        let (entity, children) = query.iter(app.world()).next().expect("some leaf was hung");

        assert!(
            children.is_none_or(|c| c.is_empty()),
            "a blade is a leaf of the tree"
        );
        assert!(
            app.world()
                .entity(entity)
                .contains::<misina_lab::scene::UsdReference>(),
            "and references its blade directly"
        );
    }

    /// The layer owns its slice of the library and rebuilds it from scratch,
    /// so a second pass must leave exactly what the first one did — and a
    /// lowered budget must leave less.
    #[test]
    fn rebuilding_rewrites_the_library_rather_than_adding_to_it() {
        let blades = |app: &App| {
            app.world()
                .resource::<Prototypes>()
                .iter()
                .filter(|(name, _)| name.starts_with(&format!("{PART}_")))
                .count()
        };

        let mut app = testing::grown(VineyardParams::default());
        assert_eq!(blades(&app), LeafParams::default().variations as usize);

        app.world_mut().resource_mut::<LeafParams>().set_changed();
        app.update();
        assert_eq!(
            blades(&app),
            LeafParams::default().variations as usize,
            "a rebuild does not double up"
        );

        app.world_mut().resource_mut::<LeafParams>().variations = 2;
        app.update();
        assert_eq!(blades(&app), 2, "and the ones it stopped using are gone");
    }
}

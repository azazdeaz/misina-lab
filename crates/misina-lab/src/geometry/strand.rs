//! Strands — the woody members plants are built from.
//!
//! A *strand* is a polyline of control points, each carrying a radius, skinned
//! into a closed tube. A trunk is a strand; so are a branch, a twig and a
//! green stem. Callers place control points where the shape is and add more
//! where the shape needs resolving; nothing here knows any botany.
//!
//! A strand may swell: all round at a [`Bulge`], a node or a graft union, and
//! on one side at a [`Collar`], where an organ's stalk leaves it. Each is given
//! by where it sits along the strand, and the skinning puts rings across it.
//!
//! A strand is ringed evenly unless it has a [`tolerance`](Strand::tolerance).
//! Then it is sampled finely and keeps only the rings that hold its mesh that
//! close to its surface: they bunch at a swelling and round a bend and spread
//! out where it runs straight, so a stem meters long can swell at every node.
//!
//! # Why the surface is hand-skinned
//!
//! curvo builds the *centerline* — [`NurbsCurve3D::interpolate`] fits a curve
//! through the control points, [`try_divide_by_count`] finds arc-length-even
//! stations along it, and [`compute_frenet_frames`] gives a frame at each. The
//! tube surface is then stitched directly from a ring per frame.
//!
//! Deliberately *not* `try_sweep`/`try_loft` + `regular_tessellate`, which is
//! how `terrain` builds its surface. `try_loft`
//! interpolates *across* the sections it is given and `regular_tessellate`
//! then re-samples the result, so the mesh's rings would not be the rings we
//! placed. That is fine for terrain, whose input is already a smooth field,
//! but here it would sand off the [`Bark`] ridges — the one detail the whole
//! module exists to produce. Skinning the rings we computed keeps them exact,
//! and costs less.
//!
//! [`try_divide_by_count`]: curvo::prelude::NurbsCurve::try_divide_by_count
//! [`compute_frenet_frames`]: curvo::prelude::NurbsCurve::compute_frenet_frames

use std::f64::consts::{PI, TAU};

use curvo::prelude::*;
use nalgebra::{Point3, Vector3};

use super::mesh::MeshData;
use crate::rng::Rng;

/// Degree of the fitted centerline, clamped down when a strand has too few
/// control points to support it (a 2-point strand is a straight line).
const DEGREE: usize = 3;

/// Rings along a tube. Three is curvo's floor, not ours: `try_divide_by_count`
/// delegates to `try_divide_by_length(total / n)`, which requires
/// `total > length` and so rejects a count of 1 outright.
const MIN_STATIONS: usize = 3;
const MAX_STATIONS: usize = 96;

/// Vertices around a ring. Below 3 the end caps are degenerate.
const MIN_SIDES: usize = 3;
const MAX_SIDES: usize = 24;

/// Control points closer together than this are treated as one. A strand
/// whose points all coincide has a zero total chord length, and curvo's
/// interpolation divides by exactly that — the whole curve comes back NaN.
const MERGE_DISTANCE: f64 = 1e-6;

/// A strand meshed to a tolerance is first sampled this far apart, in its
/// mean radii, at most this many times, and keeps a ring at least every so
/// many samples, which bounds what picking the rings costs.
const FINE: f64 = 0.5;
const MAX_FINE: usize = 8192;
const MAX_SPAN: usize = 64;

// ─── Bulges ─────────────────────────────────────────────────────────

/// A local swelling on a strand's radius.
///
/// A graft union, a branch knuckle and a node are the same shape at different
/// scales, so they are the same type. `height` is a fraction of the strand's
/// nominal radius, added on top of whatever taper the caller is already
/// applying.
#[derive(Clone, Copy, Debug)]
pub struct Bulge {
    /// Where the swelling sits: in meters along the control points for one of
    /// a strand's [`bulges`](Strand::bulges), and in whatever coordinate the
    /// caller is walking for one it samples into the radii itself.
    pub at: f64,
    /// Standard deviation of the falloff, in the same coordinate.
    pub width: f64,
    /// Peak radius fraction added at `at`.
    pub height: f64,
}

impl Bulge {
    /// The added radius fraction at `x`. A Gaussian, so it decays smoothly to
    /// nothing rather than ending in a crease.
    pub fn value(&self, x: f64) -> f64 {
        let z = (x - self.at) / self.width.max(1e-9);
        self.height * (-z * z).exp()
    }

    /// The positions a control-point list needs in order to actually resolve
    /// this bulge.
    ///
    /// Radius is piecewise-linear between control points, so a swelling
    /// narrower than the gap between two of them is simply stepped over — a
    /// 5 cm-wide graft union on a trunk sampled every 17 cm just doesn't
    /// appear. Merging these three into the caller's own point list fixes
    /// that without making the caller reason about sampling rates.
    pub fn detail_positions(&self) -> [f64; 3] {
        [self.at - self.width, self.at, self.at + self.width]
    }

    /// Whether it adds anything worth a ring at `x`.
    fn reaches(&self, x: f64) -> bool {
        (x - self.at).abs() < 3.0 * self.width
    }
}

// ─── Collars ────────────────────────────────────────────────────────

/// Where an organ leaves a strand: its stalk starts on the strand's axis `at`
/// meters along the control points, heads `toward`, and is `radius` thick.
///
/// The strand swells toward the stalk round where it comes out through the
/// surface, so the stalk grows out of the strand rather than poking through
/// it, and the swelling runs on down the strand, as a leaf's base does.
#[derive(Clone, Copy, Debug)]
pub struct Collar {
    pub at: f64,
    pub toward: Vector3<f64>,
    pub radius: f64,
}

impl Collar {
    /// The radius this collar adds `m` along a strand on the side `out`
    /// faces, as a fraction of the strand's: what anything wound round the
    /// strand has to clear. The strand heads `tangent` and is `radius` thick
    /// where the stalk leaves it.
    pub fn swelling(&self, tangent: Vector3<f64>, radius: f64, m: f64, out: &Vector3<f64>) -> f64 {
        Lobe::new(self, tangent, radius).map_or(0.0, |lobe| lobe.value(m, out))
    }
}

/// A collar's height on its stalk's side, in the stalk's radii; how much
/// wider round the strand it reaches than the stalk does; and how far down the
/// strand it runs, in the stalk's radii.
const COLLAR_HEIGHT: f64 = 0.8;
const COLLAR_SPREAD: f64 = 1.3;
const DECURRENT: f64 = 6.0;

/// The least share of a stalk's heading that crosses the strand: a stalk
/// nearer along it than this comes out of no one side, and has no collar.
const MIN_ACROSS: f64 = 0.3;

/// A collar as it shapes its strand.
#[derive(Clone, Copy, Debug)]
struct Lobe {
    /// Where along the strand it peaks, in meters, and the side it faces.
    peak: f64,
    side: Vector3<f64>,
    /// The radius it adds at its peak, as a fraction of the strand's.
    height: f64,
    /// How far down and up the strand it reaches, in meters, and `1 - cos` of
    /// how far round it, each to where it has fallen to 1/e.
    below: f64,
    above: f64,
    spread: f64,
}

impl Lobe {
    /// `collar` on a strand heading `tangent` and `radius` thick where the
    /// stalk leaves it.
    fn new(collar: &Collar, tangent: Vector3<f64>, radius: f64) -> Option<Self> {
        let toward = collar.toward.try_normalize(1e-12)?;
        let along = toward.dot(&tangent);
        let across = toward - tangent * along;
        let out = across.norm();
        if out < MIN_ACROSS {
            return None;
        }
        Some(Self {
            // The stalk comes out through the surface `radius` off the axis,
            // by when it has come this far along the strand, and its edge
            // reaches its radius over `out` further.
            peak: collar.at + radius * along / out,
            side: across / out,
            height: COLLAR_HEIGHT * collar.radius / radius,
            below: DECURRENT * collar.radius,
            above: collar.radius / out,
            spread: 1.0 - (COLLAR_SPREAD * (collar.radius / radius).min(1.0).asin()).cos(),
        })
    }

    /// Whether it adds anything worth a ring `m` along the strand.
    fn reaches(&self, m: f64) -> bool {
        (self.peak - 3.0 * self.below..=self.peak + 3.0 * self.above).contains(&m)
    }

    /// The radius it adds `m` along the strand on the side `out` faces, as a
    /// fraction of the strand's.
    fn value(&self, m: f64, out: &Vector3<f64>) -> f64 {
        let reach = if m < self.peak {
            self.below
        } else {
            self.above
        };
        let z = (m - self.peak) / reach;
        self.height * (-z * z - (1.0 - out.dot(&self.side)) / self.spread).exp()
    }
}

/// Where along the strand, in meters, its swellings need rings: across each,
/// every half of its reach out to twice it, so a swelling narrower than the
/// rings' spacing isn't stepped over.
fn seeds(bulges: &[Bulge], lobes: &[Lobe]) -> Vec<f64> {
    let across = |at: f64, below: f64, above: f64| {
        (-4..=4).map(move |k| at + k as f64 / 2.0 * if k < 0 { below } else { above })
    };
    bulges
        .iter()
        .flat_map(|b| across(b.at, b.width, b.width))
        .chain(lobes.iter().flat_map(|l| across(l.peak, l.below, l.above)))
        .collect()
}

// ─── Bark ───────────────────────────────────────────────────────────

/// Vertical bark ridges: a per-side radius multiplier drawn once per strand
/// and applied at every station, so the ridges run the strand's length.
///
/// Drawing the jitter independently at each station instead would produce
/// noise that reads as melted wax, not bark — the ridges have to stay on the
/// same side of the tube all the way up.
///
/// The profile is a sum of low harmonics *of the ring's own period*, which is
/// what makes it seamless. A plain per-side random draw leaves a discontinuity
/// between side `sides - 1` and side `0`, and that one step reads as a bright
/// crease running the entire strand — precisely the artifact the ridges are
/// meant to be.
#[derive(Clone, Debug)]
pub struct Bark {
    /// Radius offset per side, normalized to `-1.0..=1.0`.
    profile: Vec<f64>,
    /// Phase of the along-length modulation, so strands don't breathe in sync.
    phase: f64,
    /// Ridge depth as a fraction of the radius.
    depth: f64,
}

/// Harmonics the ridge profile is built from. Clamped to the ring's Nyquist
/// limit before use: a mode at or above `sides` samples to a constant, and one
/// past `sides / 2` aliases down to a lower mode — either way the harmonic is
/// spent for nothing, so a coarse ring gets coarse ridges instead.
const MODES: [usize; 3] = [2, 3, 5];

/// How many times the ridge depth swells and thins over a strand's length.
const BARK_WAVES: f64 = 1.5;

impl Bark {
    pub fn new(rng: &mut Rng, sides: usize, depth: f64) -> Self {
        let sides = sides.clamp(MIN_SIDES, MAX_SIDES);
        let nyquist = (sides / 2).max(1);

        let harmonics: Vec<(f64, f64)> = MODES
            .iter()
            .map(|mode| {
                let mode = (*mode).min(nyquist).max(1) as f64;
                // Divided by the mode so the finer harmonics ride on the
                // coarse ones rather than drowning them.
                (rng.range(0.4, 1.0) / mode, mode)
            })
            .collect();
        let phases: Vec<f64> = harmonics.iter().map(|_| rng.unit() * TAU).collect();

        let raw: Vec<f64> = (0..sides)
            .map(|j| {
                let angle = TAU * j as f64 / sides as f64;
                harmonics
                    .iter()
                    .zip(&phases)
                    .map(|((amp, mode), phase)| amp * (mode * angle + phase).sin())
                    .sum()
            })
            .collect();

        // Normalizing by the peak is what makes `depth` mean what it says: a
        // depth of 0.14 gives ridges of ±14% of the radius, whatever the draw
        // happened to be.
        let peak = raw.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
        let profile = if peak < 1e-9 {
            vec![0.0; sides]
        } else {
            raw.iter().map(|v| v / peak).collect()
        };

        Self {
            profile,
            phase: rng.unit() * TAU,
            depth,
        }
    }

    /// A strand with no ridges at all — a clean tube.
    pub fn none() -> Self {
        Self {
            profile: Vec::new(),
            phase: 0.0,
            depth: 0.0,
        }
    }

    /// Radius multiplier at ring index `side`, at arc-length fraction `s`.
    fn at(&self, side: usize, s: f64) -> f64 {
        if self.profile.is_empty() || self.depth == 0.0 {
            return 1.0;
        }
        // Stays in 0.30..=1.00 and never reaches zero, so a ridge stays a
        // ridge for the whole strand instead of inverting into a groove
        // halfway up. It breathes; it doesn't flip.
        let sway = 0.65 + 0.35 * (TAU * BARK_WAVES * s + self.phase).sin();
        let offset = self.depth * self.profile[side % self.profile.len()] * sway;
        // Defensive floor: an inverted radius would turn the tube inside out.
        // Callers cap `depth` well below the point where this binds.
        (1.0 + offset).max(0.2)
    }
}

// ─── Strands ────────────────────────────────────────────────────────

/// One woody member of a plant: a polyline of control points with a radius at
/// each, ready to be skinned into a closed tube by [`strand_mesh`].
///
/// The centerline is interpolated *through* the control points, not steered by
/// them as a control cage. A caller therefore places points where the shape is
/// and gets exactly that shape back, without having to reason about control
/// polygons.
#[derive(Clone, Debug)]
pub struct Strand {
    /// Control points in the caller's local frame, in meters.
    pub points: Vec<Point3<f64>>,
    /// Radius at each control point. Same length as `points`.
    pub radii: Vec<f64>,
    /// Ring subdivisions around the tube. Drives the silhouette.
    pub sides: usize,
    /// Evenly spaced rings along the tube, with more across each swelling.
    /// Not read when the strand has a [`tolerance`](Self::tolerance).
    pub stations: usize,
    pub bark: Bark,
    /// Swellings all round it, `at` and `width` in meters along its control
    /// points.
    pub bulges: Vec<Bulge>,
    /// Where organs' stalks leave it.
    pub collars: Vec<Collar>,
    /// How far its mesh may stray from its surface, in meters, if its rings go
    /// only where its shape needs them rather than evenly.
    pub tolerance: Option<f64>,
}

impl Strand {
    /// Builds a strand, deriving `stations` from `rings_per_meter` and the
    /// control polyline's length.
    ///
    /// Detail expressed per meter rather than per strand keeps a 1 m trunk and
    /// a 5 cm twig equally smooth from one number, which is what lets an
    /// element expose a single detail slider.
    ///
    /// Measures the polyline rather than [`NurbsCurve::try_length`]: the fit is
    /// never shorter than its chords and rarely more than a percent longer, and
    /// the polyline costs nothing.
    pub fn new(
        points: Vec<Point3<f64>>,
        radii: Vec<f64>,
        sides: usize,
        rings_per_meter: f64,
        bark: Bark,
    ) -> Self {
        let (points, radii) = merge_coincident(points, radii);
        let length: f64 = points.windows(2).map(|w| (w[1] - w[0]).norm()).sum();
        let stations =
            ((length * rings_per_meter).round() as usize + 1).clamp(MIN_STATIONS, MAX_STATIONS);
        Self {
            points,
            radii,
            sides,
            stations,
            bark,
            bulges: Vec::new(),
            collars: Vec::new(),
            tolerance: None,
        }
    }
}

/// The fewest sides round a ring that keep it within `tolerance` of a circle
/// of `radius`, which a side strays `radius (1 - cos(π / sides))` off at its
/// middle; no fewer and no more than a strand may have.
pub fn sides_within(radius: f64, tolerance: f64) -> usize {
    let sides = PI / (1.0 - (tolerance / radius).min(1.0)).acos();
    (sides.ceil() as usize).clamp(MIN_SIDES, MAX_SIDES)
}

/// Drops control points that sit on top of their predecessor, carrying their
/// radii with them. See [`MERGE_DISTANCE`] for why this is not optional.
fn merge_coincident(points: Vec<Point3<f64>>, radii: Vec<f64>) -> (Vec<Point3<f64>>, Vec<f64>) {
    let mut kept_points: Vec<Point3<f64>> = Vec::with_capacity(points.len());
    let mut kept_radii: Vec<f64> = Vec::with_capacity(radii.len());
    for (point, radius) in points.into_iter().zip(radii) {
        if kept_points
            .last()
            .is_some_and(|last| (point - last).norm() < MERGE_DISTANCE)
        {
            continue;
        }
        kept_points.push(point);
        kept_radii.push(radius);
    }
    (kept_points, kept_radii)
}

/// Skins `strand` into a closed tube: rings of `sides` vertices, stitched into
/// quad bands and capped at both ends.
pub fn strand_mesh(strand: &Strand) -> anyhow::Result<MeshData> {
    anyhow::ensure!(
        strand.points.len() >= 2,
        "a strand needs at least two control points, got {}",
        strand.points.len()
    );
    anyhow::ensure!(
        strand.points.len() == strand.radii.len(),
        "a strand needs one radius per control point, got {} and {}",
        strand.points.len(),
        strand.radii.len()
    );

    let sides = strand.sides.clamp(MIN_SIDES, MAX_SIDES);
    let degree = DEGREE.min(strand.points.len() - 1);
    let rail = NurbsCurve3D::interpolate(&strand.points, degree)?;
    let anchors = chord_parameters(&strand.points);
    let total: f64 = strand.points.windows(2).map(|w| (w[1] - w[0]).norm()).sum();
    // The rail's parameter `m` meters along the control points.
    let on_rail = |m: f64| (m / total).clamp(0.0, 1.0);
    let lobes: Vec<Lobe> = strand
        .collars
        .iter()
        .filter_map(|collar| {
            let t = on_rail(collar.at);
            let radius = radius_at(&anchors, &strand.radii, t);
            Lobe::new(collar, rail.tangent_at(t).normalize(), radius)
        })
        .collect();

    // Evenly along the rail, or finely for the tolerance to pick from, and
    // across every swelling: each stop a parameter and how far along the
    // strand it is, as a fraction.
    let mut stops: Vec<(f64, f64)> = match strand.tolerance {
        None => {
            let count = strand.stations.clamp(MIN_STATIONS, MAX_STATIONS);
            let (parameters, fractions) = stations(&rail, count);
            parameters.into_iter().zip(fractions).collect()
        }
        Some(_) => {
            let mean = strand.radii.iter().sum::<f64>() / strand.radii.len() as f64;
            let count = ((total / (FINE * mean)).ceil() as usize + 1).clamp(MIN_STATIONS, MAX_FINE);
            (0..count)
                .map(|i| {
                    let t = i as f64 / (count - 1) as f64;
                    (t, t)
                })
                .collect()
        }
    };
    stops.extend(seeds(&strand.bulges, &lobes).into_iter().map(|m| {
        let t = on_rail(m);
        (t, t)
    }));
    stops.sort_by(|a, b| a.0.total_cmp(&b.0));
    stops.dedup_by(|next, kept| (next.0 - kept.0) * total < MERGE_DISTANCE);

    let parameters: Vec<f64> = stops.iter().map(|(t, _)| *t).collect();
    let frames = rail.compute_frenet_frames(&parameters);
    anyhow::ensure!(
        frames.len() == parameters.len(),
        "curvo returned {} frames for {} stations",
        frames.len(),
        parameters.len()
    );
    let rings: Vec<Vec<Point3<f64>>> = frames
        .iter()
        .zip(&stops)
        .map(|(frame, (t, fraction))| {
            let radius = radius_at(&anchors, &strand.radii, *t);
            ring(strand, &lobes, frame, t * total, radius, *fraction, sides)
        })
        .collect();
    let kept: Vec<usize> = match strand.tolerance {
        Some(tolerance) => {
            let along: Vec<f64> = parameters.iter().map(|t| t * total).collect();
            let tangents: Vec<Vector3<f64>> = frames.iter().map(|f| *f.tangent()).collect();
            decimate(&rings, &along, &tangents, tolerance)
        }
        None => (0..rings.len()).collect(),
    };

    let points: Vec<[f32; 3]> = kept
        .iter()
        .flat_map(|k| &rings[*k])
        .map(|p| [p.x as f32, p.y as f32, p.z as f32])
        .collect();
    anyhow::ensure!(
        points.iter().flatten().all(|c| c.is_finite()),
        "the strand's centerline degenerated"
    );

    let (face_vertex_counts, face_vertex_indices) = tube_faces(kept.len(), sides);
    Ok(MeshData {
        points,
        face_vertex_counts,
        face_vertex_indices,
    })
}

/// The parameters at which the interpolated rail passes through each of its
/// control points: normalized cumulative chord length.
///
/// This mirrors what curvo's `try_interpolate_control_points` computes
/// internally, and is the whole reason a per-control-point radius can be read
/// back at an arbitrary station: the rail *is* `points[i]` at `anchors[i]`, so
/// radius is just a piecewise-linear function over these. If curvo ever
/// switched to centripetal parameterization, the
/// `the_interpolated_rail_passes_through_its_control_points...` test below is
/// what would catch it.
fn chord_parameters(points: &[Point3<f64>]) -> Vec<f64> {
    let mut anchors = Vec::with_capacity(points.len());
    let mut total = 0.0;
    anchors.push(0.0);
    for pair in points.windows(2) {
        total += (pair[1] - pair[0]).norm();
        anchors.push(total);
    }
    if total > 0.0 {
        for anchor in &mut anchors {
            *anchor /= total;
        }
    }
    anchors
}

/// `radii` sampled at parameter `t`, linearly between the two control points
/// bracketing it and clamped past both ends.
///
/// Same bracket-and-lerp shape as `terrain::segment`, on a list short enough
/// that a scan beats a binary search.
fn radius_at(anchors: &[f64], radii: &[f64], t: f64) -> f64 {
    if radii.is_empty() {
        return 0.0;
    }
    if t <= anchors[0] {
        return radii[0];
    }
    for i in 1..anchors.len() {
        if t <= anchors[i] {
            let span = anchors[i] - anchors[i - 1];
            if span <= 0.0 {
                return radii[i];
            }
            let f = (t - anchors[i - 1]) / span;
            return radii[i - 1] + (radii[i] - radii[i - 1]) * f;
        }
    }
    radii[radii.len() - 1]
}

/// Arc-length-even parameters along `rail`, paired with each station's
/// arc-length fraction (0 at the start, 1 at the end) for the bark modulation.
///
/// `try_divide_by_count` spaces stations by arc length, which uniform
/// parameters do not on a chord-length fit that bends sharply at a head. It has
/// two sharp edges, both handled here:
///
/// - it rejects a count of 1 outright (see [`MIN_STATIONS`]), and
/// - its parameters come from a binary search with a length tolerance of
///   2.5e-3, so the last one can stop just short of the domain end.
///
/// The endpoints are therefore snapped. That is not cosmetic: a station short
/// by even a little leaves a branch's tip cap hanging in mid-air and a trunk's
/// bottom cap above the ground it was supposed to be buried in.
fn stations(rail: &NurbsCurve3D<f64>, count: usize) -> (Vec<f64>, Vec<f64>) {
    let count = count.max(MIN_STATIONS);
    let (start, end) = rail.knots_domain();

    let uniform = || -> (Vec<f64>, Vec<f64>) {
        let last = (count - 1) as f64;
        (0..count)
            .map(|i| {
                let fraction = i as f64 / last;
                (start + (end - start) * fraction, fraction)
            })
            .unzip()
    };

    let Ok(stops) = rail.try_divide_by_count(count - 1) else {
        return uniform();
    };
    if stops.len() != count {
        return uniform();
    }
    let total = stops[count - 1].length();
    if !total.is_finite() || total <= 0.0 {
        return uniform();
    }
    let mut parameters: Vec<f64> = stops.iter().map(|s| s.parameter()).collect();
    // The finiteness check is not belt-and-braces: a degenerate segment makes
    // the binary search return NaN, which would otherwise flow straight into
    // `compute_frenet_frames` and come back out as NaN vertices.
    if parameters.iter().any(|u| !u.is_finite()) || parameters.windows(2).any(|w| w[1] <= w[0]) {
        return uniform();
    }

    let fractions = stops.iter().map(|s| s.length() / total).collect();
    parameters[0] = start;
    parameters[count - 1] = end;
    (parameters, fractions)
}

/// The ring of `sides` vertices round `frame`'s origin, `m` meters along the
/// strand: `radius` there, ridged by the strand's bark and swollen by each of
/// its bulges and collars that reaches it.
///
/// The ring is authored in the frame's **local XY plane at z = 0**, which is
/// the cross-section perpendicular to the strand. That is worth stating
/// because it inverts the usual intuition: [`FrenetFrame::rotation`] is
/// `Rotation3::face_towards(tangent, normal)`, and nalgebra's `face_towards`
/// maps local **+Z** onto its `dir` argument — so here local +Z *is* the curve
/// tangent, +Y is the normal, and +X is the negated binormal. Authoring the
/// ring in XZ or YZ, the reflex from classical Frenet frames, would slice the
/// tube lengthwise instead of across.
fn ring(
    strand: &Strand,
    lobes: &[Lobe],
    frame: &FrenetFrame<f64>,
    m: f64,
    radius: f64,
    fraction: f64,
    sides: usize,
) -> Vec<Point3<f64>> {
    let placement = frame.matrix();
    let swell: f64 = strand
        .bulges
        .iter()
        .filter(|b| b.reaches(m))
        .map(|b| b.value(m))
        .sum();
    let lobes: Vec<&Lobe> = lobes.iter().filter(|l| l.reaches(m)).collect();
    (0..sides)
        .map(|j| {
            let angle = TAU * j as f64 / sides as f64;
            let (x, y) = (angle.cos(), angle.sin());
            let out = placement * Vector3::new(x, y, 0.0);
            let lobe: f64 = lobes.iter().map(|l| l.value(m, &out)).sum();
            let r = radius * strand.bark.at(j, fraction) * (1.0 + swell + lobe);
            placement * Point3::new(r * x, r * y, 0.0)
        })
        .collect()
}

/// Which of `rings` to keep: the first and the last, and between them the
/// fewest, picked greedily from the start, that hold every ring dropped within
/// `tolerance` of the band the kept ones either side of it span, measured
/// across the strand. `along` is where each ring is along the strand and
/// `tangents` the way the strand heads there.
fn decimate(
    rings: &[Vec<Point3<f64>>],
    along: &[f64],
    tangents: &[Vector3<f64>],
    tolerance: f64,
) -> Vec<usize> {
    let fits = |a: usize, b: usize| {
        (a + 1..b).all(|k| {
            let f = (along[k] - along[a]) / (along[b] - along[a]);
            let t = &tangents[k];
            rings[k]
                .iter()
                .zip(&rings[a])
                .zip(&rings[b])
                .all(|((p, from), to)| {
                    let off = p.coords - from.coords.lerp(&to.coords, f);
                    (off - t * off.dot(t)).norm() <= tolerance
                })
        })
    };
    let last = rings.len() - 1;
    let mut kept = vec![0];
    let mut a = 0;
    while a < last {
        let mut b = a + 1;
        while b < last && b - a < MAX_SPAN && fits(a, b + 1) {
            b += 1;
        }
        kept.push(b);
        a = b;
    }
    kept
}

/// The faces of a tube of `stations` rings of `sides` vertices, in USD's
/// layout.
///
/// Adjacent bands **share** their ring of vertices — the tube is
/// `stations * sides` points, indexed `station * sides + side`. Two reasons:
/// `author_mesh` in the USD builder leaves normals unauthored and
/// relies on a smooth-normal fallback, which needs shared vertices to round
/// the tube off; and it makes the mesh genuinely closed, which is what gives
/// the watertightness test something to check.
///
/// # Winding
///
/// Ring index increases with angle, i.e. counter-clockwise seen from local +Z,
/// which points *along* the tube. For a straight tube along +Z, taking ring A
/// at station `i` and ring B at `i + 1`, and sides `j` then `j'`:
///
/// - `[A_j, B_j, B_j', A_j']` gives `(0,0,h) × e_tangential`, which points
///   **inward** — the tube renders inside-out.
/// - `[A_j, A_j', B_j', B_j]` gives `e_tangential × (0,0,h)`, which points
///   **outward**.
///
/// So the second one. Caps are single n-gons over the end rings rather than
/// center-vertex fans: the ring lies in its frame's XY plane, so the polygon
/// is exactly planar by construction, and it costs fewer vertices. Listing a
/// ring in increasing `j` gives it a normal along +Z, so the start cap — which
/// must face *back* down the tube — is listed in decreasing `j` instead.
fn tube_faces(stations: usize, sides: usize) -> (Vec<i32>, Vec<i32>) {
    let index = |station: usize, side: usize| (station * sides + side) as i32;

    let mut counts = vec![4; (stations - 1) * sides];
    let mut indices = Vec::with_capacity((stations - 1) * sides * 4 + 2 * sides);

    for station in 0..stations - 1 {
        for side in 0..sides {
            let next = (side + 1) % sides;
            indices.extend([
                index(station, side),
                index(station, next),
                index(station + 1, next),
                index(station + 1, side),
            ]);
        }
    }

    counts.push(sides as i32);
    indices.extend((0..sides).rev().map(|side| index(0, side)));
    counts.push(sides as i32);
    indices.extend((0..sides).map(|side| index(stations - 1, side)));

    (counts, indices)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testing::{face_centroid, face_normal, faces};
    use bevy::math::Vec3;
    use std::collections::HashMap;

    /// A straight tube along +Z, the case every winding claim above is
    /// derived for.
    fn upright(sides: usize, stations: usize, bark: Bark) -> Strand {
        Strand {
            points: vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(0.0, 0.0, 0.5),
                Point3::new(0.0, 0.0, 1.0),
            ],
            radii: vec![0.1, 0.1, 0.1],
            sides,
            stations,
            bark,
            bulges: Vec::new(),
            collars: Vec::new(),
            tolerance: None,
        }
    }

    fn bark(depth: f64) -> Bark {
        Bark::new(&mut Rng::new(42), 8, depth)
    }

    /// The upright tube a centimeter thick, ringed every quarter meter or
    /// meshed to `tolerance`, with a node's bulge half way up and, a quarter of
    /// the way up, the collar of a stalk leaving toward +X at 45°.
    fn swollen(tolerance: Option<f64>) -> Strand {
        Strand {
            radii: vec![0.01; 3],
            bulges: vec![Bulge {
                at: 0.5,
                width: 0.01,
                height: 0.3,
            }],
            collars: vec![Collar {
                at: 0.25,
                toward: Vector3::new(1.0, 0.0, 1.0),
                radius: 0.005,
            }],
            tolerance,
            ..upright(12, 5, Bark::none())
        }
    }

    /// A bent rail, for the cases where a straight one would pass trivially:
    /// arc-length spacing and frame transport only differ from the naive
    /// answer where the curve actually turns.
    fn elbow() -> Strand {
        Strand::new(
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(0.0, 0.0, 0.6),
                Point3::new(0.1, 0.0, 0.9),
                Point3::new(0.5, 0.0, 0.9),
                Point3::new(1.0, 0.0, 0.88),
            ],
            vec![0.05, 0.04, 0.035, 0.03, 0.02],
            8,
            20.0,
            Bark::none(),
        )
    }

    #[test]
    fn a_straight_strand_skins_into_a_closed_tube() {
        let mesh = strand_mesh(&upright(8, 6, Bark::none())).unwrap();
        assert_eq!(mesh.points.len(), 6 * 8, "one vertex per ring position");
        assert_eq!(
            mesh.face_vertex_counts.len(),
            5 * 8 + 2,
            "a quad per band per side, plus two caps"
        );
        assert_eq!(
            mesh.face_vertex_indices.len(),
            mesh.face_vertex_counts.iter().sum::<i32>() as usize
        );
        assert!(
            mesh.face_vertex_indices
                .iter()
                .all(|i| (*i as usize) < mesh.points.len()),
            "every index is in range"
        );
    }

    /// Every face must wind counter-clockwise seen from outside, or the tube
    /// renders inside-out under USD's default right-handed orientation: a
    /// barked tube, and a swollen one meshed to a tolerance.
    #[test]
    fn strand_faces_wind_outward() {
        for strand in [upright(8, 6, bark(0.14)), swollen(Some(2e-4))] {
            let mesh = strand_mesh(&strand).unwrap();
            let barrel = mesh.face_vertex_counts.len() - 2;

            for (face, corners) in faces(&mesh).enumerate() {
                let normal = face_normal(&mesh, corners);
                let centroid = face_centroid(&mesh, corners);

                if face < barrel {
                    // A side face points away from the axis, which for this
                    // strand is the Z axis: compare against the centroid's
                    // own radial direction.
                    let radial = Vec3::new(centroid.x, centroid.y, 0.0);
                    assert!(
                        normal.dot(radial) > 0.0,
                        "side face {face} faces away from the axis"
                    );
                } else if face == barrel {
                    assert!(normal.z < 0.0, "the start cap faces back down the tube");
                } else {
                    assert!(normal.z > 0.0, "the end cap faces along the tube");
                }
            }
        }
    }

    /// A closed, consistently oriented surface: every undirected edge is
    /// shared by exactly two faces, and every directed edge occurs once. A
    /// mesh that failed this would have holes or inverted faces, and would
    /// make a poor collider in a simulation.
    #[test]
    fn a_strand_mesh_is_watertight() {
        for strand in [upright(8, 6, bark(0.14)), swollen(Some(2e-4))] {
            assert_watertight(&strand_mesh(&strand).unwrap());
        }
    }

    fn assert_watertight(mesh: &MeshData) {
        let mut directed: HashMap<(i32, i32), usize> = HashMap::new();
        for corners in faces(mesh) {
            for i in 0..corners.len() {
                let edge = (corners[i], corners[(i + 1) % corners.len()]);
                *directed.entry(edge).or_default() += 1;
            }
        }

        assert!(
            directed.values().all(|n| *n == 1),
            "every directed edge is used exactly once"
        );
        for (a, b) in directed.keys() {
            assert!(
                directed.contains_key(&(*b, *a)),
                "edge {a}->{b} has an opposing twin"
            );
        }
    }

    /// Pins the parameterization `radius_at` is built on. curvo fits its
    /// interpolation with chord-length parameters, so the rail passes through
    /// `points[i]` at exactly `chord_parameters()[i]`. If that ever changed,
    /// radii would silently drift along the strand and every bulge would land
    /// in the wrong place — this is the test that would catch it.
    #[test]
    fn the_interpolated_rail_passes_through_its_control_points_at_their_chord_parameters() {
        let strand = elbow();
        let rail = NurbsCurve3D::interpolate(&strand.points, 3).unwrap();
        let anchors = chord_parameters(&strand.points);

        for (point, anchor) in strand.points.iter().zip(&anchors) {
            let on_rail = rail.point_at(*anchor);
            assert!(
                (on_rail - point).norm() < 1e-6,
                "the rail is at {point:?} at parameter {anchor}, got {on_rail:?}"
            );
        }
    }

    #[test]
    fn radius_at_a_control_point_parameter_is_that_control_points_radius() {
        let strand = elbow();
        let anchors = chord_parameters(&strand.points);
        for (anchor, radius) in anchors.iter().zip(&strand.radii) {
            assert!((radius_at(&anchors, &strand.radii, *anchor) - radius).abs() < 1e-9);
        }
        // And it blends in between, rather than stepping.
        let mid = (anchors[0] + anchors[1]) / 2.0;
        let blended = radius_at(&anchors, &strand.radii, mid);
        assert!(blended < strand.radii[0] && blended > strand.radii[1]);
    }

    #[test]
    fn radius_at_clamps_outside_the_control_points() {
        let anchors = vec![0.0, 1.0];
        let radii = vec![0.2, 0.05];
        assert_eq!(radius_at(&anchors, &radii, -5.0), 0.2);
        assert_eq!(radius_at(&anchors, &radii, 5.0), 0.05);
    }

    /// Guards the 2.5e-3 length tolerance in curvo's binary search: a last
    /// station that stops short of the domain end leaves the tube's end cap
    /// floating before its tip.
    #[test]
    fn stations_land_on_both_ends_of_the_rail() {
        let strand = elbow();
        let rail = NurbsCurve3D::interpolate(&strand.points, 3).unwrap();
        let (start, end) = rail.knots_domain();
        let (parameters, fractions) = stations(&rail, 12);

        assert_eq!(parameters.len(), 12);
        assert_eq!(parameters[0], start);
        assert_eq!(parameters[11], end);
        assert!((fractions[0] - 0.0).abs() < 1e-9);
        assert!((fractions[11] - 1.0).abs() < 1e-9);
        assert!(
            parameters.windows(2).all(|w| w[1] > w[0]),
            "stations advance along the rail"
        );
    }

    /// The reason `try_divide_by_count` is used at all. On a rail that bends,
    /// uniform *parameters* bunch stations up through the turn; uniform *arc
    /// length* does not.
    #[test]
    fn stations_are_evenly_spaced_by_arc_length() {
        let strand = elbow();
        let rail = NurbsCurve3D::interpolate(&strand.points, 3).unwrap();
        let (parameters, _) = stations(&rail, 16);

        let steps: Vec<f64> = parameters
            .windows(2)
            .map(|w| (rail.point_at(w[1]) - rail.point_at(w[0])).norm())
            .collect();
        let (min, max) = steps
            .iter()
            .fold((f64::MAX, f64::MIN), |(lo, hi), s| (lo.min(*s), hi.max(*s)));
        assert!(
            max / min < 1.1,
            "arc-length steps stay within 10% of each other, got {min}..{max}"
        );
    }

    /// The lowest usable strand must still skin: two control points force the
    /// degree down to 1, the way `the_coarsest_terrain_still_builds` does for
    /// the surface.
    #[test]
    fn a_two_point_strand_still_builds() {
        let strand = Strand::new(
            vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 0.0, 0.3)],
            vec![0.02, 0.01],
            6,
            20.0,
            Bark::none(),
        );
        let mesh = strand_mesh(&strand).unwrap();
        assert!(!mesh.points.is_empty());
        assert!(mesh.points.iter().flatten().all(|c| c.is_finite()));
    }

    /// curvo's interpolation divides by the total chord length, so a strand
    /// whose points all coincide would come back entirely NaN. `Strand::new`
    /// merges them away before that can happen.
    #[test]
    fn a_degenerate_rail_produces_no_nan() {
        let strand = Strand::new(
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(0.0, 0.0, 0.4),
            ],
            vec![0.03; 4],
            6,
            20.0,
            Bark::none(),
        );
        assert_eq!(strand.points.len(), 2, "the duplicates merged away");
        let mesh = strand_mesh(&strand).unwrap();
        assert!(mesh.points.iter().flatten().all(|c| c.is_finite()));
    }

    #[test]
    fn a_strand_of_one_point_is_rejected_rather_than_skinned() {
        let strand = Strand::new(
            vec![Point3::new(0.0, 0.0, 0.0); 3],
            vec![0.03; 3],
            6,
            20.0,
            Bark::none(),
        );
        assert!(strand_mesh(&strand).is_err());
    }

    /// The point of drawing the bark profile once per strand: a ridge has to
    /// stay on the same side all the way up. If the jitter were re-drawn per
    /// station, this ranking would shuffle at every ring and the strand would
    /// look melted rather than barked.
    #[test]
    fn bark_ridges_run_along_the_strand() {
        let bark = bark(0.2);
        let ranking = |s: f64| {
            let mut order: Vec<usize> = (0..8).collect();
            order.sort_by(|a, b| bark.at(*a, s).partial_cmp(&bark.at(*b, s)).unwrap());
            order
        };
        let reference = ranking(0.0);
        for step in 1..=10 {
            assert_eq!(
                ranking(step as f64 / 10.0),
                reference,
                "the ridges keep their order along the strand"
            );
        }
    }

    /// A plain per-side random draw would leave one big step between the last
    /// side and the first, and that shows up as a bright crease running the
    /// whole strand. Building the profile from harmonics of the ring's period
    /// closes the seam.
    #[test]
    fn bark_is_seamless_around_the_ring() {
        for sides in [6usize, 8, 12, 16] {
            let bark = Bark::new(&mut Rng::new(7), sides, 0.25);
            let step = |a: usize, b: usize| (bark.at(a, 0.3) - bark.at(b, 0.3)).abs();
            let seam = step(sides - 1, 0);
            let widest = (1..sides).map(|j| step(j - 1, j)).fold(0.0_f64, f64::max);
            assert!(
                seam <= widest + 1e-9,
                "the wrap-around step ({seam}) is no worse than the widest interior one \
                 ({widest}) at {sides} sides"
            );
        }
    }

    /// An inverted radius would turn the tube inside out. The multiplier must
    /// stay positive at the roughest setting any UI exposes.
    #[test]
    fn bark_never_inverts_the_radius() {
        for seed in 0..32 {
            let bark = Bark::new(&mut Rng::new(seed), 16, 0.4);
            for side in 0..16 {
                for step in 0..=20 {
                    assert!(bark.at(side, step as f64 / 20.0) > 0.0);
                }
            }
        }
    }

    #[test]
    fn bark_of_no_depth_leaves_a_clean_tube() {
        let bark = bark(0.0);
        assert!((0..8).all(|j| (bark.at(j, 0.4) - 1.0).abs() < 1e-12));
        assert!((0..8).all(|j| (Bark::none().at(j, 0.4) - 1.0).abs() < 1e-12));
    }

    #[test]
    fn strand_meshes_are_reproducible() {
        let mesh = |()| strand_mesh(&upright(8, 10, bark(0.15))).unwrap().points;
        assert_eq!(mesh(()), mesh(()), "same input, same vertices");
    }

    #[test]
    fn a_bulge_peaks_where_it_says_and_decays_away() {
        let bulge = Bulge {
            at: 0.15,
            width: 0.05,
            height: 0.45,
        };
        assert!((bulge.value(0.15) - 0.45).abs() < 1e-12);
        assert!(bulge.value(0.20) < 0.45 && bulge.value(0.20) > 0.0);
        assert!(
            bulge.value(0.60) < 1e-6,
            "gone well before the next feature"
        );

        let detail = bulge.detail_positions();
        assert!(
            detail
                .iter()
                .zip([0.10, 0.15, 0.20])
                .all(|(got, want)| (got - want).abs() < 1e-12),
            "resolves the bulge on both flanks and its peak, got {detail:?}"
        );
    }

    /// Detail is per meter, so strands of very different sizes come out
    /// equally smooth without the caller doing the arithmetic.
    #[test]
    fn stations_scale_with_a_strands_length() {
        let at = |z: f64| {
            Strand::new(
                vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 0.0, z)],
                vec![0.02, 0.02],
                6,
                20.0,
                Bark::none(),
            )
            .stations
        };
        assert_eq!(at(1.0), 21);
        assert!(at(0.05) == MIN_STATIONS, "a 5 cm twig bottoms out");
        assert!(at(100.0) == MAX_STATIONS, "and a runaway one is capped");
    }

    /// A bulge swells the strand all round to its height at its middle, and a
    /// collar on its stalk's side only, where the stalk comes out through the
    /// surface: a radius up the strand from where it leaves the axis, at 45°.
    /// Both show though the even rings are a quarter meter apart, and both
    /// within the tolerance of a strand meshed to one.
    #[test]
    fn a_strand_swells_at_its_bulges_and_toward_its_collars() {
        for tolerance in [None, Some(2e-4)] {
            let mesh = strand_mesh(&swollen(tolerance)).unwrap();
            let slack = tolerance.unwrap_or(0.0) as f32 + 1e-5;
            let near = |z: f32| mesh.points.iter().filter(move |p| (p[2] - z).abs() < 0.005);
            let widest = near(0.5).map(|p| p[0].hypot(p[1])).fold(0.0, f32::max);
            assert!(
                (widest - 0.013).abs() < slack,
                "{tolerance:?}: the bulge is {widest} m out"
            );

            let toward = mesh
                .points
                .iter()
                .max_by(|a, b| a[0].total_cmp(&b[0]))
                .unwrap();
            let height = 0.01 + COLLAR_HEIGHT as f32 * 0.005;
            assert!(
                (toward[2] - 0.26).abs() < 0.003 && (toward[0] - height).abs() < slack,
                "{tolerance:?}: the collar peaks at {toward:?}"
            );
            let away = near(0.26).map(|p| -p[0]).fold(0.0, f32::max);
            assert!(
                away < 0.01 + slack,
                "{tolerance:?}: {away} m out the far side"
            );
        }
    }

    /// Meshed to a tolerance, a straight even tube keeps few rings and a
    /// swelling gathers them, closer the less the tolerance.
    #[test]
    fn a_strand_meshed_to_a_tolerance_rings_where_its_shape_changes() {
        let rings = |strand: Strand| strand_mesh(&strand).unwrap().points.len() / strand.sides;
        let plain = Strand {
            tolerance: Some(2e-4),
            radii: vec![0.01; 3],
            ..upright(12, 5, Bark::none())
        };
        let (even, coarse, fine) = (
            rings(plain),
            rings(swollen(Some(5e-4))),
            rings(swollen(Some(1e-4))),
        );
        assert!(
            even <= 6 && even < coarse && coarse < fine,
            "rings: {even} even, {coarse} and {fine} swollen"
        );

        let mesh = strand_mesh(&swollen(Some(2e-4))).unwrap();
        let heights: Vec<f32> = mesh
            .points
            .chunks(12)
            .map(|ring| ring.iter().map(|p| p[2]).sum::<f32>() / 12.0)
            .collect();
        let gap = |from: f32, to: f32| {
            heights
                .windows(2)
                .filter(|w| (from..to).contains(&w[0]))
                .map(|w| w[1] - w[0])
                .fold(0.0, f32::max)
        };
        let (bulge, beyond) = (gap(0.49, 0.51), gap(0.7, 1.0));
        assert!(
            bulge * 4.0 < beyond,
            "{bulge} m apart at the bulge, {beyond} m past it"
        );
    }

    #[test]
    fn sides_within_hold_a_ring_to_its_tolerance_and_no_closer() {
        let strays = |radius: f64, sides: usize| radius * (1.0 - (PI / sides as f64).cos());
        for (radius, tolerance) in [(0.01, 6e-4), (0.005, 1e-4), (0.002, 5e-4)] {
            let sides = sides_within(radius, tolerance);
            assert!(strays(radius, sides) <= tolerance);
            assert!(sides == MIN_SIDES || strays(radius, sides - 1) > tolerance);
        }
        assert_eq!(sides_within(0.01, 1.0), MIN_SIDES);
        assert_eq!(sides_within(0.01, 0.0), MAX_SIDES);
    }
}

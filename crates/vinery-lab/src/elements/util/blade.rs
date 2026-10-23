//! Blade — the outline of one grapevine leaf, built from its veins.
//!
//! A grapevine leaf is palmate: five main veins radiate from the junction
//! where the petiole joins the blade, and each ends at the tip of a lobe. The
//! midrib runs to the apex; either side of it an upper lateral vein runs to
//! an upper lobe and a lower lateral vein to a lower one, and off each lower
//! vein a short basal vein runs back beside the petiole. Between two lobes
//! the margin cuts back toward the junction in a sinus, deepest between the
//! upper lobe and the apex, and the two lower lobes meet around the petiole
//! in the petiolar sinus. Teeth run the whole way round.
//!
//! ```text
//!                 the lower lobe   the upper lobe
//!            ╭───╮ ↙  ╭──────╮ ↙
//!     basal ╱     ╲  ╱        ╲ ← the upper lateral sinus
//!     lobe ╱  N3 ──╲╱── N2     ╲
//!    ╭────╯  ╲      ╲ ╱       ╱ ╲
//!   ╱ N4 ─────╲──────P──── N1 ───────▶ apex      the midrib, along +X
//!    ╰────╮  ╱      ╱ ╲       ╲ ╱
//!      ↑   ╲ ─────╱╲── ──     ╱
//!      │    ╰───╯    ╰──────╯
//!   the petiolar sinus, round the junction P
//!   ·═══════P  the petiole, from the origin to the junction
//! ```
//!
//! A lobe's edge is two cubic curves meeting at its tip, one up out of the
//! sinus behind it and one down into the sinus ahead, as a tomato leaflet's
//! is; the tangents at the tip set how pointed it is and the handle lengths
//! how full it bulges. The sinus bottoms are round, their walls open out of
//! them, and the petiolar sinus is the two basal walls leaving the petiole's
//! top either side.
//!
//! The constants hold for every blade; the fields of [`Blade`] are what
//! differs between leaves: how deeply the cultivar lobes, how far the leaf
//! has grown, and how lopsided it is. No leaf's halves are mirror images,
//! and on a vine they lean in turn: one half the larger, the midrib curving
//! away from it toward the apex, and the next leaf up the other way round. A young leaf is not a mature one scaled down: its veins
//! are laid down first and the blade fills in around them as it expands, so
//! it starts narrower, with little blade at its base around a wide-open
//! petiolar sinus, and relatively coarser-toothed, on a shorter petiole. How
//! deeply it lobes is set from the start.
//!
//! The outline is in the frame [`shapes`](misina_lab::geometry::shapes)
//! builds in: the petiole's free end on the origin, the leaf along +X with
//! its apex furthest out, its width across Y, closed across the petiole at
//! `x = 0`. It comes with its [`Veins`], in the same frame, for whatever
//! bends the filled blade along them.

use std::f64::consts::PI;

use bevy::math::DVec2;
use misina_lab::geometry::outline::Outline;
use misina_lab::geometry::shapes::{cubic, serrate, simplify, untangle};
use misina_lab::rng::Rng;
use spade::Point2;

/// Lengths below are fractions of the midrib, from the junction to the apex.
///
/// The petiole, from the origin to the junction, and its width.
const PETIOLE: f64 = 0.85;
const STALK_WIDTH: f64 = 0.025;

/// The veins either side of the midrib, from the junction: the angle off the
/// midrib in degrees and the length, the upper lateral first, then the lower,
/// then the basal vein running back beside the petiolar sinus. Middling
/// *vinifera* cultivars: the veins open about 50° apart in turn, the upper
/// lateral reaches 0.8 to 0.9 of the midrib, the lower 0.6 to 0.7, the basal
/// about 0.4, and the blade comes out about a third wider than it is long.
const VEINS: [(f64, f64); 3] = [(52.0, 0.85), (105.0, 0.65), (150.0, 0.4)];

/// How deep the sinus after each lobe cuts at a [`Blade::lobing`] of 1, from
/// the line between the two lobe tips toward the junction: the dip behind
/// the lower lobe, the lower lateral sinus and the upper lateral one.
const SINUSES: [f64; 3] = [0.12, 0.5, 0.65];

/// The half-angle a sinus's walls open at, in degrees, on a sinus of no depth
/// and on one cut to the junction.
const OPENING: (f64, f64) = (35.0, 10.0);

/// How wide a sinus is across its rounded bottom, as a fraction of the gap
/// between the lobe tips either side of it.
const NOTCH: f64 = 0.1;

/// Where a sinus sits between two lobe tips: a half is on the line between
/// them.
const LEAN: f64 = 0.5;

/// The half-angle of a lobe's tip, and of the apex, in degrees.
const POINT: f64 = 38.0;
const APEX: f64 = 28.0;

/// How far a lobe's edge bulges: each Bézier handle as a fraction of the
/// chord it spans.
const FULL: f64 = 0.45;

/// The half-angle the petiolar sinus opens at, in degrees, on a leaf full
/// grown: which way the basal walls leave the petiole's top, off the midrib
/// running back. The most variable thing about a vine's leaves, from wide
/// open to the basal lobes overlapping; a jitter of 1 spreads it 20° either
/// way.
const PETIOLAR_OPENING: f64 = 55.0;

/// How deep the teeth cut, and how many of their own depths apart they sit:
/// about one tooth per 9 mm on a 12 cm midrib.
const TEETH: f64 = 0.028;
const TOOTH_SPACING: f64 = 2.7;

/// A leaf just unfolded, relative to the same leaf full grown: how far its
/// lateral veins reach, how far its basal ones do, how full its lobes
/// bulge, how deep its teeth cut and how long its petiole is; and the
/// half-angle its petiolar sinus opens at, in degrees. The blade fills in
/// around its veins as it expands, last of all at its base.
const YOUNG_REACH: f64 = 0.85;
const YOUNG_BASE: f64 = 0.5;
const YOUNG_FULL: f64 = 0.75;
const YOUNG_TEETH: f64 = 1.4;
const YOUNG_PETIOLE: f64 = 0.6;
const YOUNG_PETIOLAR_OPENING: f64 = 85.0;

/// Points along each cubic edge, and around each sinus's bottom.
const EDGE_POINTS: usize = 32;
const SINUS_POINTS: usize = 8;

/// A blade as lopsided as [`Blade::asymmetry`] lets it be: how much longer
/// the lateral veins of its larger half run, and those of its smaller half
/// shorter, as a fraction; and how far its midrib turns toward the smaller
/// half over its length, in radians.
const SIDE: f64 = 0.08;
const BOW: f64 = 0.3;

/// How far out along each vein its crease runs, short of the lobe's tip
/// where the margin closes in on it.
const CREASE: f64 = 0.9;

/// A blade's shape.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Blade {
    /// How deep the lateral sinuses cut on the leaf full grown: 0 leaves the
    /// margin on the line between two lobe tips, an entire leaf; 1 cuts the
    /// upper sinuses two thirds of the way to the junction.
    pub lobing: f64,
    /// How far the leaf has grown, from 0 just unfolded to 1 full size.
    pub maturity: f64,
    /// How irregular the lobes are: 0 for a regular margin, 1 for a ragged
    /// one.
    pub jitter: f64,
    /// How lopsided the blade is, and which way: 0 for halves that differ
    /// only as far as the jitter makes them, 1 for the +Y half the larger
    /// by all of [`SIDE`] and the midrib curving toward -Y by all of [`BOW`],
    /// and -1 for the mirror image.
    pub asymmetry: f64,
}

impl Blade {
    /// The outline, enclosing about `area` square meters, its margin within
    /// `tolerance` meters of the curves it is sampled from. The points it can
    /// do without are dropped: a coarser tolerance costs the teeth first,
    /// then the sinuses' round bottoms.
    ///
    /// Draws from `rng` the same amount whatever the jitter and the
    /// tolerance: side by side, the -Y side first, and along a side first
    /// every vein's angle and length, basal vein first, then how far the
    /// petiolar sinus opens, then every lobe's tip half-angle and the depth
    /// and lean of the sinus after it.
    pub fn outline(&self, area: f64, tolerance: f64, rng: &mut Rng) -> (Outline, Veins) {
        let lean = self.asymmetry.clamp(-1.0, 1.0);
        // Turning toward +Y is positive, and the midrib turns away from the
        // larger half.
        let curvature = -BOW * lean;
        let junction = DVec2::new(self.petiole(), 0.0);
        let apex = junction + arc(1.0, curvature).0;
        // Each side is drawn on +Y, so the -Y one sees the midrib mirrored.
        let (mut right, right_tips) = self.side(rng, 1.0 - SIDE * lean, -curvature);
        let (mut left, left_tips) = self.side(rng, 1.0 + SIDE * lean, curvature);
        // The sides as given, the stalk and the apex at `scale`.
        let ring = |right: &[DVec2], left: &[DVec2], scale: f64| Outline {
            points: std::iter::once(DVec2::new(0.0, -STALK_WIDTH / 2.0) * scale)
                .chain(right.iter().map(|p| DVec2::new(p.x, -p.y)))
                .chain([apex * scale])
                .chain(left.iter().rev().copied())
                .chain([DVec2::new(0.0, STALK_WIDTH / 2.0) * scale])
                .map(|p| Point2::new(p.x, p.y))
                .collect(),
        };
        let scale = (area / ring(&right, &left, 1.0).area().max(1e-12)).sqrt();
        for side in [&mut right, &mut left] {
            side.push(apex);
            for point in side.iter_mut() {
                *point *= scale;
            }
            simplify(side, tolerance);
            untangle(side);
            side.pop();
        }
        let veins = Veins {
            junction: junction * scale,
            midrib: scale,
            curvature: curvature / scale,
            tips: [right_tips.map(|p| DVec2::new(p.x, -p.y)), left_tips]
                .map(|t| t.map(|p| p * scale)),
            stalk: STALK_WIDTH * scale,
        };
        (ring(&right, &left, scale), veins)
    }

    /// The petiole's length at this maturity.
    fn petiole(&self) -> f64 {
        PETIOLE * lerp(YOUNG_PETIOLE, 1.0, self.maturity.clamp(0.0, 1.0))
    }

    /// One half of the margin, on the +Y side of the midrib, in units of the
    /// midrib: from the petiole's top, up the basal wall of the petiolar
    /// sinus and round the three lobes to just short of the apex. With it,
    /// the tips of the side's veins, upper lateral first.
    ///
    /// Its lateral veins run `size` of their length, and the midrib turns
    /// `curvature` toward this side over its length.
    fn side(&self, rng: &mut Rng, size: f64, curvature: f64) -> (Vec<DVec2>, [DVec2; 3]) {
        let (jitter, grown) = (self.jitter, self.maturity.clamp(0.0, 1.0));
        let lobing = self.lobing.clamp(0.0, 1.0);
        let full = FULL * lerp(YOUNG_FULL, 1.0, grown);
        let teeth = TEETH * lerp(YOUNG_TEETH, 1.0, grown);
        let junction = DVec2::new(self.petiole(), 0.0);

        // Where each vein ends: the basal one first, round to the apex.
        let mut tips: Vec<DVec2> = VEINS
            .iter()
            .rev()
            .enumerate()
            .map(|(k, &(angle, length))| {
                let young = if k == 0 { YOUNG_BASE } else { YOUNG_REACH };
                let angle = angle + jitter * rng.range(-6.0, 6.0);
                let length = length
                    * size
                    * lerp(young, 1.0, grown)
                    * (1.0 + jitter * rng.range(-0.08, 0.08));
                junction + DVec2::from_angle(angle.to_radians()) * length
            })
            .collect();
        let (reach, turn) = arc(1.0, curvature);
        tips.push(junction + reach);

        let mut out = Vec::new();
        let mut at = DVec2::new(junction.x, STALK_WIDTH / 2.0);
        let opening =
            lerp(YOUNG_PETIOLAR_OPENING, PETIOLAR_OPENING, grown) + jitter * rng.range(-20.0, 20.0);
        let mut heading = DVec2::from_angle((180.0 - opening).to_radians());
        for (k, pair) in tips.windows(2).enumerate() {
            let (tip, next) = (pair[0], pair[1]);
            let dir = (tip - junction).normalize_or(DVec2::Y);
            let half = POINT * (1.0 + jitter * rng.range(-0.2, 0.2));
            let depth = (SINUSES[k] * lobing * (1.0 + jitter * rng.range(-0.25, 0.25))).min(0.9);
            let lean = LEAN + jitter * rng.range(-0.1, 0.1);

            // Up out of the sinus behind, to the lobe's tip.
            let arrive = dir.rotate(DVec2::from_angle(-half.to_radians()));
            let handle = full * at.distance(tip);
            let rise = [at, at + heading * handle, tip - arrive * handle, tip];
            edge(&mut out, rise, teeth, true);

            // The sinus ahead: cut from the line between the two tips toward
            // the junction, with a round bottom no wider than half its depth,
            // its walls opening out of it at `opening` either side.
            let rim = tip.lerp(next, lean);
            let bottom = rim.lerp(junction, depth);
            let out_of = (rim - junction).normalize_or(DVec2::Y);
            let ahead = -out_of.perp();
            let radius = (NOTCH / 2.0 * tip.distance(next)).min(0.45 * rim.distance(bottom));
            let centre = bottom + out_of * radius;
            let into = centre - ahead * radius;
            let opening = lerp(OPENING.0, OPENING.1, depth).to_radians();

            // Down from the tip into it, and round its bottom.
            let leave = dir.rotate(DVec2::from_angle((half - 180.0).to_radians()));
            let handle = full * tip.distance(into);
            let wall = out_of.rotate(DVec2::from_angle(opening));
            let fall = [tip, tip + leave * handle, into + wall * handle, into];
            edge(&mut out, fall, teeth, false);
            out.extend((0..SINUS_POINTS).map(|i| {
                let a = PI * i as f64 / SINUS_POINTS as f64;
                centre - ahead * (radius * a.cos()) - out_of * (radius * a.sin())
            }));
            (at, heading) = (
                centre + ahead * radius,
                out_of.rotate(DVec2::from_angle(-opening)),
            );
        }

        // Up out of the upper sinus to the apex, without reaching past it.
        let apex = junction + reach;
        let arrive = DVec2::from_angle(turn - APEX.to_radians());
        let handle = full * at.distance(apex);
        let lead = if heading.x > 0.0 {
            handle.min(0.8 * (apex.x - at.x) / heading.x)
        } else {
            handle
        };
        let rise = [at, at + heading * lead, apex - arrive * handle, apex];
        edge(&mut out, rise, teeth, true);
        // Never across the midrib.
        for point in &mut out {
            let ahead = (curvature * (point.x - junction.x).max(0.0)).clamp(-1.0, 1.0);
            let midrib = if curvature == 0.0 {
                0.0
            } else {
                (1.0 - (1.0 - ahead * ahead).sqrt()) / curvature
            };
            point.y = point.y.max(midrib + 1e-4);
        }
        (out, [tips[2], tips[1], tips[0]])
    }
}

/// The point `s` along an arc leaving the origin along +X and turning
/// `curvature` radians per unit of length toward +Y, and how far it has
/// turned there.
fn arc(s: f64, curvature: f64) -> (DVec2, f64) {
    let turn = curvature * s;
    let point = if turn.abs() < 1e-6 {
        DVec2::new(s, turn * s / 2.0)
    } else {
        DVec2::new(turn.sin(), 1.0 - turn.cos()) / curvature
    };
    (point, turn)
}

/// Where a blade's veins run, in the frame and at the scale of its outline.
///
/// The midrib leaves the junction, where the petiole meets the blade, along
/// +X, and curves on an arc to the apex; the lateral veins run straight from
/// the junction to the tip of every lobe.
#[derive(Clone, Debug, PartialEq)]
pub struct Veins {
    pub junction: DVec2,
    /// The midrib's length, the blade's own unit of length, and how far it
    /// turns per meter of it, positive toward +Y.
    pub midrib: f64,
    pub curvature: f64,
    /// The lateral veins' tips on the -Y side, then on the +Y side, each
    /// from the midrib outward: upper, lower, basal.
    pub tips: [[DVec2; 3]; 2],
    /// The petiole's width.
    pub stalk: f64,
}

/// Where a point of a blade sits among its veins, seen from the junction.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Bearing {
    /// How far the point is from the junction.
    pub radius: f64,
    /// Whether it is blade rather than petiole.
    pub blade: bool,
    /// A bump between the two veins either side of the point, 0 on each and
    /// at most a third of the angle between them halfway, so that a slope
    /// times the radius times this rises off both veins at about that slope.
    /// The petiole counts as the vein after the basal one.
    pub ridge: f64,
    /// The length of the veins either side, interpolated at the point's
    /// bearing: about how far out the margin is.
    pub reach: f64,
}

/// Where a point sits across the midrib's line: the arc ahead of the
/// junction, and the petiole's own line behind it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Across {
    /// The point on the line nearest it, and the line's normal there,
    /// toward +Y.
    pub foot: DVec2,
    pub normal: DVec2,
    /// How far the point is off the line, along `normal`.
    pub offset: f64,
    /// How far along the midrib the foot is from the junction, negative
    /// behind it.
    pub along: f64,
}

impl Veins {
    pub fn apex(&self) -> DVec2 {
        self.junction + arc(self.midrib, self.curvature).0
    }

    /// The veins as lines for the fill to keep as edges: the midrib, sampled
    /// along its arc, then the lateral veins, each running [`CREASE`] of the
    /// way to its tip.
    pub fn creases(&self) -> Vec<Vec<Point2<f64>>> {
        let midrib = (0..=16)
            .map(|i| self.junction + arc(CREASE * self.midrib * i as f64 / 16.0, self.curvature).0)
            .collect();
        let laterals = self
            .tips
            .iter()
            .flatten()
            .map(|&tip| vec![self.junction, self.junction.lerp(tip, CREASE)]);
        std::iter::once(midrib)
            .chain(laterals)
            .map(|line: Vec<DVec2>| line.iter().map(|p| Point2::new(p.x, p.y)).collect())
            .collect()
    }

    /// Where `point`, in the outline's frame, sits across the midrib.
    pub fn across(&self, point: DVec2) -> Across {
        let u = point - self.junction;
        let k = self.curvature;
        // Round the arc's centre, a quarter turn off the junction.
        let along = if k.abs() < 1e-9 {
            u.x
        } else {
            (k * u.x).atan2(1.0 - k * u.y) / k
        };
        if along <= 0.0 {
            return Across {
                foot: self.junction + DVec2::new(u.x, 0.0),
                normal: DVec2::Y,
                offset: u.y,
                along: u.x,
            };
        }
        let (foot, turn) = arc(along, k);
        let normal = DVec2::new(-turn.sin(), turn.cos());
        Across {
            foot: self.junction + foot,
            normal,
            offset: (u - foot).dot(normal),
            along,
        }
    }

    /// Where `point`, in the outline's frame, sits among the veins.
    pub fn bearing(&self, point: DVec2) -> Bearing {
        let u = point - self.junction;
        let radius = u.length();
        // The midrib's own bearing this far out: half of what it has turned.
        let midrib = (self.curvature * radius / 2.0).clamp(-1.0, 1.0).asin();
        let angle = u.y.atan2(u.x);
        // Angles round from +X toward the point's side, and lengths.
        let left = angle >= midrib;
        let sign = if left { 1.0 } else { -1.0 };
        let polar = |tip: DVec2| {
            let v = tip - self.junction;
            ((sign * v.y).atan2(v.x), v.length())
        };
        let side = self.tips[left as usize].map(polar);
        let veins = [
            (sign * midrib, self.midrib),
            side[0],
            side[1],
            side[2],
            (PI, side[2].1),
        ];
        let at = sign * angle;
        let k = veins.windows(2).position(|w| at <= w[1].0).unwrap_or(3);
        let ((a, near), (b, far)) = (veins[k], veins[k + 1]);
        let t = ((at - a) / (b - a)).clamp(0.0, 1.0);
        Bearing {
            radius,
            // Behind the junction and no wider than the petiole, with room
            // for the mesh's rounding.
            blade: !(u.x < 0.0 && u.y.abs() <= self.stalk / 2.0 * 1.001),
            ridge: (b - a) / PI * (PI * t).sin(),
            reach: lerp(near, far, t),
        }
    }
}

fn lerp(a: f64, b: f64, t: f64) -> f64 {
    a + (b - a) * t
}

/// Appends a cubic Bézier, short of its last point, with teeth along it
/// `depth` deep.
fn edge(out: &mut Vec<DVec2>, curve: [DVec2; 4], depth: f64, toward_end: bool) {
    let start = out.len();
    cubic(out, curve, EDGE_POINTS);
    serrate(&mut out[start..], depth, TOOTH_SPACING, toward_end);
}

#[cfg(test)]
mod tests {
    use super::*;
    use misina_lab::geometry::outline::creased_mesh;

    /// The frame every placed outline depends on, and a ring that fills with
    /// its veins: no crossings at any seed, however ragged the jitter makes
    /// it, at any growth stage, lobing or lean, nor at a tolerance that drops
    /// most of its points.
    #[test]
    fn every_blade_hangs_from_the_origin_and_fills() {
        for seed in 0..96 {
            let blade = Blade {
                lobing: (seed % 4) as f64 / 3.0,
                maturity: (seed % 5) as f64 / 4.0,
                jitter: 1.0,
                asymmetry: (seed % 3) as f64 - 1.0,
            };
            let tolerance = if seed % 2 == 0 { 0.0005 } else { 0.005 };
            let (outline, veins) = blade.outline(0.015, tolerance, &mut Rng::new(seed));
            let at = format!("seed {seed}, {blade:?}, tolerance {tolerance}");
            assert!(
                outline.points.iter().all(|p| p.x >= 0.0),
                "{at}: ahead of the attachment"
            );
            assert!(
                outline.points.iter().filter(|p| p.x == 0.0).count() == 2,
                "{at}: closed across the petiole"
            );
            let area = outline.area();
            assert!(
                (area - 0.015).abs() < 0.015 * 0.05,
                "{at}: encloses {area} m², not 0.015"
            );
            creased_mesh(&outline, &veins.creases(), area / 20.0)
                .unwrap_or_else(|e| panic!("{at} fills: {e:#}"));
        }
    }

    /// What a bend reads off the veins: blade everywhere but the petiole, no
    /// crest on a vein and one between two, on either side.
    #[test]
    fn a_bearing_finds_the_veins_either_side() {
        let blade = Blade {
            lobing: 0.5,
            maturity: 1.0,
            jitter: 1.0,
            asymmetry: 0.7,
        };
        let (_, veins) = blade.outline(0.015, 0.0005, &mut Rng::new(7));
        let root = veins.bearing(DVec2::new(0.0, 0.001));
        assert!(!root.blade, "the petiole is not blade: {root:?}");
        for side in veins.tips {
            let (upper, lower) = (side[0], side[1]);
            let on = veins.bearing(veins.junction + (upper - veins.junction) * 0.5);
            assert!(on.ridge.abs() < 1e-9 && on.blade, "on a vein: {on:?}");
            let between = (upper + lower) / 2.0;
            let off = veins.bearing(between);
            assert!(off.ridge > 0.2, "between two: {off:?}");
            assert!(
                off.reach > veins.junction.distance(lower) && off.reach < veins.midrib,
                "about as far out as the veins either side: {off:?}"
            );
        }
        // The midrib curves, and is a vein all along.
        let (s, k) = (veins.midrib * 0.6, veins.curvature);
        let on = veins.junction + DVec2::new((k * s).sin(), 1.0 - (k * s).cos()) / k;
        assert!(veins.bearing(on).ridge.abs() < 1e-9, "on the midrib");
        let normal = DVec2::new(-(k * s).sin(), (k * s).cos());
        let across = veins.across(on + normal * 0.01);
        assert!(
            (across.along - s).abs() < 1e-9 && (across.offset - 0.01).abs() < 1e-9,
            "{across:?}"
        );
    }

    /// Growth is not a scale: in units of its midrib, a leaf just unfolded
    /// is narrower than the same leaf full grown, and its basal lobes reach
    /// less far back round the petiole.
    #[test]
    fn a_young_blade_is_narrower_and_open_at_the_base() {
        let margin = |maturity: f64| {
            let blade = Blade {
                lobing: 0.6,
                maturity,
                jitter: 0.0,
                asymmetry: 0.0,
            };
            let (side, _) = blade.side(&mut Rng::new(1), 1.0, 0.0);
            let width = side.iter().fold(0.0f64, |m, p| m.max(p.y));
            let back = side
                .iter()
                .fold(0.0f64, |m, p| m.max(blade.petiole() - p.x));
            (width, back)
        };
        let (young, grown) = (margin(0.0), margin(1.0));
        assert!(young.0 < grown.0, "narrower: {young:?} against {grown:?}");
        assert!(
            young.1 < grown.1,
            "open at the base: {young:?} against {grown:?}"
        );
    }

    /// A lopsided blade's larger half is the wider, and its midrib carries
    /// the apex over toward the smaller one; leaning the other way is the
    /// mirror image.
    #[test]
    fn a_lopsided_blade_leans_away_from_its_larger_half() {
        let lean = |asymmetry: f64| {
            let blade = Blade {
                lobing: 0.5,
                maturity: 1.0,
                jitter: 0.0,
                asymmetry,
            };
            let (outline, veins) = blade.outline(0.015, 0.0005, &mut Rng::new(3));
            let wide = |sign: f64| outline.points.iter().fold(0.0f64, |m, p| m.max(sign * p.y));
            (wide(1.0), wide(-1.0), veins.apex().y)
        };
        let (left, right, apex) = lean(1.0);
        assert!(
            left > right * 1.05,
            "the +Y half is the wider: {left} against {right}"
        );
        assert!(apex < -0.005, "the apex is carried toward -Y: {apex}");
        let (mirror_left, mirror_right, mirror_apex) = lean(-1.0);
        assert!((mirror_left - right).abs() < 1e-9 && (mirror_right - left).abs() < 1e-9);
        assert!((mirror_apex + apex).abs() < 1e-9);
        let (_, _, straight) = lean(0.0);
        assert_eq!(straight, 0.0, "and a symmetric one's runs straight");
    }
}

//! Leaflet — the outline of one tomato leaflet, built from its veins.
//!
//! A tomato leaf is pinnately compound, and each leaflet is pinnately veined
//! in turn: a vein leaves the midrib for every lobe and ends at the lobe's
//! tip, and between two lobes the margin is cut back toward the midrib in a
//! sinus with a rounded bottom: a notch on a toothed leaflet, a wedge on a
//! deeply lobed one. The lobing is deepest at the base,
//! where a large leaflet's first lobes can be all but leaflets of their own,
//! and fades to coarse teeth toward the apex. Small teeth run along the
//! lobes' edges.
//!
//! ```text
//!             a lobe's tip, where its vein ends
//!         ╭──╮ ↙ ╭──╮
//!   ╭────╯   ╲  ╱    ╲──╮
//!  ═╡ ────────┴─┴─────┴──┴──→ apex      the midrib, along +X
//!   ╰────╮   ╱  ╲    ╱──╯
//!   ↑     ╰──╯ ↖ ╰──╯
//!   │          a sinus, cut back toward the midrib
//!   the petiolule's free end, on the origin
//! ```
//!
//! A lobe's edge is two cubic curves meeting at its tip: one up out of the
//! sinus behind it, one down into the sinus ahead. The tangents at the tip set
//! how pointed it is, the handle lengths how full the lobe bulges.
//!
//! The constants hold for every leaflet; the fields of [`Leaflet`] are what
//! differs between a terminal, a lateral and an intercalary one.
//!
//! The outline is in the frame [`shapes`](misina_lab::geometry::shapes)
//! builds in: the attachment on the origin, the leaflet along +X with its apex
//! at `length`, its width across Y, closed across the petiolule at `x = 0`.
//! It comes with the [`Veins`] it was built from, so what fills and bends
//! the blade can keep them as edges, pucker it between them and tell how far
//! off the midrib the margin is.

use std::f64::consts::PI;

use bevy::math::DVec2;
use misina_lab::geometry::outline::Outline;
use misina_lab::geometry::shapes::{cubic, serrate, simplify, untangle};
use misina_lab::rng::Rng;
use spade::Point2;

/// Where the first and last veins leave the midrib, as fractions of the blade.
const VEINS: (f64, f64) = (0.06, 0.77);

/// How the veins between the first and the last are spaced: under 1 crowds
/// them toward the apex, where the lobes are teeth.
const SPREAD: f64 = 0.8;

/// Where along the blade the lobes reach farthest, and how steeply the reach
/// falls off from there to the apex: an exponent, higher for a more drawn-out
/// point.
const PEAK: f64 = 0.26;
const TAPER: f64 = 1.85;

/// How deep a sinus near the apex cuts, and how fast the depth falls to that
/// from [`Leaflet::lobing`] at the base.
const DEPTH_AT_APEX: f64 = 0.27;
const FALLOFF: f64 = 1.5;

/// Where a sinus sits between two lobe tips. Under a half leans the lobes
/// forward, toward the apex.
const LEAN: f64 = 0.2;

/// How wide a sinus is across its rounded bottom, as a fraction of the gap
/// between the lobe tips either side of it.
const NOTCH: f64 = 0.1;

/// The angle between a sinus's two walls, in degrees, per unit of
/// [`Leaflet::lobing`]: a deeply lobed leaflet's sinuses open into wedges, a
/// toothed one's are notches.
const OPENING: f64 = 60.0;

/// The half-angle of a lobe's tip, and of the leaflet's apex, in degrees.
const POINT: f64 = 40.0;
const APEX: f64 = 21.0;

/// How far a lobe's edge bulges: each Bézier handle as a fraction of the
/// chord it spans.
const FULL: f64 = 0.4;

/// Which way the blade leaves the petiolule, in degrees off the midrib. Past
/// 90 the base is cordate.
const BASE: f64 = 110.0;

/// The petiolule's width, as a fraction of the leaflet's length.
const STALK_WIDTH: f64 = 0.025;

/// How far up the blade a lopsided leaflet's short side starts its veins,
/// per unit of [`Leaflet::lopsided`].
const OBLIQUE: f64 = 0.6;

/// Small teeth sit this many of their own depths apart.
const TOOTH_SPACING: f64 = 5.0;

/// Points along each cubic edge, and around each sinus's bottom.
const EDGE_POINTS: usize = 16;
const SINUS_POINTS: usize = 8;

/// A leaflet just unfolded, as fractions of what a grown one is cut to: how
/// far its lobes reach, how deep its sinuses cut and how deep its teeth are.
/// Young blades are narrow, their lobes still coarse notches, and their teeth
/// large for their size.
const YOUNG_WIDTH: f64 = 0.7;
const YOUNG_LOBING: f64 = 0.5;
const YOUNG_TEETH: f64 = 1.4;

/// How far along each vein its crease runs, as a fraction: short of the
/// margin, so the fill's edge along it ends inside the ring.
const CREASE: f64 = 0.9;

/// A leaflet's shape. Lengths are fractions of the leaflet's own length, from
/// the petiolule's free end to the apex. The presets are cut at the leaf
/// params' full lobing and asymmetry; [`leaf`](super::leaf) scales
/// [`lobing`](Self::lobing) and [`lopsided`](Self::lopsided) by them.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Leaflet {
    /// The petiolule, the leaflet's own stalk.
    pub stalk: f64,
    /// How far the farthest lobe tip reaches from the midrib.
    pub width: f64,
    /// Lobes on each side, besides the apex.
    pub lobes: usize,
    /// How deep the sinuses at the base cut: 0 leaves the margin on the line
    /// between two lobe tips, 1 would cut to the midrib. Toward the apex the
    /// depth goes to a fixed shallow one.
    pub lobing: f64,
    /// The veins' angle off the midrib in degrees, at the base and at the
    /// apex. Past 90 a lobe points back toward the petiolule.
    pub angle: (f64, f64),
    /// How much further one side reaches than the other, as a fraction of
    /// [`width`](Self::width); the other side's veins start further up the
    /// blade, which makes the base oblique. A lateral leaflet's lopsidedness.
    /// Signed, so either side can be the long one.
    pub lopsided: f64,
    /// How deep the small teeth along a lobe's edges are.
    pub serration: f64,
    /// How irregular the lobes are: 0 for a regular margin, 1 for a ragged
    /// one.
    pub jitter: f64,
    /// How far the leaflet has grown, 0 just unfolded to 1 full size: a
    /// young one is cut narrower, less lobed and coarser toothed, whatever
    /// its length.
    pub grown: f64,
}

impl Leaflet {
    /// The leaflet at the tip of a leaf: symmetric and long, lobed all the
    /// way to the apex.
    pub const TERMINAL: Self = Self {
        stalk: 0.1,
        width: 0.25,
        lobes: 5,
        lobing: 0.82,
        angle: (95.0, 35.0),
        lopsided: 0.0,
        serration: 0.004,
        jitter: 0.5,
        grown: 1.0,
    };

    /// A lateral leaflet: lopsided, its first lobes deep and the rest of it a
    /// long, toothed point.
    pub const LATERAL: Self = Self {
        stalk: 0.15,
        width: 0.28,
        lobes: 4,
        lobing: 1.0,
        angle: (100.0, 32.0),
        lopsided: 0.3,
        serration: 0.006,
        jitter: 0.5,
        grown: 1.0,
    };

    /// One of the small leaflets between the laterals: ovate, with a lobe
    /// either side of its base.
    pub const INTERCALARY: Self = Self {
        stalk: 0.13,
        width: 0.42,
        lobes: 1,
        lobing: 0.18,
        angle: (99.0, 30.0),
        lopsided: 0.1,
        serration: 0.0,
        jitter: 0.5,
        grown: 1.0,
    };

    /// The outline, `length` long, its margin within `tolerance` of the
    /// curves it is sampled from, both in meters, and the veins it was built
    /// from. The points it can do without are dropped: a coarser tolerance
    /// costs the finest teeth first, then the sinuses' round bottoms, and a
    /// small leaflet keeps fewer points than a large one.
    ///
    /// Draws from `rng` the same amount whatever the jitter and the
    /// tolerance: side by side, the -Y side first, and along a side first
    /// every vein's position, angle and reach, then every lobe's tip
    /// half-angle and the depth of the sinus after it.
    pub fn outline(&self, length: f64, tolerance: f64, rng: &mut Rng) -> (Outline, Veins) {
        let half = STALK_WIDTH / 2.0;
        let tolerance = tolerance / length.max(1e-6);
        let (right, right_veins) = self.side(-1.0, tolerance, rng);
        let (left, left_veins) = self.side(1.0, tolerance, rng);
        let ring = std::iter::once(DVec2::new(0.0, -half))
            .chain(right.iter().map(|p| DVec2::new(p.x, -p.y)))
            .chain([DVec2::X])
            .chain(left.iter().rev().copied())
            .chain([DVec2::new(0.0, half)]);
        let outline = Outline {
            points: ring
                .map(|p| Point2::new(p.x * length, p.y * length))
                .collect(),
        };
        let placed = |veins: Vec<Vein>, sign: f64| -> Vec<Vein> {
            let flip = DVec2::new(1.0, sign);
            veins
                .into_iter()
                .map(|v| Vein {
                    u: v.u,
                    foot: v.foot * length,
                    dir: v.dir * flip,
                    tip: v.tip * flip * length,
                })
                .collect()
        };
        let veins = Veins {
            length,
            stalk: self.stalk * length,
            reach: [-1.0, 1.0].map(|side| self.half_width(side) * length),
            sides: [placed(right_veins, -1.0), placed(left_veins, 1.0)],
        };
        (outline, veins)
    }

    /// How far the lobes on `side` reach from the midrib, as a fraction of
    /// the length.
    fn half_width(&self, side: f64) -> f64 {
        self.width
            * lerp(YOUNG_WIDTH, 1.0, self.grown.clamp(0.0, 1.0))
            * (1.0 + side * self.lopsided)
    }

    /// One half of the margin, on the +Y side of the midrib: from where the
    /// blade leaves the petiolule to just short of the apex, within
    /// `tolerance` of its curves, and the side's veins from the base to the
    /// apex, which closes the list as a vein of its own. `side` is +1 for the
    /// side [`lopsided`](Self::lopsided) lengthens, -1 for the other.
    fn side(&self, side: f64, tolerance: f64, rng: &mut Rng) -> (Vec<DVec2>, Vec<Vein>) {
        let (jitter, lobes) = (self.jitter, self.lobes.max(1));
        let grown = self.grown.clamp(0.0, 1.0);
        let blade = 1.0 - self.stalk;
        let width = self.half_width(side);
        let lobing = self.lobing * lerp(YOUNG_LOBING, 1.0, grown);
        let serration = self.serration * lerp(YOUNG_TEETH, 1.0, grown);
        let first = VEINS.0 + (-side * self.lopsided * OBLIQUE).max(0.0);
        let gap = (VEINS.1 - first) / lobes as f64;

        let mut veins: Vec<Vein> = (0..lobes)
            .map(|k| {
                let even = (k as f64 / (lobes - 1).max(1) as f64).powf(SPREAD);
                let u = first + (VEINS.1 - first) * even + jitter * gap * rng.range(-0.35, 0.35);
                let u = u.clamp(VEINS.0 / 2.0, 0.9);
                let base = (1.0 - u).powf(FALLOFF);
                let angle = lerp(self.angle.1, self.angle.0, base) + jitter * rng.range(-8.0, 8.0);
                let dir = DVec2::from_angle(angle.to_radians());
                // Held off the midrib, so no lobe is a spike back along it.
                let reach = (width * reach(u) * (1.0 + jitter * rng.range(-0.15, 0.15)))
                    .max(2.0 * STALK_WIDTH);
                let foot = DVec2::new(self.stalk + u * blade, 0.0);
                let mut tip = foot + dir * (reach / dir.y.max(0.2));
                tip.x = tip.x.min(lerp(foot.x, 1.0, 0.8)); // short of the apex
                Vein { u, foot, dir, tip }
            })
            .collect();
        veins.push(Vein {
            u: 1.0,
            foot: DVec2::X,
            dir: DVec2::X,
            tip: DVec2::X,
        });

        let mut out = Vec::new();
        let mut at = DVec2::new(self.stalk, STALK_WIDTH / 2.0);
        let mut heading = DVec2::from_angle(BASE.to_radians());
        for pair in veins.windows(2) {
            let (vein, next) = (pair[0], pair[1]);
            let opening = (OPENING * lobing / 2.0).to_radians();
            let half = POINT * (1.0 + jitter * rng.range(-0.2, 0.2));
            let deeper = 1.0 + jitter * rng.range(-0.25, 0.25);

            // Up out of the sinus behind, to the lobe's tip.
            let arrive = vein.dir.rotate(DVec2::from_angle(-half.to_radians()));
            let handle = FULL * at.distance(vein.tip);
            let rise = [
                at,
                at + heading * handle,
                vein.tip - arrive * handle,
                vein.tip,
            ];
            edge(&mut out, rise, serration, true);

            // The sinus ahead: cut from the line between the two tips toward
            // the midrib, with a round bottom no wider than half its depth,
            // its walls opening out of it at `opening` either side.
            let base = (1.0 - lerp(vein.u, next.u, LEAN)).powf(FALLOFF);
            let depth = (lerp(DEPTH_AT_APEX, lobing, base) * deeper).min(0.9);
            let rim = vein.tip.lerp(next.tip, LEAN);
            let foot = vein.foot.lerp(next.foot, LEAN);
            let bottom = rim.lerp(foot, depth);
            let out_of = (rim - foot).normalize_or(DVec2::Y);
            let ahead = -out_of.perp();
            let radius =
                (NOTCH / 2.0 * vein.tip.distance(next.tip)).min(0.45 * rim.distance(bottom));
            let centre = bottom + out_of * radius;
            let into = centre - ahead * radius;

            // Down from the tip into it, and round its bottom.
            let leave = vein
                .dir
                .rotate(DVec2::from_angle((half - 180.0).to_radians()));
            let handle = FULL * vein.tip.distance(into);
            let wall = out_of.rotate(DVec2::from_angle(opening));
            let fall = [
                vein.tip,
                vein.tip + leave * handle,
                into + wall * handle,
                into,
            ];
            edge(&mut out, fall, serration, false);
            out.extend((0..SINUS_POINTS).map(|i| {
                let a = PI * i as f64 / SINUS_POINTS as f64;
                centre - ahead * (radius * a.cos()) - out_of * (radius * a.sin())
            }));
            (at, heading) = (
                centre + ahead * radius,
                out_of.rotate(DVec2::from_angle(-opening)),
            );
        }

        // Up out of the last sinus to the apex, without reaching past it.
        let arrive = DVec2::from_angle(-APEX.to_radians());
        let handle = FULL * at.distance(DVec2::X);
        let lead = if heading.x > 0.0 {
            handle.min(0.8 * (1.0 - at.x) / heading.x)
        } else {
            handle
        };
        let rise = [
            at,
            at + heading * lead,
            DVec2::X - arrive * handle,
            DVec2::X,
        ];
        edge(&mut out, rise, serration, true);
        for point in &mut out {
            point.y = point.y.max(1e-4); // never across the midrib
        }
        out.push(DVec2::X);
        simplify(&mut out, tolerance);
        untangle(&mut out);
        out.pop();
        (out, veins)
    }
}

/// Where a vein leaves the midrib, which way it runs, and where it ends.
#[derive(Clone, Copy, Debug)]
pub struct Vein {
    /// How far along the blade it leaves the midrib, as a fraction of the
    /// blade.
    pub u: f64,
    pub foot: DVec2,
    /// Unit length, on its side of the midrib.
    pub dir: DVec2,
    /// At its lobe's tip, on the margin.
    pub tip: DVec2,
}

/// The veins an outline was built from, in its frame and in meters: the
/// midrib along +X from the blade's start to the apex, and on each side,
/// the -Y side first, a vein per lobe from the base toward the apex, the
/// apex closing each side's list as a vein of its own.
#[derive(Clone, Debug)]
pub struct Veins {
    pub length: f64,
    /// Where the blade starts along the midrib: the petiolule's length.
    pub stalk: f64,
    /// Per side, how far the farthest lobe tip reaches from the midrib.
    pub reach: [f64; 2],
    pub sides: [Vec<Vein>; 2],
}

impl Veins {
    /// The lines a fill keeps as edges, so the blade can fold on them: the
    /// midrib, and the lobes' veins on a side where they are at least
    /// `spacing` apart along it, so a fill too coarse to pucker between two
    /// veins is not made to crease on them. Each is stopped [`CREASE`] of the
    /// way to its end so that it ends inside the margin.
    pub fn creases(&self, spacing: f64) -> Vec<Vec<Point2<f64>>> {
        let apex = lerp(self.stalk, self.length, CREASE);
        let midrib = vec![DVec2::new(self.stalk, 0.0), DVec2::new(apex, 0.0)];
        let lobes = self
            .sides
            .iter()
            .filter(|side| (self.length - self.stalk) / side.len() as f64 >= spacing)
            .flatten()
            .filter(|v| v.u < 1.0)
            .map(|v| vec![v.foot, v.foot.lerp(v.tip, CREASE)]);
        std::iter::once(midrib)
            .chain(lobes)
            .map(|line| line.iter().map(|p| Point2::new(p.x, p.y)).collect())
            .collect()
    }

    /// How far off the midrib the lobes reach at `x`, on the side of `y`:
    /// where the margin would be but for the sinuses cut into it.
    pub fn half(&self, x: f64, y: f64) -> f64 {
        let u = ((x - self.stalk) / (self.length - self.stalk).max(1e-9)).clamp(0.0, 1.0);
        let side = self.reach[(y >= 0.0) as usize];
        (side * reach(u)).max(STALK_WIDTH * self.length)
    }

    /// The pucker between the two veins either side of `at`: a hump from one
    /// vein to the next as high as the gap between them is wide, in meters,
    /// zero on the veins themselves. The gap is taken at the point's own
    /// distance off the midrib, so a slanting vein stays a valley all along
    /// it. Zero behind the first vein.
    pub fn ridge(&self, at: DVec2) -> f64 {
        let out = at.y.abs();
        let crossing = |v: &Vein| v.foot.x + out * v.dir.x / v.dir.y.abs().max(0.2);
        self.sides[(at.y >= 0.0) as usize]
            .windows(2)
            .map(|pair| (crossing(&pair[0]), crossing(&pair[1])))
            .find(|(a, b)| (*a..*b).contains(&at.x))
            .map_or(0.0, |(a, b)| (b - a) * (PI * (at.x - a) / (b - a)).sin())
    }
}

/// How far the lobe of a vein leaving the midrib `u` of the way along the
/// blade reaches, as a fraction of the farthest: `u^a (1 - u)^TAPER`, with `a`
/// set so the hump peaks at [`PEAK`].
fn reach(u: f64) -> f64 {
    let u = u.clamp(1e-4, 1.0 - 1e-4);
    let a = TAPER * PEAK / (1.0 - PEAK);
    u.powf(a) * (1.0 - u).powf(TAPER) / (PEAK.powf(a) * (1.0 - PEAK).powf(TAPER))
}

fn lerp(a: f64, b: f64, t: f64) -> f64 {
    a + (b - a) * t
}

/// Appends a cubic Bézier, short of its last point, with small teeth along
/// it `serration` deep.
fn edge(out: &mut Vec<DVec2>, curve: [DVec2; 4], serration: f64, toward_end: bool) {
    let start = out.len();
    cubic(out, curve, EDGE_POINTS);
    serrate(&mut out[start..], serration, TOOTH_SPACING, toward_end);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::elements::leaf::TOLERANCE;
    use misina_lab::geometry::outline::creased_mesh;

    const ALL: [(&str, Leaflet); 3] = [
        ("terminal", Leaflet::TERMINAL),
        ("lateral", Leaflet::LATERAL),
        ("intercalary", Leaflet::INTERCALARY),
    ];

    /// The frame every placed outline depends on, and a ring that fills with
    /// its veins as edges: no crossings at any seed, however ragged the
    /// jitter makes it, nor at a size where most of its points are dropped,
    /// nor on a leaflet just unfolded.
    #[test]
    fn every_leaflet_hangs_from_the_origin_and_fills() {
        for (name, shape) in ALL {
            for seed in 0..64 {
                let ragged = Leaflet {
                    jitter: 1.0,
                    grown: (seed % 3) as f64 / 2.0,
                    ..shape
                };
                let length = if seed % 2 == 0 { 0.1 } else { 0.005 };
                let (outline, veins) = ragged.outline(length, TOLERANCE, &mut Rng::new(seed));
                let reach = outline.points.iter().fold(0.0f64, |m, p| m.max(p.x));
                assert!(
                    (reach - length).abs() < 1e-9,
                    "{name} {seed}: apex at the length, {reach}"
                );
                assert!(
                    outline.points.iter().all(|p| p.x >= 0.0),
                    "{name} {seed}: ahead of the attachment"
                );
                assert!(
                    outline.points.iter().any(|p| p.x == 0.0),
                    "{name} {seed}: closed across the petiolule"
                );
                let mesh = creased_mesh(&outline, &veins.creases(0.0), outline.area() / 20.0)
                    .unwrap_or_else(|e| panic!("{name} {seed} fills: {e:#}"));
                assert!(
                    mesh.points.len() > outline.points.len(),
                    "{name} {seed}: the veins are edges of the fill"
                );
                for vein in veins.sides.iter().flatten() {
                    assert!(
                        vein.foot.y == 0.0 && (0.0..=length).contains(&vein.foot.x),
                        "{name} {seed}: a vein leaves the midrib, {vein:?}"
                    );
                    // Probed a hair along it: on the midrib itself the other
                    // side's veins answer.
                    assert!(
                        veins.ridge(vein.foot + vein.dir * 1e-6).abs() < 1e-9,
                        "{name} {seed}: a vein is a valley, {vein:?}"
                    );
                }
                let (first, second) = (veins.sides[1][0].foot.x, veins.sides[1][1].foot.x);
                let between = DVec2::new((first + second) / 2.0, 1e-6);
                assert!(
                    veins.ridge(between) > 0.0 && veins.half(between.x, 1.0) > 0.0,
                    "{name} {seed}: the blade puckers between two veins"
                );
            }
        }
    }
}

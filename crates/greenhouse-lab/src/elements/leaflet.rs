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

use std::f64::consts::PI;

use bevy::math::DVec2;
use misina_lab::geometry::outline::Outline;
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

/// How far the margin may stray from the curves it is sampled from, in
/// meters: about a pixel to a camera half a meter away. The points it can do
/// without are dropped, which keeps the lobes and the sinuses, costs the
/// finest teeth, and leaves a small leaflet fewer points than a large one.
const TOLERANCE: f64 = 0.0006;

/// A leaflet's shape. Lengths are fractions of the leaflet's own length, from
/// the petiolule's free end to the apex.
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
}

impl Leaflet {
    /// The leaflet at the tip of a leaf: symmetric and long, lobed all the
    /// way to the apex.
    pub const TERMINAL: Self = Self {
        stalk: 0.1,
        width: 0.25,
        lobes: 5,
        lobing: 0.7,
        angle: (95.0, 35.0),
        lopsided: 0.0,
        serration: 0.004,
        jitter: 0.5,
    };

    /// A lateral leaflet: lopsided, its first lobes deep and the rest of it a
    /// long, toothed point.
    pub const LATERAL: Self = Self {
        stalk: 0.15,
        width: 0.28,
        lobes: 4,
        lobing: 0.85,
        angle: (100.0, 32.0),
        lopsided: 0.15,
        serration: 0.006,
        jitter: 0.5,
    };

    /// One of the small leaflets between the laterals: ovate, with a lobe
    /// either side of its base.
    pub const INTERCALARY: Self = Self {
        stalk: 0.13,
        width: 0.42,
        lobes: 1,
        lobing: 0.15,
        angle: (99.0, 30.0),
        lopsided: 0.05,
        serration: 0.0,
        jitter: 0.5,
    };

    /// The outline, `length` long.
    ///
    /// Draws from `rng` the same amount whatever the jitter: side by side,
    /// the -Y side first, and along a side first every vein's position, angle
    /// and reach, then every lobe's tip half-angle and the depth of the sinus
    /// after it.
    pub fn outline(&self, length: f64, rng: &mut Rng) -> Outline {
        let half = STALK_WIDTH / 2.0;
        let tolerance = TOLERANCE / length.max(1e-6);
        let right = self.side(-1.0, tolerance, rng);
        let left = self.side(1.0, tolerance, rng);
        let ring = std::iter::once(DVec2::new(0.0, -half))
            .chain(right.iter().map(|p| DVec2::new(p.x, -p.y)))
            .chain([DVec2::X])
            .chain(left.iter().rev().copied())
            .chain([DVec2::new(0.0, half)]);
        Outline {
            points: ring
                .map(|p| Point2::new(p.x * length, p.y * length))
                .collect(),
        }
    }

    /// One half of the margin, on the +Y side of the midrib: from where the
    /// blade leaves the petiolule to just short of the apex, within
    /// `tolerance` of its curves. `side` is +1 for the side
    /// [`lopsided`](Self::lopsided) lengthens, -1 for the other.
    fn side(&self, side: f64, tolerance: f64, rng: &mut Rng) -> Vec<DVec2> {
        let (jitter, lobes) = (self.jitter, self.lobes.max(1));
        let blade = 1.0 - self.stalk;
        let width = self.width * (1.0 + side * self.lopsided);
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
            let opening = (OPENING * self.lobing / 2.0).to_radians();
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
            edge(&mut out, rise, self.serration, true);

            // The sinus ahead: cut from the line between the two tips toward
            // the midrib, with a round bottom no wider than half its depth,
            // its walls opening out of it at `opening` either side.
            let base = (1.0 - lerp(vein.u, next.u, LEAN)).powf(FALLOFF);
            let depth = (lerp(DEPTH_AT_APEX, self.lobing, base) * deeper).min(0.9);
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
            edge(&mut out, fall, self.serration, false);
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
        edge(&mut out, rise, self.serration, true);
        for point in &mut out {
            point.y = point.y.max(1e-4); // never across the midrib
        }
        out.push(DVec2::X);
        simplify(&mut out, tolerance);
        untangle(&mut out);
        out.pop();
        out
    }
}

/// Where a vein leaves the midrib, which way it runs, and where it ends.
#[derive(Clone, Copy)]
struct Vein {
    /// How far along the blade it leaves the midrib.
    u: f64,
    foot: DVec2,
    dir: DVec2,
    tip: DVec2,
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
fn edge(out: &mut Vec<DVec2>, [a, b, c, d]: [DVec2; 4], serration: f64, toward_end: bool) {
    let start = out.len();
    out.extend((0..EDGE_POINTS).map(|i| {
        let t = i as f64 / EDGE_POINTS as f64;
        let s = 1.0 - t;
        a * (s * s * s) + b * (3.0 * s * s * t) + c * (3.0 * s * t * t) + d * (t * t * t)
    }));
    serrate(&mut out[start..], serration, toward_end);
}

/// Drops every point of a polyline that strays less than `tolerance` from the
/// chord between the points kept either side of it (Douglas–Peucker). Both
/// ends are kept.
fn simplify(line: &mut Vec<DVec2>, tolerance: f64) {
    let last = line.len() - 1;
    let mut keep = vec![false; line.len()];
    (keep[0], keep[last]) = (true, true);
    let mut spans = vec![(0, last)];
    while let Some((a, b)) = spans.pop() {
        let chord = (line[b] - line[a]).normalize_or(DVec2::X);
        let off = |i: &usize| chord.perp_dot(line[*i] - line[a]).abs();
        let far = (a + 1..b).max_by(|i, j| off(i).total_cmp(&off(j)));
        if let Some(far) = far.filter(|i| off(i) > tolerance) {
            keep[far] = true;
            spans.extend([(a, far), (far, b)]);
        }
    }
    let mut kept = keep.into_iter();
    line.retain(|_| kept.next().unwrap());
}

/// Cuts every loop out of a polyline where it crosses itself, keeping the
/// crossing point. A ragged jitter can fold a slit's walls over each other.
fn untangle(points: &mut Vec<DVec2>) {
    let mut i = 0;
    while i + 3 < points.len() {
        let (a, b) = (points[i], points[i + 1]);
        let hit = (i + 2..points.len() - 1)
            .find_map(|j| crossing(a, b, points[j], points[j + 1]).map(|at| (j, at)));
        match hit {
            Some((j, at)) => drop(points.splice(i + 1..=j, [at])),
            None => i += 1,
        }
    }
}

/// Where segments `a`–`b` and `c`–`d` cross, if they do.
fn crossing(a: DVec2, b: DVec2, c: DVec2, d: DVec2) -> Option<DVec2> {
    let (r, s) = (b - a, d - c);
    let den = r.perp_dot(s);
    if den.abs() < 1e-15 {
        return None;
    }
    let t = (c - a).perp_dot(s) / den;
    let u = (c - a).perp_dot(r) / den;
    ((1e-9..1.0 - 1e-9).contains(&t) && (1e-9..1.0 - 1e-9).contains(&u)).then(|| a + r * t)
}

/// Small sawtooth teeth along a polyline on the +Y side, `depth` deep and
/// [`TOOTH_SPACING`] depths apart, leaning toward its end (or its start) and
/// fading out at both ends.
fn serrate(edge: &mut [DVec2], depth: f64, toward_end: bool) {
    let mut arcs = vec![0.0];
    for pair in edge.windows(2) {
        arcs.push(arcs[arcs.len() - 1] + pair[0].distance(pair[1]));
    }
    let length = arcs[arcs.len() - 1];
    if depth <= 0.0 || length < 2.0 * TOOTH_SPACING * depth {
        return;
    }
    let teeth = (length / (TOOTH_SPACING * depth)).floor();
    let rise = if toward_end { 0.75 } else { 0.25 };
    let outward: Vec<DVec2> = (0..edge.len())
        .map(|i| {
            let along = edge[(i + 1).min(edge.len() - 1)] - edge[i.saturating_sub(1)];
            along.normalize_or_zero().perp()
        })
        .collect();
    for (i, point) in edge.iter_mut().enumerate() {
        let f = (arcs[i] / length * teeth).fract();
        let saw = if f < rise {
            f / rise
        } else {
            (1.0 - f) / (1.0 - rise)
        };
        *point += outward[i] * (depth * saw * (PI * arcs[i] / length).sin());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use misina_lab::geometry::outline::outline_mesh;

    const ALL: [(&str, Leaflet); 3] = [
        ("terminal", Leaflet::TERMINAL),
        ("lateral", Leaflet::LATERAL),
        ("intercalary", Leaflet::INTERCALARY),
    ];

    /// The frame every placed outline depends on, and a ring that fills: no
    /// crossings at any seed, however ragged the jitter makes it, nor at a
    /// size where most of its points are dropped.
    #[test]
    fn every_leaflet_hangs_from_the_origin_and_fills() {
        for (name, shape) in ALL {
            for seed in 0..64 {
                let ragged = Leaflet {
                    jitter: 1.0,
                    ..shape
                };
                let length = if seed % 2 == 0 { 0.1 } else { 0.005 };
                let outline = ragged.outline(length, &mut Rng::new(seed));
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
                outline_mesh(&outline, outline.area() / 20.0)
                    .unwrap_or_else(|e| panic!("{name} {seed} fills: {e:#}"));
            }
        }
    }
}

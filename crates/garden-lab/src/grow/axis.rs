//! Axis — a stem a recipe grows: how it is asked for, and the polyline with
//! radii it becomes, which organs anchor on and the plant layer meshes and
//! collides.

use std::f32::consts::TAU;

use bevy::prelude::*;
use misina_lab::rng::Rng;

use super::builder::leaf_frame;
use super::phyllotaxis::{AxisNode, Spiral};

/// Points along a grown axis per meter, at least this many in all; a twining
/// axis takes more, to round its turns.
const STATIONS_PER_METER: f32 = 30.0;
const TWINE_STATIONS_PER_METER: f32 = 80.0;
const MIN_STATIONS: usize = 4;

/// Where a leaf's base sits on a stem, as a fraction of the stem's radius
/// there: a little inside the surface, so it grows out of the stem.
const ATTACH: f32 = 0.9;

/// An axis as a recipe asks for it.
#[derive(Clone, Copy, Debug)]
pub struct Axis {
    course: Course,
    radius: f32,
    taper: f32,
    lean: f32,
    wobble: f32,
    bends: bool,
    tint: [f32; 3],
}

/// The way an axis runs from its foot.
#[derive(Clone, Copy, Debug)]
enum Course {
    /// Straight up, bar its lean and wobble.
    Upright { height: f32 },
    /// Out along a heading, turning upward on its way: a branch.
    Arc {
        length: f32,
        azimuth: f32,
        elevation: f32,
        rise: f32,
    },
    /// Round and up the line from the foot to `top`: a twining vine.
    Twine {
        top: Vec3,
        reach: f32,
        pitch: f32,
        offset: f32,
    },
}

impl Axis {
    fn new(course: Course, radius: f32) -> Self {
        Self {
            course,
            radius,
            taper: 0.5,
            lean: 0.0,
            wobble: 0.0,
            bends: false,
            tint: [0.3, 0.4, 0.2],
        }
    }

    /// A stem standing `height` up from its foot, `radius` thick at the
    /// base and half that at the tip.
    pub fn upright(height: f32, radius: f32) -> Self {
        Self::new(Course::Upright { height }, radius)
    }

    /// A branch `length` long leaving its foot level along +X and turning
    /// up by 0.8 radians on its way, `radius` thick at the base; see
    /// [`heading`](Self::heading) and [`rise`](Self::rise).
    pub fn arc(length: f32, radius: f32) -> Self {
        Self::new(
            Course::Arc {
                length,
                azimuth: 0.0,
                elevation: 0.0,
                rise: 0.8,
            },
            radius,
        )
    }

    /// A vine twining round the line from its foot to `top`, anticlockwise
    /// seen from above, `reach` meters along the line or to `top` if that is
    /// nearer, `radius` thick at the base; one turn every 0.2 m and 1 cm off
    /// the line unless [`pitch`](Self::pitch) and [`offset`](Self::offset)
    /// say otherwise.
    pub fn twine(top: Vec3, reach: f32, radius: f32) -> Self {
        Self::new(
            Course::Twine {
                top,
                reach,
                pitch: 0.2,
                offset: 0.01,
            },
            radius,
        )
    }

    /// An arc's heading at its foot: `azimuth` round from +X, `elevation`
    /// radians off level.
    pub fn heading(mut self, azimuth: f32, elevation: f32) -> Self {
        debug_assert!(
            matches!(self.course, Course::Arc { .. }),
            "a heading is an arc's"
        );
        if let Course::Arc {
            azimuth: a,
            elevation: e,
            ..
        } = &mut self.course
        {
            (*a, *e) = (azimuth, elevation);
        }
        self
    }

    /// How far an arc turns upward over its length, in radians.
    pub fn rise(mut self, radians: f32) -> Self {
        debug_assert!(
            matches!(self.course, Course::Arc { .. }),
            "a rise is an arc's"
        );
        if let Course::Arc { rise, .. } = &mut self.course {
            *rise = radians;
        }
        self
    }

    /// How far a twine climbs per turn, in meters.
    pub fn pitch(mut self, meters: f32) -> Self {
        debug_assert!(
            matches!(self.course, Course::Twine { .. }),
            "a pitch is a twine's"
        );
        if let Course::Twine { pitch, .. } = &mut self.course {
            *pitch = meters;
        }
        self
    }

    /// How far a twine's centreline stays off its line, in meters: the
    /// string's radius, the vine's and the gap between them.
    pub fn offset(mut self, meters: f32) -> Self {
        debug_assert!(
            matches!(self.course, Course::Twine { .. }),
            "an offset is a twine's"
        );
        if let Course::Twine { offset, .. } = &mut self.course {
            *offset = meters;
        }
        self
    }

    /// The tip's radius as a fraction of the base's.
    pub fn taper(mut self, fraction: f32) -> Self {
        self.taper = fraction;
        self
    }

    /// How far an upright stem leans, in a direction drawn per plant: its
    /// tip stands `height · tan(radians)` off its foot's vertical, the stem
    /// curving into it as a parabola, so the tip's tangent leans twice this.
    pub fn lean(mut self, radians: f32) -> Self {
        debug_assert!(
            matches!(self.course, Course::Upright { .. }),
            "a lean is an upright stem's"
        );
        self.lean = radians;
        self
    }

    /// How far an upright stem strays sideways on its way up, as a fraction
    /// of its height, in a sine bow whose direction and phase are drawn per
    /// plant and which may still displace the tip by that much.
    pub fn wobble(mut self, fraction: f32) -> Self {
        debug_assert!(
            matches!(self.course, Course::Upright { .. }),
            "a wobble is an upright stem's"
        );
        self.wobble = fraction;
        self
    }

    /// The stem's colour, linear RGB.
    pub fn tint(mut self, color: [f32; 3]) -> Self {
        self.tint = color;
        self
    }

    /// Marks the axis one that bends: a rod the solver moves when the
    /// scene's stems are flexible, its organs riding it, and wood like any
    /// other otherwise.
    pub fn bends(mut self) -> Self {
        self.bends = true;
        self
    }

    /// The axis grown from `foot`. Three draws from `rng` whatever its
    /// course: an upright stem's lean direction, bow direction and bow
    /// phase, which a twine takes for its starting angle and an arc leaves
    /// unused, so a recipe's later draws sit the same for every course.
    pub(crate) fn grow(&self, foot: Vec3, rng: &mut Rng) -> Grown {
        let lean_toward = rng.range(0.0, TAU as f64) as f32;
        let bow_toward = rng.range(0.0, TAU as f64) as f32;
        let phase = rng.range(0.0, TAU as f64) as f32;
        let points = match self.course {
            Course::Upright { height } => {
                let height = height.max(0.005);
                let lean = Vec2::from_angle(lean_toward) * height * self.lean.tan();
                let bow = Vec2::from_angle(bow_toward) * height * self.wobble;
                let stations = ((height * STATIONS_PER_METER).ceil() as usize).max(MIN_STATIONS);
                (0..=stations)
                    .map(|i| {
                        let t = i as f32 / stations as f32;
                        // The lean grows with the square of the height, a
                        // curve rather than a tilt; the bow is a sine that
                        // fades in.
                        let across = lean * t * t + bow * t * (TAU * 0.75 * t + phase).sin();
                        foot + Vec3::new(across.x, across.y, height * t)
                    })
                    .collect()
            }
            Course::Arc {
                length,
                azimuth,
                elevation,
                rise,
            } => arc(foot, length, azimuth, elevation, rise),
            Course::Twine {
                top,
                reach,
                pitch,
                offset,
            } => twine(foot, top, reach, pitch, offset, phase),
        };
        let last = points.len() - 1;
        let radii = (0..=last)
            .map(|i| {
                let t = i as f32 / last as f32;
                self.radius.max(1e-4) * (1.0 - t * (1.0 - self.taper))
            })
            .collect();
        Grown {
            points,
            radii,
            tint: self.tint,
            bends: self.bends,
            index: 0,
        }
    }
}

/// `length` of branch from `foot`, heading `azimuth` round from +X and
/// `elevation` off level, its heading rising by `rise` evenly along it.
fn arc(foot: Vec3, length: f32, azimuth: f32, elevation: f32, rise: f32) -> Vec<Vec3> {
    let length = length.max(0.005);
    let out = Vec2::from_angle(azimuth);
    let stations = ((length * STATIONS_PER_METER).ceil() as usize).max(MIN_STATIONS);
    let step = length / stations as f32;
    let mut points = Vec::with_capacity(stations + 1);
    let mut at = foot;
    points.push(at);
    for i in 0..stations {
        // Each step along the heading half way through it: a unit heading,
        // so the length comes out exact.
        let climb = elevation + rise * (i as f32 + 0.5) / stations as f32;
        at += step * Vec3::new(out.x * climb.cos(), out.y * climb.cos(), climb.sin());
        points.push(at);
    }
    points
}

/// A helix round the line from `foot` to `top`, `offset` off it, one turn
/// every `pitch` meters along the line, anticlockwise seen from above,
/// starting `phase` round, `reach` meters along the line or to `top` if
/// that is nearer.
fn twine(foot: Vec3, top: Vec3, reach: f32, pitch: f32, offset: f32, phase: f32) -> Vec<Vec3> {
    let line = top - foot;
    let span = line.length();
    let dir = if span > 1e-4 { line / span } else { Vec3::Z };
    let reach = reach.clamp(0.005, span.max(0.005));
    // A right-handed frame about the line, so a turn from `u` toward `v`
    // is anticlockwise seen from its top.
    let u = dir.any_orthonormal_vector();
    let v = dir.cross(u);
    let stations = ((reach * TWINE_STATIONS_PER_METER).ceil() as usize).max(MIN_STATIONS);
    (0..=stations)
        .map(|i| {
            let s = reach * i as f32 / stations as f32;
            let turn = phase + TAU * s / pitch.max(0.01);
            foot + dir * s + (u * turn.cos() + v * turn.sin()) * offset
        })
        .collect()
}

/// A grown axis: its points and radii in the plant's frame, which organs
/// anchor on and the plant layer builds as wood, or as a rod if it bends
/// and the scene's stems are flexible.
#[derive(Clone, Debug)]
pub struct Grown {
    pub points: Vec<Vec3>,
    pub radii: Vec<f32>,
    pub tint: [f32; 3],
    /// Whether it is a rod when the scene's stems are flexible.
    pub bends: bool,
    /// Its index among the plant's axes, which its anchors name.
    pub index: usize,
}

impl Grown {
    /// Its length along its points.
    pub fn length(&self) -> f32 {
        self.points.windows(2).map(|w| w[0].distance(w[1])).sum()
    }

    /// The point `station` meters along it, and the radius there.
    pub fn at(&self, station: f32) -> (Vec3, f32) {
        let mut left = station.max(0.0);
        for (pair, radii) in self.points.windows(2).zip(self.radii.windows(2)) {
            let span = pair[0].distance(pair[1]);
            if left <= span || span == 0.0 {
                let t = if span > 0.0 { left / span } else { 0.0 };
                return (
                    pair[0].lerp(pair[1], t),
                    radii[0] + (radii[1] - radii[0]) * t,
                );
            }
            left -= span;
        }
        (*self.points.last().unwrap(), *self.radii.last().unwrap())
    }

    /// Its tip.
    pub fn tip(&self) -> Vec3 {
        *self.points.last().unwrap()
    }

    /// The nodes `spiral` puts along it, from the foot up.
    pub fn nodes(&self, spiral: &Spiral) -> Vec<AxisNode> {
        spiral.along(self.length())
    }

    /// Where a leaf at `node` leaves the axis, pointing out along the node's
    /// azimuth and rising `elevation` radians off level, its face turned up
    /// and in.
    pub fn anchor(&self, node: &AxisNode, elevation: f32) -> Anchor {
        let (at, radius) = self.at(node.station);
        let foot = at + (Vec2::from_angle(node.azimuth) * radius * ATTACH).extend(0.0);
        self.on(node, leaf_frame(foot, node.azimuth, elevation))
    }

    /// Where a fruit hangs off the axis at `node`: on its surface `azimuth`
    /// round from +X, its -Z, which the fruit hangs down, tilted `tilt`
    /// radians outward.
    pub fn hang(&self, node: &AxisNode, azimuth: f32, tilt: f32) -> Anchor {
        let (at, radius) = self.at(node.station);
        let foot = at + (Vec2::from_angle(azimuth) * radius).extend(0.0);
        let frame = Transform::from_translation(foot)
            .with_rotation(Quat::from_rotation_z(azimuth) * Quat::from_rotation_y(-tilt));
        self.on(node, frame)
    }

    /// `frame` as an anchor on this axis at `node`.
    fn on(&self, node: &AxisNode, frame: Transform) -> Anchor {
        Anchor {
            frame,
            on: Some((self.index, node.station)),
        }
    }
}

/// Where an organ sits on its plant: its frame in the plant's, and the axis
/// and the station along it that it sits on, if it sits on one, which is
/// what carries it when that axis is a rod.
#[derive(Clone, Copy, Debug)]
pub struct Anchor {
    pub frame: Transform,
    pub on: Option<(usize, f32)>,
}

impl From<Transform> for Anchor {
    /// A frame on nothing that moves: the crown, or the plant itself.
    fn from(frame: Transform) -> Self {
        Self { frame, on: None }
    }
}

#[cfg(test)]
mod tests {
    use std::f32::consts::FRAC_PI_2;

    use super::*;

    /// A grown axis starts at its foot, reaches about its height, leans by
    /// about the angle asked for at the tip, and thins to its taper; a
    /// station part way along interpolates.
    #[test]
    fn an_upright_axis_reaches_its_height_and_leans_at_the_tip() {
        let mut rng = Rng::new(3);
        let axis = Axis::upright(0.6, 0.02).taper(0.5).lean(0.2).wobble(0.0);
        let grown = axis.grow(Vec3::new(0.1, 0.0, 0.0), &mut rng);
        assert_eq!(grown.points[0], Vec3::new(0.1, 0.0, 0.0));
        let tip = grown.tip();
        assert!((tip.z - 0.6).abs() < 1e-5, "{tip}");
        let over = tip.xy() - Vec2::new(0.1, 0.0);
        assert!((over.length() - 0.6 * 0.2f32.tan()).abs() < 1e-4, "{over}");
        assert!((grown.radii[0] - 0.02).abs() < 1e-6);
        assert!((grown.radii.last().unwrap() - 0.01).abs() < 1e-6);
        let (at, radius) = grown.at(grown.length() / 2.0);
        assert!((at.z - 0.3).abs() < 0.02, "{at}");
        assert!((radius - 0.015).abs() < 0.001, "{radius}");
    }

    /// An arc is as long as asked, leaves along its heading and ends headed
    /// up by its rise; a twine stays its clearance off its line, climbs to
    /// its reach, turns anticlockwise seen from above and stops at the
    /// line's top.
    #[test]
    fn an_arc_turns_up_and_a_twine_winds_round_its_line() {
        let mut rng = Rng::new(5);
        let foot = Vec3::new(0.0, 0.0, 0.1);
        let arc = Axis::arc(0.4, 0.01)
            .heading(FRAC_PI_2, 0.2)
            .rise(1.0)
            .grow(foot, &mut rng);
        assert_eq!(arc.points[0], foot);
        assert!((arc.length() - 0.4).abs() < 1e-4, "{}", arc.length());
        let first = arc.points[1] - arc.points[0];
        assert!(first.y > 0.0 && first.x.abs() < 1e-6, "heads +Y: {first}");
        let last = arc.tip() - arc.points[arc.points.len() - 2];
        let climb = last.z.atan2(last.y);
        assert!((climb - 1.2).abs() < 0.1, "ends headed {climb} rad up");

        let top = Vec3::new(0.0, 0.3, 2.0);
        let twine = Axis::twine(top, 1.0, 0.004)
            .pitch(0.25)
            .offset(0.01)
            .grow(Vec3::ZERO, &mut rng);
        let dir = top.normalize();
        let mut turned = 0.0;
        let mut previous: Option<Vec3> = None;
        for point in &twine.points {
            let along = point.dot(dir);
            let off = *point - dir * along;
            assert!((off.length() - 0.01).abs() < 1e-5, "{off}");
            if let Some(before) = previous {
                // Anticlockwise about the line: the turn is a positive
                // rotation about it.
                turned += before.cross(off).dot(dir).signum();
            }
            previous = Some(off);
        }
        assert!(turned > 0.0, "turns anticlockwise");
        let reach = twine.tip().dot(dir);
        assert!((reach - 1.0).abs() < 1e-5, "{reach}");
        let whole = Axis::twine(top, 9.0, 0.004).grow(Vec3::ZERO, &mut rng);
        assert!((whole.tip().dot(dir) - top.length()).abs() < 1e-4);
    }
}

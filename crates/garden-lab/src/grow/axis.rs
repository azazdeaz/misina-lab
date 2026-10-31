//! Axis — a stem a recipe grows: how it is asked for, and the polyline with
//! radii it becomes, which organs anchor on and the plant layer meshes and
//! collides.

use std::f32::consts::TAU;

use bevy::prelude::*;
use misina_lab::rng::Rng;

use super::builder::leaf_frame;
use super::phyllotaxis::{AxisNode, Spiral};

/// Points along a grown axis per meter, at least this many in all.
const STATIONS_PER_METER: f32 = 30.0;
const MIN_STATIONS: usize = 4;

/// Where a leaf's base sits on a stem, as a fraction of the stem's radius
/// there: a little inside the surface, so it grows out of the stem.
const ATTACH: f32 = 0.9;

/// An axis as a recipe asks for it.
#[derive(Clone, Copy, Debug)]
pub struct Axis {
    height: f32,
    radius: f32,
    taper: f32,
    lean: f32,
    wobble: f32,
    tint: [f32; 3],
}

impl Axis {
    /// A stem standing `height` up from its foot, `radius` thick at the
    /// base and half that at the tip.
    pub fn upright(height: f32, radius: f32) -> Self {
        Self {
            height,
            radius,
            taper: 0.5,
            lean: 0.0,
            wobble: 0.0,
            tint: [0.3, 0.4, 0.2],
        }
    }

    /// The tip's radius as a fraction of the base's.
    pub fn taper(mut self, fraction: f32) -> Self {
        self.taper = fraction;
        self
    }

    /// How far the stem leans, in a direction drawn per plant: its tip
    /// stands `height · tan(radians)` off its foot's vertical, the stem
    /// curving into it as a parabola, so the tip's tangent leans twice this.
    pub fn lean(mut self, radians: f32) -> Self {
        self.lean = radians;
        self
    }

    /// How far the stem strays sideways on its way up, as a fraction of its
    /// height, in a sine bow whose direction and phase are drawn per plant
    /// and which may still displace the tip by that much.
    pub fn wobble(mut self, fraction: f32) -> Self {
        self.wobble = fraction;
        self
    }

    /// The stem's colour, linear RGB.
    pub fn tint(mut self, color: [f32; 3]) -> Self {
        self.tint = color;
        self
    }

    /// The axis grown from `foot`, its lean's direction, its bow's direction
    /// and its bow's phase drawn from `rng` in that order.
    pub(crate) fn grow(&self, foot: Vec3, rng: &mut Rng) -> Grown {
        let height = self.height.max(0.005);
        let lean_toward = rng.range(0.0, TAU as f64) as f32;
        let bow_toward = rng.range(0.0, TAU as f64) as f32;
        let phase = rng.range(0.0, TAU as f64) as f32;
        let lean = Vec2::from_angle(lean_toward) * height * self.lean.tan();
        let bow = Vec2::from_angle(bow_toward) * height * self.wobble;
        let stations = ((height * STATIONS_PER_METER).ceil() as usize).max(MIN_STATIONS);
        let points: Vec<Vec3> = (0..=stations)
            .map(|i| {
                let t = i as f32 / stations as f32;
                // The lean grows with the square of the height, a curve
                // rather than a tilt; the bow is a sine that fades in.
                let across = lean * t * t + bow * t * (TAU * 0.75 * t + phase).sin();
                foot + Vec3::new(across.x, across.y, height * t)
            })
            .collect();
        let radii = (0..=stations)
            .map(|i| {
                let t = i as f32 / stations as f32;
                self.radius.max(1e-4) * (1.0 - t * (1.0 - self.taper))
            })
            .collect();
        Grown {
            points,
            radii,
            tint: self.tint,
        }
    }
}

/// A grown axis: its points and radii in the plant's frame, which organs
/// anchor on and the plant layer meshes as `Wood` and collides.
#[derive(Clone, Debug)]
pub struct Grown {
    pub points: Vec<Vec3>,
    pub radii: Vec<f32>,
    pub tint: [f32; 3],
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
    pub fn anchor(&self, node: &AxisNode, elevation: f32) -> Transform {
        let (at, radius) = self.at(node.station);
        let foot = at + (Vec2::from_angle(node.azimuth) * radius * ATTACH).extend(0.0);
        leaf_frame(foot, node.azimuth, elevation)
    }
}

#[cfg(test)]
mod tests {
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
}

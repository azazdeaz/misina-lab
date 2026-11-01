//! Solids — fruit and roots: a profile turned about an axis, every one
//! hanging from the origin down -Z.

use std::f32::consts::TAU;

use bevy::prelude::*;
use misina_lab::geometry::mesh::MeshData;

use crate::elements::fruit::FruitConfig;

/// Points round a ring, and rings along a solid.
const SECTORS: usize = 12;
const RINGS: usize = 10;

/// How blunt a root's shoulder is: the exponent its profile falls off with.
const SHOULDER: f32 = 1.6;

/// A pod: its rings, how square its ends are (the exponent its profile
/// falls to the tips with), how much thinner it gets from its middle to its
/// tip, and how far its tip swings aside as a fraction of its length.
const POD_RINGS: usize = 14;
const POD_END: f32 = 6.0;
const POD_TAPER: f32 = 0.35;
const POD_CURVE: f32 = 0.08;

/// Rings along a solid [`turned`] from a profile.
const PROFILE_RINGS: usize = 12;

/// A berry: how round its ends are (the exponent its profile falls to them
/// with), and how much narrower it is at its stalk than at its widest.
const BERRY_END: f32 = 3.0;
const BERRY_NECK: f32 = 0.4;

/// A strawberry: how round its shoulder is, the exponent its profile rises
/// from the stalk with, and how fast it narrows to its tip, the power the
/// way left is raised to.
const STRAWBERRY_SHOULDER: f32 = 3.0;
const STRAWBERRY_TAPER: f32 = 0.6;

/// A closed surface through `rings`, from `top` down to `bottom`, every ring
/// the same number of points counter-clockwise seen from above. Faces wind
/// outward.
pub fn lathe(top: Vec3, rings: &[Vec<Vec3>], bottom: Vec3) -> MeshData {
    let n = rings[0].len() as i32;
    let last = rings.len() - 1;
    let at = |j: usize, i: i32| 1 + j as i32 * n + i.rem_euclid(n);
    let end = 1 + rings.len() as i32 * n;
    let mut points = vec![top.to_array()];
    points.extend(rings.iter().flatten().map(|p| p.to_array()));
    points.push(bottom.to_array());

    let (mut counts, mut indices) = (Vec::new(), Vec::new());
    for i in 0..n {
        counts.extend([3, 3]);
        indices.extend([0, at(0, i), at(0, i + 1)]);
        indices.extend([at(last, i), end, at(last, i + 1)]);
    }
    for j in 0..last {
        for i in 0..n {
            counts.push(4);
            indices.extend([at(j, i), at(j + 1, i), at(j + 1, i + 1), at(j, i + 1)]);
        }
    }
    MeshData {
        points,
        face_vertex_counts: counts,
        face_vertex_indices: indices,
    }
}

/// A ring of [`SECTORS`] points `radius` out, at `z`.
fn ring(radius: f32, z: f32) -> Vec<Vec3> {
    (0..SECTORS)
        .map(|i| {
            let a = TAU * i as f32 / SECTORS as f32;
            Vec3::new(radius * a.cos(), radius * a.sin(), z)
        })
        .collect()
}

/// A taproot hanging from the origin down -Z: as wide as its shoulder near
/// the top, tapering to a point at its length.
pub fn root(config: &FruitConfig) -> MeshData {
    let (radius, length) = (config.diameter.max(1e-3) / 2.0, config.length.max(1e-3));
    let rings: Vec<Vec<Vec3>> = (0..RINGS)
        .map(|j| {
            let t = (j as f32 + 0.5) / RINGS as f32;
            ring(radius * (1.0 - t.powf(SHOULDER)).sqrt(), -length * t)
        })
        .collect();
    lathe(Vec3::ZERO, &rings, Vec3::new(0.0, 0.0, -length))
}

/// `profile`, the radius `t` of the way down as a fraction of the widest,
/// sampled at [`PROFILE_RINGS`] rings and turned about -Z from the origin
/// down to `length`, the widest ring `radius` out.
fn turned(config: &FruitConfig, profile: impl Fn(f32) -> f32) -> MeshData {
    let (radius, length) = (config.diameter.max(1e-3) / 2.0, config.length.max(1e-3));
    let stations: Vec<f32> = (0..PROFILE_RINGS)
        .map(|j| (j as f32 + 0.5) / PROFILE_RINGS as f32)
        .collect();
    let widest = stations.iter().map(|&t| profile(t)).fold(0.0, f32::max);
    let rings: Vec<Vec<Vec3>> = stations
        .iter()
        .map(|&t| ring(radius * profile(t) / widest, -length * t))
        .collect();
    lathe(Vec3::ZERO, &rings, Vec3::new(0.0, 0.0, -length))
}

/// A berry hanging from the origin down -Z: a cylinder rounded at both
/// ends, its diameter toward its far end and narrowing to its stalk.
pub fn berry(config: &FruitConfig) -> MeshData {
    turned(config, |t| {
        let ends = (1.0 - t.powf(BERRY_END)).sqrt() * (1.0 - (1.0 - t).powf(BERRY_END)).sqrt();
        ends * (1.0 - BERRY_NECK * (1.0 - t))
    })
}

/// A strawberry hanging from the origin down -Z: a shoulder rounding out
/// from its stalk, widest a third of the way down and narrowing to a point
/// at its tip.
pub fn strawberry(config: &FruitConfig) -> MeshData {
    turned(config, |t| {
        let left = 1.0 - t;
        (1.0 - left.powf(STRAWBERRY_SHOULDER)).sqrt() * left.powf(STRAWBERRY_TAPER)
    })
}

/// A pod hanging from the origin down -Z: a slender solid rounded at both
/// ends, its diameter across its upper half, thinning from its middle to
/// its tip and swinging a little toward +X on its way down.
pub fn pod(config: &FruitConfig) -> MeshData {
    let (radius, length) = (config.diameter.max(1e-3) / 2.0, config.length.max(1e-3));
    let aside = |t: f32| POD_CURVE * length * t * t;
    let rings: Vec<Vec<Vec3>> = (0..POD_RINGS)
        .map(|j| {
            let t = (j as f32 + 0.5) / POD_RINGS as f32;
            let ends = (1.0 - t.powf(POD_END)).sqrt() * (1.0 - (1.0 - t).powf(POD_END)).sqrt();
            let r = radius * ends * (1.0 - POD_TAPER * (2.0 * t - 1.0).max(0.0));
            (0..SECTORS)
                .map(|i| {
                    let a = TAU * i as f32 / SECTORS as f32;
                    Vec3::new(aside(t) + r * a.cos(), r * a.sin(), -length * t)
                })
                .collect()
        })
        .collect();
    lathe(Vec3::ZERO, &rings, Vec3::new(aside(1.0), 0.0, -length))
}

#[cfg(test)]
mod tests {
    use misina_lab::testing::bounds;

    use super::*;

    /// A root hangs from the origin to its length and is as wide as its
    /// diameter near the shoulder.
    #[test]
    fn a_root_hangs_from_its_shoulder_to_its_tip() {
        let mesh = root(&FruitConfig::root(0.03, 0.18));
        assert_eq!(bounds(&mesh, 2), (-0.18, 0.0));
        let (x0, x1) = bounds(&mesh, 0);
        assert!(
            x1 > 0.0145 && x1 <= 0.015 && (x0 + x1).abs() < 1e-6,
            "{x0}..{x1}"
        );
        assert_eq!(
            mesh.face_vertex_counts.len(),
            2 * SECTORS + (RINGS - 1) * SECTORS
        );
    }

    /// A berry hangs its length, is as wide as its diameter below its middle
    /// and well under it at its stalk.
    #[test]
    fn a_berry_is_widest_below_its_middle_and_narrow_at_its_stalk() {
        let mesh = berry(&FruitConfig::berry(0.05, 0.2));
        assert_eq!(bounds(&mesh, 2), (-0.2, 0.0));
        let (y0, y1) = bounds(&mesh, 1);
        assert!(
            y1 > 0.0249 && y1 <= 0.025 && (y0 + y1).abs() < 1e-6,
            "{y0}..{y1}"
        );
        let widest = mesh.points.iter().find(|p| p[1] > 0.0249).unwrap()[2];
        assert!(widest < -0.1, "widest at {widest}");
        let stalk_end = -0.2 * 0.5 / PROFILE_RINGS as f32;
        let neck = mesh
            .points
            .iter()
            .filter(|p| (p[2] - stalk_end).abs() < 1e-5)
            .map(|p| p[1].abs())
            .fold(0.0, f32::max);
        assert!(neck > 0.005 && neck < 0.6 * 0.025, "{neck}");
    }

    /// A strawberry hangs its length, is as wide as its diameter in its
    /// upper half and narrows to a point: its last ring well under half its
    /// widest.
    #[test]
    fn a_strawberry_is_widest_in_its_upper_half_and_pointed() {
        let mesh = strawberry(&FruitConfig::strawberry(0.025, 0.03));
        assert_eq!(bounds(&mesh, 2), (-0.03, 0.0));
        let (y0, y1) = bounds(&mesh, 1);
        assert!(
            y1 > 0.01249 && y1 <= 0.0125 + 1e-6 && (y0 + y1).abs() < 1e-6,
            "{y0}..{y1}"
        );
        let widest = mesh.points.iter().find(|p| p[1] > 0.01249).unwrap()[2];
        assert!(widest > -0.015 && widest < -0.005, "widest at {widest}");
        let tip_ring = -0.03 * (PROFILE_RINGS as f32 - 0.5) / PROFILE_RINGS as f32;
        let tip = mesh
            .points
            .iter()
            .filter(|p| (p[2] - tip_ring).abs() < 1e-6)
            .map(|p| p[1].abs())
            .fold(0.0, f32::max);
        assert!(tip > 0.0 && tip < 0.4 * 0.0125, "{tip}");
    }

    /// A pod hangs its length, is as wide as its diameter about its middle,
    /// and swings its tip aside by its curve, its last ring's radius past
    /// that.
    #[test]
    fn a_pod_hangs_its_length_and_swings_aside() {
        let mesh = pod(&FruitConfig::pod(0.012, 0.15));
        assert_eq!(bounds(&mesh, 2), (-0.15, 0.0));
        let (y0, y1) = bounds(&mesh, 1);
        assert!(
            y1 > 0.0055 && y1 <= 0.006 && (y0 + y1).abs() < 1e-6,
            "{y0}..{y1}"
        );
        let (x0, x1) = bounds(&mesh, 0);
        let swing = POD_CURVE * 0.15;
        assert!(
            x0 > -0.006 && x1 > swing && x1 < swing + 0.006,
            "{x0}..{x1}"
        );
    }
}

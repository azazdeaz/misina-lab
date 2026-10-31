//! Solids — fruit and roots: a profile turned about an axis.

use std::f32::consts::TAU;

use bevy::prelude::*;
use misina_lab::geometry::mesh::MeshData;

use crate::elements::fruit::FruitConfig;

/// Points round a ring, and rings along a solid.
const SECTORS: usize = 12;
const RINGS: usize = 10;

/// How blunt a root's shoulder is: the exponent its profile falls off with.
const SHOULDER: f32 = 1.6;

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

/// A taproot hanging from the origin down -Z: as wide as its shoulder near
/// the top, tapering to a point at its length.
pub fn root(config: &FruitConfig) -> MeshData {
    let (radius, length) = (config.diameter.max(1e-3) / 2.0, config.length.max(1e-3));
    let rings: Vec<Vec<Vec3>> = (0..RINGS)
        .map(|j| {
            let t = (j as f32 + 0.5) / RINGS as f32;
            let r = radius * (1.0 - t.powf(SHOULDER)).sqrt();
            (0..SECTORS)
                .map(|i| {
                    let a = TAU * i as f32 / SECTORS as f32;
                    Vec3::new(r * a.cos(), r * a.sin(), -length * t)
                })
                .collect()
        })
        .collect();
    lathe(Vec3::ZERO, &rings, Vec3::new(0.0, 0.0, -length))
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
}

//! Blades — a leaf in its own frame: the anchor at the origin, +X along the
//! petiole and midrib, the face up +Z.
//!
//! A simple leaf is an obovate outline filled flat and deformed: the margin
//! ruffled by waves that fade toward the midrib, the blade cupped about its
//! midrib and its tip drooped, on a stalk when the template has one. A
//! compound leaf is a rachis with toothed leaflets in pairs along it and one
//! at its tip, the whole drooped. A ruffle displaces the margin out of the
//! plane rather than lengthening it, which a flat triangulation cannot do.

use std::f64::consts::{FRAC_PI_2, PI, TAU};

use bevy::prelude::*;
use misina_lab::geometry::mesh::{MeshData, bend, cylinder_mesh, merge_meshes};
use misina_lab::geometry::outline::{Outline, outline_mesh};
use misina_lab::geometry::shapes::{bent, toothed};
use misina_lab::rng::Rng;
use spade::Point2;

use crate::elements::leaf::{LeafConfig, Template};

/// Stations along each edge of a simple blade's outline: enough that a
/// ruffle's waves have margin points to ride.
const STATIONS: usize = 56;

/// Narrowest a blade's base is held, as a fraction of the half-width: the
/// stub it attaches by; and where it is widest, as a fraction of the way
/// along: obovate.
const BASE: f64 = 0.12;
const PEAK: f64 = 0.62;

/// Triangles a simple blade is filled at, and a leaflet.
pub const TRIANGLES: f64 = 360.0;
const LEAFLET_TRIANGLES: f64 = 24.0;

/// Waves summed into the ruffle, their wavelengths as fractions of the
/// blade's length at no ruffle, how far a full ruffle shortens them, and
/// how steep a wave is at full ruffle: its depth over its wavelength, so a
/// short wave is as steep as a long one.
const WAVES: usize = 3;
const WAVELENGTH: (f64, f64) = (0.12, 0.35);
const CURLING: f64 = 0.5;
const STEEPNESS: f64 = 0.35;

/// How far off the midrib the ruffle reaches its full depth, as a fraction
/// of the half-width there: the midrib is stiff.
const CLEAR: f64 = 0.6;

/// The stream a blade's ruffle phases are drawn from, salted with its seed.
const RUFFLE_STREAM: u64 = 0x3C6E_F372_FE94_F82B;

/// A stalk's radius as a fraction of the leaf's length, its sides, and how
/// much it thins toward the blade.
const STALK_RADIUS: f64 = 0.012;
const RACHIS_RADIUS: f64 = 0.006;
const STALK_SIDES: usize = 5;
const STALK_TAPER: f64 = 0.7;

/// A compound leaf's leaflets: how far each pair spreads off the rachis and
/// rises off its plane, in radians, how much shorter the pairs get toward
/// the tip, each leaflet's width as a fraction of its length and its teeth,
/// and how it folds.
const SPREAD: f32 = 1.0;
const RISE: f32 = 0.25;
const TIP_SHRINK: f32 = 0.45;
const LEAFLET_WIDTH: f64 = 0.45;
const LEAFLET_TEETH: usize = 3;
const LEAFLET_DROOP: f64 = 0.3;
const LEAFLET_KEEL: f64 = 8.0;

/// A leaf from its config.
pub fn mesh(config: &LeafConfig) -> anyhow::Result<MeshData> {
    match config.template {
        Template::Simple => simple(config),
        Template::Compound => compound(config),
    }
}

/// The blade's half-width `t` of the way along, as a fraction of its
/// widest: a hump peaking at `peak`.
fn profile(t: f64, peak: f64) -> f64 {
    let peak = peak.clamp(0.05, 0.95);
    // `sin(pi * t^p)` peaks where `t^p = 1/2`, so `p` is solved from the peak.
    let p = 0.5f64.ln() / peak.ln();
    (PI * t.powf(p)).sin().max(0.0)
}

/// The ring: down the -Y edge from the base to the tip, back up the +Y
/// edge, closing across the base.
fn outline(length: f64, width: f64, peak: f64) -> Outline {
    let half = |i: usize| {
        let t = i as f64 / STATIONS as f64;
        let fraction = if i == 0 {
            profile(0.0, peak).max(BASE)
        } else {
            profile(t, peak)
        };
        fraction * width / 2.0
    };
    let x = |i: usize| length * i as f64 / STATIONS as f64;
    let mut points = Vec::with_capacity(2 * STATIONS + 1);
    for i in 0..STATIONS {
        points.push(Point2::new(x(i), -half(i)));
    }
    points.push(Point2::new(length, 0.0));
    for i in (1..STATIONS).rev() {
        points.push(Point2::new(x(i), half(i)));
    }
    points.push(Point2::new(0.0, half(0)));
    Outline { points }
}

/// A stalk from the origin out along +X, `length` long and `radius` thick
/// at its foot.
fn stalk(length: f64, radius: f64) -> MeshData {
    cylinder_mesh(
        radius as f32,
        (radius * STALK_TAPER) as f32,
        length as f32,
        STALK_SIDES,
    )
    .transformed(&Transform::from_rotation(Quat::from_rotation_y(
        FRAC_PI_2 as f32,
    )))
}

/// `mesh` moved by `offset` and turned by `rotation`.
fn placed(mesh: &MeshData, offset: Vec3, rotation: Quat) -> MeshData {
    mesh.transformed(&Transform::from_translation(offset).with_rotation(rotation))
}

/// A simple blade on its petiole, if it has one.
///
/// The ruffle's waves are drawn in order: each wave's wavelength, then its
/// phase.
fn simple(config: &LeafConfig) -> anyhow::Result<MeshData> {
    let whole = config.length.max(0.005) as f64;
    let petiole = (config.petiole as f64).clamp(0.0, 0.9 * whole);
    let length = whole - petiole;
    let width = (config.width as f64 * whole).max(0.001);
    let ring = outline(length, width, PEAK);
    let mut mesh = outline_mesh(&ring, ring.area() / TRIANGLES)?;

    let mut rng = Rng::new(RUFFLE_STREAM ^ config.seed);
    let waves: Vec<(f64, f64, f64)> = (0..WAVES)
        .map(|_| {
            let ruffle = config.ruffle as f64;
            let wavelength =
                rng.range(WAVELENGTH.0, WAVELENGTH.1) * length * (1.0 - CURLING * ruffle);
            let depth = STEEPNESS * ruffle * wavelength / WAVES as f64;
            (TAU / wavelength, rng.range(0.0, TAU), depth)
        })
        .collect();
    // The edges turn `cup` radians over the half-width, and the tip `droop`
    // radians over the length.
    let keel = config.cup as f64 / (width / 2.0);
    let droop = config.droop as f64 / length;

    for point in &mut mesh.points {
        let (mut x, mut y, mut z) = (point[0] as f64, point[1] as f64, point[2] as f64);
        let half = profile((x / length).clamp(0.0, 1.0), PEAK).max(BASE) * width / 2.0;
        let margin = (y.abs() / (CLEAR * half)).min(1.0).powi(2);
        z += margin
            * waves
                .iter()
                .map(|(k, phase, depth)| depth * (k * x + phase).sin())
                .sum::<f64>();
        if keel != 0.0 {
            (y, z) = bend(y, z, -keel);
        }
        if droop != 0.0 {
            (x, z) = bend(x, z, droop);
        }
        *point = [(x + petiole) as f32, y as f32, z as f32];
    }
    if petiole > 0.0 {
        mesh = merge_meshes(&[stalk(petiole, STALK_RADIUS * whole), mesh]);
    }
    Ok(mesh)
}

/// A compound leaf: a rachis with `pairs` pairs of toothed leaflets along
/// it, shorter toward the tip, and one at its tip, the whole drooped.
fn compound(config: &LeafConfig) -> anyhow::Result<MeshData> {
    let whole = config.length.max(0.005) as f64;
    let pairs = config.pairs.max(1);
    let leaflet = |i: u32| -> f64 {
        config.width as f64 * whole * (1.0 - TIP_SHRINK as f64 * i as f64 / pairs as f64)
    };
    // The rachis runs to the terminal leaflet's base, and the petiole leaves
    // room on it for the pairs.
    let petiole = (config.petiole as f64).clamp(0.0, 0.9 * (whole - leaflet(pairs)).max(0.0));
    let cut = |length: f64| -> anyhow::Result<MeshData> {
        bent(
            toothed(length, LEAFLET_WIDTH * length, 0.45, LEAFLET_TEETH),
            LEAFLET_DROOP,
            LEAFLET_KEEL,
            LEAFLET_TRIANGLES,
        )
    };
    let terminal = leaflet(pairs);
    let mut parts = vec![stalk(whole - terminal, RACHIS_RADIUS * whole)];
    for i in 0..pairs {
        let station = petiole + (whole - terminal - petiole) * (i as f64 + 0.5) / pairs as f64;
        let blade = cut(leaflet(i))?;
        for side in [1.0f32, -1.0] {
            parts.push(placed(
                &blade,
                Vec3::new(station as f32, 0.0, 0.0),
                Quat::from_rotation_z(side * SPREAD) * Quat::from_rotation_y(-RISE),
            ));
        }
    }
    parts.push(placed(
        &cut(terminal)?,
        Vec3::new((whole - terminal) as f32, 0.0, 0.0),
        Quat::from_rotation_y(-RISE),
    ));
    let mut mesh = merge_meshes(&parts);
    let droop = config.droop as f64 / whole;
    if droop != 0.0 {
        for point in &mut mesh.points {
            let (x, z) = bend(point[0] as f64, point[2] as f64, droop);
            point[0] = x as f32;
            point[2] = z as f32;
        }
    }
    Ok(mesh)
}

#[cfg(test)]
mod tests {
    use misina_lab::testing::bounds;

    use super::*;

    /// A blade starts at the origin, reaches about its length however it is
    /// deformed, keeps inside its width, and stays a modest mesh; a flat one
    /// is flat and a ruffled one is not; a petiole puts the blade further out.
    #[test]
    fn a_blade_reaches_its_length_inside_its_width() {
        let flat = LeafConfig::simple(0.18, 0.75).tint([0.1, 0.3, 0.05]);
        let ruffled = flat.ruffle(1.0);
        let folded = ruffled.cup(1.4).droop(1.2);
        let stalked = flat.petiole(0.06);
        for (what, config) in [
            ("flat", flat),
            ("ruffled", ruffled),
            ("folded", folded),
            ("stalked", stalked),
        ] {
            let mesh = mesh(&config).unwrap();
            let (x0, x1) = bounds(&mesh, 0);
            let (y0, y1) = bounds(&mesh, 1);
            assert!(x0.abs() < 1e-6, "{what}: starts at {x0}");
            assert!(
                x1 > 0.7 * config.length && x1 <= config.length + 1e-6,
                "{what}: reaches {x1}"
            );
            // A cup rolls a ruffled edge a little outward: the arc is exact
            // only for a flat sheet.
            let half = 1.05 * config.width * config.length / 2.0;
            assert!(y0 >= -half && y1 <= half, "{what}: {y0}..{y1}");
            let triangles = mesh.face_vertex_counts.len();
            assert!(triangles < 900, "{what}: {triangles} triangles");
        }
        let depth = |config: &LeafConfig| {
            let (z0, z1) = bounds(&mesh(config).unwrap(), 2);
            z1 - z0
        };
        assert!(depth(&flat) < 1e-6);
        assert!(depth(&ruffled) > 0.005, "{}", depth(&ruffled));
        let (x0, _) = bounds(&blade_only(&stalked), 0);
        assert!((x0 - 0.06).abs() < 1e-5, "the blade starts at {x0}");
    }

    /// The blade part of a stalked leaf: everything past the stalk's sides.
    fn blade_only(config: &LeafConfig) -> MeshData {
        let mut mesh = mesh(config).unwrap();
        let stalk_points = stalk(1.0, 1.0).points.len();
        mesh.points.drain(..stalk_points);
        mesh
    }

    /// A compound leaf reaches about its length, spreads to both sides, and
    /// droops its tip below its base when asked.
    #[test]
    fn a_compound_leaf_spreads_its_leaflets_both_sides() {
        let leaf = LeafConfig::compound(0.3, 5, 0.3).petiole(0.08);
        let mesh = mesh(&leaf).unwrap();
        let (x0, x1) = bounds(&mesh, 0);
        let (y0, y1) = bounds(&mesh, 1);
        assert!(
            x0.abs() < 1e-6 && (0.85..=1.0).contains(&(x1 / 0.3)),
            "{x0}..{x1}"
        );
        assert!(
            y0 < -0.03 && y1 > 0.03 && (y0 + y1).abs() < 1e-4,
            "{y0}..{y1}"
        );
        let drooped = super::mesh(&leaf.droop(1.5)).unwrap();
        assert!(bounds(&drooped, 2).0 < -0.05, "{:?}", bounds(&drooped, 2));
        assert!(mesh.face_vertex_counts.len() < 1200);
    }
}

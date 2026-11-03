//! Solids — fruit and roots: a profile turned about an axis and painted, a
//! colour per point, every one hanging from the origin down -Z, below its
//! stalk if it has one. How finely a solid is turned is the scene's
//! `detail`.

use std::f32::consts::TAU;

use bevy::prelude::*;
use misina_lab::geometry::mesh::{MeshData, cylinder_mesh, lathe, merge_meshes};
use misina_lab::geometry::outline::outline_mesh;
use misina_lab::geometry::shapes::{fold, leaf};
use misina_lab::palette::{mix, srgb};

use crate::elements::fruit::{FruitConfig, Kind};
use crate::grow::builder::smooth;

/// Points round a ring and rings along a solid at a detail of 1, both
/// scaling with its square root, and the fewest of either.
const SECTORS: f32 = 16.0;
const RINGS: f32 = 12.0;
const MIN_SECTORS: usize = 8;
const MIN_RINGS: usize = 6;

/// How blunt a root's shoulder is, the exponent its profile falls off with,
/// and how far down it its shoulder colour reaches, as a fraction.
const SHOULDER: f32 = 1.6;
const ROOT_SHOULDER: f32 = 0.15;

/// A pod: its rings at a detail of 1, how square its ends are (the exponent
/// its profile falls to the tips with), how much thinner it gets from its
/// middle to its tip, how far its tip swings aside as a fraction of its
/// length, the beans along it and how far each swells it, as a fraction of
/// its radius.
const POD_RINGS: f32 = 14.0;
const POD_END: f32 = 6.0;
const POD_TAPER: f32 = 0.35;
const POD_CURVE: f32 = 0.08;
const BEANS: f32 = 5.0;
const BEAN_SWELL: f32 = 0.08;

/// A berry: how round its ends are (the exponent its profile falls to them
/// with), how much narrower it is at its stalk than at its widest, the pale
/// stripes round it and how far they pale the skin toward its shoulder
/// colour, and how far down from the stalk the shoulder colour reaches.
const BERRY_END: f32 = 3.0;
const BERRY_NECK: f32 = 0.4;
const STRIPES: f32 = 9.0;
const STRIPE: f32 = 0.35;
const BERRY_SHOULDER: f32 = 0.12;

/// A strawberry: how round its shoulder is, the exponent its profile rises
/// from the stalk with; how fast it narrows to its tip, the power the way
/// left is raised to; over how much of its length its colour front blends;
/// how deep its achenes pit it, as a fraction of its radius, how far they
/// turn it their colour, and between where along it they sit.
const STRAWBERRY_SHOULDER: f32 = 3.0;
const STRAWBERRY_TAPER: f32 = 0.6;
const FRONT_BAND: f32 = 0.3;
const ACHENE_DEPTH: f32 = 0.05;
const ACHENE_PALE: f32 = 0.5;
const ACHENES: (f32, f32) = (0.1, 0.95);

/// A strawberry's calyx: its sepals, each so long as a fraction of the
/// berry's diameter and so wide as a fraction of its length, widest so far
/// along, tilted so far down off level where it leaves the stalk and
/// drooping so far further by its tip, in radians, and this far off the
/// skin, in meters.
const SEPALS: usize = 7;
const SEPAL: f32 = 0.5;
const SEPAL_WIDTH: f32 = 0.35;
const SEPAL_PEAK: f64 = 0.35;
const SEPAL_TILT: f32 = 0.25;
const SEPAL_DROOP: f32 = 0.5;
const SEPAL_GAP: f32 = 0.0008;

/// The colours no config gives: a strawberry's achenes and its calyx, and a
/// stalk.
const ACHENE: u32 = 0xD8C060;
const CALYX: u32 = 0x4A7A32;
const STALK: u32 = 0x6E9A48;

/// A stalk's sides, and how much thinner it is at the body than at the
/// anchor.
const STALK_SIDES: usize = 6;
const STALK_TAPER: f32 = 0.8;

/// A fruit's solid from its config, turned at `detail` and painted, hanging
/// below its stalk if it has one.
pub fn mesh(config: &FruitConfig, detail: f32) -> anyhow::Result<MeshData> {
    let body = match config.kind {
        Kind::Root => root(config, detail),
        Kind::Pod => pod(config, detail),
        Kind::Berry => berry(config, detail),
        Kind::Strawberry => strawberry(config, detail)?,
    };
    Ok(stalked(config, body))
}

/// `body` hung below the config's stalk, a tube from the anchor down to it,
/// thinner at the body; as it is without one.
fn stalked(config: &FruitConfig, body: MeshData) -> MeshData {
    if config.stalk <= 0.0 {
        return body;
    }
    let down = Transform::from_xyz(0.0, 0.0, -config.stalk);
    let radius = config.stalk_radius();
    let stalk = cylinder_mesh(radius * STALK_TAPER, radius, config.stalk, STALK_SIDES)
        .transformed(&down)
        .painted(srgb(STALK));
    merge_meshes(&[stalk, body.transformed(&down)])
}

/// Rings along a solid that has `rings` at a detail of 1, and points round
/// each, at `detail`.
fn counts(detail: f32, rings: f32) -> (usize, usize) {
    let scale = detail.max(0.01).sqrt();
    (
        ((rings * scale).round() as usize).max(MIN_RINGS),
        ((SECTORS * scale).round() as usize).max(MIN_SECTORS),
    )
}

/// `profile`, the radius `t` of the way down as a fraction of the widest,
/// sampled at the rings `counts` gives for `rings` and turned about -Z from
/// the origin down to the config's length, the widest ring its radius out
/// and every ring `aside(t)` off the axis along +X. `skin(t, angle,
/// pitted)` is how far a point is pushed in or out of the profile, as a
/// fraction, and its colour; `pitted` marks every fourth point, on a
/// staggered lattice whose every neighbour is unmarked, so a pitted skin
/// shows dimples rather than the stripes a checkerboard blends into. The
/// apexes take the skin at either end.
fn turned(
    config: &FruitConfig,
    detail: f32,
    rings: f32,
    profile: impl Fn(f32) -> f32,
    aside: impl Fn(f32) -> f32,
    skin: impl Fn(f32, f32, bool) -> (f32, [f32; 3]),
) -> MeshData {
    let (rings, sectors) = counts(detail, rings);
    let (radius, length) = (config.diameter.max(1e-3) / 2.0, config.length.max(1e-3));
    let stations: Vec<f32> = (0..rings)
        .map(|j| (j as f32 + 0.5) / rings as f32)
        .collect();
    let widest = stations
        .iter()
        .map(|&t| profile(t))
        .fold(0.0, f32::max)
        .max(1e-6);
    let mut colors = vec![skin(0.0, 0.0, false).1];
    let mut turned = Vec::with_capacity(rings);
    for (j, &t) in stations.iter().enumerate() {
        let mut ring = Vec::with_capacity(sectors);
        for i in 0..sectors {
            let angle = TAU * i as f32 / sectors as f32;
            let (push, color) = skin(t, angle, (i + 2 * j) % 4 == 0);
            colors.push(color);
            let r = radius * profile(t) / widest * (1.0 + push);
            ring.push(Vec3::new(
                aside(t) + r * angle.cos(),
                r * angle.sin(),
                -length * t,
            ));
        }
        turned.push(ring);
    }
    colors.push(skin(1.0, 0.0, false).1);
    let mut mesh = lathe(Vec3::ZERO, &turned, Vec3::new(aside(1.0), 0.0, -length));
    mesh.colors = Some(colors);
    mesh
}

/// A taproot hanging from the origin down -Z: as wide as its shoulder near
/// the top, tapering to a point at its length, its shoulder the shoulder
/// colour over the skin's.
fn root(config: &FruitConfig, detail: f32) -> MeshData {
    let shoulder = config.shoulder.unwrap_or(config.tint);
    turned(
        config,
        detail,
        RINGS,
        |t| (1.0 - t.powf(SHOULDER)).sqrt(),
        |_| 0.0,
        |t, _, _| (0.0, mix(shoulder, config.tint, smooth(t / ROOT_SHOULDER))),
    )
}

/// A berry hanging from the origin down -Z: a cylinder rounded at both
/// ends, its diameter toward its far end and narrowing to its stalk, its
/// skin striped with its shoulder colour and that colour round its stalk.
fn berry(config: &FruitConfig, detail: f32) -> MeshData {
    let pale = config.shoulder.unwrap_or(config.tint);
    turned(
        config,
        detail,
        RINGS,
        |t| {
            let ends = (1.0 - t.powf(BERRY_END)).sqrt() * (1.0 - (1.0 - t).powf(BERRY_END)).sqrt();
            ends * (1.0 - BERRY_NECK * (1.0 - t))
        },
        |_| 0.0,
        |t, angle, _| {
            let stripe = STRIPE * (0.5 + 0.5 * (STRIPES * angle).cos());
            let shoulder = smooth((BERRY_SHOULDER - t) / BERRY_SHOULDER);
            (0.0, mix(mix(config.tint, pale, stripe), pale, shoulder))
        },
    )
}

/// A strawberry hanging from the origin down -Z: a shoulder rounding out
/// from its stalk, widest a third of the way down and narrowing to a point
/// at its tip, under a calyx. It colours from the tip: at a ripeness of 0
/// it is its shoulder colour throughout, at 1 its tint, and between them
/// the front sits that far from the tip, blending over [`FRONT_BAND`] of
/// the length. Its achenes pit every fourth point and turn it their colour.
fn strawberry(config: &FruitConfig, detail: f32) -> anyhow::Result<MeshData> {
    let shoulder = config.shoulder.unwrap_or(config.tint);
    let front = (1.0 - config.ripeness.clamp(0.0, 1.0)) * (1.0 + FRONT_BAND) - FRONT_BAND;
    let body = turned(
        config,
        detail,
        RINGS,
        |t| {
            let left = 1.0 - t;
            (1.0 - left.powf(STRAWBERRY_SHOULDER)).sqrt() * left.powf(STRAWBERRY_TAPER)
        },
        |_| 0.0,
        |t, _, pitted| {
            let color = mix(shoulder, config.tint, smooth((t - front) / FRONT_BAND));
            if pitted && (ACHENES.0..ACHENES.1).contains(&t) {
                (-ACHENE_DEPTH, mix(color, srgb(ACHENE), ACHENE_PALE))
            } else {
                (0.0, color)
            }
        },
    );
    let mut parts = vec![body];
    parts.extend(calyx(config, detail)?);
    Ok(merge_meshes(&parts))
}

/// A strawberry's calyx: [`SEPALS`] sepals spread evenly round the stalk,
/// each a flat leaf outline lying out from the stalk over the shoulder,
/// tilted down off level and drooping further toward its tip, a hair off
/// the skin, painted its green.
fn calyx(config: &FruitConfig, detail: f32) -> anyhow::Result<Vec<MeshData>> {
    let length = SEPAL * config.diameter.max(1e-3);
    let width = SEPAL_WIDTH * length;
    let shape = leaf(length as f64, width as f64, SEPAL_PEAK);
    let (rings, _) = counts(detail, RINGS);
    let mut sheet = outline_mesh(&shape, shape.area() / rings as f64)?;
    fold(&mut sheet, 0.0, (SEPAL_DROOP / length) as f64);
    let sheet = sheet.painted(srgb(CALYX));
    Ok((0..SEPALS)
        .map(|k| {
            let round = Quat::from_rotation_z(TAU * k as f32 / SEPALS as f32);
            let placed = Transform::from_xyz(0.0, 0.0, SEPAL_GAP)
                .with_rotation(round * Quat::from_rotation_y(SEPAL_TILT));
            sheet.transformed(&placed)
        })
        .collect())
}

/// A pod hanging from the origin down -Z: a slender solid rounded at both
/// ends, its diameter across its upper half, thinning from its middle to
/// its tip, swelling a little at each bean along it and swinging a little
/// toward +X on its way down.
fn pod(config: &FruitConfig, detail: f32) -> MeshData {
    let length = config.length.max(1e-3);
    turned(
        config,
        detail,
        POD_RINGS,
        |t| {
            let ends = (1.0 - t.powf(POD_END)).sqrt() * (1.0 - (1.0 - t).powf(POD_END)).sqrt();
            let beans = 1.0 + BEAN_SWELL * (0.5 - 0.5 * (TAU * BEANS * t).cos());
            ends * (1.0 - POD_TAPER * (2.0 * t - 1.0).max(0.0)) * beans
        },
        |t| POD_CURVE * length * t * t,
        |_, _, _| (0.0, config.tint),
    )
}

#[cfg(test)]
mod tests {
    use misina_lab::testing::bounds;

    use super::*;

    /// A root hangs from the origin to its length, is as wide as its
    /// diameter near the shoulder, and is its shoulder colour there over
    /// the skin's below.
    #[test]
    fn a_root_hangs_from_its_shoulder_to_its_tip() {
        let config = FruitConfig::root(0.03, 0.18)
            .tint([0.8, 0.3, 0.05])
            .shoulder([0.3, 0.4, 0.1]);
        let mesh = root(&config, 1.0);
        assert_eq!(bounds(&mesh, 2), (-0.18, 0.0));
        let (x0, x1) = bounds(&mesh, 0);
        assert!(
            x1 > 0.0145 && x1 <= 0.015 && (x0 + x1).abs() < 1e-6,
            "{x0}..{x1}"
        );
        let (rings, sectors) = counts(1.0, RINGS);
        assert_eq!(
            mesh.face_vertex_counts.len(),
            2 * sectors + (rings - 1) * sectors
        );
        let colors = mesh.colors.as_ref().unwrap();
        assert_eq!(colors.len(), mesh.points.len());
        assert_eq!(colors[0], [0.3, 0.4, 0.1], "the top is the shoulder's");
        assert_eq!(
            colors[colors.len() - 1],
            [0.8, 0.3, 0.05],
            "the tip is the skin's"
        );
    }

    /// A berry hangs its length, is as wide as its diameter below its middle
    /// and well under it at its stalk.
    #[test]
    fn a_berry_is_widest_below_its_middle_and_narrow_at_its_stalk() {
        let mesh = berry(&FruitConfig::berry(0.05, 0.2), 1.0);
        assert_eq!(bounds(&mesh, 2), (-0.2, 0.0));
        let (y0, y1) = bounds(&mesh, 1);
        assert!(
            y1 > 0.0249 && y1 <= 0.025 && (y0 + y1).abs() < 1e-6,
            "{y0}..{y1}"
        );
        let widest = mesh.points.iter().find(|p| p[1] > 0.0249).unwrap()[2];
        assert!(widest < -0.1, "widest at {widest}");
        let (rings, _) = counts(1.0, RINGS);
        let stalk_end = -0.2 * 0.5 / rings as f32;
        let neck = mesh
            .points
            .iter()
            .filter(|p| (p[2] - stalk_end).abs() < 1e-5)
            .map(|p| p[1].abs())
            .fold(0.0, f32::max);
        assert!(neck > 0.005 && neck < 0.6 * 0.025, "{neck}");
    }

    /// A strawberry hangs its length under its calyx, which sits a hair up
    /// the stalk, is as wide as its diameter in its upper half but for its
    /// achenes' pits, and narrows to a point: its last ring well under half
    /// its widest.
    #[test]
    fn a_strawberry_is_widest_in_its_upper_half_and_pointed() {
        let mesh = strawberry(&FruitConfig::strawberry(0.025, 0.03), 1.0).unwrap();
        let (z0, z1) = bounds(&mesh, 2);
        assert!(z0 == -0.03 && z1 <= SEPAL_GAP + 1e-6, "{z0}..{z1}");
        let (y0, y1) = bounds(&mesh, 1);
        assert!(
            y1 > 0.0125 * (1.0 - ACHENE_DEPTH) - 1e-6
                && y1 <= 0.0125 + 1e-6
                && (y0 + y1).abs() < 1e-6,
            "{y0}..{y1}"
        );
        let widest = mesh
            .points
            .iter()
            .find(|p| p[1] > 0.0125 * (1.0 - ACHENE_DEPTH) - 1e-6)
            .unwrap()[2];
        assert!(widest > -0.015 && widest < -0.005, "widest at {widest}");
        let (rings, _) = counts(1.0, RINGS);
        let tip_ring = -0.03 * (rings as f32 - 0.5) / rings as f32;
        let tip = mesh
            .points
            .iter()
            .filter(|p| (p[2] - tip_ring).abs() < 1e-6)
            .map(|p| p[1].abs())
            .fold(0.0, f32::max);
        assert!(tip > 0.0 && tip < 0.4 * 0.0125, "{tip}");
    }

    /// A strawberry colours from its tip: unripe it is its shoulder colour
    /// throughout, ripe its tint, half ripe its tint at the tip and its
    /// shoulder colour at the stalk; its achenes pale every other point of
    /// its body; and its calyx lies over its shoulder, clear of the skin.
    #[test]
    fn a_strawberry_colours_from_its_tip_under_its_calyx() {
        let (pale, red) = ([0.8, 0.85, 0.6], [0.55, 0.02, 0.03]);
        let berry = |ripeness: f32| {
            FruitConfig::strawberry(0.025, 0.03)
                .tint(red)
                .shoulder(pale)
                .ripeness(ripeness)
        };
        let body = |ripeness: f32| {
            let mesh = strawberry(&berry(ripeness), 1.0).unwrap();
            let (rings, sectors) = counts(1.0, RINGS);
            let colors = mesh.colors.clone().unwrap();
            // The turned body's points come first, apex to apex.
            (mesh, colors[..2 + rings * sectors].to_vec())
        };
        let close = |a: &[f32; 3], b: [f32; 3]| (0..3).all(|i| (a[i] - b[i]).abs() < 1e-5);
        let achene = |c: &[f32; 3], base: [f32; 3]| {
            !close(c, base) && close(c, mix(base, srgb(ACHENE), ACHENE_PALE))
        };
        let (_, unripe) = body(0.0);
        assert!(unripe.iter().all(|c| close(c, pale) || achene(c, pale)));
        assert!(
            unripe.iter().filter(|c| achene(c, pale)).count() > 10,
            "no achenes"
        );
        let (mesh, ripe) = body(1.0);
        assert!(ripe.iter().all(|c| close(c, red) || achene(c, red)));
        let (_, half) = body(0.5);
        assert!(close(&half[0], pale), "the stalk end is still pale");
        assert!(close(&half[half.len() - 1], red), "the tip has coloured");

        let (rings, sectors) = counts(1.0, RINGS);
        let sepals = &mesh.points[2 + rings * sectors..];
        assert!(!sepals.is_empty(), "no calyx");
        let reach = sepals.iter().map(|p| p[0].hypot(p[1])).fold(0.0, f32::max);
        assert!(
            reach > 0.4 * 0.025 && reach <= SEPAL * 0.025 + 1e-6,
            "{reach}"
        );
        assert!(
            sepals
                .iter()
                .all(|p| p[2] > -0.4 * 0.03 && p[2] <= SEPAL_GAP + 1e-6),
            "the sepals lie over the shoulder"
        );
        let green = srgb(CALYX);
        assert!(
            mesh.colors.unwrap()[2 + rings * sectors..]
                .iter()
                .all(|c| *c == green)
        );
    }

    /// A pod hangs its length, is as wide as its diameter about its middle,
    /// and swings its tip aside by its curve, its last ring's radius past
    /// that; on a stalk it hangs that much lower, under a thin tube from
    /// the anchor.
    #[test]
    fn a_pod_hangs_its_length_and_swings_aside() {
        let config = FruitConfig::pod(0.012, 0.15);
        let mesh = pod(&config, 1.0);
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

        let stalked = super::mesh(&config.stalk(0.015), 1.0).unwrap();
        let (z0, z1) = bounds(&stalked, 2);
        assert!((z0 + 0.165).abs() < 1e-6 && z1 == 0.0, "{z0}..{z1}");
        let stalk = stalked
            .points
            .iter()
            .filter(|p| p[2] > -0.015 + 1e-6)
            .map(|p| p[0].hypot(p[1]))
            .fold(0.0, f32::max);
        assert!(
            stalk > 0.0 && stalk <= config.stalk_radius() + 1e-6,
            "the stalk is {stalk} m thick"
        );
    }
}

//! Solids — fruit and roots: a profile turned about an axis and painted, a
//! colour per point, every one hanging from the origin down -Z, below its
//! stalk if it has one, with what grows on it: a strawberry's achenes and
//! calyx, a pod's calyx and beak, a berry's blossom scar, a root's tail. How
//! finely a solid is turned is the scene's `detail`.

use std::f32::consts::{PI, TAU};

use bevy::prelude::*;
use misina_lab::geometry::mesh::{MeshData, cylinder_mesh, lathe, merge_meshes};
use misina_lab::geometry::outline::outline_mesh;
use misina_lab::geometry::shapes::{fold, leaf};
use misina_lab::palette::{mix, srgb};

use crate::elements::fruit::{FruitConfig, Kind};
use crate::grow::builder::smooth;
use crate::grow::phyllotaxis::GOLDEN;

/// Points round a ring and rings along a solid at a detail of 1, both
/// scaling with its square root, and the fewest of either.
const SECTORS: f32 = 16.0;
const RINGS: f32 = 12.0;
const MIN_SECTORS: usize = 8;
const MIN_RINGS: usize = 6;

/// A root: its rings at a detail of 1, how blunt its shoulder is (the
/// exponent its profile falls off with), and how far down it its shoulder
/// colour reaches, as a fraction; the creases round it along its length and
/// how far one draws it in where it is deepest, as a fraction of its
/// radius; and its tail: how long, as a fraction of the root's length, how
/// thick, as a fraction of its radius, and how far it curls, in radians.
const ROOT_RINGS: f32 = 30.0;
const SHOULDER: f32 = 1.6;
const ROOT_SHOULDER: f32 = 0.15;
const CREASES: f32 = 9.0;
const CREASE: f32 = 0.09;
const TAIL: f32 = 0.15;
const TAIL_RADIUS: f32 = 0.08;
const TAIL_CURL: f32 = 0.5;

/// A pod: its rings at a detail of 1, how square its stalk end is and how
/// pointed its tip (the exponents its profile falls to them with), how much
/// thinner it gets from its middle to its tip, how far its tip swings aside
/// as a fraction of its length, the beans along it and how far each swells
/// it, as a fraction of its radius, and how much thinner it is across its
/// faces than across its edges, which lie in the plane it swings in.
const POD_RINGS: f32 = 14.0;
const POD_END: f32 = 6.0;
const POD_TIP: f32 = 3.0;
const POD_TAPER: f32 = 0.35;
const POD_CURVE: f32 = 0.08;
const BEANS: f32 = 5.0;
const BEAN_SWELL: f32 = 0.08;
const POD_FLAT: f32 = 0.25;

/// A pod's beak, what is left of its style: how long, as a fraction of the
/// pod's length, how thick where it leaves the tip, as a fraction of the
/// pod's radius, and how far it curls on past the pod's own swing, in
/// radians.
const BEAK: f32 = 0.07;
const BEAK_RADIUS: f32 = 0.15;
const BEAK_CURL: f32 = 0.6;

/// The calyx cupping a pod's top: how far up the stalk and down the pod it
/// reaches, as fractions of the pod's radius, how far proud of each, as a
/// fraction, and its sides.
const CUP_UP: f32 = 0.3;
const CUP_DOWN: f32 = 0.5;
const CUP_PROUD: f32 = 0.15;
const CUP_SIDES: usize = 6;

/// A berry: its rings at a detail of 1, its ribs and the points round it
/// to each, and how far they stand out, as a fraction of its radius; how
/// its ends round off, the exponents its profile falls to its stalk end and
/// to its blossom end with, and how much narrower it is at its stalk end
/// than at its blossom end; how far its ribs pale the skin toward its
/// shoulder colour, and how far down from the stalk the shoulder colour
/// reaches.
const BERRY_RINGS: f32 = 16.0;
const RIBS: usize = 5;
const RIB_SECTORS: usize = 4;
const RIB: f32 = 0.035;
const NECK: f32 = 5.0;
const BLOSSOM: f32 = 12.0;
const CLUB: f32 = 0.15;
const STRIPE: f32 = 0.25;
const BERRY_SHOULDER: f32 = 0.12;

/// A berry's blossom scar: how wide, as a fraction of the berry's radius,
/// how far it stands proud of the blossom end and sinks into it, as
/// fractions of its own radius, how much narrower it is at its face, and
/// its sides.
const SCAR: f32 = 0.22;
const SCAR_PROUD: f32 = 0.15;
const SCAR_SUNK: f32 = 0.5;
const SCAR_FACE: f32 = 0.85;
const SCAR_SIDES: usize = 8;

/// A berry's stalk: how far it flares where it meets the berry, as a
/// fraction of its radius, its rings, and how deep the grooves between its
/// ridges run, as a fraction of its radius; it has a ridge to each of the
/// berry's ribs, in line with it.
const FLARE: f32 = 0.5;
const PEDUNCLE_RINGS: usize = 6;
const GROOVE: f32 = 0.3;

/// A strawberry: how round its shoulder is, the exponent its profile rises
/// from the stalk with; how fast it narrows to its tip, the power the way
/// left is raised to; and over how much of its length its colour front
/// blends.
const STRAWBERRY_SHOULDER: f32 = 3.0;
const STRAWBERRY_TAPER: f32 = 0.6;
const FRONT_BAND: f32 = 0.3;

/// A strawberry's achenes: how many at a detail of 1, scaling with its
/// square root, and the fewest; between where along it they sit; each
/// one's length as a fraction of the spacing between them, its width as a
/// fraction of its length, how far it stands proud of the skin and sinks
/// into it, as fractions of its length; how far its pit darkens the skin
/// round it toward black; and the steps the skin's area is summed over to
/// spread them.
const ACHENES: f32 = 120.0;
const MIN_ACHENES: usize = 24;
const ACHENE_BAND: (f32, f32) = (0.1, 0.95);
const ACHENE: f32 = 0.35;
const ACHENE_WIDTH: f32 = 0.7;
const ACHENE_PROUD: f32 = 0.3;
const ACHENE_SUNK: f32 = 0.3;
const PIT: f32 = 0.45;
const AREA_STEPS: usize = 64;

/// A strawberry's calyx: its sepals, each so long as a fraction of the
/// berry's diameter and so wide as a fraction of its length, widest so far
/// along, tilted so far down off level where it leaves the stalk and
/// drooping so far further by its tip, in radians, and at least this far
/// off the skin, in meters.
const SEPALS: usize = 7;
const SEPAL: f32 = 0.5;
const SEPAL_WIDTH: f32 = 0.35;
const SEPAL_PEAK: f64 = 0.35;
const SEPAL_TILT: f32 = 0.15;
const SEPAL_DROOP: f32 = 0.45;
const SEPAL_GAP: f32 = 0.0008;

/// A whip, a pod's beak or a root's tail: its sides and its rings.
const WHIP_SIDES: usize = 4;
const WHIP_RINGS: usize = 3;

/// The colours no config gives: a strawberry's achenes, its calyx and a
/// pod's, a stalk, and a berry's blossom scar.
const ACHENE_COLOR: u32 = 0xD8C060;
const CALYX: u32 = 0x4A7A32;
const STALK: u32 = 0x6E9A48;
const SCAR_COLOR: u32 = 0x9C9058;

/// A plain stalk's sides, and how much thinner it is at the body than at
/// the anchor.
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

/// `body` hung below the config's stalk, a tube from the anchor down to it:
/// a berry's ridged and flaring into it, any other's plain and thinner at
/// the body; as it is without one.
fn stalked(config: &FruitConfig, body: MeshData) -> MeshData {
    if config.stalk <= 0.0 {
        return body;
    }
    let down = Transform::from_xyz(0.0, 0.0, -config.stalk);
    let radius = config.stalk_radius();
    let stalk = match config.kind {
        Kind::Berry => peduncle(config.stalk, radius),
        _ => cylinder_mesh(radius * STALK_TAPER, radius, config.stalk, STALK_SIDES)
            .transformed(&down),
    };
    merge_meshes(&[stalk.painted(srgb(STALK)), body.transformed(&down)])
}

/// Rings along a solid and points round each at `detail`, from those it
/// has at a detail of 1.
fn counts(detail: f32, rings: f32, sectors: f32) -> (usize, usize) {
    let scale = detail.max(0.01).sqrt();
    (
        ((rings * scale).round() as usize).max(MIN_RINGS),
        ((sectors * scale).round() as usize).max(MIN_SECTORS),
    )
}

/// The radius of a solid turned with `rings` along it, `t` of the way down:
/// `profile` scaled so the widest ring is the config's radius out.
fn body(config: &FruitConfig, rings: usize, profile: impl Fn(f32) -> f32) -> impl Fn(f32) -> f32 {
    let widest = (0..rings)
        .map(|j| profile((j as f32 + 0.5) / rings as f32))
        .fold(0.0, f32::max)
        .max(1e-6);
    let radius = config.diameter.max(1e-3) / 2.0;
    move |t| radius * profile(t) / widest
}

/// A solid `length` long turned about -Z from the origin down, `rings`
/// rings of `sectors` points, each `radius(t)` out at `t` of the way down
/// and `aside(t)` off the axis along +X. `skin(t, angle)` is how far a
/// point is pushed in or out of that, as a fraction, and its colour. The
/// apexes take the skin at either end.
fn turned(
    length: f32,
    (rings, sectors): (usize, usize),
    radius: impl Fn(f32) -> f32,
    aside: impl Fn(f32) -> f32,
    skin: impl Fn(f32, f32) -> (f32, [f32; 3]),
) -> MeshData {
    let length = length.max(1e-3);
    let mut colors = vec![skin(0.0, 0.0).1];
    let mut turned = Vec::with_capacity(rings);
    for j in 0..rings {
        let t = (j as f32 + 0.5) / rings as f32;
        let mut ring = Vec::with_capacity(sectors);
        for i in 0..sectors {
            let angle = TAU * i as f32 / sectors as f32;
            let (push, color) = skin(t, angle);
            colors.push(color);
            let r = radius(t) * (1.0 + push);
            ring.push(Vec3::new(
                aside(t) + r * angle.cos(),
                r * angle.sin(),
                -length * t,
            ));
        }
        turned.push(ring);
    }
    colors.push(skin(1.0, 0.0).1);
    let mut mesh = lathe(Vec3::ZERO, &turned, Vec3::new(aside(1.0), 0.0, -length));
    mesh.colors = Some(colors);
    mesh
}

/// A thin cone from `from` to a point `length` on, heading down along
/// `heading` and curling `curl` radians on toward +X over its length,
/// `radius` thick at its first ring, which is sunk `sink` back into what it
/// grows from.
fn whip(from: Vec3, heading: Vec3, length: f32, radius: f32, curl: f32, sink: f32) -> MeshData {
    let step = (length + sink) / WHIP_RINGS as f32;
    // Which way it heads `k` steps along: turned toward +X about Y.
    let way = |k: usize| Quat::from_rotation_y(-curl * k as f32 / WHIP_RINGS as f32) * heading;
    let mut at = from - heading * sink;
    let foot = at - heading * step;
    let mut rings = Vec::with_capacity(WHIP_RINGS);
    for k in 0..WHIP_RINGS {
        // Square to the way it heads, anticlockwise seen from above.
        let across = way(k).cross(Vec3::Y).normalize();
        let r = radius * (1.0 - k as f32 / WHIP_RINGS as f32);
        rings.push(
            (0..WHIP_SIDES)
                .map(|i| {
                    let angle = TAU * i as f32 / WHIP_SIDES as f32;
                    at + r * (angle.cos() * across + angle.sin() * Vec3::Y)
                })
                .collect::<Vec<_>>(),
        );
        at += way(k + 1) * step;
    }
    lathe(foot, &rings, at)
}

/// How far back into the end of a solid `length` long turned with `rings`
/// rings, the last `last` out, a whip `radius` thick sinks its first ring
/// to come out of the cone closing that end without a step.
fn sink(radius: f32, last: f32, rings: usize, length: f32) -> f32 {
    radius * length * 0.5 / rings as f32 / last.max(1e-6)
}

/// A taproot hanging from the origin down -Z: as wide as its shoulder near
/// the top, tapering to a point at its length and on into a thin tail,
/// creased round its length, its shoulder the shoulder colour over the
/// skin's.
fn root(config: &FruitConfig, detail: f32) -> MeshData {
    let shoulder = config.shoulder.unwrap_or(config.tint);
    let grid = counts(detail, ROOT_RINGS, SECTORS);
    let radius = body(config, grid.0, |t| (1.0 - t.powf(SHOULDER)).sqrt());
    let skin = turned(
        config.length,
        grid,
        &radius,
        |_| 0.0,
        |t, angle| {
            let color = mix(shoulder, config.tint, smooth(t / ROOT_SHOULDER));
            (-crease(t, angle), color)
        },
    );
    let (rings, length) = (grid.0, config.length.max(1e-3));
    let thick = TAIL_RADIUS * config.diameter / 2.0;
    let tail = whip(
        Vec3::new(0.0, 0.0, -length),
        Vec3::NEG_Z,
        TAIL * length,
        thick,
        TAIL_CURL,
        sink(thick, radius(1.0 - 0.5 / rings as f32), rings, length),
    );
    merge_meshes(&[skin, tail.painted(config.tint)])
}

/// How far a root's skin is drawn in `t` of the way down and `angle` round,
/// as a fraction of its radius: [`CREASES`] creases along it, each deepest
/// a golden angle round from the last, so they read as wrinkles rather than
/// rings.
fn crease(t: f32, angle: f32) -> f32 {
    let along = (TAU * CREASES * t).cos().max(0.0);
    let round = 0.5 + 0.5 * (angle + GOLDEN * (CREASES * t).round()).cos();
    CREASE * along * along * round
}

/// A berry hanging from the origin down -Z: a cylinder rounded at both
/// ends, faintly ribbed along its length, its diameter toward its blossom
/// end and narrowing toward its stalk end, a scar on its blossom end; its
/// skin paling along its ribs toward its shoulder colour, and to that
/// colour round its stalk.
fn berry(config: &FruitConfig, detail: f32) -> MeshData {
    let pale = config.shoulder.unwrap_or(config.tint);
    let grid = counts(detail, BERRY_RINGS, (RIBS * RIB_SECTORS) as f32);
    let radius = body(config, grid.0, |t| {
        let ends = (1.0 - (1.0 - t).powf(NECK)).sqrt() * (1.0 - t.powf(BLOSSOM)).sqrt();
        ends * (1.0 - CLUB * (1.0 - t))
    });
    let skin = turned(
        config.length,
        grid,
        &radius,
        |_| 0.0,
        |t, angle| {
            let rib = (RIBS as f32 * angle).cos();
            let shoulder = smooth((BERRY_SHOULDER - t) / BERRY_SHOULDER);
            let color = mix(
                mix(config.tint, pale, STRIPE * rib.max(0.0)),
                pale,
                shoulder,
            );
            (RIB * rib, color)
        },
    );
    // A low button on the blossom end, sunk into it and narrower at its face.
    let scar = SCAR * config.diameter / 2.0;
    let button = cylinder_mesh(
        scar,
        scar * SCAR_FACE,
        scar * (SCAR_PROUD + SCAR_SUNK),
        SCAR_SIDES,
    )
    .transformed(
        &Transform::from_xyz(0.0, 0.0, scar * SCAR_SUNK - config.length.max(1e-3))
            .with_rotation(Quat::from_rotation_x(PI)),
    );
    merge_meshes(&[skin, button.painted(srgb(SCAR_COLOR))])
}

/// A berry's stalk `length` long and `radius` thick from the origin down
/// -Z: a ridge to each of the berry's ribs, grooved between them, flaring
/// where it meets the berry.
fn peduncle(length: f32, radius: f32) -> MeshData {
    let sectors = 3 * RIBS;
    let rings: Vec<Vec<Vec3>> = (0..=PEDUNCLE_RINGS)
        .map(|j| {
            let s = j as f32 / PEDUNCLE_RINGS as f32;
            let out = radius * (1.0 + FLARE * s * s * s);
            (0..sectors)
                .map(|i| {
                    let angle = TAU * i as f32 / sectors as f32;
                    let r = if i % 3 == 0 {
                        out
                    } else {
                        out * (1.0 - GROOVE)
                    };
                    Vec3::new(r * angle.cos(), r * angle.sin(), -length * s)
                })
                .collect()
        })
        .collect();
    lathe(Vec3::ZERO, &rings, Vec3::new(0.0, 0.0, -length))
}

/// A strawberry's radius `t` of the way down, as a fraction of its widest:
/// a shoulder rounding out from its stalk, widest a third of the way down
/// and narrowing to a point at its tip.
fn strawberry_profile(t: f32) -> f32 {
    let left = 1.0 - t;
    (1.0 - left.powf(STRAWBERRY_SHOULDER)).sqrt() * left.powf(STRAWBERRY_TAPER)
}

/// A strawberry hanging from the origin down -Z, shaped by
/// [`strawberry_profile`], under a calyx and set with achenes. It colours
/// from the tip: at a ripeness of 0 it is its shoulder colour throughout,
/// at 1 its tint, and between them the front sits that far from the tip,
/// blending over [`FRONT_BAND`] of the length.
fn strawberry(config: &FruitConfig, detail: f32) -> anyhow::Result<MeshData> {
    let shoulder = config.shoulder.unwrap_or(config.tint);
    let front = (1.0 - config.ripeness.clamp(0.0, 1.0)) * (1.0 + FRONT_BAND) - FRONT_BAND;
    let color = |t: f32| mix(shoulder, config.tint, smooth((t - front) / FRONT_BAND));
    let grid = counts(detail, RINGS, SECTORS);
    let radius = body(config, grid.0, strawberry_profile);
    let skin = turned(
        config.length,
        grid,
        &radius,
        |_| 0.0,
        |t, _| (0.0, color(t)),
    );
    let mut parts = vec![skin, achenes(config, detail, &radius, &color)];
    parts.extend(calyx(config, grid.0, &radius)?);
    Ok(merge_meshes(&parts))
}

/// A strawberry's achenes, [`ACHENES`] at a detail of 1, spread evenly over
/// its skin between [`ACHENE_BAND`] of the way down, each a golden angle
/// round from the last: a three-sided pyramid set into the skin `radius(t)`
/// out and pointing toward the tip, its foot the skin's `color(t)` darkened
/// by its pit and its peak the achene's.
fn achenes(
    config: &FruitConfig,
    detail: f32,
    radius: &impl Fn(f32) -> f32,
    color: &impl Fn(f32) -> [f32; 3],
) -> MeshData {
    let count = ((ACHENES * detail.max(0.01).sqrt()).round() as usize).max(MIN_ACHENES);
    let length = config.length.max(1e-3);
    // The skin's area from the top of the band down, summed over narrow
    // frusta, which a seed's share of the whole finds its depth in.
    let (from, to) = ACHENE_BAND;
    let depth = |k: usize| from + (to - from) * k as f32 / AREA_STEPS as f32;
    let mut area = vec![0.0];
    for k in 0..AREA_STEPS {
        let (a, b) = (radius(depth(k)), radius(depth(k + 1)));
        let slant = (b - a).hypot(length * (depth(k + 1) - depth(k)));
        area.push(area[k] + PI * (a + b) * slant);
    }
    let total = area[AREA_STEPS];
    let size = ACHENE * (total / count as f32).sqrt();
    let peak = srgb(ACHENE_COLOR);
    let (mut points, mut colors, mut indices) = (Vec::new(), Vec::new(), Vec::new());
    for k in 0..count {
        let share = total * (k as f32 + 0.5) / count as f32;
        let i = area.partition_point(|a| *a < share).clamp(1, AREA_STEPS);
        let f = (share - area[i - 1]) / (area[i] - area[i - 1]).max(1e-12);
        let t = depth(i - 1) + f * (depth(i) - depth(i - 1));
        // The frame of the skin there: out from the axis, round it, down
        // the skin toward the tip and square off it.
        let angle = k as f32 * GOLDEN;
        let out = Vec3::new(angle.cos(), angle.sin(), 0.0);
        let round = Vec3::new(-angle.sin(), angle.cos(), 0.0);
        let slope = (radius(t + 1e-3) - radius(t - 1e-3)) / 2e-3;
        let down = (out * slope - Vec3::Z * length).normalize();
        let normal = (out * length + Vec3::Z * slope).normalize();
        let at = out * radius(t) - Vec3::Z * length * t;
        let foot = at - normal * ACHENE_SUNK * size;
        let side = round * ACHENE_WIDTH * size / 2.0;
        let first = points.len() as i32;
        points.extend(
            [
                at + normal * ACHENE_PROUD * size,
                foot + down * 0.6 * size,
                foot - down * 0.4 * size + side,
                foot - down * 0.4 * size - side,
            ]
            .map(|p| p.to_array()),
        );
        let pit = mix(color(t), [0.0; 3], PIT);
        colors.extend([peak, pit, pit, pit]);
        // Its three sides, wound outward round the peak.
        for (a, b) in [(1, 2), (2, 3), (3, 1)] {
            indices.extend([first + a, first + b, first]);
        }
    }
    MeshData {
        face_vertex_counts: vec![3; indices.len() / 3],
        face_vertex_indices: indices,
        points,
        colors: Some(colors),
    }
}

/// A strawberry's calyx: [`SEPALS`] sepals spread evenly round the stalk,
/// each a flat leaf outline lying out from the stalk over the shoulder,
/// tilted down off level and drooping further toward its tip, and pushed
/// out to [`SEPAL_GAP`] off the skin, `radius(t)` out `t` of the way down,
/// wherever the droop would take it nearer; filled at `rings` triangles,
/// painted its green.
fn calyx(
    config: &FruitConfig,
    rings: usize,
    radius: &impl Fn(f32) -> f32,
) -> anyhow::Result<Vec<MeshData>> {
    let length = SEPAL * config.diameter.max(1e-3);
    let width = SEPAL_WIDTH * length;
    let shape = leaf(length as f64, width as f64, SEPAL_PEAK);
    let mut sheet = outline_mesh(&shape, shape.area() / rings as f64)?;
    fold(&mut sheet, 0.0, (SEPAL_DROOP / length) as f64);
    let sheet = sheet.painted(srgb(CALYX));
    let deep = config.length.max(1e-3);
    Ok((0..SEPALS)
        .map(|k| {
            let round = Quat::from_rotation_z(TAU * k as f32 / SEPALS as f32);
            let placed = Transform::from_xyz(0.0, 0.0, SEPAL_GAP)
                .with_rotation(round * Quat::from_rotation_y(SEPAL_TILT));
            let mut sepal = sheet.transformed(&placed);
            for p in &mut sepal.points {
                let t = -p[2] / deep;
                let (out, clear) = (p[0].hypot(p[1]), radius(t) + SEPAL_GAP);
                if (0.0..1.0).contains(&t) && out > 1e-6 && out < clear {
                    (p[0], p[1]) = (p[0] * clear / out, p[1] * clear / out);
                }
            }
            sepal
        })
        .collect())
}

/// A pod hanging from the origin down -Z: a slender solid flattened across
/// its faces, its diameter across its edges at its middle, blunt at its
/// stalk end and narrowing to its tip, swelling a little at each bean along
/// it and swinging a little toward +X on its way down; a calyx cups its top
/// and its beak curls on from its tip.
fn pod(config: &FruitConfig, detail: f32) -> MeshData {
    let length = config.length.max(1e-3);
    let grid = counts(detail, POD_RINGS, SECTORS);
    let radius = body(config, grid.0, |t| {
        let ends = (1.0 - (1.0 - t).powf(POD_END)).sqrt() * (1.0 - t.powf(POD_TIP)).sqrt();
        let beans = 1.0 + BEAN_SWELL * (0.5 - 0.5 * (TAU * BEANS * t).cos());
        ends * (1.0 - POD_TAPER * (2.0 * t - 1.0).max(0.0)) * beans
    });
    let aside = |t: f32| POD_CURVE * length * t * t;
    let shell = turned(length, grid, &radius, aside, |_, angle| {
        (-POD_FLAT * angle.sin().powi(2), config.tint)
    });
    // The beak leaves the tip headed the way the pod swings there, out of
    // its narrower side.
    let (rings, thick) = (grid.0, BEAK_RADIUS * config.diameter / 2.0);
    let last = radius(1.0 - 0.5 / rings as f32) * (1.0 - POD_FLAT);
    let beak = whip(
        Vec3::new(aside(1.0), 0.0, -length),
        Vec3::new(2.0 * POD_CURVE, 0.0, -1.0).normalize(),
        BEAK * length,
        thick,
        BEAK_CURL,
        sink(thick, last, rings, length),
    );
    // The cup runs from round the stalk out over the pod's top, proud of
    // both.
    let (up, down) = (
        CUP_UP * config.diameter / 2.0,
        CUP_DOWN * config.diameter / 2.0,
    );
    let cup = cylinder_mesh(
        radius(down / length) * (1.0 + CUP_PROUD),
        config.stalk_radius() * (1.0 + CUP_PROUD),
        up + down,
        CUP_SIDES,
    )
    .transformed(&Transform::from_xyz(0.0, 0.0, -down));
    merge_meshes(&[shell, beak.painted(config.tint), cup.painted(srgb(CALYX))])
}

#[cfg(test)]
mod tests {
    use misina_lab::testing::bounds;

    use super::*;

    /// A root hangs from the origin to its length, then on into its thin
    /// tail, is as wide as its diameter near the shoulder but for its
    /// creases, which draw it in round its length, and is its shoulder
    /// colour there over the skin's.
    #[test]
    fn a_root_hangs_from_its_shoulder_to_its_tip() {
        let config = FruitConfig::root(0.03, 0.18)
            .tint([0.8, 0.3, 0.05])
            .shoulder([0.3, 0.4, 0.1]);
        let mesh = root(&config, 1.0);
        let (z0, z1) = bounds(&mesh, 2);
        assert!(z1 == 0.0 && z0 < -0.18 - 0.5 * TAIL * 0.18, "{z0}..{z1}");
        let (x0, x1) = bounds(&mesh, 0);
        assert!(x1 > 0.014 && x1 <= 0.015 && x0 >= -0.015, "{x0}..{x1}");
        let grid = counts(1.0, ROOT_RINGS, SECTORS);
        let (rings, sectors) = grid;
        // The turned skin's points come first, apex to apex.
        let skin = &mesh.points[..2 + rings * sectors];
        let radius = body(&config, rings, |t| (1.0 - t.powf(SHOULDER)).sqrt());
        let drawn_in = skin[1..skin.len() - 1]
            .iter()
            .filter(|p| p[0].hypot(p[1]) < radius(-p[2] / 0.18) * (1.0 - CREASE / 4.0))
            .count();
        assert!(drawn_in > rings, "{drawn_in} points in creases");
        let colors = mesh.colors.as_ref().unwrap();
        assert_eq!(colors.len(), mesh.points.len());
        assert_eq!(colors[0], [0.3, 0.4, 0.1], "the top is the shoulder's");
        assert_eq!(
            colors[skin.len() - 1],
            [0.8, 0.3, 0.05],
            "the tip is the skin's"
        );
    }

    /// A berry hangs its length and its scar past it, is as wide as its
    /// diameter toward its blossom end, ribs and all, and well under it at
    /// its stalk; on a stalk it hangs that much lower under a ridged tube
    /// flaring into it.
    #[test]
    fn a_berry_is_widest_toward_its_blossom_end_and_narrow_at_its_stalk() {
        let config = FruitConfig::berry(0.05, 0.2);
        let mesh = berry(&config, 1.0);
        let (z0, z1) = bounds(&mesh, 2);
        let proud = SCAR * 0.025 * SCAR_PROUD;
        assert!(z1 == 0.0 && (z0 + 0.2 + proud).abs() < 1e-6, "{z0}..{z1}");
        let widest = mesh
            .points
            .iter()
            .max_by(|a, b| a[0].hypot(a[1]).total_cmp(&b[0].hypot(b[1])))
            .unwrap();
        let out = widest[0].hypot(widest[1]);
        assert!(
            out > 0.025 && out <= 0.025 * (1.0 + RIB) + 1e-6,
            "{out} out"
        );
        assert!(widest[2] < -0.1, "widest at {}", widest[2]);
        let (rings, _) = counts(1.0, BERRY_RINGS, (RIBS * RIB_SECTORS) as f32);
        let stalk_end = -0.2 * 0.5 / rings as f32;
        let neck = mesh
            .points
            .iter()
            .filter(|p| (p[2] - stalk_end).abs() < 1e-5)
            .map(|p| p[0].hypot(p[1]))
            .fold(0.0, f32::max);
        assert!(neck > 0.005 && neck < 0.6 * 0.025, "{neck}");

        let stalked = super::mesh(&config.stalk(0.03), 1.0).unwrap();
        let (z0, z1) = bounds(&stalked, 2);
        assert!(z1 == 0.0 && (z0 + 0.23 + proud).abs() < 1e-6, "{z0}..{z1}");
        let radius = config.stalk_radius();
        let thickness = |z: f32| {
            stalked
                .points
                .iter()
                .filter(|p| (p[2] - z).abs() < 1e-6)
                .map(|p| p[0].hypot(p[1]))
                .fold(0.0, f32::max)
        };
        assert!((thickness(0.0) - radius).abs() < 1e-6);
        assert!((thickness(-0.03) - radius * (1.0 + FLARE)).abs() < 1e-6);
    }

    /// A strawberry hangs its length under its calyx, which sits a hair up
    /// the stalk, is as wide as its diameter in its upper half, and narrows
    /// to a point: its last ring well under half its widest.
    #[test]
    fn a_strawberry_is_widest_in_its_upper_half_and_pointed() {
        let mesh = strawberry(&FruitConfig::strawberry(0.025, 0.03), 1.0).unwrap();
        let (z0, z1) = bounds(&mesh, 2);
        assert!(z0 == -0.03 && z1 <= SEPAL_GAP + 1e-6, "{z0}..{z1}");
        let (rings, sectors) = counts(1.0, RINGS, SECTORS);
        let skin = &mesh.points[..2 + rings * sectors];
        let widest = skin.iter().max_by(|a, b| a[1].total_cmp(&b[1])).unwrap();
        assert!((widest[1] - 0.0125).abs() < 1e-6, "{widest:?}");
        assert!(
            widest[2] > -0.015 && widest[2] < -0.005,
            "widest at {widest:?}"
        );
        let tip_ring = -0.03 * (rings as f32 - 0.5) / rings as f32;
        let tip = skin
            .iter()
            .filter(|p| (p[2] - tip_ring).abs() < 1e-6)
            .map(|p| p[1].abs())
            .fold(0.0, f32::max);
        assert!(tip > 0.0 && tip < 0.4 * 0.0125, "{tip}");
    }

    /// A strawberry colours from its tip: unripe it is its shoulder colour
    /// throughout, ripe its tint, half ripe its tint at the tip and its
    /// shoulder colour at the stalk; its achenes stand on its skin, their
    /// peaks the achene's colour and their feet its skin's, darkened.
    #[test]
    fn a_strawberry_colours_from_its_tip_and_wears_its_achenes() {
        let (pale, red) = ([0.8, 0.85, 0.6], [0.55, 0.02, 0.03]);
        let berry = |ripeness: f32| {
            FruitConfig::strawberry(0.025, 0.03)
                .tint(red)
                .shoulder(pale)
                .ripeness(ripeness)
        };
        let (rings, sectors) = counts(1.0, RINGS, SECTORS);
        let skin = 2 + rings * sectors;
        let close = |a: &[f32; 3], b: [f32; 3]| (0..3).all(|i| (a[i] - b[i]).abs() < 1e-5);
        let colors = |ripeness: f32| strawberry(&berry(ripeness), 1.0).unwrap().colors.unwrap();
        assert!(colors(0.0)[..skin].iter().all(|c| close(c, pale)));
        assert!(colors(1.0)[..skin].iter().all(|c| close(c, red)));
        let half = colors(0.5);
        assert!(close(&half[0], pale), "the stalk end is still pale");
        assert!(close(&half[skin - 1], red), "the tip has coloured");

        let config = berry(1.0);
        let radius = body(&config, rings, strawberry_profile);
        let seeds = achenes(&config, 1.0, &radius, &|_| red);
        assert_eq!(seeds.points.len(), 4 * ACHENES as usize);
        let colors = seeds.colors.unwrap();
        for (seed, colors) in seeds.points.chunks(4).zip(colors.chunks(4)) {
            assert_eq!(colors[0], srgb(ACHENE_COLOR));
            assert!(
                colors[1..]
                    .iter()
                    .all(|c| close(c, mix(red, [0.0; 3], PIT)))
            );
            let off = |p: &[f32; 3]| p[0].hypot(p[1]) - radius(-p[2] / 0.03);
            assert!(off(&seed[0]) > 0.0, "a peak stands proud: {seed:?}");
            assert!(seed[1..].iter().all(|p| off(p) < 0.0), "the feet sink");
        }
    }

    /// Every point of a strawberry's sepals lies at least the gap off its
    /// skin at that depth, and they reach out over its shoulder.
    #[test]
    fn a_strawberrys_sepals_lie_clear_of_its_skin() {
        let config = FruitConfig::strawberry(0.024, 0.03);
        let (rings, _) = counts(1.0, RINGS, SECTORS);
        let radius = body(&config, rings, strawberry_profile);
        let sepals = calyx(&config, rings, &radius).unwrap();
        assert_eq!(sepals.len(), SEPALS);
        let points: Vec<&[f32; 3]> = sepals.iter().flat_map(|s| &s.points).collect();
        for p in &points {
            let t = -p[2] / 0.03;
            if (0.0..1.0).contains(&t) {
                let clear = p[0].hypot(p[1]) - radius(t);
                assert!(clear >= SEPAL_GAP - 1e-6, "{p:?} is {clear} off the skin");
            }
        }
        let reach = points.iter().map(|p| p[0].hypot(p[1])).fold(0.0, f32::max);
        assert!(reach > 0.4 * 0.024 && reach < 0.024, "{reach}");
        assert!(
            points
                .iter()
                .all(|p| p[2] > -0.4 * 0.03 && p[2] <= SEPAL_GAP + 1e-6)
        );
    }

    /// A pod hangs its length and its beak past it, is as wide as its
    /// diameter across its edges about its middle and narrower across its
    /// faces, and swings its tip aside by its curve; on a stalk it hangs that
    /// much lower, under a thin tube from the anchor and in a calyx wider
    /// than the stalk.
    #[test]
    fn a_pod_hangs_its_length_and_swings_aside() {
        let config = FruitConfig::pod(0.012, 0.15);
        let mesh = pod(&config, 1.0);
        let (z0, z1) = bounds(&mesh, 2);
        assert!(z0 < -0.15 - 0.5 * BEAK * 0.15 && z1 > 0.0, "{z0}..{z1}");
        let (rings, sectors) = counts(1.0, POD_RINGS, SECTORS);
        let shell = &mesh.points[..2 + rings * sectors];
        let (y0, y1) = bounds(
            &MeshData {
                points: shell.to_vec(),
                ..default()
            },
            1,
        );
        let edges = shell.iter().map(|p| p[0].abs()).fold(0.0, f32::max);
        assert!(
            y1 > 0.0055 * (1.0 - POD_FLAT) && y1 <= 0.006 * (1.0 - POD_FLAT) + 1e-6,
            "{y0}..{y1} across its faces"
        );
        let swing = POD_CURVE * 0.15;
        assert!(
            edges > 0.0055 && edges < swing + 0.006,
            "{edges} across its edges"
        );
        let (x0, x1) = bounds(&mesh, 0);
        assert!(x0 > -0.006 && x1 > swing, "{x0}..{x1}");

        let stalked = super::mesh(&config.stalk(0.015), 1.0).unwrap();
        let (_, z1) = bounds(&stalked, 2);
        assert_eq!(z1, 0.0);
        let stalk = stalked
            .points
            .iter()
            .filter(|p| p[2] > -0.01)
            .map(|p| p[0].hypot(p[1]))
            .fold(0.0, f32::max);
        assert!(
            stalk > 0.0 && stalk <= config.stalk_radius() + 1e-6,
            "the stalk is {stalk} m thick"
        );
        let (up, down) = (CUP_UP * 0.006, CUP_DOWN * 0.006);
        let cup = stalked
            .points
            .iter()
            .filter(|p| (-0.015 - down - 1e-6..-0.015 + up + 1e-6).contains(&p[2]))
            .map(|p| p[0].hypot(p[1]))
            .fold(0.0, f32::max);
        assert!(cup > 1.1 * config.stalk_radius(), "the cup is {cup} m wide");
    }
}

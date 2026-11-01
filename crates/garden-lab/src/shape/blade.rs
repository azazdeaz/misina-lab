//! Blades — a leaf in its own frame: the anchor at the origin, +X along the
//! petiole and midrib, the face up +Z.
//!
//! A simple leaf is an obovate outline filled flat and deformed: the margin
//! ruffled by waves that fade toward the midrib, the blade cupped about its
//! midrib and its tip drooped, on a stalk when the template has one. A
//! compound leaf is a rachis with toothed leaflets in pairs along it and one
//! at its tip, the whole drooped; a trifoliate leaf a petiole with a pair of
//! entire leaflets where it ends and one past them, the same. A lobed leaf
//! is a palmate ring, an ellipse carved by sinuses, its lobes sagging from
//! its centre and the whole hinged down where the petiole ends. A ruffle
//! displaces the margin out of the plane rather than lengthening it, which a
//! flat triangulation cannot do.

use std::f64::consts::{FRAC_PI_2, PI, TAU};

use bevy::prelude::*;
use misina_lab::geometry::mesh::{MeshData, bend, cylinder_mesh, merge_meshes};
use misina_lab::geometry::outline::{Outline, outline_mesh};
use misina_lab::geometry::shapes::{bent, leaf, toothed};
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

/// Triangles a simple blade is filled at, a compound leaf's leaflet, and a
/// trifoliate leaf's.
pub const TRIANGLES: f64 = 360.0;
const LEAFLET_TRIANGLES: f64 = 24.0;
const TRIFOLIATE_TRIANGLES: f64 = 80.0;

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

/// A trifoliate leaf's leaflets: the stalk from the pair to the terminal
/// one as a fraction of what the petiole leaves, how much shorter the pair
/// is than the terminal leaflet, where a leaflet is widest along its length,
/// how far the pair spreads off the midrib in radians, how far every
/// leaflet rises off the leaf's plane, and how it folds.
const TRIFOLIATE_RACHIS: f64 = 0.15;
const LATERAL: f64 = 0.9;
const LEAFLET_PEAK: f64 = 0.4;
const TRIFOLIATE_SPREAD: f32 = 1.1;
const TRIFOLIATE_RISE: f32 = 0.2;
const TRIFOLIATE_DROOP: f64 = 0.35;
const TRIFOLIATE_KEEL: f64 = 3.0;

/// A lobed blade: its lobes; how deep the sinuses between them cut, as a
/// fraction of the way from the margin to the blade's centre; how narrow
/// they are, the power the sinus profile is raised to; and how far a lobe's
/// tip sags below the centre, as a fraction of the blade's length.
const LOBES: usize = 5;
const SINUS: f64 = 0.45;
const SINUS_SHARPNESS: i32 = 2;
const LOBE_SAG: f64 = 0.08;

/// The share of its box a blade or a leaflet fills: [`profile`]'s hump.
const OUTLINE_FILL: f32 = 0.62;

/// The least half thickness of the box round a blade, in meters: a flat
/// blade still collides as a slab.
const BOX_SKIN: f32 = 0.002;

/// The blade area a config draws, in square meters: every blade or leaflet
/// as its box times [`OUTLINE_FILL`], the petiole and the stalks left out.
pub fn area(config: &LeafConfig) -> f32 {
    let whole = config.length.max(0.0);
    let blade = (whole - config.petiole.max(0.0)).max(0.0);
    let boxes = match config.template {
        // The ring itself: a lobed blade's fill depends on its sinuses.
        Template::Lobed => {
            return lobed_outline(blade as f64, (config.width * blade) as f64).area() as f32;
        }
        Template::Simple => blade * config.width * whole,
        Template::Compound => {
            let pairs = config.pairs.max(1);
            let leaflet =
                |i: u32| config.width * whole * (1.0 - TIP_SHRINK * i as f32 / pairs as f32);
            let along = (0..pairs)
                .map(|i| 2.0 * leaflet(i) * leaflet(i))
                .sum::<f32>();
            (along + leaflet(pairs) * leaflet(pairs)) * LEAFLET_WIDTH as f32
        }
        Template::Trifoliate => {
            let terminal = blade * (1.0 - TRIFOLIATE_RACHIS as f32);
            let lateral = LATERAL as f32 * terminal;
            (terminal * terminal + 2.0 * lateral * lateral) * config.width
        }
    };
    OUTLINE_FILL * boxes
}

/// The box round a blade `mesh` was built from `config`: the leaf less its
/// stalk, bounded in the frame it lies flattest in, turned down by a lobed
/// blade's whole hinge and by half of a curled blade's droop. Its placement
/// in the leaf's frame and its half extents, at least [`BOX_SKIN`] each.
pub fn collider(config: &LeafConfig, mesh: &MeshData) -> (Transform, Vec3) {
    let turn = match config.template {
        Template::Lobed => config.droop,
        _ => config.droop / 2.0,
    };
    let frame = Quat::from_rotation_y(turn);
    let back = frame.inverse();
    let stalked = match config.template {
        Template::Compound | Template::Trifoliate => true,
        Template::Simple | Template::Lobed => config.petiole > 0.0,
    };
    let skip = if stalked {
        stalk(1.0, 1.0).points.len()
    } else {
        0
    };
    let (lo, hi) = mesh.points[skip.min(mesh.points.len())..]
        .iter()
        .map(|p| back * Vec3::from(*p))
        .fold((Vec3::MAX, Vec3::MIN), |(lo, hi), p| (lo.min(p), hi.max(p)));
    if lo.cmpgt(hi).any() {
        return (Transform::IDENTITY, Vec3::splat(BOX_SKIN));
    }
    let half = ((hi - lo) / 2.0).max(Vec3::splat(BOX_SKIN));
    (
        Transform::from_translation(frame * ((hi + lo) / 2.0)).with_rotation(frame),
        half,
    )
}

/// A leaf from its config.
pub fn mesh(config: &LeafConfig) -> anyhow::Result<MeshData> {
    match config.template {
        Template::Simple => simple(config),
        Template::Compound => compound(config),
        Template::Trifoliate => trifoliate(config),
        Template::Lobed => lobed(config),
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

/// A palmate ring `length` long and `width` across: an ellipse with
/// [`LOBES`] lobes fanning from its basal notch, [`SINUS`]-deep sinuses
/// between them, centred so that the notch, where the petiole joins, sits
/// on the origin and the middle lobe's tip at `length`. Counter-clockwise
/// from the notch, as [`outline`] runs.
fn lobed_outline(length: f64, width: f64) -> Outline {
    let (a, centre) = lobed_axes(length);
    let count = 2 * STATIONS;
    let points = (0..count)
        .map(|i| {
            let theta = TAU * i as f64 / count as f64 - PI;
            let sinus = ((1.0 - (LOBES as f64 * theta).cos()) / 2.0).powi(SINUS_SHARPNESS);
            let (sin, cos) = theta.sin_cos();
            let out = 1.0 - SINUS * sinus;
            Point2::new(centre + a * cos * out, width / 2.0 * sin * out)
        })
        .collect();
    Outline { points }
}

/// A lobed blade's ellipse: its semi-axis along the blade and its centre's
/// station, placed so the tip, at `centre + a`, is at `length` and the
/// notch, at `centre - a (1 - SINUS)`, on the origin.
fn lobed_axes(length: f64) -> (f64, f64) {
    let a = length / (2.0 - SINUS);
    (a, length - a)
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
    Ok(drooped(merge_meshes(&parts), config.droop as f64 / whole))
}

/// A trifoliate leaf: a petiole, a pair of entire leaflets spread either
/// side where it ends, and one straight on past them on a short stalk, the
/// whole drooped.
fn trifoliate(config: &LeafConfig) -> anyhow::Result<MeshData> {
    let whole = config.length.max(0.005) as f64;
    let petiole = (config.petiole as f64).clamp(0.0, 0.8 * whole);
    let rachis = TRIFOLIATE_RACHIS * (whole - petiole);
    let terminal = whole - petiole - rachis;
    let cut = |length: f64| -> anyhow::Result<MeshData> {
        bent(
            leaf(length, config.width as f64 * length, LEAFLET_PEAK),
            TRIFOLIATE_DROOP,
            TRIFOLIATE_KEEL,
            TRIFOLIATE_TRIANGLES,
        )
    };
    let mut parts = vec![stalk(petiole + rachis, STALK_RADIUS * whole)];
    let lateral = cut(LATERAL * terminal)?;
    for side in [1.0f32, -1.0] {
        parts.push(placed(
            &lateral,
            Vec3::new(petiole as f32, 0.0, 0.0),
            Quat::from_rotation_z(side * TRIFOLIATE_SPREAD)
                * Quat::from_rotation_y(-TRIFOLIATE_RISE),
        ));
    }
    parts.push(placed(
        &cut(terminal)?,
        Vec3::new((petiole + rachis) as f32, 0.0, 0.0),
        Quat::from_rotation_y(-TRIFOLIATE_RISE),
    ));
    Ok(drooped(merge_meshes(&parts), config.droop as f64 / whole))
}

/// A lobed blade on its petiole: the ring filled flat, its lobes sagging
/// from the centre by the square of how far out they reach, the notch held
/// level with the petiole, and the whole turned down `droop` where the
/// petiole ends.
fn lobed(config: &LeafConfig) -> anyhow::Result<MeshData> {
    let whole = config.length.max(0.005) as f64;
    let petiole = (config.petiole as f64).clamp(0.0, 0.9 * whole);
    let length = whole - petiole;
    let width = (config.width as f64 * length).max(0.001);
    let ring = lobed_outline(length, width);
    let mut mesh = outline_mesh(&ring, ring.area() / TRIANGLES)?;
    let (a, centre) = lobed_axes(length);
    let reach = |x: f64, y: f64| ((x - centre) / a).hypot(y / (width / 2.0));
    let notch = reach(0.0, 0.0);
    let hinge = Quat::from_rotation_y(config.droop);
    for point in &mut mesh.points {
        let (x, y) = (point[0] as f64, point[1] as f64);
        let out = reach(x, y);
        let z = -LOBE_SAG * length * (out * out - notch * notch);
        let p = hinge * Vec3::new(x as f32, y as f32, z as f32);
        *point = [p.x + petiole as f32, p.y, p.z];
    }
    if petiole > 0.0 {
        mesh = merge_meshes(&[stalk(petiole, STALK_RADIUS * whole), mesh]);
    }
    Ok(mesh)
}

/// `mesh` bent over its length by the curvature `droop`, tip down.
fn drooped(mut mesh: MeshData, droop: f64) -> MeshData {
    if droop != 0.0 {
        for point in &mut mesh.points {
            let (x, z) = bend(point[0] as f64, point[2] as f64, droop);
            point[0] = x as f32;
            point[2] = z as f32;
        }
    }
    mesh
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
        let whole_box = 0.18 * 0.75 * 0.18;
        assert!((area(&flat) - OUTLINE_FILL * whole_box).abs() < 1e-6);
        assert!(area(&stalked) < area(&flat) && area(&stalked) > 0.5 * area(&flat));
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

    /// A lobed ring has its notch on the origin and its tip at its length,
    /// five lobes, and about two thirds of its ellipse's area; the leaf on
    /// its petiole turns down whole at the petiole's end, keeps inside its
    /// width, and weighs by its ring.
    #[test]
    fn a_lobed_blade_fans_five_lobes_from_its_notch_and_hinges_at_its_petiole() {
        let ring = lobed_outline(0.3, 0.33);
        let at = |x: f64, y: f64| {
            ring.points
                .iter()
                .any(|p| (p.x - x).abs() < 1e-9 && (p.y - y).abs() < 1e-9)
        };
        assert!(at(0.0, 0.0) && at(0.3, 0.0));
        let (a, centre) = lobed_axes(0.3);
        let radii: Vec<f64> = ring
            .points
            .iter()
            .map(|p| (p.x - centre).hypot(p.y))
            .collect();
        let n = radii.len();
        let lobes = (0..n)
            .filter(|&i| radii[i] > radii[(i + n - 1) % n] && radii[i] >= radii[(i + 1) % n])
            .count();
        assert_eq!(lobes, LOBES);
        let fill = ring.area() / (PI * a * 0.33 / 2.0);
        assert!((0.55..0.85).contains(&fill), "{fill}");

        let leaf = LeafConfig::lobed(0.65, 1.1).petiole(0.35).droop(0.7);
        let mesh = mesh(&leaf).unwrap();
        let (x0, x1) = bounds(&mesh, 0);
        let (y0, y1) = bounds(&mesh, 1);
        let (z0, _) = bounds(&mesh, 2);
        let (blade, hinge) = (0.3f32, 0.7f32);
        assert!(
            x0.abs() < 1e-6 && (x1 - (0.35 + blade * hinge.cos())).abs() < 0.02,
            "{x0}..{x1}"
        );
        let half = 1.1 * blade / 2.0;
        assert!(
            y0 < -0.9 * half && y1 <= half + 1e-6 && (y0 + y1).abs() < 1e-4,
            "{y0}..{y1}"
        );
        assert!(z0 < -0.9 * blade * hinge.sin(), "{z0}");
        assert!((area(&leaf) - lobed_outline(0.3, 0.33).area() as f32).abs() < 1e-6);
        assert!(mesh.face_vertex_counts.len() < 900);
    }

    /// Every template's box holds every point of its blade and leaves the
    /// stalk's foot out; a hinged lobed blade's is a slab turned with it.
    #[test]
    fn a_blade_box_holds_the_blade_and_not_the_stalk() {
        let leaves = [
            LeafConfig::lobed(0.65, 1.1).petiole(0.35).droop(0.7),
            LeafConfig::simple(0.35, 0.4)
                .petiole(0.14)
                .ruffle(0.8)
                .cup(0.3)
                .droop(0.9),
            LeafConfig::compound(0.3, 8, 0.14).petiole(0.09).droop(0.7),
            LeafConfig::trifoliate(0.18, 0.75).petiole(0.06).droop(0.4),
        ];
        for leaf in leaves {
            let mesh = mesh(&leaf).unwrap();
            let (placed, half) = collider(&leaf, &mesh);
            let inside = |p: Vec3| {
                let q = placed.rotation.inverse() * (p - placed.translation);
                (q.abs() - half).max_element() <= 1e-5
            };
            let stalk_points = stalk(1.0, 1.0).points.len();
            for point in &mesh.points[stalk_points..] {
                assert!(inside(Vec3::from(*point)), "{leaf:?}: {point:?} is out");
            }
            assert!(!inside(Vec3::ZERO), "{leaf:?}: the foot is in");
            assert!(half.min_element() >= BOX_SKIN);
        }
        let lobed = leaves[0];
        let (placed, half) = collider(&lobed, &mesh(&lobed).unwrap());
        assert!(half.z < 0.03 && half.x > 0.12 && half.y > 0.15, "{half}");
        assert!(
            placed
                .rotation
                .abs_diff_eq(Quat::from_rotation_y(0.7), 1e-6)
                && placed.translation.x > 0.35,
            "{placed:?}"
        );
    }

    /// A trifoliate leaf reaches its length, its pair spreads both sides
    /// out along the petiole (a turned leaflet's base stub reaches a little
    /// back), and it costs three leaflets' triangles.
    #[test]
    fn a_trifoliate_leaf_spreads_a_pair_past_its_petiole() {
        let leaf = LeafConfig::trifoliate(0.2, 0.7).petiole(0.08);
        let mesh = mesh(&leaf).unwrap();
        let (x0, x1) = bounds(&mesh, 0);
        let (y0, y1) = bounds(&mesh, 1);
        assert!(
            x0.abs() < 1e-6 && (0.9..=1.0).contains(&(x1 / 0.2)),
            "{x0}..{x1}"
        );
        assert!(
            y0 < -0.05 && y1 > 0.05 && (y0 + y1).abs() < 1e-4,
            "{y0}..{y1}"
        );
        let stalk_points = stalk(1.0, 1.0).points.len();
        let (blade_x0, _) = bounds(
            &MeshData {
                points: mesh.points[stalk_points..].to_vec(),
                face_vertex_counts: vec![],
                face_vertex_indices: vec![],
            },
            0,
        );
        assert!(
            blade_x0 > 0.06,
            "the leaflets start out along the petiole: {blade_x0}"
        );
        assert!(
            mesh.face_vertex_counts.len() < 400,
            "{}",
            mesh.face_vertex_counts.len()
        );
    }
}

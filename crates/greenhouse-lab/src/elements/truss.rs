//! Truss — a truss of tomatoes: the peduncle off the stem, the rachis the
//! fruit hang along, and a pedicel to every fruit.
//!
//! A truss is built in its own frame, on the stem's axis at the node it grew
//! from, facing +X with +Z up. The peduncle leaves the stem `peduncle_angle`
//! off vertical and bends over under the fruit's weight into the rachis,
//! which hangs [`RACHIS_TILT`] off straight down, out from the stem:
//!
//! ```text
//!         Bend
//!     ╭────╮
//!  ───╯     ╲        a pedicel to every fruit, off alternate sides
//!  Cut       ╲ ●
//!          ●  ╲  ●
//!               ●
//! ```
//!
//! A branched truss, the cherry kind, forks into two rachises at the first
//! fruit and shares its fruit between them.
//!
//! Peduncle, rachis and pedicels are one mesh, quantized like any organ, to a
//! budget of `variations`. The representative a truss drew decides where its
//! `Cut` and `Bend` and its fruit go; which fruit are missing and how big each
//! one is are drawn per truss.
//!
//! ```text
//! Truss_04            Xform, a TrussConfig
//!   Peduncle          -> parts/Truss_k
//!   Cut               Xform at the peduncle's base, +X along the peduncle
//!   Bend              Xform where the peduncle bends most, +X along it
//!   Fruit_00 …        a FruitConfig, hung from a pedicel's tip
//! ```
//!
//! `Cut` and `Bend` are where the two kinds of harvesting gripper take a
//! truss: one cuts flush at the stem, the other clamps the bend and cuts
//! beside it. Both are frames, since the approach matters as much as the
//! point.

use std::f32::consts::{FRAC_PI_2, PI, TAU};

use bevy::prelude::*;
use misina_lab::geometry::mesh::{MeshData, merge_meshes};
use misina_lab::geometry::par_map;
use misina_lab::geometry::strand::{Bark, Strand, strand_mesh};
use misina_lab::palette::{COLOR_STREAM, Response, shade, srgb};
use misina_lab::params::{Label, Slider};
use misina_lab::quantize::{Metric, farthest_first};
use misina_lab::rng::{Rng, salt};
use misina_lab::scene::{Library, Order, configs_changed};
use nalgebra::Point3;

use super::fruit::{FruitConfig, FruitParams, Hung};
use super::{Grow, SceneParams};

/// The mesh-library prefix the trusses register under.
pub const PART: &str = "Truss";

/// Salts splitting a truss's shape draws, its fruit draws and each fruit's
/// own shape off the scene seed.
const SHAPE_STREAM: u64 = 0x94D0_49BB_1331_11EB;
const FRUIT_STREAM: u64 = 0xD6E8_FEB8_6659_FD93;
const FRUIT_SHAPE_STREAM: u64 = 0xA076_1D64_78BD_642F;

/// How far a truss's peduncle and fruit vary about the params, as fractions.
const LENGTH_JITTER: f64 = 0.2;
const RADIUS_JITTER: f64 = 0.15;
const ANGLE_JITTER: f64 = 0.15;
const COUNT_JITTER: f64 = 0.2;
const DIAMETER_JITTER: f64 = 0.08;

/// How far one fruit's diameter varies about its truss's, as a fraction, and
/// how much smaller the fruit at a truss's tip is than the one at its base.
const FRUIT_JITTER: f64 = 0.05;
const TIP_SHRINK: f32 = 0.15;

/// How far the rachis hangs off straight down, out from the stem, in
/// radians.
const RACHIS_TILT: f32 = 0.6;

/// Rachis between neighbouring fruit, in fruit diameters. Neighbours hang off
/// alternate sides, so the fruit down one side are 1.1 diameters apart.
const FRUIT_PITCH: f32 = 0.55;

/// Rachis left past the last fruit, in fruit diameters.
const RACHIS_TIP: f32 = 0.3;

/// Where a pedicel's tip sits off the rachis, in fruit radii: out to the
/// side, and down. Far enough that its fruit clears the rachis.
const PEDICEL: Vec2 = Vec2::new(0.9, 0.3);

/// The share of a peduncle that runs straight out of the stem before it
/// bends.
const STRAIGHT: f32 = 0.5;

/// Control points round the bend.
const BEND_POINTS: usize = 6;

/// Least the bend turns, in radians, so a peduncle already pointing down
/// still has a bend to clamp.
const MIN_TURN: f32 = 0.1;

/// A branched truss's two rachises, either side of where a simple one would
/// hang, in radians.
const BRANCH: f32 = 0.45;

/// The rachis's radius at its tip, as a fraction of the peduncle's, and a
/// pedicel's radius, in meters, which a fruit's stub carries on at.
const RACHIS_TAPER: f32 = 0.6;
pub const PEDICEL_RADIUS: f32 = 0.0012;

/// Floors on what Python may set, so that every truss still builds.
const MIN_LENGTH: f32 = 0.01;
const MIN_RADIUS: f32 = 0.0005;
const MIN_DIAMETER: f32 = 0.005;
const ANGLE: (f32, f32) = (0.05, PI - RACHIS_TILT - MIN_TURN);

/// Vertices around the peduncle and round a pedicel, and rings per meter.
const SIDES: usize = 6;
const PEDICEL_SIDES: usize = 4;
const DETAIL: f64 = 120.0;

/// A green peduncle.
const COLOR: u32 = 0x6A8A34;
const RESPONSE: Response = Response {
    roughness: 0.7,
    reflectance: 0.35,
};

/// One truss's shape, as the stem that grew it specified.
#[derive(Component, Clone, Copy, Debug, PartialEq)]
pub struct TrussConfig {
    /// Stem to the first fruit, in meters.
    pub peduncle_length: f32,
    pub peduncle_radius: f32,
    /// Between the stem and the peduncle where it leaves, in radians.
    pub peduncle_angle: f32,
    pub fruits: u32,
    pub fruit_diameter: f32,
    pub branched: bool,
    /// What it takes from its stem, kept so the params can be re-applied to a
    /// truss already hanging.
    pub node: Node,
}

/// What a truss takes from the node of the stem it hangs on. Placement and
/// ripening, not shape: [`TrussMetric`] reads none of it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Node {
    /// The stem's radius at the node; the `Cut` sits that far out along the
    /// peduncle.
    pub stem_radius: f32,
    /// How many weeks this one is behind its stem's ripening front, a truss
    /// a week, which its fruit's ripeness is read off.
    pub rank: f32,
    /// How far its fruit have come toward full size, 0 just set to 1: the
    /// trusses nearest the head set theirs last.
    pub growth: f32,
}

impl TrussConfig {
    /// The truss these params call for at the `order`-th node a stem hung one
    /// on, its jitter drawn off a stream of its own.
    ///
    /// The *draw order* is part of this element's output: the peduncle's
    /// length, radius and angle, then the fruit count and diameter.
    pub fn new(params: &TrussParams, seed: u64, order: u64, node: Node) -> Self {
        let mut rng = Rng::new(seed ^ SHAPE_STREAM ^ salt(order));
        let mut about = |jitter: f64| rng.range(1.0 - jitter, 1.0 + jitter) as f32;
        let length = params.peduncle_length * about(LENGTH_JITTER);
        let radius = params.peduncle_diameter / 2.0 * about(RADIUS_JITTER);
        let angle = params.peduncle_angle.to_radians() * about(ANGLE_JITTER);
        let fruits = (params.fruits as f32 * about(COUNT_JITTER)).round();
        let diameter = params.fruit_diameter * about(DIAMETER_JITTER);
        Self {
            peduncle_length: length.max(MIN_LENGTH),
            peduncle_radius: radius.max(MIN_RADIUS),
            peduncle_angle: angle.clamp(ANGLE.0, ANGLE.1),
            fruits: fruits.max(1.0) as u32,
            fruit_diameter: diameter.max(MIN_DIAMETER),
            branched: params.branched,
            node,
        }
    }
}

/// Two trusses share a mesh when they are close in every dimension that
/// shows, weighted as roughly how far apart each moves a fruit.
pub struct TrussMetric;

impl Metric<TrussConfig> for TrussMetric {
    fn distance(&self, a: &TrussConfig, b: &TrussConfig) -> f32 {
        // A forked truss and a simple one are different shapes, not near
        // sizes of one.
        if a.branched != b.branched {
            return 1.0;
        }
        [
            a.peduncle_length - b.peduncle_length,
            (a.peduncle_radius - b.peduncle_radius) * 8.0,
            // A radian swings the rachis's root by a peduncle's length.
            (a.peduncle_angle - b.peduncle_angle) * a.peduncle_length.max(b.peduncle_length),
            // A fruit more is a pedicel more, half a fruit off the last.
            (a.fruits as f32 - b.fruits as f32) * a.fruit_diameter.max(b.fruit_diameter) / 2.0,
            // The rachis is spaced by the fruit, so its tip moves by the
            // difference times the fruit along it.
            (a.fruit_diameter - b.fruit_diameter) * FRUIT_PITCH * a.fruits.max(b.fruits) as f32,
        ]
        .iter()
        .map(|d| d * d)
        .sum::<f32>()
        .sqrt()
    }
}

/// The trusses: a peduncle off the stem, bending over into a rachis with the
/// fruit hung along it.
///
/// The peduncle numbers default to the means measured on medium trusses in a
/// high-wire house, and every truss draws its own about them. A medium truss
/// tomato is about 6 fruit of 60 mm; a cherry truss about 15 of 25 mm,
/// branched.
#[derive(Resource, Reflect, Clone, Debug, PartialEq)]
#[cfg_attr(
    feature = "python",
    pyo3::pyclass(get_all, set_all, skip_from_py_object)
)]
pub struct TrussParams {
    /// Fruit on a truss. Each truss draws a few more or fewer.
    #[reflect(@Slider { min: 1.0, max: 24.0, step: 1.0 })]
    pub fruits: u32,
    /// Diameter of a fruit at the base of a truss, in meters; the fruit
    /// toward its tip come out a little smaller.
    #[reflect(@Slider { min: 0.02, max: 0.08, step: 0.002 })]
    pub fruit_diameter: f32,
    /// Whether the rachis forks in two at the first fruit, as a cherry truss
    /// does, rather than hanging as one.
    pub branched: bool,
    /// Length of the peduncle from the stem to the first fruit, in meters.
    #[reflect(@Slider { min: 0.03, max: 0.14, step: 0.001 })]
    pub peduncle_length: f32,
    /// Diameter of the peduncle, in meters.
    #[reflect(@Slider { min: 0.004, max: 0.012, step: 0.0005 })]
    pub peduncle_diameter: f32,
    /// Angle between the stem and the peduncle where it leaves, in degrees:
    /// 90 leaves level, less leaves rising.
    #[reflect(@Slider { min: 5.0, max: 139.0, step: 1.0 })]
    pub peduncle_angle: f32,
    /// Chance that a fruit is missing from its pedicel.
    #[reflect(@Slider { min: 0.0, max: 0.3, step: 0.01 })]
    pub missing_fruit: f32,
    /// How many distinct truss meshes the scene may hold. A budget, not a
    /// count: the trusses are clustered and this is how many representatives
    /// the clustering may keep.
    #[reflect(@Slider { min: 1.0, max: 64.0, step: 1.0 }, @Label("Truss variations"))]
    pub variations: u32,
}

impl Default for TrussParams {
    fn default() -> Self {
        Self {
            fruits: 6,
            fruit_diameter: 0.06,
            branched: false,
            peduncle_length: 0.079,
            peduncle_diameter: 0.007,
            peduncle_angle: 70.0,
            missing_fruit: 0.05,
            variations: 16,
        }
    }
}

pub fn plugin(app: &mut App) {
    // `SceneParams` is not gated on although the configs read the seed:
    // `stem::author` gates on it and hangs fresh trusses when it moves.
    app.init_resource::<TrussParams>().add_systems(
        PreUpdate,
        (
            reauthor.run_if(resource_changed::<TrussParams>),
            build.run_if(
                configs_changed::<TrussConfig>
                    // `variations` and `missing_fruit` reach no config.
                    .or_eager(resource_changed::<TrussParams>),
            ),
        )
            .chain()
            .in_set(Grow::Trusses),
    );
}

/// Re-applies the params to every truss already hanging, in place, from the
/// same draws: they are keyed on its `Order`.
fn reauthor(
    params: Res<TrussParams>,
    scene: Res<SceneParams>,
    mut trusses: Query<(&Order, &mut TrussConfig)>,
) {
    for (order, mut config) in &mut trusses {
        let next = TrussConfig::new(&params, scene.seed, order.0, config.node);
        config.set_if_neq(next);
    }
}

/// One representative truss: its mesh, and the frames its instances hang
/// things on.
struct Built {
    mesh: Mesh,
    /// The peduncle's direction leaving the stem, which `Cut` faces.
    leaves: Vec3,
    bend: Transform,
    /// Every pedicel's tip, from the truss's base to its tip.
    tips: Vec<Vec3>,
}

/// The direction `angle` off +Z toward +X.
fn heading(angle: f32) -> Vec3 {
    Vec3::new(angle.sin(), 0.0, angle.cos())
}

fn strand(points: &[Vec3], radii: &[f32], sides: usize) -> Strand {
    let points = points
        .iter()
        .map(|p| Point3::new(p.x as f64, p.y as f64, p.z as f64))
        .collect();
    let radii = radii.iter().map(|r| *r as f64).collect();
    Strand::new(points, radii, sides, DETAIL, Bark::none())
}

/// Builds one truss in its own frame.
fn build_truss(config: &TrussConfig) -> Result<Built> {
    let angle = config.peduncle_angle;
    let turn = (PI - RACHIS_TILT - angle).max(MIN_TURN);
    let straight = config.peduncle_length * STRAIGHT;
    let radius = config.peduncle_length * (1.0 - STRAIGHT) / turn;
    // Out of the stem along `angle`, then round a circle of `radius` until it
    // heads `turn` further over.
    let start = heading(angle) * straight;
    let arc =
        |a: f32| start + radius * Vec3::new(angle.cos() - a.cos(), 0.0, a.sin() - angle.sin());
    let mut peduncle = vec![Vec3::ZERO, start / 2.0, start];
    peduncle.extend((1..=BEND_POINTS).map(|i| arc(angle + turn * i as f32 / BEND_POINTS as f32)));
    let fork = arc(angle + turn);
    let hang = heading(angle + turn);

    let arms: &[f32] = if config.branched {
        &[BRANCH, -BRANCH]
    } else {
        &[0.0]
    };
    let per_arm = (config.fruits as usize).div_ceil(arms.len());
    let (pitch, fruit) = (
        config.fruit_diameter * FRUIT_PITCH,
        config.fruit_diameter / 2.0,
    );
    let reach = ((per_arm - 1) as f32 + RACHIS_TIP) * pitch;
    let r = config.peduncle_radius;

    let mut strands = Vec::new();
    let mut tips = vec![Vec3::ZERO; config.fruits as usize];
    for (arm, yaw) in arms.iter().enumerate() {
        let along = Quat::from_rotation_z(*yaw) * hang;
        let side = Vec3::Z.cross(along).normalize();
        // The first rachis carries on from the peduncle; a second starts a
        // little back inside it.
        let mut points = if arm == 0 {
            peduncle.clone()
        } else {
            vec![fork - hang * pitch * 0.5, fork]
        };
        let mut radii = vec![r; points.len()];
        let rachis = (1..=4).map(|i| reach * i as f32 / 4.0);
        for s in rachis {
            points.push(fork + along * s);
            radii.push(r * (1.0 - (1.0 - RACHIS_TAPER) * s / reach));
        }
        strands.push(strand(&points, &radii, SIDES));

        // Fruit alternate between the arms, then between the sides of each.
        for (i, j) in (arm..config.fruits as usize)
            .step_by(arms.len())
            .enumerate()
        {
            let base = fork + along * (i as f32 * pitch);
            let flip = if (i % 2 == 0) == (arm == 0) {
                1.0
            } else {
                -1.0
            };
            let tip = base + (side * flip * PEDICEL.x - Vec3::Z * PEDICEL.y) * fruit;
            let mid = base.lerp(tip, 0.5) + Vec3::Z * fruit * 0.15;
            strands.push(strand(
                &[base, mid, tip],
                &[PEDICEL_RADIUS; 3],
                PEDICEL_SIDES,
            ));
            tips[j] = tip;
        }
    }

    let meshes: Vec<MeshData> = strands.iter().map(strand_mesh).collect::<Result<_, _>>()?;
    let middle = angle + turn / 2.0;
    Ok(Built {
        mesh: merge_meshes(&meshes).to_mesh(),
        leaves: heading(angle),
        bend: Transform::from_translation(arc(middle))
            .with_rotation(Quat::from_rotation_y(middle - FRAC_PI_2)),
        tips,
    })
}

/// The rotation turning +X onto `direction`, which lies in the XZ plane.
fn facing(direction: Vec3) -> Quat {
    Quat::from_rotation_y(-direction.z.atan2(direction.x))
}

/// Builds one mesh per distinct truss, and hangs its frames and its fruit off
/// every truss.
///
/// Each truss's fruit draw from a stream keyed on its [`Order`]: three draws
/// per pedicel, whether its fruit is missing, how big it is and how it is
/// turned about its axis, made for every pedicel so a higher `missing_fruit`
/// drops fruit without resizing the rest. A fruit's own shape is drawn off a
/// stream keyed on its truss and its pedicel.
pub(crate) fn build(
    mut commands: Commands,
    mut library: Library,
    scene: Res<SceneParams>,
    params: Res<TrussParams>,
    fruit_params: Res<FruitParams>,
    trusses: Query<(Entity, &Order, &TrussConfig)>,
) -> Result<()> {
    library.clear(PART);

    let mut hung: Vec<(Order, Entity, TrussConfig)> = trusses
        .iter()
        .map(|(entity, order, config)| (*order, entity, *config))
        .collect();
    hung.sort_by_key(|(order, ..)| *order);

    let configs: Vec<TrussConfig> = hung.iter().map(|(_, _, config)| *config).collect();
    let book = farthest_first(
        &configs,
        params.variations.max(1) as usize,
        0.0,
        &TrussMetric,
    );
    let shapes = par_map(&book.representatives, |_, config| build_truss(config));
    let mut built = Vec::with_capacity(book.len());
    for (index, shape) in shapes.into_iter().enumerate() {
        let shape = shape?;
        let surface = RESPONSE.surface(shade(
            srgb(COLOR),
            &mut Rng::new(scene.seed ^ COLOR_STREAM ^ salt(index as u64)),
        ));
        let geometry = library.part(PART, index, shape.mesh, surface);
        built.push((geometry, shape.leaves, shape.bend, shape.tips));
    }

    let mut fruit_order = 0u64;
    for ((order, entity, config), drew) in hung.iter().zip(&book.assignment) {
        let (geometry, leaves, bend, tips) = &built[*drew as usize];
        let mut truss = commands.entity(*entity);
        // The layer owns everything below a truss, and a rebuild may hang a
        // different number of fruit than the last one did.
        truss.despawn_children();
        truss.with_child((Name::new("Peduncle"), geometry.clone()));
        truss.with_child((
            Name::new("Cut"),
            Transform::from_translation(*leaves * config.node.stem_radius)
                .with_rotation(facing(*leaves)),
            Visibility::default(),
        ));
        truss.with_child((Name::new("Bend"), *bend, Visibility::default()));

        let mut rng = Rng::new(scene.seed ^ FRUIT_STREAM ^ salt(order.0));
        let last = (tips.len() - 1).max(1) as f32;
        for (j, tip) in tips.iter().enumerate() {
            let missing = rng.unit() < params.missing_fruit as f64;
            let size = rng.range(1.0 - FRUIT_JITTER, 1.0 + FRUIT_JITTER) as f32;
            let spin = rng.unit() as f32 * TAU;
            if missing {
                continue;
            }
            let along = j as f32 / last;
            fruit_order += 1;
            truss.with_child((
                Name::new(format!("Fruit_{j:02}")),
                Transform::from_translation(*tip).with_rotation(Quat::from_rotation_z(spin)),
                Visibility::default(),
                FruitConfig::new(
                    &fruit_params,
                    // A truss holds fewer than 256 fruit.
                    scene.seed ^ FRUIT_SHAPE_STREAM ^ salt(order.0 << 8 | j as u64),
                    config.fruit_diameter * (1.0 - TIP_SHRINK * along) * size,
                    Hung {
                        rank: config.node.rank,
                        along,
                        growth: config.node.growth,
                    },
                ),
                Order(fruit_order),
            ));
        }
    }
    Ok(())
}

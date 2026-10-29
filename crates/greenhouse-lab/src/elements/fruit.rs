//! Fruit — the tomatoes, each its own prim, hung from a pedicel's tip.
//!
//! A fruit is built round its own axis and hangs below its origin, where the
//! truss's pedicel ends. A stub carries on from there down to the calyx,
//! with a knuckle on a jointed variety, and the body hangs from the calyx by
//! its stem scar, which sits in a dip between the shoulders. The sepals,
//! narrow and pointed, lie on the shoulder for a short way and lift off it
//! to spread, each folded along its midrib and twisted a little:
//!
//! ```text
//!          ┃      the pedicel's tip at the origin, a knuckle just below it
//!    ╲     ┃     ╱   the stub down to the calyx
//!     ╲ ╭──┸──╮ ╱    sepals off the shoulders, spreading out and up
//!      (         )
//!       ╲       ╱    colour starts at the blossom end
//!        `─────'
//! ```
//!
//! Every fruit is an Xform carrying its stage, with the mesh as `Skin` under
//! it and a `Collision` capsule round the body: a sphere on a round fruit,
//! a capsule standing on an elongated one. The calyx and the stub stand
//! clear of it.
//!
//! A fruit's `stage` runs from fruit set through expanding and mature green,
//! over which it grows to full size, to the USDA colour classes breaker,
//! turning, pink, light red and red, which are defined by how much of the
//! surface has coloured: up to 10%, 30%, 60%, 90% and the rest. The colour
//! climbs from the blossom end, so the shoulders turn last. The calyx is the
//! size the flower left it, so a young fruit's sepals are large next to it.
//!
//! A stem ripens from the bottom up, about a truss a week, and a truss from
//! its base to its tip. A fruit's ripening is read off how far its truss
//! hangs below its stem's ripening front and how far toward the truss's tip
//! it hangs, in trusses of the breaker-to-red ripening:
//!
//! ```text
//! ripening = ripe_trusses - rank - spread × along
//! ```
//!
//! Over 0 the fruit is colouring, 1 is fully red, and the truss above the
//! front is mature green. Further up, toward the head, the fruit are still
//! growing.
//!
//! Colour is baked per mesh, one per point, so fruit are quantized on their
//! stage as well as their shape, and every fruit keeps its own prim with its
//! own stage and ripeness on it as attributes.

use std::f32::consts::{PI, TAU};

use bevy::prelude::*;
use misina_lab::geometry::mesh::{MeshData, merge_meshes};
use misina_lab::geometry::outline::outline_mesh;
use misina_lab::geometry::par_map;
use misina_lab::geometry::shapes::leaf;
use misina_lab::palette::{Response, mix, srgb};
use misina_lab::params::{Choices, Label, Slider};
use misina_lab::quantize::{Metric, farthest_first};
use misina_lab::rng::Rng;
use misina_lab::scene::{
    COLLISION, Library, Order, Surface, UsdAttributes, capsule, configs_changed,
};

use super::truss::pedicel_radius;
use super::{Grow, smooth};

/// The mesh-library prefix the fruit register under, and the prim under a
/// fruit that draws it.
pub const PART: &str = "Fruit";
pub const SKIN_PRIM: &str = "Skin";

/// Salt splitting a fruit's mesh draws off the stream its config draws from.
const CALYX_STREAM: u64 = 0xC2B2_AE3D_27D4_EB4F;

/// The stages, as `stage` counts them: a fruit at 4.5 is half way through
/// turning, and one at 8 is fully red.
pub const STAGES: [&str; 8] = [
    "set",
    "expanding",
    "mature_green",
    "breaker",
    "turning",
    "pink",
    "light_red",
    "red",
];
const MATURE_GREEN: f32 = 2.0;
const BREAKER: f32 = 3.0;
const RIPE: f32 = 8.0;

/// The share of the surface coloured at the start of each stage from breaker
/// on, and at the end of red: the bounds of the USDA classes.
const COLOURED: [f32; 6] = [0.0, 0.1, 0.3, 0.6, 0.9, 1.0];

/// A fruit's diameter at set, as a fraction of its full size.
const SET_SIZE: f32 = 0.15;

/// How far a fruit's height-to-width varies about the params, as a fraction.
const ASPECT_JITTER: f64 = 0.05;

/// A fruit's density, in kg/m³: about water's (est.).
const DENSITY: f32 = 1000.0;

/// Lobes a fruit may have, and the fewest sides round the body: a whole
/// number of sides per lobe, at least this many in all.
const LOBES: (u32, u32) = (3, 5);
const SECTORS: u32 = 16;

/// The meridian is a superellipse, `|sin|` and `|cos|` raised to this: under
/// 1 gives the broad shoulders and broad blossom end of a tomato.
const SQUARE: f32 = 0.83;

/// The stem scar, and the rim of the dip round it, as polar angles from the
/// stem end; and the dip's depth at the scar, as a fraction of the radius.
const SCAR: f32 = 0.1;
const RIM: f32 = 0.38;
const DIP: f32 = 0.17;

/// Rings round the body below the rim, down to the blossom end.
const BELOW_RIM: usize = 6;

/// How far the blossom end leans off the axis and one side bulges, as
/// fractions of the radius, at most.
const LEAN: f64 = 0.06;
const BULGE: f64 = 0.04;

/// The stub from the pedicel's tip to the calyx, and a sepal, as fractions
/// of the full-grown diameter; a sepal's width as a fraction of its length,
/// and where along it it is widest.
const STUB: f32 = 0.2;
const SEPAL: f32 = 0.5;
const SEPAL_WIDTH: f32 = 0.2;
const SEPAL_PEAK: f64 = 0.2;

/// How far a sepal's length and width vary, and how far it strays from even
/// spacing, as fractions.
const SEPAL_JITTER: f64 = 0.15;
const SEPAL_STRAY: f64 = 0.25;

/// The share of a sepal that lies on the fruit; how far it lifts off the
/// skin where it leaves, in radians; and how much further the rest of it
/// curls that way, in radians: each drawn per sepal.
const DROOP: (f64, f64) = (0.08, 0.3);
const LIFT: (f64, f64) = (0.3, 0.9);
const CURL: (f64, f64) = (0.1, 0.8);

/// How far a sepal twists about its midrib over its length, in radians, at
/// most either way, drawn per sepal; and how steeply its edges rise off the
/// midrib toward its upper face, as a slope, the fold every sepal has.
const TWIST: f64 = 0.6;
const KEEL: f32 = 0.5;

/// How far down the body a sepal may lie before it leaves it, as a polar
/// angle; how far into the scar it starts, as a fraction of the scar; and its
/// clearance off the skin, in meters.
const HUG: f32 = 1.2;
const SEPAL_START: f32 = 0.4;
const GAP: f32 = 0.0004;

/// Sides round the stub, the knuckle's radius in pedicels, and where it sits
/// down the stub: just under the pedicel's tip, where the fruit's tilt bends
/// the stub off the pedicel, so the bend is at the joint.
const STUB_SIDES: usize = 5;
const KNUCKLE: (f32, f32) = (1.5, 0.12);

/// How far past a point the colour front has to climb before the point is
/// fully coloured, as a share of the fruit's height.
const FRONT: f32 = 0.3;

/// What a stage is worth in meters of diameter to the clustering: a stage of
/// colour shows about as much as a centimeter of size.
const STAGE_WEIGHT: f32 = 0.01;

/// A young fruit, a mature green one, and the tan a fruit breaks to, before
/// its ripe colour comes through.
const YOUNG: u32 = 0x74A83A;
const MATURE: u32 = 0x9DB55A;
const BLUSH: u32 = 0xD6A35E;

/// The calyx and the stub.
const SEPAL_COLOR: u32 = 0x355E1E;
const STUB_COLOR: u32 = 0x6A8A34;

/// A tomato's waxy skin.
const SKIN: Response = Response {
    roughness: 0.3,
    reflectance: 0.55,
};

/// The colour a fruit ripens to.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Ripe {
    #[default]
    Red,
    Orange,
    Yellow,
    Brown,
}

impl Ripe {
    pub const ALL: [Ripe; 4] = [Ripe::Red, Ripe::Orange, Ripe::Yellow, Ripe::Brown];

    /// The names, in [`ALL`](Self::ALL) order.
    pub const NAMES: [&'static str; 4] = ["red", "orange", "yellow", "brown"];

    pub fn name(self) -> &'static str {
        Self::NAMES[self as usize]
    }

    pub fn parse(name: &str) -> Option<Ripe> {
        Self::ALL.into_iter().find(|ripe| ripe.name() == name)
    }

    /// The fully ripe skin, as sRGB.
    fn hex(self) -> u32 {
        [0xB3241A, 0xE0671C, 0xE6B92C, 0x5E2A1C][self as usize]
    }
}

/// The name of the stage a fruit at `stage` is in.
pub fn stage_name(stage: f32) -> &'static str {
    STAGES[(stage.max(0.0) as usize).min(STAGES.len() - 1)]
}

/// The share of a fruit's surface that has coloured at `stage`, 0 to 1.
pub fn ripeness(stage: f32) -> f32 {
    let s = (stage - BREAKER).clamp(0.0, RIPE - BREAKER);
    let i = (s as usize).min(COLOURED.len() - 2);
    COLOURED[i] + (COLOURED[i + 1] - COLOURED[i]) * (s - i as f32)
}

/// Where a fruit hangs, as the truss that holds it saw it: kept so the params
/// can be re-applied to a fruit already hanging. [`FruitMetric`] reads none
/// of it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Hung {
    /// How many weeks its truss is behind its stem's ripening front, a truss
    /// a week.
    pub rank: f32,
    /// How far along its truss toward the tip it hangs, 0 to 1.
    pub along: f32,
    /// How far its truss's fruit have come toward full size, 0 to 1.
    pub growth: f32,
}

/// One fruit's shape and stage, as the truss that holds it specified.
#[derive(Component, Clone, Copy, Debug, PartialEq)]
pub struct FruitConfig {
    /// Across when full grown, in meters. The size it is now follows from its
    /// stage, and the calyx and stub from this.
    pub diameter: f32,
    /// How far it has come, 0 just set to 8 fully red; see [`STAGES`].
    pub stage: f32,
    /// Height over width.
    pub aspect: f32,
    /// Depth of the grooves between its lobes, as a fraction of its radius.
    pub ribbing: f32,
    pub lobes: u32,
    pub sepals: u32,
    pub jointed: bool,
    /// The stream its calyx and its lean are drawn from.
    pub seed: u64,
    pub hung: Hung,
}

impl FruitConfig {
    /// The fruit these params call for, `diameter` across when full grown,
    /// its own variation drawn off `seed`.
    ///
    /// The *draw order* is part of this element's output: the height to
    /// width, the lobe count and the sepal count.
    pub fn new(params: &FruitParams, seed: u64, diameter: f32, hung: Hung) -> Self {
        let mut rng = Rng::new(seed);
        let aspect = params.aspect * rng.range(1.0 - ASPECT_JITTER, 1.0 + ASPECT_JITTER) as f32;
        let lobes = LOBES.0 + (rng.unit() * (LOBES.1 - LOBES.0 + 1) as f64) as u32;
        let sepals = if rng.unit() < 0.5 { 5 } else { 6 };

        let ripening = params.ripe_trusses - hung.rank - params.spread * hung.along;
        let stage = if ripening >= 0.0 {
            BREAKER + (RIPE - BREAKER) * ripening.min(1.0)
        } else {
            // Growing toward mature green, then through it in the truss above
            // the front.
            (MATURE_GREEN * hung.growth.clamp(0.0, 1.0) + (1.0 + ripening).max(0.0)).min(BREAKER)
        };
        Self {
            diameter,
            stage,
            aspect: aspect.max(0.2),
            ribbing: params.ribbing.clamp(0.0, 0.5),
            lobes,
            sepals,
            jointed: params.jointed,
            seed,
            hung,
        }
    }

    /// Across now, in meters: it grows from set to mature green.
    pub fn across(&self) -> f32 {
        let t = (self.stage / MATURE_GREEN).clamp(0.0, 1.0);
        self.diameter * (SET_SIZE + (1.0 - SET_SIZE) * t * t * (3.0 - 2.0 * t))
    }

    /// Its mass now, in kilograms: a spheroid `across` wide and `aspect` as
    /// tall as that.
    pub fn mass(&self) -> f32 {
        DENSITY * PI / 6.0 * self.across().powi(3) * self.aspect
    }
}

/// Two fruit share a mesh when they are near in size, in shape and in colour.
pub struct FruitMetric;

impl Metric<FruitConfig> for FruitMetric {
    fn distance(&self, a: &FruitConfig, b: &FruitConfig) -> f32 {
        // A knuckle more or less is a different stub, not a near one.
        if a.jointed != b.jointed {
            return 1.0;
        }
        let (across_a, across_b) = (a.across(), b.across());
        let radius = across_a.max(across_b) / 2.0;
        [
            across_a - across_b,
            // The calyx is sized off the full-grown fruit.
            (a.diameter - b.diameter) * SEPAL,
            // Each moves the surface by its difference times the radius.
            (a.aspect - b.aspect) * radius,
            (a.ribbing - b.ribbing) * radius,
            (a.stage - b.stage) * STAGE_WEIGHT,
        ]
        .iter()
        .map(|d| d * d)
        .sum::<f32>()
        .sqrt()
    }
}

/// The fruit: their shape, how ripe the crop is, and what colour ripe is.
///
/// Every fruit has a stage: set, expanding and mature green while it grows,
/// then breaker, turning, pink, light red and red as it colours from its
/// blossom end up. A truss sets its fruit near the head, and they reach full
/// size five trusses down. A stem colours from its lowest truss up and a
/// truss from its base to its tip, so in full production the lowest
/// `ripe_trusses` trusses on a stem are colouring and the ones above them
/// still green; a young crop's trusses are all green until the first are
/// old enough. A truss is due once its tip fruit is turning. Every stem runs
/// a little ahead of or behind the rest.
///
/// Every fruit draws its own lobe count, its five or six sepals and how each
/// curls, and a slight lean, within the ranges the research notes give.
#[derive(Resource, Reflect, Clone, Debug, PartialEq)]
#[cfg_attr(
    feature = "python",
    pyo3::pyclass(get_all, set_all, skip_from_py_object)
)]
pub struct FruitParams {
    /// Trusses per stem that have coloured in full production, on average,
    /// counted up from the lowest: a fraction is the next one part way.
    #[reflect(@Slider { min: 0.0, max: 3.0, step: 0.1 })]
    pub ripe_trusses: f32,
    /// How far a truss's tip fruit lags behind its base fruit, as a share of
    /// the ripening from breaker to red: 0 colours a truss all at once.
    #[reflect(@Slider { min: 0.0, max: 1.0, step: 0.05 }, @Label("Ripeness spread in a truss"))]
    pub spread: f32,
    /// The colour a ripe fruit turns: `red`, `orange`, `yellow` or `brown`.
    #[reflect(@Choices(&Ripe::NAMES))]
    pub color: String,
    /// Height of a fruit over its width: under 1 is oblate, as round truss
    /// tomatoes and beefsteaks are, and over 1 elongated, as plum tomatoes
    /// are. Each fruit draws a little either side.
    #[reflect(@Slider { min: 0.5, max: 1.8, step: 0.05 }, @Label("Fruit height to width"))]
    pub aspect: f32,
    /// Depth of the grooves between a fruit's lobes, as a fraction of its
    /// radius: 0 is smooth, and a beefsteak's are about 0.15.
    #[reflect(@Slider { min: 0.0, max: 0.3, step: 0.01 })]
    pub ribbing: f32,
    /// Whether a pedicel has a knuckle, the joint a fruit breaks off at, as
    /// most truss varieties do.
    #[reflect(@Label("Jointed pedicels"))]
    pub jointed: bool,
    /// How many distinct fruit meshes the scene may hold, by size, shape and
    /// stage. A budget, not a count: the fruit are clustered and this is how
    /// many representatives the clustering may keep.
    #[reflect(@Slider { min: 1.0, max: 64.0, step: 1.0 }, @Label("Fruit variations"))]
    pub variations: u32,
}

impl Default for FruitParams {
    fn default() -> Self {
        Self {
            ripe_trusses: 1.0,
            spread: 0.2,
            color: Ripe::default().name().into(),
            aspect: 0.9,
            ribbing: 0.03,
            jointed: true,
            variations: 24,
        }
    }
}

impl FruitParams {
    /// The parsed colour. An unknown name is read as the default: Python has
    /// already rejected it, and a build system is no place to fail.
    pub fn ripe(&self) -> Ripe {
        Ripe::parse(&self.color).unwrap_or_else(|| {
            warn!(
                "fruit color {:?} is not one of {:?}; using {:?}",
                self.color,
                Ripe::NAMES,
                Ripe::default().name()
            );
            Ripe::default()
        })
    }
}

pub fn plugin(app: &mut App) {
    app.init_resource::<FruitParams>().add_systems(
        PreUpdate,
        (
            reauthor.run_if(resource_changed::<FruitParams>),
            build.run_if(
                configs_changed::<FruitConfig>
                    // `color` and `variations` reach no config.
                    .or_eager(resource_changed::<FruitParams>),
            ),
        )
            .chain()
            .in_set(Grow::Fruit),
    );
}

/// Re-applies the params to every fruit already hanging, in place, from the
/// same draws.
fn reauthor(params: Res<FruitParams>, mut fruit: Query<&mut FruitConfig>) {
    for mut config in &mut fruit {
        let next = FruitConfig::new(&params, config.seed, config.diameter, config.hung);
        config.set_if_neq(next);
    }
}

// ─── The mesh ───────────────────────────────────────────────────────

/// A closed surface through `rings`, from `top` down to `bottom`, every ring
/// the same number of points counter-clockwise seen from above. Faces wind
/// outward.
fn lathe(top: Vec3, rings: &[Vec<Vec3>], bottom: Vec3) -> MeshData {
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

/// The body's surface, solved from a config: its stem scar on the axis at
/// `top`, the stub's foot.
struct Body {
    radius: f32,
    half: f32,
    dip: f32,
    ribbing: f32,
    lobes: f32,
    /// Where the blossom end leans to, and the side that bulges and by how
    /// much, as fractions of the radius.
    lean: Vec2,
    bulge: (f32, f32),
    top: f32,
    /// Polar angles of the rings, from the stem end, and the azimuths of the
    /// points round each.
    rings: Vec<f32>,
    sectors: Vec<f32>,
}

impl Body {
    /// Draws, first off the mesh's stream: the lean's size and bearing, then
    /// the bulge's.
    fn new(config: &FruitConfig, rng: &mut Rng) -> Self {
        let radius = config.across() / 2.0;
        let lean = rng.range(0.0, LEAN) as f32;
        let lean = Vec2::from_angle(rng.unit() as f32 * TAU) * lean;
        let bulge = (rng.range(0.0, BULGE) as f32, rng.unit() as f32 * TAU);
        let mut rings = vec![SCAR, (SCAR + RIM) / 2.0, RIM];
        rings.extend((1..=BELOW_RIM).map(|i| RIM + (PI - RIM) * i as f32 / (BELOW_RIM + 1) as f32));
        let n = config.lobes * SECTORS.div_ceil(config.lobes.max(1));
        let dip = DIP * radius;
        let half = radius * config.aspect;
        Self {
            radius,
            half,
            dip,
            ribbing: config.ribbing,
            lobes: config.lobes as f32,
            lean,
            bulge,
            top: -STUB * config.diameter,
            rings,
            sectors: (0..n).map(|i| TAU * i as f32 / n as f32).collect(),
        }
    }

    /// The point `theta` from the stem end and `phi` round.
    fn at(&self, theta: f32, phi: f32) -> Vec3 {
        let (sin, cos) = theta.sin_cos();
        let rho = self.radius * sin.abs().powf(SQUARE);
        let z = self.half * cos.signum() * cos.abs().powf(SQUARE);
        let rib = 1.0 - self.ribbing * (1.0 - (self.lobes * phi).cos()) / 2.0;
        let bulge = 1.0 + self.bulge.0 * (phi - self.bulge.1).cos();
        let dip = self.dip * (1.0 - smooth((theta - SCAR) / (RIM - SCAR)));
        // 0 at the stem end to 1 at the blossom end.
        let down = (1.0 - z / self.half) / 2.0;
        let r = rho * rib * bulge;
        Vec3::new(
            r * phi.cos(),
            r * phi.sin(),
            self.top - self.half + self.dip + z - dip,
        ) + (self.lean * self.radius * down).extend(0.0)
    }

    fn mesh(&self) -> MeshData {
        let rings: Vec<Vec<Vec3>> = self
            .rings
            .iter()
            .map(|theta| {
                self.sectors
                    .iter()
                    .map(|phi| self.at(*theta, *phi))
                    .collect()
            })
            .collect();
        lathe(self.at(0.0, 0.0), &rings, self.at(PI, 0.0))
    }

    /// The meridian `phi` round, the points the body's own rings put on it,
    /// from the stem scar down as far as a sepal may lie.
    fn meridian(&self, phi: f32) -> Vec<Vec3> {
        let mut line = vec![self.at(0.0, phi)];
        line.extend(
            self.rings
                .iter()
                .take_while(|theta| **theta <= HUG)
                .map(|theta| self.at(*theta, phi)),
        );
        line
    }
}

/// The skin at `h` of the way up the body, from the blossom end, at `stage`.
fn skin(stage: f32, h: f32, ripe: [f32; 3]) -> [f32; 3] {
    let green = mix(srgb(YOUNG), srgb(MATURE), stage / BREAKER);
    let coloured = ripeness(stage);
    // The front passes the shoulders by `FRONT` as the last of the surface
    // turns, so a fruit at the end of red is red to its stem.
    let front = coloured * (1.0 + FRONT * coloured);
    let turned = ((front - h) / FRONT).clamp(0.0, 1.0);
    let depth = ((stage - BREAKER) / (RIPE - BREAKER))
        .clamp(0.0, 1.0)
        .sqrt();
    mix(green, mix(srgb(BLUSH), ripe, turned * depth), turned)
}

/// A position along a polyline, by arc length, with its direction and the
/// normal `direction × side` blended between the segments either side.
struct Path {
    points: Vec<Vec3>,
    arcs: Vec<f32>,
    normals: Vec<Vec3>,
}

impl Path {
    fn new(points: Vec<Vec3>, side: Vec3) -> Self {
        let mut arcs = vec![0.0];
        for w in points.windows(2) {
            arcs.push(arcs.last().unwrap() + w[0].distance(w[1]));
        }
        let faces: Vec<Vec3> = points
            .windows(2)
            .map(|w| (w[1] - w[0]).cross(side).normalize_or(Vec3::Z))
            .collect();
        let normals = (0..points.len())
            .map(|j| {
                let (a, b) = (faces[j.saturating_sub(1)], faces[j.min(faces.len() - 1)]);
                (a + b).normalize_or(a)
            })
            .collect();
        Self {
            points,
            arcs,
            normals,
        }
    }

    fn length(&self) -> f32 {
        *self.arcs.last().unwrap()
    }

    /// The point `s` along, its direction and its normal.
    fn at(&self, s: f32) -> (Vec3, Vec3, Vec3) {
        let j = self
            .arcs
            .partition_point(|a| *a <= s)
            .clamp(1, self.points.len() - 1)
            - 1;
        let span = (self.arcs[j + 1] - self.arcs[j]).max(1e-9);
        let t = ((s - self.arcs[j]) / span).clamp(0.0, 1.0);
        let along = (self.points[j + 1] - self.points[j]) / span;
        let normal = self.normals[j]
            .lerp(self.normals[j + 1], t)
            .normalize_or(Vec3::Z);
        (self.points[j].lerp(self.points[j + 1], t), along, normal)
    }
}

/// The calyx: one sheet per sepal, each lying on the shoulder from the stem
/// scar out along a meridian of the body's own, lifting off it to spread,
/// folded along its midrib and twisted about it.
///
/// Draws, after the body's: where the first sepal sits, then per sepal its
/// stray from even spacing, its length and width, its droop, its lift, its
/// curl and its twist.
fn calyx(config: &FruitConfig, body: &Body, rng: &mut Rng) -> Result<Vec<MeshData>> {
    let first = rng.unit() as f32 * TAU;
    let length = SEPAL * config.diameter;
    let sectors = body.sectors.len();
    (0..config.sepals)
        .map(|i| {
            let even = TAU / config.sepals as f32;
            let stray = rng.range(-SEPAL_STRAY, SEPAL_STRAY) as f32 * even;
            let long = length * rng.range(1.0 - SEPAL_JITTER, 1.0 + SEPAL_JITTER) as f32;
            let wide =
                long * SEPAL_WIDTH * rng.range(1.0 - SEPAL_JITTER, 1.0 + SEPAL_JITTER) as f32;
            let droop = rng.range(DROOP.0, DROOP.1) as f32;
            let lift = rng.range(LIFT.0, LIFT.1) as f32;
            let curl = rng.range(CURL.0, CURL.1) as f32;
            let twist = rng.range(-TWIST, TWIST) as f32;

            // On a meridian the body's own points lie on, so the sepal lies on
            // its edges rather than across its facets.
            let bearing = (first + i as f32 * even + stray).rem_euclid(TAU);
            let phi = body.sectors[(bearing / TAU * sectors as f32).round() as usize % sectors];
            let side = Vec3::new(-phi.sin(), phi.cos(), 0.0);
            let path = Path::new(body.meridian(phi), side);
            let start = SEPAL_START * path.arcs[1];
            // It leaves past the rim, where the shoulder is convex: the
            // meridian's fourth point, after the pole, the scar and the dip.
            let ridge = path.arcs[3] - start;
            let leave = (droop * long).max(ridge).min(path.length() - start);
            let (from, ahead, up) = path.at(start + leave);
            // Off the skin by `lift` where it leaves, then round an arc that
            // turns the same way by `curl` over what is left.
            let (sin, cos) = lift.sin_cos();
            let (ahead, up) = (ahead * cos + up * sin, up * cos - ahead * sin);
            let bend = curl / (long - leave).max(1e-4);

            let shape = leaf(long as f64, wide as f64, SEPAL_PEAK);
            let mut sheet = outline_mesh(&shape, shape.area())?;
            for point in &mut sheet.points {
                let (x, y) = (point[0], point[1]);
                let (at, along, normal) = if x <= leave {
                    path.at(start + x)
                } else {
                    let a = bend * (x - leave);
                    (
                        from + ahead * (a.sin() / bend) + up * ((1.0 - a.cos()) / bend),
                        ahead * a.cos() + up * a.sin(),
                        up * a.cos() - ahead * a.sin(),
                    )
                };
                // Across the midrib, the edges raised toward the upper face,
                // the whole turned about the midrib by how far along it is.
                let across = side * y + normal * (GAP + KEEL * y.abs());
                let roll = Quat::from_axis_angle(along, twist * x / long);
                *point = (at + roll * across).to_array();
            }
            Ok(sheet)
        })
        .collect()
}

/// The stub from the pedicel's tip down to the calyx, with a knuckle on a
/// jointed fruit, flaring where it meets the sepals.
fn stub(config: &FruitConfig) -> MeshData {
    let (length, r) = (STUB * config.diameter, pedicel_radius(config.diameter));
    let ring = |z: f32, radius: f32| -> Vec<Vec3> {
        (0..STUB_SIDES)
            .map(|i| {
                let a = TAU * i as f32 / STUB_SIDES as f32;
                Vec3::new(radius * a.cos(), radius * a.sin(), -z * length)
            })
            .collect()
    };
    let (swell, at) = KNUCKLE;
    let mut rings = if config.jointed {
        vec![ring(at - 0.1, r), ring(at, r * swell), ring(at + 0.1, r)]
    } else {
        vec![ring(0.06, r)]
    };
    rings.extend([ring(0.85, r * 1.15), ring(1.0, r * 2.0)]);
    lathe(Vec3::ZERO, &rings, Vec3::new(0.0, 0.0, -length - r))
}

/// One fruit in its own frame, hanging below the origin, with a colour per
/// point.
fn fruit_mesh(config: &FruitConfig, ripe: [f32; 3]) -> Result<Mesh> {
    let mut rng = Rng::new(config.seed ^ CALYX_STREAM);
    let body = Body::new(config, &mut rng);
    let skin_mesh = body.mesh();
    let (low, high) = skin_mesh
        .points
        .iter()
        .fold((f32::MAX, f32::MIN), |(lo, hi), p| {
            (lo.min(p[2]), hi.max(p[2]))
        });
    let mut colors: Vec<[f32; 3]> = skin_mesh
        .points
        .iter()
        .map(|p| skin(config.stage, (p[2] - low) / (high - low).max(1e-6), ripe))
        .collect();

    let sepals = calyx(config, &body, &mut rng)?;
    let stub = stub(config);
    for (piece, hex) in sepals
        .iter()
        .map(|s| (s, SEPAL_COLOR))
        .chain([(&stub, STUB_COLOR)])
    {
        colors.extend(std::iter::repeat_n(srgb(hex), piece.points.len()));
    }
    let mut pieces = vec![skin_mesh];
    pieces.extend(sepals);
    pieces.push(stub);

    let rgba: Vec<[f32; 4]> = colors.iter().map(|[r, g, b]| [*r, *g, *b, 1.0]).collect();
    Ok(merge_meshes(&pieces)
        .to_mesh()
        .with_inserted_attribute(Mesh::ATTRIBUTE_COLOR, rgba))
}

/// Builds one mesh per distinct fruit, and hangs every fruit's skin and its
/// collider under it, labelled with its own stage and ripeness.
fn build(
    mut commands: Commands,
    mut library: Library,
    params: Res<FruitParams>,
    fruit: Query<(Entity, &Order, &FruitConfig)>,
) -> Result<()> {
    library.clear(PART);

    let mut hung: Vec<(Order, Entity, FruitConfig)> = fruit
        .iter()
        .map(|(entity, order, config)| (*order, entity, *config))
        .collect();
    hung.sort_by_key(|(order, ..)| *order);

    let configs: Vec<FruitConfig> = hung.iter().map(|(_, _, config)| *config).collect();
    let book = farthest_first(
        &configs,
        params.variations.max(1) as usize,
        0.0,
        &FruitMetric,
    );
    let ripe = srgb(params.ripe().hex());
    let meshes = par_map(&book.representatives, |_, config| fruit_mesh(config, ripe));
    // The points carry the colour, and the sepals are sheets seen from both
    // sides.
    let surface = Surface {
        double_sided: true,
        ..SKIN.surface([1.0; 3])
    };
    let mut geometry = Vec::with_capacity(meshes.len());
    for (index, mesh) in meshes.into_iter().enumerate() {
        geometry.push(library.part(PART, index, mesh?, surface));
    }

    for ((_, entity, config), drew) in hung.iter().zip(&book.assignment) {
        // Round the body: its radius across, and its height along the axis,
        // which a capsule this wide spans only on an elongated fruit.
        let radius = config.across() / 2.0;
        let half = radius * config.aspect;
        let middle = -STUB * config.diameter - half + DIP * radius;
        commands
            .entity(*entity)
            .insert(
                UsdAttributes::default()
                    .with("stage", stage_name(config.stage))
                    .with("ripeness", ripeness(config.stage)),
            )
            // The layer owns everything below a fruit.
            .despawn_children()
            .with_child((Name::new(SKIN_PRIM), geometry[*drew as usize].clone()))
            .with_child((
                Name::new(COLLISION),
                capsule(radius, middle - half, middle + half),
            ));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use std::collections::{BTreeMap, HashMap};

    use super::*;
    use crate::elements::{grown, small};
    use misina_lab::testing::organs;

    /// A fruit of the default params, 60 mm when grown, at `stage`.
    fn config(stage: f32, edit: fn(&mut FruitParams)) -> FruitConfig {
        let mut params = FruitParams::default();
        edit(&mut params);
        FruitConfig {
            stage,
            ..FruitConfig::new(
                &params,
                7,
                0.06,
                Hung {
                    rank: 0.0,
                    along: 0.0,
                    growth: 1.0,
                },
            )
        }
    }

    /// The body and the stub are closed and wound outward: every edge is
    /// walked once each way, and the volume they enclose is positive. They
    /// hang below the fruit's origin; it carries a sheet per sepal, and grows
    /// from set to mature green while its calyx stays the size it was.
    #[test]
    fn a_fruit_is_closed_wound_outward_and_grows_inside_its_calyx() {
        let shapes: [fn(&mut FruitParams); 3] = [
            |_| {},
            |p| (p.aspect, p.ribbing) = (0.6, 0.2),
            |p| (p.aspect, p.jointed) = (1.6, false),
        ];
        for edit in shapes {
            let mut widths = Vec::new();
            for stage in [0.0, 0.5, 1.0, 1.5, 2.0, 5.0, 8.0] {
                let config = config(stage, edit);
                let mut rng = Rng::new(config.seed ^ CALYX_STREAM);
                let body = Body::new(&config, &mut rng);
                let sepals = calyx(&config, &body, &mut rng).unwrap();
                assert_eq!(sepals.len(), config.sepals as usize);

                for (what, mesh) in [("body", body.mesh()), ("stub", stub(&config))] {
                    let mut edges: HashMap<(i32, i32), i32> = HashMap::new();
                    let mut volume = 0.0;
                    let mut cursor = 0;
                    for count in &mesh.face_vertex_counts {
                        let face = &mesh.face_vertex_indices[cursor..cursor + *count as usize];
                        cursor += face.len();
                        for (k, a) in face.iter().enumerate() {
                            *edges.entry((*a, face[(k + 1) % face.len()])).or_default() += 1;
                        }
                        let p = |i: i32| Vec3::from(mesh.points[i as usize]);
                        for k in 1..face.len() - 1 {
                            volume += p(face[0]).dot(p(face[k]).cross(p(face[k + 1])));
                        }
                    }
                    assert!(
                        edges
                            .iter()
                            .all(|((a, b), n)| *n == 1 && edges.get(&(*b, *a)) == Some(&1)),
                        "{what} at {stage} is open or mis-wound"
                    );
                    assert!(volume > 0.0, "{what} at {stage} is inside out");
                }

                // The sepals spread from the shoulders, so a young fruit's,
                // the size the flower left them, may reach up past the stub.
                let hung = merge_meshes(&[body.mesh(), stub(&config)]);
                assert!(
                    hung.points.iter().all(|p| p[2] <= 1e-6),
                    "{stage}: above the pedicel's tip"
                );
                let triangles = fruit_mesh(&config, [1.0; 3])
                    .unwrap()
                    .indices()
                    .unwrap()
                    .len()
                    / 3;
                assert!(triangles < 600, "{triangles} triangles");
                let across = body
                    .mesh()
                    .points
                    .iter()
                    .fold(0.0f32, |m, p| m.max(p[0].hypot(p[1])));
                widths.push((stage, across));
            }
            for w in widths.windows(2) {
                let ((_, a), (stage, b)) = (w[0], w[1]);
                let grows = if stage <= MATURE_GREEN {
                    b > a
                } else {
                    (b - a).abs() < 1e-6
                };
                assert!(grows, "at {stage}: {a} then {b}");
            }
        }
    }

    /// Colour starts at the blossom end: at pink the bottom of a fruit is
    /// redder than its shoulders, at mature green neither has turned, and at
    /// the end of red both have.
    #[test]
    fn a_fruit_colours_from_its_blossom_end() {
        let ripe = srgb(Ripe::Red.hex());
        let redness = |stage: f32, h: f32| {
            let [r, g, _] = skin(stage, h, ripe);
            r - g
        };
        assert!(redness(5.5, 0.0) > redness(5.5, 0.9) + 0.05);
        assert!(redness(MATURE_GREEN, 0.0) < 0.0 && redness(MATURE_GREEN, 1.0) < 0.0);
        assert!(
            redness(RIPE, 0.0) > 0.05 && (redness(RIPE, 1.0) - redness(RIPE, 0.0)).abs() < 1e-4
        );
        assert_eq!(
            (ripeness(BREAKER), ripeness(4.0), ripeness(RIPE)),
            (0.0, 0.1, 1.0)
        );
        assert_eq!((stage_name(2.5), stage_name(RIPE)), ("mature_green", "red"));
    }

    /// Every fruit hangs its skin and a collider under its own prim, the
    /// collider as wide as the body and as tall, or a sphere on a fruit
    /// wider than tall, standing where the body hangs.
    #[test]
    fn a_fruit_hangs_its_skin_and_a_collider_round_its_body() {
        use misina_lab::scene::Collider;
        use misina_lab::testing::named_children;

        for aspect in [0.9, 1.6] {
            let mut params = small();
            params.fruit.aspect = aspect;
            let mut app = grown(params);
            let world = app.world_mut();
            let fruit: Vec<(Entity, FruitConfig)> = world
                .query::<(Entity, &FruitConfig)>()
                .iter(world)
                .map(|(entity, config)| (entity, *config))
                .collect();
            assert!(!fruit.is_empty());
            for (entity, config) in fruit {
                let children: BTreeMap<String, Entity> =
                    named_children(world, entity).into_iter().collect();
                assert_eq!(
                    children.keys().collect::<Vec<_>>(),
                    [COLLISION, SKIN_PRIM],
                    "{:?}",
                    world.entity(entity).get::<Name>()
                );
                let at = world.entity(children[COLLISION]);
                let (shape, placed) = (
                    at.get::<Collider>().unwrap().0,
                    at.get::<Transform>().unwrap(),
                );
                let (radius, half) = (config.across() / 2.0, config.across() / 2.0 * config.aspect);
                assert!((shape.radius - radius).abs() < 1e-6);
                assert!((shape.height - 2.0 * (half - radius).max(0.0)).abs() < 1e-6);
                // Centered on the body, which hangs below the stub.
                let middle = -STUB * config.diameter - half + DIP * radius;
                assert!((placed.translation.z - middle).abs() < 1e-6, "{placed:?}");
            }
        }
    }

    /// At one ripe truss a stem, the lowest truss has coloured on most stems,
    /// one just picked hanging its next still breaking, and every stem's third
    /// is still green; every truss is ripest at its base, and each fruit says
    /// so on its prim.
    #[test]
    fn a_stem_ripens_from_the_bottom_and_a_truss_from_its_base() {
        let mut app = grown(small());
        let world = app.world_mut();
        let mut trusses: BTreeMap<String, Vec<f32>> = BTreeMap::new();
        // In authoring order, which is base to tip within a truss.
        for fruit in organs::<FruitConfig>(world) {
            let (truss, _) = fruit.path.rsplit_once('/').unwrap();
            trusses
                .entry(truss.into())
                .or_default()
                .push(fruit.config.stage);
        }
        assert!(!trusses.is_empty());
        for (truss, stages) in &trusses {
            assert!(
                stages.windows(2).all(|w| w[0] >= w[1]),
                "{truss}: {stages:?}"
            );
            if truss.ends_with("Truss_02") {
                assert!(stages.iter().all(|s| *s < BREAKER), "{truss}: {stages:?}");
            }
        }
        let lowest: Vec<&Vec<f32>> = trusses
            .iter()
            .filter(|(truss, _)| truss.ends_with("Truss_00"))
            .map(|(_, stages)| stages)
            .collect();
        let coloured = lowest.iter().filter(|stages| stages[0] >= BREAKER).count();
        assert!(
            4 * coloured >= 3 * lowest.len(),
            "{coloured} of {} lowest trusses coloured",
            lowest.len()
        );

        let mut labelled = world.query::<(&FruitConfig, &UsdAttributes)>();
        for (config, attributes) in labelled.iter(world) {
            assert_eq!(
                attributes,
                &UsdAttributes::default()
                    .with("stage", stage_name(config.stage))
                    .with("ripeness", ripeness(config.stage))
            );
        }
    }
}

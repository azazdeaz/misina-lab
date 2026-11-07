//! Fruit — the tomatoes, each its own prim, hung from a pedicel's tip, and
//! the flowers they set from.
//!
//! A fruit is built round its own axis and hangs below its origin, where the
//! truss's pedicel ends in the calyx. The body hangs from the calyx by its
//! stem scar, which sits in a dip between the shoulders. The sepals, narrow
//! and pointed, lie on the shoulder for a short way and lift off it to
//! spread, each folded along its midrib and twisted a little:
//!
//! ```text
//!          ┃         the truss's pedicel, ending in the calyx at the origin
//!    ╲  ╭──┸──╮  ╱   sepals off the shoulders, spreading out and up
//!      (         )
//!       ╲       ╱    colour starts at the blossom end
//!        `─────'
//! ```
//!
//! Every fruit is an Xform carrying its stage, with the mesh as `Skin` under
//! it and a `Collision` capsule round the body: a sphere on a round fruit,
//! a capsule standing on an elongated one. The calyx stands clear of it.
//!
//! Before it sets, a fruit is a flower, nodding below the same origin: a
//! calyx of narrow sepals, a petal between each two of them, and the anther
//! cone in the middle. A bud is closed, its petals meeting over the cone;
//! an open flower's petals sweep back past level, `petal_reflex` off
//! pointing straight ahead, and close up again as it withers:
//!
//! ```text
//!         ┃          the pedicel
//!    ╭─╮  ┃  ╭─╮     petals swept back
//!     ╲ ╲─┸─╱ ╱      sepals between them
//!        ▕ ▏
//!         ▼          the anther cone round the style
//! ```
//!
//! A truss shows at the head as a cluster of buds. Its first flower opens
//! [`OPENS`] later, about where growers measure the top flowering truss
//! below the head, the rest open toward its tip over [`FLOWERING`], and
//! each stays open for [`ANTHESIS`] before it sets.
//!
//! A fruit's `stage` runs from bud and flower through fruit set, expanding
//! and mature green, over which it grows to full size, to the USDA colour
//! classes breaker, turning, pink, light red and red, which are defined by
//! how much of the surface has coloured: up to 10%, 30%, 60%, 90% and the
//! rest. The colour climbs from the blossom end, so the shoulders turn last.
//! The calyx grows on from the flower's as the fruit grows, so a young
//! fruit's sepals are long next to it.
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
//! growing, and the youngest trusses are in flower.
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

use super::{Grow, smooth};

/// The mesh-library prefix the fruit register under, and the prim under a
/// fruit that draws it.
pub const PART: &str = "Fruit";
pub const SKIN_PRIM: &str = "Skin";

/// Salt splitting a fruit's mesh draws off the stream its config draws from.
const CALYX_STREAM: u64 = 0xC2B2_AE3D_27D4_EB4F;

/// The stages, as `stage` counts them up from [`BUD`]: a flower at -0.5 is
/// half way through its anthesis, a fruit at 4.5 half way through turning,
/// and one at 8 is fully red.
pub const STAGES: [&str; 10] = [
    "bud",
    "flower",
    "set",
    "expanding",
    "mature_green",
    "breaker",
    "turning",
    "pink",
    "light_red",
    "red",
];
const BUD: f32 = -2.0;
const FLOWER: f32 = -1.0;
const SET: f32 = 0.0;
const MATURE_GREEN: f32 = 2.0;
const BREAKER: f32 = 3.0;
const RIPE: f32 = 8.0;

/// Weeks from a truss showing at the head to its first flower opening: 15 cm
/// of stem below the head at a week's growth of 0.34 m, where growers find
/// the top flowering truss (est.). Then the weeks its flowers take to open
/// from its base to its tip (est.), how long each stays open before it sets,
/// about the two days a flower lasts and a day to wither, and the weeks from
/// set to full size, at mature green (est.).
const OPENS: f32 = 0.45;
const FLOWERING: f32 = 1.0;
const ANTHESIS: f32 = 0.4;
const GROWING: f32 = 5.0;

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

/// A sepal's length, as a fraction of the full-grown diameter; its width as
/// a fraction of its length, and where along it it is widest.
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

/// The calyx.
const SEPAL_COLOR: u32 = 0x355E1E;

/// A tomato's waxy skin.
const SKIN: Response = Response {
    roughness: 0.3,
    reflectance: 0.55,
};

/// A flower's petal: its width as a fraction of its length, where along it
/// it is widest, and how steeply its edges cup toward its inner face, as a
/// slope.
const PETAL_WIDTH: f32 = 0.3;
const PETAL_PEAK: f64 = 0.3;
const CUP: f32 = 0.3;

/// A flower's sepal's length, in petal lengths. The calyx grows on from it
/// to [`SEPAL`] as the fruit grows.
const FLOWER_SEPAL: f32 = 0.8;

/// The ring round the axis the petals and sepals leave the flower from, in
/// petal lengths; the sepals leave it a little higher, behind the petals.
const RING: f32 = 0.16;
const SEPAL_RISE: f32 = 0.04;

/// The anther cone round the style: its length, and its radius where it
/// leaves the flower, in petal lengths. Grooved between its five or six
/// anthers by this share of its radius.
const CONE: (f32, f32) = (0.75, 0.12);
const GROOVE: f32 = 0.15;

/// How large a bud is when its truss shows at the head, as a fraction of the
/// flower it opens into.
const BUD_SIZE: f32 = 0.45;

/// The share of a flower's anthesis it takes to open, and the share at its
/// end over which it withers: its petals fold back in, shrink and fade.
const OPENING: f32 = 0.3;
const WITHERING: f32 = 0.35;

/// Where a petal bends back, as shares of its length: straight out of the
/// short tube the petals are joined in, then round to `petal_reflex` over
/// this stretch.
const REFLEX: (f32, f32) = (0.12, 0.45);

/// How far a petal or a sepal turns toward the axis over its length in a
/// bud, in radians, from heading this far out of it where it leaves; and how
/// far an open flower's sepals spread off its axis, and curl on out, in
/// radians.
const CLOSED: (f32, f32) = (0.25, 0.8);
const SPREAD: (f32, f32) = (0.9, 0.4);

/// How far a petal's or a sepal's length varies and it strays from even
/// spacing, as fractions, and how far it twists about its midrib, in
/// radians, at most either way.
const PETAL_JITTER: f64 = 0.1;
const PETAL_STRAY: f64 = 0.15;
const PETAL_TWIST: f64 = 0.4;

/// Control points along a petal's or a flower's sepal's midrib.
const MIDRIB_POINTS: usize = 12;

/// The petals, opening from a bud's green-yellow and withering to tan, and
/// the anther cone.
const PETAL_COLOR: u32 = 0xF5D52A;
const BUD_COLOR: u32 = 0xB5BE3C;
const WITHERED: u32 = 0xB49A50;
const ANTHER_COLOR: u32 = 0xD9AE1C;

/// The petals' matte, thin sheets.
const PETALS: Response = Response {
    roughness: 0.6,
    reflectance: 0.35,
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
    STAGES[((stage - BUD).max(0.0) as usize).min(STAGES.len() - 1)]
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
    /// Weeks since its truss showed at the head, a truss a week.
    pub age: f32,
}

/// One fruit's shape and stage, as the truss that holds it specified.
#[derive(Component, Clone, Copy, Debug, PartialEq)]
pub struct FruitConfig {
    /// Across when full grown, in meters. The size it is now follows from its
    /// stage, and the calyx's from this.
    pub diameter: f32,
    /// How far it has come, -2 a bud just showing, -1 a flower opening, 0
    /// just set and 8 fully red; see [`STAGES`].
    pub stage: f32,
    /// Height over width.
    pub aspect: f32,
    /// Depth of the grooves between its lobes, as a fraction of its radius.
    pub ribbing: f32,
    pub lobes: u32,
    /// Sepals round its calyx, and petals round its flower.
    pub sepals: u32,
    /// Its flower's petal length, in meters, and how far back they sweep
    /// once open, in radians off pointing straight ahead.
    pub petal: f32,
    pub reflex: f32,
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

        // Weeks since its own flower opened: a truss opens its flowers from
        // its base to its tip.
        let opened = hung.age - OPENS - FLOWERING * hung.along;
        let ripening = params.ripe_trusses - hung.rank - params.spread * hung.along;
        let stage = if opened < 0.0 {
            // A bud from when its truss shows, at the tip latest.
            (FLOWER + opened / (OPENS + FLOWERING)).max(BUD)
        } else if opened < ANTHESIS {
            FLOWER + opened / ANTHESIS
        } else if ripening >= 0.0 {
            BREAKER + (RIPE - BREAKER) * ripening.min(1.0)
        } else {
            // Growing toward mature green, then through it in the truss above
            // the front.
            let growth = ((opened - ANTHESIS) / GROWING).min(1.0);
            (MATURE_GREEN * growth + (1.0 + ripening).max(0.0)).min(BREAKER)
        };
        Self {
            diameter,
            stage,
            aspect: aspect.max(0.2),
            ribbing: params.ribbing.clamp(0.0, 0.5),
            lobes,
            sepals,
            petal: params.petal_length.max(0.002),
            reflex: params.petal_reflex.clamp(0.0, 180.0).to_radians(),
            seed,
            hung,
        }
    }

    /// Whether it is still a flower, or a bud.
    pub fn in_flower(&self) -> bool {
        self.stage < SET
    }

    /// How far it has grown from set to mature green, eased, 0 to 1.
    fn grown(&self) -> f32 {
        smooth(self.stage / MATURE_GREEN)
    }

    /// Across now, in meters: it grows from set to mature green, and a flower
    /// is a fruit just set across.
    pub fn across(&self) -> f32 {
        self.diameter * (SET_SIZE + (1.0 - SET_SIZE) * self.grown())
    }

    /// Its mass now, in kilograms: a spheroid `across` wide and `aspect` as
    /// tall as that.
    pub fn mass(&self) -> f32 {
        DENSITY * PI / 6.0 * self.across().powi(3) * self.aspect
    }
}

/// Two fruit share a mesh when they are near in size, in shape and in colour,
/// and two flowers when they are near in how far they have opened.
pub struct FruitMetric;

impl Metric<FruitConfig> for FruitMetric {
    fn distance(&self, a: &FruitConfig, b: &FruitConfig) -> f32 {
        // A flower and a fruit are different shapes, not near sizes of one.
        if a.in_flower() != b.in_flower() {
            return 1.0;
        }
        let norm = |terms: &[f32]| terms.iter().map(|d| d * d).sum::<f32>().sqrt();
        if a.in_flower() {
            // A stage of a flower's opening swings its petals' tips by about
            // twice their length.
            let petal = a.petal.max(b.petal);
            return norm(&[
                (a.stage - b.stage) * 2.0 * petal,
                a.petal - b.petal,
                (a.reflex - b.reflex) * petal,
            ]);
        }
        let (across_a, across_b) = (a.across(), b.across());
        let radius = across_a.max(across_b) / 2.0;
        norm(&[
            across_a - across_b,
            // The calyx grows toward a size set by the full-grown fruit.
            (a.diameter - b.diameter) * SEPAL,
            // Each moves the surface by its difference times the radius.
            (a.aspect - b.aspect) * radius,
            (a.ribbing - b.ribbing) * radius,
            (a.stage - b.stage) * STAGE_WEIGHT,
        ])
    }
}

/// The fruit and the flowers they set from: their shape, how ripe the crop
/// is, and what colour ripe is.
///
/// Every fruit has a stage: bud and flower before it sets, set, expanding
/// and mature green while it grows, then breaker, turning, pink, light red
/// and red as it colours from its blossom end up. A truss shows at the head
/// in bud, opens its flowers from its base to its tip from about 15 cm below
/// the head, sets them over the week after, and they reach full size about
/// five trusses further down. A stem colours from its lowest truss up and a
/// truss from its base to its tip, so in full production the lowest
/// `ripe_trusses` trusses on a stem are colouring and the ones above them
/// still green; a young crop's trusses are all green until the first are
/// old enough. A truss is due once its tip fruit is turning. Every stem runs
/// a little ahead of or behind the rest.
///
/// Every fruit draws its own lobe count, its five or six sepals and how each
/// curls, and a slight lean, within the ranges the research notes give; its
/// flower has a petal between each two sepals.
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
    /// Length of a flower's petals, in meters: an open tomato flower is under
    /// 25 mm across, and a cherry tomato's smaller.
    #[reflect(@Slider { min: 0.004, max: 0.02, step: 0.0005 })]
    pub petal_length: f32,
    /// How far an open flower's petals sweep back, in degrees off pointing
    /// straight ahead: 90 opens it flat, and a tomato's sweep back past that
    /// toward its pedicel.
    #[reflect(@Slider { min: 20.0, max: 170.0, step: 1.0 })]
    pub petal_reflex: f32,
    /// How many distinct fruit and flower meshes the scene may hold, by size,
    /// shape and stage. A budget, not a count: the fruit are clustered and
    /// this is how many representatives the clustering may keep, though a
    /// crop in flower keeps one for its flowers and one for its fruit at
    /// least.
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
            petal_length: 0.011,
            petal_reflex: 125.0,
            variations: 32,
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

/// The body's surface, solved from a config: its stem scar at the origin.
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
        Vec3::new(r * phi.cos(), r * phi.sin(), z - self.half + self.dip - dip)
            + (self.lean * self.radius * down).extend(0.0)
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
    let flower = FLOWER_SEPAL * config.petal;
    let length = flower + (SEPAL * config.diameter - flower) * config.grown();
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

/// A midrib `length` long from `from`, leaving down the flower's axis, -Z,
/// turned toward `out` by `angle` of how far along it is, 0 to 1.
fn midrib(from: Vec3, out: Vec3, length: f32, angle: impl Fn(f32) -> f32) -> Vec<Vec3> {
    let step = length / MIDRIB_POINTS as f32;
    let mut points = vec![from];
    for k in 0..MIDRIB_POINTS {
        let a = angle((k as f32 + 0.5) / MIDRIB_POINTS as f32);
        points.push(points[k] + (out * a.sin() - Vec3::Z * a.cos()) * step);
    }
    points
}

/// A [`leaf`] outline `long` by `wide`, widest at `peak`, laid along a
/// midrib bent in the plane across `side`: its edges cupped `cup` toward the
/// face that looks back at the axis, and the whole twisted `twist` about the
/// midrib over its length.
fn sheet(
    midrib: Vec<Vec3>,
    side: Vec3,
    (long, wide, peak): (f32, f32, f64),
    cup: f32,
    twist: f32,
) -> Result<MeshData> {
    let path = Path::new(midrib, side);
    let outline = leaf(long as f64, wide as f64, peak);
    let mut sheet = outline_mesh(&outline, outline.area())?;
    for point in &mut sheet.points {
        let (x, y) = (point[0], point[1]);
        let (at, along, normal) = path.at(x);
        let across = side * y - normal * (cup * y.abs());
        *point = (at + Quat::from_axis_angle(along, twist * x / long) * across).to_array();
    }
    Ok(sheet)
}

/// The anther cone round the style, `size` a full petal's length: from inside
/// the ring the petals leave down to its point, grooved between its
/// `anthers`.
fn anther_cone(anthers: u32, size: f32) -> MeshData {
    let sectors = 2 * anthers;
    let (length, radius) = CONE;
    let ring = |z: f32, r: f32| -> Vec<Vec3> {
        (0..sectors)
            .map(|i| {
                let phi = TAU * i as f32 / sectors as f32;
                let r = r * if i % 2 == 1 { 1.0 - GROOVE } else { 1.0 };
                Vec3::new(r * phi.cos(), r * phi.sin(), z) * size
            })
            .collect()
    };
    lathe(
        Vec3::ZERO,
        &[
            ring(-0.02, radius),
            ring(-0.3 * length, 0.9 * radius),
            ring(-0.7 * length, 0.5 * radius),
        ],
        Vec3::new(0.0, 0.0, -length * size),
    )
}

/// One flower in its own frame, nodding below the origin where its pedicel
/// ends, with a colour per point: the calyx, a petal between each two sepals,
/// and the anther cone. A bud grows to the flower's size, closed, its petals
/// folded in over the cone and its sepals round them; it opens, its petals
/// sweeping back to `reflex` as its sepals spread, and withers, the petals
/// folding back in and fading.
///
/// Draws, after where the first sepal sits, per sepal and per petal round the
/// flower: its stray from even spacing, its length and its twist.
fn flower(config: &FruitConfig, rng: &mut Rng) -> Result<(Vec<MeshData>, Vec<[f32; 3]>)> {
    // How far through its bud, and through its anthesis, 0 to 1.
    let bud = (config.stage - BUD).clamp(0.0, 1.0);
    let anthesis = (config.stage - FLOWER).clamp(0.0, 1.0);
    let wither = smooth((anthesis - 1.0 + WITHERING) / WITHERING);
    let opened = smooth(anthesis / OPENING);
    let open = opened * (1.0 - wither);
    // The sepals part as the bud swells, and stay spread once open.
    let parted = 0.3 * bud + 0.7 * opened;
    let size = (BUD_SIZE + (1.0 - BUD_SIZE) * bud) * config.petal;
    let shrunk = 1.0 - 0.3 * wither;

    let closed = |s: f32| CLOSED.0 - CLOSED.1 * s;
    let swept = |s: f32| {
        CLOSED.0 + (config.reflex - CLOSED.0) * smooth((s - REFLEX.0) / (REFLEX.1 - REFLEX.0))
    };
    let spread = |s: f32| SPREAD.0 + SPREAD.1 * s;
    let petal_color = mix(
        mix(srgb(BUD_COLOR), srgb(PETAL_COLOR), bud),
        srgb(WITHERED),
        wither,
    );

    let (mut pieces, mut colors) = (Vec::new(), Vec::new());
    let mut paint = |piece: MeshData, color: [f32; 3]| {
        colors.extend(std::iter::repeat_n(color, piece.points.len()));
        pieces.push(piece);
    };
    let even = TAU / config.sepals as f32;
    let first = rng.unit() as f32 * TAU;
    for i in 0..2 * config.sepals {
        let stray = rng.range(-PETAL_STRAY, PETAL_STRAY) as f32;
        let jitter = rng.range(1.0 - PETAL_JITTER, 1.0 + PETAL_JITTER) as f32;
        let twist = rng.range(-PETAL_TWIST, PETAL_TWIST) as f32;
        let bearing = first + (i as f32 / 2.0 + stray) * even;
        let out = Vec3::new(bearing.cos(), bearing.sin(), 0.0);
        let side = Vec3::new(-bearing.sin(), bearing.cos(), 0.0);
        let from = out * RING * size;
        if i % 2 == 1 {
            let long = size * jitter * shrunk;
            let midrib = midrib(from, out, long, |s| {
                closed(s) + (swept(s) - closed(s)) * open
            });
            // Folded at its edges while closed, as a bud's petals are.
            let cup = CUP + 0.6 * (1.0 - open);
            let shape = (long, long * PETAL_WIDTH * shrunk, PETAL_PEAK);
            paint(sheet(midrib, side, shape, cup, twist)?, petal_color);
        } else {
            let long = FLOWER_SEPAL * size * jitter;
            // Round the petals while closed.
            let hug = |s: f32| closed(s) + 0.15;
            let from = from + Vec3::Z * SEPAL_RISE * size;
            let midrib = midrib(from, out, long, |s| hug(s) + (spread(s) - hug(s)) * parted);
            let shape = (long, long * SEPAL_WIDTH, SEPAL_PEAK);
            paint(sheet(midrib, side, shape, KEEL, twist)?, srgb(SEPAL_COLOR));
        }
    }
    let anthers = mix(srgb(ANTHER_COLOR), srgb(WITHERED), wither);
    paint(anther_cone(config.sepals, size), anthers);
    Ok((pieces, colors))
}

/// One fruit in its own frame, hanging below the origin, with a colour per
/// point: its body and its calyx.
fn fruit(
    config: &FruitConfig,
    ripe: [f32; 3],
    rng: &mut Rng,
) -> Result<(Vec<MeshData>, Vec<[f32; 3]>)> {
    let body = Body::new(config, rng);
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

    let sepals = calyx(config, &body, rng)?;
    for sepal in &sepals {
        colors.extend(std::iter::repeat_n(srgb(SEPAL_COLOR), sepal.points.len()));
    }
    let mut pieces = vec![skin_mesh];
    pieces.extend(sepals);
    Ok((pieces, colors))
}

/// One fruit, or the flower it sets from, in its own frame below the origin,
/// with a colour per point.
fn fruit_mesh(config: &FruitConfig, ripe: [f32; 3]) -> Result<Mesh> {
    let mut rng = Rng::new(config.seed ^ CALYX_STREAM);
    let (pieces, colors) = if config.in_flower() {
        flower(config, &mut rng)?
    } else {
        fruit(config, ripe, &mut rng)?
    };
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
    // A crop holding both flowers and fruit keeps a mesh for each: they are
    // further apart than any two of a kind, so the second pick is the other.
    let kinds = [true, false].map(|flower| configs.iter().any(|c| c.in_flower() == flower));
    let book = farthest_first(
        &configs,
        (params.variations as usize).max(kinds.iter().filter(|any| **any).count()),
        0.0,
        &FruitMetric,
    );
    let ripe = srgb(params.ripe().hex());
    let meshes = par_map(&book.representatives, |_, config| fruit_mesh(config, ripe));
    let mut geometry = Vec::with_capacity(meshes.len());
    for (index, (config, mesh)) in book.representatives.iter().zip(meshes).enumerate() {
        let response = if config.in_flower() { PETALS } else { SKIN };
        // The points carry the colour, and the sepals and petals are sheets
        // seen from both sides.
        let surface = Surface {
            double_sided: true,
            ..response.surface([1.0; 3])
        };
        geometry.push(library.part(PART, index, mesh?, surface));
    }

    for ((_, entity, config), drew) in hung.iter().zip(&book.assignment) {
        // Round the body: its radius across, and its height along the axis,
        // which a capsule this wide spans only on an elongated fruit.
        let radius = config.across() / 2.0;
        let half = radius * config.aspect;
        let middle = DIP * radius - half;
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
                    age: 8.0,
                },
            )
        }
    }

    /// The body is closed and wound outward: every edge is walked once each
    /// way, and the volume it encloses is positive. It hangs from its stem
    /// scar at the fruit's origin, its shoulders rising round it no higher
    /// than the dip is deep; it carries a sheet per sepal, and grows from set
    /// to mature green while its calyx stays the size it was.
    #[test]
    fn a_fruit_is_closed_wound_outward_and_grows_inside_its_calyx() {
        let shapes: [fn(&mut FruitParams); 3] = [
            |_| {},
            |p| (p.aspect, p.ribbing) = (0.6, 0.2),
            |p| p.aspect = 1.6,
        ];
        for edit in shapes {
            let mut widths = Vec::new();
            for stage in [0.0, 0.5, 1.0, 1.5, 2.0, 5.0, 8.0] {
                let config = config(stage, edit);
                let mut rng = Rng::new(config.seed ^ CALYX_STREAM);
                let body = Body::new(&config, &mut rng);
                let sepals = calyx(&config, &body, &mut rng).unwrap();
                assert_eq!(sepals.len(), config.sepals as usize);

                let mesh = body.mesh();
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
                    "body at {stage} is open or mis-wound"
                );
                assert!(volume > 0.0, "body at {stage} is inside out");

                assert!(body.at(0.0, 0.0).length() < 1e-6, "{stage}: off its calyx");
                assert!(
                    mesh.points.iter().all(|p| p[2] <= body.dip + 1e-6),
                    "{stage}: above its calyx"
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

    /// A bud is closed, its petals within a little of the axis, and swells;
    /// open, its petals reach out and sweep back above where they leave; and
    /// they fold back in as it withers. Its petals and sepals alternate round
    /// it, and every stage of it stays light.
    #[test]
    fn a_bud_opens_its_petals_back_and_withers_before_it_sets() {
        let petal = FruitParams::default().petal_length;
        // How far its petals reach off its axis, and how high they rise.
        let reach = |stage: f32| {
            let config = config(stage, |_| {});
            let (pieces, _) = flower(&config, &mut Rng::new(config.seed ^ CALYX_STREAM)).unwrap();
            assert_eq!(pieces.len(), 2 * config.sepals as usize + 1);
            let triangles = fruit_mesh(&config, [1.0; 3])
                .unwrap()
                .indices()
                .unwrap()
                .len()
                / 3;
            assert!(triangles < 800, "{stage}: {triangles} triangles");
            let petals = pieces
                .iter()
                .skip(1)
                .step_by(2)
                .take(config.sepals as usize);
            petals
                .flat_map(|piece| &piece.points)
                .fold((0.0f32, f32::MIN), |(r, z), p| {
                    (r.max(p[0].hypot(p[1])), z.max(p[2]))
                })
        };
        let (bud, swelling, open, withered) = (reach(BUD), reach(-1.1), reach(-0.6), reach(-0.01));
        assert!(
            bud.0 < swelling.0 && swelling.0 < 0.3 * petal,
            "{bud:?} {swelling:?}"
        );
        assert!(open.0 > 0.6 * petal && open.1 > 0.1 * petal, "{open:?}");
        assert!(
            withered.0 < 0.8 * open.0 && withered.1 < 0.0,
            "{withered:?}"
        );
        assert_eq!([-1.5, -0.5, 0.0].map(stage_name), ["bud", "flower", "set"]);
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
                // Centered on the body, which hangs below the calyx.
                let middle = DIP * radius - half;
                assert!((placed.translation.z - middle).abs() < 1e-6, "{placed:?}");
            }
        }
    }

    /// A flower never draws a fruit's mesh, nor a fruit a flower's, even on a
    /// budget of one.
    #[test]
    fn flowers_and_fruit_never_share_a_mesh() {
        use misina_lab::scene::UsdReference;
        use misina_lab::testing::named_children;

        let mut params = small();
        params.fruit.variations = 1;
        let mut app = grown(params);
        let world = app.world_mut();
        let fruit: Vec<(Entity, bool)> = world
            .query::<(Entity, &FruitConfig)>()
            .iter(world)
            .map(|(entity, config)| (entity, config.in_flower()))
            .collect();
        let mut drawn: [std::collections::BTreeSet<String>; 2] = Default::default();
        for (entity, flower) in fruit {
            let (_, skin) = named_children(world, entity)
                .into_iter()
                .find(|(name, _)| name == SKIN_PRIM)
                .unwrap();
            let part = world.entity(skin).get::<UsdReference>().unwrap();
            drawn[flower as usize].insert(part.0.clone());
        }
        assert!(
            drawn.iter().all(|parts| parts.len() == 1) && drawn[0] != drawn[1],
            "{drawn:?}"
        );
    }

    /// At one ripe truss a stem, the lowest truss has coloured on most stems,
    /// one just picked hanging its next still breaking, and every stem's third
    /// is still green; every truss is ripest at its base, and each fruit says
    /// so on its prim. At the head, every stem's youngest truss is still in bud
    /// at its tip, and the crop is in flower.
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
        let mut youngest = BTreeMap::new();
        for (truss, stages) in &trusses {
            youngest.insert(truss.rsplit_once('/').unwrap().0, stages.last().unwrap());
        }
        assert!(
            youngest.values().all(|tip| stage_name(**tip) == "bud"),
            "{youngest:?}"
        );
        assert!(
            trusses
                .values()
                .flatten()
                .any(|s| stage_name(*s) == "flower")
        );

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

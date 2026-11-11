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
//! The body has a lobe per locule round its axis, each its own width and
//! fullness, and a groove over each wall between two, deepest round the
//! shoulders and fading toward the blossom end. It is meshed in rings round
//! its axis, as finely as `detail` asks: the rings go where its meridians
//! curve, and as many sides go round it as its sharpest ring needs, so it
//! keeps within the same distance of its shape at any size. Each sepal lies
//! along one of the body's own meridians, its edges raised over a groove.
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
use misina_lab::geometry::packing::nearest;
use misina_lab::geometry::par_map;
use misina_lab::geometry::shapes::{leaf, simplify_with};
use misina_lab::palette::{Response, mix, srgb};
use misina_lab::params::{Choices, Label, Slider};
use misina_lab::quantize::{Metric, farthest_first};
use misina_lab::rng::Rng;
use misina_lab::scene::{
    COLLISION, Library, Order, Surface, UsdAttributes, capsule, configs_changed,
};

use super::leaf::TOLERANCE;
use super::stem::MIN_DETAIL;
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

/// The meridian is a superellipse, `|sin|` and `|cos|` raised to this: under
/// 1 gives the broad shoulders and broad blossom end of a tomato.
const SQUARE: f32 = 0.9;

/// The stem scar, and the rim of the dip round it, as polar angles from the
/// stem end; and the dip's depth at the scar, as a fraction of the radius.
const SCAR: f32 = 0.1;
const RIM: f32 = 0.38;
const DIP: f32 = 0.10;

/// How finely the meridians are sampled for the rings to be picked from, as
/// a step of polar angle; how far apart the rings are that the sides round
/// the body are sized on, and how many points round each.
const STEP: f32 = 0.01;
const PROBE: f32 = 0.05;
const SAMPLES: usize = 1024;

/// At an irregularity of 1: how far the blossom end leans off the axis, as a
/// fraction of the radius, at most; how far a lobe's width and the depth of
/// the groove at its edge vary, as fractions; and how far a lobe falls short
/// of the fullest, as a fraction of the radius, at most.
const LEAN: f64 = 0.12;
const WIDTH_SPREAD: f64 = 0.25;
const DEPTH_SPREAD: f64 = 0.4;
const FILL: f64 = 0.03;

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

/// The ball a flower about fills, as its radius and how far below the
/// origin its middle is, in petal lengths: a closed bud's, about as long as
/// its petals, and an open flower's, out to the tips of its swept-back
/// petals (est.).
const BUD_BALL: (f32, f32) = (0.5, 0.5);
const FLOWER_BALL: (f32, f32) = (0.85, 0.25);

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

impl Hung {
    /// Weeks since its flower opened, under 0 while it is a bud: a truss
    /// opens its flowers from its base to its tip.
    pub fn opened(&self) -> f32 {
        self.age - OPENS - FLOWERING * self.along
    }
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
    /// Its lobes, one per locule, and how irregular they are, 0 for a
    /// regular fruit.
    pub lobes: u32,
    pub irregularity: f32,
    /// Depth of the grooves between its lobes round its shoulders, as a
    /// fraction of its radius, and at its girth, as a share of that.
    pub ribbing: f32,
    pub rib_reach: f32,
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
        // Up to a locule either side of the params', the nearest likeliest.
        let locules = params.locules.clamp(2.0, 12.0) as f64;
        let lobes = (locules + rng.range(-1.0, 1.0)).round().max(2.0) as u32;
        let sepals = if rng.unit() < 0.5 { 5 } else { 6 };

        let opened = hung.opened();
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
            lobes,
            irregularity: params.irregularity.clamp(0.0, 2.0),
            ribbing: params.ribbing.clamp(0.0, 0.5),
            rib_reach: params.rib_reach.clamp(0.05, 0.5),
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

    /// The ball it about fills, as its radius, how far below the origin,
    /// where its pedicel ends, the ball's middle is, and how far either way
    /// of that along its axis it is drawn out into a capsule: a fruit's as
    /// wide as it is, drawn out to its length if it is elongated, and a
    /// flower's from [`BUD_BALL`] to [`FLOWER_BALL`] as far as it has opened,
    /// and back as it withers, never drawn out.
    pub fn ball(&self) -> (f32, f32, f32) {
        if !self.in_flower() {
            let radius = self.across() / 2.0;
            let drawn = (self.aspect - 1.0).max(0.0) * radius;
            return (radius, (self.aspect - DIP) * radius, drawn);
        }
        let (_, opened, wither, size) = self.bloom();
        let open = opened * (1.0 - wither);
        let ball = |bud: f32, flower: f32| (bud + (flower - bud) * open) * size;
        (
            ball(BUD_BALL.0, FLOWER_BALL.0),
            ball(BUD_BALL.1, FLOWER_BALL.1),
            0.0,
        )
    }

    /// How far a flower is through its bud, through opening and through
    /// withering, each 0 to 1, and how long its petals have grown, in meters.
    fn bloom(&self) -> (f32, f32, f32, f32) {
        let bud = (self.stage - BUD).clamp(0.0, 1.0);
        let anthesis = (self.stage - FLOWER).clamp(0.0, 1.0);
        let wither = smooth((anthesis - 1.0 + WITHERING) / WITHERING);
        let opened = smooth(anthesis / OPENING);
        let size = (BUD_SIZE + (1.0 - BUD_SIZE) * bud) * self.petal;
        (bud, opened, wither, size)
    }
}

/// Two fruit share a mesh when they are near in size, in shape and in colour,
/// and two flowers when they are near in how far they have opened. The lobe
/// and sepal counts each fruit draws are left out: the fruit sharing a mesh
/// show its own, drawn from the same params as theirs, and a term for them
/// would spend the variations on counts, not on sizes and colours.
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
/// A fruit has a lobe per locule, the chambers its seeds grow in, and a
/// groove over each wall between two: deepest round its shoulders, where it
/// pleats out from the calyx, and fading toward its blossom end. Every fruit
/// draws its own lobe count, how wide and how full each lobe is and how deep
/// each groove, its lean, and its five or six sepals and how each curls,
/// within the ranges the research notes give; its flower has a petal between
/// each two sepals.
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
    /// How many locules a fruit has, and so lobes: 2 in a cherry or a plum
    /// tomato, 3 to 6 in a round or a truss tomato, and more in a beefsteak.
    /// Each fruit draws up to one either side.
    #[reflect(@Slider { min: 2.0, max: 12.0, step: 0.5 })]
    pub locules: f32,
    /// Depth of the grooves between a fruit's lobes round its shoulders,
    /// where they are deepest, as a fraction of its radius: 0 is smooth, and
    /// a beefsteak's are about 0.2.
    #[reflect(@Slider { min: 0.0, max: 0.3, step: 0.01 })]
    pub ribbing: f32,
    /// How far down a fruit its grooves reach: their depth at its girth, as a
    /// share of their depth round its shoulders. Below that they fade out
    /// before the blossom end.
    #[reflect(@Slider { min: 0.05, max: 0.5, step: 0.01 })]
    pub rib_reach: f32,
    /// How irregular a fruit is: how far its lobes differ in width and in
    /// fullness and its grooves in depth, and how far its blossom end leans.
    /// 0 is a regular fruit; at 1 about one fruit in five leans visibly, and
    /// 2 is lopsided.
    #[reflect(@Slider { min: 0.0, max: 2.0, step: 0.05 }, @Label("Fruit irregularity"))]
    pub irregularity: f32,
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
    /// least. A fruit is drawn at its representative's size, so on a small
    /// budget a truss's fruit, packed to touch at their own sizes, are drawn
    /// into each other.
    #[reflect(@Slider { min: 1.0, max: 64.0, step: 1.0 }, @Label("Fruit variations"))]
    pub variations: u32,
    /// How finely a fruit's skin is meshed, as a multiplier: at 1 its rings
    /// and meridians keep within 0.6 mm of its shape, about a pixel to a
    /// camera half a meter away, as a leaf's margin does at its own detail of
    /// 1, and its faces within twice that; at 3 its rings and meridians keep
    /// within 0.2 mm and its faces within 0.4 mm, which leaves its outline
    /// round from close up. A full-grown 60 mm fruit
    /// comes to about 1250 triangles with its calyx at 3, about 400 at 1 and
    /// 3700 at 8; each fruit is meshed for its own size, so a young one
    /// costs less.
    #[reflect(@Slider { min: 0.1, max: 8.0, step: 0.05 }, @Label("Fruit detail"))]
    pub detail: f32,
}

impl Default for FruitParams {
    fn default() -> Self {
        Self {
            ripe_trusses: 1.0,
            spread: 0.2,
            color: Ripe::default().name().into(),
            aspect: 0.9,
            locules: 4.0,
            ribbing: 0.08,
            rib_reach: 0.35,
            irregularity: 1.0,
            petal_length: 0.011,
            petal_reflex: 125.0,
            variations: 32,
            detail: 3.0,
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
                    // `color`, `variations` and `detail` reach no config.
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
    /// The grooves' depth falls off from the shoulders as `(1 + cos theta) / 2`
    /// raised to this.
    reach: f32,
    /// Where the blossom end leans to, as a fraction of the radius.
    lean: Vec2,
    /// The azimuths of the septa, the walls between the locules, from 0 round
    /// to `TAU`, the first again at the end; the depth of the groove over
    /// each, as a share of `ribbing`; and how far short of the fullest lobe
    /// each falls, as a fraction of the radius, 0 or under.
    septa: Vec<f32>,
    depth: Vec<f32>,
    fill: Vec<f32>,
    /// Polar angles of the rings, from the stem end, and the azimuths of the
    /// points round each.
    rings: Vec<f32>,
    sectors: Vec<f32>,
}

impl Body {
    /// Meshed for its rings and meridians to keep within `tolerance` of its
    /// surface, and so its faces within about twice that: as many sides
    /// round it as its sharpest ring needs, and its rings where its meridians
    /// need them.
    ///
    /// Draws, first off the mesh's stream: the lean's size and bearing, then
    /// per lobe its width, the depth of the groove before it and its fill.
    fn new(config: &FruitConfig, tolerance: f32, rng: &mut Rng) -> Self {
        let radius = config.across() / 2.0;
        let irregular = config.irregularity as f64;
        // Most fruit hardly lean, and a few lean far.
        let lean = (LEAN * irregular * rng.unit().powi(3)) as f32;
        let lean = Vec2::from_angle(rng.unit() as f32 * TAU) * lean;
        let (mut width, mut depth, mut fill) = (Vec::new(), Vec::new(), Vec::new());
        for _ in 0..config.lobes.max(2) {
            width.push(1.0 + rng.range(-1.0, 1.0) * WIDTH_SPREAD * irregular);
            depth.push((1.0 + rng.range(-1.0, 1.0) * DEPTH_SPREAD * irregular) as f32);
            fill.push(rng.range(0.0, FILL * irregular) as f32);
        }
        let fullest = fill.iter().copied().fold(0.0, f32::max);
        let total: f64 = width.iter().sum();
        let septa = std::iter::once(0.0)
            .chain(width.iter().scan(0.0, |edge, w| {
                *edge += w;
                Some((TAU as f64 * *edge / total) as f32)
            }))
            .collect();
        let mut body = Self {
            radius,
            half: radius * config.aspect,
            dip: DIP * radius,
            ribbing: config.ribbing,
            // `rib_reach` of the shoulders' depth at the girth.
            reach: -config.rib_reach.log2(),
            lean,
            septa,
            depth,
            fill: fill.iter().map(|f| f - fullest).collect(),
            rings: Vec::new(),
            sectors: Vec::new(),
        };
        body.sectors = body.sectors_within(tolerance);
        body.rings = body.rings_within(tolerance);
        body
    }

    /// The meridian `theta` from the stem end, before the lobes and the
    /// lean: how far off the axis it is and how high; and how far down the
    /// body, 0 at the stem end to 1 at the blossom end.
    fn profile(&self, theta: f32) -> (Vec2, f32) {
        let (sin, cos) = theta.sin_cos();
        let rho = self.radius * sin.abs().powf(SQUARE);
        let z = self.half * cos.signum() * cos.abs().powf(SQUARE);
        let dip = self.dip * (1.0 - smooth((theta - SCAR) / (RIM - SCAR)));
        let down = (1.0 - z / self.half) / 2.0;
        (Vec2::new(rho, z - self.half + self.dip - dip), down)
    }

    /// The point `theta` from the stem end and `phi` round.
    fn at(&self, theta: f32, phi: f32) -> Vec3 {
        let (profile, down) = self.profile(theta);
        let (groove, fill) = self.lobe(phi);
        let (sin, cos) = theta.sin_cos();
        // The grooves are deepest round the dip's rim, `rib_reach` of that at
        // the girth and gone at the blossom end, and a lobe falls short of the
        // fullest most at the girth.
        let shoulder = smooth(theta / RIM) * ((1.0 + cos) / 2.0).powf(self.reach);
        let s = 1.0 - self.ribbing * shoulder * groove + sin * sin * fill;
        // Drawn in toward the body's middle, so near the stem a groove pulls
        // the shoulder down as well as in: a pleat running out from the calyx.
        let middle = self.dip - self.half;
        let r = s * profile.x;
        Vec3::new(
            r * phi.cos(),
            r * phi.sin(),
            middle + s * (profile.y - middle),
        ) + (self.lean * self.radius * down).extend(0.0)
    }

    /// The groove `phi` round, as a share of `ribbing`: deepest over a septum
    /// and gone by the middle of the lobes either side; and how far short of
    /// the fullest lobe the skin there falls, most at its lobe's middle.
    fn lobe(&self, phi: f32) -> (f32, f32) {
        let lobes = self.depth.len();
        let phi = phi.rem_euclid(TAU);
        let i = self.septa.partition_point(|s| *s <= phi).clamp(1, lobes) - 1;
        let t = (phi - self.septa[i]) / (self.septa[i + 1] - self.septa[i]);
        let septum = if t < 0.5 { i } else { (i + 1) % lobes };
        let (sin, cos) = (PI * t).sin_cos();
        (self.depth[septum] * cos.powi(4), self.fill[i] * sin * sin)
    }

    /// The points round the ring `theta` from the stem end.
    fn ring(&self, theta: f32) -> Vec<Vec3> {
        self.sectors
            .iter()
            .map(|phi| self.at(theta, *phi))
            .collect()
    }

    /// The polar angles of the rings, from the stem end: one at the scar and
    /// one at the rim, and between those and the poles the fewest that keep
    /// every meridian within `tolerance` of its curve.
    fn rings_within(&self, tolerance: f32) -> Vec<f32> {
        let mut rings = Vec::new();
        for (from, to) in [(0.0, SCAR), (SCAR, RIM), (RIM, PI)] {
            let steps = ((to - from) / STEP).ceil() as usize;
            let mut rows: Vec<(f32, Vec<Vec3>)> = (0..steps)
                .map(|i| from + (to - from) * i as f32 / steps as f32)
                .chain([to])
                .map(|theta| (theta, self.ring(theta)))
                .collect();
            // A ring strays as far as its farthest point does off the chord
            // down its meridian.
            simplify_with(&mut rows, tolerance as f64, |(_, a), (_, b), (_, p)| {
                let off =
                    (0..p.len()).map(|k| p[k].distance(nearest((a[k], b[k]), (p[k], p[k])).0));
                off.fold(0.0, f32::max) as f64
            });
            // Each span starts where the last ended, the first at the pole.
            rings.extend(rows.iter().skip(1).map(|(theta, _)| *theta));
        }
        // The blossom end is a pole, not a ring.
        rings.pop();
        rings
    }

    /// The azimuths of the points round each ring, evenly spaced: as many as
    /// keep its sharpest ring within `tolerance` of its curve, three per lobe
    /// at least.
    fn sectors_within(&self, tolerance: f32) -> Vec<f32> {
        // A side across an arc bending `bend` a meter strays
        // `(1 - cos(a / 2)) / bend` off it, spanning the `a / bend` of it
        // that turns through `a`. Each probed ring is sampled round, and at
        // each sample the sides are counted as if the whole ring bent as
        // sharply and ran as fast as it does there.
        let sides = |theta: f32| {
            let q: Vec<Vec3> = (0..SAMPLES)
                .map(|i| self.at(theta, TAU * i as f32 / SAMPLES as f32))
                .collect();
            let counts = (0..SAMPLES).map(|i| {
                let u = q[i] - q[(i + SAMPLES - 1) % SAMPLES];
                let v = q[(i + 1) % SAMPLES] - q[i];
                let run = (u.length() + v.length()) / 2.0;
                let bend = ((v.normalize() - u.normalize()).length() / run).max(1e-6);
                let a = 2.0 * (1.0 - (tolerance * bend).min(1.0)).acos();
                run * SAMPLES as f32 * bend / a
            });
            counts.fold(0.0, f32::max)
        };
        let probes = (1..)
            .map(|k| k as f32 * PROBE)
            .take_while(|theta| *theta < PI);
        let sides = probes.map(sides).fold(0.0, f32::max);
        // No finer than the meridians are sampled, however fine the tolerance.
        let lobes = self.depth.len() as u32;
        let n = (sides.min(TAU / STEP).ceil() as u32).max(3 * lobes).max(8);
        (0..n).map(|i| TAU * i as f32 / n as f32).collect()
    }

    fn mesh(&self) -> MeshData {
        let rings: Vec<Vec<Vec3>> = self.rings.iter().map(|theta| self.ring(*theta)).collect();
        lathe(self.at(0.0, 0.0), &rings, self.at(PI, 0.0))
    }

    /// The meridian `phi` round, the points the body's own rings put on it,
    /// from the stem scar down as far as a sepal may lie, each with its polar
    /// angle.
    fn meridian(&self, phi: f32) -> Vec<(f32, Vec3)> {
        std::iter::once(0.0)
            .chain(self.rings.iter().copied().take_while(|theta| *theta <= HUG))
            .map(|theta| (theta, self.at(theta, phi)))
            .collect()
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
            // How deep the groove it lies along is round the shoulder, below
            // the lobes either side: its edges rise that much more.
            let groove = body.ribbing * body.radius * body.lobe(phi).0;
            let meridian = body.meridian(phi);
            let path = Path::new(meridian.iter().map(|(_, at)| *at).collect(), side);
            // How far along it the body's ring at `theta` is.
            let arc = |theta: f32| path.arcs[meridian.partition_point(|(t, _)| *t < theta)];
            let start = SEPAL_START * arc(SCAR);
            // It leaves past the rim, where the shoulder is convex.
            let ridge = arc(RIM) - start;
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
                let rise = KEEL * y.abs() + groove * (2.0 * y.abs() / wide).min(1.0);
                let across = side * y + normal * (GAP + rise);
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
    let (bud, opened, wither, size) = config.bloom();
    let open = opened * (1.0 - wither);
    // The sepals part as the bud swells, and stay spread once open.
    let parted = 0.3 * bud + 0.7 * opened;
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
/// point: its body, meshed to `tolerance`, and its calyx.
fn fruit(
    config: &FruitConfig,
    ripe: [f32; 3],
    tolerance: f32,
    rng: &mut Rng,
) -> Result<(Vec<MeshData>, Vec<[f32; 3]>)> {
    let body = Body::new(config, tolerance, rng);
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
/// with a colour per point, a fruit's body meshed to `tolerance`.
fn fruit_mesh(config: &FruitConfig, ripe: [f32; 3], tolerance: f32) -> Result<Mesh> {
    let mut rng = Rng::new(config.seed ^ CALYX_STREAM);
    let (pieces, colors) = if config.in_flower() {
        flower(config, &mut rng)?
    } else {
        fruit(config, ripe, tolerance, &mut rng)?
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
    let tolerance = TOLERANCE as f32 / params.detail.max(MIN_DETAIL);
    let meshes = par_map(&book.representatives, |_, config| {
        fruit_mesh(config, ripe, tolerance)
    });
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
        FruitConfig {
            stage,
            ..seeded(7, edit)
        }
    }

    /// A ripe fruit of the default params drawn off `seed`, 60 mm across.
    fn seeded(seed: u64, edit: fn(&mut FruitParams)) -> FruitConfig {
        let mut params = FruitParams::default();
        edit(&mut params);
        let hung = Hung {
            rank: 0.0,
            along: 0.0,
            age: 8.0,
        };
        FruitConfig::new(&params, seed, 0.06, hung)
    }

    /// A beefsteak's params: many locules, deep grooves reaching far down,
    /// lopsided.
    fn beefsteak(p: &mut FruitParams) {
        (p.aspect, p.locules, p.ribbing) = (0.65, 8.0, 0.2);
        (p.rib_reach, p.irregularity) = (0.5, 1.5);
    }

    /// The farthest `curve` strays from the chords between its points at
    /// `knots`.
    fn strays(knots: &[f32], curve: impl Fn(f32) -> Vec3) -> f32 {
        let mut farthest = 0.0f32;
        for w in knots.windows(2) {
            let (a, b) = (curve(w[0]), curve(w[1]));
            for k in 1..16 {
                let p = curve(w[0] + (w[1] - w[0]) * k as f32 / 16.0);
                let t = ((p - a).dot(b - a) / (b - a).length_squared()).clamp(0.0, 1.0);
                farthest = farthest.max(p.distance(a.lerp(b, t)));
            }
        }
        farthest
    }

    /// The farthest the body strays from its mesh's edges, down every
    /// meridian and round every ring.
    fn edges_stray(body: &Body) -> f32 {
        let poles = [&[0.0][..], &body.rings, &[PI]].concat();
        let round = [&body.sectors[..], &[TAU]].concat();
        let down = body
            .sectors
            .iter()
            .map(|phi| strays(&poles, |theta| body.at(theta, *phi)));
        let across = body
            .rings
            .iter()
            .map(|theta| strays(&round, |phi| body.at(*theta, phi)));
        down.chain(across).fold(0.0, f32::max)
    }

    /// The farthest the body strays from the triangles its mesh lays over
    /// it: two per face between rings, fanned from the face's first corner
    /// as `MeshData::to_mesh` fans it, and one per face at a pole.
    fn faces_stray(body: &Body) -> f32 {
        let thetas = [&[0.0][..], &body.rings, &[PI]].concat();
        let phis = [&body.sectors[..], &[TAU]].concat();
        let last = thetas.len() - 2;
        let mut farthest = 0.0f32;
        for (j, t) in thetas.windows(2).enumerate() {
            for f in phis.windows(2) {
                let [a, b, c, d] = [(t[0], f[0]), (t[1], f[0]), (t[1], f[1]), (t[0], f[1])]
                    .map(|(theta, phi)| body.at(theta, phi));
                let fan = match j {
                    0 => vec![[a, b, c]],
                    _ if j == last => vec![[a, b, d]],
                    _ => vec![[a, b, c], [a, c, d]],
                };
                for k in 0..81 {
                    let (u, v) = ((k / 9) as f32 / 8.0, (k % 9) as f32 / 8.0);
                    let p = body.at(t[0] + (t[1] - t[0]) * u, f[0] + (f[1] - f[0]) * v);
                    let off = fan.iter().map(|abc| off_triangle(p, *abc));
                    farthest = farthest.max(off.fold(f32::MAX, f32::min));
                }
            }
        }
        farthest
    }

    /// How far `p` is from the triangle `abc`.
    fn off_triangle(p: Vec3, [a, b, c]: [Vec3; 3]) -> f32 {
        let n = (b - a).cross(c - a).normalize();
        let q = p - n * (p - a).dot(n);
        let edges = [(a, b), (b, c), (c, a)];
        if edges
            .iter()
            .all(|(u, v)| (*v - *u).cross(q - *u).dot(n) >= 0.0)
        {
            return p.distance(q);
        }
        edges
            .iter()
            .map(|(u, v)| {
                let t = ((p - *u).dot(*v - *u) / (*v - *u).length_squared()).clamp(0.0, 1.0);
                p.distance(u.lerp(*v, t))
            })
            .fold(f32::MAX, f32::min)
    }

    /// At any detail the body is closed and wound outward: every edge is
    /// walked once each way, and the volume it encloses is positive; and it
    /// keeps within its tolerance of its surface down every meridian and
    /// round every ring, and within about twice that across every face, where
    /// the two add up and its unequal lobes twist the face a little. It hangs
    /// from its stem scar at the fruit's origin,
    /// its shoulders rising round it no higher than the dip is deep; it
    /// carries a sheet per sepal, and grows from set to mature green while
    /// its calyx stays the size it was, meshed the finer the bigger it gets.
    #[test]
    fn a_fruit_is_closed_wound_outward_and_grows_inside_its_calyx() {
        let shapes: [fn(&mut FruitParams); 3] = [
            |_| {},
            |p| (p.aspect, p.ribbing) = (0.6, 0.2),
            |p| p.aspect = 1.6,
        ];
        let details = [0.1, FruitParams::default().detail, 8.0];
        for edit in shapes {
            // A full-grown fruit's triangles at each detail.
            let mut full = [0; 3];
            for (k, detail) in details.into_iter().enumerate() {
                let tolerance = TOLERANCE as f32 / detail;
                let mut widths = Vec::new();
                for stage in [0.0, 0.5, 1.0, 1.5, 2.0, 5.0, 8.0] {
                    let at = format!("detail {detail}, stage {stage}");
                    let config = config(stage, edit);
                    let mut rng = Rng::new(config.seed ^ CALYX_STREAM);
                    let body = Body::new(&config, tolerance, &mut rng);
                    let sepals = calyx(&config, &body, &mut rng).unwrap();
                    assert_eq!(sepals.len(), config.sepals as usize);

                    // The rings are picked off meridians sampled a step apart,
                    // so between the samples they may stray a little farther.
                    let strayed = edges_stray(&body);
                    assert!(strayed <= 1.05 * tolerance, "{at}: {strayed} m off");
                    let faced = faces_stray(&body);
                    assert!(faced <= 2.05 * tolerance, "{at}: a face {faced} m off");

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
                        "{at}: open or mis-wound"
                    );
                    assert!(volume > 0.0, "{at}: inside out");

                    assert!(body.at(0.0, 0.0).length() < 1e-6, "{at}: off its calyx");
                    assert!(
                        mesh.points.iter().all(|p| p[2] <= body.dip + 1e-6),
                        "{at}: above its calyx"
                    );
                    let triangles = fruit_mesh(&config, [1.0; 3], tolerance)
                        .unwrap()
                        .indices()
                        .unwrap()
                        .len()
                        / 3;
                    let across = mesh
                        .points
                        .iter()
                        .fold(0.0f32, |m, p| m.max(p[0].hypot(p[1])));
                    widths.push((stage, across, triangles));
                    full[k] = triangles;
                }
                for w in widths.windows(2) {
                    let ((_, a, fewer), (stage, b, more)) = (w[0], w[1]);
                    let grows = if stage <= MATURE_GREEN {
                        b > a
                    } else {
                        (b - a).abs() < 1e-6
                    };
                    assert!(
                        grows && fewer <= more,
                        "detail {detail}, stage {stage}: {a} then {b} across, \
                         {fewer} then {more} triangles"
                    );
                }
            }
            assert!(
                full[0] < full[1] && full[1] < full[2] && full[1] < 2000,
                "triangles at {details:?}: {full:?}"
            );
        }
    }

    /// However its lobes fall, a fruit keeps within its tolerance down every
    /// meridian and round every ring, at any size, a round truss tomato or a
    /// lopsided beefsteak; and however fine a tolerance it is asked for, it
    /// is meshed no finer than its meridians are sampled.
    #[test]
    fn a_fruit_keeps_round_however_its_lobes_fall_and_is_meshed_no_finer_than_sampled() {
        let tolerance = TOLERANCE as f32 / 8.0;
        let shapes: [fn(&mut FruitParams); 2] = [|_| {}, beefsteak];
        for edit in shapes {
            for seed in 0..20 {
                let config = FruitConfig {
                    diameter: (50 + seed) as f32 / 1000.0,
                    ..seeded(seed, edit)
                };
                let body = Body::new(&config, tolerance, &mut Rng::new(seed ^ CALYX_STREAM));
                let strayed = edges_stray(&body);
                assert!(strayed <= 1.05 * tolerance, "seed {seed}: {strayed} m off");
            }
        }

        // A side round it and a ring down it every `STEP`, and the scar's and
        // the rim's rings.
        let config = config(8.0, |_| {});
        let finest = Body::new(&config, 0.0, &mut Rng::new(config.seed ^ CALYX_STREAM));
        assert!(finest.sectors.len() as f32 <= TAU / STEP + 1.0);
        assert!(finest.rings.len() as f32 <= PI / STEP + 3.0);
    }

    /// A regular fruit's lobes are alike, and its grooves draw its skin in
    /// toward its middle by about `ribbing` round the shoulders, by
    /// `rib_reach` of that at its girth, and not at all at its poles.
    #[test]
    fn a_regular_fruits_grooves_are_deepest_round_its_shoulders() {
        let config = seeded(7, |p| p.irregularity = 0.0);
        let body = Body::new(&config, TOLERANCE as f32, &mut Rng::new(7 ^ CALYX_STREAM));
        let middle = Vec3::Z * (body.dip - body.half);
        let crest = PI / config.lobes as f32;
        // How far in a septum `phi` round is drawn, against the crest after it.
        let sunk = |theta: f32, phi: f32| {
            let out = |phi: f32| (body.at(theta, phi) - middle).length();
            1.0 - out(phi) / out(phi + crest)
        };
        let septa = (0..config.lobes).map(|k| 2.0 * crest * k as f32);
        let girth = septa.map(|phi| sunk(PI / 2.0, phi) / config.ribbing);
        assert!(
            girth.clone().all(|g| (g - config.rib_reach).abs() < 1e-4),
            "{:?}",
            girth.collect::<Vec<_>>()
        );
        let shoulder = (1..40)
            .map(|k| sunk(0.02 * k as f32, 0.0))
            .fold(0.0, f32::max);
        assert!(
            (0.9..=1.0).contains(&(shoulder / config.ribbing)),
            "{shoulder}"
        );
        assert!(sunk(0.0, 0.0).abs() < 1e-6 && sunk(PI, 0.0).abs() < 1e-6);
    }

    /// A fruit's sepals lie clear of its skin, over a groove as well as on a
    /// lobe: on a round truss tomato, a ribbed one and a beefsteak.
    #[test]
    fn a_fruits_sepals_lie_clear_of_its_skin() {
        let shapes: [fn(&mut FruitParams); 3] =
            [|_| {}, |p| (p.aspect, p.ribbing) = (0.6, 0.2), beefsteak];
        for edit in shapes {
            for seed in 0..8 {
                let config = seeded(seed, edit);
                let mut rng = Rng::new(seed ^ CALYX_STREAM);
                let tolerance = TOLERANCE as f32 / FruitParams::default().detail;
                let mut body = Body::new(&config, tolerance, &mut rng);
                // Upright, every point of its skin is the only one that way
                // from its middle, and lies at the polar angle its profile does.
                body.lean = Vec2::ZERO;
                let middle = Vec3::Z * (body.dip - body.half);
                let polar = |q: Vec2| q.x.atan2(q.y - middle.z);
                let sepals = calyx(&config, &body, &mut rng).unwrap();
                let points = sepals.iter().flat_map(|sheet| &sheet.points);
                let deepest = points
                    .map(|p| {
                        let d = Vec3::from(*p) - middle;
                        let toward = d.truncate().length().atan2(d.z);
                        let (mut lo, mut hi) = (0.0, PI);
                        for _ in 0..40 {
                            let theta = (lo + hi) / 2.0;
                            if polar(body.profile(theta).0) < toward {
                                lo = theta;
                            } else {
                                hi = theta;
                            }
                        }
                        d.length() - (body.at(lo, d.y.atan2(d.x)) - middle).length()
                    })
                    .fold(f32::MAX, f32::min);
                // Its clearance off the body's own meridian, which keeps to
                // the tolerance.
                assert!(
                    deepest > GAP - tolerance,
                    "seed {seed}: a sepal {deepest} m off its skin"
                );
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
            let triangles = fruit_mesh(&config, [1.0; 3], TOLERANCE as f32)
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

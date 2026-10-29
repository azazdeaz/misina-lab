//! Stem — the crop: two stems to a plant, trained in a V up to the wires.
//!
//! A stem is grown forward from the week it was planted, as one strand from
//! the block it is rooted in. It grows a truss a week, three nodes to a
//! truss, up a twine hung from its hook on the wire, until its head is tied
//! under the hook's reel. From then on it is lowered as it grows: every week
//! the hooks are slid along the wire and the stems let down, so the oldest
//! part of a stem lies bare beside the gutter and only its top few meters
//! stand:
//!
//! ```text
//!         ┬   the hook on the wire, its twine reel HOOK below
//!         │   the head, tied under the reel once it has grown that tall
//!         │   a leaf at each node down from the head, `leaves_kept` of them
//!         ╲   a truss every third internode, the lowest about `harvest_height`;
//!         │   the stem kinks at each one, away from it
//!       ╭─╯   the take-off, out of the bundle up to below the lowest truss
//!  ~~~~~╯     the lowered run: all of the stem the wire is too low for
//!  ▣          the block
//! ```
//!
//! The runs go round the gutter the way growers move them: toward the far
//! end on the left, round the gutter's end, and back toward the row head on
//! the right. A stem rooted near an end may so rise on the other wire, and
//! every stretch of the row stands as dense as the rest.
//!
//! Every plant within a run's length of a place has a run lying there, so
//! late in the season a bundle of a dozen or more lies along each side of
//! the gutter. A run drifts across and up the bundle, crossing the others,
//! rests every half meter or so and sags between, and kinks at each old
//! truss. It takes off from where it last rests, climbing along the row up
//! to its twine a little below its lowest truss, as a heavy stem does where
//! a sharp bend would crack it.
//!
//! A tomato's stem is sympodial: each truss ends a shoot, and the stem
//! carries on from a side shoot below it, so the stem kinks at every truss,
//! and a little at every node, away from its leaf. On the rise the kinks are
//! level, so a node keeps its height. A stem thickens over its first 0.8 m
//! down from its head to half as thick again as its head, thins to a point
//! at its tip, and its girth undulates slowly along it.
//!
//! A stem swells at every node and every truss, the more on the side its leaf
//! or its truss leaves, and the swelling runs on down the stem as a leaf's
//! base does, which ridges it. What a stripped leaf or a picked truss leaves
//! is a stub, or a scar where it was snapped off flush: at every node and
//! truss of the stem, down its take-off and its lowered run to the block.
//! How finely all this is meshed is `detail`'s: a stem's rings go where its
//! shape needs them, and its tube strays no further from it than `detail`
//! allows.
//!
//! A week's growth is set by the crop in full production: the live height
//! from `harvest_height` to the head shared out between `trusses`, 0.34 m at
//! the defaults, inside the 25–35 cm a week growers measure. A plant goes in
//! [`TRANSPLANT`] tall with its first truss flowering at its head. A truss
//! flowers at the head every week, sets its fruit within the week, and is
//! picked `trusses` weeks after it flowered, by then that many weeks' growth
//! down the stem. So a young crop carries green fruit only, and its first
//! trusses colour about the week its heads reach the wire, the ninth at the
//! defaults. How far a truss's fruit have grown and how ripe they are follow
//! from its age.
//!
//! No two stems keep in step. Each truss comes a little sooner or later than
//! the last, so the stems drift apart as the crop ages: each runs ahead of
//! the crop or behind it by up to `development_spread` times the square root
//! of its age, in weeks, and grows up to [`GROWTH_JITTER`] faster or slower.
//! Its trusses hang that much higher or lower than its neighbours', and
//! ripen that much later or sooner.
//!
//! A leaf grows at every node, the leaves winding round the stem, and a truss
//! in every third internode. A node is placed by the length of stem between
//! it and the head, so it keeps its place on the stem as the stem is lowered.
//! Growers strip all but the top `leaves_kept` leaves, and every leaf in the
//! harvest band round the two trusses nearest their picking. A young crop has
//! no trusses that old and keeps its lowest leaves.
//!
//! A stem collides from where it takes off up, as a chain of capsules along
//! its axis, each straight from one of the axis's points to another, none of
//! the points between further than [`STRAY`] off it: a capsule from truss to
//! truss and one over each kink, the take-off a capsule for each pair of its
//! points, cut across as chords. Each reaches a radius past both its points,
//! so the chain keeps its radius over every joint, as a rail's pipe does.
//! The lowered run, below anything a robot reaches for, does not collide.
//!
//! No two stems lie along the gutter alike, so each is a mesh of its own and
//! nothing here is quantized. A stem authors a [`LeafConfig`] at each node
//! that keeps its leaf and a [`TrussConfig`] in every third internode, and
//! [`leaf`](super::leaf) and [`truss`](super::truss) build them.
//!
//! ```text
//! Row_02/Plant_017       Xform on the block's top
//!   Stem_0               Xform
//!     Tube               -> parts/Stem_n, this stem's own, with its stubs
//!     Collision_00 …     a capsule along a straight span of the stem, the take-off up
//!     Hook               -> parts/Hook_0, on the wire
//!     Twine              -> parts/Twine_n, wound down the rise
//!     Leaf_00 …          a LeafConfig, on the stem's axis at its node
//!     Truss_00 …         a TrussConfig, on the stem's axis between two nodes
//! ```

use std::f32::consts::{FRAC_PI_2, PI, TAU};

use bevy::prelude::*;
use misina_lab::geometry::mesh::{MeshData, cylinder_mesh, merge_meshes};
use misina_lab::geometry::par_map;
use misina_lab::geometry::strand::{Bark, Bulge, Collar, Strand, sides_within, strand_mesh};
use misina_lab::palette::{Response, srgb};
use misina_lab::params::{Label, Slider};
use misina_lab::rng::{Rng, salt};
use misina_lab::scene::{COLLISION, Library, Order, capsule_between};
use nalgebra::Point3;

use super::gutter::{self, GutterParams};
use super::layout::{GreenhouseLayout, Row};
use super::leaf::{LeafConfig, LeafParams, TOLERANCE};
use super::truss::{Node, TrussConfig, TrussParams};
use super::wire::WireParams;
use super::{Grow, SceneParams, smooth, vector};

/// The mesh-library prefixes: a tube and a twine for every stem, and one hook
/// for them all.
const TUBE: &str = "Stem";
const TWINE: &str = "Twine";
const HOOK: &str = "Hook";

/// Salts splitting a plant's draws, a stem's and each leaf's off the scene
/// seed.
const PLANT_STREAM: u64 = 0x9E37_79B9_7F4A_7C15;
const STEM_STREAM: u64 = 0xBF58_476D_1CE4_E5B9;
const LEAF_STREAM: u64 = 0x632B_E59B_D9B4_E019;
/// How a stem lies and kinks, its girth and its nodes, and its ridges: off
/// streams of their own, so they leave its trusses and leaves as they were.
const LIE_STREAM: u64 = 0xA076_1D64_78BD_642F;
const SURFACE_STREAM: u64 = 0xE703_7ED1_A0B4_28DB;
const RIDGE_STREAM: u64 = 0x8EBC_6AF0_9C88_C6E3;

/// Nodes per truss: a leaf at every node, and a truss in every third
/// internode.
const NODES_PER_TRUSS: u32 = 3;

/// The trusses nearest their picking, which hang on bare stem with the stem
/// up to the next one, whatever `leaves_kept` asks: the harvest band, stripped
/// for the picking.
const BARE_TRUSSES: u32 = 2;

/// A plant's stem as it goes in, in meters, its first truss flowering at its
/// head (est.).
const TRANSPLANT: f32 = 0.4;

/// Weeks after planting an extra stem is taken on from a side shoot, and
/// grown from as a stem of its own (est.).
const EXTRA_START: f32 = 6.0;

/// The turn round the stem from one leaf to the next, in radians: the golden
/// angle of a spiral phyllotaxis. How far each turn strays from it, as a
/// fraction.
const DIVERGENCE: f32 = 2.4;
const DIVERGENCE_JITTER: f64 = 0.1;

/// How much a node or a truss swells the stem all round, drawn per node, as
/// a fraction of its radius, and how far up and down the stem it reaches, in
/// meters: the stem 1.1–1.3 times as thick over 15–30 mm, the more on the
/// stalk's side (est.).
const NODE_SWELL: (f64, f64) = (0.08, 0.16);
const NODE_WIDTH: f64 = 0.008;

/// The stalk a leaf or a truss leaves the stem by, as thick as a fraction of
/// the stem, and how far off the stem's axis a leaf's petiole and a truss's
/// peduncle head, in radians (est.).
const STALK: f32 = 0.5;
const PETIOLE: f32 = 0.9;
const PEDUNCLE: f32 = 1.2;

/// How far past the stem's surface a stripped leaf's stub and a picked
/// truss's reach, at most, in meters, the short ones the most often; one
/// drawn shorter than `FLUSH` was snapped off flush and left only its scar,
/// as most leaves are. How thick a dried stub is, as a fraction of its
/// stalk, and how much thinner at its cut (est.).
const STUB: f32 = 0.012;
const PICKED: f32 = 0.025;
const FLUSH: f32 = 0.004;
const STUB_THIN: f32 = 0.9;
const STUB_TAPER: f32 = 0.85;

/// How thick a stalk a scar swells the stem as, as a fraction of the stalk
/// it healed over: flatter than a leaf's base with its leaf on (est.).
const SCAR: f32 = 0.5;

/// The most stems a plant carries: two, and an extra one.
const MAX_STEMS: usize = 3;

/// A hook's reach below the wire, in meters, which is how far below it a head
/// is held; the hook's wire and its twine reel.
const HOOK_LENGTH: f32 = 0.22;
const HOOK_RADIUS: f32 = 0.002;
const REEL: Vec2 = Vec2::new(0.018, 0.02);

/// Where a stem's head is measured, in meters below its tip, as growers
/// measure it 15–25 cm down; and how far a stem's head strays from
/// `head_diameter`, as a fraction.
const HEAD_AT: f32 = 0.2;
const HEAD_JITTER: f64 = 0.15;

/// How much thinner a stem is at its head, as a fraction, and over how much of
/// the rise below the head it thins, in meters: at `HEAD_AT` two thirds as
/// thick as from `HEAD_REACH` down, a 10 mm head over the 13–16 mm measured
/// at the lowest trusses.
const HEAD_TAPER: f32 = 0.4;
const HEAD_REACH: f32 = 0.8;

/// How deep a stem starts in its block, and how far it reaches out of it, up
/// for a young stem and leaning over toward its run for a lowered one, in
/// meters.
const ROOT_DEPTH: f32 = 0.03;
const ROOT_RISE: f32 = 0.04;

/// Where the runs lie: out past the gutter's rim, and the bundle they share
/// beside it, across and up from there, in meters.
const RUN_OUT: f32 = 0.02;
const BUNDLE: Vec2 = Vec2::new(0.12, 0.08);

/// How far a run drifts across and up the bundle, as a fraction of it, in
/// two waves each way, each between these lengths, in meters.
const DRIFT: f32 = 0.6;
const DRIFT_WAVE: (f64, f64) = (0.8, 3.0);

/// How far apart a run rests, in meters; how deep it sags between, at most,
/// the deep loops few; and how far along the run it comes to sag that much.
const REST: (f64, f64) = (0.5, 1.0);
const SAG: f32 = 0.2;
const SAG_FROM: f32 = 0.4;

/// Stem between the block and the run, and the spacing of a run's control
/// points, in meters.
const RUN_START: f32 = 0.12;
const RUN_STEP: f32 = 0.1;

/// The take-off: how far along the row a stem climbs out of the bundle, how
/// far below the lowest truss it meets its twine, and how high it climbs at
/// least, in meters; and the spacing of its control points.
const LEAN: (f64, f64) = (0.4, 1.2);
const JOIN_BELOW: f32 = 0.2;
const CLIMB: f32 = 0.15;
const TAKE_OFF_STEP: f32 = 0.08;

/// How far each kink sets the stem off, in meters: level on the rise, at
/// every truss and, by at most `NODE_KINK`, at every node; and across and up,
/// either way, on the run, at every week's growth of it.
const KINK: (f64, f64) = (0.006, 0.016);
const NODE_KINK: f64 = 0.002;
const RUN_KINK: f64 = 0.015;

/// How much a stem's girth undulates, as a fraction, in two waves, each
/// between these lengths, in meters (est.).
const GIRTH_WAVE: f32 = 0.06;
const GIRTH_WAVES: (f64, f64) = (0.5, 1.5);

/// How much thinner still a stem is at its very tip, as a fraction, and over
/// how much of it, in meters (est.).
const TIP: (f32, f32) = (0.5, 0.08);

/// How deep the ridges along a stem are, as a fraction of its radius.
const RIDGES: f64 = 0.08;

/// How far in from each end of the gutter the runs turn round it, in meters.
const RUN_TURN: f32 = 0.2;

/// How far a run's length varies about what its stem has grown past the
/// wire, as a fraction.
const LOWERED_JITTER: f64 = 0.15;

/// How much faster or slower a stem grows than the crop, as a fraction, so
/// how much further apart or nearer its trusses hang (est.).
const GROWTH_JITTER: f64 = 0.05;

/// How much further an extra stem is lowered, in meters, so that it rises
/// between its plant's two rather than beside one of them, from the week it
/// is taken on.
const EXTRA_RUN: f32 = 0.15;

/// Weeks after it flowered, so trusses below the head, that a truss's fruit
/// reach full size: the trusses nearest the head set their fruit last.
const GROWING_TRUSSES: f32 = 5.0;

/// How far a hook sits along the wire from where its stem would rise but for
/// its take-off, and how far a stem bows off the straight line between its
/// take-off and its head, in meters.
const HOOK_JITTER: f64 = 0.05;
const BOW: f32 = 0.03;

/// Least height between the lowest truss and the head, and between the
/// young stem's head and the top of its take-off, in meters, so a wire set
/// too low still grows a stem.
const MIN_LIVE: f32 = 0.3;

/// How far a truss turned to the path wanders off facing it, in radians.
const FACING_SPREAD: f64 = 0.5;

/// How far a stem's ripening runs ahead of or behind the rest, in trusses.
const RIPENING_JITTER: f64 = 0.3;

/// The twine: its radius, the stem one turn round it takes, and the spacing
/// of its control points, in meters.
const TWINE_RADIUS: f32 = 0.001;
const TWINE_PITCH: f32 = 0.3;
const TWINE_STEP: f32 = 0.03;

/// How far off a stem's axis its colliders may run, in meters: a capsule
/// spans as many of the axis's points as all lie this near it. The axis is
/// drawn as a curve through the points, which a bend's few points leave
/// bowing further off the chord between them.
const STRAY: f32 = 0.003;

/// The fewest vertices round a stem and its stubs, enough to turn a collar;
/// round the twine, and rings per meter along it; and the least `detail`
/// Python may set.
const MIN_SIDES: usize = 6;
const TWINE_SIDES: usize = 4;
const DETAIL: f64 = 16.0;
pub(crate) const MIN_DETAIL: f32 = 0.01;

/// A green stem, matte from its hairs.
const STEM_COLOR: u32 = 0x5F7F35;
const STEM_RESPONSE: Response = Response {
    roughness: 0.75,
    reflectance: 0.35,
};

/// Black plastic hooks and white polypropylene twine.
const HOOK_COLOR: u32 = 0x2B2B2B;
const TWINE_COLOR: u32 = 0xE6E1D3;
const PLASTIC: Response = Response {
    roughness: 0.5,
    reflectance: 0.5,
};

/// The crop's stems: two to a plant, trained alternately to the left and the
/// right wire, and lowered as they grow.
///
/// Each stem grows a truss a week from planting, up its twine to a hook on
/// the wire. Once its head is a hook's length below the wire it is lowered,
/// and all the stem the wire is too low for lies along the gutter. In full
/// production the stem between `harvest_height` and the head carries
/// `trusses` trusses with three leaves to each, so how much a stem grows in a
/// week follows from the wire height rather than being set. Only the top
/// `leaves_kept` leaves are left on; the stem below them is stripped bare.
#[derive(Resource, Reflect, Clone, Debug, PartialEq)]
#[cfg_attr(
    feature = "python",
    pyo3::pyclass(get_all, set_all, skip_from_py_object)
)]
pub struct StemParams {
    /// Height above the floor of the lowest truss still on a stem once the
    /// crop is in full production, in meters: where a harvesting arm reaches
    /// to. A stem's lowest hangs up to a week's growth above it, the higher
    /// the sooner after its last picking, and a stem that grows faster than
    /// the rest hangs it up to 5% of its live height lower. The stem below it
    /// has been picked and stripped bare.
    #[reflect(@Slider { min: 0.8, max: 1.8, step: 0.05 })]
    pub harvest_height: f32,
    /// Trusses on a stem in full production, from the lowest still on it up
    /// to the head. At a truss a week, also the weeks from a truss flowering
    /// at the head to its picking.
    #[reflect(@Slider { min: 6.0, max: 12.0, step: 1.0 })]
    pub trusses: u32,
    /// How far stems drift apart in their development, in weeks: each truss
    /// comes a little sooner or later than the last, so a stem `week` weeks
    /// old runs up to this times √`week` ahead of the crop or behind it, its
    /// trusses hanging that much of a week's growth lower or higher and
    /// ripening that much sooner or later. At 0.2, up to 0.8 of a week at
    /// week 16; at 0 every stem keeps in step with the crop, and its trusses
    /// stray from its neighbours' heights only as far as it grows up to 5%
    /// faster or slower.
    #[reflect(@Slider { min: 0.0, max: 0.5, step: 0.01 })]
    pub development_spread: f32,
    /// Leaves on a stem, counted down from the head. Growers strip the ones
    /// below, so the stem under them is bare, and the two trusses nearest
    /// their picking always hang on bare stem however many are kept.
    #[reflect(@Slider { min: 10.0, max: 21.0, step: 1.0 })]
    pub leaves_kept: u32,
    /// Share of plants grown on with a third stem, which growers keep to raise
    /// the stem density.
    #[reflect(@Slider { min: 0.0, max: 0.5, step: 0.05 })]
    pub extra_stems: f32,
    /// Share of trusses turned to face the path. The rest face anywhere, into
    /// the crop as often as not, where a harvester has to skip them.
    #[reflect(@Slider { min: 0.0, max: 1.0, step: 0.05 }, @Label("Trusses turned to the path"))]
    pub turned_to_path: f32,
    /// Diameter of a stem's head, in meters: how thick it is 20 cm below its
    /// tip, where growers measure it. About 10 mm on a balanced crop; under
    /// 8 mm the plant is spent on its fruit, over 12 mm it grows leaf, and
    /// beefsteak stems run thicker than cherry. Each stem draws its own about
    /// it, and is half as thick again from 0.8 m below its tip down.
    #[reflect(@Slider { min: 0.006, max: 0.014, step: 0.0005 })]
    pub head_diameter: f32,
    /// Height above the floor, in meters, below which a truss bends: one
    /// whose node is lower is exported as a rod a physics engine bends and a
    /// robot can cut, carrying its fruit, rather than as a static mesh. 0
    /// bends none.
    ///
    /// A bent truss is about ten rigid bodies, so this is what sets the cost
    /// of simulating the crop.
    #[reflect(@Slider { min: 0.0, max: 5.5, step: 0.05 }, @Label("Trusses bend below"))]
    pub flexible_below: f32,
    /// How finely the stems and the trusses' stems are meshed. At 1 a stem's
    /// tube strays from its shape by about a pixel to a camera half a meter
    /// away, as a leaf does at its own detail of 1; at 4 by a quarter of
    /// that. Down to about 0.5 every node shows its swelling and the swelling
    /// round its leaf or truss; lower is cheaper, and at 0.1 a stem is a
    /// plain tube with its stubs.
    #[reflect(@Slider { min: 0.1, max: 4.0, step: 0.05 }, @Label("Stem detail"))]
    pub detail: f32,
}

impl Default for StemParams {
    fn default() -> Self {
        Self {
            harvest_height: 1.2,
            trusses: 9,
            development_spread: 0.2,
            leaves_kept: 16,
            extra_stems: 0.0,
            turned_to_path: 0.8,
            head_diameter: 0.01,
            flexible_below: 0.0,
            detail: 0.5,
        }
    }
}

/// Marks a plant's prim, which every rebuild replaces with all it holds.
#[derive(Component)]
struct Owned;

pub fn plugin(app: &mut App) {
    // `LeafParams` and `TrussParams` are not gated on: `leaf` and `truss`
    // re-apply them to the configs authored here, in place.
    app.init_resource::<StemParams>().add_systems(
        PreUpdate,
        author
            .run_if(
                resource_changed::<StemParams>
                    .or_eager(resource_changed::<GutterParams>)
                    .or_eager(resource_changed::<WireParams>)
                    .or_eager(resource_changed::<SceneParams>)
                    .or_eager(resource_changed::<GreenhouseLayout>),
            )
            .in_set(Grow::Stems),
    );
}

/// The path the runs follow round one gutter, `out` from its centreline: out
/// along the left side, round the far end, back along the right and round
/// the row head. A place on it is how far round it is, from the row head on
/// the left.
#[derive(Clone, Copy, Debug)]
struct Round {
    from: f32,
    to: f32,
    out: f32,
}

impl Round {
    fn straight(&self) -> f32 {
        self.to - self.from
    }

    fn turn(&self) -> f32 {
        PI * self.out
    }

    fn length(&self) -> f32 {
        2.0 * (self.straight() + self.turn())
    }

    /// Where `u` round puts a stem in the row, and which way it runs there.
    fn at(&self, u: f32) -> (Vec2, Vec2) {
        let (straight, turn) = (self.straight(), self.turn());
        let u = u.rem_euclid(self.length());
        if u < straight {
            return (Vec2::new(self.from + u, self.out), Vec2::X);
        }
        let u = u - straight;
        if u < turn {
            let a = u / self.out;
            return (
                Vec2::new(self.to + self.out * a.sin(), self.out * a.cos()),
                Vec2::new(a.cos(), -a.sin()),
            );
        }
        let u = u - turn;
        if u < straight {
            return (Vec2::new(self.to - u, -self.out), -Vec2::X);
        }
        let a = (u - straight) / self.out;
        (
            Vec2::new(self.from - self.out * a.sin(), -self.out * a.cos()),
            Vec2::new(-a.cos(), a.sin()),
        )
    }

    /// Where a stem rooted at `x` starts round, on the left (`side` 1) or
    /// the right (-1).
    fn start(&self, x: f32, side: f32) -> f32 {
        let x = x.clamp(self.from, self.to);
        if side > 0.0 {
            x - self.from
        } else {
            self.straight() + self.turn() + self.to - x
        }
    }

    /// `u`, or the end of the turn it falls in: a stem rises beside the
    /// gutter, never off its end.
    fn beside(&self, u: f32) -> f32 {
        let (straight, turn) = (self.straight(), self.turn());
        let w = u.rem_euclid(self.length());
        let past = if w < straight {
            0.0
        } else if w < straight + turn {
            straight + turn - w
        } else if w < 2.0 * straight + turn {
            0.0
        } else {
            self.length() - w
        };
        u + past
    }
}

/// What every stem grows by, in the row frame.
#[derive(Clone, Copy, Debug)]
struct Live {
    /// Where the runs lie: the gutter's rim.
    rim: f32,
    /// The head's height once it has grown up to the wire.
    head: f32,
    /// The lowest truss's height in full production.
    harvest: f32,
    trusses: u32,
    /// How far a stem's development strays from the crop's, in weeks, times
    /// the square root of its age.
    development_spread: f32,
    spread: f32,
    /// How thick a stem's head is, about which each stem draws its own.
    head_diameter: f32,
    /// Where along the row the runs turn round the gutter.
    ends: (f32, f32),
    /// How high a truss's node may be and the truss still bend.
    flexible_below: f32,
    /// How many leaves a stem keeps, down from its head.
    leaves_kept: u32,
}

/// One stem, solved in the row frame: everything its meshes and its trusses
/// are made from.
#[derive(Clone, Debug)]
struct Stem {
    /// The tube's control points, root to head, and its radius at each.
    points: Vec<Vec3>,
    radii: Vec<f32>,
    /// The point the stem takes off from its lowered run at, or the root's
    /// if it has none: the rise starts there, and so do the colliders.
    take_off: usize,
    /// Where a leaf or a truss grows out of the stem, or did: at every node
    /// and every truss, down to the block.
    joints: Vec<Joint>,
    /// The twine's, from the reel down the rise.
    twine: Vec<Vec3>,
    /// The hook's place on its wire.
    hook: Vec3,
    /// Each truss, lowest first: its place on the stem's axis, the yaw it
    /// faces, and what it takes from the stem there.
    trusses: Vec<(Vec3, f32, Node)>,
    /// Each node of the rise, the head's first: its place on the stem's axis
    /// and the yaw its leaf faces.
    nodes: Vec<(Vec3, f32)>,
    /// How many nodes from the head keep their leaf: up to `leaves_kept` of
    /// those above the harvest band.
    kept: usize,
}

/// Where a leaf or a truss grows out of a stem, or did.
#[derive(Clone, Copy, Debug)]
struct Joint {
    /// How far along the tube's control points it is, in meters, and its
    /// place on the stem's axis there.
    at: f32,
    on: Vec3,
    /// The way its stalk heads off the axis, and how thick it is.
    toward: Vec3,
    radius: f32,
    /// How much it swells the stem all round, as a fraction of its radius.
    swell: f32,
    /// The stub it left, from the axis to the cut, in meters: none while it
    /// hangs, or where it was snapped off flush.
    stub: f32,
}

/// The way a stalk heads off a stem heading `tangent`: `angle` off it, toward
/// `side` round it.
fn stalk(tangent: Vec3, side: Vec3, angle: f32) -> Vec3 {
    let across = side
        .reject_from_normalized(tangent)
        .normalize_or(tangent.any_orthonormal_vector());
    across * angle.sin() + tangent * angle.cos()
}

/// The swellings `joints` put on their stem, in the strand's terms.
fn swellings(joints: &[Joint]) -> (Vec<Bulge>, Vec<Collar>) {
    joints
        .iter()
        .map(|joint| {
            let at = joint.at as f64;
            let bulge = Bulge {
                at,
                width: NODE_WIDTH,
                height: joint.swell as f64,
            };
            let collar = Collar {
                at,
                toward: vector(joint.toward),
                radius: joint.radius as f64,
            };
            (bulge, collar)
        })
        .unzip()
}

/// The offset at `at` along a line kinked at `knots`, each a place along it
/// and the offset there, in order: straight from one to the next, and the
/// nearest end's past either end.
fn zigzag(knots: &[(f32, Vec2)], at: f32) -> Vec2 {
    let next = knots.partition_point(|(place, _)| *place <= at);
    match (
        next.checked_sub(1).map(|i| knots[i]),
        knots.get(next).copied(),
    ) {
        (Some((a, p)), Some((b, q))) => p.lerp(q, (at - a) / (b - a)),
        (Some((_, p)), None) | (None, Some((_, p))) => p,
        (None, None) => Vec2::ZERO,
    }
}

/// How thick a stem is `d` below its tip, as a fraction of how thick it is
/// from `HEAD_REACH` down: thinner toward its head, and to a point at its tip.
fn thinning(d: f32) -> f32 {
    (1.0 - HEAD_TAPER * smooth(1.0 - d / HEAD_REACH)) * (1.0 - TIP.0 * smooth(1.0 - d / TIP.1))
}

/// Grows the stem rooted at `root` that starts round on `side`, `age` weeks
/// after it was planted and lowered `more` further than its growth calls for,
/// from its own three streams.
///
/// The *draw order* is part of this element's output. `rng` draws how thick
/// its head is, how far its development runs ahead, how much faster it grows,
/// how far the run's length strays, where across and up the bundle
/// the run lies, the hook's place along the wire, the bow's bearing and the
/// ripening offset, then three per hung truss — whether it was turned to the
/// path, how far it wanders off it, and the bearing it faces when it was not
/// — and last the yaw of the leaf nearest the head and, node by node down,
/// how far the next one turns from it. `lie` draws the four waves the run
/// drifts in, each's length and phase, and the take-off's lean; then for a
/// lowered stem how far apart its run rests, the sag of each span back from
/// the take-off and the kink at each week's growth back from it but the
/// take-off's own, across then up; then the kink at each truss up the rise,
/// hung or picked, its bearing first when no truss hangs there; and last the
/// kink at each node up the rise, head first. `surface` draws the two waves
/// the girth undulates in, each's length and phase; then three at every
/// node, head first, and then at every truss, youngest first, down to the
/// block: how much it swells the stem, how far its stub would reach, and the
/// side it leaves to where nothing on the stem says.
#[allow(clippy::too_many_arguments)]
fn grow(
    root: Vec3,
    side: f32,
    age: f32,
    more: f32,
    turned_to_path: f32,
    live: Live,
    rng: &mut Rng,
    lie: &mut Rng,
    surface: &mut Rng,
) -> Stem {
    // The girth undulates along the stem in two waves.
    let waves: [(f32, f32); 2] = std::array::from_fn(|_| {
        let length = surface.range(GIRTH_WAVES.0, GIRTH_WAVES.1) as f32;
        (TAU / length, surface.unit() as f32 * TAU)
    });
    let undulation = |d: f32| {
        let sum: f32 = waves.iter().map(|(n, phase)| (n * d + phase).sin()).sum();
        1.0 + GIRTH_WAVE * sum / 2.0
    };
    // The radius the stem is thinned and undulated from, so that its head at
    // `HEAD_AT` is as thick as drawn.
    let drawn = rng.range(1.0 - HEAD_JITTER, 1.0 + HEAD_JITTER) as f32;
    let radius = live.head_diameter / 2.0 * drawn / (thinning(HEAD_AT) * undulation(HEAD_AT));
    // How far its development runs ahead of the crop's, the further the
    // older it is, so how old it is as a stem; and how much faster it grows.
    let ahead = live.development_spread * age.sqrt() * rng.range(-1.0, 1.0) as f32;
    let age = (age + ahead).max(0.0);
    let growth = rng.range(1.0 - GROWTH_JITTER, 1.0 + GROWTH_JITTER) as f32;
    let stray = rng.range(1.0 - LOWERED_JITTER, 1.0 + LOWERED_JITTER) as f32;
    let round = Round {
        from: live.ends.0,
        to: live.ends.1,
        out: gutter::WIDTH / 2.0 + RUN_OUT + radius,
    };
    // Where across and up the bundle the run lies, as fractions of it.
    let lies = Vec2::new(rng.unit() as f32, rng.unit() as f32);
    let hook_x = rng.range(-HOOK_JITTER, HOOK_JITTER) as f32;
    let bow = Vec2::from_angle(rng.unit() as f32 * TAU).extend(0.0) * BOW;
    let ripening = rng.range(-RIPENING_JITTER, RIPENING_JITTER) as f32;

    // A week's growth: the live height in full production, from the lowest
    // truss to the head, shared out between the trusses on it, one a week.
    // A stem that grows faster spaces them further apart, so hangs its lowest
    // lower.
    let bundle = live.rim + radius + BUNDLE.y;
    let head = live.head.max(bundle + CLIMB + 2.0 * MIN_LIVE);
    let harvest = live.harvest.clamp(bundle + CLIMB, head - MIN_LIVE);
    let harvest = head - (head - harvest) * growth;
    let weekly = (head - harvest) / live.trusses as f32;
    let node = weekly / NODES_PER_TRUSS as f32;
    // How tall it would stand, so how much of it the wire is too low for.
    let tall = root.z + TRANSPLANT + weekly * age;
    let run = ((tall - head).max(0.0) + more) * stray;
    let start = round.start(root.x, side);
    let end = round.beside(start + run);
    let lowered = end > start + RUN_START;
    let rise_side = if lowered {
        round.at(end).0.y.signum()
    } else {
        side
    };

    // A truss flowers at the head every week, the transplant's first, and
    // `t` weeks on hangs `t` weeks' growth below it: hung from then on, its
    // fruit set within the week, and picked `trusses` weeks on, so a stem in
    // full production carries `trusses` of them whatever the day. The
    // oldest, the lowest, first, each with the yaw it faces while it hangs.
    let flowered: Vec<(f32, Option<f32>)> = (0..)
        .map(|f| age - f as f32)
        .take_while(|t| *t > 0.0)
        .map(|t| {
            let hung = t <= live.trusses as f32;
            let yaw = hung.then(|| {
                let turned = rng.unit() < turned_to_path as f64;
                let wander = rng.range(-FACING_SPREAD, FACING_SPREAD) as f32;
                let anywhere = rng.unit() as f32 * TAU;
                if turned {
                    rise_side * FRAC_PI_2 + wander
                } else {
                    anywhere
                }
            });
            (t, yaw)
        })
        .collect();

    // The run drifts across and up the bundle in two waves each way, out of
    // step with every other run.
    let waves: [(f32, f32); 4] = std::array::from_fn(|_| {
        let length = lie.range(DRIFT_WAVE.0, DRIFT_WAVE.1) as f32;
        (TAU / length, lie.unit() as f32 * TAU)
    });
    let drift = |u: f32, way: usize| {
        let wave = |(n, phase): (f32, f32)| (n * u + phase).sin();
        DRIFT * (wave(waves[2 * way]) + wave(waves[2 * way + 1])) / 2.0
    };
    let lean = lie.range(LEAN.0, LEAN.1) as f32;

    // Out of the block, over the rim onto the run, along it, and off it
    // where it last rests, climbing along the row up to the twine; with no
    // run, up out of the block.
    let mut points = vec![root - Vec3::Z * ROOT_DEPTH];
    let (take_off, base) = if lowered {
        let lean = lean.min(end - start - RUN_START);
        let off = end - lean;
        let rest = lie.range(REST.0, REST.1) as f32;
        let spans = ((off - start) / rest).ceil() as usize + 1;
        let sags: Vec<f32> = (0..spans)
            .map(|_| SAG * (lie.unit() as f32).powi(2))
            .collect();
        let weeks = ((off - start) / weekly).ceil() as usize + 2;
        let kinks: Vec<(f32, Vec2)> = (0..weeks)
            .map(|k| {
                let mut kink = || lie.range(-RUN_KINK, RUN_KINK) as f32;
                let set = if k == 0 {
                    Vec2::ZERO
                } else {
                    Vec2::new(kink(), kink())
                };
                (k as f32 * weekly, set)
            })
            .collect();
        // The run at `u` round the gutter, `back` from the take-off: in the
        // bundle, out from the round along its outward normal, sagging
        // between rests but beside its block, and kinked.
        let lying = |u: f32| {
            let (at, along) = round.at(u);
            let back = off - u;
            let span = back / rest;
            let sag = sags[(span as usize).min(spans - 1)]
                * 4.0
                * span.fract()
                * (1.0 - span.fract())
                * smooth((u - start - RUN_START) / SAG_FROM);
            let kink = zigzag(&kinks, back);
            let across = BUNDLE.x * (lies.x + drift(u, 0)).clamp(0.0, 1.0) + kink.x;
            let up = BUNDLE.y * (lies.y + drift(u, 1)).clamp(0.0, 1.0) + kink.y - sag;
            let outward = Vec2::new(-along.y, along.x);
            (at + outward * across).extend((live.rim + radius + up).max(radius))
        };

        let first = lying(start + RUN_START);
        let toward = (first - root).with_z(0.0).normalize_or_zero();
        points.push(root + (Vec3::Z / 2.0 + toward) * ROOT_RISE);
        let mut u = start + RUN_START;
        while u < off - RUN_STEP / 2.0 {
            points.push(lying(u));
            u += RUN_STEP;
        }
        let from = lying(off);
        points.push(from);
        // Below where it would stand but for its lean.
        (points.len() - 1, round.at(end).0.extend(from.z))
    } else {
        (0, root + Vec3::Z * ROOT_RISE)
    };

    // The twine hangs from its reel to the stem, which is wound up it to its
    // head, tied under the reel once it has grown that tall. A lowered stem
    // meets its twine a little below its lowest truss, a young one at its
    // block.
    let top = Vec3::new(base.x + hook_x, rise_side * live.spread, head);
    let join = if lowered {
        let from = points[take_off];
        let height = (harvest - JOIN_BELOW)
            .min(tall - MIN_LIVE)
            .max(from.z + CLIMB);
        let join = base.lerp(top, (height - base.z) / (head - base.z));
        // A cubic out along the run and into the line up to the head.
        let along = (base - from).with_z(0.0).normalize_or(Vec3::X);
        let ctrl = [
            from,
            from + along * lean.min(base.distance(from)) / 2.0,
            join - (top - join).normalize() * (join.z - from.z) / 2.0,
            join,
        ];
        let steps = (from.distance(join) / TAKE_OFF_STEP).ceil().max(2.0) as usize;
        points.extend((1..steps).map(|i| {
            let (t, s) = (i as f32 / steps as f32, 1.0 - i as f32 / steps as f32);
            ctrl[0] * s * s * s
                + ctrl[1] * 3.0 * s * s * t
                + ctrl[2] * 3.0 * s * t * t
                + ctrl[3] * t * t * t
        }));
        join
    } else {
        base
    };
    let tip = join.lerp(top, (tall.min(head) - join.z) / (head - join.z));
    let reach = join.distance(tip);

    // A level kink at every truss up the rise, hung or picked, away from
    // the truss while it hangs, fading out over the top node and none at the
    // head or the take-off; and the bearing each of those trusses leaves the
    // stem at.
    let mut knots = vec![(0.0, Vec2::ZERO)];
    let mut bearings = Vec::new();
    for (t, yaw) in flowered.iter().rev() {
        let d = weekly * t;
        if d > reach - weekly / 2.0 {
            break;
        }
        let away = yaw.map_or_else(|| lie.unit() as f32 * TAU, |yaw| yaw + PI);
        let kink = lie.range(KINK.0, KINK.1) as f32 * smooth(d / node);
        knots.push((d, Vec2::from_angle(away) * kink));
        bearings.push(away - PI);
    }
    knots.push((reach, Vec2::ZERO));

    // A node every `node` of stem down from the head, half a node off the
    // trusses, the newest grown out half a node or more below the head.
    // The leaves of the rise wind round the stem from the head down, and the
    // stem kinks a little away from each.
    let newest = ((NODES_PER_TRUSS as f32 * age).fract() + 0.5) * node;
    let below: Vec<f32> = (0..)
        .map(|i| newest + i as f32 * node)
        .take_while(|d| *d < reach - node / 2.0)
        .collect();
    let mut yaw = rng.unit() as f32 * TAU;
    let yaws: Vec<f32> = below
        .iter()
        .map(|_| {
            let at = yaw;
            yaw += DIVERGENCE * rng.range(1.0 - DIVERGENCE_JITTER, 1.0 + DIVERGENCE_JITTER) as f32;
            at
        })
        .collect();
    let mut nodding = vec![(0.0, Vec2::ZERO)];
    nodding.extend(below.iter().zip(&yaws).map(|(d, yaw)| {
        (
            *d,
            Vec2::from_angle(yaw + PI) * lie.range(0.0, NODE_KINK) as f32,
        )
    }));
    nodding.push((reach, Vec2::ZERO));
    // The rise at `d` of stem below the head: straight from the take-off to
    // the head but for a bow pinned at both ends and the kinks, the nodes'
    // on the trusses'. All are level, so the nodes are as evenly spaced in
    // height as along the stem.
    let rise = |d: f32| {
        let f = 1.0 - d / reach;
        let kinked = zigzag(&knots, d) + zigzag(&nodding, d);
        join.lerp(tip, f) + bow * (PI * f).sin() + kinked.extend(0.0)
    };
    // Thinned toward the head and undulating along it.
    let girth = |d: f32| radius * thinning(d) * undulation(d);

    // A control point at the join, at each node and each kinked truss of the
    // rise, and at the head.
    let mut placed: Vec<f32> = flowered
        .iter()
        .map(|(t, _)| weekly * t)
        .filter(|d| *d <= reach - weekly / 2.0)
        .chain(below.iter().copied())
        .collect();
    placed.sort_by(|a, b| b.total_cmp(a));
    let join_at = points.len();
    points.push(join);
    points.extend(placed.iter().chain([&0.0]).map(|d| rise(*d)));
    // How far along the control points each is, in meters, and the stem
    // between it and the head: up the rise, where it was placed, and below
    // the rise, by the stem between it and the join.
    let lengths: Vec<f32> = points
        .iter()
        .scan((0.0, points[0]), |(length, last), p| {
            *length += last.distance(*p);
            *last = *p;
            Some(*length)
        })
        .collect();
    let dists: Vec<f32> = (0..points.len())
        .map(|i| match i.checked_sub(join_at) {
            Some(0) => reach,
            Some(k) => placed.get(k - 1).copied().unwrap_or(0.0),
            None => reach + lengths[join_at] - lengths[i],
        })
        .collect();
    // How far along the control points `d` of stem below the head is, its
    // place on the axis there, and the way the stem heads.
    let place = |d: f32| {
        let i = dists.partition_point(|x| *x > d).clamp(1, dists.len() - 1);
        let span = dists[i - 1] - dists[i];
        let f = if span > 0.0 {
            ((dists[i - 1] - d) / span).clamp(0.0, 1.0)
        } else {
            0.0
        };
        (
            lengths[i - 1] + (lengths[i] - lengths[i - 1]) * f,
            points[i - 1].lerp(points[i], f),
            (points[i] - points[i - 1]).normalize_or(Vec3::Z),
        )
    };
    let radii = dists.iter().map(|d| girth(*d)).collect();

    // The harvest band: the stem below the oldest truss with `BARE_TRUSSES`
    // weeks or more left before its picking, or below where it will hang. As
    // many leaves above it as growers keep stay on.
    let due = live.trusses as f32 - BARE_TRUSSES as f32;
    let band = weekly * (due - (due - age).rem_euclid(1.0));
    let kept = below
        .iter()
        .take_while(|d| **d < band)
        .count()
        .min(live.leaves_kept as usize);

    // A joint at every node and every truss down to where the stem leaves
    // its block, a stub on it once it is stripped or picked. A leaf of the
    // rise and a truss there leave toward their bearing; below the rise the
    // stem has turned as it was lowered, so to any side.
    let last = dists[1];
    let mut joint = |d: f32, bearing: Option<f32>, angle: f32, stub: Option<f32>| {
        let swell = surface.range(NODE_SWELL.0, NODE_SWELL.1) as f32;
        let past = surface.unit() as f32;
        let side = surface.unit() as f32 * TAU;
        let (at, on, tangent) = place(d);
        let side = bearing.map_or_else(
            || Quat::from_axis_angle(tangent, side) * Vec3::Z,
            |bearing| Vec2::from_angle(bearing).extend(0.0),
        );
        let r = girth(d);
        // Out through the stem, swollen and collared, and on to the cut; or
        // snapped off flush, healed over.
        let (stub, radius) = match stub.map(|most| most * past * past) {
            Some(past) if past < FLUSH => (0.0, SCAR * STALK * r),
            Some(past) => (r * (1.0 + swell + STALK) / angle.sin() + past, STALK * r),
            None => (0.0, STALK * r),
        };
        Joint {
            at,
            on,
            toward: stalk(tangent, side, angle),
            radius,
            swell,
            stub,
        }
    };
    let mut joints: Vec<Joint> = (0..)
        .map(|k| (k, newest + k as f32 * node))
        .take_while(|(_, d)| *d < last)
        .map(|(k, d)| {
            joint(
                d,
                yaws.get(k).copied(),
                PETIOLE,
                (k >= kept).then_some(STUB),
            )
        })
        .collect();
    joints.extend(
        flowered
            .iter()
            .rev()
            .enumerate()
            .map(|(k, (t, yaw))| (k, weekly * t, yaw.is_none()))
            .take_while(|(_, d, _)| *d < last)
            .map(|(k, d, picked)| {
                joint(
                    d,
                    bearings.get(k).copied(),
                    PEDUNCLE,
                    picked.then_some(PICKED),
                )
            }),
    );

    let trusses = flowered
        .iter()
        .filter_map(|(t, yaw)| {
            let d = weekly * t;
            let node = Node {
                stem_radius: girth(d),
                rank: live.trusses as f32 - t - ripening,
                growth: (t / GROWING_TRUSSES).min(1.0),
                flexible: rise(d).z < live.flexible_below,
            };
            yaw.map(|yaw| (rise(d), yaw, node))
        })
        .collect();

    // From the reel straight down to the head, then wound down the rise,
    // just clear of the stem: of its girth with a ridge on it, and of
    // whatever swells it there.
    let (bulges, collars) = swellings(&joints);
    let axis = (top - join).normalize();
    let across = axis.cross(Vec3::X).normalize();
    let over = axis.cross(across);
    let reel = top + Vec3::Z * 0.01;
    let drop = (reel.distance(tip) / TWINE_STEP).ceil();
    let mut twine: Vec<Vec3> = (0..drop as usize)
        .map(|i| reel.lerp(tip, i as f32 / drop))
        .collect();
    let mut d = 0.0;
    while d < reach {
        let turn = TAU * d / TWINE_PITCH;
        let out = across * turn.cos() + over * turn.sin();
        let (m, r) = (place(d).0 as f64, girth(d) as f64);
        let swollen: f64 = bulges.iter().map(|b| b.value(m)).sum::<f64>()
            + collars
                .iter()
                .map(|c| c.swelling(vector(axis), r, m, &vector(out)))
                .sum::<f64>();
        let clear = r * (1.0 + RIDGES) * (1.0 + swollen);
        twine.push(rise(d) + out * (clear as f32 + TWINE_RADIUS));
        d += TWINE_STEP;
    }

    Stem {
        points,
        radii,
        take_off,
        joints,
        twine,
        hook: top + Vec3::Z * HOOK_LENGTH,
        trusses,
        nodes: below
            .iter()
            .zip(yaws)
            .map(|(d, yaw)| (rise(*d), yaw))
            .collect(),
        kept,
    }
}

/// The stub `joint` left, `offset` into its plant's frame: its stalk, dried
/// thinner, from the stem's axis to the cut.
fn stub(joint: &Joint, offset: Vec3, sides: usize) -> Strand {
    let (from, to) = (joint.on, joint.on + joint.toward * joint.stub);
    let radius = joint.radius * STUB_THIN;
    strand(
        &[from, from.lerp(to, 0.5), to],
        &[radius, radius, radius * STUB_TAPER],
        offset,
        sides,
        Bark::none(),
    )
}

/// A strand through `points`, `offset` into its plant's frame.
fn strand(points: &[Vec3], radii: &[f32], offset: Vec3, sides: usize, bark: Bark) -> Strand {
    let points = points
        .iter()
        .map(|p| {
            let p = *p - offset;
            Point3::new(p.x as f64, p.y as f64, p.z as f64)
        })
        .collect();
    let radii = radii.iter().map(|r| *r as f64).collect();
    Strand::new(points, radii, sides, DETAIL, bark)
}

/// The capsules `stem` collides as, from its take-off to its head: each a
/// radius and the two points of its axis it spans, straight between them,
/// as thick as the stem is anywhere along it, and no point between further
/// than [`STRAY`] off it.
fn colliders(stem: &Stem) -> Vec<(f32, Vec3, Vec3)> {
    let (points, radii) = (&stem.points[stem.take_off..], &stem.radii[stem.take_off..]);
    let off = |a: Vec3, b: Vec3, p: Vec3| {
        let d = b - a;
        p.distance(a + d * ((p - a).dot(d) / d.length_squared()).clamp(0.0, 1.0))
    };
    let mut spans = Vec::new();
    let mut from = 0;
    while from + 1 < points.len() {
        let mut to = from + 1;
        while to + 1 < points.len()
            && (from + 1..=to).all(|i| off(points[from], points[to + 1], points[i]) < STRAY)
        {
            to += 1;
        }
        let radius = radii[from..=to].iter().copied().fold(0.0, f32::max);
        spans.push((radius, points[from], points[to]));
        from = to;
    }
    spans
}

/// A hook hung from its origin on the wire: the wire it is bent from, and the
/// twine reel at its foot.
fn hook_mesh() -> MeshData {
    let hung = Transform::from_xyz(0.0, 0.0, -HOOK_LENGTH);
    let reel = Transform::from_xyz(0.0, REEL.y / 2.0, -HOOK_LENGTH)
        .with_rotation(Quat::from_rotation_x(FRAC_PI_2));
    merge_meshes(&[
        cylinder_mesh(HOOK_RADIUS, HOOK_RADIUS, HOOK_LENGTH, 6).transformed(&hung),
        cylinder_mesh(REEL.x, REEL.x, REEL.y, 10).transformed(&reel),
    ])
}

/// One stem to build: its row, its plant, its index on the plant, the salt
/// its streams are drawn with, and what it was solved to.
type Planted = (Entity, usize, usize, u64, Stem);

#[allow(clippy::too_many_arguments)]
fn author(
    mut commands: Commands,
    mut library: Library,
    params: Res<StemParams>,
    leaf_params: Res<LeafParams>,
    truss_params: Res<TrussParams>,
    gutter: Res<GutterParams>,
    wire: Res<WireParams>,
    scene: Res<SceneParams>,
    layout: Res<GreenhouseLayout>,
    rows: Query<(Entity, &Row)>,
) -> Result<()> {
    commands.despawn_all::<With<Owned>>();
    for prefix in [TUBE, TWINE, HOOK] {
        library.clear(prefix);
    }
    let hook = library.part(
        HOOK,
        0,
        hook_mesh().to_mesh(),
        PLASTIC.surface(srgb(HOOK_COLOR)),
    );

    let root_z = gutter.block_top();
    let live = Live {
        rim: gutter.floor() + gutter::DEPTH,
        head: wire.height - HOOK_LENGTH,
        harvest: params.harvest_height,
        trusses: params.trusses.max(1),
        development_spread: params.development_spread,
        spread: wire.spread,
        head_diameter: params.head_diameter,
        ends: (RUN_TURN, (layout.length - RUN_TURN).max(RUN_TURN)),
        flexible_below: params.flexible_below,
        leaves_kept: params.leaves_kept,
    };
    let week = scene.week.max(0.0);

    // In row order, so the trusses' `Order` follows from the layout alone.
    let mut rows: Vec<(usize, Entity)> = rows.iter().map(|(e, row)| (row.0, e)).collect();
    rows.sort();
    let mut stems: Vec<Planted> = Vec::new();
    for &(row, entity) in &rows {
        for (plant, &x) in layout.plants.iter().enumerate() {
            let at = (row * layout.plants.len() + plant) as u64;
            let extra = Rng::new(scene.seed ^ PLANT_STREAM ^ salt(at)).unit();
            let count = if extra < params.extra_stems as f64 && week >= EXTRA_START {
                3
            } else {
                2
            };
            for index in 0..count {
                // Alternately left and right, and an extra stem, taken on
                // later, to the side its plant's place along the row picks.
                let (side, more, age) = match index {
                    0 => (1.0, 0.0, week),
                    1 => (-1.0, 0.0, week),
                    _ => (
                        if plant % 2 == 0 { 1.0 } else { -1.0 },
                        EXTRA_RUN,
                        week - EXTRA_START,
                    ),
                };
                let salted = salt(at * MAX_STEMS as u64 + index as u64);
                let stem = grow(
                    Vec3::new(x, 0.0, root_z),
                    side,
                    age,
                    more,
                    params.turned_to_path,
                    live,
                    &mut Rng::new(scene.seed ^ STEM_STREAM ^ salted),
                    &mut Rng::new(scene.seed ^ LIE_STREAM ^ salted),
                    &mut Rng::new(scene.seed ^ SURFACE_STREAM ^ salted),
                );
                stems.push((entity, plant, index, salted, stem));
            }
        }
    }

    // Each stem's tube, swollen at its joints and with their stubs on it,
    // meshed as finely as `detail` says, and its twine, grown in parallel and
    // registered serially: `Assets<Mesh>` takes one writer.
    let tolerance = TOLERANCE / params.detail.max(MIN_DETAIL) as f64;
    let meshes = par_map(
        &stems,
        |_, (_, plant, _, salted, stem)| -> Result<(Mesh, Mesh)> {
            let origin = Vec3::new(layout.plants[*plant], 0.0, root_z);
            let thickest = stem.radii.iter().copied().fold(0.0, f32::max) as f64;
            let sides = sides_within(thickest, tolerance).max(MIN_SIDES);
            let ridges = Bark::new(
                &mut Rng::new(scene.seed ^ RIDGE_STREAM ^ salted),
                sides,
                RIDGES,
            );
            let (bulges, collars) = swellings(&stem.joints);
            let mut tube = vec![strand_mesh(&Strand {
                bulges,
                collars,
                tolerance: Some(tolerance),
                ..strand(&stem.points, &stem.radii, origin, sides, ridges)
            })?];
            for joint in stem.joints.iter().filter(|joint| joint.stub > 0.0) {
                tube.push(strand_mesh(&stub(joint, origin, sides))?);
            }
            let twine = vec![TWINE_RADIUS; stem.twine.len()];
            Ok((
                merge_meshes(&tube).to_mesh(),
                strand_mesh(&strand(
                    &stem.twine,
                    &twine,
                    origin,
                    TWINE_SIDES,
                    Bark::none(),
                ))?
                .to_mesh(),
            ))
        },
    );

    let (green, white) = (
        STEM_RESPONSE.surface(srgb(STEM_COLOR)),
        PLASTIC.surface(srgb(TWINE_COLOR)),
    );
    let mut plant_prim = Entity::PLACEHOLDER;
    let (mut order, mut leaf_order) = (0u64, 0u64);
    for (number, ((row, plant, index, _, stem), meshes)) in stems.iter().zip(meshes).enumerate() {
        let (tube, twine) = meshes?;
        let origin = Vec3::new(layout.plants[*plant], 0.0, root_z);
        // A plant's stems come one after another, the first opening its prim.
        if *index == 0 {
            plant_prim = commands
                .spawn((
                    Owned,
                    Name::new(format!("Plant_{plant:03}")),
                    Transform::from_translation(origin),
                    Visibility::default(),
                    ChildOf(*row),
                ))
                .id();
        }
        let tube = library.part(TUBE, number, tube, green);
        let twine = library.part(TWINE, number, twine, white);
        commands
            .spawn((
                Name::new(format!("Stem_{index}")),
                Transform::IDENTITY,
                Visibility::default(),
                ChildOf(plant_prim),
            ))
            .with_children(|parent| {
                parent.spawn((Name::new("Tube"), tube));
                for (k, (radius, from, to)) in colliders(stem).into_iter().enumerate() {
                    // Straight across the whole span, its caps past both
                    // ends, so it overlaps the next one over their joint.
                    let out = (to - from).normalize_or_zero() * radius;
                    parent.spawn((
                        Name::new(format!("{COLLISION}_{k:02}")),
                        capsule_between(radius, from - out - origin, to + out - origin),
                    ));
                }
                parent.spawn((
                    Name::new("Hook"),
                    hook.clone(),
                    Transform::from_translation(stem.hook - origin),
                ));
                parent.spawn((Name::new("Twine"), twine));
                // The leaves kept, the lowest first. Each draws off its stem
                // and its rank from the head, so keeping more or fewer leaves
                // re-rolls none of the rest; a stem has fewer than 256 nodes.
                let kept = stem.kept;
                for (rank, (at, yaw)) in stem.nodes[..kept].iter().enumerate().rev() {
                    leaf_order += 1;
                    let seed = scene.seed ^ LEAF_STREAM ^ salt((number as u64) << 8 | rank as u64);
                    parent.spawn((
                        Name::new(format!("Leaf_{:02}", kept - 1 - rank)),
                        Transform::from_translation(*at - origin)
                            .with_rotation(Quat::from_rotation_z(*yaw)),
                        Visibility::default(),
                        LeafConfig::new(&leaf_params, seed, rank as u32),
                        Order(leaf_order),
                    ));
                }
                for (k, (at, yaw, node)) in stem.trusses.iter().enumerate() {
                    order += 1;
                    parent.spawn((
                        Name::new(format!("Truss_{k:02}")),
                        Transform::from_translation(*at - origin)
                            .with_rotation(Quat::from_rotation_z(*yaw)),
                        Visibility::default(),
                        TrussConfig::new(&truss_params, scene.seed, order, *node),
                        Order(order),
                    ));
                }
            });
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeMap;

    use super::*;
    use crate::elements::fruit::{FruitConfig, FruitParams, Hung, ripeness};
    use crate::elements::{GreenhouseParams, grown, in_scene, small};
    use misina_lab::scene::{Collider, UsdReference};
    use misina_lab::testing::{named_children, prim_path};

    /// How far `p` is from the polyline through `axis`.
    fn off_axis(p: Vec3, axis: &[Vec3]) -> f32 {
        axis.windows(2)
            .map(|w| {
                let d = w[1] - w[0];
                let t = ((p - w[0]).dot(d) / d.length_squared()).clamp(0.0, 1.0);
                p.distance(w[0] + d * t)
            })
            .fold(f32::MAX, f32::min)
    }

    /// On every stem in full production the lowest truss hangs less than a
    /// week's growth above the harvest height, give or take how much faster
    /// the stem grows, and the rest a week's growth apart above it, 34 cm at
    /// the defaults give or take the same, and every truss carries its
    /// peduncle and the two frames a gripper takes it by. The stems are out
    /// of step, so their lowest trusses hang over most of a week's growth. A
    /// leaf hangs at each of the top `leaves_kept` nodes down from the newest,
    /// on the stem's axis half a node off the trusses, a childless prim that
    /// references its mesh, and none in the harvest band round the two
    /// trusses nearest their picking. The stem's colliders chain straight
    /// through every joint between them. At the defaults with an extra stem
    /// on half the plants, on a low gutter under a high wire, and on a stem
    /// too short for the leaves asked for.
    #[test]
    fn trusses_and_leaves_hang_at_the_stems_nodes() {
        let extra: fn(&mut GreenhouseParams) = |p| p.stem.extra_stems = 0.5;
        let tall: fn(&mut GreenhouseParams) = |p| {
            (p.gutter.height, p.wire.height) = (0.3, 5.5);
            (p.stem.harvest_height, p.stem.trusses) = (0.8, 12);
        };
        let short: fn(&mut GreenhouseParams) = |p| (p.stem.trusses, p.stem.leaves_kept) = (6, 21);
        for (edit, growth) in [(extra, 0.32..=0.36), (tall, 0.0..=1.0), (short, 0.0..=1.0)] {
            let mut params = small();
            edit(&mut params);
            let stem = params.stem.clone();
            let mut app = grown(params);
            let world = app.world_mut();
            let trusses: Vec<Entity> = world
                .query_filtered::<Entity, With<TrussConfig>>()
                .iter(world)
                .collect();
            let leaves: Vec<Entity> = world
                .query_filtered::<Entity, With<LeafConfig>>()
                .iter(world)
                .collect();
            // Each stem's trusses, then its leaves, by path and where each
            // hangs.
            let mut stems: BTreeMap<Entity, [Vec<(String, Vec3)>; 2]> = BTreeMap::new();
            for leaf in leaves {
                let path = prim_path(world, leaf);
                let at = world.entity(leaf);
                assert!(
                    at.get::<Children>().is_none() && at.contains::<UsdReference>(),
                    "{path} is not a childless geometry prim"
                );
                let on = at.get::<ChildOf>().unwrap().parent();
                stems.entry(on).or_default()[1].push((path, in_scene(world, leaf).translation));
            }
            for truss in trusses {
                let path = prim_path(world, truss);
                let held: Vec<String> = named_children(world, truss)
                    .into_iter()
                    .map(|(name, _)| name)
                    .collect();
                for frame in ["Peduncle", "Cut", "Bend"] {
                    assert!(
                        held.iter().any(|name| name == frame),
                        "{path} has no {frame}"
                    );
                }
                let on = world.entity(truss).get::<ChildOf>().unwrap().parent();
                let node = in_scene(world, truss).translation;
                stems.entry(on).or_default()[0].push((path, node));
            }
            assert!(!stems.is_empty());
            // How far into its week each stem is, as the share of a week's
            // growth its lowest truss hangs below where it would at the
            // week's start.
            let mut phases = Vec::new();
            for (on, [mut trusses, leaves]) in stems {
                trusses.sort_by(|a, b| a.0.cmp(&b.0));
                let (path, lowest) = &trusses[0];
                let top = trusses[trusses.len() - 1].1;
                assert_eq!(trusses.len(), stem.trusses as usize, "{path}");
                let per = lowest.distance(top) / (trusses.len() - 1) as f32;
                assert!(growth.contains(&per), "{path}: {per} m of stem a truss");
                let faster = GROWTH_JITTER as f32 * per * stem.trusses as f32;
                assert!(
                    (stem.harvest_height - faster..stem.harvest_height + per + faster)
                        .contains(&lowest.z),
                    "{path} hangs at {}",
                    lowest.z
                );

                // The colliders chain straight through every joint: each
                // one's straight part ends where the next one's begins.
                let capsules: Vec<(Vec3, Vec3)> = named_children(world, on)
                    .into_iter()
                    .filter(|(name, _)| name.starts_with(COLLISION))
                    .map(|(_, capsule)| {
                        let at = world.entity(capsule);
                        let (place, shape) = (
                            at.get::<Transform>().unwrap(),
                            at.get::<Collider>().unwrap().0,
                        );
                        let half = place.rotation * Vec3::Z * shape.height / 2.0;
                        (place.translation - half, place.translation + half)
                    })
                    .collect();
                assert!(capsules.len() > 1, "{path}: {} colliders", capsules.len());
                for pair in capsules.windows(2) {
                    assert!(
                        pair[0].1.distance(pair[1].0) < 1e-4,
                        "{path}: a neck at {}",
                        pair[0].1
                    );
                }

                // The axis through the trusses up to the head, under the hook.
                let hook = named_children(world, on)
                    .into_iter()
                    .find(|(name, _)| name == "Hook")
                    .unwrap();
                let head = in_scene(world, hook.1).translation - Vec3::Z * HOOK_LENGTH;
                let axis: Vec<Vec3> = trusses.iter().map(|(_, at)| *at).chain([head]).collect();
                let nodes = NODES_PER_TRUSS * stem.trusses;
                let node = (top.z - lowest.z) / (nodes - NODES_PER_TRUSS) as f32;
                phases.push(
                    (head.z - lowest.z) / node / NODES_PER_TRUSS as f32 - (stem.trusses - 1) as f32,
                );
                let band = lowest.z + (BARE_TRUSSES * NODES_PER_TRUSS) as f32 * node;
                // The newest node, half a node to a node and a half below the head.
                let newest = ((head.z - lowest.z) / node - 1.0 + 1e-3).floor() as u32;
                let mut kept: Vec<u32> = leaves
                    .iter()
                    .map(|(path, at)| {
                        let n = (at.z - lowest.z) / node - 0.5;
                        assert!((n - n.round()).abs() < 1e-3, "{path} is off its node: {n}");
                        assert!(at.z > band, "{path} hangs in the harvest band");
                        let off = off_axis(*at, &axis);
                        assert!(
                            off < 2e-3 + NODE_KINK as f32,
                            "{path} is {off} m off the stem"
                        );
                        n.round() as u32
                    })
                    .collect();
                kept.sort();
                let count = stem
                    .leaves_kept
                    .min(newest + 1 - BARE_TRUSSES * NODES_PER_TRUSS);
                assert_eq!(
                    kept,
                    (newest + 1 - count..=newest).collect::<Vec<_>>(),
                    "{path}: a leaf at each of the top {count} nodes"
                );
            }
            let (first, last) = phases
                .iter()
                .fold((f32::MAX, f32::MIN), |(lo, hi), p| (lo.min(*p), hi.max(*p)));
            assert!(last - first > 0.5, "in step: {first} to {last} of a week");
        }
    }

    /// A stem's head, `HEAD_AT` below its tip, is as thick as it was drawn
    /// about `head_diameter`, however its girth undulates there.
    #[test]
    fn a_head_is_as_thick_as_its_diameter_says() {
        let live = Live {
            rim: 0.56,
            head: 4.28,
            harvest: 1.2,
            trusses: 9,
            development_spread: 0.0,
            spread: 0.35,
            head_diameter: 0.01,
            ends: (0.2, 39.8),
            flexible_below: 0.0,
            leaves_kept: 16,
        };
        let root = Vec3::new(1.0, 0.0, 0.64);
        for seed in 0..32 {
            // `rng` draws how thick the head is first.
            let drawn = Rng::new(seed).range(1.0 - HEAD_JITTER, 1.0 + HEAD_JITTER) as f32;
            let stem = grow(
                root,
                1.0,
                12.0,
                0.0,
                0.8,
                live,
                &mut Rng::new(seed),
                &mut Rng::new(seed + 100),
                &mut Rng::new(seed + 200),
            );
            // The radius `HEAD_AT` down from the tip, between the control
            // points either side of it, by their length back from the tip.
            let back: Vec<(f32, f32)> = stem
                .points
                .iter()
                .zip(&stem.radii)
                .rev()
                .scan(
                    (0.0, *stem.points.last().unwrap()),
                    |(length, last), (p, r)| {
                        *length += last.distance(*p);
                        *last = *p;
                        Some((*length, *r))
                    },
                )
                .collect();
            let k = back.iter().position(|(d, _)| *d >= HEAD_AT).unwrap();
            let ((d0, r0), (d1, r1)) = (back[k - 1], back[k]);
            let head = 2.0 * (r0 + (r1 - r0) * (HEAD_AT - d0) / (d1 - d0));
            let off = head / (live.head_diameter * drawn) - 1.0;
            assert!(off.abs() < 0.01, "seed {seed}: head {head} m, {off} off");
        }
    }

    /// A stem stands on its twine with green fruit only until its head
    /// reaches the wire, about week 9, when its lowest truss colours. From
    /// then on it is lowered: its hook slides along the wire about a week's
    /// growth a week, and its lowest truss hangs less than a week's growth
    /// above the harvest height whatever the week, give or take how much
    /// faster it grows.
    #[test]
    fn a_stem_grows_up_its_twine_and_is_lowered() {
        let live = Live {
            rim: 0.56,
            head: 4.28,
            harvest: 1.2,
            trusses: 9,
            development_spread: 0.0,
            spread: 0.35,
            head_diameter: 0.01,
            ends: (0.2, 39.8),
            flexible_below: 0.0,
            leaves_kept: 16,
        };
        let weekly = (live.head - live.harvest) / live.trusses as f32;
        let faster = GROWTH_JITTER as f32 * (live.head - live.harvest);
        let root = Vec3::new(1.0, 0.0, 0.64);
        let stem = |age: f32| {
            grow(
                root,
                1.0,
                age,
                0.0,
                0.8,
                live,
                &mut Rng::new(3),
                &mut Rng::new(4),
                &mut Rng::new(7),
            )
        };
        // How much of the fruit at a truss's base has coloured.
        let coloured = |(_, _, node): &(Vec3, f32, Node)| {
            let hung = Hung {
                rank: node.rank,
                along: 0.0,
                growth: node.growth,
            };
            ripeness(FruitConfig::new(&FruitParams::default(), 0, 0.06, hung).stage)
        };

        for age in [0.0, 3.0, 6.5] {
            let young = stem(age);
            let head = young.points.last().unwrap().z;
            assert!(
                young
                    .points
                    .iter()
                    .all(|p| (p.x - root.x).abs() < (HOOK_JITTER + KINK.1 + NODE_KINK) as f32 + BOW),
                "week {age}: lowered"
            );
            assert!(head < live.head - 0.5, "week {age}: the head at {head}");
            assert!(
                young.trusses.iter().all(|truss| coloured(truss) == 0.0),
                "week {age}: a truss colours"
            );
        }
        for age in [10.0, 11.5, 12.0] {
            let grown = stem(age);
            let head = grown.points.last().unwrap().z;
            assert!(
                (head - live.head).abs() < 1e-4,
                "week {age}: the head at {head}"
            );
            assert!(coloured(&grown.trusses[0]) > 0.0, "week {age}: all green");
        }
        let hooks = [20.0, 20.5, 21.0].map(|age| {
            let lowered = stem(age);
            let lowest = lowered.trusses[0].0.z;
            assert!(
                (live.harvest - faster..live.harvest + weekly + 2.0 * faster).contains(&lowest),
                "week {age}: the lowest truss at {lowest}"
            );
            lowered.hook.x
        });
        let slid = hooks[2] - hooks[0];
        let (g, l) = (GROWTH_JITTER as f32, LOWERED_JITTER as f32);
        assert!(
            (weekly * (1.0 - g) * (1.0 - l)..=weekly * (1.0 + g) * (1.0 + l)).contains(&slid),
            "slid {slid} m in a week, grew {weekly} m"
        );
    }

    /// A lowered stem's colliders run from its take-off to its head along
    /// its axis, end to end, each as thick as the stem and straight within
    /// `STRAY` of every point of the axis it spans, and none below the
    /// gutter's rim: its run along the gutter collides not at all.
    #[test]
    fn a_stem_collides_from_its_take_off_up() {
        let live = Live {
            rim: 0.56,
            head: 4.28,
            harvest: 1.2,
            trusses: 9,
            development_spread: 0.0,
            spread: 0.35,
            head_diameter: 0.01,
            ends: (0.2, 9.8),
            flexible_below: 0.0,
            leaves_kept: 16,
        };
        let stem = grow(
            Vec3::new(9.0, 0.0, 0.64),
            1.0,
            30.0,
            0.0,
            0.8,
            live,
            &mut Rng::new(5),
            &mut Rng::new(6),
            &mut Rng::new(8),
        );
        let spans = colliders(&stem);
        assert!(stem.take_off > 2, "not lowered");
        assert_eq!(spans[0].1, stem.points[stem.take_off]);
        assert_eq!(spans.last().unwrap().2, *stem.points.last().unwrap());
        for pair in spans.windows(2) {
            assert_eq!(pair[0].2, pair[1].1);
        }
        let thickest = stem.radii.iter().copied().fold(0.0, f32::max);
        for (radius, from, to) in spans {
            assert!(radius > 0.0 && radius <= thickest);
            assert!(
                from.z.min(to.z) > live.rim,
                "{from} to {to} dips below the rim"
            );
            let first = stem.points.iter().position(|p| *p == from).unwrap();
            let last = stem.points.iter().position(|p| *p == to).unwrap();
            for p in &stem.points[first..=last] {
                assert!(off_axis(*p, &[from, to]) < STRAY, "{p} is off its capsule");
            }
        }
    }

    /// A place round the gutter maps onto the left side running +X, the far
    /// turn, the right side running -X and the head turn, and every place a
    /// run may end at is beside the gutter.
    #[test]
    fn the_runs_go_round_the_gutter() {
        let round = Round {
            from: 0.2,
            to: 9.8,
            out: 0.13,
        };
        let (left, along) = round.at(round.start(3.0, 1.0));
        assert!(left.distance(Vec2::new(3.0, 0.13)) < 1e-5 && along == Vec2::X);
        let (right, along) = round.at(round.start(3.0, -1.0));
        assert!(right.distance(Vec2::new(3.0, -0.13)) < 1e-4 && along == -Vec2::X);
        for u in (0..400).map(|i| i as f32 * 0.07) {
            let (at, _) = round.at(round.beside(u));
            assert!(
                (at.y.abs() - round.out).abs() < 1e-4 && (round.from..=round.to).contains(&at.x),
                "{u}: {at}"
            );
        }
        // Round the far end from the left lands on the right.
        let (back, _) = round.at(round.beside(round.start(9.0, 1.0) + 2.0));
        assert!(
            back.y < 0.0 && (back.x - (9.8 - 1.2 + round.turn())).abs() < 1e-3,
            "{back}"
        );
    }
}

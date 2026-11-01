//! Strawberry — a crown of toothed trifoliate leaves on long petioles, every
//! berry hanging from a stalk of its own arching out from the crown, a rod
//! when the stems are flexible, and runners sent out over and along the
//! soil with a plantlet at the end of each.

use std::f32::consts::{FRAC_PI_2, TAU};

use super::prelude::*;

/// Foliage: a grown leaf's green and a young one's, a stalk's, and a
/// berry's skin unripe and ripe.
const LEAF: u32 = 0x2F6B2A;
const YOUNG: u32 = 0x6FAE4E;
const STEM: u32 = 0x7FA860;
const UNRIPE: u32 = 0xD8E4B0;
const RIPE: u32 = 0xC0122A;

/// A grown crown's radius, in meters.
const CROWN: f32 = 0.015;

/// A leaf: the petiole as a fraction of its length, a leaflet's width as a
/// fraction of its own length, its teeth to an edge, how far the petioles
/// rise off level from the outermost leaf to the innermost, and how far the
/// leaf turns down from there to its tip, in radians.
const PETIOLE: f32 = 0.55;
const WIDTH: f32 = 0.8;
const TEETH: u32 = 7;
const RISE: (f32, f32) = (0.6, 1.3);
const DROOP: f32 = 0.8;

/// A fruit stalk: its length as a fraction of the leaves', its radius at
/// the crown, in meters, how much it thins to the tip, how far it rises off
/// level at the crown and how far it turns down on its way, in radians; and
/// a berry's diameter as a fraction of its length.
const STALK: f32 = 0.7;
const STALK_RADIUS: f32 = 0.0025;
const STALK_TAPER: f32 = 0.8;
const STALK_RISE: f32 = 1.0;
const STALK_SAG: f32 = 1.4;
const FRUIT_ASPECT: f32 = 0.8;

/// Berries: the week the first sets, the weeks from one setting to the
/// next, and the weeks one takes from setting to ripe.
const FRUITS_FROM: f32 = 6.5;
const SET_GAP: f32 = 1.0;
const RIPENING: f32 = 2.0;

/// Runners: the week they start and the weeks one takes to its full length,
/// that length as a fraction of the leaves', its radius, in meters, how far
/// it rises off level at the crown and how far it turns down on its way, in
/// radians, more than twice the steepest rise so every runner lands before
/// its tip; and the plantlet at its tip: its leaves, their size as a
/// fraction of the mother's, and how far they rise off level.
const RUNNERS_FROM: f32 = 5.0;
const RUNNER_GROWING: f32 = 3.0;
const RUNNER: f32 = 1.8;
const RUNNER_RADIUS: f32 = 0.002;
const RUNNER_RISE: f32 = 0.5;
const RUNNER_SAG: f32 = 2.0;
const PLANTLET_LEAVES: u32 = 3;
const PLANTLET: f32 = 0.4;
const PLANTLET_RISE: f32 = 0.9;

/// How far each leaf strays from its rank's: in length as a fraction, in
/// elevation and azimuth in radians; and each stalk and runner in its rise.
const LENGTH_JITTER: f64 = 0.1;
const ELEVATION_JITTER: f64 = 0.12;
const AZIMUTH_JITTER: f64 = 0.2;
const RISE_JITTER: f64 = 0.25;

/// A stalk's stream is numbered from this by its berry and a runner's from
/// the second, apart whatever their counts; a plantlet's leaves are
/// numbered from the third times its runner's number from one, past any
/// rosette's leaves, so a leaf more on the crown renumbers none of them.
const STALK_STREAM: u64 = 1000;
const RUNNER_STREAM: u64 = 2000;
const PLANTLET_FIRST: u64 = 1000;

species! {
    /// Strawberry: a crown of toothed trifoliate leaves on long petioles,
    /// every berry hanging from a stalk of its own arching out from the
    /// crown and cut when ripe, and runners sent out along the soil, grown
    /// to harvest over eleven weeks.
    pub struct StrawberryParams for Strawberry as "strawberry" {
        /// Leaves on a harvest-ready plant.
        #[reflect(@Slider { min: 4.0, max: 14.0, step: 1.0 })]
        pub leaves: u32 = 8,
        /// Length of a grown leaf, petiole and leaflets, in meters.
        #[reflect(@Slider { min: 0.1, max: 0.35, step: 0.01 })]
        pub leaf_length: f32 = 0.2,
        /// Berries a harvest-ready plant bears, each on a stalk of its own,
        /// set a week apart once the plant is older than six and a half
        /// weeks.
        #[reflect(@Slider { min: 0.0, max: 10.0, step: 1.0 })]
        pub fruits: u32 = 4,
        /// Length of a ripe berry, in meters.
        #[reflect(@Slider { min: 0.015, max: 0.045, step: 0.005 })]
        pub fruit_length: f32 = 0.03,
        /// Runners a plant older than five weeks sends out along the soil.
        #[reflect(@Slider { min: 0.0, max: 3.0, step: 1.0 })]
        pub runners: u32 = 1,
    }
    common { share: 0.25, spacing: 0.15, sown_week: 0.0, bed: 0, variations: 12 }
}

impl Recipe for Strawberry {
    const HARVEST: f32 = 11.0;

    /// Draw order from the plant's stream: the rosette's phase, then for
    /// every leaf from the outermost in its length, elevation and azimuth,
    /// so a leaf more re-rolls none before it; then three per axis, which
    /// an arc leaves unused. The runners are grown before the stalks: their
    /// count is fixed once they start, before the first berry sets, so a
    /// berry more renumbers no axis. Each stalk's and each runner's own
    /// draws come from a stream of its own.
    fn grow(p: &StrawberryParams, plant: &PlantConfig, b: &mut Builder) {
        let stage = plant.grown(Self::HARVEST);
        let size = (stage * plant.vigour).max(0.1);
        let crown = b.crown(CROWN * size.max(0.2), 0.0);
        let tint = mix(srgb(LEAF), srgb(YOUNG), 1.0 - stage);
        let leaves = (p.leaves as f32 * stage).round().max(2.0) as u32;
        let rosette = Rosette::golden(leaves).phased(b.rng());
        for mut node in rosette.nodes() {
            let rng = b.rng();
            let length = 1.0 + rng.range(-LENGTH_JITTER, LENGTH_JITTER) as f32;
            let elevation = rng.range(-ELEVATION_JITTER, ELEVATION_JITTER) as f32;
            node.azimuth += rng.range(-AZIMUTH_JITTER, AZIMUTH_JITTER) as f32;
            let whole = p.leaf_length * size * node.size() * length;
            b.leaf(
                crown.anchor(&node, lerp(RISE.0, RISE.1, node.inner()) + elevation),
                leaf(whole, tint),
            );
        }

        // Runners from the crown's foot, each humping over and down onto the
        // soil and along it, a plantlet of a few small leaves at its tip.
        let running = smooth((plant.age - RUNNERS_FROM) / RUNNER_GROWING);
        let runners = if running > 0.0 { p.runners } else { 0 };
        for k in 0..runners {
            let mut rng = b.stream(RUNNER_STREAM + k as u64);
            let azimuth = rng.range(0.0, TAU as f64) as f32;
            let rise = RUNNER_RISE + rng.range(-RISE_JITTER, RISE_JITTER) as f32;
            let foot = (Vec2::from_angle(azimuth) * crown.radius).extend(crown.base);
            let runner = b.axis(
                foot,
                Axis::arc(
                    RUNNER * p.leaf_length * plant.vigour * running,
                    RUNNER_RADIUS,
                )
                .heading(azimuth, rise)
                .rise(-RUNNER_SAG)
                .grounded()
                .tint(srgb(STEM)),
            );
            let first = (k as u64 + 1) * PLANTLET_FIRST;
            let whole = PLANTLET * p.leaf_length * plant.vigour * running;
            for i in 0..PLANTLET_LEAVES {
                let node = AxisNode {
                    rank: i,
                    count: PLANTLET_LEAVES,
                    station: runner.length(),
                    azimuth: azimuth + TAU * i as f32 / PLANTLET_LEAVES as f32,
                };
                b.leaf_at(
                    first + i as u64,
                    runner.anchor(&node, PLANTLET_RISE),
                    leaf(whole, tint),
                );
            }
        }

        // A stalk per berry, the berries set a week apart: each arches out
        // from the crown's side and turns down under its berry, which hangs
        // on from the tip along the stalk's line, grows out as it ripens,
        // and lies out on the soil instead of hanging into it where the
        // stalk reaches down that far.
        let set = |k: u32| FRUITS_FROM + k as f32 * SET_GAP;
        for k in (0..p.fruits).take_while(|&k| plant.age > set(k)) {
            let mut rng = b.stream(STALK_STREAM + k as u64);
            let azimuth = rng.range(0.0, TAU as f64) as f32;
            let rise = STALK_RISE + rng.range(-RISE_JITTER, RISE_JITTER) as f32;
            let ripeness = ((plant.age - set(k)) / RIPENING).clamp(0.0, 1.0);
            let foot = (Vec2::from_angle(azimuth) * crown.radius)
                .extend(crown.base + crown.height() / 2.0);
            let stalk = b.axis(
                foot,
                Axis::arc(STALK * p.leaf_length * size, STALK_RADIUS * size.max(0.5))
                    .heading(azimuth, rise)
                    .rise(-STALK_SAG)
                    .taper(STALK_TAPER)
                    .tint(srgb(STEM))
                    .bends(),
            );
            let tip = AxisNode {
                rank: 0,
                count: 1,
                station: stalk.length(),
                azimuth,
            };
            let length = p.fruit_length * plant.vigour * (0.3 + 0.7 * ripeness);
            let hanging = FRAC_PI_2 + rise - STALK_SAG;
            let tilt = hanging.max((stalk.tip().z / length).min(1.0).acos());
            b.fruit_at(
                k as u64,
                stalk.hang(&tip, azimuth, tilt),
                FruitConfig::strawberry(FRUIT_ASPECT * length, length)
                    .ripeness(ripeness)
                    .tint(mix(srgb(UNRIPE), srgb(RIPE), ripeness)),
            );
        }
    }
}

/// A leaf `whole` long in `tint`: toothed trifoliate leaflets past a
/// petiole over half its length, turned down toward its tip.
fn leaf(whole: f32, tint: [f32; 3]) -> LeafConfig {
    LeafConfig::trifoliate(whole, WIDTH)
        .petiole(PETIOLE * whole)
        .teeth(TEETH)
        .droop(DROOP)
        .tint(tint)
}

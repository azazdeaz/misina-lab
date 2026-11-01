//! Zucchini — a stout, leaning stem under a mound of giant palmate leaves on
//! long hollow petioles, the fruit lying low in the canopy on short stalks.

use std::f32::consts::FRAC_PI_2;

use super::prelude::*;

/// Foliage: a grown blade's green and a young one's, the stem's pale green,
/// and a fruit's dark green.
const LEAF: u32 = 0x3E7A33;
const YOUNG: u32 = 0x7DB356;
const STEM: u32 = 0x8FAE6C;
const FRUIT: u32 = 0x24501E;

/// The stem: a grown stem's radius at its foot, in meters, how much it
/// thins to the tip, how far it leans, in radians, and its internode: a leaf
/// every 1.2 cm.
const RADIUS: f32 = 0.025;
const TAPER: f32 = 0.6;
const LEAN: f32 = 0.3;
const INTERNODE: f32 = 0.012;

/// A leaf: the petiole as a fraction of its length, the blade's width as a
/// fraction of its own length, how far the petioles rise off level from the
/// oldest leaf to the youngest, in radians, how far the blade turns down
/// where the petiole ends, in radians, and over how many nodes from the tip
/// a leaf grows to full size.
const PETIOLE: f32 = 0.55;
const WIDTH: f32 = 1.1;
const RISE: (f32, f32) = (0.45, 1.25);
const HINGE: f32 = 0.7;
const GROWING: f32 = 4.0;

/// Fruit: a ripe fruit's diameter as a fraction of its length, the lowest
/// node that bears one and how many nodes up from it do, the week the first
/// sets and the weeks between one and the next, the weeks one takes from
/// setting to ripe, how far it turns round its node's azimuth, and how far
/// it rises off level, in radians.
const FRUIT_ASPECT: f32 = 0.22;
const FIRST_FRUIT: u32 = 3;
const FRUITS: u32 = 3;
const FRUITS_FROM: f32 = 5.0;
const SET_GAP: f32 = 1.0;
const RIPENING: f32 = 2.0;
const FRUIT_SWING: f32 = 0.9;
const FRUIT_RISE: f32 = 0.2;

/// How far each leaf strays from its node's: in length as a fraction, in
/// elevation and azimuth in radians; and each fruit in azimuth.
const LENGTH_JITTER: f64 = 0.1;
const ELEVATION_JITTER: f64 = 0.12;
const AZIMUTH_JITTER: f64 = 0.25;
const FRUIT_JITTER: f64 = 0.3;

species! {
    /// Zucchini: a stout, leaning stem under a mound of giant palmate leaves
    /// on long hollow petioles, the fruit lying low in the canopy and cut as
    /// they ripen, grown to harvest over eight weeks.
    pub struct ZucchiniParams for Zucchini as "zucchini" {
        /// Height of a harvest-ready stem, in meters; the leaves stand well
        /// above it.
        #[reflect(@Slider { min: 0.05, max: 0.3, step: 0.01 })]
        pub height: f32 = 0.14,
        /// Length of a grown leaf, petiole and blade, in meters.
        #[reflect(@Slider { min: 0.3, max: 1.0, step: 0.05 })]
        pub leaf_length: f32 = 0.65,
        /// Length of a ripe fruit, in meters.
        #[reflect(@Slider { min: 0.1, max: 0.35, step: 0.01 })]
        pub fruit_length: f32 = 0.2,
    }
    common { share: 0.25, spacing: 0.9, sown_week: 0.0, bed: 0, variations: 12 }
}

impl Recipe for Zucchini {
    const HARVEST: f32 = 8.0;

    /// Draw order from the plant's stream: the stem's lean and bow and the
    /// spiral's phase. Every node's jitter, leaf and fruit alike, comes from
    /// the node's own stream, so a stem a node longer re-rolls nothing.
    fn grow(p: &ZucchiniParams, plant: &PlantConfig, b: &mut Builder) {
        let stage = plant.grown(Self::HARVEST);
        let size = (stage * plant.vigour).max(0.05);
        let stem = b.axis(
            Vec3::ZERO,
            Axis::upright(p.height * size, RADIUS * size.max(0.2))
                .taper(TAPER)
                .lean(LEAN)
                .tint(srgb(STEM)),
        );
        let tint = mix(srgb(LEAF), srgb(YOUNG), 1.0 - stage);
        for node in stem.nodes(&Spiral::golden(INTERNODE).phased(b.rng())) {
            let index = node.rank as u64;
            let mut rng = b.stream(index);
            let length = 1.0 + rng.range(-LENGTH_JITTER, LENGTH_JITTER) as f32;
            let elevation = rng.range(-ELEVATION_JITTER, ELEVATION_JITTER) as f32;
            let azimuth = node.azimuth + rng.range(-AZIMUTH_JITTER, AZIMUTH_JITTER) as f32;
            let swing = rng.range(-FRUIT_JITTER, FRUIT_JITTER) as f32;

            // Leaves grow to full size over the top few nodes; the oldest
            // lie out near level and the youngest stand steep.
            let from_tip = node.from_tip() as f32;
            let grown = smooth((from_tip + 1.0) / GROWING);
            let settled = from_tip / (node.count.max(2) - 1) as f32;
            let whole = p.leaf_length * size * (0.3 + 0.7 * grown) * length;
            let leaf = LeafConfig::lobed(whole, WIDTH)
                .petiole(PETIOLE * whole)
                .droop(HINGE)
                .tint(tint);
            let turned = AxisNode { azimuth, ..node };
            b.leaf_at(
                index,
                stem.anchor(&turned, lerp(RISE.1, RISE.0, settled) + elevation),
                leaf,
            );

            // One fruit to a bearing node, set a week apart from the lowest
            // up, growing out as it ripens, lying out from the stem a little
            // above level so it clears the soil.
            let set = FRUITS_FROM + node.rank.saturating_sub(FIRST_FRUIT) as f32 * SET_GAP;
            if (FIRST_FRUIT..FIRST_FRUIT + FRUITS).contains(&node.rank) && plant.age > set {
                let ripeness = ((plant.age - set) / RIPENING).clamp(0.0, 1.0);
                let length = p.fruit_length * plant.vigour * (0.2 + 0.8 * ripeness);
                b.fruit_at(
                    index,
                    stem.hang(&node, azimuth + FRUIT_SWING + swing, FRAC_PI_2 + FRUIT_RISE),
                    FruitConfig::berry(FRUIT_ASPECT * length, length)
                        .ripeness(ripeness)
                        .tint(srgb(FRUIT)),
                );
            }
        }
    }
}

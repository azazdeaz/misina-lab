//! Carrot — a rosette of finely divided leaves over a taproot, whose orange
//! shoulder shows at the soil.

use super::prelude::*;

/// Foliage: a grown leaf's green and a young one's; the root's skin from
/// pale to full orange as it ripens, and the green its shoulder turns above
/// the soil.
const LEAF: u32 = 0x4A8A3A;
const YOUNG: u32 = 0x86B85C;
const PALE: u32 = 0xE8A060;
const ORANGE: u32 = 0xE8702A;
const GREENED: u32 = 0x8FA050;

/// A leaf: its petiole and each leaflet as fractions of its length, its
/// pairs of leaflets, how far it arches over, in radians, and how far the
/// leaves rise off level from the outermost to the innermost.
const PETIOLE: f32 = 0.3;
const LEAFLET: f32 = 0.14;
const PAIRS: u32 = 8;
const DROOP: f32 = 0.7;
const RISE: (f32, f32) = (0.5, 1.2);

/// How much of the root's length stands proud of the soil at harvest, and
/// how far the crown sinks into the root's top.
const EXPOSED: f32 = 0.1;
const SUNK: f32 = 0.3;

/// How far each leaf strays from its rank's: in length as a fraction, in
/// elevation and azimuth in radians.
const LENGTH_JITTER: f64 = 0.12;
const ELEVATION_JITTER: f64 = 0.12;
const AZIMUTH_JITTER: f64 = 0.2;

species! {
    /// Carrot: a rosette of finely divided leaves over a taproot whose
    /// shoulder shows at the soil, pulled by its tops, grown to harvest over
    /// twelve weeks.
    pub struct CarrotParams for Carrot as "carrot" {
        /// Leaves on a harvest-ready plant.
        #[reflect(@Slider { min: 4.0, max: 16.0, step: 1.0 })]
        pub leaves: u32 = 9,
        /// Length of a grown leaf, in meters.
        #[reflect(@Slider { min: 0.15, max: 0.5, step: 0.01 })]
        pub leaf_length: f32 = 0.3,
        /// Length of a harvest-ready root, in meters.
        #[reflect(@Slider { min: 0.08, max: 0.3, step: 0.01 })]
        pub root_length: f32 = 0.18,
        /// Diameter of a harvest-ready root at its shoulder, in meters.
        #[reflect(@Slider { min: 0.015, max: 0.06, step: 0.005 })]
        pub root_diameter: f32 = 0.03,
    }
    common { share: 0.25, spacing: 0.075, sown_week: 0.0, bed: 0, variations: 12 }
}

impl Recipe for Carrot {
    const HARVEST: f32 = 12.0;

    /// Draw order: the rosette's phase, then for every leaf from the
    /// outermost in its length, elevation and azimuth.
    fn grow(p: &CarrotParams, plant: &PlantConfig, b: &mut Builder) {
        let stage = plant.grown(Self::HARVEST);
        let size = (stage * plant.vigour).max(0.1);
        // The root swells as it grows and pushes its shoulder up through the
        // soil; the crown sits in the shoulder. It ripens with its age, so it
        // is ready to pull at nine tenths of the way to harvest.
        let (length, diameter) = (p.root_length * size, p.root_diameter * size);
        let shoulder = length * EXPOSED * stage;
        let ripeness = (plant.age / Self::HARVEST).clamp(0.0, 1.0);
        b.fruit(
            Transform::from_xyz(0.0, 0.0, shoulder),
            FruitConfig::root(diameter, length)
                .ripeness(ripeness)
                .tint(mix(srgb(PALE), srgb(ORANGE), ripeness))
                .shoulder(srgb(GREENED)),
        );
        let radius = diameter / 2.0 * 0.8;
        let crown = b.crown(radius, (shoulder - SUNK * radius).max(0.0));
        let tint = mix(srgb(LEAF), srgb(YOUNG), 1.0 - stage);
        let leaves = (p.leaves as f32 * stage).round().max(2.0) as u32;
        let rosette = Rosette::golden(leaves).phased(b.rng());
        for mut node in rosette.nodes() {
            let rng = b.rng();
            let length = 1.0 + rng.range(-LENGTH_JITTER, LENGTH_JITTER) as f32;
            let elevation = rng.range(-ELEVATION_JITTER, ELEVATION_JITTER) as f32;
            node.azimuth += rng.range(-AZIMUTH_JITTER, AZIMUTH_JITTER) as f32;
            let inner = node.inner();
            let whole = p.leaf_length * size * node.size() * length;
            let config = LeafConfig::compound(whole, PAIRS, LEAFLET)
                .petiole(PETIOLE * whole)
                .droop(DROOP * (1.0 - 0.5 * inner))
                .tint(tint);
            b.leaf(
                crown.anchor(&node, lerp(RISE.0, RISE.1, inner) + elevation),
                config,
            );
        }
    }
}

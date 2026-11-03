//! Pole bean — a vine twining anticlockwise up the string from its bed's
//! trellis, a large trifoliate leaf at every node and pods hanging in twos
//! and threes from the upper nodes.

use super::prelude::*;

/// Foliage: a grown leaflet's green and a young one's, its veins' paler
/// green, the vine's, and a pod's.
const LEAF: u32 = 0x3E7A33;
const YOUNG: u32 = 0x7DB45A;
const VEIN: u32 = 0x86B85E;
const STEM: u32 = 0x5D8A3C;
const POD: u32 = 0x5F9A3A;

/// The vine: a grown vine's radius at its foot, in meters, how much it thins
/// to the tip, how far it climbs per turn round its string, how far it stays
/// off touching the string, and its internode. A rod's segments run straight
/// between their points and so cut inside the vine's turns; the gap keeps
/// the longest segment the scene allows from being authored into the
/// string, which would let the rod through it for good instead of letting
/// the string hold it.
const RADIUS: f32 = 0.004;
const TAPER: f32 = 0.6;
const PITCH: f32 = 0.2;
const GAP: f32 = 0.005;
const INTERNODE: f32 = 0.12;

/// A leaf: the petiole as a fraction of its length, a leaflet's width as a
/// fraction of its own length, how far it rises off level and how far it
/// droops, in radians, and over how many nodes from the tip it grows to full
/// size.
const PETIOLE: f32 = 0.4;
const WIDTH: f32 = 0.75;
const ELEVATION: f32 = 0.35;
const DROOP: f32 = 0.5;
const GROWING: f32 = 4.0;

/// Pods: a ripe pod's diameter and the stalk it hangs by, in meters, the
/// lowest node that bears them, how many hang at a node at least and at
/// most, how many numbers a node's pods take after its leaf's (which the
/// most may not exceed), how far they tilt outward, in radians, and the
/// week they set.
const POD_DIAMETER: f32 = 0.012;
const PEDICEL: f32 = 0.015;
const FIRST_POD: u32 = 3;
const PODS: (u32, u32) = (2, 3);
const POD_SLOTS: u64 = 8;
const POD_TILT: f32 = 0.35;
const PODS_FROM: f32 = 6.0;

/// How far each leaf strays from its node's: in length as a fraction, in
/// elevation and azimuth in radians; and each pod in azimuth and tilt.
const LENGTH_JITTER: f64 = 0.1;
const ELEVATION_JITTER: f64 = 0.15;
const AZIMUTH_JITTER: f64 = 0.25;
const POD_JITTER: f64 = 0.4;

species! {
    /// Pole bean: a vine twining up a string from its bed's trellis, a large
    /// trifoliate leaf at every node and pods hanging from the upper nodes,
    /// grown to harvest over ten weeks.
    pub struct PoleBeanParams for PoleBean as "pole bean" {
        /// Length of a harvest-ready vine up its string, in meters; a vine
        /// stops at the string's top.
        #[reflect(@Slider { min: 1.0, max: 3.0, step: 0.1 })]
        pub height: f32 = 2.2,
        /// Length of a grown leaf, petiole and leaflets, in meters.
        #[reflect(@Slider { min: 0.1, max: 0.4, step: 0.01 })]
        pub leaf_length: f32 = 0.25,
        /// Length of a ripe pod, in meters.
        #[reflect(@Slider { min: 0.08, max: 0.25, step: 0.01 })]
        pub pod_length: f32 = 0.16,
    }
    common { share: 0.25, spacing: 0.15, sown_week: 0.0, bed: 0, variations: 12 }
}

impl Recipe for PoleBean {
    const HARVEST: f32 = 10.0;
    const CLIMBS: bool = true;

    /// Draw order from the plant's stream: the vine's three, then the
    /// spiral's phase. Every node's jitter, leaf and pods alike, comes from
    /// the node's own stream, so a vine a node longer re-rolls nothing.
    fn grow(p: &PoleBeanParams, plant: &PlantConfig, b: &mut Builder) {
        let stage = plant.grown(Self::HARVEST);
        let size = (stage * plant.vigour).max(0.05);
        let radius = RADIUS * size.max(0.3);
        // Every climber is planted with a string; without one the vine
        // would wind up a vertical line its own height.
        let top = plant.string.unwrap_or(Vec3::Z * p.height);
        let vine = b.axis(
            Vec3::ZERO,
            Axis::twine(top, p.height * size, radius)
                .pitch(PITCH)
                .offset(STRING_RADIUS + radius + GAP)
                .taper(TAPER)
                .tint(srgb(STEM))
                .bends(),
        );
        let tint = mix(srgb(LEAF), srgb(YOUNG), 1.0 - stage);
        let ripeness = ((plant.age - PODS_FROM) / (Self::HARVEST - PODS_FROM)).clamp(0.0, 1.0);
        for node in vine.nodes(&Spiral::golden(INTERNODE).phased(b.rng())) {
            let index = node.rank as u64;
            let mut rng = b.stream(index);
            let length = 1.0 + rng.range(-LENGTH_JITTER, LENGTH_JITTER) as f32;
            let elevation = rng.range(-ELEVATION_JITTER, ELEVATION_JITTER) as f32;
            let azimuth = node.azimuth + rng.range(-AZIMUTH_JITTER, AZIMUTH_JITTER) as f32;
            let pods = PODS.0 + (rng.unit() * (PODS.1 - PODS.0 + 1) as f64) as u32;
            let grown = smooth((node.from_tip() as f32 + 1.0) / GROWING);
            let whole = p.leaf_length * size * (0.3 + 0.7 * grown) * length;
            let leaf = LeafConfig::trifoliate(whole, WIDTH)
                .petiole(PETIOLE * whole)
                .droop(DROOP)
                .tint(tint)
                .vein(srgb(VEIN));
            let turned = AxisNode { azimuth, ..node };
            b.leaf_at(index, vine.anchor(&turned, ELEVATION + elevation), leaf);

            let bearing = node.rank >= FIRST_POD && ripeness > 0.0;
            for k in 0..pods {
                let spread = rng.range(-POD_JITTER, POD_JITTER) as f32;
                let tilt = POD_TILT + rng.range(-POD_JITTER, POD_JITTER) as f32 / 2.0;
                if !bearing {
                    continue;
                }
                let grown = 0.3 + 0.7 * ripeness;
                let pod = FruitConfig::pod(POD_DIAMETER * grown, p.pod_length * grown)
                    .stalk(PEDICEL * grown)
                    .ripeness(ripeness)
                    .tint(srgb(POD));
                b.fruit_at(
                    index * POD_SLOTS + k as u64,
                    vine.hang(&node, azimuth + spread, tilt),
                    pod,
                );
            }
        }
    }
}

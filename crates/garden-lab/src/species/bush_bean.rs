//! Bush bean — a short stem branching from its lowest nodes into a bush of
//! trifoliate leaves, pods hanging in clusters under the upper leaves.

use super::prelude::*;

/// Foliage: a grown leaflet's green and a young one's, the stem's, and a
/// pod's.
const LEAF: u32 = 0x4C8A3A;
const YOUNG: u32 = 0x8ABF60;
const STEM: u32 = 0x6E9A48;
const POD: u32 = 0x5F9A3A;

/// The main stem: a grown stem's radius at its foot, in meters, how much it
/// thins to the tip, how far it leans and bows, and its internode.
const RADIUS: f32 = 0.006;
const TAPER: f32 = 0.6;
const LEAN: f32 = 0.15;
const WOBBLE: f32 = 0.05;
const INTERNODE: f32 = 0.05;

/// A branch: its length as a fraction of the stem's, its radius as a
/// fraction of the stem's, how far off level it leaves and how far it turns
/// up on its way, in radians.
const BRANCH: f32 = 0.7;
const BRANCH_RADIUS: f32 = 0.7;
const BRANCH_ELEVATION: f32 = 0.6;
const BRANCH_RISE: f32 = 0.7;

/// A leaf: the petiole as a fraction of its length, a leaflet's width as a
/// fraction of its own length, how far it rises off level and how far it
/// droops, in radians, and over how many nodes from the tip it grows to full
/// size.
const PETIOLE: f32 = 0.35;
const WIDTH: f32 = 0.75;
const ELEVATION: f32 = 0.45;
const DROOP: f32 = 0.4;
const GROWING: f32 = 4.0;

/// Pods: a ripe pod's diameter, in meters, the lowest node that bears them,
/// how many hang at a node at least and at most, how far they tilt outward,
/// in radians, and the week they set.
const POD_DIAMETER: f32 = 0.011;
const FIRST_POD: u32 = 3;
const PODS: (u32, u32) = (2, 4);
const POD_TILT: f32 = 0.4;
const PODS_FROM: f32 = 5.0;

/// How far each leaf strays from its node's: in length as a fraction, in
/// elevation and azimuth in radians; and each pod in azimuth and tilt.
const LENGTH_JITTER: f64 = 0.1;
const ELEVATION_JITTER: f64 = 0.15;
const AZIMUTH_JITTER: f64 = 0.25;
const POD_JITTER: f64 = 0.4;

/// Leaf numbers on a branch start from this times the branch's number from
/// one, clear of the main stem's for a stem under a thousand nodes; and a
/// node's pods take this many numbers after its leaf's, which `PODS.1` may
/// not exceed.
const BRANCH_LEAVES: u64 = 1000;
const POD_SLOTS: u64 = 8;

species! {
    /// Bush bean: a short stem branching from its lowest nodes into a bush
    /// of trifoliate leaves, pods hanging in clusters under the upper
    /// leaves, grown to harvest over eight weeks.
    pub struct BushBeanParams for BushBean as "bush bean" {
        /// Height of a harvest-ready plant's main stem, in meters.
        #[reflect(@Slider { min: 0.2, max: 0.6, step: 0.05 })]
        pub height: f32 = 0.4,
        /// Branches leaving the lowest nodes.
        #[reflect(@Slider { min: 0.0, max: 5.0, step: 1.0 })]
        pub branches: u32 = 3,
        /// Length of a grown leaf, petiole and leaflets, in meters.
        #[reflect(@Slider { min: 0.08, max: 0.3, step: 0.01 })]
        pub leaf_length: f32 = 0.18,
        /// Length of a ripe pod, in meters.
        #[reflect(@Slider { min: 0.06, max: 0.2, step: 0.01 })]
        pub pod_length: f32 = 0.13,
    }
    common { share: 0.25, spacing: 0.1, sown_week: 0.0, bed: 0, variations: 12 }
}

impl Recipe for BushBean {
    const HARVEST: f32 = 8.0;

    /// Draw order from the plant's stream: the stem's lean and bow, the
    /// spiral's phase, then for each branch its arc's three and its spiral's
    /// phase. Every node's jitter, leaf and pods alike, comes from the node's
    /// own stream, so a stem a node longer re-rolls nothing.
    fn grow(p: &BushBeanParams, plant: &PlantConfig, b: &mut Builder) {
        let stage = plant.grown(Self::HARVEST);
        let size = (stage * plant.vigour).max(0.05);
        let height = p.height * size;
        let radius = RADIUS * size.max(0.2);
        let stem = b.axis(
            Vec3::ZERO,
            Axis::upright(height, radius)
                .taper(TAPER)
                .lean(LEAN)
                .wobble(WOBBLE)
                .tint(srgb(STEM)),
        );
        let nodes = stem.nodes(&Spiral::golden(INTERNODE).phased(b.rng()));
        let bush = Bush {
            p,
            size,
            tint: mix(srgb(LEAF), srgb(YOUNG), 1.0 - stage),
            ripeness: ((plant.age - PODS_FROM) / (Self::HARVEST - PODS_FROM)).clamp(0.0, 1.0),
        };
        for node in &nodes {
            bush.node(b, &stem, node, node.rank as u64);
        }
        // Branches from the lowest nodes, each leaving on its node's side.
        for (i, node) in nodes.iter().take(p.branches as usize).enumerate() {
            let (foot, at_radius) = stem.at(node.station);
            let branch = b.axis(
                foot + (Vec2::from_angle(node.azimuth) * at_radius).extend(0.0),
                Axis::arc(BRANCH * height, BRANCH_RADIUS * radius)
                    .heading(node.azimuth, BRANCH_ELEVATION)
                    .rise(BRANCH_RISE)
                    .taper(TAPER)
                    .tint(srgb(STEM)),
            );
            let first = (i as u64 + 1) * BRANCH_LEAVES;
            for node in branch.nodes(&Spiral::golden(INTERNODE).phased(b.rng())) {
                bush.node(b, &branch, &node, first + node.rank as u64);
            }
        }
    }
}

/// What every node of a bush bean's axes grows, main stem or branch.
struct Bush<'a> {
    p: &'a BushBeanParams,
    size: f32,
    tint: [f32; 3],
    ripeness: f32,
}

impl Bush<'_> {
    /// The leaf at `node` and, at a bearing node once the pods have set, its
    /// pods, numbered from `index`: the leaf by it and the pods under it.
    fn node(&self, b: &mut Builder, axis: &Grown, node: &AxisNode, index: u64) {
        let mut rng = b.stream(index);
        let length = 1.0 + rng.range(-LENGTH_JITTER, LENGTH_JITTER) as f32;
        let elevation = rng.range(-ELEVATION_JITTER, ELEVATION_JITTER) as f32;
        let azimuth = node.azimuth + rng.range(-AZIMUTH_JITTER, AZIMUTH_JITTER) as f32;
        let pods = PODS.0 + (rng.unit() * (PODS.1 - PODS.0 + 1) as f64) as u32;
        let grown = smooth((node.from_tip() as f32 + 1.0) / GROWING);
        let whole = self.p.leaf_length * self.size * (0.3 + 0.7 * grown) * length;
        let leaf = LeafConfig::trifoliate(whole, WIDTH)
            .petiole(PETIOLE * whole)
            .droop(DROOP)
            .tint(self.tint);
        let turned = AxisNode { azimuth, ..*node };
        b.leaf_at(index, axis.anchor(&turned, ELEVATION + elevation), leaf);

        let bearing = node.rank >= FIRST_POD && self.ripeness > 0.0;
        for k in 0..pods {
            let spread = rng.range(-POD_JITTER, POD_JITTER) as f32;
            let tilt = POD_TILT + rng.range(-POD_JITTER, POD_JITTER) as f32 / 2.0;
            if !bearing {
                continue;
            }
            // A pod grows out as it ripens.
            let grown = 0.3 + 0.7 * self.ripeness;
            let pod = FruitConfig::pod(POD_DIAMETER * grown, self.p.pod_length * grown)
                .ripeness(self.ripeness)
                .tint(srgb(POD));
            b.fruit_at(
                index * POD_SLOTS + k as u64,
                axis.hang(node, azimuth + spread, tilt),
                pod,
            );
        }
    }
}

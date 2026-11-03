//! Kale — an upright stem with a crown of large curled blades on long
//! petioles, stripped below as the lower leaves are picked, a scar at
//! every node they left. Its leaves, the crop, carry no harvest frame yet.

use super::prelude::*;

/// Foliage: a grown blade's blue-green and a young one's, the pale of the
/// midribs and petioles, and the stem's pale green.
const LEAF: u32 = 0x3F6B3A;
const YOUNG: u32 = 0x7FA85A;
const VEIN: u32 = 0xB9C8A4;
const STEM: u32 = 0x9AA68A;

/// How far a blade twists about its midrib by its tip, in radians; its
/// secondary veins along each side and how high it blisters between them,
/// as a fraction of the gap between two; how many times higher a curly
/// blade's pleats frill at its margin; and the notches cut along each edge
/// of a curly blade.
const TWIST: f32 = 0.3;
const VEINS: u32 = 16;
const RELIEF: f32 = 0.35;
const FRILL: f32 = 3.0;
const NOTCHES: u32 = 6;

/// A grown stem's radius at its foot, in meters, how much it thins to the
/// head, how far it leans and bows, and its internode.
const RADIUS: f32 = 0.02;
const TAPER: f32 = 0.6;
const LEAN: f32 = 0.1;
const WOBBLE: f32 = 0.04;
const INTERNODE: f32 = 0.035;

/// A leaf: the petiole as a fraction of its length, the blade's width as a
/// fraction, how far the blade cups, and how far the lowest leaf kept droops
/// and the top one rises, in radians.
const PETIOLE: f32 = 0.4;
const WIDTH: f32 = 0.4;
const CUP: f32 = 0.3;
const DROOP: f32 = 0.9;
const RISE: (f32, f32) = (0.3, 1.3);

/// Leaves from the head over which a leaf grows to full size.
const GROWING: f32 = 6.0;

/// How thick the base was that a picked leaf left its scar by, as a
/// fraction of the leaf's length: a petiole widening where it clasps the
/// stem.
const SCAR: f32 = 0.025;

/// How far each leaf strays from its rank's: in length as a fraction, in
/// elevation and azimuth in radians.
const LENGTH_JITTER: f64 = 0.1;
const ELEVATION_JITTER: f64 = 0.1;
const AZIMUTH_JITTER: f64 = 0.2;

species! {
    /// Kale: an upright stem carrying a crown of large curled blades on long
    /// petioles, the nodes below the leaves kept stripped bare, grown to
    /// harvest over ten weeks.
    pub struct KaleParams for Kale as "kale" {
        /// Height of a harvest-ready stem, in meters.
        #[reflect(@Slider { min: 0.3, max: 1.0, step: 0.05 })]
        pub height: f32 = 0.6,
        /// Leaves left on the stem, counted down from its head; the nodes
        /// below are stripped. A stem grows a node every 3.5 cm.
        #[reflect(@Slider { min: 4.0, max: 20.0, step: 1.0 })]
        pub leaves_kept: u32 = 10,
        /// Length of a grown leaf, petiole and blade, in meters.
        #[reflect(@Slider { min: 0.15, max: 0.6, step: 0.01 })]
        pub leaf_length: f32 = 0.35,
        /// How tightly the margin curls: 0 is a flat lacinato, 1 a curly
        /// kale, its edges notched past halfway.
        #[reflect(@Slider { min: 0.0, max: 1.0, step: 0.05 })]
        pub curl: f32 = 0.8,
    }
    common { share: 0.25, spacing: 0.3, sown_week: 0.0, bed: 0, variations: 12 }
}

impl Recipe for Kale {
    const HARVEST: f32 = 10.0;

    /// Draw order: the stem's lean and bow, the spiral's phase, then for
    /// every node from the foot up its length, elevation and azimuth,
    /// stripped nodes included, so stripping a node re-rolls no leaf. Each
    /// leaf is numbered by its node.
    fn grow(p: &KaleParams, plant: &PlantConfig, b: &mut Builder) {
        let stage = plant.grown(Self::HARVEST);
        let size = (stage * plant.vigour).max(0.05);
        let stem = b.axis(
            Vec3::ZERO,
            Axis::upright(p.height * size, RADIUS * size.max(0.2))
                .taper(TAPER)
                .lean(LEAN)
                .wobble(WOBBLE)
                .tint(srgb(STEM)),
        );
        let tint = mix(srgb(LEAF), srgb(YOUNG), 1.0 - stage);
        let nodes = stem.nodes(&Spiral::golden(INTERNODE).phased(b.rng()));
        let stripped = nodes.len().saturating_sub(p.leaves_kept as usize);
        for (i, mut node) in nodes.into_iter().enumerate() {
            let rng = b.rng();
            let length = 1.0 + rng.range(-LENGTH_JITTER, LENGTH_JITTER) as f32;
            let elevation = rng.range(-ELEVATION_JITTER, ELEVATION_JITTER) as f32;
            node.azimuth += rng.range(-AZIMUTH_JITTER, AZIMUTH_JITTER) as f32;
            // A stripped node keeps the scar of its leaf, grown and spreading
            // when it was picked.
            if i < stripped {
                let picked = p.leaf_length * size * length;
                b.scar(stem.anchor(&node, RISE.0 + elevation), SCAR * picked);
                continue;
            }

            // Leaves grow to full size over the top few nodes, and settle
            // from upright at the head to spreading and drooping below.
            let from_tip = node.from_tip() as f32;
            let grown = smooth((from_tip + 1.0) / GROWING);
            let settled = smooth(from_tip / p.leaves_kept.max(1) as f32);
            let whole = p.leaf_length * size * (0.3 + 0.7 * grown) * length;
            let config = LeafConfig::simple(whole, WIDTH)
                .petiole(PETIOLE * whole)
                .ruffle(p.curl)
                .cup(CUP)
                .droop(DROOP * settled)
                .twist(TWIST)
                .relief(VEINS, RELIEF)
                .frill(FRILL * p.curl)
                .teeth(if p.curl > 0.5 { NOTCHES } else { 0 })
                .tint(tint)
                .vein(srgb(VEIN));
            b.leaf_at(
                node.rank as u64,
                stem.anchor(&node, lerp(RISE.1, RISE.0, settled) + elevation),
                config,
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A grown kale keeps the leaves asked for at its head and a scar at
    /// every node below them.
    #[test]
    fn a_stripped_node_keeps_the_scar_of_its_leaf() {
        let plant = PlantConfig {
            age: 10.0,
            vigour: 1.0,
            seed: 7,
            string: None,
        };
        let p = KaleParams::default();
        let mut b = Builder::new(&plant);
        Kale::grow(&p, &plant, &mut b);
        assert_eq!(b.leaves.len(), p.leaves_kept as usize);
        let lowest = b
            .leaves
            .iter()
            .map(|(_, anchor, _)| anchor.on.unwrap().1)
            .fold(f32::INFINITY, f32::min);
        let stripped = (lowest / INTERNODE).round() as usize - 1;
        assert!(stripped > 0);
        assert_eq!(b.scars.len(), stripped);
        assert!(
            b.scars
                .iter()
                .all(|(anchor, radius)| anchor.on.unwrap().1 < lowest && *radius > 0.0)
        );
    }
}

//! Phyllotaxis — where leaves sit round an axis: a rosette's, round a crown,
//! and a spiral's, up a stem.

use std::f32::consts::{FRAC_PI_2, TAU};

use misina_lab::rng::Rng;

/// The golden angle, in radians: the divergence between one leaf and the
/// next that packs a rosette without any two shading each other.
pub const GOLDEN: f32 = TAU * (1.0 - 0.618_034);

/// The first leaves of a rosette grow over this many ranks to full size, the
/// blades keep it to this far in, and the youngest inside shrink from there
/// toward the centre, to no less than this fraction.
const GROWING_RANKS: f32 = 5.0;
const PLATEAU: f32 = 0.4;
const SMALLEST: f32 = 0.15;

/// How far short of an axis's tip its last node sits, in internodes.
const TIP_GAP: f32 = 0.5;

/// Leaves round a crown, one golden angle apart, the oldest outermost.
pub struct Rosette {
    count: u32,
    phase: f32,
}

impl Rosette {
    pub fn golden(count: u32) -> Self {
        Self { count, phase: 0.0 }
    }

    /// Turned to a start drawn from `rng`, so no two plants face alike.
    pub fn phased(mut self, rng: &mut Rng) -> Self {
        self.phase = rng.range(0.0, TAU as f64) as f32;
        self
    }

    /// The nodes from the outermost, rank 0, in.
    pub fn nodes(&self) -> impl Iterator<Item = Node> + '_ {
        (0..self.count).map(move |rank| Node {
            rank,
            count: self.count,
            azimuth: (self.phase + rank as f32 * GOLDEN) % TAU,
        })
    }
}

/// One node of a rosette: its rank from the outside in, and where it faces.
#[derive(Clone, Copy, Debug)]
pub struct Node {
    pub rank: u32,
    pub count: u32,
    /// Where the leaf points, in radians about +Z from +X.
    pub azimuth: f32,
}

impl Node {
    /// How far in the node is: 0 the outermost, 1 the innermost.
    pub fn inner(&self) -> f32 {
        if self.count <= 1 {
            0.0
        } else {
            self.rank as f32 / (self.count - 1) as f32
        }
    }

    /// A leaf's size against the largest on the plant: the first few leaves
    /// smaller, the skirt and the middle full, the youngest inside shrinking
    /// toward the centre.
    pub fn size(&self) -> f32 {
        let grown = ((self.rank as f32 + 2.0) / GROWING_RANKS).min(1.0);
        let young = ((self.inner() - PLATEAU) / (1.0 - PLATEAU)).max(0.0);
        (grown * (FRAC_PI_2 * young).cos()).max(SMALLEST)
    }

    /// A leaf's rise off level: the outer leaves lying out near flat, the
    /// inner near upright.
    pub fn elevation(&self) -> f32 {
        0.15 + 1.1 * self.inner().powf(1.5)
    }
}

/// Leaves up a stem, one every `internode`, each a golden angle round from
/// the last, the oldest at the foot.
pub struct Spiral {
    internode: f32,
    phase: f32,
}

impl Spiral {
    pub fn golden(internode: f32) -> Self {
        Self {
            internode: internode.max(1e-3),
            phase: 0.0,
        }
    }

    /// Turned to a start drawn from `rng`, so no two plants face alike.
    pub fn phased(mut self, rng: &mut Rng) -> Self {
        self.phase = rng.range(0.0, TAU as f64) as f32;
        self
    }

    /// The nodes along an axis `length` long, from the foot up: the first an
    /// internode up, the last short of the tip.
    pub fn along(&self, length: f32) -> Vec<AxisNode> {
        let count = ((length / self.internode - TIP_GAP).floor().max(0.0)) as u32;
        (0..count)
            .map(|rank| AxisNode {
                rank,
                count,
                station: (rank + 1) as f32 * self.internode,
                azimuth: self.azimuth(rank),
            })
            .collect()
    }

    /// Where the node of `rank` faces, in radians about +Z from +X, whether
    /// or not the spiral puts one there.
    pub fn azimuth(&self, rank: u32) -> f32 {
        (self.phase + rank as f32 * GOLDEN) % TAU
    }
}

/// One node of a spiral: its rank from the foot up, where it sits along the
/// axis and where it faces.
#[derive(Clone, Copy, Debug)]
pub struct AxisNode {
    pub rank: u32,
    pub count: u32,
    /// Meters along the axis from its foot.
    pub station: f32,
    /// Where the leaf points, in radians about +Z from +X.
    pub azimuth: f32,
}

impl AxisNode {
    /// Nodes between this one and the tip: 0 for the youngest.
    pub fn from_tip(&self) -> u32 {
        self.count - 1 - self.rank
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Neighbouring ranks diverge by the golden angle, the sizes rise from
    /// the outermost to the middle and fall to the innermost, and the
    /// elevation rises from out to in.
    #[test]
    fn a_rosette_turns_by_the_golden_angle_and_sizes_by_rank() {
        let nodes: Vec<Node> = Rosette::golden(12).nodes().collect();
        for pair in nodes.windows(2) {
            let turn = (pair[1].azimuth - pair[0].azimuth).rem_euclid(TAU);
            assert!((turn - GOLDEN).abs() < 1e-4, "{turn}");
            assert!(pair[1].elevation() > pair[0].elevation());
        }
        let sizes: Vec<f32> = nodes.iter().map(Node::size).collect();
        assert!(sizes[0] < sizes[4] && sizes[11] < sizes[6], "{sizes:?}");
        assert!(
            sizes.iter().all(|s| (SMALLEST..=1.0).contains(s)),
            "{sizes:?}"
        );
    }

    /// A spiral's nodes climb an internode at a time from one internode up
    /// to short of the tip, turning by the golden angle, and a stem shorter
    /// than an internode and a half has none.
    #[test]
    fn a_spiral_climbs_an_internode_at_a_time() {
        let spiral = Spiral::golden(0.05);
        let nodes = spiral.along(0.6);
        assert_eq!(nodes.len(), 11, "{nodes:?}");
        assert!((nodes[0].station - 0.05).abs() < 1e-6);
        assert!(nodes.last().unwrap().station <= 0.6 - 0.5 * 0.05 + 1e-6);
        assert_eq!(nodes[0].from_tip(), 10);
        for pair in nodes.windows(2) {
            assert!((pair[1].station - pair[0].station - 0.05).abs() < 1e-6);
            let turn = (pair[1].azimuth - pair[0].azimuth).rem_euclid(TAU);
            assert!((turn - GOLDEN).abs() < 1e-4);
        }
        assert!(spiral.along(0.07).is_empty());
    }
}

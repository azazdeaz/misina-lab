//! Builder — what a recipe says, and what the plant layer makes of it.
//!
//! A recipe reads its plant's [`PlantConfig`], draws from the builder's
//! stream and hands the builder a crown or axes, and leaves and fruit at
//! anchors on them; the plant layer spawns what it collected. Nothing here
//! touches the ECS, so a recipe is a plain function a test can call.

use bevy::prelude::*;
use misina_lab::rng::{Rng, salt};

use super::axis::{Anchor, Axis, Grown};
use super::phyllotaxis::Node;
use crate::elements::fruit::{FruitConfig, Kind};
use crate::elements::leaf::{LeafConfig, Template};
use crate::elements::planting::PlantConfig;

/// `t` eased in and out over `0..1`.
pub fn smooth(t: f32) -> f32 {
    let t = t.clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// `a` at `t` = 0, `b` at 1.
pub fn lerp(a: f32, b: f32, t: f32) -> f32 {
    a + (b - a) * t
}

/// Where a crown's leaves attach, as a fraction of its radius.
const ATTACH: f32 = 0.8;

/// Fruit seeds are salted apart from leaf seeds by this bit.
const FRUIT_SALT: u64 = 1 << 40;

/// The stream a node's draws come from, salted with the node's index.
const NODE_STREAM: u64 = 0x7F4A_3C21_9E0B_D5A7;

/// What one recipe grew, collected for the plant layer to spawn.
pub struct Builder {
    rng: Rng,
    seed: u64,
    /// The crown, if the recipe grew one.
    pub(crate) crown: Option<Crown>,
    /// The axes, in the order grown: the plant's stems, wood or rods.
    pub(crate) axes: Vec<Grown>,
    /// Each leaf's index on the plant, its anchor, and its config.
    pub(crate) leaves: Vec<(u64, Anchor, LeafConfig)>,
    /// Each fruit's index on the plant, its anchor, and its config.
    pub(crate) fruits: Vec<(u64, Anchor, FruitConfig)>,
}

impl Builder {
    pub fn new(plant: &PlantConfig) -> Self {
        Self {
            rng: Rng::new(plant.seed),
            seed: plant.seed,
            crown: None,
            axes: Vec::new(),
            leaves: Vec::new(),
            fruits: Vec::new(),
        }
    }

    /// The plant's stream.
    pub fn rng(&mut self) -> &mut Rng {
        &mut self.rng
    }

    /// A stream of the plant's for the node numbered `index`, the same
    /// whatever was drawn before it and unrelated to its neighbours': a
    /// recipe whose node count grows with age draws each node's jitter from
    /// here, so a node more re-rolls no other.
    pub fn stream(&self, index: u64) -> Rng {
        Rng::new(self.seed ^ NODE_STREAM ^ salt(index))
    }

    /// The stub of stem at the plant's foot, `radius` across and standing
    /// on `base`, that a rosette's leaves anchor on.
    pub fn crown(&mut self, radius: f32, base: f32) -> Crown {
        let crown = Crown {
            radius: radius.max(0.001),
            base,
        };
        self.crown = Some(crown);
        crown
    }

    /// An axis grown from `foot`, three draws from the plant's stream
    /// whatever its course; see [`Axis::grow`].
    pub fn axis(&mut self, foot: Vec3, axis: Axis) -> Grown {
        let mut grown = axis.grow(foot, &mut self.rng);
        grown.index = self.axes.len();
        self.axes.push(grown.clone());
        grown
    }

    /// A leaf at `anchor`, in the plant's frame, numbered after the leaves
    /// before it. Its own draws come from a stream of the plant's and its
    /// number, so one leaf more re-rolls no other.
    pub fn leaf(&mut self, anchor: impl Into<Anchor>, leaf: LeafConfig) {
        self.leaf_at(self.leaves.len() as u64, anchor, leaf);
    }

    /// A leaf at `anchor` numbered `index`: a recipe that leaves some of a
    /// stem's nodes bare numbers its leaves by node, so a node stripped
    /// renumbers and re-rolls no other leaf.
    pub fn leaf_at(&mut self, index: u64, anchor: impl Into<Anchor>, mut leaf: LeafConfig) {
        leaf.seed = salt(self.seed ^ index);
        self.leaves.push((index, anchor.into(), leaf));
    }

    /// A fruit at `anchor`, in the plant's frame, hanging down its own -Z,
    /// numbered after the fruit before it.
    pub fn fruit(&mut self, anchor: impl Into<Anchor>, fruit: FruitConfig) {
        self.fruit_at(self.fruits.len() as u64, anchor, fruit);
    }

    /// A fruit at `anchor` numbered `index`: a recipe whose fruiting nodes
    /// come and go with age numbers its fruit by node and place, so a node
    /// that starts fruiting renumbers and re-rolls no other fruit.
    pub fn fruit_at(&mut self, index: u64, anchor: impl Into<Anchor>, mut fruit: FruitConfig) {
        fruit.seed = salt(self.seed ^ FRUIT_SALT ^ index);
        self.fruits.push((index, anchor.into(), fruit));
    }
}

/// A crown, as the anchor of a rosette's leaves.
#[derive(Clone, Copy, Debug)]
pub struct Crown {
    pub radius: f32,
    /// The height of its foot above the plant's.
    pub base: f32,
}

impl Crown {
    /// Its height: short, a rosette's internodes being millimetres.
    pub fn height(&self) -> f32 {
        1.5 * self.radius
    }

    /// Where a leaf at `node` leaves the crown, pointing out along the
    /// node's azimuth and rising `elevation` radians off level, its face
    /// turned up and in: on the crown's side, higher the further in.
    pub fn anchor(&self, node: &Node, elevation: f32) -> Transform {
        let foot = (Vec2::from_angle(node.azimuth) * self.radius * ATTACH)
            .extend(self.base + self.height() * node.inner());
        leaf_frame(foot, node.azimuth, elevation)
    }
}

/// A leaf's frame: its base at `foot`, pointing out along `azimuth` and
/// rising `elevation` radians off level, its face turned up and in.
pub fn leaf_frame(foot: Vec3, azimuth: f32, elevation: f32) -> Transform {
    Transform::from_translation(foot)
        .with_rotation(Quat::from_rotation_z(azimuth) * Quat::from_rotation_y(-elevation))
}

/// The vocabulary a recipe describes a leaf in: a template, then its
/// modifiers. The builder fills in the seed.
impl LeafConfig {
    fn template(template: Template, length: f32, width: f32) -> Self {
        Self {
            template,
            length,
            width,
            petiole: 0.0,
            pairs: 0,
            ruffle: 0.0,
            cup: 0.0,
            droop: 0.0,
            tint: [0.1, 0.3, 0.05],
            seed: 0,
        }
    }

    /// An entire blade `length` long and `width` of that across at its
    /// widest.
    pub fn simple(length: f32, width: f32) -> Self {
        Self::template(Template::Simple, length, width)
    }

    /// A compound leaf `length` long from its base to the tip of its
    /// terminal leaflet, with `pairs` pairs of leaflets along its rachis,
    /// each `width` of the leaf's length long.
    pub fn compound(length: f32, pairs: u32, width: f32) -> Self {
        Self {
            pairs,
            ..Self::template(Template::Compound, length, width)
        }
    }

    /// A trifoliate leaf `length` long from its base to the tip of its
    /// terminal leaflet: a pair of leaflets where the petiole ends and one
    /// past them, each `width` of its own length across.
    pub fn trifoliate(length: f32, width: f32) -> Self {
        Self::template(Template::Trifoliate, length, width)
    }

    /// A palmate blade, five lobes fanning from a basal notch, `length`
    /// long from its base to its middle lobe's tip and `width` of its
    /// blade's length across; its droop turns the blade down whole where
    /// the petiole ends.
    pub fn lobed(length: f32, width: f32) -> Self {
        Self::template(Template::Lobed, length, width)
    }

    /// A bare stalk `length` long between the anchor and the blade, counted
    /// in the leaf's length.
    pub fn petiole(mut self, length: f32) -> Self {
        self.petiole = length;
        self
    }

    /// How far the margin ruffles, 0 to 1.
    pub fn ruffle(mut self, amount: f32) -> Self {
        self.ruffle = amount;
        self
    }

    /// How far the blade cups about its midrib: the turn across its
    /// half-width at the widest, in radians, curling the edges up toward the
    /// face.
    pub fn cup(mut self, turn: f32) -> Self {
        self.cup = turn;
        self
    }

    /// How far the tip turns down, away from the face, in radians; negative
    /// turns it over the face.
    pub fn droop(mut self, radians: f32) -> Self {
        self.droop = radians;
        self
    }

    /// The blade's colour, linear RGB.
    pub fn tint(mut self, color: [f32; 3]) -> Self {
        self.tint = color;
        self
    }
}

/// The vocabulary a recipe describes a fruit in. The builder fills in the
/// seed.
impl FruitConfig {
    /// A taproot `diameter` across at its shoulder and `length` long,
    /// hanging from its anchor.
    pub fn root(diameter: f32, length: f32) -> Self {
        Self {
            kind: Kind::Root,
            diameter,
            length,
            ripeness: 0.0,
            tint: [0.8, 0.3, 0.05],
            seed: 0,
        }
    }

    /// A pod `diameter` across at its middle and `length` long, hanging
    /// from its anchor.
    pub fn pod(diameter: f32, length: f32) -> Self {
        Self {
            kind: Kind::Pod,
            tint: [0.25, 0.45, 0.1],
            ..Self::root(diameter, length)
        }
    }

    /// A berry `diameter` across at its widest and `length` long, narrower
    /// toward its stalk, hanging down its own -Z from its anchor: a
    /// zucchini, which its recipe lays out near level.
    pub fn berry(diameter: f32, length: f32) -> Self {
        Self {
            kind: Kind::Berry,
            tint: [0.02, 0.09, 0.015],
            ..Self::root(diameter, length)
        }
    }

    /// How ripe it is, 0 to 1: written as the `ripeness` attribute, and
    /// `harvest_ready` from 0.9.
    pub fn ripeness(mut self, ripeness: f32) -> Self {
        self.ripeness = ripeness;
        self
    }

    /// The skin's colour, linear RGB.
    pub fn tint(mut self, color: [f32; 3]) -> Self {
        self.tint = color;
        self
    }
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeSet;

    use super::*;

    /// A node's stream is its own: the first draws of sixteen neighbouring
    /// nodes share no value, where streams a step apart would.
    #[test]
    fn neighbouring_nodes_draw_from_unrelated_streams() {
        let plant = PlantConfig {
            age: 4.0,
            vigour: 1.0,
            seed: 0x1234_5678,
            string: None,
        };
        let b = Builder::new(&plant);
        let mut seen = BTreeSet::new();
        for index in 0..16 {
            let mut rng = b.stream(index);
            for _ in 0..10 {
                assert!(
                    seen.insert(rng.unit().to_bits()),
                    "node {index} repeats a draw"
                );
            }
        }
    }
}

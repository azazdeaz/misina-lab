//! Builder — what a recipe says, and what the plant layer makes of it.
//!
//! A recipe reads its plant's [`PlantConfig`], draws from the builder's
//! stream and hands the builder a crown or axes, and leaves and fruit at
//! anchors on them; the plant layer spawns what it collected. Nothing here
//! touches the ECS, so a recipe is a plain function a test can call.

use bevy::prelude::*;
use misina_lab::rng::{Rng, salt};

use super::axis::{Axis, Grown};
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

/// What one recipe grew, collected for the plant layer to spawn.
pub struct Builder {
    rng: Rng,
    seed: u64,
    /// The crown, if the recipe grew one.
    pub(crate) crown: Option<Crown>,
    /// The axes, in the order grown: the plant's rigid wood.
    pub(crate) axes: Vec<Grown>,
    /// Each leaf's index on the plant, its anchor in the plant's frame, and
    /// its config.
    pub(crate) leaves: Vec<(u64, Transform, LeafConfig)>,
    /// Each fruit's anchor in the plant's frame, and its config.
    pub(crate) fruits: Vec<(Transform, FruitConfig)>,
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

    /// An axis grown from `foot`; its lean and bow are drawn from the
    /// plant's stream, three draws.
    pub fn axis(&mut self, foot: Vec3, axis: Axis) -> Grown {
        let grown = axis.grow(foot, &mut self.rng);
        self.axes.push(grown.clone());
        grown
    }

    /// A leaf at `anchor`, in the plant's frame, numbered after the leaves
    /// before it. Its own draws come from a stream of the plant's and its
    /// number, so one leaf more re-rolls no other.
    pub fn leaf(&mut self, anchor: Transform, leaf: LeafConfig) {
        self.leaf_at(self.leaves.len() as u64, anchor, leaf);
    }

    /// A leaf at `anchor` numbered `index`: a recipe that leaves some of a
    /// stem's nodes bare numbers its leaves by node, so a node stripped
    /// renumbers and re-rolls no other leaf.
    pub fn leaf_at(&mut self, index: u64, anchor: Transform, mut leaf: LeafConfig) {
        leaf.seed = salt(self.seed ^ index);
        self.leaves.push((index, anchor, leaf));
    }

    /// A fruit at `anchor`, in the plant's frame, hanging down its own -Z.
    pub fn fruit(&mut self, anchor: Transform, mut fruit: FruitConfig) {
        fruit.seed = salt(self.seed ^ FRUIT_SALT ^ self.fruits.len() as u64);
        self.fruits.push((anchor, fruit));
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

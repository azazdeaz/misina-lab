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

/// A crown: its height as a fraction of its radius, short, a rosette's
/// internodes being millimetres; how blunt its dome is, the exponent its
/// side falls to its top with; where its leaves attach, as a fraction of its
/// radius at their height; and how far up it the innermost leaf leaves it,
/// as a fraction of its height.
const CROWN_HEIGHT: f32 = 1.5;
const DOME: f32 = 2.0;
const ATTACH: f32 = 0.8;
const INNERMOST: f32 = 0.8;

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
    /// Where a leaf was taken off, and how thick its stalk was.
    pub(crate) scars: Vec<(Anchor, f32)>,
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
            scars: Vec::new(),
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
    /// on `base`, that a rosette's leaves anchor on, in `tint`, linear RGB:
    /// a species' crowns are one part, in the first one's tint.
    pub fn crown(&mut self, radius: f32, base: f32, tint: [f32; 3]) -> Crown {
        let crown = Crown {
            radius: radius.max(0.001),
            base,
            tint,
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

    /// The scar a leaf leaves where it is taken off at `anchor`, its stalk
    /// `radius` thick: the stem swells there as though the leaf still grew
    /// from it.
    pub fn scar(&mut self, anchor: impl Into<Anchor>, radius: f32) {
        self.scars.push((anchor.into(), radius));
    }
}

/// A crown, as the anchor of a rosette's leaves: a dome on its foot, its
/// leaves leaving its side, higher the further in.
#[derive(Clone, Copy, Debug)]
pub struct Crown {
    pub radius: f32,
    /// The height of its foot above the plant's.
    pub base: f32,
    /// Its colour, linear RGB.
    pub tint: [f32; 3],
}

impl Crown {
    /// Its height.
    pub fn height(&self) -> f32 {
        CROWN_HEIGHT * self.radius
    }

    /// Its radius `up` of the way from its foot to its top, as a fraction
    /// of its foot's.
    pub fn profile(up: f32) -> f32 {
        (1.0 - up.clamp(0.0, 1.0).powf(DOME)).sqrt()
    }

    /// The point on its side `up` of the way from its foot to its top,
    /// facing `azimuth`.
    pub fn side(&self, azimuth: f32, up: f32) -> Vec3 {
        (Vec2::from_angle(azimuth) * self.radius * Self::profile(up))
            .extend(self.base + self.height() * up)
    }

    /// Where a leaf at `node` leaves the crown, pointing out along the
    /// node's azimuth and rising `elevation` radians off level, its face
    /// turned up and in: in from its side, higher the further in.
    pub fn anchor(&self, node: &Node, elevation: f32) -> Transform {
        let side = self.side(node.azimuth, INNERMOST * node.inner());
        let foot = (side.xy() * ATTACH).extend(side.z);
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
            teeth: 0,
            veins: 0,
            relief: 0.0,
            frill: 0.0,
            ruffle: 0.0,
            cup: 0.0,
            droop: 0.0,
            twist: 0.0,
            tint: [0.1, 0.3, 0.05],
            vein: None,
            edge: None,
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

    /// Notches cut into each edge of a simple blade, teeth into each edge
    /// of a trifoliate leaflet, or round a lobed blade's margin; none leaves
    /// it entire.
    pub fn teeth(mut self, teeth: u32) -> Self {
        self.teeth = teeth;
        self
    }

    /// Pleats the blade on `veins` secondary veins along each side, every
    /// crest between two standing `depth` of the gap between them above
    /// them; a lobed blade pleats on its main veins, its crests `depth` of
    /// their distance out from its middle high, and reads only `depth`.
    pub fn relief(mut self, veins: u32, depth: f32) -> Self {
        self.veins = veins;
        self.relief = depth;
        self
    }

    /// How much higher the pleats stand at the margin than off the midrib,
    /// as a multiple of the relief: a frilled edge.
    pub fn frill(mut self, amount: f32) -> Self {
        self.frill = amount;
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

    /// How far the blade turns about its midrib by its tip, in radians;
    /// which way is the leaf's own draw.
    pub fn twist(mut self, radians: f32) -> Self {
        self.twist = radians;
        self
    }

    /// The blade's colour, linear RGB.
    pub fn tint(mut self, color: [f32; 3]) -> Self {
        self.tint = color;
        self
    }

    /// The colour of the petiole, the midrib and the blade beside it,
    /// linear RGB, where it differs from the blade's.
    pub fn vein(mut self, color: [f32; 3]) -> Self {
        self.vein = Some(color);
        self
    }

    /// The colour the margin and the tip fade to, linear RGB, where it
    /// differs from the blade's.
    pub fn edge(mut self, color: [f32; 3]) -> Self {
        self.edge = Some(color);
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
            shoulder: None,
            stalk: 0.0,
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
    /// toward its stalk, faintly ribbed, a scar on its blossom end, hanging
    /// down its own -Z from its anchor: a zucchini, which its recipe lays
    /// out near level. Its stalk is stout and ridged, flaring into it.
    pub fn berry(diameter: f32, length: f32) -> Self {
        Self {
            kind: Kind::Berry,
            tint: [0.02, 0.09, 0.015],
            ..Self::root(diameter, length)
        }
    }

    /// A strawberry `diameter` across at its widest, a third of the way
    /// down, and `length` long, its shoulder rounded below its stalk and
    /// its tip pointed, hanging down its own -Z from its anchor.
    pub fn strawberry(diameter: f32, length: f32) -> Self {
        Self {
            kind: Kind::Strawberry,
            tint: [0.55, 0.02, 0.03],
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

    /// The skin's colour at its stalk end, linear RGB, where it differs
    /// from the tint: a strawberry's before it colours, pushed toward the tip
    /// by its ripeness; a root's shoulder; a berry's paler flecks.
    pub fn shoulder(mut self, color: [f32; 3]) -> Self {
        self.shoulder = Some(color);
        self
    }

    /// A bare stalk `length` long between the anchor and the body, which
    /// hangs below it.
    pub fn stalk(mut self, length: f32) -> Self {
        self.stalk = length;
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

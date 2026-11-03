//! Lettuce — loose-leaf or heading: a rosette of entire, ruffled blades on a
//! stub of a stem.

use super::prelude::*;

/// Foliage: a green loose-leaf's, a red one's at the margins over a blade
/// this far from the green toward it, the yellow-green of a young plant
/// either way, and the pale of the midribs and petioles.
const GREEN: u32 = 0x6FA83C;
const RED: u32 = 0x7A2A3A;
const RED_BLEND: f32 = 0.3;
const YOUNG: u32 = 0xA8CF5A;
const VEIN: u32 = 0xD8E6AE;

/// How far a blade twists about its midrib by its tip, in radians.
const TWIST: f32 = 0.25;

/// A grown crown's radius, in meters.
const CROWN: f32 = 0.012;

/// A blade's width at its widest, as a fraction of its length.
const WIDTH: f32 = 0.75;

/// How far an open blade cups, as the turn across its half-width, and how
/// far the outermost droops, in radians; and how far a heading plant's inner
/// blades cup and turn over the crown.
const CUP: f32 = 0.35;
const DROOP: f32 = 0.6;
const HEAD_CUP: f32 = 1.4;
const HEAD_TURN: f32 = -2.2;

/// Where a heading plant's blades start closing, from the outside in, and
/// over how much of the rosette they close.
const CLOSING: (f32, f32) = (0.4, 0.2);

/// How far each blade strays from its rank's: in length and droop as a
/// fraction, in elevation and azimuth in radians.
const LENGTH_JITTER: f64 = 0.1;
const DROOP_JITTER: f64 = 0.25;
const ELEVATION_JITTER: f64 = 0.1;
const AZIMUTH_JITTER: f64 = 0.15;

/// Foliage colour.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Tint {
    #[default]
    Green,
    Red,
    /// Plants of either, half and half.
    Mix,
}

impl Tint {
    pub const ALL: [Tint; 3] = [Tint::Green, Tint::Red, Tint::Mix];

    /// The names, in [`ALL`](Self::ALL) order.
    pub const NAMES: [&'static str; 3] = ["green", "red", "mix"];

    pub fn name(self) -> &'static str {
        Self::NAMES[self as usize]
    }

    pub fn parse(name: &str) -> Option<Tint> {
        Self::ALL.into_iter().find(|tint| tint.name() == name)
    }
}

species! {
    /// Lettuce, loose-leaf or heading: a rosette of entire blades, ruffled or
    /// smooth, round a stub of stem a couple of centimetres tall, grown to
    /// harvest over eight weeks.
    pub struct LettuceParams for Lettuce as "lettuce" {
        /// Blades on a harvest-ready plant.
        #[reflect(@Slider { min: 8.0, max: 48.0, step: 1.0 })]
        pub leaves: u32 = 28,
        /// Length of the largest blade at harvest, in meters.
        #[reflect(@Slider { min: 0.08, max: 0.3, step: 0.01 })]
        pub leaf_length: f32 = 0.15,
        /// How much the margin ruffles: 0 is a smooth butterhead, 1 a frilly
        /// loose-leaf.
        #[reflect(@Slider { min: 0.0, max: 1.0, step: 0.05 })]
        pub ruffle: f32 = 0.6,
        /// Whether the inner blades close over the crown into a head.
        pub heading: bool = false,
        /// Foliage colour: `green`, `red`, or a `mix` of plants of either
        /// across the bed.
        #[reflect(@Choices(&Tint::NAMES))]
        pub tint: String = Tint::default().name().into(),
    }
    common { share: 0.25, spacing: 0.15, sown_week: 0.0, bed: 0, variations: 12 }
}

impl LettuceParams {
    /// The tint, or the default with a warning for a name not in the list.
    fn tint(&self) -> Tint {
        Tint::parse(&self.tint).unwrap_or_else(|| {
            warn!(
                "lettuce tint {:?} is not one of {:?}; using {:?}",
                self.tint,
                Tint::NAMES,
                Tint::default().name()
            );
            Tint::default()
        })
    }
}

impl Recipe for Lettuce {
    const HARVEST: f32 = 8.0;

    /// Draw order: the tint's coin, the rosette's phase, then for every
    /// blade from the outermost in its length, droop, elevation and azimuth,
    /// so a blade more re-rolls none before it.
    fn grow(p: &LettuceParams, plant: &PlantConfig, b: &mut Builder) {
        let stage = plant.grown(Self::HARVEST);
        let size = stage * plant.vigour;
        // Two blades from the start: the seedling's.
        let leaves = (p.leaves as f32 * stage).round().max(2.0) as u32;
        let coin = b.rng().unit();
        let red = match p.tint() {
            Tint::Green => false,
            Tint::Red => true,
            Tint::Mix => coin < 0.5,
        };
        // A red lettuce is red at its margins and tips over a blade only
        // leaning toward red; a green one is green throughout.
        let blade = if red {
            mix(srgb(GREEN), srgb(RED), RED_BLEND)
        } else {
            srgb(GREEN)
        };
        let tint = mix(blade, srgb(YOUNG), 1.0 - stage);
        let crown = b.crown(CROWN * size.max(0.15), 0.0);
        let rosette = Rosette::golden(leaves).phased(b.rng());
        for mut node in rosette.nodes() {
            let rng = b.rng();
            let length = 1.0 + rng.range(-LENGTH_JITTER, LENGTH_JITTER) as f32;
            let droop = 1.0 + rng.range(-DROOP_JITTER, DROOP_JITTER) as f32;
            let open = node.elevation() + rng.range(-ELEVATION_JITTER, ELEVATION_JITTER) as f32;
            node.azimuth += rng.range(-AZIMUTH_JITTER, AZIMUTH_JITTER) as f32;

            // How far this blade has closed into the head: not at all on a
            // loose-leaf plant, and from none to all over the inner half of a
            // heading one, which starts low and wraps over the crown.
            let inner = node.inner();
            let closing = if p.heading {
                smooth((inner - CLOSING.0) / CLOSING.1)
            } else {
                0.0
            };
            let closed = 0.35 + 0.6 * ((inner - 0.5) * 2.0).clamp(0.0, 1.0);
            // An open blade droops no further than just below level.
            let drooping = (DROOP * (1.0 - inner) * droop).min(open + 0.1);
            let mut config = LeafConfig::simple(p.leaf_length * size * node.size() * length, WIDTH)
                .ruffle(p.ruffle * (1.0 - 0.5 * inner))
                .cup(lerp(CUP, HEAD_CUP, closing))
                .droop(lerp(drooping, HEAD_TURN, closing))
                .twist(TWIST)
                .tint(tint)
                .vein(srgb(VEIN));
            if red {
                config = config.edge(srgb(RED));
            }
            b.leaf(crown.anchor(&node, lerp(open, closed, closing)), config);
        }
    }
}

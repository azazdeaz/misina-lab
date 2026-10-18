//! Fruit — the tomatoes, each its own prim, hung by its top from a pedicel.
//!
//! A fruit is a slightly flattened sphere with its origin at its top, where
//! the pedicel holds it, so it hangs below its prim whatever size it drew.
//! Colour is baked per mesh, so fruit are quantized on their ripeness as well
//! as their diameter, and every fruit keeps its own prim and its own exact
//! ripeness on its config.
//!
//! A stem ripens from the bottom up, about a truss a week, and a truss from
//! its base to its tip. So a fruit's ripeness, its red fraction from 0 green
//! to 1 fully coloured, is read off how far its truss hangs above its stem's
//! ripening front and how far toward the truss's tip it hangs:
//!
//! ```text
//! ripeness = ripe_trusses - rank - spread × along
//! ```

use std::f32::consts::FRAC_PI_2;

use bevy::prelude::*;
use misina_lab::palette::{Response, mix, srgb};
use misina_lab::params::{Choices, Label, Slider};
use misina_lab::quantize::{Metric, farthest_first};
use misina_lab::scene::{Library, Order, configs_changed};

use super::Grow;

/// The mesh-library prefix the fruit register under.
pub const PART: &str = "Fruit";

/// A fruit's height as a fraction of its diameter: a round tomato is a
/// little flattened.
const FLAT: f32 = 0.85;

/// Smallest fruit built, so a diameter of zero from Python still makes a
/// mesh.
const MIN_DIAMETER: f32 = 0.005;

/// What a red fraction of 1 is worth in meters of diameter to the clustering:
/// a tenth of the way from green to ripe shows about as much as 5 mm of size.
const RIPENESS_WEIGHT: f32 = 0.05;

/// Segments round a fruit and from pole to pole.
const SECTORS: u32 = 16;
const STACKS: u32 = 10;

/// A mature green fruit, before it colours.
const GREEN: u32 = 0x7D9C3E;

/// A tomato's waxy skin.
const SKIN: Response = Response {
    roughness: 0.3,
    reflectance: 0.55,
};

/// The colour a fruit ripens to.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Ripe {
    #[default]
    Red,
    Orange,
    Yellow,
    Brown,
}

impl Ripe {
    pub const ALL: [Ripe; 4] = [Ripe::Red, Ripe::Orange, Ripe::Yellow, Ripe::Brown];

    /// The names, in [`ALL`](Self::ALL) order.
    pub const NAMES: [&'static str; 4] = ["red", "orange", "yellow", "brown"];

    pub fn name(self) -> &'static str {
        Self::NAMES[self as usize]
    }

    pub fn parse(name: &str) -> Option<Ripe> {
        Self::ALL.into_iter().find(|ripe| ripe.name() == name)
    }

    /// The fully ripe skin, as sRGB.
    fn hex(self) -> u32 {
        [0xB3241A, 0xE0671C, 0xE6B92C, 0x5E2A1C][self as usize]
    }
}

/// One fruit's shape and colour, as the truss that holds it specified.
#[derive(Component, Clone, Copy, Debug, PartialEq)]
pub struct FruitConfig {
    pub diameter: f32,
    /// Red fraction, 0 green to 1 fully coloured.
    pub ripeness: f32,
    /// How many trusses above its stem's ripening front its truss hangs, and
    /// how far along the truss toward its tip, 0 to 1, this fruit hangs. Kept
    /// so the params can be re-applied to a fruit already hanging;
    /// [`FruitMetric`] does not read them.
    pub rank: f32,
    pub along: f32,
}

impl FruitConfig {
    pub fn new(params: &FruitParams, diameter: f32, rank: f32, along: f32) -> Self {
        Self {
            diameter: diameter.max(MIN_DIAMETER),
            ripeness: (params.ripe_trusses - rank - params.spread * along).clamp(0.0, 1.0),
            rank,
            along,
        }
    }
}

/// Two fruit share a mesh when they are near in size and in colour.
pub struct FruitMetric;

impl Metric<FruitConfig> for FruitMetric {
    fn distance(&self, a: &FruitConfig, b: &FruitConfig) -> f32 {
        Vec2::new(
            a.diameter - b.diameter,
            (a.ripeness - b.ripeness) * RIPENESS_WEIGHT,
        )
        .length()
    }
}

/// The fruit: how ripe the crop is, and what colour ripe is.
///
/// A stem ripens from its lowest truss up, and a truss from its base to its
/// tip, so the lowest `ripe_trusses` trusses on a stem are coloured and the
/// ones above them still green. A truss is due once its tip fruit has
/// started turning. Every stem runs a little ahead of or behind the rest.
#[derive(Resource, Reflect, Clone, Debug, PartialEq)]
#[cfg_attr(
    feature = "python",
    pyo3::pyclass(get_all, set_all, skip_from_py_object)
)]
pub struct FruitParams {
    /// Trusses per stem that have coloured, counted up from the lowest: a
    /// fraction is the next one part way.
    #[reflect(@Slider { min: 0.0, max: 3.0, step: 0.1 })]
    pub ripe_trusses: f32,
    /// How far a truss's tip fruit lags behind its base fruit, as a red
    /// fraction: 0 colours a truss all at once.
    #[reflect(@Slider { min: 0.0, max: 1.0, step: 0.05 }, @Label("Ripeness spread in a truss"))]
    pub spread: f32,
    /// The colour a ripe fruit turns: `red`, `orange`, `yellow` or `brown`.
    #[reflect(@Choices(&Ripe::NAMES))]
    pub color: String,
    /// How many distinct fruit meshes the scene may hold, by size and colour.
    /// A budget, not a count: the fruit are clustered and this is how many
    /// representatives the clustering may keep.
    #[reflect(@Slider { min: 1.0, max: 64.0, step: 1.0 }, @Label("Fruit variations"))]
    pub variations: u32,
}

impl Default for FruitParams {
    fn default() -> Self {
        Self {
            ripe_trusses: 1.0,
            spread: 0.2,
            color: Ripe::default().name().into(),
            variations: 24,
        }
    }
}

impl FruitParams {
    /// The parsed colour. An unknown name is read as the default: Python has
    /// already rejected it, and a build system is no place to fail.
    pub fn ripe(&self) -> Ripe {
        Ripe::parse(&self.color).unwrap_or_else(|| {
            warn!(
                "fruit color {:?} is not one of {:?}; using {:?}",
                self.color,
                Ripe::NAMES,
                Ripe::default().name()
            );
            Ripe::default()
        })
    }
}

pub fn plugin(app: &mut App) {
    app.init_resource::<FruitParams>().add_systems(
        PreUpdate,
        (
            reauthor.run_if(resource_changed::<FruitParams>),
            build.run_if(
                configs_changed::<FruitConfig>
                    // `color` and `variations` reach no config.
                    .or_eager(resource_changed::<FruitParams>),
            ),
        )
            .chain()
            .in_set(Grow::Fruit),
    );
}

/// Re-colours every fruit already hanging, in place.
fn reauthor(params: Res<FruitParams>, mut fruit: Query<&mut FruitConfig>) {
    for mut config in &mut fruit {
        let next = FruitConfig::new(&params, config.diameter, config.rank, config.along);
        config.set_if_neq(next);
    }
}

/// A fruit `diameter` across, hanging from its origin.
fn fruit_mesh(diameter: f32) -> Mesh {
    let radius = diameter / 2.0;
    // Bevy's sphere has its poles on Y: flattened along them, then stood on Z.
    Sphere::new(radius)
        .mesh()
        .uv(SECTORS, STACKS)
        .transformed_by(Transform {
            translation: Vec3::new(0.0, 0.0, -radius * FLAT),
            rotation: Quat::from_rotation_x(FRAC_PI_2),
            scale: Vec3::new(1.0, FLAT, 1.0),
        })
}

/// Builds one mesh per distinct fruit, each in its own colour.
fn build(
    mut commands: Commands,
    mut library: Library,
    params: Res<FruitParams>,
    fruit: Query<(Entity, &Order, &FruitConfig)>,
) {
    library.clear(PART);

    let mut hung: Vec<(Order, Entity, FruitConfig)> = fruit
        .iter()
        .map(|(entity, order, config)| (*order, entity, *config))
        .collect();
    hung.sort_by_key(|(order, ..)| *order);

    let configs: Vec<FruitConfig> = hung.iter().map(|(_, _, config)| *config).collect();
    let book = farthest_first(
        &configs,
        params.variations.max(1) as usize,
        0.0,
        &FruitMetric,
    );
    let (green, ripe) = (srgb(GREEN), srgb(params.ripe().hex()));
    let geometry: Vec<_> = book
        .representatives
        .iter()
        .enumerate()
        .map(|(index, config)| {
            let color = mix(green, ripe, config.ripeness);
            library.part(
                PART,
                index,
                fruit_mesh(config.diameter),
                SKIN.surface(color),
            )
        })
        .collect();

    for ((_, entity, _), drew) in hung.iter().zip(&book.assignment) {
        commands
            .entity(*entity)
            .insert(geometry[*drew as usize].clone());
    }
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeMap;

    use super::*;
    use crate::elements::{grown, small};
    use misina_lab::testing::organs;

    /// At one ripe truss a stem, every stem's lowest truss has coloured and
    /// its third is still green, and every truss is ripest at its base.
    #[test]
    fn a_stem_ripens_from_the_bottom_and_a_truss_from_its_base() {
        let mut app = grown(small());
        let mut trusses: BTreeMap<String, Vec<f32>> = BTreeMap::new();
        // In authoring order, which is base to tip within a truss.
        for fruit in organs::<FruitConfig>(app.world_mut()) {
            let (truss, _) = fruit.path.rsplit_once('/').unwrap();
            trusses
                .entry(truss.into())
                .or_default()
                .push(fruit.config.ripeness);
        }
        assert!(!trusses.is_empty());
        for (truss, ripeness) in &trusses {
            assert!(
                ripeness.windows(2).all(|w| w[0] >= w[1]),
                "{truss}: {ripeness:?}"
            );
            if truss.ends_with("Truss_00") {
                assert!(ripeness.iter().all(|r| *r > 0.4), "{truss}: {ripeness:?}");
            } else if truss.ends_with("Truss_02") {
                assert!(ripeness.iter().all(|r| *r == 0.0), "{truss}: {ripeness:?}");
            }
        }
    }
}

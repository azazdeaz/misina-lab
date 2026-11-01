//! Fruit — every species' fruit and roots meshed and shared, a system per
//! species.
//!
//! [`build`] is the framework's quantized layer for the solids: it collects
//! the species' [`FruitConfig`]s in authoring order, clusters them to the
//! species' `variations` budget, turns one solid per representative and
//! hands each fruit the mesh it drew. A fruit is an Xform the plant layer
//! spawned, `Fruit_NN`, carrying the harvest attributes, with a `Skin` child
//! this layer gives the mesh, a `Cut` frame and, when the colliders include
//! fruit, a `Collision` capsule.

use bevy::prelude::*;
use misina_lab::geometry::par_map;
use misina_lab::palette::{COLOR_STREAM, Response, shade};
use misina_lab::quantize::{Metric, farthest_first};
use misina_lab::rng::Rng;
use misina_lab::scene::{Library, Order};

use crate::shape::solid;
use crate::species::{OrganOf, Species};

/// How a skin answers light.
const SKIN: Response = Response {
    roughness: 0.5,
    reflectance: 0.4,
};

/// The ripeness from which a fruit is `harvest_ready`.
pub const READY: f32 = 0.9;

/// A fruit's density, in kg/m³, and how much of the cylinder round it a
/// fruit fills.
const DENSITY: f32 = 1000.0;
const FILL: f32 = 0.6;

/// What a fruit is, which decides its solid and how it is picked.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    /// A taproot: pulled by its tops.
    Root,
    /// A pod: snapped off its stalk.
    Pod,
    /// A berry: cut off its stalk.
    Berry,
}

impl Kind {
    /// How a harvester takes it: the `pick` attribute.
    pub fn pick(self) -> &'static str {
        match self {
            Kind::Root => "pull",
            Kind::Pod => "snap",
            Kind::Berry => "cut",
        }
    }
}

/// One fruit's shape, as its recipe specified it; the constructors are in
/// `grow::builder`.
#[derive(Component, Clone, Copy, Debug, PartialEq)]
pub struct FruitConfig {
    pub kind: Kind,
    /// Across at its widest, in meters.
    pub diameter: f32,
    /// Along its axis, in meters.
    pub length: f32,
    /// 0 to 1: the `ripeness` attribute, and `harvest_ready` from [`READY`].
    pub ripeness: f32,
    /// The skin's colour, linear RGB.
    pub tint: [f32; 3],
    /// The stream its shade comes from; [`FruitMetric`] reads it not.
    pub seed: u64,
}

impl FruitConfig {
    /// Its mass, in kilograms: what it weighs on a rod it rides.
    pub fn mass(&self) -> f32 {
        let radius = self.diameter / 2.0;
        DENSITY * FILL * std::f32::consts::PI * radius * radius * self.length
    }
}

/// Marks a fruit's `Skin` child, which this layer gives the mesh.
#[derive(Component)]
pub struct Skin;

/// Two fruit share a mesh when they are the same kind and tint and close in
/// size.
pub struct FruitMetric;

impl Metric<FruitConfig> for FruitMetric {
    fn distance(&self, a: &FruitConfig, b: &FruitConfig) -> f32 {
        let reach = a.length.max(b.length).max(0.1);
        let step = |same: bool| if same { 0.0 } else { 10.0 * reach };
        [
            a.length - b.length,
            a.diameter - b.diameter,
            step(a.kind == b.kind),
            step(a.tint == b.tint),
        ]
        .iter()
        .map(|d| d * d)
        .sum::<f32>()
        .sqrt()
    }
}

/// Run condition: a fruit of `S` was grown, regrown or dropped.
pub fn changed<S: Species>(
    fruit: Query<(), (Changed<FruitConfig>, With<OrganOf<S>>)>,
    mut removed: RemovedComponents<OrganOf<S>>,
) -> bool {
    let removed = removed.read().count() > 0;
    !fruit.is_empty() || removed
}

/// Builds one solid per distinct fruit of `S` and gives it to the `Skin` of
/// every fruit that drew it.
pub fn build<S: Species>(
    mut commands: Commands,
    mut library: Library,
    params: Res<S::Params>,
    fruit: Query<(Entity, &Order, &FruitConfig, &Children), With<OrganOf<S>>>,
    skins: Query<(), With<Skin>>,
) -> Result {
    let prefix = format!("{}Fruit", S::PREFIX);
    library.clear(&prefix);

    let mut hung: Vec<(Order, Entity, FruitConfig)> = fruit
        .iter()
        .filter_map(|(_, order, config, children)| {
            let skin = children.iter().find(|child| skins.contains(*child))?;
            Some((*order, skin, *config))
        })
        .collect();
    hung.sort_by_key(|(order, ..)| *order);

    let configs: Vec<FruitConfig> = hung.iter().map(|(_, _, config)| *config).collect();
    let budget = S::variations(&params).max(1) as usize;
    let book = farthest_first(&configs, budget, 0.0, &FruitMetric);
    let meshes = par_map(&book.representatives, |_, config| match config.kind {
        Kind::Root => solid::root(config),
        Kind::Pod => solid::pod(config),
        Kind::Berry => solid::berry(config),
    });
    let mut geometry = Vec::with_capacity(meshes.len());
    for (index, (config, mesh)) in book.representatives.iter().zip(meshes).enumerate() {
        let color = shade(config.tint, &mut Rng::new(COLOR_STREAM ^ config.seed));
        geometry.push(library.part(&prefix, index, mesh.to_mesh(), SKIN.surface(color)));
    }
    for ((_, skin, _), drew) in hung.iter().zip(&book.assignment) {
        commands
            .entity(*skin)
            .insert(geometry[*drew as usize].clone());
    }
    Ok(())
}

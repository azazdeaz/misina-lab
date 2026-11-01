//! Leaf — every species' leaves meshed and shared, a system per species.
//!
//! [`build`] is the framework's quantized layer: it collects the species'
//! [`LeafConfig`]s in authoring order, clusters them to the species'
//! `variations` budget, meshes one leaf per representative and hands each
//! leaf the mesh it drew. A leaf has nothing hanging off it, so the leaf
//! entity is the geometry prim itself, placed by the plant layer.

use bevy::prelude::*;
use misina_lab::geometry::par_map;
use misina_lab::palette::{COLOR_STREAM, Response, shade};
use misina_lab::quantize::{Metric, farthest_first};
use misina_lab::rng::Rng;
use misina_lab::scene::{Library, Order};

use crate::shape::blade;
use crate::species::{OrganOf, Species};

/// How a blade answers light.
const FOLIAGE: Response = Response {
    roughness: 0.65,
    reflectance: 0.4,
};

/// A fresh leaf's mass per area of blade, in kg/m².
const AREAL_MASS: f32 = 0.3;

/// What a leaf is built as.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Template {
    /// One entire blade.
    Simple,
    /// A rachis with leaflets in pairs along it and one at its tip.
    Compound,
    /// Three entire leaflets on a petiole: a pair where it ends and one past
    /// them.
    Trifoliate,
    /// A palmate blade: lobes fanning from a basal notch, hinged down where
    /// its petiole ends.
    Lobed,
}

/// One leaf's shape, as its recipe specified it; the constructors are in
/// `grow::builder`.
#[derive(Component, Clone, Copy, Debug, PartialEq)]
pub struct LeafConfig {
    pub template: Template,
    /// From the anchor to the tip, petiole included, in meters.
    pub length: f32,
    /// A simple blade's width at its widest or a compound leaf's leaflet's
    /// length, as a fraction of the length; a trifoliate leaflet's or a
    /// lobed blade's width as a fraction of its own length.
    pub width: f32,
    /// The bare stalk between the anchor and the blade, in meters.
    pub petiole: f32,
    /// A compound leaf's pairs of leaflets.
    pub pairs: u32,
    /// How far a simple blade's margin ruffles, 0 to 1.
    pub ruffle: f32,
    /// How far a simple blade cups about its midrib: the turn across its
    /// half-width at the widest, in radians, curling the edges up toward the
    /// face.
    pub cup: f32,
    /// How far the tip turns down, away from the face, in radians; negative
    /// turns it over the face. A lobed blade turns down by it whole, at its
    /// petiole's end.
    pub droop: f32,
    /// The leaf's colour, linear RGB.
    pub tint: [f32; 3],
    /// The stream its ruffle's phases and its colour's shade come from;
    /// [`LeafMetric`] reads it not.
    pub seed: u64,
}

impl LeafConfig {
    /// Its mass, in kilograms: what it weighs on a rod it rides, from the
    /// blade area its template draws.
    pub fn mass(&self) -> f32 {
        AREAL_MASS * blade::area(self)
    }
}

/// Two leaves share a mesh when they are built the same way and are close
/// in every dimension that shows, weighted as roughly how far apart each
/// moves the margin. A different template or tint is a different part, so
/// each is a step wider than any shape difference; the fields a template's
/// builder never reads count for nothing.
pub struct LeafMetric;

impl Metric<LeafConfig> for LeafMetric {
    fn distance(&self, a: &LeafConfig, b: &LeafConfig) -> f32 {
        let reach = a.length.max(b.length);
        let step = |same: bool| if same { 0.0 } else { 10.0 * reach.max(0.1) };
        let simple = (a.template == Template::Simple) as u8 as f32;
        let compound = (a.template == Template::Compound) as u8 as f32;
        [
            a.length - b.length,
            (a.width - b.width) * reach,
            a.petiole - b.petiole,
            // A pair more is two leaflets more.
            (a.pairs as f32 - b.pairs as f32) * reach / 6.0 * compound,
            // A full ruffle is a few percent of the length deep.
            (a.ruffle - b.ruffle) * reach / 20.0 * simple,
            // The edge's rise under the cup: the turn over half the width.
            (a.cup - b.cup) * reach / 4.0 * simple,
            (a.droop - b.droop) * reach / 2.0,
            step(a.template == b.template),
            step(a.tint == b.tint),
        ]
        .iter()
        .map(|d| d * d)
        .sum::<f32>()
        .sqrt()
    }
}

/// Run condition: a leaf of `S` was grown, regrown or dropped.
pub fn changed<S: Species>(
    leaves: Query<(), (Changed<LeafConfig>, With<OrganOf<S>>)>,
    mut removed: RemovedComponents<OrganOf<S>>,
) -> bool {
    let removed = removed.read().count() > 0;
    !leaves.is_empty() || removed
}

/// Builds one mesh per distinct leaf of `S` and gives it to every leaf that
/// drew it.
pub fn build<S: Species>(
    mut commands: Commands,
    mut library: Library,
    params: Res<S::Params>,
    leaves: Query<(Entity, &Order, &LeafConfig), With<OrganOf<S>>>,
) -> Result {
    let prefix = format!("{}Leaf", S::PREFIX);
    library.clear(&prefix);

    let mut hung: Vec<(Order, Entity, LeafConfig)> = leaves
        .iter()
        .map(|(entity, order, config)| (*order, entity, *config))
        .collect();
    hung.sort_by_key(|(order, ..)| *order);

    let configs: Vec<LeafConfig> = hung.iter().map(|(_, _, config)| *config).collect();
    let budget = S::variations(&params).max(1) as usize;
    let book = farthest_first(&configs, budget, 0.0, &LeafMetric);
    let meshes = par_map(&book.representatives, |_, config| blade::mesh(config));
    let mut geometry = Vec::with_capacity(meshes.len());
    for (index, (config, mesh)) in book.representatives.iter().zip(meshes).enumerate() {
        // Shaded off the representative's own stream, not its index, so a
        // budget change moves no shade that stays.
        let color = shade(config.tint, &mut Rng::new(COLOR_STREAM ^ config.seed));
        geometry.push(library.part(&prefix, index, mesh?.to_mesh(), FOLIAGE.blade(color)));
    }
    for ((_, entity, _), drew) in hung.iter().zip(&book.assignment) {
        commands
            .entity(*entity)
            .insert(geometry[*drew as usize].clone());
    }
    Ok(())
}

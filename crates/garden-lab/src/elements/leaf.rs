//! Leaf — every species' leaves meshed and shared, a system per species.
//!
//! [`build`] is the framework's quantized layer: it collects the species'
//! [`LeafConfig`]s in authoring order, clusters them to the species'
//! `variations` budget, meshes one leaf per representative and hands each
//! leaf the mesh it drew. A leaf has nothing hanging off it, so the leaf
//! entity is the geometry prim itself, placed by the plant layer; when the
//! leaves collide it is an Xform instead, over a `Blade` drawing the mesh
//! and a `Collision` box round the blade, since a prim that references a
//! part may carry no child.

use bevy::prelude::*;
use misina_lab::geometry::par_map;
use misina_lab::palette::{COLOR_STREAM, Response, shade};
use misina_lab::quantize::{Metric, farthest_first};
use misina_lab::rng::Rng;
use misina_lab::scene::{COLLISION, Library, Order, block};

use super::{PhysicsParams, SceneParams};
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
    /// Three leaflets on a petiole, entire or toothed: a pair where it ends
    /// and one past them.
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
    /// Notches cut into each edge of a simple blade, teeth into each edge
    /// of a trifoliate leaflet, or round a lobed blade's margin; 0 leaves
    /// it entire.
    pub teeth: u32,
    /// Secondary veins along each side of a simple blade or a leaflet, the
    /// lines its pleats fold on; a lobed blade pleats on its main veins.
    pub veins: u32,
    /// How far the blade stands up between two veins, as a fraction of the
    /// gap between them (of the distance out from its middle, on a lobed
    /// blade); 0 leaves it smooth.
    pub relief: f32,
    /// How much higher a simple blade's or a leaflet's pleats stand at its
    /// margin than off its midrib, as a multiple of its relief: a frilled
    /// edge.
    pub frill: f32,
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
    /// How far the blade turns about its midrib by its tip, in radians,
    /// either way as its own draw says.
    pub twist: f32,
    /// The blade's colour, linear RGB.
    pub tint: [f32; 3],
    /// The colour of the petiole, the midrib and the blade beside it, where
    /// it differs from the blade's; none paints them the tint.
    pub vein: Option<[f32; 3]>,
    /// The colour the margin and the tip fade to, where it differs from the
    /// blade's; none leaves them the tint.
    pub edge: Option<[f32; 3]>,
    /// The stream its ruffle's phases, its twist's way and its colour's
    /// shade come from; [`LeafMetric`] reads it not.
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
            // A tooth is a notch a few percent of a leaflet deep.
            (a.teeth as f32 - b.teeth as f32) * reach / 30.0,
            // A pleat stands a fraction of the gap between two veins high.
            (a.relief - b.relief) * reach / 10.0,
            (a.frill - b.frill) * (a.relief + b.relief) * reach / 20.0,
            (a.veins as f32 - b.veins as f32) * reach / 20.0,
            // A full ruffle is a few percent of the length deep.
            (a.ruffle - b.ruffle) * reach / 20.0 * simple,
            // The edge's rise under the cup: the turn over half the width.
            (a.cup - b.cup) * reach / 4.0 * simple,
            (a.droop - b.droop) * reach / 2.0,
            // A twist moves the margin by its turn over the half-width.
            (a.twist - b.twist) * reach / 4.0,
            step(a.template == b.template),
            step(a.tint == b.tint),
            step(a.vein == b.vein),
            step(a.edge == b.edge),
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
/// drew it, with the box round it when the leaves collide. A leaf is painted
/// per point, so its part's own colour is the representative's shade of
/// white.
pub fn build<S: Species>(
    mut commands: Commands,
    mut library: Library,
    params: Res<S::Params>,
    physics: Res<PhysicsParams>,
    scene: Res<SceneParams>,
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
    let detail = scene.mesh_detail();
    let meshes = par_map(&book.representatives, |_, config| {
        blade::mesh(config, detail)
    });
    let mut geometry = Vec::with_capacity(meshes.len());
    for (index, (config, mesh)) in book.representatives.iter().zip(meshes).enumerate() {
        let mesh = mesh?;
        // Shaded off the representative's own stream, not its index, so a
        // budget change moves no shade that stays.
        let color = shade([1.0; 3], &mut Rng::new(COLOR_STREAM ^ config.seed));
        let collider = physics
            .leaves_collide
            .then(|| blade::collider(config, &mesh));
        let part = library.part(&prefix, index, mesh.to_mesh(), FOLIAGE.blade(color));
        geometry.push((part, collider));
    }
    for ((_, entity, _), drew) in hung.iter().zip(&book.assignment) {
        let (part, collider) = &geometry[*drew as usize];
        match collider {
            Some((placed, half)) => {
                commands.entity(*entity).with_children(|leaf| {
                    leaf.spawn((Name::new("Blade"), part.clone()));
                    leaf.spawn((Name::new(COLLISION), block(*half, *placed)));
                });
            }
            None => {
                commands.entity(*entity).insert(part.clone());
            }
        }
    }
    Ok(())
}

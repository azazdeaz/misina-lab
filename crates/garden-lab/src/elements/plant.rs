//! Plant — one plant's recipe into its organs, a system per species.
//!
//! [`grow`] runs the species' [`Recipe`] for every plant of it, replacing
//! the organs it grew last time: its axes as one `Wood` mesh of its own, a
//! capsule chain along each axis when the stems collide, the crown, one
//! part per species scaled to each plant, a `Leaf_NN` child per leaf at its
//! anchor carrying the [`LeafConfig`](super::leaf::LeafConfig) the leaf
//! layer meshes, and a `Fruit_NN` Xform per fruit carrying its
//! [`FruitConfig`](super::fruit::FruitConfig) and harvest attributes, with
//! the `Skin` the fruit layer meshes, a `Cut` frame and its own capsule when
//! the fruit collide. A plant is grown in its own frame: its foot at the
//! origin, +Z up.

use std::f32::consts::FRAC_PI_2;

use bevy::prelude::*;
use misina_lab::geometry::mesh::{MeshData, cylinder_mesh, merge_meshes};
use misina_lab::geometry::strand::{Bark, Strand, sides_within, strand_mesh};
use misina_lab::palette::{Response, srgb};
use misina_lab::scene::{
    COLLISION, Geometry, Library, Order, UsdAttributes, capsule, capsule_between,
};
use nalgebra::Point3;

use super::PhysicsParams;
use super::fruit::{READY, Skin};
use super::planting::PlantConfig;
use crate::grow::axis::Grown;
use crate::grow::builder::Builder;
use crate::species::{Of, OrganOf, Recipe, Species};

/// Marks what a recipe grew on a plant, which every regrowth replaces.
#[derive(Component)]
pub struct Organ;

/// A crown stub: its sides, how much it narrows toward its top, and its
/// pale green.
const CROWN_SIDES: usize = 8;
const CROWN_TAPER: f32 = 0.85;
const CROWN: (u32, Response) = (
    0xB7C4A0,
    Response {
        roughness: 0.7,
        reflectance: 0.3,
    },
);

/// How an axis is meshed: how far its tube may stray from round, in meters,
/// and its rings per meter; and how it answers light.
const WOOD_TOLERANCE: f64 = 0.0015;
const WOOD_RINGS: f64 = 40.0;
const WOOD: Response = Response {
    roughness: 0.75,
    reflectance: 0.3,
};

/// How far a point of an axis may lie off the capsule covering it, in
/// meters.
const STRAY: f32 = 0.003;

/// An organ's `Order` is its plant's shifted by this, with the fruit above
/// the leaves; a plant has fewer organs than the fruit bit.
const ORGAN_BITS: u32 = 24;
const FRUIT_ORDER: u64 = 1 << 23;

/// Planting places a bed holds at most, which keeps a `Wood` part's index,
/// `bed * PLACES_PER_BED + place`, one per plant.
const PLACES_PER_BED: u64 = 10_000;

/// Run condition: a plant of `S` was planted, replanted or pulled.
pub fn changed<S: Species>(
    plants: Query<(), (Changed<PlantConfig>, With<Of<S>>)>,
    mut removed: RemovedComponents<Of<S>>,
) -> bool {
    // Read them all: one left unread would count again next frame.
    let removed = removed.read().count() > 0;
    !plants.is_empty() || removed
}

/// Grows every plant of `S` from its recipe, in authoring order.
///
/// Every recipe runs and every `Wood` is meshed before anything is spawned
/// or despawned, so a meshing error leaves the standing scene whole.
pub fn grow<S: Recipe>(
    mut commands: Commands,
    mut library: Library,
    params: Res<S::Params>,
    physics: Res<PhysicsParams>,
    plants: Query<(Entity, &Order, &PlantConfig), With<Of<S>>>,
    children: Query<&Children>,
    organs: Query<(), With<Organ>>,
) -> Result {
    let crown_prefix = format!("{}Crown", S::PREFIX);
    library.clear(S::PREFIX);
    library.clear(&crown_prefix);
    let mut grown: Vec<(Order, Entity, PlantConfig)> = plants
        .iter()
        .map(|(entity, order, config)| (*order, entity, *config))
        .collect();
    grown.sort_by_key(|(order, ..)| *order);
    let mut built: Vec<(Order, Entity, Builder, Option<Geometry>)> =
        Vec::with_capacity(grown.len());
    for (order, plant, config) in grown {
        let mut builder = Builder::new(&config);
        S::grow(&params, &config, &mut builder);
        // The plant's rigid axes, as one mesh of its own in the first axis's
        // colour: no two stems lie alike, and a herb's tube is a few
        // kilobytes. Named by the plant's bed and place, so a plant missing
        // renames no other's.
        let wood = match builder.axes.first() {
            Some(first) => {
                let tubes: anyhow::Result<Vec<MeshData>> = builder
                    .axes
                    .iter()
                    .map(|axis| strand_mesh(&strand(axis)))
                    .collect();
                let (bed, place) = (order.0 >> 32, order.0 & 0xFFFF_FFFF);
                Some(library.part(
                    S::PREFIX,
                    (bed * PLACES_PER_BED + place) as usize,
                    merge_meshes(&tubes?).to_mesh(),
                    WOOD.surface(first.tint),
                ))
            }
            None => None,
        };
        built.push((order, plant, builder, wood));
    }

    // One stub for the species, a unit radius and height, scaled per plant;
    // registered only if some plant grew one, so no part goes undrawn.
    let crown = built
        .iter()
        .any(|(_, _, builder, _)| builder.crown.is_some())
        .then(|| {
            let (color, response) = CROWN;
            library.part(
                &crown_prefix,
                0,
                cylinder_mesh(1.0, CROWN_TAPER, 1.0, CROWN_SIDES).to_mesh(),
                response.surface(srgb(color)),
            )
        });

    for (order, plant, builder, wood) in built {
        for &child in children.get(plant).into_iter().flatten() {
            if organs.contains(child) {
                commands.entity(child).despawn();
            }
        }
        let Builder {
            crown: grew,
            axes,
            leaves,
            fruits,
            ..
        } = builder;
        let organ_order = order.0 << ORGAN_BITS;
        commands.entity(plant).with_children(|parent| {
            if let Some(wood) = wood {
                parent.spawn((Organ, Name::new("Wood"), wood));
            }
            if physics.stems_collide {
                let mut k = 0;
                for axis in &axes {
                    for (radius, from, to) in chain(axis) {
                        // Straight across the whole span, its caps past both
                        // ends, so it overlaps the next one over their joint.
                        let out = (to - from).normalize_or_zero() * radius;
                        parent.spawn((
                            Organ,
                            Name::new(format!("{COLLISION}_{k:02}")),
                            capsule_between(radius, from - out, to + out),
                        ));
                        k += 1;
                    }
                }
            }
            if let (Some(stub), Some(geometry)) = (grew, &crown) {
                parent.spawn((
                    Organ,
                    Name::new("Crown"),
                    geometry.clone(),
                    Transform::from_xyz(0.0, 0.0, stub.base).with_scale(Vec3::new(
                        stub.radius,
                        stub.radius,
                        stub.height(),
                    )),
                ));
            }
            for (k, anchor, leaf) in leaves {
                parent.spawn((
                    Organ,
                    OrganOf::<S>::default(),
                    Name::new(format!("Leaf_{k:02}")),
                    anchor,
                    Visibility::default(),
                    leaf,
                    Order(organ_order | k),
                ));
            }
            for (k, (anchor, fruit)) in fruits.into_iter().enumerate() {
                parent
                    .spawn((
                        Organ,
                        OrganOf::<S>::default(),
                        Name::new(format!("Fruit_{k:02}")),
                        anchor,
                        Visibility::default(),
                        fruit,
                        Order(organ_order | FRUIT_ORDER | k as u64),
                        UsdAttributes::default()
                            .with("ripeness", fruit.ripeness)
                            .with("pick", fruit.kind.pick())
                            .with("harvest_ready", fruit.ripeness >= READY),
                    ))
                    .with_children(|prim| {
                        prim.spawn((
                            Skin,
                            Name::new("Skin"),
                            Transform::IDENTITY,
                            Visibility::default(),
                        ));
                        // Where a harvester takes it: the fruit's top, +X up
                        // the stalk.
                        prim.spawn((
                            Name::new("Cut"),
                            Transform::from_rotation(Quat::from_rotation_y(-FRAC_PI_2)),
                            Visibility::default(),
                        ));
                        if physics.fruit_collides {
                            // A capsule the fruit's whole length, its tips at
                            // the top and the bottom.
                            let radius = (fruit.diameter / 2.0).min(fruit.length / 2.0);
                            prim.spawn((Name::new(COLLISION), capsule(radius, -fruit.length, 0.0)));
                        }
                    });
            }
        });
    }
    Ok(())
}

/// An axis as a tube to mesh: round within [`WOOD_TOLERANCE`], unbarked.
fn strand(axis: &Grown) -> Strand {
    let points = axis
        .points
        .iter()
        .map(|p| Point3::new(p.x as f64, p.y as f64, p.z as f64))
        .collect();
    let radii = axis.radii.iter().map(|r| *r as f64).collect();
    let thickest = axis.radii.iter().copied().fold(0.0, f32::max) as f64;
    Strand::new(
        points,
        radii,
        sides_within(thickest, WOOD_TOLERANCE),
        WOOD_RINGS,
        Bark::none(),
    )
}

/// The capsules an axis collides as, from its foot to its tip: each a radius
/// and the two points of the axis it spans, straight between them, as thick
/// as the axis is anywhere along it, and no point between further than
/// [`STRAY`] off it.
fn chain(axis: &Grown) -> Vec<(f32, Vec3, Vec3)> {
    let (points, radii) = (&axis.points, &axis.radii);
    let off = |a: Vec3, b: Vec3, p: Vec3| {
        let d = b - a;
        p.distance(a + d * ((p - a).dot(d) / d.length_squared().max(1e-12)).clamp(0.0, 1.0))
    };
    let mut spans = Vec::new();
    let mut from = 0;
    while from + 1 < points.len() {
        let mut to = from + 1;
        while to + 1 < points.len()
            && (from + 1..=to).all(|i| off(points[from], points[to + 1], points[i]) < STRAY)
        {
            to += 1;
        }
        let radius = radii[from..=to].iter().copied().fold(0.0, f32::max);
        spans.push((radius, points[from], points[to]));
        from = to;
    }
    spans
}

#[cfg(test)]
mod tests {
    use misina_lab::rng::Rng;

    use super::*;
    use crate::grow::axis::Axis;

    /// A straight axis collides as one capsule, a leaning one as a few, each
    /// no thinner than the axis anywhere along it and together covering it
    /// foot to tip.
    #[test]
    fn a_chain_covers_the_axis_in_few_capsules() {
        let mut rng = Rng::new(1);
        let straight = Axis::upright(0.6, 0.02).grow(Vec3::ZERO, &mut rng);
        assert_eq!(chain(&straight).len(), 1);
        let bent = Axis::upright(0.6, 0.02)
            .lean(0.3)
            .grow(Vec3::ZERO, &mut rng);
        let spans = chain(&bent);
        assert!((2..8).contains(&spans.len()), "{} capsules", spans.len());
        assert_eq!(spans[0].1, bent.points[0]);
        assert_eq!(spans.last().unwrap().2, bent.tip());
        for pair in spans.windows(2) {
            assert_eq!(pair[0].2, pair[1].1, "the chain is joined");
        }
        assert!(spans.iter().all(|(radius, ..)| *radius >= 0.01));
    }
}

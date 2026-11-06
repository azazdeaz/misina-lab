//! Plant — one plant's recipe into its organs, a system per species.
//!
//! [`grow`] runs the species' [`Recipe`] for every plant of it, replacing
//! the organs it grew last time: its rigid axes as one `Wood` mesh of its
//! own, a capsule chain along each when the stems collide, the crown, one
//! part per species scaled to each plant, a `Leaf_NN` child per leaf at its
//! anchor carrying the [`LeafConfig`](super::leaf::LeafConfig) the leaf
//! layer meshes, and a `Fruit_NN` Xform per fruit carrying its
//! [`FruitConfig`](super::fruit::FruitConfig) and harvest attributes, with
//! the `Skin` the fruit layer meshes, a `Cut` frame and its own capsule when
//! the fruit collide. An axis that bends becomes a `Rod_NN` instead when
//! the scene's stems are flexible: the framework's cable, a prim per
//! segment carrying its tube, and under those the leaves and fruit that sit
//! on it, their masses in the cable's loads; the rod collides as itself. A
//! plant is grown in its own frame: its foot at the origin, +Z up.
//!
//! The wood is meshed to the scene's `detail`: its rings where its shape
//! needs them, swelling at every node an organ leaves it at, or left a scar
//! at, and toward that organ's stalk, so a petiole or a pedicel grows out of
//! the stem rather than poking through it, ridged along its length where the
//! recipe asked, and rounding off to a point past its highest organ. A rod's
//! segments are plain tubes, shared by size.

use std::collections::HashMap;
use std::f32::consts::{FRAC_PI_2, TAU};

use bevy::prelude::*;
use misina_lab::geometry::mesh::{MeshData, lathe, merge_meshes};
use misina_lab::geometry::strand::{Bark, Bulge, Collar, Strand, sides_within, strand_mesh};
use misina_lab::palette::Response;
use misina_lab::rng::{Rng, salt};
use misina_lab::scene::{
    COLLISION, Geometry, Library, Order, UsdAttributes, cable, capsule, capsule_between, rebase,
    segment_frames, segment_of, segment_tubes, spawn_rod,
};
use nalgebra::{Point3, Vector3};

use super::fruit::{READY, Skin};
use super::planting::PlantConfig;
use super::{PhysicsParams, SceneParams};
use crate::grow::axis::{Anchor, Grown};
use crate::grow::builder::{Builder, Crown};
use crate::shape::blade;
use crate::species::{Of, OrganOf, Recipe, Species};

/// Marks what a recipe grew on a plant, which every regrowth replaces.
#[derive(Component)]
pub struct Organ;

/// A crown: its sides, its rings from its top down to its foot, and how it
/// answers light.
const CROWN_SIDES: usize = 10;
const CROWN_RINGS: usize = 6;
const CROWN: Response = Response {
    roughness: 0.7,
    reflectance: 0.3,
};

/// How an axis is meshed: how far its tube may stray from its shape at a
/// detail of 1, in meters and at most this share of its thickest radius,
/// which also sets its sides, and the rings per meter its strand is built
/// with before the tolerance places them; and how it answers light.
const WOOD_TOLERANCE: f64 = 0.0015;
const WOOD_ROUNDNESS: f64 = 0.2;
const WOOD_RINGS: f64 = 40.0;
const WOOD: Response = Response {
    roughness: 0.75,
    reflectance: 0.3,
};

/// A node: how much the organs leaving an axis there swell it all round, as
/// a fraction of its radius, and how far up and down, in its radii, capped
/// at a third of the way to the next node so dense nodes stay distinct.
const NODE_SWELL: f64 = 0.1;
const NODE_WIDTH: f64 = 1.0;

/// How far short of its tip an axis starts rounding off, in its radii
/// there, and never below its highest organ.
const APEX: f64 = 3.0;

/// The stream an axis's ridges are laid out from, salted with the axis's
/// index.
const SURFACE_STREAM: u64 = 0x3B9A_C5E1_7D24_F80B;

/// How far a point of an axis may lie off the capsule covering it, in
/// meters.
const STRAY: f32 = 0.003;

/// A rod: its held first segment, in meters, the framework's usual, so an
/// axis has to be two of them long for a segment that moves; the sides of a
/// segment's tube; and the grid the tubes are shared on, in meters: two
/// segments alike in length and end widths to it draw one part.
const HELD: f32 = 0.0135;
const TUBE_SIDES: usize = 8;
const TUBE_GRID: f32 = 0.0002;

/// An organ's `Order` is its plant's shifted by this, with the fruit above
/// the leaves; a plant has fewer organs than the fruit bit.
const ORGAN_BITS: u32 = 24;
const FRUIT_ORDER: u64 = 1 << 23;

/// Planting places a bed holds at most, which keeps a `Wood` part's index,
/// `bed * PLACES_PER_BED + place`, one per plant.
const PLACES_PER_BED: u64 = 10_000;

/// One plant grown and meshed, waiting to be spawned: its order, its
/// entity, what its recipe collected, its wood if it has any, and its rods.
type Built = (Order, Entity, Builder, Option<Geometry>, Vec<Rod>);

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
/// Every recipe runs and every mesh is registered before anything is
/// spawned or despawned, so a meshing error leaves the standing scene whole.
#[allow(clippy::too_many_arguments)]
pub fn grow<S: Recipe>(
    mut commands: Commands,
    mut library: Library,
    params: Res<S::Params>,
    physics: Res<PhysicsParams>,
    scene: Res<SceneParams>,
    plants: Query<(Entity, &Order, &PlantConfig), With<Of<S>>>,
    children: Query<&Children>,
    organs: Query<(), With<Organ>>,
) -> Result {
    let crown_prefix = format!("{}Crown", S::PREFIX);
    let tube_prefix = format!("{}Tube", S::PREFIX);
    library.clear(S::PREFIX);
    library.clear(&crown_prefix);
    library.clear(&tube_prefix);
    let detail = scene.mesh_detail();
    let mut grown: Vec<(Order, Entity, PlantConfig)> = plants
        .iter()
        .map(|(entity, order, config)| (*order, entity, *config))
        .collect();
    grown.sort_by_key(|(order, ..)| *order);
    // The rods' tubes, shared across the species by size as they are met.
    let mut tubes: HashMap<[u32; 6], Geometry> = HashMap::new();
    let mut built: Vec<Built> = Vec::with_capacity(grown.len());
    for (order, plant, config) in grown {
        let mut builder = Builder::new(&config);
        S::grow(&params, &config, &mut builder);
        let (bending, rigid): (Vec<&Grown>, Vec<&Grown>) = builder
            .axes
            .iter()
            .partition(|axis| physics.flexible_stems && axis.bends && axis.length() > 2.0 * HELD);
        // Where every organ leaves its axis, which the wood swells toward: a
        // leaf, or the scar of one, along its frame's +X, a fruit down its -Z.
        let stalks: Vec<Stalk> = builder
            .leaves
            .iter()
            .map(|(_, anchor, leaf)| Stalk {
                anchor: *anchor,
                toward: anchor.frame.rotation * Vec3::X,
                radius: blade::stalk_radius(leaf),
            })
            .chain(builder.fruits.iter().map(|(_, anchor, fruit)| Stalk {
                anchor: *anchor,
                toward: anchor.frame.rotation * -Vec3::Z,
                radius: fruit.stalk_radius(),
            }))
            .chain(builder.scars.iter().map(|(anchor, radius)| Stalk {
                anchor: *anchor,
                toward: anchor.frame.rotation * Vec3::X,
                radius: *radius,
            }))
            .collect();
        // The plant's rigid axes, as one mesh of its own in the first axis's
        // colour: no two stems lie alike, and a herb's tube is a few
        // kilobytes. Named by the plant's bed and place, so a plant missing
        // renames no other's.
        let wood = match rigid.first() {
            Some(first) => {
                let tubes: anyhow::Result<Vec<MeshData>> = rigid
                    .iter()
                    .map(|axis| strand_mesh(&strand(axis, &stalks, config.seed, detail)))
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
        let mut rods: Vec<Rod> = bending
            .into_iter()
            .map(|axis| {
                Rod::new(
                    axis,
                    physics.segment,
                    &mut library,
                    &tube_prefix,
                    &mut tubes,
                )
            })
            .collect();
        for (_, anchor, leaf) in &builder.leaves {
            load(&mut rods, anchor, leaf.mass());
        }
        for (_, anchor, fruit) in &builder.fruits {
            load(&mut rods, anchor, fruit.mass());
        }
        built.push((order, plant, builder, wood, rods));
    }

    // One crown for the species, a unit radius and height, scaled per plant
    // and in the first one's tint; registered only if some plant grew one,
    // so no part goes undrawn.
    let crown = built
        .iter()
        .find_map(|(_, _, builder, ..)| builder.crown)
        .map(|first| {
            library.part(
                &crown_prefix,
                0,
                dome().to_mesh(),
                CROWN.surface(first.tint),
            )
        });

    for (order, plant, builder, wood, rods) in built {
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
        if let Some(wood) = wood {
            commands.spawn((Organ, Name::new("Wood"), wood, ChildOf(plant)));
        }
        if physics.stems_collide {
            let mut k = 0;
            for axis in axes
                .iter()
                .filter(|axis| !rods.iter().any(|rod| rod.axis == axis.index))
            {
                for (radius, from, to) in chain(axis) {
                    // Straight across the whole span, its caps past both
                    // ends, so it overlaps the next one over their joint.
                    let out = (to - from).normalize_or_zero() * radius;
                    commands.spawn((
                        Organ,
                        Name::new(format!("{COLLISION}_{k:02}")),
                        capsule_between(radius, from - out, to + out),
                        ChildOf(plant),
                    ));
                    k += 1;
                }
            }
        }
        if let (Some(stub), Some(geometry)) = (grew, &crown) {
            commands.spawn((
                Organ,
                Name::new("Crown"),
                geometry.clone(),
                Transform::from_xyz(0.0, 0.0, stub.base).with_scale(Vec3::new(
                    stub.radius,
                    stub.radius,
                    stub.height(),
                )),
                ChildOf(plant),
            ));
        }
        // Each rod under an Xform of its own at the plant's frame, so the
        // segments are siblings of the cable, where the solver looks.
        let rods: Vec<(Rod, Vec<Entity>)> = rods
            .into_iter()
            .map(|rod| {
                let mut organ = commands.spawn((
                    Organ,
                    Name::new(format!("Rod_{:02}", rod.axis)),
                    Transform::IDENTITY,
                    Visibility::default(),
                    ChildOf(plant),
                ));
                let curve = cable(
                    rod.points.clone(),
                    rod.widths.clone(),
                    rod.loads.clone(),
                    WOOD.surface(rod.tint),
                );
                let segments = spawn_rod(&mut organ, curve, &rod.frames, &rod.tubes);
                (rod, segments)
            })
            .collect();
        for (k, anchor, leaf) in leaves {
            let (host, placement) = host(&anchor, &rods, plant);
            commands.spawn((
                Organ,
                OrganOf::<S>::default(),
                Name::new(format!("Leaf_{k:02}")),
                placement,
                Visibility::default(),
                leaf,
                Order(organ_order | k),
                ChildOf(host),
            ));
        }
        for (k, anchor, fruit) in fruits {
            let (host, placement) = host(&anchor, &rods, plant);
            commands
                .spawn((
                    Organ,
                    OrganOf::<S>::default(),
                    Name::new(format!("Fruit_{k:02}")),
                    placement,
                    Visibility::default(),
                    fruit,
                    Order(organ_order | FRUIT_ORDER | k),
                    UsdAttributes::default()
                        .with("ripeness", fruit.ripeness)
                        .with("pick", fruit.kind.pick())
                        .with("harvest_ready", fruit.ripeness >= READY),
                    ChildOf(host),
                ))
                .with_children(|prim| {
                    prim.spawn((
                        Skin,
                        Name::new("Skin"),
                        Transform::IDENTITY,
                        Visibility::default(),
                    ));
                    // Where a harvester takes it: halfway down its stalk,
                    // or the fruit's top without one, +X up the stalk.
                    prim.spawn((
                        Name::new("Cut"),
                        Transform::from_xyz(0.0, 0.0, -fruit.stalk / 2.0)
                            .with_rotation(Quat::from_rotation_y(-FRAC_PI_2)),
                        Visibility::default(),
                    ));
                    if physics.fruit_collides {
                        // A capsule the body's whole length, its tips at
                        // the top and the bottom, below the stalk.
                        let radius = (fruit.diameter / 2.0).min(fruit.length / 2.0);
                        prim.spawn((
                            Name::new(COLLISION),
                            capsule(radius, -fruit.stalk - fruit.length, -fruit.stalk),
                        ));
                    }
                });
        }
    }
    Ok(())
}

/// A bending axis as a rod: its points at `stations` along the axis, the
/// frames and tubes of its segments, and what its riders weigh on each
/// point.
struct Rod {
    /// The axis it stands in for, by its index among the plant's.
    axis: usize,
    stations: Vec<f32>,
    points: Vec<[f32; 3]>,
    widths: Vec<f32>,
    frames: Vec<Transform>,
    tubes: Vec<Geometry>,
    loads: Vec<f32>,
    tint: [f32; 3],
}

impl Rod {
    /// `axis` sampled at the held stub and then even steps as near `segment`
    /// as divide the rest, its tubes drawn from `shared` or registered under
    /// `prefix` as the next part.
    fn new(
        axis: &Grown,
        segment: f32,
        library: &mut Library,
        prefix: &str,
        shared: &mut HashMap<[u32; 6], Geometry>,
    ) -> Self {
        let length = axis.length();
        let steps = ((length - HELD) / segment.max(0.005)).round().max(1.0);
        let mut stations = vec![0.0, HELD];
        stations.extend((1..=steps as usize).map(|i| HELD + (length - HELD) * i as f32 / steps));
        let (points, widths): (Vec<[f32; 3]>, Vec<f32>) = stations
            .iter()
            .map(|&station| {
                let (at, radius) = axis.at(station);
                (at.to_array(), 2.0 * radius)
            })
            .unzip();
        let frames = segment_frames(&points);
        let surface = WOOD.surface(axis.tint);
        let grid = |x: f32| (x / TUBE_GRID).round() as u32;
        let [r, g, b] = axis.tint.map(f32::to_bits);
        let tubes = segment_tubes(&points, &widths, TUBE_SIDES)
            .into_iter()
            .zip(points.windows(2).zip(widths.windows(2)))
            .map(|(mesh, (pair, ends))| {
                let span = Vec3::from(pair[0]).distance(pair[1].into());
                let key = [grid(span), grid(ends[0]), grid(ends[1]), r, g, b];
                let next = shared.len();
                shared
                    .entry(key)
                    .or_insert_with(|| library.part(prefix, next, mesh.to_mesh(), surface))
                    .clone()
            })
            .collect();
        Self {
            axis: axis.index,
            loads: vec![0.0; points.len()],
            stations,
            points,
            widths,
            frames,
            tubes,
            tint: axis.tint,
        }
    }
}

/// Puts `mass` on the rod `anchor` sits on, if it sits on one: split
/// between the two points of its segment, the nearer taking more.
fn load(rods: &mut [Rod], anchor: &Anchor, mass: f32) {
    let Some((axis, station)) = anchor.on else {
        return;
    };
    let Some(rod) = rods.iter_mut().find(|rod| rod.axis == axis) else {
        return;
    };
    let k = segment_of(&rod.stations, station);
    let (from, to) = (rod.stations[k], rod.stations[k + 1]);
    let t = ((station - from) / (to - from).max(1e-6)).clamp(0.0, 1.0);
    rod.loads[k] += mass * (1.0 - t);
    rod.loads[k + 1] += mass * t;
}

/// What an organ at `anchor` is spawned under, and where: the segment prim
/// that carries it, in that segment's frame, when its axis is a rod, else
/// the plant, as it is.
fn host(anchor: &Anchor, rods: &[(Rod, Vec<Entity>)], plant: Entity) -> (Entity, Transform) {
    anchor
        .on
        .and_then(|(axis, station)| {
            rods.iter()
                .find(|(rod, _)| rod.axis == axis)
                .map(|rod| (rod, station))
        })
        .map(|((rod, segments), station)| {
            let k = segment_of(&rod.stations, station);
            (segments[k], rebase(&rod.frames[k], &anchor.frame))
        })
        .unwrap_or((plant, anchor.frame))
}

/// An organ's stalk as it leaves the plant: where it is anchored, which
/// way the stalk heads and how thick it is.
struct Stalk {
    anchor: Anchor,
    toward: Vec3,
    radius: f32,
}

/// The crown of a unit radius and height: a dome turned from
/// [`Crown::profile`], its rings closer together toward its top, where it
/// turns, and a disc under its foot.
fn dome() -> MeshData {
    let rings: Vec<Vec<Vec3>> = (1..=CROWN_RINGS)
        .map(|k| {
            let up = 1.0 - (k as f32 / CROWN_RINGS as f32).powi(2);
            let radius = Crown::profile(up);
            (0..CROWN_SIDES)
                .map(|i| {
                    let angle = TAU * i as f32 / CROWN_SIDES as f32;
                    Vec3::new(radius * angle.cos(), radius * angle.sin(), up)
                })
                .collect()
        })
        .collect();
    lathe(Vec3::Z, &rings, Vec3::ZERO)
}

/// An axis as a tube to mesh: within [`WOOD_TOLERANCE`] of its shape at
/// `detail`, or [`WOOD_ROUNDNESS`] of its radius if that is less, its
/// rings where the shape needs them, swollen at every node
/// one of `stalks` leaves it at and toward each of them, ridged from the
/// plant's `seed` and its own index if its recipe asked, and rounding off
/// over its last [`APEX`] radii, or what is left past its highest stalk.
fn strand(axis: &Grown, stalks: &[Stalk], seed: u64, detail: f32) -> Strand {
    let points = axis
        .points
        .iter()
        .map(|p| Point3::new(p.x as f64, p.y as f64, p.z as f64))
        .collect();
    let radii = axis.radii.iter().map(|r| *r as f64).collect();
    let thickest = axis.radii.iter().copied().fold(0.0, f32::max) as f64;
    let tolerance = WOOD_TOLERANCE.min(WOOD_ROUNDNESS * thickest) / detail as f64;
    let sides = sides_within(thickest, tolerance);
    let bark = if axis.ridges > 0.0 {
        let mut rng = Rng::new(seed ^ SURFACE_STREAM ^ salt(axis.index as u64));
        Bark::new(&mut rng, sides, axis.ridges as f64)
    } else {
        Bark::none()
    };
    let leaving: Vec<(f32, &Stalk)> = stalks
        .iter()
        .filter_map(|stalk| {
            let (on, station) = stalk.anchor.on?;
            (on == axis.index).then_some((station, stalk))
        })
        .collect();
    // One swelling per node, however many organs leave it.
    let mut nodes: Vec<f32> = leaving.iter().map(|(station, _)| *station).collect();
    nodes.sort_by(f32::total_cmp);
    nodes.dedup_by(|a, b| (*a - *b).abs() < 1e-4);
    let bulges = nodes
        .iter()
        .enumerate()
        .map(|(k, &station)| {
            let (_, radius) = axis.at(station);
            let gap = [k.checked_sub(1), (k + 1 < nodes.len()).then_some(k + 1)]
                .into_iter()
                .flatten()
                .map(|j| (nodes[j] - station).abs() as f64)
                .fold(f64::INFINITY, f64::min);
            Bulge {
                at: station as f64,
                width: (NODE_WIDTH * radius as f64).min(gap / 3.0),
                height: NODE_SWELL,
            }
        })
        .collect();
    let collars = leaving
        .iter()
        .map(|(station, stalk)| {
            let (_, radius) = axis.at(*station);
            Collar {
                at: *station as f64,
                toward: vector(stalk.toward),
                radius: stalk.radius.min(radius) as f64,
            }
        })
        .collect();
    let highest = nodes.last().copied().unwrap_or(0.0);
    let tip = *axis.radii.last().unwrap() as f64;
    Strand {
        bulges,
        collars,
        tolerance: Some(tolerance),
        apex: (APEX * tip).min((axis.length() - highest).max(0.0) as f64),
        ..Strand::new(points, radii, sides, WOOD_RINGS, bark)
    }
}

fn vector(v: Vec3) -> Vector3<f64> {
    Vector3::new(v.x as f64, v.y as f64, v.z as f64)
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
    use std::f32::consts::PI;

    use misina_lab::rng::Rng;

    use super::super::leaf::LeafConfig;
    use super::*;
    use crate::grow::axis::Axis;
    use crate::grow::phyllotaxis::AxisNode;

    /// A stem's strand is meshed to the wood tolerance at the scene's
    /// detail, swells once at every node its leaves leave it at, however
    /// many, and toward each leaf's petiole, so it is wider at a node than
    /// between two, and rounds off over its last radii, or what is left past
    /// a leaf near its tip; an axis grown with ridges is ridged, one without
    /// is round up to its apex.
    #[test]
    fn a_stem_swells_at_its_nodes_toward_its_leaves_and_may_be_ridged() {
        let mut rng = Rng::new(2);
        let stem = Axis::upright(0.5, 0.01)
            .taper(1.0)
            .grow(Vec3::ZERO, &mut rng);
        let leaf = LeafConfig::simple(0.2, 0.6).petiole(0.08);
        let stalk = |station: f32, azimuth: f32| {
            let node = AxisNode {
                rank: 0,
                count: 1,
                station,
                azimuth,
            };
            let anchor = stem.anchor(&node, 0.3);
            Stalk {
                anchor,
                toward: anchor.frame.rotation * Vec3::X,
                radius: blade::stalk_radius(&leaf),
            }
        };
        let stalks = [stalk(0.2, 0.0), stalk(0.2, PI), stalk(0.35, 1.0)];
        let swollen = strand(&stem, &stalks, 1, 2.0);
        assert_eq!(swollen.bulges.len(), 2, "one bulge per node");
        assert_eq!(swollen.collars.len(), 3, "a collar per leaf");
        let tolerance = |strand: &Strand| strand.tolerance.unwrap();
        assert!((tolerance(&swollen) - WOOD_TOLERANCE / 2.0).abs() < 1e-9);
        let thin = Axis::upright(0.5, 0.003).grow(Vec3::ZERO, &mut rng);
        let thin = strand(&thin, &[], 1, 2.0);
        assert!((tolerance(&thin) - WOOD_ROUNDNESS * 0.003 / 2.0).abs() < 1e-9);
        assert!((swollen.apex - APEX * 0.01).abs() < 1e-9);
        let topped = strand(&stem, &[stalk(0.49, 0.0)], 1, 2.0);
        assert!((topped.apex - 0.01).abs() < 1e-6, "{}", topped.apex);
        let mesh = strand_mesh(&swollen).unwrap();
        let widest_near = |z: f32| {
            mesh.points
                .iter()
                .filter(|p| (p[2] - z).abs() < 0.015)
                .map(|p| p[0].hypot(p[1]))
                .fold(0.0, f32::max)
        };
        assert!(
            widest_near(0.2) > 0.0115,
            "{} at the node",
            widest_near(0.2)
        );
        assert!(widest_near(0.45) < 0.0102, "{} between", widest_near(0.45));

        // How far the radius strays round a tube of one radius up to its
        // apex.
        let roundness = |axis: &Grown| {
            let mesh = strand_mesh(&strand(axis, &[], 1, 1.0)).unwrap();
            let radii: Vec<f32> = mesh
                .points
                .iter()
                .filter(|p| p[2] < 0.5 - APEX as f32 * 0.01)
                .map(|p| p[0].hypot(p[1]))
                .collect();
            radii.iter().copied().fold(0.0, f32::max) - radii.iter().copied().fold(1.0, f32::min)
        };
        let ridged = Axis::upright(0.5, 0.01)
            .taper(1.0)
            .ridges(0.1)
            .grow(Vec3::ZERO, &mut rng);
        assert!(roundness(&ridged) > 0.0005, "{}", roundness(&ridged));
        assert!(roundness(&stem) < 1e-5, "{}", roundness(&stem));
    }

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

    /// A load lands on the two points of the segment its station falls in,
    /// the nearer taking more, and nowhere when the anchor sits on no rod.
    #[test]
    fn a_load_splits_between_its_segments_points() {
        let mut rod = Rod {
            axis: 2,
            stations: vec![0.0, HELD, 0.1, 0.2],
            points: vec![],
            widths: vec![],
            frames: vec![],
            tubes: vec![],
            loads: vec![0.0; 4],
            tint: [0.0; 3],
        };
        let on = |axis, station| Anchor {
            frame: Transform::IDENTITY,
            on: Some((axis, station)),
        };
        load(std::slice::from_mut(&mut rod), &on(2, 0.175), 0.4);
        load(std::slice::from_mut(&mut rod), &on(1, 0.175), 9.0);
        load(
            std::slice::from_mut(&mut rod),
            &Transform::IDENTITY.into(),
            9.0,
        );
        assert!((rod.loads[2] - 0.1).abs() < 1e-6 && (rod.loads[3] - 0.3).abs() < 1e-6);
        assert_eq!(rod.loads[..2], [0.0, 0.0]);
    }
}

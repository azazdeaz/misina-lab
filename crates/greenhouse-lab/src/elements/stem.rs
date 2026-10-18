//! Stem — the crop: two stems to a plant, trained in a V up to the wires.
//!
//! A high-wire crop is lowered as it grows. Every week the hooks are slid
//! along the wire and the stems let down, so the oldest part of a stem lies
//! bare beside the gutter and only its top few meters stand. A stem is built
//! that way, as one strand from the block it is rooted in:
//!
//! ```text
//!         ┬   the hook on the wire, its twine reel HOOK below
//!         │   the head, tied under the reel
//!         │   a truss every third node, the lowest at `harvest_height`
//!        ╱    the rise, out to its wire
//!   ════╯     the lowered run, `lowered` long, beside the gutter
//!  ▣          the block
//! ```
//!
//! The runs go round the gutter the way growers move them: toward the far
//! end on the left, round the gutter's end, and back toward the row head on
//! the right. A stem rooted near an end may so rise on the other wire, and
//! every stretch of the row stands as dense as the rest.
//!
//! The live height from `harvest_height` to the head is shared out between
//! `trusses`, so how much stem a truss takes follows from the wire height:
//! 0.34 m at the defaults, inside the 25–35 cm a week's growth gives.
//!
//! No two stems lie along the gutter alike, so each is a mesh of its own and
//! nothing here is quantized. A stem authors a [`TrussConfig`] at every third
//! node of its rise, and [`truss`](super::truss) builds them.
//!
//! ```text
//! Row_02/Plant_017       Xform on the block's top
//!   Stem_0               Xform
//!     Tube               -> parts/Stem_n, this stem's own
//!     Hook               -> parts/Hook_0, on the wire
//!     Twine              -> parts/Twine_n, wound down the rise
//!     Truss_00 …         a TrussConfig, on the stem's axis at its node
//! ```

use std::f32::consts::{FRAC_PI_2, PI, TAU};

use bevy::prelude::*;
use misina_lab::geometry::mesh::{MeshData, cylinder_mesh, merge_meshes};
use misina_lab::geometry::par_map;
use misina_lab::geometry::strand::{Bark, Strand, strand_mesh};
use misina_lab::palette::{Response, srgb};
use misina_lab::params::{Label, Slider};
use misina_lab::rng::{Rng, salt};
use misina_lab::scene::{Library, Order};
use nalgebra::Point3;

use super::gutter::{self, GutterParams};
use super::layout::{GreenhouseLayout, Row};
use super::truss::{Node, TrussConfig, TrussParams};
use super::wire::WireParams;
use super::{Grow, SceneParams};

/// The mesh-library prefixes: a tube and a twine for every stem, and one hook
/// for them all.
const TUBE: &str = "Stem";
const TWINE: &str = "Twine";
const HOOK: &str = "Hook";

/// Salts splitting a plant's draws and a stem's off the scene seed.
const PLANT_STREAM: u64 = 0x9E37_79B9_7F4A_7C15;
const STEM_STREAM: u64 = 0xBF58_476D_1CE4_E5B9;

/// Nodes per truss: a truss at every third node, a leaf at each of the rest.
const NODES_PER_TRUSS: u32 = 3;

/// The most stems a plant carries: two, and an extra one.
const MAX_STEMS: usize = 3;

/// A hook's reach below the wire, in meters, which is how far below it a head
/// is held; the hook's wire and its twine reel.
const HOOK_LENGTH: f32 = 0.22;
const HOOK_RADIUS: f32 = 0.002;
const REEL: Vec2 = Vec2::new(0.018, 0.02);

/// A stem's radius, in meters, drawn per stem: 12 to 20 mm across.
const RADIUS: (f64, f64) = (0.006, 0.010);

/// How much thinner a stem is at its head, as a fraction, and over how much of
/// the rise below the head it thins, in meters.
const HEAD_TAPER: f32 = 0.4;
const HEAD_REACH: f32 = 0.8;

/// How deep a stem starts in its block, and how far it stands out of it before
/// it lies over, in meters.
const ROOT_DEPTH: f32 = 0.03;
const ROOT_RISE: f32 = 0.04;

/// Where the runs lie: out past the gutter's rim, and how far the stems of one
/// run spread across and up from there, in meters.
const RUN_OUT: f32 = 0.02;
const RUN_SPREAD: Vec2 = Vec2::new(0.04, 0.03);

/// Stem between the block and the run, and the spacing of a run's control
/// points, in meters.
const RUN_START: f32 = 0.12;
const RUN_STEP: f32 = 0.1;

/// How far in from each end of the gutter the runs turn round it, in meters.
const RUN_TURN: f32 = 0.2;

/// How far a run's length varies about `lowered`, as a fraction.
const LOWERED_JITTER: f64 = 0.15;

/// How much further an extra stem is lowered, in meters, so that it rises
/// between its plant's two rather than beside one of them.
const EXTRA_RUN: f32 = 0.15;

/// The bend from the run into the rise: a quarter circle of this radius, in
/// meters, through this many control points.
const BEND: f32 = 0.15;
const BEND_POINTS: usize = 4;

/// How many trusses below the head a truss's fruit reach full size: the
/// trusses nearest the head set their fruit last.
const GROWING_TRUSSES: f32 = 5.0;

/// How far a hook sits along the wire from straight above the bend, and how
/// far a stem bows off the straight line between them, in meters.
const HOOK_JITTER: f64 = 0.05;
const BOW: f32 = 0.03;

/// Least height between the bend, the lowest truss and the head, in meters,
/// so a wire set too low still grows a stem.
const MIN_LIVE: f32 = 0.3;

/// How far a truss turned to the path wanders off facing it, in radians.
const FACING_SPREAD: f64 = 0.5;

/// How far a stem's ripening runs ahead of or behind the rest, in trusses.
const RIPENING_JITTER: f64 = 0.3;

/// The twine: its radius, the stem one turn round it takes, and the spacing
/// of its control points, in meters.
const TWINE_RADIUS: f32 = 0.001;
const TWINE_PITCH: f32 = 0.3;
const TWINE_STEP: f32 = 0.03;

/// Vertices around a stem and round its twine, and rings per meter along
/// both.
const SIDES: usize = 8;
const TWINE_SIDES: usize = 4;
const DETAIL: f64 = 16.0;

/// A green stem, matte from its hairs.
const STEM_COLOR: u32 = 0x5F7F35;
const STEM_RESPONSE: Response = Response {
    roughness: 0.75,
    reflectance: 0.35,
};

/// Black plastic hooks and white polypropylene twine.
const HOOK_COLOR: u32 = 0x2B2B2B;
const TWINE_COLOR: u32 = 0xE6E1D3;
const PLASTIC: Response = Response {
    roughness: 0.5,
    reflectance: 0.5,
};

/// The crop's stems: two to a plant, trained alternately to the left and the
/// right wire, and lowered as they grow.
///
/// Each stem lies `lowered` meters along the gutter, then rises to its hook,
/// with its head a hook's length below the wire. The stem between
/// `harvest_height` and the head carries `trusses` trusses at every third
/// node, so how much stem a truss takes follows from the wire height rather
/// than being set.
#[derive(Resource, Reflect, Clone, Debug, PartialEq)]
#[cfg_attr(
    feature = "python",
    pyo3::pyclass(get_all, set_all, skip_from_py_object)
)]
pub struct StemParams {
    /// Height above the floor of the lowest truss still on a stem, in meters:
    /// where a harvesting arm reaches to. The stem below it has been picked
    /// and stripped bare.
    #[reflect(@Slider { min: 0.8, max: 1.8, step: 0.05 })]
    pub harvest_height: f32,
    /// Trusses on a stem, from the lowest still on it up to the head.
    #[reflect(@Slider { min: 6.0, max: 12.0, step: 1.0 })]
    pub trusses: u32,
    /// Length of bare stem lying beside the gutter, in meters: how far into
    /// the season the crop is, at about 25 cm of lowering a week. 0 is a crop
    /// before its first lowering.
    #[reflect(@Slider { min: 0.0, max: 6.0, step: 0.1 })]
    pub lowered: f32,
    /// Share of plants grown on with a third stem, which growers keep to raise
    /// the stem density.
    #[reflect(@Slider { min: 0.0, max: 0.5, step: 0.05 })]
    pub extra_stems: f32,
    /// Share of trusses turned to face the path. The rest face anywhere, into
    /// the crop as often as not, where a harvester has to skip them.
    #[reflect(@Slider { min: 0.0, max: 1.0, step: 0.05 }, @Label("Trusses turned to the path"))]
    pub turned_to_path: f32,
}

impl Default for StemParams {
    fn default() -> Self {
        Self {
            harvest_height: 1.2,
            trusses: 9,
            lowered: 2.0,
            extra_stems: 0.0,
            turned_to_path: 0.8,
        }
    }
}

/// Marks a plant's prim, which every rebuild replaces with all it holds.
#[derive(Component)]
struct Owned;

pub fn plugin(app: &mut App) {
    // `TrussParams` is not gated on: `truss` re-applies it to the configs
    // authored here, in place.
    app.init_resource::<StemParams>().add_systems(
        PreUpdate,
        author
            .run_if(
                resource_changed::<StemParams>
                    .or_eager(resource_changed::<GutterParams>)
                    .or_eager(resource_changed::<WireParams>)
                    .or_eager(resource_changed::<SceneParams>)
                    .or_eager(resource_changed::<GreenhouseLayout>),
            )
            .in_set(Grow::Stems),
    );
}

/// The path the runs follow round one gutter, `out` from its centreline: out
/// along the left side, round the far end, back along the right and round
/// the row head. A place on it is how far round it is, from the row head on
/// the left.
#[derive(Clone, Copy, Debug)]
struct Round {
    from: f32,
    to: f32,
    out: f32,
}

impl Round {
    fn straight(&self) -> f32 {
        self.to - self.from
    }

    fn turn(&self) -> f32 {
        PI * self.out
    }

    fn length(&self) -> f32 {
        2.0 * (self.straight() + self.turn())
    }

    /// Where `u` round puts a stem in the row, and which way it runs there.
    fn at(&self, u: f32) -> (Vec2, Vec2) {
        let (straight, turn) = (self.straight(), self.turn());
        let u = u.rem_euclid(self.length());
        if u < straight {
            return (Vec2::new(self.from + u, self.out), Vec2::X);
        }
        let u = u - straight;
        if u < turn {
            let a = u / self.out;
            return (
                Vec2::new(self.to + self.out * a.sin(), self.out * a.cos()),
                Vec2::new(a.cos(), -a.sin()),
            );
        }
        let u = u - turn;
        if u < straight {
            return (Vec2::new(self.to - u, -self.out), -Vec2::X);
        }
        let a = (u - straight) / self.out;
        (
            Vec2::new(self.from - self.out * a.sin(), -self.out * a.cos()),
            Vec2::new(-a.cos(), a.sin()),
        )
    }

    /// Where a stem rooted at `x` starts round, on the left (`side` 1) or
    /// the right (-1).
    fn start(&self, x: f32, side: f32) -> f32 {
        let x = x.clamp(self.from, self.to);
        if side > 0.0 {
            x - self.from
        } else {
            self.straight() + self.turn() + self.to - x
        }
    }

    /// `u`, or the end of the turn it falls in: a stem rises beside the
    /// gutter, never off its end.
    fn beside(&self, u: f32) -> f32 {
        let (straight, turn) = (self.straight(), self.turn());
        let w = u.rem_euclid(self.length());
        let past = if w < straight {
            0.0
        } else if w < straight + turn {
            straight + turn - w
        } else if w < 2.0 * straight + turn {
            0.0
        } else {
            self.length() - w
        };
        u + past
    }
}

/// What a stem's height is shared out from, in the row frame.
#[derive(Clone, Copy, Debug)]
struct Live {
    /// Where the runs lie: the gutter's rim.
    rim: f32,
    head: f32,
    harvest: f32,
    trusses: u32,
    spread: f32,
    /// Where along the row the runs turn round the gutter.
    ends: (f32, f32),
}

/// One stem, solved in the row frame: everything its meshes and its trusses
/// are made from.
#[derive(Clone, Debug)]
struct Stem {
    /// The tube's control points, root to head, and its radius at each.
    points: Vec<Vec3>,
    radii: Vec<f32>,
    /// The twine's, from the reel down the rise.
    twine: Vec<Vec3>,
    /// The hook's place on its wire.
    hook: Vec3,
    /// Each truss, lowest first: its node on the stem's axis, the yaw it
    /// faces, and what it takes from the stem there.
    trusses: Vec<(Vec3, f32, Node)>,
}

/// Grows the stem rooted at `root` that starts round on `side`, lowered
/// `lowered` meters, from its own stream.
///
/// The *draw order* is part of this element's output: the radius, the run's
/// length, where across and up in the run it lies, the hook's place along the
/// wire, the bow's bearing and the ripening offset, then three per truss —
/// whether it was turned to the path, how far it wanders off it, and the
/// bearing it faces when it was not.
fn grow(
    root: Vec3,
    side: f32,
    lowered: f32,
    turned_to_path: f32,
    live: Live,
    rng: &mut Rng,
) -> Stem {
    let radius = rng.range(RADIUS.0, RADIUS.1) as f32;
    let run = lowered * rng.range(1.0 - LOWERED_JITTER, 1.0 + LOWERED_JITTER) as f32;
    let round = Round {
        from: live.ends.0,
        to: live.ends.1,
        out: gutter::WIDTH / 2.0 + RUN_OUT + radius + RUN_SPREAD.x * rng.unit() as f32,
    };
    let run_z = live.rim + radius + RUN_SPREAD.y * rng.unit() as f32;
    let hook_x = rng.range(-HOOK_JITTER, HOOK_JITTER) as f32;
    let bow = Vec2::from_angle(rng.unit() as f32 * TAU).extend(0.0) * BOW;
    let ripening = rng.range(-RIPENING_JITTER, RIPENING_JITTER) as f32;

    // Out of the block and over onto the run, along it, and up off its end
    // round a quarter circle into the rise.
    let mut points = vec![root - Vec3::Z * ROOT_DEPTH, root + Vec3::Z * ROOT_RISE];
    let start = round.start(root.x, side);
    let end = round.beside(start + run);
    let (bend, rise_side) = if end > start + RUN_START {
        let first = round.at(start + RUN_START).0.extend(run_z);
        points.push(points[1].lerp(first, 0.5) + Vec3::Z * BEND / 4.0);
        let mut u = start + RUN_START;
        while u < end - RUN_STEP / 2.0 {
            points.push(round.at(u).0.extend(run_z));
            u += RUN_STEP;
        }
        let (at, along) = round.at(end);
        let arc = |i: usize| {
            let a = FRAC_PI_2 * i as f32 / (BEND_POINTS - 1) as f32;
            at.extend(run_z) + (along * a.sin()).extend(1.0 - a.cos()) * BEND
        };
        points.extend((0..BEND_POINTS - 1).map(arc));
        (arc(BEND_POINTS - 1), at.y.signum())
    } else {
        (root + Vec3::Z * (ROOT_RISE + BEND), side)
    };

    let head = live.head.max(bend.z + 2.0 * MIN_LIVE);
    let harvest = live.harvest.clamp(bend.z, head - MIN_LIVE);
    let top = Vec3::new(bend.x + hook_x, rise_side * live.spread, head);
    // The rise, straight from the bend to the head but for a bow pinned at
    // both ends; the bow is level, so a node is exactly as high as it was
    // spaced.
    let rise = |z: f32| {
        let f = (z - bend.z) / (head - bend.z);
        bend.lerp(top, f) + bow * (PI * f).sin()
    };
    let thin = |z: f32| {
        let t = ((z - head + HEAD_REACH) / HEAD_REACH).clamp(0.0, 1.0);
        radius * (1.0 - HEAD_TAPER * t * t * (3.0 - 2.0 * t))
    };

    // A control point at every node, the bare ones below the harvest included.
    let node = (head - harvest) / (live.trusses * NODES_PER_TRUSS) as f32;
    let below = ((harvest - bend.z) / node).floor() as i32;
    let mut radii = vec![radius; points.len() + 1];
    points.push(bend);
    for n in (1 - below)..=(live.trusses * NODES_PER_TRUSS) as i32 {
        let z = harvest + n as f32 * node;
        points.push(rise(z));
        radii.push(thin(z));
    }

    let trusses = (0..live.trusses)
        .map(|k| {
            let z = harvest + (k * NODES_PER_TRUSS) as f32 * node;
            let turned = rng.unit() < turned_to_path as f64;
            let wander = rng.range(-FACING_SPREAD, FACING_SPREAD) as f32;
            let anywhere = rng.unit() as f32 * TAU;
            let yaw = if turned {
                rise_side * FRAC_PI_2 + wander
            } else {
                anywhere
            };
            let node = Node {
                stem_radius: thin(z),
                rank: k as f32 - ripening,
                growth: ((live.trusses - k) as f32 / GROWING_TRUSSES).min(1.0),
            };
            (rise(z), yaw, node)
        })
        .collect();

    // Wound down the rise from the reel, just clear of the stem.
    let axis = (top - bend).normalize();
    let across = axis.cross(Vec3::X).normalize();
    let over = axis.cross(across);
    let mut twine = vec![top + Vec3::Z * 0.01];
    let mut z = head;
    while z > bend.z {
        let turn = TAU * z / TWINE_PITCH;
        let out = (thin(z) + TWINE_RADIUS) * (across * turn.cos() + over * turn.sin());
        twine.push(rise(z) + out);
        z -= TWINE_STEP;
    }

    Stem {
        points,
        radii,
        twine,
        hook: top + Vec3::Z * HOOK_LENGTH,
        trusses,
    }
}

/// A strand through `points`, `offset` into its plant's frame.
fn strand(points: &[Vec3], radii: &[f32], offset: Vec3, sides: usize) -> Strand {
    let points = points
        .iter()
        .map(|p| {
            let p = *p - offset;
            Point3::new(p.x as f64, p.y as f64, p.z as f64)
        })
        .collect();
    let radii = radii.iter().map(|r| *r as f64).collect();
    Strand::new(points, radii, sides, DETAIL, Bark::none())
}

/// A hook hung from its origin on the wire: the wire it is bent from, and the
/// twine reel at its foot.
fn hook_mesh() -> MeshData {
    let hung = Transform::from_xyz(0.0, 0.0, -HOOK_LENGTH);
    let reel = Transform::from_xyz(0.0, REEL.y / 2.0, -HOOK_LENGTH)
        .with_rotation(Quat::from_rotation_x(FRAC_PI_2));
    merge_meshes(&[
        cylinder_mesh(HOOK_RADIUS, HOOK_RADIUS, HOOK_LENGTH, 6).transformed(&hung),
        cylinder_mesh(REEL.x, REEL.x, REEL.y, 10).transformed(&reel),
    ])
}

/// One stem to build: its row, its plant, its index on the plant, and what it
/// was solved to.
type Planted = (Entity, usize, usize, Stem);

#[allow(clippy::too_many_arguments)]
fn author(
    mut commands: Commands,
    mut library: Library,
    params: Res<StemParams>,
    truss_params: Res<TrussParams>,
    gutter: Res<GutterParams>,
    wire: Res<WireParams>,
    scene: Res<SceneParams>,
    layout: Res<GreenhouseLayout>,
    rows: Query<(Entity, &Row)>,
) -> Result<()> {
    commands.despawn_all::<With<Owned>>();
    for prefix in [TUBE, TWINE, HOOK] {
        library.clear(prefix);
    }
    let hook = library.part(
        HOOK,
        0,
        hook_mesh().to_mesh(),
        PLASTIC.surface(srgb(HOOK_COLOR)),
    );

    let root_z = gutter.block_top();
    let live = Live {
        rim: gutter.floor() + gutter::DEPTH,
        head: wire.height - HOOK_LENGTH,
        harvest: params.harvest_height,
        trusses: params.trusses.max(1),
        spread: wire.spread,
        ends: (RUN_TURN, (layout.length - RUN_TURN).max(RUN_TURN)),
    };
    let lowered = params.lowered.max(0.0);

    // In row order, so the trusses' `Order` follows from the layout alone.
    let mut rows: Vec<(usize, Entity)> = rows.iter().map(|(e, row)| (row.0, e)).collect();
    rows.sort();
    let mut stems: Vec<Planted> = Vec::new();
    for &(row, entity) in &rows {
        for (plant, &x) in layout.plants.iter().enumerate() {
            let at = (row * layout.plants.len() + plant) as u64;
            let extra = Rng::new(scene.seed ^ PLANT_STREAM ^ salt(at)).unit();
            let count = if extra < params.extra_stems as f64 {
                3
            } else {
                2
            };
            for index in 0..count {
                // Alternately left and right, and an extra stem to the side
                // its plant's place along the row picks.
                let (side, more) = match index {
                    0 => (1.0, 0.0),
                    1 => (-1.0, 0.0),
                    _ => (if plant % 2 == 0 { 1.0 } else { -1.0 }, EXTRA_RUN),
                };
                let mut rng =
                    Rng::new(scene.seed ^ STEM_STREAM ^ salt(at * MAX_STEMS as u64 + index as u64));
                let stem = grow(
                    Vec3::new(x, 0.0, root_z),
                    side,
                    lowered + more,
                    params.turned_to_path,
                    live,
                    &mut rng,
                );
                stems.push((entity, plant, index, stem));
            }
        }
    }

    // Each stem's tube and twine, grown in parallel and registered serially:
    // `Assets<Mesh>` takes one writer.
    let meshes = par_map(&stems, |_, (_, plant, _, stem)| -> Result<(Mesh, Mesh)> {
        let origin = Vec3::new(layout.plants[*plant], 0.0, root_z);
        let twine = vec![TWINE_RADIUS; stem.twine.len()];
        Ok((
            strand_mesh(&strand(&stem.points, &stem.radii, origin, SIDES))?.to_mesh(),
            strand_mesh(&strand(&stem.twine, &twine, origin, TWINE_SIDES))?.to_mesh(),
        ))
    });

    let (green, white) = (
        STEM_RESPONSE.surface(srgb(STEM_COLOR)),
        PLASTIC.surface(srgb(TWINE_COLOR)),
    );
    let mut plant_prim = Entity::PLACEHOLDER;
    let mut order = 0u64;
    for (number, ((row, plant, index, stem), meshes)) in stems.iter().zip(meshes).enumerate() {
        let (tube, twine) = meshes?;
        let origin = Vec3::new(layout.plants[*plant], 0.0, root_z);
        // A plant's stems come one after another, the first opening its prim.
        if *index == 0 {
            plant_prim = commands
                .spawn((
                    Owned,
                    Name::new(format!("Plant_{plant:03}")),
                    Transform::from_translation(origin),
                    Visibility::default(),
                    ChildOf(*row),
                ))
                .id();
        }
        let tube = library.part(TUBE, number, tube, green);
        let twine = library.part(TWINE, number, twine, white);
        commands
            .spawn((
                Name::new(format!("Stem_{index}")),
                Transform::IDENTITY,
                Visibility::default(),
                ChildOf(plant_prim),
            ))
            .with_children(|parent| {
                parent.spawn((Name::new("Tube"), tube));
                parent.spawn((
                    Name::new("Hook"),
                    hook.clone(),
                    Transform::from_translation(stem.hook - origin),
                ));
                parent.spawn((Name::new("Twine"), twine));
                for (k, (at, yaw, node)) in stem.trusses.iter().enumerate() {
                    order += 1;
                    parent.spawn((
                        Name::new(format!("Truss_{k:02}")),
                        Transform::from_translation(*at - origin)
                            .with_rotation(Quat::from_rotation_z(*yaw)),
                        Visibility::default(),
                        TrussConfig::new(&truss_params, scene.seed, order, *node),
                        Order(order),
                    ));
                }
            });
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeMap;

    use super::*;
    use crate::elements::{GreenhouseParams, grown, in_scene, small};
    use misina_lab::testing::{named_children, prim_path};

    /// On every stem the lowest truss hangs at the harvest height and the
    /// rest one stem's growth apart above it, 25 to 35 cm at the defaults;
    /// and every truss carries its peduncle and the two frames a gripper
    /// takes it by. At the defaults, and on a low gutter under a high wire.
    #[test]
    fn the_lowest_truss_hangs_at_the_harvest_height_with_its_frames() {
        let defaults: fn(&mut GreenhouseParams) = |_| {};
        let tall: fn(&mut GreenhouseParams) = |p| {
            (p.gutter.height, p.wire.height) = (0.3, 5.5);
            (p.stem.harvest_height, p.stem.trusses) = (0.8, 12);
        };
        for (edit, growth) in [(defaults, 0.25..=0.35), (tall, 0.0..=1.0)] {
            let mut params = small();
            edit(&mut params);
            let stem = params.stem.clone();
            let mut app = grown(params);
            let world = app.world_mut();
            let trusses: Vec<Entity> = world
                .query_filtered::<Entity, With<TrussConfig>>()
                .iter(world)
                .collect();
            let mut stems: BTreeMap<Entity, Vec<(String, Vec3)>> = BTreeMap::new();
            for truss in trusses {
                let path = prim_path(world, truss);
                let held: Vec<String> = named_children(world, truss)
                    .into_iter()
                    .map(|(name, _)| name)
                    .collect();
                for frame in ["Peduncle", "Cut", "Bend"] {
                    assert!(
                        held.iter().any(|name| name == frame),
                        "{path} has no {frame}"
                    );
                }
                let on = world.entity(truss).get::<ChildOf>().unwrap().parent();
                let node = in_scene(world, truss).translation;
                stems.entry(on).or_default().push((path, node));
            }
            assert!(!stems.is_empty());
            for (_, mut trusses) in stems {
                trusses.sort_by(|a, b| a.0.cmp(&b.0));
                let (path, lowest) = &trusses[0];
                assert_eq!(trusses.len(), stem.trusses as usize, "{path}");
                assert!(
                    (lowest.z - stem.harvest_height).abs() < 1e-4,
                    "{path} hangs at {}",
                    lowest.z
                );
                let per =
                    lowest.distance(trusses[trusses.len() - 1].1) / (trusses.len() - 1) as f32;
                assert!(growth.contains(&per), "{path}: {per} m of stem a truss");
            }
        }
    }

    /// A place round the gutter maps onto the left side running +X, the far
    /// turn, the right side running -X and the head turn, and every place a
    /// run may end at is beside the gutter.
    #[test]
    fn the_runs_go_round_the_gutter() {
        let round = Round {
            from: 0.2,
            to: 9.8,
            out: 0.13,
        };
        let (left, along) = round.at(round.start(3.0, 1.0));
        assert!(left.distance(Vec2::new(3.0, 0.13)) < 1e-5 && along == Vec2::X);
        let (right, along) = round.at(round.start(3.0, -1.0));
        assert!(right.distance(Vec2::new(3.0, -0.13)) < 1e-4 && along == -Vec2::X);
        for u in (0..400).map(|i| i as f32 * 0.07) {
            let (at, _) = round.at(round.beside(u));
            assert!(
                (at.y.abs() - round.out).abs() < 1e-4 && (round.from..=round.to).contains(&at.x),
                "{u}: {at}"
            );
        }
        // Round the far end from the left lands on the right.
        let (back, _) = round.at(round.beside(round.start(9.0, 1.0) + 2.0));
        assert!(
            back.y < 0.0 && (back.x - (9.8 - 1.2 + round.turn())).abs() < 1e-3,
            "{back}"
        );
    }
}

//! Rail — the pipe rail in every path: two heating pipes a trolley rides on.
//!
//! A path's two pipes are one heating loop. At the row head each turns down
//! through an elbow into the floor, to the mains under the main path that
//! feed one pipe and take the water back from the other, and at the far end
//! a return bend joins the pair: two more elbows and a pipe across.
//!
//! The pipes rest on a support every meter and run straight from one support
//! to the next. `unevenness` raises or lowers every support by a draw of its
//! own, which tilts the pair across and pitches each pipe along: at 1 the two
//! pipes at a support differ in height by up to `gauge × tan 2°`, the most
//! the AgroArbo pipe rail spec allows (15 to 21 mm at the common gauges).
//!
//! A segment is a meter-long cylinder stretched to its length, beside a
//! capsule collider whose straight part spans the whole segment, so a wheel
//! rolls from one segment onto the next without dipping at the joint. A bend
//! collides as a chain of capsules round each turn and one capsule along
//! each straight run.

use std::f32::consts::{FRAC_PI_2, PI};

use bevy::prelude::*;
use misina_lab::geometry::mesh::{MeshData, cylinder_mesh};
use misina_lab::geometry::strand::{Bark, Strand, strand_mesh};
use misina_lab::palette::srgb;
use misina_lab::params::Slider;
use misina_lab::rng::{Rng, salt};
use misina_lab::scene::{COLLISION, Library, capsule, capsule_between};
use nalgebra::Point3;

use super::layout::{GreenhouseLayout, Path};
use super::{Grow, STEEL, SceneParams, cuboid, vector};

/// Salt splitting the rails' draws off the scene seed.
const STREAM: u64 = 0x5851_F42D_4C95_7F2D;

/// Distance between the supports along a pipe, in meters.
const SUPPORT_SPACING: f32 = 1.0;

/// The steepest tilt across the pair the rail spec allows, in degrees.
const SPEC_TILT: f32 = 2.0;

/// A support's footprint under one pipe, in meters: along the rail, and
/// across it.
const SUPPORT: Vec2 = Vec2::new(0.04, 0.08);

/// Vertices around a pipe.
const SIDES: usize = 12;

/// A head elbow's bend radius, in pipe diameters: a long-radius elbow's.
const ELBOW: f32 = 1.5;

/// The most a bend turns from one of its collider's capsules to the next, in
/// radians.
const BEND_STEP: f32 = PI / 12.0;

/// How far a bend's mesh may stray from its surface, in meters: its rings
/// bunch round the turns and spread out along the straight runs.
const TOLERANCE: f64 = 0.0003;

/// Thinnest pipe built, so a diameter of zero from Python still makes a mesh.
const MIN_DIAMETER: f32 = 0.005;

/// The prim a pipe segment's geometry takes, beside its collider.
const STEEL_PRIM: &str = "Steel";

/// Light galvanized pipe.
const PIPE_COLOR: u32 = 0xC4C7C5;

/// The darker steel of a support.
const SUPPORT_COLOR: u32 = 0x5E6163;

/// The pipe rail in every path: two heating pipes a trolley rides on.
///
/// The pipes rest on a support every meter, and `unevenness` moves the
/// supports off level, each by a draw of its own: the pair tilts across and
/// each pipe pitches along its length. Each pipe turns down into the floor at
/// the row head, and a return bend joins the two at the far end.
#[derive(Resource, Reflect, Clone, Debug, PartialEq)]
#[cfg_attr(
    feature = "python",
    pyo3::pyclass(get_all, set_all, skip_from_py_object)
)]
pub struct RailParams {
    /// Distance between the two pipes, centre to centre, in meters. Growers
    /// lay 425, 500, 550, 600 and 800 mm.
    #[reflect(@Slider { min: 0.425, max: 0.8, step: 0.025 })]
    pub gauge: f32,
    /// Outer diameter of a pipe, in meters: the slider stops at the 45 and
    /// 51 mm heating pipes.
    #[reflect(@Slider { min: 0.045, max: 0.051, step: 0.006 })]
    pub pipe_diameter: f32,
    /// Height of the pipes' axis above the floor, in meters. An estimate, to
    /// be set from a measured site.
    #[reflect(@Slider { min: 0.05, max: 0.3, step: 0.01 })]
    pub height: f32,
    /// How far the supports stand off level. 0 is a perfect rail; at 1 the
    /// pair tilts across by up to the 2° the rail spec allows; 2 is twice out
    /// of spec.
    #[reflect(@Slider { min: 0.0, max: 2.0, step: 0.1 })]
    pub unevenness: f32,
}

impl Default for RailParams {
    fn default() -> Self {
        Self {
            gauge: 0.6,
            pipe_diameter: 0.051,
            height: 0.15,
            unevenness: 0.5,
        }
    }
}

/// Marks a path's `Rail` prim, which every rebuild replaces.
#[derive(Component)]
struct Owned;

pub fn plugin(app: &mut App) {
    app.init_resource::<RailParams>().add_systems(
        PreUpdate,
        build
            .run_if(
                resource_changed::<RailParams>
                    .or_eager(resource_changed::<SceneParams>)
                    .or_eager(resource_changed::<GreenhouseLayout>),
            )
            .in_set(Grow::Fixtures),
    );
}

fn build(
    mut commands: Commands,
    mut library: Library,
    params: Res<RailParams>,
    scene: Res<SceneParams>,
    layout: Res<GreenhouseLayout>,
    paths: Query<(Entity, &Path)>,
) {
    commands.despawn_all::<With<Owned>>();
    for prefix in ["Pipe", "Elbow", "ReturnBend", "Support"] {
        library.clear(prefix);
    }

    let radius = params.pipe_diameter.max(MIN_DIAMETER) / 2.0;
    let pipe = library.part(
        "Pipe",
        0,
        cylinder_mesh(radius, radius, 1.0, SIDES).to_mesh(),
        STEEL.surface(srgb(PIPE_COLOR)),
    );
    let reach =
        (params.unevenness.max(0.0) * params.gauge * SPEC_TILT.to_radians().tan()) as f64 / 2.0;

    // A quarter turn from a pipe's end at the row head, out over the main
    // path and down, then straight on down past the floor under the highest
    // support: a long-radius elbow, tightened on a rail too low for one to
    // clear the floor, but never tighter than the pipe is thick.
    let turn = (ELBOW * 2.0 * radius).min(params.height).max(radius);
    let step = turn * BEND_STEP;
    let mut elbow_line = arc(FRAC_PI_2, |a| {
        turn * Vec3::new(-a.sin(), 0.0, a.cos() - 1.0)
    });
    let low = elbow_line[elbow_line.len() - 1];
    let bottom = -(params.height + reach as f32 + radius);
    elbow_line.extend(straight(low, low.with_z(bottom.min(low.z)), step));
    let elbow = library.part(
        "Elbow",
        0,
        tube(&elbow_line, radius).to_mesh(),
        STEEL.surface(srgb(PIPE_COLOR)),
    );
    // At the far end the same elbow turns the left pipe across the pair, a
    // pipe runs across, and its mirror image turns it back into the right
    // one; on a pair too narrow for two, the elbows tighten into a half
    // circle.
    let half = (params.gauge / 2.0).max(MIN_DIAMETER);
    let corner = turn.min(half);
    let across = arc(FRAC_PI_2, |a| {
        Vec3::new(corner * a.sin(), half - corner * (1.0 - a.cos()), 0.0)
    });
    let end = across[across.len() - 1];
    let bend_line: Vec<Vec3> = (across.iter().copied())
        .chain(straight(end, end.with_y(-end.y), step))
        .chain(across.iter().rev().skip(1).map(|p| p.with_y(-p.y)))
        .collect();
    let bend = library.part(
        "ReturnBend",
        0,
        tube(&bend_line, radius).to_mesh(),
        STEEL.surface(srgb(PIPE_COLOR)),
    );
    // A meter tall, standing on its origin.
    let footprint = SUPPORT.extend(0.0) / 2.0;
    let support = library.part(
        "Support",
        0,
        cuboid(-footprint, footprint.with_z(1.0)).to_mesh(),
        STEEL.surface(srgb(SUPPORT_COLOR)),
    );

    let segments = (layout.length / SUPPORT_SPACING).ceil().max(1.0) as usize;
    let spacing = layout.length / segments as f32;

    for (path, &Path(index)) in &paths {
        let rail = commands
            .spawn((
                Owned,
                Name::new("Rail"),
                Transform::IDENTITY,
                Visibility::default(),
                ChildOf(path),
            ))
            .id();
        // Salted by the path, so another row count leaves each path's draws
        // alone; drawn support by support, the left pipe first.
        let mut rng = Rng::new(scene.seed ^ STREAM ^ salt(index as u64));
        let heights: Vec<[f32; 2]> = (0..=segments)
            .map(|_| [0, 1].map(|_| params.height + rng.range(-reach, reach) as f32))
            .collect();

        // Left and right looking along the row from its head.
        let ys = [params.gauge / 2.0, -params.gauge / 2.0];
        let on = |side: usize, i: usize| Vec3::new(i as f32 * spacing, ys[side], heights[i][side]);
        for (side, name) in ["L", "R"].into_iter().enumerate() {
            let at = |i: usize| on(side, i);
            for i in 0..=segments {
                let top = at(i).z - radius;
                if top > 0.0 {
                    commands.spawn((
                        Name::new(format!("Support_{name}_{i:03}")),
                        support.clone(),
                        Transform::from_translation(at(i).with_z(0.0))
                            .with_scale(Vec3::new(1.0, 1.0, top)),
                        ChildOf(rail),
                    ));
                }
            }
            for i in 0..segments {
                let (from, to) = (at(i), at(i + 1));
                let length = from.distance(to);
                commands
                    .spawn((
                        Name::new(format!("Pipe_{name}_{i:03}")),
                        Transform::from_translation(from)
                            .with_rotation(Quat::from_rotation_arc(Vec3::Z, (to - from) / length)),
                        Visibility::default(),
                        ChildOf(rail),
                    ))
                    .with_child((
                        Name::new(STEEL_PRIM),
                        pipe.clone(),
                        Transform::from_scale(Vec3::new(1.0, 1.0, length)),
                    ))
                    .with_child((
                        Name::new(COLLISION),
                        capsule(radius, -radius, length + radius),
                    ));
            }

            commands
                .spawn((
                    Name::new(format!("Elbow_{name}")),
                    Transform::from_translation(at(0)),
                    Visibility::default(),
                    ChildOf(rail),
                ))
                .with_children(|parent| {
                    parent.spawn((Name::new(STEEL_PRIM), elbow.clone()));
                    spawn_capsules(parent, &elbow_line, radius);
                });
        }

        // The return bend, tilted across as the pair's last supports are.
        let (left, right) = (on(0, segments), on(1, segments));
        let tilt = ((left.z - right.z) / (2.0 * half)).clamp(-1.0, 1.0).asin();
        commands
            .spawn((
                Name::new("ReturnBend"),
                Transform::from_translation((left + right) / 2.0)
                    .with_rotation(Quat::from_rotation_x(tilt)),
                Visibility::default(),
                ChildOf(rail),
            ))
            .with_children(|parent| {
                parent.spawn((Name::new(STEEL_PRIM), bend.clone()));
                spawn_capsules(parent, &bend_line, radius);
            });
    }
}

/// Points along a bend turning `angle` radians, no more than [`BEND_STEP`]
/// apart: `at(a)` is the point it has turned `a` at.
fn arc(angle: f32, at: impl Fn(f32) -> Vec3) -> Vec<Vec3> {
    let pieces = (angle / BEND_STEP).ceil() as usize;
    (0..=pieces)
        .map(|k| at(angle * k as f32 / pieces as f32))
        .collect()
}

/// Points on from `from` to `to`, `from` left out, no further than `step`
/// apart, for the strand through them to keep straight.
fn straight(from: Vec3, to: Vec3, step: f32) -> impl Iterator<Item = Vec3> {
    let pieces = (from.distance(to) / step).ceil() as usize;
    (1..=pieces).map(move |k| from.lerp(to, k as f32 / pieces as f32))
}

/// A pipe of `radius` along `line`.
fn tube(line: &[Vec3], radius: f32) -> MeshData {
    let points = line.iter().map(|&p| Point3::from(vector(p))).collect();
    let strand = Strand {
        tolerance: Some(TOLERANCE),
        ..Strand::new(
            points,
            vec![radius as f64; line.len()],
            SIDES,
            0.0,
            Bark::none(),
        )
    };
    strand_mesh(&strand).expect("a bend's line has distinct points")
}

/// A capsule along each piece of `line`, a straight run of pieces as one,
/// its straight part spanning the piece, so that neighbours meet in the round
/// at every point.
fn spawn_capsules(parent: &mut ChildSpawnerCommands, line: &[Vec3], radius: f32) {
    let turns = line.windows(3).filter_map(|w| {
        let (a, b) = ((w[1] - w[0]).normalize(), (w[2] - w[1]).normalize());
        (a.dot(b) < 1.0 - 1e-4).then_some(w[1])
    });
    let corners: Vec<Vec3> = (line.first().copied())
        .into_iter()
        .chain(turns)
        .chain(line.last().copied())
        .collect();
    for (k, piece) in corners.windows(2).enumerate() {
        let out = (piece[1] - piece[0]).normalize_or_zero() * radius;
        parent.spawn((
            Name::new(format!("{COLLISION}_{k:02}")),
            capsule_between(radius, piece[0] - out, piece[1] + out),
        ));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::elements::{GreenhouseParams, grown};
    use misina_lab::scene::Collider;
    use misina_lab::testing::{named_children, prim};

    /// The built scene, and the prims of path 0's rail by name.
    fn rail(params: GreenhouseParams) -> (App, Vec<(String, Entity)>) {
        let mut app = grown(params);
        let rail = prim(app.world_mut(), &["Paths", "Path_00", "Rail"]).unwrap();
        let prims = named_children(app.world_mut(), rail);
        (app, prims)
    }

    /// A level rail runs the whole row at its height, segment after segment,
    /// and each segment's collider is the pipe with a straight part as long
    /// as the segment, so a wheel meets no dip at a joint.
    #[test]
    fn a_level_rail_runs_the_row_on_smooth_colliders() {
        let mut params = GreenhouseParams::default();
        params.rail.unevenness = 0.0;
        let (spec, length) = (params.rail.clone(), params.layout.row_length);
        let (mut app, prims) = rail(params);
        let world = app.world_mut();

        let pipes: Vec<Entity> = prims
            .iter()
            .filter(|(name, _)| name.starts_with("Pipe_L_"))
            .map(|(_, pipe)| *pipe)
            .collect();
        let step = length / pipes.len() as f32;
        assert!(step <= SUPPORT_SPACING, "{} segments", pipes.len());
        for (i, &pipe) in pipes.iter().enumerate() {
            let placed = *world.entity(pipe).get::<Transform>().unwrap();
            let start = Vec3::new(i as f32 * step, spec.gauge / 2.0, spec.height);
            assert!(placed.translation.distance(start) < 1e-4, "segment {i}");
            assert!(
                (placed.rotation * Vec3::Z).distance(Vec3::X) < 1e-5,
                "segment {i}"
            );

            let [steel, collision] = [0, 1].map(|k| named_children(world, pipe)[k].1);
            let drawn = world.entity(steel).get::<Transform>().unwrap().scale.z;
            let shape = world.entity(collision).get::<Collider>().unwrap().0;
            assert!((drawn - step).abs() < 1e-4, "segment {i} is {drawn} long");
            assert_eq!(shape.radius, spec.pipe_diameter / 2.0);
            assert!((shape.height - step).abs() < 1e-4, "segment {i}");
        }
    }

    /// Where a pipe segment starts and ends, in the rail's frame.
    fn span(world: &mut World, pipe: Entity) -> (Vec3, Vec3) {
        let placed = *world.entity(pipe).get::<Transform>().unwrap();
        let steel = named_children(world, pipe)[0].1;
        let length = world.entity(steel).get::<Transform>().unwrap().scale.z;
        let along = placed.rotation * Vec3::Z * length;
        (placed.translation, placed.translation + along)
    }

    /// The return bend meets both pipes' last segments, tilted across with
    /// them twice out of spec, and each elbow starts on its pipe's first
    /// segment and runs down into the floor.
    #[test]
    fn the_bends_meet_the_pipe_ends_and_the_elbows_the_floor() {
        let mut params = GreenhouseParams::default();
        params.rail.unevenness = 2.0;
        let spec = params.rail.clone();
        let (mut app, prims) = rail(params);
        let world = app.world_mut();
        let entity = |name: &str| prims.iter().find(|(n, _)| n == name).unwrap().1;
        let last = prims
            .iter()
            .filter(|(n, _)| n.starts_with("Pipe_L_"))
            .count()
            - 1;
        let bend = *world
            .entity(entity("ReturnBend"))
            .get::<Transform>()
            .unwrap();

        for (side, y) in [("L", spec.gauge / 2.0), ("R", -spec.gauge / 2.0)] {
            let (_, end) = span(world, entity(&format!("Pipe_{side}_{last:03}")));
            let off = bend.transform_point(Vec3::Y * y).distance(end);
            assert!(off < 1e-3, "{side}: the bend ends {off} m off its pipe");

            let elbow = entity(&format!("Elbow_{side}"));
            let head = world.entity(elbow).get::<Transform>().unwrap().translation;
            let (start, _) = span(world, entity(&format!("Pipe_{side}_000")));
            assert!(head.distance(start) < 1e-6, "{side}: {head} off {start}");
            // Its last capsule, the one standing down from the turn.
            let last = named_children(world, elbow).last().unwrap().1;
            let placed = world.entity(last).get::<Transform>().unwrap();
            let shape = world.entity(last).get::<Collider>().unwrap().0;
            let bottom = head.z + placed.translation.z - shape.height / 2.0 - shape.radius;
            assert!(
                bottom < 0.0,
                "{side}: the elbow stops {bottom} m off the floor"
            );
        }
    }

    /// At 1 the two pipes at a support never differ by more than the spec
    /// allows, and across a long row some support comes well into that range.
    /// The difference of two uniform draws peaks at zero, so the row's largest
    /// lands anywhere from about 0.7 of the limit up.
    #[test]
    fn unevenness_one_stays_inside_the_spec_and_reaches_it() {
        let mut params = GreenhouseParams::default();
        params.rail.unevenness = 1.0;
        (params.layout.rows, params.layout.row_length) = (1, 120.0);
        let limit = params.rail.gauge * SPEC_TILT.to_radians().tan();
        let (app, prims) = rail(params);

        let top = |side: &str, i: usize| {
            let name = format!("Support_{side}_{i:03}");
            let (_, support) = prims.iter().find(|(n, _)| *n == name).unwrap();
            app.world()
                .entity(*support)
                .get::<Transform>()
                .unwrap()
                .scale
                .z
        };
        let most = (0..=120)
            .map(|i| (top("L", i) - top("R", i)).abs())
            .fold(0.0, f32::max);
        assert!(most <= limit + 1e-6, "{most} m, over the {limit} m allowed");
        assert!(
            most > 0.6 * limit,
            "{most} m: never near the {limit} m allowed"
        );
    }
}

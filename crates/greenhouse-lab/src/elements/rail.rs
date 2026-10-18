//! Rail — the pipe rail in every path: two heating pipes a trolley rides on.
//!
//! The pipes rest on a support every meter and run straight from one support
//! to the next. `unevenness` raises or lowers every support by a draw of its
//! own, which tilts the pair across and pitches each pipe along: at 1 the two
//! pipes at a support differ in height by up to `gauge × tan 2°`, the most
//! the AgroArbo pipe rail spec allows (15 to 21 mm at the common gauges).
//!
//! A segment is a meter-long cylinder stretched to its length, beside a
//! capsule collider whose straight part spans the whole segment, so a wheel
//! rolls from one segment onto the next without dipping at the joint.

use bevy::prelude::*;
use misina_lab::geometry::mesh::cylinder_mesh;
use misina_lab::palette::srgb;
use misina_lab::params::Slider;
use misina_lab::rng::{Rng, salt};
use misina_lab::scene::{COLLISION, Library, capsule};

use super::layout::{GreenhouseLayout, Path};
use super::{Grow, STEEL, SceneParams, cuboid};

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
/// each pipe pitches along its length.
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
    library.clear("Pipe");
    library.clear("Support");

    let radius = params.pipe_diameter.max(MIN_DIAMETER) / 2.0;
    let pipe = library.part(
        "Pipe",
        0,
        cylinder_mesh(radius, radius, 1.0, SIDES).to_mesh(),
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
    let step = layout.length / segments as f32;
    let reach =
        (params.unevenness.max(0.0) * params.gauge * SPEC_TILT.to_radians().tan()) as f64 / 2.0;

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
        for (side, (name, y)) in [("L", params.gauge / 2.0), ("R", -params.gauge / 2.0)]
            .into_iter()
            .enumerate()
        {
            let at = |i: usize| Vec3::new(i as f32 * step, y, heights[i][side]);
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
        }
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

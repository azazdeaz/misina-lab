//! The viewer's collider overlay: everything a physics engine collides with,
//! drawn as lines over the scene while the footer's **Colliders** box is
//! ticked.
//!
//! One colour per kind of collider:
//!
//! - magenta, [`STATIC`]: the [`Collider`] capsules, and the first segment of
//!   every rod, which is held where it was authored. Nothing moves these.
//! - yellow, [`ROD`]: every other segment of a [`Cable`], one capsule each, as
//!   a solver's rod importer builds them.
//! - cyan, [`SURFACE`]: the parts that are their own collider
//!   ([`Part::collision`](crate::scene::Part::collision)), the ground among
//!   them, drawn as their triangle edges.
//!
//! A capsule is drawn solid where it is in sight and dashed where something
//! hides it, since most of one lies inside the mesh it stands in for. A
//! surface is drawn only where it is in sight, or the ground would show
//! through everything.
//!
//! The lines are retained [`Gizmo`]s, rebuilt with the scene rather than every
//! frame: a parcel holds thousands of capsules.

use std::collections::HashMap;
use std::f32::consts::PI;

use bevy::color::palettes::css;
use bevy::math::Affine3A;
use bevy::mesh::VertexAttributeValues;
use bevy::prelude::*;

use crate::scene::{Cable, Collider, Prototypes, UsdReference};

pub fn plugin(app: &mut App) {
    app.init_resource::<ShowColliders>().add_systems(
        PostUpdate,
        // After propagation, so a layer spawned this frame is drawn where it
        // stands. Gated on the library, which every layer clears before
        // rebuilding, so the lines are rebuilt when the scene is.
        draw.after(TransformSystems::Propagate)
            .run_if(resource_changed::<ShowColliders>.or_eager(resource_changed::<Prototypes>)),
    );
}

/// Whether the overlay is drawn. Written by the footer's **Colliders** box.
#[derive(Resource, Default)]
pub struct ShowColliders(pub bool);

/// Capsules nothing moves.
pub const STATIC: Srgba = css::FUCHSIA;
/// Rod segments a solver moves.
pub const ROD: Srgba = css::YELLOW;
/// Meshes that are their own collider. Half see-through, since from afar a
/// ground's grid is denser than the pixels it covers.
pub const SURFACE: Srgba = Srgba::new(0.0, 1.0, 1.0, 0.5);

/// Segments per half turn of a capsule's outline.
const ARC: usize = 6;

/// Marks the overlay's gizmos, which every rebuild replaces.
#[derive(Component)]
struct Overlay;

#[allow(clippy::too_many_arguments)]
fn draw(
    mut commands: Commands,
    show: Res<ShowColliders>,
    mut gizmos: ResMut<Assets<GizmoAsset>>,
    prototypes: Res<Prototypes>,
    meshes: Res<Assets<Mesh>>,
    capsules: Query<(&Collider, &GlobalTransform)>,
    cables: Query<(&Cable, &ChildOf)>,
    frames: Query<&GlobalTransform>,
    instances: Query<(&UsdReference, &GlobalTransform)>,
) {
    commands.despawn_all::<With<Overlay>>();
    if !show.0 {
        return;
    }

    let mut shapes = GizmoAsset::new();
    for (collider, placed) in &capsules {
        let collider = collider.0;
        capsule_outline(
            &mut shapes,
            placed.affine(),
            collider.height / 2.0,
            collider.radius,
            STATIC,
        );
    }
    for (cable, organ) in &cables {
        // A cable has no transform of its own: its points are in its organ's
        // frame.
        let Ok(organ) = frames.get(organ.parent()) else {
            continue;
        };
        let cable = &cable.0;
        let segments = cable.points.windows(2).zip(cable.widths.windows(2));
        for (index, (pair, widths)) in segments.enumerate() {
            let [from, to] = [pair[0], pair[1]].map(Vec3::from);
            let along = (to - from).try_normalize().unwrap_or(Vec3::Z);
            let segment = Affine3A::from_rotation_translation(
                Quat::from_rotation_arc(Vec3::Z, along),
                (from + to) / 2.0,
            );
            // The cylinder spans the whole segment and the caps reach past
            // both of its points, unlike a proxy's, which end on them. Its
            // diameter is the mean of the two ends', as the importer sizes it.
            capsule_outline(
                &mut shapes,
                organ.affine() * segment,
                from.distance(to) / 2.0,
                (widths[0] + widths[1]) / 4.0,
                if index == 0 { STATIC } else { ROD },
            );
        }
    }

    let solid: HashMap<&str, &Mesh> = prototypes
        .iter()
        .filter(|(_, part)| part.collision.is_some())
        .filter_map(|(name, part)| Some((name.as_str(), meshes.get(&part.mesh)?)))
        .collect();
    let mut surfaces = GizmoAsset::new();
    for (reference, placed) in &instances {
        if let Some(mesh) = solid.get(reference.0.as_str()) {
            triangle_edges(&mut surfaces, mesh, placed.affine(), SURFACE);
        }
    }

    let solid = GizmoLineConfig {
        width: 1.5,
        ..default()
    };
    let dashed = GizmoLineConfig {
        style: GizmoLineStyle::Dashed {
            gap_scale: 3.0,
            line_scale: 3.0,
        },
        ..solid.clone()
    };
    let shapes = gizmos.add(shapes);
    // Depth tested, and pulled just far enough forward to win over a surface
    // a line lies on.
    let in_sight = -0.002;
    commands.spawn_batch([
        (
            Overlay,
            Gizmo {
                handle: shapes.clone(),
                line_config: solid.clone(),
                depth_bias: in_sight,
            },
        ),
        // The same lines again over everything, so a hidden one shows dashed.
        (
            Overlay,
            Gizmo {
                handle: shapes,
                line_config: dashed,
                depth_bias: -1.0,
            },
        ),
        (
            Overlay,
            Gizmo {
                handle: gizmos.add(surfaces),
                line_config: solid,
                depth_bias: in_sight,
            },
        ),
    ]);
}

/// A capsule about its own `+Z`, centered on the origin of `frame`: a ring
/// round each end of the cylinder, and its outline seen side on from two
/// directions a quarter turn apart.
fn capsule_outline(
    gizmo: &mut GizmoAsset,
    frame: Affine3A,
    half_length: f32,
    radius: f32,
    color: Srgba,
) {
    let turn = |i: usize| PI * i as f32 / ARC as f32;
    for z in [-half_length, half_length] {
        gizmo.linestrip(
            (0..=2 * ARC).map(|i| {
                let (sin, cos) = turn(i).sin_cos();
                frame.transform_point3(Vec3::new(radius * cos, radius * sin, z))
            }),
            color,
        );
    }
    for across in [Vec3::X, Vec3::Y] {
        // Over the top cap. The bottom cap is the same arc mirrored through
        // the center, and joining the two draws both sides.
        let cap: Vec<Vec3> = (0..=ARC)
            .map(|i| {
                let (sin, cos) = turn(i).sin_cos();
                across * radius * cos + Vec3::Z * (half_length + radius * sin)
            })
            .collect();
        let outline = cap
            .iter()
            .copied()
            .chain(cap.iter().map(|p| -*p))
            .chain([cap[0]]);
        gizmo.linestrip(outline.map(|p| frame.transform_point3(p)), color);
    }
}

/// Every triangle of `mesh`, edges only. An edge two triangles share is drawn
/// once for each.
fn triangle_edges(gizmo: &mut GizmoAsset, mesh: &Mesh, frame: Affine3A, color: Srgba) {
    let (Some(points), Some(indices)) = (
        mesh.attribute(Mesh::ATTRIBUTE_POSITION)
            .and_then(VertexAttributeValues::as_float3),
        mesh.indices(),
    ) else {
        return;
    };
    let points: Vec<Vec3> = points
        .iter()
        .map(|p| frame.transform_point3(Vec3::from(*p)))
        .collect();
    let corners: Vec<usize> = indices.iter().collect();
    for triangle in corners.as_chunks::<3>().0 {
        let [a, b, c] = triangle.map(|i| points[i]);
        gizmo.linestrip([a, b, c, a], color);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scene::doc::TRIANGLE_MESH;
    use crate::scene::{Part, Surface, cable, capsule};

    fn part(mesh: Handle<Mesh>, collision: Option<&'static str>) -> Part {
        Part {
            mesh,
            color: [0.5; 3],
            roughness: 0.8,
            reflectance: 0.5,
            translucency: 0.0,
            thickness: 0.0,
            double_sided: false,
            collision,
            heightfield_resolution: None,
        }
    }

    /// The box around everything drawn in `color`.
    fn bounds(world: &mut World, color: Srgba) -> Option<(Vec3, Vec3)> {
        let color = LinearRgba::from(color);
        let handles: Vec<_> = world
            .query::<&Gizmo>()
            .iter(world)
            .map(|gizmo| gizmo.handle.clone())
            .collect();
        let assets = world.resource::<Assets<GizmoAsset>>();
        handles
            .iter()
            .filter_map(|handle| assets.get(handle))
            .flat_map(|asset| asset.strip_positions.iter().zip(&asset.strip_colors))
            // A strip ends on a NaN.
            .filter(|(p, c)| p.is_finite() && **c == color)
            .map(|(p, _)| (*p, *p))
            .reduce(|(lo, hi), (p, _)| (lo.min(p), hi.max(p)))
    }

    fn close(actual: Option<(Vec3, Vec3)>, lo: Vec3, hi: Vec3) {
        let (a, b) = actual.expect("something drawn in this colour");
        assert!(
            a.abs_diff_eq(lo, 1e-5) && b.abs_diff_eq(hi, 1e-5),
            "{a}..{b}, not {lo}..{hi}"
        );
    }

    /// Each kind lands where it stands and in its own colour, and nothing is
    /// drawn once the box is unticked.
    #[test]
    fn every_collider_is_drawn_where_it_stands() {
        let mut app = App::new();
        app.add_plugins((
            MinimalPlugins,
            AssetPlugin::default(),
            TransformPlugin,
            plugin,
        ))
        .init_asset::<Mesh>()
        .init_asset::<GizmoAsset>()
        .init_resource::<Prototypes>();
        let world = app.world_mut();
        world.resource_mut::<ShowColliders>().0 = true;

        // A proxy from the ground to 1 m, under an organ moved and doubled.
        let organ = world
            .spawn(Transform::from_xyz(10.0, 0.0, 0.0).with_scale(Vec3::splat(2.0)))
            .id();
        world.spawn((capsule(0.1, 0.0, 1.0), ChildOf(organ)));

        // Three points a metre apart, 10 cm thick tapering to 6: two segments,
        // the first held.
        let shoot = world.spawn(Transform::IDENTITY).id();
        let skin = Surface {
            color: [0.5; 3],
            roughness: 0.8,
            reflectance: 0.5,
            translucency: 0.0,
            thickness: 0.0,
            double_sided: false,
        };
        let points = vec![[0.0; 3], [0.0, 0.0, 1.0], [0.0, 0.0, 2.0]];
        world.spawn((cable(points, vec![0.1, 0.1, 0.06], skin), ChildOf(shoot)));

        // A unit cube that collides, and one that does not.
        let cube = world
            .resource_mut::<Assets<Mesh>>()
            .add(Mesh::from(Cuboid::default()));
        let mut prototypes = world.resource_mut::<Prototypes>();
        prototypes.insert("Solid", 0, part(cube.clone(), Some(TRIANGLE_MESH)));
        prototypes.insert("Shown", 0, part(cube, None));
        world.spawn((
            Transform::from_xyz(-10.0, 0.0, 0.0),
            UsdReference("Solid_0".into()),
        ));
        world.spawn((
            Transform::from_xyz(-20.0, 0.0, 0.0),
            UsdReference("Shown_0".into()),
        ));
        app.update();

        let world = app.world_mut();
        // The proxy reaches 0.2 m round its axis and 0..2 m up it; the held
        // segment reaches 5 cm past its points, the other 4 cm past its own.
        close(
            bounds(world, STATIC),
            Vec3::new(-0.05, -0.2, -0.05),
            Vec3::new(10.2, 0.2, 2.0),
        );
        close(
            bounds(world, ROD),
            Vec3::new(-0.04, -0.04, 0.96),
            Vec3::new(0.04, 0.04, 2.04),
        );
        close(
            bounds(world, SURFACE),
            Vec3::new(-10.5, -0.5, -0.5),
            Vec3::new(-9.5, 0.5, 0.5),
        );

        world.resource_mut::<ShowColliders>().0 = false;
        app.update();
        assert!(bounds(app.world_mut(), STATIC).is_none());
    }
}

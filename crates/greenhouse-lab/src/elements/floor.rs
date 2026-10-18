//! Floor — white film under the crop, and the concrete main path.
//!
//! Both are flat slabs with their top on z = 0, sized off the layout, and
//! each is its own collider: a robot drives on the concrete and rides onto
//! the rails from it.

use bevy::prelude::*;
use misina_lab::palette::{Response, srgb};
use misina_lab::scene::doc::TRIANGLE_MESH;
use misina_lab::scene::{Library, PrimRoot, UsdType};

use super::layout::GreenhouseLayout;
use super::{Grow, cuboid};

/// How far the floor reaches below z = 0, in meters, so that it collides as a
/// solid rather than as a sheet.
const DEPTH: f32 = 0.1;

/// White ground film, smoother and shinier than the concrete beside it.
const FILM: (u32, Response) = (
    0xE4E5DF,
    Response {
        roughness: 0.6,
        reflectance: 0.45,
    },
);

/// Brushed concrete.
const CONCRETE: (u32, Response) = (
    0x9C9A94,
    Response {
        roughness: 0.9,
        reflectance: 0.3,
    },
);

/// Marks the `Floor` prim, which every rebuild replaces.
#[derive(Component)]
struct Owned;

pub fn plugin(app: &mut App) {
    app.add_systems(
        PreUpdate,
        build
            .run_if(resource_changed::<GreenhouseLayout>)
            .in_set(Grow::Fixtures),
    );
}

fn build(
    mut commands: Commands,
    mut library: Library,
    layout: Res<GreenhouseLayout>,
    root: Res<PrimRoot>,
    existing: Query<Entity, With<Owned>>,
) {
    for entity in &existing {
        commands.entity(entity).despawn();
    }
    let floor = commands
        .spawn((
            Owned,
            Name::new("Floor"),
            UsdType("Scope"),
            Transform::IDENTITY,
            Visibility::default(),
            ChildOf(root.0),
        ))
        .id();

    let half = layout.width / 2.0;
    for (name, from, to, (color, response)) in [
        ("Film", 0.0, layout.length, FILM),
        ("Concrete", -layout.main_path, 0.0, CONCRETE),
    ] {
        library.clear(name);
        // A main path of no width is not laid at all.
        if to <= from {
            continue;
        }
        let mesh = cuboid(Vec3::new(from, -half, -DEPTH), Vec3::new(to, half, 0.0));
        let geometry = library.part(name, 0, mesh.to_mesh(), response.surface(srgb(color)));
        library.collide(&geometry, TRIANGLE_MESH);
        commands.spawn((Name::new(name), geometry, ChildOf(floor)));
    }
}

//! Floor — the level ground the garden stands on, its top at z = 0.
//!
//! A slab reaching a path's width past the outermost beds, sized off the
//! layout, and its own collider: a robot drives on it and the beds stand on
//! it. Level rather than the framework's terrain, since a bed is built on
//! level ground.

use bevy::prelude::*;
use misina_lab::palette::{Response, srgb};
use misina_lab::scene::doc::TRIANGLE_MESH;
use misina_lab::scene::{Library, PrimRoot};

use super::layout::GardenLayout;
use super::{Grow, cuboid};

/// How far the floor reaches below z = 0, in meters, so that it collides as a
/// solid rather than as a sheet.
const DEPTH: f32 = 0.1;

/// Mown lawn.
const LAWN: (u32, Response) = (
    0x6B8E4E,
    Response {
        roughness: 0.95,
        reflectance: 0.25,
    },
);

/// Marks the `Floor` prim, which every rebuild replaces.
#[derive(Component)]
struct Owned;

pub fn plugin(app: &mut App) {
    app.add_systems(
        PreUpdate,
        build
            .run_if(resource_changed::<GardenLayout>)
            .in_set(Grow::Fixtures),
    );
}

fn build(
    mut commands: Commands,
    mut library: Library,
    layout: Res<GardenLayout>,
    root: Res<PrimRoot>,
) {
    commands.despawn_all::<With<Owned>>();
    library.clear("Floor");
    let (color, response) = LAWN;
    let mesh = cuboid((-layout.floor).extend(-DEPTH), layout.floor.extend(0.0));
    let geometry = library.part("Floor", 0, mesh.to_mesh(), response.surface(srgb(color)));
    library.collide(&geometry, TRIANGLE_MESH);
    commands.spawn((Owned, Name::new("Floor"), geometry, ChildOf(root.0)));
}

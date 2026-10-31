//! Bed — what fills a bed's prim: the timber frame round it and the soil in
//! it.
//!
//! The frame is boards of [`BOARD`] thickness standing on the floor up to the
//! rim, on a post inside each corner and against each long side every
//! [`POST_SPACING`] at most; the soil is a block filling the frame up to the
//! soil surface. Every bed is the same, so the garden holds one frame mesh
//! and one soil mesh, each its own collider, referenced from every bed. A
//! bed that holds a climber also gets the [`trellis`], which the planting
//! layer spawns with the plants it strings to it.

use bevy::prelude::*;
use misina_lab::geometry::mesh::{MeshData, merge_meshes};
use misina_lab::palette::{Response, srgb};
use misina_lab::scene::Library;
use misina_lab::scene::doc::TRIANGLE_MESH;

use super::layout::{BOARD, Bed, GardenLayout};
use super::{Grow, cuboid};

/// A post's square section, in meters.
const POST: f32 = 0.07;

/// How far a post stands above the rim, in meters.
const POST_PROUD: f32 = 0.03;

/// The most a long side runs from one post to the next, in meters.
const POST_SPACING: f32 = 1.2;

/// A trellis post's and rail's square section, in meters, and how deep a
/// post stands in the soil.
pub const TRELLIS_TIMBER: f32 = 0.05;
const TRELLIS_SUNK: f32 = 0.15;

/// Weathered timber.
pub const TIMBER: (u32, Response) = (
    0x9B7B5B,
    Response {
        roughness: 0.8,
        reflectance: 0.3,
    },
);

/// Moist loam.
const SOIL: (u32, Response) = (
    0x4A3728,
    Response {
        roughness: 1.0,
        reflectance: 0.2,
    },
);

/// Marks what this element spawned in a bed, which every rebuild replaces.
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
    beds: Query<Entity, With<Bed>>,
) {
    commands.despawn_all::<With<Owned>>();
    let parts = [
        ("Frame", frame(layout.bed, layout.rim_z), TIMBER),
        ("Soil", soil(layout.bed, layout.soil_z), SOIL),
    ]
    .map(|(name, mesh, (color, response))| {
        library.clear(name);
        let geometry = library.part(name, 0, mesh.to_mesh(), response.surface(srgb(color)));
        library.collide(&geometry, TRIANGLE_MESH);
        (name, geometry)
    });
    for bed in &beds {
        for (name, geometry) in &parts {
            commands.spawn((Owned, Name::new(*name), geometry.clone(), ChildOf(bed)));
        }
    }
}

/// The frame of a bed `size` across and `rim_z` high, centred on the origin
/// on the floor.
pub fn frame(size: Vec2, rim_z: f32) -> MeshData {
    let half = size / 2.0;
    let inner = half - Vec2::splat(BOARD);
    // The long sides along X, and the short sides between them.
    let mut boxes = vec![
        cuboid(
            Vec3::new(-half.x, inner.y, 0.0),
            Vec3::new(half.x, half.y, rim_z),
        ),
        cuboid(
            Vec3::new(-half.x, -half.y, 0.0),
            Vec3::new(half.x, -inner.y, rim_z),
        ),
        cuboid(
            Vec3::new(inner.x, -inner.y, 0.0),
            Vec3::new(half.x, inner.y, rim_z),
        ),
        cuboid(
            Vec3::new(-half.x, -inner.y, 0.0),
            Vec3::new(-inner.x, inner.y, rim_z),
        ),
    ];
    // Posts inside the boards: one in each corner, and along each long side
    // as many between as keep them `POST_SPACING` apart at most.
    let spans = (size.x / POST_SPACING).round().max(1.0);
    let reach = inner - Vec2::splat(POST / 2.0);
    for i in 0..=spans as usize {
        let x = (-reach.x + 2.0 * reach.x * i as f32 / spans).clamp(-reach.x, reach.x);
        for y in [reach.y, -reach.y] {
            boxes.push(cuboid(
                Vec3::new(x - POST / 2.0, y - POST / 2.0, 0.0),
                Vec3::new(x + POST / 2.0, y + POST / 2.0, rim_z + POST_PROUD),
            ));
        }
    }
    merge_meshes(&boxes)
}

/// The soil in a bed `size` across, filling its frame up to `soil_z`.
pub fn soil(size: Vec2, soil_z: f32) -> MeshData {
    let inner = size / 2.0 - Vec2::splat(BOARD);
    cuboid((-inner).extend(0.0), inner.extend(soil_z))
}

/// The trellis of a bed `size` across: a post inside each north corner,
/// sunk in the soil and standing to the rail, and the rail along the bed
/// between them, centred at `rail_z`; see [`trellis_y`] for where it stands.
pub fn trellis(size: Vec2, soil_z: f32, rail_z: f32) -> MeshData {
    let (y, half) = (trellis_y(size), TRELLIS_TIMBER / 2.0);
    let reach = size.x / 2.0 - BOARD - half;
    let mut boxes: Vec<MeshData> = [-reach, reach]
        .into_iter()
        .map(|x| {
            cuboid(
                Vec3::new(x - half, y - half, soil_z - TRELLIS_SUNK),
                Vec3::new(x + half, y + half, rail_z + half),
            )
        })
        .collect();
    boxes.push(cuboid(
        Vec3::new(-reach - half, y - half, rail_z - half),
        Vec3::new(reach + half, y + half, rail_z + half),
    ));
    merge_meshes(&boxes)
}

/// Where a bed's trellis stands across the bed: its posts' and rail's
/// centre line, just inside the north boards.
pub fn trellis_y(size: Vec2) -> f32 {
    size.y / 2.0 - BOARD - TRELLIS_TIMBER / 2.0
}

#[cfg(test)]
mod tests {
    use misina_lab::testing::bounds;

    use super::*;

    /// The soil fills the frame to its surface, and the frame stands on the
    /// floor with only its posts above the rim.
    #[test]
    fn the_soil_fills_the_frame_up_to_its_surface() {
        let (size, rim_z, soil_z) = (Vec2::new(2.4, 1.2), 0.3, 0.26);
        let (frame, soil) = (frame(size, rim_z), soil(size, soil_z));
        assert_eq!(bounds(&soil, 2), (0.0, soil_z));
        assert_eq!(
            bounds(&soil, 0),
            (-(size.x / 2.0 - BOARD), size.x / 2.0 - BOARD)
        );
        assert_eq!(bounds(&frame, 2), (0.0, rim_z + POST_PROUD));
        assert_eq!(bounds(&frame, 0), (-size.x / 2.0, size.x / 2.0));
        assert_eq!(bounds(&frame, 1), (-size.y / 2.0, size.y / 2.0));
        let trellis = trellis(size, soil_z, 2.26);
        let close = |got: (f32, f32), want: (f32, f32)| {
            assert!(
                (got.0 - want.0).abs() < 1e-5 && (got.1 - want.1).abs() < 1e-5,
                "{got:?} is not {want:?}"
            );
        };
        close(
            bounds(&trellis, 2),
            (soil_z - TRELLIS_SUNK, 2.26 + TRELLIS_TIMBER / 2.0),
        );
        close(
            bounds(&trellis, 0),
            (-(size.x / 2.0 - BOARD), size.x / 2.0 - BOARD),
        );
        let (y0, y1) = bounds(&trellis, 1);
        // Flush inside the north boards.
        assert!(
            (y1 - (size.y / 2.0 - BOARD)).abs() < 1e-5 && (y1 - y0 - TRELLIS_TIMBER).abs() < 1e-5,
            "{y0}..{y1}"
        );
    }
}

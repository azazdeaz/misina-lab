//! Layout — where the beds go, and how far the garden reaches.
//!
//! Beds lie in `columns` along X and `rows` across Y, a path's width apart
//! and with a path's width of floor round the whole garden, centred on the
//! origin. Every bed is the same size. [`author`] solves the layout and
//! replaces the `Beds` prim: an Xform per bed at its centre on the floor,
//! which the fixtures fill in bed coordinates.

use bevy::prelude::*;
use misina_lab::params::Slider;
use misina_lab::scene::{PrimRoot, UsdAttributes, UsdType};

use super::Grow;

/// Thickness of a frame's boards, in meters: two-inch lumber.
pub const BOARD: f32 = 0.04;

/// Floors on what Python may set, so that nothing is laid out at zero size:
/// a bed is at least a square of this, and its rim at least this high.
const MIN_SIDE: f32 = 0.3;
const MIN_HEIGHT: f32 = 0.05;

/// The raised beds: their size, and how they are laid out.
///
/// Beds run east–west along X in `columns`, and the `rows` are numbered
/// across Y from the south, with a path between any two beds and round the
/// whole garden.
#[derive(Resource, Reflect, Clone, Debug, PartialEq)]
#[cfg_attr(
    feature = "python",
    pyo3::pyclass(get_all, set_all, skip_from_py_object)
)]
pub struct BedParams {
    /// Width of a bed across Y, outside the frame, in meters. Up to 1.2 m is
    /// reachable from both sides; 1.5 m is FarmBot Genesis's bed.
    #[reflect(@Slider { min: 0.6, max: 1.5, step: 0.1 })]
    pub width: f32,
    /// Length of a bed along X, outside the frame, in meters. 4 by 8 feet is
    /// the common footprint, and lumber comes in 8 feet.
    #[reflect(@Slider { min: 1.2, max: 4.8, step: 0.1 })]
    pub length: f32,
    /// Height of the frame's rim above the floor, in meters. 15 to 30 cm is
    /// typical; 60 to 90 cm is an accessible bed, which a wheeled robot
    /// reaches too. Below 0.25 m the soil is shallower than the 25 cm growing
    /// guides ask for.
    #[reflect(@Slider { min: 0.15, max: 0.9, step: 0.05 })]
    pub height: f32,
    /// How far the soil surface lies below the rim, in meters.
    #[reflect(@Slider { min: 0.0, max: 0.1, step: 0.01 })]
    pub rim: f32,
    /// Width of the paths between the beds and round the garden, in meters.
    /// 45 to 60 cm is comfortable on foot; 90 cm and up takes a cart or a
    /// wheeled robot.
    #[reflect(@Slider { min: 0.45, max: 1.5, step: 0.05 })]
    pub path: f32,
    /// Beds along X in each row.
    #[reflect(@Slider { min: 1.0, max: 4.0, step: 1.0 })]
    pub columns: u32,
    /// Rows of beds across Y.
    #[reflect(@Slider { min: 1.0, max: 6.0, step: 1.0 })]
    pub rows: u32,
}

impl Default for BedParams {
    fn default() -> Self {
        Self {
            width: 1.2,
            length: 2.4,
            height: 0.3,
            rim: 0.04,
            path: 0.9,
            columns: 2,
            rows: 3,
        }
    }
}

/// The solved layout: every bed the same size, on a floor centred on the
/// origin.
#[derive(Resource, Clone, Debug, Default, PartialEq)]
pub struct GardenLayout {
    /// Half the floor's extent along X and across Y.
    pub floor: Vec2,
    /// Each bed's centre on the floor, west to east along the south row first.
    pub beds: Vec<Vec2>,
    /// A bed's footprint outside its frame: its length along X and its width
    /// across Y.
    pub bed: Vec2,
    /// Height of a frame's rim above the floor.
    pub rim_z: f32,
    /// Height of the soil surface above the floor.
    pub soil_z: f32,
}

/// A bed's prim, by its index into [`GardenLayout::beds`].
#[derive(Component, Clone, Copy, Debug)]
pub struct Bed(pub usize);

/// Marks the `Beds` prim, which every re-solve replaces.
#[derive(Component)]
struct Owned;

pub fn plugin(app: &mut App) {
    app.init_resource::<BedParams>()
        .init_resource::<GardenLayout>()
        .add_systems(
            PreUpdate,
            author
                .run_if(resource_changed::<BedParams>)
                .in_set(Grow::Layout),
        );
}

/// Re-solves the layout and replaces the bed prims, and with them everything
/// the fixtures hung there; the fixtures gate on the layout, so they refill
/// them in the same frame.
fn author(
    mut commands: Commands,
    params: Res<BedParams>,
    root: Res<PrimRoot>,
    mut layout: ResMut<GardenLayout>,
) {
    commands.despawn_all::<With<Owned>>();
    *layout = solve(&params);

    // A Scope: the group carries no transform of its own.
    let beds = commands
        .spawn((
            Owned,
            Name::new("Beds"),
            UsdType("Scope"),
            Transform::IDENTITY,
            Visibility::default(),
            ChildOf(root.0),
        ))
        .id();
    let inner = layout.bed - Vec2::splat(2.0 * BOARD);
    for (index, centre) in layout.beds.iter().enumerate() {
        commands.spawn((
            Bed(index),
            Name::new(format!("Bed_{index:02}")),
            // What a consumer placing or reading something in the bed needs,
            // without measuring the meshes: the heights of the soil and the
            // rim, and the soil's extent inside the frame.
            UsdAttributes::default()
                .with("soil_z", layout.soil_z)
                .with("rim_z", layout.rim_z)
                .with("inner_size", [inner.x, inner.y, layout.rim_z]),
            Transform::from_translation(centre.extend(0.0)),
            Visibility::default(),
            ChildOf(beds),
        ));
    }
}

/// The layout `params` asks for.
pub fn solve(params: &BedParams) -> GardenLayout {
    let bed = Vec2::new(params.length.max(MIN_SIDE), params.width.max(MIN_SIDE));
    let path = params.path.max(0.0);
    let (columns, rows) = (params.columns.max(1), params.rows.max(1));
    let rim_z = params.height.max(MIN_HEIGHT);
    let soil_z = (rim_z - params.rim.max(0.0)).max(MIN_HEIGHT / 2.0);

    // Centre to centre; the floor is a path wider than the beds on each side.
    let pitch = bed + Vec2::splat(path);
    let counts = Vec2::new(columns as f32, rows as f32);
    let beds = (0..rows)
        .flat_map(|row| (0..columns).map(move |column| Vec2::new(column as f32, row as f32)))
        .map(|at| (at - (counts - Vec2::ONE) / 2.0) * pitch)
        .collect();
    GardenLayout {
        floor: (counts * pitch + Vec2::splat(path)) / 2.0,
        beds,
        bed,
        rim_z,
        soil_z,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Beds a path apart in both directions, centred on the origin, and the
    /// floor a path past the outermost.
    #[test]
    fn beds_lie_a_path_apart_with_a_path_of_floor_round_them() {
        for (columns, rows) in [(1, 1), (2, 3), (4, 6)] {
            let params = BedParams {
                columns,
                rows,
                ..default()
            };
            let layout = solve(&params);
            let pitch = layout.bed + Vec2::splat(params.path);
            assert_eq!(layout.beds.len(), (columns * rows) as usize);
            for (i, bed) in layout.beds.iter().enumerate() {
                let (column, row) = (i % columns as usize, i / columns as usize);
                if column > 0 {
                    assert!((bed.x - layout.beds[i - 1].x - pitch.x).abs() < 1e-5, "{i}");
                }
                if row > 0 {
                    let south = layout.beds[i - columns as usize];
                    assert!((bed.y - south.y - pitch.y).abs() < 1e-5, "{i}");
                }
            }
            let centre = layout.beds.iter().sum::<Vec2>() / layout.beds.len() as f32;
            assert!(
                centre.length() < 1e-4,
                "{columns}x{rows}: centred on {centre}"
            );
            let last = layout.beds[layout.beds.len() - 1];
            let edge = layout.floor - (last + layout.bed / 2.0);
            assert!((edge - Vec2::splat(params.path)).length() < 1e-5, "{edge}");
        }
    }

    /// Python's stops: zero and negative sizes still lay out a bed with room
    /// for its frame, its soil under its rim.
    #[test]
    fn zero_sizes_still_make_a_bed() {
        let layout = solve(&BedParams {
            width: 0.0,
            length: -1.0,
            height: 0.0,
            rim: 9.0,
            path: -1.0,
            columns: 0,
            rows: 0,
        });
        assert_eq!(layout.beds.len(), 1);
        assert!(layout.bed.min_element() > 4.0 * BOARD);
        assert!(layout.soil_z > 0.0 && layout.soil_z <= layout.rim_z);
        assert_eq!(layout.floor, layout.bed / 2.0);
    }
}

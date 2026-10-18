//! Layout — where the rows and the paths go.
//!
//! Gutters run along +X from the row head at x = 0, `pitch` apart and
//! centred on y = 0, with a path on both sides of every one: each crop wall
//! faces a path a robot can drive, so there is one path more than there are
//! rows. Every row holds the same slabs and plants. The floor reaches half a
//! pitch past the outermost paths, and the main path runs across the row
//! heads at negative x.
//!
//! [`author`] solves the layout and replaces the `Rows` and `Paths` prims: an
//! Xform per row on its gutter's centreline and one per path on the path's,
//! which the fixtures fill in row and path coordinates.

use bevy::prelude::*;
use misina_lab::params::Slider;
use misina_lab::scene::{PrimRoot, UsdType};

use super::Grow;

/// A stone wool slab, in meters: its length along the row, its width and its
/// height.
pub const SLAB: Vec3 = Vec3::new(1.2, 0.195, 0.075);

/// The least gap left between two slabs along a gutter, in meters.
const SLAB_GAP: f32 = 0.05;

/// Floors on what Python may set, so that nothing is laid out at zero size.
const MIN_PITCH: f32 = 0.1;
const MIN_PLANT_SPACING: f32 = 0.05;

/// Where the rows go.
///
/// Rows run along X from the row head, where the concrete main path crosses
/// them, and are numbered across Y. Every gutter has a path on both sides, so
/// there is one path more than there are rows.
#[derive(Resource, Reflect, Clone, Debug, PartialEq)]
#[cfg_attr(
    feature = "python",
    pyo3::pyclass(get_all, set_all, skip_from_py_object)
)]
pub struct LayoutParams {
    /// How many rows, one gutter each.
    #[reflect(@Slider { min: 1.0, max: 12.0, step: 1.0 })]
    pub rows: u32,
    /// Distance between neighbouring gutters, in meters: the row pitch.
    #[reflect(@Slider { min: 1.5, max: 1.8, step: 0.05 })]
    pub pitch: f32,
    /// Length of a row from the main path to its far end, in meters. Farm rows
    /// run 90 to 115 m; a shorter one simulates faster.
    #[reflect(@Slider { min: 5.0, max: 120.0, step: 1.0 })]
    pub row_length: f32,
    /// Width of the concrete main path across the row heads, in meters: where
    /// a robot changes rows.
    #[reflect(@Slider { min: 3.0, max: 5.0, step: 0.1 })]
    pub main_path: f32,
    /// Distance between plants along a gutter, in meters, each in a
    /// propagation block on a slab. A slab carries the whole number of plants
    /// closest to this, so the real spacing comes out close to it rather than
    /// equal to it.
    #[reflect(@Slider { min: 0.3, max: 0.5, step: 0.01 })]
    pub plant_spacing: f32,
}

impl Default for LayoutParams {
    fn default() -> Self {
        Self {
            rows: 6,
            pitch: 1.6,
            row_length: 20.0,
            main_path: 3.0,
            plant_spacing: 0.36,
        }
    }
}

/// The solved layout: x along the rows, y across them.
#[derive(Resource, Clone, Debug, Default, PartialEq)]
pub struct GreenhouseLayout {
    /// Each gutter's centreline, as its y.
    pub rows: Vec<f32>,
    /// Each path's centreline, as its y.
    pub paths: Vec<f32>,
    /// How far the rows run along +X from the row head.
    pub length: f32,
    /// The floor's extent across the rows, centred on y = 0.
    pub width: f32,
    /// How far the main path reaches along -X from the row head.
    pub main_path: f32,
    /// Each slab's centre along a row, the same in every row.
    pub slabs: Vec<f32>,
    /// Each plant's place along a row, the same in every row.
    pub plants: Vec<f32>,
}

/// A row's prim, by its index into [`GreenhouseLayout::rows`].
#[derive(Component, Clone, Copy, Debug)]
pub struct Row(pub usize);

/// A path's prim, by its index into [`GreenhouseLayout::paths`].
#[derive(Component, Clone, Copy, Debug)]
pub struct Path(pub usize);

/// Marks the `Rows` and `Paths` prims, which every re-solve replaces.
#[derive(Component)]
struct Owned;

pub fn plugin(app: &mut App) {
    app.init_resource::<LayoutParams>()
        .init_resource::<GreenhouseLayout>()
        .add_systems(
            PreUpdate,
            author
                .run_if(resource_changed::<LayoutParams>)
                .in_set(Grow::Layout),
        );
}

/// Re-solves the layout and replaces the row and path prims, and with them
/// everything the fixtures hung there; the fixtures gate on the layout, so
/// they refill them in the same frame.
fn author(
    mut commands: Commands,
    params: Res<LayoutParams>,
    root: Res<PrimRoot>,
    mut layout: ResMut<GreenhouseLayout>,
    existing: Query<Entity, With<Owned>>,
) {
    for entity in &existing {
        commands.entity(entity).despawn();
    }
    *layout = solve(&params);

    // Scopes: a group carries no transform of its own.
    let mut group = |name: &'static str| {
        commands
            .spawn((
                Owned,
                Name::new(name),
                UsdType("Scope"),
                Transform::IDENTITY,
                Visibility::default(),
                ChildOf(root.0),
            ))
            .id()
    };
    let (rows, paths) = (group("Rows"), group("Paths"));
    for (index, &y) in layout.rows.iter().enumerate() {
        commands.spawn((
            Row(index),
            Name::new(format!("Row_{index:02}")),
            Transform::from_xyz(0.0, y, 0.0),
            Visibility::default(),
            ChildOf(rows),
        ));
    }
    for (index, &y) in layout.paths.iter().enumerate() {
        commands.spawn((
            Path(index),
            Name::new(format!("Path_{index:02}")),
            Transform::from_xyz(0.0, y, 0.0),
            Visibility::default(),
            ChildOf(paths),
        ));
    }
}

/// The layout `params` asks for. A row is at least one slab long.
fn solve(params: &LayoutParams) -> GreenhouseLayout {
    let count = params.rows.max(1);
    let pitch = params.pitch.max(MIN_PITCH);
    let length = params.row_length.max(SLAB.x);

    // As many slabs as fit with `SLAB_GAP` beside each, spread evenly over the
    // whole row.
    let slabs = (length / (SLAB.x + SLAB_GAP)).floor().max(1.0) as usize;
    let step = length / slabs as f32;
    let slabs: Vec<f32> = (0..slabs).map(|i| (i as f32 + 0.5) * step).collect();
    let per_slab = (SLAB.x / params.plant_spacing.max(MIN_PLANT_SPACING))
        .round()
        .max(1.0) as usize;
    let plants = slabs
        .iter()
        .flat_map(|centre| {
            (0..per_slab).map(move |i| centre + SLAB.x * ((i as f32 + 0.5) / per_slab as f32 - 0.5))
        })
        .collect();

    GreenhouseLayout {
        rows: (0..count)
            .map(|k| (k as f32 - (count - 1) as f32 / 2.0) * pitch)
            .collect(),
        paths: (0..=count)
            .map(|k| (k as f32 - count as f32 / 2.0) * pitch)
            .collect(),
        length,
        width: (count + 1) as f32 * pitch,
        main_path: params.main_path.max(0.0),
        slabs,
        plants,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Gutters `pitch` apart and centred on y = 0, each with a path half a
    /// pitch to either side, and the floor half a pitch past the outermost
    /// path.
    #[test]
    fn every_gutter_has_a_path_on_both_sides() {
        let pitch = LayoutParams::default().pitch;
        for rows in [1, 2, 6] {
            let layout = solve(&LayoutParams { rows, ..default() });
            assert_eq!(layout.rows.len(), rows as usize);
            assert_eq!(layout.paths.len(), rows as usize + 1);
            for (k, y) in layout.rows.iter().enumerate() {
                let sides = [layout.paths[k] - y, layout.paths[k + 1] - y];
                assert!(
                    (sides[0] + pitch / 2.0).abs() < 1e-5 && (sides[1] - pitch / 2.0).abs() < 1e-5,
                    "{rows} rows: gutter {k} has its paths at {sides:?}"
                );
            }
            assert!(layout.rows.iter().sum::<f32>().abs() < 1e-4, "centred");
            let edge = layout.width / 2.0 - layout.paths[rows as usize];
            assert!((edge - pitch / 2.0).abs() < 1e-5, "{rows} rows: {edge}");
        }
    }

    /// Slabs lie inside the row with a gap between each two, and every plant
    /// stands on one, at about the spacing asked for — Python's stops
    /// included.
    #[test]
    fn every_plant_stands_on_a_slab_inside_the_row() {
        for (row_length, plant_spacing) in [(20.0, 0.36), (5.0, 0.5), (0.0, 0.3), (120.0, 0.0)] {
            let params = LayoutParams {
                row_length,
                plant_spacing,
                ..default()
            };
            let layout = solve(&params);
            let half = SLAB.x / 2.0;
            let (first, last) = (layout.slabs[0], layout.slabs[layout.slabs.len() - 1]);
            assert!(
                first >= half && last + half <= layout.length + 1e-4,
                "{params:?}"
            );
            assert!(
                layout
                    .slabs
                    .windows(2)
                    .all(|pair| pair[1] - pair[0] >= SLAB.x + SLAB_GAP - 1e-4),
                "{params:?}: slabs touch"
            );
            assert!(
                layout
                    .plants
                    .iter()
                    .all(|x| layout.slabs.iter().any(|slab| (x - slab).abs() < half)),
                "{params:?}: a plant off every slab"
            );
            let asked = plant_spacing.max(MIN_PLANT_SPACING);
            let spacing = SLAB.x * layout.slabs.len() as f32 / layout.plants.len() as f32;
            assert!(
                (spacing - asked).abs() <= asked / 2.0,
                "{params:?}: {spacing}"
            );
        }
    }
}

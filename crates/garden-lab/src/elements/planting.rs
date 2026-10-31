//! Planting — what stands in the beds, and where.
//!
//! [`author`] deals each bed's soil out in 30 cm squares to the species
//! [`Commons`] lists, by their shares, and fills every square with as many
//! plants as its species' spacing asks for, on a grid jittered off exact.
//! It spawns one entity per plant under its bed's prim, carrying the
//! species' marker and the [`PlantConfig`] its recipe reads; the plant
//! layers grow the rest.
//!
//! # Path stability
//!
//! `Bed_00/Plant_017` names a planting place of the bed, not the eighteenth
//! plant: a square has sixteen numbers, a plant's being its row and column
//! on the four-by-four lattice of the densest spacing, and a place left
//! empty, by the miss rate, by a wider spacing or by a crop not yet sown,
//! leaves a gap in the numbering, so a config keyed on a path is never
//! repointed at another plant.

use bevy::prelude::*;
use misina_lab::params::Slider;
use misina_lab::rng::{Rng, salt};
use misina_lab::scene::{Order, UsdAttributes};

use super::layout::{BOARD, Bed, GardenLayout};
use super::{Grow, SceneParams};
use crate::grow::builder::smooth;
use crate::species::{Commons, Entry};

/// Salts splitting the planting's draws, and each plant's own stream, off
/// the scene seed.
const STREAM: u64 = 0x5A17_2B8C_E3D9_4F61;
const PLANT_STREAM: u64 = 0xD3B1_7E42_0C9F_A685;

/// Side of a planting square, in meters: square-foot gardening's foot.
pub const SQUARE: f32 = 0.3;

/// Most plants along a square's side, and so the places a square numbers.
const MAX_PER_SIDE: u32 = 4;
const PLACES: u64 = (MAX_PER_SIDE * MAX_PER_SIDE) as u64;

/// How far a plant's vigour strays from 1 either way.
const VIGOUR: f64 = 0.15;

/// How a bed is filled, beyond what each crop says for itself.
#[derive(Resource, Reflect, Clone, Debug, PartialEq)]
#[cfg_attr(
    feature = "python",
    pyo3::pyclass(get_all, set_all, skip_from_py_object)
)]
pub struct PlantingParams {
    /// How far a plant stands off its exact place, as a fraction of the
    /// distance to its neighbours: 0 is a stamped grid.
    #[reflect(@Slider { min: 0.0, max: 0.5, step: 0.05 })]
    pub jitter: f32,
    /// Share of planting places left empty: seeds that never came up.
    #[reflect(@Slider { min: 0.0, max: 0.5, step: 0.05 })]
    pub misses: f32,
}

impl Default for PlantingParams {
    fn default() -> Self {
        Self {
            jitter: 0.2,
            misses: 0.05,
        }
    }
}

/// One plant, as planting decided it: what its recipe reads.
#[derive(Component, Clone, Copy, Debug, PartialEq)]
pub struct PlantConfig {
    /// Weeks since it was sown: the scene's week less its species' sowing
    /// week.
    pub age: f32,
    /// How well it grows, about 1: its size against its species'.
    pub vigour: f32,
    /// Its own stream, which every draw of its recipe comes from.
    pub seed: u64,
}

impl PlantConfig {
    /// How far grown the plant is toward `harvest` weeks, eased, 0 to 1.
    pub fn grown(&self, harvest: f32) -> f32 {
        smooth(self.age / harvest.max(1e-3))
    }
}

/// Marks the plants, which every replant replaces.
#[derive(Component)]
struct Owned;

pub fn plugin(app: &mut App) {
    app.init_resource::<PlantingParams>().add_systems(
        PreUpdate,
        author
            .run_if(
                resource_changed::<GardenLayout>
                    .or_eager(resource_changed::<Commons>)
                    .or_eager(resource_changed::<PlantingParams>)
                    .or_eager(resource_changed::<SceneParams>),
            )
            .in_set(Grow::Planting),
    );
}

/// Replants every bed.
///
/// The draw order is part of this element's output: per planting place, the
/// miss, the jitter along and across, then the vigour, every place from a
/// stream of its own, so the miss rate re-rolls nothing.
fn author(
    mut commands: Commands,
    layout: Res<GardenLayout>,
    commons: Res<Commons>,
    params: Res<PlantingParams>,
    scene: Res<SceneParams>,
    beds: Query<(Entity, &Bed)>,
) {
    commands.despawn_all::<With<Owned>>();
    let inner = layout.bed - Vec2::splat(2.0 * BOARD);
    // Whole squares, centred in the bed; a bed narrower than one holds none.
    let squares = (inner / SQUARE).floor().as_uvec2();
    let corner = -squares.as_vec2() * SQUARE / 2.0;
    let jitter = params.jitter.clamp(0.0, 0.5);
    for (bed, &Bed(index)) in &beds {
        let bed_index = index as u64;
        for (cell, plot) in deal(&commons, index as u32 + 1, (squares.x * squares.y) as usize)
            .into_iter()
            .enumerate()
        {
            let Some((name, entry)) = plot else {
                continue;
            };
            let square = Vec2::new(
                (cell as u32 % squares.x) as f32,
                (cell as u32 / squares.x) as f32,
            );
            let per_side = (SQUARE / entry.common.spacing.max(1e-3))
                .round()
                .clamp(1.0, MAX_PER_SIDE as f32) as u32;
            let pitch = SQUARE / per_side as f32;
            let age = scene.week - entry.common.sown_week;
            for k in 0..per_side * per_side {
                let (col, row) = (k % per_side, k / per_side);
                let number = cell as u64 * PLACES + (row * MAX_PER_SIDE + col) as u64;
                let mut rng = Rng::new(scene.seed ^ STREAM ^ salt(bed_index << 32 | number));
                let missed = rng.unit() < params.misses as f64;
                let off = Vec2::new(rng.range(-1.0, 1.0) as f32, rng.range(-1.0, 1.0) as f32)
                    * jitter
                    * pitch;
                let vigour = rng.range(1.0 - VIGOUR, 1.0 + VIGOUR) as f32;
                // Not sown yet, or never came up.
                if age < 0.0 || missed {
                    continue;
                }
                let slot = Vec2::new(col as f32, row as f32) + Vec2::splat(0.5);
                let at = corner + square * SQUARE + slot * pitch + off;
                let mut plant = commands.spawn((
                    Owned,
                    Name::new(format!("Plant_{number:03}")),
                    Order(bed_index << 32 | number),
                    PlantConfig {
                        age,
                        vigour,
                        seed: scene.seed ^ PLANT_STREAM ^ salt(bed_index << 32 | number),
                    },
                    UsdAttributes::default()
                        .with("species", name)
                        .with("crop", true)
                        .with("age", age),
                    Transform::from_translation(at.extend(layout.soil_z)),
                    Visibility::default(),
                    ChildOf(bed),
                ));
                (entry.tag)(&mut plant);
            }
        }
    }
}

/// Which species holds each of a bed's `squares`: the species allowed in
/// bed `bed` (from 1), in name order, each a block of squares in proportion
/// to its share. With no species allowed, every square is empty.
fn deal(commons: &Commons, bed: u32, squares: usize) -> Vec<Option<(&'static str, &Entry)>> {
    let allowed: Vec<(&'static str, &Entry)> = commons
        .0
        .iter()
        .filter(|(_, entry)| {
            entry.common.share > 0.0 && (entry.common.bed == 0 || entry.common.bed == bed)
        })
        .map(|(name, entry)| (*name, entry))
        .collect();
    let total: f32 = allowed.iter().map(|(_, entry)| entry.common.share).sum();
    let mut plots = vec![None; squares];
    if total <= 0.0 {
        return plots;
    }
    let (mut from, mut cumulative) = (0, 0.0);
    for (name, entry) in allowed {
        cumulative += entry.common.share / total;
        let to = ((cumulative * squares as f32).round() as usize).min(squares);
        for plot in &mut plots[from.min(to)..to] {
            *plot = Some((name, entry));
        }
        from = to;
    }
    plots
}

#[cfg(test)]
mod tests {
    use bevy::ecs::system::EntityCommands;

    use super::*;
    use crate::species::Common;

    fn entry(share: f32, bed: u32) -> Entry {
        Entry {
            common: Common {
                share,
                spacing: 0.3,
                sown_week: 0.0,
                bed,
            },
            tag: |_: &mut EntityCommands| {},
        }
    }

    /// Squares go out in blocks by share, in name order; a species confined
    /// to another bed gets none here, and a share of 0 none anywhere.
    #[test]
    fn squares_are_dealt_by_share() {
        let mut commons = Commons::default();
        commons.0.insert("a", entry(0.75, 0));
        commons.0.insert("b", entry(0.25, 0));
        commons.0.insert("c", entry(1.0, 2));
        commons.0.insert("d", entry(0.0, 0));
        let names = |commons: &Commons, bed: u32| -> Vec<Option<&str>> {
            deal(commons, bed, 8)
                .into_iter()
                .map(|plot| plot.map(|(name, _)| name))
                .collect()
        };
        assert_eq!(
            names(&commons, 1),
            [vec![Some("a"); 6], vec![Some("b"); 2]].concat()
        );
        assert_eq!(
            names(&commons, 2),
            [vec![Some("a"); 3], vec![Some("b"); 1], vec![Some("c"); 4]].concat()
        );
        commons.0.clear();
        assert_eq!(names(&commons, 1), vec![None; 8]);
    }
}

//! Wire — the two crop wires over every gutter.
//!
//! A row's stems are trained in a V, alternately up to one wire or the other,
//! `spread` either side of the gutter. Each wire is one straight span the
//! length of the row: a meter-long cylinder stretched to it.

use bevy::prelude::*;
use misina_lab::geometry::mesh::cylinder_mesh;
use misina_lab::palette::srgb;
use misina_lab::params::Slider;
use misina_lab::scene::Library;

use super::layout::{GreenhouseLayout, Row};
use super::{Grow, STEEL};

/// A crop wire's radius, in meters.
const RADIUS: f32 = 0.0015;

/// Galvanized wire, a shade lighter than the rails.
const COLOR: u32 = 0xB2B6B6;

/// The two crop wires over every gutter, which the stems are trained up to in
/// a V.
#[derive(Resource, Reflect, Clone, Debug, PartialEq)]
#[cfg_attr(
    feature = "python",
    pyo3::pyclass(get_all, set_all, skip_from_py_object)
)]
pub struct WireParams {
    /// Height of the crop wires above the floor, in meters: how tall the crop
    /// stands.
    #[reflect(@Slider { min: 3.5, max: 5.5, step: 0.1 })]
    pub height: f32,
    /// How far each of a row's two wires runs to the side of its gutter, in
    /// meters. An estimate, to be set from a measured site.
    #[reflect(@Slider { min: 0.1, max: 0.6, step: 0.01 })]
    pub spread: f32,
}

impl Default for WireParams {
    fn default() -> Self {
        Self {
            height: 4.5,
            spread: 0.35,
        }
    }
}

/// Marks a row's wires, which every rebuild replaces.
#[derive(Component)]
struct Owned;

pub fn plugin(app: &mut App) {
    app.init_resource::<WireParams>().add_systems(
        PreUpdate,
        build
            .run_if(resource_changed::<WireParams>.or_eager(resource_changed::<GreenhouseLayout>))
            .in_set(Grow::Fixtures),
    );
}

fn build(
    mut commands: Commands,
    mut library: Library,
    params: Res<WireParams>,
    layout: Res<GreenhouseLayout>,
    rows: Query<Entity, With<Row>>,
    existing: Query<Entity, With<Owned>>,
) {
    for entity in &existing {
        commands.entity(entity).despawn();
    }
    library.clear("Wire");
    let geometry = library.part(
        "Wire",
        0,
        cylinder_mesh(RADIUS, RADIUS, 1.0, 6).to_mesh(),
        STEEL.surface(srgb(COLOR)),
    );

    // Left and right looking along the row from its head, both running +X.
    for row in &rows {
        for (name, y) in [("Wire_L", params.spread), ("Wire_R", -params.spread)] {
            commands.spawn((
                Owned,
                Name::new(name),
                geometry.clone(),
                Transform {
                    translation: Vec3::new(0.0, y, params.height),
                    rotation: Quat::from_rotation_arc(Vec3::Z, Vec3::X),
                    scale: Vec3::new(1.0, 1.0, layout.length),
                },
                ChildOf(row),
            ));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::elements::{GreenhouseParams, grown};
    use misina_lab::testing::prim;

    /// Every row carries its two wires at the height and spread asked for,
    /// from the row head to its far end.
    #[test]
    fn every_row_carries_two_wires_the_length_of_it() {
        let params = GreenhouseParams::default();
        let (wire, length) = (params.wire.clone(), params.layout.row_length);
        let mut app = grown(params.clone());
        for row in 0..params.layout.rows {
            for (name, y) in [("Wire_L", wire.spread), ("Wire_R", -wire.spread)] {
                let row = format!("Row_{row:02}");
                let span = prim(app.world_mut(), &["Rows", &row, name]).unwrap();
                let placed = *app.world().entity(span).get::<Transform>().unwrap();
                let end = placed.transform_point(Vec3::Z);
                assert!(
                    placed.translation.distance(Vec3::new(0.0, y, wire.height)) < 1e-6,
                    "{row}/{name}"
                );
                assert!(
                    end.distance(Vec3::new(length, y, wire.height)) < 1e-4,
                    "{row}/{name} ends at {end}"
                );
            }
        }
    }
}

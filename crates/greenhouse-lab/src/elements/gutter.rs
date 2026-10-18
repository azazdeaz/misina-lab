//! Gutter — the trough each row's crop stands in, with its slabs and blocks.
//!
//! A U-shaped steel channel runs the length of the row at `height`, either
//! hung from a rod every few meters, up to the crop wires, or set on a stand
//! every few meters. Stone wool slabs lie end to end in it, and a propagation
//! block sits on a slab at every plant. The slabs and blocks have one look and
//! no knobs of their own; the layout says where they go.

use bevy::prelude::*;
use misina_lab::geometry::mesh::{MeshData, cylinder_mesh, merge_meshes};
use misina_lab::palette::{Response, srgb};
use misina_lab::params::{Choices, Slider};
use misina_lab::scene::{Geometry, Library};

use super::layout::{GreenhouseLayout, Row, SLAB};
use super::wire::WireParams;
use super::{Grow, STEEL, cuboid};

/// The channel's outer width, wall height and sheet thickness, in meters.
pub const WIDTH: f32 = 0.2;
pub const DEPTH: f32 = 0.06;
const SHEET: f32 = 0.002;

/// A propagation block, in meters.
const BLOCK: Vec3 = Vec3::new(0.1, 0.1, 0.065);

/// Distance between the hangers, or the stands, along a gutter, in meters. An
/// estimate.
const SUPPORT_SPACING: f32 = 2.5;

/// A hanger rod's radius, in meters.
const HANGER_RADIUS: f32 = 0.004;

/// A stand's footprint, in meters.
const STAND: Vec2 = Vec2::splat(0.05);

/// Pale coated steel: the channel, its hangers and its stands.
const GUTTER_COLOR: u32 = 0xD3D6D3;

/// The slab's white plastic wrap.
const SLAB_SURFACE: (u32, Response) = (
    0xEDEDE7,
    Response {
        roughness: 0.55,
        reflectance: 0.5,
    },
);

/// Bare stone wool: tan, and as matte as anything in the scene.
const BLOCK_SURFACE: (u32, Response) = (
    0xA8986A,
    Response {
        roughness: 0.95,
        reflectance: 0.2,
    },
);

/// What holds a gutter up.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Mount {
    #[default]
    Hanging,
    Standing,
}

impl Mount {
    pub const ALL: [Mount; 2] = [Mount::Hanging, Mount::Standing];

    /// The names, in [`ALL`](Self::ALL) order.
    pub const NAMES: [&'static str; 2] = ["hanging", "standing"];

    pub fn name(self) -> &'static str {
        Self::NAMES[self as usize]
    }

    pub fn parse(name: &str) -> Option<Mount> {
        Self::ALL.into_iter().find(|mount| mount.name() == name)
    }
}

/// The gutter each row's crop stands in, and what holds it up.
///
/// Stone wool slabs lie end to end in the gutter, and a propagation block sits
/// on a slab at every plant; the rows' plant spacing places them.
#[derive(Resource, Reflect, Clone, Debug, PartialEq)]
#[cfg_attr(
    feature = "python",
    pyo3::pyclass(get_all, set_all, skip_from_py_object)
)]
pub struct GutterParams {
    /// Height of the gutter's floor above the ground, in meters: where the
    /// slabs rest.
    #[reflect(@Slider { min: 0.3, max: 1.0, step: 0.05 })]
    pub height: f32,
    /// What holds the gutter up: `hanging` from a rod every 2.5 m, which
    /// leaves the floor under it clear, or `standing` on a stand every 2.5 m.
    #[reflect(@Choices(&Mount::NAMES))]
    pub mount: String,
}

impl Default for GutterParams {
    fn default() -> Self {
        Self {
            height: 0.5,
            mount: Mount::default().name().into(),
        }
    }
}

impl GutterParams {
    /// Height of the gutter's floor above the ground, as built.
    pub fn floor(&self) -> f32 {
        self.height.max(0.0)
    }

    /// Height of the blocks' tops above the ground, where the stems start.
    pub fn block_top(&self) -> f32 {
        self.floor() + SHEET + SLAB.z + BLOCK.z
    }

    /// The parsed mount. An unknown name is read as the default: Python has
    /// already rejected it, and a build system is no place to fail.
    pub fn mount(&self) -> Mount {
        Mount::parse(&self.mount).unwrap_or_else(|| {
            warn!(
                "gutter mount {:?} is not one of {:?}; using {:?}",
                self.mount,
                Mount::NAMES,
                Mount::default().name()
            );
            Mount::default()
        })
    }
}

/// Marks what this element spawned in a row, which every rebuild replaces.
#[derive(Component)]
struct Owned;

pub fn plugin(app: &mut App) {
    // `WireParams` because a hanger reaches up to the wires.
    app.init_resource::<GutterParams>().add_systems(
        PreUpdate,
        build
            .run_if(
                resource_changed::<GutterParams>
                    .or_eager(resource_changed::<WireParams>)
                    .or_eager(resource_changed::<GreenhouseLayout>),
            )
            .in_set(Grow::Fixtures),
    );
}

fn build(
    mut commands: Commands,
    mut library: Library,
    params: Res<GutterParams>,
    wire: Res<WireParams>,
    layout: Res<GreenhouseLayout>,
    rows: Query<Entity, With<Row>>,
) {
    commands.despawn_all::<With<Owned>>();
    for prefix in ["Gutter", "Hanger", "Stand", "Slab", "Block"] {
        library.clear(prefix);
    }

    let height = params.floor();
    let steel = STEEL.surface(srgb(GUTTER_COLOR));
    let gutter = library.part("Gutter", 0, channel(layout.length).to_mesh(), steel);
    let [slab, block] = [
        ("Slab", SLAB, SLAB_SURFACE),
        ("Block", BLOCK, BLOCK_SURFACE),
    ]
    .map(|(name, size, (color, response))| {
        let mesh = cuboid(-size / 2.0, size / 2.0).to_mesh();
        library.part(name, 0, mesh, response.surface(srgb(color)))
    });

    // What holds the gutter up: a part a meter tall on its origin, where its
    // foot is in the row, and how far it reaches.
    let (name, mesh, foot, reach) = match params.mount() {
        Mount::Hanging => (
            "Hanger",
            cylinder_mesh(HANGER_RADIUS, HANGER_RADIUS, 1.0, 6),
            Vec3::new(0.0, WIDTH / 2.0, height + DEPTH),
            wire.height - height - DEPTH,
        ),
        Mount::Standing => {
            let footprint = STAND.extend(0.0) / 2.0;
            let mesh = cuboid(-footprint, footprint.with_z(1.0));
            ("Stand", mesh, Vec3::ZERO, height)
        }
    };
    let support = (reach > 0.0).then(|| library.part(name, 0, mesh.to_mesh(), steel));
    let supports = (layout.length / SUPPORT_SPACING).round().max(1.0) as usize;
    // A slab rests on the channel's floor, and a block on the slab.
    let slab_z = height + SHEET + SLAB.z / 2.0;
    let block_z = height + SHEET + SLAB.z + BLOCK.z / 2.0;

    for row in &rows {
        let mut place = |name: String, geometry: &Geometry, transform: Transform| {
            commands.spawn((
                Owned,
                Name::new(name),
                geometry.clone(),
                transform,
                ChildOf(row),
            ));
        };
        place(
            "Gutter".into(),
            &gutter,
            Transform::from_xyz(0.0, 0.0, height),
        );
        if let Some(support) = &support {
            for i in 0..supports {
                let x = (i as f32 + 0.5) * layout.length / supports as f32;
                let transform = Transform::from_translation(foot.with_x(x))
                    .with_scale(Vec3::new(1.0, 1.0, reach));
                place(format!("{name}_{i:02}"), support, transform);
            }
        }
        for (i, &x) in layout.slabs.iter().enumerate() {
            place(
                format!("Slab_{i:02}"),
                &slab,
                Transform::from_xyz(x, 0.0, slab_z),
            );
        }
        for (i, &x) in layout.plants.iter().enumerate() {
            place(
                format!("Block_{i:03}"),
                &block,
                Transform::from_xyz(x, 0.0, block_z),
            );
        }
    }
}

/// The channel, open at the top: `length` along +X from the origin, centred on
/// y = 0, its underside on z = 0.
fn channel(length: f32) -> MeshData {
    let (side, wall) = (WIDTH / 2.0, WIDTH / 2.0 - SHEET);
    merge_meshes(&[
        cuboid(Vec3::new(0.0, -side, 0.0), Vec3::new(length, side, SHEET)),
        cuboid(
            Vec3::new(0.0, -side, SHEET),
            Vec3::new(length, -wall, DEPTH),
        ),
        cuboid(Vec3::new(0.0, wall, SHEET), Vec3::new(length, side, DEPTH)),
    ])
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::elements::{GreenhouseParams, grown};
    use misina_lab::testing::{named_children, prim};

    /// Row 0's prims by name, each with its transform in the row.
    fn row(params: GreenhouseParams) -> Vec<(String, Transform)> {
        let mut app = grown(params);
        let world = app.world_mut();
        let row = prim(world, &["Rows", "Row_00"]).unwrap();
        named_children(world, row)
            .into_iter()
            .map(|(name, prim)| (name, *world.entity(prim).get::<Transform>().unwrap()))
            .collect()
    }

    /// The slabs rest on the gutter's floor and every block on a slab, at any
    /// height the gutter is set to.
    #[test]
    fn the_blocks_stand_on_the_slabs_on_the_gutter_floor() {
        for height in [0.3, 0.5, 1.0] {
            let mut params = GreenhouseParams::default();
            params.gutter.height = height;
            for (name, placed) in row(params) {
                let underside = match name.split_once('_').map_or(name.as_str(), |(kind, _)| kind) {
                    "Gutter" => placed.translation.z,
                    "Slab" => placed.translation.z - SLAB.z / 2.0 - SHEET,
                    "Block" => placed.translation.z - BLOCK.z / 2.0 - SHEET - SLAB.z,
                    _ => continue,
                };
                assert!((underside - height).abs() < 1e-5, "{height}: {name}");
            }
        }
    }

    /// A hanging gutter hangs from rods reaching from its rim to the crop
    /// wires; a standing one stands on posts from the floor to its underside.
    #[test]
    fn a_gutter_hangs_to_the_wires_or_stands_on_the_floor() {
        for mount in Mount::ALL {
            let mut params = GreenhouseParams::default();
            params.gutter.mount = mount.name().into();
            let (gutter, wire) = (params.gutter.height, params.wire.height);
            let (kind, from, to) = match mount {
                Mount::Hanging => ("Hanger_", gutter + DEPTH, wire),
                Mount::Standing => ("Stand_", 0.0, gutter),
            };
            let prims = row(params);
            let supports: Vec<&(String, Transform)> = prims
                .iter()
                .filter(|(name, _)| name.starts_with("Hanger_") || name.starts_with("Stand_"))
                .collect();
            assert!(!supports.is_empty(), "{mount:?}");
            for (name, placed) in supports {
                let (bottom, top) = (placed.translation.z, placed.translation.z + placed.scale.z);
                assert!(name.starts_with(kind), "{mount:?}: {name}");
                assert!(
                    (bottom - from).abs() < 1e-5 && (top - to).abs() < 1e-5,
                    "{mount:?}: {name} spans {bottom}..{top}"
                );
            }
        }
    }
}

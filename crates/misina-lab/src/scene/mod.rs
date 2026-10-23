//! The scene graph the generator builds, and the components that tell the
//! exporter what to make of it.
//!
//! Every layer spawns ordinary Bevy entities — a [`Transform`], a [`Name`], a
//! `Mesh3d` where there is geometry — and the viewer renders them with no
//! translation step at all. The components here are *export directives*: they
//! say nothing about how an entity draws, only about what prim it becomes.
//!
//! The one that matters is [`UsdReference`]. An entity carrying it still holds
//! a real `Mesh3d` with a shared handle, which is exactly how Bevy batches
//! tens of thousands of leaves; the component only tells the exporter to emit a
//! reference to the shared part instead of a copy of its points.
//!
//! [`UsdAttributes`] is how an element labels a prim with ground truth a
//! consumer reads off the stage: whether a robot may grasp it, how much room
//! it has around it.
//!
//! # The shape of the tree
//!
//! Structural prims are unique and have children. Geometry prims reference a
//! part, carry no children, and are instanceable:
//!
//! ```text
//! /Scene/Planting/Row_00/Plant_047        Xform, unique
//!   /Wood                                 -> parts/Plant_3, instanceable
//!   /Collision                            Capsule, the trunk's proxy
//!   /Stem_00                              Xform, unique
//!     /Tube                               -> parts/Stem_1, instanceable
//!     /Leaf_00                            -> parts/Leaf_2, instanceable
//! ```
//!
//! Keeping geometry at the leaves is what makes `instanceable` safe to set
//! unconditionally: an instanceable prim's descendants are not addressable,
//! and these have no descendants to lose.
//!
//! # What a robot bumps into
//!
//! Only what it has to: the ground, the posts and the trunks. The ground
//! collides as its own mesh ([`Library::collide`]) because the shape *is* the
//! terrain; everything else gets a [`capsule`] proxy, which needs no cooking
//! and is the only round shape PhysX has natively. None of it is a rigid body
//! — a generated scene is scenery that stands still.
//!
//! Except where it does not: a [`cable`] is a flexible organ, which a solver
//! that understands one turns into bodies of its own. It replaces the mesh
//! rather than standing in for it, so it is geometry and collider at once.
//! What the organ is drawn with, and whatever hangs off it, rides a prim per
//! segment, which the solver drives; see [`spawn_rod`].

pub mod doc;
pub mod export;

use std::collections::BTreeMap;
use std::f32::consts::FRAC_PI_2;

use bevy::ecs::system::SystemParam;
use bevy::prelude::*;
use bevy::transform::TransformSystems;

use crate::Generator;
use crate::geometry::mesh::{MeshData, cylinder_mesh};

/// The mesh library, the scene root named after `G` in `G`'s namespace, and
/// [`mark_rebuilt_parents`].
pub fn plugin<G: Generator>(app: &mut App) {
    app.init_resource::<Prototypes>()
        .add_systems(Startup, spawn_root::<G>)
        .add_systems(
            PostUpdate,
            mark_rebuilt_parents.before(TransformSystems::Propagate),
        );
}

/// Marks every entity whose children changed as moved, so that transform
/// propagation reaches the children a rebuild spawned.
///
/// Bevy 0.20.0's multi-threaded `mark_dirty_trees` records the entities it has
/// visited by index alone, and visits despawned ones too. A child spawned into
/// an index that a despawn freed in the same frame therefore looks visited, the
/// climb to its parent stops there, and `propagate_parent_transforms` skips the
/// unmarked parent's subtree: the child keeps an identity `GlobalTransform` and
/// draws at the origin. Layers rebuilt in one frame do exactly that, the later
/// one spawning into the indices the earlier one freed. A parent marked here is
/// marked by its own change, whatever indices its children have.
///
/// Bevy 0.20.1 no longer visits despawned entities there; this system, and the
/// test that covers it, go once the workspace is on it.
fn mark_rebuilt_parents(mut parents: Query<&mut Transform, Changed<Children>>) {
    for mut transform in &mut parents {
        transform.set_changed();
    }
}

/// The entity every element spawns its prims under — the scene root, and the
/// stage's default prim.
#[derive(Resource, Debug, Clone, Copy)]
pub struct PrimRoot(pub Entity);

/// Takes the scene's `+Z` onto the world's `+Y`, standing a Z-up scene upright
/// for Bevy's Y-up renderer.
///
/// The scene is authored Z-up because that is what the export target needs, and
/// `upAxis` is root-layer-only metadata that does not compose through
/// references — a stage cannot declare Y-up and leave a consumer to correct for
/// it. So the correction belongs to the viewer alone, which is why it sits on a
/// parent of [`PrimRoot`] rather than inside the exported subtree.
pub fn z_up_to_y_up() -> Quat {
    Quat::from_rotation_x(-FRAC_PI_2)
}

/// Spawns the scene root under the Z-up correction, and publishes it as
/// [`PrimRoot`].
fn spawn_root<G: Generator>(mut commands: Commands) {
    let upright = commands
        .spawn((
            Transform::from_rotation(z_up_to_y_up()),
            Visibility::default(),
        ))
        .id();
    // The identity `Transform` and `Visibility` are load-bearing: propagation
    // stops at an entity that has neither, which would leave everything below
    // this one unrotated and never drawn.
    //
    // Unnamed parent, named child: the export walk starts at the `Name`, so
    // the correction never reaches the document.
    let root = commands
        .spawn((
            UsdRoot {
                namespace: G::PACKAGE,
            },
            Name::new(G::NAME),
            Transform::IDENTITY,
            Visibility::default(),
            ChildOf(upright),
        ))
        .id();
    commands.insert_resource(PrimRoot(root));
}

/// Marks the entity the export walk starts from, which becomes the stage's
/// default prim.
///
/// Its parent carries the Z-up correction the viewer needs (see
/// [`viewer`](crate::viewer)) and is deliberately *not* exported: the scene
/// below this entity is already authored in the target's Z-up convention, and
/// the rotation exists only so Bevy's Y-up camera has something upright to
/// look at.
#[derive(Component, Debug)]
pub struct UsdRoot {
    /// The namespace every [`UsdAttributes`] name is authored in: the
    /// generator's `PACKAGE`.
    pub namespace: &'static str,
}

/// Draw the [`Part`] of this name instead of inlining geometry.
///
/// The name is a key into [`Prototypes`], not a prim path — the exporter turns
/// it into one. An entity carrying this must have no children.
#[derive(Component, Clone, Debug, PartialEq, Eq)]
pub struct UsdReference(pub String);

/// Overrides the prim type, which is otherwise `Xform`.
///
/// Only worth setting for `Scope`, on a prim that groups without placing —
/// a row carries no transform of its own, because its plants are each
/// draped onto terrain that one row transform could not follow.
#[derive(Component, Clone, Copy, Debug)]
pub struct UsdType(pub &'static str);

/// Custom attributes for the prim this entity becomes, by name within the
/// [`UsdRoot`]'s namespace.
///
/// For ground truth a consumer reads off the stage and nothing renders or
/// simulates. Authored on the prim itself, so a referencing prim may carry
/// them: they stay on the instance, readable and per prim, while its geometry
/// is shared.
///
/// ```ignore
/// UsdAttributes::default().with("graspable", true).with("clearance", 0.12)
/// ```
#[derive(Component, Clone, Debug, Default, PartialEq)]
pub struct UsdAttributes(pub BTreeMap<String, doc::AttributeValue>);

impl UsdAttributes {
    /// Sets `name` to `value`, replacing what it held.
    pub fn with(mut self, name: &str, value: impl Into<doc::AttributeValue>) -> Self {
        self.0.insert(name.to_string(), value.into());
        self
    }
}

/// The prim name a collision proxy takes, under the organ it stands in for.
///
/// One name for every element, so a consumer filtering the scene down to what
/// it can bump into has a single rule.
pub const COLLISION: &str = "Collision";

/// A collision proxy — geometry a physics engine reads instead of the mesh
/// beside it. See [`capsule`], which is the only way one is built.
#[derive(Component, Clone, Copy, Debug)]
pub struct Collider(pub doc::Capsule);

/// The prim a capsule collider spanning `z0..z1` up the organ's own `+Z`
/// becomes. See [`capsule_between`].
pub fn capsule(radius: f32, z0: f32, z1: f32) -> impl Bundle + Copy {
    capsule_between(radius, Vec3::Z * z0, Vec3::Z * z1)
}

/// The prim a capsule collider from `from` to `to`, in the organ's own frame,
/// becomes: its type, its shape, and the transform laying it along that line,
/// centered on it.
///
/// USD measures a capsule's `height` across its cylindrical section alone, so
/// a span shorter than its own two caps degenerates into a sphere reaching
/// past both ends of it.
pub fn capsule_between(radius: f32, from: Vec3, to: Vec3) -> impl Bundle + Copy {
    let span = to - from;
    (
        UsdType("Capsule"),
        Collider(doc::Capsule {
            radius,
            height: (span.length() - 2.0 * radius).max(0.0),
        }),
        Transform {
            translation: (from + to) / 2.0,
            rotation: Quat::from_rotation_arc(Vec3::Z, span.try_normalize().unwrap_or(Vec3::Z)),
            scale: Vec3::ONE,
        },
    )
}

/// The prim name a deformable curve takes, under the organ it stands in for.
///
/// One name for every element, so a consumer selecting the flexible parts of
/// the scene has a single rule.
pub const CABLE: &str = "Cable";

/// A deformable curve — geometry a physics engine bends, in place of the mesh
/// it replaces. See [`cable`], which is the only way one is built.
#[derive(Component, Clone, Debug)]
pub struct Cable(pub doc::Cable);

/// The prim a flexible organ becomes: its type, the centerline a solver bends,
/// the taper it is drawn and simulated with, and the colour it is drawn in.
/// `points` are in the organ's own frame, held end first, and `widths` gives
/// the diameter at each of them.
///
/// `skin` is the [`Surface`] of the mesh this curve stands in for, so a
/// flexible organ and a rigid one of the same kind draw alike.
///
/// `loads` is the mass hanging from each point, in kilograms, or empty for an
/// organ that carries nothing; see [`doc::Cable::loads`].
///
/// No transform of its own — the points are already where the organ put them,
/// and the first segment between them is what gets held.
pub fn cable(
    points: Vec<[f32; 3]>,
    widths: Vec<f32>,
    loads: Vec<f32>,
    skin: Surface,
) -> impl Bundle {
    assert_eq!(points.len(), widths.len(), "one width per point");
    assert!(
        loads.is_empty() || loads.len() == points.len(),
        "one load per point, or none"
    );
    (
        UsdType("BasisCurves"),
        Cable(doc::Cable {
            points,
            widths,
            loads,
            display_color: skin.color,
            roughness: skin.roughness,
            reflectance: skin.reflectance,
        }),
    )
}

/// The prim each segment's tube takes, under the segment's own: one name for
/// every element, as [`CABLE`] is. `Shears` in `misina_lab/isaaclab/cutting.py`
/// finds a tube by it, to redraw it when a cut shortens its segment.
pub const STEM: &str = "Stem";

/// The prim name Newton gives the rigid body it builds for segment `index` of
/// a cable.
///
/// A **sibling** of the curve rather than a child: the label is the curve's own
/// prim path with this suffix. Authoring the prim ourselves is the whole of how
/// anything rides a rod — the importer adopts a prim already sitting at the
/// path, keeps its children, and drives its transform from then on.
///
/// Get the name wrong and nothing fails loudly: the importer defines its own
/// prim beside ours, and whatever hangs on ours stays at the rest shape.
pub fn segment_name(index: usize) -> String {
    format!("{CABLE}_edge_body_{index}")
}

/// The rest frame of every segment `points` becomes, in the organ's own frame:
/// origin at the segment's midpoint, local +Z along it.
///
/// That is Newton's frame for the capsule body, so a prim authored here starts
/// exactly where the solver will go on keeping it.
pub fn segment_frames(points: &[[f32; 3]]) -> Vec<Transform> {
    points
        .windows(2)
        .map(|pair| {
            let (from, to) = (Vec3::from(pair[0]), Vec3::from(pair[1]));
            Transform {
                translation: (from + to) * 0.5,
                rotation: Quat::from_rotation_arc(Vec3::Z, (to - from).normalize()),
                scale: Vec3::ONE,
            }
        })
        .collect()
}

/// One tube per segment of a cable through `points`, `widths` across at them,
/// each in that segment's own frame — what a flexible organ is drawn with, in
/// place of its curve.
///
/// Each tube is centered on the segment midpoint to match [`segment_frames`],
/// and overruns both of its ends by its own radius there: the tubes are
/// butt-jointed and turn by up to about 45° at a joint, where flush ends would
/// leave a wedge of daylight.
pub fn segment_tubes(points: &[[f32; 3]], widths: &[f32], sides: usize) -> Vec<MeshData> {
    points
        .windows(2)
        .zip(widths.windows(2))
        .map(|(pair, widths)| {
            let (from, to) = (Vec3::from(pair[0]), Vec3::from(pair[1]));
            let length = (to - from).length();
            let (base, top) = (widths[0] / 2.0, widths[1] / 2.0);
            let mut tube = cylinder_mesh(base, top, length + base + top, sides);
            for point in &mut tube.points {
                point[2] -= length / 2.0 + base;
            }
            tube
        })
        .collect()
}

/// Which segment of a cable whose points sit at `stations` carries whatever
/// sits at station `s`: the one it falls in, the first before the cable starts
/// and the last past its end.
///
/// Segment 0 is the held one, so what lands there rides the wood.
pub fn segment_of(stations: &[f32], s: f32) -> usize {
    stations
        .iter()
        .rposition(|station| *station <= s)
        .unwrap_or(0)
        .min(stations.len() - 2)
}

/// `placement` expressed in `frame`, so that a prim under `frame` still lands
/// where `placement` put it. A frame is a rotation and a translation only,
/// which is what makes the inverse this short.
pub fn rebase(frame: &Transform, placement: &Transform) -> Transform {
    let turn = frame.rotation.inverse();
    Transform {
        translation: turn * (placement.translation - frame.translation),
        rotation: turn * placement.rotation,
        scale: placement.scale,
    }
}

/// A rod under `organ`: its `cable`, and a prim per segment at that segment's
/// rest frame in `frames` (see [`segment_frames`]), each holding its tube from
/// `tubes` as [`STEM`]. Returns the segment prims, one per frame, for whatever
/// else rides them; place a rider with [`rebase`].
///
/// Every segment gets its prim, carrying something or not, so an index into
/// `frames` is an index into what comes back. The tube rides its segment
/// rather than the segment prim carrying the mesh itself, because the solver
/// drives that prim and a prim holding a reference can have no children.
pub fn spawn_rod(
    organ: &mut EntityCommands,
    cable: impl Bundle,
    frames: &[Transform],
    tubes: &[Geometry],
) -> Vec<Entity> {
    assert_eq!(frames.len(), tubes.len(), "one tube per segment");
    organ.with_child((Name::new(CABLE), cable));
    let mut segments = Vec::with_capacity(frames.len());
    organ.with_children(|organ| {
        segments.extend(
            frames
                .iter()
                .zip(tubes)
                .enumerate()
                .map(|(index, (frame, tube))| {
                    organ
                        .spawn((
                            Name::new(segment_name(index)),
                            *frame,
                            Visibility::default(),
                        ))
                        .with_child((Name::new(STEM), tube.clone()))
                        .id()
                }),
        );
    });
    segments
}

/// One entry of the mesh library: the geometry built for a single
/// representative, shared by every organ that drew it.
#[derive(Clone, Debug)]
pub struct Part {
    pub mesh: Handle<Mesh>,
    /// Linear RGB. The one channel both consumers read: Bevy draws it, and
    /// USD carries it as `displayColor`, so the viewer and the export agree
    /// on colour by construction.
    pub color: [f32; 3],
    /// Microfacet roughness. 0 is a mirror, 1 is chalk.
    pub roughness: f32,
    /// Specular intensity.
    pub reflectance: f32,
    /// How much light passes through rather than reflecting off — a leaf
    /// blade's stand-in for subsurface scattering.
    pub translucency: f32,
    /// Thickness in meters of the volume behind the surface, which only a
    /// translucent one has.
    pub thickness: f32,
    /// A surface with no inside — a leaf blade — which has to be lit and drawn
    /// from behind as well, since a canopy is looked up into as often as down
    /// onto.
    pub double_sided: bool,
    /// The `physics:approximation` this mesh collides as, for a part that is
    /// its own collider. See [`Library::collide`].
    pub collision: Option<&'static str>,
    /// The grid spacing this mesh is a height field at. See
    /// [`Library::heightfield`].
    pub heightfield_resolution: Option<f32>,
}

impl Part {
    /// The viewer's material for this part.
    ///
    /// Untextured, because the colour is the whole material: nothing here is
    /// metallic, and a texture-free surface is distinguished only by how it
    /// responds to light.
    pub fn material(&self) -> StandardMaterial {
        let [red, green, blue] = self.color;
        let mut material = StandardMaterial {
            base_color: Color::linear_rgb(red, green, blue),
            perceptual_roughness: self.roughness,
            reflectance: self.reflectance,
            // A second, flipped diffuse lobe: what light this surface passes
            // takes the base colour with it, so a backlit leaf reads green
            // rather than black. Energy conserving — Bevy scales the reflected
            // lobe by `1 - translucency` — so it redistributes light rather
            // than adding any.
            diffuse_transmission: self.translucency,
            thickness: self.thickness,
            metallic: 0.0,
            ..default()
        };
        if self.double_sided {
            material.double_sided = true;
            // Back faces are culled by default, which would leave a canopy
            // half missing whenever it is looked up into.
            material.cull_mode = None;
        }
        material
    }
}

/// A stable ordering key, assigned in authoring order.
///
/// Bevy's query iteration order is not stable across runs and a codebook has to
/// be a function of its population alone, so every organ records where it was
/// authored and a layer sorts by this before quantizing.
#[derive(Component, Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct Order(pub u64);

/// Run condition: some organ of this layer was added, re-authored or removed.
/// A removal counts so that a layer left with nothing to build still clears
/// its parts from the library.
pub fn configs_changed<C: Component>(
    configs: Query<(), Changed<C>>,
    mut removed: RemovedComponents<C>,
) -> bool {
    // Read them all: one left unread would count again next frame.
    let removed = removed.read().count() > 0;
    !configs.is_empty() || removed
}

/// An organ standing at `position`, tipped by `tilt` in its own frame and then
/// turned to `yaw` along the row.
///
/// Composed explicitly rather than through `Quat::from_euler`, so the order the
/// rotations apply in is on the page: tilt is a lean in the prototype's own
/// frame, and applying it after the yaw would swing it around with the row.
pub fn placed(position: Vec3, yaw: f32, tilt: Vec2, scale: f32) -> Transform {
    Transform {
        translation: position,
        rotation: Quat::from_rotation_z(yaw)
            * Quat::from_rotation_y(tilt.y)
            * Quat::from_rotation_x(tilt.x),
        scale: Vec3::splat(scale),
    }
}

/// How a part looks — a [`Part`] without its geometry.
#[derive(Clone, Copy, Debug)]
pub struct Surface {
    /// Linear RGB.
    pub color: [f32; 3],
    /// Microfacet roughness. 0 is a mirror, 1 is chalk.
    pub roughness: f32,
    /// Specular intensity.
    pub reflectance: f32,
    /// How much light passes through rather than reflecting off — a leaf
    /// blade's stand-in for subsurface scattering.
    pub translucency: f32,
    /// Thickness in meters of the volume behind the surface.
    pub thickness: f32,
    pub double_sided: bool,
}

/// What an organ needs to draw its geometry and export it as a reference.
///
/// Cloned onto every instance that drew the same representative, so they share
/// one mesh handle and one material — which is what lets Bevy batch them.
#[derive(Bundle, Clone)]
pub struct Geometry {
    pub reference: UsdReference,
    pub mesh: Mesh3d,
    pub material: MeshMaterial3d<StandardMaterial>,
}

/// The mesh library and the asset storage behind it, as one system param.
#[derive(SystemParam)]
pub struct Library<'w> {
    prototypes: ResMut<'w, Prototypes>,
    meshes: ResMut<'w, Assets<Mesh>>,
    materials: ResMut<'w, Assets<StandardMaterial>>,
}

impl Library<'_> {
    /// Drops everything `prefix` registered, ahead of rebuilding it.
    pub fn clear(&mut self, prefix: &str) {
        self.prototypes.clear_layer(prefix);
    }

    /// Registers `mesh` as `<prefix>_<index>` and hands back what an organ
    /// drawing it needs.
    pub fn part(&mut self, prefix: &str, index: usize, mesh: Mesh, surface: Surface) -> Geometry {
        let part = Part {
            mesh: self.meshes.add(mesh),
            color: surface.color,
            roughness: surface.roughness,
            reflectance: surface.reflectance,
            translucency: surface.translucency,
            thickness: surface.thickness,
            double_sided: surface.double_sided,
            collision: None,
            heightfield_resolution: None,
        };
        Geometry {
            mesh: Mesh3d(part.mesh.clone()),
            material: MeshMaterial3d(self.materials.add(part.material())),
            reference: UsdReference(self.prototypes.insert(prefix, index, part)),
        }
    }

    /// Makes a registered part its own collider, at `approximation` — see
    /// [`doc::TRIANGLE_MESH`].
    ///
    /// Worth it only where the mesh *is* the shape to collide with: the
    /// ground. Anything a proxy describes better gets a [`capsule`] instead.
    pub fn collide(&mut self, part: &Geometry, approximation: &'static str) {
        // The name came off a `Geometry` this library handed out, so the entry
        // is there by construction.
        if let Some(entry) = self.prototypes.get_mut(&part.reference.0) {
            entry.collision = Some(approximation);
        }
    }

    /// Records that a registered collider's surface is a height field in xy,
    /// sampled every `resolution` meters — see
    /// [`PartEntry::heightfield_resolution`](doc::PartEntry::heightfield_resolution).
    ///
    /// A consumer fits the nearest whole number of spans of `resolution`
    /// across each axis of the mesh, so its field is the mesh only where both
    /// counts are the mesh's own;
    /// [`Ground::resampling_spacing`](crate::terrain::Ground::resampling_spacing)
    /// is such a spacing for a grid.
    pub fn heightfield(&mut self, part: &Geometry, resolution: f32) {
        if let Some(entry) = self.prototypes.get_mut(&part.reference.0) {
            entry.heightfield_resolution = Some(resolution);
        }
    }
}

/// The mesh library: every representative any layer has built, by name.
///
/// A layer registers its representatives under `<Layer>_<index>`, and
/// downstream layers read them from here — elements compose by typed value, not
/// by prim path.
///
/// A `BTreeMap` rather than a hash map, so iteration is in name order and an
/// exported document is byte-identical across runs without a sort.
#[derive(Resource, Default, Debug)]
pub struct Prototypes {
    parts: BTreeMap<String, Part>,
}

impl Prototypes {
    /// Names the part `<prefix>_<index>` and registers it.
    pub fn insert(&mut self, prefix: &str, index: usize, part: Part) -> String {
        let name = format!("{prefix}_{index}");
        self.parts.insert(name.clone(), part);
        name
    }

    pub fn get(&self, name: &str) -> Option<&Part> {
        self.parts.get(name)
    }

    pub fn get_mut(&mut self, name: &str) -> Option<&mut Part> {
        self.parts.get_mut(name)
    }

    pub fn iter(&self) -> impl Iterator<Item = (&String, &Part)> {
        self.parts.iter()
    }

    pub fn len(&self) -> usize {
        self.parts.len()
    }

    pub fn is_empty(&self) -> bool {
        self.parts.is_empty()
    }

    /// Drops everything one layer registered, ahead of it rebuilding.
    ///
    /// A rebuild may produce fewer representatives than the last one; without
    /// this the leftovers stay in the library, exported and unreferenced.
    pub fn clear_layer(&mut self, prefix: &str) {
        let head = format!("{prefix}_");
        self.parts.retain(|name, _| !name.starts_with(&head));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testing::fixture::Boxes;

    fn part() -> Part {
        Part {
            mesh: Handle::default(),
            color: [0.5, 0.5, 0.5],
            roughness: 0.8,
            reflectance: 0.5,
            translucency: 0.0,
            thickness: 0.0,
            double_sided: false,
            collision: None,
            heightfield_resolution: None,
        }
    }

    #[test]
    fn parts_are_named_by_layer_and_index() {
        let mut prototypes = Prototypes::default();
        assert_eq!(prototypes.insert("Leaf", 2, part()), "Leaf_2");
        assert!(prototypes.get("Leaf_2").is_some());
    }

    /// A rebuild that produces fewer representatives must not leave the extras
    /// behind, or the library grows every time a budget is lowered.
    #[test]
    fn clearing_a_layer_leaves_the_others_alone() {
        let mut prototypes = Prototypes::default();
        for i in 0..4 {
            prototypes.insert("Leaf", i, part());
        }
        prototypes.insert("Plant", 0, part());

        prototypes.clear_layer("Leaf");

        assert_eq!(prototypes.len(), 1);
        assert!(prototypes.get("Plant_0").is_some());
    }

    /// `LeafBlade_0` is not a `Leaf` part, and clearing one layer must not
    /// take the other with it.
    #[test]
    fn clearing_a_layer_matches_whole_names_only() {
        let mut prototypes = Prototypes::default();
        prototypes.insert("Leaf", 0, part());
        prototypes.insert("LeafBlade", 0, part());

        prototypes.clear_layer("Leaf");

        assert_eq!(prototypes.len(), 1);
        assert!(prototypes.get("LeafBlade_0").is_some());
    }

    /// The correction has to actually reach what is spawned under the root, not
    /// just be right in isolation: propagation stops at an entity with no
    /// `Transform`, and the scene below a broken link renders lying on its side.
    #[test]
    fn the_correction_reaches_everything_under_the_root() {
        let mut app = App::new();
        app.add_plugins((MinimalPlugins, TransformPlugin, super::plugin::<Boxes>));
        app.update();

        // A metre up in scene space, which is a metre up in world space too.
        let root = app.world().resource::<PrimRoot>().0;
        let marker = app
            .world_mut()
            .spawn((Transform::from_xyz(0.0, 0.0, 1.0), ChildOf(root)))
            .id();
        app.update();

        let placed = app.world().entity(marker).get::<GlobalTransform>().unwrap();
        assert!(
            (placed.translation() - Vec3::Y).length() < 1e-6,
            "scene +Z lands on world +Y, got {}",
            placed.translation()
        );
    }

    /// A child spawned into an index a despawn freed in the same frame is still
    /// propagated — see [`mark_rebuilt_parents`].
    #[test]
    fn children_spawned_into_freed_indices_are_propagated() {
        let mut app = App::new();
        app.add_plugins((MinimalPlugins, TransformPlugin, super::plugin::<Boxes>));
        let world = app.world_mut();
        // `new`'s children take the indices `old`'s free. `other`'s children are
        // detached rather than despawned: they only lengthen the list of orphans
        // Bevy walks before the new children, which makes the miss certain
        // rather than a race between its threads.
        let [old, new, other] = [Vec3::ZERO, Vec3::X, Vec3::ZERO]
            .map(|at| world.spawn(Transform::from_translation(at)).id());
        for (parent, count) in [(old, 2048), (other, 16384)] {
            for _ in 0..count {
                world.spawn((Transform::IDENTITY, ChildOf(parent)));
            }
        }
        app.update();

        let world = app.world_mut();
        for child in world.entity(old).get::<Children>().unwrap().to_vec() {
            world.despawn(child);
        }
        for child in world.entity(other).get::<Children>().unwrap().to_vec() {
            world.entity_mut(child).remove::<ChildOf>();
        }
        let spawned: Vec<Entity> = (0..1024)
            .map(|_| world.spawn((Transform::IDENTITY, ChildOf(new))).id())
            .collect();
        app.update();

        let stale = spawned
            .iter()
            .filter(|&&child| {
                app.world()
                    .get::<GlobalTransform>(child)
                    .unwrap()
                    .translation()
                    != Vec3::X
            })
            .count();
        assert_eq!(stale, 0, "{stale} new children were never propagated");
    }

    /// The rotation belongs to the viewer, so it must sit above the prim the
    /// export starts from — inside it, every prim in the file would be tipped.
    #[test]
    fn the_correction_sits_above_the_exported_root() {
        let mut app = App::new();
        app.add_plugins(super::plugin::<Boxes>);
        app.update();

        let root = app.world().resource::<PrimRoot>().0;
        let entity = app.world().entity(root);
        assert!(entity.contains::<UsdRoot>());
        assert_eq!(
            entity.get::<Transform>().copied().unwrap_or_default(),
            Transform::IDENTITY,
            "the exported root is untransformed"
        );

        let parent = entity.get::<ChildOf>().expect("the root has a parent").0;
        assert_eq!(
            app.world()
                .entity(parent)
                .get::<Transform>()
                .unwrap()
                .rotation,
            z_up_to_y_up()
        );
    }

    /// The library iterates in name order, which is what makes an exported
    /// document reproducible without sorting it on the way out.
    #[test]
    fn the_library_iterates_in_name_order() {
        let mut prototypes = Prototypes::default();
        for i in [3, 0, 2, 1] {
            prototypes.insert("Plant", i, part());
        }
        let names: Vec<&str> = prototypes.iter().map(|(n, _)| n.as_str()).collect();
        assert_eq!(names, ["Plant_0", "Plant_1", "Plant_2", "Plant_3"]);
    }

    /// A segment's frame sits at its midpoint with +Z along it, a station maps
    /// to the segment it falls in, and a rider rebased into its segment's
    /// frame lands where it was placed.
    #[test]
    fn a_rider_rebased_onto_its_segment_stays_where_it_was_placed() {
        let points = [[0.0; 3], [0.0, 0.0, 0.1], [0.1, 0.0, 0.2], [0.2, 0.0, 0.2]];
        let frames = segment_frames(&points);
        assert_eq!(frames.len(), 3);
        for (frame, pair) in frames.iter().zip(points.windows(2)) {
            let (from, to) = (Vec3::from(pair[0]), Vec3::from(pair[1]));
            assert!(frame.translation.distance((from + to) / 2.0) < 1e-6);
            assert!((frame.rotation * Vec3::Z).distance((to - from).normalize()) < 1e-6);
        }

        let stations = [0.0, 0.1, 0.24, 0.34];
        let segments = [-0.1, 0.05, 0.1, 0.3, 0.34, 0.5].map(|s| segment_of(&stations, s));
        assert_eq!(segments, [0, 0, 1, 2, 2, 2]);

        let placement =
            Transform::from_xyz(0.12, 0.03, 0.21).with_rotation(Quat::from_rotation_x(0.4));
        let hung = frames[2] * rebase(&frames[2], &placement);
        assert!(hung.translation.distance(placement.translation) < 1e-6);
        assert!((hung.rotation * Vec3::Y).distance(placement.rotation * Vec3::Y) < 1e-6);
    }
}

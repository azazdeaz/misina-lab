//! The export document — the whole contract between this crate and the USD
//! builder in `python/misina-lab/misina_lab/usd/build.py`.
//!
//! Rust owns the *scene*: what geometry exists, where it goes, and what
//! references what. Python owns *USD*: prim types, schemas, composition arcs,
//! stage metadata. This module is the line between them. Keep it a plain serde
//! struct — it is the only thing both test suites can assert against, and it
//! has to stay readable when a scene comes out wrong.
//!
//! # Conventions the builder relies on
//!
//! - Coordinates are **Z-up, meters**, already in the convention the export
//!   targets. Nothing downstream rotates anything.
//! - Every mesh is a **triangle list**, so USD's `faceVertexCounts` is
//!   `[3; indices.len() / 3]` and is not transmitted.
//! - Rotations are **quaternions in xyzw order**, authored as
//!   `xformOp:orient`. Do not switch to Euler triples: USD's `rotateXYZ` and
//!   Bevy's `EulerRot` disagree about intrinsic versus extrinsic composition,
//!   and a mismatch gives a scene that is plausibly wrong rather than obviously
//!   wrong.
//! - A [`Node`] with a `reference` draws the [`PartEntry`] of that name and
//!   has **no children** of its own — see [`Node::instanceable`].
//! - A part's and a cable's surface response — `roughness`, `reflectance`,
//!   `translucency` — is authored as a bound material, one per part. A
//!   translucent surface gets a different shader from an opaque one; see
//!   `build.py`.
//! - Colliders are **static**: nothing here is a rigid body, so the builder
//!   authors collision without mass, and the ground may be an exact triangle
//!   mesh, which a dynamic collider could not be. A [`Cable`] is the one thing
//!   that moves, and it is not a collider — a solver builds its own bodies
//!   from the curve.
//! - A [`Node`]'s `attributes` are custom attributes of that prim, authored
//!   in the [`SceneDoc::namespace`]. An attribute's JSON type is its USD type;
//!   see [`AttributeValue`].
//! - A part with [`colors`](PartEntry::colors) is coloured per point, and its
//!   material reads that primvar instead of a constant; see `build.py`.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

/// Bumped when the shape of this document changes incompatibly. The builder
/// refuses anything it does not recognise, so a stale cached scene fails
/// loudly instead of composing into something subtly wrong.
pub const FORMAT: u32 = 11;

/// One generated scene, ready to be turned into a USD stage.
#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct SceneDoc {
    pub format: u32,
    /// Always `"Z"` — the export target is robotics simulation (Isaac Lab and
    /// ROS, both REP-103 right-handed Z-up). Transmitted rather than assumed
    /// so the builder authors stage metadata from the document rather than
    /// from a constant of its own.
    pub up_axis: String,
    pub meters_per_unit: f64,
    /// The generator's Python package, `Generator::PACKAGE`. Every name in a
    /// [`Node::attributes`] is authored under it — `boxlab:` for `boxlab` —
    /// so a generator's attributes cannot collide with a schema's or with
    /// another generator's.
    pub namespace: String,
    /// The mesh library, sorted by name. Becomes `/Scene/parts`.
    pub parts: Vec<PartEntry>,
    /// The scene root, which becomes the stage's default prim.
    pub root: Node,
}

/// One entry of the mesh library: the geometry of a single representative,
/// referenced by every organ that drew it.
#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct PartEntry {
    /// Unique within the document, and the name a [`Node::reference`] uses.
    /// By convention `<Layer>_<representative index>` — `Leaf_2`, `Plant_11`.
    pub name: String,
    pub points: Vec<[f32; 3]>,
    /// Flat triangle list into `points`.
    pub indices: Vec<u32>,
    /// Per-point normals. Absent when the mesh carries none, in which case the
    /// builder leaves `normals` unauthored and a renderer computes its own.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub normals: Option<Vec<[f32; 3]>>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub uvs: Option<Vec<[f32; 2]>>,
    /// Linear RGB. Authored both as a constant-interpolation `displayColor`
    /// and as the bound material's diffuse colour — the first for a consumer
    /// that shades nothing, the second for one that does.
    pub display_color: [f32; 3],
    /// Per-point linear RGB, one per point, with `display_color` already
    /// multiplied in. Present on a part whose colour varies over it; the
    /// builder then authors `displayColor` per vertex and ignores
    /// `display_color`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub colors: Option<Vec<[f32; 3]>>,
    /// Microfacet roughness. 0 is a mirror, 1 is chalk.
    pub roughness: f32,
    /// Specular intensity, on a linear 0..1 scale where 0.5 is the 4% every
    /// ordinary dielectric reflects.
    pub reflectance: f32,
    /// How much light passes through the surface rather than reflecting off
    /// it. Non-zero only on a blade, and what makes the builder reach for a
    /// subsurface-capable material instead of the plain one.
    #[serde(default, skip_serializing_if = "is_zero")]
    pub translucency: f32,
    /// A surface with no inside — a leaf blade — which has to be lit and drawn
    /// from behind as well, since a canopy is looked up into as often as down
    /// onto.
    pub double_sided: bool,
    /// This mesh is also its own collider, at this `physics:approximation` —
    /// [`TRIANGLE_MESH`] for the exact triangles. Absent on a part nothing
    /// collides with.
    ///
    /// Every prim referencing such a part is authored non-instanceable, so the
    /// collider lands on a real prim rather than inside a prototype — see
    /// [`Node::instanceable`].
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub collision: Option<String>,
    /// Horizontal grid spacing, in meters, of a collider whose surface is a
    /// height field in xy — authored as `newton:heightfield:resolution`.
    /// A backend that collides height fields rasterizes the mesh at this
    /// spacing instead of approximating it; MuJoCo, for one, collides every
    /// mesh as its convex hull. Absent on a part that is not one.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub heightfield_resolution: Option<f32>,
}

/// The `physics:approximation` of an exact triangle-mesh collider. USD spells
/// it "no approximation"; it is legal only on a static collider.
pub const TRIANGLE_MESH: &str = "none";

/// One prim.
#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct Node {
    /// The prim name. The path is this joined onto the ancestors' names, so
    /// these have to stay stable across regeneration — a downstream Isaac Lab
    /// config keyed on a path silently starts pointing at a different plant
    /// otherwise.
    pub name: String,
    /// `"Xform"` or `"Scope"`.
    pub type_name: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub xform: Option<Xform>,
    /// The [`PartEntry`] this prim draws, by name. A prim with a reference is
    /// a leaf of the tree.
    ///
    /// One of the three payloads — this, [`collider`](Self::collider),
    /// [`cable`](Self::cable) — at most, and `type_name` says which:
    /// `"Capsule"` carries a collider, `"BasisCurves"` a cable, and a prim
    /// with a reference is left typeless for the part to supply the type.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub reference: Option<String>,
    /// Whether USD may treat this prim as an instance of what it references.
    ///
    /// Set on every referencing prim, which is what makes tens of thousands of
    /// unique leaf paths affordable: the prims stay individually addressable
    /// while the renderer draws one prototype per part. It is safe precisely
    /// because a referencing prim has no children — an instanceable prim's
    /// descendants are not addressable, and these have no descendants to lose.
    ///
    /// Cleared on a prim referencing a part that carries
    /// [`collision`](PartEntry::collision): a collider inside a prototype is
    /// reached only through an instance proxy, and the one part that applies
    /// to has a single instance, so it shares nothing anyway.
    #[serde(default, skip_serializing_if = "is_false")]
    pub instanceable: bool,
    /// A collision proxy standing in for geometry too detailed to collide
    /// with. Present only on prims whose `type_name` is `"Capsule"`, which
    /// carry it instead of a `reference`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub collider: Option<Capsule>,
    /// A deformable curve simulated in place of a mesh. Present only on prims
    /// whose `type_name` is `"BasisCurves"`, which carry it instead of a
    /// `reference`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub cable: Option<Cable>,
    /// Custom attributes of this prim, by name within
    /// [`SceneDoc::namespace`] — ground truth a consumer reads off the stage,
    /// which nothing renders or simulates. Any prim may carry them, an
    /// instanceable one included: they are authored on the prim itself, like
    /// its transform, and not inside the part it references.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub attributes: BTreeMap<String, AttributeValue>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub children: Vec<Node>,
}

/// One custom attribute's value. Serialized as the bare JSON value, so the
/// JSON type is the tag: `true`, `3`, `0.5`, `"edge"`, `[0.0, 0.1, 0.2]`
/// become USD's `bool`, `int`, `float`, `string` and `float3`.
///
/// A whole `Float` keeps its point (`1.0`, not `1`), which is what keeps it
/// from coming back an `Int`.
#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
#[serde(untagged)]
pub enum AttributeValue {
    Bool(bool),
    Int(i32),
    Float(f32),
    Float3([f32; 3]),
    String(String),
}

impl From<bool> for AttributeValue {
    fn from(value: bool) -> Self {
        Self::Bool(value)
    }
}

impl From<i32> for AttributeValue {
    fn from(value: i32) -> Self {
        Self::Int(value)
    }
}

impl From<f32> for AttributeValue {
    fn from(value: f32) -> Self {
        Self::Float(value)
    }
}

impl From<[f32; 3]> for AttributeValue {
    fn from(value: [f32; 3]) -> Self {
        Self::Float3(value)
    }
}

impl From<&str> for AttributeValue {
    fn from(value: &str) -> Self {
        Self::String(value.to_string())
    }
}

/// A prim's transform, as the `translate` / `orient` / `scale` op stack.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq)]
pub struct Xform {
    pub translate: [f32; 3],
    /// Quaternion in **xyzw** order — Bevy's `Quat` layout. USD's `Gf.Quatf`
    /// takes the real part first, so the builder reorders on the way in.
    pub orient: [f32; 4],
    pub scale: [f32; 3],
}

/// A capsule about the prim's own **+Z**, centered on its origin — USD's
/// default axis, so the builder authors none.
///
/// A capsule rather than a cylinder because PhysX has a native capsule and no
/// native cylinder: a `Cylinder` collider is convex-hull-approximated or
/// routed through custom geometry, both dearer than the analytic shape.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq)]
pub struct Capsule {
    pub radius: f32,
    /// The **cylindrical section alone**, as USD measures it: the capsule
    /// reaches `height / 2 + radius` either side of its origin.
    pub height: f32,
}

/// A flexible organ, simulated as a deformable curve rather than drawn as a
/// mesh.
///
/// A backend that reads one builds a chain of capsules joined by spring
/// joints, holding the first of them where it was authored; a backend that
/// does not is left with an inert curve, which is what every non-VBD solver
/// sees.
#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub struct Cable {
    /// Centerline control points in the prim's own frame, **held end first**:
    /// the segment between the first two is the one bolted down, and does not
    /// move. `n` points become `n - 1` segments, so at least three.
    ///
    /// Evenly spaced past the held segment: a point's mass is split between
    /// the two segments it joins, which weighs each segment right only where
    /// its neighbours match it in length.
    pub points: Vec<[f32; 3]>,
    /// The diameter at each point, in meters: what the curve is drawn at and
    /// the rod is simulated at. Each segment's capsule is the mean of its two
    /// ends, and each joint's bending stiffness goes with the fourth power of
    /// the diameter at its point.
    pub widths: Vec<f32>,
    /// Each point's normal, in the prim's own frame: the +Y of the segment
    /// leaving it in [`segment_frames`](super::segment_frames), and the last
    /// point the last segment's. A rod's importer turns each segment's body
    /// about the segment to put its +Y there, so the body's frame is its
    /// prim's, and whatever rides the prim stays where it was put.
    pub normals: Vec<[f32; 3]>,
    /// The mass hanging from each point, in kilograms, on top of the rod's
    /// own: what rides the organ without being simulated, such as its fruit.
    /// A point's load is split between the two segments it joins, as its share
    /// of the rod's own mass is, and one on the held segment's two points
    /// weighs on whatever holds it. Empty for a rod that carries nothing.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub loads: Vec<f32>,
    /// Linear RGB — the same channel a [`PartEntry`] carries, since a curve is
    /// drawn beside the meshes and has to agree with them.
    pub display_color: [f32; 3],
    /// Microfacet roughness. See [`PartEntry::roughness`].
    pub roughness: f32,
    /// Specular intensity. See [`PartEntry::reflectance`].
    pub reflectance: f32,
}

fn is_false(b: &bool) -> bool {
    !*b
}

fn is_zero(f: &f32) -> bool {
    *f == 0.0
}

impl Node {
    /// A structural prim: a name, a type, and whatever hangs below it.
    pub fn group(name: impl Into<String>, type_name: &str) -> Self {
        Self {
            name: name.into(),
            type_name: type_name.to_string(),
            xform: None,
            reference: None,
            instanceable: false,
            collider: None,
            cable: None,
            attributes: BTreeMap::new(),
            children: Vec::new(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// One of each type, the float whole so it has to keep its point.
    fn attributes() -> BTreeMap<String, AttributeValue> {
        BTreeMap::from([
            ("flag".into(), true.into()),
            ("count".into(), 3.into()),
            ("share".into(), 1.0.into()),
            ("offset".into(), [0.0, 0.1, 0.2].into()),
            ("label".into(), "edge".into()),
        ])
    }

    fn doc() -> SceneDoc {
        let mut leaf = Node::group("Leaf_00", "Xform");
        leaf.reference = Some("Leaf_2".into());
        leaf.instanceable = true;
        leaf.attributes = attributes();
        leaf.xform = Some(Xform {
            translate: [0.1, 0.2, 0.3],
            orient: [0.0, 0.0, 0.0, 1.0],
            scale: [1.0, 1.0, 1.0],
        });

        let mut collider = Node::group("Collision", "Capsule");
        collider.collider = Some(Capsule {
            radius: 0.04,
            height: 1.72,
        });

        let mut rod = Node::group("Cable", "BasisCurves");
        rod.cable = Some(Cable {
            points: vec![[0.0; 3], [0.0, 0.0, 0.1], [0.0, 0.0, 0.2]],
            widths: vec![0.011, 0.010, 0.009],
            normals: vec![[0.0, 1.0, 0.0]; 3],
            loads: vec![0.0, 0.0, 0.05],
            display_color: [0.2, 0.5, 0.1],
            roughness: 0.85,
            reflectance: 0.25,
        });

        let mut root = Node::group("Scene", "Xform");
        root.children.push(leaf);
        root.children.push(collider);
        root.children.push(rod);

        SceneDoc {
            format: FORMAT,
            up_axis: "Z".into(),
            meters_per_unit: 1.0,
            namespace: "boxlab".into(),
            parts: vec![PartEntry {
                name: "Leaf_2".into(),
                points: vec![[0.0; 3], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
                indices: vec![0, 1, 2],
                normals: None,
                uvs: None,
                display_color: [0.2, 0.5, 0.1],
                colors: None,
                roughness: 0.5,
                reflectance: 0.55,
                translucency: 0.45,
                double_sided: true,
                collision: None,
                heightfield_resolution: None,
            }],
            root,
        }
    }

    #[test]
    fn a_document_round_trips_through_json() {
        let json = serde_json::to_string(&doc()).unwrap();
        let back: SceneDoc = serde_json::from_str(&json).unwrap();

        assert_eq!(back.format, FORMAT);
        assert_eq!(back.up_axis, "Z");
        assert_eq!(back.parts[0].name, "Leaf_2");
        assert_eq!(back.namespace, "boxlab");
        assert_eq!(back.root.children[0].reference.as_deref(), Some("Leaf_2"));
        assert!(back.root.children[0].instanceable);
        assert_eq!(back.root.children[0].attributes, attributes());
        assert_eq!(
            back.root.children[1].collider,
            Some(Capsule {
                radius: 0.04,
                height: 1.72
            })
        );
        assert_eq!(back.root.children[2].cable, doc().root.children[2].cable);
    }

    /// At tens of thousands of prims the defaults are most of the bytes, so
    /// the empty ones have to stay out of the file.
    #[test]
    fn defaulted_fields_are_left_out_of_the_json() {
        let json = serde_json::to_string(&doc()).unwrap();

        // The cable's, which it always carries; the part has none.
        assert_eq!(json.matches("\"normals\"").count(), 1, "got:\n{json}");
        assert!(!json.contains("\"uvs\""), "got:\n{json}");
        assert!(!json.contains("\"colors\""), "got:\n{json}");
        assert!(!json.contains("\"collision\""), "got:\n{json}");
        assert_eq!(json.matches("\"collider\"").count(), 1, "got:\n{json}");
        assert_eq!(json.matches("\"cable\"").count(), 1, "got:\n{json}");
        // The root has no transform, no reference and is not instanceable;
        // the leaf has all three, so each key appears exactly once.
        assert_eq!(json.matches("\"xform\"").count(), 1, "got:\n{json}");
        assert_eq!(json.matches("\"instanceable\"").count(), 1, "got:\n{json}");
        assert_eq!(json.matches("\"children\"").count(), 1, "got:\n{json}");
        assert_eq!(json.matches("\"attributes\"").count(), 1, "got:\n{json}");
    }

    /// The JSON type is the only tag, so each value has to come out as the
    /// JSON a reader types it by.
    #[test]
    fn an_attribute_is_its_bare_json_value() {
        let json = serde_json::to_string(&attributes()).unwrap();
        assert_eq!(
            json,
            r#"{"count":3,"flag":true,"label":"edge","offset":[0.0,0.1,0.2],"share":1.0}"#
        );
    }
}

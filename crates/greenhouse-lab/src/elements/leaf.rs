//! Leaf — a compound tomato leaf: a petiole off the stem, a rachis that
//! arches out and droops under its leaflets, and the leaflets.
//!
//! A tomato leaf is pinnately compound. The rachis ends in a terminal
//! leaflet and carries three or four pairs of near-opposite laterals, the
//! largest nearest the tip, with small intercalary leaflets in the gaps; a
//! lateral may carry a pair of secondary leaflets on its stalk. Every leaflet
//! is a [`Leaflet`] outline, folded along its midrib, angled off the rachis
//! toward the tip and hung a little below it:
//!
//! ```text
//!              lateral pairs        an intercalary leaflet
//!            ╲   ╱      ╲   ╱      ╲ ╱  ╱
//!   ──────────┴──────────┴──────────┴──·───▶   the terminal leaflet
//!   petiole  ╱   ╲      ╱   ╲      ╱ ╲
//!   ↑
//!   the stem's axis
//! ```
//!
//! A leaf is built in its own frame, on the stem's axis at its node, facing
//! +X with +Z up. The petiole leaves `angle` off vertical and the rachis
//! turns `droop` further down by the tip, the turn gathering toward the tip,
//! where the rachis is thinnest.
//!
//! A stem hangs a leaf at each of its top nodes, `rank` counting down from
//! the head: small and upright there, full grown a couple of trusses down,
//! and drooping more the older it is. The leaves below have been stripped, so
//! the trusses ripening there hang on bare stem.
//!
//! Rachis and leaflets are one mesh, quantized like any organ to a budget of
//! `variations`; which leaflets a leaf carries and how each one is cut are
//! the representative's. Leaves are visual only: they have no collider.
//!
//! ```text
//! Stem_0
//!   Leaf_00 …         -> parts/Leaf_k, the lowest still on the stem first
//! ```

use bevy::prelude::*;
use misina_lab::geometry::mesh::{MeshData, merge_meshes};
use misina_lab::geometry::par_map;
use misina_lab::geometry::shapes::bent;
use misina_lab::geometry::strand::{Bark, Strand, strand_mesh};
use misina_lab::palette::{COLOR_STREAM, Response, shade, srgb};
use misina_lab::params::{Label, Slider};
use misina_lab::quantize::{Metric, farthest_first};
use misina_lab::rng::{Rng, salt};
use misina_lab::scene::{Library, Order, configs_changed};
use nalgebra::Point3;

use super::leaflet::Leaflet;
use super::{Grow, SceneParams, smooth};

/// The mesh-library prefix the leaves register under.
pub const PART: &str = "Leaf";

/// Salt splitting a leaf's mesh draws off the stream its config draws from.
const LEAFLET_STREAM: u64 = 0x2545_F491_4F6C_DD1D;

/// Nodes below the head a leaf takes to grow to full size, and to settle
/// from upright to its full angle and droop.
const GROWING: f32 = 8.0;
const SETTLING: f32 = 12.0;

/// The leaf at the head, as a fraction of a full-grown one's length and of
/// an old one's droop.
const YOUNG_LENGTH: f32 = 0.2;
const YOUNG_DROOP: f32 = 0.2;

/// The petiole's angle off the stem, in degrees, at the head and on an old
/// leaf.
const ANGLE: (f32, f32) = (30.0, 65.0);

/// How far a leaf's length, angle and droop vary about what its node calls
/// for, as fractions.
const LENGTH_JITTER: f64 = 0.1;
const ANGLE_JITTER: f64 = 0.15;
const DROOP_JITTER: f64 = 0.25;

/// The fewest and the most lateral pairs a leaf has, besides its terminal
/// leaflet.
const PAIRS: (u32, u32) = (3, 4);

/// Lengths along a leaf, as fractions of it: the petiole, from the stem to
/// the first pair; the terminal leaflet; a lateral of the top pair; and an
/// intercalary leaflet.
const PETIOLE: f32 = 0.16;
const TERMINAL: f32 = 0.28;
const LATERAL: f32 = 0.36;
const INTERCALARY: f32 = 0.08;

/// A basal lateral, as a fraction of a top one: the laterals grow toward the
/// tip.
const BASAL: f32 = 0.6;

/// How far the petiole and a leaflet vary in length, as fractions.
const SIZE_JITTER: f64 = 0.15;

/// A lateral's angle off the rachis toward the tip, in degrees, on the basal
/// pair and on the top one; an intercalary leaflet's; and how far either
/// strays.
const SPREAD: (f32, f32) = (75.0, 50.0);
const INTERCALARY_SPREAD: f32 = 80.0;
const SPREAD_JITTER: f64 = 10.0;

/// How far apart along the rachis a pair's two laterals sit, at most, as a
/// fraction of the gap between two pairs.
const OPPOSITE: f64 = 0.2;

/// How far a leaflet hangs below the leaf's plane, in degrees, and droops
/// toward its own tip, in radians.
const HANG: (f64, f64) = (5.0, 25.0);
const DROOP: (f64, f64) = (0.2, 0.6);

/// How far a leaflet folds up along its midrib: the angle its margin turns
/// through, in radians.
const FOLD: f64 = 0.35;

/// The chance of an intercalary leaflet on each side of each gap, and of a
/// pair of secondary leaflets on a lateral.
const INTERCALARY_CHANCE: f64 = 0.8;
const SECONDARY_CHANCE: f64 = 0.5;

/// A lateral with secondary leaflets: its stalk, longer than a plain one's,
/// where along the stalk the pair sits, and how long each of them is, all as
/// fractions of the lateral's length; and their angle off it, in degrees.
const SECONDARY_STALK: f64 = 0.3;
const SECONDARY_AT: f32 = 0.15;
const SECONDARY: f32 = 0.3;
const SECONDARY_SPREAD: f32 = 60.0;

/// The petiole's radius where it leaves the stem, as a fraction of the leaf's
/// length, and the rachis's at the terminal leaflet, as a fraction of that.
const RADIUS: f32 = 0.008;
const TAPER: f32 = 0.3;

/// The power the rachis's turn grows by along it: over 1 keeps the petiole
/// straight and bends the far end most.
const CURVE: f32 = 1.8;

/// Steps along the midline, vertices round the rachis, and rings per meter
/// along it at a `detail` of 1.
const STATIONS: usize = 24;
const SIDES: usize = 5;
const RINGS: f64 = 40.0;

/// How far a leaflet's margin may stray from its curves at a `detail` of 1,
/// in meters: about a pixel to a camera half a meter away.
pub const TOLERANCE: f64 = 0.0006;

/// Floors on what Python may set, so that every leaf still builds.
const MIN_LENGTH: f32 = 0.02;
const MIN_DETAIL: f32 = 0.01;

/// A tomato leaf's green, matte from its hairs.
const COLOR: u32 = 0x41672A;
const RESPONSE: Response = Response {
    roughness: 0.65,
    reflectance: 0.4,
};

/// One leaf's shape, as the stem that grew it specified.
#[derive(Component, Clone, Copy, Debug, PartialEq)]
pub struct LeafConfig {
    /// From the stem's axis to the tip of the terminal leaflet, in meters.
    pub length: f32,
    /// Between the stem and the petiole where it leaves, in radians.
    pub angle: f32,
    /// How far the rachis turns down from the stem to its tip, in radians.
    pub droop: f32,
    /// Pairs of lateral leaflets.
    pub pairs: u32,
    /// The stream its own draws and its leaflets' come from.
    pub seed: u64,
    /// Nodes between it and its stem's head, 0 for the youngest. Kept, with
    /// `seed`, so the params can be re-applied to a leaf already hanging;
    /// [`LeafMetric`] reads neither.
    pub rank: u32,
}

impl LeafConfig {
    /// The leaf these params call for `rank` nodes below the head, its own
    /// variation drawn off `seed`.
    ///
    /// The *draw order* is part of this element's output: the length, the
    /// angle and the droop, then the lateral pairs.
    pub fn new(params: &LeafParams, seed: u64, rank: u32) -> Self {
        let mut rng = Rng::new(seed);
        let mut about = |jitter: f64| rng.range(1.0 - jitter, 1.0 + jitter) as f32;
        let grown = smooth(rank as f32 / GROWING);
        let settled = smooth(rank as f32 / SETTLING);
        let length =
            params.length * (YOUNG_LENGTH + (1.0 - YOUNG_LENGTH) * grown) * about(LENGTH_JITTER);
        let angle = (ANGLE.0 + (ANGLE.1 - ANGLE.0) * settled).to_radians() * about(ANGLE_JITTER);
        let droop = params.droop.to_radians()
            * (YOUNG_DROOP + (1.0 - YOUNG_DROOP) * settled)
            * about(DROOP_JITTER);
        let pairs = PAIRS.0 + (rng.unit() * (PAIRS.1 - PAIRS.0 + 1) as f64) as u32;
        Self {
            length: length.max(MIN_LENGTH),
            angle,
            droop: droop.max(0.0),
            pairs,
            seed,
            rank,
        }
    }
}

/// Two leaves share a mesh when they are close in every dimension that
/// shows, weighted as roughly how far apart each moves the leaf's tip.
pub struct LeafMetric;

impl Metric<LeafConfig> for LeafMetric {
    fn distance(&self, a: &LeafConfig, b: &LeafConfig) -> f32 {
        let reach = a.length.max(b.length);
        [
            a.length - b.length,
            (a.angle - b.angle) * reach,
            // The droop bends the far half of the leaf.
            (a.droop - b.droop) * reach / 2.0,
            // A pair more is two leaflets more, each a third of the leaf.
            (a.pairs as f32 - b.pairs as f32) * reach / 3.0,
        ]
        .iter()
        .map(|d| d * d)
        .sum::<f32>()
        .sqrt()
    }
}

/// The leaves: a compound leaf at each of a stem's top nodes, as many as the
/// stem keeps.
///
/// A leaf is a petiole and a rachis with a terminal leaflet, three or four
/// pairs of laterals and small leaflets between the pairs. The leaves near
/// the head are still growing, small and upright; a couple of trusses down
/// they are full grown, and the older they are the more they droop. Every
/// leaf draws its own size, angle, droop and leaflet count about these.
/// Leaves are visual only, with no collider.
#[derive(Resource, Reflect, Clone, Debug, PartialEq)]
#[cfg_attr(
    feature = "python",
    pyo3::pyclass(get_all, set_all, skip_from_py_object)
)]
pub struct LeafParams {
    /// Length of a full-grown leaf, from the stem to the tip of its terminal
    /// leaflet, in meters. Each leaf draws a little more or less.
    #[reflect(@Slider { min: 0.3, max: 0.6, step: 0.01 })]
    pub length: f32,
    /// How far an old leaf's rachis bends down from where it leaves the stem
    /// to its tip, in degrees. The young leaves near the head droop less.
    #[reflect(@Slider { min: 0.0, max: 150.0, step: 1.0 })]
    pub droop: f32,
    /// How many distinct leaf meshes the scene may hold. A budget, not a
    /// count: the leaves are clustered and this is how many representatives
    /// the clustering may keep.
    #[reflect(@Slider { min: 1.0, max: 128.0, step: 1.0 }, @Label("Leaf variations"))]
    pub variations: u32,
    /// How finely a leaf is meshed, as a multiplier: at 1 a leaflet's margin
    /// keeps within 0.6 mm of its curves, about a pixel to a camera half a
    /// meter away, and a full-grown leaf comes to about 1500 triangles; about
    /// 500 at 0.1 and 4500 at 4. Coarser drops the finest teeth first, then
    /// rounds off the sinuses, and keeps the lobes.
    #[reflect(@Slider { min: 0.1, max: 4.0, step: 0.05 }, @Label("Leaf detail"))]
    pub detail: f32,
}

impl Default for LeafParams {
    fn default() -> Self {
        Self {
            length: 0.45,
            droop: 90.0,
            variations: 32,
            detail: 1.0,
        }
    }
}

pub fn plugin(app: &mut App) {
    // `SceneParams` is not gated on although the colours read the seed:
    // `stem::author` gates on it and hangs fresh leaves when it moves.
    app.init_resource::<LeafParams>().add_systems(
        PreUpdate,
        (
            reauthor.run_if(resource_changed::<LeafParams>),
            build.run_if(
                configs_changed::<LeafConfig>
                    // `variations` and `detail` reach no config.
                    .or_eager(resource_changed::<LeafParams>),
            ),
        )
            .chain()
            .in_set(Grow::Leaves),
    );
}

/// Re-applies the params to every leaf already hanging, in place, from the
/// same draws.
fn reauthor(params: Res<LeafParams>, mut leaves: Query<&mut LeafConfig>) {
    for mut config in &mut leaves {
        let next = LeafConfig::new(&params, config.seed, config.rank);
        config.set_if_neq(next);
    }
}

// ─── The mesh ───────────────────────────────────────────────────────

/// The leaf's midline, from the stem's axis to the tip of the terminal
/// leaflet, in even steps.
struct Midline {
    points: Vec<Vec3>,
    length: f32,
    angle: f32,
    droop: f32,
}

impl Midline {
    fn new(config: &LeafConfig) -> Self {
        let mut midline = Self {
            points: vec![Vec3::ZERO],
            length: config.length,
            angle: config.angle,
            droop: config.droop,
        };
        let step = config.length / STATIONS as f32;
        for i in 0..STATIONS {
            let next = midline.points[i] + midline.along((i as f32 + 0.5) * step) * step;
            midline.points.push(next);
        }
        midline
    }

    /// The direction `s` along: `angle` off vertical at the stem, turning
    /// `droop` further down by the tip.
    fn along(&self, s: f32) -> Vec3 {
        let t = (s / self.length).clamp(0.0, 1.0);
        let heading = self.angle + self.droop * t.powf(CURVE);
        Vec3::new(heading.sin(), 0.0, heading.cos())
    }

    /// The point `s` along.
    fn at(&self, s: f32) -> Vec3 {
        let x = (s / self.length * STATIONS as f32).clamp(0.0, STATIONS as f32);
        let i = (x as usize).min(STATIONS - 1);
        self.points[i].lerp(self.points[i + 1], x - i as f32)
    }
}

/// The frame a leaflet hangs in at `at` off a midrib running `along`, in the
/// XZ plane: turned `spread` toward `side` (1 for +Y, -1 for -Y) and hung
/// `hang` below the midrib's plane; +X along the leaflet, +Z off its upper
/// face.
fn off_midrib(at: Vec3, along: Vec3, side: f32, spread: f32, hang: f32) -> Transform {
    let up = along.cross(Vec3::Y);
    let out = along * spread.cos() + Vec3::Y * side * spread.sin();
    let x = out * hang.cos() - up * hang.sin();
    let z = up * hang.cos() + out * hang.sin();
    Transform::from_translation(at).with_rotation(Quat::from_mat3(&Mat3::from_cols(
        x,
        z.cross(x),
        z,
    )))
}

/// A leaflet of `shape`, `length` long, folded along its midrib and drooping
/// `droop` radians by its tip, in its own frame. Its margin keeps within
/// `tolerance` of its curves.
fn sheet(
    shape: &Leaflet,
    length: f32,
    droop: f64,
    tolerance: f64,
    rng: &mut Rng,
) -> Result<MeshData> {
    let outline = shape.outline(length as f64, tolerance, rng);
    let points = outline.points.len() as f64;
    let keel = FOLD / (shape.width * length as f64);
    Ok(bent(outline, droop, keel, points)?)
}

/// One leaf in its own frame: the rachis, and every leaflet hung off it,
/// meshed as finely as [`LeafParams::detail`] says.
///
/// Draws, off the leaf's leaflet stream: the petiole's length; per pair how
/// far apart its laterals sit, and per lateral, left before right, its size,
/// spread, hang and droop, whether it carries secondary leaflets, and its
/// outline and theirs; then per gap, left before right, whether it holds an
/// intercalary leaflet and, if so, its place, size, spread, hang, droop and
/// outline; and last the terminal leaflet's outline.
fn leaf_mesh(config: &LeafConfig, detail: f32) -> Result<MeshData> {
    let detail = detail.max(MIN_DETAIL) as f64;
    let tolerance = TOLERANCE / detail;
    let mut rng = Rng::new(config.seed ^ LEAFLET_STREAM);
    let midline = Midline::new(config);
    let length = config.length;
    let jitter = |rng: &mut Rng| rng.range(1.0 - SIZE_JITTER, 1.0 + SIZE_JITTER) as f32;
    let petiole = PETIOLE * length * jitter(&mut rng);
    let top = length * (1.0 - TERMINAL);
    let pairs = config.pairs.max(1);
    let gap = (top - petiole) / pairs as f32;

    let mut sheets = Vec::new();
    for j in 0..pairs {
        let f = j as f32 / (pairs - 1).max(1) as f32;
        let apart = rng.range(-OPPOSITE, OPPOSITE) as f32 * gap / 2.0;
        for side in [1.0, -1.0] {
            let size = LATERAL * length * (BASAL + (1.0 - BASAL) * f) * jitter(&mut rng);
            let spread = SPREAD.0 + (SPREAD.1 - SPREAD.0) * f;
            let spread = (spread + rng.range(-SPREAD_JITTER, SPREAD_JITTER) as f32).to_radians();
            let hang = rng.range(HANG.0, HANG.1).to_radians() as f32;
            let droop = rng.range(DROOP.0, DROOP.1);
            let secondary = rng.unit() < SECONDARY_CHANCE;
            // Lopsided toward the leaf's base, on either side.
            let shape = Leaflet {
                stalk: if secondary {
                    SECONDARY_STALK
                } else {
                    Leaflet::LATERAL.stalk
                },
                lopsided: side as f64 * Leaflet::LATERAL.lopsided,
                ..Leaflet::LATERAL
            };
            let s = petiole + gap * j as f32 + side * apart;
            let frame = off_midrib(midline.at(s), midline.along(s), side, spread, hang);
            sheets.push(sheet(&shape, size, droop, tolerance, &mut rng)?.transformed(&frame));
            if secondary {
                for side in [1.0, -1.0] {
                    let on = off_midrib(
                        Vec3::X * SECONDARY_AT * size,
                        Vec3::X,
                        side,
                        SECONDARY_SPREAD.to_radians(),
                        hang,
                    );
                    let leaflet = sheet(
                        &Leaflet::INTERCALARY,
                        SECONDARY * size,
                        droop,
                        tolerance,
                        &mut rng,
                    )?;
                    sheets.push(leaflet.transformed(&(frame * on)));
                }
            }
        }
    }

    // In the gaps above each pair, the last one under the terminal leaflet.
    for j in 0..pairs {
        for side in [1.0, -1.0] {
            if rng.unit() >= INTERCALARY_CHANCE {
                continue;
            }
            let s = petiole + gap * (j as f32 + rng.range(0.3, 0.7) as f32);
            let size = INTERCALARY * length * jitter(&mut rng);
            let spread = INTERCALARY_SPREAD + rng.range(-SPREAD_JITTER, SPREAD_JITTER) as f32;
            let hang = rng.range(HANG.0, HANG.1).to_radians() as f32;
            let droop = rng.range(DROOP.0, DROOP.1);
            let frame = off_midrib(
                midline.at(s),
                midline.along(s),
                side,
                spread.to_radians(),
                hang,
            );
            let leaflet = sheet(&Leaflet::INTERCALARY, size, droop, tolerance, &mut rng)?;
            sheets.push(leaflet.transformed(&frame));
        }
    }

    // Carrying on the midline's turn to the tip.
    let (along, tip) = (midline.along(top), midline.along(length));
    let droop = along.angle_between(tip) as f64;
    let frame = off_midrib(midline.at(top), along, 0.0, 0.0, 0.0);
    let terminal = sheet(&Leaflet::TERMINAL, length - top, droop, tolerance, &mut rng)?;
    sheets.push(terminal.transformed(&frame));

    // The rachis, from the stem's axis into the terminal leaflet's stalk.
    let reach = top + (length - top) * Leaflet::TERMINAL.stalk as f32;
    let n = (reach / length * STATIONS as f32).ceil() as usize;
    let (points, radii) = (0..=n)
        .map(|i| {
            let f = i as f32 / n as f32;
            let p = midline.at(reach * f);
            let r = RADIUS * length * (1.0 - (1.0 - TAPER) * f);
            (Point3::new(p.x as f64, p.y as f64, p.z as f64), r as f64)
        })
        .unzip();
    sheets.push(strand_mesh(&Strand::new(
        points,
        radii,
        SIDES,
        RINGS * detail,
        Bark::none(),
    ))?);
    Ok(merge_meshes(&sheets))
}

/// Builds one mesh per distinct leaf and gives it to every leaf that drew it.
fn build(
    mut commands: Commands,
    mut library: Library,
    scene: Res<SceneParams>,
    params: Res<LeafParams>,
    leaves: Query<(Entity, &Order, &LeafConfig)>,
) -> Result<()> {
    library.clear(PART);

    let mut hung: Vec<(Order, Entity, LeafConfig)> = leaves
        .iter()
        .map(|(entity, order, config)| (*order, entity, *config))
        .collect();
    hung.sort_by_key(|(order, ..)| *order);

    let configs: Vec<LeafConfig> = hung.iter().map(|(_, _, config)| *config).collect();
    let book = farthest_first(
        &configs,
        params.variations.max(1) as usize,
        0.0,
        &LeafMetric,
    );
    let meshes = par_map(&book.representatives, |_, config| {
        leaf_mesh(config, params.detail)
    });
    let mut geometry = Vec::with_capacity(meshes.len());
    for (index, mesh) in meshes.into_iter().enumerate() {
        let color = shade(
            srgb(COLOR),
            &mut Rng::new(scene.seed ^ COLOR_STREAM ^ salt(index as u64)),
        );
        geometry.push(library.part(PART, index, mesh?.to_mesh(), RESPONSE.blade(color)));
    }

    // A leaf has nothing hanging off it, so it is the geometry prim itself.
    for ((_, entity, _), drew) in hung.iter().zip(&book.assignment) {
        commands
            .entity(*entity)
            .insert(geometry[*drew as usize].clone());
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every leaf the defaults call for builds at the coarsest, the default
    /// and the finest detail the panel offers, hangs from the stem's axis and
    /// reaches about its length; it costs fewer triangles the coarser it is,
    /// and stays inside a budget at the default; and a young leaf at the head
    /// is shorter, more upright and less drooping than an old one, on average
    /// over the seeds.
    #[test]
    fn a_leaf_hangs_from_its_node_and_settles_as_it_ages() {
        let params = LeafParams::default();
        let details = [0.1, params.detail, 4.0];
        let mut triangles = [0; 3];
        let (mut young, mut old) = ([0.0; 3], [0.0; 3]);
        for seed in 0..24 {
            for rank in [0, 4, 8, 20] {
                let config = LeafConfig::new(&params, seed, rank);
                for (sum, at) in [(&mut young, 0), (&mut old, 20)] {
                    if rank == at {
                        sum[0] += config.length;
                        sum[1] += config.angle;
                        sum[2] += config.droop;
                    }
                }
                for (k, detail) in details.into_iter().enumerate() {
                    let mesh = leaf_mesh(&config, detail).unwrap();
                    let (near, far) =
                        mesh.points
                            .iter()
                            .fold((f32::MAX, 0.0f32), |(near, far), p| {
                                let d = Vec3::from(*p).length();
                                (near.min(d), far.max(d))
                            });
                    let at = format!("seed {seed}, rank {rank}, detail {detail}");
                    assert!(
                        near <= RADIUS * config.length + 1e-5,
                        "{at}: {near} m off the axis"
                    );
                    assert!(
                        (0.75..=1.0).contains(&(far / config.length)),
                        "{at}: reaches {far} m of {}",
                        config.length
                    );
                    let count = mesh.to_mesh().indices().unwrap().len() / 3;
                    if detail == params.detail {
                        assert!(count < 2500, "{at}: {count} triangles");
                    }
                    triangles[k] += count;
                }
            }
        }
        assert!(
            triangles[0] < triangles[1] && triangles[1] < triangles[2],
            "triangles at {details:?}: {triangles:?}"
        );
        assert!(
            (0..3).all(|i| young[i] < old[i]),
            "length, angle and droop: young {young:?}, old {old:?}"
        );
    }
}

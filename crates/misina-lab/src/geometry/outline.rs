//! A closed ring on the XY plane, and the flat mesh filled into it.
//!
//! A flat organ, a leaf blade or a petal, is a ring of points, and this
//! module knows nothing else about it: not botany, and not what built the
//! ring. [`Outline`] is the ring, with its area and a rescale to a given
//! area; [`outline_mesh`] fills it with triangles, and [`creased_mesh`] does
//! so with lines across it kept as edges, for a later bend to crease on.
//!
//! # Size
//!
//! [`Outline::with_area`] rescales a ring about the origin to enclose the area
//! asked for, so rings built at any scale still come out as a set of
//! consistently sized things.

use anyhow::{Context, Result, bail};
use spade::{
    AngleLimit, ConstrainedDelaunayTriangulation, Point2, RefinementParameters, Triangulation,
};

use super::mesh::MeshData;

/// Angle-based refinement is **off**, and area is left to drive the fill
/// alone.
///
/// Ruppert's algorithm chases a minimum inner angle by inserting points
/// wherever it finds a sharper one, and a toothed outline is sharp corners
/// almost everywhere — every tooth along the margin is one. Chasing them
/// costs a great deal and gets nowhere, because a corner *on a constraint
/// edge* cannot be opened up at all: at spade's 30° default a leaf comes out
/// four times the triangles of the same leaf at 0°, all of them crowded into
/// the margin, and `max_triangle_area` stops making any visible difference.
///
/// Turning it off leaves the only knob that matters — how finely the *inside*
/// is divided, which is the part that has no vertices of its own and the part
/// a later deformation pass has to bend. The slivers this leaves at the teeth
/// are the shape of the teeth.
const ANGLE_LIMIT_DEG: f64 = 0.0;

/// A closed ring of points on the XY plane.
///
/// The first point is not repeated at the end; the ring closes implicitly.
/// Winding is whatever the ring was built with — [`outline_mesh`] does not depend on
/// it, and neither should anything else.
#[derive(Clone, Debug)]
pub struct Outline {
    pub points: Vec<Point2<f64>>,
}

impl Outline {
    /// The area the ring encloses, by the shoelace formula.
    pub fn area(&self) -> f64 {
        ring_area(&self.points).abs()
    }

    /// The same shape scaled about the origin to enclose `area`.
    ///
    /// Area rather than length or width, because it is what makes a set of
    /// differently-proportioned outlines read as the same size: a long narrow
    /// one and a broad round one matched by length look nothing alike, and
    /// matched by area look like two of the same thing.
    pub fn with_area(mut self, area: f64) -> Self {
        let scale = (area / self.area()).sqrt();
        for p in &mut self.points {
            *p = Point2::new(p.x * scale, p.y * scale);
        }
        self
    }
}

/// Twice-signed area, halved: positive counter-clockwise, negative clockwise.
fn ring_area(ring: &[Point2<f64>]) -> f64 {
    let n = ring.len();
    (0..n)
        .map(|i| {
            let (a, b) = (ring[i], ring[(i + 1) % n]);
            a.x * b.y - b.x * a.y
        })
        .sum::<f64>()
        / 2.0
}

/// Whether `point` is inside the ring, by how many of its edges a ray from
/// it along +X crosses.
fn encloses(ring: &[Point2<f64>], point: Point2<f64>) -> bool {
    let n = ring.len();
    (0..n)
        .filter(|&i| {
            let (a, b) = (ring[i], ring[(i + 1) % n]);
            (a.y > point.y) != (b.y > point.y)
                && point.x < a.x + (point.y - a.y) * (b.x - a.x) / (b.y - a.y)
        })
        .count()
        % 2
        == 1
}

// ─── Filling ────────────────────────────────────────────────────────

type Cdt = ConstrainedDelaunayTriangulation<Point2<f64>>;

/// Fills an outline with triangles, flat on the XY plane at z = 0.
///
/// The ring's own points become constraint edges, so every tooth and notch
/// survives exactly as given; `max_triangle_area` then controls how finely
/// the inside is subdivided beyond that. The interior points this inserts are
/// not decoration — they are what a later pass has to work with when it bends
/// the flat shape into something that isn't flat.
///
/// Faces come out counter-clockwise seen from +Z, which under USD's default
/// right-handed orientation puts the front of the shape up.
pub fn outline_mesh(outline: &Outline, max_triangle_area: f64) -> Result<MeshData> {
    creased_mesh(outline, &[], max_triangle_area)
}

/// Fills an outline as [`outline_mesh`] does, with every crease kept as a
/// line of edges through the fill.
///
/// A crease is an open polyline across the ring. It is split into pieces no
/// longer than the side of a triangle at `max_triangle_area`, and every piece
/// becomes an edge of the mesh: a later pass can fold the sheet sharply along
/// it, on an edge, and bend it along its length, where a fold that falls
/// across triangles comes out rounded over their width. A crease is kept
/// wherever it lies inside the ring, and left out where it runs outside it,
/// across a notch in it or across another crease; creases may share points.
pub fn creased_mesh(
    outline: &Outline,
    creases: &[Vec<Point2<f64>>],
    max_triangle_area: f64,
) -> Result<MeshData> {
    let n = outline.points.len();
    let edges: Vec<[usize; 2]> = (0..n).map(|i| [i, (i + 1) % n]).collect();

    // The non-panicking bulk load: a ring that crosses itself is a mistake in
    // whatever built it, and it has to arrive as an error an author can read
    // rather than as a panic out of an author system mid-frame.
    let mut crossings = 0usize;
    let mut cdt = Cdt::try_bulk_load_cdt(outline.points.clone(), edges, |_| crossings += 1)
        .context("the outline could not be triangulated")?;
    if crossings > 0 {
        bail!("the outline crosses itself, at {crossings} of its edges");
    }

    // A crease's points go in before the refinement, so the fill is refined
    // around them, and its edges after it: spade tells inside from outside by
    // counting the constraint edges it crosses, which an open crease would
    // throw off.
    let side = 1.5 * max_triangle_area.sqrt();
    let mut lines = Vec::new();
    for crease in creases {
        let mut line = Vec::new();
        for (i, &to) in crease.iter().enumerate() {
            let from = crease[i.saturating_sub(1)];
            let pieces = ((to.x - from.x).hypot(to.y - from.y) / side)
                .ceil()
                .max(1.0) as usize;
            // Each piece's far end, landing on the crease's own points
            // exactly so two pieces share theirs.
            for k in 1..=pieces {
                let t = k as f64 / pieces as f64;
                let at = if k == pieces {
                    to
                } else {
                    Point2::new(from.x + (to.x - from.x) * t, from.y + (to.y - from.y) * t)
                };
                if encloses(&outline.points, at) {
                    line.push(
                        cdt.insert(at)
                            .context("a crease could not be triangulated")?,
                    );
                } else if !line.is_empty() {
                    lines.push(std::mem::take(&mut line));
                }
            }
        }
        lines.push(line);
    }

    // `exclude_outer_faces` is what `triangle -p` does: refine only what the
    // constraint ring encloses. Without it the outside would be refined too,
    // into points nothing uses.
    cdt.refine(
        RefinementParameters::new()
            .with_angle_limit(AngleLimit::from_deg(ANGLE_LIMIT_DEG))
            .with_max_allowed_area(max_triangle_area)
            .exclude_outer_faces(true),
    );
    for line in &lines {
        for pair in line.windows(2) {
            if pair[0] != pair[1] && cdt.can_add_constraint(pair[0], pair[1]) {
                cdt.add_constraint(pair[0], pair[1]);
            }
        }
    }

    // `vertices()` walks the triangulation's vertex storage in order, so a
    // handle's `index()` is its position in this list. That identity is what
    // lets the faces below be emitted as plain indices with no side table.
    let points: Vec<[f32; 3]> = cdt
        .vertices()
        .map(|v| [v.position().x as f32, v.position().y as f32, 0.0])
        .collect();

    // Inside the ring or out by where a face's middle falls: no face straddles
    // a constraint edge, and the ring is made of them. Without this the fill
    // would be the ring's convex hull, webbing over every notch.
    let face_vertex_indices: Vec<i32> = cdt
        .inner_faces()
        .filter(|face| {
            let [a, b, c] = face.positions();
            let middle = Point2::new((a.x + b.x + c.x) / 3.0, (a.y + b.y + c.y) / 3.0);
            encloses(&outline.points, middle)
        })
        .flat_map(|face| face.vertices().map(|v| v.fix().index() as i32))
        .collect();

    Ok(MeshData {
        face_vertex_counts: vec![3; face_vertex_indices.len() / 3],
        face_vertex_indices,
        points,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testing::{face_normal, faces};

    fn ring(points: &[(f64, f64)]) -> Outline {
        Outline {
            points: points.iter().map(|&(x, y)| Point2::new(x, y)).collect(),
        }
    }

    /// Two units wide and eight long, running along +X from the origin.
    fn square() -> Outline {
        ring(&[(8.0, 1.0), (8.0, -1.0), (0.0, -1.0), (0.0, 1.0)])
    }

    #[test]
    fn area_is_the_area_enclosed() {
        assert_eq!(square().area(), 16.0, "two units by eight");
    }

    #[test]
    fn with_area_rescales_to_the_area_asked_for() {
        let scaled = square().with_area(4.0);
        assert!((scaled.area() - 4.0).abs() < 1e-9);
        // Uniformly, so the shape is unchanged: a quarter of the area is half
        // the size in each direction.
        let reach = scaled.points.iter().fold(0.0f64, |m, p| m.max(p.x));
        assert!((reach - 4.0).abs() < 1e-9, "got {reach}");
    }

    // ─── Filling ────────────────────────────────────────────────────

    fn filled(outline: &Outline) -> MeshData {
        outline_mesh(outline, outline.area() / 32.0).expect("a square fills")
    }

    #[test]
    fn a_fill_is_flat_and_covers_the_outline() {
        let outline = square();
        let mesh = filled(&outline);

        assert!(
            mesh.points.iter().all(|p| p[2] == 0.0),
            "every point lies on the XY plane"
        );
        assert_eq!(
            mesh.face_vertex_counts.iter().sum::<i32>() as usize,
            mesh.face_vertex_indices.len(),
        );
        assert!(mesh.face_vertex_counts.iter().all(|c| *c == 3));

        let area: f32 = faces(&mesh)
            .map(|face| face_normal(&mesh, face).length() / 2.0)
            .sum();
        assert!(
            (area - outline.area() as f32).abs() < 1e-3,
            "the triangles add up to the outline's area, got {area}"
        );
    }

    /// Wound the wrong way, a leaf would be invisible from the side it is
    /// meant to be seen from.
    #[test]
    fn every_face_winds_counter_clockwise_seen_from_above() {
        let mesh = filled(&square());
        for (i, face) in faces(&mesh).enumerate() {
            assert!(
                face_normal(&mesh, face).z > 0.0,
                "face {i} faces +Z, got {:?}",
                face_normal(&mesh, face)
            );
        }
    }

    #[test]
    fn a_finer_limit_spends_more_triangles() {
        let outline = square();
        let coarse = outline_mesh(&outline, outline.area() / 8.0).unwrap();
        let fine = outline_mesh(&outline, outline.area() / 200.0).unwrap();
        assert!(
            fine.face_vertex_counts.len() > coarse.face_vertex_counts.len() * 4,
            "{} vs {}",
            fine.face_vertex_counts.len(),
            coarse.face_vertex_counts.len()
        );
    }

    /// A concave shape is the whole reason the fill is constrained rather
    /// than a plain Delaunay triangulation of the ring's points: the notch
    /// must stay empty instead of being webbed over.
    #[test]
    fn a_notch_stays_outside_the_fill() {
        // A chevron: two arms with a deep notch between them.
        let chevron = ring(&[
            (4.0, 2.0),
            (16.0, 2.0),
            (16.0, 18.0),
            (13.0, 18.0),
            (10.0, 6.0),
            (7.0, 18.0),
            (4.0, 18.0),
        ]);

        let mesh = outline_mesh(&chevron, chevron.area() / 16.0).unwrap();
        let filled: f32 = faces(&mesh)
            .map(|face| face_normal(&mesh, face).length() / 2.0)
            .sum();
        assert!(
            (filled - chevron.area() as f32).abs() < 1e-2,
            "the fill is the outline's area, not its hull: {filled} vs {}",
            chevron.area()
        );
    }

    /// The identity `outline_mesh` emits faces with: a vertex handle's index
    /// is its position in `vertices()`. Nothing in spade's API states it, and
    /// a change to it would scramble every face silently.
    #[test]
    fn vertex_handles_index_the_order_vertices_are_listed_in() {
        let mut cdt = Cdt::new();
        for p in square().points {
            cdt.insert(p).unwrap();
        }
        for (i, v) in cdt.vertices().enumerate() {
            assert_eq!(v.fix().index(), i);
        }
    }

    /// A crease comes out as a line of edges from end to end, split about as
    /// finely as the fill is, and the fill still covers what the ring does.
    #[test]
    fn a_crease_is_a_line_of_edges_through_the_fill() {
        let square = ring(&[(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0)]);
        let (from, to) = (Point2::new(0.5, 0.5), Point2::new(3.5, 3.0));
        let max = square.area() / 64.0;
        let mesh = creased_mesh(&square, &[vec![from, to]], max).unwrap();

        // The vertices on the crease, in order along it.
        let (dx, dy) = (to.x - from.x, to.y - from.y);
        let mut on: Vec<(f64, i32)> = mesh
            .points
            .iter()
            .enumerate()
            .filter_map(|(i, p)| {
                let (px, py) = (p[0] as f64 - from.x, p[1] as f64 - from.y);
                let t = (px * dx + py * dy) / (dx * dx + dy * dy);
                let off = (px * dy - py * dx).abs() / dx.hypot(dy);
                ((-1e-6..=1.0 + 1e-6).contains(&t) && off < 1e-5).then_some((t, i as i32))
            })
            .collect();
        on.sort_by(|a, b| a.0.total_cmp(&b.0));
        assert!(on.len() > 4, "split along its length: {on:?}");
        assert!(on[0].0.abs() < 1e-6 && (on[on.len() - 1].0 - 1.0).abs() < 1e-6);

        let edges: Vec<[i32; 2]> = mesh
            .face_vertex_indices
            .chunks(3)
            .flat_map(|f| [[f[0], f[1]], [f[1], f[2]], [f[2], f[0]]])
            .collect();
        let length = dx.hypot(dy);
        for pair in on.windows(2) {
            let (a, b) = (pair[0].1, pair[1].1);
            assert!(
                edges.contains(&[a, b]) || edges.contains(&[b, a]),
                "{a} to {b} is an edge of the fill"
            );
            assert!((pair[1].0 - pair[0].0) * length <= 1.5 * max.sqrt() + 1e-9);
        }

        let area: f32 = faces(&mesh)
            .map(|face| face_normal(&mesh, face).length() / 2.0)
            .sum();
        assert!((area - 16.0).abs() < 1e-3, "covers the ring, got {area}");

        // One that runs out of the ring is cut where it leaves.
        let out = vec![Point2::new(2.0, 2.3), Point2::new(6.0, 2.3)];
        let cut = creased_mesh(&square, &[out], max).unwrap();
        assert!(
            cut.points.iter().all(|p| (0.0..=4.0).contains(&p[0])),
            "nothing of it outside: {:?}",
            cut.points
        );
        let area: f32 = faces(&cut)
            .map(|face| face_normal(&cut, face).length() / 2.0)
            .sum();
        assert!(
            (area - 16.0).abs() < 1e-3,
            "and the fill is whole, got {area}"
        );
    }

    /// A ring that crosses itself is a mistake in whatever built it. It has to
    /// come back as an error, because spade's bulk load panics on one and an
    /// author system is not a place to panic from.
    #[test]
    fn a_self_crossing_outline_is_an_error_rather_than_a_panic() {
        // A bowtie: the two long edges cross in the middle.
        let bowtie = Outline {
            points: vec![
                Point2::new(0.0, -1.0),
                Point2::new(0.0, 1.0),
                Point2::new(4.0, -1.0),
                Point2::new(4.0, 1.0),
            ],
        };
        let err = outline_mesh(&bowtie, 1.0).expect_err("a bowtie cannot be filled");
        assert!(err.to_string().contains("crosses itself"), "got: {err:#}");
    }
}

//! What a stalk's load does to its shape: where a rod settles under the
//! weight it carries, bent against its own stiffness.
//!
//! A rod here is a polyline held by its first segment, as a
//! [`cable`](crate::scene::cable) is, with a flexural rigidity and a hanging
//! mass at each point. [`sag`] finds where it comes to rest under gravity:
//! each joint, a point between two segments, turns until the moment the
//! loads past it pull with about it is the moment its rigidity holds at that
//! turn. Segments keep their length, and a load's lever shrinks as the rod
//! bends under it, so a heavy load ends up hanging below the rod rather
//! than swinging it round.
//!
//! An organ is drawn in the shape it grew into carrying nothing, then sagged
//! under what it carries now: a stalk bends further as its load grows, and
//! one carrying next to nothing keeps the shape it grew in.

use std::f32::consts::PI;

use bevy::math::{Quat, Vec3};

/// Gravity, in m/s², along -Z.
pub const GRAVITY: f32 = 9.81;

/// Sweeps over the joints at most, and the largest turn, in radians, a sweep
/// may still make once the rod has settled.
const SWEEPS: usize = 200;
const SETTLED: f32 = 1e-6;

/// The flexural rigidity of a round section `radius` in meters, of a
/// material whose Young's modulus is `modulus` in pascals: E·πr⁴/4, in N·m².
pub fn rigidity(modulus: f32, radius: f32) -> f32 {
    modulus * PI / 4.0 * radius.powi(4)
}

/// Where a rod through `points` settles under `loads`, the mass hanging from
/// each point in kilograms, bending against `rigidity`, its flexural rigidity
/// at each point in N·m². A joint whose rigidity is infinite does not bend,
/// nor does one between two segments of no length, or one with no rigidity
/// and nothing past it to bend it; the ends' are never read: the first
/// segment is held, and nothing hangs past the last point.
///
/// Returns each point where it settles, and how each turned: the rotation of
/// the segment leaving it, and for the last point the segment into it.
/// Whatever rides the rod at a point turns with it.
///
/// Each joint's turn is solved in turn from the tip, damped by how far its
/// turn could move the loads past it, until a sweep moves none by more than
/// [`SETTLED`]. Treated as a static beam, a joint resists twisting about the
/// rod as it does bending.
pub fn sag(points: &[Vec3], rigidity: &[f32], loads: &[f32]) -> (Vec<Vec3>, Vec<Quat>) {
    assert!(
        rigidity.len() == points.len() && loads.len() == points.len(),
        "one rigidity and one load per point"
    );
    let runs: Vec<Vec3> = points.windows(2).map(|w| w[1] - w[0]).collect();
    // Each joint's turn as a rotation vector, in the frame of the segment
    // before it, so changing one turns everything past it rigidly about it.
    let mut bends = vec![Vec3::ZERO; runs.len()];
    let joints: Vec<usize> = (1..runs.len())
        .rev()
        .filter(|k| rigidity[*k].is_finite())
        .collect();
    let weights: Vec<Vec3> = loads.iter().map(|m| Vec3::NEG_Z * m * GRAVITY).collect();
    let mut shape = settle(points, &runs, &bends);
    for _ in 0..SWEEPS {
        let mut worst: f32 = 0.0;
        for &k in &joints {
            let (at, frames) = &shape;
            let (mut moment, mut lever) = (Vec3::ZERO, 0.0);
            for (point, weight) in at[k + 1..].iter().zip(&weights[k + 1..]) {
                let arm = *point - at[k];
                moment += arm.cross(*weight);
                lever += arm.length() * weight.length();
            }
            // The joint's stiffness to a turn, over the rod it stands for.
            let stiffness = rigidity[k] * 2.0 / (runs[k - 1].length() + runs[k].length());
            if !stiffness.is_finite() || stiffness + lever == 0.0 {
                continue;
            }
            let unbalanced = frames[k - 1].inverse() * moment - bends[k] * stiffness;
            let step = unbalanced / (stiffness + lever);
            bends[k] += step;
            worst = worst.max(step.length());
            shape = settle(points, &runs, &bends);
        }
        if worst < SETTLED {
            break;
        }
    }
    let (at, mut frames) = shape;
    // The last point turns with the segment into it, and a rod of one point
    // not at all.
    frames.push(*frames.last().unwrap_or(&Quat::IDENTITY));
    frames.truncate(at.len());
    (at, frames)
}

/// The points `runs` lay out from `points[0]` with each joint turned by its
/// `bends`, and each segment's turn.
fn settle(points: &[Vec3], runs: &[Vec3], bends: &[Vec3]) -> (Vec<Vec3>, Vec<Quat>) {
    let mut at = Vec::with_capacity(points.len());
    at.extend(points.first());
    let mut frame = Quat::IDENTITY;
    let frames = runs
        .iter()
        .zip(bends)
        .map(|(run, bend)| {
            frame = (frame * Quat::from_scaled_axis(*bend)).normalize();
            at.push(at[at.len() - 1] + frame * *run);
            frame
        })
        .collect();
    (at, frames)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A level rod a meter long in `count` segments, its tip loaded with
    /// `mass`.
    fn level(count: usize, ei: f32, mass: f32) -> (Vec<Vec3>, Vec<Quat>) {
        let points: Vec<Vec3> = (0..=count)
            .map(|i| Vec3::X * i as f32 / count as f32)
            .collect();
        let mut loads = vec![0.0; points.len()];
        loads[count] = mass;
        sag(&points, &vec![ei; points.len()], &loads)
    }

    /// Under a light load a rod bends as a cantilever does, its tip PL³/3EI
    /// down; the bending starts past the held segment.
    #[test]
    fn a_light_load_bends_a_rod_as_beam_theory_says() {
        let (count, ei, mass) = (100, 100.0, 0.1);
        let (at, _) = level(count, ei, mass);
        let length = 1.0 - 1.0 / count as f32;
        let expected = mass * GRAVITY * length.powi(3) / (3.0 * ei);
        let drop = -at[count].z;
        assert!(
            (drop / expected - 1.0).abs() < 0.03,
            "{drop} against {expected}"
        );
    }

    /// Under a load far heavier than it can hold, a rod hangs straight down
    /// from where it bends, and no segment stretches.
    #[test]
    fn a_heavy_load_hangs_plumb_below_the_rod() {
        let (count, ei) = (40, 0.01);
        let (at, turns) = level(count, ei, 1.0);
        let last = at[count] - at[count - 1];
        assert!(last.angle_between(Vec3::NEG_Z) < 0.05, "{last}");
        for (pair, turn) in at.windows(2).zip(&turns) {
            assert!(((pair[1] - pair[0]).length() * count as f32 - 1.0).abs() < 1e-3);
            assert!((*turn * Vec3::X).distance((pair[1] - pair[0]).normalize()) < 1e-4);
        }
    }

    /// A rod that carries nothing, or cannot bend, keeps its shape: one with
    /// no rigidity and no load, and one doubled back on a point, too. Every
    /// point has a turn, a rod of none or one as well.
    #[test]
    fn an_unloaded_or_rigid_rod_keeps_its_shape() {
        let points = [Vec3::ZERO, Vec3::X, Vec3::new(2.0, 0.0, 1.0)];
        for (rigidity, loads) in [
            ([1.0; 3], [0.0; 3]),
            ([0.0; 3], [0.0; 3]),
            ([f32::INFINITY; 3], [0.0, 0.0, 5.0]),
        ] {
            let (at, turns) = sag(&points, &rigidity, &loads);
            assert_eq!((at.as_slice(), turns.len()), (points.as_slice(), 3));
        }
        let stacked = [Vec3::ZERO, Vec3::X, Vec3::X, Vec3::X];
        assert_eq!(sag(&stacked, &[1.0; 4], &[0.0, 0.0, 0.0, 1.0]).0, stacked);
        for count in [0, 1] {
            let (at, turns) = sag(&points[..count], &[1.0; 3][..count], &[0.0; 3][..count]);
            assert_eq!((at.len(), turns.len()), (count, count));
        }
    }
}

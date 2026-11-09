//! Where a cluster of fruit settles on its stalks: each hung from a node of
//! its own, moved until no two overlap and none lies in a stem or on
//! another's stalk.
//!
//! A fruit here is a ball on a stalk of two straight runs, from its node to
//! a joint and on to the stalk's end, where the ball hangs from it the way
//! that run heads. Laid out where each would hang alone, a crowded cluster's
//! fruit run into each other; [`pack`] moves them apart from there, the
//! joints swinging about their nodes and the balls about their joints, every
//! run keeping its length. Each fruit ends up touching its neighbours
//! rather than in them, turned to face its own stalk.

use bevy::math::Vec3;

/// Rounds [`pack`] runs.
const ROUNDS: usize = 16;

/// How far, in [`pack`]'s first round, a joint springs back toward where it
/// was laid out and a ball swings back toward hanging off its joint as it
/// was, as shares of the way. Both fade to none by the last round.
const SPRING: f32 = 0.2;
const HANG: f32 = 0.3;

/// A ball hung on a stalk of two straight runs: from `node` to `joint`, and
/// on to `end`, where the ball hangs from it the way that run heads.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Bob {
    pub node: Vec3,
    pub joint: Vec3,
    pub end: Vec3,
    /// The ball's radius, and how far past the stalk's end its middle is.
    pub radius: f32,
    pub inset: f32,
    /// The stalk's radius, which every other ball keeps clear of.
    pub stalk: f32,
}

impl Bob {
    /// Where the ball's middle is.
    fn middle(&self) -> Vec3 {
        self.end + (self.end - self.joint).normalize_or(Vec3::NEG_Z) * self.inset
    }
}

/// Moves `bobs` from where they were laid out until none overlaps another,
/// lies in one of `stems`, each a segment and its radius, or lies on another
/// bob's stalk. A joint laid out at its node stays there.
///
/// Each round pulls every joint back toward where it was laid out and every
/// ball toward hanging off its joint as it did, pushes apart whatever
/// overlaps, and puts each stalk's runs back to their lengths, a ball
/// dragging its joint round its node. The pulls fade to none by the last
/// round, so the cluster ends where the pushes and the stalks agree rather
/// than balanced against the pulls. Every round ends with every stalk its
/// length, so a cluster too crowded to clear is left overlapping a little.
///
/// ponytail: every pair, every round; a grid once clusters reach hundreds.
/// A ball pushed straight along the line of its own stalk has nothing to
/// swing it clear, which a laid-out cluster's jitter never lines up.
pub fn pack(bobs: &mut [Bob], stems: &[(Vec3, Vec3, f32)]) {
    let laid = bobs.to_vec();
    // What moves is each joint and each ball's middle; a stalk's end lies on
    // the line between them.
    let hangs: Vec<Vec3> = laid.iter().map(|bob| bob.middle() - bob.joint).collect();
    let mut joints: Vec<Vec3> = laid.iter().map(|bob| bob.joint).collect();
    let mut middles: Vec<Vec3> = laid.iter().map(Bob::middle).collect();
    let end = |i: usize, joint: Vec3, middle: Vec3| {
        middle + (joint - middle).normalize_or(Vec3::Z) * laid[i].inset
    };
    for round in 0..ROUNDS {
        let fade = 1.0 - round as f32 / (ROUNDS - 1) as f32;
        for (i, bob) in laid.iter().enumerate() {
            joints[i] = joints[i].lerp(bob.joint, SPRING * fade);
            middles[i] = middles[i].lerp(joints[i] + hangs[i], HANG * fade);
        }
        for i in 0..laid.len() {
            for k in i + 1..laid.len() {
                let off = middles[k] - middles[i];
                let apart = laid[i].radius + laid[k].radius;
                if off.length_squared() < apart * apart {
                    let push = off.normalize_or(Vec3::Z) * (apart - off.length()) / 2.0;
                    middles[i] -= push;
                    middles[k] += push;
                }
            }
        }
        let stalks: Vec<[(Vec3, Vec3, f32); 2]> = (laid.iter().enumerate())
            .map(|(i, bob)| {
                let (joint, end) = (joints[i], end(i, joints[i], middles[i]));
                [(bob.node, joint, bob.stalk), (joint, end, bob.stalk)]
            })
            .collect();
        for (i, middle) in middles.iter_mut().enumerate() {
            let others = (stalks.iter().enumerate())
                .filter(|(k, _)| *k != i)
                .flat_map(|(_, runs)| *runs);
            for (a, b, radius) in stems.iter().copied().chain(others) {
                let near = closest(a, b, *middle);
                let clear = laid[i].radius + radius;
                if near.distance_squared(*middle) < clear * clear {
                    *middle = near + (*middle - near).normalize_or(Vec3::NEG_Z) * clear;
                }
            }
        }
        for (i, bob) in laid.iter().enumerate() {
            let (joint, middle) = (&mut joints[i], &mut middles[i]);
            // Each run back to its length: the joint gives way half as far
            // as the ball's run is off, round its node, and the ball then
            // hangs that run's length from it.
            let (reach, length) = (bob.joint.distance(bob.node), hangs[i].length());
            let off = *middle - *joint;
            let toward = *joint + off.normalize_or(Vec3::NEG_Z) * (off.length() - length) / 2.0;
            *joint = bob.node + (toward - bob.node).normalize_or(bob.joint - bob.node) * reach;
            *middle = *joint + (*middle - *joint).normalize_or(hangs[i]) * length;
        }
    }
    for (i, bob) in bobs.iter_mut().enumerate() {
        bob.joint = joints[i];
        bob.end = end(i, joints[i], middles[i]);
    }
}

/// The point of the segment from `a` to `b` nearest `p`.
fn closest(a: Vec3, b: Vec3, p: Vec3) -> Vec3 {
    let ab = b - a;
    a + ab * ((p - a).dot(ab) / ab.length_squared().max(f32::EPSILON)).clamp(0.0, 1.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A cluster and the stems it hangs among.
    type Scene = (Vec<Bob>, Vec<(Vec3, Vec3, f32)>);

    /// Six balls 30 mm across off a level stem, 10 mm apart down alternate
    /// sides, the last hanging off the stem's end from a joint at its node,
    /// and a plumb stem down through them: laid out, each lies in its
    /// neighbours, and the middle two in the plumb stem.
    fn crowded() -> Scene {
        let bobs = (0..6)
            .map(|i| {
                let node = Vec3::X * 0.01 * i as f32;
                let side = if i % 2 == 0 { Vec3::Y } else { Vec3::NEG_Y };
                let joint = if i < 5 {
                    node + (side - Vec3::Z) * 0.008
                } else {
                    node
                };
                Bob {
                    node,
                    joint,
                    end: joint + Vec3::NEG_Z * 0.008,
                    radius: 0.015,
                    inset: 0.011,
                    stalk: 0.001,
                }
            })
            .collect();
        let level = (Vec3::ZERO, Vec3::X * 0.05, 0.003);
        let plumb = (Vec3::X * 0.025, Vec3::new(0.025, 0.0, -0.1), 0.003);
        (bobs, vec![level, plumb])
    }

    /// A ball laid out on the stalk of another, its middle 2 mm off it and
    /// clear of the other's ball.
    fn pierced() -> Scene {
        let under = Bob {
            node: Vec3::ZERO,
            joint: Vec3::NEG_Z * 0.03,
            end: Vec3::NEG_Z * 0.04,
            radius: 0.005,
            inset: 0.004,
            stalk: 0.002,
        };
        let across = Bob {
            node: Vec3::X * 0.03,
            joint: Vec3::new(0.02, 0.0, -0.01),
            end: Vec3::new(0.0055, 0.0, -0.0181),
            radius: 0.006,
            inset: 0.004,
            stalk: 0.002,
        };
        (vec![under, across], Vec::new())
    }

    /// Packed, no ball lies more than a millimeter into another, a stem or
    /// another's stalk, and every stalk keeps its runs' lengths: a crowded
    /// cluster among two stems, and a ball laid out across another's stalk.
    /// A ball with room stays as it was laid out.
    #[test]
    fn packing_parts_crowded_balls_on_stalks_their_length() {
        const SLACK: f32 = 0.001;
        for (scene, (laid, stems)) in [crowded(), pierced()].into_iter().enumerate() {
            let mut bobs = laid.clone();
            pack(&mut bobs, &stems);
            for (i, (bob, was)) in bobs.iter().zip(&laid).enumerate() {
                let middle = bob.middle();
                for (a, b, radius) in &stems {
                    let near = closest(*a, *b, middle).distance(middle);
                    assert!(
                        near > bob.radius + radius - SLACK,
                        "scene {scene}, {i}: {near} m off a stem"
                    );
                }
                for (k, other) in bobs.iter().enumerate().filter(|(k, _)| *k != i) {
                    let apart = middle.distance(other.middle());
                    assert!(
                        apart > bob.radius + other.radius - SLACK,
                        "scene {scene}, {i}, {k}: {apart} m apart"
                    );
                    for (a, b) in [(other.node, other.joint), (other.joint, other.end)] {
                        let near = closest(a, b, middle).distance(middle);
                        assert!(
                            near > bob.radius + other.stalk - SLACK,
                            "scene {scene}, {i}: {near} m off {k}'s stalk"
                        );
                    }
                }
                let runs = |bob: &Bob| [bob.node.distance(bob.joint), bob.joint.distance(bob.end)];
                let ([first, second], [laid_first, laid_second]) = (runs(bob), runs(was));
                assert!(
                    (first - laid_first).abs() < 1e-6 && (second - laid_second).abs() < 1e-6,
                    "scene {scene}, {i}: runs {first}, {second} laid out {laid_first}, {laid_second}"
                );
            }
        }

        let lone = crowded().0[0];
        let mut packed = [lone];
        pack(&mut packed, &[]);
        assert!(packed[0].end.distance(lone.end) < 1e-6, "{packed:?}");
    }
}

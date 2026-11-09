//! Where a cluster of fruit settles on its stalks: each hung from a node of
//! its own, moved until no two overlap and none lies in a stem or on
//! another's stalk.
//!
//! A fruit here is a ball on a stalk of two straight runs, from its node to
//! a joint and on to the stalk's end, where the ball hangs from it the way
//! that run heads, drawn out that way into a capsule if the fruit is
//! elongated. Laid out where each would hang alone, a crowded cluster's
//! fruit run into each other; [`pack`] moves them apart from there, the
//! joints swinging about their nodes and the balls about their joints, every
//! run keeping its length. Each fruit ends up touching its neighbours
//! rather than in them, turned to face its own stalk.
//!
//! Stalks packed for bigger fruit than they carry, as when one cluster's
//! stalks are drawn under several clusters' fruit, hang those apart.
//! [`swing`] swings each about its stalk's end, the stalks held, back toward
//! the way it would hang alone until it touches its neighbours.

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
    /// How far the ball is drawn out either way from its middle, the way it
    /// hangs, into a capsule: 0 for a ball.
    pub drawn: f32,
    /// The stalk's radius, which every other ball keeps clear of.
    pub stalk: f32,
}

impl Bob {
    /// Where the ball's middle is.
    fn middle(&self) -> Vec3 {
        self.end + (self.end - self.joint).normalize_or(Vec3::NEG_Z) * self.inset
    }

    /// The segment the ball is drawn out along, from its top to its bottom.
    #[cfg(test)]
    fn spine(&self) -> (Vec3, Vec3) {
        let along = (self.end - self.joint).normalize_or(Vec3::NEG_Z) * self.drawn;
        (self.middle() - along, self.middle() + along)
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
        // How far each ball's spine runs either way of its middle, the way it
        // hangs as the pushes start.
        let along: Vec<Vec3> = (laid.iter().enumerate())
            .map(|(i, bob)| (middles[i] - joints[i]).normalize_or(Vec3::NEG_Z) * bob.drawn)
            .collect();
        let stalks = |middles: &[Vec3]| {
            (laid.iter().enumerate())
                .map(|(i, bob)| {
                    let (joint, end) = (joints[i], end(i, joints[i], middles[i]));
                    [(bob.node, joint, bob.stalk), (joint, end, bob.stalk)]
                })
                .collect()
        };
        part(&laid, &mut middles, &along, stalks, stems);
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

/// Swings each of `bobs`' balls about its stalk's end, every stalk held where
/// it is, from hanging the way its stalk's last run heads toward hanging
/// `toward`, as far as it stays clear of the others, of `stems` and of the
/// other stalks, and no further than `most` radians off that run. Returns
/// the way each then hangs.
///
/// Stalks packed for balls bigger than the ones they carry leave those apart;
/// swung, each hangs as near the way it would alone as there is room for, so
/// crowded ones touch. Pulls and pushes run as [`pack`]'s do.
pub fn swing(bobs: &[Bob], toward: &[Vec3], stems: &[(Vec3, Vec3, f32)], most: f32) -> Vec<Vec3> {
    let heads: Vec<Vec3> = (bobs.iter())
        .map(|bob| (bob.end - bob.joint).normalize_or(Vec3::NEG_Z))
        .collect();
    let stalks: Vec<[(Vec3, Vec3, f32); 2]> = (bobs.iter())
        .map(|bob| {
            [
                (bob.node, bob.joint, bob.stalk),
                (bob.joint, bob.end, bob.stalk),
            ]
        })
        .collect();
    let mut middles: Vec<Vec3> = bobs.iter().map(Bob::middle).collect();
    for round in 0..ROUNDS {
        let fade = 1.0 - round as f32 / (ROUNDS - 1) as f32;
        for (i, bob) in bobs.iter().enumerate() {
            middles[i] = middles[i].lerp(bob.end + toward[i] * bob.inset, HANG * fade);
        }
        let along: Vec<Vec3> = (bobs.iter().enumerate())
            .map(|(i, bob)| (middles[i] - bob.end).normalize_or(heads[i]) * bob.drawn)
            .collect();
        part(bobs, &mut middles, &along, |_| stalks.clone(), stems);
        for (i, bob) in bobs.iter().enumerate() {
            let way = (middles[i] - bob.end).normalize_or(heads[i]);
            middles[i] = bob.end + heads[i].rotate_towards(way, most) * bob.inset;
        }
    }
    (bobs.iter().enumerate())
        .map(|(i, bob)| (middles[i] - bob.end).normalize_or(heads[i]))
        .collect()
}

/// Pushes apart every two of `bobs`' balls that overlap, each at its middle
/// in `middles` and drawn out either way along `along`, then each off `stems`
/// and off every other one's stalk, as `stalks` lays them by the balls'
/// middles.
fn part(
    bobs: &[Bob],
    middles: &mut [Vec3],
    along: &[Vec3],
    stalks: impl Fn(&[Vec3]) -> Vec<[(Vec3, Vec3, f32); 2]>,
    stems: &[(Vec3, Vec3, f32)],
) {
    let spines = |i: usize, middle: Vec3| (middle - along[i], middle + along[i]);
    for i in 0..bobs.len() {
        for k in i + 1..bobs.len() {
            let (on_i, on_k) = nearest(spines(i, middles[i]), spines(k, middles[k]));
            let off = on_k - on_i;
            let apart = bobs[i].radius + bobs[k].radius;
            if off.length_squared() < apart * apart {
                let push = off.normalize_or(Vec3::Z) * (apart - off.length()) / 2.0;
                middles[i] -= push;
                middles[k] += push;
            }
        }
    }
    let stalks = stalks(middles);
    for (i, middle) in middles.iter_mut().enumerate() {
        let others = (stalks.iter().enumerate())
            .filter(|(k, _)| *k != i)
            .flat_map(|(_, runs)| *runs);
        for (a, b, radius) in stems.iter().copied().chain(others) {
            let (near, on) = nearest((a, b), spines(i, *middle));
            let (off, clear) = (on - near, bobs[i].radius + radius);
            if off.length_squared() < clear * clear {
                *middle += off.normalize_or(Vec3::NEG_Z) * clear - off;
            }
        }
    }
}

/// The point of the segment from `a` to `b` nearest `p`.
fn closest(a: Vec3, b: Vec3, p: Vec3) -> Vec3 {
    let ab = b - a;
    a + ab * ((p - a).dot(ab) / ab.length_squared().max(f32::MIN_POSITIVE)).clamp(0.0, 1.0)
}

/// The points of the segments `a` and `b` nearest each other, either of
/// which may be a point.
///
/// Where along `a` the lines through the two come nearest, held to `a`, or
/// its start if they run parallel; the point of `b` nearest that, and the
/// point of `a` nearest that one in turn.
pub fn nearest(a: (Vec3, Vec3), b: (Vec3, Vec3)) -> (Vec3, Vec3) {
    let (da, db, off) = (a.1 - a.0, b.1 - b.0, a.0 - b.0);
    let (aa, bb, ab) = (da.length_squared(), db.length_squared(), da.dot(db));
    let det = aa * bb - ab * ab;
    let s = if det > 1e-6 * aa * bb {
        ((ab * db.dot(off) - bb * da.dot(off)) / det).clamp(0.0, 1.0)
    } else {
        0.0
    };
    let on_b = closest(b.0, b.1, a.0 + da * s);
    (closest(a.0, a.1, on_b), on_b)
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
                    drawn: 0.0,
                    stalk: 0.001,
                }
            })
            .collect();
        let level = (Vec3::ZERO, Vec3::X * 0.05, 0.003);
        let plumb = (Vec3::X * 0.025, Vec3::new(0.025, 0.0, -0.1), 0.003);
        (bobs, vec![level, plumb])
    }

    /// A ball drawn out 4 mm either way, laid out across the stalk of
    /// another: its middle 2 mm off the stalk, the segment it is drawn out
    /// along through it, and clear of the other's ball.
    fn pierced() -> Scene {
        let under = Bob {
            node: Vec3::ZERO,
            joint: Vec3::NEG_Z * 0.03,
            end: Vec3::NEG_Z * 0.04,
            radius: 0.005,
            inset: 0.004,
            drawn: 0.0,
            stalk: 0.002,
        };
        let across = Bob {
            node: Vec3::X * 0.03,
            joint: Vec3::new(0.02, 0.0, -0.01),
            end: Vec3::new(0.0055, 0.0, -0.0181),
            radius: 0.006,
            inset: 0.004,
            drawn: 0.004,
            stalk: 0.002,
        };
        (vec![under, across], Vec::new())
    }

    /// Two balls 12 mm across, drawn out 10 mm either way, laid out leaning
    /// into each other off nodes 40 mm apart: their middles lie clear of
    /// each other, and their bottoms 2 mm apart.
    fn leaning() -> Scene {
        let bobs = [-1.0, 1.0]
            .map(|side: f32| {
                let joint = Vec3::new(-0.02 * side, 0.0, -0.01);
                Bob {
                    node: Vec3::new(-0.02 * side, 0.0, 0.0),
                    joint,
                    end: joint + Vec3::new(0.006 * side, 0.0, -0.008),
                    radius: 0.006,
                    inset: 0.012,
                    drawn: 0.01,
                    stalk: 0.001,
                }
            })
            .to_vec();
        (bobs, Vec::new())
    }

    /// Packed, no ball lies more than a millimeter into another, a stem or
    /// another's stalk, and every stalk keeps its runs' lengths: a crowded
    /// cluster among two stems, a drawn-out ball laid out across another's
    /// stalk, and two drawn out and leaning into each other. A ball with
    /// room stays as it was laid out.
    #[test]
    fn packing_parts_crowded_balls_on_stalks_their_length() {
        const SLACK: f32 = 0.001;
        let apart = |a, b| {
            let (on_a, on_b) = nearest(a, b);
            on_a.distance(on_b)
        };
        for (scene, (laid, stems)) in [crowded(), pierced(), leaning()].into_iter().enumerate() {
            let mut bobs = laid.clone();
            pack(&mut bobs, &stems);
            for (i, (bob, was)) in bobs.iter().zip(&laid).enumerate() {
                let spine = bob.spine();
                for (a, b, radius) in &stems {
                    let near = apart((*a, *b), spine);
                    assert!(
                        near > bob.radius + radius - SLACK,
                        "scene {scene}, {i}: {near} m off a stem"
                    );
                }
                for (k, other) in bobs.iter().enumerate().filter(|(k, _)| *k != i) {
                    let near = apart(spine, other.spine());
                    assert!(
                        near > bob.radius + other.radius - SLACK,
                        "scene {scene}, {i}, {k}: {near} m apart"
                    );
                    for run in [(other.node, other.joint), (other.joint, other.end)] {
                        let near = apart(run, spine);
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

    /// On stalks packed for balls a fifth bigger, a cluster's own balls,
    /// swung back toward the way each was laid out, lie no further into
    /// another, a stem or another's stalk than packing leaves them, and turn
    /// no further off their stalks than they may: the crowded cluster's, which
    /// then each come within a millimeter of another unless held to a turn
    /// too slight for that, and the drawn-out ball laid out across another's
    /// stalk.
    #[test]
    fn swung_balls_touch_on_stalks_packed_for_bigger_ones() {
        const SLACK: f32 = 0.001;
        let apart = |a, b| {
            let (on_a, on_b) = nearest(a, b);
            on_a.distance(on_b)
        };
        let way = |bob: &Bob| (bob.end - bob.joint).normalize();
        for (scene, (laid, stems)) in [crowded(), pierced()].into_iter().enumerate() {
            let mut packed: Vec<Bob> = (laid.iter())
                .map(|bob| Bob {
                    radius: bob.radius * 1.2,
                    inset: bob.inset * 1.2,
                    drawn: bob.drawn * 1.2,
                    ..*bob
                })
                .collect();
            pack(&mut packed, &stems);
            let own: Vec<Bob> = (packed.iter().zip(&laid))
                .map(|(packed, bob)| Bob {
                    radius: bob.radius,
                    inset: bob.inset,
                    drawn: bob.drawn,
                    ..*packed
                })
                .collect();
            let alone: Vec<Vec3> = laid.iter().map(way).collect();
            for most in [0.05, 0.5] {
                let ways = swing(&own, &alone, &stems, most);
                let spines: Vec<(Vec3, Vec3)> = (own.iter().zip(&ways))
                    .map(|(bob, w)| {
                        let middle = bob.end + *w * bob.inset;
                        (middle - *w * bob.drawn, middle + *w * bob.drawn)
                    })
                    .collect();
                for (i, bob) in own.iter().enumerate() {
                    let turned = way(bob).angle_between(ways[i]);
                    assert!(
                        turned < most + 1e-4,
                        "{scene}, {most}, {i}: turned {turned}"
                    );
                    for (a, b, radius) in &stems {
                        let near = apart((*a, *b), spines[i]);
                        let clear = near > bob.radius + radius - SLACK;
                        assert!(clear, "{scene}, {most}, {i}: {near} m off a stem");
                    }
                    let mut gap = f32::INFINITY;
                    for (k, other) in own.iter().enumerate().filter(|(k, _)| *k != i) {
                        let near = apart(spines[i], spines[k]) - bob.radius - other.radius;
                        assert!(
                            near > -SLACK,
                            "{scene}, {most}, {i}, {k}: {near} m into each other"
                        );
                        gap = gap.min(near);
                        for run in [(other.node, other.joint), (other.joint, other.end)] {
                            let near = apart(run, spines[i]);
                            let clear = near > bob.radius + other.stalk - SLACK;
                            assert!(clear, "{scene}, {most}, {i}: {near} m off {k}'s stalk");
                        }
                    }
                    let touch = scene > 0 || most < 0.5 || gap < SLACK;
                    assert!(touch, "{most}, {i}: {gap} m from the nearest");
                }
            }
        }
    }

    /// The segment from the origin 4 m along X comes nearest a skew segment
    /// a meter above it where the two cross, and nearest a point a meter off
    /// it beside that point.
    #[test]
    fn two_segments_come_nearest_where_their_common_perpendicular_runs() {
        let x = (Vec3::ZERO, Vec3::X * 4.0);
        let skew = (Vec3::new(0.0, 1.0, 1.0), Vec3::new(2.0, -1.0, 1.0));
        let point = (Vec3::new(1.0, 1.0, 0.0), Vec3::new(1.0, 1.0, 0.0));
        for (other, there) in [(skew, Vec3::new(1.0, 0.0, 1.0)), (point, point.0)] {
            let (on_x, on_other) = nearest(x, other);
            assert!(
                on_x.distance(Vec3::X) < 1e-6 && on_other.distance(there) < 1e-6,
                "{on_x}, {on_other}"
            );
        }
    }
}

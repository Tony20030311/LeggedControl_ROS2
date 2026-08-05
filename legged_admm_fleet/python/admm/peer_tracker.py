"""Turn one lidar sweep into "peer J is at (x, y)", with no help from what J says.

This is the arithmetic behind scripts/lidar_peer_tracker_node.py, kept ROS-free so
test_peer_tracker.py can pin it. The node does message plumbing and nothing else.

The pipeline is four steps, each a function below:

  1. to_world      sensor-frame points -> world, using the OBSERVER's own pose
  2. keep_bodies   drop the floor and anything absurdly high
  3. cluster_xy    single-link grouping into blobs
  4. associate     decide which blob is which peer

Step 4 is the one that matters for the threat model. It never reads a peer's broadcast
position: tracks start from the roster's spawn poses (fleet configuration, not testimony)
and afterwards follow continuity alone. An attacker lying about where it is therefore
cannot steer which returns get attributed to it -- which is the whole reason the
observation channel is worth anything.

Frames: the observer's own pose comes from its own odometry. That is the same crutch the
agent already leans on for self_p, not a new one, and on real hardware it is the robot's
own state estimator. Nothing here needs a peer's pose in a shared frame.
"""

import numpy as np

# Above this many candidate points after the floor filter, something is wrong with the
# filter (a slope, a mis-set z_min) and the O(n^2) link step would stall the node. The
# measured count for a three-dog fleet is under 100. See cluster_xy.
MAX_CLUSTER_POINTS = 4000


def to_world(pts, sensor_xyz, sensor_yaw):
    """Rotate/translate sensor-frame points into the world frame.

    pts: (N, 3). sensor_xyz: the lidar's world position. sensor_yaw: its world heading.
    Only yaw is applied: the mount is fixed and level on the base, and pitch/roll of a
    trotting body are what the wide vertical scan is there to absorb (see lidar.xacro).
    """
    pts = np.asarray(pts, dtype=float)
    if pts.size == 0:
        return pts.reshape(0, 3)
    c, s = np.cos(sensor_yaw), np.sin(sensor_yaw)
    out = np.empty_like(pts)
    out[:, 0] = c * pts[:, 0] - s * pts[:, 1] + sensor_xyz[0]
    out[:, 1] = s * pts[:, 0] + c * pts[:, 1] + sensor_xyz[1]
    out[:, 2] = pts[:, 2] + sensor_xyz[2]
    return out


def keep_bodies(pts_world, z_min, z_max, sensor_xy, r_min, r_max):
    """Boolean mask for returns that could be a peer's body.

    The floor is the dominant return by an order of magnitude -- a measured sweep had
    11264 of 11321 in-range points on the ground -- so dropping it is not an optimisation,
    it is what makes clustering mean anything. A flat z band is enough on a flat arena;
    a sloped one would need the ground plane fitted instead.
    """
    pts_world = np.asarray(pts_world, dtype=float)
    if pts_world.size == 0:
        return np.zeros(0, dtype=bool)
    finite = np.isfinite(pts_world).all(axis=1)
    z = pts_world[:, 2]
    d = np.hypot(pts_world[:, 0] - sensor_xy[0], pts_world[:, 1] - sensor_xy[1])
    # np.errstate: non-finite rows are already excluded by `finite`, but their comparisons
    # would still raise warnings here.
    with np.errstate(invalid='ignore'):
        return finite & (z >= z_min) & (z <= z_max) & (d >= r_min) & (d <= r_max)


def cluster_xy(pts_xy, gap):
    """Single-link clustering in the ground plane. Returns a list of index arrays.

    Two points join if they are within `gap`. Dogs are held at least D_MIN = 1.3 m apart
    and a body is about 0.9 x 0.6 m, so any gap between roughly 0.15 m (coarser than the
    return spacing on one body) and 0.6 m separates them; the caller's default sits in
    the middle of that.

    O(n^2), which is right for the tens of points that survive keep_bodies and wrong by a
    lot if the floor filter fails -- hence MAX_CLUSTER_POINTS, which raises rather than
    quietly returning a truncated answer.
    """
    pts_xy = np.asarray(pts_xy, dtype=float)
    n = len(pts_xy)
    if n == 0:
        return []
    if n > MAX_CLUSTER_POINTS:
        raise ValueError(
            f'{n} points reached cluster_xy (limit {MAX_CLUSTER_POINTS}). The floor '
            f'filter is not doing its job; check z_min against the actual ground height.')

    parent = np.arange(n)

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    d2 = gap * gap
    for i in range(n):
        # Only j > i: the relation is symmetric and union is idempotent.
        dx = pts_xy[i + 1:, 0] - pts_xy[i, 0]
        dy = pts_xy[i + 1:, 1] - pts_xy[i, 1]
        for k in np.nonzero(dx * dx + dy * dy <= d2)[0]:
            a, b = find(i), find(i + 1 + k)
            if a != b:
                parent[a] = b

    groups = {}
    for i in range(n):
        groups.setdefault(find(i), []).append(i)
    return [np.array(v) for v in groups.values()]


def blob_centres(pts_xy, groups, min_points, sensor_xy, push=0.0, body=None, priors=None):
    """Centre estimate per blob, as (centres (M,2), sizes (M,), kept groups).

    `priors` is an optional list, one per kept blob, passed straight to fit_box. The caller
    only knows which prior belongs to which blob AFTER association, so the node runs this
    twice: once without priors to get candidates to associate, then again for the matched
    blobs with each peer's track as its prior. The kept groups come back so the second call
    can address the same blobs.

    With `body=(length, width)` each blob is fitted with fit_box, which is the only way to
    get the CENTRE out of returns that all came off the near faces. Without it the centroid
    is used, optionally nudged `push` metres directly away from the sensor -- a single
    scalar that cannot track a bias which changes with viewing aspect, kept only so the
    uncorrected behaviour stays reachable for comparison.

    A blob that fit_box cannot fit (too few points) falls back to the centroid rather than
    disappearing: a rough position for a peer beats none.
    """
    centres, sizes, kept = [], [], []
    for g in groups:
        if len(g) < min_points:
            continue
        pts = pts_xy[g]
        prior = priors[len(kept)] if priors is not None and len(kept) < len(priors) else None
        fit = fit_box(pts, sensor_xy, body[0], body[1], prior=prior) if body else None
        if fit is not None:
            c = fit[0]
        else:
            c = pts.mean(axis=0)
            if push:
                u = c - np.asarray(sensor_xy, dtype=float)
                n = np.hypot(u[0], u[1])
                if n > 1e-9:
                    c = c + push * u / n
        centres.append(c)
        sizes.append(len(g))
        kept.append(g)
    if not centres:
        return np.zeros((0, 2)), np.zeros(0, dtype=int), []
    return np.array(centres), np.array(sizes, dtype=int), kept


def associate(tracks, centres, gate):
    """Match blobs to peers by continuity alone. Returns {peer_id: centre_index}.

    tracks: {peer_id: (x, y)} -- where each peer was last seen (or its spawn pose, the
    first time). centres: (M, 2). gate: the furthest a peer can plausibly have moved
    since the last sweep; a candidate beyond it is refused rather than accepted badly.

    Greedy over the globally smallest distance, each blob and each peer used once. With
    at most a handful of both, greedy and optimal agree except in configurations where
    the gate should have refused anyway.

    NOTE what is absent: no peer's broadcast position appears here. That is deliberate --
    see the module docstring.
    """
    ids = sorted(tracks)
    out = {}
    if len(centres) == 0 or not ids:
        return out
    centres = np.asarray(centres, dtype=float)

    pairs = []
    for i in ids:
        p = np.asarray(tracks[i], dtype=float)
        d = np.hypot(centres[:, 0] - p[0], centres[:, 1] - p[1])
        for k in range(len(centres)):
            if d[k] <= gate:
                pairs.append((d[k], i, k))
    pairs.sort()

    used_tracks, used_centres = set(), set()
    for _, i, k in pairs:
        if i in used_tracks or k in used_centres:
            continue
        used_tracks.add(i)
        used_centres.add(k)
        out[i] = k
    return out

# The peer's silhouette as a lidar actually sees it, at the torso height band.
#
# NOT const.xacro's base_length x base_width (0.83 x 0.25). Those are the COLLISION box;
# the lidar sees base.stl, the visual mesh, and at torso height that includes the hip and
# shoulder housings, which stand well proud of the collision box's sides. Measured
# 2026-08-05 by putting 22753 returns from two peers into their own body frames:
#
#     along  u   -0.50 .. +0.46      (collision box assumed +-0.415)
#     across v   -0.28 .. +0.26      (collision box assumed +-0.125)
#
# and the silhouette is centred on the base origin to within 0.03 m, so only the size is
# wrong, not the offset. Fitting with the collision width left every side-on view 0.155 m
# short, which matched the -0.169 m residual the evaluation was reporting.
#
# fit_box needs the size the RETURNS came off, not the one the physics uses.
BODY_L = 0.95
BODY_W = 0.55


def _rot(pts, ang):
    c, s = np.cos(ang), np.sin(ang)
    return np.stack([c * pts[:, 0] + s * pts[:, 1],
                     -s * pts[:, 0] + c * pts[:, 1]], axis=1)


def _boundary_gap(q, half):
    """Signed distance of each point to a centred axis-aligned box's SURFACE.

    Returns (inside_depth, outside_dist), both non-negative. A return that came off the
    body should have both near zero: it sits on the surface.
    """
    d = np.abs(q) - np.asarray(half)
    outside = np.hypot(np.maximum(d[:, 0], 0.0), np.maximum(d[:, 1], 0.0))
    inside = np.where((d[:, 0] <= 0) & (d[:, 1] <= 0), np.minimum(-d[:, 0], -d[:, 1]), 0.0)
    return inside, outside


def fit_box(pts_xy, sensor_xy, length=BODY_L, width=BODY_W, n_angles=90,
            outside_weight=5.0, coverage_weight=1.0, prior=None):
    """Recover a peer's CENTRE by fitting its known body box to the returns it produced.

    Averaging the returns cannot work. A lidar only ever sees the faces turned toward it,
    so the centroid lands on the surface, short of the centre by half of whatever face is
    visible -- 0.125 m across the width, 0.415 m along the length, and something in between
    at a corner. Measured on the fleet 2026-08-05: -0.138 m for a peer seen side-on,
    -0.26 at a rear quarter, -0.34 for the two the V apex sees at its scan edge. No single
    offset cancels a bias that changes with aspect, which is what this replaces.

    The size is known, so the fit is cheap: for each candidate yaw, slide the box until its
    near faces touch the returns (the sensor's side decides which face that is on each
    axis) and score how well the returns sit on the surface. The best yaw wins.

    `prior` is where this peer was last seen, and it only matters when a face is partly
    visible. Looking along a face the returns normally run its whole length, so their
    midpoint is the centre. When they do not -- the apex dog sees 0.36 m of a body whose
    side is 0.95 m, about a hundred returns against the four hundred the rear dogs get --
    that midpoint is the midpoint of a FRAGMENT, and using it puts the centre wherever the
    fragment happened to fall. It showed as error that was almost entirely lateral: 0.156 m
    across the face against 0.032 m along the line of sight.

    What a fragment actually says is that the centre lies somewhere in an interval, and
    nothing more. So with a prior the estimate is projected into that interval: the observed
    axis is untouched and the unobserved one stops inventing a number. The prior is our own
    track history; no peer's broadcast position is involved.

    Returns (centre (2,), yaw, residual), or None if there is not enough to fit.
    """
    pts_xy = np.asarray(pts_xy, dtype=float)
    if len(pts_xy) < 3:
        return None
    sensor = np.asarray(sensor_xy, dtype=float)
    half = (length / 2.0, width / 2.0)

    best = None
    prior_xy = None if prior is None else np.asarray(prior, dtype=float)
    # The box is 180-degree symmetric, so half a turn covers every distinct orientation.
    for ang in np.linspace(0.0, np.pi, n_angles, endpoint=False):
        p = _rot(pts_xy, ang)
        s = _rot(sensor.reshape(1, 2), ang)[0]
        pr = None if prior_xy is None else _rot(prior_xy.reshape(1, 2), ang)[0]
        c = np.empty(2)
        short = 0.0
        for k in (0, 1):
            lo, hi = p[:, k].min(), p[:, k].max()
            # Everything this axis knows is one interval: the centre has to sit where the
            # box still contains every return, which is [hi - half, lo + half]. For a face
            # seen end to end that interval collapses to a point and there is nothing to
            # choose. For a fragment it is wide, and how wide is exactly how little the
            # returns say -- the apex dog sees 0.36 m of a 0.95 m side, so a 0.59 m
            # interval. One rule covers both, with no case for "is this a fragment".
            lo_c, hi_c = hi - half[k], lo + half[k]
            if lo_c > hi_c:
                # The returns are wider than the face: this orientation is wrong, and the
                # outside penalty below will say so. Centre them so the score is honest.
                c[k] = 0.5 * (lo + hi)
            elif pr is not None:
                # Nearest feasible point to where this peer was last seen. On a full face
                # the interval is a point, so the prior changes nothing and cannot outvote
                # the returns; on a fragment it supplies the only information there is.
                c[k] = min(max(pr[k], lo_c), hi_c)
            elif s[k] < lo:
                # No prior: fall back to putting the box's near face on the returns, which
                # is right whenever the face really was seen end to end.
                c[k] = hi_c
            elif s[k] > hi:
                c[k] = lo_c
            else:
                c[k] = 0.5 * (lo + hi)
                # ...and if they do NOT run its whole length, this orientation is claiming
                # the sensor sees a fragment of a face it is looking straight down. That is
                # how a genuine ambiguity resolves: a flat 0.25 m return is equally
                # consistent with the box's 0.25 m end (correct) and with the middle of its
                # 0.83 m side, and both put every point exactly on a surface, so the fit
                # residual alone cannot separate them. Coverage can.
                short += max(2.0 * half[k] - (hi - lo), 0.0)
        inside, outside = _boundary_gap(p - c, half)
        # Outside is weighted up: a body contains all of its own returns, so a pose that
        # leaves some out is wrong in a way that a slightly loose surface fit is not.
        # Coverage is weighted below both, so real occlusion bends the answer rather than
        # breaking it.
        score = float(inside.mean() + outside_weight * outside.mean()
                      + coverage_weight * short)
        if best is None or score < best[0]:
            best = (score, ang, c)

    score, ang, c = best
    return _rot(c.reshape(1, 2), -ang)[0], float(ang), score

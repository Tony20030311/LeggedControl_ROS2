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


def blob_centres(pts_xy, groups, min_points, sensor_xy, push):
    """Centre estimate per blob, as (centres (M,2), sizes (M,)).

    A lidar only ever sees the near face, so the centroid of the returns sits short of the
    body centre by roughly half a body depth. `push` moves each centroid directly away
    from the sensor by that much. It defaults to 0.0 and is meant to be set from the
    measured bias against ground truth (scripts/lidar_tracker_eval.py reports it) rather
    than derived on paper, because the visible face is a box edge whose apparent depth
    depends on the viewing angle.
    """
    centres, sizes = [], []
    for g in groups:
        if len(g) < min_points:
            continue
        c = pts_xy[g].mean(axis=0)
        if push:
            u = c - np.asarray(sensor_xy, dtype=float)
            n = np.hypot(u[0], u[1])
            if n > 1e-9:
                c = c + push * u / n
        centres.append(c)
        sizes.append(len(g))
    if not centres:
        return np.zeros((0, 2)), np.zeros(0, dtype=int)
    return np.array(centres), np.array(sizes, dtype=int)


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

"""Gate for peer_tracker.py -- the arithmetic that turns a lidar sweep into peer positions.

Run: pytest legged_admm_fleet/python/admm/test_peer_tracker.py

The properties worth pinning are not "does it compute a mean". They are:
  - a lie cannot move which returns get attributed to a peer (associate ignores claims)
  - the floor really is removed, because it outnumbers bodies ~200:1
  - clustering separates dogs held at D_MIN and does not split one dog in half
  - a failed floor filter raises instead of stalling the node for minutes
"""

import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(__file__))
from peer_tracker import (  # noqa: E402
    MAX_CLUSTER_POINTS, associate, blob_centres, cluster_xy, keep_bodies, to_world)


def _box(cx, cy, cz, n=40, w=0.6, l=0.9, h=0.3, seed=0):
    """A slab of returns standing in for a dog's body, centred on (cx, cy, cz)."""
    rng = np.random.default_rng(seed)
    return np.stack([cx + rng.uniform(-l / 2, l / 2, n),
                     cy + rng.uniform(-w / 2, w / 2, n),
                     cz + rng.uniform(-h / 2, h / 2, n)], axis=1)


# --- to_world -----------------------------------------------------------------------
def test_to_world_yaw_zero_is_translation():
    p = np.array([[1.0, 2.0, 3.0]])
    np.testing.assert_allclose(to_world(p, (10.0, 20.0, 30.0), 0.0), [[11.0, 22.0, 33.0]])


def test_to_world_rotates_by_yaw():
    # A point 1 m dead ahead, observer facing +y, standing at the origin.
    out = to_world(np.array([[1.0, 0.0, 0.0]]), (0.0, 0.0, 0.0), np.pi / 2)
    np.testing.assert_allclose(out, [[0.0, 1.0, 0.0]], atol=1e-12)


def test_to_world_preserves_relative_geometry():
    # Whatever the pose, distances between points must survive the transform: a bug here
    # would show up as a peer that is the right shape in the wrong place, or vice versa.
    pts = _box(2.0, 0.0, 0.5, n=30)
    a = to_world(pts, (5.0, -3.0, 0.7), 0.9)
    d_before = np.linalg.norm(pts[0] - pts[1])
    d_after = np.linalg.norm(a[0] - a[1])
    assert abs(d_before - d_after) < 1e-12


def test_to_world_empty():
    assert to_world(np.zeros((0, 3)), (0, 0, 0), 0.0).shape == (0, 3)


# --- keep_bodies --------------------------------------------------------------------
def test_keep_bodies_drops_the_floor_which_is_most_of_the_sweep():
    # Proportions from a measured sweep: 11264 ground returns against 57 body returns.
    ground = np.stack([np.random.default_rng(1).uniform(-3, 3, 2000),
                       np.random.default_rng(2).uniform(-3, 3, 2000),
                       np.zeros(2000)], axis=1)
    body = _box(1.4, 0.0, 0.45, n=40)
    pts = np.vstack([ground, body])
    m = keep_bodies(pts, z_min=0.15, z_max=1.2, sensor_xy=(0.0, 0.0), r_min=0.6, r_max=4.0)
    assert m.sum() == 40
    np.testing.assert_allclose(pts[m].mean(axis=0)[0], 1.4, atol=0.2)


def test_keep_bodies_range_gate_both_ends():
    pts = np.array([[0.3, 0.0, 0.5],    # inside r_min: the observer's own legs
                    [1.4, 0.0, 0.5],    # a peer
                    [9.0, 0.0, 0.5]])   # past r_max
    m = keep_bodies(pts, 0.15, 1.2, (0.0, 0.0), 0.6, 4.0)
    assert list(m) == [False, True, False]


def test_keep_bodies_drops_non_finite():
    pts = np.array([[np.nan, 0.0, 0.5], [1.4, np.inf, 0.5], [1.4, 0.0, 0.5]])
    assert list(keep_bodies(pts, 0.15, 1.2, (0.0, 0.0), 0.6, 4.0)) == [False, False, True]


def test_keep_bodies_empty():
    assert keep_bodies(np.zeros((0, 3)), 0.15, 1.2, (0, 0), 0.6, 4.0).shape == (0,)


# --- cluster_xy ---------------------------------------------------------------------
def test_cluster_separates_two_dogs_at_formation_spacing():
    # The V formation's side is 1.40 m and D_MIN is 1.3 m, so this is the closest two
    # bodies are ever meant to be.
    a, b = _box(0.0, 0.0, 0.45, seed=1), _box(1.4, 0.0, 0.45, seed=2)
    pts = np.vstack([a, b])[:, :2]
    groups = cluster_xy(pts, gap=0.3)
    assert len(groups) == 2
    assert sorted(len(g) for g in groups) == [40, 40]


def test_cluster_does_not_split_one_body():
    pts = _box(1.4, 0.0, 0.45, n=60, seed=3)[:, :2]
    groups = [g for g in cluster_xy(pts, gap=0.3) if len(g) >= 5]
    assert len(groups) == 1


def test_cluster_gap_below_return_spacing_shatters_one_body():
    # The failure mode the gap has to stay above: too fine and one dog becomes many peers.
    pts = _box(1.4, 0.0, 0.45, n=20, seed=4)[:, :2]
    assert len(cluster_xy(pts, gap=0.005)) > 1


def test_cluster_chains_through_a_bridge():
    # Single-link is transitive on purpose: a body is one blob even when no two returns
    # on opposite ends are within `gap`.
    pts = np.array([[0.0, 0.0], [0.25, 0.0], [0.5, 0.0]])
    assert len(cluster_xy(pts, gap=0.3)) == 1


def test_cluster_empty():
    assert cluster_xy(np.zeros((0, 2)), 0.3) == []


def test_cluster_refuses_a_failed_floor_filter_instead_of_stalling():
    pts = np.zeros((MAX_CLUSTER_POINTS + 1, 2))
    with pytest.raises(ValueError, match='floor filter'):
        cluster_xy(pts, 0.3)


# --- blob_centres -------------------------------------------------------------------
def test_blob_centres_reports_centroid_and_size():
    pts = np.vstack([_box(0.0, 0.0, 0.45, seed=5), _box(1.4, 0.0, 0.45, seed=6)])[:, :2]
    groups = cluster_xy(pts, 0.3)
    c, n = blob_centres(pts, groups, min_points=5, sensor_xy=(0.0, 0.0), push=0.0)
    order = np.argsort(c[:, 0])
    np.testing.assert_allclose(c[order][:, 0], [0.0, 1.4], atol=0.1)
    assert list(n) == [40, 40]


def test_blob_centres_drops_specks():
    pts = np.array([[1.4, 0.0], [1.41, 0.0]])
    c, _ = blob_centres(pts, cluster_xy(pts, 0.3), min_points=5,
                        sensor_xy=(0.0, 0.0), push=0.0)
    assert len(c) == 0


def test_push_moves_the_centre_directly_away_from_the_sensor():
    # The near-face bias correction: +0.2 along the line of sight, whatever the bearing.
    for bearing in (0.0, 0.7, 2.0, -2.5):
        cx, cy = 1.4 * np.cos(bearing), 1.4 * np.sin(bearing)
        pts = np.array([[cx, cy]])
        c, _ = blob_centres(pts, [np.array([0])], 1, (0.0, 0.0), push=0.2)
        np.testing.assert_allclose(np.hypot(*c[0]), 1.6, atol=1e-9)
        assert abs(np.arctan2(c[0][1], c[0][0]) - bearing) < 1e-9


def test_push_zero_leaves_the_centroid_alone():
    pts = np.array([[1.4, 0.3]])
    c, _ = blob_centres(pts, [np.array([0])], 1, (0.0, 0.0), push=0.0)
    np.testing.assert_allclose(c[0], [1.4, 0.3])


# --- associate ----------------------------------------------------------------------
def test_associate_matches_each_peer_to_its_nearest_blob():
    tracks = {1: (0.8, 0.0), 3: (-0.4, 0.7)}
    centres = np.array([[-0.42, 0.72], [0.81, 0.02]])
    assert associate(tracks, centres, gate=0.5) == {1: 1, 3: 0}


def test_associate_ignores_what_a_peer_claims():
    """The security property. Same returns, same tracks, and the attacker's broadcast
    position is nowhere in the call signature -- so no lie can change the result."""
    tracks = {1: (0.8, 0.0), 3: (-0.4, 0.7)}
    centres = np.array([[0.81, 0.02], [-0.42, 0.72]])
    honest = associate(tracks, centres, 0.5)
    # Whatever robot 3 broadcasts, the only inputs are the tracks and the returns.
    assert honest == associate(dict(tracks), centres.copy(), 0.5) == {1: 0, 3: 1}


def test_associate_refuses_a_blob_beyond_the_gate():
    # A peer that has vanished must produce no match at all, not a wrong one: publishing
    # nothing makes the agent hold its last correction, which is the safe behaviour.
    assert associate({1: (0.0, 0.0)}, np.array([[3.0, 0.0]]), gate=0.5) == {}


def test_associate_never_gives_one_blob_to_two_peers():
    tracks = {1: (0.0, 0.0), 2: (0.1, 0.0)}
    got = associate(tracks, np.array([[0.05, 0.0]]), gate=0.5)
    assert len(got) == 1 and set(got.values()) == {0}


def test_associate_never_gives_one_peer_two_blobs():
    got = associate({1: (0.0, 0.0)}, np.array([[0.05, 0.0], [0.06, 0.0]]), gate=0.5)
    assert len(got) == 1


def test_associate_prefers_the_globally_closest_pair():
    # Greedy order matters: peer 1 is near both blobs, peer 2 only near one. Taking the
    # smallest distance first leaves peer 2 a partner; taking peers in id order does not.
    tracks = {1: (0.0, 0.0), 2: (0.30, 0.0)}
    centres = np.array([[0.32, 0.0], [0.02, 0.0]])
    assert associate(tracks, centres, gate=0.5) == {1: 1, 2: 0}


def test_associate_with_nothing_seen():
    assert associate({1: (0.0, 0.0)}, np.zeros((0, 2)), 0.5) == {}


def test_associate_with_no_peers():
    assert associate({}, np.array([[1.0, 1.0]]), 0.5) == {}


# --- end to end ---------------------------------------------------------------------
def test_full_pipeline_recovers_the_v_formation():
    """Observer at the robot2 slot facing +x; robot1 and robot3 where the roster spawns
    them. Sensor-frame input, peer positions out, with a floor to reject on the way."""
    obs_xy, obs_yaw = np.array([-0.404, -0.7]), 0.0
    truth = {1: np.array([0.808, 0.0]), 3: np.array([-0.404, 0.7])}

    world = [np.stack([np.random.default_rng(7).uniform(-3, 3, 1500),
                       np.random.default_rng(8).uniform(-3, 3, 1500),
                       np.zeros(1500)], axis=1)]
    for k, (pid, p) in enumerate(truth.items()):
        world.append(_box(p[0], p[1], 0.45, n=40, seed=20 + k))
    world = np.vstack(world)

    # Round-trip through the sensor frame so to_world is exercised, not bypassed.
    sensor_xyz = (obs_xy[0], obs_xy[1], 0.74)
    rel = world - np.array(sensor_xyz)
    m = keep_bodies(to_world(rel, sensor_xyz, obs_yaw), 0.15, 1.2, obs_xy, 0.6, 4.0)
    pts_xy = to_world(rel, sensor_xyz, obs_yaw)[m][:, :2]

    centres, _ = blob_centres(pts_xy, cluster_xy(pts_xy, 0.3), 5, obs_xy, push=0.0)
    assert len(centres) == 2

    got = associate({pid: tuple(p) for pid, p in truth.items()}, centres, gate=0.5)
    assert set(got) == {1, 3}
    for pid, k in got.items():
        assert np.linalg.norm(centres[k] - truth[pid]) < 0.15   # the decision boundary

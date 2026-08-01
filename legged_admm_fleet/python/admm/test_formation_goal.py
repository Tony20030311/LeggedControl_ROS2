"""Unit tests for the /formation/goal virtual-centroid command math in
ocs2_fleet_publisher (centroid_slot_targets + min_cost_assignment). Pure functions,
no ROS. Run: python3 -c "import test_formation_goal as T,inspect; [getattr(T,n)() for n in dir(T) if n.startswith('test_')]"
"""
import os, sys
import numpy as np
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "..", "scripts"))
from admm_impl import centroid_slot_targets, min_cost_assignment, rot2d, FORMATIONS

V = FORMATIONS["V"]                       # equilateral tri: [(0.808,0),(-0.404,0.7),(-0.404,-0.7)]


def test_centroid_is_the_commanded_point():
    """The 3 slots' centroid == the commanded goal (mean-centring), any yaw."""
    for yaw in (0.0, 0.7, np.pi / 2, -2.5, np.pi):
        slots = centroid_slot_targets((5.0, 1.0), V, yaw)
        c = np.mean(slots, axis=0)
        assert np.allclose(c, [5.0, 1.0], atol=1e-9), (yaw, c)


def test_shape_preserved_under_rotation():
    """Rotation is rigid: pairwise slot distances match the offsets' pairwise distances."""
    offs = [np.array(o) - np.mean(V, axis=0) for o in V]
    d_off = [np.linalg.norm(offs[a] - offs[b]) for a in range(3) for b in range(a + 1, 3)]
    slots = centroid_slot_targets((2.0, -3.0), V, 1.3)
    d_slot = [np.linalg.norm(slots[a] - slots[b]) for a in range(3) for b in range(a + 1, 3)]
    assert np.allclose(d_off, d_slot, atol=1e-9)


def test_yaw_zero_points_v_along_x():
    """yaw=0: the lead slot (offset (0,0)) is EAST of the two rear slots."""
    slots = centroid_slot_targets((0.0, 0.0), V, 0.0)
    assert slots[0][0] > slots[1][0] and slots[0][0] > slots[2][0]   # dog1 slot is front (+x)


def test_min_cost_assignment_picks_nearest():
    """Dogs sitting on the slots in SWAPPED order -> assignment returns the swap, not id."""
    slots = centroid_slot_targets((5.0, 0.0), V, 0.0)
    pos = [slots[2], slots[0], slots[1]]                 # dog order maps to slots 2,0,1
    assign = min_cost_assignment(pos, slots)
    assert assign == (2, 0, 1), assign
    # cost of chosen assignment is ~0 (dogs already on their nearest slots)
    cost = sum(float(np.dot(pos[k] - slots[assign[k]], pos[k] - slots[assign[k]])) for k in range(3))
    assert cost < 1e-12


def test_180_turn_reassigns_no_crossing():
    """Fleet faced east (yaw 0); now command a goal to the WEST (V flips 180). The nearest
    assignment must swap the rear slots so no dog crosses the whole formation."""
    east = centroid_slot_targets((6.0, 0.0), V, 0.0)     # where dogs currently sit
    west = centroid_slot_targets((0.0, 0.0), V, np.pi)   # flipped V slots
    assign = min_cost_assignment(east, west)
    # identity would send the front dog (east slot0) to the now-west slot0 = a full crossing;
    # the min-cost assignment must NOT be identity here.
    assert assign != (0, 1, 2), assign


# --- degraded (2-dog) formation: exercised after one peer is evicted -----------
COL2 = FORMATIONS["COL2"]


def test_col2_spacing_clears_d_min():
    """The 2-dog fallback shape must sit clear of the inter-agent CBF floor, else its slot
    targets fight the CBF forever. (admm_impl also asserts this at import for EVERY shape;
    this pins the specific value so a silent shrink is a named failure.)"""
    from admm_impl import constants
    d = np.linalg.norm(np.array(COL2[0]) - np.array(COL2[1]))
    assert abs(d - 1.5) < 1e-12, d
    assert d >= constants.D_MIN, (d, constants.D_MIN)


def test_col2_is_a_column_along_travel():
    """COL2 must string the pair out ALONG the direction of travel, not abreast: the plum-post
    gaps only fit one body width. centroid_slot_targets rotates by yaw = bearing to the goal,
    so the two slots must differ only along that bearing."""
    for yaw in (0.0, 0.7, np.pi / 2, -2.5, np.pi):
        slots = centroid_slot_targets((5.0, 2.0), COL2, yaw)
        assert np.allclose(np.mean(slots, axis=0), [5.0, 2.0], atol=1e-9), yaw
        heading = np.array([np.cos(yaw), np.sin(yaw)])
        sep = np.array(slots[0]) - np.array(slots[1])
        # fully along the heading -> zero component on the perpendicular
        lateral = float(sep @ np.array([-heading[1], heading[0]]))
        assert abs(lateral) < 1e-9, (yaw, lateral)
        assert abs(np.linalg.norm(sep) - 1.5) < 1e-9, (yaw, sep)


def test_col2_assignment_does_not_cross():
    """Two survivors already strung out along the path keep their order (no swap-through)."""
    slots = centroid_slot_targets((5.0, 0.0), COL2, 0.0)
    assign = min_cost_assignment([slots[0], slots[1]], slots)
    assert assign == (0, 1), assign


def test_col2_is_byte_identical_to_the_hand_written_shape_it_replaced():
    """COL2 used to be a literal in fleet_config.cpp; it is now the n=2 instance of a generated
    column rule. That was justified as a SIMPLIFICATION -- the rule reproduces the old literal
    exactly -- so the claim has to be checkable rather than asserted. Exact equality, not a
    tolerance: a generator that drifts by 1e-9 has stopped being the same shape."""
    assert [tuple(p) for p in COL2] == [(0.75, 0.0), (-0.75, 0.0)], COL2


def test_degraded_columns_exist_for_every_size_a_fleet_can_shrink_to():
    """shape_for() resolves a degraded fleet to "COL<n_live>". If any n has no entry,
    set_formation() treats the missing name as a SILENT no-op, the coordinator's size guard then
    rejects every goal ("no N-slot formation offsets"), and the survivors never get another
    target -- a wedged fleet with one WARN line. The 4-slot case is the one a five-dog fleet
    needs the moment it evicts a member."""
    from admm_impl import constants
    for n in range(2, 9):
        col = FORMATIONS["COL%d" % n]
        assert len(col) == n, (n, col)
        pts = np.array(col)
        assert np.allclose(pts.mean(axis=0), [0.0, 0.0], atol=1e-12), (n, pts.mean(axis=0))
        assert np.allclose(pts[:, 1], 0.0), (n, "a column has no lateral offset")
        gaps = np.abs(np.diff(np.sort(pts[:, 0])))
        assert np.allclose(gaps, 1.5), (n, gaps)
        assert gaps.min() >= constants.D_MIN, (n, gaps.min(), constants.D_MIN)


def test_v5_is_a_regular_pentagon_every_pair_can_observe():
    """The five-dog shape. Its min spacing must clear the geometry floor (or the slot targets
    fight the inter-agent CBF) and its LONGEST diagonal must stay inside obs_range, because the
    point of N=5 is that a symmetric two-way disagreement finally has third parties to break it
    -- which requires those third parties to be able to see both of them."""
    from admm_impl import constants
    pts = np.array(FORMATIONS["V5"])
    assert len(pts) == 5
    assert np.allclose(pts.mean(axis=0), [0.0, 0.0], atol=1e-6), pts.mean(axis=0)
    d = [np.linalg.norm(pts[i] - pts[j]) for i in range(5) for j in range(i + 1, 5)]
    assert min(d) >= constants.D_MIN, (min(d), constants.D_MIN)
    assert min(d) >= 1.4, min(d)          # the 2026-07-24 goal/formation geometry floor
    assert max(d) <= 4.0, max(d)          # obs_range: every pair mutually observable

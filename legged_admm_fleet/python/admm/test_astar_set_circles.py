"""set_circles() swaps the A* circle set in place, keeping bounds/resolution/rects.

This is what lets an agent add or drop a dead peer's keep-out without rebuilding the planner
and losing the arena-sized bounds it was constructed with (rebuilding with a fleet-local box
made later out-of-box goals unplannable -> straight line through a keep-out -> frozen dog).

The obstacles are BAKED into the occupancy grid at construction, so the point of these tests
is that set_circles recomputes that grid rather than just swapping the vector.
"""
import os, sys
import numpy as np
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "..", "scripts"))
from admm_impl import AStarPlanner

_BOX = dict(x_min=-2.0, x_max=12.0, y_min=-6.0, y_max=6.0, boundary_margin=0.45)
_START, _GOAL = (0.0, 0.0), (8.0, 0.0)
_BLOCKER = [{"pos": (4.0, 0.0), "radius": 0.9}]     # straddles the straight line


def _max_lateral(path):
    return max(abs(p[1]) for p in path) if path else 0.0


def test_added_circle_forces_a_detour():
    p = AStarPlanner(resolution=0.15, robot_radius=0.3, obstacles=[], **_BOX)
    assert _max_lateral(p.plan(_START, _GOAL)) < 1e-9, "empty map should go straight"
    p.set_circles(_BLOCKER)
    path = p.plan(_START, _GOAL)
    assert path, "blocked but still routable around"
    assert _max_lateral(path) > 0.9, _max_lateral(path)


def test_cleared_circles_restore_the_straight_line():
    p = AStarPlanner(resolution=0.15, robot_radius=0.3, obstacles=_BLOCKER, **_BOX)
    assert _max_lateral(p.plan(_START, _GOAL)) > 0.9
    p.set_circles([])
    assert _max_lateral(p.plan(_START, _GOAL)) < 1e-9, "corpse removal must free the lane"


def test_bounds_and_grid_shape_survive_the_swap():
    """The whole reason set_circles exists: bounds are preserved, so a goal that was
    plannable before the swap is still plannable after."""
    p = AStarPlanner(resolution=0.15, robot_radius=0.3, obstacles=[], **_BOX)
    omap0 = np.asarray(p._debug_omap())
    far = (11.0, 5.0)                                  # near the far corner, inside the box
    assert p.plan(_START, far), "far goal must be reachable before the swap"
    p.set_circles(_BLOCKER)
    omap1 = np.asarray(p._debug_omap())
    assert omap1.shape == omap0.shape, (omap0.shape, omap1.shape)
    assert omap1.sum() > omap0.sum(), "the new circle must actually occupy cells"
    assert p.plan(_START, far), "far goal must STILL be reachable after the swap"

"""ADMM implementation binding — the C++ port (admm_core_cpp) is the sole path.

The Python golden reference and the ADMM_IMPL switch were retired at C6h; C++ is
now the only implementation. Gates still import the swappable symbols THROUGH this
shim (C, ac, nq, ma, rti, es, LaplacianFormation, reference/build_reference,
AStarPlanner, and the fleet data ARENAS/FORMATIONS/DEFAULT_GOALS + slot math), so
this stays the single binding surface.
"""

import math
import os
import sys
import types

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.dirname(_HERE))

import admm_core_cpp as _cpp

constants = _cpp.constants
ac = _cpp.coordinator
ma = _cpp.motion_adapter
rti = _cpp.rti
LaplacianFormation = _cpp.LaplacianFormation
nq = types.SimpleNamespace(
    NodeSubproblem=_cpp.NodeSubproblem,
    h_obstacle=_cpp.h_obstacle,
    h_wall=_cpp.h_wall,
)
es = types.SimpleNamespace(EdgeSubproblem=_cpp.EdgeSubproblem)
reference = _cpp.reference
build_reference = _cpp.reference.build_reference
AStarPlanner = _cpp.AStarPlanner
ARENAS = _cpp.ARENAS
FORMATIONS = _cpp.FORMATIONS
DEFAULT_GOALS = _cpp.DEFAULT_GOALS
rot2d = _cpp.rot2d
centroid_slot_targets = _cpp.centroid_slot_targets
min_cost_assignment = _cpp.min_cost_assignment


# ---- static feasibility guards -------------------------------------------------
# The inter-agent CBF statically forbids any terminal set whose slots sit closer than
# D_MIN; before C6h this drift went unnoticed (D_MIN 1.0->1.3 left goals at 0.86 m).
# These run at import time over the C++ fleet data (the single source), so a geometry
# regression fails the whole test suite / any verify_* import instead of at runtime.
def assert_min_spacing(points, label, d_min=None):
    """Pairwise spacing of goal/start/formation slots must be >= D_MIN."""
    d = constants.D_MIN if d_min is None else d_min
    pts = [tuple(p) for p in (points.values() if isinstance(points, dict) else points)]
    for i in range(len(pts)):
        for j in range(i + 1, len(pts)):
            dist = math.hypot(pts[i][0] - pts[j][0], pts[i][1] - pts[j][1])
            assert dist >= d - 1e-9, \
                "%s: slots %d,%d spaced %.3f < D_MIN %.3f" % (label, i, j, dist, d)


def assert_gaps_passable(obstacles, robot_margin, extra, label):
    """Generalized verify_door._assert_two_abreast_passable: every obstacle pair must
    leave a corridor >= r_eff_a + r_eff_b + extra, where r_eff = radius + robot_margin.
    extra=0 -> single-file passability; extra=D_MIN -> two dogs abreast."""
    obs = [(tuple(o["pos"]), o["radius"]) for o in obstacles]
    for a in range(len(obs)):
        for b in range(a + 1, len(obs)):
            d = math.hypot(obs[a][0][0] - obs[b][0][0], obs[a][0][1] - obs[b][0][1])
            thr = (obs[a][1] + robot_margin) + (obs[b][1] + robot_margin) + extra
            assert d >= thr - 1e-9, \
                "%s: obstacles (%.2f,%.2f)-(%.2f,%.2f) gap %.3f < %.3f" % (
                    label, obs[a][0][0], obs[a][0][1], obs[b][0][0], obs[b][0][1], d, thr)


for _name, _offs in FORMATIONS.items():
    assert_min_spacing(_offs, "FORMATIONS[%s]" % _name)
for _name, _arena in ARENAS.items():
    assert_min_spacing(_arena["goals"], "ARENAS[%s].goals" % _name)
assert_min_spacing(DEFAULT_GOALS, "DEFAULT_GOALS")

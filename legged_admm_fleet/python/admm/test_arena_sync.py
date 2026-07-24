"""Guard against arena geometry drift between fleet_config.cpp (the single source, exposed
as admm_impl.ARENAS) and its two hand-synced copies: the Gazebo world generator's obstacle
coordinates and arena_run.sh's per-arena GOALS. The import-time feasibility guard only sees
the C++ data; these copies live outside it, so this test fails the suite the moment they
diverge (the exact geometry-drift class this branch set out to eliminate).
"""
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.normpath(os.path.join(HERE, "..", "..", "scripts"))
sys.path.insert(0, HERE)
sys.path.insert(0, SCRIPTS)
from admm_impl import ARENAS as CFG            # noqa: E402  C++ single source of truth
import gen_arena_world as gen                  # noqa: E402  standalone world generator

ARENAS_WITH_WORLD = ("plum", "plum_dense", "door")


def _positions(obstacles):
    return sorted((round(o["pos"][0], 3), round(o["pos"][1], 3)) for o in obstacles)


def test_world_generator_positions_match_config():
    for name in ARENAS_WITH_WORLD:
        cfg = _positions(CFG[name]["obstacles"])
        world = sorted((round(x, 3), round(y, 3)) for (x, y, _r) in gen.ARENAS[name]["circles"])
        assert cfg == world, \
            "%s: gen_arena_world obstacle coords drifted from fleet_config.cpp" % name


def test_arena_run_goals_match_config():
    sh = open(os.path.join(SCRIPTS, "arena_run.sh")).read()
    for name in ARENAS_WITH_WORLD:
        m = re.search(r'^\s*%s\)\s*GOALS="([^"]+)"' % re.escape(name), sh, re.M)
        assert m, "arena_run.sh has no built-in GOALS line for %s" % name
        got = [float(x) for x in m.group(1).split()]
        goals = CFG[name]["goals"]
        flat = [c for i in sorted(goals) for c in goals[i]]   # robot-id order, x then y
        assert len(got) == len(flat) and all(abs(a - b) < 1e-6 for a, b in zip(got, flat)), \
            "%s: arena_run.sh GOALS drifted from fleet_config.cpp goals" % name

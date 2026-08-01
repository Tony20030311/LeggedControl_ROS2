#!/usr/bin/env python3
"""Find a spot where the victim is OCCLUDED from every survivor but still within sensor range.

    hide_spot.py <dist.csv> <victim_id> <survivor ids...>   ->  "x y"  (or nothing, exit 1)

Spec §9 protocol item 6 requires the occlusion result to be a POSITIVE record -- abstain code 5
from both observers -- not "the log went quiet". Code 5 and code 4 (out_of_range) both come from
admm::visible() returning false, so a hiding spot that is merely FAR proves nothing about
occlusion. This searches for in-range-but-blocked, which is the only configuration that can
produce code 5.

The geometry here MIRRORS admm::visible() (trust.hpp) but is not authoritative and does not need
to be: it only proposes where to walk the victim. What the experiment actually reports is the
abstain code the C++ code itself wrote into CycleStats.tel_abstain. If this search and the real
visible() ever disagreed, the run would simply fail to produce code 5 and say so.

The pile list mirrors fleet_config.cpp's "plum" arena. Keep in sync -- but again, a stale copy
here can only send the victim to a worse spot, never falsify a result.
"""
import csv
import math
import sys

# fleet_config.cpp arenas()["plum"]: 7 rows / 17 piles, CBF radius 0.30 (physical 0.20).
PLUM = [(4.76, 1.54), (4.76, -1.54), (6.58, 0.0), (6.58, 2.94), (6.58, -2.94),
        (8.40, 1.54), (8.40, -1.54), (10.22, 0.0), (10.22, 2.94), (10.22, -2.94),
        (12.04, 1.54), (12.04, -1.54), (13.86, 0.0), (13.86, 2.94), (13.86, -2.94),
        (15.68, 1.54), (15.68, -1.54)]
R_PILE = 0.30
OBS_RANGE = 4.0
D_MIN = 1.3          # inter-agent CBF floor: a spot inside it is not reachable, only fought over
PILE_CLEAR = 0.90    # CBF radius 0.30 + robot_margin 0.60


def visible(pi, pj, obs, rng):
    """admm::visible(), same order of tests."""
    dx, dy = pj[0] - pi[0], pj[1] - pi[1]
    ln = math.hypot(dx, dy)
    if not (ln <= rng):
        return False
    if ln < 1e-9:
        return True
    for (ox, oy) in obs:
        wx, wy = ox - pi[0], oy - pi[1]
        t = (wx * dx + wy * dy) / (ln * ln)
        if t <= 0.0 or t >= 1.0:
            continue
        if math.hypot(wx - t * dx, wy - t * dy) < R_PILE:
            return False
    return True


rows = list(csv.DictReader(open(sys.argv[1])))
if not rows:
    sys.exit(1)
last = rows[-1]
victim = int(sys.argv[2])
survivors = [int(a) for a in sys.argv[3:]]
obs_pos = [(float(last["x%d" % s]), float(last["y%d" % s])) for s in survivors]
vic = (float(last["x%d" % victim]), float(last["y%d" % victim]))

best, best_cost = None, float("inf")
# Search around the survivors, not around the arena: the spot has to be reachable and in range.
cx = sum(p[0] for p in obs_pos) / len(obs_pos)
cy = sum(p[1] for p in obs_pos) / len(obs_pos)
step = 0.10
n = int(OBS_RANGE / step)
for i in range(-n, n + 1):
    for j in range(-n, n + 1):
        p = (cx + i * step, cy + j * step)
        if any(math.hypot(p[0] - o[0], p[1] - o[1]) < D_MIN for o in obs_pos):
            continue
        if any(math.hypot(p[0] - q[0], p[1] - q[1]) < PILE_CLEAR for q in PLUM):
            continue
        # IN RANGE of every observer (so a false verdict is occlusion, not distance) and blocked
        # for every one of them. Both conditions on EVERY observer -- one blind observer is the
        # ordinary case and proves nothing.
        if not all(math.hypot(p[0] - o[0], p[1] - o[1]) <= OBS_RANGE for o in obs_pos):
            continue
        if any(visible(o, p, PLUM, OBS_RANGE) for o in obs_pos):
            continue
        cost = math.hypot(p[0] - vic[0], p[1] - vic[1])   # nearest such spot to where it already is
        if cost < best_cost:
            best, best_cost = p, cost

if best is None:
    sys.exit(1)
print("%.3f %.3f" % best)

#!/usr/bin/env python3
"""Positive stall detection over a dist.csv trace.

"The end-to-end script gave up" is NOT evidence of a safe stall — that is how a fake PASS
looks. This decides the question from the recorded trajectory instead, and it is allowed to
answer "no": a fleet that was still crawling toward the goal returns None and the caller
must fail.

A stall is a window of at least T seconds where ALL THREE hold:

  1. the watched robots' centroid stays within `move_eps` of where it was when the window
     opened                                        — nobody is making progress
  2. the smallest pairwise distance AMONG THE WATCHED robots varies by less than `pair_eps`
     — they are pinned against each other, not merely idling
  3. the centroid is still further than `goal_tol` from the goal
     — they have not simply arrived and stopped

Dropping any one of the three breaks it: without (2) a fleet resting on target reads as a
stall, without (3) a successful run does, and without (1) a slow crawl does.

Reused by experiment D (`d_run.sh EXPECT=stall`, plum_dense two-dog degraded mode) and by
the Phase 3 convoy work — hence a plain function over dist.csv rows with no ROS import.

    python3 stall_detect.py <dist.csv> --robots 1,2 --goal 18.0,0.0 [--T 15]
        exit 0  stall confirmed (window printed)
        exit 2  no stall — the fleet was still moving / had arrived
        exit 1  usage or data error
    python3 stall_detect.py            # run the self-check
"""
import csv
import math
import sys


def series(rows, robots):
    """(t, cx, cy, min_pair) column-wise, centroid and min pair over `robots` only."""
    ids = [str(r) for r in robots]
    t, cx, cy, mp = [], [], [], []
    for r in rows:
        pts = [(float(r["x" + i]), float(r["y" + i])) for i in ids]
        t.append(float(r["t"]))
        cx.append(sum(p[0] for p in pts) / len(pts))
        cy.append(sum(p[1] for p in pts) / len(pts))
        # inf for a single robot: condition (2) then imposes nothing (a lone dog cannot be
        # wedged against a peer), which is the honest reading, not a silent pass.
        mp.append(min((math.hypot(a[0] - b[0], a[1] - b[1])
                       for k, a in enumerate(pts) for b in pts[k + 1:]), default=math.inf))
    return t, cx, cy, mp


def find_stall(rows, robots, goal, T=15.0, move_eps=0.15, pair_eps=0.05, goal_tol=0.45):
    """First (t_start, t_end) window of >= T seconds satisfying all three, else None."""
    t, cx, cy, mp = series(rows, robots)
    gx, gy = goal
    for i in range(len(t)):
        x0, y0, lo, hi = cx[i], cy[i], mp[i], mp[i]
        for k in range(i, len(t)):
            if math.hypot(cx[k] - x0, cy[k] - y0) >= move_eps:
                break                                   # (1) it moved
            lo, hi = min(lo, mp[k]), max(hi, mp[k])
            if hi - lo >= pair_eps:
                break                                   # (2) the pair distance is working
            if math.hypot(cx[k] - gx, cy[k] - gy) <= goal_tol:
                break                                   # (3) it is at the goal, not stuck
            if t[k] - t[i] >= T:
                return (t[i], t[k])
    return None


def _synth(n, step, pair, t0=0.0, dt=0.05):
    """Two dogs in column, centroid advancing `step` per sample, `pair` apart."""
    return [{"t": f"{t0 + k * dt:.3f}",
             "x1": f"{k * step:.4f}", "y1": "0.0",
             "x2": f"{k * step - pair:.4f}", "y2": "0.0"}
            for k in range(n)]


def demo():
    goal = (30.0, 0.0)
    walking = _synth(600, 0.02, 1.5)                       # 12 m of progress
    assert find_stall(walking, [1, 2], goal) is None, "a walking fleet is not stalled"

    jammed = walking + _synth(600, 0.0, 1.06, t0=30.0)     # frozen 12 m short of the goal
    win = find_stall(jammed, [1, 2], goal)
    assert win is not None, "a wedged fleet must be detected"
    assert win[1] - win[0] >= 15.0, win

    arrived = _synth(600, 0.0, 1.5)
    for r in arrived:                                      # parked exactly on the goal
        r["x1"], r["x2"] = "30.0", "30.0"
    assert find_stall(arrived, [1, 2], goal) is None, "arriving is not stalling"

    creeping = _synth(600, 0.0006, 1.06, t0=30.0)          # 1.8 cm/s — slow, but moving
    assert find_stall(walking + creeping, [1, 2], goal) is None, "a crawl is not a stall"

    breathing = [dict(r) for r in _synth(600, 0.0, 1.06, t0=30.0)]
    for k, r in enumerate(breathing):                      # pair distance still being worked
        r["x2"] = f"{-1.06 + 0.06 * math.sin(k / 10.0):.4f}"
    assert find_stall(walking + breathing, [1, 2], goal) is None, "a working CBF is not a stall"
    print("stall_detect self-check OK")


def main(argv):
    if len(argv) < 2:
        demo()
        return 0
    path, robots, goal, T = argv[1], [1, 2, 3], None, 15.0
    for k, a in enumerate(argv):
        if a == "--robots":
            robots = [s for s in argv[k + 1].replace(",", " ").split()]
        elif a == "--goal":
            gx, gy = argv[k + 1].replace(",", " ").split()
            goal = (float(gx), float(gy))
        elif a == "--T":
            T = float(argv[k + 1])
    if goal is None:
        print("stall_detect: --goal x,y is required", file=sys.stderr)
        return 1
    with open(path) as f:
        rows = list(csv.DictReader(f))
    if len(rows) < 10:
        print(f"stall_detect: only {len(rows)} sample(s) in {path}", file=sys.stderr)
        return 1
    win = find_stall(rows, robots, goal, T=T)
    if win is None:
        print(f"stall_detect: NO stall (robots={robots} goal={goal} T={T}) — "
              f"the fleet was moving or had arrived")
        return 2
    print(f"stall_detect: STALL {win[0]:.2f}..{win[1]:.2f}s "
          f"({win[1] - win[0]:.1f}s, robots={robots}, goal={goal})")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))

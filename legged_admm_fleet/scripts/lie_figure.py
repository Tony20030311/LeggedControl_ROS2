#!/usr/bin/env python3
"""The figures the false-signal demo rests on.

Two runs of the SAME scenario, differing only in whether the survivors act on Gate 2's verdict.

Top row is the argument, and it is spatial: the survivors' PATHS. With detection on they bend
around the compromised robot's real body; with it off they bend around a position 2.83 m away
and walk past the body instead. Two ribbons curving to opposite sides of the same object — no
threshold, no axis-reading.

Bottom row is the receipt: how close the real body actually got, and (for the detection-off run)
the pair of curves showing the fleet was not being careless, just careful about the wrong thing.

    python3 lie_figure.py ON/dist.csv OFF/dist.csv 1 out.png [offset]

Deliberately not: styling, CLI framework, arenas other than plum. One page, one job.
"""
import csv
import math
import sys

import matplotlib
matplotlib.use("Agg")  # headless: this runs over docker exec, there is no display
import matplotlib.pyplot as plt

# plum: 7-row quincunx, 17 piles. Physical radius 0.20 in the SDF, 0.30 to the CBF (fleet_config
# .cpp). Drawn at the physical radius — this is a picture of the world, not of the constraint.
PLUM = [(4.76, 1.54), (4.76, -1.54), (6.58, 0.0), (6.58, 2.94), (6.58, -2.94),
        (8.40, 1.54), (8.40, -1.54), (10.22, 0.0), (10.22, 2.94), (10.22, -2.94),
        (12.04, 1.54), (12.04, -1.54), (13.86, 0.0), (13.86, 2.94), (13.86, -2.94),
        (15.68, 1.54), (15.68, -1.54)]

C_GHOST, C_REAL, C_SURV = "#3d7ea6", "#d1495b", "#0f7173"


def load(path, victim, offset):
    """-> dict with per-robot xy tracks, plus the ghost track and the nearest-survivor series."""
    t, vic, ghost, surv, near_real, near_ghost = [], [], [], {}, [], []
    with open(path) as f:
        for r in csv.DictReader(f):
            vx, vy = float(r["x%d" % victim]), float(r["y%d" % victim])
            others = {i: (float(r["x%d" % i]), float(r["y%d" % i]))
                      for i in (1, 2, 3) if i != victim and "x%d" % i in r}
            if not others:
                continue
            t.append(float(r["t"]))
            vic.append((vx, vy))
            ghost.append((vx + offset, vy + offset))
            for i, p in others.items():
                surv.setdefault(i, []).append(p)
            near_real.append(min(math.hypot(vx - x, vy - y) for x, y in others.values()))
            near_ghost.append(min(math.hypot(vx + offset - x, vy + offset - y)
                                  for x, y in others.values()))
    t0 = t[0] if t else 0.0
    return dict(t=[x - t0 for x in t], vic=vic, ghost=ghost, surv=surv,
                near_real=near_real, near_ghost=near_ghost)


def paths(ax, d, title):
    for x, y in PLUM:
        ax.add_patch(plt.Circle((x, y), 0.20, color="#b0b0b0", zorder=1))
    ax.plot(*zip(*d["ghost"]), color=C_GHOST, lw=1.4, ls="--", zorder=2,
            label="where it CLAIMED to be")
    ax.plot(*zip(*d["vic"]), color=C_REAL, lw=2.0, zorder=3, label="its REAL body")
    for n, (i, track) in enumerate(sorted(d["surv"].items())):
        ax.plot(*zip(*track), color=C_SURV, lw=1.6, zorder=4,
                label="survivors" if n == 0 else None)
    # Where the body ended up is where the keep-out circle got anchored, so mark it.
    if d["vic"]:
        ax.plot(*d["vic"][-1], marker="X", ms=9, color=C_REAL, zorder=5)
    ax.set_title(title, fontsize=10)
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.grid(alpha=0.2)


def rules(ax):
    # 1.30 is the enforced separation, 0.867 where the two base boxes touch. Measured constants
    # (CLAUDE.md), not decoration.
    ax.axhline(1.30, ls="--", lw=0.9, color="grey")
    ax.axhline(0.867, ls=":", lw=0.9, color="grey")
    ax.text(0.01, 1.33, "D_MIN 1.30", transform=ax.get_yaxis_transform(), fontsize=7,
            color="grey")
    ax.text(0.01, 0.90, "contact 0.87", transform=ax.get_yaxis_transform(), fontsize=7,
            color="grey")
    ax.set_xlabel("time (s)")
    ax.grid(alpha=0.25)


def main():
    if len(sys.argv) < 5:
        sys.exit(__doc__)
    on = load(sys.argv[1], int(sys.argv[3]), float(sys.argv[5]) if len(sys.argv) > 5 else 2.0)
    off = load(sys.argv[2], int(sys.argv[3]), float(sys.argv[5]) if len(sys.argv) > 5 else 2.0)

    # Three panels, not four. A "they held station against the ghost" panel was here and had to
    # go: the counterfactual fleet DOES eventually evict the liar (its roster stops including
    # them, which is positive evidence with or without Gate 2), and after that it stops consulting
    # the claim at all. Measured 2026-07-29: a survivor passed within 0.060 m of the claimed
    # position. "Fooled for the whole run" is not what happens and must not be drawn as if it is.
    fig = plt.figure(figsize=(13, 8.5))
    ax = [[fig.add_subplot(2, 2, 1), fig.add_subplot(2, 2, 2)],
          [fig.add_subplot(2, 1, 2)]]
    paths(ax[0][0], on, "detection ON — survivors route around the real body")
    paths(ax[0][1], off, "detection OFF — survivors route around the claim")
    ax[0][0].legend(fontsize=8, loc="upper left")

    a = ax[1][0]
    a.plot(off["t"], off["near_real"], lw=1.8, color=C_REAL, label="detection OFF")
    a.plot(on["t"], on["near_real"], lw=1.8, color=C_SURV, label="detection ON")
    rules(a)
    a.set_ylabel("nearest survivor to the REAL body (m)")
    a.set_title("what the lie cost in true separation", fontsize=10)
    a.legend(fontsize=8, loc="upper right")
    for d, colour in ((off, C_REAL), (on, C_SURV)):
        if d["near_real"]:
            m = min(d["near_real"])
            a.annotate("%.3f" % m, xy=(d["t"][d["near_real"].index(m)], m), xytext=(4, 6),
                       textcoords="offset points", fontsize=8, color=colour)

    fig.tight_layout()
    fig.savefig(sys.argv[4], dpi=150)
    print("wrote", sys.argv[4])


if __name__ == "__main__":
    main()

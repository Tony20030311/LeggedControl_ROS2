#!/usr/bin/env python3
"""Side-by-side top-down animation of the two false-signal runs.

The Gazebo recording cannot carry this demo on its own: the lie has no body, so the one thing a
viewer needs to see — the position robot1 CLAIMS to be at — is not in the world and never
appears on camera. This renders it.

Left = detection ON, right = detection OFF. Same clock, same scale. Per frame:

  filled red      robot1's real body
  dashed blue     the position it BROADCASTS — the lie, which has no body and never appears on
                  camera, so this is the whole reason a rendered diagram beats the recording
  teal            the two survivors
  live readout    nearest survivor to the REAL body, which is the number the demo is about

    python3 lie_anim.py ON/dist.csv OFF/dist.csv 1 out.mp4 [offset] [fps]

Deliberately not: 3D, textures, camera moves. A diagram that is legible at a glance beats
footage that is technically real and unreadable.
"""
import csv
import math
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.animation as animation
import matplotlib.pyplot as plt

PLUM = [(4.76, 1.54), (4.76, -1.54), (6.58, 0.0), (6.58, 2.94), (6.58, -2.94),
        (8.40, 1.54), (8.40, -1.54), (10.22, 0.0), (10.22, 2.94), (10.22, -2.94),
        (12.04, 1.54), (12.04, -1.54), (13.86, 0.0), (13.86, 2.94), (13.86, -2.94),
        (15.68, 1.54), (15.68, -1.54)]

C_GHOST, C_REAL, C_SURV = "#3d7ea6", "#d1495b", "#0f7173"
BODY_R = 0.433          # half the 0.867 m contact distance: draw dogs at the size that matters


def load(path, victim, offset):
    out = []
    with open(path) as f:
        for r in csv.DictReader(f):
            v = (float(r["x%d" % victim]), float(r["y%d" % victim]))
            surv = [(float(r["x%d" % i]), float(r["y%d" % i]))
                    for i in (1, 2, 3) if i != victim and "x%d" % i in r]
            out.append(dict(t=float(r["t"]), vic=v, ghost=(v[0] + offset, v[1] + offset),
                            surv=surv,
                            near=min(math.hypot(v[0] - x, v[1] - y) for x, y in surv)))
    t0 = out[0]["t"] if out else 0.0
    for row in out:
        row["t"] -= t0
    return out


class Panel:
    def __init__(self, ax, rows, title):
        self.rows = rows
        ax.set_title(title, fontsize=11)
        ax.set_xlim(-1.5, 18.5)
        ax.set_ylim(-4.2, 5.2)
        ax.set_aspect("equal")
        ax.grid(alpha=0.2)
        ax.set_xlabel("x (m)")
        for x, y in PLUM:
            ax.add_patch(plt.Circle((x, y), 0.20, color="#b0b0b0", zorder=1))
        self.trail_s, = ax.plot([], [], color=C_SURV, lw=1.0, alpha=0.55, zorder=2)
        self.trail_v, = ax.plot([], [], color=C_REAL, lw=1.0, alpha=0.55, zorder=2)
        # NO keep-out ring. One was drawn here and it lied in both directions: it appeared before
        # any eviction had happened (robot1 was still an ordinary peer under the pairwise CBF,
        # with no keep-out at all), and it was centred on the claim in BOTH panels — which is the
        # exact opposite of what the detection-ON fleet does once it has caught the lie. A marker
        # that contradicts the claim it illustrates is worse than no marker.
        self.ghost = plt.Circle((0, 0), BODY_R, fill=False, lw=1.8, ls="--", color=C_GHOST,
                                zorder=4, label="where robot1 CLAIMS to be")
        ax.add_patch(self.ghost)
        self.body = plt.Circle((0, 0), BODY_R, color=C_REAL, zorder=5,
                               label="robot1's real body")
        ax.add_patch(self.body)
        self.dogs = [plt.Circle((0, 0), BODY_R, color=C_SURV, zorder=5,
                                label="survivors" if k == 0 else None) for k in range(2)]
        for d in self.dogs:
            ax.add_patch(d)
        ax.legend(loc="upper right", fontsize=8, framealpha=0.9)
        self.txt = ax.text(0.02, 0.96, "", transform=ax.transAxes, fontsize=10, va="top",
                           family="monospace")
        self.sx, self.sy, self.vx, self.vy = [], [], [], []

    def draw(self, k):
        r = self.rows[min(k, len(self.rows) - 1)]
        self.body.center = r["vic"]
        self.ghost.center = r["ghost"]
        for d, p in zip(self.dogs, r["surv"]):
            d.center = p
        self.vx.append(r["vic"][0]); self.vy.append(r["vic"][1])
        self.sx.append(r["surv"][0][0]); self.sy.append(r["surv"][0][1])
        self.trail_v.set_data(self.vx, self.vy)
        self.trail_s.set_data(self.sx, self.sy)
        self.txt.set_text("t=%5.1fs\nto real body: %.2f m" % (r["t"], r["near"]))


def main():
    if len(sys.argv) < 5:
        sys.exit(__doc__)
    victim = int(sys.argv[3])
    offset = float(sys.argv[5]) if len(sys.argv) > 5 else 2.0
    fps = int(sys.argv[6]) if len(sys.argv) > 6 else 25
    on = load(sys.argv[1], victim, offset)
    off = load(sys.argv[2], victim, offset)

    fig, axes = plt.subplots(2, 1, figsize=(11, 9))
    p_on = Panel(axes[0], on, "detection ON — the fleet fences the real body")
    p_off = Panel(axes[1], off, "detection OFF — the fleet fences the lie")
    axes[0].set_ylabel("y (m)")
    axes[1].set_ylabel("y (m)")
    fig.tight_layout()

    # dist.csv is 20 Hz; step it to land near the requested frame rate rather than resampling.
    step = max(1, round(20.0 / fps))
    n = max(len(on), len(off)) // step

    def frame(i):
        p_on.draw(i * step)
        p_off.draw(i * step)
        return []

    animation.FuncAnimation(fig, frame, frames=n, interval=1000 // fps, blit=False).save(
        sys.argv[4], writer=animation.FFMpegWriter(fps=fps, bitrate=2400))
    print("wrote %s (%d frames)" % (sys.argv[4], n))


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# Offline analysis for the distributed-fleet arena runs. Reads one or more run dirs (each with
# stats.csv + traj.csv from g5_logger.py) and writes 3 PNGs to <outdir>:
#   latency.png      - comm latency: CDF of per-cycle recv wait time (state/xi/z + total), ms
#   traj_error.png   - trajectory tracking error: pos (m) and vel (m/s) error vs time, per dog
#   convergence.png  - ADMM: mean primal-residual vs round + histogram of rounds-to-converge
#
# Usage: g5_plots.py <outdir> <run_dir> [<run_dir> ...]
#   env: CONV_REL=0.1 (converged when residual <= this fraction of round-0 residual), CONV_ABS=1e-3
import csv
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

OUT = sys.argv[1]
RUN_DIRS = sys.argv[2:]
CONV_REL = float(os.environ.get("CONV_REL", "0.1"))
CONV_ABS = float(os.environ.get("CONV_ABS", "1e-3"))
os.makedirs(OUT, exist_ok=True)


def read_stats(d):
    rows = []
    p = os.path.join(d, "stats.csv")
    if not os.path.exists(p):
        return rows
    with open(p) as f:
        for r in csv.DictReader(f):
            try:
                hist = [float(x) for x in r["r_prim_hist"].split("|") if x != ""]
            except (KeyError, ValueError):
                hist = []
            rows.append({
                "t_wait_state": float(r["t_wait_state"]), "t_wait_xi": float(r["t_wait_xi"]),
                "t_wait_z": float(r["t_wait_z"]), "achieved_rounds": int(r["achieved_rounds"]),
                "hold": int(r["hold"]), "hist": hist,
                "t_rx_mean": float(r.get("t_rx_mean", 0.0) or 0.0),
                "reset": int(r.get("reset", 0) or 0), "n_stale": int(r.get("n_stale", 0) or 0),
            })
    return rows


def read_traj(d):
    p = os.path.join(d, "traj.csv")
    if not os.path.exists(p):
        return None, [], {}
    with open(p) as f:
        rd = csv.DictReader(f)
        cols = rd.fieldnames
        robots = sorted({c[len("pos_err"):] for c in cols if c.startswith("pos_err")}, key=int)
        data = {c: [] for c in cols}
        for row in rd:
            for c in cols:
                try:
                    data[c].append(float(row[c]))
                except ValueError:
                    data[c].append(np.nan)
    t = np.array(data["t"])
    t = t - t[0] if len(t) else t
    return t, robots, data


# ---------- 1. comm latency ----------
stats = [r for d in RUN_DIRS for r in read_stats(d)]
if stats:
    # CDF excludes HOLD cycles (no barrier completes) -> report hold-rate alongside so the
    # exclusion is explicit, not silent (§C: the stressed cycles are exactly the held ones).
    hold_rate = 100.0 * np.mean([s["hold"] for s in stats]) if stats else 0.0
    st = np.array([s["t_wait_state"] for s in stats if not s["hold"]]) * 1e3
    xi = np.array([s["t_wait_xi"] for s in stats if not s["hold"]]) * 1e3
    z = np.array([s["t_wait_z"] for s in stats if not s["hold"]]) * 1e3
    rx = np.array([s["t_rx_mean"] for s in stats if not s["hold"] and s["t_rx_mean"] > 0]) * 1e3
    tot = st + xi + z
    HOP_DEADLINE_MS = 20.0  # per-hop recv deadline (admm_agent_node hop_deadline_ms default)
    fig, ax = plt.subplots(figsize=(7, 4.5))
    curves = [(st, "state barrier"), (xi, "edge xi"), (z, "edge z"), (tot, "total / cycle")]
    if len(rx):
        curves.append((rx, "one-way rx"))
    for arr, lab in curves:
        if len(arr) == 0:
            continue
        xs = np.sort(arr)
        ys = np.arange(1, len(xs) + 1) / len(xs)
        ax.plot(xs, ys, label=f"{lab}  (med {np.median(arr):.2f} ms, p95 {np.percentile(arr,95):.2f})")
    ax.axvline(HOP_DEADLINE_MS, color="k", ls="--", lw=1, alpha=0.6, label=f"hop deadline {HOP_DEADLINE_MS:.0f} ms")
    ax.set_xlabel("per-cycle DDS recv wait [ms]")
    ax.set_ylabel("CDF")
    ax.set_title(f"Distributed ADMM communication latency (DDS recv wait); hold rate {hold_rate:.1f}%")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "latency.png"), dpi=130)
    p95 = np.percentile(tot, 95) if len(tot) else float("nan")
    slo = "PASS" if len(tot) and p95 < HOP_DEADLINE_MS else "FAIL"
    print(f"latency.png: n={len(tot)} cycles, total/cycle median {np.median(tot):.2f} ms, "
          f"mean {np.mean(tot):.2f} ms, p95 {p95:.2f} ms")
    print(f"SLO: p95 total wait {p95:.2f} ms vs {HOP_DEADLINE_MS:.0f} ms hop deadline -> {slo}; "
          f"hold rate {hold_rate:.1f}%")

# ---------- 2. trajectory error ----------
fig, (a1, a2) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
any_traj = False
for d in RUN_DIRS:
    t, robots, data = read_traj(d)
    if t is None or len(t) == 0:
        continue
    any_traj = True
    tag = os.path.basename(d).replace("arena_", "")
    for r in robots:
        pe = np.array(data[f"pos_err{r}"])
        ve = np.array(data[f"vel_err{r}"])
        a1.plot(t, pe, lw=1, label=f"{tag} dog{r} (RMS {np.sqrt(np.nanmean(pe**2)):.3f})")
        a2.plot(t, ve, lw=1)
if any_traj:
    a1.set_ylabel("position error [m]")
    a1.set_title("Distributed fleet trajectory tracking error (actual odom vs commanded reference)")
    a1.grid(True, alpha=0.3); a1.legend(fontsize=7, ncol=2)
    a2.set_ylabel("velocity error [m/s]"); a2.set_xlabel("time [s]")
    a2.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "traj_error.png"), dpi=130)
    print("traj_error.png written")

# ---------- 3. convergence (warm-start aware) ----------
# Most cycles are warm-started (RTI): at 10 Hz the problem barely changes (~4 cm/cycle), so the last
# cycle's solution is reused and the residual starts at ~machine-zero — 0 iterations needed. Those
# would trivially read as "converged in 1 round" and swamp the metric, so split them out. Only cycles
# that actually START far from converged (r_prim[0] > WARM_MAX) are measured, and among THOSE we
# separate ones that reach TOL from ones the ACTIVE CBF constraints plateau (binding inequality ->
# consensus residual cannot go to 0; it holds at ~1e-2 and uses the full P_ITERS budget).
WARM_MAX = 1e-4
TOL = 1e-6
allh = [s["hist"] for s in stats if len(s["hist"]) >= 2]
warm = [h for h in allh if h[0] <= WARM_MAX]
iter_ = [h for h in allh if h[0] > WARM_MAX]
if allh:
    conv_rounds, plateau_final = [], []
    for h in iter_:
        k = next((j + 1 for j, v in enumerate(h) if v <= TOL), None)
        (conv_rounds if k is not None else plateau_final).append(k if k is not None else h[-1])
    conv = np.array(conv_rounds, dtype=float)

    fig, (a1, a2) = plt.subplots(1, 2, figsize=(11.5, 4.4))
    if iter_:
        maxr = max(len(h) for h in iter_)
        grid = np.full((len(iter_), maxr), np.nan)
        for i, h in enumerate(iter_):
            grid[i, :len(h)] = h
        for i in range(min(len(iter_), 80)):
            a1.semilogy(np.arange(1, len(iter_[i]) + 1), iter_[i], color="#bbb", lw=0.5, alpha=0.4)
        a1.semilogy(np.arange(1, maxr + 1), np.nanmedian(grid, axis=0), "o-", color="#c00", ms=4, label="median")
        a1.axhline(TOL, color="g", ls=":", label=f"tol {TOL:g}")
    a1.set_xlabel("ADMM round"); a1.set_ylabel("primal residual r_prim")
    a1.set_title(f"Convergence of real-iterating cycles (r0 > {WARM_MAX:g})")
    a1.grid(True, alpha=0.3, which="both"); a1.legend(fontsize=8)

    cats = ["steady\n(warm-start,\n0 iter)", f"iterate\n-> {TOL:g}", "iterate\nplateau\n(CBF active)"]
    vals = [len(warm), len(conv), len(plateau_final)]
    bars = a2.bar(cats, vals, color=["#4472c4", "#2e8b57", "#d98a3d"], edgecolor="k")
    for b, v in zip(bars, vals):
        pct = 100 * v / max(1, len(allh))
        a2.text(b.get_x() + b.get_width() / 2, v, f"{v}\n{pct:.0f}%", ha="center", va="bottom", fontsize=8)
    a2.set_ylabel("cycles"); a2.set_title(f"Cycle breakdown ({len(allh)} total)")
    a2.margins(y=0.15)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "convergence.png"), dpi=130)
    cm = f"{conv.mean():.1f}" if len(conv) else "n/a"
    pf = f"{np.median(plateau_final):.1e}" if plateau_final else "n/a"
    print(f"convergence.png: {len(warm)} warm(0-iter, {100*len(warm)/len(allh):.0f}%), "
          f"{len(conv)} iterate->{TOL:g} (mean {cm} rounds), "
          f"{len(plateau_final)} CBF-plateau (median final {pf}), of {len(allh)} cycles")

print(f"plots -> {OUT}")

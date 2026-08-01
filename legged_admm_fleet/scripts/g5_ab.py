#!/usr/bin/env python3
"""G5 A/B: what the belief layer costs on the wire, beyond its own payload.

Usage: g5_ab.py <a1_run_dir> <a2_run_dir>

bytes_tx/bytes_rx are read-and-RESET by take_bytes() at every publishStats, so each stats.csv row
carries the traffic since that robot's previous row. The rate is a sum over INTERVALS, never a
first-to-last span: g5_logger's subscriptions do not all establish at once, and a span-based rate
turns a hole in the recording into invented bandwidth.

achieved_rounds is reported alongside because it is the confound. EdgeXi/EdgeZ dominate the
traffic and scale with it, so a difference in mean rounds between the two runs is a difference in
bytes that has nothing to do with evidence.
"""
import csv, collections, sys, math

TS = 0.10
EV_TX = 40.0    # 4*|ev_peer| + 8*|ev_pos| with 2 peers observed, from wire_bytes()
EV_RX = 80.0    # two peers' broadcasts


def rates(d, t0=None, t1=None):
    rows = list(csv.DictReader(open("%s/stats.csv" % d)))
    if t0 is not None:
        rows = [r for r in rows if t0 <= float(r["t"]) <= t1]
    per = collections.defaultdict(list)
    for r in rows:
        per[r["robot"]].append((float(r["t"]), float(r["bytes_tx"]), float(r["bytes_rx"]),
                                int(r["achieved_rounds"])))
    out = {}
    for rid, seq in per.items():
        seq.sort()
        tx = rx = dur = 0.0
        ar = []
        for (ta, _, _, _), (tb, btx, brx, a) in zip(seq, seq[1:]):
            dt = tb - ta
            if 0 < dt <= 1.0:
                tx += btx; rx += brx; dur += dt; ar.append(a)
        if dur > 0:
            out[rid] = (tx / dur, rx / dur, dur, sum(ar) / len(ar), len(ar))
    return out


def mean(v):
    return sum(v) / len(v)


T0, T1 = (float(sys.argv[3]), float(sys.argv[4])) if len(sys.argv) > 4 else (None, None)
if T0 is not None:
    print("matched sim-time window: %.1f .. %.1f s\n" % (T0, T1))
a, b = rates(sys.argv[1], T0, T1), rates(sys.argv[2], T0, T1)
for name, d, r in (("a1", sys.argv[1], a), ("a2", sys.argv[2], b)):
    print("%s  %s" % (name, d.rstrip("/").split("/")[-1]))
    for rid in sorted(r):
        tx, rx, dur, ar, n = r[rid]
        print("   robot%s  tx=%9.1f B/s  rx=%9.1f B/s  over %6.1fs  rounds=%.3f (n=%d)"
              % (rid, tx, rx, dur, ar, n))
    print("   MEAN     tx=%9.1f B/s  rx=%9.1f B/s   rounds=%.3f"
          % (mean([v[0] for v in r.values()]), mean([v[1] for v in r.values()]),
             mean([v[3] for v in r.values()])))

dtx = mean([v[0] for v in b.values()]) - mean([v[0] for v in a.values()])
drx = mean([v[1] for v in b.values()]) - mean([v[1] for v in a.values()])
dar = mean([v[3] for v in b.values()]) - mean([v[3] for v in a.values()])
base_tx = mean([v[0] for v in a.values()])

# Bytes per ADMM round, from a1 alone: total minus the one AgentState per cycle, divided by
# rounds/s. Lets the round-count difference be converted into bytes and compared with the effect.
agentstate_tx = 1053.0 / TS
per_round = (base_tx - agentstate_tx) / (mean([v[3] for v in a.values()]) / TS)

print()
print("measured delta   tx %+9.1f B/s   rx %+9.1f B/s" % (dtx, drx))
print("predicted payload tx %+9.1f B/s   rx %+9.1f B/s" % (EV_TX / TS, EV_RX / TS))
print("achieved_rounds  %+.4f  ->  %+.1f B/s of tx from convergence alone (%.0f B/round)"
      % (dar, dar / TS * per_round, per_round))
print()
print("UNEXPLAINED (measured - payload - rounds): tx %+.1f B/s"
      % (dtx - EV_TX / TS - dar / TS * per_round))
print("...against a %.0f B/s baseline = %+.4f %%"
      % (base_tx, 100.0 * (dtx - EV_TX / TS - dar / TS * per_round) / base_tx))

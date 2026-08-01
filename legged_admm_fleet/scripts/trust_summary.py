#!/usr/bin/env python3
"""One row of Task-12 metrics for one d_run.sh log directory.

    trust_summary.py /root/legged_ros2_ws/g2_logs/d_0801_035834

Everything here is read from the files the run already wrote; nothing is re-derived from a
number printed by the run script itself, because the point of the results table is to be
checkable against the raw data rather than against d_run.sh's own summary line.

Two deliberate choices, both paid for by this project's recorded mistakes:

  * contact is read from phys_gap.csv, NEVER from dist.csv. Centre-to-centre was measured on
    2026-07-29 to read ~0.68 m high -- three dogs standing undisturbed are 1.40 m apart by
    centre and 0.717 m by surface -- so a run whose centres never dropped below 0.90 can still
    have had bodies 0.07 m apart.
  * every distance is a minimum over EVERY row, never a sampled poll. d_run.sh's own min_pair
    is sampled once per ~5 s and a shorter excursion slips past it (a plum run reached 0.526 m
    for 4.3 s and still printed PASS).
"""
import csv
import itertools
import math
import os
import re
import sys

LOGD = sys.argv[1].rstrip("/")


def rows(name):
    p = os.path.join(LOGD, name)
    if not os.path.exists(p) or os.path.getsize(p) == 0:
        return []
    with open(p) as f:
        return list(csv.DictReader(f))


def fnum(x, default=float("nan")):
    try:
        return float(x)
    except (TypeError, ValueError):
        return default


dlog = open(os.path.join(LOGD, "d.log")).read() if os.path.exists(os.path.join(LOGD, "d.log")) else ""
alog = open(os.path.join(LOGD, "admm.log"), errors="replace").read() \
    if os.path.exists(os.path.join(LOGD, "admm.log")) else ""
glog = open(os.path.join(LOGD, "gazebo.log"), errors="replace").read() \
    if os.path.exists(os.path.join(LOGD, "gazebo.log")) else ""


def dgrep(pat, default=""):
    m = re.search(pat, dlog)
    return m.group(1) if m else default


arm = dgrep(r"ARM=(\S+)")
seed = dgrep(r"obs_noise_seed=(\d+)")
victim = dgrep(r"victim=robot(\d+)")
survivors = re.findall(r"survivors='([^']*)'", dlog)
survivors = survivors[0].split() if survivors else []
lie = dgrep(r"lie=([\d.]+)m")
arrive = dgrep(r"arrive_dist=([\d.]+)")
verdict = "PASS" if "D PASS" in dlog else ("STALL-SAFE" if "STALL-SAFE" in dlog else "FAIL")
cause = dgrep(r"FAIL(?:\(infra\))?: (.+)")
skew = dgrep(r"burst skew <= ([\d.]+)s")

# ---- separation, every row ----------------------------------------------------------------
d = rows("dist.csv")
ids = sorted({int(m) for r in d for c in r for m in re.findall(r"^x(\d+)$", c)}) if d else []


def worst_centre(subset):
    pairs = list(itertools.combinations(sorted(subset), 2))
    if not d or not pairs:
        return float("nan"), "none"
    best, who = float("inf"), "none"
    for i, j in pairs:
        g = min(math.hypot(fnum(r["x%d" % i]) - fnum(r["x%d" % j]),
                           fnum(r["y%d" % i]) - fnum(r["y%d" % j])) for r in d)
        if g < best:
            best, who = g, "%d-%d" % (i, j)
    return best, who


surv_ids = [int(s) for s in survivors]
sv_centre, sv_centre_pair = worst_centre(surv_ids)
all_centre, all_centre_pair = worst_centre(ids)

# ---- contact, every row, surface to surface ------------------------------------------------
pg = rows("phys_gap.csv")
gap_cols = [c for c in (pg[0] if pg else {}) if c.startswith("gap_") and c != "gap_min"]
gap_by_pair = {c.replace("gap_", "").replace("_", "-"): min(fnum(r[c]) for r in pg)
               for c in gap_cols}
sv_gap = min((g for p, g in gap_by_pair.items() if victim not in p.split("-")),
             default=float("nan"))
sv_gap_pair = min((p for p, g in gap_by_pair.items() if victim not in p.split("-")),
                  key=lambda p: gap_by_pair[p], default="none")
vic_gap = min((g for p, g in gap_by_pair.items() if victim in p.split("-")),
              default=float("nan"))
vic_gap_pair = min((p for p, g in gap_by_pair.items() if victim in p.split("-")),
                   key=lambda p: gap_by_pair[p], default="none")

# ---- consensus health, comm cost, belief telemetry -----------------------------------------
# bytes_tx/bytes_rx are read-and-RESET by take_bytes() at every publishStats, so each row is
# the traffic since the previous row: summing them and dividing by the sim span is the honest
# per-agent rate. Summing the raw column as if it were cumulative would be wrong.
st = rows("stats.csv")
ar = [fnum(r["achieved_rounds"]) for r in st if r.get("achieved_rounds")]
per_robot_bytes, per_robot_span = {}, {}
for r in st:
    rid = r.get("robot")
    if rid is None:
        continue
    tx, rx, t = fnum(r["bytes_tx"], 0), fnum(r["bytes_rx"], 0), fnum(r["t"])
    b = per_robot_bytes.setdefault(rid, [0.0, 0.0])
    b[0] += tx
    b[1] += rx
    s = per_robot_span.setdefault(rid, [t, t])
    s[0], s[1] = min(s[0], t), max(s[1], t)
tx_rate = {k: v[0] / max(per_robot_span[k][1] - per_robot_span[k][0], 1e-9)
           for k, v in per_robot_bytes.items()}
rx_rate = {k: v[1] / max(per_robot_span[k][1] - per_robot_span[k][0], 1e-9)
           for k, v in per_robot_bytes.items()}

# tel = "peer:resid:l_self:l_total:abstain" groups, '|'-joined. Empty for A0/A1 (beliefStep
# never runs there), which is itself the measurement: those arms broadcast no evidence, so the
# A1-vs-A2 byte rates above are a real A/B of what the defence costs on the wire.
tel = []       # (sim_t, observer, peer, resid, l_self, l_total, abstain)
for r in st:
    for g in (r.get("tel") or "").split("|"):
        p = g.split(":")
        if len(p) == 5:
            tel.append((fnum(r["t"]), r["robot"], p[0], fnum(p[1]), fnum(p[2]),
                        fnum(p[3]), int(fnum(p[4], 0))))

# Attack onset in SIM time: the first sample whose residual is past anything clean flight ever
# produced (0.0890 m, calibration doc). Not the throttled COMPROMISED log line, which is wall
# clock, and not d_run.sh's own timestamp, which is the moment it ASKED for the lie.
CLEAN_MAX = 0.0890
onset = min((t for t, _o, p, r_, *_ in tel if p == victim and r_ > CLEAN_MAX),
            default=float("nan"))
# v_div: how fast claim and truth diverge. The harness injects a STEP, so this is a difference
# quotient across the step, not a ramp rate -- reported for the record and flagged as such.
vic_tel = sorted([x for x in tel if x[2] == victim], key=lambda x: x[0])
v_div = float("nan")
for a, b in zip(vic_tel, vic_tel[1:]):
    dt = b[0] - a[0]
    if dt > 1e-6:
        v_div = max(v_div if v_div == v_div else 0.0, abs(b[3] - a[3]) / dt)

# ---- detector attribution and eviction -----------------------------------------------------
def distinct(pat):
    return sorted({m for m in re.findall(pat, alog)})


blk_gate2 = distinct(r"\[admm_agent_(\d+)\]: \[agent\d+\] REJECTED AgentState from robot%s"
                     r" — gate2 odom residual" % victim)
blk_belief = distinct(r"\[admm_agent_(\d+)\]: \[agent\d+\] REJECTED AgentState from robot%s"
                      r" — belief threshold" % victim)
blk_roster = distinct(r"\[admm_agent_(\d+)\]: \[agent\d+\] REJECTED AgentState from robot%s"
                      r" — left the consensus" % victim)
evicted = distinct(r"\[admm_agent_(\d+)\]: \[agent\d+\] EVICT robot%s" % victim)
# Any block of a SURVIVOR is a false positive -- the thing the whole detector exists to avoid.
false_pos = sorted({(o, t) for o, t in
                    re.findall(r"\[admm_agent_(\d+)\]: \[agent\d+\] REJECTED AgentState from "
                               r"robot(\d+) — (?:gate2 odom residual|belief threshold)", alog)
                    if t != victim})
wbc = glog.count("Deactivating")
nopath = alog.count("A* found NO path")
obs_dropped = sum(int(fnum(r.get("n_obs_dropped"), 0)) for r in st)
refuted = sum(int(fnum(r.get("n_refuted"), 0)) for r in st)
checked = sum(int(fnum(r.get("n_evidence_checked"), 0)) for r in st)


def f(x, n=3):
    return "nan" if x != x else ("%.*f" % (n, x))


print("run              %s" % os.path.basename(LOGD))
print("arm/lie/seed     %s / %s m / %s      verdict=%s%s" %
      (arm, lie or "-", seed, verdict, ("  cause=" + cause) if cause else ""))
print("burst skew       %s s" % (skew or "n/a (no dual-channel burst)"))
print("arrival dist     %s m" % (arrive or "n/a"))
print("BODY GAP surv    %s m  (pair %s)   <- contact criterion, every row" %
      (f(sv_gap, 4), sv_gap_pair))
print("BODY GAP victim  %s m  (pair %s)" % (f(vic_gap, 4), vic_gap_pair))
print("centre surv      %s m  (pair %s)   [proxy only]" % (f(sv_centre, 4), sv_centre_pair))
print("centre all       %s m  (pair %s)   [proxy only]" % (f(all_centre, 4), all_centre_pair))
print("achieved_rounds  min=%s mean=%s n=%d" %
      (f(min(ar), 0) if ar else "nan", f(sum(ar) / len(ar), 2) if ar else "nan", len(ar)))
print("WBC deactivate   %d      A* no-path %d      obs channel drops %d" % (wbc, nopath, obs_dropped))
print("blocked victim   gate2=%s belief=%s roster=%s   EVICT=%s" %
      (",".join(blk_gate2) or "-", ",".join(blk_belief) or "-",
       ",".join(blk_roster) or "-", ",".join(evicted) or "-"))
print("FALSE POSITIVES  %s" % (", ".join("agent%s blocked robot%s" % x for x in false_pos) or "none"))
print("refutations      %d refuted / %d checked" % (refuted, checked))
print("attack onset     t=%s s (sim)   v_div=%s m/s  [STEP injection: difference quotient, "
      "not a ramp rate]" % (f(onset, 2), f(v_div, 3)))
for o in sorted(tx_rate):
    print("comm robot%s      tx=%s B/s  rx=%s B/s  (sim span %.1f s)" %
          (o, f(tx_rate[o], 0), f(rx_rate[o], 0),
           per_robot_span[o][1] - per_robot_span[o][0]))
print("belief telemetry %d per-(slot,observer,peer) samples%s" %
      (len(tel), "" if tel else "   <- none: beliefStep never ran (A0/A1 broadcast no evidence)"))

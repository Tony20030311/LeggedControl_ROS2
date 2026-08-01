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

# ---- what the survivors actually fenced (the lie's second effect) ---------------------------
# A lie does not only shrink clearance -- it MOVES the corpse keep-out, and a keep-out is a
# no-go disc the fleet's own goal can end up inside. The EVICT line prints the anchor and the
# radius corpse_keepout chose, so both come from the agents rather than being re-derived here.
#
# "ghost offset" is the CLOSEST the fenced disc's centre ever came to the victim's true body,
# minimised over every dist.csv row. Deliberately a minimum over the whole run and not a value
# at the eviction instant: that would need a wall-to-sim conversion, and a lower bound is the
# stronger claim anyway -- if even the closest approach is large, the fence never covered the
# body at any point. Conservative anchoring should drive this toward zero.
ko = re.findall(r"EVICT robot%s .*?corpse CBF at predicted rest \(([-\d.]+),([-\d.]+)\) r=([\d.]+)"
                % victim, alog)
ROBOT_MARGIN = 0.60      # admm_agent_node.cpp's robot_margin, added on top of the keep-out radius
ko_anchor = (float(ko[0][0]), float(ko[0][1])) if ko else None
ko_r = float(ko[0][2]) + ROBOT_MARGIN if ko else float("nan")
goal_m = re.search(r"outbound formation goal = \(([-\d.]+),([-\d.]+)\)", dlog)
goal = (float(goal_m.group(1)), float(goal_m.group(2))) if goal_m else None
goal_to_anchor = math.dist(goal, ko_anchor) if (goal and ko_anchor) else float("nan")
ghost = float("nan")
if ko_anchor and d and victim:
    try:
        ghost = min(math.hypot(fnum(r["x%s" % victim]) - ko_anchor[0],
                               fnum(r["y%s" % victim]) - ko_anchor[1]) for r in d)
    except KeyError:
        pass

# ---- consensus health, comm cost, belief telemetry -----------------------------------------
# bytes_tx/bytes_rx are read-and-RESET by take_bytes() at every publishStats, so each row is
# the traffic since the previous row: summing them and dividing by the sim span is the honest
# per-agent rate. Summing the raw column as if it were cumulative would be wrong.
st = rows("stats.csv")
ar = [fnum(r["achieved_rounds"]) for r in st if r.get("achieved_rounds")]
# Rate from INTERVALS, not from first-to-last span. take_bytes() exchanges the counters to zero
# at every publishStats, so each row's bytes belong to the gap since that robot's PREVIOUS row --
# which makes the honest denominator the sum of those gaps, and makes the first row (everything
# since node start, i.e. all of bring-up) unusable as a numerator.
#
# It also has to survive holes in the recording. g5_logger's subscriptions do not all establish
# at once: in a2 rep 1 robot1 and robot2 had a 37.5 s hole while robot3's stream simply started
# at 37.6 s, so a first-to-last span read 124.2 s for two robots and 86.6 s for the third and
# put robot3's apparent rate 1.4x above robot1's for no physical reason. Intervals longer than
# 1 s are dropped from both numerator and denominator: across a hole the counter did keep
# accumulating in the node, but the row that closes it is one publish's worth, not the hole's.
per_robot = {}
for r in st:
    rid = r.get("robot")
    if rid is not None:
        per_robot.setdefault(rid, []).append(
            (fnum(r["t"]), fnum(r["bytes_tx"], 0), fnum(r["bytes_rx"], 0)))
tx_rate, rx_rate, comm_window = {}, {}, {}
for rid, seq in per_robot.items():
    seq.sort()
    tx = rx = dur = 0.0
    for (ta, _, _), (tb, btx, brx) in zip(seq, seq[1:]):
        dt = tb - ta
        if 0 < dt <= 1.0:
            tx += btx
            rx += brx
            dur += dt
    if dur > 0:
        tx_rate[rid], rx_rate[rid], comm_window[rid] = tx / dur, rx / dur, dur

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

# Attack onset in SIM time, per observer: the first of ONSET_RUN consecutive samples whose
# residual is past anything clean flight produced (0.0890 m, calibration doc). Not the throttled
# COMPROMISED log line, which is wall clock, and not d_run.sh's own timestamp, which is the
# moment it ASKED for the lie.
#
# CONSECUTIVE, not a single sample, and the reason is measured rather than precautionary: in
# a2 rep 2 agent3 recorded one isolated residual of 0.1160 at t=66.35 with its neighbours at
# 0.012-0.026, i.e. 2.9 s before the attack, which a single-sample rule read as onset and turned
# agent3's true 0.5 s eviction latency into a reported 3.37 s -- a fabricated 6.7x spread
# between two observers of the same attack. The belief layer itself convicts only on a run of
# consecutive full penalties, so requiring a run here matches what it is measuring.
CLEAN_MAX = 0.0890
ONSET_RUN = 3


def onset_of(seq):
    """First t starting a run of ONSET_RUN consecutive over-threshold residuals."""
    hits = 0
    for t, r_ in seq:
        if r_ == r_ and r_ > CLEAN_MAX:
            hits += 1
            if hits == ONSET_RUN:
                return t - (ONSET_RUN - 1) * 0.1
        else:
            hits = 0
    return None


onset = min((o for o in
             (onset_of(sorted([(x[0], x[3]) for x in tel if x[2] == victim and x[1] == ob]))
              for ob in {x[1] for x in tel}) if o is not None), default=float("nan"))
# v_div: how fast claim and truth diverge. The harness injects a STEP, so this is a difference
# quotient across the step, not a ramp rate -- reported for the record and flagged as such.
#
# PER OBSERVER, and skipping abstained rows. Pooling both observers into one time-ordered list
# differences observer A's residual against observer B's at nearly the same timestamp, so the
# denominator is the two agents' publish skew (~1 ms) and the numerator is their independent
# noise draws: that read 25.1 m/s on a clean NO_KILL run with no attacker at all. A residual is
# a property of one observer's view; only consecutive samples from the SAME observer are a rate.
#
# TWO numbers, because the obvious one is useless on its own. `v_div_max` (the largest quotient
# anywhere in the run) has a NOISE FLOOR: consecutive residuals are independent Rayleigh(sigma)
# draws one slot apart, so a clean NO_KILL run with no attacker measured 0.726 m/s -- 2.3x the
# 0.31 m/s design envelope. Comparing that statistic to the envelope would "exceed" it on every
# honest run ever recorded. `v_div_step` is the quotient across the onset transition itself,
# which is what the injection actually does.
v_div_max = float("nan")
v_div_step = float("nan")
for obs_id in {x[1] for x in tel}:
    seq = sorted([x for x in tel if x[2] == victim and x[1] == obs_id and x[3] == x[3]],
                 key=lambda x: x[0])
    # Anchored on the SAME confirmed onset time-to-evict uses, not on the first single sample to
    # cross the threshold. Anchoring on a bare crossing picked up the isolated 0.1160 m noise
    # spike described above and reported v_div_step = 0.306 m/s -- a number that looks like it
    # sits just inside the 0.31 m/s design envelope while measuring nothing but noise.
    t_on = onset_of([(x[0], x[3]) for x in seq])
    for a, b in zip(seq, seq[1:]):
        dt = b[0] - a[0]
        if dt < 0.05:    # one slot is 0.1 s; anything shorter is a duplicate publish, not a step
            continue
        q = abs(b[3] - a[3]) / dt
        v_div_max = q if v_div_max != v_div_max else max(v_div_max, q)
        if t_on is not None and abs(b[0] - t_on) < 0.05 and v_div_step != v_div_step:
            v_div_step = q

# TIME-TO-EVICT, per survivor, in SIM time, exactly (spec §9 protocol item 5). Both ends come
# from CycleStats, which carries the sim clock -- so no wall-to-sim conversion and no RTF
# estimate enters this number. Onset is that observer's first non-abstained residual past
# anything clean flight produced; the block is its first row where the peer reads abstain=1
# (peer_blocked), which beliefStep writes only after block_peer has actually latched.
#
# Only arms that run beliefStep (obs_gate2=true) can report this. A0/A1 have no belief
# telemetry at all and their attacker leaves via roster-exclusion, a different mechanism on a
# different clock -- reporting an RTF-converted wall delta beside an exact sim figure would
# invite exactly the comparison it cannot support, so those arms report "n/a (no belief path)".
tte = {}
for obs_id in {x[1] for x in tel}:
    seq = sorted([x for x in tel if x[2] == victim and x[1] == obs_id], key=lambda x: x[0])
    t_on = onset_of([(x[0], x[3]) for x in seq])
    t_blk = next((t for t, _o, _p, _r, _ls, _lt, ab in seq if ab == 1 and (t_on is None or t >= t_on)),
                 None)
    if t_on is not None and t_blk is not None:
        tte[obs_id] = t_blk - t_on

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
if ko_anchor:
    print("keep-out         anchored (%.2f,%.2f) r_eff=%s m; GHOST OFFSET %s m (closest the fenced"
          " disc ever came to the real body)" % (ko_anchor[0], ko_anchor[1], f(ko_r, 2), f(ghost, 3)))
    print("mission denial   goal is %s m from that anchor -> %s the keep-out" %
          (f(goal_to_anchor, 3),
           "INSIDE (goal unreachable by construction)" if goal_to_anchor < ko_r else "outside"))
print("refutations      %d refuted / %d checked" % (refuted, checked))
print("time-to-evict    %s   [sim seconds, both ends from CycleStats' own clock]" %
      (", ".join("agent%s %ss (%.0f slots)" % (o, f(v, 2), round(v / 0.1)) for o, v in sorted(tte.items()))
       or "n/a (no belief path in this arm)"))
print("attack onset     t=%s s (sim)   v_div_step=%s m/s   v_div_max=%s m/s "
      "[STEP injection: quotients, not ramp rates; v_div_max floor on a clean run is 0.726]"
      % (f(onset, 2), f(v_div_step, 3), f(v_div_max, 3)))
for o in sorted(tx_rate):
    print("comm robot%s      tx=%s B/s  rx=%s B/s  (over %.1f s of recorded intervals)" %
          (o, f(tx_rate[o], 0), f(rx_rate[o], 0), comm_window[o]))
print("belief telemetry %d per-(slot,observer,peer) samples%s" %
      (len(tel), "" if tel else "   <- none: beliefStep never ran (A0/A1 broadcast no evidence)"))

# ---- occlusion (spec §9 protocol item 6): a POSITIVE record, never an absence of log lines ----
# Reported for every arm that has belief telemetry, not only the occl probe: incidental occlusion
# happens in the peg field and a reader is entitled to see it. The acceptance question is
# specifically about the LONGEST window in which EVERY observer abstained with code 5, because
# one blind observer is the ordinary case and proves nothing.
ABSTAIN = {0: "evidence", 1: "peer_blocked", 2: "no_fresh_claim", 3: "no_obs_buffer",
           4: "out_of_range", 5: "occluded"}
if tel:
    observers = sorted({x[1] for x in tel})
    by_t = {}
    for t, o, p, _r, l_self, _lt, ab in tel:
        if p == victim:
            by_t.setdefault(round(t, 1), {})[o] = (ab, l_self)
    best, cur = [], []
    for t in sorted(by_t):
        v = by_t[t]
        if len(v) == len(observers) and all(a == 5 for a, _ in v.values()):
            cur.append((t, v))
        else:
            if len(cur) > len(best):
                best = cur
            cur = []
    if len(cur) > len(best):
        best = cur
    hist = {}
    for _t, o, p, _r, _ls, _lt, ab in tel:
        if p == victim:
            hist[ABSTAIN[ab]] = hist.get(ABSTAIN[ab], 0) + 1
    print("abstain codes    %s   [about robot%s, every observer, whole run]" % (hist, victim))
    if best:
        span = best[-1][0] - best[0][0]
        moved = {o: max(abs(w[o][1] - best[0][1][o][1]) for _t, w in best) for o in observers}
        print("OCCLUSION window %.2f s (%d slots) with EVERY observer at code 5; |dL_self| over it: "
              "%s   <- must be 0.000: suspicion does not decay while unobservable" %
              (span, len(best), ", ".join("agent%s %.3f" % (o, m) for o, m in sorted(moved.items()))))
    else:
        print("OCCLUSION window none (no slot had every observer reporting code 5)")

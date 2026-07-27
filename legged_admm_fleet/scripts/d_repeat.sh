#!/bin/bash
# Repeat the experiment-D scenario N times and summarise. One run proves the mechanism works;
# N runs are what tells you whether the SAFETY MARGIN is stable or whether run 1 was lucky —
# the plum-density sweep already showed dog-dog clearance swinging 0.128..0.46 m across trials.
#
#   ./scripts/d_repeat.sh [N] [victim]
# Env: ARENA, GOAL_X, KILL_AT_X, RECORD_RUN (which repetition records video, default 1), REJOIN.
WS=/root/legged_ros2_ws
SCRIPTS=$WS/src/legged_fleet/legged_admm_fleet/scripts
N=${1:-5}
VICTIM=${2:-2}
RECORD_RUN=${RECORD_RUN:-1}
SUMD=$WS/g2_logs/drepeat_$(date +%m%d_%H%M%S)
mkdir -p "$SUMD"
SUM="$SUMD/summary.csv"
echo "run,logdir,result,evicted_by,corpse_r,anchor_err,min_survivor_corpse_parked,min_pair_all,astar_nopath,crossed_both_ways" > "$SUM"

for i in $(seq 1 "$N"); do
  echo "=== repetition $i/$N ==="
  REC=0; [ "$i" = "$RECORD_RUN" ] && REC=1
  RECORD=$REC bash "$SCRIPTS/d_run.sh" "$VICTIM" > "$SUMD/run$i.out" 2>&1
  RC=$?
  # newest d_* log dir belongs to the run that just finished
  D=$(ls -dt $WS/g2_logs/d_* 2>/dev/null | head -1)
  RES=PASS; [ "$RC" = 0 ] || RES=FAIL
  python3 - "$i" "$D" "$RES" "$SUM" <<'PY'
import csv, math, os, re, sys
i, D, res, out = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4]
def blank(reason):
    with open(out, "a") as f: f.write(f"{i},{D},{res},,,,,,,{reason}\n")
try:
    rows = list(csv.DictReader(open(os.path.join(D, "dist.csv"))))
except Exception:
    blank("no-dist-csv"); sys.exit()
if len(rows) < 10:
    blank("too-few-samples"); sys.exit()
P = lambda r, s: (float(r["x" + s]), float(r["y" + s]))
log = open(os.path.join(D, "d.log"), errors="replace").read()
adm = ""
try: adm = open(os.path.join(D, "admm.log"), errors="replace").read()
except Exception: pass
n_ev = len(re.findall(r"EVICT robot", adm))
m = re.search(r"corpse CBF at predicted rest \((-?[\d.]+),(-?[\d.]+)\) r=([\d.]+)", adm)
ax, ay, cr = (float(m.group(1)), float(m.group(2)), float(m.group(3))) if m else (None, None, None)
# survivors are whoever is not the victim; victim id from the log line
vm = re.search(r"victim=robot(\d+)", log)
vic = vm.group(1) if vm else "2"
surv = [s for s in ("1", "2", "3") if s != vic]
fx, fy = P(rows[-1], vic)
moved = [k for k, r in enumerate(rows) if math.hypot(*(a - b for a, b in zip(P(r, vic), (fx, fy)))) > 0.05]
i_stop = (max(moved) + 1) if moved else 0
parked = rows[i_stop:]
best = min((math.hypot(*(a - b for a, b in zip(P(r, s), P(r, vic)))) for r in parked for s in surv), default=float("nan"))
mp = min(float(r["min_pair"]) for r in rows)
xs = [float(r["x" + surv[0]]) for r in rows]
both = "yes" if (min(xs) < 4.0 and max(xs) > 16.0) else ("n/a-empty-world" if max(xs) < 10 else "no")
anch = f"{math.hypot(fx - ax, fy - ay):.2f}" if ax is not None else ""
with open(out, "a") as f:
    f.write(f"{i},{D},{res},{n_ev},{cr if cr is not None else ''},{anch},"
            f"{best:.3f},{mp:.3f},{adm.count('A* found NO path')},{both}\n")
PY
  tail -1 "$SUM"
done

echo
echo "=== summary: $SUM ==="
column -s, -t < "$SUM"

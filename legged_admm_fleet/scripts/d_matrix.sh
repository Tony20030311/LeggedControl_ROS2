#!/bin/bash
# Coverage matrix for experiment D. d_repeat.sh answers "is this point stable?"; this answers
# "have we only ever tested ONE point?" — which we had: victim=2, killed at x=7, plum, every time.
#
# The axes matter for different reasons:
#   victim    the V has a leader and two followers; killing the leader leaves a different
#             geometry (and a different COL2 re-assignment) than killing a follower
#   kill x    4 = entering the field, 7 = mid, 11 = deep, 14 = about to exit. Where the corpse
#             lands decides which lane it blocks and how boxed-in the survivors are
#   arena     plum_dense has 2.20 m diagonal gaps vs plum's 2.30 — the measured dog-dog bottleneck
#   goal      a return target far outside the fleet's own bounding box is the ONLY thing that
#             exercises the "don't rebuild the planner" fix (T1.4); every run so far came home
#             to the start, which sits inside the box either way
#
#   ./scripts/d_matrix.sh            run the default matrix
#   CASES="victim1 deep dense" ...   run a subset by name
WS=/root/legged_ros2_ws
SCRIPTS=$WS/src/legged_fleet/legged_admm_fleet/scripts
OUT=$WS/g2_logs/dmatrix_$(date +%m%d_%H%M%S)
mkdir -p "$OUT"
SUM="$OUT/matrix.csv"
echo "case,result,logdir,evicted_by,min_survivor_corpse_parked,min_pair_all,astar_nopath,crossed_both_ways" > "$SUM"

# name|env overrides
ALL=(
  "victim1|VICTIM=1 KILL_AT_X=7.0"                    # kill the V leader, not a follower
  "victim3|VICTIM=3 KILL_AT_X=7.0"                    # the other follower (mirror of the baseline)
  "early|VICTIM=2 KILL_AT_X=4.0"                      # dies entering the field
  "deep|VICTIM=2 KILL_AT_X=11.0"                      # dies deep inside the quincunx
  "late|VICTIM=2 KILL_AT_X=14.0"                      # dies one row from the exit
  # 2.20 m gaps, the measured dog-dog bottleneck. Held as a STRICT case: it was written off as
  # "needs Phase 3 convoy, two abreast cannot fit 2.20/2 - 0.30 - 0.60 = 0.20 m", but the premise
  # was wrong — two survivors re-form as COL2 {+-0.75, 0}, which is single file already. It has
  # since walked both legs clean twice (d_0728_033054, d_0728_044753), so anything less than
  # reaching the goal here is a regression, not a geometry limit. (d_run.sh still understands
  # EXPECT=stall; that is for the 2.10 m stress point, where the fleet genuinely wedges.)
  "dense|VICTIM=2 KILL_AT_X=7.0 ARENA=plum_dense"
  "farhome|VICTIM=2 KILL_AT_X=7.0 HOME_OVERRIDE=-5.0" # return target outside the fleet bbox (T1.4)
)

want="${CASES:-}"
for entry in "${ALL[@]}"; do
  name="${entry%%|*}"; envs="${entry#*|}"
  [ -n "$want" ] && case " $want " in *" $name "*) ;; *) continue;; esac
  echo "=== case $name : $envs ==="
  ( export ARENA=plum GOAL_X=18.0 RECORD=0; eval "export $envs"
    bash "$SCRIPTS/d_run.sh" "${VICTIM:-2}" ) > "$OUT/$name.out" 2>&1
  RC=$?
  D=$(ls -dt $WS/g2_logs/d_* 2>/dev/null | head -1)
  RES=PASS; [ "$RC" = 0 ] || RES=FAIL
  # A confirmed safe stall is its own verdict, not a PASS — it must stay visible in the matrix.
  grep -q "D STALL-SAFE" "$OUT/$name.out" && RES=STALL-SAFE
  python3 - "$name" "$D" "$RES" "$SUM" <<'PY'
import csv, math, os, re, sys
name, D, res, out = sys.argv[1:5]
def w(*f): open(out, "a").write(",".join(str(x) for x in f) + "\n")
try:
    rows = list(csv.DictReader(open(os.path.join(D, "dist.csv"))))
except Exception:
    w(name, res, D, "", "", "", "", "no-csv"); sys.exit()
if len(rows) < 10:
    w(name, res, D, "", "", "", "", "few-samples"); sys.exit()
P = lambda r, s: (float(r["x" + s]), float(r["y" + s]))
log = open(os.path.join(D, "d.log"), errors="replace").read()
adm = open(os.path.join(D, "admm.log"), errors="replace").read() if os.path.exists(os.path.join(D, "admm.log")) else ""
vic = (re.search(r"victim=robot(\d+)", log) or [None, "2"])[1]
surv = [s for s in "123" if s != vic]
fx, fy = P(rows[-1], vic)
moved = [k for k, r in enumerate(rows) if math.hypot(*(a - b for a, b in zip(P(r, vic), (fx, fy)))) > 0.05]
parked = rows[(max(moved) + 1) if moved else 0:]
best = min((math.hypot(*(a - b for a, b in zip(P(r, s), P(r, vic)))) for r in parked for s in surv), default=float("nan"))
xs = [float(r["x" + surv[0]]) for r in rows]
w(name, res, D, len(re.findall(r"EVICT robot", adm)), f"{best:.3f}",
  f"{min(float(r['min_pair']) for r in rows):.3f}", adm.count("A* found NO path"),
  "yes" if (min(xs) < 4.0 and max(xs) > 16.0) else "no")
PY
  tail -1 "$SUM"
done
echo; echo "=== $SUM ==="; column -s, -t < "$SUM"

#!/bin/bash
# Task 12 experiment matrix driver: run a list of arms n times each, unattended, and leave one
# machine-readable line per ATTEMPT behind.
#
#   ARMS="a0 a1 a2" N=3 ./trust_matrix.sh
#   ARMS="smear duty" N=3 SMEAR_TARGET=1 ./trust_matrix.sh
#
# Every attempt is recorded, including the ones that abort -- spec §9 protocol item 7 pre-registers
# that exclusions cover infrastructure failure ONLY, and a driver that quietly retries until it
# gets a clean set is how a matrix ends up reporting a survivorship-biased success rate. d_run.sh
# already distinguishes the two cases by exit code (2 = die_infra, 1 = the experiment produced
# this outcome), so that distinction is recorded rather than re-judged here.
#
# No re-runs are issued automatically, not even for exit 2. A lab flake that repeats is a finding.
set -u
WS=${WS:-/root/legged_ros2_ws}
SCRIPTS=$WS/src/legged_fleet/legged_admm_fleet/scripts
OUT=${OUT:-$WS/g2_logs/matrix_$(date +%m%d_%H%M%S)}
mkdir -p "$OUT"
LEDGER=$OUT/attempts.tsv
[ -s "$LEDGER" ] || printf 'attempt\tarm\trep\texit\tclass\tlogdir\tverdict\n' > "$LEDGER"

ARMS=${ARMS:-"a0 a1 a2"}
N=${N:-3}
# plum arena, 3 dogs -- spec §9's registered scenario. GOAL_X=9 is the distance calibration run B
# completed in, so the walk itself is known-good; KILL_AT_X stages the attack with field left to
# route through afterwards.
export ARENA=${ARENA:-plum}
export GOAL_X=${GOAL_X:-9}
export KILL_AT_X=${KILL_AT_X:-3.0}
export LIE=${LIE:-0.30}
VICTIM=${VICTIM:-2}

i=0
for arm in $ARMS; do
  for rep in $(seq 1 "$N"); do
    i=$((i + 1))
    echo "=== attempt $i: ARM=$arm rep $rep/$N ==="
    BEFORE=$(ls -dt "$WS"/g2_logs/d_*/ 2>/dev/null | head -1)
    ARM=$arm "$SCRIPTS/d_run.sh" "$VICTIM" 2>&1 | tail -40
    rc=${PIPESTATUS[0]}
    case "$rc" in
      0) class=complete ;;
      2) class=infra ;;      # the only pre-registered exclusion class
      *) class=outcome ;;    # the experiment ran and produced this; it stays in the table
    esac
    D=$(ls -dt "$WS"/g2_logs/d_*/ 2>/dev/null | head -1)
    [ "$D" = "$BEFORE" ] && D="(no log dir created)"
    v=$(grep -oE "D (PASS|STALL-SAFE)|FAIL(\(infra\))?: .*" "$D/d.log" 2>/dev/null | tail -1)
    printf '%d\t%s\t%d\t%d\t%s\t%s\t%s\n' \
      "$i" "$arm" "$rep" "$rc" "$class" "$(basename "$D")" "${v:-unknown}" >> "$LEDGER"
    if [ -f "$D/d.log" ]; then
      { echo "########## attempt $i  ARM=$arm  rep=$rep  exit=$rc ($class)"
        python3 "$SCRIPTS/trust_summary.py" "$D" 2>&1
        echo; } >> "$OUT/summaries.txt"
    fi
    echo "--- attempt $i done: exit=$rc ($class) $D"
  done
done
echo "=== matrix done -> $OUT ==="
cat "$LEDGER"

#!/bin/bash
# Experiment D — communication-loss failure semantics, end to end in Gazebo.
#
#   ./scripts/d_run.sh [victim]          scenario A: victim goes silent and never comes back
#   REJOIN=1 ./scripts/d_run.sh [victim] scenario B: victim comes back mid-run and rejoins
#
# The victim is silenced with `kill -STOP` (resumed with -CONT), which freezes ONLY the ADMM
# agent process. Its lower-level controller keeps running, so the dog finishes the trajectory
# prefix it was last handed and parks — exactly the "stuck in the middle of the field" case the
# survivors' keep-out circle assumes. -CONT then reproduces a link coming back, with the agent's
# internal state intact (a restart would instead be a cold start, a different failure mode).
#
# Env: ROBOTS, V, DEADLINE, ARENA (e.g. plum_dense), GOAL_X, DMIN_ABORT, REJOIN.
WS=/root/legged_ros2_ws
TAG=d
LOGD=$WS/g2_logs/d_$(date +%m%d_%H%M%S)
mkdir -p "$LOGD"
source $WS/src/legged_fleet/legged_admm_fleet/scripts/_fleet_bringup.sh

VICTIM=${1:-2}
GOAL_X=${GOAL_X:-3.0}
# 0.90, not 0.95: a legitimate detour around the corpse was measured at 0.93 and 0.95 killed
# the run. Contact is 0.867 (2x base half-diagonal 0.433).
DMIN_ABORT=${DMIN_ABORT:-0.90}
REJOIN=${REJOIN:-0}
SURVIVORS=$(echo $ROBOTS | tr ' ' '\n' | grep -vx "$VICTIM" | tr '\n' ' ')

grep_agents() { grep -h "$1" "$LOGD/admm.log" 2>/dev/null; }
count_agents() { grep_agents "$1" | wc -l; }

# Wait until $2 agents have logged pattern $1, or $3 seconds elapse. Echoes the count.
wait_for_log() {
  local pat="$1" want="$2" secs="$3"
  for i in $(seq 1 "$secs"); do
    [ "$(count_agents "$pat")" -ge "$want" ] && break
    sleep 1
  done
  count_agents "$pat"
}

# EXPECT=stall: this scenario is known to be geometrically infeasible (plum_dense in two-dog
# degraded mode — 0.20 m of free corridor half-width against a 1.30 m effective corpse radius).
# Not reaching the goal is then the ACCEPTED outcome, but only if an independent detector says
# the fleet is genuinely wedged. "The script gave up" is not evidence; stall_detect.py reads the
# trajectory and is allowed to answer "it was still moving", in which case this still fails.
# Everything else stays strict: both survivors must evict, min_pair must hold, WBC must not drop.
EXPECT=${EXPECT:-reach}
STALL_T=${STALL_T:-15}
# $1,$2 = goal, $3 = robots. Echoes the verdict; returns 0 only for a CONFIRMED safe stall.
confirm_stall() {
  say "EXPECT=stall: asking stall_detect.py whether robots '$3' are genuinely wedged"
  python3 "$SCRIPTS/stall_detect.py" "$LOGD/dist.csv" \
    --robots "$(echo $3 | tr ' ' ',')" --goal "$1,$2" --T "$STALL_T" 2>&1 | tee -a "$LOGD/$TAG.log"
  return "${PIPESTATUS[0]}"
}

walk_until() {  # $1 = goal x, $2 = goal y, $3 = robots to watch, $4 = max polls
  # `local` is load-bearing: without it this clobbered the caller's saved home coordinates and
  # the return leg became "walk to where you already are" — a PASS that tested nothing.
  local wx wy D M i BEST=99 STALE=0
  for i in $(seq 1 "$4"); do
    read wx wy <<< "$(fleet_centroid "$3")"
    D=$(python3 -c "import math;print(f'{math.hypot($wx-$1, $wy-$2):.3f}')")
    M=$(min_pair)
    say "  centroid=($wx,$wy) dist=$D min_pair=$M"
    [ "$(python3 -c "print(1 if $M<$DMIN_ABORT else 0)")" = 1 ] && die "pairwise $M < $DMIN_ABORT (collision guard)"
    gz_deactivated && die "WBC deactivated mid-walk"
    [ "$(python3 -c "print(1 if $D<0.45 else 0)")" = 1 ] && return 0
    # A SLOT can be inside the corpse keep-out, and then the centroid can never reach the goal.
    # Measured (late kill at x=15.3): corpse at (16.16,-0.55), COL2 rear slot at (17.24,-0.02) is
    # 1.20 m away against an effective 1.30 m (r 0.70 + robot_margin 0.60) -- unreachable BY
    # DESIGN. The fleet parked at 0.48 m for 2.5 min, safe and still tracking; hand-sending the
    # next goal made it walk off immediately, so this is arrival, not a freeze.
    # The bound is derived, not tuned: one blocked dog is displaced by at most the effective
    # radius 1.30 m, which moves a two-dog centroid by at most half of that. Past 0.65 m no
    # blocked slot explains it and the fleet really is stuck -- keep failing there.
    if [ "$(python3 -c "print(1 if $BEST-$D > 0.03 else 0)")" = 1 ]; then BEST=$D; STALE=0
    else STALE=$((STALE + 1)); fi
    if [ "$STALE" -ge 6 ] && [ "$(python3 -c "print(1 if $D<0.65 else 0)")" = 1 ]; then
      say "  converged at dist=$D (>0.45) with no progress for ${STALE} polls — a slot is inside a"
      say "  keep-out; accepting as arrived (bound 0.65 = half the 1.30 m effective corpse radius)"
      return 0
    fi
    sleep 5
  done
  return 1
}

say "victim=robot$VICTIM survivors='$SURVIVORS' rejoin=$REJOIN arena='${ARENA:-none}'"
fleet_bringup

# ---------- optional recording (RECORD=1) ----------
# Overhead arena_cam sensor -> ros_gz_bridge -> g5_video.py (cv2 mp4). No GUI, all on :99.
# Only worlds with an arena_cam have this, i.e. ARENA must be set.
RECPID=""; BRPID=""
if [ "${RECORD:-0}" = 1 ]; then
  if [ -z "$ARENA" ]; then
    say "RECORD=1 ignored: the empty world has no arena_cam sensor"
  else
    say "recording arena_cam -> demo.mp4"
    setsid ros2 run ros_gz_bridge parameter_bridge \
      /arena_cam/image@sensor_msgs/msg/Image@gz.msgs.Image >"$LOGD/bridge.log" 2>&1 &
    BRPID=$!
    sleep 3
    setsid python3 $SCRIPTS/g5_video.py /arena_cam/image "$LOGD/demo.mp4" 30 \
      >"$LOGD/g5_video.log" 2>&1 &
    RECPID=$!
  fi
fi
stop_recording() {
  [ -n "$RECPID" ] || return 0
  sleep 3                       # let the last few frames land
  kill -INT "$RECPID" 2>/dev/null; sleep 4   # SIGINT so cv2 VideoWriter.release() runs
  kill -9 "$BRPID" 2>/dev/null
  # cv2.VideoWriter only ever writes MPEG-4 Part 2 (fourcc mp4v) — browsers and most players
  # refuse it, so the raw demo.mp4 looks broken. Transcode to H.264/yuv420p + faststart, which
  # plays everywhere. Keep the original: it is the lossless-ish source if the clip gets re-cut.
  if command -v ffmpeg >/dev/null 2>&1 && [ -s "$LOGD/demo.mp4" ]; then
    ffmpeg -v error -y -i "$LOGD/demo.mp4" -c:v libx264 -preset medium -crf 23 \
      -pix_fmt yuv420p -movflags +faststart "$LOGD/demo_h264.mp4" >"$LOGD/ffmpeg.log" 2>&1 \
      && say "video: $LOGD/demo_h264.mp4 ($(du -h "$LOGD/demo_h264.mp4" 2>/dev/null | cut -f1)), source demo.mp4" \
      || say "video: $LOGD/demo.mp4 (H.264 transcode failed, see ffmpeg.log)"
  else
    say "video: $LOGD/demo.mp4 ($(du -h "$LOGD/demo.mp4" 2>/dev/null | cut -f1))"
  fi
}

# ---------- phase 4: trot + walk out ----------
say "phase 4: trot + /formation/goal centroid+$GOAL_X"
arm_trot
read HX HY <<< "$(fleet_centroid "$ROBOTS")"
GX=$(python3 -c "print(f'{$HX+$GOAL_X:.3f}')"); GY=$HY
say "home=($HX,$HY) outbound formation goal = ($GX,$GY)"
send_formation_goal "$GX" "$GY"
# WHERE the victim dies decides what gets tested, so trigger on position, not a timer: a
# fixed sleep depends on RTF and made the first run kill next to the goal, so nothing had to
# route around anything. KILL_AT_X=0 falls back to the timer.
KILL_AT_X=${KILL_AT_X:-0}
if [ "$(python3 -c "print(1 if $KILL_AT_X>0 else 0)")" = 1 ]; then
  say "waiting for fleet centroid to reach x=$KILL_AT_X before silencing robot$VICTIM"
  KX_OK=0
  for i in $(seq 1 120); do
    read kx ky <<< "$(fleet_centroid "$ROBOTS")"
    [ "$(python3 -c "print(1 if $kx>=$KILL_AT_X else 0)")" = 1 ] && { KX_OK=1; break; }
    gz_deactivated && die "WBC deactivated before the kill point"
    sleep 3
  done
  [ "$KX_OK" = 1 ] || die "fleet never reached x=$KILL_AT_X (last centroid x=$kx)"
  say "fleet centroid at x=$kx — killing now"
else
  sleep ${KILL_AFTER:-7}
fi

# ---------- phase 5: silence the victim (NO_KILL=1 skips: no-failure regression run) ----------
# The whole point of NO_KILL is the OTHER half of the contract: with nobody silenced the fleet
# must behave exactly as it did before any of the failure machinery existed — three dogs, V
# shape, and crucially NO spurious eviction. So this branch asserts EVICT never appears.
if [ "${NO_KILL:-0}" = 1 ]; then
  say "phase 5: SKIPPED (NO_KILL=1) — nobody silenced; verifying undisturbed 3-dog operation"
  say "phase 6: all three continue to ($GX,$GY)"
  walk_until "$GX" "$GY" "$ROBOTS" 60 || die "fleet never reached the outbound goal"
  N_EV=$(count_agents "EVICT robot")
  [ "$N_EV" = 0 ] || { grep_agents "EVICT robot" | sed 's/^/    /'; die "spurious EVICT x$N_EV with nobody silenced"; }
  N_RB=$(count_agents "REBUILD")
  [ "$N_RB" = 0 ] || { grep_agents "REBUILD" | sed 's/^/    /'; die "spurious REBUILD x$N_RB with nobody silenced"; }
  say "no eviction, no rebuild — fleet undisturbed"
  N_EVICT=0
  WATCH="$ROBOTS"
else

# pgrep -f on the NODE NAME: ros2 launch passes parameters via a temp --params-file, so
# robot_id never appears on the command line; only the `-r __node:=admm_agent_N` remap does.
# pgrep -f is read-only and safe here; `pkill -f` is banned (it matches this shell).
PIDV=$(pgrep -f "admm_agent_$VICTIM" | head -1)
[ -n "$PIDV" ] || die "cannot find admm_agent_$VICTIM (try: pgrep -af admm_agent)"
# LIE=<metres>: experiment C instead of experiment D. The victim is not silenced — it is
# COMPROMISED. Two knobs, because a real attacker has both: it broadcasts a position offset by
# LIE metres (sender side, so every survivor judges the same bytes), and it is steered into the
# formation with its own per-dog goal. A liar that keeps walking its honest plan is a
# "cooperative liar" and never tests the thing that matters — being pushed away by the CBF.
if [ -n "${LIE:-}" ]; then
  say "phase 5: COMPROMISE admm_agent_$VICTIM — lie=${LIE}m, steering it into the fleet"
  read CX CY <<< "$(fleet_centroid "$SURVIVORS")"
  # The daemon caches the node graph and goes stale across a fleet restart; a param set against
  # a stale daemon fails with no useful message. Bounce it first (measured: this is exactly why
  # the first scripted run of this scenario died at "inject_fake_offset failed").
  ros2 daemon stop >/dev/null 2>&1; sleep 2
  timeout 12 ros2 param set /admm_agent_$VICTIM detection_log_only false >/dev/null 2>&1
  timeout 12 ros2 param set /admm_agent_$VICTIM inject_fake_offset "$LIE" \
    || die "inject_fake_offset failed (is the param declared?)"
  # arm the DETECTORS on the survivors, not on the liar
  for j in $SURVIVORS; do
    timeout 12 ros2 param set /admm_agent_$j detection_log_only false >/dev/null 2>&1 \
      || die "could not arm Gate 2 on agent$j"
  done
  # walk it at the survivors' centroid
  # transient_local + a short burst: same discovery-race lesson as send_formation_goal.
  timeout 8 ros2 topic pub -r 5 --qos-durability transient_local \
    /robot$VICTIM/goal geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: world}, pose: {position: {x: $CX, y: $CY, z: 0.5}}}" >/dev/null 2>&1
else
say "phase 5: SIGSTOP admm_agent_$VICTIM (pid $PIDV)"
kill -STOP "$PIDV" || die "SIGSTOP failed"
fi

N_EVICT=$(wait_for_log "EVICT robot$VICTIM" 2 60)
[ "$N_EVICT" -ge 2 ] || die "only $N_EVICT survivor(s) evicted robot$VICTIM within 60s"
say "EVICT seen on $N_EVICT survivor(s)"
grep_agents "EVICT robot$VICTIM" | sed 's/^/    /' | tee -a "$LOGD/$TAG.log"
grep_agents "REBUILD" | tail -2 | sed 's/^/    /' | tee -a "$LOGD/$TAG.log"

# survivors must keep going to the ORIGINAL goal, routing around the corpse
say "phase 6: survivors continue to ($GX,$GY)"
if ! walk_until "$GX" "$GY" "$SURVIVORS" 60; then
  [ "$EXPECT" = stall ] || die "survivors never reached the outbound goal"
  # Geometrically infeasible by design — but prove it rather than assume it. The collision
  # guard and the WBC check inside walk_until already ran on every poll and did not fire.
  confirm_stall "$GX" "$GY" "$SURVIVORS" \
    || die "EXPECT=stall but the detector found no stall — the fleet was moving, so this is a plain miss"
  stop_recording
  python3 "$SCRIPTS/dist_summary.py" "$LOGD/dist.csv" \
    || die "collision guard produced no data — STALL-SAFE not claimable"
  say "D STALL-SAFE: victim=robot$VICTIM, evicted by $N_EVICT, survivors wedged without collision; logs $LOGD"
  exit 0
fi
say "survivors reached the outbound goal with robot$VICTIM down"

if [ "$REJOIN" = 1 ]; then
  # ---------- phase 7: victim comes back (scenario B, needs the Phase-2 view-change) ----------
  say "phase 7: SIGCONT admm_agent_$VICTIM — expecting rejoin"
  kill -CONT "$PIDV" || die "SIGCONT failed"
  N_RE=$(wait_for_log "REJOIN robot$VICTIM" 2 60)
  if [ "$N_RE" -lt 2 ]; then
    say "NOTE: no REJOIN on $((2 - N_RE)) survivor(s) — rejoin is Phase 2, not yet implemented?"
    die "rejoin not observed within 60s"
  fi
  say "REJOIN seen on $N_RE survivor(s)"
  grep_agents "corpse of robot$VICTIM removed" | sed 's/^/    /' | tee -a "$LOGD/$TAG.log"
  WATCH="$ROBOTS"
else
  WATCH="$SURVIVORS"
fi

fi   # end NO_KILL branch

# ---------- phase 8: NEW goal back home — the real test of the degraded/restored fleet ----------
# HOME_OVERRIDE pushes the return target outside the fleet's own bounding box, which is the
# only thing that exercises "don't rebuild the planner on eviction" (T1.4): the old code swapped
# the arena-sized A* bounds for a box around the dogs, so a later far goal became unplannable.
[ -n "${HOME_OVERRIDE:-}" ] && HX="$HOME_OVERRIDE"
say "phase 8: new /formation/goal back HOME to ($HX,$HY) — must route around the corpse"
send_formation_goal "$HX" "$HY"
sleep 3
say "current /formation/plan:"
timeout 6 ros2 topic echo /formation/plan --once 2>/dev/null | sed 's/^/    /' | tee -a "$LOGD/$TAG.log"
N_PLAN=$(timeout 6 ros2 topic echo /formation/plan --once 2>/dev/null | python3 -c "
import sys,re
t=sys.stdin.read()
m=re.search(r'robot_ids:\n((?:- \d+\n)*)', t)
print(len(m.group(1).split()) // 2 if m else 0)" 2>/dev/null)
N_WANT=$(echo $WATCH | wc -w)
say "plan covers ${N_PLAN:-?} robot(s) (expect $N_WANT)"
[ "${N_PLAN:-0}" = "$N_WANT" ] || die "FleetPlan covers ${N_PLAN:-?} robot(s), expected $N_WANT"

walk_until "$HX" "$HY" "$WATCH" 90 || die "fleet never got home"

# A straight-line fallback means A* found no route; that is the silent precursor to a frozen dog.
NOPATH=$(count_agents "A\* found NO path")
[ "$NOPATH" = 0 ] || say "WARNING: $NOPATH A* failure(s) — check $LOGD/admm.log"

stop_recording
python3 "$SCRIPTS/dist_summary.py" "$LOGD/dist.csv" \
  || die "collision guard produced no data — PASS not claimable"
say "D PASS (rejoin=$REJOIN): victim=robot$VICTIM, evicted by $N_EVICT, home reached; logs $LOGD"

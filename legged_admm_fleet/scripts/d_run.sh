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
# NOT `local` inside walk_until: the caller prints how close the fleet actually got, not just
# whether it crossed 0.45 (real arrival) or the 0.65 keep-out-blocked convergence (only a defended
# arm plants a keep-out that can trip that one) -- a number is comparable across arms, a boolean
# that can mean two different things is not.
ARRIVE_DIST=99
# Set by walk_until when the goal turns out to be inside a corpse keep-out. A COUNTED OUTCOME,
# not something to be inferred later from a failure message: "the lie moved a 1.63 m exclusion
# disc onto our own goal" is a result the table has to be able to tally.
MISSION_DENIED=0
REJOIN=${REJOIN:-0}
SURVIVORS=$(echo $ROBOTS | tr ' ' '\n' | grep -vx "$VICTIM" | tr '\n' ' ')

# Task 11 item 6: the arm selector. Which detector is armed and whether it blocks are BRING-UP
# (launch-time) decisions, not runtime ones -- obs_gate2 is deliberately absent from
# admm_agent_node.cpp's runtime param callback allowlist (design spec section 9 protocol item 1:
# A1's mechanism must survive the whole matrix unchanged, so which detector runs cannot itself
# flip mid-run). detection_log_only IS runtime-settable (kept that way for LIE_LOGONLY's pre-arm
# sequence below), so this is only its bring-up default.
ARM=${ARM:-a1}
case "$ARM" in
  a0)         OBS_GATE2=false ;;   # no defence at all (log-only forced below)
  a1)         OBS_GATE2=false ;;   # self-report gate: gate2()'s single-shot EMA odom residual
  a2)         OBS_GATE2=true  ;;   # observation belief layer (this spec)
  smear)      OBS_GATE2=true  ;;   # A2 + attacker fabricates a sighting of an honest peer
  duty)       OBS_GATE2=true  ;;   # A2 + attacker lies only every other window (inject_duty_cycle)
  forged_obs) OBS_GATE2=true  ;;   # A2 + attacker also impersonates a peer's observation channel
  # The occlusion probe (spec §9 protocol item 6 / acceptance 4). A2's detector, but log-only:
  # the attacker has to SURVIVE long enough to be walked behind a pillar, and A2 convicts in
  # 5 slots. Slowing the lie instead is not an option -- the graded band is 5.33 mm wide, so
  # any lie a noise floor of sigma=0.02 does not swallow already earns the full -2.0 penalty.
  # Nor can the probe use an honest peer: an unsuspected peer sits at the +4.6 ceiling, and a
  # saturated value is "numerically unchanged" whether or not evidence is arriving, so it cannot
  # discriminate. The peer must be MID-DESCENT when it hides, which needs blocking disarmed.
  occl)       OBS_GATE2=true  ;;
  *) die_infra "unknown ARM='$ARM' (want a0|a1|a2|smear|duty|forged_obs|occl)" ;;
esac
# a0 must survive its own breach (design spec protocol item 3): its own detector still computes
# and logs a verdict, but is forced never to act on it, for the whole run, not just from the
# trigger -- the same "measure before you block" default admm_agent_node.cpp ships if unset.
# NOTE: there is no EXPECT_EVICT here anymore (review round 2, C1/C2) -- see the eviction-check
# block below for why "will an EVICT line appear" is not a per-arm question at all.
if [ "$ARM" = a0 ] || [ "$ARM" = occl ]; then DETECT_LOG_ONLY=true
else DETECT_LOG_ONLY=false; fi
# Deafness is the DEFAULT because a liar that still runs its own avoidance is a "cooperative
# liar" and the separation that survives is then partly its doing. Two arms need it off, and in
# both cases it is a property of what the arm measures, not a convenience:
#   smear -- a deafened attacker's peer_xnow() empties, so its outgoing ev_peer array is empty,
#            and applyFakeEvidence corrupts a sighting ALREADY IN the message rather than
#            fabricating one. The smear channel would transmit nothing and the arm would report
#            "no honest peer was evicted" for entirely the wrong reason.
#   occl  -- deafness makes the attacker announce a roster excluding the survivors, they block
#            it as "left the consensus" (a path detection_log_only does not gate), evidence
#            stops with abstain=1, and there is nothing left to observe going behind a pillar.
# Overridable, so the LIE_DEAF=0 control arm can also be run against a0/a2 (see the eviction
# block below for why that configuration is the only genuinely undefended control).
case "$ARM" in
  smear|occl) LIE_DEAF=${LIE_DEAF:-0} ;;
  *)          LIE_DEAF=${LIE_DEAF:-1} ;;
esac
# forged_obs, FIRST-MOVER case (FORGED_FIRST=1). Task 10 item 4: one knob, two cases separated
# only by WHEN it is set. Set at runtime (the default path, in the arm_attack.py dispatch) the
# attacker arrives after the real Gazebo publisher and the GID pin rejects it -- the defence's
# coverage case. Given at CONSTRUCTION it publishes first, WINS the pin, and the genuine
# publisher is the one dropped. That second outcome is the honest limit of trust-on-first-use
# and has to be run and reported, not designed around.
if [ "${FORGED_FIRST:-0}" = 1 ]; then
  [ "$ARM" = forged_obs ] || die_infra "FORGED_FIRST=1 only means anything with ARM=forged_obs"
  LAUNCH_EXTRA="forged_obs_attacker:=$VICTIM forged_obs_target:=${FORGED_TARGET:-1} $LAUNCH_EXTRA"
fi
# Task 11 item 7: must vary per run, or a false-positive/refutation-rate measurement is n=1 in
# the only dimension that produces one (admm::obs_noise). $RANDOM is bash's own PRNG.
OBS_NOISE_SEED=${OBS_NOISE_SEED:-$RANDOM}
LAUNCH_EXTRA="obs_gate2:=$OBS_GATE2 detection_log_only:=$DETECT_LOG_ONLY obs_noise_seed:=$OBS_NOISE_SEED $LAUNCH_EXTRA"

grep_agents() { grep -h "$1" "$LOGD/admm.log" 2>/dev/null; }
count_agents() { grep_agents "$1" | wc -l; }
# EVICT/REJOIN lines carry their origin as rclcpp's own `[admm_agent_N]` prefix. A single agent
# can log the same pattern more than once (re-evicting as its corpse-CBF estimate updates), so a
# LINE count can satisfy a "2 independent agents agreed" assertion off ONE agent alone -- dedup
# on the agent tag instead of counting lines.
count_distinct_agents() { grep_agents "$1" | grep -oE '\[admm_agent_[0-9]+\]' | sort -u | wc -l; }

# die_infra: the results protocol says only an infrastructure failure justifies a re-run. A
# distinct exit code lets a sweep across six arms tell "the lab flaked" (2) from "the experiment
# produced this outcome" (1, ordinary die) without a human reading every log.
die_infra() { say "FAIL(infra): $*"; exit 2; }

# Wait until $2 DISTINCT agents have logged pattern $1, or $3 seconds elapse. Echoes the count.
wait_for_log() {
  local pat="$1" want="$2" secs="$3"
  for i in $(seq 1 "$secs"); do
    [ "$(count_distinct_agents "$pat")" -ge "$want" ] && break
    sleep 1
  done
  count_distinct_agents "$pat"
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

# Worst centre distance among a SUBSET of robots, over EVERY row of dist.csv. Two departures
# from min_pair (col 8), both deliberate:
#   subset  — with a live, deaf attacker being steered into the fleet, the all-robot minimum
#             measures how hard IT rammed, not whether the defence worked.
#   history — min_pair is sampled once per ~8 s poll, so an excursion shorter than the poll
#             slips past it; dist_summary.py carries the same warning after a plum run reached
#             0.526 m for 4.3 s and still printed PASS. Every row, or the number is fiction.
# `ros2 param set` is a round trip through the daemon's rebuilt node graph, and in the plum world
# with the recorder attached it intermittently exceeds the timeout — which killed a 10-minute run
# at the exact moment of injection (2026-07-29, "could not arm Gate 2 on agent2"). One slow
# discovery must not decide whether the experiment happens, so retry before giving up.
pset() {  # $1 = node, $2 = param, $3 = value
  local i
  for i in 1 2 3; do
    timeout 20 ros2 param set "$1" "$2" "$3" >/dev/null 2>&1 && return 0
    say "  param set $1 $2 failed (attempt $i/3) — retrying"
    sleep 1
  done
  return 1
}

pair_among() {  # $@ = robot ids
  python3 - "$LOGD/dist.csv" "$@" <<'PY' 2>/dev/null || echo 99
import sys, csv, math, itertools
rows = list(csv.DictReader(open(sys.argv[1])))
ids = [int(a) for a in sys.argv[2:]]
pairs = list(itertools.combinations(ids, 2))
if not rows or not pairs:
    print(99); sys.exit()
print('%.4f' % min(math.hypot(float(r['x%d' % i]) - float(r['x%d' % j]),
                              float(r['y%d' % i]) - float(r['y%d' % j]))
                   for r in rows for i, j in pairs))
PY
}

# Is the commanded goal inside a corpse keep-out this run actually built? Echoes "anchor r_eff"
# and returns 0 when it is. Both numbers come from the EVICT line the AGENTS logged ("corpse CBF
# at predicted rest (x,y) r=1.03"), so the only constant duplicated here is robot_margin, which
# the agent adds on top of that radius.
ROBOT_MARGIN=${ROBOT_MARGIN:-0.60}   # admm_agent_node.cpp's robot_margin (swing-foot FK envelope)
goal_fenced() {  # $1,$2 = goal
  python3 - "$LOGD/admm.log" "$1" "$2" "$ROBOT_MARGIN" <<'PY' 2>/dev/null
import math, re, sys
txt = open(sys.argv[1], errors="replace").read()
gx, gy, margin = float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
for x, y, r in re.findall(r"corpse CBF at predicted rest \(([-\d.]+),([-\d.]+)\) r=([\d.]+)", txt):
    reff = float(r) + margin
    if math.hypot(float(x) - gx, float(y) - gy) < reff:
        print("anchor (%s,%s) r_eff %.2f" % (x, y, reff))
        sys.exit(0)
sys.exit(1)
PY
}

# How far the CENTROID can legitimately sit from the goal once everyone is on station. A
# formation has extent: a dog cannot be on its slot and on the goal at the same time, so with
# COL2 (slots 1.5 m apart along the path) the centroid lands on the goal only if both dogs
# straddle it exactly, and any lag leaves a residual no amount of walking removes. Echoes that
# half-extent, computed from the slots the coordinator ACTUALLY PUBLISHED -- no constant, and it
# tracks whatever shape the live count resolved to. 0 when no plan is readable, which is strict.
form_half_extent() {
  timeout 6 ros2 topic echo /formation/plan --once 2>/dev/null | python3 -c "
import re, sys, math
t = sys.stdin.read()
m = re.search(r'goals:\n((?:- [-\d.e]+\n)*)', t)
v = [float(x) for x in re.findall(r'- ([-\d.e]+)', m.group(1))] if m else []
p = list(zip(v[::2], v[1::2]))
if not p:
    print('0'); sys.exit()
cx, cy = sum(a for a, _ in p) / len(p), sum(b for _, b in p) / len(p)
print('%.3f' % max(math.hypot(a - cx, b - cy) for a, b in p))" 2>/dev/null || echo 0
}

walk_until() {  # $1 = goal x, $2 = goal y, $3 = robots to watch, $4 = max polls
  # `local` is load-bearing: without it this clobbered the caller's saved home coordinates and
  # the return leg became "walk to where you already are" — a PASS that tested nothing.
  local wx wy D M i BEST=99 STALE=0
  # Read once, not per poll: it is a topic echo, and the shape does not change mid-leg.
  local HALF=$(form_half_extent)
  for i in $(seq 1 "$4"); do
    read wx wy <<< "$(fleet_centroid "$3")"
    D=$(python3 -c "import math;print(f'{math.hypot($wx-$1, $wy-$2):.3f}')")
    M=$(min_pair)
    say "  centroid=($wx,$wy) dist=$D min_pair=$M"
    ARRIVE_DIST=$D   # global (see declaration) -- covers every return below, success or timeout
    if [ "$(python3 -c "print(1 if $M<$DMIN_ABORT else 0)")" = 1 ]; then
      # LIE_CHASE joins LIE_LOGONLY here for the same reason: with a hostile robot deliberately
      # driven into the fleet, the all-robot minimum measures how hard IT rammed, and failing the
      # run on it fails the experiment for the ATTACKER doing its job (measured 0.64 m, aborted a
      # complete take). The survivors hitting EACH OTHER is still a hard stop, below.
      if [ "${LIE_LOGONLY:-0}" = 1 ] || [ "${LIE_CHASE:-0}" = 1 ]; then
        # THE COUNTERFACTUAL ARM IS SUPPOSED TO LOSE SEPARATION. Its survivors are fencing a
        # ghost 2.83 m from the compromised robot's real body, so that body closing on them is
        # the measurement this arm exists to produce (0.557 m, measured 2026-07-29) — aborting
        # would throw away the result and leave the arm permanently reporting FAIL.
        # What must still abort: the two SURVIVORS hitting each other. That is a real collision
        # in any arm, and it is the exact shape of the false-positive failure this whole detector
        # has to avoid, so it keeps the hard guard.
        local S=$(pair_among $SURVIVORS)
        [ "$(python3 -c "print(1 if $S<$DMIN_ABORT else 0)")" = 1 ] \
          && die "survivor pair $S < $DMIN_ABORT — survivors collided, not the expected breach"
        say "  BREACH $M (expected: survivors hold the ghost, the real body closes in; survivors $S)"
      else
        die "pairwise $M < $DMIN_ABORT (collision guard)"
      fi
    fi
    # NOT die_infra: this fires on every poll of every walk_until call, most of which happen
    # AFTER the lie/silence has landed. gz_deactivated only sees "Deactivating" in gazebo.log --
    # it cannot tell the pre-existing upstream activation race from an attacker's ram genuinely
    # destabilising a robot hard enough to trip the WBC. Cannot distinguish -> fail closed.
    gz_deactivated && die "WBC deactivated mid-walk"
    [ "$(python3 -c "print(1 if $D<0.45 else 0)")" = 1 ] && return 0
    # A GOAL can be inside the corpse keep-out, and then the centroid can never reach it.
    # Measured (late kill at x=15.3): corpse at (16.16,-0.55), COL2 rear slot at (17.24,-0.02) is
    # 1.20 m away against an effective 1.30 m -- unreachable BY DESIGN. The fleet parked at
    # 0.48 m for 2.5 min, safe and still tracking; hand-sending the next goal made it walk off
    # immediately, so this is arrival, not a freeze.
    #
    # This used to be a distance BOUND: accept below 0.65 m, "half the 1.30 m effective corpse
    # radius". Two things were wrong with it. 1.30 is the STATIC corpse (D_MIN - robot_margin
    # = 0.70, plus the agent's own 0.60); a peer evicted for LYING is still walking, so
    # corpse_keepout adds MAX_VX*TAU_REACT and the radius is 1.63 -- every LIE arm was being
    # judged by the wrong number. And the corrected bound would have been 0.815 against a run
    # that finished at 0.820 (d_0801_041928), which is a 5 mm miss and a sign that a worst-case
    # bound is the wrong SHAPE of question entirely.
    #
    # So ask the question the run can actually answer. The agents log the anchor and the radius
    # they chose; the goal is right here. If the goal is inside the disc it is unreachable, no
    # threshold required, and if it is not then no amount of stalling should be forgiven. One
    # rule, read off this run's own geometry, with the constant deleted rather than re-derived.
    if [ "$(python3 -c "print(1 if $BEST-$D > 0.03 else 0)")" = 1 ]; then BEST=$D; STALE=0
    else STALE=$((STALE + 1)); fi
    if [ "$STALE" -ge 6 ]; then
      # Two things the run's own geometry can explain, either of which means "as arrived as this
      # fleet is going to get". Neither is a tuned threshold: the first is the published slot
      # geometry, the second is the keep-out the agents logged.
      if FENCE=$(goal_fenced "$1" "$2"); then
        say "  no progress for ${STALE} polls and the goal ($1,$2) is INSIDE a corpse keep-out"
        say "  [$FENCE] — unreachable by construction. MISSION DENIED BY THE FENCE; accepting as"
        say "  converged at dist=$D so the run still reports its safety numbers."
        MISSION_DENIED=1
        return 0
      fi
      if [ "$(python3 -c "print(1 if $D < ${HALF:-0} else 0)")" = 1 ]; then
        say "  no progress for ${STALE} polls at dist=$D, within the formation's own half-extent"
        say "  ${HALF} m (from the published slots) — everyone is on station and the centroid"
        say "  cannot get closer. Accepting as arrived."
        return 0
      fi
    fi
    sleep 5
  done
  return 1
}

say "victim=robot$VICTIM survivors='$SURVIVORS' rejoin=$REJOIN arena='${ARENA:-none}'"
say "ARM=$ARM -> obs_gate2=$OBS_GATE2 detection_log_only(bring-up default)=$DETECT_LOG_ONLY obs_noise_seed=$OBS_NOISE_SEED"
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
    sleep 3   # let the bridge advertise before the recorder subscribes (discovery margin)
    setsid python3 $SCRIPTS/g5_video.py /arena_cam/image "$LOGD/demo.mp4" 30 \
      >"$LOGD/g5_video.log" 2>&1 &
    RECPID=$!
  fi
fi
# ---------- optional RViz capture (RVIZ=1) ----------
# Ported from arena_run.sh, and for this scenario it is not a nicety: a lie has no body, so the
# overhead camera records survivors swerving away from empty ground with no visible cause.
# fleet_viz_markers.py draws the missing half (ghost / evidence line / status) and RViz is the
# only place those live. rviz2 gets its OWN Xvfb (:98, software GL) so it never fights the gz
# GUI on :99; ffmpeg x11grabs :98.
# TRUE body clearance, not centre distance. dist.csv logs centre-to-centre against an isotropic
# 0.867 m "contact line" (2 x the corner envelope), which answers a different question from the
# one everybody actually asks about this scenario — did the bodies touch. This uses each dog's
# real yaw and the 0.83 x 0.25 m base box, so a near-miss is reported as the surface gap it is.
# The empty third arg is load-bearing: argv[3] is the optional "x,y;x,y" pile list, so passing
# --ros-args straight into that slot makes the logger die on startup parsing it as coordinates —
# silently, because it is backgrounded and only the summary line at the end goes missing.
setsid python3 $SCRIPTS/phys_gap_logger.py "$ROBOTS" "$LOGD/phys_gap.csv" "" \
  --ros-args -p use_sim_time:=true >"$LOGD/phys_gap.log" 2>&1 &
PHYSPID=$!

# Task 11 items 1-3: g5_logger.py's stats.csv is CycleStats already published every cycle and
# already recorded (per-cycle, not the 1 Hz throttled RCLCPP_*_THROTTLE log lines) -- it now also
# carries n_obs_dropped/n_refuted/tel (see g5_logger.py), the unthrottled per-peer belief
# telemetry the calibration procedure (obs_sigma, refute_ratio) reads from. traj.csv is unused
# here but comes from the same node at no extra cost.
setsid python3 $SCRIPTS/g5_logger.py "$ROBOTS" "$LOGD" \
  --ros-args -p use_sim_time:=true >"$LOGD/g5_logger.log" 2>&1 &
G5PID=$!

RVIZPID=""; RVGRABPID=""; VIZPID=""; CHASEPID=""
# Without this, a `die` anywhere below leaves rviz2, the x11grab and the marker node running —
# they outlive the script, hold :98, and the next run's capture fails for no visible reason.
# $DLPID (the dist logger) belongs here too: _fleet_bringup.sh's `trap ... EXIT` inside
# fleet_bringup() gets silently REPLACED by this one (bash traps don't stack, the last `trap`
# call for a given signal wins), so without adding it here $DLPID was never actually killed by
# anything -- it kept running past every d_run.sh exit, its 20 Hz timer re-writing the last
# cached pose forever with no liveness check, corrupting that run's dist.csv for anyone who
# re-read it later (found via review round 2's unreproducible peer-distance figures).
trap 'kill -9 $RVIZPID $RVGRABPID $VIZPID $RECPID $BRPID $PHYSPID $G5PID $CHASEPID $DLPID 2>/dev/null' EXIT
if [ "${RVIZ:-0}" = 1 ]; then
  say "recording RViz -> rviz.mp4"
  RVIZ_CFG=$WS/src/legged_fleet/legged_admm_fleet/rviz/fleet.rviz
  # Recreate :98 at EXACTLY the grab size: a leftover :98 of another size makes x11grab fail
  # "Invalid argument" and the whole capture silently produces nothing.
  # A previous run's rviz2/grabber/marker node survive `die` (this script has no trap and the
  # shared bringup cleanup does not know about them), and a stale grabber holds :98 open.
  pkill -9 -x rviz2 2>/dev/null; pkill -9 -x ffmpeg 2>/dev/null
  pkill -9 -f fleet_viz_markers 2>/dev/null   # comm is python3; -x cannot reach it, and this
                                             # pattern cannot match our own `bash d_run.sh` line
  pkill -9 -f "Xvfb :98" 2>/dev/null; sleep 1
  Xvfb :98 -screen 0 1600x1000x24 >/tmp/xvfb98.log 2>&1 & sleep 2
  setsid python3 $SCRIPTS/fleet_viz_markers.py "$ROBOTS" "${ARENA:-plum}" \
    --ros-args -p use_sim_time:=true >"$LOGD/fleet_viz.log" 2>&1 &
  VIZPID=$!
  DISPLAY=:98 LIBGL_ALWAYS_SOFTWARE=1 QT_QPA_PLATFORM=xcb setsid \
    rviz2 -d "$RVIZ_CFG" --ros-args -p use_sim_time:=true >"$LOGD/rviz2.log" 2>&1 &
  RVIZPID=$!
  sleep 15   # let rviz2 open and build the GL scene before grabbing
  setsid ffmpeg -y -f x11grab -video_size 1600x1000 -framerate 20 -i :98 \
    -c:v libx264 -pix_fmt yuv420p -movflags +faststart "$LOGD/rviz.mp4" \
    >"$LOGD/rviz_grab.log" 2>&1 &
  RVGRABPID=$!
fi

stop_recording() {
  if [ -n "$RVGRABPID" ]; then
    sleep 2
    kill -INT "$RVGRABPID" 2>/dev/null; sleep 3   # SIGINT so ffmpeg writes the moov atom
    kill -9 "$RVIZPID" "$VIZPID" 2>/dev/null
    [ -s "$LOGD/rviz.mp4" ] && say "rviz: $LOGD/rviz.mp4 ($(du -h "$LOGD/rviz.mp4" | cut -f1))" \
      || say "rviz: capture produced nothing — see rviz_grab.log"
  fi
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
# PRE-ARM BEFORE THE GOAL IS SENT, so the fleet is standing still through it. It used to run
# after send_formation_goal, "while the fleet walks up to the trigger point" -- but the block
# costs 10-30 s of `ros2 param set` round trips, and at 0.4 m/s the fleet covers that ground
# before the KILL_AT_X wait loop even starts sampling: d_0801_040522 asked for x=3.0 and
# triggered at x=5.766. Where the attack lands then depends on RTF, which is not comparable
# across arms. Nothing here changes behaviour, so paying for it at a standstill costs only
# wall-clock and makes KILL_AT_X mean what it says.
KILL_AT_X=${KILL_AT_X:-0}
if [ -n "${LIE:-}" ]; then
  ros2 daemon stop >/dev/null 2>&1; sleep 4
  PIDV=$(pgrep -f "admm_agent_$VICTIM" | head -1)
  if [ "${LIE_CHASE:-0}" = 1 ]; then
    pset /admm_agent_$VICTIM inject_ignore_corpses true \
      || say "  (inject_ignore_corpses unavailable — the attacker will swerve around them)"
  fi
  if [ "${LIE_LOGONLY:-0}" = 1 ]; then
    for j in $SURVIVORS; do
      pset /admm_agent_$j detection_log_only true || die_infra "could not disarm Gate 2 on agent$j"
    done
  fi
  say "pre-armed: the whole attack now costs one arm_attack.py dispatch at the trigger"
fi
send_formation_goal "$GX" "$GY"
if [ "$(python3 -c "print(1 if $KILL_AT_X>0 else 0)")" = 1 ]; then
  say "waiting for fleet centroid to reach x=$KILL_AT_X before silencing robot$VICTIM"
  KX_OK=0
  for i in $(seq 1 120); do
    read kx ky <<< "$(fleet_centroid "$ROBOTS")"
    [ "$(python3 -c "print(1 if $kx>=$KILL_AT_X else 0)")" = 1 ] && { KX_OK=1; break; }
    gz_deactivated && die_infra "WBC deactivated before the kill point"
    sleep 1
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
  walk_until "$GX" "$GY" "$ROBOTS" 60 || die "fleet never reached the outbound goal (arrive_dist=$ARRIVE_DIST)"
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
[ -n "$PIDV" ] || die_infra "cannot find admm_agent_$VICTIM (try: pgrep -af admm_agent)"
# LIE=<metres>: experiment C instead of experiment D. The victim is not silenced — it is
# COMPROMISED. Two knobs, because a real attacker has both: it broadcasts a position offset by
# LIE metres (sender side, so every survivor judges the same bytes), and it is steered into the
# formation with its own per-dog goal. A liar that keeps walking its honest plan is a
# "cooperative liar" and never tests the thing that matters — being pushed away by the CBF.
if [ -n "${LIE:-}" ]; then
  say "phase 5: COMPROMISE admm_agent_$VICTIM — lie=${LIE}m, steering it into the fleet"
  read CX CY <<< "$(fleet_centroid "$SURVIVORS")"
  # ORDER MATTERS, and getting it wrong killed the whole scenario once. Each `ros2 param set` is
  # a round trip and the three of them span ~4 s. Deafening FIRST gave the attacker those 4 s
  # with no detector armed anywhere: it heard nobody, timed out, evicted both survivors, rebuilt
  # itself solo, and a solo cold start broadcasts reset=true — which the survivors obeyed,
  # dropping their warm start. From there nobody could ever solve again (they are cold and see a
  # warm peer -> announce-only HOLD forever) and Gate 2, which used to run only on the solved
  # path, never executed once (d_0728_095209: "GATE2 would" x0, run timed out).
  #
  # So: detectors first, then the lie, then deafness. The lie needs ~1-2 slots to cross the gate
  # plus 3 slots to evict (~0.5 s), which comfortably beats the deafness clock (10 slots of
  # silence, ~1 s). Winning that race is also what pins the keep-out circle to the liar's odom:
  # the survivors block it for LYING, not for having gone quiet.
  #
  # arm the DETECTORS on the survivors, not on the liar.
  # LIE_LOGONLY=1 is the COUNTERFACTUAL arm: the survivors still compute the residual and log
  # it, but they never block, so the lie reaches the consensus and the CBF. Both arms therefore
  # have identical code, identical logging and one difference — whether the verdict is acted on.
  # That is what separates "detection worked" from "the CBF would have avoided it anyway": with
  # the lie accepted, the survivors keep 1.3 m from a GHOST and close on the real body.
  # EVERY `ros2 param set` here is ~3 s of round trip and the fleet KEEPS WALKING through all of
  # them: the handshake used to run 32 s, i.e. 13 m of ground, which put the attack two peg rows
  # downfield of where it was staged (d_0729_100815 — aimed at obs7, fired at obs12). So set only
  # what actually differs from the running configuration.
  # Detector arming and the hostile-mode knobs were PRE-ARMED before the trigger wait, so the lie
  # is the only round trip left here. Everything below happens while the fleet is where it was
  # staged instead of 8 m downfield.
  # Review round 2, I3': the smear arm is DEFINED as "the attacker's own position stays honest"
  # (see admm_agent_node.cpp's beliefStep comment on the smear check) -- it fabricates a sighting
  # of an honest peer instead. Sending inject_fake_offset unconditionally here made the smear
  # attacker ALSO lie about its own position, so a2 convicted it on the first-hand residual in
  # under a second: the transport blocks it, blocked reporters' relay entries are erased, and the
  # smear channel dies before it ever accumulates anything -- the arm would report "no honest
  # peer was evicted" for entirely the wrong reason and prove nothing about acceptance criterion
  # 5. Skip the position lie for smear; every other arm still sends it.
  # Spec §9 protocol item 2's other half: assert the detector has said NOTHING about the victim
  # before the attack exists. A pre-trigger flag means the arm was never measured -- whatever
  # blocked the attacker did so for a setup artefact, and both the time-to-evict and the A1-vs-A2
  # comparison built on it are fiction. Cheaper to catch here than to discover in the results.
  PRE_DET=$(count_agents "GATE2 would reject robot$VICTIM")
  PRE_BLK=$(count_agents "REJECTED AgentState from robot$VICTIM")
  [ "$((PRE_DET + PRE_BLK))" = 0 ] \
    || die_infra "detector flagged robot$VICTIM $((PRE_DET + PRE_BLK)) time(s) BEFORE the attack was armed — nothing measured after this point is attributable to the attack"
  # THE WHOLE ATTACK IN ONE DISPATCH. Every knob that defines this arm goes out from a single
  # rclpy process (arm_attack.py) that waits for all the parameter services and then fires all
  # the requests back to back, so the spread is service latency instead of process startup.
  #
  # Three separate `ros2 param set` calls do not work here even backgrounded: each pays its own
  # graph discovery, and three contending measured 2.78 s apart (d_0801_035833). In that gap
  # gate2() differences one forged channel against one honest one -- r = 0.424 m for LIE=0.30,
  # since the offset goes into x AND y -- against a 0.30 m gate, and agent1 blocked the attacker
  # 0.61 s in. Its "A1 caught the lie" was the harness, not the detector.
  #
  # --max-skew is a hard gate, not a report: past ~one slot the arm is measuring the race again,
  # and a silently-skewed run is indistinguishable in the results table from a clean one.
  SPECS=""
  case "$ARM" in
    smear)
      # Spec §9 A2-smear: this attacker's OWN position stays honest (so no claim lie and no odom
      # forgery -- sending them made a2 convict it on the first-hand residual in under a second,
      # erasing the blocked reporter's relay entries before the smear channel accumulated
      # anything). Target first, then the value: they are applied in array order and the value's
      # own log line names the target.
      SMEAR_TARGET=${SMEAR_TARGET:-${SURVIVORS%% *}}
      SPECS="/admm_agent_$VICTIM:inject_fake_evidence_target:int:$SMEAR_TARGET"
      SPECS="$SPECS /admm_agent_$VICTIM:inject_fake_evidence:double:$LIE"
      say "  smear: robot$VICTIM fabricates a ${LIE}m false sighting of robot$SMEAR_TARGET"
      ;;
    *)
      # DUAL-CHANNEL FORGERY (spec §9 protocol item 2). A real attacker owns its estimator as
      # well as its planner, so the claim lie alone is not the threat model: gate2() differences
      # the claim against /robotJ/controller/odom, which the same compromised process produces,
      # and an attacker that makes both agree leaves NO residual. That is exactly the attack A1
      # is structurally blind to and A2 has to catch, so every position-lying arm faces it.
      # _peer names WHICH peer's odom copy each survivor forges and must precede the offset in
      # the array: parameters are applied in order, and the offset's own log line names the peer.
      SPECS="/admm_agent_$VICTIM:inject_fake_offset:double:$LIE"
      for j in $SURVIVORS; do
        SPECS="$SPECS /admm_agent_$j:inject_odom_fake_peer:int:$VICTIM"
        SPECS="$SPECS /admm_agent_$j:inject_odom_fake:double:$LIE"
      done
      say "  dual-channel forgery: robot$VICTIM claims +${LIE}m AND every survivor's odom copy of it reads +${LIE}m"
      ;;
  esac
  case "$ARM" in
    duty)
      SPECS="$SPECS /admm_agent_$VICTIM:inject_duty_cycle:int:${DUTY_PERIOD:-2}"
      say "  duty: robot$VICTIM lies for the first half of every ${DUTY_PERIOD:-2}-slot window"
      ;;
    forged_obs)
      FORGED_TARGET=${FORGED_TARGET:-${SURVIVORS%% *}}
      if [ "${FORGED_FIRST:-0}" = 1 ]; then
        say "  forged_obs: robot$VICTIM's impersonating publisher was created at CONSTRUCTION"
        say "  (first-mover case) — it already holds the GID pin; nothing to fire here"
      else
        SPECS="$SPECS /admm_agent_$VICTIM:inject_forged_obs:int:$FORGED_TARGET"
        say "  forged_obs: robot$VICTIM impersonates robot$FORGED_TARGET's observation channel"
      fi
      ;;
  esac
  # DEAF as well as lying (LIE_DEAF=1, the default). A liar that still runs its own avoidance is
  # a "cooperative liar": the separation that survives is then partly ITS doing, and the demo
  # cannot claim the survivors kept themselves safe. Dropping all its incoming peer states makes
  # it genuinely non-cooperative -- it times out, evicts everyone, and walks solo to its goal.
  # In the same dispatch as everything else: staggering it re-opens the race that used to let the
  # attacker evict both survivors and rebuild solo before any detector was armed.
  [ "${LIE_DEAF:-1}" = 1 ] && SPECS="$SPECS /admm_agent_$VICTIM:inject_drop_p:double:1.0"
  # Attacker speed: OFF by default. Running the hunter at 0.55 against survivors capped at 0.4
  # meant they could not out-accelerate the closing rate -- the keep-out was violated faster than
  # a step could answer it, so the film showed a hit and no dodge. Set CHASE_V to bring it back.
  [ -n "${CHASE_V:-}" ] && SPECS="$SPECS /admm_agent_$VICTIM:v:double:$CHASE_V"
  python3 "$SCRIPTS/arm_attack.py" --max-skew="${MAX_SKEW:-0.20}" $SPECS 2>&1 \
    | tee -a "$LOGD/$TAG.log"
  [ "${PIPESTATUS[0]}" = 0 ] || die_infra "could not arm the attack in one dispatch (see the skew line above) — a staggered arming measures the harness, not the detector"
  # CHARGE NOW, not after the eviction is logged. Waiting for that cost ~10 s, and at 0.4 m/s the
  # attacker walks 4 m in it — past the very peg it was supposed to round, which is why three
  # takes filmed a stroll instead of an attack. The eviction lands ~1 s after the lie regardless.
  if [ "${LIE_CHASE:-0}" = 1 ]; then
    CHASE_TARGET=${CHASE_TARGET:-${SURVIVORS%% *}}
    say "phase 5b: robot$VICTIM charges robot$CHASE_TARGET (rounding whatever peg is in the way)"
    # The whole attacker arc — charge, break off, take station, go dark — runs HERE, on its own
    # clock. It used to be spliced into phase 6/6a/6b, which meant the charge lasted until the
    # SURVIVORS finished their outbound leg: ~2 minutes of chasing instead of a brief rush, and
    # the attacker was dragged right out of the peg field. What the attacker does must not depend
    # on how far the fleet still has to walk.
    (
      CHASE_END=$((SECONDS + ${CHASE_T:-20}))
      while [ "$SECONDS" -lt "$CHASE_END" ]; do
        # Aim 2.5 m PAST the quarry: a goal ON it makes the planner decelerate to arrive, which
        # reads as politely stopping alongside. Overshooting keeps it at speed THROUGH the
        # quarry's position, so the survivor's own keep-out is what has to move it.
        TXY=$(python3 - "$LOGD/dist.csv" "$VICTIM" "$CHASE_TARGET" <<'PY' 2>/dev/null
import csv, math, sys
rows = list(csv.DictReader(open(sys.argv[1])))
if not rows:
    print(); sys.exit()
r = rows[-1]
v, t = int(sys.argv[2]), int(sys.argv[3])
vx, vy = float(r['x%d' % v]), float(r['y%d' % v])
tx, ty = float(r['x%d' % t]), float(r['y%d' % t])
# Aim AT the quarry, overshooting so the planner never decelerates to arrive. Leading the target
# was tried and is worse, not better: aiming 2 m ahead of it plus a 1.5 m overshoot put the goal
# 3.5 m up the quarry's own track, so the attacker walked a parallel course and never turned in —
# closest approach 1.75 m and opening (measured d_0729_105720). Straight at the body, always.
d = math.hypot(tx - vx, ty - vy)
if d > 0.05:
    tx += (tx - vx) / d * 2.5
    ty += (ty - vy) / d * 2.5
print('%.3f %.3f' % (tx, ty))
PY
)
        read CX2 CY2 <<< "$TXY"
        case "$CX2" in ''|nan) sleep 1; continue;; esac
        timeout 2 ros2 topic pub -r 5 --qos-durability transient_local \
          /robot$VICTIM/goal geometry_msgs/msg/PoseStamped \
          "{header: {frame_id: world}, pose: {position: {x: $CX2, y: $CY2, z: 0.5}}}" >/dev/null 2>&1
        sleep 0.5
      done
      # BREAK OFF and take station. A brief rush then a roadblock is the scenario; an endless
      # pursuit is not, and it also drags the attacker out of the arena.
      RB=${ROADBLOCK:-"14.0 1.2"}
      say "  robot$VICTIM breaks off after ${CHASE_T:-20}s — walking to ($RB) to park"
      timeout 4 ros2 topic pub -r 5 --qos-durability transient_local \
        /robot$VICTIM/goal geometry_msgs/msg/PoseStamped \
        "{header: {frame_id: world}, pose: {position: {x: ${RB% *}, y: ${RB#* }, z: 0.5}}}" \
        >/dev/null 2>&1
      # Freeze it only once it ARRIVES: a timed guess froze it mid-stride, which reads as a crash
      # rather than as a robot taking up station.
      for _i in $(seq 1 "${PARK_T:-40}"); do
        RD=$(python3 - "$LOGD/dist.csv" "$VICTIM" ${RB% *} ${RB#* } <<'PY' 2>/dev/null
import csv, math, sys
rows = list(csv.DictReader(open(sys.argv[1])))
r = rows[-1] if rows else None
v = int(sys.argv[2])
print('%.2f' % math.hypot(float(r['x%d' % v]) - float(sys.argv[3]),
                          float(r['y%d' % v]) - float(sys.argv[4])) if r else 99)
PY
)
        [ "$(python3 -c "print(1 if ${RD:-99} < 0.7 else 0)")" = 1 ] && break
        sleep 2
      done
      say "  robot$VICTIM on station (${RD:-?} m) — going dark"
      kill -STOP "$PIDV" 2>/dev/null
    ) &
    CHASEPID=$!
  else
    # walk it at the survivors' centroid
    # transient_local + a short burst: same discovery-race lesson as send_formation_goal.
    timeout 3 ros2 topic pub -r 5 --qos-durability transient_local \
      /robot$VICTIM/goal geometry_msgs/msg/PoseStamped \
      "{header: {frame_id: world}, pose: {position: {x: $CX, y: $CY, z: 0.5}}}" >/dev/null 2>&1
  fi
  # (CHASE_V and the LIE_DEAF deafening used to be two more `ros2 param set` round trips here;
  # they now ride in the single arm_attack.py dispatch above, so nothing about the attacker's
  # configuration changes after the fleet has already reacted to part of it.)
else
say "phase 5: SIGSTOP admm_agent_$VICTIM (pid $PIDV)"
kill -STOP "$PIDV" || die_infra "SIGSTOP failed"
fi

# Review round 2, C1/C2: an EVICT line is NOT evidence any detector fired, for ANY arm,
# including a0. LIE_DEAF defaults to 1 -- the victim stops hearing the survivors, so ITS OWN
# evict_after_misses silence timeout drops them from ITS roster; each survivor then hits
# dds_transport.hpp's roster-exclusion path ("its roster excludes us") and blocks the VICTIM
# right back, and evict_after_lying_'s 3-slot debounce prints "EVICT robot$VICTIM" on both
# survivors about 1.3s after the trigger -- REGARDLESS of ARM, regardless of whether gate2 or
# the belief layer ever saw the position lie at all. The old assertion ("EVICT line appeared =
# 2" for every arm but a0, "no EVICT line" for a0) was therefore vacuous everywhere: it always
# passed via this one silence-driven path, so a control arm whose detector never fired (a1's
# gate2 EMA approaches its gate asymptotically and may never cross it for a lie AT the gate) and
# an undefended arm whose own detector never fired (a0, correctly) looked identical to a run
# where the belief layer genuinely convicted the attacker.
#
# Fix: treat "an EVICT happened" as an infrastructure sanity check only (it should ALWAYS be
# true given LIE_DEAF=1, independent of ARM), and attribute the REAL per-arm signal by the block
# REASON dds_transport.hpp already logs -- "REJECTED AgentState from robotN -- <reason>", thrown
# by block_peer() the first time a peer's messages start being rejected for that reason.
if [ -n "${LIE:-}" ]; then
  if [ "${LIE_DEAF:-1}" = 1 ]; then
    N_EVICT=$(wait_for_log "EVICT robot$VICTIM" 2 60)
    [ "$N_EVICT" -ge 2 ] \
      || die_infra "only $N_EVICT survivor(s) evicted robot$VICTIM within 60s -- roster-exclusion (LIE_DEAF=1) should do this regardless of ARM; if it didn't, the lab flaked, not the experiment"
    say "EVICT seen on $N_EVICT survivor(s) (roster-exclusion sanity check -- NOT proof any detector fired)"
  else
    # LIE_DEAF=0 -- the attacker still HEARS the fleet, so its own silence timeout never fires,
    # it never broadcasts a roster excluding the survivors, and the roster-exclusion path that
    # evicts it "regardless of ARM" is ABSENT BY CONSTRUCTION. That is the entire point of this
    # configuration: it is the only way to get a genuinely undefended control, since the recorded
    # 0.557 vs 0.866 comparison was measured with that path blocking the liar in BOTH arms and so
    # measured about a second of exposure rather than defence versus none. Here "nobody evicted
    # the liar" is the RESULT, and asserting against it would write the hypothesis into the
    # harness. The smear arm needs this too: a deafened attacker's peer_xnow() empties, so its
    # ev_peer array is empty and applyFakeEvidence -- which corrupts a sighting already in the
    # message and never fabricates one -- has nothing to corrupt.
    N_EVICT=$(count_distinct_agents "EVICT robot$VICTIM")
    say "LIE_DEAF=0 (true control): $N_EVICT survivor(s) evicted robot$VICTIM -- recorded, not required"
  fi
  grep_agents "EVICT robot$VICTIM" | sed 's/^/    /' | tee -a "$LOGD/$TAG.log"

  N_G2=$(count_distinct_agents "REJECTED AgentState from robot$VICTIM.*gate2 odom residual")
  N_BEL=$(count_distinct_agents "REJECTED AgentState from robot$VICTIM.*belief threshold")
  case "$ARM" in
    a0)
      # Undefended by design (detection_log_only forced true for the whole run): eviction via
      # roster-exclusion is fine and expected; a DETECTOR-attributed block must never appear --
      # that would mean log_only somehow didn't actually disarm blocking.
      [ "$((N_G2 + N_BEL))" = 0 ] \
        || die "ARM=a0 is supposed to be undefended, but $((N_G2 + N_BEL)) survivor(s)' own detector blocked robot$VICTIM anyway (detection_log_only not actually disarmed?)"
      say "ARM=a0: no detector-attributed block (as expected) -- eviction was roster-exclusion only"
      ;;
    a1)
      # Control arm: a MISS is a legitimate, reportable result (design spec acceptance criterion
      # 1 -- A2 must catch what A1 structurally cannot), not a script failure. Never `die` on it.
      say "ARM=a1: gate2 (odom residual) caught the lie on $N_G2/2 survivor(s)$( [ "$N_G2" -lt 2 ] && echo " -- MISS recorded (expected control-arm result, not a failure)")"
      ;;
    a2)
      # The flagship defended arm: LIE is well past d_lie/2 (undetectable band is only +-2.7mm
      # around d_lie/2 -- see the calibration doc), so a miss here IS a real defence failure.
      # MEASURED 2026-08-01 (d_0801_043929): it misses about one run in three, and not because
      # the residual was ambiguous. The belief layer needs five consecutive evidence-bearing
      # slots; the attacker's own deafness stalls the AgentState barrier that sets claims_fresh,
      # and roster-exclusion blocks the peer at ~9 slots regardless. Lose two slots to a barrier
      # hold and the accumulator is FROZEN where it stood (trust_step_observed returns L
      # unchanged once blocked) at about half the threshold. Kept as a `die` on purpose: this is
      # a real defence failure and must be visible as one, not softened into a note.
      [ "$N_BEL" -ge 1 ] \
        || die "ARM=a2: no survivor's belief layer ever blocked robot$VICTIM -- only roster-exclusion fired, so this run proves nothing about detection"
      say "ARM=a2: belief layer caught the lie on $N_BEL survivor(s)"
      ;;
    occl)
      # Blocking is disarmed here by construction, so a block would mean log-only did not take.
      [ "$((N_G2 + N_BEL))" = 0 ] \
        || die "ARM=occl runs log-only so the attacker survives to be occluded, but $((N_G2 + N_BEL)) survivor(s) blocked it anyway"
      say "ARM=occl: no detector-attributed block (as required) -- the attacker stays alive to hide"
      ;;
    smear|duty|forged_obs)
      # Recorded, not enforced: EXPECT_EVICT=1 here would write the hypothesis into the harness
      # (review round 2, "also" bullet) -- these arms test degradation/evasion, and the run's
      # whole point can legitimately be that the attacker evades, which must not exit as a
      # script failure indistinguishable from a lab flake.
      say "ARM=$ARM: belief layer caught the lie on $N_BEL survivor(s) -- recorded, not enforced"
      ;;
  esac
else
  # Plain silence (no LIE): unaffected by ARM -- the silence-timeout eviction path (evict_after_)
  # doesn't read the detector or its log-only flag at all.
  N_EVICT=$(wait_for_log "EVICT robot$VICTIM" 2 60)
  [ "$N_EVICT" -ge 2 ] || die "only $N_EVICT survivor(s) evicted robot$VICTIM within 60s"
  say "EVICT seen on $N_EVICT survivor(s)"
  grep_agents "EVICT robot$VICTIM" | sed 's/^/    /' | tee -a "$LOGD/$TAG.log"
fi
grep_agents "REBUILD" | tail -2 | sed 's/^/    /' | tee -a "$LOGD/$TAG.log"

# ---------- phase 5c/5d: the occlusion probe (ARM=occl) ----------
# Spec §9 protocol item 6 / acceptance 4. Blocking is disarmed for this arm, so the attacker is
# alive and MID-DESCENT -- L sits somewhere it would keep moving if evidence kept arriving, which
# is exactly what makes "numerically unchanged" a discriminating observation rather than a
# restatement of saturation. Walk it behind cover, hold, walk it back out.
#
# hide_spot.py picks a position occluded from EVERY survivor while still inside obs_range: code 4
# (out_of_range) and code 5 (occluded) both come from visible() returning false, and only the
# second says anything about occlusion. Its geometry mirrors admm::visible() but is not
# authoritative -- what the run reports is the abstain code the C++ itself wrote into CycleStats,
# so a stale mirror can only send the attacker somewhere less useful, never falsify a result.
if [ "$ARM" = occl ]; then
  HIDE=$(python3 "$SCRIPTS/hide_spot.py" "$LOGD/dist.csv" "$VICTIM" $SURVIVORS) \
    || die_infra "no position is occluded from every survivor while still inside obs_range -- there is nothing to probe from where the fleet currently stands"
  say "phase 5c: walking robot$VICTIM to ($HIDE) — occluded from every survivor, still in range"
  timeout 4 ros2 topic pub -r 5 --qos-durability transient_local \
    /robot$VICTIM/goal geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: world}, pose: {position: {x: ${HIDE% *}, y: ${HIDE#* }, z: 0.5}}}" \
    >/dev/null 2>&1
  sleep "${HIDE_T:-30}"
  say "phase 5d: robot$VICTIM comes back out — abstention must end and L must resume moving"
  read RX RY <<< "$(fleet_centroid "$SURVIVORS")"
  timeout 4 ros2 topic pub -r 5 --qos-durability transient_local \
    /robot$VICTIM/goal geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: world}, pose: {position: {x: $RX, y: $RY, z: 0.5}}}" >/dev/null 2>&1
  sleep "${SHOW_T:-20}"
fi


# survivors must keep going to the ORIGINAL goal, routing around the corpse
say "phase 6: survivors continue to ($GX,$GY)"
if ! walk_until "$GX" "$GY" "$SURVIVORS" 60; then
  [ "$EXPECT" = stall ] || die "survivors never reached the outbound goal (arrive_dist=$ARRIVE_DIST)"
  # Geometrically infeasible by design — but prove it rather than assume it. The collision
  # guard and the WBC check inside walk_until already ran on every poll and did not fire.
  confirm_stall "$GX" "$GY" "$SURVIVORS" \
    || die "EXPECT=stall but the detector found no stall — the fleet was moving, so this is a plain miss"
  stop_recording
  # NOT die_infra: dist_summary.py exits 1 for two different reasons (see its own docstring) —
  # too few rows/unreadable file (genuinely infra: the logger never ran), OR it read the file
  # fine and found real time below the contact line (a genuine collision finding). This call site
  # cannot tell which one fired, and mislabeling the second case as infra would let an automated
  # re-run quietly discard the exact kind of result this whole exercise exists to keep.
  python3 "$SCRIPTS/dist_summary.py" "$LOGD/dist.csv" \
    || die "collision guard produced no data (or found contact) — STALL-SAFE not claimable"
  say "D STALL-SAFE: victim=robot$VICTIM, evicted by $N_EVICT, survivors wedged without collision, arrive_dist=$ARRIVE_DIST; logs $LOGD"
  exit 0
fi
say "survivors reached the outbound goal with robot$VICTIM down"

# LIE_THEN_KILL=1 — the compound scenario. A compromised robot that is caught does not politely
# stop existing: it keeps walking, and it may later fail for real. Silencing it HERE, after the
# fleet has already identified and fenced it, is the case neither single experiment covers —
# the survivors have to carry a keep-out from "alive, lying, moving" through to "actually dead"
# without the transition looking like a new fault. The corpse anchor follows odom either way,
# so a stopped body should simply stop dragging its circle.
# With LIE_CHASE the attacker arc (charge -> break off -> park -> dark) already ran on its own
# clock in phase 5b; this older beat would SIGSTOP a second time and re-send the block goal.
if [ -n "${LIE:-}" ] && [ "${LIE_THEN_KILL:-0}" = 1 ] && [ "${LIE_CHASE:-0}" != 1 ]; then
  # With LIE_CHASE the outbound leg is over almost as soon as eviction lands (the survivors
  # kept walking through the whole inject/evict handshake), so darking the liar here amputates
  # the hunt at ~17 s (measured d_0729_093445). Hold the blackout so the chase actually plays:
  # robot1 keeps charging the parked survivors and they keep slipping the circle.
  if [ "${LIE_CHASE:-0}" = 1 ]; then
    say "phase 6b-hold: letting the hunt play ${CHASE_T:-45}s before the blackout"
    sleep "${CHASE_T:-45}"
    # Then it stops hunting and parks INSIDE the pegs, on the survivors' way home: the attacker
    # becomes the obstacle. ROADBLOCK defaults to a spot on the return line through the field.
    kill -9 $CHASEPID 2>/dev/null; CHASEPID=""
    RB=${ROADBLOCK:-"13.6 0.3"}
    say "phase 6a: robot$VICTIM walks into the pegs to park as a roadblock at ($RB)"
    timeout 8 ros2 topic pub -r 5 --qos-durability transient_local \
      /robot$VICTIM/goal geometry_msgs/msg/PoseStamped \
      "{header: {frame_id: world}, pose: {position: {x: ${RB% *}, y: ${RB#* }, z: 0.5}}}" \
      >/dev/null 2>&1
    # WAIT for it to actually get there. A fixed sleep cut the walk short and the attacker was
    # frozen mid-stride, which reads as a crash rather than as it taking up station.
    for _i in $(seq 1 "${PARK_T:-40}"); do
      RD=$(python3 - "$LOGD/dist.csv" "$VICTIM" ${RB% *} ${RB#* } <<'PY' 2>/dev/null
import csv, math, sys
rows = list(csv.DictReader(open(sys.argv[1])))
r = rows[-1] if rows else None
v = int(sys.argv[2])
print('%.2f' % math.hypot(float(r['x%d' % v]) - float(sys.argv[3]),
                          float(r['y%d' % v]) - float(sys.argv[4])) if r else 99)
PY
)
      [ "$(python3 -c "print(1 if ${RD:-99} < 0.7 else 0)")" = 1 ] \
        && { say "  robot$VICTIM is on station (${RD} m from the block point)"; break; }
      sleep 2
    done
  fi
  say "phase 6b: SIGSTOP admm_agent_$VICTIM — the liar now goes dark for real"
  kill -STOP "$PIDV" 2>/dev/null || say "  WARNING: SIGSTOP failed (pid $PIDV gone?)"
  sleep 6   # let its lower layer run out the last published trajectory and park
  read LX LY <<< "$(fleet_centroid "$VICTIM")"
  say "  robot$VICTIM parked near ($LX,$LY); survivors must now route home around a dead body"
fi

if [ "$REJOIN" = 1 ]; then
  # ---------- phase 7: victim comes back (scenario B, needs the Phase-2 view-change) ----------
  say "phase 7: SIGCONT admm_agent_$VICTIM — expecting rejoin"
  kill -CONT "$PIDV" || die_infra "SIGCONT failed"
  N_RE=$(wait_for_log "REJOIN robot$VICTIM" 2 60)
  if [ "$N_RE" -lt 2 ]; then
    say "NOTE: no REJOIN on $((2 - N_RE)) survivor(s) — rejoin is Phase 2, not yet implemented?"
    die "rejoin not observed within 60s"
  fi
  say "REJOIN seen on $N_RE survivor(s)"
  grep_agents "corpse of robot$VICTIM removed" | sed 's/^/    /' | tee -a "$LOGD/$TAG.log"
  WATCH="$ROBOTS"
else
  # Whom to watch home: the fleet that still exists. With LIE_DEAF=0 -- which the true-control
  # arm, smear and occl all require -- nothing evicts the attacker, so it is still a member, the
  # coordinator still plans for it, and demanding a survivors-only plan below would fail the run
  # for the absence of an eviction those configurations are specifically designed not to cause.
  if [ "${N_EVICT:-0}" -ge 1 ]; then WATCH="$SURVIVORS"; else WATCH="$ROBOTS"; fi
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

walk_until "$HX" "$HY" "$WATCH" 90 || die "fleet never got home (arrive_dist=$ARRIVE_DIST)"

# A straight-line fallback means A* found no route; that is the silent precursor to a frozen dog.
NOPATH=$(count_agents "A\* found NO path")
[ "$NOPATH" = 0 ] || say "WARNING: $NOPATH A* failure(s) — check $LOGD/admm.log"

stop_recording
# dist_summary reads every row and is the honest separation report. Its exit code is a
# clean-run verdict on ALL robots, which is the right gate when the victim is a corpse — and the
# wrong one the moment the victim is alive. See below.
python3 "$SCRIPTS/dist_summary.py" "$LOGD/dist.csv" || DIRTY=1

# DID THEY TOUCH. Every other guard in this script works on centre-to-centre distance, which was
# measured on 2026-07-29 to read ~0.68 m high: three dogs standing undisturbed are 1.40 m apart
# by centre and 0.717 m by surface. A run whose centres never dropped below 0.90 can still have
# had bodies 0.07 m apart. This is the check that actually answers the question, and it applies
# to every scenario — a clean formation run has to clear it too.
#
# 0.15 m, not 0: phys_gap measures the 0.83 x 0.25 base BOX, and the legs swing outside it. A
# positive box gap therefore cannot prove the legs missed; it can only prove the bodies did.
# Anything under 0.15 is reported as unproven rather than passed.
if [ -s "$LOGD/phys_gap.csv" ]; then
  # Per PAIR, not per run: GAPV_S is the worst surface gap ever seen between two SURVIVORS,
  # GAPV_V the worst involving $VICTIM. gap_min/pair (the run-wide worst) conflates them, and a
  # live attacker being steered into the fleet can legitimately drive its own pair's gap far
  # below a survivor-survivor gap that never moved -- downgrading on the WHOLE ARM instead of
  # the PAIR would silently accept a genuine survivor-survivor overlap too. Reuses the same
  # gap_{i}_{j} columns the TRUE BODY GAP printer below already reads per pair.
  read GAPV_V GAPV_VPAIR GAPV_S GAPV_SPAIR <<< "$(python3 - "$LOGD/phys_gap.csv" "$VICTIM" <<'PY'
import csv, sys
rows = list(csv.DictReader(open(sys.argv[1])))
victim = sys.argv[2]
if not rows:
    print("nan none nan none")
    sys.exit()
pair_cols = [c for c in rows[0] if c.startswith("gap_") and c != "gap_min"]
worst_v, worst_v_pair = 99.0, "none"
worst_s, worst_s_pair = 99.0, "none"
for c in pair_cols:
    i, j = c.split("_")[1], c.split("_")[2]
    g = min(float(r[c]) for r in rows)
    if i == victim or j == victim:
        if g < worst_v:
            worst_v, worst_v_pair = g, "%s-%s" % (i, j)
    elif g < worst_s:
        worst_s, worst_s_pair = g, "%s-%s" % (i, j)
print("%.4f %s %.4f %s" % (worst_v, worst_v_pair, worst_s, worst_s_pair))
PY
)"
  case "$GAPV_V" in
    nan) say "WARNING: no body-gap samples — 'did they touch' is UNANSWERED for this run" ;;
    *)
      # Survivor-survivor overlap is ALWAYS fatal, LIE or not: nobody attacked THAT pair, it is
      # exactly the false-positive-shaped failure this whole detector exists to avoid, and centre
      # distance (the SVMIN gate) has already been measured to miss real contact by up to 0.68 m
      # -- this surface gap must not be softened for it.
      [ "$(python3 -c "print(1 if $GAPV_S <= 0 else 0)")" = 1 ] \
        && die "BODIES OVERLAPPED (survivors $GAPV_SPAIR): true surface gap ${GAPV_S} m"
      if [ "$(python3 -c "print(1 if $GAPV_V <= 0 else 0)")" = 1 ]; then
        # A live, deaf attacker steered INTO the fleet can genuinely make contact with a survivor
        # -- that is the attack's effect, not a script defect, and an undefended (or
        # weakly-defended) arm exists specifically to measure how bad it gets. Dying here would
        # erase that arm's own verdict before the counterfactual summary below ever runs. A
        # lie-free run has no attacker to blame, so overlap there stays a real failure.
        if [ -n "${LIE:-}" ]; then
          say "WARNING: BODIES OVERLAPPED (robot$VICTIM pair $GAPV_VPAIR): true surface gap ${GAPV_V} m"
        else
          die "BODIES OVERLAPPED (robot$VICTIM pair $GAPV_VPAIR): true surface gap ${GAPV_V} m"
        fi
      fi
      GAPV=$(python3 -c "print(min($GAPV_V, $GAPV_S))")
      [ "$(python3 -c "print(1 if $GAPV < 0.15 else 0)")" = 1 ] \
        && say "WARNING: closest body surfaces ${GAPV} m — boxes cleared, LEGS NOT PROVEN CLEAR"
      ;;
  esac
fi
if [ -n "${LIE:-}" ]; then
  # LIE scenario: the "victim" is alive, deaf, and deliberately steered INTO the fleet. How close
  # its body gets is a measure of how hard the ATTACKER pushed, not of whether the defence held,
  # so gating on it fails the run for the adversary's behaviour and would quietly reward a
  # feebler attack. Measured 2026-07-29: detection ON let the real body reach 0.866 m, detection
  # OFF 0.557 m — the defence is the 0.31 m, not an absolute floor.
  #
  # What the survivors DO owe us is not colliding with each other. That is the false-positive
  # failure this whole detector exists to avoid (2026-07-28: mutual eviction, pairwise CBF gone,
  # 0.832 m), so it stays a hard gate — over every row, not a sampled poll.
  SVMIN=$(pair_among $SURVIVORS)
  [ "$(python3 -c "print(1 if $SVMIN<$DMIN_ABORT else 0)")" = 1 ] \
    && die "survivors closed to $SVMIN < $DMIN_ABORT — survivors collided, a real failure"
  say "RESULT: survivors held ${SVMIN} m from each other; the compromised body reached $(pair_among $ROBOTS) m"
  # Centre distance is a proxy. This is the number to quote when someone asks whether they
  # touched: <= 0 means the base boxes overlapped, full stop.
  if [ -s "$LOGD/phys_gap.csv" ]; then
    say "TRUE BODY GAP (surface-to-surface, real yaw): $(python3 -c "
import csv
rows = list(csv.DictReader(open('$LOGD/phys_gap.csv')))
if not rows:
    print('no data')
else:
    # Per pair, not just the overall worst: 'they nearly touched' means something different when
    # it is the liar closing on a survivor than when it is the two survivors squeezing past a peg.
    pairs = [c for c in rows[0] if c.startswith('gap_') and c != 'gap_min']
    parts = ['%s %.3f' % (p.replace('gap_', 'r').replace('_', '-r'), min(float(r[p]) for r in rows))
             for p in pairs]
    worst = min(float(r['gap_min']) for r in rows)
    print('%.3f m worst  [%s]%s' % (worst, '  '.join(parts),
                                    '  *** BODIES OVERLAPPED ***' if worst <= 0 else ''))
" 2>/dev/null || echo "unreadable")"
  fi
elif [ -n "${DIRTY:-}" ]; then
  die "separation was not clean (see dist_summary above) — PASS not claimable"
fi
if [ "${LIE_LOGONLY:-0}" = 1 ]; then
  # The old BREACH tracker sampled min_pair() (the CSV's last row) once per ~5s poll inside
  # walk_until, the same sparse instrument dist_summary.py's own docstring warns about: an
  # excursion shorter than the poll interval slips past it. pair_among scans every row, exactly
  # like the post-hoc gate two lines up (SVMIN) — the headline owes the same rigor.
  BREACH_FULL=$(pair_among $ROBOTS)
  if [ "$(python3 -c "print(1 if $BREACH_FULL<$DMIN_ABORT else 0)")" = 1 ]; then
    say "COUNTERFACTUAL RESULT: the accepted lie cost $BREACH_FULL m of true separation"
    say "  (the survivors never breached D_MIN against the GHOST — only against the real body)"
  fi
fi
say "D PASS (rejoin=$REJOIN): victim=robot$VICTIM, evicted by $N_EVICT, home reached, arrive_dist=$ARRIVE_DIST, mission_denied=$MISSION_DENIED; logs $LOGD"

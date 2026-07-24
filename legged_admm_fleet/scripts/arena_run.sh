#!/bin/bash
# Arena run (plum / door) for the distributed fleet: same phased bring-up as g4_run.sh, but the
# Gazebo world is our obstacle arena (worlds/<arena>.sdf via gazebo_package:=legged_admm_fleet),
# the agents run with arena:=<arena> use_astar:=true (obstacle-CBF + A* detour), and each dog gets
# its OWN goal (/robotN/goal) instead of a formation centroid. g5_logger records comm latency +
# convergence + trajectory error the whole time; g3_dist_logger guards collisions.
#
# Usage: arena_run.sh <plum|door>
#   env: GOALS="x1 y1 x2 y2 x3 y3" (override per-dog goals), RUNTAG=<label>, RECORD=1 (screen-capture)
WS=/root/legged_ros2_ws
ARENA=${1:?usage: arena_run.sh <plum|door>}
RUNTAG=${RUNTAG:-$ARENA}
LOGD=$WS/g2_logs/arena_${RUNTAG}_$(date +%m%d_%H%M%S)
mkdir -p "$LOGD"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash
source /root/gridmap_ws/install/setup.bash
source $WS/install/setup.bash
set -u

# Render to a virtual framebuffer (Xvfb :99), NOT the host display :1 — the launch REQUIRES the gz
# GUI to load the world, so we give it an invisible display instead of the user's screen. (unset
# DISPLAY / Qt offscreen both break spawning.) This same :99 is what the recorded run captures with
# ffmpeg. Start Xvfb if it isn't already up.
export DISPLAY=:99
# Check for THIS display, not any Xvfb — the RViz path (RVIZ=1) also runs an Xvfb on :98, so a
# display-agnostic `pgrep -x Xvfb` would false-positive on :98 and skip starting :99, leaving the gz
# GUI unable to connect to :99 (sim never steps -> controller activation hangs -> odom missing).
pgrep -f "Xvfb :99" >/dev/null || { Xvfb :99 -screen 0 1600x900x24 >/tmp/xvfb99.log 2>&1 & sleep 2; }

ROBOTS=${ROBOTS:-"1 2 3"}
ROSTER=$WS/install/legged_admm_fleet/share/legged_admm_fleet/config/fleet_robots.yaml
CTRL_YAML=$WS/install/legged_admm_fleet/share/legged_admm_fleet/config/vision60_fleet_controller.yaml
V=${V:-0.4}
DEADLINE=${DEADLINE:-20}
DMIN_ABORT=${DMIN_ABORT:-0.5}
GOAL_TOL=${GOAL_TOL:-0.5}
GOALS_IN="${GOALS:-}"          # was a single goal-set given explicitly on the env?

# per-dog goals — MUST match fleet_config.cpp arenas() <arena>.goals (kept in sync manually).
if [ -z "${GOALS:-}" ]; then
  case "$ARENA" in
    plum) GOALS="18.0 0.0 16.788 0.7 16.788 -0.7" ;;
    plum_dense) GOALS="17.2 0.0 15.988 0.7 15.988 -0.7" ;;   # 0.958x plum, gap 2.20; A* plans full weave (single set)
    door) GOALS="13.02 0.7 11.808 1.4 11.808 0.0" ;;
    *) echo "unknown arena $ARENA"; exit 1 ;;
  esac
fi

# SEQ = a multi-waypoint sequence: goal-sets separated by ';', each set "x1 y1 x2 y2 x3 y3". The dogs
# visit each set in order (longer run + video). Precedence: explicit SEQ > explicit single GOALS >
# a built-in per-arena traverse. Per-set arrival is NON-fatal (log + advance on timeout) so one hard
# leg can't abort the whole recorded sequence; the collision guard stays fatal.
if [ -z "${SEQ:-}" ]; then
  if [ -n "$GOALS_IN" ]; then SEQ="$GOALS"          # explicit single set -> keep old behaviour
  else case "$ARENA" in
    # staggered columns (leader ahead, two followers offset): every pair >= D_MIN=1.3 so the
    # inter-robot CBF never forbids the goal, and every waypoint sits >=0.90 m (live r_eff) off
    # any pillar. Abreast sets would deadlock 3 dogs in a sub-D_MIN gap.
    # plum: 4 sets threading the 7-row field (min pair 1.95, min waypoint-to-pile 0.91).
    plum) SEQ="5.67 0.0 4.0 1.0 4.0 -1.0 ; 9.31 0.0 7.6 1.0 7.6 -1.0 ; 12.95 0.0 11.2 1.0 11.2 -1.0 ; 18.0 0.0 16.788 0.7 16.788 -0.7" ;;
    door) SEQ="6.5 0.0 5.2 0.9 5.2 -0.9 ; 10.0 0.0 8.9 1.1 8.9 -1.1 ; 12.5 0.0 12.5 2.2 12.5 -3.0" ;;
    *) SEQ="$GOALS" ;;
  esac; fi
fi
IFS=';' read -ra SETS <<< "$SEQ"
NSET=${#SETS[@]}
SET_ITERS=${SET_ITERS:-70}     # per-set arrival cap (iters of the 5s monitor loop) before advancing

# FORMATION=1: traverse AS A SCALE-FREE V (LaplacianFormation similarity) instead of per-dog goals.
# We publish only the TEAM CENTROID waypoints to /formation/goal; fleet_coordinator_node assigns slots
# and the agents hold the V SHAPE (soft w_form, scale-invariant) while the obstacle/edge CBF (hard)
# lets each dog take its own gap and the V expand/contract freely. FSEQ = centroids "cx cy ; cx cy".
if [ -z "${FSEQ:-}" ]; then
  case "$ARENA" in
    plum) FSEQ="4.0 0.0 ; 8.0 0.0 ; 12.0 0.0 ; 16.0 0.0" ;;
    door) FSEQ="5.0 0.0 ; 9.0 0.0 ; 12.5 0.0" ;;
    *) FSEQ="6.0 0.0" ;;
  esac
fi
IFS=';' read -ra FSETS <<< "$FSEQ"
NFSET=${#FSETS[@]}
FGOAL_TOL=${FGOAL_TOL:-0.6}     # fleet-centroid arrival tolerance (m)
TOTSET=$([ "${FORMATION:-0}" = 1 ] && echo "$NFSET" || echo "$NSET")

say() { echo "[arena $(date +%H:%M:%S)] $*" | tee -a "$LOGD/arena.log"; }
die() { say "FAIL: $*"; exit 1; }

odom_field() {
  timeout 5 ros2 topic echo "$1" --once 2>/dev/null | python3 -c "
import sys,re
raw=sys.stdin.read()
m=re.search(r'position:\s*\n\s*x: ([\-\d.e]+)\s*\n\s*y: ([\-\d.e]+)\s*\n\s*z: ([\-\d.e]+)',raw)
print(*(m.groups()) if m else '')" ;
}

say "phase 0: cleanup (arena=$ARENA goals=[$GOALS] deadline=${DEADLINE}ms)"
pkill -9 -x ros2 2>/dev/null; pkill -9 -x admm_agent_node 2>/dev/null; pkill -9 -x fleet_coordinat 2>/dev/null
pkill -9 -x ruby 2>/dev/null; pkill -9 -x robot_state_pub 2>/dev/null; pkill -9 -x static_transfor 2>/dev/null
pkill -9 -x parameter_bridg 2>/dev/null; pkill -9 -f g5_logger 2>/dev/null; pkill -9 -f g3_dist_logger 2>/dev/null
pkill -9 -f g5_video 2>/dev/null; pkill -9 -x legged_common_g 2>/dev/null; pkill -9 -x legged_common_b 2>/dev/null
pkill -9 -x rviz2 2>/dev/null; pkill -9 -x ffmpeg 2>/dev/null; pkill -9 -f fleet_viz_markers 2>/dev/null
# A `ros2 launch` parent has comm=python3, so `pkill -x ros2` above misses it: a FAILED run leaks its
# gazebo/controller launch, which keeps holding domain 42 and poisons the next run's DDS discovery
# (spawner "Could not contact service .../controller_manager"). Kill any leaked launch parent by cmdline
# (bracket regex so this line never self-matches; the script's own cmdline is just its path).
pkill -9 -f "launch[.]py" 2>/dev/null
sleep 2
rm -rf /dev/shm/fastrtps* /dev/shm/fast_datasharing* /dev/shm/sem.fastrtps* 2>/dev/null

# ---------- phase 1: gazebo (OUR arena world) + settle ----------
say "phase 1: gazebo up (world=$ARENA from legged_admm_fleet)"
setsid ros2 launch legged_gazebo gazebo.launch.py world:=$ARENA gazebo_package:=legged_admm_fleet \
  robots_config_file:=$ROSTER > "$LOGD/gazebo.log" 2>&1 &
for i in $(seq 1 90); do
  ALL=1
  for r in $ROBOTS; do F=$(odom_field /robot$r/hardware/odom); [ -n "$F" ] || { ALL=0; break; }; done
  [ "$ALL" = 1 ] && break; sleep 2
done
[ "$ALL" = 1 ] || die "odom missing after 180s"
say "all odoms flowing; settling"
sleep 8
say "settled"

# ---------- phase 2: controllers (immediate re-activate on the activation race) ----------
reactivate() {
  timeout 20 ros2 service call /robot$1/controller_manager/switch_controller \
    controller_manager_msgs/srv/SwitchController \
    "{activate_controllers: [legged_robot_controller], strictness: 1, timeout: {sec: 10}}" \
    >> "$LOGD/reactivate.log" 2>&1
  ST=$(timeout 6 ros2 control list_controllers -c /robot$1/controller_manager 2>/dev/null | grep legged_robot_controller | awk '{print $NF}')
  [ "$ST" = active ]
}
say "phase 2: load_controller"
setsid ros2 launch legged_robot_controller load_controller.launch.py \
  robots_config_file:=$ROSTER controller_param_file:=$CTRL_YAML \
  gait_command:=true base_target_command:=false > "$LOGD/controller.log" 2>&1 &
declare -A RETRIES; for r in $ROBOTS; do RETRIES[$r]=0; done
for i in $(seq 1 240); do
  ALL=1
  for r in $ROBOTS; do
    ST=$(timeout 6 ros2 control list_controllers -c /robot$r/controller_manager 2>/dev/null | grep legged_robot_controller | awk '{print $NF}')
    if [ "$ST" = inactive ]; then
      RETRIES[$r]=$((RETRIES[$r] + 1))
      [ "${RETRIES[$r]}" -le 3 ] || die "robot$r dead after 3 re-activations"
      say "  robot$r deactivated -> re-activate (try ${RETRIES[$r]})"; reactivate $r; ALL=0
    elif [ "$ST" != active ]; then ALL=0; fi
  done
  [ "$ALL" = 1 ] && break; sleep 0.5
done
[ "$ALL" = 1 ] || die "controllers not all active"
say "all controllers ACTIVE"

# ---------- phase 3: distributed agents (arena + A*) + coordinator + loggers ----------
GZMARK=$(wc -l < "$LOGD/gazebo.log")
gz_deactivated() { tail -n +$((GZMARK + 1)) "$LOGD/gazebo.log" | grep -q "Deactivating"; }
IDS="[${ROBOTS// /, }]"
say "phase 3: distributed agents (arena=$ARENA use_astar=true)"
setsid ros2 launch legged_admm_fleet admm_fleet.launch.py mode:=distributed \
  robot_ids:="$IDS" v:=$V hop_deadline_ms:=$DEADLINE arena:=$ARENA use_astar:=true \
  > "$LOGD/admm.log" 2>&1 &
setsid python3 $WS/src/legged_fleet/legged_admm_fleet/scripts/g5_logger.py \
  "$ROBOTS" "$LOGD" --ros-args -p use_sim_time:=true > "$LOGD/g5_logger.log" 2>&1 &
G5PID=$!
setsid python3 $WS/src/legged_fleet/legged_admm_fleet/scripts/g3_dist_logger.py \
  "$ROBOTS" "$LOGD/dist.csv" 1.3 --ros-args -p use_sim_time:=true > "$LOGD/dist_logger.log" 2>&1 &  # 1.3 = admm::D_MIN
DLPID=$!
RVIZPID=""; RVGRABPID=""; VIZPID=""; RECPID=""; BRPID=""
trap 'kill -9 $G5PID $DLPID $RVIZPID $RVGRABPID $VIZPID $RECPID $BRPID 2>/dev/null' EXIT
# LATCH per-robot: once a dog is seen at z>0.42 it counts as stood for good. The odom read is a flaky
# `ros2 echo --once` (DDS late-join drops it under load), so requiring all 3 to read tall in the SAME
# iter gives false "did not stand" timeouts. Latching + only failing if a robot NEVER read tall fixes it.
declare -A STOOD SRETRY; for r in $ROBOTS; do STOOD[$r]=0; SRETRY[$r]=0; done
for i in $(seq 1 50); do
  # The activation race (upstream) can auto-deactivate a controller on the first stand-up update
  # (huge period -> QP infeasible). Phase 2 re-activates on this; phase 3 must too — dying here throws
  # away an otherwise-fine bring-up. Re-activate any controller that fell inactive, then keep standing.
  for r in $ROBOTS; do
    [ "${STOOD[$r]}" = 1 ] && continue
    ST=$(timeout 6 ros2 control list_controllers -c /robot$r/controller_manager 2>/dev/null | grep legged_robot_controller | awk '{print $NF}')
    if [ "$ST" = inactive ]; then
      SRETRY[$r]=$((SRETRY[$r] + 1))
      [ "${SRETRY[$r]}" -le 4 ] || die "robot$r dead after 4 stand-up re-activations"
      say "  robot$r deactivated during stand-up -> re-activate (try ${SRETRY[$r]})"; reactivate $r
    fi
  done
  ALL=1
  for r in $ROBOTS; do
    [ "${STOOD[$r]}" = 1 ] && continue
    Z=$(timeout 5 ros2 topic echo /robot$r/controller/odom --once 2>/dev/null | grep -m1 -A3 position: | grep 'z:' | awk '{print $2}')
    if [ "$(python3 -c "print(1 if ${Z:-0}>0.42 else 0)")" = 1 ]; then STOOD[$r]=1; else ALL=0; fi
  done
  [ "$ALL" = 1 ] && break
  sleep 3
done
for r in $ROBOTS; do [ "${STOOD[$r]}" = 1 ] || die "robot$r did not stand"; done
say "all STANDING"

# ---------- optional recording (RECORD=1): clean overhead arena_cam sensor -> cv2 mp4 ----------
# The world's arena_cam camera sensor renders the scene (no GUI clutter); ros_gz_bridge exposes it
# as a ROS Image; g5_video.py encodes to mp4 with cv2. All headless on :99.
RECPID=""; BRPID=""
if [ "${RECORD:-0}" = 1 ]; then
  say "recording arena_cam -> demo.mp4"
  setsid ros2 run ros_gz_bridge parameter_bridge \
    /arena_cam/image@sensor_msgs/msg/Image@gz.msgs.Image >"$LOGD/bridge.log" 2>&1 &
  BRPID=$!
  sleep 3
  setsid python3 $WS/src/legged_fleet/legged_admm_fleet/scripts/g5_video.py \
    /arena_cam/image "$LOGD/demo.mp4" 30 >"$LOGD/g5_video.log" 2>&1 &
  RECPID=$!
fi

# ---------- optional RViz capture (RVIZ=1): MPC-horizon markers + robot models -> rviz.mp4 ----------
# rviz2 renders on its OWN headless Xvfb (:98, software GL) so it never fights the gz GUI on :99;
# fleet_viz_markers.py feeds it the arena + per-dog MPC-horizon MarkerArrays; ffmpeg x11grabs :98.
if [ "${RVIZ:-0}" = 1 ]; then
  say "recording RViz -> rviz.mp4"
  RVIZ_CFG=$WS/src/legged_fleet/legged_admm_fleet/rviz/fleet.rviz
  # Always (re)create :98 at EXACTLY the ffmpeg grab size — a pre-existing :98 of a different size
  # (e.g. a 1280x720 left by a smoke test) makes `x11grab -video_size 1600x1000` fail "Invalid argument".
  pkill -9 -f "Xvfb :98" 2>/dev/null; sleep 1
  Xvfb :98 -screen 0 1600x1000x24 >/tmp/xvfb98.log 2>&1 & sleep 2
  setsid python3 $WS/src/legged_fleet/legged_admm_fleet/scripts/fleet_viz_markers.py \
    "$ROBOTS" "$ARENA" --ros-args -p use_sim_time:=true >"$LOGD/fleet_viz.log" 2>&1 &
  VIZPID=$!
  DISPLAY=:98 LIBGL_ALWAYS_SOFTWARE=1 QT_QPA_PLATFORM=xcb setsid \
    rviz2 -d "$RVIZ_CFG" --ros-args -p use_sim_time:=true >"$LOGD/rviz2.log" 2>&1 &
  RVIZPID=$!
  sleep 15   # let rviz2 open + build the GL scene before grabbing
  setsid ffmpeg -y -f x11grab -video_size 1600x1000 -framerate 20 -i :98 \
    -c:v libx264 -pix_fmt yuv420p -movflags +faststart "$LOGD/rviz.mp4" >"$LOGD/rviz_grab.log" 2>&1 &
  RVGRABPID=$!
fi

# ---------- phase 4: trot, then walk the waypoint sequence ----------
# trot can't be latched (the cmd_gait subscriber is the upstream controller, read-only submodule, and
# stays volatile), so instead keep each publisher alive 8 s at 2 Hz: discovery completes well within
# that and re-sending trot to an already-trotting dog is idempotent -> no "one dog stuck in STANCE".
say "phase 4: trot (persistent burst to beat the DDS discovery race)"
for r in $ROBOTS; do
  timeout 8 ros2 topic pub -r 2 /robot$r/cmd_gait std_msgs/msg/String "{data: trot}" >/dev/null 2>&1 &
done
sleep 9

# Walk one goal-set: persistent per-dog publish (3 Hz/5 s — a bare --once races DDS discovery and can
# leave a dog on its stand-in-place goal), then a monitor loop until all within GOAL_TOL or SET_ITERS.
# Returns 0 on arrival, 1 on timeout (non-fatal -> caller advances). Collision guard / WBC = fatal.
goto_set() {
  local set="$1" idx="$2" k r gx gy cx cy e ok DONE MIND BAD LINE i
  read -a G <<< "$set"
  k=0
  for r in $ROBOTS; do
    gx=${G[$((k*2))]}; gy=${G[$((k*2+1))]}
    say "  set $idx/$NSET robot$r goal=($gx,$gy)"
    # transient_local (latched) matches the agent's now-latched goal_sub_: while this 8 s publisher
    # lives, an agent that matches even late still gets the last goal -> no more "one dog parked at
    # spawn" discovery race. (A --once latched pub would drop its cache on exit, so keep it alive 8 s.)
    timeout 8 ros2 topic pub -r 5 --qos-durability transient_local /robot$r/goal geometry_msgs/msg/PoseStamped \
      "{header: {frame_id: world}, pose: {position: {x: $gx, y: $gy, z: 0.5}}}" >/dev/null 2>&1 &
    k=$((k+1))
  done
  sleep 9
  for i in $(seq 1 "$SET_ITERS"); do
    DONE=1; LINE=""; k=0
    for r in $ROBOTS; do
      C=($(odom_field /robot$r/controller/odom)); cx=${C[0]:-99}; cy=${C[1]:-99}
      gx=${G[$((k*2))]}; gy=${G[$((k*2+1))]}
      e=$(python3 -c "import math;print(f'{math.hypot($cx-$gx,$cy-$gy):.2f}')")
      LINE="$LINE r$r=($cx,$cy)/e$e"
      ok=$(python3 -c "print(1 if $e < $GOAL_TOL else 0)"); [ "$ok" = 1 ] || DONE=0
      k=$((k+1))
    done
    MIND=$(tail -1 "$LOGD/dist.csv" 2>/dev/null | cut -d, -f8); case "$MIND" in ""|*[!0-9.-]*) MIND=99;; esac
    say "  [set $idx] $LINE min_pair=$MIND"
    BAD=$(python3 -c "print(1 if $MIND<$DMIN_ABORT else 0)")
    [ "$BAD" = 1 ] && die "pairwise $MIND < $DMIN_ABORT (collision guard)"
    [ "$DONE" = 1 ] && return 0
    gz_deactivated && die "WBC deactivated mid-walk"
    sleep 5
  done
  say "  [set $idx] timed out after $((SET_ITERS*5))s — advancing"
  return 1
}

# Formation mode: publish ONE team centroid to /formation/goal (coordinator -> latched /formation/plan
# -> agents). Wait until the fleet centroid (mean of the ROBOTS odoms) is within FGOAL_TOL. The V shape
# and per-dog gap-threading are handled by the agents (Laplacian w_form + CBF); we only steer the team.
goto_formation() {
  local cx="$1" cy="$2" idx="$3" i r XY mx my e MIND BAD
  say "  fset $idx/$NFSET centroid=($cx,$cy)"
  # coordinator's /formation/goal sub is volatile but the node is persistent (discovered early); keep
  # an 8 s burst so a late match still lands. /formation/plan on to the agents is latched anyway.
  timeout 8 ros2 topic pub -r 5 /formation/goal geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: world}, pose: {position: {x: $cx, y: $cy, z: 0.5}}}" >/dev/null 2>&1 &
  sleep 9
  for i in $(seq 1 "$SET_ITERS"); do
    XY=""
    for r in $ROBOTS; do C=($(odom_field /robot$r/controller/odom)); XY="$XY ${C[0]:-99} ${C[1]:-99}"; done
    read mx my e < <(python3 -c "
import math
v=[float(x) for x in '$XY'.split()]; xs=v[0::2]; ys=v[1::2]
mx=sum(xs)/len(xs); my=sum(ys)/len(ys)
print(f'{mx:.2f} {my:.2f} {math.hypot(mx-$cx,my-$cy):.2f}')")
    MIND=$(tail -1 "$LOGD/dist.csv" 2>/dev/null | cut -d, -f8); case "$MIND" in ""|*[!0-9.-]*) MIND=99;; esac
    say "  [fset $idx] centroid=($mx,$my)/e$e min_pair=$MIND"
    BAD=$(python3 -c "print(1 if $MIND<$DMIN_ABORT else 0)")
    [ "$BAD" = 1 ] && die "pairwise $MIND < $DMIN_ABORT (collision guard)"
    [ "$(python3 -c "print(1 if $e<$FGOAL_TOL else 0)")" = 1 ] && return 0
    gz_deactivated && die "WBC deactivated mid-walk"
    sleep 5
  done
  say "  [fset $idx] timed out — advancing"
  return 1
}

T_WALL0=$(date +%s); T_SIM0=$(timeout 4 ros2 topic echo /clock --once 2>/dev/null | grep -m1 sec: | awk '{print $2}')
NARR=0
if [ "${FORMATION:-0}" = 1 ]; then
  say "phase 4: FORMATION traverse — $NFSET centroid waypoint(s), scale-free V"
  for s in $(seq 1 "$NFSET"); do
    fset=($(echo "${FSETS[$((s-1))]}" | xargs))
    goto_formation "${fset[0]}" "${fset[1]}" "$s" && { NARR=$((NARR+1)); say "  fset $s reached"; }
  done
else
  say "phase 4: per-dog walk — $NSET waypoint set(s)"
  for s in $(seq 1 "$NSET"); do
    set="$(echo "${SETS[$((s-1))]}" | xargs)"   # trim
    goto_set "$set" "$s" && { NARR=$((NARR+1)); say "  set $s reached"; }
  done
fi
T_WALL1=$(date +%s); T_SIM1=$(timeout 4 ros2 topic echo /clock --once 2>/dev/null | grep -m1 sec: | awk '{print $2}')
RTF=$(python3 -c "w=$T_WALL1-$T_WALL0;s=${T_SIM1:-0}-${T_SIM0:-0};print(f'{s/w:.2f}' if w>0 else '?')")
sleep 3   # let the loggers flush a few post-arrival samples
if [ -n "$RECPID" ]; then kill -INT "$RECPID" 2>/dev/null; sleep 3; kill -9 "$BRPID" 2>/dev/null; \
  say "video: $LOGD/demo.mp4 ($(du -h "$LOGD/demo.mp4" 2>/dev/null | cut -f1))"; fi
if [ -n "$RVGRABPID" ]; then kill -INT "$RVGRABPID" 2>/dev/null; sleep 3; \
  say "rviz video: $LOGD/rviz.mp4 ($(du -h "$LOGD/rviz.mp4" 2>/dev/null | cut -f1))"; fi
say "ARENA DONE: $ARENA reached $NARR/$TOTSET waypoint sets (mode=$([ "${FORMATION:-0}" = 1 ] && echo formation-V || echo per-dog)); min_pair>=$DMIN_ABORT; RTF=$RTF; logs $LOGD"
say "csv: stats=$LOGD/stats.csv traj=$LOGD/traj.csv dist=$LOGD/dist.csv"

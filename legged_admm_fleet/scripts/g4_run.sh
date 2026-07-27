#!/bin/bash
# G4 gate: 3x vision60, distributed — one admm_agent_node per dog, ADMM consensus over DDS.
# ponytail: phases 0-3 below are duplicated in scripts/_fleet_bringup.sh (which d_run.sh sources).
# Left duplicated so this verified gate stays byte-identical; fix bring-up in BOTH, or fold this
# script onto _fleet_bringup.sh and re-run the gate.
# Same phased bring-up as g3_run.sh; phase 3 launches the distributed agents instead of the
# one centralized node. Each agent stands its dog in place (goal := current odom) until the
# /formation/goal -> FleetPlan (from the standalone fleet_coordinator_node) moves them.
WS=/root/legged_ros2_ws
LOGD=$WS/g2_logs/g4_$(date +%m%d_%H%M%S)
mkdir -p "$LOGD"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST   # isolate DDS from the shared lab subnet
export ROS_DOMAIN_ID=42                           # (foreign /clock injection stalls the WBC)
source /opt/ros/jazzy/setup.bash
source /root/gridmap_ws/install/setup.bash
source $WS/install/setup.bash
set -u

ROBOTS=${ROBOTS:-"1 2 3"}
ROSTER=$WS/install/legged_admm_fleet/share/legged_admm_fleet/config/fleet_robots.yaml
CTRL_YAML=$WS/install/legged_admm_fleet/share/legged_admm_fleet/config/vision60_fleet_controller.yaml
V=${V:-0.4}
GOAL_X=${GOAL_X:-2.0}
DEADLINE=${DEADLINE:-20}        # per-hop recv deadline [ms]
DMIN_ABORT=${DMIN_ABORT:-0.87}  # contact line (base half-diag 0.433 x2); see g3_run.sh
SOAK=${SOAK:-0}                 # if >0: after reaching goal, keep running this many more sec

say() { echo "[g4 $(date +%H:%M:%S)] $*" | tee -a "$LOGD/g4.log"; }
die() { say "FAIL: $*"; exit 1; }

odom_field() {
  timeout 5 ros2 topic echo "$1" --once 2>/dev/null | python3 -c "
import sys,re
raw=sys.stdin.read()
m=re.search(r'position:\s*\n\s*x: ([\-\d.e]+)\s*\n\s*y: ([\-\d.e]+)\s*\n\s*z: ([\-\d.e]+)',raw)
q=re.search(r'orientation:\s*\n\s*x: ([\-\d.e]+)\s*\n\s*y: ([\-\d.e]+)\s*\n\s*z: ([\-\d.e]+)\s*\n\s*w: ([\-\d.e]+)',raw)
print(*(m.groups()+q.groups()) if m and q else '')" ;
}

# ---------- phase 0: clean slate ----------
say "phase 0: cleanup (robots: $ROBOTS, deadline=${DEADLINE}ms)"
pkill -9 -x ros2 2>/dev/null; pkill -9 -x admm_agent_node 2>/dev/null
pkill -9 -x ruby 2>/dev/null; pkill -9 -x fleet_centraliz 2>/dev/null; pkill -9 -x fleet_coordinat 2>/dev/null
pkill -9 -x robot_state_pub 2>/dev/null; pkill -9 -x static_transfor 2>/dev/null
pkill -9 -x legged_common_g 2>/dev/null; pkill -9 -x legged_common_b 2>/dev/null
pkill -9 -x parameter_bridg 2>/dev/null
# leaked `ros2 launch` parents have comm=python3 — pkill -x ros2 misses them (see arena_run.sh)
pkill -9 -f "launch[.]py" 2>/dev/null
sleep 2
rm -rf /dev/shm/fastrtps* /dev/shm/fast_datasharing* /dev/shm/sem.fastrtps* 2>/dev/null

mkdir -p /tmp/legged_robot_ocs2
xacro $WS/install/vision60_description/share/vision60_description/urdf/vision60/robot.xacro \
  robot_type:=vision60 robot_name:=legged_robot > /tmp/legged_robot_ocs2/vision60.urdf || die "xacro"

# ---------- phase 1: gazebo + SETTLE ----------
say "phase 1: gazebo up"
setsid ros2 launch legged_gazebo gazebo.launch.py robots_config_file:=$ROSTER \
  > "$LOGD/gazebo.log" 2>&1 &
for i in $(seq 1 90); do
  ALL=1
  for r in $ROBOTS; do F=$(odom_field /robot$r/hardware/odom); [ -n "$F" ] || { ALL=0; break; }; done
  [ "$ALL" = 1 ] && break; sleep 2
done
[ "$ALL" = 1 ] || die "odom missing for robot$r after 180s"
say "all odoms flowing"
declare -A SA
for i in $(seq 1 20); do
  for r in $ROBOTS; do SA[$r]="$(odom_field /robot$r/hardware/odom)"; done
  sleep 3
  OK=1
  for r in $ROBOTS; do
    a=(${SA[$r]}); b=($(odom_field /robot$r/hardware/odom))
    ok=$(python3 -c "print(1 if abs(${b[3]:-1})<0.1 and abs(${b[4]:-1})<0.1 and abs(${a[2]:-9}-${b[2]:-0})<0.005 else 0)")
    [ "$ok" = 1 ] || { OK=0; break; }
  done
  [ "$OK" = 1 ] && break
done
[ "$OK" = 1 ] || die "fleet never settled upright"
say "all settled upright"

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
[ "$ALL" = 1 ] || die "controllers not all active (last robot$r=$ST)"
say "all controllers ACTIVE"

# ---------- phase 3: distributed ADMM agents (one process per dog) ----------
GZMARK=$(wc -l < "$LOGD/gazebo.log")
gz_deactivated() { tail -n +$((GZMARK + 1)) "$LOGD/gazebo.log" | grep -q "Deactivating"; }
IDS="[${ROBOTS// /, }]"
say "phase 3: distributed agents (ids=$IDS v=$V deadline=${DEADLINE}ms)"
setsid ros2 launch legged_admm_fleet admm_fleet.launch.py mode:=distributed \
  robot_ids:="$IDS" v:=$V hop_deadline_ms:=$DEADLINE > "$LOGD/admm.log" 2>&1 &
setsid python3 $WS/src/legged_fleet/legged_admm_fleet/scripts/g3_dist_logger.py \
  "$ROBOTS" "$LOGD/dist.csv" 1.3 --ros-args -p use_sim_time:=true > "$LOGD/dist_logger.log" 2>&1 &  # 1.3 = admm::D_MIN
DLPID=$!
trap 'kill -9 $DLPID 2>/dev/null' EXIT
# agents stand each dog in place (goal := current odom). Wait for standing.
for i in $(seq 1 30); do
  ALL=1
  for r in $ROBOTS; do
    C=($(odom_field /robot$r/controller/odom))
    z=$(python3 -c "print(1 if ${C[2]:-0}>0.42 else 0)"); [ "$z" = 1 ] || { ALL=0; break; }
  done
  [ "$ALL" = 1 ] && break
  gz_deactivated && die "WBC deactivated during stand-up"
  sleep 3
done
[ "$ALL" = 1 ] || die "robot$r did not stand (z=${C[2]:-?})"
say "all STANDING (distributed)"

# ---------- phase 4: trot + /formation/goal ----------
say "phase 4: trot + /formation/goal centroid+$GOAL_X"
# Send trot 3x per dog over ~3s: a bare one-shot races discovery and a dog that misses it
# stays in STANCE — it then chases its goal but can't step (the "random follower stuck at
# spawn", NOT a goal-delivery problem). Plain repeated --once, no -w (which hung phase 4).
# persistent 8 s / 2 Hz per dog (arena_run pattern) — 3x --once still lost the race
# occasionally (robot2 stuck in STANCE, 2026-07-23); re-sending trot is idempotent.
for r in $ROBOTS; do
  timeout 8 ros2 topic pub -r 2 /robot$r/cmd_gait std_msgs/msg/String "{data: trot}" >/dev/null 2>&1 &
done
sleep 9
POSLINE=""; for r in $ROBOTS; do C=($(odom_field /robot$r/controller/odom)); POSLINE="$POSLINE ${C[0]:-99} ${C[1]:-99}"; done
read GX GY <<< "$(python3 -c "
v=[float(x) for x in '''$POSLINE'''.split()]; pts=list(zip(v[::2],v[1::2]))
print(f'{sum(p[0] for p in pts)/len(pts)+$GOAL_X:.3f} {sum(p[1] for p in pts)/len(pts):.3f}')")"
say "formation goal = ($GX,$GY)"
# persistent burst, NOT --once: the coordinator's /formation/goal sub is volatile and a
# one-shot pub races DDS discovery (bit us 2026-07-23: coordinator never got the goal,
# dogs stood at spawn the whole run). Same pattern as arena_run/g5_sweep.
timeout 8 ros2 topic pub -r 5 /formation/goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {position: {x: $GX, y: $GY, z: 0.5}}}" >/dev/null 2>&1

T_WALL0=$(date +%s); T_SIM0=$(timeout 4 ros2 topic echo /clock --once 2>/dev/null | grep -m1 sec: | awk '{print $2}')
R=0
for i in $(seq 1 120); do
  POSLINE=""; for r in $ROBOTS; do C=($(odom_field /robot$r/controller/odom)); POSLINE="$POSLINE ${C[0]:-99} ${C[1]:-99}"; done
  D=$(python3 -c "
import math
v=[float(x) for x in '''$POSLINE'''.split()]; pts=list(zip(v[::2],v[1::2]))
cx=sum(p[0] for p in pts)/len(pts); cy=sum(p[1] for p in pts)/len(pts)
print(f'{math.hypot(cx-$GX, cy-$GY):.3f}')")
  MIND=$(tail -1 "$LOGD/dist.csv" 2>/dev/null | cut -d, -f8); case "$MIND" in ''|*[!0-9.-]*) MIND=99;; esac
  say "  pos=($POSLINE ) centroid_dist=$D min_pair=$MIND"
  BAD=$(python3 -c "print(1 if $MIND<$DMIN_ABORT else 0)")
  [ "$BAD" = 1 ] && die "pairwise $MIND < $DMIN_ABORT (collision guard)"
  R=$(python3 -c "print(1 if $D<0.30 else 0)"); [ "$R" = 1 ] && break
  gz_deactivated && die "WBC deactivated mid-walk (pos=$POSLINE)"
  sleep 5
done
T_WALL1=$(date +%s); T_SIM1=$(timeout 4 ros2 topic echo /clock --once 2>/dev/null | grep -m1 sec: | awk '{print $2}')
RTF=$(python3 -c "w=$T_WALL1-$T_WALL0;s=${T_SIM1:-0}-${T_SIM0:-0};print(f'{s/w:.2f}' if w>0 else '?')")
[ "$R" = 1 ] || die "centroid not at goal in 600s (last dist=$D)"

# Barrier stats are informational; "the logger produced nothing" is not — a run whose
# collision guard never ran must not claim PASS (this was a WARN, which nothing read).
python3 "$WS/src/legged_fleet/legged_admm_fleet/scripts/dist_summary.py" "$LOGD/dist.csv" \
  || die "collision guard produced no data — PASS not claimable"
# achieved_rounds sample (G5 telemetry sanity)
AR=$(timeout 5 ros2 topic echo /robot1/admm/stats --once 2>/dev/null | grep achieved_rounds | awk '{print $2}')
say "G4 PASS: centroid reached ($GX,$GY); min_pair>=$DMIN_ABORT; RTF=$RTF; achieved_rounds(r1)=${AR:-?}; logs $LOGD"

# ---------- optional soak: keep running to check for deadlock over time ----------
if [ "$SOAK" -gt 0 ]; then
  say "soak: holding formation for ${SOAK}s (deadlock watch)"
  END=$(( $(date +%s) + SOAK ))
  while [ "$(date +%s)" -lt "$END" ]; do
    MIND=$(tail -1 "$LOGD/dist.csv" 2>/dev/null | cut -d, -f8); case "$MIND" in ''|*[!0-9.-]*) MIND=99;; esac
    AR=$(timeout 5 ros2 topic echo /robot1/admm/stats --once 2>/dev/null | grep achieved_rounds | awk '{print $2}')
    say "  soak min_pair=$MIND achieved_rounds(r1)=${AR:-?}"
    gz_deactivated && die "WBC deactivated during soak"
    sleep 30
  done
  say "SOAK PASS: ${SOAK}s no deadlock, no WBC blowup"
fi

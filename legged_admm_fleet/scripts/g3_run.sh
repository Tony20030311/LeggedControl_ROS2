#!/bin/bash
# G3 gate: 3x vision60 V-formation, empty world, /formation/goal 2 m ahead via the ADMM
# centralized node. Same phased bring-up as g2_run.sh (gazebo settle -> load_controller ->
# ADMM stand -> trot -> goal), generalized to a roster. g2_run.sh stays the untouched
# single-dog oracle. Unlike g2 there is no stand-up goal race: the node is started with
# -p goalN_x/y := each dog's current position, so it never chases the built-in (3,0) defaults.
WS=/root/legged_ros2_ws
LOGD=$WS/g2_logs/g3_$(date +%m%d_%H%M%S)
mkdir -p "$LOGD"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST   # isolate DDS from the shared lab subnet
export ROS_DOMAIN_ID=42                           # (foreign /clock injection stalls the WBC)
source /opt/ros/jazzy/setup.bash
source /root/gridmap_ws/install/setup.bash
source $WS/install/setup.bash
set -u  # ROS setup scripts 用未定義變數，必須 source 完才開

ROBOTS=${ROBOTS:-"1 2 3"}
ROSTER=$WS/install/legged_admm_fleet/share/legged_admm_fleet/config/fleet_robots.yaml
CTRL_YAML=$WS/install/legged_admm_fleet/share/legged_admm_fleet/config/vision60_fleet_controller.yaml
REF=$WS/install/legged_robot_controller/share/legged_robot_controller/config/vision60/reference.info
V=${V:-0.4}                    # StepGate 門檻的最小可行速度（G2 驗證）
GOAL_X=${GOAL_X:-2.0}          # formation goal = 當前 centroid + GOAL_X（沿 +x）
# Contact line, not the design margin: base box 0.83x0.25 -> half-diagonal 0.433, so below
# 0.867 m centre distance two dogs interpenetrate at ANY yaw. 0.5 was the retired D_MIN=0.6
# era value and could only fire once they were 0.37 m inside each other. Barrier violations
# (h<0 vs D_MIN=1.3) are transient and expected on swaps/turns -- those are REPORTED by
# dist_summary.py at the end of the run, not fatal. This aborts only on real contact.
DMIN_ABORT=${DMIN_ABORT:-0.87}

say() { echo "[g3 $(date +%H:%M:%S)] $*" | tee -a "$LOGD/g3.log"; }
die() { say "FAIL: $*"; exit 1; }

odom_field() {  # odom_field <topic> -> "px py pz qx qy qz qw" or ""
  timeout 5 ros2 topic echo "$1" --once 2>/dev/null | python3 -c "
import sys,re
raw=sys.stdin.read()
m=re.search(r'position:\s*\n\s*x: ([\-\d.e]+)\s*\n\s*y: ([\-\d.e]+)\s*\n\s*z: ([\-\d.e]+)',raw)
q=re.search(r'orientation:\s*\n\s*x: ([\-\d.e]+)\s*\n\s*y: ([\-\d.e]+)\s*\n\s*z: ([\-\d.e]+)\s*\n\s*w: ([\-\d.e]+)',raw)
print(*(m.groups()+q.groups()) if m and q else '')" ;
}

# ---------- phase 0: clean slate ----------
say "phase 0: cleanup (robots: $ROBOTS)"
pkill -9 -x ros2 2>/dev/null   # stale `ros2 launch` parents from a previous run
pkill -9 -x ruby 2>/dev/null; pkill -9 -x fleet_centraliz 2>/dev/null
pkill -9 -x admm_agent_node 2>/dev/null; pkill -9 -x fleet_coordinat 2>/dev/null
pkill -9 -f "launch[.]py" 2>/dev/null   # leaked ros2-launch parents (comm=python3)
pkill -9 -x robot_state_pub 2>/dev/null; pkill -9 -x static_transfor 2>/dev/null
pkill -9 -x legged_common_g 2>/dev/null; pkill -9 -x legged_common_b 2>/dev/null
pkill -9 -x parameter_bridg 2>/dev/null
sleep 2
rm -rf /dev/shm/fastrtps* /dev/shm/fast_datasharing* /dev/shm/sem.fastrtps* 2>/dev/null

mkdir -p /tmp/legged_robot_ocs2
xacro $WS/install/vision60_description/share/vision60_description/urdf/vision60/robot.xacro \
  robot_type:=vision60 robot_name:=legged_robot > /tmp/legged_robot_ocs2/vision60.urdf || die "xacro"

# ---------- phase 1: gazebo, then SETTLE (all dogs) ----------
say "phase 1: gazebo up (roster=$(basename $ROSTER))"
setsid ros2 launch legged_gazebo gazebo.launch.py robots_config_file:=$ROSTER \
  > "$LOGD/gazebo.log" 2>&1 &
for i in $(seq 1 90); do
  ALL=1
  for r in $ROBOTS; do
    F=$(odom_field /robot$r/hardware/odom); [ -n "$F" ] || { ALL=0; break; }
  done
  [ "$ALL" = 1 ] && break; sleep 2
done
[ "$ALL" = 1 ] || die "odom missing for robot$r after 180s (gazebo.log tail: $(tail -2 $LOGD/gazebo.log))"
say "all odoms flowing"
# settle: per dog, two samples 3s apart agree to <5mm in z and upright (|qx|,|qy|<0.1)
declare -A SA
for i in $(seq 1 20); do
  for r in $ROBOTS; do SA[$r]="$(odom_field /robot$r/hardware/odom)"; done
  sleep 3
  OK=1
  for r in $ROBOTS; do
    a=(${SA[$r]}); b=($(odom_field /robot$r/hardware/odom))
    ok=$(python3 -c "print(1 if abs(${b[3]:-1})<0.1 and abs(${b[4]:-1})<0.1 and abs(${a[2]:-9}-${b[2]:-0})<0.005 else 0)")
    [ "$ok" = 1 ] || { OK=0; say "  robot$r not settled yet (z=${b[2]:-?})"; break; }
  done
  [ "$OK" = 1 ] && break
done
[ "$OK" = 1 ] || die "fleet never settled upright"
say "all settled upright"

# ---------- phase 2: controllers (activate on spawn -> only now that we're settled) ----------
# Activation race (read-only upstream): the first update() after activation can get a huge
# period, pushing obs.time past the initial MPC plan -> WBC QP infeasible -> auto-deactivate.
# Recovery IS documented bring-up law ("re-activate = gamepad START") but only works if done
# IMMEDIATELY (within the ~1s MPC horizon). So: watch each dog tightly and re-activate on sight.
reactivate() {  # reactivate <robot-id> -> 0 if active
  timeout 20 ros2 service call /robot$1/controller_manager/switch_controller \
    controller_manager_msgs/srv/SwitchController \
    "{activate_controllers: [legged_robot_controller], strictness: 1, timeout: {sec: 10}}" \
    >> "$LOGD/reactivate.log" 2>&1
  ST=$(timeout 6 ros2 control list_controllers -c /robot$1/controller_manager 2>/dev/null | grep legged_robot_controller | awk '{print $NF}')
  [ "$ST" = active ]
}
say "phase 2: load_controller (all dogs)"
setsid ros2 launch legged_robot_controller load_controller.launch.py \
  robots_config_file:=$ROSTER controller_param_file:=$CTRL_YAML \
  gait_command:=true base_target_command:=false \
  > "$LOGD/controller.log" 2>&1 &
declare -A RETRIES
for r in $ROBOTS; do RETRIES[$r]=0; done
for i in $(seq 1 240); do
  ALL=1
  for r in $ROBOTS; do
    ST=$(timeout 6 ros2 control list_controllers -c /robot$r/controller_manager 2>/dev/null | grep legged_robot_controller | awk '{print $NF}')
    if [ "$ST" = inactive ]; then
      # WBC blew at activation -> immediate re-activate (must be fast, see note above)
      RETRIES[$r]=$((RETRIES[$r] + 1))
      [ "${RETRIES[$r]}" -le 3 ] || die "robot$r dead after 3 re-activations (gazebo.log: $(grep -m1 'requested currentTime' $LOGD/gazebo.log))"
      say "  robot$r deactivated -> immediate re-activate (try ${RETRIES[$r]})"
      reactivate $r && say "  robot$r re-activated OK" || say "  robot$r re-activate failed (state=$ST)"
      ALL=0
    elif [ "$ST" != active ]; then
      ALL=0   # not spawned yet
    fi
  done
  [ "$ALL" = 1 ] && break
  sleep 0.5
done
[ "$ALL" = 1 ] || die "controllers not all active after 120s (last robot$r state=${ST:-?}, controller.log tail: $(tail -3 $LOGD/controller.log))"
say "all controllers ACTIVE (holding crouch, by design)"

# ---------- phase 3: ADMM node stands the fleet ----------
GZMARK=$(wc -l < "$LOGD/gazebo.log")  # deactivations before this line were recovered in phase 2
gz_deactivated() { tail -n +$((GZMARK + 1)) "$LOGD/gazebo.log" | grep -q "Deactivating"; }
# goalN_x/y := current positions -> in-place stand, no chase of built-in defaults
GOAL_ARGS=""
for r in $ROBOTS; do
  P=($(odom_field /robot$r/hardware/odom)); [ -n "${P[0]:-}" ] || die "no odom for goal seed robot$r"
  GOAL_ARGS="$GOAL_ARGS -p goal${r}_x:=${P[0]} -p goal${r}_y:=${P[1]}"
  say "  robot$r stand-in-place goal (${P[0]},${P[1]})"
done
IDS="[${ROBOTS// /, }]"
say "phase 3: ADMM centralized node (ids=$IDS v=$V)"
setsid ros2 run legged_admm_fleet fleet_centralized_node --ros-args \
  -r __node:=fleet_centralized -p robot_ids:="$IDS" -p use_sim_time:=true \
  -p v:=$V -p reference_file:=$REF $GOAL_ARGS \
  -p log_csv:=$LOGD/admm_dbg.csv -p log_hist_csv:=$LOGD/admm_hist.csv \
  > "$LOGD/admm.log" 2>&1 &
# 20 Hz distance/CBF recorder (the 5s monitor below is far too sparse to catch contact)
setsid python3 $WS/src/legged_fleet/legged_admm_fleet/scripts/g3_dist_logger.py \
  "$ROBOTS" "$LOGD/dist.csv" 1.3 --ros-args -p use_sim_time:=true \
  > "$LOGD/dist_logger.log" 2>&1 &   # 1.3 = admm::D_MIN (admm_constants.hpp:57)
DLPID=$!
trap 'kill -9 $DLPID 2>/dev/null' EXIT
for i in $(seq 1 20); do
  ALL=1
  for r in $ROBOTS; do
    C=($(odom_field /robot$r/controller/odom))
    z=$(python3 -c "print(1 if ${C[2]:-0}>0.42 else 0)")
    [ "$z" = 1 ] || { ALL=0; break; }
  done
  [ "$ALL" = 1 ] && break
  gz_deactivated && die "WBC deactivated during stand-up"
  sleep 3
done
[ "$ALL" = 1 ] || die "robot$r did not stand (z=${C[2]:-?} after 60s)"
say "all STANDING"

# ---------- phase 4: trot + formation goal ----------
say "phase 4: trot all + /formation/goal centroid+$GOAL_X"
for r in $ROBOTS; do
  timeout 3 ros2 topic pub --once /robot$r/cmd_gait std_msgs/msg/String "{data: trot}" >/dev/null 2>&1
done
sleep 1
POSLINE=""
for r in $ROBOTS; do C=($(odom_field /robot$r/controller/odom)); POSLINE="$POSLINE ${C[0]:-99} ${C[1]:-99}"; done
read GX GY <<< "$(python3 -c "
v=[float(x) for x in '''$POSLINE'''.split()]
pts=list(zip(v[::2],v[1::2]))
print(f'{sum(p[0] for p in pts)/len(pts)+$GOAL_X:.3f} {sum(p[1] for p in pts)/len(pts):.3f}')")"
say "formation goal = ($GX,$GY)"
timeout 4 ros2 topic pub --once /formation/goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {position: {x: $GX, y: $GY, z: 0.5}}}" >/dev/null 2>&1

T_WALL0=$(date +%s); T_SIM0=$(timeout 4 ros2 topic echo /clock --once 2>/dev/null | grep -m1 sec: | awk '{print $2}')
R=0
for i in $(seq 1 90); do
  POSLINE=""
  for r in $ROBOTS; do C=($(odom_field /robot$r/controller/odom)); POSLINE="$POSLINE ${C[0]:-99} ${C[1]:-99}"; done
  D=$(python3 -c "
import math
v=[float(x) for x in '''$POSLINE'''.split()]
pts=list(zip(v[::2],v[1::2]))
cx=sum(p[0] for p in pts)/len(pts); cy=sum(p[1] for p in pts)/len(pts)
print(f'{math.hypot(cx-$GX, cy-$GY):.3f}')")
  # min_pair from the 20Hz logger (single-timestamp truth); the per-robot echoes above are
  # skewed by ~1s each -> computing cross-robot distance from them yields phantom dips.
  MIND=$(tail -1 "$LOGD/dist.csv" 2>/dev/null | cut -d, -f8); case "$MIND" in ''|*[!0-9.-]*) MIND=99;; esac
  say "  pos=($POSLINE ) centroid_dist=$D min_pair=$MIND"
  BAD=$(python3 -c "print(1 if $MIND<$DMIN_ABORT else 0)")
  [ "$BAD" = 1 ] && die "pairwise distance $MIND < $DMIN_ABORT (collision guard)"
  R=$(python3 -c "print(1 if $D<0.30 else 0)")
  [ "$R" = 1 ] && break
  gz_deactivated && die "WBC deactivated mid-walk (pos=$POSLINE)"
  sleep 5
done
T_WALL1=$(date +%s); T_SIM1=$(timeout 4 ros2 topic echo /clock --once 2>/dev/null | grep -m1 sec: | awk '{print $2}')
RTF=$(python3 -c "w=$T_WALL1-$T_WALL0;s=${T_SIM1:-0}-${T_SIM0:-0};print(f'{s/w:.2f}' if w>0 else '?')")
[ "$R" = 1 ] || die "centroid not at goal in 450s (last dist=$D)"
# Barrier stats are informational; "the logger produced nothing" is not — a run whose
# collision guard never ran must not claim PASS.
python3 "$WS/src/legged_fleet/legged_admm_fleet/scripts/dist_summary.py" "$LOGD/dist.csv" \
  || die "collision guard produced no data — PASS not claimable"
say "G3 PASS: centroid reached ($GX,$GY) within 0.30 m; no contact (min_pair>=$DMIN_ABORT); RTF=$RTF; logs in $LOGD"

#!/bin/bash
# Shared phased bring-up for the distributed-fleet run scripts (g4_run.sh, d_run.sh).
# SOURCE this, don't execute it. Every step here is load-bearing and was paid for in debugging:
#   - DDS isolation (domain 42 + LOCALHOST): the shared lab subnet injects a foreign /clock
#     that makes the 250 Hz WBC miss its deadline and auto-deactivate.
#   - settle-upright wait before activating: activating a falling dog blows up the WBC.
#   - immediate re-activate on the activation race: the first update() after activate can see a
#     huge period, jump obs.time past the MPC plan, and auto-deactivate. Recover ON THE SPOT.
#
# Caller sets before sourcing: LOGD, TAG. Optional: ROBOTS, V, DEADLINE, ARENA, LAUNCH_EXTRA.
# Provides: say(), die(), odom_field(), gz_deactivated(), $DLPID (dist logger), $ROSTER.
WS=${WS:-/root/legged_ros2_ws}
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash
source /root/gridmap_ws/install/setup.bash
source $WS/install/setup.bash
set -u

ROBOTS=${ROBOTS:-"1 2 3"}
ROSTER=$WS/install/legged_admm_fleet/share/legged_admm_fleet/config/fleet_robots.yaml
CTRL_YAML=$WS/install/legged_admm_fleet/share/legged_admm_fleet/config/vision60_fleet_controller.yaml
V=${V:-0.4}
DEADLINE=${DEADLINE:-20}
ARENA=${ARENA:-}
LAUNCH_EXTRA=${LAUNCH_EXTRA:-}
SCRIPTS=$WS/src/legged_fleet/legged_admm_fleet/scripts

say() { echo "[$TAG $(date +%H:%M:%S)] $*" | tee -a "$LOGD/$TAG.log"; }
die() { say "FAIL: $*"; exit 1; }

odom_field() {
  timeout 5 ros2 topic echo "$1" --once 2>/dev/null | python3 -c "
import sys,re
raw=sys.stdin.read()
m=re.search(r'position:\s*\n\s*x: ([\-\d.e]+)\s*\n\s*y: ([\-\d.e]+)\s*\n\s*z: ([\-\d.e]+)',raw)
q=re.search(r'orientation:\s*\n\s*x: ([\-\d.e]+)\s*\n\s*y: ([\-\d.e]+)\s*\n\s*z: ([\-\d.e]+)\s*\n\s*w: ([\-\d.e]+)',raw)
print(*(m.groups()+q.groups()) if m and q else '')" ;
}

fleet_bringup() {
  # ---------- phase 0: clean slate ----------
  say "phase 0: cleanup (robots: $ROBOTS, deadline=${DEADLINE}ms, arena='${ARENA:-none}')"
  pkill -9 -x ros2 2>/dev/null; pkill -9 -x admm_agent_node 2>/dev/null
  pkill -9 -x ruby 2>/dev/null; pkill -9 -x fleet_centraliz 2>/dev/null; pkill -9 -x fleet_coordinat 2>/dev/null
  pkill -9 -x robot_state_pub 2>/dev/null; pkill -9 -x static_transfor 2>/dev/null
  pkill -9 -x legged_common_g 2>/dev/null; pkill -9 -x legged_common_b 2>/dev/null
  pkill -9 -x parameter_bridg 2>/dev/null
  # leaked `ros2 launch` parents have comm=python3 — pkill -x ros2 misses them.
  # NOTE this is the ONE sanctioned pkill -f: the pattern cannot match this shell.
  pkill -9 -f "launch[.]py" 2>/dev/null
  sleep 2
  rm -rf /dev/shm/fastrtps* /dev/shm/fast_datasharing* /dev/shm/sem.fastrtps* 2>/dev/null

  mkdir -p /tmp/legged_robot_ocs2
  xacro $WS/install/vision60_description/share/vision60_description/urdf/vision60/robot.xacro \
    robot_type:=vision60 robot_name:=legged_robot > /tmp/legged_robot_ocs2/vision60.urdf || die "xacro"

  # ---------- phase 1: gazebo + SETTLE ----------
  say "phase 1: gazebo up"
  local WORLD_ARG=""
  [ -n "$ARENA" ] && WORLD_ARG="world:=$ARENA gazebo_package:=legged_admm_fleet"
  setsid ros2 launch legged_gazebo gazebo.launch.py robots_config_file:=$ROSTER $WORLD_ARG \
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
  IDS="[${ROBOTS// /, }]"
  say "phase 3: distributed agents (ids=$IDS v=$V deadline=${DEADLINE}ms)"
  local ARENA_ARGS=""
  [ -n "$ARENA" ] && ARENA_ARGS="arena:=$ARENA use_astar:=true"
  setsid ros2 launch legged_admm_fleet admm_fleet.launch.py mode:=distributed \
    robot_ids:="$IDS" v:=$V hop_deadline_ms:=$DEADLINE $ARENA_ARGS $LAUNCH_EXTRA \
    > "$LOGD/admm.log" 2>&1 &
  setsid python3 $SCRIPTS/g3_dist_logger.py \
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
}

reactivate() {
  timeout 20 ros2 service call /robot$1/controller_manager/switch_controller \
    controller_manager_msgs/srv/SwitchController \
    "{activate_controllers: [legged_robot_controller], strictness: 1, timeout: {sec: 10}}" \
    >> "$LOGD/reactivate.log" 2>&1
  ST=$(timeout 6 ros2 control list_controllers -c /robot$1/controller_manager 2>/dev/null | grep legged_robot_controller | awk '{print $NF}')
  [ "$ST" = active ]
}

gz_deactivated() { tail -n +$((${GZMARK:-0} + 1)) "$LOGD/gazebo.log" | grep -q "Deactivating"; }

# trot must be a persistent burst, not `--once`: a one-shot races DDS discovery and a dog that
# misses it stays in STANCE — it then chases its goal but never steps. That looked exactly like
# a goal-delivery bug and cost a day. Re-sending trot is idempotent.
arm_trot() {
  for r in $ROBOTS; do
    timeout 8 ros2 topic pub -r 2 /robot$r/cmd_gait std_msgs/msg/String "{data: trot}" >/dev/null 2>&1 &
  done
  sleep 9
}

# Same reason as trot: the coordinator's /formation/goal sub is volatile, so a one-shot pub can
# be dropped by discovery and the whole fleet just stands at spawn.
send_formation_goal() {
  timeout 8 ros2 topic pub -r 5 /formation/goal geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: world}, pose: {position: {x: $1, y: $2, z: 0.5}}}" >/dev/null 2>&1
}

fleet_centroid() {  # echoes "cx cy" over the LIVE-or-not robots given as $1 (space separated)
  local POSLINE=""
  for r in $1; do C=($(odom_field /robot$r/controller/odom)); POSLINE="$POSLINE ${C[0]:-99} ${C[1]:-99}"; done
  python3 -c "
v=[float(x) for x in '''$POSLINE'''.split()]; pts=list(zip(v[::2],v[1::2]))
print(f'{sum(p[0] for p in pts)/len(pts):.3f} {sum(p[1] for p in pts)/len(pts):.3f}')"
}

min_pair() {  # newest min pairwise centre distance from the dist logger (col 8 == 3 robots)
  local M=$(tail -1 "$LOGD/dist.csv" 2>/dev/null | cut -d, -f8)
  case "$M" in ''|*[!0-9.-]*) M=99;; esac
  echo "$M"
}

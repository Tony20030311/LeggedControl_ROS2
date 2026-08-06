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
# Gazebo cannot resolve the model:// URIs in the robot visuals without this, and the way it
# fails is silent and total: every mesh in the fleet drops out of the RENDER scene while
# physics carries on from the collision boxes. The dogs still walk, the gate still passes,
# and nothing in the log stands out -- but a lidar looks straight through them. Measured
# 2026-08-05: 337 "Unable to find file with URI [model://vision60_description/...]" errors,
# and a peer 1.4 m away returned 22 points (all off its lidar mast, the one part built from
# primitives) against 2104 from the ground it should have been shadowing. With the path set:
# 0 errors, 343 torso returns, and the mast is no longer what the fleet sees of itself.
#
# Only perception cares, which is why this went unnoticed for so long.
export GZ_SIM_RESOURCE_PATH=$(ls -d $WS/install/*/share 2>/dev/null | tr '\n' ':')${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}
source /opt/ros/jazzy/setup.bash
source /root/gridmap_ws/install/setup.bash
source $WS/install/setup.bash
set -u

ROBOTS=${ROBOTS:-"1 2 3"}
# Roster follows the fleet SIZE by default, so `ROBOTS="1 2 3 4 5"` picks up fleet_robots_5.yaml
# without the caller having to know the filename -- and, more to the point, without it being
# possible to ask for five dogs and silently get the three-dog spawn poses. Overridable for a
# roster that is not just "the default layout at size n".
ROSTER=${ROSTER:-$WS/install/legged_admm_fleet/share/legged_admm_fleet/config/fleet_robots$(
  [ "$(echo $ROBOTS | wc -w)" = 3 ] || echo "_$(echo $ROBOTS | wc -w)").yaml}
CTRL_YAML=$WS/install/legged_admm_fleet/share/legged_admm_fleet/config/vision60_fleet_controller.yaml
V=${V:-0.4}
DEADLINE=${DEADLINE:-20}
ARENA=${ARENA:-}
LAUNCH_EXTRA=${LAUNCH_EXTRA:-}
# Where "I observe peer j" comes from; sources are in config/observation_sources.yaml.
# A source with needs_perception wants config/fleet_robots_lidar.yaml as the roster.
OBSERVATION=${OBSERVATION:-truth}
# ANCHOR=false runs the control arm: conservative anchoring off (see the launch arg).
ANCHOR=${ANCHOR:-true}
SCRIPTS=$WS/src/legged_fleet/legged_admm_fleet/scripts
. $SCRIPTS/_perception.sh

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
  # comm is "python3" for these, so no -x pattern reaches them; left alive they keep
  # subscribing to the 500 kB cloud topics and starve the next run's trackers.
  pkill -9 -f "lidar_peer_tracker_node[.]py" 2>/dev/null
  pkill -9 -f "perception_viz[.]py" 2>/dev/null
  sleep 2
  rm -rf /dev/shm/fastrtps* /dev/shm/fast_datasharing* /dev/shm/sem.fastrtps* 2>/dev/null

  mkdir -p /tmp/legged_robot_ocs2
  xacro $WS/install/vision60_description/share/vision60_description/urdf/vision60/robot.xacro \
    robot_type:=vision60 robot_name:=legged_robot > /tmp/legged_robot_ocs2/vision60.urdf || die "xacro"

  # ---------- phase 1: gazebo + SETTLE ----------
  say "phase 1: gazebo up"
  # gazebo_package is now unconditional. It used to ride along only with ARENA, so an arena run
  # got legged_admm_fleet/worlds/<arena>.sdf while a plain run silently got legged_gazebo's
  # empty.sdf -- a DIFFERENT world: 1 ms / 1000 Hz physics against our 2 ms / 500 Hz. That is
  # why the 2026-07-30 step-size change only ever showed up in arena numbers. Measured
  # 2026-08-05: G4, three dogs, no lidar, RTF 0.80 on the 1 ms world and 1.00 on ours.
  local WORLD_ARG="gazebo_package:=legged_admm_fleet"
  [ -n "$ARENA" ] && WORLD_ARG="world:=$ARENA $WORLD_ARG"
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
    robot_ids:="$IDS" v:=$V hop_deadline_ms:=$DEADLINE observation:=$OBSERVATION \
    enable_conservative_anchor:=$ANCHOR \
    $ARENA_ARGS $LAUNCH_EXTRA \
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

  # Whatever the observation source needs (see config/observation_sources.yaml). Here
  # because the dogs are standing and have not been given a goal yet.
  perception_bringup
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
  # Read from dist.csv, the same source min_pair() below already uses, NOT from three
  # `ros2 topic echo --once` processes. Two reasons, both measured:
  #
  #   CORRECTNESS. Each echo pays its own DDS discovery and can lose the race; the previous
  #   version worked around that with a retry and by DROPPING unreadable robots from the mean
  #   (it used to substitute (99,99), which moved the centroid to ~34 m and fired every
  #   "have we passed x=N yet" test instantly -- 2 of 7 LIE runs on 2026-07-29 began their
  #   attack while the fleet was still at x~2, and the runs looked normal afterwards).
  #
  #   LATENCY, which turned out to be the same bug wearing a different hat. Three echoes with
  #   a 5 s timeout each can take ~15 s, and the fleet walks 6 m in that time: on 2026-08-01
  #   a run asked for KILL_AT_X=3.0 and triggered at centroid x=6.9. Where the attack lands
  #   decides what the run tests, so a centroid that takes 15 s to read is not fit for the job.
  #   The dist logger already writes every robot's position at 20 Hz.
  #
  # Staleness is a real failure mode (a dead logger would otherwise freeze the centroid at a
  # position that silently passes every threshold), so a file older than 2 s is treated as
  # unreadable. Unknown must FAIL a threshold, never pass it: this echoes nan, which loses
  # every `>=` and leaves the caller waiting and eventually timing out loudly.
  python3 - "$LOGD/dist.csv" $1 <<'PY'
import csv, os, sys, time
path, ids = sys.argv[1], sys.argv[2:]
try:
    if time.time() - os.path.getmtime(path) > 2.0:
        raise OSError("stale")
    with open(path) as f:
        row = None
        for row in csv.DictReader(f):
            pass
    pts = [(float(row["x%s" % i]), float(row["y%s" % i])) for i in ids]
    print("%.3f %.3f" % (sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts)))
except Exception:
    print("nan nan")
PY
}

# Newest min pairwise centre distance from the dist logger, located BY NAME in the header rather
# than by position. The column index is 8 for three robots and 12 for five (t + 2 per robot +
# min_pair + min_h), and the hardcoded 8 does not fail loudly at N=5 -- it silently returns x4, a
# world coordinate, as if it were a separation. Measured on d_0801_080532: the collision guard
# aborted a five-dog run on "pairwise 0.8108 < 0.90" while dist.csv's own worst min_pair for the
# whole run was 1.4468 and the true body gap never came below 0.6007 m. A perfect pentagon, failed
# for a robot's x coordinate.
min_pair() {
  local M=$(awk -F, 'NR==1{for(i=1;i<=NF;i++) if($i=="min_pair") c=i; next} {v=$c} END{print v}' \
            "$LOGD/dist.csv" 2>/dev/null)
  case "$M" in ''|*[!0-9.-]*) M=99;; esac
  echo "$M"
}

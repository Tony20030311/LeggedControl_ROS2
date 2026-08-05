#!/bin/bash
# G2 gate: single vision60, empty world, walk to a 2 m goal via the ADMM centralized node.
# Runs INSIDE the legged_stack container. Encodes the official bring-up order learned from
# the legged_stack read-through:
#   gazebo (settle!) -> load_controller (auto-activates; must happen AFTER settle)
#   -> ADMM node (targets z=comHeight stand the dog; it never stands by itself)
#   -> per-dog goal (kills the V-slot default) -> trot (StepGate needs foothold err > 5cm => v>=0.4)
WS=/root/legged_ros2_ws
LOGD=$WS/g2_logs/$(date +%m%d_%H%M%S)
mkdir -p "$LOGD"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST   # domain-0 cross-talk = WBC killer (see CLAUDE.md)
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash
source /root/gridmap_ws/install/setup.bash
source $WS/install/setup.bash
set -u  # ROS setup scripts 用未定義變數，必須 source 完才開

ROSTER=$WS/install/legged_admm_fleet/share/legged_admm_fleet/config/single_robot.yaml
CTRL_YAML=$WS/install/legged_admm_fleet/share/legged_admm_fleet/config/vision60_fleet_controller.yaml
REF=$WS/install/legged_robot_controller/share/legged_robot_controller/config/vision60/reference.info
V=${V:-0.4}          # ponytail: 0.4 m/s = StepGate 門檻的最小可行速度,P2 還會再調
GOAL_X=${GOAL_X:-2.0}

say() { echo "[g2 $(date +%H:%M:%S)] $*" | tee -a "$LOGD/g2.log"; }
die() { say "FAIL: $*"; exit 1; }

odom_field() {  # odom_field <grep-anchor> — one float from /robot1/controller/odom or hardware/odom
  timeout 5 ros2 topic echo "$1" --once 2>/dev/null | python3 -c "
import sys,re
raw=sys.stdin.read()
m=re.search(r'position:\s*\n\s*x: ([\-\d.e]+)\s*\n\s*y: ([\-\d.e]+)\s*\n\s*z: ([\-\d.e]+)',raw)
q=re.search(r'orientation:\s*\n\s*x: ([\-\d.e]+)\s*\n\s*y: ([\-\d.e]+)\s*\n\s*z: ([\-\d.e]+)\s*\n\s*w: ([\-\d.e]+)',raw)
print(*(m.groups()+q.groups()) if m and q else '')" ;
}

# ---------- phase 0: clean slate ----------
say "phase 0: cleanup"
pkill -9 -x ruby 2>/dev/null; pkill -9 -x fleet_centraliz 2>/dev/null
pkill -9 -x admm_agent_node 2>/dev/null; pkill -9 -x fleet_coordinat 2>/dev/null
pkill -9 -x ros2 2>/dev/null; pkill -9 -f "launch[.]py" 2>/dev/null
pkill -9 -x robot_state_pub 2>/dev/null; pkill -9 -x static_transfor 2>/dev/null
pkill -9 -x legged_common_g 2>/dev/null; pkill -9 -x legged_common_b 2>/dev/null
pkill -9 -x parameter_bridg 2>/dev/null
sleep 2
rm -rf /dev/shm/fastrtps* /dev/shm/fast_datasharing* /dev/shm/sem.fastrtps* 2>/dev/null

# URDF for the controller (upstream expects it pre-generated)
mkdir -p /tmp/legged_robot_ocs2
xacro $WS/install/vision60_description/share/vision60_description/urdf/vision60/robot.xacro \
  robot_type:=vision60 robot_name:=legged_robot > /tmp/legged_robot_ocs2/vision60.urdf || die "xacro"

# ---------- phase 1: gazebo, then SETTLE ----------
say "phase 1: gazebo up"
# gazebo_package:=legged_admm_fleet -- ours is 2 ms / 500 Hz physics, legged_gazebo's
# empty.sdf (the old implicit default) is 1 ms / 1000 Hz. See g4_run.sh for the numbers.
setsid ros2 launch legged_gazebo gazebo.launch.py robots_config_file:=$ROSTER \
  gazebo_package:=legged_admm_fleet \
  > "$LOGD/gazebo.log" 2>&1 &
GZPID=$!
for i in $(seq 1 60); do
  F=$(odom_field /robot1/hardware/odom); [ -n "$F" ] && break; sleep 2
done
[ -n "${F:-}" ] || die "no /robot1/hardware/odom after 120s (gazebo.log tail: $(tail -2 $LOGD/gazebo.log))"
say "odom flowing: $F"
# settle: two samples 3s apart must agree to <5mm in z and be upright (|qx|,|qy|<0.1)
for i in $(seq 1 20); do
  A=($(odom_field /robot1/hardware/odom)); sleep 3; B=($(odom_field /robot1/hardware/odom))
  DZ=$(python3 -c "print(abs(${A[2]:-9}-${B[2]:-0}))")
  OK=$(python3 -c "print(1 if abs(${B[3]:-1})<0.1 and abs(${B[4]:-1})<0.1 and $DZ<0.005 else 0)")
  [ "$OK" = 1 ] && break
done
[ "$OK" = 1 ] || die "robot never settled upright (last: ${B[*]:-none})"
say "settled at z=${B[2]} (upright)"

# ---------- phase 2: controller (activates immediately -> only now that we're settled) ----------
say "phase 2: load_controller"
setsid ros2 launch legged_robot_controller load_controller.launch.py \
  robots_config_file:=$ROSTER controller_param_file:=$CTRL_YAML \
  gait_command:=true base_target_command:=false \
  > "$LOGD/controller.log" 2>&1 &
for i in $(seq 1 40); do
  ST=$(timeout 6 ros2 control list_controllers -c /robot1/controller_manager 2>/dev/null | grep legged_robot_controller | awk '{print $NF}')
  [ "$ST" = active ] && break
  grep -q "Deactivating" "$LOGD/gazebo.log" && die "WBC blew up at activation (gazebo.log)"
  sleep 3
done
[ "$ST" = active ] || die "controller not active (state=$ST, controller.log tail: $(tail -3 $LOGD/controller.log))"
say "controller ACTIVE (holding crouch, by design)"

# ---------- phase 3: ADMM node stands the dog ----------
say "phase 3: ADMM centralized node (v=$V)"
setsid ros2 run legged_admm_fleet fleet_centralized_node --ros-args \
  -r __node:=fleet_centralized -p robot_ids:='[1]' -p use_sim_time:=true \
  -p v:=$V -p reference_file:=$REF \
  > "$LOGD/admm.log" 2>&1 &
sleep 3
# per-dog goal at current pos: disables the V-slot default, target z=comHeight -> stand
P=($(odom_field /robot1/controller/odom)); [ -n "${P[0]:-}" ] || P=($(odom_field /robot1/hardware/odom))
timeout 3 ros2 topic pub --once /robot1/goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {position: {x: ${P[0]}, y: ${P[1]}, z: 0.5}}}" >/dev/null 2>&1
for i in $(seq 1 20); do
  C=($(odom_field /robot1/controller/odom))
  Z_OK=$(python3 -c "print(1 if ${C[2]:-0}>0.42 else 0)")
  [ "$Z_OK" = 1 ] && break
  grep -q "Deactivating" "$LOGD/gazebo.log" && die "WBC deactivated during stand-up"
  sleep 3
done
[ "$Z_OK" = 1 ] || die "did not stand (z=${C[2]:-?} after 60s)"
say "STANDING at z=${C[2]}"

# ---------- phase 4: trot + 2m goal ----------
say "phase 4: trot + goal x=$GOAL_X"
timeout 3 ros2 topic pub --once /robot1/cmd_gait std_msgs/msg/String "{data: trot}" >/dev/null 2>&1
sleep 1
timeout 3 ros2 topic pub --once /robot1/goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {position: {x: $GOAL_X, y: 0.0, z: 0.5}}}" >/dev/null 2>&1
T_WALL0=$(date +%s); T_SIM0=$(timeout 4 ros2 topic echo /clock --once 2>/dev/null | grep -m1 sec: | awk '{print $2}')
for i in $(seq 1 60); do
  C=($(odom_field /robot1/controller/odom))
  D=$(python3 -c "import math;print(f'{math.hypot(${C[0]:-99}-$GOAL_X,${C[1]:-99}):.3f}')")
  say "  pos=(${C[0]},${C[1]}) z=${C[2]} dist_to_goal=$D"
  R=$(python3 -c "print(1 if $D<0.30 else 0)")
  [ "$R" = 1 ] && break
  grep -q "Deactivating" "$LOGD/gazebo.log" && die "WBC deactivated mid-walk at pos=(${C[0]},${C[1]})"
  sleep 5
done
T_WALL1=$(date +%s); T_SIM1=$(timeout 4 ros2 topic echo /clock --once 2>/dev/null | grep -m1 sec: | awk '{print $2}')
RTF=$(python3 -c "w=$T_WALL1-$T_WALL0;s=${T_SIM1:-0}-${T_SIM0:-0};print(f'{s/w:.2f}' if w>0 else '?')")
[ "$R" = 1 ] || die "goal not reached in 300s (last dist=$D)"
say "G2 PASS: reached ($GOAL_X,0) within 0.30 m; RTF=$RTF; logs in $LOGD"

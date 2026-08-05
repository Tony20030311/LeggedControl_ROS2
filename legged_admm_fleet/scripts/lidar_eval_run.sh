#!/bin/bash
# Score the lidar peer tracker against the ground truth it is meant to replace.
#
# Runs the ordinary G4 gate on the lidar roster, then bolts three things onto the live
# fleet: the gz->ROS bridges for the point clouds, one lidar_peer_tracker_node per dog,
# and lidar_tracker_eval.py. Nothing here touches the control path -- the trackers publish
# to /robot<i>/perceived/robot<j>, which nothing subscribes to yet. The gate's own verdict
# is reported too, so a run that quietly broke the fleet cannot pass as a perception result.
#
#   bash scripts/lidar_eval_run.sh [seconds]      # default 60 s of scoring
#
# Exits nonzero if the gate failed, if the bridges never delivered a cloud, or if the
# evaluation's own verdict is not PASS. A quiet failure that reports plausible numbers is
# the one outcome this must never produce.
set +u
WS=/root/legged_ros2_ws
PKG=$WS/install/legged_admm_fleet/share/legged_admm_fleet
SECONDS_EVAL=${1:-60}
ROBOTS=${ROBOTS:-"1 2 3"}
ROSTER=$PKG/config/fleet_robots_lidar.yaml
LOGD=$WS/g2_logs/lidareval_$(date +%m%d_%H%M%S)
mkdir -p "$LOGD"
export ROS_DOMAIN_ID=42 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
say() { echo "[lidareval $(date +%H:%M:%S)] $*"; }
die() { echo "[lidareval] FAIL: $*" >&2; cleanup; exit 1; }

cleanup() {
  pkill -x lidar_peer_tra 2>/dev/null      # comm is truncated at 15 chars
  pkill -x bridge_node 2>/dev/null
  pkill -x parameter_brid 2>/dev/null
  pkill -9 -x gz 2>/dev/null
  pkill -9 -x ruby 2>/dev/null
  pkill -x admm_agent_node 2>/dev/null
  pkill -x Xvfb 2>/dev/null
}
trap cleanup EXIT

cleanup; sleep 2
# Let the machine go quiet before starting. A fleet brought up while the previous run is
# still winding down does not stand: three attempts failed that way on 2026-08-05 with
# load average still near 10, and the same gate passed at 3.
for i in $(seq 1 60); do
  L=$(awk '{print int($1)}' /proc/loadavg)
  [ "$L" -le 4 ] && break
  [ "$i" = 1 ] && say "waiting for load average $L to fall below 5"
  sleep 5
done
rm -f /tmp/.X99-lock
Xvfb :99 -screen 0 1280x720x24 >/dev/null 2>&1 &
sleep 2
export DISPLAY=:99
source /opt/ros/jazzy/setup.bash
source /root/gridmap_ws/install/setup.bash
source $WS/install/setup.bash

# ---------- the fleet, via the unmodified gate ----------
# SOAK holds the fleet walking after it reaches the goal, which is the window the tracker
# is scored in. Scoring a standing fleet would be the easy case and not the one that
# matters: the earlier narrow-scan check looked fine on collapsed dogs and lost half its
# detections the moment they trotted.
say "phase A: G4 with the lidar roster (soak ${SECONDS_EVAL}s + margin)"
SOAK=$((SECONDS_EVAL + 40)) ROSTER=$ROSTER \
  bash $WS/src/legged_fleet/legged_admm_fleet/scripts/g4_run.sh > "$LOGD/g4.log" 2>&1 &
G4PID=$!

# Nothing attaches until the fleet is up. Bridging three 32768-point clouds is roughly
# 15 MB/s of DDS on the loopback and the stand-up is the fragile moment, so it is not worth
# competing with. Do NOT read the 2026-08-05 stand-up failures as proof that the bridges
# caused them: a later run with nothing attached at all failed the same way, and a bare
# gate passed once the machine went quiet. The residue of earlier runs was the common
# factor, which is what the load wait above is for.
#
# Attaching this late leaves the seeds stale; the tracker carries them forward by the
# observer's own displacement instead.
for i in $(seq 1 150); do
  grep -q 'all STANDING' "$LOGD/g4.log" 2>/dev/null && break
  grep -qE 'FAIL' "$LOGD/g4.log" 2>/dev/null && die "gate failed during bring-up: $(grep -m1 FAIL "$LOGD/g4.log")"
  sleep 1
done
grep -q 'all STANDING' "$LOGD/g4.log" || die "fleet never stood in 240 s"
say "fleet up -- attaching perception"

# ---------- the point clouds ----------
# g4_run launches legged_gazebo's gazebo.launch.py, whose bridge pass reads its OWN config
# root, where vision60 has no gz_bridge.yaml. Ours is reached only from sim.launch.py, so
# the bridges have to be started here by hand. (If the tracker ever moves onto the control
# path, this belongs in the bring-up, not in an eval script.)
for r in $ROBOTS; do
  ros2 run ros_gz_bridge parameter_bridge \
    "/robot$r/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked" \
    > "$LOGD/bridge_$r.log" 2>&1 &
done
sleep 8
# One message each is all that needs proving. Give discovery room: an 8 s budget was not
# enough on 2026-08-05 and failed a run that was otherwise healthy. There is no longer any
# hurry -- stale seeds are the tracker's problem to solve, not the harness's to outrun.
for r in $ROBOTS; do
  timeout 20 ros2 topic echo /robot$r/lidar/points --once --field width >/dev/null 2>&1 \
    || die "no point cloud on /robot$r/lidar/points after bridging -- nothing to evaluate"
done
say "  clouds arriving on all $(echo $ROBOTS | wc -w) dogs"

# ---------- the trackers ----------
IDS=$(echo $ROBOTS | tr ' ' ',')
for r in $ROBOTS; do
  python3 $PKG/../../lib/legged_admm_fleet/lidar_peer_tracker_node.py \
    --ros-args -r __node:=lidar_peer_tracker_$r \
    -r points:=/robot$r/lidar/points \
    -p use_sim_time:=true -p robot_id:=$r -p "robot_ids:=[$IDS]" \
    -p debug_stages:=${DEBUG_STAGES:-false} \
    -p roster_file:=$ROSTER > "$LOGD/tracker_$r.log" 2>&1 &
done
for i in $(seq 1 40); do
  READY=0
  for r in $ROBOTS; do grep -q 'tracking peers' "$LOGD/tracker_$r.log" 2>/dev/null && READY=$((READY+1)); done
  [ "$READY" = "$(echo $ROBOTS | wc -w)" ] && break
  sleep 0.5
done
for r in $ROBOTS; do
  grep -q 'tracking peers' "$LOGD/tracker_$r.log" \
    || die "tracker for robot$r did not start: $(tail -3 "$LOGD/tracker_$r.log")"
done

# Only now let the scoring window open: the interesting case is a walking fleet, and a
# standing one hides the failure the narrow scan had (see lidar.xacro).
for i in $(seq 1 120); do
  grep -q 'phase 4' "$LOGD/g4.log" 2>/dev/null && break
  sleep 2
done
grep -q 'phase 4' "$LOGD/g4.log" || die "fleet stood up but never started walking"
say "phase B: fleet walking, scoring for ${SECONDS_EVAL}s"

# ---------- the score ----------
python3 $PKG/../../lib/legged_admm_fleet/lidar_tracker_eval.py \
  --ids $ROBOTS --seconds "$SECONDS_EVAL" --out "$LOGD/report.txt" 2>&1 | tee "$LOGD/eval.log"
EVAL_RC=${PIPESTATUS[0]}

# ---------- the gate's own verdict ----------
wait $G4PID 2>/dev/null
G4_LINE=$(grep -E 'G4 (PASS|FAIL)' "$LOGD/g4.log" | tail -1)
say "gate: ${G4_LINE:-<no verdict line>}"
for r in $ROBOTS; do
  say "  $(grep -m1 'peers seen per sweep' "$LOGD/tracker_$r.log" || echo "robot$r: no sweep report")"
done
say "logs in $LOGD"

echo "$G4_LINE" | grep -q 'G4 PASS' || die "the fleet itself did not pass; perception numbers from this run are not usable"
[ "$EVAL_RC" = 0 ] || die "tracker evaluation did not pass (see $LOGD/report.txt)"
say "PASS: fleet gate green and tracker inside the decision boundary"

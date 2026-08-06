# Bring up the perception stack an observation source needs, if it needs one. SOURCE this.
#
# Callers set OBSERVATION (default truth), LOGD, ROBOTS, IDS, ROSTER, WS, and provide say()
# and die(). Sources and their properties live in config/observation_sources.yaml, so
# adding one is a yaml entry plus whatever it needs started here -- not an edit to every
# gate. g4_run.sh and _fleet_bringup.sh both source this; they deliberately keep their own
# copies of the verified phases, but there is no reason for them to keep separate copies of
# this.

# Does this source need a perception stack? Reads the same yaml the launch file does, so
# the two can never disagree about what a source is.
obs_needs_perception() {
  python3 - "$WS/install/legged_admm_fleet/share/legged_admm_fleet/config/observation_sources.yaml" \
            "${OBSERVATION:-truth}" <<'PY'
import sys, yaml
path, name = sys.argv[1], sys.argv[2]
src = (yaml.safe_load(open(path)) or {}).get(name)
if src is None:
    print(f"unknown observation source '{name}'", file=sys.stderr)
    sys.exit(2)
sys.exit(0 if src.get('needs_perception') else 1)
PY
}

# Start the gz->ROS cloud bridge and one tracker per dog, then wait for both to prove they
# are working. Refusing to continue is the point: a fleet running blind produces a run that
# looks completely clean, so every check here fails the run rather than warning.
perception_bringup() {
  obs_needs_perception
  case $? in
    0) ;;                       # yes
    1) return 0 ;;              # no
    *) die "observation source '${OBSERVATION:-truth}' is not in observation_sources.yaml" ;;
  esac

  local SPECS="" r i READY
  for r in $ROBOTS; do
    SPECS="$SPECS /robot$r/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked"
  done
  # One bridge for the fleet: three would all default to the node name "ros_gz_bridge".
  # gazebo.launch.py's own bridge pass cannot do this -- it reads legged_gazebo's config
  # root, where vision60 has no gz_bridge.yaml.
  setsid ros2 run ros_gz_bridge parameter_bridge $SPECS > "$LOGD/bridge.log" 2>&1 &
  sleep 6
  for r in $ROBOTS; do
    timeout 20 ros2 topic echo /robot$r/lidar/points --once --field width >/dev/null 2>&1 \
      || die "no point cloud on /robot$r/lidar/points -- the fleet would run blind and the run would still look clean"
  done

  # Called after the dogs are standing and before any goal: the trackers seed peer tracks
  # from the roster's spawn poses, and those are only good while the fleet is still on
  # them. (A late start is survivable -- the tracker carries the seeds forward by the
  # observer's own displacement -- but there is no reason to lean on that.)
  for r in $ROBOTS; do
    setsid python3 $WS/install/legged_admm_fleet/lib/legged_admm_fleet/lidar_peer_tracker_node.py \
      --ros-args -r __node:=lidar_peer_tracker_$r -r points:=/robot$r/lidar/points \
      -p use_sim_time:=true -p robot_id:=$r -p "robot_ids:=$IDS" -p roster_file:=$ROSTER \
      > "$LOGD/tracker_$r.log" 2>&1 &
  done
  for i in $(seq 1 40); do
    READY=0
    for r in $ROBOTS; do
      grep -q 'tracking peers' "$LOGD/tracker_$r.log" 2>/dev/null && READY=$((READY + 1))
    done
    [ "$READY" = "$(echo $ROBOTS | wc -w)" ] && break
    sleep 0.5
  done
  for r in $ROBOTS; do
    grep -q 'tracking peers' "$LOGD/tracker_$r.log" \
      || die "tracker for robot$r did not start: $(tail -3 "$LOGD/tracker_$r.log" 2>/dev/null)"
  done

  # RViz only. Repeats the tracker's geometry to publish the returns that landed on a peer
  # and a ray to each peer it resolved -- a raw cloud is 11264 ground points burying the
  # dog-shaped part, and it would not show WHICH peer a blob was attributed to, which is
  # the whole story. Off unless something is going to display it; nothing reads these
  # topics but RViz.
  if [ "${RVIZ:-0}" = 1 ]; then
    for r in $ROBOTS; do
      setsid python3 $WS/install/legged_admm_fleet/lib/legged_admm_fleet/perception_viz.py \
        --ros-args -r __node:=perception_viz_$r -r points:=/robot$r/lidar/points \
        -p use_sim_time:=true -p robot_id:=$r -p "robot_ids:=$IDS" -p roster_file:=$ROSTER \
        > "$LOGD/percviz_$r.log" 2>&1 &
    done
    say "perception viz up (RViz: hits + attribution rays)"
  fi
  say "perception up: $(echo $ROBOTS | wc -w) trackers on their own lidars (observation=$OBSERVATION)"
}

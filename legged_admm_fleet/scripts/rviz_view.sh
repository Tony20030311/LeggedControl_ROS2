#!/bin/bash
# Watch the distributed fleet in RViz instead of the CPU-hungry gz GUI.
# Run it AFTER (or alongside) a running arena_run.sh sim, inside the legged_stack container:
#     ARENA=plum ./scripts/arena_run.sh          # in one shell (starts gz + fleet)
#     ./scripts/rviz_view.sh "1 2 3"             # in another  (goes headless + opens RViz)
#
# What it does:
#   1. Kills the gz GUI so physics runs headless (frees ~4 CPU cores -> better RTF).
#   2. Starts fleet_viz_markers.py (plum pillars + per-dog MPC horizon MarkerArrays).
#   3. Opens rviz2 with rviz/fleet_plum.rviz (Fixed Frame = world).
#
# HEADLESS-GZ (why it is done this way):
#   gazebo.launch.py hardcodes gz_args='-r -v 4' (server+GUI) and legged_gazebo is a
#   read-only submodule, so we cannot pass '-s'. `gz sim` forks three procs:
#       parent ruby launcher  +  fork "gz sim server" (physics)  +  fork "gz sim gui".
#   The parent runs Process.wait2 and, when EITHER fork dies, kills the survivor
#   (cmdsim8.rb) -- so a naive kill of the GUI takes the physics server down with it.
#   Its SIGTERM/SIGINT trap also kills both. So we SIGKILL the launcher FIRST (SIGKILL
#   cannot be trapped -> the reap/kill logic never runs), THEN kill the GUI. The server
#   fork did setpgid(0,0) and is now orphaned -> it keeps stepping headless. gz_sim.launch.py
#   uses on_exit_shutdown=false, so the dying launcher does NOT tear down the ROS2 launch
#   (robot_state_publisher / TF / bridge stay up). ponytail: pgrep+kill by PID, never
#   pkill -f (CLAUDE.md rule; pkill -f self-matches and has killed gazebo before).
set -u
ROBOTS=${1:-"1 2 3"}
ARENA=${2:-plum}
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash
source /root/gridmap_ws/install/setup.bash 2>/dev/null || true
source /root/legged_ros2_ws/install/setup.bash
HERE=$(cd "$(dirname "$0")" && pwd)
RVIZ_CFG=$HERE/../rviz/fleet.rviz

go_headless() {
  local gui srv p
  gui=$(pgrep -f -x 'gz sim gui' || true)
  srv=$(pgrep -f -x 'gz sim server' || true)
  if [ -z "$srv" ]; then
    echo "[rviz_view] no 'gz sim server' found -- is arena_run.sh running? skipping headless step."
    return
  fi
  if [ -z "$gui" ]; then
    echo "[rviz_view] GUI already gone; server up. Running headless already."
    return
  fi
  # every gz-sim proc that is neither the server nor the gui fork == launcher(s)+shell wrapper.
  for p in $(pgrep -f 'gz sim' || true); do
    [ "$p" = "$gui" ] && continue
    [ "$p" = "$srv" ] && continue
    kill -9 "$p" 2>/dev/null   # SIGKILL: untrappable, so the reap-and-kill-survivor never fires
  done
  kill -9 "$gui" 2>/dev/null   # drop the GUI window/renderer; orphaned server keeps stepping
  sleep 1
  if kill -0 "$srv" 2>/dev/null; then
    echo "[rviz_view] gz GUI killed; physics server (pid $srv) running headless."
  else
    echo "[rviz_view] WARNING: server died with the GUI -- ordering/race; check gz version."
  fi
}

# Keep the gz GUI by default (you watch it). HEADLESS=1 kills it — only for automated log-only
# testing where the GUI just wastes ~4 cores.
[ "${HEADLESS:-0}" = 1 ] && go_headless

echo "[rviz_view] starting fleet_viz_markers.py for robots: $ROBOTS arena: $ARENA"
python3 "$HERE/fleet_viz_markers.py" "$ROBOTS" "$ARENA" --ros-args -p use_sim_time:=true &
VIZ_PID=$!
trap 'kill -9 $VIZ_PID 2>/dev/null' EXIT

echo "[rviz_view] opening rviz2 -d $RVIZ_CFG"
rviz2 -d "$RVIZ_CFG" --ros-args -p use_sim_time:=true

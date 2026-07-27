#!/usr/bin/env bash
# Arm trot on the whole fleet, closed-loop. The cmd_gait subscriber (upstream GaitPublisher,
# read-only submodule) is depth-1 volatile — no latch, so a one-shot pub that races DDS
# discovery is silently lost (the "random dog stuck in STANCE" bug). Instead of a blind
# timed burst, keep a persistent publisher alive until we see the ACK: GaitPublisher
# republishes /robotN/robotN_mode_schedule for every cmd_gait it receives, so one message
# there proves receipt. Re-sending trot to an already-trotting dog is idempotent.
# Running this "too early" (before gait_publisher is up) is now fine — it just waits.
ROBOTS=${ROBOTS:-"robot1 robot2 robot3"}
ACK_SECS=${ACK_SECS:-60}
FAIL=0
for r in $ROBOTS; do
  (
    ros2 topic pub -r 2 "/${r}/cmd_gait" std_msgs/msg/String "{data: trot}" >/dev/null 2>&1 &
    pub=$!
    timeout "$ACK_SECS" ros2 topic echo "/${r}/${r}_mode_schedule" --once >/dev/null 2>&1
    ok=$?
    kill "$pub" 2>/dev/null
    if [ "$ok" -eq 0 ]; then
      echo "trot ACK: $r"
    else
      echo "trot NO ACK after ${ACK_SECS}s: $r — gait_publisher up? ROS_DOMAIN_ID=42 sourced?" >&2
      exit 1
    fi
  ) &
done
for j in $(jobs -p); do wait "$j" || FAIL=1; done
exit $FAIL

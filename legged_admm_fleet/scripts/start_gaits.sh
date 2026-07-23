#!/usr/bin/env bash
# Arm trot on the whole fleet. Publish at a rate for a few seconds (NOT `pub -1`): a one-shot pub
# exits the instant it sends, so a gait_publisher that finished DDS discovery a beat late never gets
# the message and that dog stays in STANCE (chases its goal but never steps — the "random dog stuck
# at spawn" bug). Keeping each publisher alive covers late discovery; re-sending trot is idempotent.
ROBOTS=${ROBOTS:-"robot1 robot2 robot3"}
SECS=${SECS:-6}
for r in $ROBOTS; do
  timeout "$SECS" ros2 topic pub -r 3 "/${r}/cmd_gait" std_msgs/msg/String "{data: trot}" >/dev/null 2>&1 &
done
wait
echo "trot armed (${SECS}s burst) on: $ROBOTS"

#!/usr/bin/env python3
"""Print the latched /formation/plan, and exit nonzero if it never arrives.

Exists because `ros2 topic echo /formation/plan --once --qos-durability transient_local`
cannot be relied on here. It goes through the ros2 daemon for discovery, and the run
scripts kill that daemon in their own phase-0 cleanup; when it loses the publisher it
prints

    WARNING: topic [/formation/plan] does not appear to be published yet

to STDOUT and exits IMMEDIATELY, so retrying is not a fix -- five attempts fail in
fourteen seconds. Measured 2026-08-06 against a coordinator that was logging "plan for 3
dog(s)" every 200 ms throughout: nothing was wrong with the fleet, only with the read, and
it aborted several attack runs at phase 8 after the measurement they existed for was
already on disk.

A direct rclpy subscription has no daemon in it.

  read_plan.py [--timeout S]      prints the plan as YAML-ish text; exit 1 if none arrived
"""

import argparse
import sys

import rclpy
from admm_fleet_msgs.msg import FleetPlan
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--timeout', type=float, default=20.0)
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node('read_plan')
    got = []
    # transient_local to match the coordinator, which publishes once per goal and latches.
    qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(FleetPlan, '/formation/plan', lambda m: got.append(m), qos)

    end = node.get_clock().now().nanoseconds * 1e-9 + args.timeout
    while rclpy.ok() and not got and node.get_clock().now().nanoseconds * 1e-9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)

    if not got:
        print(f'no /formation/plan within {args.timeout:.0f}s', file=sys.stderr)
        sys.exit(1)

    m = got[-1]
    # Printed in the shape the callers already parse, so this is a drop-in for the echo.
    print('robot_ids:')
    for r in m.robot_ids:
        print(f'- {r}')
    print('goals:')
    for g in m.goals:
        print(f'- {g}')


if __name__ == '__main__':
    main()

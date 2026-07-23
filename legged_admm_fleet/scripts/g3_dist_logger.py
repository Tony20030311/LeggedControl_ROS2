#!/usr/bin/env python3
# 20 Hz fleet distance logger for the G3 gate: writes t(sim), per-dog x/y,
# min pairwise centre distance, and min CBF barrier h = d^2 - D_MIN^2.
# Usage: g3_dist_logger.py "1 2 3" /path/out.csv 0.6 --ros-args -p use_sim_time:=true
import itertools
import math
import sys

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

ROBOTS = [int(x) for x in sys.argv[1].split()]
OUT = sys.argv[2]
D_MIN = float(sys.argv[3])


class DistLogger(Node):
    def __init__(self):
        super().__init__("g3_dist_logger")
        self.pose = {}
        for r in ROBOTS:
            self.create_subscription(
                Odometry, f"/robot{r}/controller/odom",
                lambda m, r=r: self.pose.__setitem__(r, (m.pose.pose.position.x, m.pose.pose.position.y)), 10)
        self.f = open(OUT, "w", buffering=1)
        self.f.write("t," + ",".join(f"x{r},y{r}" for r in ROBOTS) + ",min_pair,min_h\n")
        self.create_timer(0.05, self.tick)

    def tick(self):
        if len(self.pose) < len(ROBOTS):
            return
        t = self.get_clock().now().nanoseconds * 1e-9
        pts = [self.pose[r] for r in ROBOTS]
        dmin = min(math.dist(a, b) for a, b in itertools.combinations(pts, 2))
        self.f.write(f"{t:.3f}," + ",".join(f"{p[0]:.4f},{p[1]:.4f}" for p in pts)
                     + f",{dmin:.4f},{dmin * dmin - D_MIN * D_MIN:.4f}\n")


rclpy.init(args=sys.argv)
rclpy.spin(DistLogger())

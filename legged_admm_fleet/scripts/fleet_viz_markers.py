#!/usr/bin/env python3
# RViz helper for the distributed fleet. Publishes MarkerArrays in frame `world`:
#   /fleet_viz/arena_markers -- arena obstacles + walls (latched)   [static scene]
#   /fleet_viz/mpc_horizons  -- each dog's MPC horizon (LINE_STRIP) [live]
# Arena geometry is read from gen_arena_world.py ARENAS (single source of truth, so a
# rescaled arena updates the markers automatically). Door jamb posts (radius 0.15) are NOT
# drawn (user preference: keep them for CBF/A*/gazebo, hide only in RViz).
# Horizon base (x,y) = state value[BASE_PX], value[BASE_PY] (=6,7; admm_motion_adapter.hpp).
# Usage: fleet_viz_markers.py "1 2 3" [arena]  [--ros-args ...]   (arena default: plum)
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from geometry_msgs.msg import Point
from ocs2_msgs.msg import MpcTargetTrajectories
from visualization_msgs.msg import Marker, MarkerArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_arena_world import ARENAS  # noqa: E402  single source of truth for arena geometry

BASE_PX, BASE_PY = 6, 7   # admm_motion_adapter.hpp: 24-D state base position indices
H = 1.0                   # obstacle height (matches gen_arena_world H)
POST_R = 0.15             # door jamb posts -> hidden in RViz (kept everywhere else)
PALETTE = [(0.9, 0.1, 0.1), (0.1, 0.8, 0.1), (0.2, 0.4, 0.95),
           (0.1, 0.8, 0.8), (0.9, 0.2, 0.9), (0.9, 0.8, 0.1)]


class FleetViz(Node):
    def __init__(self, robots, arena):
        super().__init__("fleet_viz_markers")
        self.robots = robots
        self.arena = arena
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.arena_pub = self.create_publisher(MarkerArray, "/fleet_viz/arena_markers", latched)
        self.horizon_pub = self.create_publisher(MarkerArray, "/fleet_viz/mpc_horizons", 10)
        self.horizons = {}
        for r in robots:
            self.create_subscription(
                MpcTargetTrajectories, f"/robot{r}/robot{r}_mpc_target",
                lambda m, r=r: self.on_target(r, m), 10)
        self.arena_markers = self.build_arena()
        self.arena_pub.publish(self.arena_markers)
        self.create_timer(2.0, lambda: self.arena_pub.publish(self.arena_markers))

    def build_arena(self):
        a = ARENAS.get(self.arena, {"circles": [], "rects": []})
        arr = MarkerArray()
        mid = 0
        for (x, y, r) in a.get("circles", []):
            if r <= POST_R:      # skip door jamb posts in RViz only
                continue
            m = Marker()
            m.header.frame_id = "world"; m.ns = "arena"; m.id = mid; mid += 1
            m.type = Marker.CYLINDER; m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), H / 2
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 2.0 * r; m.scale.z = H
            m.color.r, m.color.g, m.color.b, m.color.a = 0.8, 0.2, 0.2, 0.9
            arr.markers.append(m)
        for (cx, cy, sx, sy) in a.get("rects", []):   # walls
            m = Marker()
            m.header.frame_id = "world"; m.ns = "arena"; m.id = mid; mid += 1
            m.type = Marker.CUBE; m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = float(cx), float(cy), H / 2
            m.pose.orientation.w = 1.0
            m.scale.x, m.scale.y, m.scale.z = float(sx), float(sy), H
            m.color.r, m.color.g, m.color.b, m.color.a = 0.35, 0.35, 0.4, 0.9
            arr.markers.append(m)
        return arr

    def on_target(self, r, msg):
        m = Marker()
        m.header.frame_id = "world"; m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "mpc_horizon"; m.id = r
        m.type = Marker.LINE_STRIP; m.action = Marker.ADD
        m.pose.orientation.w = 1.0; m.scale.x = 0.04
        cr, cg, cb = PALETTE[(r - 1) % len(PALETTE)]
        m.color.r, m.color.g, m.color.b, m.color.a = cr, cg, cb, 1.0
        for st in msg.state_trajectory:
            if len(st.value) > BASE_PY:
                m.points.append(Point(x=float(st.value[BASE_PX]),
                                      y=float(st.value[BASE_PY]), z=0.05))
        self.horizons[r] = m
        arr = MarkerArray(); arr.markers = list(self.horizons.values())
        self.horizon_pub.publish(arr)


def main():
    if len(sys.argv) < 2 or not sys.argv[1].strip():
        sys.exit('usage: fleet_viz_markers.py "1 2 3" [arena]')
    robots = [int(x) for x in sys.argv[1].split()]
    arena = sys.argv[2] if len(sys.argv) > 2 and not sys.argv[2].startswith("-") else "plum"
    rclpy.init(args=sys.argv)
    rclpy.spin(FleetViz(robots, arena))


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# RViz helper for the distributed fleet. Publishes MarkerArrays in frame `world`:
#   /fleet_viz/arena_markers -- arena obstacles + walls (latched)   [static scene]
#   /fleet_viz/mpc_horizons  -- each dog's MPC horizon (LINE_STRIP) [live]
#   /fleet_viz/truth         -- claim vs body, for the false-signal demo [live]
#
# The false-signal demo cannot be told by a camera. A lie has no body: nothing in the world moves
# when a robot starts broadcasting a position 2.8 m from where it stands, so Gazebo footage shows
# only the survivors swerving for no visible reason. The three markers here are that missing half:
#
#   ghost      a translucent body at the position the robot BROADCASTS. While it is honest this
#              sits exactly on the real one and reads as a tint; the moment it lies, the ghost
#              detaches and walks away on its own. Nobody has to be told which robot is lying.
#   evidence   a line from the real body to the ghost. Its LENGTH is literally the quantity
#              Gate 2 thresholds on, so the detector's input is on screen, not just its verdict.
#              Green under the gate, red over it.
#   status     OK / SUSPECT / EVICTED, from the same majority-of-member-views rule the
#              coordinator uses (admm::majority_excluded) — so the label cannot disagree with
#              what the fleet actually did.
#
# Deliberately NOT drawn: the keep-out circle. It only exists after an eviction and its anchor
# differs by arm (odom when the lie was caught, the claim when the peer merely quit), so a naive
# ring contradicts the very claim it illustrates. Wrong marker is worse than no marker.
# Arena geometry is read from gen_arena_world.py ARENAS (single source of truth, so a
# rescaled arena updates the markers automatically). Door jamb posts (radius 0.15) are NOT
# drawn (user preference: keep them for CBF/A*/gazebo, hide only in RViz).
# Horizon base (x,y) = state value[BASE_PX], value[BASE_PY] (=6,7; admm_motion_adapter.hpp).
# Usage: fleet_viz_markers.py "1 2 3" [arena]  [--ros-args ...]   (arena default: plum)
import os
import sys

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from admm_fleet_msgs.msg import AgentState
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from ocs2_msgs.msg import MpcTargetTrajectories
from visualization_msgs.msg import Marker, MarkerArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_arena_world import ARENAS  # noqa: E402  single source of truth for arena geometry

BASE_PX, BASE_PY = 6, 7   # admm_motion_adapter.hpp: 24-D state base position indices
H = 1.0                   # obstacle height (matches gen_arena_world H)
POST_R = 0.15             # door jamb posts -> hidden in RViz (kept everywhere else)
BODY_R = 0.433            # half the 0.867 m contact distance; the ghost is drawn body-sized
GATE = 0.5                # odom_residual_gate default (admm_agent_node) — colours the evidence
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

        # False-signal layer. claim/members come off the SAME broadcast the fleet judges each
        # other on, and body off the odom Gate 2 anchors to, so the picture cannot drift from
        # the decision. Redrawn on a timer rather than per message: three robots x two topics is
        # ~60 Hz of callbacks and RViz only needs to see the current state.
        self.truth_pub = self.create_publisher(MarkerArray, "/fleet_viz/truth", 10)
        self.claim, self.body, self.views = {}, {}, {}
        for r in robots:
            self.create_subscription(AgentState, f"/robot{r}/admm/state",
                                     lambda m, r=r: self.on_state(r, m), 10)
            self.create_subscription(Odometry, f"/robot{r}/controller/odom",
                                     lambda m, r=r: self.on_odom(r, m), 1)
        self.create_timer(0.1, self.publish_truth)

    def on_state(self, r, m):
        self.claim[r] = (float(m.xnow[0]), float(m.xnow[1]))
        self.views[r] = list(m.members)

    def on_odom(self, r, m):
        self.body[r] = (float(m.pose.pose.position.x), float(m.pose.pose.position.y))

    def evicted(self, i):
        """Majority of the OTHER robots' rosters exclude i — the coordinator's rule verbatim.

        One vote is not enough on purpose: a compromised robot broadcasting a roster of only
        itself excludes everyone else, and a single-vote label would show the honest fleet as
        evicted and the attacker as fine — the picture inverted at exactly the wrong moment.
        """
        voters = votes = 0
        for j, mem in self.views.items():
            if j == i or not mem:
                continue
            voters += 1
            votes += i not in mem
        return voters > 0 and 2 * votes > voters

    def publish_truth(self):
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()

        def base(ns, i, kind):
            m = Marker()
            m.header.frame_id = "world"; m.header.stamp = now
            m.ns = ns; m.id = i; m.type = kind; m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            return m

        for r in self.robots:
            claim, body = self.claim.get(r), self.body.get(r)
            if claim is None or body is None:
                continue
            resid = math.hypot(claim[0] - body[0], claim[1] - body[1])
            hot = resid > GATE

            g = base("ghost", r, Marker.CYLINDER)
            g.pose.position.x, g.pose.position.y, g.pose.position.z = claim[0], claim[1], 0.30
            g.scale.x = g.scale.y = 2.0 * BODY_R; g.scale.z = 0.55
            # Barely there while it coincides with the body, unmistakable once it detaches.
            g.color.r, g.color.g, g.color.b = (0.95, 0.25, 0.25) if hot else (0.35, 0.55, 0.95)
            g.color.a = 0.55 if hot else 0.22
            arr.markers.append(g)

            e = base("evidence", r, Marker.LINE_STRIP)
            e.scale.x = 0.06 if hot else 0.03
            e.color.r, e.color.g, e.color.b = (0.95, 0.15, 0.15) if hot else (0.3, 0.8, 0.3)
            e.color.a = 0.95 if hot else 0.35
            e.points = [Point(x=body[0], y=body[1], z=0.30),
                        Point(x=claim[0], y=claim[1], z=0.30)]
            arr.markers.append(e)

            t = base("status", r, Marker.TEXT_VIEW_FACING)
            t.pose.position.x, t.pose.position.y, t.pose.position.z = body[0], body[1], 1.05
            t.scale.z = 0.35
            if self.evicted(r):
                t.text, rgb = f"robot{r} EVICTED", (0.95, 0.15, 0.15)
            elif hot:
                t.text, rgb = f"robot{r} SUSPECT  r={resid:.2f} m", (0.98, 0.65, 0.1)
            else:
                t.text, rgb = f"robot{r} OK", (0.85, 0.85, 0.85)
            t.color.r, t.color.g, t.color.b = rgb
            t.color.a = 1.0
            arr.markers.append(t)
        self.truth_pub.publish(arr)

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

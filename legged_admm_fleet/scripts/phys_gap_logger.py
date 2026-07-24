#!/usr/bin/env python3
"""Physical body-gap logger (measurement tool, not part of the gate).

The collision guard / dist.csv logs CENTER-to-center distance and compares against the
isotropic R_eff=0.433 (corner envelope). This instead computes the TRUE surface-to-surface
gap between the two nearest Vision60 base boxes (0.83 x 0.25 m) using each dog's real yaw,
so a side-by-side pair (which only needs 0.25 m center gap) isn't scored as if it were
corner-to-corner (0.867 m). Writes phys_gap.csv; prints the running minimum.

Usage: phys_gap_logger.py "1 2 3" <out.csv>   (run with ROS_DOMAIN_ID=42 sourced)
"""
import math
import sys

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

L, W = 0.83, 0.25          # Vision60 base collision box (m); half-diag 0.433 == CBF R_eff
HL, HW = L / 2, W / 2


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def corners(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    out = []
    for sx, sy in ((HL, HW), (HL, -HW), (-HL, -HW), (-HL, HW)):
        out.append((x + sx * c - sy * s, y + sx * s + sy * c))
    return out


def seg_pt(p, a, b):
    ax, ay = a; bx, by = b; px, py = p
    dx, dy = bx - ax, by - ay
    l2 = dx * dx + dy * dy
    if l2 == 0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / l2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def overlap(A, B):                         # SAT on both rects' edge normals
    for poly in (A, B):
        for i in range(4):
            x1, y1 = poly[i]; x2, y2 = poly[(i + 1) % 4]
            ax, ay = -(y2 - y1), (x2 - x1)
            pa = [px * ax + py * ay for px, py in A]
            pb = [px * ax + py * ay for px, py in B]
            if max(pa) < min(pb) or max(pb) < min(pa):
                return False
    return True


def rect_gap(a, b):                        # surface-to-surface distance of two oriented boxes
    if overlap(a, b):
        return 0.0
    d = float("inf")
    for p in a:
        for i in range(4):
            d = min(d, seg_pt(p, b[i], b[(i + 1) % 4]))
    for p in b:
        for i in range(4):
            d = min(d, seg_pt(p, a[i], a[(i + 1) % 4]))
    return d


def pt_in_box(p, box):                     # winding sign test for a convex quad
    sign = None
    for i in range(4):
        x1, y1 = box[i]; x2, y2 = box[(i + 1) % 4]
        cross = (x2 - x1) * (p[1] - y1) - (y2 - y1) * (p[0] - x1)
        s = cross > 0
        if sign is None:
            sign = s
        elif s != sign:
            return False
    return True


PILE_R = 0.20                              # physical pile radius (SDF), not the 0.30 CBF radius


def box_pile_gap(box, cx, cy):             # dog base-box surface to pile surface
    if pt_in_box((cx, cy), box):
        return -PILE_R
    d = min(seg_pt((cx, cy), box[i], box[(i + 1) % 4]) for i in range(4))
    return d - PILE_R


class PhysGap(Node):
    def __init__(self, robots, out, piles=None):
        super().__init__("phys_gap_logger")
        self.robots = robots
        self.piles = piles or []
        self.pose = {}
        self.run_min = float("inf")
        self.run_min_pile = float("inf")
        self.f = open(out, "w")
        self.f.write("t,pair,gap_min,pile_gap_min," + ",".join(f"gap_{i}_{j}" for i in robots for j in robots if i < j) + "\n")
        for r in robots:
            self.create_subscription(
                Odometry, f"/robot{r}/controller/odom",
                lambda m, r=r: self.pose.__setitem__(
                    r, (m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation))), 10)
        self.create_timer(0.05, self.tick)

    def tick(self):
        if len(self.pose) < len(self.robots):
            return
        boxes = {r: corners(*self.pose[r]) for r in self.robots}
        gaps, worst, worst_pair = [], float("inf"), ""
        for i in self.robots:
            for j in self.robots:
                if i < j:
                    g = rect_gap(boxes[i], boxes[j])
                    gaps.append(g)
                    if g < worst:
                        worst, worst_pair = g, f"{i}-{j}"
        pile_min = float("inf")
        for r in self.robots:
            for (cx, cy) in self.piles:
                pile_min = min(pile_min, box_pile_gap(boxes[r], cx, cy))
        t = self.get_clock().now().nanoseconds * 1e-9
        pm = pile_min if pile_min != float("inf") else -1
        self.f.write(f"{t:.3f},{worst_pair},{worst:.4f},{pm:.4f}," + ",".join(f"{g:.4f}" for g in gaps) + "\n")
        self.f.flush()
        if worst < self.run_min:
            self.run_min = worst
            self.get_logger().info(f"new min dog-dog gap = {worst:.4f} m  (pair {worst_pair})")
        if self.piles and pile_min < self.run_min_pile:
            self.run_min_pile = pile_min
            self.get_logger().info(f"new min dog-pile gap = {pile_min:.4f} m")


def main():
    robots = [int(x) for x in sys.argv[1].split()]
    out = sys.argv[2]
    piles = []
    if len(sys.argv) > 3 and sys.argv[3]:      # "x,y;x,y;..." pile centers
        for tok in sys.argv[3].split(";"):
            if tok.strip():
                x, y = tok.split(",")
                piles.append((float(x), float(y)))
    rclpy.init()
    node = PhysGap(robots, out, piles)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f"FINAL min dog-dog body gap = {node.run_min:.4f} m; "
                               f"min dog-pile gap = {node.run_min_pile:.4f} m")
        node.f.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# Leg-to-leg clearance between robots, measured rather than assumed.
#
# D_MIN = 1.3 comes from an FK sketch: rear knee-to-knee contact at 1.22 m centre distance plus
# 0.08 margin. Nothing has ever checked it against a running sim, and on 2026-07-28 the fleet was
# repeatedly measured at 0.85 m centre distance -- 0.37 m INSIDE that figure -- with no WBC drop
# and no disturbance in the trajectories. Either the legs never actually met, or they met harmlessly.
# This decides it: thigh and calf are real links with real dimensions, so treat each as a capsule
# and report the surface-to-surface distance between every leg of one robot and every leg of another.
#
# Body-box clearance (phys_gap_logger.py) already says the CHASSIS is clear by ~0.24 m at those
# moments. Legs stick out much further than the chassis, so they are the binding case.
#
# Usage: leg_gap_logger.py "1 2 3" /out/dir --ros-args -p use_sim_time:=true
import math
import sys

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener

ROBOTS = [int(x) for x in sys.argv[1].split()]
OUTDIR = sys.argv[2]
LEGS = ("LF", "LH", "RF", "RH")
# Capsule radii from vision60_description/urdf/vision60/const.xacro. Thigh is a box 0.055 wide;
# half of that is the honest inscribed radius, and using it (rather than the diagonal) keeps the
# reported gap CONSERVATIVE only where it matters -- a thigh-thigh pass is a broadside, not a corner.
R_THIGH, R_CALF, R_FOOT = 0.055 / 2, 0.02, 0.040


def quat_yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def seg_seg(p1, q1, p2, q2):
    """Shortest distance between two 3-D segments (Ericson, Real-Time Collision Detection)."""
    d1 = [q1[i] - p1[i] for i in range(3)]
    d2 = [q2[i] - p2[i] for i in range(3)]
    r = [p1[i] - p2[i] for i in range(3)]
    a = sum(x * x for x in d1)
    e = sum(x * x for x in d2)
    f = sum(d2[i] * r[i] for i in range(3))
    if a <= 1e-12 and e <= 1e-12:
        return math.dist(p1, p2)
    if a <= 1e-12:
        s, t = 0.0, min(1.0, max(0.0, f / e))
    else:
        c = sum(d1[i] * r[i] for i in range(3))
        if e <= 1e-12:
            t, s = 0.0, min(1.0, max(0.0, -c / a))
        else:
            b = sum(d1[i] * d2[i] for i in range(3))
            den = a * e - b * b
            s = min(1.0, max(0.0, (b * f - c * e) / den)) if den > 1e-12 else 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t, s = 0.0, min(1.0, max(0.0, -c / a))
            elif t > 1.0:
                t, s = 1.0, min(1.0, max(0.0, (b - c) / a))
    c1 = [p1[i] + d1[i] * s for i in range(3)]
    c2 = [p2[i] + d2[i] * t for i in range(3)]
    return math.dist(c1, c2)


class LegGap(Node):
    def __init__(self):
        super().__init__("leg_gap_logger")
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.base = {}   # r -> (x, y, yaw) of base in world
        for r in ROBOTS:
            self.create_subscription(Odometry, f"/robot{r}/controller/odom",
                                     lambda m, r=r: self.on_odom(m, r), 10)
        self.f = open(f"{OUTDIR}/leg_gap.csv", "w", buffering=1)
        self.f.write("t,min_leg_gap,pair,min_centre\n")
        self.worst = (9.9, None)
        self.create_timer(0.05, self.tick)

    def on_odom(self, m, r):
        p = m.pose.pose
        self.base[r] = (p.position.x, p.position.y, quat_yaw(p.orientation))

    def links(self, r):
        """World-frame capsules (a, b, radius) for every leg segment of robot r, or None."""
        if r not in self.base:
            return None
        bx, by, yaw = self.base[r]
        c, s = math.cos(yaw), math.sin(yaw)
        out = []
        for leg in LEGS:
            pts = {}
            for frame, _ in (("thigh", 0), ("calf", 0), ("FOOT", 0)):
                try:
                    tr = self.buf.lookup_transform(f"robot{r}/base", f"robot{r}/{leg}_{frame}",
                                                   rclpy.time.Time()).transform.translation
                except Exception:
                    return None
                # base frame -> world (planar yaw; base roll/pitch is small on flat ground and
                # ignoring it can only make the reported gap slightly optimistic, never a false alarm)
                pts[frame] = (bx + c * tr.x - s * tr.y, by + s * tr.x + c * tr.y, tr.z)
            out.append((pts["thigh"], pts["calf"], R_THIGH))
            out.append((pts["calf"], pts["FOOT"], max(R_CALF, R_FOOT)))
        return out

    def tick(self):
        caps = {r: self.links(r) for r in ROBOTS}
        live = [r for r in ROBOTS if caps[r]]
        if len(live) < 2:
            return
        t = self.get_clock().now().nanoseconds * 1e-9
        best, pair = 9.9, ""
        centre = 9.9
        for i, a in enumerate(live):
            for b in live[i + 1:]:
                centre = min(centre, math.dist(self.base[a][:2], self.base[b][:2]))
                for ca in caps[a]:
                    for cb in caps[b]:
                        g = seg_seg(ca[0], ca[1], cb[0], cb[1]) - ca[2] - cb[2]
                        if g < best:
                            best, pair = g, f"{a}-{b}"
        self.f.write(f"{t:.3f},{best:.4f},{pair},{centre:.4f}\n")
        if best < self.worst[0]:
            self.worst = (best, pair)
            self.get_logger().warn(f"leg gap {best:.3f} m ({pair}) centre {centre:.3f}")


def main():
    rclpy.init()
    n = LegGap()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.get_logger().warn(f"WORST leg gap {n.worst[0]:.3f} m ({n.worst[1]})")


if __name__ == "__main__":
    main()

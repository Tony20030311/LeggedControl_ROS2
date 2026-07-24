#!/usr/bin/env python3
# G5 logger — two CSVs for offline plotting of the distributed-fleet arena runs:
#   stats.csv : per CycleStats msg  -> comm latency (t_wait_*) + convergence (r_prim_hist)
#   traj.csv  : 20 Hz, per dog      -> actual (odom) vs reference (mpc_target) tracking error
#
# Trajectory tracking error is TIME-ALIGNED: the reference position/velocity for the current sim
# time t is linearly interpolated from the latest mpc_target the dog published (its knots carry
# absolute times), then compared to the actual odom pos/vel. That is the real tracking error, not
# the trivially-zero "odom vs knot-0" (a reference always starts at the current state).
#
# Usage: g5_logger.py "1 2 3" /out/dir --ros-args -p use_sim_time:=true
import bisect
import sys

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ocs2_msgs.msg import MpcTargetTrajectories
from admm_fleet_msgs.msg import CycleStats

ROBOTS = [int(x) for x in sys.argv[1].split()]
OUTDIR = sys.argv[2]
BASE_PX, BASE_PY, MOM_LIN_X, MOM_LIN_Y = 6, 7, 0, 1  # indices into the 24-D MPC state


def _interp(ts, xs, t):
    if not ts:
        return None
    if t <= ts[0]:
        return xs[0]
    if t >= ts[-1]:
        return xs[-1]
    i = bisect.bisect_right(ts, t)
    t0, t1, x0, x1 = ts[i - 1], ts[i], xs[i - 1], xs[i]
    a = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
    return x0 + a * (x1 - x0)


class G5Logger(Node):
    def __init__(self):
        super().__init__("g5_logger")
        self.odom = {}          # r -> (x, y, vx, vy)
        self.ref = {}           # r -> (times[], px[], py[], vx[], vy[])  latest target
        self.sf = open(f"{OUTDIR}/stats.csv", "w", buffering=1)
        self.sf.write("t,robot,cycle,achieved_rounds,n_timeouts,"
                      "t_wait_state,t_wait_xi,t_wait_z,t_cycle_wall,"
                      "t_node,t_edge_solve,bytes_tx,bytes_rx,hold,"
                      "reset,n_stale,t_rx_mean,r_prim_hist\n")
        self.tf = open(f"{OUTDIR}/traj.csv", "w", buffering=1)
        cols = ",".join(f"pos_err{r},vel_err{r},ax{r},ay{r},cx{r},cy{r}" for r in ROBOTS)
        self.tf.write("t," + cols + "\n")
        for r in ROBOTS:
            self.create_subscription(
                Odometry, f"/robot{r}/controller/odom",
                lambda m, r=r: self.odom.__setitem__(
                    r, (m.pose.pose.position.x, m.pose.pose.position.y,
                        m.twist.twist.linear.x, m.twist.twist.linear.y)), 10)
            self.create_subscription(
                MpcTargetTrajectories, f"/robot{r}/robot{r}_mpc_target",
                lambda m, r=r: self.on_target(m, r), 10)
            self.create_subscription(CycleStats, f"/robot{r}/admm/stats", self.on_stats, 10)
        self.create_timer(0.05, self.tick)

    def on_target(self, m, r):
        if not m.state_trajectory:
            return
        ts = list(m.time_trajectory)
        px = [s.value[BASE_PX] for s in m.state_trajectory]
        py = [s.value[BASE_PY] for s in m.state_trajectory]
        vx = [s.value[MOM_LIN_X] for s in m.state_trajectory]
        vy = [s.value[MOM_LIN_Y] for s in m.state_trajectory]
        self.ref[r] = (ts, px, py, vx, vy)

    def on_stats(self, m):
        t = self.get_clock().now().nanoseconds * 1e-9
        hist = "|".join(f"{x:.6g}" for x in m.r_prim_hist)
        self.sf.write(f"{t:.3f},{m.robot_id},{m.cycle_id},{m.achieved_rounds},{m.n_timeouts},"
                      f"{m.t_wait_state:.6g},{m.t_wait_xi:.6g},{m.t_wait_z:.6g},"
                      f"{m.t_cycle_wall:.6g},{m.t_node:.6g},{m.t_edge_solve:.6g},"
                      f"{m.bytes_tx},{m.bytes_rx},{int(m.hold)},"
                      f"{int(m.reset)},{m.n_stale},{m.t_rx_mean:.6g},{hist}\n")

    def tick(self):
        if len(self.odom) < len(ROBOTS) or len(self.ref) < len(ROBOTS):
            return
        t = self.get_clock().now().nanoseconds * 1e-9
        row = [f"{t:.3f}"]
        for r in ROBOTS:
            ax, ay, avx, avy = self.odom[r]
            ts, px, py, vx, vy = self.ref[r]
            cx, cy = _interp(ts, px, t), _interp(ts, py, t)
            cvx, cvy = _interp(ts, vx, t), _interp(ts, vy, t)
            pos_err = ((ax - cx) ** 2 + (ay - cy) ** 2) ** 0.5
            vel_err = ((avx - cvx) ** 2 + (avy - cvy) ** 2) ** 0.5
            row += [f"{pos_err:.4f}", f"{vel_err:.4f}", f"{ax:.4f}", f"{ay:.4f}",
                    f"{cx:.4f}", f"{cy:.4f}"]
        self.tf.write(",".join(row) + "\n")


def main():
    rclpy.init(args=sys.argv)
    node = G5Logger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

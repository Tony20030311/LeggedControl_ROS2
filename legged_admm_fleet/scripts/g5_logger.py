#!/usr/bin/env python3
# G5 logger — two CSVs for offline plotting of the distributed-fleet arena runs:
#   stats.csv : per CycleStats msg  -> comm latency (t_wait_*) + convergence (r_prim_hist)
#   traj.csv  : 20 Hz, per dog      -> actual (odom) vs reference (mpc_target) tracking error
#
# Trajectory tracking error is TIME-ALIGNED: the reference position/velocity for the current
# controller time is linearly interpolated from the latest mpc_target the dog published, then
# compared to the actual odom pos/vel.
#
# THE TWO CLOCKS. A target's knot times are t0 + k*TS with t0 = MpcObservation.time, and OCS2
# resets that to 0.0 on controller activation and accumulates the control period from there
# (LeggedControllerBase-impl.hpp:828,837) -- it is time SINCE ACTIVATION, not sim epoch. This
# logger's own clock is sim epoch, and the sim runs for ~90 s of bring-up before the controllers
# activate, so interpolating with get_clock().now() lands past the end of every target and
# _interp clamps to the LAST knot. pos_err then measures the reference LOOKAHEAD DISTANCE, not
# tracking error: measured pos_err / (speed * K_SEND*TS) = 0.955 +- 0.075 over 936 walking
# samples (arena_plum_0727_022609), i.e. the ratio was pinned at the horizon length.
# So: track the per-robot offset between the two clocks from MpcObservation and interpolate in
# controller time. Per robot, because each controller activates at its own moment.
#
# Usage: g5_logger.py "1 2 3" /out/dir --ros-args -p use_sim_time:=true
import bisect
import sys

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ocs2_msgs.msg import MpcObservation, MpcTargetTrajectories
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
        self.tofs = {}          # r -> sim_epoch - controller_time (see THE TWO CLOCKS above)
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
            self.create_subscription(
                MpcObservation, f"/robot{r}/robot{r}_mpc_observation",
                lambda m, r=r: self.on_obs(m, r), 10)
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

    def on_obs(self, m, r):
        # Re-measured every observation rather than latched once: if the controller is
        # re-activated mid-run (the documented activation-race recovery) its time restarts
        # at 0 and a latched offset would be silently wrong for the rest of the run.
        self.tofs[r] = self.get_clock().now().nanoseconds * 1e-9 - m.time

    def on_stats(self, m):
        t = self.get_clock().now().nanoseconds * 1e-9
        hist = "|".join(f"{x:.6g}" for x in m.r_prim_hist)
        self.sf.write(f"{t:.3f},{m.robot_id},{m.cycle_id},{m.achieved_rounds},{m.n_timeouts},"
                      f"{m.t_wait_state:.6g},{m.t_wait_xi:.6g},{m.t_wait_z:.6g},"
                      f"{m.t_cycle_wall:.6g},{m.t_node:.6g},{m.t_edge_solve:.6g},"
                      f"{m.bytes_tx},{m.bytes_rx},{int(m.hold)},"
                      f"{int(m.reset)},{m.n_stale},{m.t_rx_mean:.6g},{hist}\n")

    def tick(self):
        # tofs too: without an observation from a dog there is no way to place its target's
        # knots on this clock, and interpolating anyway is what produced the lookahead-distance
        # bug. Wait rather than write a number that looks like a tracking error and is not.
        if (len(self.odom) < len(ROBOTS) or len(self.ref) < len(ROBOTS)
                or len(self.tofs) < len(ROBOTS)):
            return
        t = self.get_clock().now().nanoseconds * 1e-9
        row = [f"{t:.3f}"]
        for r in ROBOTS:
            ax, ay, avx, avy = self.odom[r]
            ts, px, py, vx, vy = self.ref[r]
            tc = t - self.tofs[r]   # sim epoch -> this dog's controller time
            cx, cy = _interp(ts, px, tc), _interp(ts, py, tc)
            cvx, cvy = _interp(ts, vx, tc), _interp(ts, vy, tc)
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

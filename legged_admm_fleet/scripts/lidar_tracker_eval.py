#!/usr/bin/env python3
"""Score lidar_peer_tracker_node against the ground truth it is meant to replace.

Runs alongside a live fleet, subscribes to every /robot<i>/perceived/robot<j> and every
/robot<j>/hardware/odom (the Gazebo model pose), and answers the three questions that
decide whether the tracker can take over the observation channel:

  1. Is it accurate enough?   The trust layer's decision boundary is d_lie/2 = 0.15 m.
     An observation error near that is not a detail, it is a false conviction.
  2. Is the association right? Every sample is re-tested: of all the peers, is the one
     this message claims to be about actually the closest to the reported position? A
     wrong answer here means the tracker has swapped two dogs.
  3. What is the near-face bias? A lidar sees the near face, so the centroid sits short
     of the body centre along the line of sight. The mean of that component is what
     centre_push should be set to.

It also reports detection rate: sweeps where a peer was in range but produced no message.
Publishing nothing is safe (the agent holds), but a channel that is quiet half the time
is a different sensor than one that is quiet a tenth of the time.

Usage:  lidar_tracker_eval.py --ids 1 2 3 --seconds 60 [--out report.txt]
"""

import argparse
import math
import sys
from collections import defaultdict

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

# The trust layer convicts on a residual past d_lie/2. Anything the sensor itself
# contributes near this size is indistinguishable from a small lie.
DECISION_BOUNDARY_M = 0.15


class Eval(Node):
    def __init__(self, ids):
        super().__init__('lidar_tracker_eval')
        self.ids = ids
        self.truth = {}                       # id -> (t, x, y), latest
        self.samples = defaultdict(list)      # (observer, peer) -> list of dicts
        self.n_perceived = defaultdict(int)
        for j in ids:
            self.create_subscription(
                Odometry, f'/robot{j}/hardware/odom',
                lambda m, j=j: self._truth(j, m), qos_profile_sensor_data)
        for i in ids:
            for j in ids:
                if i == j:
                    continue
                self.create_subscription(
                    Odometry, f'/robot{i}/perceived/robot{j}',
                    lambda m, i=i, j=j: self._perceived(i, j, m), 10)

    def _truth(self, j, m):
        self.truth[j] = (self.get_clock().now().nanoseconds * 1e-9,
                         m.pose.pose.position.x, m.pose.pose.position.y)

    def _perceived(self, i, j, m):
        self.n_perceived[(i, j)] += 1
        # Every peer's truth must be in hand, not just this pair's: question 2 needs to
        # ask whether some OTHER dog was closer to the reported position.
        if not all(k in self.truth for k in self.ids):
            return
        p = np.array([m.pose.pose.position.x, m.pose.pose.position.y])
        tj = np.array(self.truth[j][1:])
        ti = np.array(self.truth[i][1:])
        err = p - tj

        los = tj - ti
        n = np.linalg.norm(los)
        radial = float(err @ (los / n)) if n > 1e-9 else float('nan')

        nearest = min((k for k in self.ids if k != i),
                      key=lambda k: np.linalg.norm(p - np.array(self.truth[k][1:])))
        self.samples[(i, j)].append({
            'err': float(np.linalg.norm(err)),
            'radial': radial,
            'lateral': float(math.sqrt(max(np.linalg.norm(err) ** 2 - radial ** 2, 0.0)))
            if math.isfinite(radial) else float('nan'),
            'assoc_ok': nearest == j,
            'range': float(n),
        })


def report(ev, seconds, sweep_hz):
    lines = []
    add = lines.append
    add('=' * 78)
    add('lidar_peer_tracker vs /robot<j>/hardware/odom (Gazebo model pose)')
    add(f'window {seconds:.0f} s   decision boundary d_lie/2 = {DECISION_BOUNDARY_M} m')
    add('=' * 78)
    if not ev.samples:
        add('NO PERCEIVED SAMPLES AT ALL. The tracker published nothing; this is a')
        add('harness or bring-up failure, not a measurement. Check that the gz->ROS')
        add('bridge is running and that lidar_peer_tracker_node started.')
        return '\n'.join(lines), False

    add(f'{"pair":>10} {"n":>6} {"det%":>6} {"assoc%":>7} {"mean":>7} {"p95":>7} '
        f'{"max":>7} {"radial":>8} {"lateral":>8}')
    all_err, all_radial, worst_assoc, ok = [], [], 1.0, True
    expected = max(int(seconds * sweep_hz), 1)
    for key in sorted(ev.samples):
        s = ev.samples[key]
        e = np.array([x['err'] for x in s])
        r = np.array([x['radial'] for x in s])
        lat = np.array([x['lateral'] for x in s])
        a = float(np.mean([x['assoc_ok'] for x in s]))
        det = 100.0 * ev.n_perceived[key] / expected
        all_err.append(e)
        all_radial.append(r[np.isfinite(r)])
        worst_assoc = min(worst_assoc, a)
        add(f'{key[0]}->{key[1]:>7} {len(s):>6} {det:>6.1f} {100 * a:>7.1f} '
            f'{e.mean():>7.3f} {np.percentile(e, 95):>7.3f} {e.max():>7.3f} '
            f'{np.mean(r[np.isfinite(r)]):>8.3f} {np.mean(lat[np.isfinite(lat)]):>8.3f}')

    e = np.concatenate(all_err)
    r = np.concatenate(all_radial)
    add('-' * 78)
    add(f'all pairs: n={len(e)}  mean={e.mean():.3f}  p95={np.percentile(e, 95):.3f}  '
        f'max={e.max():.3f}  sigma={e.std():.3f}')
    add(f'over the {DECISION_BOUNDARY_M} m decision boundary: '
        f'{100.0 * np.mean(e > DECISION_BOUNDARY_M):.1f}% of samples')
    add('')
    add(f'near-face bias (radial mean) = {r.mean():+.3f} m   '
        f'-> set centre_push to {-r.mean():.3f}')
    add(f'association correct on {100 * worst_assoc:.1f}% of samples (worst pair)')
    add('')

    # Verdict. Deliberately strict on association: a swap is not a large error, it is a
    # wrong answer about which robot is where, and the trust layer would act on it.
    if worst_assoc < 1.0:
        add('VERDICT: FAIL -- association is not perfect. A swapped peer feeds the trust')
        add('layer a residual the size of the formation, not the size of a lie.')
        ok = False
    elif np.percentile(e, 95) > DECISION_BOUNDARY_M:
        add(f'VERDICT: NOT YET -- p95 error {np.percentile(e, 95):.3f} m exceeds the '
            f'{DECISION_BOUNDARY_M} m boundary.')
        add('Set centre_push from the radial bias above and re-run before switching the')
        add('observation channel over; the residual after that is the real sigma.')
        ok = False
    else:
        add(f'VERDICT: PASS -- p95 {np.percentile(e, 95):.3f} m is inside the '
            f'{DECISION_BOUNDARY_M} m boundary and no sample was mis-associated.')
    return '\n'.join(lines), ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ids', type=int, nargs='+', required=True)
    ap.add_argument('--seconds', type=float, default=60.0)
    ap.add_argument('--sweep-hz', type=float, default=5.0,
                    help='lidar update_rate, for the detection-rate denominator')
    ap.add_argument('--out', default=None)
    args = ap.parse_args()

    rclpy.init()
    ev = Eval(args.ids)
    end = ev.get_clock().now().nanoseconds * 1e-9 + args.seconds
    while rclpy.ok() and ev.get_clock().now().nanoseconds * 1e-9 < end:
        rclpy.spin_once(ev, timeout_sec=0.2)

    text, ok = report(ev, args.seconds, args.sweep_hz)
    print(text)
    if args.out:
        with open(args.out, 'w', encoding='utf-8') as f:
            f.write(text + '\n')
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()

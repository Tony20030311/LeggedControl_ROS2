#!/usr/bin/env python3
"""Publish where this dog SEES its peers, from its own lidar.

One node per dog. Subscribes to that dog's point cloud and its own odometry, and
publishes a nav_msgs/Odometry per peer on

    /robot<self>/perceived/robot<peer>

which is the shape admm_agent_node already consumes on `observed/robotJ` (it reads
pose.pose.position.x and .y and nothing else). Today that local name is remapped to
/robotJ/hardware/odom, the Gazebo model pose -- ground truth wearing a sensor's clothes.
This node exists to replace it. It does NOT do so yet: it publishes alongside, so the two
can be compared before anything depends on the answer (scripts/lidar_tracker_eval.py).

The arithmetic lives in python/admm/peer_tracker.py and is unit-tested there. This file
is message plumbing.

Two things it deliberately does not use:

  a peer's broadcast position  -- association is by continuity from the roster's spawn
      poses. A peer that lies about where it is cannot change which returns get
      attributed to it, which is the only reason this channel is worth anything.

  any peer's ground truth      -- the observer's OWN odometry is used to place the sensor
      in the world, which is the same source admm_agent_node already uses for self_p, and
      on hardware is the robot's own estimator. Nothing here needs a peer's pose.

Publishing NOTHING is a meaningful output: when a peer is not seen this sweep, no message
goes out, admm_agent_node's interp_at returns nullopt, and it holds its last correction.
That is the safe behaviour and it is already implemented over there.
"""

import math
import os
import sys

import numpy as np
import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'python', 'admm'))
sys.path.insert(0, '/root/legged_ros2_ws/install/legged_admm_fleet/lib/'
                   'legged_admm_fleet/python/admm')
from peer_tracker import (  # noqa: E402
    BODY_L, BODY_W, associate, blob_centres, cluster_xy, keep_bodies, to_world)

# Where the lidar sits on the base, from urdf/vision60/lidar.xacro: the joint puts it at
# (0.25, 0, 0.2) and the optical frame another 0.036 up. Kept here rather than read from
# TF because it is fixed geometry and a missing transform would fail silently at 5 Hz.
MOUNT_XYZ = (0.25, 0.0, 0.236)


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _cloud_xyz(msg):
    """PointCloud2 -> (N, 3) float array, using the message's own field offsets.

    Built from the declared fields rather than assuming a layout: gz's PointCloudPacked
    carries extra channels, and hardcoding a stride is how this silently reads garbage.
    """
    names = {f.name: f for f in msg.fields}
    if not {'x', 'y', 'z'} <= set(names):
        return np.zeros((0, 3))
    # 7 == FLOAT32 in sensor_msgs/PointField. Anything else here would need its own
    # decode; refuse rather than reinterpret the bytes as something they are not.
    if any(names[c].datatype != 7 for c in 'xyz'):
        raise ValueError('lidar cloud is not float32 xyz; refusing to guess its layout')
    n = msg.width * msg.height
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8, count=n * msg.point_step)
    raw = raw.reshape(n, msg.point_step)
    out = np.empty((n, 3), dtype=np.float32)
    for k, c in enumerate('xyz'):
        o = names[c].offset
        out[:, k] = raw[:, o:o + 4].copy().view(np.float32).ravel()
    return out.astype(float)


class LidarPeerTracker(Node):
    def __init__(self):
        super().__init__('lidar_peer_tracker')
        self.self_id = self.declare_parameter('robot_id', 1).value
        ids = list(self.declare_parameter('robot_ids', [1, 2, 3]).value)
        self.peers = [int(j) for j in ids if int(j) != int(self.self_id)]

        # Geometry / filter knobs. Defaults match the arena and the lidar in
        # urdf/vision60/lidar.xacro; see python/admm/peer_tracker.py for what each gates.
        # The height window is measured from OUR OWN base, not from the world floor: every
        # dog in the fleet is the same robot standing on the same ground, so our own torso
        # height is the best available estimate of a peer's, and it tracks the fleet's
        # stance without a parameter to keep in sync.
        #
        # It has to be this tight because of what else is up there. Every dog carries the
        # lidar on a mast at (0.25, 0, 0.2) on its base, and the mast is a far better
        # reflector than the body: measured 2026-08-05, 28 of 35 returns off a peer landed
        # in the mast band and none on the torso, putting the blob centroid 0.25 m along
        # the PEER's heading. That is not a near-face bias and no single scalar can undo
        # it -- the eval showed radial +0.19 for a peer ahead, -0.25 for one behind, and
        # 0.25 purely lateral for one abeam, which is the signature of a fixed offset in
        # the target's own frame. The fix is to not look at the mast: the torso box is
        # +-0.095 m about the base (base_height 0.19 in const.xacro) and the mast starts at
        # +0.14, so a +-0.11 window separates them with 0.03 m to spare for body bob.
        self.band = self.declare_parameter('body_band_m', 0.11).value
        # Absolute floor, independent of the band: if our own odometry is wrong the band
        # could otherwise swing down into the ground, which outnumbers bodies ~200:1.
        self.floor_z = self.declare_parameter('floor_z', 0.15).value
        self.r_min = self.declare_parameter('r_min', 0.60).value        # past own legs
        self.r_max = self.declare_parameter('r_max', 4.00).value        # = obs_range
        self.gap = self.declare_parameter('cluster_gap', 0.30).value
        self.min_points = self.declare_parameter('min_points', 5).value
        # A peer cannot cross this much between sweeps. At 5 Hz and the fleet's 0.4 m/s
        # reference speed a dog moves 0.08 m; the rest is tracking slack. Too wide and a
        # neighbouring dog becomes a candidate, too tight and a turn drops the track.
        self.gate = self.declare_parameter('gate_m', 0.50).value
        # Fit the known body box to each blob instead of averaging its returns. A lidar
        # only sees the faces turned toward it, so the centroid lands on the surface: short
        # of the centre by 0.125 m across the width, 0.415 m along the length, and a mix at
        # a corner. Measured on this fleet before the fit, radial bias per pair was -0.138
        # side-on, -0.26 at a rear quarter, -0.34 at the scan edge -- a bias that changes
        # with aspect, which no single offset can cancel.
        #
        # Set body_fit false to fall back to centroid + centre_push, which is the older,
        # weaker correction; it exists so the two can be compared, not because it works.
        self.body_fit = self.declare_parameter('body_fit', True).value
        self.push = self.declare_parameter('centre_push', 0.0).value
        # Per-sweep dump of every stage. Off by default: it is throttled but still
        # one line per two seconds per dog.
        self.debug = self.declare_parameter('debug_stages', False).value

        # How many consecutive unmatched sweeps before a track is called lost. It is not
        # re-seeded when that happens: the only prior available after a loss would be the
        # peer's own broadcast position, and taking it would hand an attacker exactly the
        # influence this node exists to deny. Loss is reported and the channel goes quiet,
        # which the agent already handles by holding its last correction.
        self.lost_after = self.declare_parameter('lost_after_sweeps', 15).value

        self.tracks = self._spawn_tracks(self.declare_parameter('roster_file', '').value)
        self.pose = None            # (x, y, yaw) of our own base, from our own odometry
        self.n_clouds = 0
        self.n_seen = {j: 0 for j in self.peers}
        self.misses = {j: 0 for j in self.peers}
        self.lost = set()

        self.create_subscription(Odometry, f'/robot{self.self_id}/controller/odom',
                                 self._on_odom, 1)
        self.create_subscription(PointCloud2, 'points', self._on_cloud,
                                 qos_profile_sensor_data)
        self.pubs = {j: self.create_publisher(
            Odometry, f'/robot{self.self_id}/perceived/robot{j}', 1) for j in self.peers}
        self.create_timer(5.0, self._report)
        self.get_logger().info(
            f'tracking peers {self.peers} from own lidar; tracks initialised at '
            + ', '.join(f'robot{j}=({p[0]:.2f},{p[1]:.2f})' for j, p in self.tracks.items()))

    def _spawn_tracks(self, roster_file):
        """Seed each peer's track from the roster's spawn pose.

        The roster is fleet configuration -- the same file that told Gazebo where to put
        the models -- not a claim made by a peer over the wire, so seeding from it does
        not hand an attacker any influence. Refuse to start without it rather than fall
        back to a peer's broadcast position, which is exactly the input this whole node
        exists to avoid depending on.
        """
        if not roster_file or not os.path.exists(roster_file):
            raise SystemExit(f'roster_file is required and must exist (got "{roster_file}"): '
                             'peer tracks are seeded from spawn poses, and the only other '
                             'seed available is a peer\'s own claim, which is untrustworthy')
        with open(roster_file, 'r', encoding='utf-8') as f:
            roster = yaml.safe_load(f) or {}
        by_name = {str(c.get('robot_name', k)): c for k, c in roster.items()
                   if isinstance(c, dict)}
        tracks = {}
        for j in self.peers:
            cfg = by_name.get(f'robot{j}')
            if cfg is None or 'spawn_poses' not in cfg:
                raise SystemExit(f'roster {roster_file} has no spawn_poses for robot{j}')
            p = cfg['spawn_poses']
            tracks[j] = (float(p[0]), float(p[1]))
        # Our own spawn pose, kept only so _check_seed_is_fresh can tell whether the fleet
        # has already moved. Nothing in the pipeline reads it.
        mine = by_name.get(f'robot{self.self_id}')
        self.own_spawn = (float(mine['spawn_poses'][0]), float(mine['spawn_poses'][1])) \
            if mine and 'spawn_poses' in mine else None
        return tracks

    def _on_odom(self, m):
        first = self.pose is None
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y,
                     _yaw(m.pose.pose.orientation), m.pose.pose.position.z)
        if first:
            self._check_seed_is_fresh()

    def _check_seed_is_fresh(self):
        """Carry the peer seeds forward if the fleet has already left its spawn poses.

        The seeds are only good while the fleet is where the roster put it. Starting later
        used to be silently fatal: every sweep failed to match and the node reported zeros
        forever, which reads exactly like a broken sensor. Two evaluation runs died that
        way on 2026-08-05 before the cause was found, and the timing is not something a
        caller can reliably win -- attaching early enough to keep the seeds fresh means
        running three bridges and three trackers through the stand-up, and that load by
        itself stopped the fleet standing at all (robot1 stuck at z=0.125).

        So instead of demanding the right moment, recover from the wrong one. We know
        where WE spawned and where we are now; the fleet holds formation, so our own
        displacement is the best available estimate of everyone else's, and applying it to
        the seeds puts them within a gate's reach. Both inputs are ours: our odometry and
        the roster. No peer's broadcast position is consulted, which is the property that
        has to survive -- an attacker still cannot influence which returns are attributed
        to it.

        This is a prior for ACQUISITION only; from the first match onward the track
        follows the returns. If the formation had rotated or slots had been reassigned the
        shift could seed toward the wrong peer, so the evaluation reports association
        correctness per pair and that number, not this reasoning, is the evidence.
        """
        if self.own_spawn is None:
            return
        d = (self.pose[0] - self.own_spawn[0], self.pose[1] - self.own_spawn[1])
        moved = math.hypot(*d)
        if moved <= self.gate:
            return
        self.tracks = {j: (p[0] + d[0], p[1] + d[1]) for j, p in self.tracks.items()}
        self.get_logger().warn(
            f'started {moved:.2f} m from robot{self.self_id}\'s own spawn pose, so the peer '
            f'seeds were stale by about that much. Carried them forward by our own '
            f'displacement ({d[0]:+.2f},{d[1]:+.2f}): '
            + ', '.join(f'robot{j}=({p[0]:.2f},{p[1]:.2f})' for j, p in self.tracks.items())
            + '. Acquisition only -- assumes the fleet translated together.')

    def _on_cloud(self, msg):
        if self.pose is None:
            return                      # not localised yet; publishing now would be a guess
        x, y, yaw, z = self.pose
        self.n_clouds += 1

        # Sensor pose = base pose plus the mount offset turned by our heading. Only yaw is
        # applied: roll and pitch of a trotting body move the mount by a couple of
        # centimetres, well under the 0.15 m decision boundary, and reading them would
        # need the full rotation here and in to_world.
        c, s = math.cos(yaw), math.sin(yaw)
        sx = x + c * MOUNT_XYZ[0] - s * MOUNT_XYZ[1]
        sy = y + s * MOUNT_XYZ[0] + c * MOUNT_XYZ[1]
        sensor_xyz = (sx, sy, z + MOUNT_XYZ[2])

        world = to_world(_cloud_xyz(msg), sensor_xyz, yaw)
        # Torso band around our own base height, clamped so it can never dip into the floor.
        z_lo = max(z - self.band, self.floor_z)
        z_hi = z + self.band
        mask = keep_bodies(world, z_lo, z_hi, (sx, sy), self.r_min, self.r_max)
        pts = world[mask][:, :2]
        try:
            groups = cluster_xy(pts, self.gap)
        except ValueError as e:
            # The floor filter let a flood through. Say so and drop the sweep; silently
            # returning a truncated clustering would look like a working sensor.
            self.get_logger().error(str(e), throttle_duration_sec=5.0)
            return
        centres, sizes = blob_centres(pts, groups, self.min_points, (sx, sy), self.push,
                                      (BODY_L, BODY_W) if self.body_fit else None)

        matched = associate(self.tracks, centres, self.gate)
        if self.debug:
            self.get_logger().info(
                f'sensor=({sx:.2f},{sy:.2f},{sensor_xyz[2]:.2f}) yaw={math.degrees(yaw):.0f} '
                f'band=[{z_lo:.2f},{z_hi:.2f}] kept={int(mask.sum())} clusters={len(groups)} '
                f'centres=[{", ".join(f"({c[0]:.2f},{c[1]:.2f})x{n}" for c, n in zip(centres, sizes))}] '
                f'tracks={ {j: (round(p[0], 2), round(p[1], 2)) for j, p in self.tracks.items()} } '
                f'matched={matched}', throttle_duration_sec=2.0)
        for j, k in matched.items():
            self.tracks[j] = (float(centres[k][0]), float(centres[k][1]))
            self.n_seen[j] += 1
            self.misses[j] = 0
            if j in self.lost:
                self.lost.discard(j)
                self.get_logger().info(f'robot{j} reacquired at '
                                       f'({centres[k][0]:.2f},{centres[k][1]:.2f})')
            out = Odometry()
            out.header.stamp = msg.header.stamp
            out.header.frame_id = 'world'
            out.pose.pose.position.x = float(centres[k][0])
            out.pose.pose.position.y = float(centres[k][1])
            out.pose.pose.orientation.w = 1.0
            self.pubs[j].publish(out)

        for j in self.peers:
            if j in matched:
                continue
            self.misses[j] += 1
            if self.misses[j] == self.lost_after and j not in self.lost:
                self.lost.add(j)
                # Say which of the two it is. A stale seed looks identical to a peer that
                # walked away, and confusing them cost a whole evaluation run: the seed
                # was two metres behind the fleet and nothing was ever within the gate.
                t = self.tracks[j]
                self.get_logger().warn(
                    f'robot{j} unmatched for {self.lost_after} sweeps; its track is stuck at '
                    f'({t[0]:.2f},{t[1]:.2f}) with {len(centres)} blob(s) in view. If that '
                    f'position is where robot{j} SPAWNED, this node started after the fleet '
                    f'moved and the seed was already stale. Not re-seeding: the only prior '
                    f'left would be robot{j}\'s own claim.')

    def _report(self):
        if not self.n_clouds:
            self.get_logger().warn('no lidar clouds received yet')
            return
        rates = ' '.join(f'robot{j}={self.n_seen[j]}/{self.n_clouds}'
                         + ('(LOST)' if j in self.lost else '') for j in self.peers)
        self.get_logger().info(f'peers seen per sweep: {rates}')


def main():
    rclpy.init()
    node = LidarPeerTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()

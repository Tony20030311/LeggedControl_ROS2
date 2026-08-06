#!/usr/bin/env python3
"""Show what a dog's lidar is doing, in RViz, without showing 32768 points of floor.

Publishes two things per observing robot:

  <ns>/viz/hits   PointCloud2  the returns that survived the tracker's own filters, i.e.
                               the ones that landed on a peer's body. About a hundred
                               points instead of 32768, and every one of them is evidence.
  <ns>/viz/rays   MarkerArray  one line from the sensor to each peer it resolved this
                               sweep, plus a ring at the resolved position.

Raw clouds are the obvious thing to display and the wrong one: 11264 of 32768 returns are
ground, they bury the dog-shaped part, and three of them at 5 Hz is more than software GL
manages while Gazebo has the machine. More to the point, a raw cloud does not show
ATTRIBUTION -- the story here is "robot1 decided robot2 is HERE while robot2 says it is
THERE", and that needs the association drawn, not the returns dumped.

Runs beside lidar_peer_tracker_node.py and repeats its geometry rather than reaching into
it, so nothing on the control path changes to make a picture.
"""

import math
import os
import sys

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

sys.path.insert(0, '/root/legged_ros2_ws/install/legged_admm_fleet/lib/'
                   'legged_admm_fleet/python/admm')
sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'python', 'admm'))
from peer_tracker import (  # noqa: E402
    BODY_L, BODY_W, associate, blob_centres, cluster_xy, keep_bodies, to_world)

MOUNT_XYZ = (0.25, 0.0, 0.236)   # same mount as lidar_peer_tracker_node.py


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _cloud_xyz(msg):
    names = {f.name: f for f in msg.fields}
    if not {'x', 'y', 'z'} <= set(names):
        return np.zeros((0, 3))
    n = msg.width * msg.height
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8,
                        count=n * msg.point_step).reshape(n, msg.point_step)
    out = np.empty((n, 3), dtype=np.float32)
    for k, c in enumerate('xyz'):
        o = names[c].offset
        out[:, k] = raw[:, o:o + 4].copy().view(np.float32).ravel()
    return out.astype(float)


def _cloud_msg(stamp, frame, pts):
    m = PointCloud2()
    m.header.stamp = stamp
    m.header.frame_id = frame
    m.height = 1
    m.width = len(pts)
    m.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                for i, n in enumerate('xyz')]
    m.is_bigendian = False
    m.point_step = 12
    m.row_step = 12 * len(pts)
    m.is_dense = True
    m.data = np.asarray(pts, dtype=np.float32).tobytes()
    return m


class PerceptionViz(Node):
    def __init__(self):
        super().__init__('perception_viz')
        self.self_id = self.declare_parameter('robot_id', 1).value
        ids = list(self.declare_parameter('robot_ids', [1, 2, 3]).value)
        self.peers = [int(j) for j in ids if int(j) != int(self.self_id)]
        self.band = self.declare_parameter('body_band_m', 0.11).value
        self.floor_z = self.declare_parameter('floor_z', 0.15).value
        self.r_min = self.declare_parameter('r_min', 0.60).value
        self.r_max = self.declare_parameter('r_max', 4.00).value
        self.gap = self.declare_parameter('cluster_gap', 0.30).value
        self.min_points = self.declare_parameter('min_points', 5).value
        self.gate = self.declare_parameter('gate_m', 0.50).value
        self.tracks = self._spawn_tracks(self.declare_parameter('roster_file', '').value)
        self.pose = None

        self.create_subscription(Odometry, f'/robot{self.self_id}/controller/odom',
                                 self._on_odom, 1)
        self.create_subscription(PointCloud2, 'points', self._on_cloud,
                                 qos_profile_sensor_data)
        self.pub_hits = self.create_publisher(PointCloud2,
                                              f'/robot{self.self_id}/viz/hits', 1)
        self.pub_rays = self.create_publisher(MarkerArray,
                                              f'/robot{self.self_id}/viz/rays', 1)
        self.get_logger().info(f'perception viz for robot{self.self_id}, peers {self.peers}')

    def _spawn_tracks(self, roster_file):
        with open(roster_file, 'r', encoding='utf-8') as f:
            roster = yaml.safe_load(f) or {}
        by_name = {str(c.get('robot_name', k)): c for k, c in roster.items()
                   if isinstance(c, dict)}
        return {j: (float(by_name[f'robot{j}']['spawn_poses'][0]),
                    float(by_name[f'robot{j}']['spawn_poses'][1])) for j in self.peers}

    def _on_odom(self, m):
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y,
                     _yaw(m.pose.pose.orientation), m.pose.pose.position.z)

    def _on_cloud(self, msg):
        if self.pose is None:
            return
        x, y, yaw, z = self.pose
        c, s = math.cos(yaw), math.sin(yaw)
        sx = x + c * MOUNT_XYZ[0] - s * MOUNT_XYZ[1]
        sy = y + s * MOUNT_XYZ[0] + c * MOUNT_XYZ[1]
        sz = z + MOUNT_XYZ[2]

        world = to_world(_cloud_xyz(msg), (sx, sy, sz), yaw)
        mask = keep_bodies(world, max(z - self.band, self.floor_z), z + self.band,
                           (sx, sy), self.r_min, self.r_max)
        hits = world[mask]
        self.pub_hits.publish(_cloud_msg(msg.header.stamp, 'world', hits))

        arr = MarkerArray()
        if len(hits):
            pts = hits[:, :2]
            try:
                groups = cluster_xy(pts, self.gap)
            except ValueError:
                return
            centres, _, kept = blob_centres(pts, groups, self.min_points, (sx, sy),
                                            0.0, (BODY_L, BODY_W))
            for j, k in associate(self.tracks, centres, self.gate).items():
                self.tracks[j] = (float(centres[k][0]), float(centres[k][1]))
                arr.markers.append(self._ray(msg.header.stamp, j, (sx, sy, sz),
                                             centres[k]))
                arr.markers.append(self._ring(msg.header.stamp, j, centres[k], z))
        # Always publish, even empty: RViz keeps the last array otherwise, so a peer that
        # went behind a post would keep its ray drawn -- exactly the moment the picture is
        # supposed to show the observer losing it.
        for j in self.peers:
            if not any(m.id // 10 == j for m in arr.markers):
                for suffix in (0, 1):
                    d = Marker()
                    d.header.stamp = msg.header.stamp
                    d.header.frame_id = 'world'
                    d.ns = f'obs{self.self_id}'
                    d.id = j * 10 + suffix
                    d.action = Marker.DELETE
                    arr.markers.append(d)
        self.pub_rays.publish(arr)

    def _base(self, stamp, j, suffix):
        m = Marker()
        m.header.stamp = stamp
        m.header.frame_id = 'world'
        m.ns = f'obs{self.self_id}'
        m.id = j * 10 + suffix
        m.action = Marker.ADD
        # One hue per OBSERVER, so a ray's colour says who is looking, which is the thing
        # the picture is about. Slots 1-3 of the validated categorical palette.
        r, g, b = {1: (0.165, 0.471, 0.839), 2: (0.922, 0.408, 0.204),
                   3: (0.106, 0.686, 0.478)}.get(self.self_id, (0.5, 0.5, 0.5))
        m.color = ColorRGBA(r=r, g=g, b=b, a=0.85)
        return m

    def _ray(self, stamp, j, sensor, centre):
        m = self._base(stamp, j, 0)
        m.type = Marker.LINE_STRIP
        m.scale.x = 0.012
        m.points = [Point(x=sensor[0], y=sensor[1], z=sensor[2]),
                    Point(x=float(centre[0]), y=float(centre[1]), z=sensor[2])]
        return m

    def _ring(self, stamp, j, centre, z):
        m = self._base(stamp, j, 1)
        m.type = Marker.CYLINDER
        m.pose.position.x = float(centre[0])
        m.pose.position.y = float(centre[1])
        m.pose.position.z = z
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = 0.45
        m.scale.z = 0.02
        m.color.a = 0.55
        return m


def main():
    rclpy.init()
    node = PerceptionViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()

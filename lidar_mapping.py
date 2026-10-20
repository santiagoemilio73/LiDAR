#!/usr/bin/env python3
"""
Stationary Velodyne VLP-16 dynamic 3D map for ROS 2 Jazzy.

Subscribes:
  /points  (sensor_msgs/msg/PointCloud2)

Publishes:
  /lidar_map/static       (sensor_msgs/msg/PointCloud2)
  /lidar_map/dynamic      (sensor_msgs/msg/PointCloud2)
  /lidar_map/combined     (sensor_msgs/msg/PointCloud2)

Method:
  - Voxelizes each incoming scan.
  - Learns a background from the first BACKGROUND_LEARN_SECONDS seconds.
  - Marks voxels differing from the learned background as dynamic.
  - Publishes recent dynamic points so moving objects do not leave trails.

Important:
  Keep the room mostly clear of moving people during background learning.
  This is a simple baseline, not a production-grade people detector.
"""

import math
import time
from collections import defaultdict

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2


def make_cloud(points: np.ndarray, header) -> PointCloud2:
    """Build an XYZ PointCloud2 from an Nx3 float array."""
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg = point_cloud2.create_cloud(header, fields, points.astype(np.float32).tolist())
    return msg


class DynamicLidarMap(Node):
    def __init__(self):
        super().__init__('dynamic_lidar_map')

        self.declare_parameter('input_topic', '/points')
        self.declare_parameter('voxel_size', 0.15)
        self.declare_parameter('min_range', 0.5)
        self.declare_parameter('max_range', 12.0)
        self.declare_parameter('min_z', -1.5)
        self.declare_parameter('max_z', 2.5)
        self.declare_parameter('background_learn_seconds', 8.0)
        self.declare_parameter('dynamic_ttl_seconds', 0.8)
        self.declare_parameter('publish_rate', 5.0)
        self.declare_parameter('max_voxels', 100000)

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.voxel_size = float(self.get_parameter('voxel_size').value)
        self.min_range = float(self.get_parameter('min_range').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.min_z = float(self.get_parameter('min_z').value)
        self.max_z = float(self.get_parameter('max_z').value)
        self.learn_seconds = float(self.get_parameter('background_learn_seconds').value)
        self.dynamic_ttl = float(self.get_parameter('dynamic_ttl_seconds').value)
        self.max_voxels = int(self.get_parameter('max_voxels').value)

        self.start_time = time.monotonic()
        self.background_counts = defaultdict(int)
        self.background = set()
        self.dynamic_latest = {}  # voxel -> (latest XYZ point, monotonic timestamp)
        self.background_ready = False
        self.latest_header = None
        self.scan_count = 0

        self.static_pub = self.create_publisher(PointCloud2, '/lidar_map/static', 10)
        self.dynamic_pub = self.create_publisher(PointCloud2, '/lidar_map/dynamic', 10)
        self.combined_pub = self.create_publisher(PointCloud2, '/lidar_map/combined', 10)

        self.sub = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.cloud_callback,
            qos_profile_sensor_data
        )

        rate = max(0.5, float(self.get_parameter('publish_rate').value))
        self.timer = self.create_timer(1.0 / rate, self.publish_maps)

        self.get_logger().info(
            f'Listening on {self.input_topic}. Keep the room still for '
            f'{self.learn_seconds:.1f}s while the static background is learned.'
        )

    def cloud_callback(self, msg: PointCloud2):
        try:
            raw = point_cloud2.read_points(
                msg, field_names=('x', 'y', 'z'), skip_nans=True
            )
            # sensor_msgs_py may return a structured ndarray in ROS 2 Jazzy.
            if isinstance(raw, np.ndarray) and raw.dtype.names:
                points = np.column_stack((raw['x'], raw['y'], raw['z'])).astype(np.float32)
            else:
                points = np.asarray(list(raw), dtype=np.float32).reshape((-1, 3))
        except Exception as exc:
            self.get_logger().error(f'Could not decode PointCloud2: {exc}')
            return

        if points.size == 0:
            return

        finite = np.isfinite(points).all(axis=1)
        points = points[finite]
        if points.size == 0:
            return

        ranges = np.linalg.norm(points, axis=1)
        keep = (
            (ranges >= self.min_range) &
            (ranges <= self.max_range) &
            (points[:, 2] >= self.min_z) &
            (points[:, 2] <= self.max_z)
        )
        points = points[keep]
        if points.size == 0:
            return

        # Use voxel keys to compare occupancy from scan to scan.
        keys = np.floor(points / self.voxel_size).astype(np.int32)
        unique_keys, first_indices = np.unique(keys, axis=0, return_index=True)
        points = points[first_indices]
        keys = unique_keys

        now = time.monotonic()
        self.latest_header = msg.header
        self.scan_count += 1

        if not self.background_ready:
            for key in keys:
                self.background_counts[tuple(int(v) for v in key)] += 1

            if now - self.start_time >= self.learn_seconds:
                # Keep voxels seen repeatedly during the learning window.
                # The threshold is modest because a spinning LiDAR sees many
                # surfaces intermittently due to scan pattern and occlusion.
                min_hits = max(2, int(self.scan_count * 0.08))
                self.background = {
                    key for key, count in self.background_counts.items()
                    if count >= min_hits
                }
                self.background_ready = True
                self.get_logger().info(
                    f'Background learned: {len(self.background)} voxels. '
                    'Dynamic detection is now active.'
                )
            return

        # Occupied voxels absent from the learned background are candidates
        # for dynamic points. Refresh their timestamp to avoid trails.
        for key, point in zip(keys, points):
            voxel = tuple(int(v) for v in key)
            if voxel not in self.background:
                self.dynamic_latest[voxel] = (point.copy(), now)

        # Remove stale dynamic voxels.
        expired = [
            key for key, (_, stamp) in self.dynamic_latest.items()
            if now - stamp > self.dynamic_ttl
        ]
        for key in expired:
            del self.dynamic_latest[key]

        # Bound memory if the environment is unusually large/noisy.
        if len(self.dynamic_latest) > self.max_voxels:
            oldest = sorted(self.dynamic_latest.items(), key=lambda item: item[1][1])
            remove_count = len(self.dynamic_latest) - self.max_voxels
            for key, _ in oldest[:remove_count]:
                self.dynamic_latest.pop(key, None)

    def publish_maps(self):
        if self.latest_header is None:
            return

        if not self.background_ready:
            return

        static_points = np.asarray(
            [((np.asarray(k, dtype=np.float32) + 0.5) * self.voxel_size)
             for k in self.background],
            dtype=np.float32
        ).reshape((-1, 3))

        dynamic_points = np.asarray(
            [value[0] for value in self.dynamic_latest.values()],
            dtype=np.float32
        ).reshape((-1, 3))

        static_msg = make_cloud(static_points, self.latest_header)
        dynamic_msg = make_cloud(dynamic_points, self.latest_header)
        combined = np.vstack((static_points, dynamic_points)) if len(dynamic_points) else static_points
        combined_msg = make_cloud(combined, self.latest_header)

        self.static_pub.publish(static_msg)
        self.dynamic_pub.publish(dynamic_msg)
        self.combined_pub.publish(combined_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DynamicLidarMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

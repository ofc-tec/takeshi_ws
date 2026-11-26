#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
)

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


INPUT_TOPIC = '/head_rgbd_sensor/depth_registered/rectified_points'
OUTPUT_TOPIC = '/takeshi/octomap_points'


class CloudRepublisher(Node):

    def __init__(self):
        super().__init__('cloud_republisher')

        qos_in = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        qos_out = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        self.pub = self.create_publisher(
            PointCloud2,
            OUTPUT_TOPIC,
            qos_out
        )

        self.sub = self.create_subscription(
            PointCloud2,
            INPUT_TOPIC,
            self.callback,
            qos_in
        )

        # Performance statistics
        self.count = 0
        self.total_points = 0
        self.total_processing_ms = 0.0
        self.total_age_ms = 0.0
        self.age_samples = 0

        self.last_report = time.monotonic()

        self.get_logger().info(
            f'Cloud republisher ready: {INPUT_TOPIC} -> {OUTPUT_TOPIC}'
        )

    def callback(self, msg):

        start = time.perf_counter()

        # Age relative to simulation clock.
        # Requires use_sim_time=true and a valid /clock.
        now_ns = self.get_clock().now().nanoseconds

        stamp_ns = (
            msg.header.stamp.sec * 1_000_000_000
            + msg.header.stamp.nanosec
        )

        if now_ns > 0 and stamp_ns > 0:
            age_ms = (now_ns - stamp_ns) / 1e6
            self.total_age_ms += age_ms
            self.age_samples += 1

        # Original working XYZ conversion
        xyz = point_cloud2.read_points_numpy(
            msg,
            field_names=("x", "y", "z"),
            skip_nans=True
        )

        cloud = point_cloud2.create_cloud_xyz32(
            msg.header,
            xyz
        )

        self.pub.publish(cloud)

        processing_ms = (
            time.perf_counter() - start
        ) * 1000.0

        self.count += 1
        self.total_points += len(xyz)
        self.total_processing_ms += processing_ms

        # Report every five seconds, using wall time
        now = time.monotonic()
        elapsed = now - self.last_report

        if elapsed >= 5.0:

            hz = self.count / elapsed

            avg_processing = (
                self.total_processing_ms / self.count
            )

            avg_points = (
                self.total_points / self.count
            )

            avg_age = (
                self.total_age_ms / self.age_samples
                if self.age_samples else float('nan')
            )

            self.get_logger().info(
                f'XYZ rate: {hz:.2f} Hz | '
                f'Points: {avg_points:.0f} | '
                f'Processing: {avg_processing:.1f} ms | '
                f'Cloud age: {avg_age:.1f} ms'
            )

            # Reset statistics
            self.count = 0
            self.total_points = 0
            self.total_processing_ms = 0.0
            self.total_age_ms = 0.0
            self.age_samples = 0

            self.last_report = now


def main(args=None):

    rclpy.init(args=args)

    node = CloudRepublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
import math
import time

import py_trees
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion
from rclpy.time import Time
from tf2_geometry_msgs import do_transform_point
from tf2_ros import TransformException
from tf_transformations import euler_from_quaternion, quaternion_from_euler


class ComputePregrasp(py_trees.behaviour.Behaviour):
    """Convert one fresh YOLO detection into an HSR top-down palm goal."""

    def __init__(self, node, tf_buffer, clearance):
        super().__init__("ComputePregrasp")
        self.node = node
        self.tf_buffer = tf_buffer
        self.clearance = clearance
        self.bb = self.attach_blackboard_client()
        for key in ("detections_log", "target_object"):
            self.bb.register_key(key, py_trees.common.Access.READ)
        self.bb.register_key("pregrasp_pose", py_trees.common.Access.WRITE)

    def initialise(self):
        self.bb.pregrasp_pose = None
        self.deadline = time.monotonic() + 5.0
        self.detection = None
        detections = self.bb.detections_log
        if detections:
            latest = detections[-1]
            matches = [
                pose for label, pose in zip(latest["classes"], latest["poses"])
                if label == self.bb.target_object and pose.header.frame_id
                and (pose.header.stamp.sec or pose.header.stamp.nanosec)
                and all(math.isfinite(value) for value in (
                    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z
                ))
            ]
            if matches:
                # The service returns positions in its sensor frame.
                self.detection = min(
                    matches,
                    key=lambda pose: math.hypot(
                        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z
                    ),
                )
                self.node.get_logger().info(
                    f"Selected the closest of {len(matches)} valid "
                    f"{self.bb.target_object!r} detections in "
                    f"{self.detection.header.frame_id}: {self.detection.pose}"
                )
                return

        self.node.get_logger().error(
            f"No valid 3D detection of {self.bb.target_object!r}"
        )

    def update(self):
        status = py_trees.common.Status
        if self.detection is None:
            return status.FAILURE

        source = self.detection.header.frame_id
        stamp = Time.from_msg(self.detection.header.stamp)
        try:
            sensor_tf = self.tf_buffer.lookup_transform("odom", source, stamp)
            base_tf = self.tf_buffer.lookup_transform("odom", "base_link", stamp)
        except TransformException as error:
            if time.monotonic() < self.deadline:
                return status.RUNNING
            self.node.get_logger().error(f"Cannot transform detection to odom: {error}")
            return status.FAILURE

        object_point = do_transform_point(
            PointStamped(header=self.detection.header, point=self.detection.pose.position),
            sensor_tf,
        )
        rotation = base_tf.transform.rotation
        _, _, yaw = euler_from_quaternion(
            [rotation.x, rotation.y, rotation.z, rotation.w]
        )
        # The original HSR above approach points the palm's +Z down and
        # aligns its horizontal heading with the base.
        x, y, z, w = quaternion_from_euler(math.pi, 0.0, yaw)
        goal = PoseStamped()
        goal.header.frame_id = "odom"
        goal.header.stamp = self.detection.header.stamp
        goal.pose.position = object_point.point
        goal.pose.position.z += self.clearance
        goal.pose.orientation = Quaternion(x=x, y=y, z=z, w=w)
        self.bb.pregrasp_pose = goal
        self.node.get_logger().info(
            f"Top-down hand_palm_link target in {goal.header.frame_id} "
            f"with {self.clearance:.3f} m vertical clearance: {goal.pose}"
        )
        return status.SUCCESS

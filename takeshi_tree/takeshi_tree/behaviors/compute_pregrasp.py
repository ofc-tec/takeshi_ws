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
        self.selected_point = None
        self.candidates = []
        detections = self.bb.detections_log
        target = self.bb.target_object
        if detections:
            latest = detections[-1]
            self.candidates = [
                (label, PointStamped(header=pose.header, point=pose.pose.position))
                for label, pose in zip(latest["classes"], latest["poses"])
                if (target == "any" or label == target) and pose.header.frame_id
                and (pose.header.stamp.sec or pose.header.stamp.nanosec)
                and all(math.isfinite(value) for value in (
                    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z
                ))
            ]
        if not self.candidates:
            self.node.get_logger().error(f"No valid 3D detection for {target!r}")

    def _select_closest(self):
        ranked = []
        for label, candidate in self.candidates:
            stamp = Time.from_msg(candidate.header.stamp)
            base_tf = self.tf_buffer.lookup_transform(
                "base_link", candidate.header.frame_id, stamp
            )
            point = do_transform_point(candidate, base_tf)
            # Keep the detection time for the subsequent odom lookup.
            point.header.stamp = candidate.header.stamp
            distance = math.hypot(point.point.x, point.point.y)
            ranked.append((distance, label, point))

        closest_distance, closest_label, closest_point = min(
            ranked, key=lambda candidate: candidate[0]
        )
        self.node.get_logger().info(
            f"Selected {closest_label!r} in base_link at "
            f"({closest_point.point.x:.3f}, {closest_point.point.y:.3f}, "
            f"{closest_point.point.z:.3f}) m, "
            f"planar distance {closest_distance:.3f} m"
        )
        return closest_point

    def update(self):
        status = py_trees.common.Status
        if not self.candidates:
            return status.FAILURE

        try:
            if self.selected_point is None:
                self.selected_point = self._select_closest()
            stamp = Time.from_msg(self.selected_point.header.stamp)
            base_tf = self.tf_buffer.lookup_transform("odom", "base_link", stamp)
        except TransformException as error:
            if time.monotonic() < self.deadline:
                return status.RUNNING
            self.node.get_logger().error(f"Cannot transform detection: {error}")
            return status.FAILURE

        object_point = do_transform_point(self.selected_point, base_tf)
        rotation = base_tf.transform.rotation
        _, _, yaw = euler_from_quaternion(
            [rotation.x, rotation.y, rotation.z, rotation.w]
        )
        # The original HSR above approach points the palm's +Z down and
        # aligns its horizontal heading with the base.
        x, y, z, w = quaternion_from_euler(math.pi, 0.0, yaw)
        goal = PoseStamped()
        goal.header.frame_id = "odom"
        goal.header.stamp = self.selected_point.header.stamp
        goal.pose.position = object_point.point
        goal.pose.position.z += self.clearance
        goal.pose.orientation = Quaternion(x=x, y=y, z=z, w=w)
        self.bb.pregrasp_pose = goal
        self.node.get_logger().info(
            f"Top-down hand_palm_link target in {goal.header.frame_id} "
            f"with {self.clearance:.3f} m vertical clearance: {goal.pose}"
        )
        return status.SUCCESS

#!/usr/bin/env python3

import math
import time

import py_trees
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
)
from rclpy.action import ActionClient
from shape_msgs.msg import SolidPrimitive


class MoveToPose(py_trees.behaviour.Behaviour):
    """Plan and execute a MoveIt pose goal asynchronously."""

    def __init__(
        self,
        name,
        node,
        pose=None,
        group_name="whole_body",
        end_effector_link="hand_palm_link",
        blackboard_key=None,
        timeout_sec=60.0,
        position_tolerance=0.01,
        orientation_tolerance=0.01,
        planning_time=10.0,
        planning_attempts=10,
        velocity_scaling=0.5,
        acceleration_scaling=0.5,
        plan_only=False,
        base_orientation_constraint=None,
        base_orientation_tolerance=0.1,
        base_orientation_link="base_footprint",
    ):
        super().__init__(name)

        self.node = node
        self.pose = pose
        self.group_name = group_name
        self.end_effector_link = end_effector_link
        self.blackboard_key = blackboard_key
        self.timeout_sec = float(timeout_sec)
        self.position_tolerance = float(position_tolerance)
        self.orientation_tolerance = float(orientation_tolerance)
        self.planning_time = float(planning_time)
        self.planning_attempts = int(planning_attempts)
        self.velocity_scaling = float(velocity_scaling)
        self.acceleration_scaling = float(acceleration_scaling)
        self.plan_only = bool(plan_only)
        self.base_orientation_constraint = base_orientation_constraint
        self.base_orientation_tolerance = float(base_orientation_tolerance)
        self.base_orientation_link = base_orientation_link

        self._blackboard = None
        if self.blackboard_key is not None:
            self._blackboard = self.attach_blackboard_client()
            self._blackboard.register_key(
                key=self.blackboard_key,
                access=py_trees.common.Access.READ,
            )

        self._client = ActionClient(node, MoveGroup, "/move_action")
        self._goal_future = None
        self._goal_handle = None
        self._result_future = None
        self._start_time = None

    def initialise(self):
        self._goal_future = None
        self._goal_handle = None
        self._result_future = None
        self._start_time = None

        try:
            self._validate_configuration()
            pose = self._get_pose()
            self._validate_pose(pose)
        except (TypeError, ValueError, AttributeError) as error:
            self.node.get_logger().error(f"Invalid MoveToPose goal: {error}")
            return

        if not self._client.wait_for_server(timeout_sec=10.0):
            self.node.get_logger().error(
                "MoveIt action server not available"
            )
            return

        self._start_time = time.monotonic()
        goal = self._build_goal(pose)
        self._goal_future = self._client.send_goal_async(
            goal,
            feedback_callback=self._feedback_callback,
        )
        self.node.get_logger().info(
            f"MoveIt: {self.group_name} -> "
            f"{self.end_effector_link} in {pose.header.frame_id}"
        )

    def _validate_configuration(self):
        if self.group_name not in ("arm", "whole_body"):
            raise ValueError(
                "group_name must be 'arm' or 'whole_body'"
            )
        if not self.end_effector_link:
            raise ValueError("end_effector_link must not be empty")
        if self.blackboard_key is None and self.pose is None:
            raise ValueError("provide pose or blackboard_key")
        if self.timeout_sec <= 0.0:
            raise ValueError("timeout_sec must be positive")
        if self.position_tolerance <= 0.0:
            raise ValueError("position_tolerance must be positive")
        if self.orientation_tolerance <= 0.0:
            raise ValueError("orientation_tolerance must be positive")
        if self.planning_time <= 0.0:
            raise ValueError("planning_time must be positive")
        if self.planning_attempts <= 0:
            raise ValueError("planning_attempts must be positive")
        if not 0.0 < self.velocity_scaling <= 1.0:
            raise ValueError("velocity_scaling must be in (0, 1]")
        if not 0.0 < self.acceleration_scaling <= 1.0:
            raise ValueError("acceleration_scaling must be in (0, 1]")
        if self.base_orientation_tolerance <= 0.0:
            raise ValueError("base_orientation_tolerance must be positive")
        if (
            self.base_orientation_constraint is not None
            and not self.base_orientation_link
        ):
            raise ValueError(
                "base_orientation_link must not be empty when enabled"
            )

        numeric_values = (
            self.timeout_sec,
            self.position_tolerance,
            self.orientation_tolerance,
            self.planning_time,
            self.base_orientation_tolerance,
        )
        if not all(math.isfinite(value) for value in numeric_values):
            raise ValueError("numeric parameters must be finite")

    def _get_pose(self):
        if self._blackboard is not None:
            try:
                pose = getattr(self._blackboard, self.blackboard_key)
            except AttributeError as error:
                raise ValueError(
                    f"blackboard key '{self.blackboard_key}' is unavailable"
                ) from error
        else:
            pose = self.pose

        if not isinstance(pose, PoseStamped):
            raise TypeError("pose must be a geometry_msgs/PoseStamped")
        return pose

    @staticmethod
    def _validate_quaternion(quaternion, label):
        values = (
            quaternion.x,
            quaternion.y,
            quaternion.z,
            quaternion.w,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f"{label} quaternion must be finite")
        if sum(value * value for value in values) < 1e-12:
            raise ValueError(f"{label} quaternion must not be zero")

    def _validate_pose(self, pose):
        if not pose.header.frame_id:
            raise ValueError("pose.header.frame_id must not be empty")
        position = (
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
        )
        if not all(math.isfinite(value) for value in position):
            raise ValueError("pose position must be finite")
        self._validate_quaternion(pose.pose.orientation, "pose")

        if self.base_orientation_constraint is not None:
            self._validate_quaternion(
                self.base_orientation_constraint,
                "base orientation constraint",
            )

    def _build_goal(self, pose):
        goal_constraints = Constraints()

        position_constraint = PositionConstraint()
        position_constraint.header = pose.header
        position_constraint.link_name = self.end_effector_link
        position_constraint.weight = 1.0

        region = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [self.position_tolerance]
        center = pose.pose
        region.primitives.append(sphere)
        region.primitive_poses.append(center)
        position_constraint.constraint_region = region
        goal_constraints.position_constraints.append(position_constraint)

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header = pose.header
        orientation_constraint.link_name = self.end_effector_link
        orientation_constraint.orientation = pose.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = (
            self.orientation_tolerance
        )
        orientation_constraint.absolute_y_axis_tolerance = (
            self.orientation_tolerance
        )
        orientation_constraint.absolute_z_axis_tolerance = (
            self.orientation_tolerance
        )
        orientation_constraint.weight = 1.0
        goal_constraints.orientation_constraints.append(
            orientation_constraint
        )

        goal = MoveGroup.Goal()
        goal.request.group_name = self.group_name
        goal.request.start_state.is_diff = True
        goal.request.goal_constraints = [goal_constraints]
        goal.request.num_planning_attempts = self.planning_attempts
        goal.request.allowed_planning_time = self.planning_time
        goal.request.max_velocity_scaling_factor = self.velocity_scaling
        goal.request.max_acceleration_scaling_factor = (
            self.acceleration_scaling
        )
        goal.planning_options.plan_only = self.plan_only
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        if self.base_orientation_constraint is not None:
            path_constraint = OrientationConstraint()
            path_constraint.header = pose.header
            path_constraint.link_name = self.base_orientation_link
            path_constraint.orientation = self.base_orientation_constraint
            path_constraint.absolute_x_axis_tolerance = (
                self.base_orientation_tolerance
            )
            path_constraint.absolute_y_axis_tolerance = (
                self.base_orientation_tolerance
            )
            path_constraint.absolute_z_axis_tolerance = (
                self.base_orientation_tolerance
            )
            path_constraint.weight = 1.0
            goal.request.path_constraints.orientation_constraints.append(
                path_constraint
            )

        return goal

    def update(self):
        status = py_trees.common.Status

        if self._goal_future is None or self._start_time is None:
            return status.FAILURE

        if time.monotonic() - self._start_time > self.timeout_sec:
            self.node.get_logger().warn("MoveIt timeout")
            return status.FAILURE

        try:
            if self._goal_handle is None:
                if not self._goal_future.done():
                    return status.RUNNING

                self._goal_handle = self._goal_future.result()
                if not self._goal_handle.accepted:
                    self.node.get_logger().error("MoveIt goal rejected")
                    return status.FAILURE

                self.node.get_logger().info("MoveIt goal accepted")
                self._result_future = (
                    self._goal_handle.get_result_async()
                )
                return status.RUNNING

            if not self._result_future.done():
                return status.RUNNING

            result = self._result_future.result()
            result_status = result.status
            error = result.result.error_code.val
            self.node.get_logger().info(
                f"MoveIt result: status={result_status}, error={error}"
            )
            if (
                result_status == GoalStatus.STATUS_SUCCEEDED
                and error == MoveItErrorCodes.SUCCESS
            ):
                return status.SUCCESS
            return status.FAILURE
        except Exception as error:
            self.node.get_logger().error(f"MoveIt exception: {error}")
            return status.FAILURE

    def _feedback_callback(self, feedback_msg):
        self.node.get_logger().info(
            f"MoveIt feedback: {feedback_msg.feedback.state}"
        )

    def terminate(self, new_status):
        if self._goal_handle is not None:
            if (
                self._goal_handle.accepted
                and (
                    self._result_future is None
                    or not self._result_future.done()
                )
            ):
                self.node.get_logger().info("Cancelling MoveIt goal")
                self._goal_handle.cancel_goal_async()
        elif self._goal_future is not None:
            def cancel_pending(future):
                try:
                    handle = future.result()
                    if handle.accepted:
                        handle.cancel_goal_async()
                except Exception:
                    self.node.get_logger().error(
                        "Failed to cancel pending MoveIt goal"
                    )

            self._goal_future.add_done_callback(cancel_pending)

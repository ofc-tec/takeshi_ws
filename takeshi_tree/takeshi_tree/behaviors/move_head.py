#!/usr/bin/env python3

import math
import time
import py_trees

from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes


class MoveHead(py_trees.behaviour.Behaviour):

    def __init__(self, name, node, target=(0.0, 0.0), timeout_sec=60.0):
        super().__init__(name)

        self.node = node
        self.target = target
        self.timeout_sec = timeout_sec

        self._client = ActionClient(
            node, MoveGroup, "/move_action"
        )

        self._goal_future = None
        self._goal_handle = None
        self._result_future = None
        self._start_time = None

    def initialise(self):
        self._goal_future = None
        self._goal_handle = None
        self._result_future = None

        if not self._client.wait_for_server(timeout_sec=10.0):
            self.node.get_logger().error(
                "MoveIt action server not available"
            )
            return

        try:
            target = tuple(float(value) for value in self.target)
        except (TypeError, ValueError):
            self.node.get_logger().error(
                "Head target must contain two numeric joint positions"
            )
            return

        if len(target) != 2 or not all(math.isfinite(value) for value in target):
            self.node.get_logger().error(
                "Head target must contain two finite joint positions"
            )
            return

        # Wall-clock timeout, independent of simulation time
        self._start_time = time.monotonic()

        constraints = Constraints()

        for joint_name, position in zip(
            ("head_pan_joint", "head_tilt_joint"), target
        ):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = position
            joint_constraint.tolerance_above = 0.01
            joint_constraint.tolerance_below = 0.01
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)

        goal = MoveGroup.Goal()
        goal.request.group_name = "head"
        goal.request.start_state.is_diff = True
        goal.request.goal_constraints = [constraints]

        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 10.0
        goal.request.max_velocity_scaling_factor = 0.5
        goal.request.max_acceleration_scaling_factor = 0.5

        goal.planning_options.plan_only = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        self._goal_future = self._client.send_goal_async(
            goal,
            feedback_callback=self._feedback_callback,
        )

        self.node.get_logger().info(
            f"MoveIt: head -> {target}"
        )

    def update(self):
        status = py_trees.common.Status

        if self._goal_future is None:
            return status.FAILURE

        elapsed = time.monotonic() - self._start_time

        if elapsed > self.timeout_sec:
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
            self.node.get_logger().error(
                f"MoveIt exception: {error}"
            )
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
                    pass

            self._goal_future.add_done_callback(cancel_pending)

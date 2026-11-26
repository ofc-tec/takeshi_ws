#!/usr/bin/env python3

import time
import py_trees

from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes


class MoveToNamedPose(py_trees.behaviour.Behaviour):

    def __init__(self, name, node, group="arm",
                 target="go", timeout_sec=60.0):

        super().__init__(name)

        self.node = node
        self.group = group
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

        # Wall-clock timeout, independent of simulation time
        self._start_time = time.monotonic()

        # Temporary named target for initial testing.
        # Matches the successful terminal action request.

        if self.group != "arm" or self.target != "go":
            self.node.get_logger().error(
                f"Unsupported target: {self.group}/{self.target}"
            )
            return

        joints = {
            "arm_lift_joint": 0.0,
            "arm_flex_joint": 0.0,
            "arm_roll_joint": -1.57,
            "wrist_flex_joint": -1.57,
            "wrist_roll_joint": 0.0,
        }

        constraints = Constraints()

        for joint_name, position in joints.items():

            jc = JointConstraint()

            jc.joint_name = joint_name
            jc.position = position
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0

            constraints.joint_constraints.append(jc)

        # Build MoveIt action goal

        goal = MoveGroup.Goal()

        goal.request.group_name = self.group
        goal.request.start_state.is_diff = True
        goal.request.goal_constraints = [constraints]

        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 10.0

        goal.request.max_velocity_scaling_factor = 0.5
        goal.request.max_acceleration_scaling_factor = 0.5

        # Plan and execute

        goal.planning_options.plan_only = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        self._goal_future = self._client.send_goal_async(
            goal,
            feedback_callback=self._feedback_callback,
        )

        self.node.get_logger().info(
            f"MoveIt: {self.group} -> {self.target}"
        )

    def update(self):

        S = py_trees.common.Status

        if self._goal_future is None:
            return S.FAILURE

        # Independent of ROS simulation clock
        elapsed = time.monotonic() - self._start_time

        if elapsed > self.timeout_sec:
            self.node.get_logger().warn("MoveIt timeout")
            return S.FAILURE

        try:

            # Wait for goal acceptance

            if self._goal_handle is None:

                if not self._goal_future.done():
                    return S.RUNNING

                self._goal_handle = self._goal_future.result()

                if not self._goal_handle.accepted:
                    self.node.get_logger().error(
                        "MoveIt goal rejected"
                    )
                    return S.FAILURE

                self.node.get_logger().info(
                    "MoveIt goal accepted"
                )

                self._result_future = (
                    self._goal_handle.get_result_async()
                )

                return S.RUNNING

            # Wait for planning and execution

            if not self._result_future.done():
                return S.RUNNING

            result = self._result_future.result()

            status = result.status
            error = result.result.error_code.val

            self.node.get_logger().info(
                f"MoveIt result: status={status}, error={error}"
            )

            if (
                status == GoalStatus.STATUS_SUCCEEDED
                and error == MoveItErrorCodes.SUCCESS
            ):
                return S.SUCCESS

            return S.FAILURE

        except Exception as e:

            self.node.get_logger().error(
                f"MoveIt exception: {e}"
            )

            return S.FAILURE

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
                self.node.get_logger().info(
                    "Cancelling MoveIt goal"
                )

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
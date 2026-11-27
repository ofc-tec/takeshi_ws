from unittest.mock import patch

import py_trees
from geometry_msgs.msg import PoseStamped, Quaternion

from takeshi_tree.behaviors.move_to_pose import MoveToPose


def create_behavior(pose, **kwargs):
    with patch("takeshi_tree.behaviors.move_to_pose.ActionClient"):
        return MoveToPose(
            name="test_move_to_pose",
            node=object(),
            pose=pose,
            **kwargs,
        )


def test_builds_pose_goal_and_plan_only_configuration():
    pose = PoseStamped()
    pose.header.frame_id = "odom"
    pose.pose.position.x = 1.0
    pose.pose.orientation.w = 1.0
    behavior = create_behavior(
        pose,
        group_name="arm",
        end_effector_link="hand_palm_link",
        position_tolerance=0.03,
        orientation_tolerance=0.2,
        planning_time=4.0,
        planning_attempts=5,
        velocity_scaling=0.25,
        acceleration_scaling=0.4,
        plan_only=True,
    )

    goal = behavior._build_goal(pose)
    constraints = goal.request.goal_constraints[0]
    position = constraints.position_constraints[0]
    orientation = constraints.orientation_constraints[0]

    assert goal.request.group_name == "arm"
    assert goal.request.num_planning_attempts == 5
    assert goal.request.allowed_planning_time == 4.0
    assert goal.request.max_velocity_scaling_factor == 0.25
    assert goal.request.max_acceleration_scaling_factor == 0.4
    assert goal.planning_options.plan_only
    assert position.header.frame_id == "odom"
    assert position.link_name == "hand_palm_link"
    assert position.constraint_region.primitives[0].dimensions[0] == 0.03
    assert orientation.header.frame_id == "odom"
    assert orientation.link_name == "hand_palm_link"
    assert orientation.absolute_x_axis_tolerance == 0.2
    assert orientation.absolute_y_axis_tolerance == 0.2
    assert orientation.absolute_z_axis_tolerance == 0.2
    assert not goal.request.path_constraints.orientation_constraints


def test_adds_optional_base_orientation_path_constraint():
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.orientation.w = 1.0
    base_orientation = Quaternion(w=1.0)
    behavior = create_behavior(
        pose,
        base_orientation_constraint=base_orientation,
        base_orientation_tolerance=0.15,
    )

    goal = behavior._build_goal(pose)
    constraints = goal.request.path_constraints.orientation_constraints

    assert len(constraints) == 1
    assert constraints[0].header.frame_id == "map"
    assert constraints[0].link_name == "base_footprint"
    assert constraints[0].orientation.w == base_orientation.w
    assert constraints[0].absolute_z_axis_tolerance == 0.15


def test_reads_pose_from_blackboard_key():
    py_trees.blackboard.Blackboard.clear()
    pose = PoseStamped()
    pose.header.frame_id = "odom"
    writer = py_trees.blackboard.Client(name="test_pose_writer")
    writer.register_key(
        key="target_pose",
        access=py_trees.common.Access.WRITE,
    )
    writer.target_pose = pose

    with patch("takeshi_tree.behaviors.move_to_pose.ActionClient"):
        behavior = MoveToPose(
            name="test_move_to_pose_blackboard",
            node=object(),
            blackboard_key="target_pose",
        )

    assert behavior._get_pose() is pose

    py_trees.blackboard.Blackboard.clear()

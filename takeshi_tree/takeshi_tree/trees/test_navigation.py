import py_trees
from py_trees.common import OneShotPolicy
from robotino_bts.behaviors.init_blackboard import InitBlackboard
from robotino_bts.behaviors.yolo_detect import YoloDetectBehaviour
from tf2_ros import Buffer, TransformListener

from takeshi_tree.behaviors.compute_pregrasp import ComputePregrasp
from takeshi_tree.behaviors.navigate_to_known_location import NavToKnownLocation
from takeshi_tree.behaviors.move_to_named_pose import MoveToNamedPose
from takeshi_tree.behaviors.move_head import MoveHead
from takeshi_tree.behaviors.move_to_pose import MoveToPose


def create_behavior_tree(node):
    target = node.declare_parameter("target_object", "cup").value.strip()
    group = node.declare_parameter("planning_group", "arm").value
    if not target:
        raise ValueError("target_object must be a YOLO class label")
    if group not in ("arm", "whole_body"):
        raise ValueError("planning_group must be 'arm' or 'whole_body'")

    tf_buffer = Buffer(node=node)
    node.pregrasp_tf_listener = TransformListener(tf_buffer, node)

    seq = py_trees.composites.Sequence(
        name="NavigationAndPregrasp",
        memory=True,
    )

    arm_go = MoveToNamedPose(
        name="ArmGo",
        node=node,
        group="arm",
        target="go",
    )

    nav = NavToKnownLocation(
        name="NavToLivingRoom",
        node=node,
        location_name="living_room",
    )

    move_head = MoveHead(
        name="MoveHead",
        node=node,
        target=[0.0, -0.2],
    )

    seq.add_children([
        arm_go,
        nav,
        move_head,
        InitBlackboard(target_object=target),
        py_trees.decorators.Timeout(
            name="DetectTargetWithTimeout",
            child=YoloDetectBehaviour("DetectTarget", node),
            duration=15.0,
        ),
        ComputePregrasp(node, tf_buffer, clearance=0.20),
        MoveToPose(
            "MoveToPregrasp",
            node,
            group_name=group,
            blackboard_key="pregrasp_pose",
            orientation_tolerance=0.10,
            velocity_scaling=0.15,
            acceleration_scaling=0.15,
        ),
    ])

    root = py_trees.decorators.OneShot(
        name="ROOT",
        child=seq,
        policy=OneShotPolicy.ON_COMPLETION,
    )

    return py_trees.trees.BehaviourTree(root)

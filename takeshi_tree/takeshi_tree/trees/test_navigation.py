import py_trees
from py_trees.common import OneShotPolicy

from takeshi_tree.behaviors.navigate_to_known_location import NavToKnownLocation
from takeshi_tree.behaviors.move_to_named_pose import MoveToNamedPose
from takeshi_tree.behaviors.move_head import MoveHead


def create_behavior_tree(node):

    seq = py_trees.composites.Sequence(
        name="NavigationSequence",
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
    ])

    root = py_trees.decorators.OneShot(
        name="ROOT",
        child=seq,
        policy=OneShotPolicy.ON_COMPLETION,
    )

    return py_trees.trees.BehaviourTree(root)
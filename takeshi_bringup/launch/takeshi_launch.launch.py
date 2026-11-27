from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    with_viewer = LaunchConfiguration("with_viewer")
    launch_dir = Path(__file__).parent
    bringup = [
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(launch_dir / filename)
                )
            ),
        ])
        for filename in (
            "takeshi_rc2025.launch.py",
            "hsrb_moveit.launch.py",
            "vision_takeshi.launch.py",
        )
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            "with_viewer",
            default_value="true",
            description="Start py_trees_ros_viewer GUI (py-trees-tree-viewer)",
        ),
        DeclareLaunchArgument(
            "start_tree",
            default_value="false",
            description="Start the behavior tree concurrently with simulation bringup.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use the simulation clock for the optional behavior tree.",
        ),
        *bringup,
        Node(
            package="takeshi_tree",
            executable="task_manager",
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_tree")),
            parameters=[{
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            }],
        ),
        ExecuteProcess(
            condition=IfCondition(with_viewer),
            cmd=["py-trees-tree-viewer"],
            output="screen",
        ),
    ])

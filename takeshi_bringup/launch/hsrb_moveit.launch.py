#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    moveit_launch = os.path.join(
        get_package_share_directory('hsrb_moveit_config'),
        'launch',
        'hsrb_demo.launch.py'
    )

    return LaunchDescription([

        # XYZ point cloud adapter for OctoMap
        Node(
            package='takeshi_bringup',
            executable='cloud_republisher.py',
            name='cloud_republisher',
            output='screen'
        ),

        # Official Toyota MoveIt 2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(moveit_launch),
            launch_arguments={
                'use_sim_time': 'true'
            }.items()
        )
    ])
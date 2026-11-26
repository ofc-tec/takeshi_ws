#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():

    # RC2025 world from Festeshi
    world = os.path.join(
        get_package_share_directory('rto_description'),
        'worlds',
        'robotino_gz_rc2025_xarm.sdf'
    )

    # Official Toyota HSR bringup
    gazebo_launch = os.path.join(
        get_package_share_directory('hsrb_gazebo_bringup'),
        'launch',
        'gazebo_bringup.launch.py'
    )

    # Official Toyota navigation
    navigation_launch = os.path.join(
        get_package_share_directory('hsrb_common_launch'),
        'launch',
        'navigation.py'
    )

    # Same empty map as Toyota's empty world
    empty_map = os.path.join(
        get_package_share_directory('tmc_potential_maps'),
        'maps',
        'white_space',
        'map.yaml'
    )

    # Official Toyota RViz configuration
    rviz_config = os.path.join(
        get_package_share_directory('hsrb_gazebo_launch'),
        'config',
        'display_full_config.rviz'
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'world',
            default_value=world
        ),

        DeclareLaunchArgument(
            'map',
            default_value=empty_map
        ),

        DeclareLaunchArgument(
            'rviz',
            default_value='true'
        ),

        DeclareLaunchArgument(
            'use_navigation',
            default_value='true'
        ),
        # Use NVIDIA GPU for Gazebo rendering
        SetEnvironmentVariable(
            name='__NV_PRIME_RENDER_OFFLOAD',
            value='1'
        ),

        SetEnvironmentVariable(
            name='__GLX_VENDOR_LIBRARY_NAME',
            value='nvidia'
        ),

        # Gazebo + Takeshi + controllers + sensors
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                'world_file_name': LaunchConfiguration('world'),
                'robot_pos_x': '2.443',
                'robot_pos_y': '-0.528',
                'robot_pos_z': '0.0',
                'robot_rpy_Y': '-3.068'
            }.items()
        ),

        # Toyota navigation stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch),
            launch_arguments={
                'use_sim_time': 'true',
                'map': LaunchConfiguration('map'),
                'odom_topic': 'odom'
            }.items(),
            condition=IfCondition(
                LaunchConfiguration('use_navigation')
            )
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(
                LaunchConfiguration('rviz')
            )
        )
    ])
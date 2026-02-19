#!/usr/bin/env python3
"""
Launch file for Line Follower with simple line track.
Uses Gazebo Ignition (Harmonic) for ROS2 Jazzy.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories
    pkg_line_follower = get_package_share_directory('line_follower_pkg')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    # World file path
    world_file = os.path.join(pkg_line_follower, 'worlds', 'line_track.sdf')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Set TURTLEBOT3_MODEL environment variable
    set_tb3_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle')

    # TurtleBot3 Gazebo launch (includes Gazebo + robot spawn)
    turtlebot3_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'turtlebot3_world.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    # Line follower node
    line_follower_node = Node(
        package='line_follower_pkg',
        executable='line_follower_node',
        name='line_follower_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Launch description
    ld = LaunchDescription()

    # Declare launch arguments
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    ))

    # Add actions
    ld.add_action(set_tb3_model)
    ld.add_action(turtlebot3_gazebo)
    ld.add_action(line_follower_node)

    return ld

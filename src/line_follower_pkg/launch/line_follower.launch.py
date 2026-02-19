#!/usr/bin/env python3
"""
Launch file for TurtleBot3 Line Follower
Launches Gazebo with custom track and TurtleBot3 Waffle
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories
    pkg_line_follower = get_package_share_directory('line_follower_pkg')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='-1.5')
    yaw = LaunchConfiguration('yaw', default='0.0')

    # World file path
    world_file = os.path.join(pkg_line_follower, 'worlds', 'line_follower_track.world')

    # Gazebo server
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    # Gazebo client
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # Robot state publisher (for TurtleBot3)
    urdf_file = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'urdf',
        'turtlebot3_waffle.urdf'
    )

    # Spawn TurtleBot3 Waffle
    spawn_turtlebot_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'turtlebot3_waffle',
            '-file', urdf_file,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01',
            '-Y', yaw
        ],
        output='screen'
    )

    # Robot state publisher
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
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
        description='Use simulation (Gazebo) clock if true'
    ))
    ld.add_action(DeclareLaunchArgument(
        'x_pose',
        default_value='0.0',
        description='Initial X position of the robot'
    ))
    ld.add_action(DeclareLaunchArgument(
        'y_pose',
        default_value='-1.5',
        description='Initial Y position of the robot'
    ))
    ld.add_action(DeclareLaunchArgument(
        'yaw',
        default_value='0.0',
        description='Initial yaw angle of the robot'
    ))

    # Add actions
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(line_follower_node)

    return ld

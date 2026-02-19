"""
Real Robot Line Follower Launch File

Launch configuration for running line follower on real TurtleBot3 hardware.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('line_follower_pkg')

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation time (False for real robot)'
    )

    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='True',
        description='Enable debug visualizations'
    )

    show_camera_arg = DeclareLaunchArgument(
        'show_camera',
        default_value='False',
        description='Launch rqt_image_view for camera visualization'
    )

    # Real Robot Line Follower Node
    line_follower_node = Node(
        package='line_follower_pkg',
        executable='real_robot_line_follower',
        name='real_robot_line_follower_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        emulate_tty=True,
        # Set environment variables for network discovery
        additional_env={
            'ROS_DOMAIN_ID': '36',
            'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
            'ROS_AUTOMATIC_DISCOVERY_RANGE': 'SUBNET',
            'ROS_LOCALHOST_ONLY': '0'
        }
    )

    # Optional: RQT Image View for debugging
    rqt_image_view = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('show_camera')),
        cmd=['ros2', 'run', 'rqt_image_view', 'rqt_image_view'],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        debug_arg,
        show_camera_arg,
        line_follower_node,
        rqt_image_view,
    ])

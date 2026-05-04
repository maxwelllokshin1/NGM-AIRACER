#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Launch the safety node with parameters
    """

    # Get package directory
    pkg_dir = get_package_share_directory('AEB_System')

    # Path to parameter file
    params_file = os.path.join(pkg_dir, 'config', 'safety_params.yaml')

    # Declare launch arguments
    ttc_threshold_arg = DeclareLaunchArgument(
        'ttc_threshold',
        default_value='0.5',
        description='Time to Collision threshold in seconds'
    )

    speed_threshold_arg = DeclareLaunchArgument(
        'speed_threshold',
        default_value='0.1',
        description='Minimum speed to activate AEB (m/s)'
    )

    # Safety node
    safety_node = Node(
        package='AEB_System',
        executable='safety_node',
        name='safety_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            params_file,
            {
                'ttc_threshold': LaunchConfiguration('ttc_threshold'),
                'speed_threshold': LaunchConfiguration('speed_threshold'),
            }
        ]
    )

    return LaunchDescription([
        ttc_threshold_arg,
        speed_threshold_arg,
        safety_node,
    ])
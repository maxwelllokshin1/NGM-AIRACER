#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch.actions import TimerAction, ExecuteProcess
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    movement_dir = get_package_share_directory('follow_the_gap')

    gap_params = os.path.join(movement_dir, 'config', 'gap_params.yaml')
    # use the slam map 
    slam_params = os.path.join(movement_dir, 'config', 'slam_params.yaml')

    movement_node = Node(
        package='follow_the_gap',
        executable='reactive_node',
        name='reactive_node',
        output='screen',
        emulate_tty=True,
        parameters=[gap_params],
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params],
    )

    return LaunchDescription([
        movement_node,
        # slam_node
    ])
#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch.actions import TimerAction, ExecuteProcess
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    movement_dir = get_package_share_directory('movement_node')
    aeb_dir = get_package_share_directory('AEB_System')
    vesc_config = os.path.join(
        get_package_share_directory('f1tenth_stack'), 'config', 'vesc.yaml')
    mux_config = os.path.join(
        get_package_share_directory('f1tenth_stack'), 'config', 'mux.yaml')

    safety_params = os.path.join(aeb_dir, 'config', 'safety_params.yaml')
    reactive_params = os.path.join(movement_dir, 'config', 'movement.yaml')

    # ── Lidar node (lifecycle) ──────────────────────────────────────
    lidar_node = LifecycleNode(
        package='urg_node2',
        executable='urg_node2_node',
        name='urg_node2_node',
        namespace='',
        output='screen',
        parameters=[{
            'ip_address': '192.168.0.10',
            'ip_port': 10940,
            'frame_id': 'laser',
            'angle_min': -2.0944,
            'angle_max': 2.0944,
        }],
        remappings=[('scan', '/scan')],
    )

    # Jetson takes ~3s to connect to lidar hardware.
    # Configure at 4s (safely after connection), activate at 7s (3s after configure).
    lidar_configure = TimerAction(
        period=4.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/urg_node2_node', 'configure'],
                output='screen',
            )
        ]
    )

    lidar_activate = TimerAction(
        period=7.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/urg_node2_node', 'activate'],
                output='screen',
            )
        ]
    )

    # ── VESC / motor nodes ──────────────────────────────────────────
    # Delay VESC driver until after lidar is active (t=7s) and trajectory node
    # has had time to receive scan data and publish a valid drive command.
    # This prevents the default servo value from killing the VESC on startup.
    vesc_driver_node = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        parameters=[vesc_config],
        output='screen',
    )

    vesc_driver = TimerAction(
        period=10.0,
        actions=[vesc_driver_node],
    )

    ackermann_to_vesc = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc_node',
        parameters=[vesc_config],
        output='screen',
    )

    vesc_to_odom = Node(
        package='vesc_ackermann',
        executable='vesc_to_odom_node',
        name='vesc_to_odom_node',
        parameters=[vesc_config],
        output='screen',
    )

    mux = Node(
        package='ackermann_mux',
        executable='ackermann_mux',
        name='ackermann_mux',
        parameters=[mux_config],
        remappings=[('ackermann_drive_out', 'ackermann_cmd')],
        output='screen',
    )

    # ── Autonomy nodes ──────────────────────────────────────────────
    safety_node = Node(
        package='AEB_System',
        executable='safety_node',
        name='safety_node',
        output='screen',
        emulate_tty=True,
        parameters=[safety_params],
    )

    movement_node = Node(
        package='movement_node',
        executable='trajectory',
        name='trajectory',
        output='screen',
        emulate_tty=True,
        parameters=[reactive_params],
    )

    return LaunchDescription([
        lidar_node,
        lidar_configure,
        lidar_activate,
        vesc_driver,          # delayed 8s
        ackermann_to_vesc,
        vesc_to_odom,
        mux,
        safety_node,
        movement_node,
    ])
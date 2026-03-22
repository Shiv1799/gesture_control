#!/usr/bin/env python3
"""
Launch file: gesture tracking (webcam + MediaPipe) and MoveIt Servo bridge.
Use this if you already have Panda+MoveIt+Servo running separately.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_pkg = get_package_share_directory('panda_gesture_bringup')
    gesture_control_pkg = get_package_share_directory('gesture_control')

    workspace_bounds_file = os.path.join(
        bringup_pkg, 'config', 'workspace_bounds.yaml')
    gesture_params_file = os.path.join(
        gesture_control_pkg, 'config', 'gesture_params.yaml')

    camera_id_arg = DeclareLaunchArgument('camera_id', default_value='0')
    smoothing_arg = DeclareLaunchArgument('smoothing_factor', default_value='0.4')
    velocity_gain_arg = DeclareLaunchArgument('velocity_gain', default_value='3.0')

    gesture_tracker_node = Node(
        package='hand_gesture_node',
        executable='gesture_tracker',
        name='gesture_tracker',
        output='screen',
        parameters=[
            workspace_bounds_file,
            {
                'camera_id': LaunchConfiguration('camera_id'),
                'smoothing_factor': LaunchConfiguration('smoothing_factor'),
            },
        ],
    )

    gesture_bridge_node = Node(
        package='gesture_control',
        executable='gesture_bridge',
        name='gesture_bridge',
        output='screen',
        parameters=[
            gesture_params_file,
            workspace_bounds_file,
            {
                'velocity_gain': LaunchConfiguration('velocity_gain'),
            },
        ],
    )

    return LaunchDescription([
        camera_id_arg,
        smoothing_arg,
        velocity_gain_arg,
        gesture_tracker_node,
        gesture_bridge_node,
    ])

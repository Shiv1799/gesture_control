#!/usr/bin/env python3
"""
Unified Launch: Panda MoveIt2 + Servo + Gesture Tracking + Bridge

Usage:
  ros2 launch panda_gesture_bringup gesture_panda_all.launch.py
  ros2 launch panda_gesture_bringup gesture_panda_all.launch.py use_rviz:=false camera_id:=1
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    moveit_config_pkg = get_package_share_directory(
        'moveit_resources_panda_moveit_config')
    bringup_pkg = get_package_share_directory('panda_gesture_bringup')
    gesture_control_pkg = get_package_share_directory('gesture_control')

    # --- Launch arguments ---
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true')
    camera_id_arg = DeclareLaunchArgument(
        'camera_id', default_value='0')
    smoothing_arg = DeclareLaunchArgument(
        'smoothing_factor', default_value='0.4')
    velocity_gain_arg = DeclareLaunchArgument(
        'velocity_gain', default_value='3.0')

    # --- 1. MoveIt2 Panda demo ---
    demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_pkg, 'launch', 'demo.launch.py')
        ),
        launch_arguments={
            'rviz_config': os.path.join(
                moveit_config_pkg, 'launch', 'moveit.rviz'),
        }.items(),
    )

    # --- 2. MoveIt Servo ---
    urdf_path = os.path.join(
        get_package_share_directory('moveit_resources_panda_description'),
        'urdf', 'panda.urdf')
    with open(urdf_path, 'r') as f:
        robot_description_content = f.read()
    robot_description = {'robot_description': robot_description_content}

    srdf_path = os.path.join(moveit_config_pkg, 'config', 'panda.srdf')
    with open(srdf_path, 'r') as f:
        srdf_content = f.read()
    robot_description_semantic = {'robot_description_semantic': srdf_content}

    servo_params_path = os.path.join(
        bringup_pkg, 'config', 'panda_servo_config.yaml')
    with open(servo_params_path, 'r') as f:
        servo_params = yaml.safe_load(f)

    servo_node = Node(
        package='moveit_servo',
        executable='servo_node_main',
        name='servo_node',
        output='screen',
        parameters=[servo_params, robot_description, robot_description_semantic],
    )

    # --- Config files ---
    workspace_bounds_file = os.path.join(
        bringup_pkg, 'config', 'workspace_bounds.yaml')
    gesture_params_file = os.path.join(
        gesture_control_pkg, 'config', 'gesture_params.yaml')

    # --- 3. Gesture tracker (delayed 3s) ---
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

    # --- 4. Gesture bridge (delayed 5s) ---
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

    delayed_tracker = TimerAction(period=3.0, actions=[gesture_tracker_node])
    delayed_bridge = TimerAction(period=5.0, actions=[gesture_bridge_node])

    return LaunchDescription([
        use_rviz_arg,
        camera_id_arg,
        smoothing_arg,
        velocity_gain_arg,
        demo_launch,
        servo_node,
        delayed_tracker,
        delayed_bridge,
    ])

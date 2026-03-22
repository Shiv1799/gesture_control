#!/usr/bin/env python3
"""
Launch file: Panda + MoveIt2 + Servo.
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_yaml(package_name, file_path):
    full_path = os.path.join(
        get_package_share_directory(package_name), file_path
    )
    with open(full_path, 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz2 with MoveIt2 plugin')

    moveit_config_pkg = get_package_share_directory(
        'moveit_resources_panda_moveit_config')

    demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_pkg, 'launch', 'demo.launch.py')
        ),
        launch_arguments={
            'rviz_config': os.path.join(
                moveit_config_pkg, 'launch', 'moveit.rviz'
            ),
        }.items(),
    )

    urdf_path = os.path.join(
        get_package_share_directory('moveit_resources_panda_description'),
        'urdf', 'panda.urdf',
    )
    with open(urdf_path, 'r') as f:
        robot_description_content = f.read()
    robot_description = {'robot_description': robot_description_content}

    srdf_path = os.path.join(moveit_config_pkg, 'config', 'panda.srdf')
    with open(srdf_path, 'r') as f:
        srdf_content = f.read()
    robot_description_semantic = {
        'robot_description_semantic': srdf_content
    }

    bringup_pkg = get_package_share_directory('panda_gesture_bringup')
    servo_params_path = os.path.join(
        bringup_pkg, 'config', 'panda_servo_config.yaml')
    with open(servo_params_path, 'r') as f:
        servo_params = yaml.safe_load(f)

    servo_node = Node(
        package='moveit_servo',
        executable='servo_node_main',
        name='servo_node',
        output='screen',
        parameters=[
            servo_params,
            robot_description,
            robot_description_semantic,
        ],
    )

    return LaunchDescription([
        use_rviz_arg,
        demo_launch,
        servo_node,
    ])

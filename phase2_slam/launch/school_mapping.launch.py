#!/usr/bin/env python3
"""Bring up the physical car for low-speed school SLAM mapping."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    phase2_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    repo_dir = os.path.dirname(phase2_dir)
    mapping_config = os.path.join(phase2_dir, 'config')
    f1tenth_config = os.path.join(
        get_package_share_directory('f1tenth_stack'), 'config')

    joy_config = os.path.join(mapping_config, 'joy_mapping.yaml')
    slam_config = os.path.join(
        mapping_config, 'mapper_params_online_async.yaml')
    vesc_config = os.path.join(f1tenth_config, 'vesc.yaml')
    sensors_config = os.path.join(f1tenth_config, 'sensors.yaml')
    mux_config = os.path.join(f1tenth_config, 'mux.yaml')

    return LaunchDescription([
        Node(
            package='joy', executable='joy_node', name='joy',
            parameters=[joy_config], output='screen'),
        Node(
            package='joy_teleop', executable='joy_teleop', name='joy_teleop',
            parameters=[joy_config], output='screen'),
        Node(
            package='vesc_ackermann', executable='ackermann_to_vesc_node',
            name='ackermann_to_vesc_node', parameters=[vesc_config],
            output='screen'),
        Node(
            package='vesc_ackermann', executable='vesc_to_odom_node',
            name='vesc_to_odom_node', parameters=[vesc_config],
            output='screen'),
        Node(
            package='vesc_driver', executable='vesc_driver_node',
            name='vesc_driver_node', parameters=[vesc_config],
            output='screen'),
        Node(
            package='urg_node', executable='urg_node_driver', name='urg_node',
            parameters=[sensors_config], output='screen'),
        Node(
            package='ackermann_mux', executable='ackermann_mux',
            name='ackermann_mux', parameters=[mux_config],
            remappings=[('ackermann_drive_out', 'ackermann_cmd')],
            output='screen'),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='static_baselink_to_laser',
            arguments=[
                '0.295', '0.0', '0.165', '0.0', '0.0', '0.0',
                'base_link', 'laser'],
            output='screen'),
        Node(
            package='slam_toolbox', executable='async_slam_toolbox_node',
            name='slam_toolbox', parameters=[slam_config, {'use_sim_time': False}],
            output='screen'),
        ExecuteProcess(
            cmd=[
                'python3',
                os.path.join(repo_dir, 'phase1_sysid', 'scripts', 'bno086_node.py'),
                '--ros-args', '-p', 'publish_rate_hz:=100.0'],
            output='screen'),
    ])

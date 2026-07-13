#!/usr/bin/env python3
"""Bring up the physical car for low-speed school SLAM mapping."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    # robot_localization is installed in a user-owned overlay on the Jetson so
    # this launch remains usable without sudo access to /opt/ros.
    overlay_root = os.path.expanduser('~/ros_overlay')
    overlay_ros = os.path.join(overlay_root, 'opt', 'ros', 'humble')
    overlay_libs = [
        os.path.join(overlay_ros, 'lib'),
        os.path.join(overlay_root, 'usr', 'lib', 'aarch64-linux-gnu'),
    ]
    if os.path.isdir(overlay_ros):
        os.environ['AMENT_PREFIX_PATH'] = os.pathsep.join(
            [overlay_ros, os.environ.get('AMENT_PREFIX_PATH', '')])
        os.environ['LD_LIBRARY_PATH'] = os.pathsep.join(
            overlay_libs + [os.environ.get('LD_LIBRARY_PATH', '')])

    phase2_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    repo_dir = os.path.dirname(phase2_dir)
    mapping_config = os.path.join(phase2_dir, 'config')
    f1tenth_config = os.path.join(
        get_package_share_directory('f1tenth_stack'), 'config')

    joy_config = os.path.join(mapping_config, 'joy_mapping.yaml')
    slam_config = os.path.join(
        mapping_config, 'mapper_params_online_async.yaml')
    ekf_config = os.path.join(mapping_config, 'ekf.yaml')
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
            name='vesc_to_odom_node',
            parameters=[vesc_config, {'publish_tf': False}],
            remappings=[('odom', '/wheel/odom')], output='screen'),
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
            package='tf2_ros', executable='static_transform_publisher',
            name='static_baselink_to_imu',
            arguments=[
                '0.0', '0.0', '0.1', '0.0', '0.0', '0.0',
                'base_link', 'imu_link'],
            output='screen'),
        Node(
            package='robot_localization', executable='ekf_node',
            name='ekf_filter_node', parameters=[ekf_config],
            remappings=[('/odometry/filtered', '/odom')], output='screen'),
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

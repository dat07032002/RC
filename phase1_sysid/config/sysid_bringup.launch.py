"""
Standalone bringup for Phase 1 System ID.

Runs ONLY:
  * vesc_driver_node   - talks to the VESC (servo + motor + telemetry + odom)
  * vesc_to_odom_node  - publishes /odom from VESC telemetry
  * urg_node           - Hokuyo 10LX -> /scan
  * pwm_vesc_bridge    - /servo/command + /motor/command (PWM) -> VESC commands

Deliberately does NOT start joy / joy_teleop / ackermann_mux / throttle
interpolator, so the sysid scripts have exclusive, direct control of the
VESC command topics.

Usage:
    ros2 launch <this_file> max_speed_mps:=2.0
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    f1t = get_package_share_directory('f1tenth_stack')
    vesc_config = os.path.join(f1t, 'config', 'vesc.yaml')
    sensors_config = os.path.join(f1t, 'config', 'sensors.yaml')

    bridge_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'scripts', 'pwm_vesc_bridge.py')

    max_speed = DeclareLaunchArgument('max_speed_mps', default_value='2.0',
                                      description='SAFETY cap on motor speed (m/s)')
    record = DeclareLaunchArgument('record', default_value='false',
                                   description='true -> record a rosbag of all topics')

    vesc_driver_node = Node(
        package='vesc_driver', executable='vesc_driver_node',
        name='vesc_driver_node', parameters=[vesc_config])

    vesc_to_odom_node = Node(
        package='vesc_ackermann', executable='vesc_to_odom_node',
        name='vesc_to_odom_node', parameters=[vesc_config])

    urg_node = Node(
        package='urg_node', executable='urg_node_driver',
        name='urg_node', output='screen', parameters=[sensors_config])

    bridge_node = Node(
        executable=bridge_path, name='pwm_vesc_bridge', output='screen',
        parameters=[{'max_speed_mps': LaunchConfiguration('max_speed_mps')}])

    return LaunchDescription([
        max_speed, record,
        vesc_driver_node, vesc_to_odom_node, urg_node, bridge_node,
    ])

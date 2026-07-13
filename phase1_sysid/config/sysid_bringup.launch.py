"""
Standalone bringup for Phase 1 System ID.

Runs ONLY:
  * vesc_driver_node   - talks to the VESC (servo + motor + telemetry + odom)
  * vesc_to_odom_node  - publishes /odom from VESC telemetry
  * urg_node           - Hokuyo 10LX -> /scan
  * pwm_vesc_bridge    - /servo/command + /motor/command (PWM) -> VESC commands
  * bno086_node        - optional SparkFun BNO086 -> /imu/data

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
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    f1t = get_package_share_directory('f1tenth_stack')
    vesc_config = os.path.join(f1t, 'config', 'vesc.yaml')
    sensors_config = os.path.join(f1t, 'config', 'sensors.yaml')

    bridge_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'scripts', 'pwm_vesc_bridge.py')
    bno086_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'scripts', 'bno086_node.py')

    max_speed = DeclareLaunchArgument('max_speed_mps', default_value='2.0',
                                      description='SAFETY cap on motor speed (m/s)')
    record = DeclareLaunchArgument('record', default_value='false',
                                   description='true -> record a rosbag of all topics')
    use_bridge = DeclareLaunchArgument(
        'use_bridge', default_value='true',
        description='false -> skip pwm_vesc_bridge (its watchdog brakes the motor; '
                    'disable it for the freewheel coast-down test)')
    use_bno086 = DeclareLaunchArgument(
        'use_bno086', default_value='false',
        description='true -> publish the external BNO086 on /imu/data')
    bno086_bus = DeclareLaunchArgument('bno086_bus', default_value='7')
    bno086_address = DeclareLaunchArgument('bno086_address', default_value='75')
    bno086_orientation = DeclareLaunchArgument(
        'bno086_orientation', default_value='game',
        description="game avoids motor magnetic interference; rotation uses 9-axis heading")

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
        condition=IfCondition(LaunchConfiguration('use_bridge')),
        parameters=[{'max_speed_mps': LaunchConfiguration('max_speed_mps')}])

    bno086_node = Node(
        executable=bno086_path, name='bno086_node', output='screen',
        condition=IfCondition(LaunchConfiguration('use_bno086')),
        parameters=[{
            'i2c_bus': ParameterValue(
                LaunchConfiguration('bno086_bus'), value_type=int),
            'i2c_address': ParameterValue(
                LaunchConfiguration('bno086_address'), value_type=int),
            'orientation_mode': LaunchConfiguration('bno086_orientation'),
        }])

    return LaunchDescription([
        max_speed, record, use_bridge, use_bno086,
        bno086_bus, bno086_address, bno086_orientation,
        vesc_driver_node, vesc_to_odom_node, urg_node, bridge_node, bno086_node,
    ])

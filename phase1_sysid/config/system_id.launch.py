"""
ROS 2 Launch file for Phase 1 System ID
Starts SLAM and telemetry nodes for system identification
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # SLAM node (if available)
        Node(
            package='lio_sam',
            executable='run',
            name='lio_sam',
            output='screen',
            parameters=[
                {'use_imu': True},
            ]
        ),

        # Terrain features extractor (optional, for Phase 2+)
        # Node(
        #     package='roboracer_perception',
        #     executable='terrain_features',
        #     name='terrain_features',
        #     output='screen',
        # ),

        # Rosbag recorder (logs all topics for analysis)
        Node(
            package='ros2_bag',
            executable='record',
            name='rosbag_recorder',
            arguments=['-a', '-o', 'sysid_data'],
            output='screen',
        ),
    ])

# Usage:
# ros2 launch roboracer_control system_id.launch.py

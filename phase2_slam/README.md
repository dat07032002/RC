# School SLAM mapping

The launch file brings up the VESC, wheel odometry, Hokuyo LiDAR, 8BitDo
teleoperation, BNO086, TF, and `slam_toolbox` in online asynchronous mapping
mode. Full joystick command is capped at 1.0 m/s.

Run on the Jetson from a clean process state:

```bash
source /opt/ros/humble/setup.bash
source ~/f1tenth_ws/install/setup.bash
ros2 launch ~/RC/phase2_slam/launch/school_mapping.launch.py
```

In another terminal, record the mapping inputs:

```bash
mkdir -p ~/bags ~/maps
ros2 bag record -o ~/bags/school_room_mapping \
  /scan /odom /tf /tf_static /imu/data /teleop /ackermann_cmd
```

After completing the perimeter, crossing the room, and returning to the exact
start marker, save both artifacts:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/room_empty
ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph "{filename: '/home/dat/maps/room_empty'}"
```

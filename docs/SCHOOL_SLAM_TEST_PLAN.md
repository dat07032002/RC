# School SLAM Test Plan

Use this checklist for the school testing session before moving into policy deployment.
The goal is to validate that the car can produce a reliable map, relocalize against it,
and tolerate unmapped obstacles in the room.

## Priority Order

1. LiDAR scan-plane tilt check.
2. Full stack bring-up: VESC, odom, LiDAR, TF, joystick.
3. Odometry repeatability runs.
4. SLAM mapping run.
5. Map validation: loop closure and metric accuracy.
6. Localization-mode rehearsal.
7. Obstacle preview with 2-3 boxes.
8. Optional remaining sysid: coast-down friction, weight, BNO086 wiring.

## 1. Preflight

Start from a clean process state so the LiDAR TCP slot and VESC serial port are not
held by an old launch:

```bash
pkill -9 -f "vesc_driver_node|urg_node_driver|vesc_to_odom_node|pwm_vesc_bridge|slam_toolbox|joy_node|ackermann"
ros2 daemon stop
ros2 daemon start
```

Bring up the full teleop stack you intend to use for mapping. Then verify:

```bash
ros2 topic hz /scan
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom base_link
ros2 topic list
```

Pass criteria:
- `/scan` publishes near the expected Hokuyo rate, about 40 Hz.
- `/odom` publishes while the car is moved.
- TF has a valid `odom -> base_link` chain.
- Joystick deadman and steering/throttle work at low speed.

Important transform check:
- The laser-to-base transform must use the measured LiDAR offset:
  - `x = 0.295 m`
  - `z = 0.165 m`
- If the stack still has default sensor offsets, fix that before mapping.

## 2. LiDAR Scan-Plane Tilt Check

A 2D LiDAR scans one thin plane. If the plane tilts down, the scan can hit the floor
and create phantom walls. If it tilts up too much, short obstacles disappear at range.

Quick check:

1. Put the car on the floor facing the longest clear sightline.
2. Measure or estimate the real distance to the far wall.
3. View the straight-ahead scan range in RViz or with `/scan`.
4. Rotate the car 90 degrees and repeat to catch roll tilt.

Pass criteria:
- Straight-ahead range matches the real wall distance, or reaches max range if the wall
  is beyond the sensor range.
- No consistent phantom return appears closer than the real wall.
- If the LiDAR tilts upward, keep it near 2 degrees or less for 30 cm obstacles.

Useful rule:

```text
down-tilt angle ~= atan(0.165 / phantom_floor_distance)
up-tilt angle ~= atan((obstacle_height - 0.165) / obstacle_dropout_distance)
```

If it is clearly nose-up by more than about 3 degrees, shim the rear edge of the mount.
If it is nose-down, shim the front edge until the scan plane is closer to level.

## 3. Odometry Repeatability

Repeat the straight-line odometry check 2-3 times before mapping. SLAM can absorb some
drift, but a bad odom scale makes loop closure and localization harder.

Recommended command:

```bash
python3 ~/RC/phase1_sysid/scripts/odometry_test.py \
    --duration 8 --odom-topic /odom
```

Record for each run:

```text
run_id:
physical_distance_m:
odom_distance_m:
correction_factor = physical_distance_m / odom_distance_m
notes: straight / veered / slipped / obstacle / aborted
```

Pass criteria:
- Correction factors are close to the saved value, currently about `1.035` from the
  earlier calibration context.
- Reject runs where the car visibly veered, slipped, or hit an obstacle.

## 4. Mapping Run

Launch `slam_toolbox` in online async mapping mode with:

```text
odom_frame: odom
base_frame: base_link
```

Drive rules:
- Keep speed at or below 1 m/s.
- Use smooth, wide turns.
- Drive the room perimeter first.
- Cross the middle of the room after the perimeter is established.
- Return to the exact starting marker at the end for loop-closure validation.

Save both map artifacts:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/room_empty
```

Also save the `slam_toolbox` serialized pose graph using its normal serialization
service or RViz plugin. Keep both the occupancy map and pose graph; the pose graph is
needed for better localization-mode reloads later.

## 5. Map Validation Gates

Loop closure:
- At the final return to the start marker, walls in RViz should line up with the
  existing map.
- Fail condition: doubled walls, ghost walls, or a visibly shifted room outline.

Metric accuracy:
- Tape-measure one wall-to-wall span.
- Measure the same span in RViz.
- Pass target: error is 5 cm or less.

Coverage:
- Major walls and fixed room features are present.
- No phantom wall appears across open floor.
- No large unknown gap remains in the area where the car will test.

## 6. Localization Rehearsal

Restart the stack in localization mode against the saved map.

Test at least 3 known placements:

```text
placement_id:
true_location_description:
estimated_error_cm:
converged: yes/no
notes:
```

Pass criteria:
- Estimate converges from each placement.
- Pose error is small enough for safe testing in the room.
- Estimate remains stable when the car is stationary.

## 7. Obstacle Preview

Place 2-3 boxes in the mapped room without remapping. This rehearses the future test
workflow: known static room map, new unmapped obstacles, localization still works.

Check:
- The car relocalizes against the room despite the new boxes.
- RViz scan points show the boxes as live obstacles.
- The pose estimate does not jump or drift badly near the boxes.

Pass criteria:
- Localization still converges.
- No persistent pose instability appears when boxes are in view.
- Obstacles are visible in `/scan` at useful distances.

## 8. Remaining Optional Tests

BNO086 wiring and calibration:
- Needed for the production EKF later.
- Not required for the first SLAM mapping pass if LiDAR and wheel odom are healthy.
- Target update rate is at least 100 Hz.

Coast-down friction:
- Needs about 5-6 m of clear floor.
- Run after mapping if time and space allow.

Weight:
- Weigh the complete car when a scale is available.

Policy-to-motor latency:
- Save for Phase 5 deployment.

Servo deadtime:
- Low priority because domain randomization already covers it.

## Field Notes Template

```text
date:
room:
map_name:
lidar_tilt_check:
odom_runs:
mapping_duration:
loop_closure_result:
metric_span_true_m:
metric_span_rviz_m:
localization_placement_errors_cm:
obstacle_preview_result:
issues:
next_action:
```

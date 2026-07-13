# School SLAM run — 2026-07-13

## Outcome

Artifacts were saved successfully, but the map **failed visual quality
validation**. RViz showed substantial ghosting, duplicated boundaries, and
radial scan streaks. Do not use this map for localization or policy testing.

## Run summary

- Mapping duration: 225.21 s
- Map resolution: 0.05 m/pixel
- Map dimensions: 447 x 299 pixels
- LiDAR messages: 8,988
- Odometry messages: 1,025
- BNO086 IMU messages: 22,297
- Joystick messages: 7,506
- Recorded messages: 57,323
- Bag size: 51.8 MiB

## Jetson artifacts

- `/home/dat/maps/school_room_2026-07-13.pgm`
- `/home/dat/maps/school_room_2026-07-13.yaml`
- `/home/dat/maps/school_room_2026-07-13.posegraph`
- `/home/dat/maps/school_room_2026-07-13.data`
- `/home/dat/bags/school_room_mapping_direction_fixed_2026-07-13/`

## Changes validated during the run

- Correct LiDAR transform: base_link to laser, x=0.295 m and z=0.165 m.
- Joystick speed limited to 1.0 m/s.
- 8BitDo forward-axis sign corrected (`scale: -1.0`).
- VESC odometry TF parameter-loading bug fixed and rebuilt.

## Next action

Diagnose the ghosting from the saved bag before repeating. Prioritize TF/odom
timing and LiDAR mounting/scan-plane stability; only accept the next map if
walls remain single and aligned after returning to the start.

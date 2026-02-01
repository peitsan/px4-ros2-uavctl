# SpireCV ArUco Tracker (PX4 Real Robot)

## Overview
This node uses **SpireCV** to detect ArUco markers from the **D435i RGB stream** and commands PX4 Offboard **velocity setpoints** to follow the target in the **forward/left/right** directions while maintaining fixed altitude.

## Topics
- RGB: `/camera/d435i/color/image_raw`
- Depth: `/camera/d435i/depth/image_rect_raw`

## Run
```bash
source /opt/ros/humble/setup.bash
source ~/Desktop/px4-ros2-uavctl/install/setup.bash
ros2 run px4_hexctl qr_tracker_spirecv \
  --ros-args \
  -p target_id:=1 \
  -p takeoff_alt:=1.5 \
  -p tracking_delta_x:=2.5 \
  -p tracking_delta_y:=0.0 \
  -p tracking_delta_z:=0.0 \
  -p track_z:=false
```

## Key Parameters
- `target_id`: ArUco ID to track
- `takeoff_alt`: takeoff/hold altitude (meters)
- `tracking_delta_x/y/z`: desired relative offset in body frame
- `camera_offset_x/y/z`: camera offset from body center
- `track_z`: set true to allow vertical tracking
- `kpx_track/kpy_track/kpz_track`: P gains
- `spirecv_calib` / `spirecv_algo`: SpireCV config files (optional)

## Notes
- Control uses body-frame target converted to velocity in **ENU** (assumes yaw aligned). If you need yaw compensation, add body-to-ENU conversion using attitude.

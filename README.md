# Stationary VLP-16 Dynamic 3D Map (ROS 2 Jazzy)

## Behavior
- Subscribes to `/points` (`sensor_msgs/msg/PointCloud2`).
- Learns a static voxel background for 8 seconds.
- Publishes static, dynamic, and combined point clouds.
- Dynamic candidates expire after 0.8 seconds without being seen.

## Build
Place `lidar_dynamic_map` inside your ROS 2 workspace `src` folder, then:

```bash
cd ~/your_ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select lidar_dynamic_map
source install/setup.bash
ros2 run lidar_dynamic_map lidar_mapping
```

If the topic is different:
```bash
ros2 run lidar_dynamic_map lidar_mapping --ros-args -p input_topic:=/your_points_topic
```

## View in RViz2
- Set Fixed Frame to the frame in `/points` message header.
- Add PointCloud2 `/lidar_map/static` and `/lidar_map/dynamic`.
- Set static to a small point size and dynamic to a larger point size.
- Optionally add `/lidar_map/combined`.

## Limitations
This is a baseline change detector, not a robust human detector. During the first 8 seconds, keep the room still so the static background can be learned. Dynamic objects that were present during learning may be treated as background. Occlusion and sparse VLP-16 vertical sampling can cause missed detections. Set `min_z` and `max_z` for the sensor's coordinate frame. The z limits are relative to the sensor frame unless the input points have already been transformed.

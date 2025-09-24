# Agilex Piper Utils

Simple utility for converting PoseStamped messages to Agilex Piper GPIO controller commands.

## pose_to_gpio_converter

Converts `geometry_msgs/msg/PoseStamped` to `control_msgs/msg/DynamicInterfaceGroupValues` for native Cartesian control.

## Usage

```bash
# Run the converter
ros2 run agilex_piper_utils pose_to_gpio_converter

# Send poses
ros2 topic pub /target_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'base_link'}, pose: {position: {x: 0.2, y: 0.0, z: 0.2}, orientation: {w: 1.0}}}"

# With custom parameters
ros2 run agilex_piper_utils pose_to_gpio_converter --ros-args -p input_topic:=/my_poses
```


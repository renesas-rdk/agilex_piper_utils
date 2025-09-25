# Agilex Piper Utils

Utility nodes for Agilex Piper arm message conversion and pose transformations.

## pose_to_gpio_converter

Converts `geometry_msgs/msg/PoseStamped` to `control_msgs/msg/DynamicInterfaceGroupValues` for native Cartesian control.

## pose_link_projector

Projects poses between different links in the robot's kinematic chain using TF transforms.

Given a `PoseStamped` of a **source_link** expressed in a **base frame**, publishes the **target_link** pose in the same base frame by composing with the TF transform.

### Parameters

- `base_frame` (string, default: `base_link`) - Base reference frame
- `source_link` (string, **required**) - Source link frame
- `target_link` (string, **required**) - Target link frame
- `lookup_timeout` (double, default: `0.05`) - TF lookup timeout in seconds
- `accept_mismatched_input_frame` (bool, default: `true`) - Accept input poses in frames other than base_frame

### Topics

- `~/in/pose` (geometry_msgs/PoseStamped) - Input pose of source_link
- `~/out/pose` (geometry_msgs/PoseStamped) - Output pose of target_link

## Usage

```bash
# Run the GPIO converter
ros2 run agilex_piper_utils pose_to_gpio_converter

# Send poses to converter
ros2 topic pub /target_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'base_link'}, pose: {position: {x: 0.2, y: 0.0, z: 0.2}, orientation: {w: 1.0}}}"

# Run the pose link projector
ros2 run agilex_piper_utils pose_link_projector --ros-args -p source_link:=link_a -p target_link:=tool0

# Send poses to projector
ros2 topic pub /pose_link_projector/in/pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'base_link'}, pose: {position: {x: 0.2, y: 0.0, z: 0.2}, orientation: {w: 1.0}}}"

# Monitor output
ros2 topic echo /pose_link_projector/out/pose
```

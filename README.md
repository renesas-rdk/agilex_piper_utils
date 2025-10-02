# Agilex Piper Utils

Utility nodes for Agilex Piper arm message conversion and pose transformations.

## gripper_action_adapter

Provides a `control_msgs/action/ParallelGripperCommand` action server interface that converts gripper commands to position commands for the `JointGroupPositionController`. This allows standard gripper action clients to work with the new position-based gripper controller.

### Features
- Action server compatible with standard gripper action clients
- Topic subscriber for simple gripper commands (no action feedback)
- Converts parallel gripper commands to individual joint positions
- Mock feedback system with linear interpolation for reliable operation
- Configurable execution duration for different gripper speeds
- Simple and reliable architecture without complex joint state monitoring

### Parameters
- `action_server_name` (string, default: `gripper_cmd`) - Name of the action server
- `gripper_command_topic` (string, default: `gripper_command`) - Topic for simple gripper commands
- `position_controller_topic` (string, default: `/agilex_piper_gripper_position_controller/commands`) - Topic for position controller commands
- `max_gripper_width` (double, default: `0.07`) - Maximum gripper opening width (meters)
- `execution_duration` (double, default: `2.0`) - Duration for gripper action execution (seconds)

### Topics
- `~/gripper_cmd` (control_msgs/action/ParallelGripperCommand) - Action server for gripper commands
- `~/gripper_command` (control_msgs/msg/GripperCommand) - Subscriber for simple gripper commands
- Position controller topic (std_msgs/Float64MultiArray) - Position commands to gripper controller

## pose_to_gpio_converter

Converts `geometry_msgs/msg/PoseStamped` to `control_msgs/msg/DynamicInterfaceGroupValues` for native Cartesian control.

## pose_link_projector

Projects poses between different links in the robot's kinematic chain using TF transforms.

Given a `PoseStamped` of a **source_link** expressed in a **base frame**, publishes the **target_link** pose in the same base frame by composing with the TF transform. Supports both direct `PoseStamped` input and GPIO state input (for reading poses from hardware controllers).

### Features
- Projects poses between any two links in the kinematic chain
- Supports direct PoseStamped messages
- Supports GPIO state messages (e.g., `arm_current_pose` from hardware controllers)
- Automatic conversion from xyzrpy format to quaternion
- Configurable interface group for GPIO states
- Frame transformation support

### Parameters

- `base_frame` (string, default: `base_link`) - Base reference frame
- `source_link` (string, **required**) - Source link frame
- `target_link` (string, **required**) - Target link frame
- `lookup_timeout` (double, default: `0.05`) - TF lookup timeout in seconds
- `accept_mismatched_input_frame` (bool, default: `true`) - Accept input poses in frames other than base_frame
- `interface_group` (string, default: `arm_current_pose`) - GPIO interface group name for pose state input

### Topics

- `~/in/pose` (geometry_msgs/PoseStamped) - Input pose of source_link
- `~/in/gpio_states` (control_msgs/DynamicInterfaceGroupValues) - GPIO state input (xyzrpy format)
- `~/out/pose` (geometry_msgs/PoseStamped) - Output pose of target_link

## Usage

### Gripper Action Adapter

```bash
# Run the gripper action adapter with default parameters
ros2 run agilex_piper_utils gripper_action_adapter

# Run with custom parameters
ros2 run agilex_piper_utils gripper_action_adapter --ros-args \
  -p action_server_name:=my_gripper_cmd \
  -p gripper_command_topic:=my_gripper_command \
  -p max_gripper_width:=0.08 \
  -p execution_duration:=3.0

# Send gripper commands via action (with feedback)
ros2 action send_goal /gripper_cmd control_msgs/action/ParallelGripperCommand "{command: {position: [0.05], max_effort: [10.0]}}"

# Send simple gripper commands via topic (no feedback)
ros2 topic pub /gripper_command control_msgs/msg/GripperCommand "{position: 0.05, max_effort: 10.0}"

# Close gripper via topic
ros2 topic pub /gripper_command control_msgs/msg/GripperCommand "{position: 0.0, max_effort: 5.0}"
```

### Other Utilities

```bash
# Run the GPIO converter
ros2 run agilex_piper_utils pose_to_gpio_converter

# Send poses to converter
ros2 topic pub /target_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'base_link'}, pose: {position: {x: 0.2, y: 0.0, z: 0.2}, orientation: {w: 1.0}}}"

# Run the pose link projector (projects tcp pose to tool0)
ros2 run agilex_piper_utils pose_link_projector --ros-args \
  -p source_link:=tcp \
  -p target_link:=tool0

# Send poses to projector via PoseStamped
ros2 topic pub /pose_link_projector/in/pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'base_link'}, pose: {position: {x: 0.2, y: 0.0, z: 0.2}, orientation: {w: 1.0}}}"

# Connect to GPIO controller states (arm_current_pose will be automatically projected)
ros2 run agilex_piper_utils pose_link_projector --ros-args \
  -p source_link:=tcp \
  -p target_link:=tool0 \
  -p interface_group:=arm_current_pose \
  --remap ~/in/gpio_states:=/agilex_piper_gpio_controller/gpio_states

# Monitor output
ros2 topic echo /pose_link_projector/out/pose
```

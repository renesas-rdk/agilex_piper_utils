cmake_minimum_required(VERSION 3.8)
project(agilex_piper_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# C++ executables
add_executable(pose_to_gpio_converter
  src/pose_to_gpio_converter.cpp
)

ament_target_dependencies(pose_to_gpio_converter
  rclcpp
  geometry_msgs
  control_msgs
  tf2
  tf2_geometry_msgs
)

add_executable(pose_link_projector
  src/pose_link_projector.cpp
)

ament_target_dependencies(pose_link_projector
  rclcpp
  geometry_msgs
  control_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(gripper_action_adapter
  src/gripper_action_adapter.cpp
)

ament_target_dependencies(gripper_action_adapter
  rclcpp
  rclcpp_action
  control_msgs
  sensor_msgs
  std_msgs
)

# Install C++ executables
install(TARGETS
  pose_to_gpio_converter
  pose_link_projector
  gripper_action_adapter
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
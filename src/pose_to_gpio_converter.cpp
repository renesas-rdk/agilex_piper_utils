// *********************************************************************************************************************
// Copyright [2025] Renesas Electronics Corporation and/or its licensors. All Rights Reserved.
//
// The contents of this file (the "contents") are proprietary and confidential to Renesas Electronics Corporation
// and/or its licensors ("Renesas") and subject to statutory and contractual protections.
//
// Unless otherwise expressly agreed in writing between Renesas and you: 1) you may not use, copy, modify, distribute,
// display, or perform the contents; 2) you may not use any name or mark of Renesas for advertising or publicity
// purposes or in connection with your use of the contents; 3) RENESAS MAKES NO WARRANTY OR REPRESENTATIONS ABOUT THE
// SUITABILITY OF THE CONTENTS FOR ANY PURPOSE; THE CONTENTS ARE PROVIDED "AS IS" WITHOUT ANY EXPRESS OR IMPLIED
// WARRANTY, INCLUDING THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
// NON-INFRINGEMENT; AND 4) RENESAS SHALL NOT BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, OR CONSEQUENTIAL DAMAGES,
// INCLUDING DAMAGES RESULTING FROM LOSS OF USE, DATA, OR PROJECTS, WHETHER IN AN ACTION OF CONTRACT OR TORT, ARISING
// OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THE CONTENTS. Third-party contents included in this file may
// be subject to different terms.
// *********************************************************************************************************************

// ROS2 node for converting geometry_msgs/PoseStamped to control_msgs/DynamicInterfaceGroupValues
//
// This node subscribes to geometry_msgs/msg/PoseStamped messages and converts them to
// control_msgs/msg/DynamicInterfaceGroupValues format compatible with the Agilex Piper
// GPIO controller for native Cartesian control.
//
// The conversion extracts position (x, y, z) and orientation (as Euler angles rx, ry, rz)
// from the pose and formats them for the arm_target_pose interface group.

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <control_msgs/msg/dynamic_interface_group_values.hpp>
#include <control_msgs/msg/interface_value.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace agilex_piper_utils
{

class PoseToGpioConverter : public rclcpp::Node
{
public:
  PoseToGpioConverter() : Node("pose_to_gpio_converter")
  {
    // Declare parameters
    this->declare_parameter("input_topic", "target_pose");
    this->declare_parameter("output_topic", "gpio_commands");
    this->declare_parameter("target_frame", "base_link");
    this->declare_parameter("interface_group", "arm_target_pose");

    // Get parameters
    input_topic_ = this->get_parameter("input_topic").as_string();
    output_topic_ = this->get_parameter("output_topic").as_string();
    target_frame_ = this->get_parameter("target_frame").as_string();
    interface_group_ = this->get_parameter("interface_group").as_string();

    // Create subscriber and publisher
    pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      input_topic_, 10,
      std::bind(&PoseToGpioConverter::pose_callback, this, std::placeholders::_1));

    gpio_publisher_ =
      this->create_publisher<control_msgs::msg::DynamicInterfaceGroupValues>(output_topic_, 10);

    RCLCPP_INFO(
      this->get_logger(),
      "Pose to GPIO converter started:\n"
      "  Input topic: %s\n"
      "  Output topic: %s\n"
      "  Target frame: %s\n"
      "  Interface group: %s",
      input_topic_.c_str(), output_topic_.c_str(), target_frame_.c_str(), interface_group_.c_str());
  }

private:
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // Create GPIO message
    auto gpio_msg = control_msgs::msg::DynamicInterfaceGroupValues();

    // Extract position
    double x = msg->pose.position.x;
    double y = msg->pose.position.y;
    double z = msg->pose.position.z;

    // Convert quaternion to Euler angles (roll, pitch, yaw)
    tf2::Quaternion quat;
    tf2::fromMsg(msg->pose.orientation, quat);

    tf2::Matrix3x3 matrix(quat);
    double roll, pitch, yaw;
    matrix.getRPY(roll, pitch, yaw);

    // Fill the GPIO message
    gpio_msg.interface_groups = {interface_group_};

    control_msgs::msg::InterfaceValue interface_values;
    interface_values.interface_names = {"x", "y", "z", "rx", "ry", "rz"};
    interface_values.values = {x, y, z, roll, pitch, yaw};

    gpio_msg.interface_values = {interface_values};

    // Publish the converted message
    gpio_publisher_->publish(gpio_msg);
  }

  // ROS2 components
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
  rclcpp::Publisher<control_msgs::msg::DynamicInterfaceGroupValues>::SharedPtr gpio_publisher_;

  // Parameters
  std::string input_topic_;
  std::string output_topic_;
  std::string target_frame_;
  std::string interface_group_;
};

}  // namespace agilex_piper_utils

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<agilex_piper_utils::PoseToGpioConverter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
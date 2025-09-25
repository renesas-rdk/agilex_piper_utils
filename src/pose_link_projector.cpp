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

// ROS2 node for projecting poses between different links in a robot's kinematic chain
//
// This node subscribes to geometry_msgs/msg/PoseStamped messages containing the pose of a
// source link and publishes the corresponding pose of a target link in the same base frame
// by composing with TF transforms from the robot's kinematic tree.
//
// Given a pose of source_link (A) expressed in base_frame, it computes and publishes the
// pose of target_link (B) in the same base_frame using the transform composition:
// base_T_target = base_T_source * source_T_target
//
// This is useful for applications like tool pose calculation, sensor frame transformations,
// and motion planning where you need to know the pose of one link given another's pose.

#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class PoseLinkProjector : public rclcpp::Node
{
public:
  PoseLinkProjector()
  : Node("pose_link_projector"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    // Declare parameters
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
    source_link_ = this->declare_parameter<std::string>("source_link", "");
    target_link_ = this->declare_parameter<std::string>("target_link", "");
    lookup_timeout_ = this->declare_parameter<double>("lookup_timeout", 0.05);
    accept_mismatched_input_frame_ =
      this->declare_parameter<bool>("accept_mismatched_input_frame", true);

    // Validate required parameters
    if (source_link_.empty() || target_link_.empty()) {
      RCLCPP_FATAL(get_logger(), "source_link and target_link parameters are required.");
      rclcpp::shutdown();
      return;
    }

    // Setup QoS
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    // Create subscriber and publisher
    pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/in/pose", qos, std::bind(&PoseLinkProjector::pose_callback, this, std::placeholders::_1));

    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/out/pose", qos);

    RCLCPP_INFO(
      get_logger(),
      "Pose link projector initialized:\n"
      "  base_frame: %s\n"
      "  source_link: %s\n"
      "  target_link: %s\n"
      "  lookup_timeout: %.3f s\n"
      "  accept_mismatched_input_frame: %s",
      base_frame_.c_str(), source_link_.c_str(), target_link_.c_str(), lookup_timeout_,
      accept_mismatched_input_frame_ ? "true" : "false");
  }

private:
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // Step 1: Ensure pose is in base_frame
    geometry_msgs::msg::PoseStamped pose_in_base_frame;
    if (msg->header.frame_id != base_frame_) {
      if (!accept_mismatched_input_frame_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Input frame '%s' != base_frame '%s' (dropping message).", msg->header.frame_id.c_str(),
          base_frame_.c_str());
        return;
      }

      // Transform pose to base_frame
      try {
        auto transform_stamped = tf_buffer_.lookupTransform(
          base_frame_, msg->header.frame_id, msg->header.stamp,
          tf2::durationFromSec(lookup_timeout_));
        tf2::doTransform(*msg, pose_in_base_frame, transform_stamped);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "Failed to transform pose to base_frame: %s",
          ex.what());
        return;
      }
    } else {
      pose_in_base_frame = *msg;
    }

    // Step 2: Lookup transform from source_link to target_link
    geometry_msgs::msg::TransformStamped source_to_target_transform;
    try {
      if (!tf_buffer_.canTransform(
            source_link_, target_link_, pose_in_base_frame.header.stamp,
            tf2::durationFromSec(lookup_timeout_))) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "No TF transform available from %s to %s at time %.3f",
          source_link_.c_str(), target_link_.c_str(),
          rclcpp::Time(pose_in_base_frame.header.stamp).seconds());
        return;
      }

      source_to_target_transform = tf_buffer_.lookupTransform(
        source_link_, target_link_, pose_in_base_frame.header.stamp,
        tf2::durationFromSec(lookup_timeout_));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Failed to lookup transform %s->%s: %s",
        source_link_.c_str(), target_link_.c_str(), ex.what());
      return;
    }

    // Step 3: Compose transforms
    // base_T_target = base_T_source * source_T_target
    tf2::Transform base_T_source, source_T_target;
    tf2::fromMsg(pose_in_base_frame.pose, base_T_source);
    tf2::fromMsg(source_to_target_transform.transform, source_T_target);
    tf2::Transform base_T_target = base_T_source * source_T_target;

    // Step 4: Publish result
    geometry_msgs::msg::PoseStamped output_pose;
    output_pose.header.stamp = pose_in_base_frame.header.stamp;
    output_pose.header.frame_id = base_frame_;
    tf2::toMsg(base_T_target, output_pose.pose);

    pose_publisher_->publish(output_pose);
  }

  // TF components
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // ROS2 components
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;

  // Parameters
  std::string base_frame_;
  std::string source_link_;
  std::string target_link_;
  double lookup_timeout_;
  bool accept_mismatched_input_frame_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PoseLinkProjector>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
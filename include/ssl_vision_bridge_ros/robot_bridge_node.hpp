// Copyright 2025 TRAPS
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SSL_VISION_BRIDGE_ROS__ROBOT_BRIDGE_NODE_HPP_
#define SSL_VISION_BRIDGE_ROS__ROBOT_BRIDGE_NODE_HPP_

#include <deque>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "./messages_robocup_ssl_detection.pb.h"
#include "fmt/format.h"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/node.hpp"
#include "ssl_vision_bridge_ros/robot_to_msg.hpp"
#include "ssl_vision_bridge_ros/visibility.hpp"

namespace ssl_vision_bridge_ros
{

class RobotBridgeNode : public rclcpp::Node
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(RobotBridgeNode)

  static constexpr auto kDefaultNodeName = "ssl_vision_robot_bridge";

  SSL_VISION_BRIDGE_ROS_PUBLIC
  inline RobotBridgeNode(
    const std::string & node_name, const std::string & node_namespace,
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : rclcpp::Node(node_name, node_namespace, node_options)
  {
    this->init();
  }

  SSL_VISION_BRIDGE_ROS_PUBLIC
  explicit inline RobotBridgeNode(
    const std::string & node_name, const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : RobotBridgeNode(node_name, "", node_options)
  {
  }

  SSL_VISION_BRIDGE_ROS_PUBLIC
  explicit inline RobotBridgeNode(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : RobotBridgeNode(kDefaultNodeName, "", node_options)
  {
  }

  SSL_VISION_BRIDGE_ROS_PUBLIC
  inline RobotBridgeNode(rclcpp::Node & node, const std::string & sub_namespace)
  : rclcpp::Node(node, sub_namespace)
  {
    this->init();
  }

  void bridge(const SSL_DetectionRobot ssl_detection_robot)
  {
    auto pose_msg = std::make_unique<PoseMsg>();
    pose_msg->header.stamp = this->now();
    pose_msg->header.frame_id = frame_id_;
    pose_msg->pose = to_msg(ssl_detection_robot);
    vision_pose_pub_->publish(std::move(pose_msg));
  }

private:
  using PoseMsg = geometry_msgs::msg::PoseWithCovarianceStamped;

  inline void init()
  {
    // qos overriding options
    const auto qos_overriding_options = rclcpp::QosOverridingOptions{
      rclcpp::QosPolicyKind::Depth, rclcpp::QosPolicyKind::Durability,
      rclcpp::QosPolicyKind::History, rclcpp::QosPolicyKind::Reliability};

    const auto default_qos = rclcpp::QoS(1).best_effort().durability_volatile();

    // create publisher
    {
      rclcpp::PublisherOptions options;
      options.qos_overriding_options = qos_overriding_options;

      vision_pose_pub_ = this->create_publisher<PoseMsg>("vision/pose", default_qos, options);
    }

    // declare parameter
    frame_id_ = this->declare_parameter(
      this->get_sub_namespace() + ".frame_id",
      this->get_parameter_or<std::string>("frame_id", "map"));
  }

  // parameters
  std::string frame_id_;

  // pub/sub
  rclcpp::Publisher<PoseMsg>::SharedPtr vision_pose_pub_;
};

}  // namespace ssl_vision_bridge_ros

#endif  // SSL_VISION_BRIDGE_ROS__ROBOT_BRIDGE_NODE_HPP_

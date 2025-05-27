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

#ifndef SSL_VISION_BRIDGE_ROS__EXAMPLE_NODE_HPP_
#define SSL_VISION_BRIDGE_ROS__EXAMPLE_NODE_HPP_

#include <deque>
#include <string>
#include <utility>
#include <vector>

#include "fmt/format.h"
#include "rclcpp/node.hpp"
#include "ssl_vision_bridge_ros/visibility.hpp"

namespace ssl_vision_bridge_ros
{

class BridgeNode : public rclcpp::Node
{
public:
  static constexpr auto kDefaultNodeName = "ssl_vision_bridge";

  // SSL_VISION_BRIDGE_ROS_PUBLIC
  inline BridgeNode(
    const std::string & node_name, const std::string & node_namespace,
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : rclcpp::Node(node_name, node_namespace, node_options)
  {
    // qos overriding options
    const auto qos_overriding_options = rclcpp::QosOverridingOptions{
      rclcpp::QosPolicyKind::Depth, rclcpp::QosPolicyKind::Durability,
      rclcpp::QosPolicyKind::History, rclcpp::QosPolicyKind::Reliability};

    // create publisher
    {
      rclcpp::PublisherOptions options;
      options.qos_overriding_options = qos_overriding_options;
    }

    // declare parameter
    this->declare_parameter("string", "node default");
  }

  // SSL_VISION_BRIDGE_ROS_PUBLIC
  explicit inline BridgeNode(
    const std::string & node_name, const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : BridgeNode(node_name, "", node_options)
  {
  }

  // SSL_VISION_BRIDGE_ROS_PUBLIC
  explicit inline BridgeNode(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : BridgeNode(kDefaultNodeName, "", node_options)
  {
  }

private:
  void bridge()
  {
  }

  // pub/sub
  // rclcpp::Publisher<
};

}  // namespace ssl_vision_bridge_ros

#endif  // SSL_VISION_BRIDGE_ROS__EXAMPLE_NODE_HPP_

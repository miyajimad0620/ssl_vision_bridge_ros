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

#ifndef SSL_VISION_BRIDGE_ROS__WRAPPER_BRIDGE_NODE_HPP_
#define SSL_VISION_BRIDGE_ROS__WRAPPER_BRIDGE_NODE_HPP_

#include <deque>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "./messages_robocup_ssl_wrapper.pb.h"
#include "asio/io_context.hpp"
#include "asio/ip/multicast.hpp"
#include "asio/ip/udp.hpp"
#include "ball_to_msg.hpp"
#include "field_to_msg.hpp"
#include "fmt/format.h"
#include "google/protobuf/message.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/node.hpp"
#include "ssl_vision_bridge_ros/robot_bridge_node.hpp"
#include "ssl_vision_bridge_ros/visibility.hpp"

namespace ssl_vision_bridge_ros
{

class WrapperBridgeNode : public rclcpp::Node
{
public:
  static constexpr auto kDefaultNodeName = "ssl_vision_wrapper_bridge";

  // SSL_VISION_BRIDGE_ROS_PUBLIC
  inline WrapperBridgeNode(
    const std::string & node_name, const std::string & node_namespace,
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : rclcpp::Node(node_name, node_namespace, node_options),
    frame_id_(this->declare_parameter("frame_id", "map"))
  {
    // qos overriding options
    const auto qos_overriding_options = rclcpp::QosOverridingOptions{
      rclcpp::QosPolicyKind::Depth, rclcpp::QosPolicyKind::Durability,
      rclcpp::QosPolicyKind::History, rclcpp::QosPolicyKind::Reliability};

    // create publisher
    {
      rclcpp::PublisherOptions options;
      options.qos_overriding_options = qos_overriding_options;
      map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "map", rclcpp::QoS(1).transient_local(), options);
      vision_ball_pub_ =
        this->create_publisher<PoseMsg>("vision/ball", rclcpp::QoS(1).best_effort(), options);
    }

    // create robot bridge nodes
    std::pair<std::string, std::array<RobotBridgeNode::SharedPtr, kRobotMaxCount> &>
    color_str_robot_bridge_node_pairs[] = {
      {"yellow", yellow_robot_bridge_nodes_}, {"blue", blue_robot_bridge_nodes_}};
    for (auto & [color_str, robot_bridge_nodes_] : color_str_robot_bridge_node_pairs) {
      const auto robot_ids = this->declare_parameter(
        color_str + "_robot_ids", std::vector<int>({0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10}));
      const auto robot_name_base =
        this->declare_parameter(color_str + "_robot_name", color_str + "{:02}");
      for (const auto robot_id : robot_ids) {
        if (robot_id < 0 || robot_id >= kRobotMaxCount) {
          RCLCPP_ERROR(
            this->get_logger(), "%s_robot_id must be in the range [0, %d).", color_str.c_str(),
            kRobotMaxCount);
          RCLCPP_ERROR(
            this->get_logger(), "%s_robot_id:%ld will be ignored.", color_str.c_str(), robot_id);
          continue;
        }
        auto robot_bridge_node = robot_bridge_nodes_[robot_id];
        if (robot_bridge_node) {
          RCLCPP_WARN(
            this->get_logger(), "%s_robot_id:%ld is defined multiple times", color_str.c_str(),
            robot_id);
          continue;
        }
        std::string robot_name;
        try {
          robot_name = fmt::format(robot_name_base, robot_id);
        } catch (const std::exception & e) {
          RCLCPP_ERROR(
            this->get_logger(), "invalid %s_robot:%ld name format: %s", color_str.c_str(), robot_id,
            e.what());
          continue;
        }
        robot_bridge_nodes_[robot_id] = std::make_shared<RobotBridgeNode>(*this, robot_name);
      }
    }
  }

  // SSL_VISION_BRIDGE_ROS_PUBLIC
  explicit inline WrapperBridgeNode(
    const std::string & node_name, const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : WrapperBridgeNode(node_name, "", node_options)
  {
  }

  // SSL_VISION_BRIDGE_ROS_PUBLIC
  explicit inline WrapperBridgeNode(
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : WrapperBridgeNode(kDefaultNodeName, "", node_options)
  {
  }

  void bridge(const std::uint8_t * const data, std::size_t size)
  {
    SSL_WrapperPacket ssl_wrapper_packet;
    if (!ssl_wrapper_packet.ParseFromArray(data, size)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to parse SSL_WrapperPacket");
      return;
    }

    // bridge detection
    {
      const auto & ssl_detection = ssl_wrapper_packet.detection();
      auto ssl_robots_robot_bridge_nodes_pairs = {
        std::tie(ssl_detection.robots_yellow(), yellow_robot_bridge_nodes_),
        std::tie(ssl_detection.robots_blue(), blue_robot_bridge_nodes_),
      };
      for (auto & [ssl_robots, robot_bridge_nodes_] : ssl_robots_robot_bridge_nodes_pairs) {
        for (const auto & ssl_robot : ssl_robots) {
          const auto robot_id = ssl_robot.robot_id();
          if (robot_id >= kRobotMaxCount) {
            RCLCPP_ERROR(
              this->get_logger(), "robot_id:%u must be in the range [0, %d).", robot_id,
              kRobotMaxCount);
            RCLCPP_ERROR(this->get_logger(), "received robot_id:%u will be ignored.", robot_id);
            continue;
          }
          auto & robot_bridge_node = robot_bridge_nodes_[robot_id];
          if (!robot_bridge_node) {
            continue;
          }
          robot_bridge_node->bridge(ssl_robot);
        }
      }
      for (const auto & ssl_ball : ssl_detection.balls()) {
        auto ball_msg = std::make_unique<PoseMsg>(rosidl_runtime_cpp::MessageInitialization::SKIP);
        ball_msg->header.stamp = this->now();
        ball_msg->header.frame_id = frame_id_;
        ball_msg->pose = to_msg(ssl_ball);
        vision_ball_pub_->publish(std::move(ball_msg));
      }
    }

    // bridge geometry
    {
      const auto & ssl_geometry = ssl_wrapper_packet.geometry();
      // if field updated
      if (ssl_geometry.has_field() && !is_same(ssl_field_, ssl_geometry.field())) {
        auto map_msg = std::make_unique<MapMsg>(
          to_msg<MapMsg>(frame_id_, this->now(), ssl_geometry.field(), 0.01));
        map_pub_->publish(std::move(map_msg));
        ssl_field_ = ssl_geometry.field();
      }
    }
  }

  template<class Array>
  void bridge(const Array & bytes)
  {
    this->bridge(std::data(bytes), std::size(bytes));
  }

private:
  using MapMsg = nav_msgs::msg::OccupancyGrid;
  using PoseMsg = geometry_msgs::msg::PoseWithCovarianceStamped;

  static constexpr auto kRobotMaxCount = 16;

  static bool is_same(const SSL_GeometryFieldSize & lhs, const SSL_GeometryFieldSize & rhs) noexcept
  {
    return lhs.field_length() == rhs.field_length() && lhs.field_width() == rhs.field_width() &&
           lhs.boundary_width() == rhs.boundary_width() && lhs.goal_depth() == rhs.goal_depth() &&
           lhs.goal_width() == rhs.goal_width();
  }

  // params
  std::string frame_id_;

  // field
  SSL_GeometryFieldSize ssl_field_;

  // robot bridge nodes
  std::array<RobotBridgeNode::SharedPtr, kRobotMaxCount> yellow_robot_bridge_nodes_,
    blue_robot_bridge_nodes_;

  // map publisher
  rclcpp::Publisher<MapMsg>::SharedPtr map_pub_;

  // vision ball publicher
  rclcpp::Publisher<PoseMsg>::SharedPtr vision_ball_pub_;
};

}  // namespace ssl_vision_bridge_ros

#endif  // SSL_VISION_BRIDGE_ROS__WRAPPER_BRIDGE_NODE_HPP_

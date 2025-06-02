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

#ifndef SSL_VISION_BRIDGE_ROS__WRAPPER_ASYNC_BRIDGE_NODE_HPP_
#define SSL_VISION_BRIDGE_ROS__WRAPPER_ASYNC_BRIDGE_NODE_HPP_

#include <deque>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "./messages_robocup_ssl_wrapper.pb.h"
#include "asio/io_context.hpp"
#include "asio/ip/multicast.hpp"
#include "asio/ip/udp.hpp"
#include "field_to_msg.hpp"
#include "fmt/format.h"
#include "google/protobuf/message.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/node.hpp"
#include "ssl_vision_bridge_ros/robot_bridge_node.hpp"
#include "ssl_vision_bridge_ros/visibility.hpp"
#include "ssl_vision_bridge_ros/wrapper_bridge_node.hpp"

namespace ssl_vision_bridge_ros
{

class WrapperAsyncBridgeNode : public WrapperBridgeNode
{
public:
  static constexpr auto kDefaultNodeName = "ssl_vision_bridge";

  // SSL_VISION_BRIDGE_ROS_PUBLIC
  inline WrapperAsyncBridgeNode(
    const std::string & node_name, const std::string & node_namespace,
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : WrapperBridgeNode(node_name, node_namespace, node_options),
    io_context_(),
    udp_socket_(io_context_),
    udp_buffer_(this->declare_parameter("udp_buffer_size", 65536))
  {
    // initialize udp socket
    const auto multicast_address_str = this->declare_parameter("multicast_address", "224.5.23.2");
    const auto port = this->declare_parameter("port", 10020);
    const auto multicast_address = asio::ip::make_address(multicast_address_str);
    const auto udp = multicast_address.is_v4() ? asio::ip::udp::v4() : asio::ip::udp::v6();
    udp_socket_.open(udp);
    udp_socket_.bind(asio::ip::udp::endpoint(udp, port));
    udp_socket_.set_option(asio::socket_base::reuse_address(true));
    udp_socket_.set_option(asio::ip::multicast::enable_loopback(true));
    udp_socket_.set_option(asio::ip::multicast::join_group(multicast_address));
    this->enable_async_bridge();

    // start io_context thread
    thread_ = std::thread(
      [this]() {
        try {
          io_context_.run();
        } catch (const std::exception & e) {
          RCLCPP_ERROR(this->get_logger(), "asio::io_context run error: %s", e.what());
        }
      });
  }

  // SSL_VISION_BRIDGE_ROS_PUBLIC
  explicit inline WrapperAsyncBridgeNode(
    const std::string & node_name, const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : WrapperAsyncBridgeNode(node_name, "", node_options)
  {
  }

  // SSL_VISION_BRIDGE_ROS_PUBLIC
  explicit inline WrapperAsyncBridgeNode(
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : WrapperAsyncBridgeNode(kDefaultNodeName, "", node_options)
  {
  }

  ~WrapperAsyncBridgeNode()
  {
    try {
      udp_socket_.close();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "asio::udp::socket close error: %s", e.what());
    }
    try {
      io_context_.stop();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "asio::io_context stop error: %s", e.what());
    }
    if (thread_.joinable()) {
      try {
        thread_.join();
      } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "asio::io_context thread join error: %s", e.what());
      }
    }
  }

private:
  void enable_async_bridge()
  {
    udp_socket_.async_receive(
      asio::buffer(udp_buffer_), [this](const asio::error_code & error_code, std::size_t length) {
        if (error_code) {
          RCLCPP_ERROR_ONCE(
            rclcpp::get_logger("ssl_vision_bridge"), "asio::udp::socket receive error: %s",
            error_code.message().c_str());
        } else {
          this->bridge(udp_buffer_.data(), length);
        }
        this->enable_async_bridge();
      });
  }

  // asio
  asio::io_context io_context_;
  asio::ip::udp::socket udp_socket_;
  std::vector<std::uint8_t> udp_buffer_;
  std::thread thread_;
};

}  // namespace ssl_vision_bridge_ros

#endif  // SSL_VISION_BRIDGE_ROS__WRAPPER_ASYNC_BRIDGE_NODE_HPP_

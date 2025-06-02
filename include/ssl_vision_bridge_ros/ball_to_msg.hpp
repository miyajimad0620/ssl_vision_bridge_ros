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

#ifndef SSL_VISION_BRIDGE_ROS__BALL_TO_MSG_HPP_
#define SSL_VISION_BRIDGE_ROS__BALL_TO_MSG_HPP_

#include <cmath>

#include "./messages_robocup_ssl_detection.pb.h"
#include "eigen3/Eigen/Core"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ssl_vision_bridge_ros
{

namespace
{

template<class LinearCovCalcFunc>
inline geometry_msgs::msg::PoseWithCovariance to_msg(
  const SSL_DetectionBall & ssl_ball, LinearCovCalcFunc & linear_cov_calc_func) noexcept
{
  auto pose =
    geometry_msgs::msg::PoseWithCovariance(rosidl_runtime_cpp::MessageInitialization::SKIP);
  pose.pose.position.x = ssl_ball.x() * 1e-3;
  pose.pose.position.y = ssl_ball.y() * 1e-3;
  pose.pose.position.z = ssl_ball.z() * 1e-3;
  constexpr auto kRootHarf = std::sqrt(0.5);
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = -kRootHarf;
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = kRootHarf;

  auto cov_mat = Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(pose.covariance.data());
  if (ssl_ball.has_confidence()) {
    const auto linear_cov = linear_cov_calc_func(ssl_ball.confidence());
    cov_mat = Eigen::DiagonalMatrix<double, 6>(linear_cov, linear_cov, linear_cov, 0, 0, 0);
  } else {
    cov_mat = Eigen::Matrix<double, 6, 6>::Identity() * -1;
  }
  return pose;
}

geometry_msgs::msg::PoseWithCovariance to_msg(const SSL_DetectionBall & ssl_ball) noexcept
{
  static constexpr auto cov_calc_func = [](const double confidence) {
      static constexpr auto min = 0.25;
      static constexpr auto max = 5.0;
      return max - (max - min) * confidence;
    };
  return to_msg(ssl_ball, cov_calc_func);
}

}  // namespace

}  // namespace ssl_vision_bridge_ros

#endif  // SSL_VISION_BRIDGE_ROS__BALL_TO_MSG_HPP_

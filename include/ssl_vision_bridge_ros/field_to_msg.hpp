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

#ifndef SSL_VISION_BRIDGE_ROS__FIELD_TO_MSG_HPP_
#define SSL_VISION_BRIDGE_ROS__FIELD_TO_MSG_HPP_

#include <cmath>
#include <vector>

#include "./messages_robocup_ssl_geometry.pb.h"
#include "eigen3/Eigen/Core"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ssl_vision_bridge_ros
{

namespace
{

template<class Msg>
inline Msg to_msg(
  std::string_view frame_id, const builtin_interfaces::msg::Time & stamp,
  const SSL_GeometryFieldSize & ssl_field, double resolution, double wall_width = 0.02);

template<>
inline nav_msgs::msg::OccupancyGrid to_msg<nav_msgs::msg::OccupancyGrid>(
  std::string_view frame_id, const builtin_interfaces::msg::Time & stamp,
  const SSL_GeometryFieldSize & ssl_field, double resolution, double wall_width)
{
  nav_msgs::msg::OccupancyGrid map_msg(rosidl_runtime_cpp::MessageInitialization::SKIP);
  map_msg.header.frame_id = frame_id;
  map_msg.header.stamp = stamp;

  // set grids
  {
    const auto to_cell = [resolution_inv = 1.0 / resolution](double length) {
        return static_cast<std::size_t>(std::ceil(length * resolution_inv));
      };

    const auto wall_cells_count = to_cell(wall_width);
    const auto field_x_cells_count_harf = to_cell(ssl_field.field_length() * (1e-3 * 0.5));
    const auto field_y_cells_count_harf = to_cell(ssl_field.field_width() * (1e-3 * 0.5));
    const auto field_x_cells_count = field_x_cells_count_harf * 2;
    const auto boundary_cells_count = to_cell(ssl_field.boundary_width() * 1e-3);
    const auto field_boundary_x_cells_count_harf = field_x_cells_count_harf + boundary_cells_count;
    const auto field_boundary_y_cells_count_harf = field_y_cells_count_harf + boundary_cells_count;
    const auto field_boundary_x_cells_count = field_boundary_x_cells_count_harf * 2;
    const auto field_boundary_y_cells_count = field_boundary_y_cells_count_harf * 2;
    const auto map_x_cells_count_harf =
      field_x_cells_count_harf + boundary_cells_count + wall_cells_count;
    const auto map_y_cells_count_harf =
      field_y_cells_count_harf + boundary_cells_count + wall_cells_count;
    const auto map_x_cells_count = map_x_cells_count_harf * 2;
    const auto map_y_cells_count = map_y_cells_count_harf * 2;
    const auto goal_x_cells_count = to_cell(ssl_field.goal_depth() * 1e-3);
    const auto goal_y_cells_count_harf = to_cell(ssl_field.goal_width() * (0.5 * 1e-3));
    const auto goal_y_cells_count = goal_y_cells_count_harf * 2;
    const auto goal_wall_x_cells_count = goal_x_cells_count + wall_cells_count;
    const auto map_to_left_goal_x_cell_count = boundary_cells_count - goal_x_cells_count;
    const auto map_to_right_goal_x_cell_count =
      wall_cells_count + boundary_cells_count + field_x_cells_count;
    const auto map_to_goal_upper_wall_y_cell_count =
      map_y_cells_count_harf + goal_y_cells_count_harf;
    const auto map_to_goal_lower_wall_y_cell_count =
      map_y_cells_count_harf - goal_y_cells_count_harf - wall_cells_count;
    const auto map_to_goal_side_wall_y_cell_count =
      map_y_cells_count_harf - goal_y_cells_count_harf;

    // set msg
    map_msg.data = std::vector<std::int8_t>(map_x_cells_count * map_y_cells_count, 0);
    auto grid_mat = Eigen::Map<Eigen::Matrix<std::int8_t, Eigen::Dynamic, Eigen::Dynamic>>(
      map_msg.data.data(), map_x_cells_count, map_y_cells_count);
    grid_mat
    .block(
      0, wall_cells_count + field_boundary_y_cells_count, map_x_cells_count, wall_cells_count)
    .setConstant(100);                                                           // upper wall
    grid_mat.block(0, 0, map_x_cells_count, wall_cells_count).setConstant(100);  // lower wall
    grid_mat.block(0, wall_cells_count, wall_cells_count, field_boundary_y_cells_count)
    .setConstant(100);    // left wall
    grid_mat
    .block(
      wall_cells_count + field_boundary_x_cells_count, wall_cells_count, wall_cells_count,
      field_boundary_y_cells_count)
    .setConstant(100);    // right wall
    grid_mat
    .block(
      map_to_left_goal_x_cell_count, map_to_goal_upper_wall_y_cell_count, goal_wall_x_cells_count,
      wall_cells_count)
    .setConstant(100);    // left goal upper wall
    grid_mat
    .block(
      map_to_left_goal_x_cell_count, map_to_goal_lower_wall_y_cell_count, goal_wall_x_cells_count,
      wall_cells_count)
    .setConstant(100);    // left goal lower wall
    grid_mat
    .block(
      map_to_left_goal_x_cell_count, map_to_goal_side_wall_y_cell_count, wall_cells_count,
      goal_y_cells_count)
    .setConstant(100);    // left goal left wall
    grid_mat
    .block(
      map_to_right_goal_x_cell_count, map_to_goal_upper_wall_y_cell_count,
      goal_wall_x_cells_count, wall_cells_count)
    .setConstant(100);    // right goal upper wall
    grid_mat
    .block(
      map_to_right_goal_x_cell_count, map_to_goal_lower_wall_y_cell_count,
      goal_wall_x_cells_count, wall_cells_count)
    .setConstant(100);    // right goal lower wall
    grid_mat
    .block(
      map_to_right_goal_x_cell_count + goal_x_cells_count, map_to_goal_side_wall_y_cell_count,
      wall_cells_count, goal_y_cells_count)
    .setConstant(100);    // right goal left wall

    map_msg.info.map_load_time = stamp;
    map_msg.info.resolution = resolution;
    map_msg.info.width = map_x_cells_count;
    map_msg.info.height = map_y_cells_count;
    map_msg.info.origin.position.x = -(map_x_cells_count_harf * resolution);
    map_msg.info.origin.position.y = -(map_y_cells_count_harf * resolution);
    map_msg.info.origin.position.z = 0.0;
    map_msg.info.origin.orientation.x = 0.0;
    map_msg.info.origin.orientation.y = 0.0;
    map_msg.info.origin.orientation.z = 0.0;
    map_msg.info.origin.orientation.w = 1.0;
  }

  return map_msg;
}

}  // namespace

}  // namespace ssl_vision_bridge_ros

#endif  // SSL_VISION_BRIDGE_ROS__FIELD_TO_MSG_HPP_

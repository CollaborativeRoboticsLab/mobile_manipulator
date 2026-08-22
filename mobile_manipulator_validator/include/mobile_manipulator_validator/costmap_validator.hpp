#pragma once

#include <cstddef>
#include <cstdint>

#include <geometry_msgs/msg/pose.hpp>
#include <nav2_msgs/msg/costmap.hpp>

namespace mobile_manipulator_validator::detail
{
inline bool pose_is_costmap_free(
  const nav2_msgs::msg::Costmap & costmap,
  const geometry_msgs::msg::Pose & pose,
  const uint8_t occupied_threshold)
{
  const double resolution = static_cast<double>(costmap.metadata.resolution);
  if (resolution <= 0.0)
  {
    return false;
  }

  const double origin_x = costmap.metadata.origin.position.x;
  const double origin_y = costmap.metadata.origin.position.y;
  const double cell_x = (pose.position.x - origin_x) / resolution;
  const double cell_y = (pose.position.y - origin_y) / resolution;
  if (cell_x < 0.0 || cell_y < 0.0)
  {
    return false;
  }

  const auto grid_x = static_cast<std::size_t>(cell_x);
  const auto grid_y = static_cast<std::size_t>(cell_y);
  const auto size_x = static_cast<std::size_t>(costmap.metadata.size_x);
  const auto size_y = static_cast<std::size_t>(costmap.metadata.size_y);
  if (grid_x >= size_x || grid_y >= size_y)
  {
    return false;
  }

  const auto index = grid_y * size_x + grid_x;
  if (index >= costmap.data.size())
  {
    return false;
  }

  return costmap.data[index] < occupied_threshold;
}
}  // namespace mobile_manipulator_validator::detail
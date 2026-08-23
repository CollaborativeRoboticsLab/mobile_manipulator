#include <gtest/gtest.h>

#include <geometry_msgs/msg/pose.hpp>
#include <nav2_msgs/msg/costmap.hpp>

#include <mobile_manipulator_pose_validator/costmap_validator.hpp>

TEST(CostmapValidator, ReturnsTrueForFreeCell)
{
  nav2_msgs::msg::Costmap costmap;
  costmap.metadata.resolution = 1.0F;
  costmap.metadata.size_x = 4;
  costmap.metadata.size_y = 4;
  costmap.metadata.origin.position.x = 0.0;
  costmap.metadata.origin.position.y = 0.0;
  costmap.data.assign(16, 0);

  geometry_msgs::msg::Pose pose;
  pose.position.x = 1.2;
  pose.position.y = 2.6;

  EXPECT_TRUE(mobile_manipulator_pose_validator::detail::pose_is_costmap_free(costmap, pose, 253));
}

TEST(CostmapValidator, ReturnsFalseForOccupiedCell)
{
  nav2_msgs::msg::Costmap costmap;
  costmap.metadata.resolution = 1.0F;
  costmap.metadata.size_x = 4;
  costmap.metadata.size_y = 4;
  costmap.metadata.origin.position.x = 0.0;
  costmap.metadata.origin.position.y = 0.0;
  costmap.data.assign(16, 0);
  costmap.data[9] = 254;

  geometry_msgs::msg::Pose pose;
  pose.position.x = 1.1;
  pose.position.y = 2.1;

  EXPECT_FALSE(mobile_manipulator_pose_validator::detail::pose_is_costmap_free(costmap, pose, 253));
}

TEST(CostmapValidator, ReturnsFalseForOutOfBoundsCell)
{
  nav2_msgs::msg::Costmap costmap;
  costmap.metadata.resolution = 1.0F;
  costmap.metadata.size_x = 4;
  costmap.metadata.size_y = 4;
  costmap.metadata.origin.position.x = 0.0;
  costmap.metadata.origin.position.y = 0.0;
  costmap.data.assign(16, 0);

  geometry_msgs::msg::Pose pose;
  pose.position.x = 10.0;
  pose.position.y = 10.0;

  EXPECT_FALSE(mobile_manipulator_pose_validator::detail::pose_is_costmap_free(costmap, pose, 253));
}
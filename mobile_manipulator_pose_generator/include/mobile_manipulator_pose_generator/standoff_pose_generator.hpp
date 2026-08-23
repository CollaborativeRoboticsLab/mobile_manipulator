#pragma once

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <mobile_manipulator_base/pose_generator.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace mobile_manipulator_pose_generator
{
namespace detail
{
/**
 * @brief Convert a quaternion to a yaw angle in radians.
 *
 * @param quaternion Input quaternion.
 * @return Yaw angle in radians.
 */
inline double quaternion_to_yaw(const geometry_msgs::msg::Quaternion& quaternion)
{
  tf2::Quaternion tf_quaternion(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(tf_quaternion).getRPY(roll, pitch, yaw);
  return yaw;
}

/**
 * @brief Convert a yaw angle in radians to a quaternion.
 *
 * @param yaw Input yaw angle in radians.
 * @return Output quaternion.
 */
inline geometry_msgs::msg::Quaternion quaternion_from_yaw(const double yaw)
{
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);
  geometry_msgs::msg::Quaternion out;
  out.x = quaternion.x();
  out.y = quaternion.y();
  out.z = quaternion.z();
  out.w = quaternion.w();
  return out;
}
}  // namespace detail

/**
 * @brief Default candidate generator that produces stand-off base poses around a target.
 */
class StandoffPoseGenerator : public mobile_manipulator_base::PoseGenerator
{
public:
  /**
   * @brief Configure output frame and stand-off sampling parameters.
   *
   * @param node Runtime node that owns parameters.
   * @param parameter_prefix Prefix for this plugin's parameter block.
   */
  void configure(rclcpp::Node* node, const std::string& parameter_prefix) override
  {
    node_ = node;
    parameter_prefix_ = parameter_prefix;
    output_frame_ = node_->declare_parameter<std::string>(parameter_prefix_ + ".output_frame", "map");
    distances_ = node_->declare_parameter<std::vector<double>>(parameter_prefix_ + ".candidate_standoff_distances",
                                                               { 0.55, 0.70, 0.85 });
    angle_offsets_deg_ = node_->declare_parameter<std::vector<double>>(parameter_prefix_ + ".candidate_angle_offsets_"
                                                                                           "deg",
                                                                       { 0.0, 30.0, -30.0 });
  }

  /**
   * @brief Generate stand-off candidates around the requested target pose.
   *
   * @param request Original task request.
   * @return Candidate base poses for selector-driven evaluation.
   */

  std::vector<mobile_manipulator_base::CandidatePose>
  generate(const mobile_manipulator_base::PoseValidationRequest& request) override
  {
    const std::string base_frame = request.frame_id.empty() ? output_frame_ : request.frame_id;
    const double target_yaw = detail::quaternion_to_yaw(request.pose.orientation);

    std::vector<mobile_manipulator_base::CandidatePose> candidates;
    for (const double distance : distances_)
    {
      for (const double angle_offset_deg : angle_offsets_deg_)
      {
        const double angle_offset_rad = angle_offset_deg * M_PI / 180.0;
        const double approach_yaw = target_yaw + angle_offset_rad;

        mobile_manipulator_base::CandidatePose candidate;
        candidate.base_frame_id = base_frame;
        candidate.base_pose.position.x = request.pose.position.x - std::cos(approach_yaw) * distance;
        candidate.base_pose.position.y = request.pose.position.y - std::sin(approach_yaw) * distance;
        candidate.base_pose.position.z = 0.0;
        const double facing_yaw = std::atan2(request.pose.position.y - candidate.base_pose.position.y,
                                             request.pose.position.x - candidate.base_pose.position.x);
        candidate.base_pose.orientation = detail::quaternion_from_yaw(facing_yaw);
        candidate.score = distance + std::abs(angle_offset_deg) / 180.0;
        candidates.push_back(candidate);
      }
    }
    return candidates;
  }

private:
  rclcpp::Node* node_{ nullptr };
  std::string parameter_prefix_;
  std::string output_frame_;
  std::vector<double> distances_;
  std::vector<double> angle_offsets_deg_;
};
}  // namespace mobile_manipulator_pose_generator
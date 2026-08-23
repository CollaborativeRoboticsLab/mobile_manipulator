#pragma once

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <mobile_manipulator_base/pose_selector.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace mobile_manipulator_pose_selector
{
namespace detail
{
inline double quaternion_to_yaw(const geometry_msgs::msg::Quaternion& quaternion)
{
  tf2::Quaternion tf_quaternion(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(tf_quaternion).getRPY(roll, pitch, yaw);
  return yaw;
}

inline double shortest_angular_distance(const double from, const double to)
{
  return std::atan2(std::sin(to - from), std::cos(to - from));
}
}  // namespace detail

/**
 * @brief Default selector that prefers the feasible candidate with the least added movement.
 */
class LeastMovementPoseSelector : public mobile_manipulator_base::PoseSelector
{
public:
  /**
   * @brief Configure weights used when scoring candidate movement.
   *
   * @param node Runtime node that owns parameters.
   * @param parameter_prefix Prefix for this plugin's parameter block.
   */
  void configure(rclcpp::Node* node, const std::string& parameter_prefix) override
  {
    node_ = node;
    parameter_prefix_ = parameter_prefix;
    distance_weight_ = node_->declare_parameter<double>(parameter_prefix_ + ".distance_weight", 1.0);
    rotation_weight_ = node_->declare_parameter<double>(parameter_prefix_ + ".rotation_weight", 1.0);
  }

  /**
   * @brief Evaluate and select the best feasible candidate pose.
   *
   * @param candidates Candidate poses from the generator.
   * @param request Original task request for context-sensitive scoring.
   * @param evaluate_candidate Callback used to validate each candidate.
   * @return Best feasible evaluated candidate, or std::nullopt if none pass validation.
   */
  std::optional<mobile_manipulator_base::EvaluatedCandidate>
  select(const std::vector<mobile_manipulator_base::CandidatePose>& candidates,
         const mobile_manipulator_base::PoseValidationRequest& request,
         const mobile_manipulator_base::CandidateEvaluationCallback& evaluate_candidate) override
  {
    std::optional<mobile_manipulator_base::EvaluatedCandidate> best_candidate;
    double best_score = 0.0;

    for (const auto& candidate : candidates)
    {
      const auto validation = evaluate_candidate(candidate);
      if (!validation || !validation->feasible)
      {
        continue;
      }

      auto evaluated = mobile_manipulator_base::EvaluatedCandidate{ candidate, *validation };
      const double score = combined_score(candidate, request);
      evaluated.validation.base_target_pose_valid = true;
      evaluated.validation.base_target_pose = candidate.base_pose;
      evaluated.validation.base_target_frame = candidate.base_frame_id;
      evaluated.validation.candidate_score = score;

      if (!best_candidate || score < best_score)
      {
        best_score = score;
        best_candidate = evaluated;
      }
    }

    return best_candidate;
  }

private:
  /**
   * @brief Compute a combined score for a candidate pose based on distance and rotation from the original request.
   *
   * @param candidate Candidate pose to score.
   * @param request Original task request for context.
   * @return Combined score where lower is better.
   */
  double combined_score(const mobile_manipulator_base::CandidatePose& candidate,
                        const mobile_manipulator_base::PoseValidationRequest& request) const
  {
    double score = candidate.score;
    if (request.base_pose_valid && request.base_frame_id == candidate.base_frame_id)
    {
      const double dx = candidate.base_pose.position.x - request.base_pose.position.x;
      const double dy = candidate.base_pose.position.y - request.base_pose.position.y;
      const double distance = std::hypot(dx, dy);
      const double yaw_delta =
          std::abs(detail::shortest_angular_distance(detail::quaternion_to_yaw(request.base_pose.orientation),
                                                     detail::quaternion_to_yaw(candidate.base_pose.orientation)));
      score += distance_weight_ * distance + rotation_weight_ * yaw_delta;
    }
    return score;
  }

  rclcpp::Node* node_{ nullptr };
  std::string parameter_prefix_;
  double distance_weight_{ 1.0 };
  double rotation_weight_{ 1.0 };
};
}  // namespace mobile_manipulator_pose_selector
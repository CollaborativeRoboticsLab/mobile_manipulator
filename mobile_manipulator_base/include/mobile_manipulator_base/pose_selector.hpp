#pragma once

#include <mobile_manipulator_base/types.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace mobile_manipulator_base
{
/**
 * @brief Interface for evaluating and selecting the best feasible candidate pose.
 */
class PoseSelector
{
public:
  virtual ~PoseSelector() = default;

  /**
   * @brief Configure the selector from a node-owned parameter namespace.
   *
   * @param node Runtime node that owns parameters and ROS resources.
   * @param parameter_prefix Prefix used when declaring and reading plugin parameters.
   */
  virtual void configure(rclcpp::Node* node, const std::string& parameter_prefix) = 0;

  /**
   * @brief Evaluate candidate poses and return the selected feasible result.
   *
   * The selector owns the scoring and ordering policy while feasibility for
   * each candidate is delegated through @p evaluate_candidate.
   *
   * @param candidates Candidate base poses to consider.
   * @param request Original task request that provides scoring context.
   * @param evaluate_candidate Callback that validates a single candidate pose.
   * @return Selected evaluated candidate, or std::nullopt if none is feasible.
   */
  virtual std::optional<EvaluatedCandidate> select(const std::vector<CandidatePose>& candidates,
                                                   const PoseValidationRequest& request,
                                                   const CandidateEvaluationCallback& evaluate_candidate) = 0;
};
}  // namespace mobile_manipulator_base
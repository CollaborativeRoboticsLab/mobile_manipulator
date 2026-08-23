#pragma once

#include <mobile_manipulator_base/types.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace mobile_manipulator_base
{
/**
 * @brief Interface for generating candidate base poses for mobile manipulation.
 */
class PoseGenerator
{
public:
  virtual ~PoseGenerator() = default;

  /**
   * @brief Configure the generator from a node-owned parameter namespace.
   *
   * @param node Runtime node that owns parameters and ROS resources.
   * @param parameter_prefix Prefix used when declaring and reading plugin parameters.
   */
  virtual void configure(rclcpp::Node* node, const std::string& parameter_prefix) = 0;

  /**
   * @brief Generate candidate base poses for the supplied task request.
   *
   * @param request Requested target pose and mode.
   * @return Ordered or unordered candidate base poses for later evaluation.
   */
  virtual std::vector<CandidatePose> generate(const PoseValidationRequest& request) = 0;
};
}  // namespace mobile_manipulator_base
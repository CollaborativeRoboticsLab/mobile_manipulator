#pragma once

#include <mobile_manipulator_base/types.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace mobile_manipulator_base
{
/**
 * @brief Interface for validating whether a requested task pose is currently feasible.
 *
 * Implementations can perform arm-only checks, combined arm-plus-base checks,
 * or other policy-specific validation as long as they return a populated
 * PoseValidationResult.
 */
class PoseValidator
{
public:
  virtual ~PoseValidator() = default;

  /**
   * @brief Configure the validator from a node-owned parameter namespace.
   *
   * @param node Runtime node that owns parameters, clients, and subscriptions.
   * @param parameter_prefix Prefix used when declaring and reading plugin parameters.
   */
  virtual void configure(rclcpp::Node* node, const std::string& parameter_prefix) = 0;

  /**
   * @brief Validate a pose request against the current robot and scene state.
   *
   * @param request Requested target pose, mode, and optional supplied base pose.
   * @return Structured feasibility result for planner or executor use.
   */
  virtual PoseValidationResult validate(const PoseValidationRequest& request) = 0;
};
}  // namespace mobile_manipulator_base
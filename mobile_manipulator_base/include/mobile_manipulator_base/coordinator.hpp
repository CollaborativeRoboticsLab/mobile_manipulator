#pragma once

#include <mobile_manipulator_base/types.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace mobile_manipulator_base
{
/**
 * @brief Interface for executing an already approved mobile-manipulation solution.
 */
class Coordinator
{
public:
  virtual ~Coordinator() = default;

  /**
   * @brief Configure the coordinator from a node-owned parameter namespace.
   *
   * @param node Runtime node that owns parameters and action clients.
   * @param parameter_prefix Prefix used when declaring and reading plugin parameters.
   */
  virtual void configure(rclcpp::Node* node, const std::string& parameter_prefix) = 0;

  /**
   * @brief Execute an approved solution and emit stage feedback while running.
   *
   * @param request Approved execution request, including optional base pose.
   * @param feedback Callback used to publish stage and status updates.
   * @return Structured execution result describing success or failure.
   */
  virtual ExecutionResult execute(const ExecutionRequest& request, const FeedbackCallback& feedback) = 0;
};
}  // namespace mobile_manipulator_base
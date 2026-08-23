#pragma once

#include <chrono>
#include <future>
#include <grasping_msgs/action/move_to_pose.hpp>
#include <memory>
#include <mobile_manipulator_base/coordinator.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>

namespace mobile_manipulator_coordinator
{
/**
 * @brief Default coordinator that executes an approved base move followed by arm motion.
 */
class DefaultCoordinator : public mobile_manipulator_base::Coordinator
{
public:
  /**
   * @brief Configure navigation and arm action client names.
   *
   * @param node Runtime node that owns parameters and action clients.
   * @param parameter_prefix Prefix for this plugin's parameter block.
   */
  void configure(rclcpp::Node* node, const std::string& parameter_prefix) override
  {
    node_ = node;
    parameter_prefix_ = parameter_prefix;
    navigation_action_name_ = node_->declare_parameter<std::string>(parameter_prefix_ + ".navigation_action_name", "nav"
                                                                                                                   "iga"
                                                                                                                   "te_"
                                                                                                                   "to_"
                                                                                                                   "pos"
                                                                                                                   "e");
    move_arm_action_name_ = node_->declare_parameter<std::string>(parameter_prefix_ + ".move_arm_action_name", "move_"
                                                                                                               "arm_to_"
                                                                                                               "pose");
    navigation_client_ =
        rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(node_, navigation_action_name_);
    arm_client_ = rclcpp_action::create_client<grasping_msgs::action::MoveToPose>(node_, move_arm_action_name_);
  }

  /**
   * @brief Execute the approved solution and publish staged feedback.
   *
   * @param request Approved task solution to run.
   * @param feedback Callback for execution-stage updates.
   * @return Structured execution result.
   */
  mobile_manipulator_base::ExecutionResult execute(const mobile_manipulator_base::ExecutionRequest& request,

                                                   const mobile_manipulator_base::FeedbackCallback& feedback) override
  {
    mobile_manipulator_base::ExecutionResult result;
    result.success = false;
    result.base_pose = request.approved_base_pose;
    result.base_frame = request.approved_base_frame;
    result.base_pose_valid = request.approved_base_pose_valid;

    if (request.approved_base_pose_valid)
    {
      feedback("navigating", "Navigating to approved base pose.");
      std::string navigation_message;
      if (!navigate_to_base_target(request, navigation_message))
      {
        result.stage = "navigating";
        result.failure_reason = "navigation_failed";
        result.message = navigation_message;
        return result;
      }
    }

    feedback("moving_arm", "Executing arm motion for approved target pose.");
    std::string arm_message;
    if (!move_arm_to_target(request, arm_message))
    {
      result.stage = "moving_arm";
      result.failure_reason = "arm_execution_failed";
      result.message = arm_message;
      return result;
    }

    result.success = true;
    result.stage = "completed";
    result.message = arm_message;
    return result;
  }

private:

  /**
   * @brief Send a NavigateToPose goal to the navigation action server.
   *
   * @param request Approved execution request with base pose.
   * @param message Output message describing success or failure.
   * @return True if navigation succeeded, false otherwise.
   */
  bool navigate_to_base_target(const mobile_manipulator_base::ExecutionRequest& request, std::string& message)
  {
    if (!navigation_client_->wait_for_action_server(std::chrono::seconds(5)))
    {
      message = "NavigateToPose action server is not available.";
      return false;
    }

    nav2_msgs::action::NavigateToPose::Goal goal;
    goal.pose.header.frame_id = request.approved_base_frame.empty() ? "map" : request.approved_base_frame;
    goal.pose.header.stamp = node_->now();
    goal.pose.pose = request.approved_base_pose;

    auto send_future = navigation_client_->async_send_goal(goal);
    if (send_future.wait_for(std::chrono::seconds(10)) != std::future_status::ready)
    {
      message = "Failed to send NavigateToPose goal.";
      return false;
    }

    auto goal_handle = send_future.get();
    if (!goal_handle)
    {
      message = "NavigateToPose goal was rejected.";
      return false;
    }

    auto result_future = navigation_client_->async_get_result(goal_handle);
    if (result_future.wait_for(std::chrono::seconds(120)) != std::future_status::ready)
    {
      message = "NavigateToPose result not received before timeout.";
      return false;
    }

    const auto result = result_future.get();
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED)
    {
      message = "NavigateToPose action failed.";
      return false;
    }

    message = "Base reposition completed successfully.";
    return true;
  }

  /**
   * @brief Send a MoveToPose goal to the arm action server.
   *
   * @param request Approved execution request with target pose.
   * @param message Output message describing success or failure.
   * @return True if arm motion succeeded, false otherwise.
   */
  bool move_arm_to_target(const mobile_manipulator_base::ExecutionRequest& request, std::string& message)
  {
    if (!arm_client_->wait_for_action_server(std::chrono::seconds(5)))
    {
      message = "MoveToPose action server is not available.";
      return false;
    }

    grasping_msgs::action::MoveToPose::Goal goal;
    goal.target_pose.header.frame_id = request.frame_id;
    goal.target_pose.header.stamp = node_->now();
    goal.target_pose.pose = request.pose;

    auto send_future = arm_client_->async_send_goal(goal);
    if (send_future.wait_for(std::chrono::seconds(10)) != std::future_status::ready)
    {
      message = "Failed to send MoveToPose goal.";
      return false;
    }

    auto goal_handle = send_future.get();
    if (!goal_handle)
    {
      message = "MoveToPose goal was rejected.";
      return false;
    }

    auto result_future = arm_client_->async_get_result(goal_handle);
    if (result_future.wait_for(std::chrono::seconds(120)) != std::future_status::ready)
    {
      message = "MoveToPose result not received before timeout.";
      return false;
    }

    const auto result = result_future.get();
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result || !result.result->success)
    {
      message = result.result ? result.result->message : "MoveToPose action failed.";
      return false;
    }

    message = result.result->message;
    return true;
  }

  rclcpp::Node* node_{ nullptr };
  std::string parameter_prefix_;
  std::string navigation_action_name_;
  std::string move_arm_action_name_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_client_;
  rclcpp_action::Client<grasping_msgs::action::MoveToPose>::SharedPtr arm_client_;
};
}  // namespace mobile_manipulator_coordinator
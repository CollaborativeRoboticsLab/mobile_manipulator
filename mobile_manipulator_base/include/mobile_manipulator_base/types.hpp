#pragma once

#include <functional>
#include <geometry_msgs/msg/pose.hpp>
#include <optional>
#include <string>
#include <vector>

namespace mobile_manipulator_base
{
/**
 * @brief Request structure for validating a target pose for mobile manipulation.
 * 
 * @param pose Target pose to validate.
 * @param frame_id Frame in which the target pose is expressed.
 * @param mode Grasping mode to use for validation.
 * @param base_pose_valid Whether a base pose is provided and valid.
 * @param base_pose Base pose of the robot in the world frame.
 * @param base_frame_id Frame in which the base pose is expressed.
 */
struct PoseValidationRequest
{
  geometry_msgs::msg::Pose pose;
  std::string frame_id;
  std::string mode;
  bool base_pose_valid{ false };
  geometry_msgs::msg::Pose base_pose;
  std::string base_frame_id;
};

/**
 * @brief Result structure for validating a target pose for mobile manipulation.
 * 
 * @param feasible Whether the requested pose is feasible.
 * @param mode_used The grasping mode that was used for validation.
 * @param failure_reason Reason for failure if the pose is not feasible.
 * @param suggested_fallback Suggested fallback strategy if the pose is not feasible.
 * @param message Human-readable message describing the result.
 * @param planning_frame Frame in which the planning pose is expressed.
 * @param planning_pose The planning pose that was validated.
 * @param planning_pose_valid Whether the planning pose is valid.
 * @param base_target_pose The base target pose that was validated.
 * @param base_target_frame Frame in which the base target pose is expressed.
 * @param base_target_pose_valid Whether the base target pose is valid.
 * @param candidate_score Score of the candidate pose if applicable.
 */
struct PoseValidationResult
{
  bool feasible{ false };
  std::string mode_used;
  std::string failure_reason;
  std::string suggested_fallback;
  std::string message;
  std::string planning_frame;
  geometry_msgs::msg::Pose planning_pose;
  bool planning_pose_valid{ false };
  geometry_msgs::msg::Pose base_target_pose;
  std::string base_target_frame;
  bool base_target_pose_valid{ false };
  double candidate_score{ 0.0 };
};

/**
 * @brief Structure representing a candidate base pose for mobile manipulation.
 * 
 * @param base_pose The candidate base pose.
 * @param base_frame_id Frame in which the candidate base pose is expressed.
 * @param score Score of the candidate pose.
 */
struct CandidatePose
{
  geometry_msgs::msg::Pose base_pose;
  std::string base_frame_id;
  double score{ 0.0 };
};

/**
 * @brief Structure representing an evaluated candidate base pose along with its validation result.
 * 
 * @param candidate The candidate base pose.
 * @param validation The validation result for the candidate pose.
 */
struct EvaluatedCandidate
{
  CandidatePose candidate;
  PoseValidationResult validation;
};

/**
 * @brief Structure representing a request to execute a task pose with the mobile manipulator.
 * 
 * @param pose Target pose to execute.
 * @param frame_id Frame in which the target pose is expressed.
 * @param mode Grasping mode to use for execution.
 * @param approved_base_pose_valid Whether an approved base pose is provided and valid.
 * @param approved_base_pose Approved base pose of the robot in the world frame.
 * @param approved_base_frame Frame in which the approved base pose is expressed.
 */
struct ExecutionRequest
{
  geometry_msgs::msg::Pose pose;
  std::string frame_id;
  std::string mode;
  bool approved_base_pose_valid{ false };
  geometry_msgs::msg::Pose approved_base_pose;
  std::string approved_base_frame;
};

/**
 * @brief Structure representing the result of executing a task pose with the mobile manipulator.
 * 
 * @param success Whether the execution was successful.
 * @param stage Current stage of execution.
 * @param failure_reason Reason for failure if the execution was not successful.
 * @param suggested_fallback Suggested fallback strategy if the execution was not successful.
 * @param message Human-readable message describing the result.
 * @param base_pose The base pose after execution.
 * @param base_frame Frame in which the base pose is expressed.
 * @param base_pose_valid Whether the base pose is valid after execution.
 */
struct ExecutionResult
{
  bool success{ false };
  std::string stage;
  std::string failure_reason;
  std::string suggested_fallback;
  std::string message;
  geometry_msgs::msg::Pose base_pose;
  std::string base_frame;
  bool base_pose_valid{ false };
};

using CandidateEvaluationCallback =
    std::function<std::optional<PoseValidationResult>(const CandidatePose& candidate)>;
using FeedbackCallback = std::function<void(const std::string&, const std::string&)>;
}  // namespace mobile_manipulator_base
#pragma once

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
#include <future>
#include <grasping_msgs/srv/check_cartesian_pose_feasibility.hpp>
#include <memory>
#include <mobile_manipulator_base/pose_validator.hpp>
#include <mobile_manipulator_pose_validator/costmap_validator.hpp>
#include <mutex>
#include <nav2_msgs/msg/costmap.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace mobile_manipulator_pose_validator
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
 * @brief Check if a pose is free in the given costmap.
 *
 * @param costmap Costmap to check against.
 * @param pose Pose to validate.
 * @return True if the pose is free, false otherwise.
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

/**
 * @brief Compute the inverse of a yaw quaternion.
 *
 * @param yaw Input yaw angle in radians.
 * @return Inverse quaternion.
 */
inline geometry_msgs::msg::Quaternion inverse_yaw_quaternion(const double yaw)
{
  return quaternion_from_yaw(-yaw);
}

/**
 * @brief Multiply two quaternions.
 *
 * @param left Left quaternion.
 * @param right Right quaternion.
 * @return Resulting quaternion from the multiplication.
 */
inline geometry_msgs::msg::Quaternion multiply_quaternions(const geometry_msgs::msg::Quaternion& left,
                                                           const geometry_msgs::msg::Quaternion& right)
{
  tf2::Quaternion left_quaternion(left.x, left.y, left.z, left.w);
  tf2::Quaternion right_quaternion(right.x, right.y, right.z, right.w);
  const tf2::Quaternion result = left_quaternion * right_quaternion;
  geometry_msgs::msg::Quaternion out;
  out.x = result.x();
  out.y = result.y();
  out.z = result.z();
  out.w = result.w();
  return out;
}

/**
 * @brief Project a target pose into the base frame of the robot.
 *
 * @param target_pose Target pose in the world frame.
 * @param base_pose Base pose of the robot in the world frame.
 * @return Projected pose in the base frame.
 */
inline geometry_msgs::msg::Pose project_target_into_base_frame(const geometry_msgs::msg::Pose& target_pose,
                                                               const geometry_msgs::msg::Pose& base_pose)
{
  geometry_msgs::msg::Pose projected;
  const double base_yaw = quaternion_to_yaw(base_pose.orientation);
  const double dx = target_pose.position.x - base_pose.position.x;
  const double dy = target_pose.position.y - base_pose.position.y;

  projected.position.x = std::cos(base_yaw) * dx + std::sin(base_yaw) * dy;
  projected.position.y = -std::sin(base_yaw) * dx + std::cos(base_yaw) * dy;
  projected.position.z = target_pose.position.z - base_pose.position.z;
  projected.orientation = multiply_quaternions(inverse_yaw_quaternion(base_yaw), target_pose.orientation);
  return projected;
}
}  // namespace detail

/**
 * @brief Default pose validator that delegates arm feasibility to grasping services.
 *
 * When a base pose is supplied, the validator projects the target pose into the
 * arm base frame before calling the grasping Cartesian feasibility service. It
 * can also reject candidate base poses against a Nav2 costmap when enabled.
 */
class GraspingPoseValidator : public mobile_manipulator_base::PoseValidator
{
public:
  /**
   * @brief Configure grasping service and optional costmap validation parameters.
   *
   * @param node Runtime node that owns parameters and ROS clients.
   * @param parameter_prefix Prefix for this plugin's parameter block.
   */
  void configure(rclcpp::Node* node, const std::string& parameter_prefix) override
  {
    node_ = node;
    parameter_prefix_ = parameter_prefix;
    grasping_service_name_ = node_->declare_parameter<std::string>(parameter_prefix_ + ".service_name", "check_"
                                                                                                        "cartesian_"
                                                                                                        "pose_"
                                                                                                        "feasibility");
    arm_base_frame_ = node_->declare_parameter<std::string>(parameter_prefix_ + ".arm_base_frame", "base_link");
    use_costmap_validation_ = node_->declare_parameter<bool>(parameter_prefix_ + ".use_costmap_validation", false);
    costmap_topic_ = node_->declare_parameter<std::string>(parameter_prefix_ + ".costmap_topic", "/global_costmap/"
                                                                                                 "costmap");
    costmap_occupied_threshold_ =
        static_cast<uint8_t>(node_->declare_parameter<int>(parameter_prefix_ + ".costmap_occupied_threshold", 253));

    grasping_client_ = node_->create_client<grasping_msgs::srv::CheckCartesianPoseFeasibility>(grasping_service_name_);
    if (use_costmap_validation_)
    {
      costmap_subscription_ = node_->create_subscription<nav2_msgs::msg::Costmap>(
          costmap_topic_, rclcpp::QoS(1).best_effort(), [this](const nav2_msgs::msg::Costmap::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(costmap_mutex_);
            latest_costmap_ = *msg;
          });
    }
  }

  /**
   * @brief Validate a target pose with optional base-pose projection.
   *
   * @param request Pose validation request supplied by the runtime server.
   * @return Structured feasibility result from grasping checks and local guards.
   */
  mobile_manipulator_base::PoseValidationResult
  validate(const mobile_manipulator_base::PoseValidationRequest& request) override
  {
    mobile_manipulator_base::PoseValidationResult result;
    result.mode_used = request.mode;

    if (request.base_pose_valid)
    {
      if (request.frame_id.empty() || request.base_frame_id.empty() || request.frame_id != request.base_frame_id)
      {
        result.failure_reason = "unsupported_base_frame";
        result.message = "Target pose and provided base pose must be expressed in the same frame.";
        return result;
      }

      if (use_costmap_validation_)
      {
        std::lock_guard<std::mutex> lock(costmap_mutex_);
        if (!latest_costmap_)
        {
          result.failure_reason = "costmap_unavailable";
          result.message = "Costmap validation is enabled but no costmap has been received yet.";
          return result;
        }
        if (!detail::pose_is_costmap_free(*latest_costmap_, request.base_pose, costmap_occupied_threshold_))
        {
          result.failure_reason = "base_pose_occupied";
          result.message = "Provided base pose is occupied in the current costmap.";
          return result;
        }
      }

      const auto projected_pose = detail::project_target_into_base_frame(request.pose, request.base_pose);
      auto grasping_response = call_grasping_service(projected_pose, arm_base_frame_, request.mode);
      if (!grasping_response)
      {
        result.failure_reason = "service_unavailable";
        result.message = "Failed to call grasping Cartesian feasibility service.";
        return result;
      }
      copy_grasping_response(*grasping_response, result);
      result.base_target_pose_valid = true;
      result.base_target_pose = request.base_pose;
      result.base_target_frame = request.base_frame_id;
      return result;
    }

    auto grasping_response = call_grasping_service(request.pose, request.frame_id, request.mode);
    if (!grasping_response)
    {
      result.failure_reason = "service_unavailable";
      result.message = "Failed to call grasping Cartesian feasibility service.";
      return result;
    }
    copy_grasping_response(*grasping_response, result);
    return result;
  }

private:
  /**
   * @brief Call the grasping Cartesian feasibility service with the given pose and frame.
   *
   * @param pose Target pose to validate.
   * @param frame_id Frame in which the target pose is expressed.
   * @param mode Grasping mode to use for validation.
   * @return Shared pointer to the service response, or nullptr if the call failed.
   */
  std::shared_ptr<grasping_msgs::srv::CheckCartesianPoseFeasibility::Response>
  call_grasping_service(const geometry_msgs::msg::Pose& pose, const std::string& frame_id, const std::string& mode)
  {
    if (!grasping_client_->wait_for_service(std::chrono::seconds(2)))
    {
      return nullptr;
    }

    auto request = std::make_shared<grasping_msgs::srv::CheckCartesianPoseFeasibility::Request>();
    request->pose = pose;
    request->planning_frame = frame_id;
    request->mode = mode;
    auto future = grasping_client_->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(10)) != std::future_status::ready)
    {
      return nullptr;
    }
    return future.get();
  }

  /**
   * @brief Copy the relevant fields from the grasping service response to the pose validation result.
   *
   * @param grasping_response Response from the grasping service.
   * @param result Pose validation result to populate.
   */
  static void
  copy_grasping_response(const grasping_msgs::srv::CheckCartesianPoseFeasibility::Response& grasping_response,
                         mobile_manipulator_base::PoseValidationResult& result)
  {
    result.feasible = grasping_response.feasible;
    result.mode_used = grasping_response.mode_used;
    result.failure_reason = grasping_response.failure_reason;
    result.suggested_fallback = grasping_response.suggested_fallback;
    result.message = grasping_response.message;
    result.planning_frame = grasping_response.planning_frame;
    result.planning_pose = grasping_response.planning_pose;
    result.planning_pose_valid = grasping_response.planning_pose_valid;
  }

  rclcpp::Node* node_{ nullptr };
  std::string parameter_prefix_;
  std::string grasping_service_name_;
  std::string arm_base_frame_;
  bool use_costmap_validation_{ false };
  std::string costmap_topic_;
  uint8_t costmap_occupied_threshold_{ 253 };
  std::mutex costmap_mutex_;
  std::optional<nav2_msgs::msg::Costmap> latest_costmap_;
  rclcpp::Client<grasping_msgs::srv::CheckCartesianPoseFeasibility>::SharedPtr grasping_client_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_subscription_;
};
}  // namespace mobile_manipulator_pose_validator
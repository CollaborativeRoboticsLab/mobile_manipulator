#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <grasping_msgs/srv/check_cartesian_pose_feasibility.hpp>
#include <mobile_manipulator_msgs/srv/check_task_feasibility.hpp>
#include <mobile_manipulator_validator/costmap_validator.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace mobile_manipulator_validator
{
namespace detail
{
inline double quaternion_to_yaw(const geometry_msgs::msg::Quaternion & quaternion)
{
  tf2::Quaternion tf_quaternion(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(tf_quaternion).getRPY(roll, pitch, yaw);
  return yaw;
}

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

inline geometry_msgs::msg::Quaternion inverse_yaw_quaternion(const double yaw)
{
  return quaternion_from_yaw(-yaw);
}

inline geometry_msgs::msg::Quaternion multiply_quaternions(
  const geometry_msgs::msg::Quaternion & left,
  const geometry_msgs::msg::Quaternion & right)
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
}  // namespace detail

class TaskFeasibilityServiceNode : public rclcpp::Node
{
public:
  TaskFeasibilityServiceNode()
  : Node("task_feasibility_service_node")
  {
    declare_parameter<std::string>("task_feasibility_service_name", "check_task_feasibility");
    declare_parameter<std::string>(
      "grasping_cartesian_feasibility_service_name", "check_cartesian_pose_feasibility");
    declare_parameter<std::string>("navigation_frame", "map");
    declare_parameter<std::string>("arm_base_frame", "base_link");
    declare_parameter<std::string>("mobile_recheck_mode", "arm_only_plan");
    declare_parameter<bool>("use_costmap_validation", false);
    declare_parameter<std::string>("costmap_topic", "/global_costmap/costmap");
    declare_parameter<int>("costmap_occupied_threshold", 253);
    declare_parameter<std::vector<double>>("candidate_standoff_distances", {0.55, 0.70, 0.85});
    declare_parameter<std::vector<double>>("candidate_angle_offsets_deg", {0.0, 30.0, -30.0});

    grasping_client_ = create_client<grasping_msgs::srv::CheckCartesianPoseFeasibility>(
      get_parameter("grasping_cartesian_feasibility_service_name").as_string());
    if (get_parameter("use_costmap_validation").as_bool())
    {
      costmap_subscription_ = create_subscription<nav2_msgs::msg::Costmap>(
        get_parameter("costmap_topic").as_string(),
        rclcpp::QoS(1).best_effort(),
        std::bind(&TaskFeasibilityServiceNode::costmap_callback, this, std::placeholders::_1));
    }
    service_ = create_service<mobile_manipulator_msgs::srv::CheckTaskFeasibility>(
      get_parameter("task_feasibility_service_name").as_string(),
      std::bind(&TaskFeasibilityServiceNode::handle_request, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  struct Candidate
  {
    geometry_msgs::msg::Pose base_pose;
    geometry_msgs::msg::Pose projected_target_pose;
    double score{0.0};
  };

  void handle_request(
    const std::shared_ptr<mobile_manipulator_msgs::srv::CheckTaskFeasibility::Request> request,
    std::shared_ptr<mobile_manipulator_msgs::srv::CheckTaskFeasibility::Response> response)
  {
    response->mode_used = request->mode;

    if (request->mode == "arm_only_ik" || request->mode == "arm_only_plan")
    {
      forward_arm_only_request(request->pose, request->frame_id, request->mode, response);
      return;
    }

    if (request->mode != "mobile_manipulator")
    {
      mark_failure(*response, "unsupported_mode", "", "Unsupported task feasibility mode '" + request->mode + "'.");
      return;
    }

    const std::string navigation_frame = get_parameter("navigation_frame").as_string();
    if (!request->frame_id.empty() && request->frame_id != navigation_frame)
    {
      mark_failure(
        *response,
        "unsupported_target_frame",
        "",
        "mobile_manipulator mode currently requires target poses in the configured navigation frame '" +
          navigation_frame + "'.");
      return;
    }

    const std::string recheck_mode = get_parameter("mobile_recheck_mode").as_string();
    auto arm_only_response = call_grasping_service(request->pose, request->frame_id, recheck_mode);
    if (!arm_only_response)
    {
      mark_failure(*response, "service_unavailable", "", "Failed to call grasping Cartesian feasibility service.");
      return;
    }

    if (arm_only_response->feasible)
    {
      copy_arm_response(*arm_only_response, *response);
      response->mode_used = request->mode;
      response->message = "Arm-only feasibility succeeded; no base reposition required.";
      return;
    }

    const auto candidates = generate_candidates(request->pose);
    for (const auto & candidate : candidates)
    {
      if (!candidate_is_navigation_valid(candidate))
      {
        continue;
      }

      auto projected_response = call_grasping_service(
        candidate.projected_target_pose,
        get_parameter("arm_base_frame").as_string(),
        recheck_mode);
      if (!projected_response || !projected_response->feasible)
      {
        continue;
      }

      copy_arm_response(*projected_response, *response);
      response->mode_used = request->mode;
      response->suggested_fallback = "move_base_then_arm";
      response->message = "Mobile-manipulator feasibility succeeded after projecting a candidate base pose.";
      response->base_target_pose = candidate.base_pose;
      response->base_target_frame =
        request->frame_id.empty() ? get_parameter("navigation_frame").as_string() : request->frame_id;
      response->base_target_pose_valid = true;
      response->candidate_score = candidate.score;
      return;
    }

    copy_arm_response(*arm_only_response, *response);
    response->mode_used = request->mode;
    response->feasible = false;
    response->failure_reason = "no_mobile_manipulator_candidate";
    response->suggested_fallback = "move_base_then_arm";
    response->message =
      "Arm-only feasibility failed and no projected base candidate produced a feasible arm target.";
  }

  void forward_arm_only_request(
    const geometry_msgs::msg::Pose & pose,
    const std::string & frame_id,
    const std::string & mode,
    std::shared_ptr<mobile_manipulator_msgs::srv::CheckTaskFeasibility::Response> response)
  {
    auto arm_response = call_grasping_service(pose, frame_id, mode);
    if (!arm_response)
    {
      mark_failure(*response, "service_unavailable", "", "Failed to call grasping Cartesian feasibility service.");
      return;
    }
    copy_arm_response(*arm_response, *response);
  }

  std::shared_ptr<grasping_msgs::srv::CheckCartesianPoseFeasibility::Response> call_grasping_service(
    const geometry_msgs::msg::Pose & pose,
    const std::string & frame_id,
    const std::string & mode)
  {
    if (!grasping_client_->wait_for_service(std::chrono::seconds(2)))
    {
      return nullptr;
    }

    auto request = std::make_shared<grasping_msgs::srv::CheckCartesianPoseFeasibility::Request>();
    request->pose = pose;
    request->frame_id = frame_id;
    request->mode = mode;
    auto future = grasping_client_->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(10)) != std::future_status::ready)
    {
      return nullptr;
    }
    return future.get();
  }

  std::vector<Candidate> generate_candidates(const geometry_msgs::msg::Pose & target_pose) const
  {
    const auto distances = get_parameter("candidate_standoff_distances").as_double_array();
    const auto angle_offsets_deg = get_parameter("candidate_angle_offsets_deg").as_double_array();
    const double target_yaw = detail::quaternion_to_yaw(target_pose.orientation);

    std::vector<Candidate> candidates;
    for (const double distance : distances)
    {
      for (const double angle_offset_deg : angle_offsets_deg)
      {
        const double angle_offset_rad = angle_offset_deg * M_PI / 180.0;
        const double approach_yaw = target_yaw + angle_offset_rad;

        Candidate candidate;
        candidate.base_pose.position.x = target_pose.position.x - std::cos(approach_yaw) * distance;
        candidate.base_pose.position.y = target_pose.position.y - std::sin(approach_yaw) * distance;
        candidate.base_pose.position.z = 0.0;

        const double facing_yaw = std::atan2(
          target_pose.position.y - candidate.base_pose.position.y,
          target_pose.position.x - candidate.base_pose.position.x);
        candidate.base_pose.orientation = detail::quaternion_from_yaw(facing_yaw);
        candidate.projected_target_pose = project_target_into_base_frame(target_pose, candidate.base_pose);
        candidate.score = distance + std::abs(angle_offset_deg) / 180.0;
        candidates.push_back(candidate);
      }
    }

    std::sort(candidates.begin(), candidates.end(), [](const Candidate & left, const Candidate & right) {
      return left.score < right.score;
    });
    return candidates;
  }

  geometry_msgs::msg::Pose project_target_into_base_frame(
    const geometry_msgs::msg::Pose & target_pose,
    const geometry_msgs::msg::Pose & base_pose) const
  {
    geometry_msgs::msg::Pose projected;
    const double base_yaw = detail::quaternion_to_yaw(base_pose.orientation);
    const double dx = target_pose.position.x - base_pose.position.x;
    const double dy = target_pose.position.y - base_pose.position.y;

    projected.position.x = std::cos(base_yaw) * dx + std::sin(base_yaw) * dy;
    projected.position.y = -std::sin(base_yaw) * dx + std::cos(base_yaw) * dy;
    projected.position.z = target_pose.position.z - base_pose.position.z;
    projected.orientation = detail::multiply_quaternions(
      detail::inverse_yaw_quaternion(base_yaw), target_pose.orientation);
    return projected;
  }

  static void copy_arm_response(
    const grasping_msgs::srv::CheckCartesianPoseFeasibility::Response & arm_response,
    mobile_manipulator_msgs::srv::CheckTaskFeasibility::Response & response)
  {
    response.feasible = arm_response.feasible;
    response.failure_reason = arm_response.failure_reason;
    response.suggested_fallback = arm_response.suggested_fallback;
    response.message = arm_response.message;
    response.planning_frame = arm_response.planning_frame;
    response.planning_pose = arm_response.planning_pose;
    response.planning_pose_valid = arm_response.planning_pose_valid;
  }

  static void mark_failure(
    mobile_manipulator_msgs::srv::CheckTaskFeasibility::Response & response,
    const std::string & failure_reason,
    const std::string & suggested_fallback,
    const std::string & message)
  {
    response.feasible = false;
    response.failure_reason = failure_reason;
    response.suggested_fallback = suggested_fallback;
    response.message = message;
  }

  void costmap_callback(const nav2_msgs::msg::Costmap::SharedPtr msg)
  {
    std::scoped_lock lock(costmap_mutex_);
    latest_costmap_ = msg;
  }

  bool candidate_is_navigation_valid(const Candidate & candidate) const
  {
    if (!get_parameter("use_costmap_validation").as_bool())
    {
      return true;
    }

    nav2_msgs::msg::Costmap::SharedPtr costmap;
    {
      std::scoped_lock lock(costmap_mutex_);
      costmap = latest_costmap_;
    }
    if (!costmap)
    {
      return true;
    }

    const int configured_threshold = get_parameter("costmap_occupied_threshold").as_int();
    const auto threshold = static_cast<uint8_t>(std::clamp(configured_threshold, 0, 255));
    return mobile_manipulator_validator::detail::pose_is_costmap_free(*costmap, candidate.base_pose, threshold);
  }

  rclcpp::Client<grasping_msgs::srv::CheckCartesianPoseFeasibility>::SharedPtr grasping_client_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_subscription_;
  rclcpp::Service<mobile_manipulator_msgs::srv::CheckTaskFeasibility>::SharedPtr service_;
  mutable std::mutex costmap_mutex_;
  nav2_msgs::msg::Costmap::SharedPtr latest_costmap_;
};
}  // namespace mobile_manipulator_validator
#pragma once

#include <cstddef>
#include <memory>
#include <mobile_manipulator_base/coordinator.hpp>
#include <mobile_manipulator_base/pose_generator.hpp>
#include <mobile_manipulator_base/pose_selector.hpp>
#include <mobile_manipulator_base/pose_validator.hpp>
#include <mobile_manipulator_msgs/action/execute_task_pose.hpp>
#include <mobile_manipulator_msgs/srv/check_task_feasibility.hpp>
#include <mobile_manipulator_msgs/srv/find_feasible_pose.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>
#include <thread>

namespace mobile_manipulator_ros
{
/**
 * @brief Runtime node that exposes the generic mobile-manipulator ROS APIs.
 *
 * The server loads validator, generator, selector, and coordinator plugins at
 * startup, then exposes feasibility checking, candidate search, and approved
 * execution through a single ROS node.
 */
class MobileManipulatorServer : public rclcpp::Node
{
public:
  using ExecuteTaskPose = mobile_manipulator_msgs::action::ExecuteTaskPose;
  using GoalHandleExecuteTaskPose = rclcpp_action::ServerGoalHandle<ExecuteTaskPose>;

  /**
   * @brief Construct the mobile manipulator runtime server and load plugins.
   */
  MobileManipulatorServer()
    : Node("mobile_manipulator_server")
    , pose_validator_loader_("mobile_manipulator_base", "mobile_manipulator_base::PoseValidator")
    , pose_generator_loader_("mobile_manipulator_base", "mobile_manipulator_base::PoseGenerator")
    , pose_selector_loader_("mobile_manipulator_base", "mobile_manipulator_base::PoseSelector")
    , coordinator_loader_("mobile_manipulator_base", "mobile_manipulator_base::Coordinator")
  {
    declare_parameter<std::string>("check_task_feasibility_service_name", "check_task_feasibility");
    declare_parameter<std::string>("find_feasible_pose_service_name", "find_feasible_pose");
    declare_parameter<std::string>("execute_task_pose_action_name", "execute_task_pose");

    pose_validator_ = load_plugin(pose_validator_loader_, "pose_validator_plugin",
                                  "mobile_manipulator_pose_validator::GraspingPoseValidator", "pose validator");
    pose_generator_ = load_plugin(pose_generator_loader_, "pose_generator_plugin",
                                  "mobile_manipulator_pose_generator::StandoffPoseGenerator", "pose generator");
    pose_selector_ = load_plugin(pose_selector_loader_, "pose_selector_plugin",
                                 "mobile_manipulator_pose_selector::LeastMovementPoseSelector", "pose selector");
    coordinator_ = load_plugin(coordinator_loader_, "coordinator_plugin",
                               "mobile_manipulator_coordinator::DefaultCoordinator", "coordinator");

    check_task_feasibility_service_ = create_service<mobile_manipulator_msgs::srv::CheckTaskFeasibility>(
        get_parameter("check_task_feasibility_service_name").as_string(),
        std::bind(&MobileManipulatorServer::handle_check_task_feasibility, this, std::placeholders::_1,
                  std::placeholders::_2));

    find_feasible_pose_service_ = create_service<mobile_manipulator_msgs::srv::FindFeasiblePose>(
        get_parameter("find_feasible_pose_service_name").as_string(),
        std::bind(&MobileManipulatorServer::handle_find_feasible_pose, this, std::placeholders::_1,
                  std::placeholders::_2));

    execute_task_pose_action_server_ = rclcpp_action::create_server<ExecuteTaskPose>(
        this, get_parameter("execute_task_pose_action_name").as_string(),
        std::bind(&MobileManipulatorServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&MobileManipulatorServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&MobileManipulatorServer::handle_accepted, this, std::placeholders::_1));
  }

private:
  template <typename InterfaceT>
  /**
   * @brief Load, configure, and return a plugin instance of the requested interface type.
   *
   * @param loader Pluginlib loader bound to a specific base interface.
   * @param type_param Parameter that selects the concrete plugin class.
   * @param default_type Default plugin class if no parameter override is provided.
   * @param description Human-readable plugin role for logging.
   * @return Configured plugin instance.
   */
  std::shared_ptr<InterfaceT> load_plugin(pluginlib::ClassLoader<InterfaceT>& loader, const std::string& type_param,
                                          const std::string& default_type, const std::string& description)
  {
    const auto plugin_type = declare_parameter<std::string>(type_param, default_type);
    const auto parameter_prefix = plugin_parameter_prefix(plugin_type);

    RCLCPP_INFO(get_logger(), "Loading %s plugin %s with parameter prefix %s", description.c_str(), plugin_type.c_str(),
                parameter_prefix.c_str());

    auto plugin = loader.createSharedInstance(plugin_type);
    plugin->configure(this, parameter_prefix);
    return plugin;
  }

  /**
   * @brief Derive the parameter block name from a plugin class name.
   *
   * For example, `namespace::GraspingPoseValidator` becomes
   * `GraspingPoseValidator` so configuration can live under a concise
   * class-keyed parameter subtree.
   *
   * @param plugin_type Fully qualified plugin class name.
   * @return Parameter prefix used when configuring the plugin.
   */
  static std::string plugin_parameter_prefix(const std::string& plugin_type)
  {
    const std::size_t separator = plugin_type.rfind("::");
    if (separator == std::string::npos)
    {
      return plugin_type;
    }
    return plugin_type.substr(separator + 2);
  }

  /**
   * @brief Build a PoseValidationRequest from the given parameters.
   *
   * @param pose Target pose to validate.
   * @param frame_id Frame in which the target pose is expressed.
   * @param mode Grasping mode to use for validation.
   * @param base_pose_valid Whether a base pose is provided and valid.
   * @param base_pose Base pose of the robot in the world frame.
   * @param base_frame_id Frame in which the base pose is expressed.
   * @return Constructed PoseValidationRequest.
   */
  static mobile_manipulator_base::PoseValidationRequest build_validation_request(
      const geometry_msgs::msg::Pose& pose, const std::string& frame_id, const std::string& mode,
      const bool base_pose_valid, const geometry_msgs::msg::Pose& base_pose, const std::string& base_frame_id)
  {
    mobile_manipulator_base::PoseValidationRequest request;
    request.pose = pose;
    request.frame_id = frame_id;
    request.mode = mode;
    request.base_pose_valid = base_pose_valid;
    request.base_pose = base_pose;
    request.base_frame_id = base_frame_id;
    return request;
  }

  /**
   * @brief Copy the relevant fields from a PoseValidationResult to a ROS service response.
   *
   * @param source PoseValidationResult to copy from.
   * @param response ROS service response to populate.
   */
  static void copy_validation_result(const mobile_manipulator_base::PoseValidationResult& source,
                                     mobile_manipulator_msgs::srv::CheckTaskFeasibility::Response& response)
  {
    response.feasible = source.feasible;
    response.mode_used = source.mode_used;
    response.failure_reason = source.failure_reason;
    response.suggested_fallback = source.suggested_fallback;
    response.message = source.message;
    response.planning_frame = source.planning_frame;
    response.planning_pose = source.planning_pose;
    response.planning_pose_valid = source.planning_pose_valid;
    response.base_target_pose = source.base_target_pose;
    response.base_target_frame = source.base_target_frame;
    response.base_target_pose_valid = source.base_target_pose_valid;
    response.candidate_score = source.candidate_score;
  }

  /**
   * @brief Copy the relevant fields from a PoseValidationResult to a ROS service response.
   *
   * @param source PoseValidationResult to copy from.
   * @param response ROS service response to populate.
   */
  static void copy_validation_result(const mobile_manipulator_base::PoseValidationResult& source,
                                     mobile_manipulator_msgs::srv::FindFeasiblePose::Response& response)
  {
    response.feasible = source.feasible;
    response.mode_used = source.mode_used;
    response.failure_reason = source.failure_reason;
    response.suggested_fallback = source.suggested_fallback;
    response.message = source.message;
    response.planning_frame = source.planning_frame;
    response.planning_pose = source.planning_pose;
    response.planning_pose_valid = source.planning_pose_valid;
    response.base_target_pose = source.base_target_pose;
    response.base_target_frame = source.base_target_frame;
    response.base_target_pose_valid = source.base_target_pose_valid;
    response.candidate_score = source.candidate_score;
  }

  /**
   * @brief Publish feedback for an ongoing ExecuteTaskPose action.
   *
   * @param goal_handle Handle to the current action goal.
   * @param stage Current stage of execution.
   * @param message Human-readable message describing the current state.
   */
  void handle_check_task_feasibility(
      const std::shared_ptr<mobile_manipulator_msgs::srv::CheckTaskFeasibility::Request> request,
      std::shared_ptr<mobile_manipulator_msgs::srv::CheckTaskFeasibility::Response> response)
  {
    const auto validation_request =
        build_validation_request(request->pose, request->frame_id, request->mode, request->base_pose_valid,
                                 request->base_pose, request->base_frame_id);
    const auto result = pose_validator_->validate(validation_request);
    copy_validation_result(result, *response);
  }

  /**
   * @brief Handle a request to find a feasible base pose for a given target pose.
   *
   * @param request Service request containing the target pose and parameters.
   * @param response Service response to populate with the feasibility result.
   */
  void handle_find_feasible_pose(const std::shared_ptr<mobile_manipulator_msgs::srv::FindFeasiblePose::Request> request,
                                 std::shared_ptr<mobile_manipulator_msgs::srv::FindFeasiblePose::Response> response)
  {
    const auto validation_request =
        build_validation_request(request->pose, request->frame_id, request->mode, request->base_pose_valid,
                                 request->base_pose, request->base_frame_id);

    const auto selected =
        pose_selector_->select(pose_generator_->generate(validation_request), validation_request,
                               [this, &validation_request](const mobile_manipulator_base::CandidatePose& candidate)
                                   -> std::optional<mobile_manipulator_base::PoseValidationResult> {
                                 auto candidate_request = validation_request;
                                 candidate_request.base_pose_valid = true;
                                 candidate_request.base_pose = candidate.base_pose;
                                 candidate_request.base_frame_id = candidate.base_frame_id;
                                 return pose_validator_->validate(candidate_request);
                               });

    if (!selected)
    {
      response->feasible = false;
      response->mode_used = request->mode;
      response->failure_reason = "no_feasible_candidate";
      response->suggested_fallback = "move_base_then_arm";
      response->message = "No generated candidate base pose produced a feasible arm target.";
      return;
    }

    copy_validation_result(selected->validation, *response);
  }

  /**
   * @brief Handle a new ExecuteTaskPose action goal.
   *
   * @param uuid Unique identifier for the action goal.
   * @param goal Shared pointer to the action goal.
   * @return Goal response indicating acceptance or rejection of the goal.
   */
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const ExecuteTaskPose::Goal>)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  /**
   * @brief Handle a request to cancel an ongoing ExecuteTaskPose action.
   *
   * @param goal_handle Handle to the current action goal.
   * @return Cancel response indicating acceptance or rejection of the cancel request.
   */
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleExecuteTaskPose>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  /**
   * @brief Handle an accepted ExecuteTaskPose action goal by starting its execution in a new thread.
   *
   * @param goal_handle Handle to the accepted action goal.
   */
  void handle_accepted(const std::shared_ptr<GoalHandleExecuteTaskPose> goal_handle)
  {
    std::thread(&MobileManipulatorServer::execute_task_pose, this, goal_handle).detach();
  }

  /**
   * @brief Execute the task pose action by validating and executing the approved solution.
   *
   * @param goal_handle Handle to the current action goal.
   */
  void execute_task_pose(const std::shared_ptr<GoalHandleExecuteTaskPose> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    if (!goal->validated)
    {
      auto result = std::make_shared<ExecuteTaskPose::Result>();
      result->success = false;
      result->feasible = false;
      result->mode_used = goal->mode;
      result->stage = "validation_guard";
      result->failure_reason = "pose_not_validated";
      result->message = "ExecuteTaskPose requires a validated pose and approved execution solution.";
      goal_handle->abort(result);
      return;
    }

    publish_feedback(goal_handle, "checking_feasibility",
                     "Running safety validation for the approved execution solution.");

    const auto validation_request =
        build_validation_request(goal->pose, goal->frame_id, goal->mode, goal->approved_base_pose_valid,
                                 goal->approved_base_pose, goal->approved_base_frame);
    const auto validation = pose_validator_->validate(validation_request);
    if (!validation.feasible)
    {
      auto result = result_from_validation(validation, false, "validation_guard");
      result->failure_reason =
          validation.failure_reason.empty() ? "stale_or_invalidated_pose" : validation.failure_reason;
      result->message = validation.message.empty() ? "Approved pose failed safety validation." : validation.message;
      goal_handle->abort(result);
      return;
    }

    const mobile_manipulator_base::ExecutionRequest execution_request{
      goal->pose,
      goal->frame_id,
      goal->mode,
      goal->approved_base_pose_valid,
      goal->approved_base_pose,
      goal->approved_base_frame,
    };

    const auto execution_result = coordinator_->execute(
        execution_request, [this, &goal_handle](const std::string& stage, const std::string& message) {
          publish_feedback(goal_handle, stage, message);
        });

    auto result = result_from_validation(validation, execution_result.success, execution_result.stage);
    result->failure_reason = execution_result.failure_reason;
    result->suggested_fallback = execution_result.suggested_fallback.empty() ? validation.suggested_fallback :
                                                                               execution_result.suggested_fallback;
    result->message = execution_result.message;
    result->base_target_pose_valid = execution_result.base_pose_valid;
    result->base_target_pose = execution_result.base_pose;
    result->base_target_frame = execution_result.base_frame;

    if (execution_result.success)
    {
      goal_handle->succeed(result);
      return;
    }
    goal_handle->abort(result);
  }

  /**
   * @brief Convert a PoseValidationResult into an ExecuteTaskPose::Result for action feedback.
   *
   * @param validation PoseValidationResult to convert.
   * @param success Whether the overall execution was successful.
   * @param stage Current stage of execution.
   * @return Shared pointer to the constructed ExecuteTaskPose::Result.
   */
  std::shared_ptr<ExecuteTaskPose::Result>
  result_from_validation(const mobile_manipulator_base::PoseValidationResult& validation, const bool success,
                         const std::string& stage) const
  {
    auto result = std::make_shared<ExecuteTaskPose::Result>();
    result->success = success;
    result->feasible = validation.feasible;
    result->mode_used = validation.mode_used;
    result->stage = stage;
    result->failure_reason = validation.failure_reason;
    result->suggested_fallback = validation.suggested_fallback;
    result->message = validation.message;
    result->planning_frame = validation.planning_frame;
    result->planning_pose = validation.planning_pose;
    result->planning_pose_valid = validation.planning_pose_valid;
    result->base_target_pose = validation.base_target_pose;
    result->base_target_frame = validation.base_target_frame;
    result->base_target_pose_valid = validation.base_target_pose_valid;
    result->candidate_score = validation.candidate_score;
    return result;
  }

  /**
   * @brief Publish feedback for an ongoing ExecuteTaskPose action.
   *
   * @param goal_handle Handle to the current action goal.
   * @param stage Current stage of execution.
   * @param message Human-readable message describing the current state.
   */
  void publish_feedback(const std::shared_ptr<GoalHandleExecuteTaskPose>& goal_handle, const std::string& stage,
                        const std::string& message)
  {
    auto feedback = std::make_shared<ExecuteTaskPose::Feedback>();
    feedback->stage = stage;
    feedback->message = message;
    goal_handle->publish_feedback(feedback);
  }

  pluginlib::ClassLoader<mobile_manipulator_base::PoseValidator> pose_validator_loader_;
  pluginlib::ClassLoader<mobile_manipulator_base::PoseGenerator> pose_generator_loader_;
  pluginlib::ClassLoader<mobile_manipulator_base::PoseSelector> pose_selector_loader_;
  pluginlib::ClassLoader<mobile_manipulator_base::Coordinator> coordinator_loader_;
  std::shared_ptr<mobile_manipulator_base::PoseValidator> pose_validator_;
  std::shared_ptr<mobile_manipulator_base::PoseGenerator> pose_generator_;
  std::shared_ptr<mobile_manipulator_base::PoseSelector> pose_selector_;
  std::shared_ptr<mobile_manipulator_base::Coordinator> coordinator_;
  rclcpp::Service<mobile_manipulator_msgs::srv::CheckTaskFeasibility>::SharedPtr check_task_feasibility_service_;
  rclcpp::Service<mobile_manipulator_msgs::srv::FindFeasiblePose>::SharedPtr find_feasible_pose_service_;
  rclcpp_action::Server<ExecuteTaskPose>::SharedPtr execute_task_pose_action_server_;
};
}  // namespace mobile_manipulator_ros
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <chrono>
#include <future>
#include <memory>
#include <mobile_manipulator_msgs/action/execute_task_pose.hpp>
#include <mobile_manipulator_msgs/srv/check_task_feasibility.hpp>
#include <mobile_manipulator_msgs/srv/find_feasible_pose.hpp>
#include <string>
#include <thread>

namespace mobile_manipulator_control
{
class MobileManipulatorControlNode : public rclcpp::Node
{
public:
  using CheckTaskFeasibility = mobile_manipulator_msgs::srv::CheckTaskFeasibility;
  using FindFeasiblePose = mobile_manipulator_msgs::srv::FindFeasiblePose;
  using ExecuteTaskPose = mobile_manipulator_msgs::action::ExecuteTaskPose;
  using GoalHandleExecuteTaskPose = rclcpp_action::ClientGoalHandle<ExecuteTaskPose>;

  MobileManipulatorControlNode()
  : Node("mobile_manipulator_control_node")
  {
    declare_parameter<double>("startup_delay_sec", 2.0);
    declare_parameter<bool>("shutdown_when_done", true);
    declare_parameter<bool>("execute_after_validation", true);
    declare_parameter<bool>("find_feasible_pose_on_failure", true);
    declare_parameter<double>("service_wait_timeout_sec", 5.0);
    declare_parameter<double>("service_response_timeout_sec", 15.0);
    declare_parameter<double>("action_wait_timeout_sec", 5.0);
    declare_parameter<double>("action_result_timeout_sec", 180.0);

    declare_parameter<std::string>("check_task_feasibility_service_name", "check_task_feasibility");
    declare_parameter<std::string>("find_feasible_pose_service_name", "find_feasible_pose");
    declare_parameter<std::string>("execute_task_pose_action_name", "execute_task_pose");

    declare_parameter<std::string>("mode", "mobile_manipulator");
    declare_parameter<std::string>("frame_id", "map");
    declare_parameter<double>("x", 0.5);
    declare_parameter<double>("y", 0.0);
    declare_parameter<double>("z", 0.3);
    declare_parameter<double>("qx", 0.0);
    declare_parameter<double>("qy", 0.0);
    declare_parameter<double>("qz", 0.0);
    declare_parameter<double>("qw", 1.0);

    declare_parameter<bool>("base_pose_valid", false);
    declare_parameter<std::string>("base_frame_id", "map");
    declare_parameter<double>("base_x", 0.0);
    declare_parameter<double>("base_y", 0.0);
    declare_parameter<double>("base_z", 0.0);
    declare_parameter<double>("base_qx", 0.0);
    declare_parameter<double>("base_qy", 0.0);
    declare_parameter<double>("base_qz", 0.0);
    declare_parameter<double>("base_qw", 1.0);

    check_client_ = create_client<CheckTaskFeasibility>(
        get_parameter("check_task_feasibility_service_name").as_string());
    find_client_ = create_client<FindFeasiblePose>(
        get_parameter("find_feasible_pose_service_name").as_string());
    execute_client_ = rclcpp_action::create_client<ExecuteTaskPose>(
        this, get_parameter("execute_task_pose_action_name").as_string());

    worker_ = std::thread([this]() { run_workflow(); });
  }

  ~MobileManipulatorControlNode() override
  {
    if (worker_.joinable())
    {
      worker_.join();
    }
  }

private:
  CheckTaskFeasibility::Request build_request() const
  {
    CheckTaskFeasibility::Request request;
    request.pose.position.x = get_parameter("x").as_double();
    request.pose.position.y = get_parameter("y").as_double();
    request.pose.position.z = get_parameter("z").as_double();
    request.pose.orientation.x = get_parameter("qx").as_double();
    request.pose.orientation.y = get_parameter("qy").as_double();
    request.pose.orientation.z = get_parameter("qz").as_double();
    request.pose.orientation.w = get_parameter("qw").as_double();
    request.frame_id = get_parameter("frame_id").as_string();
    request.mode = get_parameter("mode").as_string();
    request.base_pose_valid = get_parameter("base_pose_valid").as_bool();
    request.base_pose.position.x = get_parameter("base_x").as_double();
    request.base_pose.position.y = get_parameter("base_y").as_double();
    request.base_pose.position.z = get_parameter("base_z").as_double();
    request.base_pose.orientation.x = get_parameter("base_qx").as_double();
    request.base_pose.orientation.y = get_parameter("base_qy").as_double();
    request.base_pose.orientation.z = get_parameter("base_qz").as_double();
    request.base_pose.orientation.w = get_parameter("base_qw").as_double();
    request.base_frame_id = get_parameter("base_frame_id").as_string();
    return request;
  }

  void run_workflow()
  {
    std::this_thread::sleep_for(std::chrono::duration<double>(get_parameter("startup_delay_sec").as_double()));

    const auto request = build_request();
    RCLCPP_INFO(get_logger(), "Testing target pose in frame '%s' with mode '%s'.", request.frame_id.c_str(),
                request.mode.c_str());

    auto check_response = call_check_service(request);
    if (!check_response)
    {
      finish(false, "CheckTaskFeasibility service call failed.");
      return;
    }
    log_validation("CheckTaskFeasibility", *check_response);

    if (check_response->feasible)
    {
      if (!get_parameter("execute_after_validation").as_bool())
      {
        finish(true, "CheckTaskFeasibility succeeded. Execution disabled by parameter.");
        return;
      }

      const bool success = send_execute_goal(request, check_response->base_target_pose_valid,
                                             check_response->base_target_pose, check_response->base_target_frame);
      finish(success, success ? "ExecuteTaskPose completed after direct feasibility validation."
                              : "ExecuteTaskPose failed after direct feasibility validation.");
      return;
    }

    if (!get_parameter("find_feasible_pose_on_failure").as_bool())
    {
      finish(false, "CheckTaskFeasibility failed and candidate search is disabled.");
      return;
    }

    auto find_response = call_find_service(request);
    if (!find_response)
    {
      finish(false, "FindFeasiblePose service call failed.");
      return;
    }
    log_validation("FindFeasiblePose", *find_response);

    if (!find_response->feasible)
    {
      finish(false, "FindFeasiblePose did not return a feasible solution.");
      return;
    }

    if (!get_parameter("execute_after_validation").as_bool())
    {
      finish(true, "FindFeasiblePose succeeded. Execution disabled by parameter.");
      return;
    }

    const bool success = send_execute_goal(request, find_response->base_target_pose_valid,
                                           find_response->base_target_pose, find_response->base_target_frame);
    finish(success, success ? "ExecuteTaskPose completed after feasible pose search."
                            : "ExecuteTaskPose failed after feasible pose search.");
  }

  CheckTaskFeasibility::Response::SharedPtr call_check_service(const CheckTaskFeasibility::Request& request)
  {
    return call_service<CheckTaskFeasibility>(check_client_, request);
  }

  FindFeasiblePose::Response::SharedPtr call_find_service(const CheckTaskFeasibility::Request& request)
  {
    FindFeasiblePose::Request find_request;
    find_request.pose = request.pose;
    find_request.frame_id = request.frame_id;
    find_request.mode = request.mode;
    find_request.base_pose_valid = request.base_pose_valid;
    find_request.base_pose = request.base_pose;
    find_request.base_frame_id = request.base_frame_id;
    return call_service<FindFeasiblePose>(find_client_, find_request);
  }

  template <typename ServiceT>
  typename ServiceT::Response::SharedPtr
  call_service(const typename rclcpp::Client<ServiceT>::SharedPtr& client, const typename ServiceT::Request& request)
  {
    const auto wait_timeout = std::chrono::duration<double>(get_parameter("service_wait_timeout_sec").as_double());
    if (!client->wait_for_service(std::chrono::duration_cast<std::chrono::nanoseconds>(wait_timeout)))
    {
      return nullptr;
    }

    auto request_ptr = std::make_shared<typename ServiceT::Request>(request);
    auto future = client->async_send_request(request_ptr);
    const auto response_timeout =
        std::chrono::duration<double>(get_parameter("service_response_timeout_sec").as_double());
    if (future.wait_for(std::chrono::duration_cast<std::chrono::nanoseconds>(response_timeout)) !=
        std::future_status::ready)
    {
      return nullptr;
    }
    return future.get();
  }

  bool send_execute_goal(const CheckTaskFeasibility::Request& request, const bool approved_base_pose_valid,
                         const geometry_msgs::msg::Pose& approved_base_pose, const std::string& approved_base_frame)
  {
    const auto action_wait_timeout =
        std::chrono::duration<double>(get_parameter("action_wait_timeout_sec").as_double());
    if (!execute_client_->wait_for_action_server(std::chrono::duration_cast<std::chrono::nanoseconds>(action_wait_timeout)))
    {
      RCLCPP_ERROR(get_logger(), "ExecuteTaskPose action server is not available.");
      return false;
    }

    ExecuteTaskPose::Goal goal;
    goal.pose = request.pose;
    goal.frame_id = request.frame_id;
    goal.mode = request.mode;
    goal.validated = true;
    goal.approved_base_pose_valid = approved_base_pose_valid;
    goal.approved_base_pose = approved_base_pose;
    goal.approved_base_frame = approved_base_frame;

    rclcpp_action::Client<ExecuteTaskPose>::SendGoalOptions goal_options;
    goal_options.feedback_callback =
        [this](GoalHandleExecuteTaskPose::SharedPtr,
               const std::shared_ptr<const ExecuteTaskPose::Feedback> feedback) {
          RCLCPP_INFO(get_logger(), "ExecuteTaskPose feedback [%s]: %s", feedback->stage.c_str(),
                      feedback->message.c_str());
        };

    auto goal_handle_future = execute_client_->async_send_goal(goal, goal_options);
    const auto response_timeout =
        std::chrono::duration<double>(get_parameter("service_response_timeout_sec").as_double());
    if (goal_handle_future.wait_for(std::chrono::duration_cast<std::chrono::nanoseconds>(response_timeout)) !=
        std::future_status::ready)
    {
      RCLCPP_ERROR(get_logger(), "Timed out while sending ExecuteTaskPose goal.");
      return false;
    }

    auto goal_handle = goal_handle_future.get();
    if (!goal_handle)
    {
      RCLCPP_ERROR(get_logger(), "ExecuteTaskPose goal was rejected.");
      return false;
    }

    auto result_future = execute_client_->async_get_result(goal_handle);
    const auto result_timeout =
        std::chrono::duration<double>(get_parameter("action_result_timeout_sec").as_double());
    if (result_future.wait_for(std::chrono::duration_cast<std::chrono::nanoseconds>(result_timeout)) !=
        std::future_status::ready)
    {
      RCLCPP_ERROR(get_logger(), "Timed out while waiting for ExecuteTaskPose result.");
      return false;
    }

    const auto wrapped_result = result_future.get();
    if (!wrapped_result.result)
    {
      RCLCPP_ERROR(get_logger(), "ExecuteTaskPose returned no result payload.");
      return false;
    }

    RCLCPP_INFO(get_logger(), "ExecuteTaskPose result: success=%s stage=%s message=%s",
                wrapped_result.result->success ? "true" : "false", wrapped_result.result->stage.c_str(),
                wrapped_result.result->message.c_str());
    return wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED && wrapped_result.result->success;
  }

  template <typename ResponseT>
  void log_validation(const std::string& label, const ResponseT& response)
  {
    RCLCPP_INFO(get_logger(), "%s: feasible=%s mode_used=%s failure_reason=%s fallback=%s message=%s",
                label.c_str(), response.feasible ? "true" : "false", response.mode_used.c_str(),
                response.failure_reason.c_str(), response.suggested_fallback.c_str(), response.message.c_str());
    if (response.base_target_pose_valid)
    {
      RCLCPP_INFO(get_logger(), "%s base target: frame=%s x=%.3f y=%.3f z=%.3f score=%.3f", label.c_str(),
                  response.base_target_frame.c_str(), response.base_target_pose.position.x,
                  response.base_target_pose.position.y, response.base_target_pose.position.z,
                  response.candidate_score);
    }
  }

  void finish(const bool success, const std::string& message)
  {
    if (success)
    {
      RCLCPP_INFO(get_logger(), "%s", message.c_str());
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "%s", message.c_str());
    }

    if (get_parameter("shutdown_when_done").as_bool())
    {
      rclcpp::shutdown();
    }
  }

  rclcpp::Client<CheckTaskFeasibility>::SharedPtr check_client_;
  rclcpp::Client<FindFeasiblePose>::SharedPtr find_client_;
  rclcpp_action::Client<ExecuteTaskPose>::SharedPtr execute_client_;
  std::thread worker_;
};
}  // namespace mobile_manipulator_control

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mobile_manipulator_control::MobileManipulatorControlNode>());
  rclcpp::shutdown();
  return 0;
}
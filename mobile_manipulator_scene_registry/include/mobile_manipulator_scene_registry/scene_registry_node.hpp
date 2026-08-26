#pragma once

#include <grasping_msgs/action/activate_scene.hpp>
#include <grasping_msgs/action/activate_scene_by_name.hpp>
#include <grasping_msgs/msg/active_scene.hpp>
#include <grasping_msgs/msg/scene_descriptor.hpp>
#include <grasping_msgs/srv/get_active_scene.hpp>
#include <grasping_msgs/srv/list_available_scenes.hpp>
#include <mobile_manipulator_base/scene_selector.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace mobile_manipulator_scene_registry
{
/**
 * @brief Runtime node that owns the mobile-manipulator scene registry.
 *
 * The registry node loads scenes.yaml, exposes listing and activation APIs,
 * delegates scene selection policy to a SceneSelector plugin, and forwards the
 * chosen scene reference to the grasping-side ActivateScene action.
 */
class SceneRegistryNode : public rclcpp::Node
{
public:
    using ActivateScene = grasping_msgs::action::ActivateScene;
    using ActivateSceneByName = grasping_msgs::action::ActivateSceneByName;
  using GoalHandleActivateSceneByName = rclcpp_action::ServerGoalHandle<ActivateSceneByName>;

  /**
   * @brief Construct the scene registry node and declare its public ROS APIs.
   */
  SceneRegistryNode();

private:
  /**
   * @brief Load or reload the scene descriptors from scenes.yaml.
   *
   * @return True when the registry file was parsed successfully.
   */
  bool reload_registry();

  /**
   * @brief Resolve the configured scenes.yaml path.
   *
   * @return Absolute file path used for registry loading.
   */
  std::string resolve_scenes_file() const;

  /**
   * @brief Return the currently active scene name if the grasping scene manager reports one.
   *
   * @return Current active scene name, or an empty string when unavailable.
   */
  std::string current_active_scene_name();

  /**
   * @brief Handle ListAvailableScenes requests against the in-memory registry.
   *
   * @param request Filter request.
   * @param response Filtered result list.
   */
  void handle_list_available_scenes(
    const std::shared_ptr<grasping_msgs::srv::ListAvailableScenes::Request> request,
    std::shared_ptr<grasping_msgs::srv::ListAvailableScenes::Response> response);

  /**
   * @brief Accept activation goals and defer work to the selector plus scene-manager client.
   *
   * @param uuid Goal identifier.
   * @param goal Requested scene name and filters.
   * @return Goal acceptance decision.
   */
  rclcpp_action::GoalResponse handle_activate_scene_by_name_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const ActivateSceneByName::Goal> goal);

  /**
   * @brief Accept cancellation for active scene-activation goals.
   *
   * @param goal_handle Active goal handle.
   * @return Cancel acceptance decision.
   */
  rclcpp_action::CancelResponse handle_activate_scene_by_name_cancel(
      const std::shared_ptr<GoalHandleActivateSceneByName> goal_handle);

  /**
   * @brief Start execution of an accepted ActivateSceneByName goal.
   *
   * @param goal_handle Accepted goal handle.
   */
  void handle_activate_scene_by_name_accepted(const std::shared_ptr<GoalHandleActivateSceneByName> goal_handle);

  /**
   * @brief Execute the selector and activation flow for one goal.
   *
   * @param goal_handle Accepted goal handle.
   */
  void execute_activate_scene_by_name(const std::shared_ptr<GoalHandleActivateSceneByName> goal_handle);

  /**
   * @brief Select one registry entry for activation.
   *
   * @param goal Incoming activation goal.
   * @return Selected scene descriptor, or std::nullopt when no scene matches.
   */
    std::optional<grasping_msgs::msg::SceneDescriptor> select_scene(
      const ActivateSceneByName::Goal& goal) const;

  /**
   * @brief Forward a selected scene reference to the grasping-side ActivateScene action.
   *
   * @param selected_scene Registry entry chosen for activation.
   * @param force_reload Whether the grasping side should skip active-scene reuse.
   * @return Result returned by the grasping-side action client.
   */
  std::shared_ptr<typename ActivateScene::Result> forward_activate_scene(
    const grasping_msgs::msg::SceneDescriptor& selected_scene,
      bool force_reload);

  /**
   * @brief Publish feedback from this node while a scene is being selected or activated.
   *
   * @param goal_handle Active goal handle.
   * @param stage Short machine-readable stage name.
   * @param message Human-readable progress message.
   */
  static void publish_feedback(
      const std::shared_ptr<GoalHandleActivateSceneByName>& goal_handle,
      const std::string& stage,
      const std::string& message);

  pluginlib::ClassLoader<mobile_manipulator_base::SceneSelector> selector_loader_;
  std::shared_ptr<mobile_manipulator_base::SceneSelector> selector_;
    std::vector<grasping_msgs::msg::SceneDescriptor> scenes_;
    rclcpp::Service<grasping_msgs::srv::ListAvailableScenes>::SharedPtr list_available_scenes_service_;
  rclcpp_action::Server<ActivateSceneByName>::SharedPtr activate_scene_by_name_action_server_;
  rclcpp_action::Client<ActivateScene>::SharedPtr activate_scene_client_;
  rclcpp::Client<grasping_msgs::srv::GetActiveScene>::SharedPtr get_active_scene_client_;
};
}  // namespace mobile_manipulator_scene_registry
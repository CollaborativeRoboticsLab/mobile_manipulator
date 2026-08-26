#pragma once

#include <grasping_msgs/msg/scene_descriptor.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace mobile_manipulator_base
{
/**
 * @brief Request used by SceneSelector implementations.
 *
 * The scene-registry node constructs this request from the incoming
 * ActivateSceneByName goal or other local policy inputs.
 */
struct SceneSelectionRequest
{
  std::string requested_scene_name;
  std::string requested_mode;
  std::vector<std::string> required_tags;
  std::string current_scene_name;
  bool allow_active_scene_reuse{ true };
};

/**
 * @brief Result returned by a SceneSelector implementation.
 *
 * When matched is false, failure_reason and message should explain why the
 * selector could not choose a scene from the supplied registry entries.
 */
struct SceneSelectionResult
{
  bool matched{ false };
  std::string failure_reason;
  std::string message;
  grasping_msgs::msg::SceneDescriptor selected_scene;
};

/**
 * @brief Interface for choosing a scene-registry entry for activation.
 *
 * This interface owns selection policy only. Implementations must not parse
 * workspace.yaml documents or mutate the MoveIt planning scene directly.
 */
class SceneSelector
{
public:
  virtual ~SceneSelector() = default;

  /**
   * @brief Configure the selector from a node-owned parameter namespace.
   *
   * @param node Runtime node that owns parameters and ROS resources.
   * @param parameter_prefix Prefix used when declaring and reading plugin parameters.
   */
  virtual void configure(rclcpp::Node* node, const std::string& parameter_prefix) = 0;

  /**
   * @brief Select one scene descriptor from the registry candidates.
   *
   * @param scenes Registry entries loaded from scenes.yaml.
   * @param request Selection request built from scene name, mode, tags, and current state.
   * @return Selected registry entry, or std::nullopt when no entry matches.
   */
  virtual std::optional<SceneSelectionResult> select(
      const std::vector<grasping_msgs::msg::SceneDescriptor>& scenes,
      const SceneSelectionRequest& request) = 0;
};
}  // namespace mobile_manipulator_base
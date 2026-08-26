#pragma once

#include <mobile_manipulator_base/scene_selector.hpp>

namespace mobile_manipulator_scene_selector
{
/**
 * @brief Default scene selector that prefers exact name matches and then tag-compatible entries.
 */
class ExactNameThenTagsSelector : public mobile_manipulator_base::SceneSelector
{
public:
  /**
   * @brief Configure the selector from node parameters.
   *
   * @param node Runtime node that owns parameters and ROS resources.
   * @param parameter_prefix Prefix used when declaring and reading plugin parameters.
   */
  void configure(rclcpp::Node* node, const std::string& parameter_prefix) override;

  /**
   * @brief Select the best matching registry scene.
   *
   * @param scenes Registry entries loaded from scenes.yaml.
   * @param request Selection request built from scene name, mode, tags, and current state.
   * @return Selected scene descriptor, or std::nullopt when no scene matches.
   */
  std::optional<mobile_manipulator_base::SceneSelectionResult> select(
      const std::vector<grasping_msgs::msg::SceneDescriptor>& scenes,
      const mobile_manipulator_base::SceneSelectionRequest& request) override;

private:
  /**
   * @brief Return whether a scene supports the requested mode.
   */
  static bool mode_matches(const grasping_msgs::msg::SceneDescriptor& scene, const std::string& requested_mode);

  /**
   * @brief Return whether a scene contains all required semantic tags.
   */
  static bool tags_match(const grasping_msgs::msg::SceneDescriptor& scene, const std::vector<std::string>& required_tags);

  /**
   * @brief Return a normalized scene name used for string comparisons.
   */
  static std::string normalized_name(const std::string& value);

  bool prefer_active_scene_reuse_{ true };
};
}  // namespace mobile_manipulator_scene_selector
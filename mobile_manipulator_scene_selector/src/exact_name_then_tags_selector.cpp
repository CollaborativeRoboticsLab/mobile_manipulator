#include <mobile_manipulator_scene_selector/exact_name_then_tags_selector.hpp>

#include <algorithm>
#include <cctype>
#include <pluginlib/class_list_macros.hpp>

namespace mobile_manipulator_scene_selector
{
void ExactNameThenTagsSelector::configure(rclcpp::Node* node, const std::string& parameter_prefix)
{
  const auto parameter_name = parameter_prefix.empty() ? "prefer_active_scene_reuse" : parameter_prefix + ".prefer_active_scene_reuse";
  prefer_active_scene_reuse_ = node->declare_parameter<bool>(parameter_name, true);
}

std::optional<mobile_manipulator_base::SceneSelectionResult> ExactNameThenTagsSelector::select(
    const std::vector<grasping_msgs::msg::SceneDescriptor>& scenes,
    const mobile_manipulator_base::SceneSelectionRequest& request)
{
  std::vector<const grasping_msgs::msg::SceneDescriptor*> candidates;
  candidates.reserve(scenes.size());
  for (const auto& scene : scenes)
  {
    candidates.push_back(&scene);
  }

  const auto requested_name = normalized_name(request.requested_scene_name);
  if (!requested_name.empty())
  {
    candidates.erase(
        std::remove_if(candidates.begin(), candidates.end(), [&requested_name](const auto* scene) {
          return normalized_name(scene->reference.scene_name) != requested_name;
        }),
        candidates.end());
  }

  candidates.erase(
      std::remove_if(candidates.begin(), candidates.end(), [&request](const auto* scene) {
        return !mode_matches(*scene, request.requested_mode) || !tags_match(*scene, request.required_tags);
      }),
      candidates.end());

  if (candidates.empty())
  {
    return std::nullopt;
  }

  if (prefer_active_scene_reuse_ && request.allow_active_scene_reuse)
  {
    const auto current_name = normalized_name(request.current_scene_name);
    if (!current_name.empty())
    {
      const auto reuse_it = std::find_if(candidates.begin(), candidates.end(), [&current_name](const auto* scene) {
        return normalized_name(scene->reference.scene_name) == current_name;
      });
      if (reuse_it != candidates.end())
      {
        return mobile_manipulator_base::SceneSelectionResult{ true, "", "Reusing currently active matching scene.", **reuse_it };
      }
    }
  }

  return mobile_manipulator_base::SceneSelectionResult{ true, "", "Selected matching scene descriptor.", *candidates.front() };
}

bool ExactNameThenTagsSelector::mode_matches(
    const grasping_msgs::msg::SceneDescriptor& scene,
    const std::string& requested_mode)
{
  if (requested_mode.empty())
  {
    return true;
  }
  return std::find(scene.allowed_modes.begin(), scene.allowed_modes.end(), requested_mode) != scene.allowed_modes.end();
}

bool ExactNameThenTagsSelector::tags_match(
    const grasping_msgs::msg::SceneDescriptor& scene,
    const std::vector<std::string>& required_tags)
{
  for (const auto& required_tag : required_tags)
  {
    if (std::find(scene.semantic_tags.begin(), scene.semantic_tags.end(), required_tag) == scene.semantic_tags.end())
    {
      return false;
    }
  }
  return true;
}

std::string ExactNameThenTagsSelector::normalized_name(const std::string& value)
{
  std::string normalized = value;
  std::transform(normalized.begin(), normalized.end(), normalized.begin(), [](unsigned char character) {
    return static_cast<char>(std::tolower(character));
  });
  return normalized;
}
}  // namespace mobile_manipulator_scene_selector

PLUGINLIB_EXPORT_CLASS(mobile_manipulator_scene_selector::ExactNameThenTagsSelector, mobile_manipulator_base::SceneSelector)
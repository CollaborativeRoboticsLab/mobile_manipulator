#include <mobile_manipulator_scene_registry/scene_registry_node.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <future>
#include <thread>

namespace
{
std::vector<std::string> yaml_string_list(const YAML::Node& node)
{
  std::vector<std::string> values;
  if (!node || !node.IsSequence())
  {
    return values;
  }
  for (const auto& entry : node)
  {
    values.push_back(entry.as<std::string>(""));
  }
  return values;
}
}  // namespace

namespace mobile_manipulator_scene_registry
{
SceneRegistryNode::SceneRegistryNode()
  : Node("scene_registry_node")
  , selector_loader_("mobile_manipulator_base", "mobile_manipulator_base::SceneSelector")
{
  declare_parameter<std::string>("scenes_file", "scenes.yaml");
  declare_parameter<std::string>("selector_plugin", "mobile_manipulator_scene_selector::ExactNameThenTagsSelector");
  declare_parameter<std::string>("selector_parameter_prefix", "SceneSelector");
  declare_parameter<std::string>("activate_scene_action_name", "activate_scene");
  declare_parameter<std::string>("get_active_scene_service_name", "get_active_scene");
  declare_parameter<bool>("reload_registry_on_request", true);

  const auto selector_plugin = get_parameter("selector_plugin").as_string();
  selector_ = selector_loader_.createSharedInstance(selector_plugin);
  selector_->configure(this, get_parameter("selector_parameter_prefix").as_string());

  list_available_scenes_service_ = create_service<grasping_msgs::srv::ListAvailableScenes>(
      "list_available_scenes",
      std::bind(&SceneRegistryNode::handle_list_available_scenes, this, std::placeholders::_1, std::placeholders::_2));
  activate_scene_by_name_action_server_ = rclcpp_action::create_server<ActivateSceneByName>(
      this,
      "activate_scene_by_name",
      std::bind(&SceneRegistryNode::handle_activate_scene_by_name_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&SceneRegistryNode::handle_activate_scene_by_name_cancel, this, std::placeholders::_1),
      std::bind(&SceneRegistryNode::handle_activate_scene_by_name_accepted, this, std::placeholders::_1));
  activate_scene_client_ = rclcpp_action::create_client<ActivateScene>(
      this, get_parameter("activate_scene_action_name").as_string());
  get_active_scene_client_ = create_client<grasping_msgs::srv::GetActiveScene>(
      get_parameter("get_active_scene_service_name").as_string());

  if (!reload_registry())
  {
    RCLCPP_WARN(get_logger(), "Scene registry started without any loaded scene descriptors.");
  }
}

bool SceneRegistryNode::reload_registry()
{
  scenes_.clear();
  const auto scenes_file = resolve_scenes_file();
  YAML::Node root;
  try
  {
    root = YAML::LoadFile(scenes_file);
  }
  catch (const std::exception& exception)
  {
    RCLCPP_ERROR(get_logger(), "Failed to parse %s: %s", scenes_file.c_str(), exception.what());
    return false;
  }

  const auto scenes_node = root["scenes"];
  if (!scenes_node || !scenes_node.IsSequence())
  {
    RCLCPP_WARN(get_logger(), "Scene registry file %s does not contain a 'scenes' sequence.", scenes_file.c_str());
    return false;
  }

  for (const auto& scene_node : scenes_node)
  {
    grasping_msgs::msg::SceneDescriptor descriptor;
    descriptor.reference.scene_name = scene_node["name"].as<std::string>("");
    descriptor.reference.package_name = scene_node["package_name"].as<std::string>("");
    descriptor.reference.workspace_file = scene_node["workspace_file"].as<std::string>("");
    descriptor.reference.scene_revision = scene_node["scene_revision"].as<std::string>("");
    descriptor.station_frame = scene_node["station_frame"].as<std::string>("");
    descriptor.semantic_tags = yaml_string_list(scene_node["semantic_tags"]);
    descriptor.allowed_modes = yaml_string_list(scene_node["allowed_modes"]);
    descriptor.selector_profile = scene_node["selector_profile"].as<std::string>("");
    descriptor.description = scene_node["description"].as<std::string>("");
    if (!descriptor.reference.scene_name.empty() && !descriptor.reference.workspace_file.empty())
    {
      scenes_.push_back(descriptor);
    }
  }

  RCLCPP_INFO(get_logger(), "Loaded %zu scene descriptors from %s.", scenes_.size(), scenes_file.c_str());
  return !scenes_.empty();
}

std::string SceneRegistryNode::resolve_scenes_file() const
{
  const auto configured = get_parameter("scenes_file").as_string();
  const std::filesystem::path configured_path(configured);
  if (configured_path.is_absolute())
  {
    return configured;
  }
  return ament_index_cpp::get_package_share_directory("mobile_manipulator_scene_registry") + "/config/" + configured;
}

std::string SceneRegistryNode::current_active_scene_name()
{
  if (!get_active_scene_client_->wait_for_service(std::chrono::seconds(1)))
  {
    return "";
  }
  auto request = std::make_shared<grasping_msgs::srv::GetActiveScene::Request>();
  auto future = get_active_scene_client_->async_send_request(request);
  if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready)
  {
    return "";
  }
  const auto response = future.get();
  if (!response || !response->active)
  {
    return "";
  }
  return response->active_scene.reference.scene_name;
}

void SceneRegistryNode::handle_list_available_scenes(
  const std::shared_ptr<grasping_msgs::srv::ListAvailableScenes::Request> request,
  std::shared_ptr<grasping_msgs::srv::ListAvailableScenes::Response> response)
{
  if (get_parameter("reload_registry_on_request").as_bool())
  {
    reload_registry();
  }

  response->scenes.clear();
  for (const auto& scene : scenes_)
  {
    if (!request->requested_mode.empty() &&
        std::find(scene.allowed_modes.begin(), scene.allowed_modes.end(), request->requested_mode) == scene.allowed_modes.end())
    {
      continue;
    }

    bool tags_ok = true;
    for (const auto& required_tag : request->required_tags)
    {
      if (std::find(scene.semantic_tags.begin(), scene.semantic_tags.end(), required_tag) == scene.semantic_tags.end())
      {
        tags_ok = false;
        break;
      }
    }
    if (tags_ok)
    {
      response->scenes.push_back(scene);
    }
  }
  response->message = response->scenes.empty() ? "No scene descriptors matched the requested filters." : "ok";
}

rclcpp_action::GoalResponse SceneRegistryNode::handle_activate_scene_by_name_goal(
    const rclcpp_action::GoalUUID&, std::shared_ptr<const ActivateSceneByName::Goal>)
{
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SceneRegistryNode::handle_activate_scene_by_name_cancel(
    const std::shared_ptr<GoalHandleActivateSceneByName>)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void SceneRegistryNode::handle_activate_scene_by_name_accepted(
    const std::shared_ptr<GoalHandleActivateSceneByName> goal_handle)
{
  std::thread(&SceneRegistryNode::execute_activate_scene_by_name, this, goal_handle).detach();
}

void SceneRegistryNode::execute_activate_scene_by_name(
    const std::shared_ptr<GoalHandleActivateSceneByName> goal_handle)
{
  auto result = std::make_shared<ActivateSceneByName::Result>();

  if (get_parameter("reload_registry_on_request").as_bool())
  {
    publish_feedback(goal_handle, "reloading_registry", "Reloading scenes.yaml before selection.");
    reload_registry();
  }

  publish_feedback(goal_handle, "selecting_scene", "Selecting a scene descriptor from the registry.");
  const auto selected_scene = select_scene(*goal_handle->get_goal());
  if (!selected_scene)
  {
    result->success = false;
    result->failure_reason = "scene_not_found";
    result->message = "No scene descriptor matched the requested scene name, mode, and tags.";
    goal_handle->abort(result);
    return;
  }
  result->selected_scene = *selected_scene;

  publish_feedback(goal_handle, "activating_scene", "Forwarding the selected scene to the grasping scene manager.");
  const auto activation_result = forward_activate_scene(*selected_scene, goal_handle->get_goal()->force_reload);
  if (!activation_result || !activation_result->success)
  {
    result->success = false;
    result->failure_reason = activation_result ? activation_result->failure_reason : "activate_scene_failed";
    result->message = activation_result ? activation_result->message : "No result returned by the grasping scene manager.";
    goal_handle->abort(result);
    return;
  }

  result->success = true;
  result->failure_reason.clear();
  result->message = activation_result->message;
  result->active_scene = activation_result->active_scene;
  goal_handle->succeed(result);
}

std::optional<grasping_msgs::msg::SceneDescriptor> SceneRegistryNode::select_scene(
    const ActivateSceneByName::Goal& goal) const
{

  mobile_manipulator_base::SceneSelectionRequest request;
  request.requested_scene_name = goal.scene_name;
  request.requested_mode = goal.requested_mode;
  request.required_tags.assign(goal.required_tags.begin(), goal.required_tags.end());
  request.current_scene_name = const_cast<SceneRegistryNode*>(this)->current_active_scene_name();
  request.allow_active_scene_reuse = !goal.force_reload;

  const auto selection = selector_->select(scenes_, request);
  if (!selection || !selection->matched)
  {
    return std::nullopt;
  }
  return selection->selected_scene;
}

std::shared_ptr<SceneRegistryNode::ActivateScene::Result> SceneRegistryNode::forward_activate_scene(
    const grasping_msgs::msg::SceneDescriptor& selected_scene, bool force_reload)
{
  if (!activate_scene_client_->wait_for_action_server(std::chrono::seconds(5)))
  {
    return nullptr;
  }

  ActivateScene::Goal goal;
  goal.reference = selected_scene.reference;
  goal.force_reload = force_reload;

  auto send_future = activate_scene_client_->async_send_goal(goal);
  if (send_future.wait_for(std::chrono::seconds(10)) != std::future_status::ready)
  {
    return nullptr;
  }

  auto goal_handle = send_future.get();
  if (!goal_handle)
  {
    return nullptr;
  }

  auto result_future = activate_scene_client_->async_get_result(goal_handle);
  if (result_future.wait_for(std::chrono::seconds(30)) != std::future_status::ready)
  {
    return nullptr;
  }

  const auto wrapped_result = result_future.get();
  if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED)
  {
    return wrapped_result.result;
  }
  return wrapped_result.result;
}

void SceneRegistryNode::publish_feedback(
    const std::shared_ptr<GoalHandleActivateSceneByName>& goal_handle,
    const std::string& stage,
    const std::string& message)
{
  auto feedback = std::make_shared<ActivateSceneByName::Feedback>();
  feedback->stage = stage;
  feedback->message = message;
  goal_handle->publish_feedback(feedback);
}
}  // namespace mobile_manipulator_scene_registry

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mobile_manipulator_scene_registry::SceneRegistryNode>());
  rclcpp::shutdown();
  return 0;
}
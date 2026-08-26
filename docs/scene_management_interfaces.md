# Scene Management Interfaces

This document defines the first-pass ROS contracts and C++ plugin contract for scene selection and scene activation across the grasping and mobile-manipulator stacks.

## Ownership

- `grasping_control` owns parsing `workspace.yaml`, converting workspace geometry into MoveIt collision objects, applying allowed collisions, and publishing the active scene state.
- `mobile_manipulator` owns `scenes.yaml`, scene-name resolution, scene selection policy, and the client call into the grasping-side scene manager.
- `mobile_manipulator_ros` remains responsible for task feasibility, candidate search, and approved execution. Scene activation is kept outside that runtime.

## Grasping-Owned ROS Contracts

Add the scene-management messages, services, and actions to `grasping_msgs` so the grasping-side scene manager owns the runtime contract.

Suggested package tree:

```text
grasping_msgs/
  CMakeLists.txt
  package.xml
  msg/
    SceneReference.msg
    SceneDescriptor.msg
    ActiveScene.msg
  srv/
    GetActiveScene.srv
    ListAvailableScenes.srv
  action/
    ActivateScene.action
    ActivateSceneByName.action
```

## Message Files

### `msg/SceneReference.msg`

```text
# Stable reference to a workspace scene source.
#
# scene_name is the user-facing identifier resolved from scenes.yaml.
# package_name and workspace_file locate the grasping-side workspace.yaml.
# scene_revision is an optional source revision or content hash supplied by the caller.

string scene_name
string package_name
string workspace_file
string scene_revision
```

### `msg/SceneDescriptor.msg`

```text
# Mobile-manipulator-owned scene registry entry.
#
# This is the public representation returned by ListAvailableScenes and used by
# ActivateSceneByName to report which registry entry was selected.

grasping_msgs/SceneReference reference
string station_frame
string[] semantic_tags
string[] allowed_modes
string selector_profile
string description
```

### `msg/ActiveScene.msg`

```text
# Runtime view of the currently active scene on the grasping side.
#
# scene_handle is the runtime token returned by scene activation and used by
# downstream clients for observability. It is not a MoveIt planning pipeline id.

grasping_msgs/SceneReference reference
string scene_handle
string workspace_base_frame
bool workspace_area_enabled
geometry_msgs/Point[] workspace_area_corner_points
string[] object_names
builtin_interfaces/Time loaded_at
```

## Service Files

### `srv/GetActiveScene.srv`

```text
# Return the grasping-side active scene, if any.

---
bool active
grasping_msgs/ActiveScene active_scene
string message
```

### `srv/ListAvailableScenes.srv`

```text
# Filter the mobile-manipulator scene registry.
#
# requested_mode is optional. When set, only scenes that advertise the mode
# should be returned.
# required_tags is optional. When set, only scenes containing all required tags
# should be returned.

string requested_mode
string[] required_tags
---
grasping_msgs/SceneDescriptor[] scenes
string message
```

## Action Files

### `action/ActivateScene.action`

```text
# Grasping-side action that activates a specific workspace scene reference.
#
# The server resolves package_name + workspace_file, parses workspace.yaml,
# applies the MoveIt planning scene update, updates the active workspace-area
# filter, and publishes the new ActiveScene state.

grasping_msgs/SceneReference reference
bool force_reload
---
bool success
string failure_reason
string message
grasping_msgs/ActiveScene active_scene
---
string stage
string message
```

### `action/ActivateSceneByName.action`

```text
# Mobile-manipulator-side action that resolves a scene by name or by registry
# filters, then forwards the corresponding reference to the grasping-side
# ActivateScene action.

string scene_name
string requested_mode
string[] required_tags
bool force_reload
---
bool success
string failure_reason
string message
grasping_msgs/SceneDescriptor selected_scene
grasping_msgs/ActiveScene active_scene
---
string stage
string message
```

## Plugin Contract

The selector plugin belongs in `mobile_manipulator_base` as a new interface next to the existing pose-validator, pose-generator, pose-selector, and coordinator interfaces.

Suggested file:

### `include/mobile_manipulator_base/scene_selector.hpp`

```cpp
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
  virtual void configure(rclcpp::Node * node, const std::string & parameter_prefix) = 0;

  /**
   * @brief Select one scene descriptor from the registry candidates.
   *
   * @param scenes Registry entries loaded from scenes.yaml.
   * @param request Selection request built from scene name, mode, tags, and current state.
   * @return Selected registry entry, or std::nullopt when no entry matches.
   */
  virtual std::optional<SceneSelectionResult> select(
    const std::vector<grasping_msgs::msg::SceneDescriptor> & scenes,
    const SceneSelectionRequest & request) = 0;
};
}  // namespace mobile_manipulator_base
```

## First Selector Plugin

Recommended first implementation:

- package: `mobile_manipulator_scene_selector`
- class: `mobile_manipulator_scene_selector::ExactNameThenTagsSelector`

Expected behavior:

1. If `requested_scene_name` is set, require an exact scene-name match.
2. Filter remaining entries by `requested_mode` when provided.
3. Filter remaining entries by `required_tags`, requiring all tags.
4. Prefer the current active scene when `allow_active_scene_reuse=true` and it still matches.
5. Otherwise select the first surviving scene or a future profile-weighted best scene.

## Notes For V1

- Do not introduce `ApplyScenePatch` yet.
- Do not return a planning pipeline id from activation. Return `scene_handle` and scene metadata only.
- Keep `ListAvailableScenes` and `ActivateSceneByName` on the mobile-manipulator side because they depend on `scenes.yaml` ownership.
- Keep `GetActiveScene` and `ActivateScene` on the grasping side because they depend on the active MoveIt scene.
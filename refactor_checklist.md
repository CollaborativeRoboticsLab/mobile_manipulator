# Scene Management Refactor Checklist

This checklist turns the agreed architecture into concrete package and file changes against the current workspace layout.

## Current Status

Completed:

- Phase 1 scene contracts are in `grasping_msgs`
- Phase 2 `SceneSelector` base interface exists in `mobile_manipulator_base`
- Phase 3 `mobile_manipulator_scene_selector` exists with `ExactNameThenTagsSelector`
- Phase 4 `mobile_manipulator_scene_registry` exists, parses `scenes.yaml`, and forwards `ActivateScene`
- Phase 5 `scene_manager_core.py` exists for shared scene parsing and activation shaping
- Phase 6 `scene_manager_node.py` exposes services, actions, `/active_scene`, startup default-scene activation, and serialized activation
- Phase 7 `motion_execution_node.py` and `feasibility_service_node.py` now consume active-scene state from the scene manager
- Phase 8 launch integration is in place for grasping bringup, scene registry, mobile manipulator control, and Omron bringup

Remaining:

- deeper reuse of shared scene core from `workspace_creation_node.py`
- focused tests for selector, registry parsing, scene-manager validation, and active-scene propagation
- end-to-end runtime validation on the full ROS graph and robot or representative simulation

## Phase 1: Add Grasping-Owned Scene Contracts

Add files under [grasping_msgs](../../grasping/grasping_msgs):

- `msg/SceneReference.msg`
- `msg/SceneDescriptor.msg`
- `msg/ActiveScene.msg`
- `srv/GetActiveScene.srv`
- `srv/ListAvailableScenes.srv`
- `action/ActivateScene.action`
- `action/ActivateSceneByName.action`

Update [CMakeLists.txt](../../grasping/grasping_msgs/CMakeLists.txt) and [package.xml](../../grasping/grasping_msgs/package.xml):

- add `geometry_msgs` dependency
- add `builtin_interfaces` dependency
- add `rosidl_default_generators`
- export `rosidl_default_runtime`

Validation:

- build `grasping_msgs` alone
- verify generated headers and Python bindings exist

## Phase 2: Add Scene-Selector Base Interface

Modify [mobile_manipulator_base/CMakeLists.txt](../mobile_manipulator_base/CMakeLists.txt):

- add dependency on `grasping_msgs`

Add file:

- `include/mobile_manipulator_base/scene_selector.hpp`

Optional supporting file if the request/result structs grow:

- `include/mobile_manipulator_base/scene_types.hpp`

Validation:

- build `mobile_manipulator_base`
- verify downstream packages can include the new interface

## Phase 3: Create Scene-Selector Plugin Package

Create a new package under `src/mobile_manipulator`:

```text
mobile_manipulator_scene_selector/
  CMakeLists.txt
  package.xml
  plugins.xml
  include/mobile_manipulator_scene_selector/exact_name_then_tags_selector.hpp
  src/exact_name_then_tags_selector.cpp
```

First plugin:

- class `mobile_manipulator_scene_selector::ExactNameThenTagsSelector`

Responsibilities:

- exact scene-name matching
- requested-mode filtering
- all-tags filtering
- optional reuse of current active scene

Non-responsibilities:

- no `workspace.yaml` parsing
- no MoveIt planning-scene mutation
- no allowed-collision computation

Validation:

- build plugin package
- verify pluginlib can discover the plugin

## Phase 4: Create Mobile-Manipulator Scene Registry Node

Create a new package under `src/mobile_manipulator`:

```text
mobile_manipulator_scene_registry/
  CMakeLists.txt
  package.xml
  config/scenes.yaml
  include/mobile_manipulator_scene_registry/scene_registry_node.hpp
  src/scene_registry_node.cpp
  launch/scene_registry.launch.py
```

Node responsibilities:

- load `scenes.yaml`
- expose `ListAvailableScenes`
- expose `ActivateSceneByName`
- host a `SceneSelector` plugin
- call grasping-side `ActivateScene`
- relay the returned `ActiveScene`

Recommended parameters:

- `scenes_file`
- `activate_scene_action_name`
- `selector_plugin`
- `selector_parameter_prefix`

Validation:

- test list operation without grasping-side runtime
- test activation against a stub or live `scene_manager_node`

## Phase 5: Refactor Shared Grasping Scene Logic

Current files to refactor from:

- [motion_execution_node.py](../../grasping/grasping_control/grasping_control/motion_execution_node.py)
- [feasibility_service_node.py](../../grasping/grasping_control/grasping_control/feasibility_service_node.py)
- [workspace_creation_node.py](../../grasping/grasping_control/grasping_control/workspace_creation_node.py)
- [workspace_utils.py](../../grasping/grasping_control/grasping_control/workspace_utils.py)

Add files in `grasping_control/grasping_control/`:

- `scene_manager_core.py`
- `scene_manager_node.py`

Move into `scene_manager_core.py`:

- workspace-document loading and validation
- collision-object derivation
- allowed-collision derivation
- active-scene metadata construction
- scene-handle generation

Keep in [workspace_creation_node.py](../../grasping/grasping_control/grasping_control/workspace_creation_node.py):

- interactive CLI capture flow
- calibration prompts
- operator-driven save/update loop

But change it to reuse the shared scene core for:

- workspace validation
- persistence shaping
- derived geometry normalization

Validation:

- existing workspace-creation behavior still works
- no behavior regression in derived `workspace.yaml` output

## Phase 6: Add Grasping Scene Manager Runtime Node

Modify [setup.py](../../grasping/grasping_control/setup.py):

- add console script entry for `scene_manager_node`

Add file:

- `launch/scene_manager.launch.py`

Status:

- current implementation starts `scene_manager_node` from `motion_execution.launch.py` rather than from a separate dedicated launch file

Node responsibilities:

- expose `GetActiveScene`
- expose `ValidateWorkspaceDocument`
- expose `ActivateScene`
- optionally expose `LoadSceneFromContent`
- publish `/active_scene`
- serialize scene activation requests
- own singleton active scene state

Recommended parameters:

- `apply_planning_scene_service`
- `get_planning_scene_service`
- `active_scene_topic`
- `default_scene_package`
- `default_workspace_file`

Validation:

- load a known workspace from package share
- verify collision objects appear in MoveIt
- verify active-scene metadata is published

## Phase 7: Make Arm Runtime Consume Active Scene

Modify [motion_execution_node.py](../../grasping/grasping_control/grasping_control/motion_execution_node.py):

- stop treating launch-loaded workspace ROS parameters as the long-term source of truth
- remove or shrink `_load_workspace_into_planning_scene` as the primary activation path
- subscribe to `/active_scene`
- query `GetActiveScene` at startup
- update local workspace-area filter from active-scene data

Modify [feasibility_service_node.py](../../grasping/grasping_control/grasping_control/feasibility_service_node.py):

- query `GetActiveScene` at startup or subscribe to scene updates if needed
- use active workspace-area state instead of independent launch-loaded workspace ownership

Keep local node responsibilities unchanged:

- transform handling
- IK requests
- MoveIt planning requests
- action or service execution

Validation:

- activate a scene, then verify pose rejection follows the active workspace area
- swap scenes and verify new workspace-area filtering is observed without restart

## Phase 8: Update Launch Integration

Modify [motion_execution.launch.py](../../grasping/grasping_control/launch/motion_execution.launch.py):

- keep `motion_config_file`
- phase out `workspace_file` as the primary runtime source of truth
- optionally support a bootstrap default-scene activation path

Modify Omron wrapper launches:

- [ld250_tm12x_soft_two_fingers.launch.py](../../grasping_omron_moma/grasping_omron_moma/launch/ld250_tm12x_soft_two_fingers.launch.py)
- [omron_task_feasibility_stack.launch.py](../../grasping_omron_moma/grasping_omron_moma/launch/omron_task_feasibility_stack.launch.py)

Add:

- `scene_manager_node`
- mobile-manipulator `scene_registry_node` where appropriate

Status:

- completed in the current launch stack

Validation:

- combined Omron bringup still starts successfully
- default scene activates before feasibility or arm execution is used

## Phase 9: Add Docs And Tests

Update docs:

- [arm_control.md](../../grasping/docs/control/arm_control.md)
- [creation.md](../../grasping/docs/workspace/creation.md)
- [loading.md](../../grasping/docs/workspace/loading.md)
- [server.md](server.md)
- [pose_selector.md](pose_selector.md)
- [ros2_interfaces.md](ros2_interfaces.md)

Add tests:

- scene-registry parsing tests
- scene-selector plugin tests
- scene-manager validation tests
- scene activation integration test against MoveIt scene application
- active-scene propagation test into motion execution and feasibility behavior

## Suggested Delivery Order

1. Add the scene-management contracts to `grasping_msgs`.
2. Add `SceneSelector` base interface.
3. Add `mobile_manipulator_scene_selector` plugin package.
4. Add `mobile_manipulator_scene_registry` node package.
5. Refactor grasping scene logic into shared core.
6. Add `scene_manager_node`.
7. Migrate arm runtime nodes to active-scene consumption.
8. Update launches.
9. Add tests and docs.

## Scope Guards

- Do not add `ApplyScenePatch` in v1.
- Do not parse `workspace.yaml` on the mobile-manipulator side.
- Do not return a planning pipeline id from scene activation.
- Do not embed scene activation into `CheckTaskFeasibility`, `FindFeasiblePose`, or `ExecuteTaskPose`.
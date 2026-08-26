# mobile_manipulator

Umbrella package for the generic mobile-manipulator feasibility and execution stack.

## Package split

- `mobile_manipulator_msgs`: public ROS contracts for `CheckTaskFeasibility.srv`, `FindFeasiblePose.srv`, and `ExecuteTaskPose.action`
- `mobile_manipulator_base`: shared C++ interfaces and value types for validator, generator, selector, and coordinator plugins
- `mobile_manipulator_pose_validator`: default reachability and optional costmap-aware validation plugin
- `mobile_manipulator_pose_generator`: default stand-off base-pose generator plugin
- `mobile_manipulator_pose_selector`: default least-movement candidate selector plugin
- `mobile_manipulator_coordinator`: default approved-solution execution plugin for base navigation plus arm execution
- `mobile_manipulator_ros`: runtime server node, parameter files, and launch integration
- `mobile_manipulator_control`: generic one-shot ROS 2 client node for testing the three public mobile-manipulator interfaces before using capability runners
- `mobile_manipulator_scene_selector`: scene-selection plugin package used by the scene registry to choose one scene descriptor from `scenes.yaml`
- `mobile_manipulator_scene_registry`: runtime node that owns `scenes.yaml`, exposes `ListAvailableScenes` and `ActivateSceneByName`, and forwards scene activation to the grasping stack

## Documentation

- [ROS 2 interfaces](docs/ros2_interfaces.md)
- [Runtime server](docs/server.md)
- [Shared base classes](docs/base_class.md)
- [Validator plugin](docs/pose_validator.md)
- [Generator plugin](docs/pose_generator.md)
- [Selector plugin](docs/pose_selector.md)
- [Coordinator plugin](docs/coordinator.md)
- [Parameters](docs/parameters.md)

## Runtime model

`mobile_manipulator_ros` exposes three public interfaces:

- `CheckTaskFeasibility`: validate a specific requested target pose, optionally from a supplied base pose
- `FindFeasiblePose`: generate candidate base poses, validate them, and return the selected feasible solution
- `ExecuteTaskPose`: execute only a validated solution, with safety revalidation before motion

The runtime node loads its validator, generator, selector, and coordinator through pluginlib. Plugin selection is parameter-driven, and plugin configuration is provided through class-keyed parameter blocks such as `GraspingPoseValidator.*` and `DefaultCoordinator.*`.

Scene activation is intentionally separate from that task-feasibility server. The mobile-manipulator scene path is:

- `mobile_manipulator_scene_registry` loads `scenes.yaml`
- `mobile_manipulator_scene_selector` chooses a matching scene entry
- the registry calls grasping-side `ActivateScene`
- `mobile_manipulator_ros` then reasons over feasibility and execution against the already active grasping scene

## Ownership boundary

- Grasping stays responsible for active planning-scene ownership, scene activation, arm-only feasibility, and arm execution primitives.
- `mobile_manipulator` owns generic arm-plus-base orchestration, scene-name resolution, and default plugins.
- `mobile_manipulator_capabilities` stays a thin capability adapter over these ROS APIs.
- Platform-specific packages such as `omron_moma_capabilities` should customize behavior with parameters first. If new plugins are needed later, they belong under the `mobile_manipulator` stack.

In practice:

- `workspace.yaml` parsing stays on the grasping side
- `scenes.yaml` ownership stays on the mobile-manipulator side
- task execution must not implicitly switch scenes

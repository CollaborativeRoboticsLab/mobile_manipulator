# Runtime Server

The `mobile_manipulator_ros` package provides the `mobile_manipulator_server` node.

## Responsibilities

The server owns three public ROS APIs:

| Responsibility area | Details |
| --- | --- |
| Public ROS APIs | `check_task_feasibility`, `find_feasible_pose`, `execute_task_pose` |
| Plugin loading | Loads validator, generator, selector, and coordinator through `pluginlib` |
| Parameter handling | Configures plugins through class-keyed parameter blocks |
| Result mapping | Converts internal validation and execution structures into ROS service and action responses |
| Safety guard | Revalidates approved solutions before execution |

## Plugin Loading Model

At startup the server:

1. Declares public ROS interface names
2. Reads plugin class parameters
3. Loads one validator, generator, selector, and coordinator
4. Derives the parameter prefix from each plugin class name
5. Calls `configure()` on each plugin

For example:

| Item | Example |
| --- | --- |
| Plugin class | `mobile_manipulator_pose_validator::GraspingPoseValidator` |
| Derived parameter prefix | `GraspingPoseValidator` |

## Service Flows

### `CheckTaskFeasibility`

The server builds a `PoseValidationRequest` from the ROS request and forwards it directly to the validator plugin.

### `FindFeasiblePose`

| Step | Behavior |
| --- | --- |
| 1 | Builds a `PoseValidationRequest` |
| 2 | Calls the generator plugin |
| 3 | Passes each candidate to the selector through a validation callback |
| 4 | Reuses the validator plugin for each candidate through that callback |
| 5 | Returns the selected feasible solution |

## Action Flow

### `ExecuteTaskPose`

| Step | Behavior |
| --- | --- |
| 1 | Rejects execution when `validated=false` |
| 2 | Revalidates the target and approved base pose through the validator plugin |
| 3 | Calls the coordinator plugin for motion |
| 4 | Publishes feedback stages while running |
| 5 | Returns the merged validation and execution result |

## Current Default Plugin Chain

| Role | Plugin |
| --- | --- |
| Validator | `GraspingPoseValidator` |
| Generator | `StandoffPoseGenerator` |
| Selector | `LeastMovementPoseSelector` |
| Coordinator | `DefaultCoordinator` |

## Launch Surface

The generic launch entry point is `mobile_manipulator_ros/launch/task_feasibility.launch.py`.

It starts one node:

| Launch file | Starts | Overridable argument |
| --- | --- | --- |
| `mobile_manipulator_ros/launch/task_feasibility.launch.py` | `mobile_manipulator_server` | `config_file` |

This allows platform wrappers to inject their own parameter YAML without copying the node definition.

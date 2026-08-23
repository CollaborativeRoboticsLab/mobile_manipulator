# ROS 2 Interfaces

The generic runtime in `mobile_manipulator_ros` exposes two services and one action.

## `CheckTaskFeasibility.srv`

Purpose:

| Aspect | Details |
| --- | --- |
| Purpose | Validate one requested target pose, optionally from a supplied base pose |

Request fields:

| Field | Type |
| --- | --- |
| `pose` | `geometry_msgs/Pose` |
| `frame_id` | `string` |
| `mode` | `string` |
| `base_pose_valid` | `bool` |
| `base_pose` | `geometry_msgs/Pose` |
| `base_frame_id` | `string` |

Response fields:

| Field | Type |
| --- | --- |
| `feasible` | `bool` |
| `mode_used` | `string` |
| `failure_reason` | `string` |
| `suggested_fallback` | `string` |
| `message` | `string` |
| `planning_frame` | `string` |
| `planning_pose` | `geometry_msgs/Pose` |
| `planning_pose_valid` | `bool` |
| `base_target_pose` | `geometry_msgs/Pose` |
| `base_target_frame` | `string` |
| `base_target_pose_valid` | `bool` |
| `candidate_score` | `float64` |

Current use:

| Current use | Meaning |
| --- | --- |
| Direct arm-only validation | Ask whether the arm can reach the target without extra base search |
| Caller-supplied base-pose validation | Ask whether the target is feasible from a specific base pose |

## `FindFeasiblePose.srv`

| Aspect | Details |
| --- | --- |
| Purpose | Generate candidate base poses, validate each candidate through the configured validator, and return the best feasible solution selected by the configured selector |

The request and response fields mirror `CheckTaskFeasibility.srv`.

If no feasible candidate is found, the server currently returns:

| Field | Value |
| --- | --- |
| `feasible` | `false` |
| `failure_reason` | `no_feasible_candidate` |
| `suggested_fallback` | `move_base_then_arm` |

## `ExecuteTaskPose.action`

| Aspect | Details |
| --- | --- |
| Purpose | Execute only an already approved solution, revalidate the target before motion, and optionally navigate first, then move the arm |

Goal fields:

| Field | Type |
| --- | --- |
| `pose` | `geometry_msgs/Pose` |
| `frame_id` | `string` |
| `mode` | `string` |
| `validated` | `bool` |
| `approved_base_pose_valid` | `bool` |
| `approved_base_pose` | `geometry_msgs/Pose` |
| `approved_base_frame` | `string` |

Result fields:

| Field | Type |
| --- | --- |
| `success` | `bool` |
| `feasible` | `bool` |
| `mode_used` | `string` |
| `stage` | `string` |
| `failure_reason` | `string` |
| `suggested_fallback` | `string` |
| `message` | `string` |
| `planning_frame` | `string` |
| `planning_pose` | `geometry_msgs/Pose` |
| `planning_pose_valid` | `bool` |
| `base_target_pose` | `geometry_msgs/Pose` |
| `base_target_frame` | `string` |
| `base_target_pose_valid` | `bool` |
| `candidate_score` | `float64` |

Feedback fields:

| Field | Type |
| --- | --- |
| `stage` | `string` |
| `message` | `string` |

## Modes

The runtime forwards the `mode` string to the validator.

In the current system, the important values are:

| Mode | Meaning |
| --- | --- |
| `arm_only_ik` | Ask for arm-only IK reachability |
| `arm_only_plan` | Ask for arm-only planning feasibility |
| `mobile_manipulator` | Allow arm-plus-base reasoning |

The exact arm-only semantics are defined by the grasping feasibility service behind the validator.

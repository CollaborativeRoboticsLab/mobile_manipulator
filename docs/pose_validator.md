# Pose Validator

The validator plugin decides whether a requested target pose is feasible in the current scene.

## Current Default Implementation

The default validator is `mobile_manipulator_pose_validator::GraspingPoseValidator`.

It delegates arm feasibility to the grasping stack and optionally checks the candidate base pose against a Nav2 costmap.

## Arm-Only Validation

When no base pose is supplied, the validator calls the grasping Cartesian feasibility service directly with:

| Input | Source |
| --- | --- |
| Pose | `request.pose` |
| Frame | `request.frame_id` |
| Mode | `request.mode` |

The response is copied into `PoseValidationResult`.

## Base-Aware Validation

When `base_pose_valid=true`, the validator:

1. Verifies that `request.frame_id` and `request.base_frame_id` are both set and equal
2. Optionally checks whether the supplied base pose is free in the costmap
3. Projects the target pose into the arm base frame
4. Calls the grasping Cartesian feasibility service using the projected pose and configured `arm_base_frame`
5. Copies the result and attaches the supplied base pose as `base_target_pose`

## Costmap Validation

When enabled, the validator subscribes to a Nav2 costmap topic and rejects candidate base poses that are occupied.

Current parameters:

| Parameter | Purpose |
| --- | --- |
| `GraspingPoseValidator.use_costmap_validation` | Enable candidate base-pose occupancy checks |
| `GraspingPoseValidator.costmap_topic` | Select the Nav2 costmap topic |
| `GraspingPoseValidator.costmap_occupied_threshold` | Set the occupancy threshold treated as blocked |

## Important Failure Reasons

The validator itself can produce:

| Failure reason | Meaning |
| --- | --- |
| `unsupported_base_frame` | Target pose and supplied base pose are not both expressed in the same frame |
| `costmap_unavailable` | Costmap validation is enabled but no costmap has been received |
| `base_pose_occupied` | The supplied candidate base pose is occupied in the current costmap |
| `service_unavailable` | The grasping Cartesian feasibility service call failed |

It can also forward machine-readable failures returned by the grasping feasibility service, such as transform, workspace, IK, or planning failures.

## Dependency on the Grasping Stack

This plugin assumes the grasping feasibility service is available under `GraspingPoseValidator.service_name`, which defaults to `check_cartesian_pose_feasibility`.

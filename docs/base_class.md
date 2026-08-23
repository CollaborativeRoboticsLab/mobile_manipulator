# Base Classes and Shared Types

The `mobile_manipulator_base` package defines the plugin interfaces and shared value types used by the runtime server in `mobile_manipulator_ros`.

## Plugin Interfaces

The runtime is split across four plugin roles:

| Interface | Responsibility |
| --- | --- |
| `PoseValidator` | Answers whether a requested target is currently feasible |
| `PoseGenerator` | Produces candidate base poses when arm-only reach is not enough |
| `PoseSelector` | Evaluates generated candidates and chooses the best feasible one |
| `Coordinator` | Executes an already approved solution |

Each interface follows the same pattern:

- `configure(rclcpp::Node*, const std::string&)` gives the plugin access to node-owned parameters and ROS resources
- a single operation method implements the role-specific behavior

The runtime server derives a parameter prefix from the plugin class name. For example, `mobile_manipulator_pose_validator::GraspingPoseValidator` is configured from the `GraspingPoseValidator.*` parameter block.

## Shared Request and Result Types

### `PoseValidationRequest`

Carries the target pose and optional supplied base pose:

| Field | Meaning |
| --- | --- |
| `pose` | Target pose to validate |
| `frame_id` | Frame of the target pose |
| `mode` | Requested validation mode |
| `base_pose_valid` | Whether a base pose is supplied |
| `base_pose` | Supplied base pose |
| `base_frame_id` | Frame of the supplied base pose |

### `PoseValidationResult`

Used across direct validation and selected candidate evaluation:

| Field | Meaning |
| --- | --- |
| `feasible` | Overall feasibility result |
| `mode_used` | Mode actually used by the validator |
| `failure_reason` | Machine-readable failure code |
| `suggested_fallback` | Next-step hint such as `move_base_then_arm` |
| `message` | Human-readable explanation |
| `planning_frame` | Frame of the validated planning pose |
| `planning_pose` | Resolved planning pose |
| `planning_pose_valid` | Whether `planning_pose` is populated |
| `base_target_pose` | Selected or supplied base pose |
| `base_target_frame` | Frame of `base_target_pose` |
| `base_target_pose_valid` | Whether a base target pose is populated |
| `candidate_score` | Selector-assigned score for a chosen candidate |

### `CandidatePose`

Represents a generated base-pose option:

| Field | Meaning |
| --- | --- |
| `base_pose` | Candidate base pose |
| `base_frame_id` | Frame of the candidate base pose |
| `score` | Generator-provided seed score |

### `EvaluatedCandidate`

Bundles:

- a `CandidatePose`
- the `PoseValidationResult` from validating that candidate

### `ExecutionRequest`

Used by the coordinator when executing an approved solution:

| Field | Meaning |
| --- | --- |
| `pose` | Target pose to execute |
| `frame_id` | Frame of the target pose |
| `mode` | Requested execution mode |
| `approved_base_pose_valid` | Whether an approved base pose is supplied |
| `approved_base_pose` | Approved base pose |
| `approved_base_frame` | Frame of the approved base pose |

### `ExecutionResult`

Returned by the coordinator:

| Field | Meaning |
| --- | --- |
| `success` | Whether execution succeeded |
| `stage` | Current or final execution stage |
| `failure_reason` | Machine-readable failure code |
| `suggested_fallback` | Optional next-step hint |
| `message` | Human-readable result message |
| `base_pose` | Base pose associated with the result |
| `base_frame` | Frame of `base_pose` |
| `base_pose_valid` | Whether `base_pose` is populated |

## Callback Types

| Callback | Purpose |
| --- | --- |
| `CandidateEvaluationCallback` | Lets the selector ask the runtime to validate a single generated candidate |
| `FeedbackCallback` | Lets the coordinator publish stage and message updates during execution |

## Current Default Plugin Set

The default runtime configuration loads:

| Role | Default plugin |
| --- | --- |
| Validator | `mobile_manipulator_pose_validator::GraspingPoseValidator` |
| Generator | `mobile_manipulator_pose_generator::StandoffPoseGenerator` |
| Selector | `mobile_manipulator_pose_selector::LeastMovementPoseSelector` |
| Coordinator | `mobile_manipulator_coordinator::DefaultCoordinator` |

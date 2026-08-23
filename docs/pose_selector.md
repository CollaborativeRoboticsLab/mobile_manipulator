# Pose Selector

The pose selector plugin owns the policy for choosing the best feasible candidate from the generated base-pose set.

## Current Default Implementation

The default selector is `mobile_manipulator_pose_selector::LeastMovementPoseSelector`.

## Selection Strategy

The selector receives:

| Input | Meaning |
| --- | --- |
| Generated `CandidatePose` list | Candidate base poses from the generator |
| Original `PoseValidationRequest` | Original task context used for scoring |
| Candidate validation callback | Runtime callback that validates one candidate at a time |

For each candidate it:

1. Calls the validation callback
2. Skips infeasible candidates
3. Computes a combined movement score
4. Keeps the lowest-scoring feasible candidate

## Scoring Policy

The combined score starts with the generator-provided `candidate.score`.

If the original request already contains a base pose in the same frame, the selector adds:

| Additional term | Meaning |
| --- | --- |
| Translational distance | Distance from the supplied base pose |
| Rotational difference | Yaw delta from the supplied base pose |

The final score is:

- `candidate.score + distance_weight * distance + rotation_weight * yaw_delta`

Lower scores are preferred.

## Parameters

| Parameter | Purpose |
| --- | --- |
| `LeastMovementPoseSelector.distance_weight` | Weight applied to translation distance |
| `LeastMovementPoseSelector.rotation_weight` | Weight applied to yaw delta |

## Result Population

When a candidate is selected, the selector ensures the returned validation result includes:

| Populated field | Meaning |
| --- | --- |
| `base_target_pose` | Selected base pose |
| `base_target_frame` | Frame of the selected base pose |
| `base_target_pose_valid=true` | Marks that the selected base pose is valid |
| `candidate_score` | Final selector score attached to the result |

This makes the selected base pose available directly to service and action clients.

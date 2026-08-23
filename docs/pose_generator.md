# Pose Generator

The pose generator plugin is responsible for producing candidate base poses when the system needs to reason beyond arm-only reach.

## Current Default Implementation

The default generator is `mobile_manipulator_pose_generator::StandoffPoseGenerator`.

It creates stand-off base poses around the requested target pose.

## Generation Strategy

For each configured distance and angle offset, the generator:

1. Reads the target pose position and yaw
2. Computes an approach yaw by adding the configured offset to the target yaw
3. Places the base pose behind the target by the stand-off distance
4. Sets the base orientation to face the target
5. Assigns an initial score based on distance and angular offset

This produces a simple ring of candidate poses around the target.

## Frame Behavior

| Condition | Emitted frame |
| --- | --- |
| `request.frame_id` is set | `request.frame_id` |
| `request.frame_id` is empty | Configured `output_frame`, default `map` |

## Parameters

| Parameter | Default | Purpose |
| --- | --- | --- |
| `StandoffPoseGenerator.output_frame` | `map` | Fallback frame for emitted candidates |
| `StandoffPoseGenerator.candidate_standoff_distances` | `[0.55, 0.70, 0.85]` | Stand-off distances sampled behind the target |
| `StandoffPoseGenerator.candidate_angle_offsets_deg` | `[0.0, 30.0, -30.0]` | Angular offsets sampled around the target yaw |

## Notes

The generator does not decide feasibility. It only proposes candidates. Feasibility checking stays in the validator, and final choice stays in the selector.

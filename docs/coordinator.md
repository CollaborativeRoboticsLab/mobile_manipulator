# Coordinator

The coordinator plugin is responsible for executing an already approved mobile-manipulation solution.

## Current Default Implementation

The default coordinator is `mobile_manipulator_coordinator::DefaultCoordinator`.

It performs two stages in order:

1. Navigate to the approved base pose when `approved_base_pose_valid=true`
2. Send the target pose to the grasping `MoveToPose` action

If no approved base pose is present, it skips navigation and executes only the arm motion.

## Action Dependencies

| Dependency | Default name | Config parameter |
| --- | --- | --- |
| `nav2_msgs/action/NavigateToPose` | `navigate_to_pose` | `DefaultCoordinator.navigation_action_name` |
| `grasping_msgs/action/MoveToPose` | `move_arm_to_pose` | `DefaultCoordinator.move_arm_action_name` |

## Execution Flow

When the `ExecuteTaskPose` action is accepted by the runtime server:

1. The server first revalidates the approved solution through the validator plugin.
2. If validation fails, execution is aborted before motion.
3. The coordinator publishes feedback stage `navigating` while moving the base.
4. The coordinator publishes feedback stage `moving_arm` while sending the arm goal.
5. On success, the final result stage is `completed`.

## Failure Behavior

| Failure case | Reported stage | Failure reason |
| --- | --- | --- |
| Navigation failure | `navigating` | `navigation_failed` |
| Arm execution failure | `moving_arm` | `arm_execution_failed` |

The coordinator itself does not choose fallback policies. It reports the execution result back to the runtime server, which merges it into the public action result.

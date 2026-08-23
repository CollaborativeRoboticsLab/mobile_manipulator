# Parameters

The runtime server is configured through `mobile_manipulator_ros/config/mobile_manipulator_server.yaml`.

Configuration is split into:

| Scope | Purpose |
| --- | --- |
| Top-level ROS interface names | Control the public services and action exposed by the server |
| Plugin selection parameters | Choose the concrete validator, generator, selector, and coordinator classes |
| Class-keyed parameter blocks | Configure each selected plugin instance |

## Top-Level Interface Parameters

These parameters control the public ROS names exposed by `mobile_manipulator_server`:

| Parameter | Default | Purpose |
| --- | --- | --- |
| `check_task_feasibility_service_name` | `check_task_feasibility` | Name of the direct feasibility service |
| `find_feasible_pose_service_name` | `find_feasible_pose` | Name of the candidate-search service |
| `execute_task_pose_action_name` | `execute_task_pose` | Name of the approved-execution action |

## Plugin Selection Parameters

These choose the concrete plugin classes:

| Parameter | Default plugin |
| --- | --- |
| `pose_validator_plugin` | `mobile_manipulator_pose_validator::GraspingPoseValidator` |
| `pose_generator_plugin` | `mobile_manipulator_pose_generator::StandoffPoseGenerator` |
| `pose_selector_plugin` | `mobile_manipulator_pose_selector::LeastMovementPoseSelector` |
| `coordinator_plugin` | `mobile_manipulator_coordinator::DefaultCoordinator` |

## Validator Parameters

Current block: `GraspingPoseValidator.*`

| Parameter | Default | Purpose |
| --- | --- | --- |
| `service_name` | `check_cartesian_pose_feasibility` | Grasping Cartesian feasibility service name |
| `arm_base_frame` | `base_link` | Frame used when projecting a target into the arm frame |
| `use_costmap_validation` | `false` | Whether to reject candidate base poses against a Nav2 costmap |
| `costmap_topic` | `/global_costmap/costmap` | Costmap source used for base-pose occupancy checks |
| `costmap_occupied_threshold` | `253` | Occupancy threshold treated as blocked |

## Generator Parameters

Current block: `StandoffPoseGenerator.*`

| Parameter | Default | Purpose |
| --- | --- | --- |
| `output_frame` | `map` | Fallback frame for emitted candidate poses |
| `candidate_standoff_distances` | `[0.55, 0.70, 0.85]` | Distances used when placing the base behind the target |
| `candidate_angle_offsets_deg` | `[0.0, 30.0, -30.0]` | Angular offsets sampled around the target yaw |

## Selector Parameters

Current block: `LeastMovementPoseSelector.*`

| Parameter | Default | Purpose |
| --- | --- | --- |
| `distance_weight` | `1.0` | Weight applied to translation distance from the supplied base pose |
| `rotation_weight` | `1.0` | Weight applied to yaw delta from the supplied base pose |

## Coordinator Parameters

Current block: `DefaultCoordinator.*`

| Parameter | Default | Purpose |
| --- | --- | --- |
| `navigation_action_name` | `navigate_to_pose` | Nav2 action used when a base move is required |
| `move_arm_action_name` | `move_arm_to_pose` | Grasping action used for final arm execution |

## Launch-Level Parameters

The generic launch file `mobile_manipulator_ros/launch/task_feasibility.launch.py` accepts:

| Launch file | Argument | Purpose |
| --- | --- | --- |
| `mobile_manipulator_ros/launch/task_feasibility.launch.py` | `config_file` | Parameter file passed to `mobile_manipulator_server` |
| `mobile_manipulator_control/launch/mobile_manipulator_control.launch.py` | `config_file` | Parameter file passed to `mobile_manipulator_control_node` |

Platform wrappers such as `grasping_omron_moma/launch/task_feasibility.launch.py` override this with platform-specific YAML.

# mobile_manipulator_control

Generic ROS 2 test node for validating the mobile manipulator stack before capability-layer integration.

## Purpose

This package reads a target pose from a ROS parameter file and exercises the mobile manipulator workflow directly:

1. `CheckTaskFeasibility`
2. `FindFeasiblePose` when direct feasibility fails
3. `ExecuteTaskPose` when execution is enabled

The node is generic to the `mobile_manipulator` stack. Platform-specific packages should wrap it with launch/config customization rather than forking the node.

## Launch

```bash
ros2 launch mobile_manipulator_control mobile_manipulator_control.launch.py
```

## Configuration

Override the `config_file` launch argument or edit `config/mobile_manipulator_control.yaml` to control the target pose, mode, optional base pose, and execution flags.
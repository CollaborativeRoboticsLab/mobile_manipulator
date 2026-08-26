# Scene Management Architecture

This document describes the runtime interaction between the grasping-side scene manager and the mobile-manipulator-side scene registry.

## Status

The first integrated runtime is now in place:

- `scene_manager_node.py` is the grasping-side active-scene authority
- `scene_registry_node.cpp` loads `scenes.yaml`, hosts a `SceneSelector` plugin, and forwards `ActivateScene`
- `mobile_manipulator_scene_selector` provides the default `ExactNameThenTagsSelector`
- launch files now wire shared action, service, and topic names by default
- `scenes.yaml` currently contains a real `crlab_table` descriptor that points to the Omron workspace file

## API Diagram

```mermaid
flowchart LR
    Client[External client or action runner]
    Registry[scene_registry_node]
    Selector[SceneSelector plugin]
    SceneFile[scenes.yaml]
    Manager[scene_manager_node]
    WorkspaceFile[workspace.yaml]
    MoveIt[MoveIt planning scene]
    Motion[motion_execution_node]
    Feas[feasibility_service_node]
    Active[active_scene topic]

    Client -->|ListAvailableScenes| Registry
    Client -->|ActivateSceneByName| Registry
    Registry --> SceneFile
    Registry --> Selector
    Selector --> Registry
    Registry -->|ActivateScene action client| Manager
    Manager --> WorkspaceFile
    Manager --> MoveIt
    Manager --> Active
    Active --> Motion
    Active --> Feas
    Manager -->|GetActiveScene service| Registry
```

## Scene Manager Runtime Sequence

```mermaid
sequenceDiagram
    participant C as Client
    participant R as scene_registry_node
    participant P as SceneSelector plugin
    participant G as scene_manager_node

    C->>R: ActivateSceneByName(scene_name, requested_mode, required_tags)
    R->>R: Load/filter scenes.yaml entries
    R->>G: GetActiveScene()
    G-->>R: current active scene or inactive
    R->>P: select(scenes, request)
    P-->>R: selected SceneDescriptor
    R->>G: ActivateScene(reference, force_reload)
    G->>G: Resolve package + workspace_file
    G->>G: Parse workspace.yaml
    G->>G: Build collision objects + ACM + workspace area
    G->>G: Apply MoveIt scene
    G-->>R: ActiveScene(scene_handle, metadata)
    R-->>C: ActivateSceneByName result
```

## Scene Manager Node State Diagram

```mermaid
stateDiagram-v2
    [*] --> Idle

    Idle --> Validating: ActivateScene or LoadSceneFromContent goal accepted
    Idle --> Active: default scene activated at startup
    Active --> Active: matching ActivateScene with force_reload=false
    Validating --> ResolveFailed: package/file or content invalid
    Validating --> Applying: workspace document validated

    Applying --> ApplyFailed: MoveIt update rejected or timed out
    Applying --> Publishing: planning scene + active workspace-area state applied

    Publishing --> Active: active scene metadata published
    Active --> Validating: next activation request

    ResolveFailed --> Idle
    ApplyFailed --> Idle
```

## Scene Registry Node State Diagram

```mermaid
stateDiagram-v2
    [*] --> Ready

    Ready --> Listing: ListAvailableScenes request
    Listing --> Ready

    Ready --> Selecting: ActivateSceneByName goal accepted
    Selecting --> NoMatch: no scene satisfies name/mode/tags
    Selecting --> Activating: selected scene reference ready

    Activating --> ActivationFailed: grasping scene manager activation failed
    Activating --> Ready: activation succeeded
    NoMatch --> Ready
    ActivationFailed --> Ready
```

## Active Scene Semantics

For the single-robot case, the active scene is a singleton global runtime state.

Rules:

- only one active scene exists at a time
- only one activation request may run at a time
- a successful activation fully replaces the previous active scene
- task execution must not implicitly change the active scene
- scene activation remains separate from `CheckTaskFeasibility`, `FindFeasiblePose`, and `ExecuteTaskPose`

## Grasping-Side Responsibilities

`scene_manager_node` is the only runtime authority for:

- parsing `workspace.yaml`
- deriving MoveIt collision objects
- applying allowed object-link collisions
- updating the active workspace-area filter
- publishing active-scene metadata

The existing arm execution and feasibility nodes should consume active-scene state, not own independent workspace parsing logic long term.

## Mobile-Manipulator-Side Responsibilities

`scene_registry_node` is responsible for:

- loading `scenes.yaml`
- exposing `ListAvailableScenes`
- exposing `ActivateSceneByName`
- hosting a `SceneSelector` plugin for selection policy
- querying `GetActiveScene` so selection can prefer reuse of the current scene
- calling the grasping-side `ActivateScene` action
- relaying the resulting `scene_handle` and active-scene metadata

The selector plugin itself is not the public ROS runtime surface.

## Current Launch Wiring

- `grasping_control/launch/motion_execution.launch.py` starts `scene_manager_node`, `motion_execution_node`, and `feasibility_service_node` with shared scene API names
- `mobile_manipulator_scene_registry/launch/scene_registry.launch.py` starts `scene_registry_node` with matching action and service names
- `mobile_manipulator_control/launch/mobile_manipulator_control.launch.py` now includes the scene-registry launch
- `grasping_omron_moma/launch/ld250_tm12x_soft_two_fingers.launch.py` bootstraps the default Omron `crlab_table` scene
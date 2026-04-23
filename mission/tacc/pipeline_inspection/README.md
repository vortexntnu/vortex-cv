# Pipeline Inspection FSM

A ROS2 finite state machine (FSM) that enables an AUV to autonomously search for, detect, and follow a subsea pipeline. Built with [YASMIN](https://github.com/uleroboticsgroup/yasmin) for state management.

## Overview

The FSM orchestrates a multi-stage pipeline inspection mission:

1. Waits for an external start signal
2. Executes a search pattern while polling for pipeline landmarks
3. Converges to the detected pipeline start
4. Hands off to pipeline following
5. Waits for an end-of-pipeline signal, then terminates cleanly

## State Machine

```
WAIT_FOR_START
    │ (start_mission service called)
    ▼
SEARCH ─── SearchPatternState (navigate through search waypoints)
    │       LandmarkPollingState (concurrent landmark detection)
    │ (landmark found)
    ▼
CONVERGE
    │ (AUV at pipeline start)
    ▼
START_PIPELINE_TRG
    │ (start_pipeline_following service called)
    ▼
START_WM
    │ (persistent WaypointManager goal sent)
    ▼
PIPELINE_FOLLOWING
    │ (pipeline_finished service called)
    ▼
STOP_WM
    │
    ▼
DONE
```

## ROS2 Interface

### Services Provided

| Service | Type | Description |
|---|---|---|
| `pipeline_inspection_fsm/start_mission` | `std_srvs/Trigger` | Starts the mission |
| `pipeline_inspection_fsm/start_pipeline_following` | `std_srvs/Trigger` | Signals the FSM to begin persistent pipeline following |
| `pipeline_inspection_fsm/pipeline_finished` | `std_srvs/Trigger` | Signals the end of the pipeline |

### Action Clients

| Action Server | Type | Description |
|---|---|---|
| `waypoint_manager` | `vortex_msgs/action/WaypointManager` | Navigation along waypoints |
| `landmark_polling` | `vortex_msgs/action/LandmarkPolling` | Pipeline landmark detection |

## Configuration

### `search_waypoints.yaml`

Defines the search pattern the AUV follows while looking for the pipeline:

```yaml
search_waypoint_1:
  position: {x: 2.0, y: 0.0, z: 0.0}
  orientation: {roll: 0.0, pitch: 0.0, yaw: 0.0}
  mode: 0
  convergence_threshold: 0.1
```

Add additional `search_waypoint_N` entries to extend the search area.

### `pipeline_convergence.yaml`

Defines a pose offset applied to the detected landmark position when converging:

```yaml
pipeline_start_convergence:
  position: {x: 0.0, y: 0.0, z: -1.0}
  orientation: {roll: 0.0, pitch: 0.0, yaw: 0.0}
  convergence_threshold: 0.1
  dead_reckoning_threshold: 0.5
```
## Triggering the Mission

```bash
# 1. Start the mission
ros2 service call /orca/pipeline_inspection_fsm/start_mission std_srvs/srv/Trigger

# 2. (Called internally by the FSM after convergence)
ros2 service call /orca/pipeline_inspection_fsm/start_pipeline_following std_srvs/srv/Trigger

# 3. Signal end of pipeline when following is complete
ros2 service call /orca/pipeline_inspection_fsm/pipeline_finished std_srvs/srv/Trigger
```

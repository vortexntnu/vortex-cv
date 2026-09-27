# Pipeline End Detector

A ROS2 node that subscribes to the end-of-pipeline classification output and, after a configurable number of consecutive detections, sends a trigger service call to the pipeline inspection FSM to signal that the pipeline has ended.

## Overview

The node listens to a `std_msgs/UInt8` topic published by the end-of-pipeline classifier:

- `data == 1` — Class 1: end of pipeline detected
- `data == 0` — Class 0: continue following

A counter increments on each `1` and decays by one (floored at zero) on each `0`. Once the counter reaches `detection_threshold`, the node calls the `pipeline_inspection_fsm/pipeline_finished` service. If the call fails, the counter resets to zero and the node keeps accumulating from there.

## ROS2 Interface

### Subscriptions

| Topic | Type | Description |
|---|---|---|
| `classification_output` | `std_msgs/UInt8` | End-of-pipeline classification output |

### Publications

| Topic | Type | Description |
|---|---|---|
| `pipeline_end_detector/debug_counter` | `std_msgs/Int32` | Live detection counter, published whenever `debug` is `true` (default). Plot it (e.g. with `rqt_plot` or PlotJuggler) alongside `detection_threshold` to visualize the decay behavior. Set `debug: false` to disable. |

### Service Clients

| Service | Type | Description |
|---|---|---|
| `pipeline_inspection_fsm/pipeline_finished` | `std_srvs/Trigger` | Signals the FSM that the pipeline has ended |

## Configuration

Parameters are set directly in [`launch/pipeline_end_detector.launch.py`](launch/pipeline_end_detector.launch.py) (no separate YAML config file) — edit the `parameters` dict on the `Node` action to change them:

```python
parameters=[{
    'detection_threshold': 10,
    'activation_delay_sec': 30.0,
    'debug': True,
    'topics.detection': 'classification_output',
    'topics.end_of_pipeline_service': 'pipeline_inspection_fsm/pipeline_finished',
    'topics.start_detection_service': 'pipeline_end_detector/start_detection',
    'topics.debug_counter': 'pipeline_end_detector/debug_counter',
}],
```

| Parameter | Description |
|---|---|
| `detection_threshold` | Counter threshold required before triggering the service call |
| `activation_delay_sec` | Delay between the `start_detection` trigger and detection becoming active (0 = activate now) |
| `debug` | When `true`, publishes the live counter on `topics.debug_counter` for plotting |
| `topics.detection` | Topic the end-of-pipeline classification is received on |
| `topics.end_of_pipeline_service` | FSM service called when the pipeline end is detected |
| `topics.start_detection_service` | Service the FSM calls to activate detection on this node |
| `topics.debug_counter` | Topic the live counter is published on when `debug == true` |

## Running

```bash
ros2 launch pipeline_end_detector pipeline_end_detector.launch.py
```
